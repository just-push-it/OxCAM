// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/document/StepBodies.hpp"

#include <ocam/cad/Tessellation.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Sha256.hpp>
#include <ocam/geom3d/Stl.hpp>

#include <fmt/format.h>

#include <map>
#include <mutex>
#include <utility>

namespace ocam::document {
namespace {

struct Cache {
    std::mutex mutex;
    std::map<std::string, cad::Model> models;
    std::map<std::pair<std::string, double>, std::shared_ptr<const geom3d::Mesh>> meshes;
    std::filesystem::path folder;
    StepCacheCounts counts;
};

Cache& cache() {
    static Cache c;
    return c;
}

/// The model for a file; the cache's mutex is held.
cad::Model modelLocked(Cache& c, const StepFile& file, std::string_view sourceName) {
    if (const auto it = c.models.find(file.sha256); it != c.models.end()) {
        return it->second;
    }
    if (!file.bytes) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("{}: the STEP file is not loaded", sourceName));
    }
    CancelToken cancel;
    NullProgress progress;
    cad::Model model = cad::Model::importStepData(*file.bytes, sourceName, cancel, progress);
    ++c.counts.modelsRead;
    c.models.emplace(file.sha256, model);
    return model;
}

} // namespace

StepFile stepFile(std::shared_ptr<const std::string> bytes) {
    return {sha256Hex(*bytes), std::move(bytes)};
}

StepFile stepFileOf(const cad::Model& model) {
    StepFile file = stepFile(std::make_shared<const std::string>(model.exportStepData()));
    Cache& c = cache();
    const std::lock_guard lock(c.mutex);
    c.models.try_emplace(file.sha256, model);
    return file;
}

cad::Model stepModel(const StepFile& file, std::string_view sourceName) {
    Cache& c = cache();
    const std::lock_guard lock(c.mutex);
    return modelLocked(c, file, sourceName);
}

cad::Model loadedStepModel(const std::string& sha256) {
    Cache& c = cache();
    const std::lock_guard lock(c.mutex);
    const auto it = c.models.find(sha256);
    return it == c.models.end() ? cad::Model() : it->second;
}

std::shared_ptr<const geom3d::Mesh> stepMesh(const StepFile& file, double tolerance,
                                             std::string_view sourceName) {
    if (!(tolerance > 0.0)) {
        throw Error(ErrorCode::InvalidArgument, "the mesh tolerance must be positive");
    }
    Cache& c = cache();
    const std::lock_guard lock(c.mutex);
    const auto key = std::make_pair(file.sha256, tolerance);
    if (const auto it = c.meshes.find(key); it != c.meshes.end()) {
        return it->second;
    }
    const std::filesystem::path onDisk =
        c.folder.empty() ? std::filesystem::path()
                         : c.folder / fmt::format("{}-{:g}.stl", file.sha256, tolerance);
    std::shared_ptr<const geom3d::Mesh> mesh;
    if (!onDisk.empty()) {
        std::error_code ec;
        if (std::filesystem::is_regular_file(onDisk, ec)) {
            try {
                mesh = std::make_shared<const geom3d::Mesh>(
                    geom3d::readStl(readFileBytes(onDisk), sourceName));
                ++c.counts.meshesFromDisk;
            } catch (const Error&) {
                mesh.reset(); // made again below
            }
        }
    }
    if (!mesh) {
        const cad::Model model = modelLocked(c, file, sourceName);
        // Through the STL's 32-bit floats, as the disk cache keeps it: a mesh is the same
        // whether it was made now or read back.
        const std::string bytes = geom3d::writeStl(cad::tessellate(model, {tolerance, 0.5}));
        mesh = std::make_shared<const geom3d::Mesh>(geom3d::readStl(bytes, sourceName));
        ++c.counts.meshesMade;
        if (!onDisk.empty()) {
            try {
                std::error_code ec;
                std::filesystem::create_directories(c.folder, ec);
                writeFileAtomic(onDisk, bytes);
            } catch (const Error&) {
                // A cache that cannot be written is only slower.
            }
        }
    }
    c.meshes.emplace(key, mesh);
    return mesh;
}

void setStepMeshCacheFolder(const std::filesystem::path& folder) {
    Cache& c = cache();
    const std::lock_guard lock(c.mutex);
    c.folder = folder;
}

StepCacheCounts stepCacheCounts() {
    Cache& c = cache();
    const std::lock_guard lock(c.mutex);
    return c.counts;
}

void clearStepCache() {
    Cache& c = cache();
    const std::lock_guard lock(c.mutex);
    c.models.clear();
    c.meshes.clear();
    c.counts = {};
}

} // namespace ocam::document
