// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Bodies stored as STEP files named by the SHA-256 of their content: machine parts (DL-0037)
// and workholding bodies (DL-0035). The STEP file is the source and the only thing saved. The
// B-rep read from it and the meshes tessellated from it (for display and collision checks) are
// derived: made once per session and kept in memory here, the meshes also on disk when a cache
// folder is set (always safe to delete). Thread-safe.

#pragma once

#include <ocam/cad/Model.hpp>
#include <ocam/geom3d/Mesh.hpp>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>

namespace ocam::document {

/// A STEP file's bytes and their SHA-256 (the name it is stored under).
struct StepFile {
    std::string sha256;
    std::shared_ptr<const std::string> bytes;
};

/// A file for these bytes (hashed).
StepFile stepFile(std::shared_ptr<const std::string> bytes);

/// A model written as a STEP file (cad::Model::exportStepData). The model is remembered as the
/// file's B-rep, so stepModel() does not read the file back.
StepFile stepFileOf(const cad::Model& model);

/// The B-rep of a STEP file, read once per session. `sourceName` names it in errors. Throws like
/// cad::Model::importStepData.
cad::Model stepModel(const StepFile& file, std::string_view sourceName);

/// The B-rep of a file already read (stepModel) or written (stepFileOf) this session, by its
/// hash; a null model otherwise.
cad::Model loadedStepModel(const std::string& sha256);

/// The file's mesh at chord tolerance `tolerance` [mm] (0.5 rad angular, as at import):
/// tessellated once per session and tolerance, or read from the disk cache. Throws like
/// stepModel and Error(InvalidArgument) for a tolerance that is not positive.
std::shared_ptr<const geom3d::Mesh> stepMesh(const StepFile& file, double tolerance,
                                             std::string_view sourceName);

/// Keeps meshes on disk too, as <sha256>-<tolerance>.stl in `folder` (created when needed); an
/// empty path turns it off (the default). Unreadable cache files are made again.
void setStepMeshCacheFolder(const std::filesystem::path& folder);

/// What the cache did, for tests and timing: models read from STEP, meshes tessellated, meshes
/// read from the disk cache.
struct StepCacheCounts {
    std::size_t modelsRead = 0;
    std::size_t meshesMade = 0;
    std::size_t meshesFromDisk = 0;
};
StepCacheCounts stepCacheCounts();
/// Forgets every model and mesh held in memory (tests).
void clearStepCache();

} // namespace ocam::document
