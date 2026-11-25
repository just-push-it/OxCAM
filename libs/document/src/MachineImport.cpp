// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/document/MachineImport.hpp"

#include "ocam/document/StepBodies.hpp"

#include <ocam/cad/Assembly.hpp>
#include <ocam/cad/Tessellation.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/geom3d/Stl.hpp>
#include <ocam/machine/Model.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cctype>
#include <map>

namespace ocam::document {
namespace {

std::string utf8(const std::filesystem::path& p) {
    const std::u8string u8 = p.u8string();
    return {u8.begin(), u8.end()};
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

} // namespace

std::vector<ImportedPart> readMachinePartFile(const std::filesystem::path& path, double stlScale,
                                              CancelToken& cancel, ProgressSink& progress) {
    const std::string ext = lower(utf8(path.extension()));
    const std::string file = utf8(path.filename());
    std::vector<ImportedPart> out;
    if (ext == ".stl") {
        progress.report(0.0, "Reading STL file");
        geom3d::Mesh mesh = geom3d::readStl(readFileBytes(path), file);
        if (stlScale != 1.0) {
            for (Vec3& v : mesh.vertices) {
                v *= stlScale;
            }
        }
        ImportedPart p;
        p.name = utf8(path.stem());
        p.file = file;
        p.mesh = std::make_shared<const geom3d::Mesh>(std::move(mesh));
        out.push_back(std::move(p));
        progress.report(1.0, "Done");
        return out;
    }
    if (ext != ".step" && ext != ".stp") {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("{}: machine parts come from STEP or STL files", file));
    }
    std::vector<cad::AssemblyPart> parts = cad::importStepAssembly(path, cancel, progress);
    // Drop the assembly levels every part shares (the machine itself).
    std::size_t shared = parts.front().path.size();
    for (const auto& p : parts) {
        std::size_t k = 0;
        while (k < shared && k < p.path.size() && p.path[k] == parts.front().path[k]) {
            ++k;
        }
        shared = k;
    }
    for (auto& p : parts) {
        ImportedPart part;
        part.name = std::move(p.name);
        part.path.assign(p.path.begin() + static_cast<std::ptrdiff_t>(shared), p.path.end());
        part.file = file;
        part.model = std::move(p.model);
        part.colour = p.colour;
        part.solids = p.solids;
        out.push_back(std::move(part));
    }
    return out;
}

int assemblyDepth(const std::vector<ImportedPart>& parts) {
    std::size_t depth = 0;
    for (const auto& p : parts) {
        depth = std::max(depth, p.path.size());
    }
    return static_cast<int>(depth);
}

std::vector<std::string> groupParts(const std::vector<ImportedPart>& parts, int level) {
    std::vector<std::string> groups;
    std::map<std::string, int> seen;
    for (const auto& p : parts) {
        if (level > 0 && !p.path.empty()) {
            const std::size_t k = std::min(p.path.size(), static_cast<std::size_t>(level));
            groups.push_back(p.path[k - 1]);
            continue;
        }
        // On its own: repeated names are numbered so they do not merge.
        const int n = ++seen[p.name];
        groups.push_back(n == 1 ? p.name : fmt::format("{} ({})", p.name, n));
    }
    return groups;
}

std::vector<machine::ModelPart> machinePartsFrom(const std::vector<ImportedPart>& parts,
                                                 const std::vector<std::string>& groups,
                                                 double tolerance, CancelToken& cancel,
                                                 ProgressSink& progress) {
    if (groups.size() != parts.size()) {
        throw Error(ErrorCode::InvalidArgument, "one group per part");
    }
    if (!(tolerance > 0.0)) {
        throw Error(ErrorCode::InvalidArgument, "the tessellation tolerance must be positive");
    }
    std::vector<std::string> order;
    std::map<std::string, std::vector<const ImportedPart*>> members;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        const std::string& group = groups[i];
        if (group.empty()) {
            continue;
        }
        auto [it, added] = members.try_emplace(group);
        if (added) {
            order.push_back(group);
        }
        it->second.push_back(&parts[i]);
    }
    std::vector<machine::ModelPart> out;
    for (std::size_t g = 0; g < order.size(); ++g) {
        cancel.throwIfCancelled();
        progress.report(static_cast<double>(g) / static_cast<double>(order.size()),
                        "Tessellating parts");
        const std::string& group = order[g];
        const std::vector<const ImportedPart*>& in = members[group];
        const bool exact = std::all_of(in.begin(), in.end(),
                                       [](const ImportedPart* p) { return !p->model.isNull(); });
        machine::ModelPart part;
        if (exact) {
            std::vector<cad::Model> models;
            for (const ImportedPart* p : in) {
                models.push_back(p->model);
            }
            const StepFile file = stepFileOf(cad::compound(models));
            part = machine::stepPart(group, file.bytes);
            part.geometry = stepMesh(file, tolerance, group);
        } else {
            geom3d::Mesh mesh;
            for (const ImportedPart* p : in) {
                mesh.append(p->mesh ? *p->mesh : cad::tessellate(p->model, {tolerance, 0.5}));
            }
            part = machine::makePart(group, mesh);
        }
        if (!part.geometry || part.geometry->isEmpty()) {
            continue;
        }
        for (const ImportedPart* p : in) {
            if (p->colour) {
                part.colour = *p->colour;
                break;
            }
        }
        out.push_back(std::move(part));
    }
    progress.report(1.0, "Done");
    return out;
}

machine::ModelPart mergeParts(const std::vector<machine::ModelPart>& parts, std::string name,
                              double tolerance) {
    if (parts.empty()) {
        throw Error(ErrorCode::InvalidArgument, "no parts to merge");
    }
    const bool exact = std::all_of(parts.begin(), parts.end(),
                                   [](const machine::ModelPart& p) { return p.exact(); });
    machine::ModelPart part;
    if (exact) {
        std::vector<cad::Model> models;
        for (const machine::ModelPart& p : parts) {
            models.push_back(cad::transformed(partModel(p), p.placement()));
        }
        const StepFile file = stepFileOf(cad::compound(models));
        part = machine::stepPart(std::move(name), file.bytes);
        part.geometry = stepMesh(file, tolerance, part.name);
    } else {
        geom3d::Mesh merged;
        for (const machine::ModelPart& p : parts) {
            if (!p.geometry) {
                throw Error(ErrorCode::InvalidArgument,
                            fmt::format("the mesh of part '{}' is not loaded", p.name));
            }
            geom3d::Mesh mesh = *p.geometry;
            mesh.transform(p.placement());
            merged.append(mesh);
        }
        part = machine::makePart(std::move(name), merged);
    }
    const machine::ModelPart& first = parts.front();
    part.carrier = first.carrier;
    part.colour = first.colour;
    part.collides = first.collides;
    part.shown = first.shown;
    return part;
}

void meshMachineParts(machine::Machine& machine) {
    for (machine::ModelPart& p : machine.model.parts) {
        if (p.exact()) {
            p.geometry = stepMesh({p.step, p.stepFile}, machine.model.meshTolerance,
                                  fmt::format("machine '{}', part '{}'", machine.name, p.name));
        }
    }
}

machine::Machine loadMachineFile(const std::filesystem::path& path) {
    machine::Machine m = machine::loadMachine(path);
    meshMachineParts(m);
    return m;
}

cad::Model partModel(const machine::ModelPart& part) {
    if (!part.exact()) {
        return {};
    }
    if (cad::Model loaded = loadedStepModel(part.step); !loaded.isNull()) {
        return loaded;
    }
    return stepModel({part.step, part.stepFile}, fmt::format("part '{}'", part.name));
}

std::optional<geom3d::AxisLine> axisOfShape(const cad::SubShapeGeometry& picked,
                                            const Transform3& placement) {
    if (!picked.hasAxis() || picked.direction.isZero()) {
        return std::nullopt;
    }
    return geom3d::AxisLine{placement * picked.point,
                            (placement.linear() * picked.direction).normalized(), picked.radius, 0};
}

Vec3 snapOnShape(const machine::MachineModel& model, const machine::ModelPart& part,
                 const cad::SubShapeGeometry& picked, const Vec3& clicked, machine::Snap snap) {
    using Kind = cad::SubShapeGeometry::Kind;
    const Transform3 placement = part.placement();
    const Vec3 point = placement * picked.point;
    if (snap == machine::Snap::Point) {
        return picked.kind == Kind::Point ? point : clicked;
    }
    if (snap == machine::Snap::FaceCentre) {
        return point;
    }
    // Where the rotary axis meets the plane of the face (or of the circle) clicked.
    std::optional<Vec3> normal;
    if (picked.kind == Kind::Plane || picked.kind == Kind::Circle) {
        normal = placement.linear() * picked.direction;
    }
    if (!normal) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("click a flat face or a circular edge of '{}' for the rotary axis "
                                "to meet",
                                part.name));
    }
    std::string axis;
    const auto line = machine::rotaryLineOf(model, part.carrier, &axis);
    if (!line) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("no rotary axis carries '{}': snap to the face's centre instead",
                                part.name));
    }
    const auto meet = machine::lineMeetsPlane(*line, point, *normal);
    if (!meet) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("that face of '{}' runs along the {} axis: click a face square to "
                                "it",
                                part.name, axis));
    }
    return *meet;
}

} // namespace ocam::document
