// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Machine parts from CAD files (DL-0037): STEP files split into their bodies (every solid of
// every product instance) and STL files, optionally grouped so that what moves together becomes
// one machine part, placed as the file places them. STEP parts keep their exact geometry as a
// STEP file (StepBodies.hpp); their meshes are tessellated from it when the machine is loaded.
// Exact picking on them: snapOnShape and the cad::SubShapeGeometry of what was clicked.

#pragma once

#include <ocam/cad/Model.hpp>
#include <ocam/cad/SubShape.hpp>
#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Progress.hpp>
#include <ocam/geom3d/Mesh.hpp>
#include <ocam/machine/Machine.hpp>
#include <ocam/machine/Model.hpp>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace ocam::document {

/// A part found in a file, before tessellation.
struct ImportedPart {
    std::string name;
    /// The assemblies above it in its file, below what every part of the file shares.
    std::vector<std::string> path;
    std::string file; ///< The file's name.
    cad::Model model; ///< STEP parts, in millimetres, placed in the file's frame.
    std::shared_ptr<const geom3d::Mesh> mesh; ///< STL parts, scaled to millimetres.
    std::optional<std::uint32_t> colour;      ///< 0xRRGGBB
    std::size_t solids = 0;
};

/// The parts of a STEP (.step, .stp) or STL (.stl) file. STL coordinates are multiplied by
/// `stlScale` (millimetres per file unit: 1, 25.4, 1000). Throws ocam::Error (Io, Parse,
/// InvalidArgument for other extensions) and OperationCancelled.
std::vector<ImportedPart> readMachinePartFile(const std::filesystem::path& path, double stlScale,
                                              CancelToken& cancel, ProgressSink& progress);

/// Groups for merging: at `level` 0 every part on its own (repeated names numbered), at level
/// n the parts below the same assembly n levels down from the file's top share a group named
/// after that assembly (parts higher up keep their own).
std::vector<std::string> groupParts(const std::vector<ImportedPart>& parts, int level);

/// The deepest assembly level of the parts (0 for files without structure).
int assemblyDepth(const std::vector<ImportedPart>& parts);

/// Merges the parts with the same group into one machine part named after it, in the order
/// groups first appear, with the colour of its first part that has one. Parts whose group is
/// empty are left out. A group of STEP parts becomes a STEP part (their bodies as one STEP file)
/// with its mesh tessellated at chord error `tolerance` [mm]; a group with an STL part becomes a
/// mesh part. The parts keep the files' placement (their offset and rotation are zero).
std::vector<machine::ModelPart> machinePartsFrom(const std::vector<ImportedPart>& parts,
                                                 const std::vector<std::string>& groups,
                                                 double tolerance, CancelToken& cancel,
                                                 ProgressSink& progress);

/// Parts merged into one, each where it stands at machine zero (the result at offset and
/// rotation zero), with the first part's carrier, colour and flags: a STEP part when they all
/// are (meshed at `tolerance`), else a mesh part. Throws Error(InvalidArgument) for parts whose
/// geometry is not loaded.
machine::ModelPart mergeParts(const std::vector<machine::ModelPart>& parts, std::string name,
                              double tolerance);

/// Sets every STEP part's mesh from its STEP file at the model's meshTolerance (stepMesh: made
/// once per file and tolerance). Throws like stepMesh.
void meshMachineParts(machine::Machine& machine);

/// machine::loadMachine, then meshMachineParts: a machine ready for display and checks.
machine::Machine loadMachineFile(const std::filesystem::path& path);

/// The exact B-rep of a STEP part (stepModel of its file), null for mesh parts.
cad::Model partModel(const machine::ModelPart& part);

/// The point a click on the sub-shape `picked` (its geometry in the part's own coordinates)
/// snaps to, model frame at machine zero, for a part at machine zero: the face's centre (a
/// plane's centroid; a round face's axis point; a circle's or arc's centre; a vertex; a line's
/// middle), where the innermost rotary axis carrying the part meets the face (a plane, or a
/// circle's plane), or the point clicked (`clicked`, model frame; a vertex exactly). Throws
/// Error(InvalidArgument) with a message for the user like machine::snapOnPart.
Vec3 snapOnShape(const machine::MachineModel& model, const machine::ModelPart& part,
                 const cad::SubShapeGeometry& picked, const Vec3& clicked, machine::Snap snap);

/// The centre line of a picked round face, circle or arc, in the model frame at machine zero for
/// a part at `placement`; nullopt for other sub-shapes.
std::optional<geom3d::AxisLine> axisOfShape(const cad::SubShapeGeometry& picked,
                                            const Transform3& placement);

} // namespace ocam::document
