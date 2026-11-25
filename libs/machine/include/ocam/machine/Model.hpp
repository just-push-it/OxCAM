// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Machine models (DL-0037): the machine's parts as meshes, each carried by the fixed base or by
// an axis of the machine's kinematic tree, so the simulation can show the machine moving and
// check its parts for collisions.
//
// The model frame is the machine's own frame with every axis at 0 (machine zero), with +Z along
// the spindle axis from the tool tip towards the spindle. Parts are placed in it as they stand at
// machine zero. Each axis moves what it carries (the parts, the axes below it in the tree, the
// spindle or the table): a linear axis by its value along its direction, a rotary axis
// right-handed about its centre line by its value. The spindle (where the tool sits) is carried
// like a part. The work of a setup (its WCS with the stock, part and workholding) is carried by
// the axis its placement names (WorkMount); the machine itself has no work origin.
//
// The table frame of solve (Kinematics.hpp) is the model frame's axes with its origin at the
// table centre: where the rotary axes of the table chain (tableEnd) meet; with one of them, the
// point of its line nearest the WCS origin; without any, the model origin.
//
// The rotary axes follow the machine's kinematics type, which machine::solve uses (Kinematics.hpp):
// the model gives their position (pivots), the order in the tree and what they carry; validation
// checks that the model turns the table and the head as solve assumes.

#pragma once

#include "ocam/machine/Machine.hpp"

#include <ocam/geom3d/AxisFit.hpp>

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace ocam::machine {

/// Positions of the machine axes (linear [mm], rotary [rad]).
struct AxisPosition {
    Vec3 linear = Vec3::Zero(); ///< X Y Z
    IndexAngles rotary;
    double t = 0.0;  ///< A lathe's turret [rad].
    double w = 0.0;  ///< A lathe's tailstock.
    double z2 = 0.0; ///< A lathe's sub-spindle.
    double c2 = 0.0; ///< A lathe's sub-spindle [rad].
    double value(std::string_view axis) const noexcept;
    bool operator==(const AxisPosition&) const = default;
};

/// Where everything is for an axis position: the motion of what each axis carries (model frame
/// at machine zero -> model frame), the spindle frame (origin at the gauge point, +Z up the tool
/// axis) and the table chain's end (tableEnd) at the table centre for the model origin
/// (tableCentre), in the model frame.
struct ModelPose {
    std::vector<Transform3> axes; ///< Per MachineModel::axes.
    Transform3 spindle = Transform3::Identity();
    Transform3 table = Transform3::Identity();
    /// The motion of what `carrier` carries (identity for the base).
    Transform3 carried(const MachineModel& model, std::string_view carrier) const;
};

/// Throws Error(InvalidArgument) for models with unknown parents or cycles.
ModelPose poseAt(const MachineModel& model, const AxisPosition& position);

/// The model a machine moves with: its own, or for machines without one the default model of
/// its kinematics (defaultModel). A lathe without a layout has none (an empty model).
MachineModel effectiveModel(const Machine& machine);

/// A kinematic tree for the machine's kinematics type, without parts: X and Y move the table
/// (Y on the base, X on Y), Z moves the spindle head; table rotaries sit on X (the outer one
/// first), head rotaries on Z; head-head machines move everything with the head over a fixed
/// table. Table rotaries turn about the model origin, head rotaries `pivotLength` above the
/// gauge point. The spindle stands `kDefaultSpindleHeight` above the table at machine zero.
/// A lathe with a layout: Z on the base, X on it (Y on X), T on the last of them carrying the tools
/// (the model's spindle carrier); C on the base; W and Z2 with C2 on it where it has them.
MachineModel defaultModel(const Machine& machine);
inline constexpr double kDefaultSpindleHeight = 500.0;

/// The end of the table chain: the axis that carries every axis not carrying the spindle (the
/// last of the chain they form), empty for the base (nothing but the spindle moves). Where the
/// work goes unless a setup's placement says otherwise. validateModel reports axes that do not
/// form one chain; here the deepest of them wins.
std::string tableEnd(const MachineModel& model);

/// The table rotary axes: the rotary axes that carry the table chain's end, outermost first.
std::vector<const ModelAxis*> tableRotaries(const MachineModel& model);

/// The table centre (solve's table frame origin) at machine zero, model frame: where the table
/// rotary axes meet (on the innermost one, the point nearest the outermost one's line); with one,
/// the point of its line nearest `near`; without any, the model origin.
Vec3 tableCentre(const MachineModel& model, const Vec3& near);

/// Where a setup's work sits on the machine (DL-0037): the axis that carries it (empty = the
/// base) and its WCS at machine zero (WCS -> model frame).
struct WorkMount {
    std::string carrier;
    Transform3 wcs = Transform3::Identity();
};

/// The mount's WCS in solve's table frame: relative to the table centre for its WCS origin, with
/// the model frame's axes.
Transform3 wcsInTable(const MachineModel& model, const WorkMount& mount);

/// The mount for a WCS given in solve's table frame, the work on the table chain's end (the
/// table centre taken for the model origin).
WorkMount tableMount(const MachineModel& model, const Transform3& wcsInTable);

/// Why `carrier` cannot carry a setup's work on this machine, or empty: it is on the spindle's
/// side, or a linear axis that moves the table does not move it.
std::string mountProblem(const MachineModel& model, std::string_view carrier);

/// The table rotary axes that do not carry `carrier` (they must stay at 0 for the work it
/// carries).
std::vector<std::string> rotariesNotCarrying(const MachineModel& model, std::string_view carrier);

/// The model's problems as sentences (empty = consistent with the kinematics), for machines
/// with a model.
std::vector<std::string> validateModel(const Machine& machine);

/// The linear axis positions that put the tool tip, `gaugeLength` below the spindle gauge point
/// along the tool axis, at `tip` (a point carried by `carrier`, given where it is at machine
/// zero) with the rotary axes at `rotary`. Nullopt when the linear axes cannot reach every
/// direction (fewer than three independent linear axes move the tool relative to it).
std::optional<AxisPosition> positionFor(const MachineModel& model, const IndexAngles& rotary,
                                        std::string_view carrier, const Vec3& tip,
                                        double gaugeLength);

/// The machine's collision components (Machine::components) as parts with meshes: head
/// components on the spindle's carrier, table components on the table chain's end at the table
/// centre, cradle components on cradleCarrier(). For display; checks use the components' exact
/// solids.
std::vector<ModelPart> componentParts(const Machine& machine, const MachineModel& model);

/// What carries the cradle components: the outer of two table rotary axes (A of an A/C
/// trunnion), else the first linear axis (or the base) under the table chain's end, so they
/// move with the table without turning.
std::string cradleCarrier(const MachineModel& model);

/// The folder a machine file's part files (STEP and STL) are stored in: <name>.meshes next to
/// it (named for the meshes of versions 2 to 4).
std::filesystem::path meshFolder(const std::filesystem::path& machineFile);

/// Writes the machine file and its part files (partFileName() in meshFolder(), removing files
/// no part uses). Throws Error(Io) on write failures and Error(InvalidArgument) for parts
/// whose files are not loaded.
void saveMachine(const Machine& machine, const std::filesystem::path& path);

/// The axis of the round face of `part` (cylinder, cone) at `point` (model frame at machine
/// zero, on the part's surface), in the model frame at machine zero: to set a rotary axis's pivot
/// and direction from the machine's geometry. Nullopt when the part has no mesh, the point is
/// not on it or the face there is not round.
std::optional<geom3d::AxisLine> axisAtPoint(const ModelPart& part, const Vec3& point);

/// The normal of `part`'s surface at `point` (model frame at machine zero, on the surface), in
/// the model frame at machine zero; nullopt when the point is not on the part.
std::optional<Vec3> normalAtPoint(const ModelPart& part, const Vec3& point);

/// The flat face of `part` at `point` (model frame at machine zero): its centre and normal, in
/// the model frame at machine zero; nullopt when the point is not on the part.
std::optional<geom3d::FlatFace> faceAtPoint(const ModelPart& part, const Vec3& point);

/// Where a line meets the plane through `point` with normal `normal`; nullopt when the line is
/// (nearly) parallel to the plane.
std::optional<Vec3> lineMeetsPlane(const geom3d::AxisLine& line, const Vec3& point,
                                   const Vec3& normal);

/// The centre line of the innermost rotary axis carrying `carrier` (model frame at machine zero)
/// and its name; nullopt when no rotary axis carries it.
std::optional<geom3d::AxisLine> rotaryLineOf(const MachineModel& model, std::string_view carrier,
                                             std::string* axisName = nullptr);

/// How a click on a face places a setup's work (its placement, DL-0037).
enum class Snap {
    FaceCentre, ///< The centre of the flat face clicked (a zero-point plate's top).
    AxisOnFace, ///< Where the innermost rotary axis carrying the part meets that face.
    Point,      ///< The point clicked.
};

/// The point a click at `point` (model frame at machine zero, on `part`) snaps to, model frame
/// at machine zero. Throws Error(InvalidArgument) with a message for the user when it cannot:
/// the point is not on the part or not on a flat face, no rotary axis carries the part, or the
/// face runs along the axis.
Vec3 snapOnPart(const MachineModel& model, const ModelPart& part, const Vec3& point, Snap snap);

/// The spindle gauge point from the spindle's centre line (`axis`, from a round face of the
/// spindle) and a point of the face the holder seats against (square to the line): where the
/// line meets that face.
Vec3 gaugePointOn(const geom3d::AxisLine& axis, const Vec3& seat);

/// `direction` (a line's, either sense) turned to point the way `previous` does; nullopt when
/// they are nearly perpendicular and the sense is not clear.
std::optional<Vec3> sameSense(const Vec3& direction, const Vec3& previous);

/// Mesh of a collision component in its carrier's frame (cylinders as 32-sided prisms).
geom3d::Mesh componentMesh(const Component& component);

/// A mesh-only part for `mesh` (its coordinates rounded to the stored precision, its key set):
/// parts imported from STL.
ModelPart makePart(std::string name, const geom3d::Mesh& mesh);

/// A STEP part for a STEP file's bytes (its key set; no mesh yet: document::meshMachineParts).
ModelPart stepPart(std::string name, std::shared_ptr<const std::string> stepFile);

/// The name a part's file is stored under: <step>.step or <mesh>.stl.
std::string partFileName(const ModelPart& part);

/// Bytes of the file a part is stored as: its STEP file, or the binary STL of its mesh.
std::string partFileBytes(const ModelPart& part);

/// Loads every part's file from `read(partFileName)` (nullopt when missing): a STEP part's bytes
/// (`stepFile`), a mesh part's mesh (`geometry`). Throws Error(Parse) naming `sourceName` for
/// missing or corrupt files.
void attachPartFiles(Machine& machine,
                     const std::function<std::optional<std::string>(const std::string&)>& read,
                     std::string_view sourceName);

} // namespace ocam::machine
