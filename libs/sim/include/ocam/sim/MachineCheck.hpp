// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Machine collision checks (phase 9, machine models DL-0037): the machine's parts (the mesh
// parts of its model and its collision components: spindle head, table, trunnion cradle) moved
// through a toolpath by the machine's kinematics, against each other, the part, the stock, the
// fixtures and the tool assembly (the tool against the fixtures is FixtureCheck's). Checks run on
// sampled poses with a clearance, using convex solids (GJK) and triangle hierarchies for meshes.
// See docs/design-log/0025 and 0037.

#pragma once

#include "ocam/sim/FixtureCheck.hpp"
#include "ocam/sim/ToolShape.hpp"

#include <ocam/foundation/Cancel.hpp>
#include <ocam/geom3d/Mesh.hpp>
#include <ocam/machine/Kinematics.hpp>
#include <ocam/machine/Model.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <optional>
#include <string>
#include <vector>

namespace ocam::sim {

struct MachineCollision {
    std::size_t firstMove = 0; ///< Index into Toolpath::moves.
    std::size_t lastMove = 0;
    std::string first; ///< A machine part or component.
    /// What it comes too close to: "part", "stock", "tool", "tool shank", "tool holder", a
    /// fixture or another machine part.
    std::string second;
    Vec3 position = Vec3::Zero(); ///< A point of the contact (WCS).
    double distance = 0.0;        ///< Smallest distance found (0 = overlap) [mm].
    /// A lathe's machine between operations, before the first move of this one: what it was doing
    /// (machine::toString(LatheStep::Kind)); empty for the toolpath's own moves.
    std::string during;
};

struct MachineCheckOptions {
    double clearance = 1.0;   ///< Distances below this count [mm].
    double step = 2.0;        ///< Largest tip travel between checked poses [mm].
    double angleStep = 0.035; ///< Largest rotary change between checked poses [rad].
    std::size_t maxCollisions = 200;
    /// Positional toolpaths (an operation's own tool orientation, 3+2): the rotary positions the
    /// axes stay at for the whole toolpath, instead of planning them from its tool axes.
    std::optional<machine::IndexAngles> index;
};

/// From the tool tip to the spindle gauge point: the top of the holder, or the tool's visible
/// length without one [mm].
double gaugeLength(const ToolShape& tool);

/// Checks a toolpath (CL, WCS) of a setup whose work sits on the machine as `mount` has it (the
/// setup's placement). The machine moves by its model (machine::effectiveModel): rotary positions
/// as solve plans them (machine::rotaryTrack; `options.index` for positional toolpaths), the
/// linear axes so the tool tip follows the path (machine::positionFor). `partWcs`, `stockWcs`
/// (the stock's bounding box) and `fixturesWcs` are carried by the mount's axis in the WCS
/// (mounted as machine::mountedWcs has it).
///
/// Pairs of things that move together (on the same axis) or that sit on each other (an axis
/// and what carries it, the table and the part, the spindle and the tool) are not checked; nor
/// is the tool against the part and stock (the stock simulation does that) or the fixtures
/// (checkFixtures). Returns nothing for machines without collision components or colliding
/// model parts.
std::vector<MachineCollision>
checkMachine(const machine::Machine& machine, const toolpath::Toolpath& path,
             const machine::WorkMount& mount, const geom3d::Mesh& partWcs,
             const geom3d::Aabb3& stockWcs, const std::vector<FixtureBody>& fixturesWcs,
             const ToolShape& tool, const MachineCheckOptions& options = {},
             CancelToken* cancel = nullptr);

/// The same with the WCS given in solve's table frame, the work on the table chain's end
/// (machine::tableMount), without fixtures.
std::vector<MachineCollision>
checkMachine(const machine::Machine& machine, const toolpath::Toolpath& path,
             const Transform3& wcsInTable, const geom3d::Mesh& partWcs,
             const geom3d::Aabb3& stockWcs, const ToolShape& tool,
             const MachineCheckOptions& options = {}, CancelToken* cancel = nullptr);

} // namespace ocam::sim
