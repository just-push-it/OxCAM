// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Multi-axis kinematics (phase 9): the tool axis given by rotary positions (forward), the rotary
// positions that give a tool axis (inverse), machine positions with and without tool centre
// point control, and the solve step for simultaneous 4- and 5-axis toolpaths.
// See docs/design-log/0023.
//
// Frames: the table frame has the machine's axes with every rotary axis at 0 and its origin at
// the table centre (where the rotary axes meet; Model.hpp, tableCentre); the part is fixed in it,
// where the setup's placement puts it (wcsInTable). The machine frame has the same origin with
// every rotary axis at 0, and the spindle axis along +Z.

#pragma once

#include "ocam/machine/Machine.hpp"

#include <optional>
#include <string>
#include <vector>

namespace ocam::machine {

/// Tool axis in the table frame (unit vector from the tip towards the spindle) for the given
/// rotary positions.
Vec3 toolAxis(const Machine& machine, const IndexAngles& angles);

/// The rotary positions within the axis limits that give tool axis `axis` (table frame). The
/// continuous (rotating) axis is unwrapped by whole turns to lie nearest `near` when given.
/// Where the rotating axis does not change the tool axis (the tool along it), it keeps its
/// `near` value (0 without one). Empty when unreachable.
std::vector<IndexAngles> axisSolutions(const Machine& machine, const Vec3& axis,
                                       const IndexAngles* near = nullptr);

/// Machine X Y Z without tool centre point control, for a tool tip at `tip` (table frame,
/// relative to the table centre): the table turns the tip, and on head machines the tilted
/// head moves the tip by (pivot + tool length) (d - Z), d the tool axis in the machine frame.
/// `toolLength` is the gauge length (spindle gauge line to tip).
Vec3 machinePosition(const Machine& machine, const IndexAngles& angles, const Vec3& tip,
                     double toolLength);
/// Inverse of machinePosition: the tool tip in the table frame.
Vec3 tipFromMachine(const Machine& machine, const IndexAngles& angles, const Vec3& position,
                    double toolLength);

struct SolveOptions {
    /// Tool centre point control (the control keeps the tip on the programmed path while the
    /// rotary axes move): positions are the tool tip in the WCS, with its axes turned to the
    /// table's. Without it, positions are machine X Y Z relative to the table centre (machines
    /// that turn the part) or the WCS origin (head-head machines), and moves are split until
    /// the machine's linear interpolation keeps the tip within `tolerance`.
    bool tcp = true;
    double toolLength = 0.0; ///< Gauge length [mm] (head machines without TCP).
    double tolerance = 0.01; ///< [mm]
    /// Largest rotary change within one feed move before a warning (near singular positions).
    double maxRotaryStep = degToRad(30.0);
};

/// Rotary positions for each move end of a multi-axis toolpath (axes in the table frame given
/// by `axisInTable`), keeping the rotary axes continuous: from each possible start, solutions
/// nearest to the previous position are chosen, and the start with the least rotary travel
/// wins. Unreachable moves are reported in `errors` (by move index). On the moves marked in
/// `unwind` (rapids clear above the part) the rotating axis goes back by whole turns towards 0
/// when it stands more than half a turn from it, so loops round a part (swarf, DL-0085) can
/// repeat within the axis limits instead of flipping the tilt in a cut.
struct RotaryPlan {
    IndexAngles start;
    std::vector<IndexAngles> angles;
    std::vector<std::string> errors;
};
RotaryPlan planRotary(const Machine& machine, const std::vector<Vec3>& axisInTable,
                      const std::vector<char>& unwind = {});

/// Rotary positions along a toolpath (WCS) of a setup whose WCS is placed in the table frame by
/// `wcsInTable`, as solve chooses them: multi-axis toolpaths through planRotary, others at the
/// setup's 3+2 index on machines that can index, else 0. For the simulation of the machine
/// (DL-0037) and its collision checks; `errors` are those of planRotary. A positional toolpath
/// (an operation's own tool orientation, DL-0036) holds `index` for every move instead.
struct RotaryTrack {
    IndexAngles start;               ///< Before the first move.
    std::vector<IndexAngles> angles; ///< At the end of each move.
    std::vector<std::string> errors;
};
RotaryTrack rotaryTrack(const Machine& machine, const toolpath::Toolpath& path,
                        const Transform3& wcsInTable, const IndexAngles* index = nullptr);

/// How the setup's WCS sits on the table for a toolpath: `wcsInTable` for multi-axis toolpaths
/// and on machines that index; otherwise (a toolpath along WCS +Z on a machine that cannot
/// index) the part is clamped with its WCS square to the machine: the WCS axes are turned onto
/// the table's about the WCS origin.
Transform3 mountedWcs(const Machine& machine, const toolpath::Toolpath& path,
                      const Transform3& wcsInTable);

/// The rotary position that indexes the tool axis to `axis` (table frame) for positional (3+2)
/// machining: on table-table machines indexAngles (DL-0019); on the others the solution of
/// axisSolutions (the rotating axis nearest 0) with the smallest head tilt, then the smallest
/// rotation. Machines without rotary axes reach +Z only. nullopt when the limits do not allow
/// it.
std::optional<IndexAngles> indexSolution(const Machine& machine, const Vec3& axis);

/// Where the work offset of a positional toolpath is measured: a point of the table frame, with
/// the rotary axes at `angles`.
struct WorkOrigin {
    Vec3 point = Vec3::Zero();
    IndexAngles angles;
};

/// Positional (3+2) solve of a toolpath generated along its own frame's +Z, placed in the table
/// frame by `frameInTable`, with the rotary axes fixed at `angles` (which must turn the frame's
/// Z axis up the spindle; see indexSolution). Program positions are the machine X Y Z of each
/// tip (machinePosition with `options.toolLength`) relative to the machine position of
/// `origin`, so the program is right with the work offset measured there at any index. With
/// the tool along the machine Z (table rotations) XY arcs and canned cycles are kept; with a
/// tilted head arcs become lines, drilling cycles are expanded and tapping is refused. Each
/// move's axis is the tool axis in the table frame (a back-plot through the machine checks the
/// index); SolveResult::index holds `angles`.
SolveResult solveIndexed(const Machine& machine, const toolpath::Toolpath& path,
                         const Transform3& frameInTable, const IndexAngles& angles,
                         const WorkOrigin& origin, const SolveOptions& options);

/// Solves a toolpath for a setup whose WCS is placed in the table frame by `wcsInTable`.
/// Toolpaths with the tool along WCS +Z go through the 3-axis / 3+2 solve; multi-axis toolpaths
/// need a machine with rotary axes and get SolveResult::rotary. Arcs are split into straight
/// moves; canned cycles are not allowed in multi-axis toolpaths.
SolveResult solve(const Machine& machine, const toolpath::Toolpath& path,
                  const Transform3& wcsInTable, const SolveOptions& options);

} // namespace ocam::machine
