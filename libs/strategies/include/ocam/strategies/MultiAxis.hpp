// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Multi-axis milling (phase 9):
//  - 4th-axis wrapping: flat (unrolled) toolpaths wrapped onto a cylinder about the WCS X axis,
//    and wrapped pocketing of the recesses of a part turning about that axis;
//  - simultaneous 5-axis finishing with a ball end mill, the tool axis set by a user-defined
//    rule (fixed lead and tilt to the surface normal, a fixed direction, or through a point).
// Toolpaths give each move's tool axis (Toolpath.hpp). See docs/design-log/0024.

#pragma once

#include "ocam/strategies/Surface.hpp"

#include <functional>
#include <optional>
#include <vector>

namespace ocam::strategies {

// -- 4th-axis wrapping --------------------------------------------------------------------------

/// Maps a flat toolpath onto the cylinder of radius `radius` about the WCS X axis: x stays, y is
/// the arc length around the cylinder (angle y / radius, from +Z towards +Y), and z is the height
/// above the cylinder surface (radius radius + z). The tool axis is radial. Moves are split so
/// their chords stay within `chordTolerance` of the wrapped curves; arcs are sampled first.
/// Canned cycles are not supported (Error(Unsupported)).
toolpath::Toolpath wrapToolpath(const toolpath::Toolpath& flat, double radius,
                                double chordTolerance);

/// Point and radial tool axis of flat coordinates (x, y, z) wrapped at `radius`.
std::pair<Vec3, Vec3> wrapPoint(const Vec3& flat, double radius);

struct RotaryPocketParams {
    double stepDown = 1.0;       ///< Radial depth between levels [mm].
    double stepoverRatio = 0.45; ///< Of the tool diameter.
    double stockToLeaveXY = 0.0; ///< On the walls (in the unrolled plane).
    double stockToLeaveZ = 0.0;  ///< Radially, on the floors.
    double resolution = 0.0;     ///< Spacing of the radial height map; 0 = automatic.
    EntryKind entry = EntryKind::Ramp;
    double rampAngle = 3.0 * kPi / 180.0;
    CutDirection direction = CutDirection::Climb;
    /// The stock's radius at each x, when it is not all `stockRadius`: a turned part on a lathe.
    /// Only what is there gets cut, and only within reach of it does the tool go: a neck turned
    /// below the level is no recess to clear.
    std::function<double(double)> stockRadiusAt;
};

/// The part's radial height map about the WCS X axis: the largest radius of its surface at
/// each (x, angle), from rays cast towards the axis (0 where a ray misses the part).
struct RadialMap {
    double x0 = 0.0;
    double dx = 1.0;
    int nx = 0;
    int nt = 0; ///< Samples around; angle(j) = 2 pi j / nt.
    std::vector<double> radius;

    double at(int i, int j) const {
        return radius[static_cast<std::size_t>(i) * static_cast<std::size_t>(nt) +
                      static_cast<std::size_t>(j)];
    }
    double angle(int j) const { return 2.0 * kPi * j / nt; }
};
RadialMap radialMap(const geom3d::Mesh& part, double xMin, double xMax, double spacing,
                    double outerRadius, CancelToken& cancel);

/// Wrapped pocketing of the recesses of a part turning about the WCS X axis (slots, flats,
/// engraved features), between the stock radius and the part. At each radial level, from the
/// stock radius down to the deepest floor (each flat-bottomed floor gets a level of its own),
/// the region where the part lies below the level is pocketed in its own unrolled plane (arc
/// length at the level's radius, so tool offsets are exact at the floor) and wrapped back.
/// `xMin`, `xMax` limit the machined length. Throws Error(InvalidArgument) for a part that lies
/// across the axis rather than around it.
toolpath::Toolpath rotaryPocket(const geom3d::Mesh& part, double stockRadius, double xMin,
                                double xMax, const RotaryPocketParams& params, const Context& ctx);

/// Appends `wrapped`, a toolpath wrapped about the WCS X axis, to `tp`: the first sets `tp.start`
/// (`started` false), the others are joined by a rapid around the clearance cylinder of radius
/// `clearRadius`, the short way.
void appendWrapped(toolpath::Toolpath& tp, const toolpath::Toolpath& wrapped, double clearRadius,
                   double chordTolerance, bool& started);

// -- Wrapped contour (DL-0084) ------------------------------------------------------------------

/// A curve on a cylinder about the WCS X axis, as selected: a slot's centreline, the foot of a
/// wall, a groove or a line to engrave.
struct WrappedCurve {
    std::vector<Vec3> points; ///< In order (WCS); a closed one does not repeat its first point.
    bool closed = false;
};

/// A slot's floor on a cylinder about the WCS X axis: its faces, each by its boundary, and how
/// near it comes to the axis (a flat floor's middle). Faces that meet are one floor, also where
/// they meet across the angle where the unrolling starts.
struct WrappedSlot {
    std::vector<std::vector<WrappedCurve>> faces; ///< Each face's boundary: closed loops.
    double floorRadius = 0.0;
};

/// Where the tool goes along a curve, seen along its way: an open curve runs from its end at the
/// lower X (the same X: the lower angle), a closed one counter-clockwise seen from outside the
/// part, so left is inside it, and one that goes around the part with the angle.
enum class WrapSide { On, Left, Right };

struct RotaryContourParams {
    WrapSide side = WrapSide::On;
    double depth = 0.0;                  ///< Below the selection [mm].
    double stepDown = 0.0;               ///< Radially between levels [mm]; 0 = one level.
    double stepover = 0.0;               ///< Between passes across a slot wider than the tool [mm].
    double stockToLeaveXY = 0.0;         ///< On the walls (along the arc at the floor).
    double stockToLeaveZ = 0.0;          ///< Radially, on the floor.
    std::optional<double> leadRadius;    ///< Lead arcs [mm], 0 = none; unset: kContourLeadShare D.
    double leadLength = 0.0;             ///< Straight approach before the lead arc [mm].
    EntryKind entry = EntryKind::Plunge; ///< Plunge (beside the wall, with leads) or Ramp.
    double rampAngle = 3.0 * kPi / 180.0;
    CutDirection direction = CutDirection::Climb;
    /// As RotaryPocketParams: the stock's radius at each x on a turned part.
    std::function<double(double)> stockRadiusAt;
};

/// Follows curves and slots wrapped around a cylinder about the WCS X axis, the tool axis radial.
/// Each selection is unrolled at the radius its floor is cut at (angle times that radius, so
/// offsets are exact along the arc there), cut in levels from the stock down in that plane, and
/// wrapped back. Curves are cut on the line or a tool radius (and stock to leave) to the left or
/// right. A slot is cut on its centreline when it is the tool's width, else by offsets inside its
/// walls, stepping in by `stepover` until it is cleared; its ends at the part's ends (`partXMin`,
/// `partXMax`) are open. Throws Error(InvalidArgument) for a selection that reaches the axis or a
/// slot narrower than the tool.
toolpath::Toolpath rotaryContour(const std::vector<WrappedCurve>& curves,
                                 const std::vector<WrappedSlot>& slots, double stockRadius,
                                 double partXMin, double partXMax,
                                 const RotaryContourParams& params, const Context& ctx);

// -- Simultaneous 5-axis --------------------------------------------------------------------------

enum class ToolAxisMode {
    LeadTilt,  ///< The surface normal turned by `lead` towards the feed direction and `tilt`
               ///< to its left.
    Fixed,     ///< `fixedTilt` from WCS Z towards `fixedAzimuth` (from +X).
    FromPoint, ///< Along the line from `point` through the tool (away from a point below).
    ToPoint,   ///< Along the line from the tool to `point` (towards a point above).
};

struct ToolAxisParams {
    ToolAxisMode mode = ToolAxisMode::LeadTilt;
    double lead = 0.0; ///< [rad]
    double tilt = 0.0; ///< [rad]
    double fixedTilt = 0.0;
    double fixedAzimuth = 0.0;
    Vec3 point = Vec3::Zero();  ///< WCS
    double maxTilt = kPi / 3.0; ///< Largest angle from WCS Z; steeper axes are limited.
};

/// The tool axis by `params` at a ball centre with surface normal `normal` and feed direction
/// `feed` (unit vectors, WCS).
Vec3 toolAxisFor(const ToolAxisParams& params, const Vec3& centre, const Vec3& normal,
                 const Vec3& feed);

struct MultiAxisFinishParams {
    ParallelParams pattern; ///< Ball centres from 3-axis parallel finishing.
    ToolAxisParams axis;
    double clearance = 0.2; ///< Tool flutes and shank versus the part [mm].
};

/// Simultaneous 5-axis finishing with a ball end mill: the ball centres of parallel finishing
/// (exact, from drop-cutter) with the tool axis set by `params.axis`; the tip is the centre less
/// the ball radius along the axis, so the ball touches the part where it would in 3 axes.
/// Points where the tilted flutes or shank come closer than `clearance` to the part are
/// reported in the warnings (no automatic avoidance).
toolpath::Toolpath multiAxisFinish(const geom3d::Mesh& part, double floorZ,
                                   const MultiAxisFinishParams& params, const Context& ctx);

// -- Swarf (flank) milling (DL-0085) ----------------------------------------------------------

/// A ruling of a wall: points along it, bottom first, with the wall's outward unit normals (the
/// side the tool is on).
struct WallRuling {
    std::vector<Vec3> points;
    std::vector<Vec3> normals;
    Vec3 direction = Vec3::UnitZ(); ///< Unit, bottom to top.
};

/// A ruled wall: its rulings across it, in order, as cad::faceRulings gives them.
struct RuledWall {
    std::string name; ///< For messages, e.g. "face 12".
    std::vector<WallRuling> rulings;
};

/// A flat floor the tool stays above: a point on it and its upward unit normal.
struct SwarfFloor {
    Vec3 point = Vec3::Zero();
    Vec3 normal = Vec3::UnitZ();
};

struct SwarfParams {
    int depthPasses = 1;     ///< Passes down the rulings, the deepest last.
    double axisOffset = 0.0; ///< Tool end above the floor, or the wall's bottom, along the axis.
    double stockToLeave = 0.0;
    double tolerance = 0.01;    ///< Twist error of the flank the walls may give [mm].
    double maxTilt = kPi / 3.0; ///< Steeper rulings are refused.
    double leadDistance = 2.0;  ///< Onto and off the wall along its normal.
    double clearance = 0.2;     ///< Flutes and shank versus the part [mm].
    CutDirection direction = CutDirection::Climb;
};

/// Swarf milling: the side of a flat, bull-nose or ball end mill along the rulings of the walls,
/// the tool axis through the ends of each ruling offset by the tool radius along their normals.
/// Walls that share a ruling are cut in one pass, turning about it at convex corners. Refuses
/// rulings steeper than `maxTilt`, inside corners, and twisted walls the flank would leave or cut
/// into by more than the tolerance. Points where the flutes or shank come closer than
/// `clearance` to the part are reported (no automatic avoidance).
toolpath::Toolpath swarf(const geom3d::Mesh& part, const std::vector<RuledWall>& walls,
                         const std::optional<SwarfFloor>& floor, const SwarfParams& params,
                         const Context& ctx);

// -- Parameter objects ------------------------------------------------------------------------

SwarfParams swarfParams(const Json& values);
RotaryPocketParams rotaryPocketParams(const Json& values, const tooling::Tool& tool,
                                      const tooling::CuttingPreset* preset);
RotaryContourParams rotaryContourParams(const Json& values, const tooling::Tool& tool,
                                        const tooling::CuttingPreset* preset);
MultiAxisFinishParams multiAxisFinishParams(const Json& values, const tooling::Tool& tool,
                                            const tooling::CuttingPreset* preset);

} // namespace ocam::strategies
