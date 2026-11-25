// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// 2.5D milling and drilling strategies (phase 3). All geometry is in WCS coordinates; the tool
// axis is +Z. Each function validates its inputs (Error(InvalidArgument)), honours the cancel
// token (OperationCancelled) and reports non-fatal issues in Toolpath::warnings.
//
// Design notes and sources: docs/design-log/0006 to 0009.

#pragma once

#include "ocam/strategies/Common.hpp"

#include <ocam/geom2d/Region.hpp>

#include <functional>

namespace ocam::strategies {

// -- Facing --------------------------------------------------------------------------------

enum class FacingPattern { ZigZag, OneWay };

struct FacingParams {
    double stepoverRatio = 0.7; ///< Fraction of the tool diameter.
    double stepDown = 0.0;      ///< 0 = single pass.
    double stockToLeave = 0.0;  ///< Axial.
    double angle = 0.0;         ///< Pass direction from +X [rad].
    FacingPattern pattern = FacingPattern::ZigZag;
    double overrun = 2.0; ///< Tool edge travel beyond the stock outline [mm].
};

/// Faces the stock outline (`area`) from `topZ` down to `finalZ` (+ stock to leave).
toolpath::Toolpath facing(const geom2d::Regions& area, double topZ, double finalZ,
                          const FacingParams& params, const Context& ctx);

// -- Pocket ---------------------------------------------------------------------------------

enum class EntryKind { Helix, Ramp, Plunge };
enum class CutDirection { Climb, Conventional };

struct PocketParams {
    double stepoverRatio = 0.45;
    double stepDown = 0.0; ///< 0 = single level.
    double stockToLeaveXY = 0.0;
    double stockToLeaveZ = 0.0;
    EntryKind entry = EntryKind::Helix;
    double rampAngle = 2.0 * kPi / 180.0;
    std::optional<double> helixRadius; ///< Helix path radius [mm]; unset: kHelixRadiusShare r.
    CutDirection direction = CutDirection::Climb;
    /// Rest machining (DL-0013): the XY regions where material remains between a level `z` and
    /// the level above it (`zAbove`, +infinity for the first level). When set, each level clears
    /// only around remaining material; levels without any are skipped.
    std::function<geom2d::Regions(double z, double zAbove)> restMaterial;
    /// Remaining material narrower than about twice this is ignored (raster slivers) [mm].
    double restOpening = 0.0;
};

/// Clears `pocket` (outer boundary plus islands as holes, in XY) from `topZ` down to `bottomZ`
/// with contour-parallel offset loops, innermost first.
toolpath::Toolpath pocket(const geom2d::Regions& pocket, double topZ, double bottomZ,
                          const PocketParams& params, const Context& ctx);

// -- Contour --------------------------------------------------------------------------------

enum class ContourSide { Outside, Inside };

// Values taken from the tool when an operation does not set them (the interface shows them as
// lengths for the chosen tool): shares of the tool radius r or diameter D.
inline constexpr double kHelixRadiusShare = 0.8;  ///< Helix entry radius, of r.
inline constexpr double kContourLeadShare = 0.5;  ///< Contour lead arc radius, of D.
inline constexpr double kChamferLeadShare = 0.25; ///< Chamfer lead arc radius, of D.
inline constexpr double kThreadLeadShare = 0.5;   ///< External thread milling lead arc, of D.
inline constexpr double kPeckShare = 1.0;         ///< Peck depth, of D.
inline constexpr double kPlungePitchShare = 0.5;  ///< Plunge spacing along a row, of D.
enum class CompensationMode { Computer, Control };

enum class TabShape {
    Square,  ///< Straight up at one end, across at the top, straight down at the other.
    Triangle ///< Ramps up and down, leaving a triangle of material (DL-0080).
};

/// Holding tabs: bridges left uncut along each closed outline, so the part stays attached to the
/// stock (DL-0080). The tool lifts over them on the levels below their top; every move over a tab
/// is at the cut feed. The values are the operation's (contour2d's ParamDefs hold the defaults).
struct ContourTabs {
    bool enabled = false;
    int count = 0;        ///< Tabs per closed outline, when `spacing` is 0.
    double spacing = 0.0; ///< > 0: about one tab per this length of outline (at least one) [mm].
    /// Along the outline at its narrowest, where the tool's edge passes [mm]: the tool centre
    /// stays lifted over the width plus the tool diameter.
    double width = 0.0;
    double height = 0.0; ///< Above the contour's lowest level [mm].
    TabShape shape = TabShape::Square;
};

/// Where the tabs go along a closed loop of the tool path: the arc lengths (from its first
/// vertex) of their centres, for a tab whose lifted stretch is `halfSpan` either side. Evenly
/// spaced, each moved by up to 40% of the spacing off corners, short segments and the loop's
/// start (where the leads are) when it can be.
std::vector<double> tabCentres(const geom2d::Polyline& loop, const ContourTabs& tabs,
                               double halfSpan, double seamMargin);

struct ContourParams {
    ContourSide side = ContourSide::Outside;
    double stockToLeaveXY = 0.0;
    double stockToLeaveZ = 0.0;
    double stepDown = 0.0;     ///< 0 = single level.
    double bottomOffset = 0.0; ///< Extra depth below the floor (e.g. break-through).
    int passes = 1;            ///< Radial passes.
    double passSpacing = 0.5;  ///< Between radial passes [mm].
    /// Repeat the final pass once more at full depth (removes what tool deflection left).
    bool springPass = false;
    /// Start each loop at its point nearest to this (WCS XY) instead of the automatic one.
    std::optional<Vec2> start;
    /// Lead-in/out arc radius [mm], 0 = straight on; unset: kContourLeadShare D.
    std::optional<double> leadRadius;
    double leadLength = 0.0; ///< Straight approach before the lead arc [mm].
    CutDirection direction = CutDirection::Climb;
    CompensationMode compensation = CompensationMode::Computer;
    ContourTabs tabs;
};

/// Profiles the boundary of `shape` (outside: around it; inside: within it, including islands).
toolpath::Toolpath contour(const geom2d::Regions& shape, double topZ, double bottomZ,
                           const ContourParams& params, const Context& ctx);

// -- Drilling -------------------------------------------------------------------------------

struct HoleTarget {
    Vec2 position = Vec2::Zero();
    double topZ = 0.0;
    double depth = 0.0; ///< Depth of the cylindrical part below topZ.
    bool through = false;
    double diameter = 0.0; ///< Hole diameter (0 = unknown); used by boring and thread milling.
};

enum class DepthMode { Tip, FullDiameter };

struct DrillParams {
    std::optional<toolpath::CycleType> cycle; ///< nullopt = choose from tool type and depth.
    std::optional<double> peckDepth;          ///< [mm]; unset: kPeckShare D.
    double dwell = 0.0;                       ///< [s]
    double retractClearance = 2.0;            ///< R plane above each hole top [mm].
    /// An absolute R plane (the operation's feed height) instead; never below a hole's top.
    std::optional<double> rPlaneZ;
    DepthMode depthMode = DepthMode::FullDiameter;
    double breakthrough = 1.0; ///< Extra depth for through holes [mm].
    double holeDepth = 0.0;    ///< The operation's own depth [mm]; 0 = the holes' (see HoleTarget).
};

toolpath::Toolpath drill(const std::vector<HoleTarget>& holes, const DrillParams& params,
                         const Context& ctx);

// -- Chamfer --------------------------------------------------------------------------------

struct ChamferParams {
    ContourSide side = ContourSide::Outside;
    double width = 0.5;               ///< Horizontal chamfer width [mm]; the angle is the tool's.
    double tipOffset = 0.5;           ///< Tool tip below the chamfer's lower edge [mm].
    std::optional<double> leadRadius; ///< [mm], 0 = straight on; unset: kChamferLeadShare D.
    CutDirection direction = CutDirection::Climb;
};

/// Where a pointed tool must run to cut a chamfer of `width` (see DL-0014).
struct ChamferGeometry {
    double halfAngle = 0.0;    ///< Half the tool's point angle [rad] = chamfer angle from vertical.
    double depth = 0.0;        ///< Vertical size of the chamfer [mm].
    double centreOffset = 0.0; ///< Tool axis distance from the edge [mm].
    double tipBelowEdge = 0.0; ///< Tool tip depth below the edge [mm].
};
ChamferGeometry chamferGeometry(const tooling::Tool& tool, double width, double tipOffset);

/// Chamfers the boundary of `shape` (outside or inside edges) lying at height `edgeZ`.
toolpath::Toolpath chamfer(const geom2d::Regions& shape, double edgeZ, const ChamferParams& params,
                           const Context& ctx);

// -- Trace ----------------------------------------------------------------------------------

/// Where the tool centre runs from a traced line, looking along the way the tool travels.
enum class TraceSide { On, Left, Right };

/// A chain of edges to trace, flattened onto XY: open, or closed (it ends where it starts).
struct TraceChain {
    geom2d::Polyline path;
    double z = 0.0; ///< Its height: the depth is measured from here (unless on a surface).
};

struct TraceParams {
    double depth = 0.0;    ///< Below the chain (or the surface) [mm]; 0 = on it.
    double stepDown = 0.0; ///< Per level [mm]; 0 = the full depth at once.
    TraceSide side = TraceSide::On;
    double offset = 0.0;      ///< Left/Right: gap between the tool's side and the line [mm].
    int passes = 1;           ///< Left/Right: side-by-side passes, the farthest first.
    double passSpacing = 0.5; ///< Between the passes [mm].
    EntryKind entry = EntryKind::Plunge;  ///< Plunge or Ramp (Helix is refused).
    double rampAngle = 2.0 * kPi / 180.0; ///< [rad], for Ramp.
    bool reverse = false;                 ///< Run each chain from its other end.
    /// An open chain starts from its end nearer this, a closed one at its point nearest it;
    /// unset: an open chain from its end with the lower X (then Y), a closed one at
    /// preferredStart. Closed chains run counter-clockwise before `reverse`.
    std::optional<Vec2> start;
    /// Spacing of the surface samples along the path [mm] when following a surface.
    double sampleStep = 0.5;
};

/// The tool tip's height on the surface for a tool centre at a point (drop-cutter).
using TraceSurface = std::function<double(const Vec2&)>;

/// The tool centre along each chain, offset to a side if asked, level by level down to the
/// depth below the chain's height or, with `surface`, below the surface under the tool
/// (DL-0088). Plunges and ramps are fed at the plunge and ramp feeds.
toolpath::Toolpath trace(const std::vector<TraceChain>& chains, const TraceParams& params,
                         const Context& ctx, const TraceSurface& surface = {});

/// The tool-centre paths a trace runs along `chain` for one pass at `distance` from it (signed:
/// positive to the left of its direction, 0 on it), in the chain's direction: one path on it,
/// several or none when the offset splits or the tool does not fit. Open chains are offset
/// segment by segment, joined round at outside corners, and trimmed where they come nearer the
/// chain than `distance`.
std::vector<geom2d::Polyline> traceOffset(const geom2d::Polyline& chain, double distance,
                                          double tolerance);

// -- Tapping, thread milling, boring ----------------------------------------------------------

struct TapParams {
    double threadDepth = 0.0; ///< 0 = the full hole depth.
    double retractClearance = 2.0;
    std::optional<double> rPlaneZ; ///< See DrillParams.
    double breakthrough = 0.0;     ///< Extra depth for through holes [mm].
};

/// Rigid tapping (G84) with a tap; feed = pitch × spindle speed.
toolpath::Toolpath tap(const std::vector<HoleTarget>& holes, const TapParams& params,
                       const Context& ctx);

struct ThreadMillParams {
    /// Internal: in holes. External: around bosses and studs (HoleTarget is then the boss: its
    /// top, its length below it and its diameter).
    bool external = false;
    double pitch = 0.0; ///< 0 = the thread mill's pitch.
    /// 0 = internal: hole diameter + 1.0825 × pitch (ISO tap drill size); external: the boss's.
    double majorDiameter = 0.0;
    /// Radial depth of the thread form; 0 = 0.6134 × pitch external, 0.5413 × pitch internal.
    double threadDepth = 0.0;
    double depth = 0.0; ///< Thread length; 0 = the hole's depth or the boss's length.
    bool rightHand = true;
    CutDirection direction = CutDirection::Climb;
    int radialPasses = 1;    ///< Equal radial steps to the full thread depth.
    bool springPass = false; ///< The last pass again, at full depth.
    bool multiForm = false;  ///< One turn with a multi-form tool, or one per pitch (single point).
    /// External: the tangent arc onto the helix, from outside [mm]; unset: kThreadLeadShare D.
    std::optional<double> leadRadius;
    double retractClearance = 2.0;
    std::optional<double> rPlaneZ; ///< See DrillParams.
};

/// Where a thread mill's centre runs (DL-0081).
struct ThreadMillGeometry {
    double major = 0.0;       ///< Major diameter [mm].
    double threadDepth = 0.0; ///< Radial depth of the form [mm].
    /// Helix radius of each radial pass, the last one at full depth: internal major / 2 − r,
    /// external major / 2 − thread depth + r.
    std::vector<double> passRadii;
    bool counterClockwise = true; ///< Seen from the tool's side, as the tool goes around.
    bool upward = true;           ///< The helix climbs towards the tool (out of the hole).
};
/// The geometry for a hole or boss of diameter `diameter` (0 = unknown) and a tool of radius
/// `toolRadius`; throws when the thread cannot be found (no major diameter for an unknown size).
ThreadMillGeometry threadMillGeometry(const ThreadMillParams& params, double pitch, double diameter,
                                      double toolRadius);

/// Threads by helical interpolation. Internal: leads from the hole's axis on a semicircle.
/// External: from outside the major diameter on a tangent arc of `leadRadius`. The hand and
/// the cut direction set which way the helix turns and whether it climbs or descends.
toolpath::Toolpath threadMill(const std::vector<HoleTarget>& holes, const ThreadMillParams& params,
                              const Context& ctx);

struct BoreParams {
    double helixPitch = 1.0;   ///< Depth per revolution for helical boring [mm].
    double stockToLeave = 0.0; ///< Radial [mm].
    double retractClearance = 2.0;
    std::optional<double> rPlaneZ; ///< See DrillParams.
    double breakthrough = 0.5;     ///< Extra depth for through holes [mm].
    double holeDepth = 0.0;        ///< The operation's own depth [mm]; 0 = the holes'.
    CutDirection direction = CutDirection::Climb;
};

/// Reamers and boring bars: boring cycle (G85). Other milling tools: helical interpolation
/// down the hole wall and a full circle at the bottom.
toolpath::Toolpath bore(const std::vector<HoleTarget>& holes, const BoreParams& params,
                        const Context& ctx);

// -- Plunge roughing ---------------------------------------------------------------------------

struct PlungeParams {
    double stepoverRatio = 0.5;  ///< Row spacing as a fraction of the tool diameter.
    std::optional<double> pitch; ///< Plunge spacing along a row [mm]; unset: kPlungePitchShare D.
    bool stagger = true;         ///< Alternate rows shifted by half a pitch.
    bool wall = true;            ///< Plunges along the area's boundary, last.
    double stockToLeaveXY = 0.0;
    double stockToLeaveZ = 0.0;
};

/// Roughs `area` from `topZ` down to `bottomZ` by plunges along the tool axis at overlapping
/// positions (DL-0027). Between plunges the tool hops to the feed clearance above `topZ`.
toolpath::Toolpath plungeRough(const geom2d::Regions& area, double topZ, double bottomZ,
                               const PlungeParams& params, const Context& ctx);

// -- Parameter objects ------------------------------------------------------------------------

/// The step down and stepover of every strategy are the cutting preset's axial depth (ap) and
/// radial depth (ae), which an operation may change for itself (document::PresetOverrides);
/// an error names the preset when it has none.
double presetStepDown(const tooling::CuttingPreset* preset);
double presetStepover(const tooling::CuttingPreset* preset);
/// ae as a fraction of the tool's diameter, at most 1.
double presetStepoverRatio(const tooling::CuttingPreset* preset, const tooling::Tool& tool);

/// Converts an operation's parameter object to typed parameters (defaults for missing keys, the
/// step down and stepover from the preset, and its ramp angle where unset).
FacingParams facingParams(const Json& values, const tooling::Tool& tool,
                          const tooling::CuttingPreset* preset);
PocketParams pocketParams(const Json& values, const tooling::Tool& tool,
                          const tooling::CuttingPreset* preset);
ContourParams contourParams(const Json& values, const tooling::Tool& tool,
                            const tooling::CuttingPreset* preset);
PlungeParams plungeParams(const Json& values, const tooling::Tool& tool,
                          const tooling::CuttingPreset* preset);
DrillParams drillParams(const Json& values);
ChamferParams chamferParams(const Json& values);
/// The step down is the preset's (presetStepDown) only when there is a depth to step down.
TraceParams traceParams(const Json& values, const tooling::CuttingPreset* preset);
/// Whether a trace keeps its depth below the part's surface (its "follow_surface").
bool traceFollowsSurface(const Json& values);
TapParams tapParams(const Json& values);
ThreadMillParams threadMillParams(const Json& values);
BoreParams boreParams(const Json& values);

} // namespace ocam::strategies
