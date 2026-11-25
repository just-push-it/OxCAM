// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Inputs shared by all toolpath strategies. Strategies are pure functions of neutral geometry
// (regions, heights, hole positions) in WCS coordinates; they never see B-rep or the document.

#pragma once

#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Progress.hpp>
#include <ocam/tooling/Tool.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ocam::strategies {

/// Feed rates [mm/s] and spindle speed [rev/s] for one operation. Nothing is assumed: they
/// are zero until set.
struct Feeds {
    double cut = 0.0;
    double plunge = 0.0;
    double ramp = 0.0;
    double lead = 0.0;
    double partOffEnd = 0.0; ///< The end of a part-off.
    double pull = 0.0;       ///< A bar pull, the chuck open (DL-0091).
    double spindle = 0.0;
    tooling::Coolant coolant = tooling::Coolant::Flood;
};

/// The feeds of a cutting preset for the given tool, exactly as the preset gives them: the
/// spindle speed from vc, the feed rate fz z n for cuts, the lead, ramp and plunge feeds as set
/// (for taps the plunge is the pitch times the spindle speed). A value the preset leaves at
/// zero stays zero; without a preset everything is zero.
Feeds feedsFor(const tooling::Tool& tool, const tooling::CuttingPreset* preset);

/// Absolute WCS heights [mm].
struct Heights {
    double clearance = 15.0;    ///< Rapid level between regions and operations.
    double retract = 5.0;       ///< Rapid level within an operation.
    double feedClearance = 1.0; ///< Distance above uncut material where feed moves start.
};

struct Context {
    const tooling::Tool& tool;
    Feeds feeds;
    Heights heights;
    double chordTolerance = 0.002; ///< Geometry discretization / arc fitting tolerance [mm].
    CancelToken& cancel;
    ProgressSink& progress;
};

/// Starts a toolpath with the tool, spindle and coolant filled in and the tool at clearance
/// height above `startXY`.
toolpath::Toolpath beginToolpath(const Context& ctx, std::string_view strategy,
                                 const Vec2& startXY);

/// Evenly spaced depth levels from just below `top` down to `bottom` (inclusive), with at most
/// `stepDown` between levels. stepDown <= 0 gives a single level at `bottom`.
std::vector<double> depthLevels(double top, double bottom, double stepDown);

// -- Parameters ------------------------------------------------------------------------------

enum class ParamKind {
    Length, ///< mm
    Angle,  ///< degrees in the parameter object, radians in code
    Integer,
    Boolean,
    Choice,       ///< one of `choices`
    Time,         ///< seconds
    SurfaceSpeed, ///< m/min in the parameter object, mm/s in code
    Feed,         ///< mm/min in the parameter object, mm/s in code (no tool holds it, ADR-0007)
};

/// When a parameter matters: always, or only for some values of another parameter of the same
/// strategy (a ramp angle only with a ramp or helix entry, the rest resolution only with rest
/// machining on). The interface shows a parameter only when it matters.
struct ParamCondition {
    std::string key;                 ///< The parameter it depends on; empty: always.
    std::vector<std::string> values; ///< A choice's values, or "on"/"off" for a flag, that show it.
    double above = -1e300;           ///< For a number: shown when that parameter is above this.
};

struct ParamDef {
    std::string key;
    std::string label;
    ParamKind kind = ParamKind::Length;
    double defaultValue = 0.0;
    double min = 0.0;
    double max = 1e6;
    std::vector<std::pair<std::string, std::string>> choices; ///< (value, label)
    std::string defaultChoice;
    std::string help;
    ParamCondition onlyWhen;
    /// The value while the operation does not set it, from its tool (a helix radius from the
    /// tool radius, say); empty = `defaultValue`. The interface shows it as the value.
    std::function<double(const tooling::Tool&)> toolDefault;
};

/// What an operation's selected geometry is for (the interface offers a geometry selection only
/// when the strategy reads one).
enum class GeometryUse {
    None,         ///< Not used.
    Faces,        ///< Faces to machine (a pocket floor, a top face).
    FacesOrEdges, ///< Faces or closed loops of edges (contours).
    Edges,        ///< Closed loops of edges across the tool axis (chamfers).
    Chains,       ///< Edges chained open or closed, at any slope (traces).
    AnyEdges,     ///< Edges of any shape, one by one (3D edge breaking).
    Holes,        ///< Faces of holes (drilling, tapping, boring).
    Containment,  ///< Faces whose outline contains the tool (3D finishing): none = everywhere.
    Walls,        ///< Ruled walls to cut with the side of the tool, and a floor (swarf).
};

struct StrategyInfo {
    std::string id;    ///< e.g. "pocket2d"
    std::string label; ///< e.g. "2D pocket"
    std::vector<ParamDef> params;
    GeometryUse geometry = GeometryUse::None;
    std::string description; ///< What the strategy is for, in a sentence or two.

    const ParamDef* find(std::string_view key) const;
    /// Whether a parameter matters for these values (defaults where unset): its condition holds.
    bool applies(const ParamDef& def, const Json& values) const;
};

const std::vector<StrategyInfo>& strategies();
const StrategyInfo* findStrategy(std::string_view id);

/// Reads typed parameter values from an operation's parameter object, applying defaults and
/// range checks. Throws Error(InvalidArgument) naming the offending key.
class ParamReader {
public:
    ParamReader(const StrategyInfo& info, const Json& values);

    double number(std::string_view key) const; ///< Lengths in mm, angles in radians.
    int integer(std::string_view key) const;
    bool flag(std::string_view key) const;
    std::string choice(std::string_view key) const;
    /// True when the key is present in the parameter object (not just defaulted).
    bool isSet(std::string_view key) const;
    const StrategyInfo& info() const noexcept { return info_; }

private:
    const ParamDef& def(std::string_view key) const;
    const StrategyInfo& info_;
    const Json& values_;
};

} // namespace ocam::strategies
