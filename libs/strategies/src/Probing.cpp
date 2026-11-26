// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Probing cycles with a spindle probe. See docs/design-log/0090.

#include "ocam/strategies/Probing.hpp"

#include "Internal.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/toolpath/PathBuilder.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace ocam::strategies {
namespace {

using toolpath::ProbeFeature;

constexpr std::array<std::string_view, 5> kIds{"probe_surface", "probe_width", "probe_circle",
                                               "probe_corner", "probe_check"};

ParamDef number(std::string key, std::string label, ParamKind kind, double def, double min,
                double max, std::string help) {
    return {std::move(key), std::move(label), kind, def, min, max, {}, {}, std::move(help)};
}
ParamDef length(std::string key, std::string label, double def, double min, double max,
                std::string help) {
    return number(std::move(key), std::move(label), ParamKind::Length, def, min, max,
                  std::move(help));
}
ParamDef choice(std::string key, std::string label,
                std::vector<std::pair<std::string, std::string>> options, std::string def,
                std::string help) {
    return {std::move(key),     std::move(label), ParamKind::Choice, 0, 0, 0,
            std::move(options), std::move(def),   std::move(help)};
}
ParamDef when(ParamDef d, std::string key, std::vector<std::string> values) {
    d.onlyWhen.key = std::move(key);
    d.onlyWhen.values = std::move(values);
    return d;
}

constexpr double kFar = 1e5; // coordinates' range [mm]

const char* const kPick =
    "Picking the measured faces on the model fills this in (WCS); you can also type it";
const char* const kHeight =
    "Where the stylus ball's centre is while it touches the faces (WCS Z). Picking the faces "
    "fills in their middle; it must leave the ball clear of the floor and the top";

std::vector<std::pair<std::string, std::string>> directions() {
    return {{"z_minus", "-Z: down onto a face that looks up"},
            {"x_plus", "+X: onto a face that looks towards -X"},
            {"x_minus", "-X: onto a face that looks towards +X"},
            {"y_plus", "+Y: onto a face that looks towards -Y"},
            {"y_minus", "-Y: onto a face that looks towards +Y"}};
}

std::vector<std::pair<std::string, std::string>> axes() {
    return {{"x", "Across X"}, {"y", "Across Y"}};
}

/// The parameters every cycle has: its moves' feeds and distances, and what it sets.
void addCommon(std::vector<ParamDef>& params, bool check) {
    params.push_back(
        number("fast_feed", "Positioning feed", ParamKind::Feed, 1000.0, 0.0, 1e5,
               "Feed of the protected moves that take the probe to each touch and back, below the "
               "retract height: the control stops them if the stylus touches anything. Check it "
               "against your probe's data"));
    params.push_back(number(
        "measure_feed", "Measuring feed", ParamKind::Feed, 100.0, 0.0, 1e5,
        "Feed of the moves that touch the surface and measure it; the probe is calibrated at a "
        "feed, so use that one"));
    params.push_back(length("approach", "Approach distance", 5.0, 0.1, 100.0,
                            "How far from the surface each measuring move starts"));
    params.push_back(length("overtravel", "Overtravel", 5.0, 0.1, 100.0,
                            "How far past the nominal surface a measuring move goes before the "
                            "control gives up: a surface further away than this is not found"));
    if (check) {
        params.push_back(length("tolerance", "Tolerance", 0.05, 0.0, 100.0,
                                "How far the measured position or size may be from the model's "
                                "before the check fails; what the control does then is the "
                                "post's"));
    } else {
        params.push_back(number("work_offset", "Work offset to set", ParamKind::Integer, 0.0, 0.0,
                                999.0,
                                "The work offset the measurement sets, so the feature is where "
                                "the model has it: 1 = G54, 2 = G55, ...; 0 = the setup's own"));
    }
}

} // namespace

bool isProbingStrategy(std::string_view id) noexcept {
    return std::find(kIds.begin(), kIds.end(), id) != kIds.end();
}

std::vector<StrategyInfo> probingStrategies() {
    std::vector<StrategyInfo> all;
    {
        StrategyInfo s{"probe_surface", "Probe surface", {}, GeometryUse::Faces, {}};
        s.params = {choice("direction", "Probe towards", directions(), "z_minus",
                           "The way the stylus moves to touch the face (picking a face sets it "
                           "from the face's direction)"),
                    length("x", "Touch point X", 0.0, -kFar, kFar, kPick),
                    length("y", "Touch point Y", 0.0, -kFar, kFar, kPick),
                    length("z", "Touch point Z", 0.0, -kFar, kFar,
                           "Where the stylus touches the face (WCS); on a side face, the height "
                           "of the ball's centre. Picking a face fills it in")};
        addCommon(s.params, false);
        s.description = "Touches one face along X, Y or Z with a spindle probe and sets that "
                        "axis of a work offset from where it is.";
        all.push_back(std::move(s));
    }
    {
        StrategyInfo s{"probe_width", "Probe web or pocket", {}, GeometryUse::Faces, {}};
        s.params = {
            choice("feature", "Feature",
                   {{"web", "Web: two outside faces"}, {"pocket", "Pocket: two inside faces"}},
                   "web", "A web is touched from outside, a pocket or slot from inside"),
            choice("axis", "Measure", axes(), "x", "The axis the width is across"),
            length("x", "Centre X", 0.0, -kFar, kFar, kPick),
            length("y", "Centre Y", 0.0, -kFar, kFar, kPick),
            length("z", "Measuring height", 0.0, -kFar, kFar, kHeight),
            length("width", "Width", 10.0, 0.0, kFar, kPick)};
        addCommon(s.params, false);
        s.description = "Touches both sides of a web or a pocket with a spindle probe and sets "
                        "its centre, across X or Y, in a work offset.";
        all.push_back(std::move(s));
    }
    {
        StrategyInfo s{"probe_circle", "Probe bore or boss", {}, GeometryUse::Faces, {}};
        s.params = {choice("feature", "Feature",
                           {{"bore", "Bore: a hole, from inside"},
                            {"boss", "Boss: a round stud, from outside"}},
                           "bore", "Picking a cylindrical face sets it"),
                    length("x", "Centre X", 0.0, -kFar, kFar, kPick),
                    length("y", "Centre Y", 0.0, -kFar, kFar, kPick),
                    length("z", "Measuring height", 0.0, -kFar, kFar, kHeight),
                    length("diameter", "Diameter", 10.0, 0.0, kFar, kPick)};
        addCommon(s.params, false);
        s.description = "Touches a bore or a boss at four points with a spindle probe and sets "
                        "its centre in X and Y in a work offset.";
        all.push_back(std::move(s));
    }
    {
        StrategyInfo s{"probe_corner", "Probe corner", {}, GeometryUse::Faces, {}};
        s.params = {choice("feature", "Feature",
                           {{"outside_corner", "Outside corner: of the part"},
                            {"inside_corner", "Inside corner: of a pocket"}},
                           "outside_corner", "Picking two faces at a right angle sets it"),
                    choice("corner", "Corner",
                           {{"x_minus_y_minus", "-X -Y corner"},
                            {"x_plus_y_minus", "+X -Y corner"},
                            {"x_minus_y_plus", "-X +Y corner"},
                            {"x_plus_y_plus", "+X +Y corner"}},
                           "x_minus_y_minus",
                           "Which corner of the part (outside) or of the pocket (inside)"),
                    length("x", "Corner X", 0.0, -kFar, kFar, kPick),
                    length("y", "Corner Y", 0.0, -kFar, kFar, kPick),
                    length("z", "Measuring height", 0.0, -kFar, kFar, kHeight),
                    length("inset", "Distance from the corner", 5.0, 0.1, 1000.0,
                           "Where each face is touched: this far along it from the corner")};
        addCommon(s.params, false);
        s.description = "Touches the two faces of an outside or inside corner with a spindle "
                        "probe and sets the corner in X and Y in a work offset.";
        all.push_back(std::move(s));
    }
    {
        StrategyInfo s{"probe_check", "Feature check", {}, GeometryUse::Faces, {}};
        s.params = {
            choice("feature", "Feature",
                   {{"surface", "Surface: its position"},
                    {"web", "Web: width and centre"},
                    {"pocket", "Pocket: width and centre"},
                    {"bore", "Bore: diameter and centre"},
                    {"boss", "Boss: diameter and centre"}},
                   "surface", "What is measured; picking faces on the model sets it"),
            when(choice("direction", "Probe towards", directions(), "z_minus",
                        "The way the stylus moves to touch the face"),
                 "feature", {"surface"}),
            when(choice("axis", "Measure", axes(), "x", "The axis the width is across"), "feature",
                 {"web", "pocket"}),
            length("x", "X", 0.0, -kFar, kFar,
                   "The touch point on a surface, else the centre (WCS). Picking faces fills "
                   "it in"),
            length("y", "Y", 0.0, -kFar, kFar,
                   "The touch point on a surface, else the centre (WCS). Picking faces fills "
                   "it in"),
            length("z", "Z", 0.0, -kFar, kFar,
                   "The touch point on a surface, else the measuring height of the ball's "
                   "centre (WCS). Picking faces fills it in"),
            when(length("width", "Width", 10.0, 0.0, kFar, kPick), "feature", {"web", "pocket"}),
            when(length("diameter", "Diameter", 10.0, 0.0, kFar, kPick), "feature",
                 {"bore", "boss"})};
        addCommon(s.params, true);
        s.description = "Measures a surface, web, pocket, bore or boss with a spindle probe and "
                        "compares it with the model within a tolerance, setting no work offset.";
        all.push_back(std::move(s));
    }
    return all;
}

namespace {

Vec3 directionOf(const std::string& value) {
    if (value == "x_plus") {
        return Vec3::UnitX();
    }
    if (value == "x_minus") {
        return -Vec3::UnitX();
    }
    if (value == "y_plus") {
        return Vec3::UnitY();
    }
    if (value == "y_minus") {
        return -Vec3::UnitY();
    }
    return -Vec3::UnitZ();
}

ProbeFeature featureOf(const std::string& value) {
    return toolpath::parseProbeFeature(value).value_or(ProbeFeature::Surface);
}

} // namespace

ProbeParams probeParams(std::string_view strategy, const Json& values) {
    const StrategyInfo* info = findStrategy(strategy);
    if (!info || !isProbingStrategy(strategy)) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("'{}' is not a probing operation", strategy));
    }
    const ParamReader r(*info, values);
    ProbeParams p;
    p.nominal = Vec3(r.number("x"), r.number("y"), r.number("z"));
    p.approach = r.number("approach");
    p.overtravel = r.number("overtravel");
    p.fastFeed = r.number("fast_feed");
    p.measureFeed = r.number("measure_feed");
    const bool check = strategy == "probe_check";
    if (check) {
        p.tolerance = r.number("tolerance");
    } else {
        p.workOffset = r.integer("work_offset");
    }
    const std::string feature = info->find("feature") ? r.choice("feature") : "surface";
    p.feature = featureOf(feature);
    switch (p.feature) {
    case ProbeFeature::Surface:
        p.direction = directionOf(r.choice("direction"));
        break;
    case ProbeFeature::Web:
    case ProbeFeature::Pocket:
        p.direction = r.choice("axis") == "y" ? Vec3::UnitY() : Vec3::UnitX();
        p.size = r.number("width");
        break;
    case ProbeFeature::Bore:
    case ProbeFeature::Boss:
        p.size = r.number("diameter");
        break;
    case ProbeFeature::OutsideCorner:
    case ProbeFeature::InsideCorner: {
        const std::string corner = r.choice("corner");
        const Vec3 at(corner.starts_with("x_plus") ? 1.0 : -1.0,
                      corner.ends_with("y_plus") ? 1.0 : -1.0, 0.0);
        // The stylus moves towards the material: inwards from outside a part's corner, outwards
        // from inside a pocket's.
        p.direction = p.feature == ProbeFeature::OutsideCorner ? Vec3(-at) : at;
        p.inset = r.number("inset");
        break;
    }
    }
    return p;
}

std::string probeAxes(const ProbeParams& p) {
    if (p.workOffset == 0) {
        return {};
    }
    switch (p.feature) {
    case ProbeFeature::Surface:
        return std::abs(p.direction.z()) > 0.5 ? "Z" : std::abs(p.direction.y()) > 0.5 ? "Y" : "X";
    case ProbeFeature::Web:
    case ProbeFeature::Pocket:
        return std::abs(p.direction.y()) > 0.5 ? "Y" : "X";
    default:
        return "XY";
    }
}

namespace {

/// One touch: the stylus ball's centre where the measuring move starts and where it stops on the
/// nominal surface, and the way it moves.
struct Touch {
    Vec3 start;
    Vec3 stop;
    Vec3 dir;
};

/// A touch of the surface point `s` moving along `dir`, with the ball's effective radius `re`.
Touch touch(const Vec3& s, const Vec3& dir, double re, double approach) {
    return {s - (re + approach) * dir, s - re * dir, dir};
}

} // namespace

toolpath::Toolpath probe(const ProbeParams& p, const Context& ctx) {
    const tooling::Tool& tool = ctx.tool;
    if (tool.type != tooling::ToolType::Probe) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("probing needs a probe: T{} is a {}; choose a probe as the "
                                "operation's tool",
                                tool.number, tooling::displayName(tool.type)));
    }
    // Feeds are the operation's, exactly (ADR-0004): a move at 0 would never arrive.
    if (p.fastFeed <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    "the operation's positioning feed is 0: set it on the operation");
    }
    if (p.measureFeed <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    "the operation's measuring feed is 0: set it on the operation");
    }
    const double r = detail::requireMillingTool(tool);
    const double re = tooling::effectiveStylusRadius(tool.mill);
    const Vec3 n = p.nominal;
    const double reach = re + p.approach;
    auto tooSmall = [&](const char* what, double size) {
        return Error(ErrorCode::InvalidArgument,
                     fmt::format("the {} ({:g} mm) is too small for the stylus (Ø{:g}) and the "
                                 "approach distance ({:g} mm)",
                                 what, size, tool.mill.diameter, p.approach));
    };

    std::vector<Touch> touches;
    bool inside = false; // the stylus stays down between touches, around `hub`
    Vec3 hub = n;
    switch (p.feature) {
    case ProbeFeature::Surface:
        touches.push_back(touch(n, p.direction.normalized(), re, p.approach));
        break;
    case ProbeFeature::Web:
    case ProbeFeature::Pocket: {
        const Vec3 a = p.direction.normalized();
        const double h = p.size / 2.0;
        if (p.feature == ProbeFeature::Web) {
            touches.push_back(touch(n - h * a, a, re, p.approach));
            touches.push_back(touch(n + h * a, -a, re, p.approach));
        } else {
            if (h < reach) {
                throw tooSmall("pocket's width", p.size);
            }
            inside = true;
            touches.push_back(touch(n - h * a, -a, re, p.approach));
            touches.push_back(touch(n + h * a, a, re, p.approach));
        }
        break;
    }
    case ProbeFeature::Bore:
    case ProbeFeature::Boss: {
        const double h = p.size / 2.0;
        const bool bore = p.feature == ProbeFeature::Bore;
        if (bore && h < reach) {
            throw tooSmall("bore's diameter", p.size);
        }
        inside = bore;
        for (int k = 0; k < 4; ++k) {
            const double angle = k * kPi / 2.0;
            const Vec3 out(std::cos(angle), std::sin(angle), 0.0);
            touches.push_back(touch(n + h * out, bore ? out : Vec3(-out), re, p.approach));
        }
        break;
    }
    case ProbeFeature::OutsideCorner:
    case ProbeFeature::InsideCorner: {
        const double dx = p.direction.x() < 0.0 ? -1.0 : 1.0;
        const double dy = p.direction.y() < 0.0 ? -1.0 : 1.0;
        const bool outer = p.feature == ProbeFeature::OutsideCorner;
        // Along each face from the corner: into the material outside, into the pocket inside.
        const double along = outer ? 1.0 : -1.0;
        if (p.inset <= re || (!outer && p.inset < reach)) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("the distance from the corner ({:g} mm) must be more than the "
                                    "stylus radius{}",
                                    p.inset, outer ? "" : " and the approach distance"));
        }
        touches.push_back(
            touch(n + Vec3(0.0, along * dy * p.inset, 0.0), Vec3(dx, 0.0, 0.0), re, p.approach));
        touches.push_back(
            touch(n + Vec3(along * dx * p.inset, 0.0, 0.0), Vec3(0.0, dy, 0.0), re, p.approach));
        if (!outer) {
            inside = true;
            hub = n - p.inset * Vec3(dx, dy, 0.0);
        }
        break;
    }
    }

    // The toolpath runs the tool tip: the ball's centre is a radius above it.
    const Vec3 up(0.0, 0.0, r);
    auto tip = [&](const Vec3& centre) {
        return Vec3(centre - up);
    };
    const Vec3 first = inside ? hub : touches.front().start;
    toolpath::Toolpath tp = beginToolpath(ctx, "probe", first.head<2>());
    tp.spindleSpeed = 0.0; // the probe does not turn
    tp.coolant = tooling::Coolant::Off;
    toolpath::ProbeCycle cycle;
    cycle.feature = p.feature;
    cycle.nominal = p.nominal;
    cycle.direction = p.direction;
    cycle.size = p.size;
    cycle.workOffset = p.workOffset;
    cycle.axes = probeAxes(p);
    cycle.tolerance = p.tolerance;
    cycle.approach = p.approach;
    cycle.overtravel = p.overtravel;
    cycle.fastFeed = p.fastFeed;
    cycle.measureFeed = p.measureFeed;
    cycle.stylusDiameter = tool.mill.diameter;
    cycle.triggerOffset = tool.mill.triggerOffset;
    tp.probes.push_back(cycle);

    const double retract = ctx.heights.retract;
    double highest = -std::numeric_limits<double>::infinity();
    for (const Touch& t : touches) {
        highest = std::max(highest, tip(t.start).z());
    }
    if (retract < highest - 1e-9) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("the retract height ({:g}) is below where the probe starts "
                                "touching ({:g}): raise the operation's retract height",
                                retract, highest));
    }

    toolpath::PathBuilder b(tp);
    const double fast = p.fastFeed;
    auto above = [&](const Vec3& centre) {
        return Vec3(centre.x(), centre.y(), retract);
    };
    auto measure = [&](const Touch& t) {
        b.probe(tip(t.start), fast, 0, false);
        b.probe(tip(t.stop), p.measureFeed, 0, true);
        b.probe(tip(t.start), fast, 0, false); // back off
    };
    b.rapid(above(first));
    if (inside) {
        b.probe(tip(hub), fast, 0, false);
        for (const Touch& t : touches) {
            measure(t);
            b.probe(tip(hub), fast, 0, false);
        }
        b.probe(above(hub), fast, 0, false);
    } else {
        // Over the feature between touches by protected moves too: the cycle is one run of probe
        // moves, which the post receives as one instruction.
        for (const Touch& t : touches) {
            b.probe(above(t.start), fast, 0, false);
            measure(t);
            b.probe(above(t.start), fast, 0, false);
        }
    }
    b.retractTo(ctx.heights.clearance);
    ctx.cancel.throwIfCancelled();
    return tp;
}

} // namespace ocam::strategies
