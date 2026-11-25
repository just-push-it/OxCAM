// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/strategies/Common.hpp"

#include "ocam/strategies/BackBore.hpp"
#include "ocam/strategies/EdgeBreak.hpp"
#include "ocam/strategies/Probing.hpp"
#include "ocam/strategies/Strategies.hpp"
#include "ocam/strategies/Turning.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/tooling/Profile.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <functional>

namespace ocam::strategies {

Feeds feedsFor(const tooling::Tool& tool, const tooling::CuttingPreset* preset) {
    Feeds f;
    if (!preset) {
        return f;
    }
    const double d = tooling::isTurningTool(tool.type) ? 0.0 : tool.mill.diameter;
    const double n = preset->spindleSpeed(d);
    f.spindle = n;
    const double vf = preset->feedRate(d, tool.mill.flutes);
    f.cut = vf;
    f.lead = preset->leadFeed;
    f.ramp = preset->rampFeed;
    f.plunge = preset->plungeFeed;
    f.pull = preset->pullFeed;
    if (tool.type == tooling::ToolType::Tap && tool.mill.threadPitch > 0.0 && n > 0.0) {
        f.plunge = tool.mill.threadPitch * n; // synchronized tapping
    }
    f.coolant = preset->coolant;
    return f;
}

toolpath::Toolpath beginToolpath(const Context& ctx, std::string_view strategy,
                                 const Vec2& startXY) {
    toolpath::Toolpath tp;
    tp.strategy = std::string(strategy);
    tp.toolId = ctx.tool.id;
    tp.toolNumber = ctx.tool.number;
    tp.toolDiameter = tooling::isTurningTool(ctx.tool.type) ? 0.0 : ctx.tool.mill.diameter;
    tp.spindleSpeed = ctx.feeds.spindle;
    tp.coolant = ctx.feeds.coolant;
    tp.start = Vec3(startXY.x(), startXY.y(), ctx.heights.clearance);
    return tp;
}

std::vector<double> depthLevels(double top, double bottom, double stepDown) {
    std::vector<double> levels;
    if (bottom >= top - 1e-9) {
        return levels;
    }
    const int count =
        stepDown > 1e-9 ? std::max(1, static_cast<int>(std::ceil((top - bottom) / stepDown - 1e-9)))
                        : 1;
    for (int i = 1; i <= count; ++i) {
        levels.push_back(i == count ? bottom : top - (top - bottom) * i / count);
    }
    return levels;
}

namespace {

ParamDef length(std::string key, std::string label, double def, double min, double max,
                std::string help = {}) {
    return {std::move(key), std::move(label), ParamKind::Length, def, min, max, {}, {},
            std::move(help)};
}
/// `d` with its value, while not set, taken from the operation's tool.
ParamDef fromTool(ParamDef d, std::function<double(const tooling::Tool&)> value) {
    d.toolDefault = std::move(value);
    return d;
}
double toolDiameter(const tooling::Tool& t) {
    return t.mill.diameter;
}
ParamDef angle(std::string key, std::string label, double def, double min, double max,
               std::string help = {}) {
    return {std::move(key), std::move(label), ParamKind::Angle, def, min, max, {}, {},
            std::move(help)};
}
ParamDef choice(std::string key, std::string label,
                std::vector<std::pair<std::string, std::string>> options, std::string def,
                std::string help = {}) {
    ParamDef p{std::move(key),     std::move(label), ParamKind::Choice, 0, 0, 0,
               std::move(options), std::move(def),   std::move(help)};
    return p;
}

const std::vector<std::pair<std::string, std::string>> kDirections{
    {"climb", "Climb"}, {"conventional", "Conventional"}};

// Help texts several strategies share.
const char* const kLeaveWalls = "Material left on the walls for a later finishing operation";
const char* const kLeaveFloors = "Material left on the floors for a later finishing operation";
const char* const kLeaveSurface =
    "Material left on the surface; negative cuts below it (an electrode's spark gap, say)";
const char* const kEntry =
    "How the tool gets down into the material: a helix, a ramp along the path, or a straight "
    "plunge (needs a centre-cutting tool or a pre-drilled hole)";
const char* const kHelixRadius =
    "Radius of the helical entry; while not set, 80% of the tool radius";
const char* const kDirection =
    "Climb: the teeth enter the material at full chip thickness, the usual choice on CNC "
    "machines (better finish and tool life). Conventional: the opposite, for loose machines or "
    "hard skins";
const char* const kBreakthrough = "Extra depth past the bottom of through holes";
const char* const kHoleDepth =
    "Measured from the hole's top (on a lathe, from the part's front). 0 = as deep as the hole in "
    "the part goes";
const char* const kRestThin =
    "Rest material thinner than this is left alone, so the tool does not chase tiny leftovers";
const char* const kPattern =
    "Zig-zag cuts in both directions. One way cuts every pass in the same direction (climb or "
    "conventional throughout) and returns above the part";
const char* const kPassAngle = "Direction of the passes, measured from the X axis";
const char* const kLeadRadius =
    "Radius of the arcs that lead on to and off the wall; 0 = straight on. While not set, it "
    "follows the tool's size";
const char* const kResolution =
    "Spacing of the height map of the part the passes are computed on: smaller follows small "
    "details and takes longer; 0 = from the tool size";
const char* const kSampleStep =
    "Spacing of the points along each pass: smaller follows small details and takes longer; "
    "0 = from the tool size";
const char* const kRampAngle =
    "Angle of the ramp or helix entry; the cutting preset's ramp angle when not set here";

/// `d`, shown only when parameter `key` has one of `values` ("on"/"off" for a flag).
ParamDef when(ParamDef d, std::string key, std::vector<std::string> values) {
    d.onlyWhen.key = std::move(key);
    d.onlyWhen.values = std::move(values);
    return d;
}
/// `d`, shown only when the number `key` is above `above`.
ParamDef whenAbove(ParamDef d, std::string key, double above) {
    d.onlyWhen.key = std::move(key);
    d.onlyWhen.above = above;
    return d;
}
ParamDef flag(std::string key, std::string label, bool def, std::string help = {}) {
    return {std::move(key), std::move(label), ParamKind::Boolean, def ? 1.0 : 0.0, 0.0, 1.0, {}, {},
            std::move(help)};
}

/// Arc output, on the milling operations whose paths it changes (toolpath::fitArcs on, arcs as
/// lines off; applied to the finished toolpath by the document layer, DL-0032).
std::vector<ParamDef> smoothingParams() {
    return {
        flag("smoothing", "Smoothing (arcs)", true,
             "Output arcs (G2/G3, in the XY, XZ or YZ plane) and long lines in place of runs of "
             "short moves, within the smoothing tolerance: a smoother surface on curved areas. "
             "Off: arcs as short lines, for controls without arc moves"),
        when(length("smoothing_tolerance", "Smoothing tolerance", 0.005, 0.0005, 1.0,
                    "How far the arcs and lines may stray from the moves they replace"),
             "smoothing", {"on"})};
}

/// Rest machining from the setup's simulated in-process stock.
std::vector<ParamDef> restParams(std::string what) {
    return {
        flag("rest_machining", "Rest machining", false, std::move(what)),
        when(length("rest_resolution", "Rest detection resolution", 0.0, 0.0, 5.0,
                    "Ray spacing of the simulated in-process stock that finds the rest "
                    "material: smaller finds thinner rest and takes longer; 0 = from the "
                    "stock size"),
             "rest_machining", {"on"}),
        when(length("rest_min_thickness", "Ignore rest thinner than", 0.05, 0.0, 10.0, kRestThin),
             "rest_machining", {"on"})};
}

std::vector<StrategyInfo> buildStrategies() {
    std::vector<StrategyInfo> all;
    all.push_back({"facing",
                   "Facing",
                   {length("stock_to_leave", "Stock to leave (Z)", 0.0, 0.0, 100.0,
                           "Material left on the faced surface for a finishing pass"),
                    angle("angle", "Pass angle", 0.0, -360.0, 360.0, kPassAngle),
                    choice("pattern", "Pattern", {{"zigzag", "Zig-zag"}, {"one_way", "One way"}},
                           "zigzag", kPattern),
                    length("overrun", "Overrun past edge", 2.0, 0.0, 100.0,
                           "How far the tool edge travels past the stock outline, so the edges are "
                           "faced cleanly")}});
    all.push_back(
        {"plunge_rough",
         "Plunge roughing",
         {fromTool(length("plunge_spacing", "Plunge spacing", 0.0, 0.01, 1000.0,
                          "Along a row; the rows are a radial depth (ae) apart. While not set, "
                          "half the tool diameter"),
                   [](const tooling::Tool& t) { return kPlungePitchShare * toolDiameter(t); }),
          {"stagger",
           "Stagger rows",
           ParamKind::Boolean,
           1.0,
           0.0,
           1.0,
           {},
           {},
           "Shift every other row by half a plunge spacing (fewer, smaller cusps)"},
          {"wall",
           "Plunge along the wall",
           ParamKind::Boolean,
           1.0,
           0.0,
           1.0,
           {},
           {},
           "Finish with plunges along the boundary"},
          length("stock_to_leave_xy", "Stock to leave (XY)", 0.0, 0.0, 100.0, kLeaveWalls),
          length("stock_to_leave_z", "Stock to leave (Z)", 0.0, 0.0, 100.0, kLeaveFloors)}});
    all.push_back(
        {"pocket2d",
         "2D pocket",
         {length("stock_to_leave_xy", "Stock to leave (XY)", 0.0, 0.0, 100.0, kLeaveWalls),
          length("stock_to_leave_z", "Stock to leave (Z)", 0.0, 0.0, 100.0, kLeaveFloors),
          choice("entry", "Entry",
                 {{"helix", "Helix (circular)"}, {"ramp", "Ramp along path"}, {"plunge", "Plunge"}},
                 "helix", kEntry),
          when(angle("ramp_angle", "Ramp angle", 2.0, 0.1, 45.0, kRampAngle), "entry",
               {"ramp", "helix"}),
          when(fromTool(length("helix_radius", "Helix radius", 0.0, 0.01, 1000.0, kHelixRadius),
                        [](const tooling::Tool& t) {
                            return kHelixRadiusShare * 0.5 * toolDiameter(t);
                        }),
               "entry", {"helix"}),
          choice("direction", "Direction", kDirections, "climb", kDirection),
          {"rest_machining",
           "Rest machining",
           ParamKind::Boolean,
           0.0,
           0.0,
           1.0,
           {},
           {},
           "Cut only material left by earlier operations of the setup (simulated)"},
          when(length("rest_resolution", "Rest detection resolution", 0.0, 0.0, 5.0,
                      "Ray spacing of the simulated in-process stock that finds the rest material: "
                      "smaller finds thinner rest and takes longer; 0 = from the stock size"),
               "rest_machining", {"on"}),
          when(length("rest_min_thickness", "Ignore rest thinner than", 0.02, 0.0, 10.0, kRestThin),
               "rest_machining", {"on"})}});
    all.push_back(
        {"contour2d",
         "2D contour",
         {length("stock_to_leave_xy", "Stock to leave (XY)", 0.0, -10.0, 100.0,
                 "Material left on the wall; negative cuts past the outline"),
          length("stock_to_leave_z", "Stock to leave (Z)", 0.0, 0.0, 100.0, kLeaveFloors),
          {"passes",
           "Radial passes",
           ParamKind::Integer,
           1,
           1,
           50,
           {},
           {},
           "Side-by-side passes that step in to the final wall (for heavy walls)"},
          whenAbove(length("pass_spacing", "Pass spacing", 0.5, 0.0, 100.0,
                           "Distance between the radial passes"),
                    "passes", 1),
          {"spring_pass",
           "Spring pass",
           ParamKind::Boolean,
           0.0,
           0.0,
           1.0,
           {},
           {},
           "Repeat the final pass at full depth to clean up what tool deflection left"},
          fromTool(length("lead_radius", "Lead radius", 0.0, 0.0, 1000.0, kLeadRadius),
                   [](const tooling::Tool& t) { return kContourLeadShare * toolDiameter(t); }),
          length("lead_length", "Lead line length", 0.0, 0.0, 100.0,
                 "Straight approach before the lead arc"),
          choice("direction", "Direction", kDirections, "climb", kDirection),
          choice("compensation", "Compensation",
                 {{"computer", "In computer"}, {"control", "In control (G41/G42)"}}, "computer",
                 "In computer: the program is the tool centre path. In control: the program "
                 "follows the wall with G41/G42, so wear can be adjusted on the machine"),
          flag("tabs", "Holding tabs", false,
               "Bridges left uncut along each closed outline, so the part stays attached to the "
               "stock. The tool goes over them on the passes below their top, at the cut feed"),
          when(choice("tab_placement", "Tab placement",
                      {{"count", "By count"}, {"spacing", "By spacing"}}, "count",
                      "By count: the same number of tabs on every outline. By spacing: the "
                      "outline's length shared out evenly, about one tab per spacing"),
               "tabs", {"on"}),
          when({"tab_count",
                "Tabs per outline",
                ParamKind::Integer,
                4,
                1,
                100,
                {},
                {},
                "Evenly spaced along each closed outline, moved off corners and short segments "
                "where they can be"},
               "tab_placement", {"count"}),
          when(length("tab_spacing", "Tab spacing", 100.0, 1.0, 1e5,
                      "Distance between tabs along each outline; at least one tab per outline"),
               "tab_placement", {"spacing"}),
          when(length("tab_width", "Tab width", 5.0, 0.0, 1000.0,
                      "Length of each tab along the outline at its narrowest, where the tool's "
                      "edge passes: the tool stays up over this width plus its diameter"),
               "tabs", {"on"}),
          when(length("tab_height", "Tab height", 2.0, 0.01, 1000.0,
                      "Height of the tabs above the contour's bottom. Only the passes below "
                      "their top go over them"),
               "tabs", {"on"}),
          when(choice("tab_shape", "Tab shape",
                      {{"square", "Square: step up and over"},
                       {"triangle", "Triangular: ramp up and over"}},
                      "square",
                      "Square: the tool steps straight up, crosses at the tab's top and steps "
                      "down. Triangular: it ramps up and down over half the width each side, "
                      "leaving a triangle that breaks off more easily"),
               "tabs", {"on"})}});
    all.push_back(
        {"drill",
         "Drilling",
         {choice("cycle", "Cycle",
                 {{"auto", "Automatic"},
                  {"drill", "Drill (G81)"},
                  {"dwell", "Dwell (G82)"},
                  {"peck", "Peck (G83)"},
                  {"chip_break", "Chip break (G73)"},
                  {"tap", "Tap (G84)"},
                  {"bore", "Bore (G85)"}},
                 "auto",
                 "The canned cycle. Automatic: a tap taps and other tools drill, "
                 "pecking holes deeper than 3 × the tool diameter"),
          when(fromTool(length("peck_depth", "Peck depth", 0.0, 0.01, 1000.0,
                               "Depth of each peck; while not set, the tool diameter"),
                        [](const tooling::Tool& t) { return kPeckShare * toolDiameter(t); }),
               "cycle", {"auto", "peck", "chip_break"}),
          {"dwell",
           "Dwell",
           ParamKind::Time,
           0.0,
           0.0,
           60.0,
           {},
           {},
           "Pause at the bottom of the hole (clean flat bottoms, spot faces)"},
          length("hole_depth", "Hole depth", 0.0, 0.0, 1000.0, kHoleDepth),
          choice("depth_mode", "Depth", {{"full_diameter", "Full diameter"}, {"tip", "Tip"}},
                 "full_diameter",
                 "Full diameter: the full drill diameter reaches the hole depth (the "
                 "point goes deeper). Tip: the point stops at the hole depth"),
          length("breakthrough", "Breakthrough", 1.0, 0.0, 50.0, kBreakthrough)}});
    all.push_back(
        {"chamfer2d",
         "Chamfer",
         {length("width", "Chamfer width", 0.5, 0.01, 50.0, "Horizontal; the angle is the tool's"),
          length("tip_offset", "Tip below chamfer", 0.5, 0.0, 20.0,
                 "How far the tool tip goes below the chamfer's lower edge, so the flank cuts "
                 "it rather than the fragile tip"),
          fromTool(length("lead_radius", "Lead radius", 0.0, 0.0, 1000.0, kLeadRadius),
                   [](const tooling::Tool& t) { return kChamferLeadShare * toolDiameter(t); }),
          choice("direction", "Direction", kDirections, "climb", kDirection)}});
    all.push_back(
        {"trace2d",
         "2D trace",
         {length("depth", "Depth", 0.2, 0.0, 1000.0,
                 "How deep the tool tip goes below the selected edges (their lowest point), or "
                 "below the surface when following it; 0 = on them. Deeper than the step down "
                 "(the cutting preset's axial depth ap) goes in levels"),
          choice(
              "side", "Tool position",
              {{"on", "On the line"}, {"left", "Left of the line"}, {"right", "Right of the line"}},
              "on",
              "Where the tool centre runs, looking the way the tool travels. Left: the tool's "
              "side runs along the line on its right, climb milling against it with the "
              "spindle turning clockwise; Right: conventional. A closed chain runs "
              "counter-clockwise unless reversed, so Left is its inside"),
          when(length("offset", "Distance from line", 0.0, -1000.0, 1000.0,
                      "Gap between the tool's side and the line; negative takes the tool over "
                      "it"),
               "side", {"left", "right"}),
          when({"passes",
                "Side-by-side passes",
                ParamKind::Integer,
                1,
                1,
                50,
                {},
                {},
                "Passes farther from the line first, stepping in to it: a wider slot"},
               "side", {"left", "right"}),
          whenAbove(length("pass_spacing", "Pass spacing", 0.5, 0.001, 100.0,
                           "Distance between the side-by-side passes"),
                    "passes", 1),
          flag("reverse", "Reverse", false,
               "Run each chain the other way. An open chain starts from its end nearer the start "
               "point, or else from its end with the lower X; a closed one starts at the start "
               "point and runs counter-clockwise"),
          choice("entry", "Entry", {{"plunge", "Plunge"}, {"ramp", "Ramp along the path"}},
                 "plunge",
                 "How the tool gets down to each level: straight down at the plunge feed (needs "
                 "a centre-cutting or pointed tool), or along the path at the ramp feed, back "
                 "and forth on an open chain"),
          when(angle("ramp_angle", "Ramp angle", 2.0, 0.1, 45.0, kRampAngle), "entry", {"ramp"}),
          flag("follow_surface", "Follow the surface", false,
               "Keep the depth below the part's surface under the tool, found on the model with "
               "the tool's shape: engraving on curved and sloping faces. Off: at one height, the "
               "depth below the edges"),
          when(length("sample_step", "Sample spacing", 0.5, 0.01, 10.0,
                      "Spacing of the surface heights along the path: smaller follows small "
                      "details and takes longer"),
               "follow_surface", {"on"})}});
    all.push_back({"tap",
                   "Tapping",
                   {length("thread_depth", "Thread depth", 0.0, 0.0, 500.0, "0 = hole depth"),
                    length("breakthrough", "Breakthrough", 0.0, 0.0, 50.0, kBreakthrough)}});
    all.push_back(
        {"threadmill",
         "Thread milling",
         {choice(
              "side", "Thread",
              {{"internal", "Internal: in holes"}, {"external", "External: on bosses and studs"}},
              "internal",
              "Internal threads are cut in the selected holes; external ones around the "
              "selected bosses, studs or turned diameters"),
          length("pitch", "Pitch", 0.0, 0.0, 20.0, "0 = the thread mill's pitch"),
          length("major_diameter", "Major diameter", 0.0, 0.0, 500.0,
                 "0 = internal: the hole's diameter + 1.0825 × pitch (a tap drill hole); "
                 "external: the boss's diameter"),
          length("radial_depth", "Thread depth (radial)", 0.0, 0.0, 20.0,
                 "Depth of the thread form; 0 = 0.6134 × pitch external, 0.5413 × pitch "
                 "internal. The tool's centre runs at the major radius minus the tool radius "
                 "inside, and at the major radius minus this plus the tool radius outside"),
          length(
              "thread_depth", "Thread length", 0.0, 0.0, 500.0,
              "Down from the hole's or the boss's top; 0 = the hole's depth or the boss's length"),
          choice("hand", "Hand", {{"right", "Right hand"}, {"left", "Left hand"}}, "right",
                 "Right- or left-hand thread"),
          choice("direction", "Direction", kDirections, "climb",
                 "With the hand, sets which way the tool goes around and whether it goes up or "
                 "down. Climb: counter-clockwise in a hole, clockwise around a boss. A right-hand "
                 "thread goes up counter-clockwise and down clockwise; a left-hand one the other "
                 "way"),
          choice("form", "Tool",
                 {{"single", "Single point: one turn per pitch"},
                  {"multi", "Multi-form: one turn for the whole length"}},
                 "single",
                 "A single point thread mill has one tooth form and goes around once for each "
                 "pitch of the thread's length. A multi-form one has the whole length's forms "
                 "on its flutes and goes around once"),
          {"passes",
           "Radial passes",
           ParamKind::Integer,
           1,
           1,
           20,
           {},
           {},
           "Passes that go into the thread depth in equal steps, the last one at full depth"},
          flag("spring_pass", "Spring pass", false,
               "Repeat the last pass at full depth, to take what the tool's deflection left"),
          when(fromTool(length("lead_radius", "Lead radius", 0.0, 0.01, 1000.0,
                               "External: radius of the arc that comes on to the thread from "
                               "outside the major diameter and goes back off it; more than the "
                               "thread depth. While not set, half the tool's diameter"),
                        [](const tooling::Tool& t) { return kThreadLeadShare * toolDiameter(t); }),
               "side", {"external"})}});
    all.push_back(
        {"bore",
         "Boring",
         {length("hole_depth", "Hole depth", 0.0, 0.0, 1000.0, kHoleDepth),
          length("helix_pitch", "Helix pitch", 1.0, 0.01, 50.0, "End mills: depth per turn"),
          length("stock_to_leave", "Stock to leave (radial)", 0.0, 0.0, 10.0,
                 "Material left on the bore wall for a finishing pass"),
          length("breakthrough", "Breakthrough", 0.5, 0.0, 50.0, kBreakthrough),
          choice("direction", "Direction", kDirections, "climb", kDirection)}});
    all.push_back(detail::backBoreStrategy()); // DL-0083
    all.push_back(detail::edgeBreakStrategy());
    all.push_back(
        {"zlevel_rough",
         "3D roughing",
         {length("stock_to_leave_xy", "Stock to leave (XY)", 0.3, 0.0, 20.0, kLeaveWalls),
          length("stock_to_leave_z", "Stock to leave (Z)", 0.3, 0.0, 20.0, kLeaveFloors),
          length("resolution", "Surface resolution", 0.0, 0.0, 5.0, kResolution),
          choice("entry", "Entry",
                 {{"helix", "Helix (circular)"}, {"ramp", "Ramp along path"}, {"plunge", "Plunge"}},
                 "helix", kEntry),
          when(angle("ramp_angle", "Ramp angle", 2.0, 0.1, 45.0, kRampAngle), "entry",
               {"ramp", "helix"}),
          when(fromTool(length("helix_radius", "Helix radius", 0.0, 0.01, 1000.0, kHelixRadius),
                        [](const tooling::Tool& t) {
                            return kHelixRadiusShare * 0.5 * toolDiameter(t);
                        }),
               "entry", {"helix"}),
          choice("direction", "Direction", kDirections, "climb", kDirection),
          choice("open_areas", "Open areas",
                 {{"traced", "Traced passes"}, {"medial", "Circles too"}}, "traced",
                 "Closed areas (pockets, slots) are cleared on the largest circles that fit along "
                 "their medial axis. Areas with air to start from (the outside of a part): "
                 "traced passes from the air inward, or the same circles, started by a helix"),
          length("fine_step_down", "Fine step down", 1.25, 0.0, 1000.0,
                 "Extra levels this far apart between the step-down levels, cutting only next "
                 "to sloped and rounded walls (fillets, chamfers, drafts), so the stairs left "
                 "on them are at most about this tall; 0 = none"),
          choice("order", "Order", {{"levels", "Level by level"}, {"areas", "Area by area"}},
                 "levels",
                 "Area by area: each area (a pocket, say) is cut through all its levels before "
                 "the next"),
          {"flat_levels",
           "Extra levels at flat areas",
           ParamKind::Boolean,
           1.0,
           0.0,
           1.0,
           {},
           {},
           "Also rough each flat area at its height plus the Z allowance, so stock between the "
           "step-down levels does not stay on top of it"},
          {"rest_machining",
           "Rest machining",
           ParamKind::Boolean,
           0.0,
           0.0,
           1.0,
           {},
           {},
           "Rough only material left by earlier operations of the setup (simulated)"},
          when(length("rest_resolution", "Rest detection resolution", 0.0, 0.0, 5.0,
                      "Ray spacing of the simulated in-process stock that finds the rest material: "
                      "smaller finds thinner rest and takes longer; 0 = from the stock size"),
               "rest_machining", {"on"}),
          when(length("rest_min_thickness", "Ignore rest thinner than", 0.05, 0.0, 10.0, kRestThin),
               "rest_machining", {"on"})}});
    all.push_back(
        {"parallel3d",
         "3D parallel finishing",
         {length("scallop_height", "Scallop height", 0.0, 0.0, 5.0,
                 "Sets the stepover; 0 = the stepover is the cutting preset's radial depth (ae)"),
          angle("angle", "Angle", 0.0, -360.0, 360.0, kPassAngle),
          length("sample_step", "Sample step", 0.0, 0.0, 5.0, kSampleStep),
          length("stock_to_leave", "Stock to leave", 0.0, -1.0, 20.0, kLeaveSurface),
          choice("pattern", "Pattern", {{"zigzag", "Zig-zag"}, {"one_way", "One way"}}, "zigzag",
                 kPattern),
          {"rest_machining",
           "Rest machining",
           ParamKind::Boolean,
           0.0,
           0.0,
           1.0,
           {},
           {},
           "Finish only where earlier operations left stock (simulated)"},
          when(length("rest_resolution", "Rest detection resolution", 0.0, 0.0, 5.0,
                      "Ray spacing of the simulated in-process stock that finds the rest material: "
                      "smaller finds thinner rest and takes longer; 0 = from the stock size"),
               "rest_machining", {"on"}),
          when(length("rest_min_thickness", "Ignore rest thinner than", 0.05, 0.0, 10.0, kRestThin),
               "rest_machining", {"on"})}});
    all.push_back(
        {"flat3d",
         "3D flat area finishing",
         {length("stock_to_leave_xy", "Stock to leave (walls)", 0.0, 0.0, 20.0, kLeaveWalls),
          length("stock_to_leave_z", "Stock to leave (floors)", 0.0, 0.0, 20.0, kLeaveFloors),
          angle("max_slope", "Flat within", 0.5, 0.0, 10.0,
                "Faces tilted at most this much from horizontal are flat"),
          length("resolution", "Surface resolution", 0.0, 0.0, 5.0, kResolution),
          choice("entry", "Entry",
                 {{"ramp", "Ramp along path"}, {"helix", "Helix (circular)"}, {"plunge", "Plunge"}},
                 "ramp", kEntry),
          when(angle("ramp_angle", "Ramp angle", 2.0, 0.1, 45.0, kRampAngle), "entry",
               {"ramp", "helix"}),
          choice("direction", "Direction", kDirections, "climb", kDirection),
          {"bridge_openings",
           "Machine over openings",
           ParamKind::Boolean,
           0.0,
           0.0,
           1.0,
           {},
           {},
           "Cut across holes, slots and pockets in a flat as if it went on over them, when they "
           "are at most the max empty travel wide; wider ones (a large bore) are gone around"},
          when(
              length("bridge_max", "Max empty travel", 12.7, 0.0, 1000.0,
                     "With machining over openings: the widest opening crossed (the largest circle "
                     "that fits in it)"),
              "bridge_openings", {"on"})}});
    all.push_back({"waterline3d",
                   "3D waterline finishing",
                   {length("stock_to_leave", "Stock to leave", 0.0, -1.0, 20.0, kLeaveSurface),
                    length("resolution", "Surface resolution", 0.0, 0.0, 5.0, kResolution),
                    choice("direction", "Direction", kDirections, "climb", kDirection)}});
    all.push_back(
        {"contour3d",
         "3D contour finishing",
         {length("scallop_height", "Cusp height", 0.01, 0.0005, 5.0,
                 "Sets the step down between the passes from the wall's slope: steeper walls get "
                 "larger steps, curved blends (fillets) as many as the cusp needs"),
          length("max_step_down", "Max step down", 0.0, 0.0, 100.0,
                 "Caps the step on steep walls; 0 = as the cusp height allows"),
          angle("min_slope", "Shallowest wall", 5.0, 0.0, 89.0,
                "Walls shallower than this do not set the step (leave them to flat or parallel "
                "finishing)"),
          length("stock_to_leave", "Stock to leave", 0.0, -1.0, 20.0, kLeaveSurface),
          length("resolution", "Surface resolution", 0.0, 0.0, 5.0, kResolution),
          choice("direction", "Direction", kDirections, "climb", kDirection)}});
    all.push_back(
        {"pencil3d",
         "3D pencil finishing",
         {fromTool(length("reference_diameter", "Reference tool diameter", 0.0, 0.01, 1000.0,
                          "The larger tool that finished before: a pass runs wherever it touched "
                          "two faces at once, in corners and fillets tighter than it. While not "
                          "set, this operation's own tool (corners tighter than the tool)"),
                   toolDiameter),
          angle("corner_angle", "Corner angle", 20.0, 1.0, 170.0,
                "Least angle between the two faces the reference tool touches (between their "
                "directions at its two contact points) for a corner to get a pass"),
          {"offset_passes",
           "Offset passes each side",
           ParamKind::Integer,
           0,
           0,
           50,
           {},
           {},
           "Extra passes beside the pencil pass on each side, a stepover apart along the "
           "surface, out to where the reference tool touched; none up walls steeper than 80°"},
          whenAbove(length("scallop_height", "Scallop height", 0.0, 0.0, 5.0,
                           "Sets the stepover of the offset passes; 0 = the cutting preset's "
                           "radial depth (ae)"),
                    "offset_passes", 0.0),
          length("stock_to_leave", "Stock to leave", 0.0, -1.0, 20.0, kLeaveSurface),
          length("resolution", "Surface resolution", 0.0, 0.0, 5.0, kResolution)}});
    all.push_back(
        {"steep_shallow3d",
         "3D steep and shallow finishing",
         {angle("threshold_angle", "Steep from", 40.0, 5.0, 85.0,
                "Surfaces steeper than this (from horizontal) get passes around the part at "
                "constant heights, shallower ones parallel passes"),
          length("overlap", "Overlap", 0.5, 0.0, 50.0,
                 "Width of the band, about the line where the slope crosses the threshold, that "
                 "both kinds of passes cover, so no line is left between them"),
          length("steep_scallop", "Steep cusp height", 0.0, 0.0, 5.0,
                 "Spaces the constant-height passes by the wall's slope to leave this cusp; 0 = "
                 "the cutting preset's axial depth (ap) apart"),
          length("shallow_scallop", "Shallow scallop height", 0.0, 0.0, 5.0,
                 "Sets the stepover of the parallel passes; 0 = the cutting preset's radial depth "
                 "(ae)"),
          angle("angle", "Parallel pass angle", 0.0, -360.0, 360.0, kPassAngle),
          choice("pattern", "Pattern", {{"zigzag", "Zig-zag"}, {"one_way", "One way"}}, "zigzag",
                 kPattern),
          length("sample_step", "Sample step", 0.0, 0.0, 5.0, kSampleStep),
          choice("direction", "Direction", kDirections, "climb", kDirection),
          choice("order", "Order",
                 {{"steep_first", "Steep areas first"}, {"shallow_first", "Shallow areas first"}},
                 "steep_first", "Which areas are finished first"),
          length("stock_to_leave", "Stock to leave", 0.0, -1.0, 20.0, kLeaveSurface),
          length("resolution", "Surface resolution", 0.0, 0.0, 5.0, kResolution)}});

    // Turning (phase 8): common cutting data, then each operation's own parameters.
    auto turning = [&](const char* id, const char* label, std::vector<ParamDef> own) {
        // Surface speed and feed per revolution come from the insert's cutting preset.
        own.push_back({"max_rpm",
                       "Spindle limit (rpm)",
                       ParamKind::Integer,
                       3000,
                       1,
                       100000,
                       {},
                       {},
                       "Highest spindle speed: with constant surface speed, the rpm rises as the "
                       "tool nears the axis up to this limit"});
        own.push_back({"css",
                       "Constant surface speed",
                       ParamKind::Boolean,
                       1.0,
                       0.0,
                       1.0,
                       {},
                       {},
                       "G96 with the spindle limit; off: constant rpm (G97)"});
        own.push_back(length("clearance", "Clearance", 1.0, 0.1, 50.0,
                             "Air gap kept for approaches and retracts"));
        all.push_back({id, label, std::move(own)});
    };
    const std::vector<std::pair<std::string, std::string>> kSides{{"outside", "Outside (OD)"},
                                                                  {"inside", "Inside (ID)"}};
    const char* const kTurnSide = "The outside diameter (OD) or a bore (ID)";
    turning("turn_face", "Turning: face",
            {length("stock_to_leave", "Stock to leave", 0.0, 0.0, 10.0,
                    "Material left on the face for a finishing pass"),
             length("overshoot", "Past the axis", 0.5, 0.0, 10.0,
                    "How far the facing passes go past the centreline, so no pip is left")});
    const char* const kZRange =
        "The whole profile, or only what lies between two Z heights of the setup's WCS";
    const char* const kTurnDirection =
        "Which way the passes run: towards the chuck, or away from it (back turning), which needs "
        "a left-hand or neutral tool and room to start behind what it cuts";
    const std::vector<std::pair<std::string, std::string>> kTurnDirections{
        {"towards_chuck", "Towards the chuck"}, {"away_from_chuck", "Away from the chuck"}};
    const std::vector<std::pair<std::string, std::string>> kZRanges{{"whole", "Whole profile"},
                                                                    {"limited", "Between two Z"}};
    turning("turn_rough", "Turning: rough",
            {choice("side", "Side", kSides, "outside", kTurnSide),
             choice("pattern", "Passes",
                    {{"along", "Along Z (turning)"}, {"across", "Across Z (facing)"}}, "along",
                    "Level passes along the spindle axis, or radial ones stepping along it: for "
                    "parts wider than they are long"),
             choice("direction", "Direction", kTurnDirections, "towards_chuck", kTurnDirection),
             choice("z_range", "Z range", kZRanges, "whole", kZRange),
             when(length("z_front", "Z front", 0.0, -10000.0, 10000.0,
                         "Where the range starts, towards the front of the part"),
                  "z_range", {"limited"}),
             when(length("z_back", "Z back", 0.0, -10000.0, 10000.0,
                         "Where the range ends, towards the chuck"),
                  "z_range", {"limited"}),
             length("stock_to_leave", "Stock to leave", 0.3, 0.0, 10.0,
                    "Material left on the profile for the finishing operation"),
             length("retract", "Pull-off", 0.5, 0.0, 10.0,
                    "Distance the tool pulls off at 45° after each pass"),
             length("back_limit", "Run past the part's back end", 0.0, 0.0, 100.0,
                    "How far beyond the part's back end the passes may run"),
             {"profile_pass",
              "Profile pass",
              ParamKind::Boolean,
              1.0,
              0.0,
              1.0,
              {},
              {},
              "Last pass along the allowance, removing the steps of the level passes"}});
    // Soft jaws bored to fit (DL-0069): what and where come from the setup's jaws, so the jaws
    // and the operation cannot disagree.
    turning("turn_soft_jaws", "Turning: bore soft jaws",
            {length("stock_to_leave", "Stock to leave", 0.0, 0.0, 10.0,
                    "Material left on the jaws' bore (or step) for a later pass"),
             length("retract", "Pull-off", 0.5, 0.0, 10.0,
                    "Distance the tool pulls off at 45° after each pass"),
             {"profile_pass",
              "Profile pass",
              ParamKind::Boolean,
              1.0,
              0.0,
              1.0,
              {},
              {},
              "Last pass along the bore's wall and floor, to size"}});
    turning("turn_finish", "Turning: finish",
            {choice("side", "Side", kSides, "outside", kTurnSide),
             choice("compensation", "Nose compensation",
                    {{"computer", "In computer"}, {"control", "In control (G41/G42)"}}, "computer",
                    "In computer: the program is the nose already offset. In control: the program "
                    "follows the profile with G41/G42, so wear and a different nose radius can be "
                    "set on the machine; the control needs the tool's nose radius and tip "
                    "orientation on its offset page"),
             choice("direction", "Direction", kTurnDirections, "towards_chuck", kTurnDirection),
             choice("z_range", "Z range", kZRanges, "whole", kZRange),
             when(length("z_front", "Z front", 0.0, -10000.0, 10000.0,
                         "Where the range starts, towards the front of the part"),
                  "z_range", {"limited"}),
             when(length("z_back", "Z back", 0.0, -10000.0, 10000.0,
                         "Where the range ends, towards the chuck"),
                  "z_range", {"limited"}),
             length("stock_to_leave", "Stock to leave", 0.0, -1.0, 10.0,
                    "Material left on the profile; negative cuts under it"),
             length("back_limit", "Run past the part's back end", 0.0, 0.0, 100.0,
                    "How far beyond the part's back end the pass may run")});
    turning("turn_groove", "Turning: groove",
            {choice("side", "Side",
                    {{"outside", "Outside (OD)"}, {"inside", "Inside (ID)"}, {"face", "Face"}},
                    "outside",
                    "Grooves in the outside diameter, in a bore, or in a front face, plunging "
                    "along Z"),
             length("stock_to_leave", "Stock to leave", 0.0, 0.0, 10.0,
                    "Material left on the groove's walls and floor"),
             fromTool(length("plunge_step", "Plunge step", 0.0, 0.01, 100.0,
                             "Sideways step between plunges, at most the insert width; while "
                             "not set, 80% of it"),
                      [](const tooling::Tool& t) {
                          return kGrooveStepShare * tooling::turningShape(t).width;
                      }),
             length("peck", "Peck depth", 0.0, 0.0, 50.0, "0 = single plunge")});
    turning("turn_thread", "Turning: thread",
            {choice("side", "Side", kSides, "outside", kTurnSide),
             choice("hand", "Hand", {{"right", "Right hand"}, {"left", "Left hand"}}, "right",
                    "Right hand cuts towards the chuck; left hand cuts away from it, so its run-up "
                    "is behind the thread"),
             length("pitch", "Pitch", 0.0, 0.0, 20.0, "0 = the insert's pitch"),
             {"starts",
              "Starts",
              ParamKind::Integer,
              1,
              1,
              12,
              {},
              {},
              "Thread starts, each a pitch from the next: the lead, and with it the feed, is the "
              "pitch times this"},
             length("taper", "Taper (radial)", 0.0, -1000.0, 1000.0,
                    "The thread's radius at its end minus at its start; 0 = parallel"),
             length("major_diameter", "Major diameter", 0.0, 0.0, 1000.0,
                    "0 = the part's diameter (outside: the major, inside: the bore's, the minor)"),
             length("start_offset", "Start behind front face", 0.0, 0.0, 1000.0,
                    "Where the thread starts, measured from the part's front face"),
             length("length", "Thread length", 10.0, 0.01, 1000.0, "Length of the thread along Z"),
             length("depth", "Thread depth (radial)", 0.0, 0.0, 20.0,
                    "0 = 0.6134 × pitch outside, 0.5413 × pitch inside"),
             {"passes",
              "Passes",
              ParamKind::Integer,
              6,
              1,
              100,
              {},
              {},
              "Cutting passes to the full thread depth (smaller depth per pass as it goes)"},
             {"spring_passes",
              "Spring passes",
              ParamKind::Integer,
              1,
              0,
              10,
              {},
              {},
              "Extra passes at full depth that clean up what deflection left"},
             length("lead_in", "Run-up", 0.0, 0.0, 100.0, "0 = 3 pitches")});
    turning("turn_partoff", "Turning: part off",
            {length("z_offset", "Behind the part's back face", 0.0, 0.0, 100.0,
                    "Where the cut is, measured behind the part's back face"),
             length("x_end", "End radius", -0.5, -10.0, 1000.0,
                    "Radius where the cut ends; below 0 goes past the centreline so the part "
                    "comes off"),
             length("peck", "Peck depth", 0.0, 0.0, 50.0, "0 = single plunge"),
             length("slow_distance", "End feed over the last", 2.0, 0.0, 100.0,
                    "The cutting preset's part-off end feed runs over this last radial distance, "
                    "as the part comes off; 0 = the feed per revolution all the way")});

    // Bar work (DL-0091): a puller on the turret takes the bar out of the open chuck. No spindle
    // and no cutting data: the feeds are the puller's preset's plunge and pull feeds.
    all.push_back(
        {"bar_pull",
         "Bar pull",
         {length("pull_length", "Pull length", 0.0, 0.0, 10000.0,
                 "How far the bar comes out of the chuck: the next part's length with its part-off "
                 "and facing allowances. 0 refuses: it is yours to set"),
          fromTool(length("grip", "Grip", 0.0, 0.01, 1000.0,
                          "How far the puller goes over the bar from its front face; while not "
                          "set, as deep as the puller's jaws"),
                   [](const tooling::Tool& t) { return t.mill.fluteLength; }),
          length("clearance", "Clearance", 2.0, 0.0, 100.0,
                 "In front of the bar, where the puller's rapid stops and it lets go"),
          {"dwell",
           "Dwell at the chuck",
           ParamKind::Time,
           0.0,
           0.0,
           60.0,
           {},
           {},
           "After the chuck opens and after it closes; 0 = none"},
          flag("bar_feeder", "Bar feeder pushes", false,
               "The bar feeder pushes while the chuck is open: the post writes its codes for "
               "it before the chuck opens and after it closes")}});

    // The part handed to the sub-spindle (ADR-0007, DL-0092): no tool. Z2's feed and the spindles'
    // speed are the user's own: 0 refuses.
    all.push_back(
        {"spindle_transfer",
         "Sub-spindle transfer",
         {length("grip", "Grip", 0.0, 0.0, 1000.0,
                 "How far the sub-spindle's jaws go over the part from its front; 0 = the "
                 "machine's sub-spindle grip"),
          length("clearance", "Clearance", 2.0, 0.0, 100.0,
                 "In front of the part, where Z2's rapid stops and its feed starts"),
          {"z2_feed",
           "Z2 feed",
           ParamKind::Feed,
           0.0,
           0.0,
           100000.0,
           {},
           {},
           "Z2's feed over the last stretch onto the part. Yours to set: 0 refuses"},
          choice("spindles", "Spindles",
                 {{"synchronised", "Synchronised"},
                  {"synchronised_angle", "Synchronised, at an angle"},
                  {"stopped", "Both stopped"}},
                 "synchronised",
                 "While the sub-spindle takes the part: both turning together, the sub-spindle "
                 "following the main (at a set angle to it, for a part that is not round), or "
                 "both standing still"),
          when(angle("angle", "Angle", 0.0, -360.0, 360.0,
                     "The sub-spindle's angle to the main spindle"),
               "spindles", {"synchronised_angle"}),
          when({"rpm",
                "Speed (rpm)",
                ParamKind::Integer,
                0,
                0,
                100000,
                {},
                {},
                "The spindles' speed while the part is taken. Yours to set: 0 refuses"},
               "spindles", {"synchronised", "synchronised_angle"}),
          choice("then", "Then",
                 {{"part_off", "Part off: a part-off follows, both spindles holding the part"},
                  {"open_main", "Open the main chuck: the part is already free"}},
                 "part_off",
                 "A part-off operation after this one in the setup cuts the part off the bar, "
                 "with both spindles holding it; or the main chuck opens, for a part that is not "
                 "on a bar"),
          {"dwell",
           "Dwell at the chuck",
           ParamKind::Time,
           0.0,
           0.0,
           60.0,
           {},
           {},
           "After the sub-spindle's chuck closes; 0 = none"}}});

    // Multi-axis (phase 9).
    all.push_back(
        {"rotary_pocket",
         "Wrapped pocket",
         {length("stock_to_leave_xy", "Stock to leave (walls)", 0.0, 0.0, 20.0, kLeaveWalls),
          length("stock_to_leave_z", "Stock to leave (floors)", 0.0, 0.0, 20.0, kLeaveFloors),
          length("resolution", "Height map resolution", 0.0, 0.0, 5.0, "0 = from the tool size"),
          choice("entry", "Entry",
                 {{"ramp", "Ramp along path"}, {"helix", "Helix (circular)"}, {"plunge", "Plunge"}},
                 "ramp", kEntry),
          when(angle("ramp_angle", "Ramp angle", 3.0, 0.1, 45.0, kRampAngle), "entry",
               {"ramp", "helix"}),
          choice("direction", "Direction", kDirections, "climb", kDirection)}});
    // Wrapped contour (DL-0084).
    all.push_back(
        {"rotary_contour",
         "Wrapped contour",
         {choice("side", "Tool side",
                 {{"on", "On the line"},
                  {"left", "Left (inside a closed curve)"},
                  {"right", "Right (outside a closed curve)"}},
                 "on",
                 "Where the tool runs along the selected edges: on them (a slot the tool's width, "
                 "a groove, a line to engrave), or its radius to one side, seen the way the edges "
                 "go: an open curve from its end at the lower X (on a lathe, Z), a closed one "
                 "counter-clockwise seen from outside the part, one around the part with the "
                 "angle. A slot's floor (a face) is cut inside its walls whatever this is"),
          length("depth", "Depth", 0.0, 0.0, 100.0,
                 "How far below the selected edges or floor the tool goes; 0 = to them"),
          length("stock_to_leave_xy", "Stock to leave (walls)", 0.0, -1.0, 20.0, kLeaveWalls),
          length("stock_to_leave_z", "Stock to leave (floor)", 0.0, 0.0, 20.0, kLeaveFloors),
          choice("entry", "Entry", {{"plunge", "Plunge"}, {"ramp", "Ramp along the path"}},
                 "plunge",
                 "How the tool gets down to each level: a plunge, beside the wall at the start of "
                 "the lead arc where there is one, or a ramp forth and back along the start of "
                 "the path, with no lead in"),
          when(angle("ramp_angle", "Ramp angle", 3.0, 0.1, 45.0, kRampAngle), "entry", {"ramp"}),
          fromTool(length("lead_radius", "Lead radius", 0.0, 0.0, 1000.0,
                          "Radius of the arcs that lead on to and off a wall beside the path; "
                          "there are none on the line or in a slot the tool's width. 0 = straight "
                          "on. While not set, it follows the tool's size"),
                   [](const tooling::Tool& t) { return kContourLeadShare * toolDiameter(t); }),
          length("lead_length", "Lead line length", 0.0, 0.0, 100.0,
                 "Straight approach before the lead arc"),
          choice("direction", "Direction", kDirections, "climb",
                 "Beside a wall, which way the tool goes along it. Climb: the teeth enter the "
                 "material at full chip thickness, the usual choice on CNC machines. "
                 "Conventional: the opposite. On the line it goes the way the edges go")}});
    const char* const kAxisPoint = "The point the tool axis points away from or towards (WCS)";
    all.push_back(
        {"multiaxis_finish",
         "5-axis finishing",
         {length("scallop_height", "Scallop height", 0.0, 0.0, 5.0,
                 "Sets the stepover; 0 = the stepover is the cutting preset's radial depth (ae)"),
          angle("angle", "Raster angle", 0.0, -360.0, 360.0, kPassAngle),
          length("sample_step", "Sample step", 0.0, 0.0, 5.0, kSampleStep),
          length("stock_to_leave", "Stock to leave", 0.0, -1.0, 20.0, kLeaveSurface),
          choice("pattern", "Pattern", {{"zigzag", "Zig-zag"}, {"one_way", "One way"}}, "zigzag",
                 kPattern),
          choice("axis_mode", "Tool axis",
                 {{"lead_tilt", "Lead / tilt to the surface normal"},
                  {"fixed", "Fixed direction"},
                  {"from_point", "Away from a point"},
                  {"to_point", "Towards a point"}},
                 "lead_tilt",
                 "How the tool axis is set along the path: tilted from the surface normal by the "
                 "lead and tilt angles, one fixed direction, or pointing away from or towards a "
                 "point"),
          when(angle("lead_angle", "Lead angle", 0.0, -89.0, 89.0, "Towards the feed direction"),
               "axis_mode", {"lead_tilt"}),
          when(angle("tilt_angle", "Tilt angle", 0.0, -89.0, 89.0,
                     "To the left of the feed direction"),
               "axis_mode", {"lead_tilt"}),
          when(angle("fixed_tilt", "Fixed: tilt from Z", 0.0, 0.0, 120.0,
                     "Angle of the fixed tool axis from the WCS Z axis"),
               "axis_mode", {"fixed"}),
          when(angle("fixed_azimuth", "Fixed: direction from X", 0.0, -360.0, 360.0,
                     "Direction the fixed tool axis leans towards, from the X axis"),
               "axis_mode", {"fixed"}),
          when(length("point_x", "Point X", 0.0, -1e5, 1e5, kAxisPoint), "axis_mode",
               {"from_point", "to_point"}),
          when(length("point_y", "Point Y", 0.0, -1e5, 1e5, kAxisPoint), "axis_mode",
               {"from_point", "to_point"}),
          when(length("point_z", "Point Z", 0.0, -1e5, 1e5, kAxisPoint), "axis_mode",
               {"from_point", "to_point"}),
          angle("max_tilt", "Largest tilt from Z", 60.0, 0.0, 120.0,
                "Steeper tool axes are limited to this angle from the WCS Z axis"),
          length("clearance", "Shank clearance", 0.2, 0.0, 50.0,
                 "Gap the flutes and shank must keep from the part; points that come closer "
                 "are reported")}});
    // Swarf (DL-0085): the tool axis follows the walls' rulings; nothing else sets it.
    all.push_back(
        {"swarf5x",
         "5-axis swarf",
         {{"depth_passes",
           "Depth passes",
           ParamKind::Integer,
           1,
           1,
           50,
           {},
           {},
           "Passes down the walls along their rulings, each an equal share of the wall's height, "
           "the deepest last"},
          length("axis_offset", "Axis offset", 0.0, -100.0, 100.0,
                 "How far the tool's end stays above the floor (with a floor selected) or above "
                 "the walls' bottom edges, along the tool axis; negative runs past the edges, as "
                 "for a wall open at the bottom"),
          length("stock_to_leave", "Stock to leave", 0.0, -1.0, 20.0, kLeaveWalls),
          length("tolerance", "Ruling tolerance", 0.01, 0.0005, 1.0,
                 "How far a wall may stray from straight rulings, and the side of the tool from "
                 "a twisted wall between the ends of its rulings; walls beyond it are refused"),
          angle("max_tilt", "Largest tilt from Z", 60.0, 0.0, 120.0,
                "Walls whose rulings lean further than this from the Z axis are refused: the "
                "tool axis is never tilted away from a ruling"),
          length("lead_distance", "Lead distance", 2.0, 0.0, 50.0,
                 "The tool comes down the axis this far off the wall, along its normal, leads onto "
                 "the wall and off it again at the lead feed"),
          choice("direction", "Direction", kDirections, "climb", kDirection),
          length("clearance", "Shank clearance", 0.2, 0.0, 50.0,
                 "Gap the flutes and shank must keep from the part; positions that come closer "
                 "are reported")}});
    return all;
}

} // namespace

// What each strategy reads beyond its parameters, and the parameter groups they share.
std::vector<StrategyInfo> finishStrategies(std::vector<StrategyInfo> all) {
    const auto find = [&](std::string_view id) -> StrategyInfo& {
        for (StrategyInfo& s : all) {
            if (s.id == id) {
                return s;
            }
        }
        throw Error(ErrorCode::Internal, "unknown strategy " + std::string(id));
    };
    for (const char* id : {"pocket2d", "plunge_rough"}) {
        find(id).geometry = GeometryUse::Faces;
    }
    find("contour2d").geometry = GeometryUse::FacesOrEdges;
    find("rotary_contour").geometry = GeometryUse::FacesOrEdges;
    find("swarf5x").geometry = GeometryUse::Walls;
    find("chamfer2d").geometry = GeometryUse::Edges; // the edges to break
    find("trace2d").geometry = GeometryUse::Chains;  // the edges to follow
    for (const char* id : {"drill", "tap", "threadmill", "bore"}) {
        find(id).geometry = GeometryUse::Holes;
    }
    // On a lathe (the panel shows it there only), what turns: a hole is one choice or the other,
    // and nothing picks it for the user.
    for (const char* id : {"drill", "tap", "bore"}) {
        auto& params = find(id).params;
        params.insert(
            params.begin(),
            choice("drive", "Tool drive",
                   {{"spindle", "Spindle: the part turns"}, {"live", "Live tool: the tool turns"}},
                   "spindle",
                   "On a lathe: the spindle turns the part against a tool standing "
                   "still, on the spindle's axis only. A live tool turns itself, with "
                   "the part standing still, on or off the axis, if the machine has "
                   "live tooling."));
    }
    // On a lathe (the panel shows them there only), how a live tool goes around what it cuts.
    for (StrategyInfo& s : all) {
        if (s.id.starts_with("turn_") || s.id == "bar_pull" || s.id == "spindle_transfer" ||
            s.id == "drill" || s.id == "tap" || s.id == "rotary_pocket" ||
            s.id == "rotary_contour" || s.id == "multiaxis_finish" || s.id == "swarf5x") {
            continue;
        }
        ParamDef axes = choice(
            "live_axes", "Live tool axes",
            {{"polar", "C and X: polar interpolation"}, {"y", "Y axis: C held still"}}, "polar",
            "On a lathe: polar interpolation works on the part's face on any "
            "machine with a C axis. The Y axis holds C still and moves X, Y "
            "and Z, on the face or across the part.");
        ParamDef heldC = when(angle("live_c", "C held at", 0.0, -360.0, 360.0,
                                    "On a lathe's face: where C stands while X, Y and Z move. "
                                    "Across the part the tool's direction sets it."),
                              "live_axes", {"y"});
        if (s.id == "bore") {
            axes = when(std::move(axes), "drive", {"live"});
        }
        const auto at = s.params.begin() + (s.id == "bore" ? 1 : 0);
        s.params.insert(s.params.insert(at, std::move(heldC)), std::move(axes));
    }
    for (const char* id :
         {"parallel3d", "waterline3d", "contour3d", "flat3d", "pencil3d", "steep_shallow3d"}) {
        find(id).geometry = GeometryUse::Containment;
    }
    // Rest machining of contour, pencil and steep and shallow finishing (parallel finishing has
    // the same group inline).
    {
        auto rest = restParams("Finish only where earlier operations left stock (simulated)");
        for (const char* id : {"contour3d", "pencil3d", "steep_shallow3d"}) {
            auto& params = find(id).params;
            params.insert(params.end(), rest.begin(), rest.end());
        }
    }
    // Arc output on the milling operations whose paths it changes. Not on facing (straight
    // rasters), plunge roughing (plunges), the hole operations (circles by nature), turning
    // and the multi-axis ones (straight moves with a turning tool axis).
    for (const char* id :
         {"pocket2d", "contour2d", "chamfer2d", "trace2d", "zlevel_rough", "flat3d", "parallel3d",
          "waterline3d", "contour3d", "pencil3d", "steep_shallow3d"}) {
        auto smoothing = smoothingParams();
        auto& params = find(id).params;
        params.insert(params.end(), smoothing.begin(), smoothing.end());
    }
    for (ParamDef& d : find("drill").params) {
        if (d.key == "dwell") {
            d.onlyWhen = {"cycle", {"auto", "dwell"}, -1e300};
        }
    }
    // What each strategy is for (tooltips where operations are added and chosen).
    const std::pair<const char*, const char*> descriptions[] = {
        {"facing", "Flattens the top of the stock with parallel passes across the whole face, "
                   "down to the top of the part or the selected faces."},
        {"plunge_rough",
         "Clears a pocket with overlapping straight plunges, row by row, then along its wall. "
         "For long tools and deep pockets, where the tool is stiffest along its axis."},
        {"pocket2d", "Clears the material inside the selected floors and walls, level by level, "
                     "with offset passes that keep the tool load even."},
        {"contour2d", "Cuts along the outline of the selected faces or edges, inside or outside: "
                      "profiles, walls and cut-outs, with leads on and off the wall."},
        {"chamfer2d", "Breaks the selected edges with a chamfer or engraving tool running along "
                      "the outline at the chamfer's width."},
        {"trace2d", "Runs the tool along the selected edges, open chains or closed, on them or "
                    "beside them, to a depth: engraving, scribing, part marking and slots along "
                    "a curve."},
        {"drill", "Drills the selected holes with canned cycles: plain drilling, pecking, chip "
                  "breaking, dwell, tapping or boring. On a lathe: on the spindle axis, centre "
                  "holes included, with the tool standing still."},
        {"tap", "Taps the selected holes with a tap in a tapping cycle (G84), the feed matched to "
                "the thread pitch."},
        {"threadmill", "Mills threads with a thread mill on a helix at the thread pitch: internal "
                       "ones in the selected holes, external ones around bosses and studs."},
        {"bore", "Finishes the selected holes to size: a boring cycle with a boring bar or "
                 "reamer, or a helix with an end mill."},
        {"zlevel_rough", "Removes the bulk of the material around a 3D part, level by level from "
                         "the top down, leaving an even allowance for finishing."},
        {"parallel3d", "Finishes 3D surfaces with parallel passes that follow the surface; best "
                       "on shallow and gently curved areas."},
        {"contour3d", "Finishes steep walls and blends with passes around the part at constant "
                      "heights, closer together where the slope flattens."},
        {"pencil3d", "Cleans up concave corners and fillets that a larger tool could not reach "
                     "into, with single passes along them and optional offset passes beside."},
        {"steep_shallow3d",
         "Finishes the whole 3D surface in one operation: passes around the part at constant "
         "heights on steep areas, parallel passes on shallow ones, overlapping between them."},
        {"waterline3d", "Finishes steep walls with passes around the part at constant Z levels "
                        "(waterlines)."},
        {"flat3d", "Finishes only the flat areas of a 3D part (floors, ledges, tops) to their "
                   "exact heights, leaving the walls alone."},
        {"turn_face", "Faces the front of the part on a lathe, from the outside to the centre."},
        {"turn_rough", "Removes the bulk of the material along the turned profile in level "
                       "passes, outside or in a bore, leaving an allowance for finishing."},
        {"turn_finish", "Finishes the turned profile in one pass along it, outside or in a bore."},
        {"turn_soft_jaws", "Bores the setup's soft jaws to fit, as its workholding says: a bore "
                           "into the jaws for an outside grip, a step on their outside for an "
                           "inside one, with the jaws clamping a ring or plug."},
        {"turn_groove", "Cuts grooves with a grooving insert: side-by-side plunges, then the "
                        "groove's walls and floor."},
        {"turn_thread", "Cuts a thread on a lathe with a threading insert, in several passes to "
                        "the full depth."},
        {"turn_partoff", "Cuts the finished part off the bar with a parting insert."},
        {"bar_pull", "Pulls the bar out of the chuck with a puller on the turret, for the next "
                     "part: the chuck opens, the bar comes out by the pull length, the chuck "
                     "closes."},
        {"spindle_transfer", "Hands the part to the sub-spindle: its chuck comes up and closes on "
                             "the part, the spindles synchronised or stopped, for a part-off "
                             "under grip or the main chuck opening."},
        {"rotary_pocket", "Clears pockets wrapped around a turned part with the 4th axis, or a "
                          "lathe's C axis with a live tool, turning "
                          "the part under the tool."},
        {"rotary_contour",
         "Follows edges or a slot's floor wrapped around the part with the 4th axis, or a lathe's "
         "C axis with a live tool: keyways, helical grooves, cam tracks and the walls of wrapped "
         "profiles, on the line, beside it, or inside a slot's walls. The feeds are along the "
         "tool tip as set: on a 4th axis without tool centre point control the program gives "
         "each move's time instead (inverse time, G93), and on a lathe the feed is on the "
         "floor's radius of cylindrical interpolation."},
        {"multiaxis_finish",
         "Finishes surfaces with a ball end mill and a tool axis that tilts along the path "
         "(simultaneous 5-axis), to reach under overhangs or use a shorter tool."},
        {"swarf5x",
         "Cuts ruled walls (drafted, tapered or twisted) with the side of the tool in one pass "
         "down each ruling, the tool axis following the wall (simultaneous 5-axis)."},
    };
    for (const auto& [id, text] : descriptions) {
        find(id).description = text;
    }
    return all;
}

const std::vector<StrategyInfo>& strategies() {
    static const std::vector<StrategyInfo> all = [] {
        std::vector<StrategyInfo> list = finishStrategies(buildStrategies());
        std::vector<StrategyInfo> probing = probingStrategies(); // Probing.cpp (DL-0090)
        list.insert(list.end(), probing.begin(), probing.end());
        return list;
    }();
    return all;
}

const ParamDef* StrategyInfo::find(std::string_view key) const {
    for (const ParamDef& d : params) {
        if (d.key == key) {
            return &d;
        }
    }
    return nullptr;
}

bool StrategyInfo::applies(const ParamDef& def, const Json& values) const {
    if (def.onlyWhen.key.empty()) {
        return true;
    }
    const ParamDef* on = find(def.onlyWhen.key);
    if (!on) {
        return true;
    }
    if (!applies(*on, values)) {
        return false; // what it depends on does not matter either
    }
    const Json* v = values.is_object() && values.contains(on->key) ? &values.at(on->key) : nullptr;
    switch (on->kind) {
    case ParamKind::Choice: {
        const std::string value = v && v->is_string() ? v->get<std::string>() : on->defaultChoice;
        return std::find(def.onlyWhen.values.begin(), def.onlyWhen.values.end(), value) !=
               def.onlyWhen.values.end();
    }
    case ParamKind::Boolean: {
        const bool set =
            v ? (v->is_boolean() ? v->get<bool>() : (v->is_number() && v->get<double>() != 0.0))
              : on->defaultValue != 0.0;
        const std::string value = set ? "on" : "off";
        return std::find(def.onlyWhen.values.begin(), def.onlyWhen.values.end(), value) !=
               def.onlyWhen.values.end();
    }
    default: {
        const double value = v && v->is_number() ? v->get<double>() : on->defaultValue;
        return value > def.onlyWhen.above;
    }
    }
}

const StrategyInfo* findStrategy(std::string_view id) {
    for (const auto& s : strategies()) {
        if (s.id == id) {
            return &s;
        }
    }
    return nullptr;
}

ParamReader::ParamReader(const StrategyInfo& info, const Json& values)
    : info_(info), values_(values) {}

const ParamDef& ParamReader::def(std::string_view key) const {
    for (const auto& d : info_.params) {
        if (d.key == key) {
            return d;
        }
    }
    throw Error(ErrorCode::Internal, fmt::format("unknown parameter '{}' for {}", key, info_.id));
}

bool ParamReader::isSet(std::string_view key) const {
    return values_.is_object() && values_.contains(std::string(key)) &&
           !values_.at(std::string(key)).is_null();
}

double ParamReader::number(std::string_view key) const {
    const ParamDef& d = def(key);
    double v = d.defaultValue;
    if (isSet(key)) {
        const Json& j = values_.at(std::string(key));
        if (!j.is_number()) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("parameter '{}' must be a number", key));
        }
        v = j.get<double>();
    }
    if (!std::isfinite(v) || v < d.min || v > d.max) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("parameter '{}' = {} is outside [{}, {}]", key, v, d.min, d.max));
    }
    if (d.kind == ParamKind::SurfaceSpeed) {
        return v * 1000.0 / 60.0;
    }
    if (d.kind == ParamKind::Feed) {
        return v / 60.0;
    }
    return d.kind == ParamKind::Angle ? degToRad(v) : v;
}

int ParamReader::integer(std::string_view key) const {
    return static_cast<int>(std::lround(number(key)));
}

bool ParamReader::flag(std::string_view key) const {
    const ParamDef& d = def(key);
    if (!isSet(key)) {
        return d.defaultValue != 0.0;
    }
    const Json& j = values_.at(std::string(key));
    if (!j.is_boolean()) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("parameter '{}' must be true or false", key));
    }
    return j.get<bool>();
}

std::string ParamReader::choice(std::string_view key) const {
    const ParamDef& d = def(key);
    if (!isSet(key)) {
        return d.defaultChoice;
    }
    const Json& j = values_.at(std::string(key));
    const std::string v = j.is_string() ? j.get<std::string>() : std::string();
    if (std::none_of(d.choices.begin(), d.choices.end(),
                     [&](const auto& c) { return c.first == v; })) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("parameter '{}' has an invalid value", key));
    }
    return v;
}

// -- Typed parameter objects -------------------------------------------------------------

namespace {

std::string presetLabel(const tooling::CuttingPreset* preset) {
    return preset ? fmt::format("cutting preset '{}'", preset->name)
                  : std::string("no cutting preset");
}

} // namespace

double presetStepDown(const tooling::CuttingPreset* preset) {
    if (!preset || preset->axialDepth <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("{}: the step down (depth of cut) is its axial depth ap, which "
                                "is not set; set it on the tool's preset or on the operation",
                                presetLabel(preset)));
    }
    return preset->axialDepth;
}

double presetStepover(const tooling::CuttingPreset* preset) {
    if (!preset || preset->radialDepth <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("{}: the stepover is its radial depth ae, which is not set; "
                                "set it on the tool's preset or on the operation",
                                presetLabel(preset)));
    }
    return preset->radialDepth;
}

double presetStepoverRatio(const tooling::CuttingPreset* preset, const tooling::Tool& tool) {
    const double ae = presetStepover(preset);
    if (tool.mill.diameter <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the stepover needs a tool with a diameter");
    }
    return std::min(ae / tool.mill.diameter, 1.0);
}

FacingParams facingParams(const Json& values, const tooling::Tool& tool,
                          const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("facing"), values);
    FacingParams p;
    p.stepoverRatio = presetStepoverRatio(preset, tool);
    p.stepDown = presetStepDown(preset);
    p.stockToLeave = r.number("stock_to_leave");
    p.angle = r.number("angle");
    p.pattern = r.choice("pattern") == "one_way" ? FacingPattern::OneWay : FacingPattern::ZigZag;
    p.overrun = r.number("overrun");
    return p;
}

PocketParams pocketParams(const Json& values, const tooling::Tool& tool,
                          const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("pocket2d"), values);
    PocketParams p;
    p.stepoverRatio = presetStepoverRatio(preset, tool);
    p.stepDown = presetStepDown(preset);
    p.stockToLeaveXY = r.number("stock_to_leave_xy");
    p.stockToLeaveZ = r.number("stock_to_leave_z");
    const std::string entry = r.choice("entry");
    p.entry = entry == "ramp"     ? EntryKind::Ramp
              : entry == "plunge" ? EntryKind::Plunge
                                  : EntryKind::Helix;
    p.rampAngle = r.number("ramp_angle");
    if (!r.isSet("ramp_angle") && preset && preset->rampAngle > 0.0) {
        p.rampAngle = preset->rampAngle;
    }
    if (r.isSet("helix_radius")) {
        p.helixRadius = r.number("helix_radius");
    }
    p.direction =
        r.choice("direction") == "conventional" ? CutDirection::Conventional : CutDirection::Climb;
    return p;
}

PlungeParams plungeParams(const Json& values, const tooling::Tool& tool,
                          const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("plunge_rough"), values);
    PlungeParams p;
    p.stepoverRatio = presetStepoverRatio(preset, tool);
    if (r.isSet("plunge_spacing")) {
        p.pitch = r.number("plunge_spacing");
    }
    p.stagger = r.flag("stagger");
    p.wall = r.flag("wall");
    p.stockToLeaveXY = r.number("stock_to_leave_xy");
    p.stockToLeaveZ = r.number("stock_to_leave_z");
    return p;
}

ContourParams contourParams(const Json& values, const tooling::Tool& /*tool*/,
                            const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("contour2d"), values);
    ContourParams p; // the side is each outline's (flipped per outline, else automatic)
    p.stockToLeaveXY = r.number("stock_to_leave_xy");
    p.stockToLeaveZ = r.number("stock_to_leave_z");
    p.stepDown = presetStepDown(preset);
    p.passes = r.integer("passes");
    p.passSpacing = r.number("pass_spacing");
    p.springPass = r.flag("spring_pass");
    if (r.isSet("lead_radius")) {
        p.leadRadius = r.number("lead_radius");
    }
    p.leadLength = r.number("lead_length");
    p.direction =
        r.choice("direction") == "conventional" ? CutDirection::Conventional : CutDirection::Climb;
    p.compensation = r.choice("compensation") == "control" ? CompensationMode::Control
                                                           : CompensationMode::Computer;
    p.tabs.enabled = r.flag("tabs");
    if (p.tabs.enabled) {
        if (r.choice("tab_placement") == "spacing") {
            p.tabs.spacing = r.number("tab_spacing");
        } else {
            p.tabs.count = r.integer("tab_count");
        }
        p.tabs.width = r.number("tab_width");
        p.tabs.height = r.number("tab_height");
        p.tabs.shape = r.choice("tab_shape") == "triangle" ? TabShape::Triangle : TabShape::Square;
    }
    return p;
}

DrillParams drillParams(const Json& values) {
    const ParamReader r(*findStrategy("drill"), values);
    DrillParams p;
    const std::string cycle = r.choice("cycle");
    if (cycle != "auto") {
        p.cycle = toolpath::parseCycleType(cycle);
    }
    if (r.isSet("peck_depth")) {
        p.peckDepth = r.number("peck_depth");
    }
    p.dwell = r.number("dwell");
    p.depthMode = r.choice("depth_mode") == "tip" ? DepthMode::Tip : DepthMode::FullDiameter;
    p.breakthrough = r.number("breakthrough");
    p.holeDepth = r.number("hole_depth");
    return p;
}

ChamferParams chamferParams(const Json& values) {
    const ParamReader r(*findStrategy("chamfer2d"), values);
    ChamferParams p; // the side is each outline's (chooseable per outline, else automatic)
    p.width = r.number("width");
    p.tipOffset = r.number("tip_offset");
    if (r.isSet("lead_radius")) {
        p.leadRadius = r.number("lead_radius");
    }
    p.direction =
        r.choice("direction") == "conventional" ? CutDirection::Conventional : CutDirection::Climb;
    return p;
}

TraceParams traceParams(const Json& values, const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("trace2d"), values);
    const StrategyInfo& info = *findStrategy("trace2d");
    TraceParams p;
    p.depth = r.number("depth");
    if (p.depth > 0.0) {
        p.stepDown = presetStepDown(preset);
    }
    const std::string side = r.choice("side");
    p.side = side == "left" ? TraceSide::Left : side == "right" ? TraceSide::Right : TraceSide::On;
    // What does not apply (hidden in the panel) does not change the path either.
    if (p.side != TraceSide::On) {
        p.offset = r.number("offset");
        p.passes = r.integer("passes");
        if (info.applies(*info.find("pass_spacing"), values)) {
            p.passSpacing = r.number("pass_spacing");
        }
    }
    p.reverse = r.flag("reverse");
    p.entry = r.choice("entry") == "ramp" ? EntryKind::Ramp : EntryKind::Plunge;
    if (p.entry == EntryKind::Ramp) {
        p.rampAngle = r.number("ramp_angle");
        if (!r.isSet("ramp_angle") && preset && preset->rampAngle > 0.0) {
            p.rampAngle = preset->rampAngle;
        }
    }
    p.sampleStep = r.number("sample_step");
    return p;
}

bool traceFollowsSurface(const Json& values) {
    return ParamReader(*findStrategy("trace2d"), values).flag("follow_surface");
}

TapParams tapParams(const Json& values) {
    const ParamReader r(*findStrategy("tap"), values);
    TapParams p;
    p.threadDepth = r.number("thread_depth");
    p.breakthrough = r.number("breakthrough");
    return p;
}

ThreadMillParams threadMillParams(const Json& values) {
    const ParamReader r(*findStrategy("threadmill"), values);
    ThreadMillParams p;
    p.external = r.choice("side") == "external";
    p.pitch = r.number("pitch");
    p.majorDiameter = r.number("major_diameter");
    p.threadDepth = r.number("radial_depth");
    p.depth = r.number("thread_depth");
    p.rightHand = r.choice("hand") != "left";
    p.direction =
        r.choice("direction") == "conventional" ? CutDirection::Conventional : CutDirection::Climb;
    p.multiForm = r.choice("form") == "multi";
    p.radialPasses = r.integer("passes");
    p.springPass = r.flag("spring_pass");
    if (r.isSet("lead_radius")) {
        p.leadRadius = r.number("lead_radius");
    }
    return p;
}

BoreParams boreParams(const Json& values) {
    const ParamReader r(*findStrategy("bore"), values);
    BoreParams p;
    p.helixPitch = r.number("helix_pitch");
    p.stockToLeave = r.number("stock_to_leave");
    p.breakthrough = r.number("breakthrough");
    p.holeDepth = r.number("hole_depth");
    p.direction =
        r.choice("direction") == "conventional" ? CutDirection::Conventional : CutDirection::Climb;
    return p;
}

} // namespace ocam::strategies
