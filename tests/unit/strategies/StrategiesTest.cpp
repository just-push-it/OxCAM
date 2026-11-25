// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "StrategyTestUtil.hpp"

#include <ocam/foundation/Error.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>

using namespace ocam;
using namespace ocam::strategies;
using namespace ocam::strategies::testing;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using toolpath::FeedClass;
using toolpath::MoveKind;

namespace {

double uncoveredArea(const geom2d::Regions& material, const std::vector<std::vector<Vec2>>& paths,
                     double radius) {
    const geom2d::ClipOptions clip{0.001};
    const auto left = geom2d::subtract(material, geom2d::sweep(paths, radius, clip), clip);
    // Ignore slivers from discretization noise.
    return geom2d::totalArea(geom2d::offset(geom2d::offset(left, -0.01, clip), 0.01, clip));
}

} // namespace

TEST_CASE("every strategy says what it is for, every parameter what it does",
          "[strategies][params]") {
    // The interface shows these as tooltips (and translates them).
    for (const StrategyInfo& info : ocam::strategies::strategies()) {
        INFO(info.id);
        CHECK_FALSE(info.label.empty());
        CHECK_FALSE(info.description.empty());
        for (const ParamDef& def : info.params) {
            INFO(def.key);
            CHECK_FALSE(def.label.empty());
            CHECK_FALSE(def.help.empty());
            for (const auto& [value, label] : def.choices) {
                CHECK_FALSE(label.empty());
            }
        }
    }
}

TEST_CASE("parameter reader: defaults, ranges and conversions", "[strategies][params]") {
    const StrategyInfo* info = findStrategy("pocket2d");
    REQUIRE(info != nullptr);
    const Json values{{"stock_to_leave_xy", 0.3}, {"ramp_angle", 5.0}, {"entry", "ramp"}};
    const ParamReader r(*info, values);
    CHECK(r.number("stock_to_leave_xy") == 0.3);
    CHECK_THAT(r.number("ramp_angle"), WithinRel(degToRad(5.0), 1e-12));
    CHECK(r.number("stock_to_leave_z") == 0.0); // default
    CHECK(r.choice("entry") == "ramp");
    CHECK(r.choice("direction") == "climb");
    CHECK(r.isSet("entry"));
    CHECK_FALSE(r.isSet("stock_to_leave_z"));

    CHECK_THROWS_AS(
        ParamReader(*info, Json{{"stock_to_leave_xy", 500.0}}).number("stock_to_leave_xy"), Error);
    CHECK_THROWS_AS(ParamReader(*info, Json{{"entry", "teleport"}}).choice("entry"), Error);
    CHECK_THROWS_AS(
        ParamReader(*info, Json{{"stock_to_leave_z", "deep"}}).number("stock_to_leave_z"), Error);
    CHECK(findStrategy("nope") == nullptr);
    // With trace2d (DL-0088), pencil3d and steep_shallow3d (DL-0082), backbore and edgebreak3d
    // (DL-0083), rotary_contour (DL-0084), swarf5x (DL-0085), the five probing cycles (DL-0090),
    // bar_pull (DL-0091) and spindle_transfer (DL-0092).
    CHECK(ocam::strategies::strategies().size() == 37);

    // The step down and stepover are the preset's axial and radial depth (a "step_down"
    // parameter is not one); the ramp angle is the preset's where unset.
    tooling::Tool tool = tooling::makeFlatEndMill(1, 10, 22, 72);
    tooling::CuttingPreset preset;
    preset.axialDepth = 3.0;
    preset.radialDepth = 4.0;
    preset.rampAngle = degToRad(3.0);
    const PocketParams pp = pocketParams(Json::object(), tool, &preset);
    CHECK(pp.stepDown == 3.0);
    CHECK_THAT(pp.stepoverRatio, WithinRel(0.4, 1e-12));
    CHECK_THAT(pp.rampAngle, WithinRel(degToRad(3.0), 1e-12));
    CHECK(pocketParams(Json{{"step_down", 1.0}}, tool, &preset).stepDown == 3.0);
    CHECK(presetStepDown(&preset) == 3.0);
    CHECK(presetStepover(&preset) == 4.0);
    preset.radialDepth = 12.0; // wider than the tool: one diameter
    CHECK(presetStepoverRatio(&preset, tool) == 1.0);
    preset.axialDepth = 0.0;
    CHECK_THROWS_AS(pocketParams(Json::object(), tool, &preset), Error);
    CHECK_THROWS_AS(presetStepDown(nullptr), Error);
}

TEST_CASE("feeds from a cutting preset", "[strategies]") {
    tooling::Tool tool = tooling::makeFlatEndMill(1, 10, 22, 72); // 3 flutes
    tooling::CuttingPreset preset;
    preset.surfaceSpeed = 200.0 * 1000.0 / 60.0;
    preset.feedPerTooth = 0.05;
    preset.plungeFeed = 3.0;
    preset.rampFeed = 4.0;
    preset.leadFeed = 5.0;
    const Feeds f = feedsFor(tool, &preset);
    const double n = preset.spindleSpeed(10.0);
    CHECK_THAT(f.spindle, WithinRel(n, 1e-12));
    CHECK_THAT(f.cut, WithinRel(0.05 * 3 * n, 1e-12));
    CHECK(f.plunge == 3.0);
    CHECK(f.ramp == 4.0); // its own value, not the feed rate
    CHECK(f.lead == 5.0); // its own value too
    // Nothing is assumed (ADR-0004): zero stays zero, and no preset gives no feeds.
    preset.plungeFeed = 0.0;
    preset.rampFeed = 0.0;
    preset.feedPerTooth = 0.0;
    const Feeds zero = feedsFor(tool, &preset);
    CHECK(zero.plunge == 0.0);
    CHECK(zero.cut == 0.0);
    CHECK(zero.ramp == 0.0);
    CHECK(zero.spindle == f.spindle);
    const Feeds none = feedsFor(tool, nullptr);
    CHECK((none.cut == 0.0 && none.plunge == 0.0 && none.spindle == 0.0));
}

TEST_CASE("depth levels", "[strategies]") {
    CHECK(depthLevels(0, -1, 0) == std::vector<double>{-1});
    const auto levels = depthLevels(0, -5, 2);
    REQUIRE(levels.size() == 3);
    CHECK_THAT(levels[0], WithinAbs(-5.0 / 3, 1e-12));
    CHECK(levels.back() == -5);
    CHECK(depthLevels(0, 0, 1).empty());
    CHECK(depthLevels(0, 1, 1).empty());
}

TEST_CASE("facing covers the stock at every level", "[strategies][facing]") {
    Harness h(tooling::makeFlatEndMill(1, 20, 30, 80));
    const geom2d::Regions stock{rect(0, 0, 100, 60)};
    FacingParams p;
    p.stepDown = 0.6;
    const auto tp = facing(stock, 0.0, -1.0, p, h.context());
    REQUIRE(tp.warnings.empty());
    for (double z : depthLevels(0.0, -1.0, 0.6)) {
        INFO("z = " << z);
        CHECK(uncoveredArea(stock, cutPathsAt(tp, z), 10.0) < 1e-6);
    }
    // Zig-zag links run in the air outside the stock.
    const geom2d::Regions touch = geom2d::offset(stock, 10.0 - 1e-3, {0.002});
    Vec3 pos = tp.start;
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::Link || m.feedClass == FeedClass::Plunge) {
            for (int k = 0; k <= 8; ++k) {
                const Vec2 q = (pos + (m.end - pos) * (k / 8.0)).head<2>();
                CHECK_FALSE(touch[0].contains(q));
            }
        }
        pos = m.end;
    }
    CHECK(tp.moves.back().end.z() == h.heights.clearance);
    checkGolden("facing_rect_zigzag", tp);
}

TEST_CASE("facing at an angle and one-way", "[strategies][facing]") {
    Harness h(tooling::makeFlatEndMill(1, 12, 30, 80));
    const geom2d::Regions stock{{geom2d::Polyline::circle({0, 0}, 40), {}}};
    FacingParams p;
    p.angle = degToRad(30.0);
    p.pattern = FacingPattern::OneWay;
    const auto tp = facing(stock, 2.0, 0.0, p, h.context());
    CHECK(uncoveredArea(stock, cutPathsAt(tp, 0.0), 6.0) < 1e-6);
    // One-way: every pass is followed by a retract.
    const auto rapidsUp = std::count_if(tp.moves.begin(), tp.moves.end(), [](const auto& m) {
        return m.kind == MoveKind::Rapid && m.end.z() > 0.0;
    });
    CHECK(rapidsUp > 5);

    const auto none = facing(stock, 0.0, 0.0, p, h.context());
    CHECK(none.moves.empty());
    CHECK(none.warnings.size() == 1);
}

TEST_CASE("pocket: rectangular pocket with an island", "[strategies][pocket]") {
    Harness h(tooling::makeFlatEndMill(1, 6, 15, 50));
    geom2d::Polyline island = geom2d::Polyline::circle({20, 10}, 3);
    island.reverse();
    const geom2d::Regions area{{geom2d::Polyline::rectangle({0, 0}, {40, 20}), {island}}};
    PocketParams p;
    p.stepDown = 2.0;
    const auto tp = pocket(area, 0.0, -4.0, p, h.context());
    CHECK(tp.warnings.empty());

    const geom2d::ClipOptions clip{0.002};
    const auto allowed = geom2d::offset(area, -3.0, clip);
    const auto allowedGrown = geom2d::offset(allowed, 0.01, clip);
    const auto reachable = geom2d::offset(allowed, 3.0, clip);
    for (double z : {-2.0, -4.0}) {
        INFO("z = " << z);
        CHECK(uncoveredArea(reachable, cutPathsAt(tp, z), 3.0) < 1e-3);
    }
    for (const Vec3& q : feedPoints(tp)) {
        if (q.z() < -1e-9) {
            CHECK(std::any_of(allowedGrown.begin(), allowedGrown.end(),
                              [&](const auto& r) { return r.contains(q.head<2>()); }));
        }
    }
    // Climb: loops around the island run clockwise and keep true arcs.
    CHECK(std::any_of(tp.moves.begin(), tp.moves.end(),
                      [](const auto& m) { return m.kind == MoveKind::ArcCW; }));
    CHECK(std::any_of(tp.moves.begin(), tp.moves.end(),
                      [](const auto& m) { return m.feedClass == FeedClass::Ramp; }));
    checkGolden("pocket_rect_island", tp);
}

TEST_CASE("pocket entries: helix, ramp and plunge", "[strategies][pocket]") {
    Harness h(tooling::makeFlatEndMill(1, 6, 15, 50));
    const geom2d::Regions area{rect(0, 0, 30, 30)};
    for (EntryKind entry : {EntryKind::Helix, EntryKind::Ramp, EntryKind::Plunge}) {
        PocketParams p;
        p.entry = entry;
        p.rampAngle = degToRad(3.0);
        const auto tp = pocket(area, 0.0, -3.0, p, h.context());
        const auto ramps = std::count_if(tp.moves.begin(), tp.moves.end(), [](const auto& m) {
            return m.feedClass == FeedClass::Ramp;
        });
        const auto plunges = std::count_if(tp.moves.begin(), tp.moves.end(), [](const auto& m) {
            return m.feedClass == FeedClass::Plunge;
        });
        if (entry == EntryKind::Plunge) {
            CHECK(ramps == 0);
            CHECK(plunges >= 1);
        } else {
            CHECK(ramps > 0);
        }
        // Ramp slope never exceeds the ramp angle.
        Vec3 pos = tp.start;
        for (const auto& m : tp.moves) {
            if (m.feedClass == FeedClass::Ramp) {
                double run = (m.end - pos).head<2>().norm();
                if (m.isArc()) {
                    run = (pos.head<2>() - m.center.head<2>()).norm() *
                          std::abs(toolpath::arcSweep(pos, m));
                }
                CHECK(pos.z() - m.end.z() <= run * std::tan(degToRad(3.0)) + 1e-9);
            }
            pos = m.end;
        }
    }
}

TEST_CASE("pocket: tool too large and conventional direction", "[strategies][pocket]") {
    Harness big(tooling::makeFlatEndMill(1, 50, 60, 100));
    const auto tp =
        pocket(geom2d::Regions{rect(0, 0, 30, 30)}, 0, -2, PocketParams{}, big.context());
    CHECK(tp.moves.empty());
    CHECK(tp.warnings.size() == 1);

    Harness h(tooling::makeFlatEndMill(1, 6, 15, 50));
    PocketParams conventional;
    conventional.direction = CutDirection::Conventional;
    conventional.entry = EntryKind::Plunge;
    const auto conv = pocket(geom2d::Regions{rect(0, 0, 30, 30)}, 0, -2, conventional, h.context());
    // Outer loop is the last loop cut: clockwise when conventional.
    double signedArea = 0.0;
    const auto paths = cutPathsAt(conv, -2.0);
    const auto& last = paths.back();
    for (std::size_t i = 0; i + 1 < last.size(); ++i) {
        signedArea += last[i].x() * last[i + 1].y() - last[i + 1].x() * last[i].y();
    }
    CHECK(signedArea < 0.0);
}

TEST_CASE("pocket honours cancellation", "[strategies][pocket]") {
    Harness h(tooling::makeFlatEndMill(1, 6, 15, 50));
    h.cancel.requestCancel();
    CHECK_THROWS_AS(pocket(geom2d::Regions{rect(0, 0, 30, 30)}, 0, -2, PocketParams{}, h.context()),
                    OperationCancelled);
}

TEST_CASE("contour: outside profile with leads", "[strategies][contour]") {
    Harness h(tooling::makeFlatEndMill(1, 10, 25, 70));
    const geom2d::Regions part{rect(0, 0, 50, 30)};
    ContourParams p;
    p.stepDown = 5.0;
    p.bottomOffset = 0.5;
    const auto tp = contour(part, 0.0, -10.0, p, h.context());
    CHECK(tp.warnings.empty());
    CHECK(tp.compensation == toolpath::Compensation::None);

    const geom2d::ClipOptions clip{0.002};
    const auto inner = geom2d::offset(part, 5.0 - 0.01, clip);
    const auto outer = geom2d::offset(part, 5.0 + 0.01, clip);
    Vec3 pos = tp.start;
    for (const auto& m : tp.moves) {
        if (m.kind != MoveKind::Rapid) {
            const auto pts =
                m.isArc() ? toolpath::sampleArc(pos, m, 0.002) : std::vector<Vec3>{pos, m.end};
            for (const Vec3& q : pts) {
                CHECK_FALSE(inner[0].contains(q.head<2>())); // never gouges
                if (m.feedClass == FeedClass::Cut) {
                    CHECK(outer[0].contains(q.head<2>())); // stays on the offset contour
                }
            }
        }
        pos = m.end;
    }
    const auto levels = depthLevels(0.0, -10.5, 5.0);
    CHECK(levels.size() == 3);
    CHECK(std::count_if(tp.moves.begin(), tp.moves.end(), [](const auto& m) {
              return m.feedClass == FeedClass::LeadIn && m.isArc();
          }) == 3);
    // Climb outside: clockwise.
    const auto paths = cutPathsAt(tp, -10.5);
    double signedArea = 0.0;
    for (const auto& path : paths) {
        for (std::size_t i = 0; i + 1 < path.size(); ++i) {
            signedArea += path[i].x() * path[i + 1].y() - path[i + 1].x() * path[i].y();
        }
    }
    CHECK(signedArea < 0.0);
    checkGolden("contour_rect_outside", tp);

    // A spring pass repeats the final loop at full depth only.
    p.springPass = true;
    const auto spring = contour(part, 0.0, -10.0, p, h.context());
    CHECK(std::count_if(spring.moves.begin(), spring.moves.end(), [](const auto& m) {
              return m.feedClass == FeedClass::LeadIn && m.isArc();
          }) == 4);
    CHECK(cutPathsAt(spring, -10.5).size() == 2 * paths.size());
    CHECK(cutPathsAt(spring, -5.0).size() == cutPathsAt(tp, -5.0).size());
}

TEST_CASE("contour: inside, multiple passes, control compensation", "[strategies][contour]") {
    Harness h(tooling::makeFlatEndMill(1, 6, 20, 60));
    const geom2d::Regions pocketShape{rect(0, 0, 40, 25)};
    ContourParams p;
    p.side = ContourSide::Inside;
    p.passes = 2;
    p.passSpacing = 0.5;
    const auto tp = contour(pocketShape, 0.0, -3.0, p, h.context());
    const auto paths = cutPathsAt(tp, -3.0);
    REQUIRE(paths.size() >= 2);
    double signedArea = 0.0;
    for (std::size_t i = 0; i + 1 < paths.front().size(); ++i) {
        signedArea += paths.front()[i].x() * paths.front()[i + 1].y() -
                      paths.front()[i + 1].x() * paths.front()[i].y();
    }
    CHECK(signedArea > 0.0); // climb inside: counter-clockwise

    p.passes = 1;
    p.compensation = CompensationMode::Control;
    const auto ctrl = contour(pocketShape, 0.0, -3.0, p, h.context());
    CHECK(ctrl.compensation == toolpath::Compensation::Left);
    // Nominal path: on the pocket wall itself.
    const auto ctrlPaths = cutPathsAt(ctrl, -3.0);
    double maxX = 0.0;
    for (const auto& path : ctrlPaths) {
        for (const Vec2& q : path) {
            maxX = std::max(maxX, q.x());
        }
    }
    CHECK_THAT(maxX, WithinAbs(40.0, 0.01));
}

TEST_CASE("contour tabs: evenly spaced, off corners and the start", "[strategies][contour][tabs]") {
    ContourTabs tabs;
    tabs.enabled = true;
    tabs.count = 4;
    // A circle has no corners: the tabs sit exactly a quarter of the way round from each other,
    // the first half a spacing from the start.
    const geom2d::Polyline circle = geom2d::Polyline::circle({0.0, 0.0}, 20.0);
    const double circumference = circle.length();
    const auto even = tabCentres(circle, tabs, 4.0, 5.0);
    REQUIRE(even.size() == 4);
    for (std::size_t k = 0; k < even.size(); ++k) {
        CHECK_THAT(even[k], WithinAbs((k + 0.5) * circumference / 4.0, 1e-9));
    }

    // A 100 x 40 rectangle starting at a corner: corners at 0, 100, 140, 240 and 280 along it.
    const geom2d::Polyline box = geom2d::Polyline::rectangle({0.0, 0.0}, {100.0, 40.0});
    const double halfSpan = 6.0;
    const auto centres = tabCentres(box, tabs, halfSpan, 2.0);
    REQUIRE(centres.size() == 4);
    const double pitch = box.length() / 4.0;
    for (std::size_t k = 0; k < centres.size(); ++k) {
        INFO("tab " << k << " at " << centres[k]);
        CHECK(std::abs(centres[k] - (k + 0.5) * pitch) <= 0.4 * pitch + 1e-9);
        for (const double corner : {0.0, 100.0, 140.0, 240.0, 280.0}) {
            CHECK(std::abs(centres[k] - corner) >= halfSpan);
        }
    }
    CHECK_THAT(centres[0], WithinAbs(35.0, 1e-9)); // already clear of the corners: not moved
    CHECK(centres[1] >= 106.0);                    // moved off the corner at 100
    CHECK(centres[1] < 107.0);                     // but no farther than it needs

    // By spacing: the length shared out evenly, about one tab per spacing, at least one.
    tabs.spacing = 60.0;
    CHECK(tabCentres(box, tabs, halfSpan, 2.0).size() == 5); // 280 / 60 = 4.7
    tabs.spacing = 1000.0;
    CHECK(tabCentres(box, tabs, halfSpan, 2.0).size() == 1);
    tabs.enabled = false;
    CHECK(tabCentres(box, tabs, halfSpan, 2.0).empty());
}

namespace {

struct TabRun {
    Vec2 centre = Vec2::Zero();
    Vec2 along = Vec2::UnitX(); ///< Unit direction of travel over the tab.
    double length = 0.0;
};

/// The stretches the tool travels at height `top` (over the tabs), in order.
std::vector<TabRun> runsAt(const toolpath::Toolpath& tp, double top) {
    std::vector<TabRun> runs;
    Vec3 pos = tp.start;
    bool inRun = false;
    Vec2 from = Vec2::Zero();
    for (const auto& m : tp.moves) {
        const bool level = m.kind != MoveKind::Rapid && std::abs(m.end.z() - top) < 1e-9 &&
                           std::abs(pos.z() - top) < 1e-9;
        if (level && !inRun) {
            inRun = true;
            from = pos.head<2>();
            runs.push_back({});
        }
        if (level) {
            runs.back().length += (m.end.head<2>() - pos.head<2>()).norm();
        } else if (inRun) {
            inRun = false;
            const Vec2 to = pos.head<2>();
            runs.back().centre = 0.5 * (from + to);
            runs.back().along = (to - from).normalized();
        }
        pos = m.end;
    }
    return runs;
}

/// Checks that no feed move below the tabs' top cuts into a tab: a tab is `width` long along the
/// path at its narrowest, stands `height` on `floorZ` (square or a triangle) and fills the slot.
void checkTabsUncut(const toolpath::Toolpath& tp, const std::vector<TabRun>& runs, double r,
                    double width, double floorZ, double height, bool triangle) {
    const double top = floorZ + height;
    const auto tabHeight = [&](double u) { // height of the tab's top at u along it
        if (std::abs(u) >= 0.5 * width) {
            return -1e300;
        }
        return triangle ? floorZ + height * (1.0 - std::abs(u) / (0.5 * width)) : top;
    };
    Vec3 pos = tp.start;
    for (const auto& m : tp.moves) {
        if (m.kind != MoveKind::Rapid) {
            const auto pts =
                m.isArc() ? toolpath::sampleArc(pos, m, 0.002) : std::vector<Vec3>{pos, m.end};
            for (const Vec3& q : pts) {
                if (q.z() >= top - 1e-9) {
                    continue;
                }
                for (const TabRun& run : runs) {
                    const Vec2 rel = q.head<2>() - run.centre;
                    const double u = rel.dot(run.along);
                    const double lateral =
                        std::abs(rel.x() * run.along.y() - rel.y() * run.along.x());
                    if (lateral > r) {
                        continue; // the tool stays out of the tab's slot
                    }
                    // The highest the tab stands under the tool's flat bottom.
                    const double lo = u - r + 1e-6;
                    const double hi = u + r - 1e-6;
                    const double highest = lo <= 0.0 && hi >= 0.0
                                               ? tabHeight(0.0)
                                               : tabHeight(std::abs(lo) < std::abs(hi) ? lo : hi);
                    INFO("tool at " << q.transpose() << ", tab at " << run.centre.transpose());
                    CHECK(q.z() >= highest - 1e-6);
                }
            }
        }
        pos = m.end;
    }
}

} // namespace

TEST_CASE("contour tabs: the tool goes over them on the lowest levels",
          "[strategies][contour][tabs]") {
    Harness h(tooling::makeFlatEndMill(1, 10, 25, 70));
    const double r = 5.0;
    const geom2d::Regions part{rect(0, 0, 50, 30)};
    ContourParams p;
    p.stepDown = 4.0; // levels -3.33, -6.67 and -10
    p.tabs.enabled = true;
    p.tabs.count = 3;
    p.tabs.width = 4.0;
    p.tabs.height = 3.0; // top at -7: only the last level goes over them

    SECTION("square") {
        const auto tp = contour(part, 0.0, -10.0, p, h.context());
        CHECK(tp.warnings.empty());
        const auto runs = runsAt(tp, -7.0);
        REQUIRE(runs.size() == 3);
        for (const TabRun& run : runs) {
            // Lifted over the width plus the tool diameter, on a straight side clear of corners.
            CHECK_THAT(run.length, WithinAbs(4.0 + 2.0 * r, 1e-6));
            CHECK((run.centre.x() < -r + 1e-6 || run.centre.x() > 50.0 + r - 1e-6 ||
                   run.centre.y() < -r + 1e-6 || run.centre.y() > 30.0 + r - 1e-6));
        }
        // Straight up and straight down, at the cut feed: a tab never changes a feed.
        int ups = 0;
        Vec3 pos = tp.start;
        for (const auto& m : tp.moves) {
            const bool vertical = (m.end.head<2>() - pos.head<2>()).norm() < 1e-9;
            if (m.kind == MoveKind::Linear && vertical && pos.z() < -9.999 &&
                std::abs(m.end.z() + 7.0) < 1e-9) {
                ++ups;
            }
            if (m.kind != MoveKind::Rapid && std::abs(m.end.z() + 7.0) < 1e-9) {
                CHECK(m.feedClass == FeedClass::Cut);
                CHECK(m.feedRate == h.feeds.cut);
            }
            pos = m.end;
        }
        CHECK(ups == 3);
        // The levels above the tabs' top are whole loops at their height.
        for (const auto& m : tp.moves) {
            CHECK_FALSE((m.kind != MoveKind::Rapid && m.end.z() > -7.0 + 1e-9 &&
                         m.end.z() < -20.0 / 3.0 - 1e-6));
        }
        checkTabsUncut(tp, runs, r, 4.0, -10.0, 3.0, false);
        // Without tabs the same contour has no lifts.
        p.tabs.enabled = false;
        CHECK(runsAt(contour(part, 0.0, -10.0, p, h.context()), -7.0).empty());
    }
    SECTION("triangular") {
        p.tabs.shape = TabShape::Triangle;
        const auto tp = contour(part, 0.0, -10.0, p, h.context());
        const auto runs = runsAt(tp, -7.0);
        REQUIRE(runs.size() == 3);
        int ramps = 0;
        Vec3 pos = tp.start;
        for (const auto& m : tp.moves) {
            const double run = (m.end.head<2>() - pos.head<2>()).norm();
            const double rise = std::abs(m.end.z() - pos.z());
            if (m.kind != MoveKind::Rapid && pos.z() < -6.999 && m.end.z() < -6.999 &&
                rise > 1e-9) {
                REQUIRE(run > 1e-9); // no straight steps
                CHECK_THAT(rise / run,
                           WithinAbs(3.0 / 2.0, 1e-6)); // the height over half the width
                CHECK(m.feedRate == h.feeds.cut);
                ++ramps;
            }
            pos = m.end;
        }
        CHECK(ramps == 6);
        for (const TabRun& run : runs) {
            CHECK_THAT(run.length, WithinAbs(2.0 * r, 1e-6)); // flat over the tool's width
        }
        checkTabsUncut(tp, runs, r, 4.0, -10.0, 3.0, true);
    }
    SECTION("every radial pass and the spring pass leave the same tabs") {
        p.passes = 2;
        p.passSpacing = 2.0;
        p.springPass = true;
        const auto tp = contour(part, 0.0, -10.0, p, h.context());
        const auto runs = runsAt(tp, -7.0);
        REQUIRE(runs.size() == 9); // 3 tabs on the outer pass, the final pass and its repeat
        for (std::size_t k = 0; k < 3; ++k) {
            const Vec2 offset = runs[k + 3].centre - runs[k].centre;
            CHECK_THAT(offset.norm(), WithinAbs(2.0, 1e-3));                // straight across
            CHECK(std::abs(offset.dot(runs[k].along)) < 1e-3);              // not along
            CHECK((runs[k + 6].centre - runs[k + 3].centre).norm() < 1e-9); // the spring pass
        }
        checkTabsUncut(tp, {runs.begin() + 3, runs.begin() + 6}, r, 4.0, -10.0, 3.0, false);
    }
    SECTION("inside, with compensation in control") {
        p.side = ContourSide::Inside;
        p.compensation = CompensationMode::Control;
        const geom2d::Regions pocketShape{rect(0, 0, 80, 60)};
        const auto tp = contour(pocketShape, 0.0, -10.0, p, h.context());
        const auto runs = runsAt(tp, -7.0);
        REQUIRE(runs.size() == 3);
        for (const TabRun& run : runs) {
            CHECK_THAT(run.length, WithinAbs(4.0 + 2.0 * r, 1e-6)); // along the programmed wall
        }
    }
}

TEST_CASE("contour tabs: parameters", "[strategies][contour][tabs][params]") {
    const StrategyInfo* info = findStrategy("contour2d");
    REQUIRE(info != nullptr);
    const auto applies = [&](const char* key, const Json& values) {
        const ParamDef* def = info->find(key);
        REQUIRE(def != nullptr);
        return info->applies(*def, values);
    };
    // The tab settings show only with tabs on; the count or the spacing, as placed.
    for (const char* key :
         {"tab_placement", "tab_count", "tab_spacing", "tab_width", "tab_height", "tab_shape"}) {
        INFO(key);
        CHECK_FALSE(applies(key, Json::object()));
        CHECK_FALSE(applies(key, Json{{"tabs", false}, {"tab_placement", "spacing"}}));
    }
    CHECK(applies("tabs", Json::object()));
    CHECK(applies("tab_count", Json{{"tabs", true}}));
    CHECK_FALSE(applies("tab_spacing", Json{{"tabs", true}}));
    CHECK(applies("tab_spacing", Json{{"tabs", true}, {"tab_placement", "spacing"}}));
    CHECK_FALSE(applies("tab_count", Json{{"tabs", true}, {"tab_placement", "spacing"}}));
    CHECK(applies("tab_shape", Json{{"tabs", true}}));

    const tooling::Tool tool = tooling::makeFlatEndMill(1, 10, 25, 70);
    tooling::CuttingPreset preset;
    preset.axialDepth = 3.0;
    CHECK_FALSE(contourParams(Json::object(), tool, &preset).tabs.enabled);
    const ContourParams on = contourParams(
        Json{{"tabs", true}, {"tab_width", 6.0}, {"tab_height", 1.5}, {"tab_shape", "triangle"}},
        tool, &preset);
    CHECK(on.tabs.enabled);
    CHECK(on.tabs.count == 4); // the ParamDef's default
    CHECK(on.tabs.spacing == 0.0);
    CHECK(on.tabs.width == 6.0);
    CHECK(on.tabs.height == 1.5);
    CHECK(on.tabs.shape == TabShape::Triangle);
    const ContourParams spaced =
        contourParams(Json{{"tabs", true}, {"tab_placement", "spacing"}}, tool, &preset);
    CHECK(spaced.tabs.spacing == 100.0);
    CHECK(spaced.tabs.count == 0);
}

TEST_CASE("drilling: ordering, cycles and depths", "[strategies][drill]") {
    Harness h(tooling::makeDrill(1, 8.0, degToRad(118.0), 60.0, 100.0));
    std::vector<HoleTarget> holes{{{50, 0}, 0.0, 10.0, false},
                                  {{10, 0}, 0.0, 10.0, true},
                                  {{30, 0}, 0.0, 40.0, false},
                                  {{30, 30}, 5.0, 6.0, false}};
    const auto tp = drill(holes, DrillParams{}, h.context());
    const double tip = 4.0 / std::tan(degToRad(59.0));

    std::vector<Vec2> visited;
    for (const auto& m : tp.moves) {
        if (m.kind == MoveKind::Cycle) {
            visited.push_back(m.end.head<2>());
        }
    }
    REQUIRE(visited.size() == 4);
    CHECK(visited[0] == Vec2(10, 0)); // nearest to the WCS origin first

    auto cycleAt = [&](const Vec2& xy) {
        toolpath::DrillCycle found;
        for (const auto& m : tp.moves) {
            if (m.kind == MoveKind::Cycle && m.end.head<2>() == xy) {
                found = tp.cycles.at(static_cast<std::size_t>(m.cycle));
            }
        }
        return found;
    };
    CHECK_THAT(cycleAt({50, 0}).bottomZ, WithinAbs(-10.0 - tip, 1e-9));
    CHECK_THAT(cycleAt({10, 0}).bottomZ,
               WithinAbs(-10.0 - tip - 1.0, 1e-9));            // through: breakthrough
    CHECK(cycleAt({30, 0}).type == toolpath::CycleType::Peck); // deeper than 3 D
    CHECK_THAT(cycleAt({30, 0}).peck, WithinAbs(8.0, 1e-12));
    CHECK(cycleAt({50, 0}).type == toolpath::CycleType::Drill);
    CHECK_THAT(cycleAt({30, 30}).retractZ, WithinAbs(7.0, 1e-12));
    // The hole with a different top is reached at clearance height.
    CHECK(std::any_of(tp.moves.begin(), tp.moves.end(), [&](const toolpath::Move& m) {
        return m.kind == MoveKind::Rapid && m.end.z() == h.heights.clearance &&
               m.end.head<2>() != Vec2(30, 30);
    }));
    checkGolden("drill_mixed", tp);

    DrillParams tipMode;
    tipMode.depthMode = DepthMode::Tip;
    tipMode.cycle = toolpath::CycleType::ChipBreak;
    const auto tp2 = drill({holes[0]}, tipMode, h.context());
    CHECK_THAT(tp2.cycles.at(0).bottomZ, WithinAbs(-10.0, 1e-9));
    CHECK(tp2.cycles.at(0).type == toolpath::CycleType::ChipBreak);

    Harness tap(tooling::makeDefaultTool(tooling::ToolType::Tap, 2, 10.0)); // pitch 1.5
    const auto tapped = drill({holes[0]}, DrillParams{}, tap.context());
    CHECK(tapped.cycles.at(0).type == toolpath::CycleType::Tap);
    CHECK_THAT(tapped.cycles.at(0).feedRate, WithinRel(1.5 * tap.feeds.spindle, 1e-12));

    CHECK(drill({}, DrillParams{}, h.context()).warnings.size() == 1);
}

TEST_CASE("parameters show only where they do something", "[strategies][params]") {
    const StrategyInfo* rough = findStrategy("zlevel_rough");
    REQUIRE(rough != nullptr);
    const auto applies = [&](const char* key, const Json& values) {
        const ParamDef* def = rough->find(key);
        REQUIRE(def != nullptr);
        return rough->applies(*def, values);
    };
    // The ramp angle matters to ramp and helix entries, the helix radius to helixes only.
    CHECK(applies("ramp_angle", Json::object())); // default entry: helix
    CHECK(applies("ramp_angle", Json{{"entry", "ramp"}}));
    CHECK_FALSE(applies("ramp_angle", Json{{"entry", "plunge"}}));
    CHECK_FALSE(applies("helix_radius", Json{{"entry", "ramp"}}));
    // Rest settings with rest machining only; smoothing tolerance with smoothing only.
    CHECK_FALSE(applies("rest_resolution", Json::object()));
    CHECK(applies("rest_resolution", Json{{"rest_machining", true}}));
    CHECK(applies("smoothing_tolerance", Json::object())); // smoothing on by default
    CHECK_FALSE(applies("smoothing_tolerance", Json{{"smoothing", false}}));
    CHECK(applies("entry", Json{{"entry", "plunge"}})); // unconditional

    // Smoothing on every strategy that does not arc by itself; geometry where it is used.
    for (const char* id : {"pocket2d", "contour2d", "chamfer2d", "zlevel_rough", "parallel3d",
                           "waterline3d", "contour3d", "flat3d", "pencil3d", "steep_shallow3d"}) {
        INFO(id);
        REQUIRE(findStrategy(id) != nullptr);
        CHECK(findStrategy(id)->find("smoothing") != nullptr);
    }
    for (const char* id : {"bore", "threadmill", "drill", "tap"}) {
        INFO(id);
        CHECK(findStrategy(id)->find("smoothing") == nullptr);
    }
    CHECK(findStrategy("facing")->geometry == GeometryUse::None);
    CHECK(findStrategy("zlevel_rough")->geometry == GeometryUse::None);
    CHECK(findStrategy("contour2d")->geometry == GeometryUse::FacesOrEdges);
    CHECK(findStrategy("chamfer2d")->geometry == GeometryUse::Edges);
    CHECK(findStrategy("rotary_contour")->geometry == GeometryUse::FacesOrEdges);
    CHECK(findStrategy("rotary_contour")->find("live_axes") == nullptr); // it wraps
    CHECK(findStrategy("contour3d")->geometry == GeometryUse::Containment);
    CHECK(findStrategy("contour3d")->find("rest_machining") != nullptr);
    for (const char* id : {"pencil3d", "steep_shallow3d"}) {
        INFO(id);
        CHECK(findStrategy(id)->geometry == GeometryUse::Containment);
        CHECK(findStrategy(id)->find("rest_machining") != nullptr);
    }
    // The pencil's reference tool is the operation's own until set.
    CHECK(findStrategy("pencil3d")->find("reference_diameter")->toolDefault);
    CHECK(findStrategy("drill")->geometry == GeometryUse::Holes);
    CHECK(findStrategy("chamfer2d")->find("side") == nullptr); // each outline's own
    CHECK(findStrategy("contour2d")->find("side") == nullptr);

    // Sizes are lengths, never multiples of the tool; unset, they follow the tool.
    const ParamDef* helix = findStrategy("pocket2d")->find("helix_radius");
    REQUIRE(helix != nullptr);
    CHECK(helix->kind == ParamKind::Length);
    REQUIRE(helix->toolDefault);
    CHECK_THAT(helix->toolDefault(tooling::makeFlatEndMill(1, 10.0, 20.0, 50.0)),
               WithinAbs(4.0, 1e-12));
    for (const StrategyInfo& info : ocam::strategies::strategies()) {
        for (const ParamDef& def : info.params) {
            INFO(info.id << " " << def.key);
            CHECK(def.label.find("×") == std::string::npos);
            CHECK(def.key.find("_ratio") == std::string::npos);
        }
    }
}
