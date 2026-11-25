// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "StrategyTestUtil.hpp"

#include <ocam/strategies/Avoidance.hpp>
#include <ocam/strategies/Strategies.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>

using namespace ocam;
using namespace ocam::strategies;
using namespace ocam::strategies::testing;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace {

geom3d::Mesh boxMesh(const Vec3& lo, const Vec3& hi) {
    geom3d::Mesh m;
    for (int i = 0; i < 8; ++i) {
        m.vertices.emplace_back((i & 1) ? hi.x() : lo.x(), (i & 2) ? hi.y() : lo.y(),
                                (i & 4) ? hi.z() : lo.z());
    }
    m.triangles = {{0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
                   {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5}};
    return m;
}

/// XY distance from p to the rectangle [lo, hi].
double distanceToRect(const Vec2& p, const Vec2& lo, const Vec2& hi) {
    const double dx = std::max({lo.x() - p.x(), 0.0, p.x() - hi.x()});
    const double dy = std::max({lo.y() - p.y(), 0.0, p.y() - hi.y()});
    return std::hypot(dx, dy);
}

tooling::Holder testHolder() {
    tooling::HolderDimensions d;
    d.noseDiameter = 30.0;
    d.noseLength = 40.0;
    d.bodyDiameter = 0.0;
    d.flangeDiameter = 60.0;
    d.flangeLength = 10.0;
    return tooling::makeHolder("test chuck", d);
}

std::vector<Vec3> allPoints(const toolpath::Toolpath& tp) {
    std::vector<Vec3> pts{tp.start};
    for (const auto& run : toolpath::sampleRuns(tp, 0.01)) {
        pts.insert(pts.end(), run.points.begin(), run.points.end());
    }
    return pts;
}

/// Points of every move, rapids included, densely resampled.
std::vector<Vec3> densePoints(const toolpath::Toolpath& tp) {
    const std::vector<Vec3> pts = allPoints(tp);
    std::vector<Vec3> out;
    for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
        const int n = std::max(1, static_cast<int>((pts[k + 1] - pts[k]).norm() / 0.05));
        for (int i = 0; i < n; ++i) {
            out.push_back(pts[k] + (pts[k + 1] - pts[k]) * (static_cast<double>(i) / n));
        }
    }
    out.push_back(pts.back());
    return out;
}

} // namespace

TEST_CASE("obstacle field: safe heights of the cutter and the holder", "[strategies][avoid]") {
    const tooling::Tool tool = tooling::makeFlatEndMill(1, 10.0, 22.0, 72.0);
    const geom3d::Mesh clamp = boxMesh({45, 20, -20}, {55, 40, 10});
    const ObstacleField field(clamp, tool, nullptr, 1.0);
    CHECK(field.pieceCount() == 1);
    CHECK(field.safeHeight({50, 30}) == Approx(11.0));
    CHECK(field.safeHeight({50, 30 + 10 + 5.5}) > 10.0); // the rounded clearance corner
    CHECK(field.safeHeight({50, 30 + 10 + 6.5}) == -std::numeric_limits<double>::infinity());
    CHECK(field.ceiling() == Approx(11.0));
    CHECK(field.reach() == Approx(6.0));

    // With a holder 30 mm above the tip: a nose of radius 15, a flange of 30 at 70 to 80.
    tooling::Tool held = tool;
    held.stickout = 30.0;
    const tooling::Holder holder = testHolder();
    const ObstacleField withHolder(clamp, held, &holder, 1.0);
    CHECK(withHolder.pieceCount() == 3);
    CHECK(withHolder.reach() == Approx(31.0));
    // 12 mm beside the clamp the cutter is clear, the nose (from 30 mm up) is not.
    CHECK(withHolder.safeHeight({50, 40 + 12}) == Approx(10.0 - 30.0 + 1.0).margin(1e-6));
    // 25 mm beside it only the flange reaches: its bottom is 70 mm above the tip.
    CHECK(withHolder.safeHeight({50, 40 + 25}) == Approx(10.0 - 70.0 + 1.0).margin(1e-6));
}

TEST_CASE("avoidance: facing works around a clamp", "[strategies][avoid]") {
    Harness h(tooling::makeFlatEndMill(1, 10.0, 22.0, 72.0));
    FacingParams p;
    p.stepoverRatio = 0.6;
    toolpath::Toolpath tp = facing({rect(0, 0, 100, 60)}, 0.0, -1.0, p, h.context());
    const Vec2 lo(45, 20), hi(55, 40);
    const double top = 10.0;
    const ObstacleField field(boxMesh({lo.x(), lo.y(), -20}, {hi.x(), hi.y(), top}), h.tool,
                              nullptr, 1.0);
    AvoidanceParams ap;
    ap.retract = h.heights.retract;
    ap.plungeFeed = h.feeds.plunge;
    ap.clearance = 1.0;
    const AvoidanceResult r = avoidObstacles(tp, field, ap, h.cancel);
    CHECK(r.trimmedSections > 0);
    CHECK(r.trimmedLength > 0.0);
    CHECK(r.changed());
    REQUIRE_FALSE(tp.warnings.empty());
    CHECK_THAT(tp.warnings.back(), ContainsSubstring("fixtures"));

    // No position of the tool, rapids included, comes within the clearance.
    for (const Vec3& q : densePoints(tp)) {
        const double d = distanceToRect(q.head<2>(), lo, hi);
        if (d < 5.0) {
            CHECK(q.z() >= top + 1.0 - 1e-3);
        } else if (d < 6.0 - 1e-3) {
            CHECK(q.z() > top);
        }
    }
    // The rest of the face is still cut.
    const auto cuts = feedPoints(tp);
    CHECK(std::any_of(cuts.begin(), cuts.end(),
                      [](const Vec3& q) { return q.x() > 70.0 && std::abs(q.z() + 1.0) < 1e-6; }));
    CHECK(std::any_of(cuts.begin(), cuts.end(),
                      [](const Vec3& q) { return q.x() < 30.0 && std::abs(q.z() + 1.0) < 1e-6; }));
    // Beyond the clamp's reach the passes run straight past it.
    bool pastClamp = false;
    Vec3 from = tp.start;
    for (const auto& m : tp.moves) {
        pastClamp =
            pastClamp || (m.kind == toolpath::MoveKind::Linear && from.y() > 47.0 &&
                          m.end.y() > 47.0 && std::min(from.x(), m.end.x()) < 40.0 &&
                          std::max(from.x(), m.end.x()) > 60.0 && std::abs(m.end.z() + 1.0) < 1e-6);
        from = m.end;
    }
    CHECK(pastClamp);
}

TEST_CASE("avoidance: the holder keeps clear of a tall wall", "[strategies][avoid]") {
    tooling::Tool tool = tooling::makeFlatEndMill(1, 10.0, 22.0, 72.0);
    tool.stickout = 30.0;
    const tooling::Holder holder = testHolder();
    Harness h(tool);
    FacingParams p;
    p.stepoverRatio = 0.5;
    p.angle = kPi / 2.0; // passes along Y, stepping over in X
    const toolpath::Toolpath original = facing({rect(0, 0, 70, 60)}, 0.0, -1.0, p, h.context());
    const geom3d::Mesh wall = boxMesh({75, -20, -20}, {80, 80, 40});
    AvoidanceParams ap;
    ap.retract = h.heights.retract;
    ap.clearance = 1.0;

    // Without the holder only the cutter matters: the passes next to the wall stay.
    toolpath::Toolpath bare = original;
    avoidObstacles(bare, ObstacleField(wall, tool, nullptr, 1.0), ap, h.cancel);
    auto cutsIn = [](const toolpath::Toolpath& tp, double x0, double x1) {
        const auto cuts = feedPoints(tp);
        return std::count_if(cuts.begin(), cuts.end(), [&](const Vec3& q) {
            return q.x() > x0 && q.x() < x1 && std::abs(q.z() + 1.0) < 1e-6;
        });
    };
    CHECK(cutsIn(bare, 61.0, 69.0) > 0);

    // With it the nose (radius 15 + 1, from 29 mm up while the tip is at -1) cannot come
    // closer than 16 mm to the wall, which stands to 40.
    toolpath::Toolpath held = original;
    const AvoidanceResult r =
        avoidObstacles(held, ObstacleField(wall, tool, &holder, 1.0), ap, h.cancel);
    CHECK(r.trimmedSections > 0);
    CHECK(cutsIn(held, 59.0 + 1e-3, 75.0) == 0);
    CHECK(cutsIn(held, 10.0, 55.0) > 0);
}

TEST_CASE("avoidance: rapids go over, holes under a clamp are left out", "[strategies][avoid]") {
    Harness h(tooling::makeDrill(1, 6.0, degToRad(118.0), 30.0, 70.0));
    std::vector<HoleTarget> holes{{{10, 30}, 0.0, 10.0, false, 6.0},
                                  {{50, 30}, 0.0, 10.0, false, 6.0},
                                  {{90, 30}, 0.0, 10.0, false, 6.0}};
    DrillParams dp;
    toolpath::Toolpath tp = drill(holes, dp, h.context());
    const Vec2 lo(45, 20), hi(55, 40);
    const ObstacleField field(boxMesh({lo.x(), lo.y(), -20}, {hi.x(), hi.y(), 8.0}), h.tool,
                              nullptr, 1.0);
    AvoidanceParams ap;
    ap.retract = h.heights.retract;
    const AvoidanceResult r = avoidObstacles(tp, field, ap, h.cancel);
    CHECK(r.skippedHoles == 1);
    const auto cycles = std::count_if(tp.moves.begin(), tp.moves.end(), [](const auto& m) {
        return m.kind == toolpath::MoveKind::Cycle;
    });
    CHECK(cycles == 2);
    // From the first hole to the last the tool goes over the clamp.
    for (const Vec3& q : densePoints(tp)) {
        if (distanceToRect(q.head<2>(), lo, hi) < 3.0) {
            CHECK(q.z() >= 9.0 - 1e-3);
        }
    }
    CHECK_THAT(tp.warnings.back(), ContainsSubstring("hole"));
}

TEST_CASE("avoidance: nothing changes far from the obstacles", "[strategies][avoid]") {
    Harness h(tooling::makeBallEndMill(1, 6.0, 20.0, 60.0));
    PocketParams pp;
    toolpath::Toolpath tp = pocket({rect(0, 0, 40, 30)}, 0.0, -5.0, pp, h.context());
    const toolpath::Toolpath before = tp;
    const ObstacleField field(boxMesh({100, 100, -10}, {120, 120, 30}), h.tool, nullptr, 1.0);
    const AvoidanceResult r = avoidObstacles(tp, field, {}, h.cancel);
    CHECK_FALSE(r.changed());
    CHECK(tp == before);
}
