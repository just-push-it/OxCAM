// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// 2D trace along open and closed chains of edges (DL-0088).

#include "StrategyTestUtil.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/geom2d/PathOps.hpp>
#include <ocam/toolpath/PathBuilder.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <set>

using namespace ocam;
using namespace ocam::strategies;
using namespace ocam::strategies::testing;
using Catch::Matchers::WithinAbs;
using toolpath::FeedClass;
using toolpath::MoveKind;

namespace {

geom2d::Polyline open(const std::vector<Vec2>& pts) {
    return geom2d::Polyline::fromPoints(pts, false);
}

/// An L: 20 mm along X, then 10 mm up Y.
geom2d::Polyline ell() {
    return open({{0, 0}, {20, 0}, {20, 10}});
}

/// Every move's end in cutting order, with its kind and feed class.
std::vector<const toolpath::Move*> moves(const toolpath::Toolpath& tp, FeedClass fc) {
    std::vector<const toolpath::Move*> out;
    for (const auto& m : tp.moves) {
        if (m.kind != MoveKind::Rapid && m.feedClass == fc) {
            out.push_back(&m);
        }
    }
    return out;
}

/// The heights of the cut moves, in order, without repeats.
std::vector<double> cutLevels(const toolpath::Toolpath& tp) {
    std::vector<double> z;
    for (const auto* m : moves(tp, FeedClass::Cut)) {
        const double h = std::round(m->end.z() * 1e6) / 1e6;
        if (z.empty() || z.back() != h) {
            z.push_back(h);
        }
    }
    return z;
}

/// The first point the tool cuts from (where the first plunge or ramp comes down).
Vec2 firstCut(const toolpath::Toolpath& tp) {
    for (const auto& m : tp.moves) {
        if (m.kind != MoveKind::Rapid) {
            return m.end.head<2>();
        }
    }
    return Vec2::Constant(1e9);
}

/// The nearest and farthest distance of the sampled cut moves from `line`.
std::pair<double, double> distanceRange(const toolpath::Toolpath& tp,
                                        const geom2d::Polyline& line) {
    double lo = 1e9;
    double hi = 0.0;
    for (const auto& run : toolpath::sampleRuns(tp, 0.001)) {
        if (run.feedClass != FeedClass::Cut) {
            continue;
        }
        for (const Vec3& q : run.points) {
            const double d = geom2d::closestPoint(line, q.head<2>()).distance;
            lo = std::min(lo, d);
            hi = std::max(hi, d);
        }
    }
    return {lo, hi};
}

} // namespace

TEST_CASE("a trace runs the tool centre along an open chain", "[strategies][trace]") {
    Harness h(tooling::makeFlatEndMill(1, 2.0, 10.0, 50.0));
    TraceParams p;
    p.depth = 0.5;
    const auto tp = trace({{ell(), 0.0}}, p, h.context());
    REQUIRE(tp.warnings.empty());
    CHECK(tp.strategy == "trace2d");
    SECTION("on the line, at the depth below it, from its end with the lower X") {
        CHECK(cutLevels(tp) == std::vector<double>{-0.5});
        const auto [lo, hi] = distanceRange(tp, ell());
        CHECK(hi < 1e-6);
        CHECK((firstCut(tp) - Vec2(0, 0)).norm() < 1e-9);
        // It runs to the other end, not back round: an open chain.
        CHECK((moves(tp, FeedClass::Cut).back()->end.head<2>() - Vec2(20, 10)).norm() < 1e-9);
    }
    SECTION("feeds are the operation's: plunge at the plunge feed, the cut at the cut feed") {
        const auto plunges = moves(tp, FeedClass::Plunge);
        REQUIRE(plunges.size() == 1);
        CHECK(plunges.front()->feedRate == h.feeds.plunge);
        for (const auto* m : moves(tp, FeedClass::Cut)) {
            CHECK(m->feedRate == h.feeds.cut);
        }
        CHECK(moves(tp, FeedClass::Ramp).empty());
    }
    SECTION("reversed, it starts from the other end") {
        p.reverse = true;
        CHECK((firstCut(trace({{ell(), 0.0}}, p, h.context())) - Vec2(20, 10)).norm() < 1e-9);
    }
    SECTION("a start point picks the end") {
        p.start = Vec2(21, 11);
        CHECK((firstCut(trace({{ell(), 0.0}}, p, h.context())) - Vec2(20, 10)).norm() < 1e-9);
        p.reverse = true; // and reversing still goes the other way
        CHECK((firstCut(trace({{ell(), 0.0}}, p, h.context())) - Vec2(0, 0)).norm() < 1e-9);
    }
    SECTION("the chain's height sets the depth") {
        CHECK(cutLevels(trace({{ell(), 7.0}}, p, h.context())) == std::vector<double>{6.5});
    }
    SECTION("depth 0 runs on the line's height") {
        p.depth = 0.0;
        CHECK(cutLevels(trace({{ell(), 2.0}}, p, h.context())) == std::vector<double>{2.0});
    }
}

TEST_CASE("a trace steps down to its depth", "[strategies][trace]") {
    Harness h(tooling::makeFlatEndMill(1, 2.0, 10.0, 50.0));
    TraceParams p;
    p.depth = 1.0;
    p.stepDown = 0.3; // four equal levels, none deeper than the step
    SECTION("an open chain: each level from its start, retracting between") {
        const auto tp = trace({{ell(), 0.0}}, p, h.context());
        CHECK(cutLevels(tp) == std::vector<double>{-0.25, -0.5, -0.75, -1.0});
        CHECK(moves(tp, FeedClass::Plunge).size() == 4);
        // Every level starts at the chain's start.
        for (const auto* m : moves(tp, FeedClass::Plunge)) {
            CHECK((m->end.head<2>() - Vec2(0, 0)).norm() < 1e-9);
        }
        // Each plunge comes down from just above the level before it.
        const auto plunges = moves(tp, FeedClass::Plunge);
        for (std::size_t k = 0; k < tp.moves.size(); ++k) {
            if (&tp.moves[k] == plunges[1]) {
                CHECK_THAT(tp.moves[k - 1].end.z(),
                           WithinAbs(-0.25 + h.heights.feedClearance, 1e-9));
            }
        }
    }
    SECTION("a closed chain goes on down where it ends, without leaving the cut") {
        const geom2d::Polyline square = geom2d::Polyline::rectangle({0, 0}, {10, 10});
        const auto tp = trace({{square, 0.0}}, p, h.context());
        CHECK(cutLevels(tp) == std::vector<double>{-0.25, -0.5, -0.75, -1.0});
        // After the first plunge, no rapid until it leaves at the end.
        bool cutting = false;
        std::size_t rapids = 0;
        for (const auto& m : tp.moves) {
            cutting |= m.feedClass == FeedClass::Plunge;
            rapids += cutting && m.kind == MoveKind::Rapid && m.end.z() < 0.0 ? 1 : 0;
        }
        CHECK(rapids == 0);
    }
    SECTION("no step down is one level at the full depth") {
        p.stepDown = 0.0;
        CHECK(cutLevels(trace({{ell(), 0.0}}, p, h.context())) == std::vector<double>{-1.0});
    }
}

TEST_CASE("a trace offset to a side of an open chain", "[strategies][trace]") {
    Harness h(tooling::makeFlatEndMill(1, 4.0, 10.0, 50.0)); // radius 2
    TraceParams p;
    p.depth = 0.5;
    p.offset = 0.5;
    const geom2d::Polyline line = open({{0, 0}, {20, 0}});
    SECTION("left of a line along +X is +Y; right is -Y, the tool's side the offset away") {
        p.side = TraceSide::Left;
        for (const auto* m : moves(trace({{line, 0.0}}, p, h.context()), FeedClass::Cut)) {
            CHECK_THAT(m->end.y(), WithinAbs(2.5, 1e-9));
        }
        p.side = TraceSide::Right;
        for (const auto* m : moves(trace({{line, 0.0}}, p, h.context()), FeedClass::Cut)) {
            CHECK_THAT(m->end.y(), WithinAbs(-2.5, 1e-9));
        }
    }
    SECTION("round an L: every point the same distance from it, outside corner rounded") {
        for (const TraceSide side : {TraceSide::Left, TraceSide::Right}) {
            p.side = side;
            const auto tp = trace({{ell(), 0.0}}, p, h.context());
            REQUIRE(tp.warnings.empty());
            const auto [lo, hi] = distanceRange(tp, ell());
            CHECK(lo > 2.5 - 0.01);
            CHECK(hi < 2.5 + 0.01);
        }
    }
    SECTION("inside corners: where the offsets cross, and a step shorter than the offset") {
        // A U open to -X, 6 mm across: inside it (left), 2.5 mm from each side.
        const geom2d::Polyline u = open({{0, 0}, {10, 0}, {10, 6}, {0, 6}});
        p.side = TraceSide::Left;
        for (const TraceSide side : {TraceSide::Left, TraceSide::Right}) {
            p.side = side;
            const auto tp = trace({{u, 0.0}}, p, h.context());
            CHECK(moves(tp, FeedClass::Plunge).size() == 1);
            const auto [lo, hi] = distanceRange(tp, u);
            CHECK(lo > 2.5 - 0.01);
            CHECK(hi < 2.5 + 0.01);
        }
        // A 1 mm step up: on its upper side the offsets of the two runs overlap by more than the
        // step, and the loop between them is trimmed, leaving one path.
        const geom2d::Polyline step = open({{0, 0}, {10, 0}, {10, 1}, {20, 1}});
        for (const TraceSide side : {TraceSide::Left, TraceSide::Right}) {
            p.side = side;
            const auto tp = trace({{step, 0.0}}, p, h.context());
            CHECK(moves(tp, FeedClass::Plunge).size() == 1);
            const auto [lo, hi] = distanceRange(tp, step);
            CHECK(lo > 2.5 - 0.01);
            CHECK(hi < 2.5 + 0.01);
        }
    }
    SECTION("inside a U narrower than the tool there is no room, and it says so") {
        const geom2d::Polyline u = open({{0, 0}, {10, 0}, {10, 3}, {0, 3}});
        p.side = TraceSide::Left;
        const auto tp = trace({{u, 0.0}}, p, h.context());
        CHECK(moves(tp, FeedClass::Cut).empty());
        CHECK_FALSE(tp.warnings.empty());
    }
    SECTION("side-by-side passes step in to the line, the farthest first") {
        p.side = TraceSide::Left;
        p.passes = 3;
        p.passSpacing = 1.0;
        const auto tp = trace({{line, 0.0}}, p, h.context());
        std::vector<double> ys;
        for (const auto* m : moves(tp, FeedClass::Plunge)) {
            ys.push_back(m->end.y());
        }
        CHECK(ys == std::vector<double>{4.5, 3.5, 2.5});
    }
}

TEST_CASE("a trace round a closed chain", "[strategies][trace]") {
    Harness h(tooling::makeFlatEndMill(1, 4.0, 10.0, 50.0)); // radius 2
    TraceParams p;
    p.depth = 0.2;
    geom2d::Polyline square = geom2d::Polyline::rectangle({0, 0}, {20, 20});
    square.reverse(); // given clockwise
    const auto loopOf = [](const toolpath::Toolpath& tp) {
        std::vector<Vec2> pts;
        for (const auto* m : moves(tp, FeedClass::Cut)) {
            pts.push_back(m->end.head<2>());
        }
        return pts;
    };
    SECTION("on the line it runs counter-clockwise, round the whole loop") {
        const auto tp = trace({{square, 0.0}}, p, h.context());
        const auto pts = loopOf(tp);
        REQUIRE(pts.size() >= 4);
        CHECK(geom2d::Polyline::fromPoints(pts, true).signedArea() > 0.0);
        CHECK((pts.back() - firstCut(tp)).norm() < 1e-9); // back where it started
        const auto [lo, hi] = distanceRange(tp, square);
        CHECK(hi < 1e-6);
    }
    SECTION("reversed, clockwise") {
        p.reverse = true;
        CHECK(geom2d::Polyline::fromPoints(loopOf(trace({{square, 0.0}}, p, h.context())), true)
                  .signedArea() < 0.0);
    }
    SECTION("left is inside, right outside, the tool's side the offset from the line") {
        p.offset = 0.5;
        p.side = TraceSide::Left;
        const auto inside = trace({{square, 0.0}}, p, h.context());
        for (const auto* m : moves(inside, FeedClass::Cut)) {
            CHECK(m->end.x() > 2.5 - 1e-6);
            CHECK(m->end.x() < 17.5 + 1e-6);
        }
        CHECK_THAT(distanceRange(inside, square).first, WithinAbs(2.5, 0.01));
        p.side = TraceSide::Right;
        const auto outside = trace({{square, 0.0}}, p, h.context());
        const auto [lo, hi] = distanceRange(outside, square);
        CHECK(lo > 2.5 - 0.01);
        CHECK(hi < 2.5 + 0.01);
        bool beyond = false;
        for (const auto* m : moves(outside, FeedClass::Cut)) {
            beyond |= m->end.x() < 0.0 || m->end.x() > 20.0;
        }
        CHECK(beyond);
    }
    SECTION("inside a loop too small for the tool there is nothing, and it says so") {
        p.side = TraceSide::Left;
        const geom2d::Polyline small = geom2d::Polyline::rectangle({0, 0}, {3, 3});
        const auto tp = trace({{small, 0.0}}, p, h.context());
        CHECK(tp.moves.empty());
        CHECK_FALSE(tp.warnings.empty());
    }
}

TEST_CASE("a trace ramps down along its path", "[strategies][trace]") {
    Harness h(tooling::makeFlatEndMill(1, 2.0, 10.0, 50.0));
    TraceParams p;
    p.depth = 1.0;
    p.entry = EntryKind::Ramp;
    p.rampAngle = degToRad(5.0);
    const auto check = [&](const geom2d::Polyline& chain) {
        const auto tp = trace({{chain, 0.0}}, p, h.context());
        CHECK(moves(tp, FeedClass::Plunge).empty());
        const auto ramps = moves(tp, FeedClass::Ramp);
        REQUIRE_FALSE(ramps.empty());
        Vec3 at = tp.start;
        double steepest = 0.0;
        for (const auto& m : tp.moves) {
            if (m.kind == MoveKind::Linear && m.feedClass == FeedClass::Ramp) {
                CHECK(m.feedRate == h.feeds.ramp);
                const double run = (m.end.head<2>() - at.head<2>()).norm();
                REQUIRE(run > 0.0);
                steepest = std::max(steepest, (at.z() - m.end.z()) / run);
                // On the path.
                CHECK(geom2d::closestPoint(chain, m.end.head<2>()).distance < 1e-6);
            }
            at = m.end;
        }
        CHECK(steepest <= std::tan(p.rampAngle) + 1e-9);
        // The ramp ends at depth where the cut starts.
        CHECK_THAT(ramps.back()->end.z(), WithinAbs(-1.0, 1e-9));
        CHECK(cutLevels(tp) == std::vector<double>{-1.0});
        return tp;
    };
    SECTION("an open chain long enough: forward and back once") {
        const auto tp = check(open({{0, 0}, {30, 0}}));
        CHECK((moves(tp, FeedClass::Ramp).back()->end.head<2>() - Vec2(0, 0)).norm() < 1e-9);
    }
    SECTION("a short open chain: back and forth along all of it") {
        const auto tp = check(open({{0, 0}, {2, 0}}));
        CHECK(moves(tp, FeedClass::Ramp).size() > 4);
    }
    SECTION("a closed chain: round it, the lap finished at depth") {
        check(geom2d::Polyline::rectangle({0, 0}, {5, 5}));
    }
}

TEST_CASE("a trace following a surface keeps its depth below it", "[strategies][trace]") {
    Harness h(tooling::makeBallEndMill(1, 1.0, 10.0, 50.0));
    TraceParams p;
    p.depth = 0.3;
    p.sampleStep = 0.25;
    // A slope rising 1 in 10 along X.
    const TraceSurface slope = [](const Vec2& q) {
        return 0.1 * q.x();
    };
    const auto tp = trace({{ell(), -50.0}}, p, h.context(), slope);
    const auto cuts = moves(tp, FeedClass::Cut);
    REQUIRE(cuts.size() > 40);
    for (const auto* m : cuts) {
        CHECK_THAT(m->end.z(), WithinAbs(0.1 * m->end.x() - 0.3, 1e-9));
    }
    const auto plunges = moves(tp, FeedClass::Plunge);
    REQUIRE(plunges.size() == 1);
    CHECK_THAT(plunges.front()->end.z(), WithinAbs(-0.3, 1e-9));
}

TEST_CASE("a trace refuses what it cannot do", "[strategies][trace]") {
    Harness h(tooling::makeFlatEndMill(1, 2.0, 10.0, 50.0));
    TraceParams p;
    CHECK_THROWS_AS(trace({}, p, h.context()), Error);
    p.entry = EntryKind::Helix;
    CHECK_THROWS_AS(trace({{ell(), 0.0}}, p, h.context()), Error);
}

TEST_CASE("trace parameters: what does not apply does not count", "[strategies][trace][params]") {
    tooling::CuttingPreset preset;
    preset.axialDepth = 0.4;
    preset.rampAngle = degToRad(7.0);
    SECTION("defaults: on the line, plunging, the preset's step down") {
        const TraceParams p = traceParams(Json::object(), &preset);
        CHECK(p.side == TraceSide::On);
        CHECK(p.entry == EntryKind::Plunge);
        CHECK(p.depth > 0.0);
        CHECK(p.stepDown == 0.4);
    }
    SECTION("no depth needs no step down; with one, a preset without ap is refused") {
        tooling::CuttingPreset none;
        CHECK_NOTHROW(traceParams(Json{{"depth", 0.0}}, &none));
        CHECK_THROWS_AS(traceParams(Json{{"depth", 1.0}}, &none), Error);
    }
    SECTION("an offset and passes count only beside the line") {
        const Json on{{"offset", 3.0}, {"passes", 4}};
        CHECK(traceParams(on, &preset).offset == 0.0);
        CHECK(traceParams(on, &preset).passes == 1);
        Json left = on;
        left["side"] = "left";
        CHECK(traceParams(left, &preset).offset == 3.0);
        CHECK(traceParams(left, &preset).passes == 4);
    }
    SECTION("the ramp angle is the preset's unless set") {
        CHECK_THAT(traceParams(Json{{"entry", "ramp"}}, &preset).rampAngle,
                   WithinAbs(degToRad(7.0), 1e-12));
        CHECK_THAT(traceParams(Json{{"entry", "ramp"}, {"ramp_angle", 3.0}}, &preset).rampAngle,
                   WithinAbs(degToRad(3.0), 1e-12));
    }
}
