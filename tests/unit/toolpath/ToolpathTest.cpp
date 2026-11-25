// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/toolpath/PathBuilder.hpp>
#include <ocam/toolpath/Smoothing.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

using namespace ocam;
using namespace ocam::toolpath;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("arc sweep direction and full circles", "[toolpath]") {
    Move ccw;
    ccw.kind = MoveKind::ArcCCW;
    ccw.center = Vec3::Zero();
    ccw.end = Vec3(0, 1, 0);
    CHECK_THAT(arcSweep(Vec3(1, 0, 0), ccw), WithinAbs(kPi / 2, 1e-12));

    Move cw = ccw;
    cw.kind = MoveKind::ArcCW;
    CHECK_THAT(arcSweep(Vec3(1, 0, 0), cw), WithinAbs(-3 * kPi / 2, 1e-12));

    Move full = ccw;
    full.end = Vec3(1, 0, -2);
    CHECK_THAT(arcSweep(Vec3(1, 0, 0), full), WithinAbs(2 * kPi, 1e-12));
    const auto pts = sampleArc(Vec3(1, 0, 0), full, 1e-3);
    CHECK(pts.size() > 20);
    CHECK_THAT(pts[pts.size() / 2].z(), WithinAbs(-1.0, 0.1));
}

TEST_CASE("builder drops zero-length moves and tracks position", "[toolpath]") {
    Toolpath tp;
    tp.start = Vec3(0, 0, 10);
    PathBuilder b(tp);
    b.rapid(Vec3(0, 0, 10)); // no-op
    b.rapidAbove(Vec2(5, 5), 10, 2);
    CHECK(b.position() == Vec3(5, 5, 2));
    b.feed(Vec3(5, 5, -1), FeedClass::Plunge, 5.0);
    b.retractTo(10);
    b.retractTo(5); // already above: no-op
    REQUIRE(tp.moves.size() == 4);
    CHECK(tp.moves[0].kind == MoveKind::Rapid);
    CHECK(tp.moves[2].feedClass == FeedClass::Plunge);
    CHECK(tp.moves[3].end == Vec3(5, 5, 10));
}

TEST_CASE("following polylines and stats", "[toolpath]") {
    Toolpath tp;
    tp.start = Vec3(0, 0, 5);
    PathBuilder b(tp);
    b.rapid(Vec3(0, 0, 0));
    b.follow(geom2d::Polyline::rectangle({0, 0}, {10, 10}), 0.0, FeedClass::Cut, 10.0);
    b.follow(geom2d::Polyline::circle({30, 0}, 5), 0.0, FeedClass::Cut, 10.0); // feeds 35 mm across
    const Stats s = computeStats(tp, MachineLimits{100.0});
    CHECK_THAT(s.cutLength, WithinRel(40.0 + 35.0 + 2 * kPi * 5, 1e-9));
    CHECK_THAT(s.rapidLength, WithinRel(5.0, 1e-12));
    CHECK_THAT(s.feedTime, WithinRel(s.cutLength / 10.0, 1e-9));
    CHECK_THAT(s.rapidTime, WithinRel(0.05, 1e-9));
    CHECK_THAT(s.bounds.max.x(), WithinAbs(35.0, 0.02)); // bounds use sampled arcs
    CHECK_THAT(s.bounds.min.y(), WithinAbs(-5.0, 0.02));
    // The circle is two half arcs.
    CHECK(std::count_if(tp.moves.begin(), tp.moves.end(),
                        [](const Move& m) { return m.isArc(); }) == 2);
}

TEST_CASE("helix descends at most one pitch per turn", "[toolpath]") {
    Toolpath tp;
    tp.start = Vec3(4, 0, 0);
    PathBuilder b(tp);
    b.helix(Vec2(0, 0), 4.0, -5.0, 2.0, false, 10.0);
    // ceil(5 / 2) = 3 turns of two half arcs, plus a flat circle.
    REQUIRE(tp.moves.size() == 8);
    CHECK_THAT(tp.moves[5].end.z(), WithinAbs(-5.0, 1e-12));
    CHECK(tp.moves.back().end == Vec3(4, 0, -5));
    CHECK(tp.moves.back().feedClass == FeedClass::Cut);
    const double turn = std::hypot(2 * kPi * 4, 5.0 / 3.0);
    CHECK_THAT(computeStats(tp).cutLength, WithinRel(3 * turn + 2 * kPi * 4, 1e-9));
}

TEST_CASE("ramp along a loop ends at the first vertex at depth", "[toolpath]") {
    Toolpath tp;
    tp.start = Vec3(0, 0, 0);
    PathBuilder b(tp);
    const auto loop = geom2d::Polyline::rectangle({0, 0}, {20, 10}); // 60 mm per lap
    b.rampAlong(loop, -3.0, degToRad(2.0), 10.0);
    CHECK(b.position() == Vec3(0, 0, -3.0));
    double lastZ = 0.0;
    for (const Move& m : tp.moves) {
        CHECK(m.end.z() <= lastZ + 1e-12);
        lastZ = m.end.z();
    }
    // Horizontal ramp run = 3 / tan(2 deg) ~ 85.9 mm: more than one lap.
    double rampRun = 0.0;
    Vec3 pos = tp.start;
    for (const Move& m : tp.moves) {
        if (m.feedClass == FeedClass::Ramp) {
            rampRun += (m.end - pos).head<2>().norm();
        }
        pos = m.end;
    }
    CHECK_THAT(rampRun, WithinRel(3.0 / std::tan(degToRad(2.0)), 1e-9));
}

TEST_CASE("canned cycles expand into explicit moves", "[toolpath]") {
    Toolpath tp;
    tp.start = Vec3(0, 0, 20);
    PathBuilder b(tp);
    DrillCycle peck;
    peck.type = CycleType::Peck;
    peck.topZ = 0;
    peck.bottomZ = -10;
    peck.retractZ = 2;
    peck.peck = 3;
    peck.feedRate = 2.0;
    b.cycle({10, 0}, peck);
    b.cycle({20, 0}, peck);
    CHECK(tp.cycles.size() == 1); // identical cycles are shared
    CHECK(tp.moves.size() == 2);

    const Toolpath flat = expandCycles(tp);
    const auto plunges = std::count_if(flat.moves.begin(), flat.moves.end(), [](const Move& m) {
        return m.feedClass == FeedClass::Plunge;
    });
    CHECK(plunges == 8); // 3+3+3+1 per hole
    double deepest = 0;
    for (const Move& m : flat.moves) {
        deepest = std::min(deepest, m.end.z());
    }
    CHECK_THAT(deepest, WithinAbs(-10.0, 1e-12));
    CHECK(flat.moves.back().end == Vec3(20, 0, 2));
    // Feed length per hole: from the R plane (2 mm above the top) to -10, plus 3 re-entry gaps.
    CHECK_THAT(computeStats(tp).cutLength, WithinRel(2 * (12.0 + 1.5), 1e-9));

    DrillCycle dwell = peck;
    dwell.type = CycleType::Dwell;
    dwell.dwell = 1.5;
    Toolpath tp2;
    tp2.start = Vec3(0, 0, 20);
    PathBuilder b2(tp2);
    b2.cycle({0, 0}, dwell);
    const Stats s = computeStats(tp2, MachineLimits{1e9});
    CHECK_THAT(s.feedTime, WithinRel(12.0 / 2.0 + 1.5, 1e-9));
}

TEST_CASE("toolpath JSON round-trip and comparison", "[toolpath]") {
    Toolpath tp;
    tp.strategy = "test";
    tp.toolNumber = 3;
    tp.toolDiameter = 6;
    tp.spindleSpeed = 100;
    tp.start = Vec3(0, 0, 10);
    PathBuilder b(tp);
    b.rapid(Vec3(1, 2, 3));
    b.feed(Vec3(1, 2, 0), FeedClass::Plunge, 1.0);
    b.arc(Vec3(3, 2, 0), Vec2(2, 2), true, FeedClass::Cut, 2.0);
    b.dwell(0.5);
    DrillCycle c;
    c.retractZ = 2;
    c.bottomZ = -5;
    c.feedRate = 1.5;
    b.cycle({5, 5}, c);
    tp.warnings.push_back("note");

    const Toolpath back = toolpathFromJson(parseJson(dumpJson(toJson(tp)), "tp"));
    CHECK(compareToolpaths(tp, back).value_or("") == "");
    CHECK(back.toolNumber == 3);
    CHECK(back.warnings == tp.warnings);

    Toolpath changed = back;
    changed.moves[2].end.x() += 0.01;
    const auto diff = compareToolpaths(tp, changed);
    REQUIRE(diff.has_value());
    CHECK(diff->find("move 2") != std::string::npos);
}

TEST_CASE("move durations and partial moves for playback", "[toolpath]") {
    Toolpath tp;
    tp.start = Vec3(4, 0, 5);
    PathBuilder b(tp);
    b.rapid(Vec3(4, 0, 0));
    b.helix(Vec2(0, 0), 4.0, -2.0, 2.0, false, 10.0);
    const auto durations = moveDurations(tp, MachineLimits{100.0});
    REQUIRE(durations.size() == tp.moves.size());
    double total = 0.0;
    for (const double d : durations) {
        total += d;
    }
    CHECK_THAT(total, WithinRel(computeStats(tp, MachineLimits{100.0}).totalTime, 1e-9));
    CHECK_THAT(durations[0], WithinRel(5.0 / 100.0, 1e-12));

    // Half a line, half a helix arc, the whole move at t = 1.
    const auto line = partialMove(tp.start, Vec3::UnitZ(), tp.moves[0], 0.5);
    REQUIRE(line);
    CHECK_THAT((line->end - Vec3(4, 0, 2.5)).norm(), WithinAbs(0.0, 1e-12));
    const Move& arc = tp.moves[1];
    REQUIRE(arc.isArc());
    const auto half = partialMove(tp.moves[0].end, Vec3::UnitZ(), arc, 0.5);
    REQUIRE(half);
    CHECK(half->kind == arc.kind);
    CHECK_THAT((half->end.head<2>() - arc.center.head<2>()).norm(), WithinAbs(4.0, 1e-9));
    CHECK_THAT(half->end.z(), WithinAbs(0.5 * arc.end.z(), 1e-12));
    CHECK_THAT(std::abs(arcSweep(tp.moves[0].end, *half)),
               WithinRel(0.5 * std::abs(arcSweep(tp.moves[0].end, arc)), 1e-9));
    CHECK(*partialMove(tp.start, Vec3::UnitZ(), tp.moves[0], 1.0) == tp.moves[0]);
    CHECK_FALSE(partialMove(tp.start, Vec3::UnitZ(), tp.moves[0], 0.0));
}

TEST_CASE("arcs in the ZX and YZ planes", "[toolpath][arcs]") {
    Move m;
    m.kind = MoveKind::ArcCCW;
    m.plane = ArcPlane::ZX;
    m.end = Vec3(10, 3, 0);
    m.center = Vec3(0, 3, 0);
    const Vec3 start(-10, 3, 0);
    // Seen from +Y (Z to the right, X up, as G18 defines it), counter-clockwise from x -10 to
    // x 10 goes over the top (z > 0).
    CHECK_THAT(arcSweep(start, m), WithinAbs(kPi, 1e-12));
    for (const Vec3& q : sampleArc(start, m, 1e-3)) {
        CHECK_THAT(q.y(), WithinAbs(3.0, 1e-12));
        CHECK(q.z() >= -1e-12);
        CHECK_THAT((q - m.center).norm(), WithinAbs(10.0, 1e-9));
    }
    // JSON keeps the plane.
    Toolpath tp;
    tp.start = start;
    tp.moves.push_back(m);
    const Toolpath back = toolpathFromJson(toJson(tp));
    CHECK(back.moves.front().plane == ArcPlane::ZX);
    CHECK(compareToolpaths(tp, back, 1e-9).value_or("") == "");
    // Length: half a circle of radius 10.
    CHECK_THAT(computeStats(tp).cutLength, WithinAbs(kPi * 10.0, 1e-9));
}

TEST_CASE("smoothing fits arcs to runs of short lines in the XY, ZX and YZ planes",
          "[toolpath][smoothing]") {
    const auto lines = [](const std::vector<Vec3>& pts) {
        Toolpath tp;
        tp.start = pts.front();
        for (std::size_t k = 1; k < pts.size(); ++k) {
            Move m;
            m.kind = MoveKind::Linear;
            m.end = pts[k];
            m.feedRate = 10.0;
            tp.moves.push_back(m);
        }
        return tp;
    };
    // A quarter circle of radius 20 in each plane, every 0.5 degrees, and a straight line.
    for (const ArcPlane plane : {ArcPlane::XY, ArcPlane::ZX, ArcPlane::YZ}) {
        std::vector<Vec3> pts;
        for (int k = 0; k <= 180; ++k) {
            const double a = kPi / 2.0 * k / 180.0;
            pts.push_back(
                fromPlaneCoords(Vec2(20.0 * std::cos(a), 20.0 * std::sin(a)), 4.0, plane));
        }
        pts.push_back(pts.back() + fromPlaneCoords(Vec2(-15.0, 0.0), 0.0, plane));
        Toolpath tp = lines(pts);
        const SmoothingResult r = fitArcs(tp, 0.005);
        INFO("plane " << static_cast<int>(plane));
        CHECK(r.movesBefore == 181);
        CHECK(r.arcs == 1);
        REQUIRE(tp.moves.size() == 2);
        CHECK(tp.moves[0].isArc());
        CHECK(tp.moves[0].plane == plane);
        CHECK(tp.moves[0].kind == MoveKind::ArcCCW);
        CHECK((tp.moves[0].center - fromPlaneCoords(Vec2(0, 0), 4.0, plane)).norm() < 1e-6);
        CHECK(tp.moves[1].kind == MoveKind::Linear);
        CHECK((tp.moves[1].end - pts.back()).norm() < 1e-12);
    }
    // A run in no axis plane (an oblique raster) stays as it is; so do rapids and feed changes.
    std::vector<Vec3> oblique;
    for (int k = 0; k <= 90; ++k) {
        const double a = kPi * k / 180.0;
        oblique.emplace_back(std::cos(a) + 0.01 * k, 0.01 * k, std::sin(a));
    }
    Toolpath tp = lines(oblique);
    const std::size_t before = tp.moves.size();
    CHECK(fitArcs(tp, 0.005).arcs == 0);
    CHECK(tp.moves.size() == before);
}

TEST_CASE("arcs become short lines when smoothing is off", "[toolpath][smoothing]") {
    Toolpath tp;
    tp.start = Vec3(10, 0, 0);
    Move arc;
    arc.kind = MoveKind::ArcCCW;
    arc.end = Vec3(-10, 0, 0);
    arc.center = Vec3(0, 0, 0);
    arc.feedRate = 10.0;
    tp.moves.push_back(arc);
    Move line;
    line.kind = MoveKind::Linear;
    line.end = Vec3(-10, -5, 0);
    line.feedRate = 10.0;
    tp.moves.push_back(line);
    CHECK(linearizeArcs(tp, 0.001) == 1);
    REQUIRE(tp.moves.size() > 20);
    for (std::size_t k = 0; k + 1 < tp.moves.size(); ++k) {
        CHECK(tp.moves[k].kind == MoveKind::Linear);
        CHECK_THAT(tp.moves[k].end.norm(), WithinAbs(10.0, 1e-9)); // on the half circle
        CHECK(tp.moves[k].end.y() >= -1e-12);                      // counter-clockwise: over +Y
    }
    CHECK((tp.moves[tp.moves.size() - 2].end - Vec3(-10, 0, 0)).norm() < 1e-9);
    CHECK((tp.moves.back().end - Vec3(-10, -5, 0)).norm() < 1e-12);
    CHECK(linearizeArcs(tp, 0.001) == 0);
}

TEST_CASE("toolpaths in another frame keep what that frame can hold", "[toolpath][frame]") {
    Toolpath tp;
    tp.start = Vec3(0, 0, 10);
    PathBuilder b(tp);
    b.rapid({10, 0, 2});
    b.feed({10, 0, -1}, FeedClass::Plunge, 1.0);
    b.arc({-10, 0, -1}, {0, 0}, false, FeedClass::Cut, 5.0);
    DrillCycle drill;
    drill.topZ = 0;
    drill.bottomZ = -8;
    drill.retractZ = 2;
    drill.feedRate = 2.0;
    b.cycle({5, 5}, drill);
    const double length = computeStats(tp).cutLength;

    SECTION("a turn about Z and a shift keep arcs and cycles") {
        Transform3 t = Transform3::Identity();
        t.linear() = Eigen::AngleAxisd(kPi / 2, Vec3::UnitZ()).toRotationMatrix();
        t.translation() = Vec3(1, 2, 3);
        const Toolpath moved = transformed(tp, t);
        REQUIRE(moved.moves.size() == tp.moves.size());
        CHECK(moved.moves[2].kind == MoveKind::ArcCCW);
        CHECK((moved.moves[2].center - Vec3(1, 2, 2)).norm() < 1e-12);
        CHECK((moved.moves[2].end - Vec3(1, -8, 2)).norm() < 1e-12);
        CHECK(moved.moves[3].kind == MoveKind::Cycle);
        CHECK_THAT(moved.cycles[0].bottomZ, WithinAbs(-5.0, 1e-12));
        CHECK_THAT(computeStats(moved).cutLength, WithinRel(length, 1e-9));
        CHECK((moved.start - Vec3(1, 2, 13)).norm() < 1e-12);
    }
    SECTION("a tilt makes lines of the arc, expands the cycle and turns the tool axis") {
        // The frame's Z along -Y.
        Transform3 t = Transform3::Identity();
        t.linear().col(0) = Vec3::UnitX();
        t.linear().col(1) = Vec3::UnitZ();
        t.linear().col(2) = -Vec3::UnitY();
        const Toolpath moved = transformed(tp, t, 1e-3);
        CHECK(moved.cycles.empty());
        for (const Move& m : moved.moves) {
            CHECK_FALSE(m.isArc());
            CHECK(m.kind != MoveKind::Cycle);
            CHECK((m.axis - Vec3(0, -1, 0)).norm() < 1e-12);
        }
        CHECK(isMultiAxis(moved));
        CHECK_THAT(computeStats(moved).cutLength, WithinRel(length, 1e-4));
        // The plunge goes along -Z of the frame: +Y here.
        CHECK((moved.moves[1].end - Vec3(10, 1, 0)).norm() < 1e-12);
        // The way back is the same path.
        const Toolpath back = transformed(moved, t.inverse());
        CHECK((back.moves.back().end - Vec3(5, 5, 2)).norm() < 1e-9);
    }
}

TEST_CASE("a machine function rides on a dwell, in JSON and comparisons", "[toolpath][barpull]") {
    Toolpath tp;
    tp.start = Vec3(0, 0, 5);
    PathBuilder b(tp);
    b.feed(Vec3(0, 0, -10), FeedClass::Plunge, 5.0);
    Move open;
    open.kind = MoveKind::Dwell;
    open.end = b.position();
    open.dwell = 0.5;
    open.function = MachineFunction::ChuckOpen;
    tp.moves.push_back(open);
    const Json j = toJson(tp);
    CHECK(j["moves"][1]["fn"] == "chuck_open");
    CHECK_FALSE(j["moves"][0].contains("fn"));
    const Toolpath back = toolpathFromJson(j);
    CHECK(back.moves[1].function == MachineFunction::ChuckOpen);
    CHECK(compareToolpaths(tp, back).value_or("") == "");
    Toolpath changed = back;
    changed.moves[1].function = MachineFunction::ChuckClose;
    CHECK(compareToolpaths(tp, changed).value_or("").find("machine function") != std::string::npos);
    for (const MachineFunction f :
         {MachineFunction::None, MachineFunction::ChuckOpen, MachineFunction::ChuckClose,
          MachineFunction::BarFeederOn, MachineFunction::BarFeederOff}) {
        CHECK(parseMachineFunction(toString(f)) == f);
    }
    Json bad = j;
    bad["moves"][1]["fn"] = "launch";
    CHECK_THROWS(toolpathFromJson(bad));
}
