// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "StrategyTestUtil.hpp"

#include <ocam/strategies/Strategies.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>

using namespace ocam;
using namespace ocam::strategies;
using namespace ocam::strategies::testing;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using toolpath::FeedClass;
using toolpath::MoveKind;

TEST_CASE("chamfer geometry follows the tool's cone", "[strategies][chamfer]") {
    const auto mill90 = tooling::makeChamferMill(1, 12.0, 0.0, degToRad(90.0), 60.0);
    const ChamferGeometry g = chamferGeometry(mill90, 1.0, 0.5);
    CHECK_THAT(g.depth, WithinAbs(1.0, 1e-12));
    CHECK_THAT(g.centreOffset, WithinAbs(0.5, 1e-12));
    CHECK_THAT(g.tipBelowEdge, WithinAbs(1.5, 1e-12));

    const auto mill60 = tooling::makeChamferMill(1, 12.0, 1.0, degToRad(60.0), 60.0);
    const ChamferGeometry h = chamferGeometry(mill60, 1.0, 0.0);
    CHECK_THAT(h.depth, WithinAbs(1.0 / std::tan(degToRad(30.0)), 1e-12));
    CHECK_THAT(h.centreOffset, WithinAbs(0.5, 1e-12)); // the flat tip radius

    CHECK_THROWS_WITH(chamferGeometry(mill90, 6.0, 0.5), ContainsSubstring("too wide"));
    CHECK_THROWS_WITH(chamferGeometry(tooling::makeFlatEndMill(1, 6, 20, 50), 1.0, 0.5),
                      ContainsSubstring("pointed tool"));
}

TEST_CASE("chamfer runs the edge offset at the chamfer depth", "[strategies][chamfer]") {
    Harness h(tooling::makeChamferMill(1, 12.0, 0.0, degToRad(90.0), 60.0));
    const geom2d::Regions rect{{geom2d::Polyline::rectangle({0, 0}, {40, 20}), {}}};
    ChamferParams p;
    p.width = 1.0;
    p.tipOffset = 0.5;
    p.leadRadius = 0.0; // no leads: every point lies on the offset path
    const auto tp = chamfer(rect, 0.0, p, h.context());
    CHECK(tp.strategy == "chamfer2d");
    const auto paths = cutPathsAt(tp, -1.5);
    REQUIRE_FALSE(paths.empty());
    double maxX = -1e9, minY = 1e9;
    for (const auto& path : paths) {
        for (const Vec2& q : path) {
            maxX = std::max(maxX, q.x());
            minY = std::min(minY, q.y());
        }
    }
    CHECK_THAT(maxX, WithinAbs(40.5, 1e-3));
    CHECK_THAT(minY, WithinAbs(-0.5, 1e-3));
    for (const Vec3& q : feedPoints(tp)) {
        CHECK(q.z() >= -1.5 - 1e-9);
    }
}

TEST_CASE("tapping uses a synchronized tap cycle", "[strategies][tap]") {
    auto t = tooling::makeDefaultTool(tooling::ToolType::Tap, 1, 10.0);
    t.mill.threadPitch = 1.5;
    Harness h(t);
    const std::vector<HoleTarget> holes{{{10, 10}, 0.0, 15.0, false, 8.5}};
    TapParams p;
    p.threadDepth = 12.0;
    const auto tp = tap(holes, p, h.context());
    REQUIRE(tp.cycles.size() == 1);
    CHECK(tp.cycles[0].type == toolpath::CycleType::Tap);
    CHECK_THAT(tp.cycles[0].pitch, WithinAbs(1.5, 1e-12));
    CHECK_THAT(tp.cycles[0].bottomZ, WithinAbs(-12.0, 1e-9));
    CHECK_THAT(tp.cycles[0].feedRate, WithinAbs(1.5 * h.feeds.spindle, 1e-9));
    CHECK_THROWS_WITH(tap(holes, p, Harness(tooling::makeFlatEndMill(1, 6, 20, 50)).context()),
                      ContainsSubstring("needs a tap"));
    CHECK_FALSE(tp.spindleReverse);

    // The tool says the hand: the same feed and speed, the spindle the other way.
    t.mill.leftHand = true;
    Harness left(t);
    const auto lh = tap(holes, p, left.context());
    REQUIRE(lh.cycles.size() == 1);
    CHECK(lh.cycles[0].type == toolpath::CycleType::TapLeft);
    CHECK(lh.spindleReverse);
    CHECK_THAT(lh.spindleSpeed, WithinAbs(tp.spindleSpeed, 1e-12));
    CHECK_THAT(lh.cycles[0].feedRate, WithinAbs(tp.cycles[0].feedRate, 1e-12));
}

TEST_CASE("thread milling climbs a helix of the thread pitch", "[strategies][threadmill]") {
    auto t = tooling::makeDefaultTool(tooling::ToolType::ThreadMill, 1, 6.0);
    t.mill.threadPitch = 1.5;
    t.mill.fluteLength = 3.0; // single form: one pitch per revolution over the length
    Harness h(t);
    const std::vector<HoleTarget> holes{{{0, 0}, 0.0, 9.0, false, 8.5}};
    ThreadMillParams p;
    p.majorDiameter = 10.0;
    const auto tp = threadMill(holes, p, h.context());
    CHECK(tp.warnings.empty());
    const double rh = 5.0 - t.mill.diameter / 2.0;
    int helixArcs = 0;
    double zMin = 1e9, zMax = -1e9;
    Vec3 pos = tp.start;
    for (const auto& m : tp.moves) {
        if (m.kind == MoveKind::ArcCCW && m.feedClass == FeedClass::Cut) {
            ++helixArcs;
            CHECK_THAT((m.end.head<2>() - m.center.head<2>()).norm(), WithinAbs(rh, 1e-9));
            CHECK_THAT(m.end.z() - pos.z(), WithinAbs(0.75, 1e-9)); // half a pitch per half turn
            zMin = std::min(zMin, pos.z());
            zMax = std::max(zMax, m.end.z());
        }
        CHECK(m.kind != MoveKind::ArcCW); // right hand: counter-clockwise only
        pos = m.end;
    }
    CHECK(helixArcs == 2 * 6); // ceil(9 / 1.5) turns
    CHECK_THAT(zMin, WithinAbs(-9.0, 1e-9));
    CHECK_THAT(zMax, WithinAbs(0.0, 1e-9));

    // A left-hand thread, climb milled: counter-clockwise still, going down.
    p.rightHand = false;
    const auto left = threadMill(holes, p, h.context());
    pos = left.start;
    for (const auto& m : left.moves) {
        if (m.feedClass == FeedClass::Cut) {
            CHECK(m.kind == MoveKind::ArcCCW);
            CHECK_THAT(m.end.z() - pos.z(), WithinAbs(-0.75, 1e-9));
        }
        pos = m.end;
    }
}

namespace {

/// The helix's half turns of a thread milling toolpath: their radius, sense and rise.
struct HelixArc {
    double radius;
    bool ccw;
    double rise;
};
std::vector<HelixArc> helixArcs(const toolpath::Toolpath& tp) {
    std::vector<HelixArc> arcs;
    Vec3 pos = tp.start;
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::Cut && m.isArc()) {
            arcs.push_back({(m.end.head<2>() - m.center.head<2>()).norm(),
                            m.kind == MoveKind::ArcCCW, m.end.z() - pos.z()});
        }
        pos = m.end;
    }
    return arcs;
}

tooling::Tool threadMillTool(double diameter, double pitch, double flutes) {
    auto t = tooling::makeDefaultTool(tooling::ToolType::ThreadMill, 1, diameter);
    t.mill.threadPitch = pitch;
    t.mill.fluteLength = flutes;
    return t;
}

} // namespace

TEST_CASE("an external thread's helix runs at the major minus the thread depth plus the tool",
          "[strategies][threadmill]") {
    Harness h(threadMillTool(6.0, 1.5, 3.0));
    // An M10 stud, 9 long, its top at z 0.
    const std::vector<HoleTarget> studs{{{0, 0}, 0.0, 9.0, false, 10.0}};
    ThreadMillParams p;
    p.external = true;
    SECTION("the thread depth as set") {
        p.threadDepth = 1.0;
        const auto arcs = helixArcs(threadMill(studs, p, h.context()));
        REQUIRE(arcs.size() == 2 * 6);
        for (const HelixArc& a : arcs) {
            CHECK_THAT(a.radius, WithinAbs(5.0 - 1.0 + 3.0, 1e-9));
        }
    }
    SECTION("0: the ISO external depth, and the major the stud's diameter") {
        const auto arcs = helixArcs(threadMill(studs, p, h.context()));
        REQUIRE_FALSE(arcs.empty());
        CHECK_THAT(arcs.front().radius, WithinAbs(5.0 - 0.6134 * 1.5 + 3.0, 1e-9));
        const ThreadMillGeometry g = threadMillGeometry(p, 1.5, 10.0, 3.0);
        CHECK_THAT(g.major, WithinAbs(10.0, 1e-12));
        CHECK_THAT(g.threadDepth, WithinAbs(0.6134 * 1.5, 1e-12));
    }
    SECTION("the major diameter as set") {
        p.majorDiameter = 10.0;
        p.threadDepth = 0.8;
        const std::vector<HoleTarget> small{{{0, 0}, 0.0, 9.0, false, 9.9}};
        const auto arcs = helixArcs(threadMill(small, p, h.context()));
        REQUIRE_FALSE(arcs.empty());
        CHECK_THAT(arcs.front().radius, WithinAbs(5.0 - 0.8 + 3.0, 1e-9));
        // A boss bigger than the major diameter is not threaded.
        const std::vector<HoleTarget> big{{{0, 0}, 0.0, 9.0, false, 12.0}};
        const auto none = threadMill(big, p, h.context());
        CHECK(helixArcs(none).empty());
        CHECK_THAT(none.warnings.front(), ContainsSubstring("larger than the major"));
    }
}

TEST_CASE("the hand and the cut direction set the helix's sense and whether it climbs",
          "[strategies][threadmill]") {
    Harness h(threadMillTool(6.0, 1.5, 3.0));
    const std::vector<HoleTarget> hole{{{0, 0}, 0.0, 9.0, false, 8.5}};
    const std::vector<HoleTarget> stud{{{0, 0}, 0.0, 9.0, false, 10.0}};
    struct Case {
        bool external, rightHand;
        CutDirection direction;
        bool ccw, up;
    };
    // Climb with M03: counter-clockwise inside, clockwise outside. A right-hand helix rises
    // counter-clockwise.
    const Case cases[] = {
        {false, true, CutDirection::Climb, true, true},
        {false, true, CutDirection::Conventional, false, false},
        {false, false, CutDirection::Climb, true, false},
        {false, false, CutDirection::Conventional, false, true},
        {true, true, CutDirection::Climb, false, false},
        {true, true, CutDirection::Conventional, true, true},
        {true, false, CutDirection::Climb, false, true},
        {true, false, CutDirection::Conventional, true, false},
    };
    for (const Case& c : cases) {
        CAPTURE(c.external, c.rightHand, c.direction == CutDirection::Climb);
        ThreadMillParams p;
        p.external = c.external;
        p.rightHand = c.rightHand;
        p.direction = c.direction;
        const auto tp = threadMill(c.external ? stud : hole, p, h.context());
        const auto arcs = helixArcs(tp);
        REQUIRE(arcs.size() == 12);
        for (const HelixArc& a : arcs) {
            CHECK(a.ccw == c.ccw);
            CHECK_THAT(a.rise, WithinAbs(c.up ? 0.75 : -0.75, 1e-9));
        }
        // The thread's hand from the path alone: a right-hand helix rises counter-clockwise.
        CHECK((arcs.front().ccw == (arcs.front().rise > 0.0)) == c.rightHand);
        // It covers the length either way: from the bottom to the top (6 pitches).
        double lo = 1e9, hi = -1e9;
        Vec3 pos = tp.start;
        for (const auto& m : tp.moves) {
            if (m.feedClass == FeedClass::Cut) {
                lo = std::min({lo, pos.z(), m.end.z()});
                hi = std::max({hi, pos.z(), m.end.z()});
            }
            pos = m.end;
        }
        CHECK_THAT(lo, WithinAbs(-9.0, 1e-9));
        CHECK_THAT(hi, WithinAbs(0.0, 1e-9));
    }
}

TEST_CASE("an external thread leads on from outside the major diameter, tangent to the helix",
          "[strategies][threadmill]") {
    Harness h(threadMillTool(6.0, 1.5, 3.0));
    const std::vector<HoleTarget> stud{{{20, 10}, 0.0, 6.0, false, 10.0}};
    const Vec2 c(20, 10);
    ThreadMillParams p;
    p.external = true;
    p.threadDepth = 0.9;
    p.radialPasses = 3;
    p.springPass = true;
    p.leadRadius = 2.0;
    const auto tp = threadMill(stud, p, h.context());
    CHECK(tp.warnings.empty());

    // Three equal steps into the thread depth, then the last again.
    std::vector<double> radii;
    double previous = -1.0;
    for (const HelixArc& a : helixArcs(tp)) {
        if (std::abs(a.radius - previous) > 1e-9) {
            radii.push_back(a.radius);
            previous = a.radius;
        }
    }
    const double r = 3.0;
    const std::vector<double> expected{5.0 - 0.3 + r, 5.0 - 0.6 + r, 5.0 - 0.9 + r};
    REQUIRE(radii.size() == 3);
    for (std::size_t k = 0; k < 3; ++k) {
        CHECK_THAT(radii[k], WithinAbs(expected[k], 1e-9));
    }
    const auto leadIns = std::count_if(tp.moves.begin(), tp.moves.end(), [](const auto& m) {
        return m.feedClass == FeedClass::LeadIn;
    });
    CHECK(leadIns == 4); // the spring pass is a pass of its own

    Vec3 pos = tp.start;
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::LeadIn || m.feedClass == FeedClass::LeadOut) {
            REQUIRE(m.isArc());
            // The lead feed as set, for the leads; their arcs are the lead radius.
            CHECK_THAT(m.feedRate, WithinAbs(h.feeds.lead, 1e-12));
            CHECK_THAT((m.end.head<2>() - m.center.head<2>()).norm(), WithinAbs(2.0, 1e-9));
            // Off the helix's end, the tool is clear of the major diameter.
            const Vec2 outside =
                m.feedClass == FeedClass::LeadIn ? Vec2(pos.head<2>()) : Vec2(m.end.head<2>());
            CHECK((outside - c).norm() > 5.0 + r);
            // Tangent where it meets the helix: its centre on the helix's radius through there.
            const Vec2 onHelix =
                m.feedClass == FeedClass::LeadIn ? Vec2(m.end.head<2>()) : Vec2(pos.head<2>());
            const Vec2 toLead = m.center.head<2>() - onHelix;
            const Vec2 fromAxis = onHelix - c;
            CHECK(std::abs(toLead.x() * fromAxis.y() - toLead.y() * fromAxis.x()) < 1e-9);
            CHECK(toLead.dot(fromAxis) > 0.0); // outside the helix
            // Turning the other way from the helix (clockwise when climbing a boss).
            CHECK(m.kind == MoveKind::ArcCCW);
        }
        if (m.feedClass == FeedClass::Cut) {
            CHECK_THAT(m.feedRate, WithinAbs(h.feeds.cut, 1e-12));
            CHECK(m.kind == MoveKind::ArcCW);
        }
        if (m.feedClass == FeedClass::Plunge) {
            CHECK((m.end.head<2>() - c).norm() > 5.0 + r); // down beside the stud
        }
        pos = m.end;
    }

    // A lead no larger than the thread depth would start on the thread.
    p.leadRadius = 0.9;
    CHECK_THROWS_WITH(threadMill(stud, p, h.context()), ContainsSubstring("lead radius"));
}

TEST_CASE("a multi-form thread mill goes around once", "[strategies][threadmill]") {
    Harness h(threadMillTool(6.0, 1.5, 12.0));
    const std::vector<HoleTarget> stud{{{0, 0}, 0.0, 9.0, false, 10.0}};
    ThreadMillParams p;
    p.external = true;
    p.multiForm = true;
    const auto arcs = helixArcs(threadMill(stud, p, h.context()));
    CHECK(arcs.size() == 2);
    // Its flutes have to cover the thread.
    const std::vector<HoleTarget> longStud{{{0, 0}, 0.0, 15.0, false, 10.0}};
    CHECK_THROWS_WITH(threadMill(longStud, p, h.context()), ContainsSubstring("flute length"));
    // Single point: a turn per pitch.
    p.multiForm = false;
    CHECK(helixArcs(threadMill(stud, p, h.context())).size() == 12);
}

TEST_CASE("an internal thread's radial passes step out to the major diameter",
          "[strategies][threadmill]") {
    Harness h(threadMillTool(6.0, 1.5, 3.0));
    const std::vector<HoleTarget> hole{{{0, 0}, 0.0, 9.0, false, 8.5}};
    ThreadMillParams p;
    p.majorDiameter = 10.0;
    p.threadDepth = 0.8;
    p.radialPasses = 2;
    const ThreadMillGeometry g = threadMillGeometry(p, 1.5, 8.5, 3.0);
    REQUIRE(g.passRadii.size() == 2);
    CHECK_THAT(g.passRadii[0], WithinAbs(5.0 - 0.4 - 3.0, 1e-12));
    CHECK_THAT(g.passRadii[1], WithinAbs(5.0 - 3.0, 1e-12)); // major / 2 - r, as with one pass
    const auto tp = threadMill(hole, p, h.context());
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::LeadIn || m.feedClass == FeedClass::LeadOut) {
            CHECK_THAT(m.feedRate, WithinAbs(h.feeds.lead, 1e-12));
        }
    }
}

TEST_CASE("thread milling parameters", "[strategies][threadmill]") {
    const ThreadMillParams internal = threadMillParams(Json::object());
    CHECK_FALSE(internal.external);
    CHECK(internal.direction == CutDirection::Climb);
    CHECK(internal.radialPasses == 1);
    CHECK_FALSE(internal.springPass);
    CHECK_FALSE(internal.multiForm);
    CHECK_FALSE(internal.leadRadius);
    const ThreadMillParams p = threadMillParams(Json{{"side", "external"},
                                                     {"hand", "left"},
                                                     {"direction", "conventional"},
                                                     {"form", "multi"},
                                                     {"passes", 3},
                                                     {"spring_pass", true},
                                                     {"lead_radius", 2.5},
                                                     {"radial_depth", 0.7}});
    CHECK(p.external);
    CHECK_FALSE(p.rightHand);
    CHECK(p.direction == CutDirection::Conventional);
    CHECK(p.multiForm);
    CHECK(p.radialPasses == 3);
    CHECK(p.springPass);
    CHECK(p.leadRadius == 2.5);
    CHECK(p.threadDepth == 0.7);
}

TEST_CASE("boring: helical for end mills, cycle for boring bars", "[strategies][bore]") {
    Harness mill(tooling::makeFlatEndMill(1, 6.0, 20.0, 50.0));
    const std::vector<HoleTarget> holes{{{20, 10}, 0.0, 8.0, true, 16.0},
                                        {{50, 10}, 0.0, 8.0, false, 5.0}};
    BoreParams p;
    p.helixPitch = 1.0;
    const auto tp = bore(holes, p, mill.context());
    // The Ø5 hole is smaller than the tool.
    CHECK(std::any_of(tp.warnings.begin(), tp.warnings.end(),
                      [](const auto& w) { return w.find("not larger") != std::string::npos; }));
    double zMin = 1e9;
    for (const auto& m : tp.moves) {
        if (m.isArc()) {
            CHECK_THAT((m.end.head<2>() - m.center.head<2>()).norm(), WithinAbs(5.0, 1e-9));
            zMin = std::min(zMin, m.end.z());
        }
    }
    CHECK_THAT(zMin, WithinAbs(-8.0 - p.breakthrough, 1e-9));

    auto bar = tooling::makeDefaultTool(tooling::ToolType::BoringBar, 2, 16.0);
    Harness boring(bar);
    const auto cycle = bore({holes[0]}, p, boring.context());
    REQUIRE(cycle.cycles.size() == 1);
    CHECK(cycle.cycles[0].type == toolpath::CycleType::Bore);
}
