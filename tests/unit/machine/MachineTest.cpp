// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/foundation/Error.hpp>
#include <ocam/machine/Machine.hpp>
#include <ocam/toolpath/PathBuilder.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace ocam;
using namespace ocam::machine;
using Catch::Matchers::WithinRel;

namespace {

toolpath::Toolpath simplePath(double spanX, double feed, double rpm) {
    toolpath::Toolpath tp;
    tp.start = Vec3(0, 0, 10);
    tp.spindleSpeed = rpm / 60.0;
    toolpath::PathBuilder b(tp);
    b.rapid(Vec3(0, 0, 1));
    b.feed(Vec3(spanX, 0, 1), toolpath::FeedClass::Cut, feed / 60.0);
    return tp;
}

} // namespace

TEST_CASE("built-in generic 3-axis machine", "[machine]") {
    const Machine& m = genericMill3();
    CHECK(m.id == "generic-3axis-vmc");
    CHECK(m.validate().empty());
    REQUIRE(m.axis("X") != nullptr);
    CHECK(m.axis("X")->travel() == 800.0);
    CHECK_THAT(m.maxSpindle * 60.0, WithinRel(12000.0, 1e-12));
    CHECK_THAT(m.maxFeed * 60.0, WithinRel(10000.0, 1e-12));
    CHECK(builtinMachine("generic-3axis") == &m);
    CHECK(builtinMachine("") == &m);
    CHECK(builtinMachine("unknown") == nullptr);
    CHECK(machineFromJson(toJson(m)) == m);
    CHECK(loadMachine(OCAM_SOURCE_DIR "/data/machines/generic-3axis-vmc.json") == m);
}

TEST_CASE("what a lathe's turret can do", "[machine]") {
    const Machine& plain = genericLathe2();
    CHECK_FALSE(plain.liveTooling);
    CHECK_FALSE(plain.hasYAxis());
    const Machine& live = genericLatheLive();
    CHECK(live.isLathe());
    CHECK(live.liveTooling);
    CHECK(live.liveMaxSpindle == Catch::Approx(100.0)); // 6000 rpm
    CHECK_FALSE(live.hasYAxis());
    CHECK(live.validate().empty());
    const Machine& y = genericLatheLiveY();
    CHECK(y.liveTooling);
    CHECK(y.hasYAxis());
    CHECK(y.validate().empty());
    // A file keeps them, and one without says the machine has none.
    const Machine back = machineFromJson(toJson(y), "test");
    CHECK(back.liveTooling);
    CHECK(back.liveMaxSpindle == Catch::Approx(y.liveMaxSpindle));
    CHECK(back.hasYAxis());
    Json older = toJson(plain);
    older["version"] = 6;
    const Machine six = machineFromJson(older, "test");
    CHECK_FALSE(six.liveTooling);
    // Nonsense is said.
    Machine bad = plain;
    bad.liveTooling = true;
    CHECK_FALSE(bad.validate().empty());
    Machine mill = genericMill3();
    mill.liveTooling = true;
    mill.liveMaxSpindle = 100.0;
    CHECK_FALSE(mill.validate().empty());
}

TEST_CASE("invalid machine documents are rejected", "[machine]") {
    Json doc = toJson(genericMill3());
    doc["kinematics"] = "hexapod";
    CHECK_THROWS_AS(machineFromJson(doc), Error);
    Json noZ = toJson(genericMill3());
    noZ["axes"].erase(2);
    noZ["axes"].push_back({{"name", "A"}, {"min", -360}, {"max", 360}});
    CHECK_THROWS_AS(machineFromJson(noZ), Error);
}

TEST_CASE("solve accepts valid paths and refuses speeds and feeds beyond the limits", "[machine]") {
    const Machine& m = genericMill3();
    const SolveResult ok = solve(m, simplePath(100, 1000, 8000));
    CHECK(ok.ok());
    CHECK(ok.warnings.empty());

    // ADR-0004: nothing is clamped. Each value beyond a limit is an error naming the value, the
    // limit and where to change them; the toolpath keeps the operation's values.
    const auto mentions = [](const std::string& text, std::string_view part) {
        return text.find(part) != std::string::npos;
    };
    const SolveResult fast = solve(m, simplePath(100, 20000, 20000));
    CHECK_FALSE(fast.ok());
    CHECK(fast.warnings.empty());
    REQUIRE(fast.errors.size() == 2);
    CHECK(mentions(fast.errors[0], "spindle speed 20000 rpm"));
    CHECK(mentions(fast.errors[0], "spindle max 12000 rpm"));
    CHECK(mentions(fast.errors[0], "cutting preset"));
    CHECK(mentions(fast.errors[1], "cutting feed 20000 mm/min"));
    CHECK(mentions(fast.errors[1], "max feed 10000 mm/min"));
    CHECK(mentions(fast.errors[1], "machine library"));
    CHECK_THAT(fast.path.spindleSpeed * 60.0, WithinRel(20000.0, 1e-12));
    CHECK_THAT(fast.path.moves.back().feedRate * 60.0, WithinRel(20000.0, 1e-12));

    const SolveResult slow = solve(m, simplePath(100, 1000, 50));
    REQUIRE(slow.errors.size() == 1);
    CHECK(mentions(slow.errors[0], "spindle speed 50 rpm is below the machine's spindle min 100"));
    CHECK_THAT(slow.path.spindleSpeed * 60.0, WithinRel(50.0, 1e-12));
    // A stopped spindle is not below the minimum.
    CHECK(solve(m, simplePath(100, 1000, 0)).ok());

    // Plunge and cycle feeds are named; the limit itself is allowed.
    toolpath::Toolpath plunge = simplePath(100, 10000, 8000);
    toolpath::PathBuilder(plunge).feed(Vec3(100, 0, -5), toolpath::FeedClass::Plunge,
                                       12000.0 / 60.0);
    const SolveResult plunged = solve(m, plunge);
    REQUIRE(plunged.errors.size() == 1);
    CHECK(mentions(plunged.errors[0], "plunge feed 12000 mm/min"));
    CHECK_THAT(plunged.path.moves.back().feedRate * 60.0, WithinRel(12000.0, 1e-12));
    CHECK(solve(m, simplePath(100, 10000, 12000)).ok());
    toolpath::Toolpath drilled = simplePath(10, 1000, 8000);
    toolpath::DrillCycle drill;
    drill.bottomZ = -10.0;
    drill.retractZ = 2.0;
    drill.feedRate = 15000.0 / 60.0;
    toolpath::PathBuilder(drilled).cycle(Vec2(5, 0), drill);
    const SolveResult cycled = solve(m, drilled);
    REQUIRE(cycled.errors.size() == 1);
    CHECK(mentions(cycled.errors[0], "cycle feed 15000 mm/min"));
    CHECK_THAT(cycled.path.cycles.back().feedRate * 60.0, WithinRel(15000.0, 1e-12));

    const SolveResult tooWide = solve(m, simplePath(900, 1000, 8000));
    CHECK_FALSE(tooWide.ok());

    toolpath::Toolpath tilted = simplePath(10, 1000, 8000);
    tilted.moves.back().axis = Vec3(0, 1, 1).normalized();
    CHECK_FALSE(solve(m, tilted).ok());
}

TEST_CASE("3+2 index angles on a trunnion machine", "[machine][index]") {
    const Machine& m = genericMill5TableAC();
    CHECK(m.canIndex());
    CHECK(m.validate().empty());
    CHECK(builtinMachine("generic-5axis-trunnion-ac") == &m);
    // Straight up: no rotation.
    auto up = indexAngles(m, Vec3::UnitZ());
    REQUIRE(up);
    CHECK(*up == IndexAngles{});
    // Every reachable direction is turned onto the spindle axis.
    for (const Vec3& d :
         {Vec3(0, -1, 0), Vec3(1, 0, 0), Vec3(0, 1, 1), Vec3(-1, 2, 0.5), Vec3(0.3, -0.2, -0.1)}) {
        const auto angles = indexAngles(m, d);
        INFO("direction " << d.transpose());
        REQUIRE(angles);
        CHECK((tableRotation(m, *angles) * d.normalized() - Vec3::UnitZ()).norm() < 1e-9);
        CHECK(std::abs(angles->a) <= 120.0 * kPi / 180.0 + 1e-9);
    }
    // The front face (WCS Z = -Y) needs a 90 degree tilt.
    const auto front = indexAngles(m, -Vec3::UnitY());
    REQUIRE(front);
    CHECK_THAT(std::abs(front->a), WithinRel(kPi / 2.0, 1e-9));
    // Straight down is beyond the A limits.
    CHECK_FALSE(indexAngles(m, -Vec3::UnitZ()));
    // A 3-axis machine cannot index.
    CHECK_FALSE(indexAngles(genericMill3(), Vec3::UnitX()));
}

TEST_CASE("solve rotates indexed programs onto the machine axes", "[machine][index]") {
    toolpath::Toolpath tp;
    tp.start = {0, 0, 20};
    toolpath::PathBuilder b(tp);
    b.rapid({10, 0, 20});
    b.feed({10, 0, -2}, toolpath::FeedClass::Plunge, 5.0);
    b.feed({30, 0, -2}, toolpath::FeedClass::Cut, 10.0);
    // WCS whose X axis is the table's Y and whose Z is the table's -X (the left face).
    Mat3 wcs;
    wcs.col(0) = Vec3::UnitY();
    wcs.col(2) = -Vec3::UnitX();
    wcs.col(1) = wcs.col(2).cross(wcs.col(0));
    const SolveResult r = solve(genericMill5TableAC(), tp, wcs);
    REQUIRE(r.ok());
    REQUIRE(r.index);
    // Distances and heights are preserved; only the XY orientation turns.
    for (std::size_t k = 0; k < tp.moves.size(); ++k) {
        CHECK_THAT(r.path.moves[k].end.z(), WithinRel(tp.moves[k].end.z(), 1e-12));
        CHECK(std::abs(r.path.moves[k].end.head<2>().norm() - tp.moves[k].end.head<2>().norm()) <
              1e-9);
    }
    // The program's X axis is where the table rotation puts the WCS X axis.
    const Vec3 x = tableRotation(genericMill5TableAC(), *r.index) * wcs.col(0);
    const Vec3 p = r.path.moves.back().end;
    CHECK((Vec3(p.x(), p.y(), 0).normalized() - x).norm() < 1e-9);
    // On a 3-axis machine the same setup is posted as it is (separate fixturing).
    const SolveResult r3 = solve(genericMill3(), tp, wcs);
    CHECK_FALSE(r3.index);
    CHECK((r3.path.moves.back().end - tp.moves.back().end).norm() < 1e-12);
}
