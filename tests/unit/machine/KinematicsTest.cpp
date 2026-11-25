// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/machine/Kinematics.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <random>

using namespace ocam;
using namespace ocam::machine;
using Catch::Matchers::WithinAbs;

namespace {

Machine testMachine(Kinematics k) {
    Machine m;
    m.id = std::string(toString(k));
    m.name = m.id;
    m.kinematics = k;
    m.axes = {{"X", -500, 500, 0}, {"Y", -500, 500, 0}, {"Z", -500, 500, 0}};
    switch (k) {
    case Kinematics::Mill4A:
        m.axes.push_back({"A", -9999, 9999, 0});
        break;
    case Kinematics::Mill5TableAC:
        m.axes.push_back({"A", -120, 120, 0});
        m.axes.push_back({"C", -9999, 9999, 0});
        break;
    case Kinematics::Mill5TableBC:
    case Kinematics::Mill5HeadTableBC:
    case Kinematics::Mill5HeadBC:
        m.axes.push_back({"B", -120, 120, 0});
        m.axes.push_back({"C", -9999, 9999, 0});
        break;
    default:
        break;
    }
    m.maxSpindle = 200;
    m.maxFeed = 100;
    m.rapidRate = 500;
    m.pivotLength = k == Kinematics::Mill5HeadTableBC || k == Kinematics::Mill5HeadBC ? 150 : 0;
    return m;
}

Vec3 randomAxis(std::mt19937& rng, bool fourAxis) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (;;) {
        Vec3 v(fourAxis ? 0.0 : u(rng), u(rng), u(rng));
        if (v.norm() > 0.2 && v.norm() <= 1.0 && v.normalized().z() > -0.3) {
            return v.normalized();
        }
    }
}

} // namespace

TEST_CASE("inverse kinematics round trips for every kinematic class", "[machine][kinematics]") {
    const Kinematics k =
        GENERATE(Kinematics::Mill4A, Kinematics::Mill5TableAC, Kinematics::Mill5TableBC,
                 Kinematics::Mill5HeadTableBC, Kinematics::Mill5HeadBC);
    const Machine m = testMachine(k);
    REQUIRE(m.validate().empty());
    std::mt19937 rng(42 + static_cast<int>(k));
    std::uniform_real_distribution<double> angle(-3.0, 3.0);
    std::uniform_real_distribution<double> coord(-100.0, 100.0);
    for (int trial = 0; trial < 300; ++trial) {
        const Vec3 u = randomAxis(rng, k == Kinematics::Mill4A);
        const auto sols = axisSolutions(m, u);
        INFO("kinematics " << toString(k) << " axis " << u.transpose());
        REQUIRE_FALSE(sols.empty());
        for (const IndexAngles& s : sols) {
            CHECK((toolAxis(m, s) - u).norm() < 1e-9);
        }
        // Forward then inverse from random rotary positions (within limits).
        IndexAngles q;
        q.a = k == Kinematics::Mill4A ? angle(rng)
                                      : (k == Kinematics::Mill5TableAC ? 0.6 * angle(rng) : 0.0);
        q.b = k == Kinematics::Mill4A || k == Kinematics::Mill5TableAC ? 0.0 : 0.6 * angle(rng);
        q.c = k == Kinematics::Mill4A ? 0.0 : angle(rng);
        const Vec3 v = toolAxis(m, q);
        const auto back = axisSolutions(m, v, &q);
        REQUIRE_FALSE(back.empty());
        const bool found = std::any_of(back.begin(), back.end(), [&](const IndexAngles& s) {
            return std::abs(s.a - q.a) + std::abs(s.b - q.b) + std::abs(s.c - q.c) < 1e-7;
        });
        CHECK(found);
        // Machine positions without TCP invert exactly.
        const Vec3 tip(coord(rng), coord(rng), coord(rng));
        const Vec3 x = machinePosition(m, q, tip, 80.0);
        CHECK((tipFromMachine(m, q, x, 80.0) - tip).norm() < 1e-9);
    }
}

TEST_CASE("kinematic conventions of the rotary axes", "[machine][kinematics]") {
    // 4th axis: A turns the part about X; a radial tool axis at angle t from +Z towards +Y is
    // brought up by A = t.
    const Machine m4 = testMachine(Kinematics::Mill4A);
    const double t = 0.7;
    const auto s4 = axisSolutions(m4, Vec3(0, std::sin(t), std::cos(t)));
    REQUIRE(s4.size() == 1);
    CHECK_THAT(s4[0].a, WithinAbs(t, 1e-12));
    CHECK(axisSolutions(m4, Vec3(0.1, 0, 1).normalized()).empty());
    // The point on the cylinder at that angle is moved to the top by the table.
    const Vec3 p(5, 20 * std::sin(t), 20 * std::cos(t));
    CHECK((machinePosition(m4, s4[0], p, 0) - Vec3(5, 0, 20)).norm() < 1e-12);

    // Head-table: B tilts the head towards +X for positive angles; the head machine moves the
    // tip by (pivot + length)(d - Z).
    const Machine hb = testMachine(Kinematics::Mill5HeadTableBC);
    IndexAngles q;
    q.b = kPi / 2;
    CHECK((toolAxis(hb, q) - Vec3::UnitX()).norm() < 1e-12);
    CHECK((machinePosition(hb, q, Vec3::Zero(), 50) - Vec3(200, 0, -200)).norm() < 1e-9);

    // Straight up keeps the rotating axis where it was (singular position).
    const Machine ac = testMachine(Kinematics::Mill5TableAC);
    IndexAngles near;
    near.c = 1.234;
    const auto up = axisSolutions(ac, Vec3::UnitZ(), &near);
    REQUIRE_FALSE(up.empty());
    CHECK_THAT(up[0].c, WithinAbs(1.234, 1e-12));
    CHECK_THAT(up[0].a, WithinAbs(0.0, 1e-12));

    // Unreachable: beyond the tilt limit of +-120 degrees.
    CHECK(axisSolutions(ac, Vec3(0, 0, -1)).empty());
}

TEST_CASE("rotary planning keeps the axes continuous", "[machine][kinematics]") {
    const Machine ac = testMachine(Kinematics::Mill5TableAC);
    // The tool axis circles the vertical at 30 degrees for two turns: C must keep turning
    // instead of jumping back at +-180 degrees.
    std::vector<Vec3> axes;
    const double tilt = degToRad(30.0);
    for (int k = 0; k <= 720; k += 5) {
        const double az = degToRad(k);
        axes.emplace_back(std::sin(tilt) * std::cos(az), std::sin(tilt) * std::sin(az),
                          std::cos(tilt));
    }
    const RotaryPlan plan = planRotary(ac, axes);
    REQUIRE(plan.errors.empty());
    double maxStep = 0.0;
    IndexAngles prev = plan.start;
    for (const IndexAngles& q : plan.angles) {
        maxStep = std::max({maxStep, std::abs(q.a - prev.a), std::abs(q.c - prev.c)});
        prev = q;
    }
    CHECK(maxStep < degToRad(5.01));
    CHECK_THAT(std::abs(plan.angles.back().c - plan.start.c), WithinAbs(4.0 * kPi, 1e-9));

    // With C limited to +-200 degrees the table has to flip to the other solution (A changes
    // sign, C jumps half a turn) on the way.
    Machine limited = ac;
    limited.axes.back().min = -200;
    limited.axes.back().max = 200;
    const RotaryPlan flipped = planRotary(limited, axes);
    REQUIRE(flipped.errors.empty());
    double flip = 0.0;
    prev = flipped.start;
    for (const IndexAngles& q : flipped.angles) {
        flip = std::max(flip, std::abs(q.c - prev.c));
        prev = q;
    }
    CHECK(flip > degToRad(90.0));
}

TEST_CASE("the rotary table unwinds on rapids clear above the part", "[machine][kinematics]") {
    // Two loops round a boss (swarf, DL-0085): the tool leans 14 degrees towards the boss's axis
    // all the way round, each loop a whole turn of C. C is limited to +-360 degrees.
    Machine ac = testMachine(Kinematics::Mill5TableAC);
    ac.axes.back().min = -360;
    ac.axes.back().max = 360;
    const double tilt = degToRad(14.0);
    const auto axisAt = [&](double az) {
        return Vec3(-std::sin(tilt) * std::cos(az), -std::sin(tilt) * std::sin(az), std::cos(tilt));
    };
    const auto build = [&](double betweenZ) {
        toolpath::Toolpath tp;
        tp.spindleSpeed = 100;
        tp.start = Vec3(20, 0, 50);
        const auto add = [&](toolpath::MoveKind kind, const Vec3& end, const Vec3& axis) {
            toolpath::Move m;
            m.kind = kind;
            m.feedClass = kind == toolpath::MoveKind::Rapid ? toolpath::FeedClass::Rapid
                                                            : toolpath::FeedClass::Cut;
            m.end = end;
            m.axis = axis;
            m.feedRate = kind == toolpath::MoveKind::Rapid ? 0.0 : 10.0;
            tp.moves.push_back(m);
        };
        for (int loop = 0; loop < 2; ++loop) {
            add(toolpath::MoveKind::Rapid, Vec3(20, 0, loop == 0 ? 50 : betweenZ), axisAt(0));
            add(toolpath::MoveKind::Rapid, Vec3(20, 0, 10), axisAt(0));
            for (int k = 5; k <= 360; k += 5) {
                const double az = degToRad(k);
                add(toolpath::MoveKind::Linear, Vec3(20 * std::cos(az), 20 * std::sin(az), 10),
                    axisAt(az));
            }
            add(toolpath::MoveKind::Rapid, Vec3(20, 0, betweenZ), axisAt(0));
        }
        return tp;
    };
    const auto swings = [&](const SolveResult& r) {
        return std::count_if(r.warnings.begin(), r.warnings.end(), [](const std::string& w) {
            return w.find("swing") != std::string::npos;
        });
    };
    // Back up at the start height between the loops: C goes back a turn there, at the rapid.
    const SolveResult clear = solve(ac, build(50.0), Transform3::Identity(), SolveOptions{});
    REQUIRE(clear.ok());
    CHECK(swings(clear) == 0);
    double rapidTurn = 0.0;
    for (std::size_t k = 1; k < clear.path.moves.size(); ++k) {
        const double step = std::abs(clear.rotary->angles[k].c - clear.rotary->angles[k - 1].c);
        if (clear.path.moves[k].kind == toolpath::MoveKind::Rapid) {
            rapidTurn = std::max(rapidTurn, step);
        } else {
            CHECK(step < degToRad(5.01));
        }
    }
    CHECK_THAT(rapidTurn, WithinAbs(2.0 * kPi, 1e-9));
    // The machine simulation's track has the same positions.
    const RotaryTrack track = rotaryTrack(ac, build(50.0), Transform3::Identity());
    REQUIRE(track.angles.size() == clear.rotary->angles.size());
    for (std::size_t k = 0; k < track.angles.size(); ++k) {
        CHECK_THAT(track.angles[k].c, WithinAbs(clear.rotary->angles[k].c, 1e-12));
    }
    // Between the loops only part of the way up (still near the part): no unwinding, so the
    // second loop runs out of C and swings within a cut, which the solve reports.
    const SolveResult low = solve(ac, build(30.0), Transform3::Identity(), SolveOptions{});
    CHECK(swings(low) == 1);
}

TEST_CASE("multi-axis solve with and without tool centre point control", "[machine][kinematics]") {
    // A radial plunge and a quarter turn around a cylinder of radius 20 about WCS X.
    toolpath::Toolpath tp;
    tp.spindleSpeed = 100;
    auto radial = [](double x, double ang, double r) {
        return std::pair{Vec3(x, r * std::sin(ang), r * std::cos(ang)),
                         Vec3(0, std::sin(ang), std::cos(ang))};
    };
    tp.start = radial(0, 0, 40).first;
    auto add = [&](toolpath::MoveKind kind, double x, double ang, double r) {
        toolpath::Move m;
        m.kind = kind;
        m.feedClass = kind == toolpath::MoveKind::Rapid ? toolpath::FeedClass::Rapid
                                                        : toolpath::FeedClass::Cut;
        std::tie(m.end, m.axis) = radial(x, ang, r);
        m.feedRate = kind == toolpath::MoveKind::Rapid ? 0.0 : 10.0;
        tp.moves.push_back(m);
    };
    add(toolpath::MoveKind::Rapid, 0, 0, 25);
    add(toolpath::MoveKind::Linear, 0, 0, 20);
    for (int k = 1; k <= 18; ++k) {
        add(toolpath::MoveKind::Linear, k * 0.5, degToRad(5.0 * k), 20);
    }
    REQUIRE(toolpath::isMultiAxis(tp));

    const Machine& m4 = genericMill4A();
    REQUIRE(m4.validate().empty());
    // The WCS sits on the A axis at the table centre.
    const Transform3 wcs = Transform3::Identity();

    SECTION("machine positions") {
        SolveOptions opt;
        opt.tcp = false;
        const SolveResult r = solve(m4, tp, wcs, opt);
        REQUIRE(r.ok());
        REQUIRE(r.rotary);
        CHECK(r.rotary->axes == "A");
        CHECK_FALSE(r.rotary->tcp);
        REQUIRE(r.rotary->angles.size() == r.path.moves.size());
        // Every move ends on top of the cylinder: X along, Y = 0, Z = radius (split points lie
        // on the chords of the CL path, just inside the radius).
        for (std::size_t k = 0; k < r.path.moves.size(); ++k) {
            const Vec3& e = r.path.moves[k].end;
            CHECK_THAT(e.y(), WithinAbs(0.0, 1e-9));
            CHECK(e.z() > 19.97);
            CHECK(e.z() < 25.0 + 1e-9);
        }
        CHECK_THAT(r.rotary->angles.back().a, WithinAbs(degToRad(90.0), 1e-9));
        // Linear X + A interpolation follows the cylinder: the chords of the CL path are what
        // splitting compares against, so moves of 5 degrees at radius 20 (sagitta 0.019 mm)
        // are split once at a tolerance of 0.01 mm.
        CHECK(r.path.moves.size() > tp.moves.size());
        // Inverse-time data: durations sum to the CL feed length / feed.
        double t = 0.0;
        for (double d : r.rotary->times) {
            t += d;
        }
        double len = 0.0;
        Vec3 p = tp.moves[0].end;
        for (std::size_t k = 1; k < tp.moves.size(); ++k) {
            len += (tp.moves[k].end - p).norm();
            p = tp.moves[k].end;
        }
        CHECK_THAT(t, WithinAbs(len / 10.0, 1e-9));
    }

    SECTION("tool centre point control") {
        const SolveResult r = solve(m4, tp, wcs, SolveOptions{});
        REQUIRE(r.ok());
        REQUIRE(r.rotary);
        CHECK(r.rotary->tcp);
        REQUIRE(r.path.moves.size() == tp.moves.size());
        for (std::size_t k = 0; k < tp.moves.size(); ++k) {
            CHECK((r.path.moves[k].end - tp.moves[k].end).norm() < 1e-12);
            CHECK((r.path.moves[k].axis - tp.moves[k].axis).norm() < 1e-12);
        }
    }

    SECTION("machines without the needed rotary axes") {
        CHECK_FALSE(solve(genericMill3(), tp, wcs, SolveOptions{}).ok());
        // A tool tilted about Y cannot be reached with A only.
        toolpath::Toolpath bad = tp;
        bad.moves.back().axis = Vec3(0.3, 0, 1).normalized();
        CHECK_FALSE(solve(m4, bad, wcs, SolveOptions{}).ok());
        // ...but the head-table machine reaches it.
        CHECK(solve(genericMill5HeadTableBC(), bad, wcs, SolveOptions{}).ok());
    }
}

TEST_CASE("built-in multi-axis machines and components", "[machine][kinematics]") {
    for (const Machine* m : builtinMachines()) {
        INFO(m->id);
        CHECK(m->validate().empty());
        CHECK(machineFromJson(toJson(*m), m->id) == *m);
        CHECK(builtinMachine(m->id) == m);
    }
    CHECK(genericMill4A().rotaryAxes() == "A");
    CHECK(genericMill5TableAC().rotaryAxes() == "AC");
    CHECK(genericMill5HeadTableBC().rotaryAxes() == "BC");
    CHECK(genericMill5HeadTableBC().pivotLength == 150.0);
    CHECK_FALSE(genericMill5TableAC().components.empty());
    const Component& nose = genericMill5TableAC().components.front();
    CHECK(nose.attach == Component::Attach::Head);
    CHECK(nose.shape == Component::Shape::Cylinder);
    CHECK_THAT(nose.convex().bounds().max.z(), WithinAbs(120.0, 1e-12));
}

TEST_CASE("index positions for a tool orientation on every kinematic class",
          "[machine][kinematics][orientation]") {
    // Table-table machines choose as for 3+2 setups.
    const Machine ac = testMachine(Kinematics::Mill5TableAC);
    const auto front = indexSolution(ac, -Vec3::UnitY());
    REQUIRE(front);
    CHECK(*front == *indexAngles(ac, -Vec3::UnitY()));
    CHECK_THAT(front->a, WithinAbs(-kPi / 2, 1e-12));
    CHECK_FALSE(indexSolution(ac, -Vec3::UnitZ())); // beyond A +-120
    for (const Kinematics k : {Kinematics::Mill4A, Kinematics::Mill5HeadTableBC,
                               Kinematics::Mill5HeadBC, Kinematics::Mill5TableBC}) {
        const Machine m = testMachine(k);
        const Vec3 axis = Vec3(0, 1, 1).normalized();
        const auto s = indexSolution(m, axis);
        INFO(toString(k));
        REQUIRE(s);
        CHECK((toolAxis(m, *s) - axis).norm() < 1e-9);
    }
    // 4th axis: only axes across X; head-table B/C: the smaller tilt.
    CHECK_FALSE(indexSolution(testMachine(Kinematics::Mill4A), Vec3::UnitX()));
    const auto side = indexSolution(testMachine(Kinematics::Mill5HeadTableBC), Vec3::UnitX());
    REQUIRE(side);
    CHECK_THAT(side->b, WithinAbs(kPi / 2, 1e-12));
    CHECK_THAT(side->c, WithinAbs(0.0, 1e-12));
    // Without rotary axes only up.
    const Machine three = testMachine(Kinematics::Mill3);
    CHECK(indexSolution(three, Vec3::UnitZ()));
    CHECK_FALSE(indexSolution(three, Vec3::UnitX()));
}

TEST_CASE("positional solve in an operation's own frame", "[machine][kinematics][orientation]") {
    // A toolpath along its frame's Z: a line, an arc and a drilling cycle.
    toolpath::Toolpath tp;
    tp.spindleSpeed = 100;
    tp.start = Vec3(0, 0, 10);
    {
        toolpath::Move m;
        m.kind = toolpath::MoveKind::Linear;
        m.end = Vec3(10, 0, -1);
        m.feedRate = 5;
        tp.moves.push_back(m);
        m.kind = toolpath::MoveKind::ArcCCW;
        m.end = Vec3(-10, 0, -1);
        m.center = Vec3(0, 0, -1);
        tp.moves.push_back(m);
        toolpath::DrillCycle c;
        c.topZ = 0;
        c.bottomZ = -5;
        c.retractZ = 2;
        c.feedRate = 2;
        tp.cycles.push_back(c);
        m = {};
        m.kind = toolpath::MoveKind::Cycle;
        m.end = Vec3(5, 5, 2);
        m.cycle = 0;
        tp.moves.push_back(m);
    }
    // The frame: Z along -Y (the front face), origin at (0, -20, 30) in the table frame.
    Transform3 frame = Transform3::Identity();
    frame.linear().col(0) = Vec3::UnitX();
    frame.linear().col(1) = Vec3::UnitZ();
    frame.linear().col(2) = -Vec3::UnitY();
    frame.translation() = Vec3(0, -20, 30);

    SECTION("trunnion: arcs and cycles stay, positions follow the work offset") {
        const Machine ac = testMachine(Kinematics::Mill5TableAC);
        const IndexAngles index = *indexSolution(ac, frame.linear().col(2));
        // The setup's offset: its origin (0, 0, 50) measured with the table at 0.
        const WorkOrigin setupOrigin{Vec3(0, 0, 50), {}};
        const SolveResult r = solveIndexed(ac, tp, frame, index, setupOrigin, {});
        REQUIRE(r.ok());
        REQUIRE(r.index);
        CHECK(*r.index == index);
        CHECK(r.path.moves[1].isArc());
        CHECK(r.path.moves[2].kind == toolpath::MoveKind::Cycle);
        // Machine position of a tip less the origin's: T p - o.
        const Mat3 T = tableRotation(ac, index);
        const Vec3 tip = frame * Vec3(10, 0, -1);
        CHECK((r.path.moves[0].end - (T * tip - Vec3(0, 0, 50))).norm() < 1e-9);
        // The tool is along machine Z: depths along the frame's Z are the program's.
        CHECK_THAT(r.path.moves[0].end.z() - r.path.start.z(), WithinAbs(-11.0, 1e-9));
        CHECK_THAT(r.path.cycles[0].bottomZ - r.path.cycles[0].retractZ, WithinAbs(-7.0, 1e-9));
        // Each move carries the tool axis in the table frame (for the back-plot).
        CHECK((r.path.moves[0].axis - Vec3(0, -1, 0)).norm() < 1e-9);

        // A work offset of its own at the frame origin: the origin is the program zero.
        const SolveResult own =
            solveIndexed(ac, tp, frame, index, {frame.translation(), index}, {});
        REQUIRE(own.ok());
        CHECK_THAT(own.path.moves[0].end.z(), WithinAbs(-1.0, 1e-9));
        CHECK_THAT(own.path.moves[0].end.head<2>().norm(), WithinAbs(10.0, 1e-9));
    }
    SECTION("tilted head: lines, expanded cycles, and the head offset") {
        const Machine bc = testMachine(Kinematics::Mill5HeadTableBC);
        const IndexAngles index = *indexSolution(bc, frame.linear().col(2));
        SolveOptions options;
        options.toolLength = 60;
        const SolveResult r = solveIndexed(bc, tp, frame, index, {}, options);
        REQUIRE(r.ok());
        for (const auto& m : r.path.moves) {
            CHECK_FALSE(m.isArc());
            CHECK(m.kind != toolpath::MoveKind::Cycle);
        }
        const Vec3 tip = frame * Vec3(10, 0, -1);
        CHECK((r.path.moves[0].end - machinePosition(bc, index, tip, 60)).norm() < 1e-9);
        // Tapping cannot follow a tilted head.
        toolpath::Toolpath tap = tp;
        tap.cycles[0].type = toolpath::CycleType::Tap;
        CHECK_FALSE(solveIndexed(bc, tap, frame, index, {}, options).ok());
    }
    SECTION("rotary positions that do not give the frame's axis are refused") {
        const Machine ac = testMachine(Kinematics::Mill5TableAC);
        CHECK_FALSE(solveIndexed(ac, tp, frame, IndexAngles{}, {}, {}).ok());
    }
}
