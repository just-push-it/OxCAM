// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "RunPost.hpp"

#include <ocam/machine/Kinematics.hpp>
#include <ocam/post/Post.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <sstream>

using namespace ocam;
using namespace ocam::post;
using namespace ocam::post::testing;
using Catch::Matchers::ContainsSubstring;

namespace {

toolpath::Move move(toolpath::MoveKind kind, const Vec3& end, const Vec3& axis, double feed) {
    toolpath::Move m;
    m.kind = kind;
    m.feedClass =
        kind == toolpath::MoveKind::Rapid ? toolpath::FeedClass::Rapid : toolpath::FeedClass::Cut;
    m.end = end;
    m.axis = axis.normalized();
    m.feedRate = kind == toolpath::MoveKind::Rapid ? 0.0 : feed;
    return m;
}

/// A groove wound around a cylinder of radius 20 about X: radial tool axis, one and a half
/// turns while moving 30 mm along X.
toolpath::Toolpath wrappedGroove() {
    toolpath::Toolpath tp;
    tp.toolNumber = 3;
    tp.spindleSpeed = 8000.0 / 60.0;
    tp.start = Vec3(0, 0, 35);
    auto at = [](double x, double ang, double r) {
        return std::pair{Vec3(x, r * std::sin(ang), r * std::cos(ang)),
                         Vec3(0, std::sin(ang), std::cos(ang))};
    };
    auto [p0, u0] = at(0, 0, 25);
    tp.moves.push_back(move(toolpath::MoveKind::Rapid, p0, u0, 0));
    auto [p1, u1] = at(0, 0, 18);
    tp.moves.push_back(move(toolpath::MoveKind::Linear, p1, u1, 5.0));
    const int n = 270;
    for (int k = 1; k <= n; ++k) {
        const double t = static_cast<double>(k) / n;
        auto [p, u] = at(30.0 * t, 3.0 * kPi * t, 18);
        tp.moves.push_back(move(toolpath::MoveKind::Linear, p, u, 15.0));
    }
    auto [p2, u2] = at(30, 3.0 * kPi, 30);
    tp.moves.push_back(move(toolpath::MoveKind::Rapid, p2, u2, 0));
    return tp;
}

/// A ball tool rolling around a hemisphere of radius 25 with the axis tilted 20 degrees more
/// than the surface normal.
toolpath::Toolpath tiltedRing() {
    toolpath::Toolpath tp;
    tp.toolNumber = 5;
    tp.spindleSpeed = 12000.0 / 60.0;
    const double polar = degToRad(40.0), lead = degToRad(20.0);
    auto at = [&](double az) {
        const Vec3 n(std::sin(polar) * std::cos(az), std::sin(polar) * std::sin(az),
                     std::cos(polar));
        const Vec3 axis(std::sin(polar + lead) * std::cos(az),
                        std::sin(polar + lead) * std::sin(az), std::cos(polar + lead));
        const Vec3 centre = 28.0 * n; // ball radius 3 on a sphere of radius 25
        return std::pair{Vec3(centre - 3.0 * axis), axis};
    };
    auto [p0, u0] = at(0);
    tp.start = p0 + 20.0 * u0;
    tp.moves.push_back(move(toolpath::MoveKind::Rapid, p0 + 5.0 * u0, u0, 0));
    tp.moves.push_back(move(toolpath::MoveKind::Linear, p0, u0, 5.0));
    for (int k = 1; k <= 120; ++k) {
        auto [p, u] = at(2.0 * kPi * k / 120.0);
        tp.moves.push_back(move(toolpath::MoveKind::Linear, p, u, 20.0));
    }
    auto [p1, u1] = at(2.0 * kPi);
    tp.moves.push_back(move(toolpath::MoveKind::Rapid, p1 + 20.0 * u1, u1, 0));
    return tp;
}

} // namespace

TEST_CASE("4-axis program with machine positions and inverse-time feeds", "[post][multiaxis]") {
    const machine::Machine& m4 = machine::genericMill4A();
    machine::SolveOptions opt;
    opt.tcp = false;
    const toolpath::Toolpath cl = wrappedGroove();
    const machine::SolveResult solved = machine::solve(m4, cl, Transform3::Identity(), opt);
    REQUIRE(solved.ok());
    REQUIRE(solved.rotary);
    PostJob job{"wrap", solved.path, "ball 6", 1, std::nullopt, "", solved.rotary};
    const PostScript& cfg = builtinPost("fanuc-generic-mill-4axis");
    CHECK_FALSE(cfg.manifest().tcp);
    const std::string gcode = postJobs({job}, cfg, "wrap", 4000);
    INFO(gcode.substr(0, 2000));
    CHECK_THAT(gcode, ContainsSubstring("G93"));
    CHECK_THAT(gcode, ContainsSubstring("A540."));
    CHECK_THAT(gcode, ContainsSubstring("G43 H3"));
    CHECK(gcode.find("G43.4") == std::string::npos);
    // Machine positions stay on top of the cylinder: Z is the radius.
    CHECK_THAT(gcode, ContainsSubstring("Z18."));

    machine::IndexAngles home;
    BackplotOptions bo;
    bo.machine = &m4;
    const BackplotResult plotted = backplot(gcode, bo);
    double dev = 0.0;
    CHECK(compareBackplot({solved.path}, plotted.path, 2e-3, &dev).value_or("") == "");
    CHECK(dev < 2e-3);

    // A 3-axis job afterwards turns the table back to zero after homing Z.
    toolpath::Toolpath flat;
    flat.toolNumber = 3;
    flat.spindleSpeed = cl.spindleSpeed;
    flat.start = Vec3(0, 0, 30);
    flat.moves.push_back(move(toolpath::MoveKind::Rapid, Vec3(10, 0, 22), Vec3::UnitZ(), 0));
    flat.moves.push_back(move(toolpath::MoveKind::Linear, Vec3(10, 0, 19), Vec3::UnitZ(), 5));
    PostJob after{"flat", flat, "ball 6", 1, home, "A"};
    const std::string two = postJobs({job, after}, cfg, "two", 4001);
    const auto zero = two.rfind("A0.");
    REQUIRE(zero != std::string::npos);
    CHECK(two.rfind("G91 G28 Z0.", zero) != std::string::npos);
    CHECK(two.find("G94", two.rfind("G93")) != std::string::npos);
}

TEST_CASE("5-axis program with tool centre point control", "[post][multiaxis]") {
    const machine::Machine& m5 = machine::genericMill5TableAC();
    const toolpath::Toolpath cl = tiltedRing();
    const machine::SolveResult solved =
        machine::solve(m5, cl, Transform3::Identity(), machine::SolveOptions{});
    REQUIRE(solved.ok());
    REQUIRE(solved.rotary);
    CHECK(solved.rotary->axes == "AC");
    PostJob job{"ring", solved.path, "ball 6", 1, std::nullopt, "", solved.rotary};
    const NcProgram program = lower({job}, "ring", 5000);
    const std::string gcode = runPost(mill(), program);
    INFO(gcode.substr(0, 2000));
    CHECK_THAT(gcode, ContainsSubstring("G43.4 H5"));
    CHECK_THAT(gcode, ContainsSubstring("G49"));
    CHECK(gcode.find("G93") == std::string::npos);
    // The tool axis is 60 degrees from vertical: A tilts by 60 degrees.
    CHECK((gcode.find("A60.") != std::string::npos || gcode.find("A-60.") != std::string::npos));
    // C turns continuously through a full revolution.
    const auto c = gcode.rfind(" C");
    REQUIRE(c != std::string::npos);

    BackplotOptions bo;
    bo.machine = &m5;
    double dev = 0.0;
    CHECK(compareBackplot({solved.path}, backplot(gcode, bo).path, 2e-3, &dev).value_or("") == "");
    CHECK(dev < 2e-3);

    // The post API carries the rotary positions and the TCP flag.
    const NcProgram back = ncProgramFromJson(Json::parse(dumpJson(toJson(program))));
    CHECK(runPost(mill(), back) == gcode);

    // A wrong axis is caught by the verification.
    toolpath::Toolpath tampered = solved.path;
    tampered.moves[10].axis = Vec3(0, 0, 1);
    CHECK(compareBackplot({tampered}, backplot(gcode, bo).path, 2e-3).has_value());
}

TEST_CASE("Haas-style post: simultaneous 4- and 5-axis without tool centre point control",
          "[post][multiaxis][haas]") {
    const PostScript& haas = builtinPost("haas-classic-mill");
    machine::SolveOptions opt;
    opt.tcp = false;
    for (const bool fiveAxis : {false, true}) {
        CAPTURE(fiveAxis);
        const machine::Machine& m =
            fiveAxis ? machine::genericMill5TableAC() : machine::genericMill4A();
        const machine::SolveResult solved = machine::solve(
            m, fiveAxis ? tiltedRing() : wrappedGroove(), Transform3::Identity(), opt);
        REQUIRE(solved.ok());
        REQUIRE(solved.rotary);
        PostJob job{"wound", solved.path, "ball 6", 1, std::nullopt, "", solved.rotary};
        job.tool = NcTool{solved.path.toolNumber, "ball 6"};
        job.tool->gaugeLength = 50.0;
        NcProgram program = lower({job}, "wound", 1000, lowerOptionsFor(haas.manifest()));
        program.machine = ncMachine(m);
        CancelToken cancel;
        const std::string gcode = haas.run(program, haas.optionValues({}), cancel).text;
        INFO(gcode.substr(0, 2500));
        CHECK(gcode.find("G43.4") == std::string::npos);
        // The program says which tool length its positions are for.
        CHECK_THAT(gcode, ContainsSubstring("(NO TCP: VALID ONLY FOR T"));
        CHECK_THAT(gcode, ContainsSubstring(" GAUGE LENGTH 50. MM)"));
        // The brakes are released just before the first G93 move and engaged after the toolpath.
        const std::string release = fiveAxis ? "\nM11\nM13\n" : "\nM11\n";
        const std::string engage = fiveAxis ? "\nM10\nM12\n" : "\nM10\n";
        const auto m11 = gcode.find(release);
        const auto g93 = gcode.find("G93");
        REQUIRE(m11 != std::string::npos);
        REQUIRE(g93 != std::string::npos);
        CHECK(gcode.find('\n', m11 + release.size()) > g93); // the next block starts G93
        const auto m10 = gcode.find(engage, g93);
        REQUIRE(m10 != std::string::npos);
        // Inside: every block a G1 feed move with its own F, no rapids.
        std::istringstream lines(gcode.substr(m11 + release.size(), m10 - m11 - release.size()));
        int blocks = 0;
        for (std::string line; std::getline(lines, line);) {
            CAPTURE(line);
            CHECK(line.find("G0 ") == std::string::npos);
            CHECK(line.find(" F") != std::string::npos);
            ++blocks;
        }
        CHECK(blocks >= 120);
        // Positioning moves of the toolpath take the time set: 1 s, F60.
        CHECK_THAT(gcode, ContainsSubstring(" F60."));

        BackplotOptions bo;
        bo.machine = &m;
        const BackplotResult plotted = backplot(gcode, bo);
        CHECK(plotted.warnings.empty());
        double dev = 0.0;
        CHECK(compareBackplot({solved.path}, plotted.path, 2e-3, &dev).value_or("") == "");
        CHECK(dev < 2e-3);
    }
}

TEST_CASE("Haas-style post: a trunnion's C axis as B", "[post][multiaxis][haas]") {
    const PostScript& haas = builtinPost("haas-classic-mill");
    const machine::Machine& m = machine::genericMill5TableAC();
    machine::SolveOptions opt;
    opt.tcp = false;
    const machine::SolveResult solved =
        machine::solve(m, tiltedRing(), Transform3::Identity(), opt);
    REQUIRE(solved.ok());
    PostJob job{"ring", solved.path, "ball 6", 1, std::nullopt, "", solved.rotary};
    NcProgram program = lower({job}, "ring", 1000, lowerOptionsFor(haas.manifest()));
    program.machine = ncMachine(m);
    CancelToken cancel;
    const std::string c = haas.run(program, haas.optionValues({}), cancel).text;
    const std::string b =
        haas.run(program, haas.optionValues({{"fifthAxisLetter", "B"}}), cancel).text;
    CHECK(c.find(" B") == std::string::npos);
    CHECK(b.find(" C") == std::string::npos);
    CHECK(b.find("\nC") == std::string::npos);
    // The same program with every C word a B word.
    std::string renamed = c;
    for (std::size_t at = 1; at < renamed.size(); ++at) {
        if (renamed[at] == 'C' && (renamed[at - 1] == ' ' || renamed[at - 1] == '\n')) {
            renamed[at] = 'B';
        }
    }
    CHECK(renamed == b);
}
