// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "RunPost.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/geom2d/Clip.hpp>
#include <ocam/post/Post.hpp>
#include <ocam/strategies/Strategies.hpp>
#include <ocam/strategies/Turning.hpp>
#include <ocam/toolpath/PathBuilder.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdlib>
#include <filesystem>
#include <sstream>

using namespace ocam;
using namespace ocam::post;
using namespace ocam::post::testing;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

/// One left-hand tap at (10, 10), on a mill.
PostJob leftHandTap() {
    toolpath::Toolpath path;
    path.toolNumber = 1;
    path.spindleSpeed = 10.0;
    path.spindleReverse = true;
    path.start = Vec3(10.0, 10.0, 5.0);
    toolpath::DrillCycle cycle;
    cycle.type = toolpath::CycleType::TapLeft;
    cycle.bottomZ = -10.0;
    cycle.retractZ = 5.0;
    cycle.feedRate = 15.0;
    cycle.pitch = 1.5;
    path.cycles.push_back(cycle);
    toolpath::Move hole;
    hole.kind = toolpath::MoveKind::Cycle;
    hole.cycle = 0;
    hole.end = Vec3(10.0, 10.0, 5.0);
    path.moves.push_back(hole);
    PostJob job;
    job.name = "Tap";
    job.path = path;
    return job;
}

toolpath::Toolpath loadGoldenToolpath(const std::string& name) {
    const auto path = std::filesystem::path(OCAM_GOLDEN_DIR) / (name + ".json");
    return toolpath::toolpathFromJson(parseJson(readFileBytes(path), name));
}

std::vector<std::string> lines(const std::string& text) {
    std::vector<std::string> out;
    std::istringstream in(text);
    for (std::string l; std::getline(in, l);) {
        out.push_back(l);
    }
    return out;
}

// Compares G-code line by line; numbers may differ by one unit in the last printed digit.
void checkGoldenGcode(const std::string& name, const std::string& gcode) {
    const auto path = std::filesystem::path(OCAM_GOLDEN_DIR) / (name + ".nc");
    const char* update = std::getenv("OCAM_UPDATE_GOLDEN");
    if (update && std::string(update) == "1") {
        writeFileAtomic(path, gcode);
        SUCCEED("golden G-code updated");
        return;
    }
    INFO("golden file " << path.string());
    REQUIRE(std::filesystem::exists(path));
    const auto expected = lines(readFileBytes(path));
    const auto actual = lines(gcode);
    REQUIRE(expected.size() == actual.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        if (expected[i] == actual[i]) {
            continue;
        }
        // Allow last-digit rounding differences between platforms.
        const auto ew = backplot(expected[i]).path;
        const auto aw = backplot(actual[i]).path;
        INFO("line " << i + 1 << ": '" << expected[i] << "' vs '" << actual[i] << "'");
        CHECK(ew.moves.size() == aw.moves.size());
        for (std::size_t k = 0; k < std::min(ew.moves.size(), aw.moves.size()); ++k) {
            CHECK((ew.moves[k].end - aw.moves[k].end).norm() <= 1.5e-3);
        }
    }
}

} // namespace

TEST_CASE("exact formatting of a small program", "[post]") {
    toolpath::Toolpath tp;
    tp.toolNumber = 3;
    tp.spindleSpeed = 8000.0 / 60.0;
    tp.start = Vec3(10, 0, 15);
    toolpath::PathBuilder b(tp);
    b.rapid(Vec3(10, 0, 2));
    b.feed(Vec3(10, 0, -1), toolpath::FeedClass::Plunge, 300.0 / 60.0);
    b.feed(Vec3(20.5, 0, -1), toolpath::FeedClass::Cut, 1200.0 / 60.0);
    b.arc(Vec3(20.5, 10, -1), Vec2(20.5, 5), false, toolpath::FeedClass::Cut, 1200.0 / 60.0);
    b.retractTo(15);
    toolpath::DrillCycle c;
    c.type = toolpath::CycleType::Peck;
    c.topZ = 0;
    c.bottomZ = -12;
    c.retractZ = 2;
    c.peck = 4;
    c.feedRate = 150.0 / 60.0;
    b.cycle({30, 0}, c);
    b.cycle({40, 0}, c);
    b.retractTo(15);

    const std::string gcode =
        postJobs({{"Test op", tp, "6mm end mill", 1}}, mill(), "Bracket", 1001);
    const std::string expected = "%\n"
                                 "O1001 (BRACKET)\n"
                                 "N10 G17 G21 G40 G49 G80 G90 G94\n"
                                 "(TEST OP)\n"
                                 "N20 T3 M06 (6MM END MILL)\n"
                                 "N30 G0 G54 X10. Y0.\n"
                                 "N40 S8000 M03\n"
                                 "N50 G43 H3 Z15.\n"
                                 "N60 M08\n"
                                 "N70 Z2.\n"
                                 "N80 G1 Z-1. F300.\n"
                                 "N90 X20.5 F1200.\n"
                                 "N100 G3 X20.5 Y10. I0. J5.\n"
                                 "N110 G0 Z15.\n"
                                 "N120 G99 G83 X30. Y0. Z-12. R2. Q4. F150.\n"
                                 "N130 X40.\n"
                                 "N140 G80\n"
                                 "N150 G0 Z15.\n"
                                 "N160 M09\n"
                                 "N170 M05\n"
                                 "N180 G91 G28 Z0.\n"
                                 "N190 G90\n"
                                 "N200 M30\n"
                                 "%\n";
    CHECK(gcode == expected);
}

TEST_CASE("golden G-code for the golden toolpaths", "[post][golden]") {
    for (const char* name :
         {"facing_rect_zigzag", "pocket_rect_island", "contour_rect_outside", "drill_mixed"}) {
        INFO(name);
        const toolpath::Toolpath tp = loadGoldenToolpath(name);
        checkGoldenGcode(name, postJobs({{name, tp, "test tool", 1}}, mill(), name, 1000));
    }
}

TEST_CASE("back-plot of posted toolpaths matches the CL within 1e-4 mm", "[post][backplot]") {
    for (const char* name :
         {"facing_rect_zigzag", "pocket_rect_island", "contour_rect_outside", "drill_mixed"}) {
        INFO(name);
        const toolpath::Toolpath tp = loadGoldenToolpath(name);
        const std::string gcode = postJobs({{name, tp, "", 1}}, fineMill(), name, 1000);
        const BackplotResult plotted = backplot(gcode);
        CHECK(plotted.warnings.empty());
        double dev = 0.0;
        const auto diff = compareBackplot({tp}, plotted.path, 1e-4, &dev);
        CHECK(diff.value_or("") == "");
        CHECK(dev <= 1e-4);

        // The production post (3 decimals) stays within half a micron plus arc-centre rounding.
        const auto coarse = backplot(postJobs({{name, tp, "", 1}}, mill(), name, 1000));
        CHECK(compareBackplot({tp}, coarse.path, 1.5e-3).value_or("") == "");
    }
}

TEST_CASE("dialect options: radius arcs, inch, expanded cycles, no line numbers",
          "[post][backplot]") {
    const toolpath::Toolpath pocket = loadGoldenToolpath("pocket_rect_island");
    const toolpath::Toolpath drill = loadGoldenToolpath("drill_mixed");

    const PostScript r = millWith("this.decimals = { ...this.decimals, linear: 4, feed: 2 };"
                                  "this.arcFormat = 'r';"
                                  "this.maxArcSweep = 90;"); // forces splitting
    const std::string rCode = postJobs({{"p", pocket, "", 1}}, r, "r", 1);
    CHECK_THAT(rCode, ContainsSubstring(" R"));
    // Radius format rebuilds centres from rounded end points and radius: less precise than IJK.
    CHECK(compareBackplot({pocket}, backplot(rCode).path, 1e-3).value_or("") == "");

    const PostScript inch = millWith("this.decimals = { ...this.decimals, linear: 5, feed: 2 };"
                                     "this.units = 'in';"
                                     "this.safeStart = 'G17 G20 G40 G49 G80 G90 G94';");
    const std::string inchCode = postJobs({{"p", pocket, "", 1}}, inch, "in", 1);
    CHECK_THAT(inchCode, ContainsSubstring("G20"));
    // 5 decimals in inches is a 0.254 um step; centres are rounded independently of end points.
    CHECK(compareBackplot({pocket}, backplot(inchCode).path, 4e-4).value_or("") == "");

    // No canned cycles declared: OxCAM expands them before the post.
    const PostScript expanded = millWith("this.decimals = { ...this.decimals, linear: 4, feed: 2 };"
                                         "this.lineNumbers = null;",
                                         "cycles: [],");
    const std::string flat = postJobs({{"d", drill, "", 1}}, expanded, "flat", 1);
    CHECK(flat.find("G83") == std::string::npos);
    CHECK(flat.find("G81") == std::string::npos);
    CHECK(flat.find("N10") == std::string::npos);
    CHECK(compareBackplot({drill}, backplot(flat).path, 1e-4).value_or("") == "");
}

TEST_CASE("multi-tool programs and tool changes", "[post][backplot]") {
    toolpath::Toolpath a = loadGoldenToolpath("facing_rect_zigzag");
    toolpath::Toolpath b = loadGoldenToolpath("drill_mixed");
    a.toolNumber = 1;
    b.toolNumber = 5;
    const std::string gcode =
        postJobs({{"face", a, "face mill", 1}, {"drill", b, "drill", 2}}, fineMill(), "multi", 2);
    CHECK_THAT(gcode, ContainsSubstring("T5 M06"));
    CHECK_THAT(gcode, ContainsSubstring("G55"));
    const BackplotResult plotted = backplot(gcode);
    CHECK(plotted.toolChanges == std::vector<int>{1, 5});
    CHECK(compareBackplot({a, b}, plotted.path, 1e-4).value_or("") == "");

    // A missing job is detected.
    toolpath::Toolpath shifted = b;
    shifted.start.x() += 1.0;
    CHECK(compareBackplot({a, shifted}, plotted.path, 1e-4).has_value());
}

TEST_CASE("cutter compensation in control", "[post]") {
    tooling::Tool tool = tooling::makeFlatEndMill(4, 10, 25, 70);
    CancelToken cancel;
    NullProgress progress;
    const strategies::Context ctx{
        tool, strategies::Feeds{}, strategies::Heights{}, 0.002, cancel, progress};
    strategies::ContourParams p;
    p.compensation = strategies::CompensationMode::Control;
    p.leadLength = 5.0;
    const auto tp =
        strategies::contour({{geom2d::Polyline::rectangle({0, 0}, {50, 30}), {}}}, 0, -5, p, ctx);
    std::vector<std::string> warnings;
    const std::string gcode = postJobs({{"profile", tp, "", 1}}, mill(), "comp", 3, &warnings);
    CHECK(warnings.empty());
    const auto g41 = gcode.find("G41 D4 "); // on the straight lead-in line
    REQUIRE(g41 != std::string::npos);
    CHECK(gcode.find("G40", g41) != std::string::npos);

    p.leadLength = 0.0; // arc straight after the plunge: compensation cannot start there
    const auto noLine =
        strategies::contour({{geom2d::Polyline::rectangle({0, 0}, {50, 30}), {}}}, 0, -5, p, ctx);
    warnings.clear();
    postJobs({{"profile", noLine, "", 1}}, mill(), "comp", 3, &warnings);
    CHECK_FALSE(warnings.empty());
}

TEST_CASE("back-plotter handles common G-code forms", "[post][backplot]") {
    const auto r = backplot("%\nO0001 (TEST)\nG21 G90 G17\nG0 X0 Y0 Z5\nG1 Z-1 F600 ; plunge\n"
                            "G91 X10\nG90 G2 X20 Y0 R5 F1200\nG4 P250\nG0 Z5\nG98 G81 X30 Y0 Z-5 "
                            "R1 F100\nX40\nG80\nM30\n%\n");
    CHECK(r.warnings.empty());
    const auto& m = r.path.moves;
    REQUIRE(m.size() == 8);
    CHECK(m[2].end == Vec3(10, 0, -1)); // incremental
    CHECK(m[3].kind == toolpath::MoveKind::ArcCW);
    CHECK((m[3].center - Vec3(15, 0, -1)).norm() < 1e-12); // R5 over a 10 mm chord: centre on it
    CHECK_THAT(m[3].feedRate * 60, WithinAbs(1200.0, 1e-9));
    CHECK(m[4].kind == toolpath::MoveKind::Dwell);
    CHECK_THAT(m[4].dwell, WithinAbs(0.25, 1e-12));
    CHECK(m[6].kind == toolpath::MoveKind::Cycle);
    CHECK(m[6].end.z() == 5.0); // G98: back to the initial level
    CHECK(r.path.cycles.size() == 1);

    const auto inch = backplot("G20 G90\nG0 X1 Y2 Z0.5\n");
    CHECK((inch.path.moves.back().end - Vec3(25.4, 50.8, 12.7)).norm() < 1e-9);

    CHECK_THROWS_AS(backplot("G1 X10 Y#5\n"), Error);
    CHECK_THROWS_AS(backplot("G0 X0 Y0\nG2 X10 Y0 F100\n"), Error); // arc without centre
    // G18: arcs in the ZX plane, the centre from I and K.
    const auto zx = backplot("G18\nG0 X0 Y0 Z0\nG2 X10 Z10 I0 K10 F100\n");
    CHECK(zx.warnings.empty());
    REQUIRE(zx.path.moves.size() == 2);
    CHECK(zx.path.moves.back().plane == toolpath::ArcPlane::ZX);
    CHECK((zx.path.moves.back().center - Vec3(0, 0, 10)).norm() < 1e-12);
}

TEST_CASE("arcs in the ZX and YZ planes post as G18 and G19 and back-plot", "[post][backplot]") {
    // A finishing profile: a half circle over a dome in the ZX plane, one in the YZ plane, then
    // an XY arc and a canned cycle, which need G17 again.
    toolpath::Toolpath tp;
    tp.toolNumber = 1;
    tp.start = Vec3(0, 0, 20);
    toolpath::PathBuilder b(tp);
    b.rapid(Vec3(-10, 0, 20));
    b.feed(Vec3(-10, 0, 0), toolpath::FeedClass::Plunge, 5.0);
    toolpath::Move zx;
    zx.kind = toolpath::MoveKind::ArcCCW; // over the top: seen from +Y, Z right and X up (G18)
    zx.feedClass = toolpath::FeedClass::Cut;
    zx.end = Vec3(10, 0, 0);
    zx.center = Vec3(0, 0, 0);
    zx.plane = toolpath::ArcPlane::ZX;
    zx.feedRate = 20.0;
    tp.moves.push_back(zx);
    toolpath::Move yz = zx;
    yz.kind = toolpath::MoveKind::ArcCCW;
    yz.end = Vec3(10, 10, 5);
    yz.center = Vec3(10, 5, 5);
    yz.plane = toolpath::ArcPlane::YZ;
    tp.moves.push_back(yz);
    toolpath::Move xy = zx;
    xy.kind = toolpath::MoveKind::ArcCCW;
    xy.end = Vec3(0, 20, 5);
    xy.center = Vec3(0, 10, 5);
    xy.plane = toolpath::ArcPlane::XY;
    tp.moves.push_back(xy);
    const std::string gcode = postJobs({{"profile", tp, "", 1}}, fineMill(), "arcs", 1000);
    CHECK_THAT(gcode, ContainsSubstring("G18"));
    CHECK_THAT(gcode, ContainsSubstring("G19"));
    // After the vertical arcs the XY arc selects G17 again.
    const auto g19 = gcode.find("G19");
    REQUIRE(g19 != std::string::npos);
    CHECK(gcode.find("G17", g19) != std::string::npos);
    const BackplotResult plotted = backplot(gcode);
    CHECK(plotted.warnings.empty());
    double dev = 0.0;
    const auto diff = compareBackplot({tp}, plotted.path, 1e-4, &dev);
    CHECK(diff.value_or("") == "");
    // The ZX arc goes over the top: its highest point is z = 10.
    const auto stats = toolpath::computeStats(tp);
    CHECK_THAT(stats.bounds.max.z(), WithinAbs(20.0, 1e-9));
    const auto pts = toolpath::sampleArc(Vec3(-10, 0, 0), zx, 1e-3);
    double top = -1e9;
    for (const Vec3& q : pts) {
        top = std::max(top, q.z());
        CHECK_THAT(q.y(), WithinAbs(0.0, 1e-12));
    }
    CHECK_THAT(top, WithinAbs(10.0, 1e-3));
}

TEST_CASE("comments are plain ASCII", "[post]") {
    toolpath::Toolpath tp;
    tp.toolNumber = 1;
    tp.start = Vec3(0, 0, 10);
    toolpath::PathBuilder b(tp);
    b.rapid(Vec3(1, 0, 10));
    const std::string gcode = postJobs({{"\xC3\x84 (pocket); 90\xC2\xB0", tp,
                                         "\xC3\x98"
                                         "10 end mill",
                                         1}},
                                       mill(), "p\xC3\xB8st", 1);
    CHECK_THAT(gcode, ContainsSubstring("(D10 END MILL)"));
    CHECK_THAT(gcode, ContainsSubstring("( POCKET 90DEG)"));
    CHECK(std::none_of(gcode.begin(), gcode.end(),
                       [](char c) { return static_cast<unsigned char>(c) >= 0x80; }));
}

TEST_CASE("post API: NC programs round-trip through JSON", "[post][api]") {
    std::vector<PostJob> jobs;
    for (const char* name :
         {"facing_rect_zigzag", "pocket_rect_island", "contour_rect_outside", "drill_mixed"}) {
        jobs.push_back({name, loadGoldenToolpath(name), "test tool", 1});
    }
    tooling::Tool tool = tooling::makeFlatEndMill(4, 10, 25, 70);
    CancelToken cancel;
    NullProgress progress;
    const strategies::Context ctx{
        tool, strategies::Feeds{}, strategies::Heights{}, 0.002, cancel, progress};
    strategies::ContourParams cp;
    cp.compensation = strategies::CompensationMode::Control;
    cp.leadLength = 5.0;
    jobs.push_back(
        {"comp",
         strategies::contour({{geom2d::Polyline::rectangle({0, 0}, {50, 30}), {}}}, 0, -5, cp, ctx),
         "", 2});
    const NcProgram program = lower(jobs, "api", 1234);
    const Json doc = toJson(program);
    CHECK(doc.at("format") == "oxcam-ncprogram");
    CHECK(doc.at("instructions").size() == program.instrs.size());
    // Units are conventional: feeds in mm/min.
    for (const Json& i : doc.at("instructions")) {
        if (i.at("op") == "linear") {
            CHECK(i.at("feed").get<double>() > 60.0);
            break;
        }
    }
    const NcProgram back = ncProgramFromJson(Json::parse(dumpJson(doc)));
    CHECK(runPost(mill(), back) == runPost(mill(), program));
    CHECK(runPost(fineMill(), back) == runPost(fineMill(), program));
    CHECK_THROWS_AS(ncProgramFromJson(Json{{"format", "something else"}}), Error);
    Json bad = doc;
    bad["instructions"][0]["op"] = "teleport";
    CHECK_THROWS_AS(ncProgramFromJson(bad), Error);
}

TEST_CASE("3+2 jobs index the table between operations", "[post][index]") {
    const toolpath::Toolpath a = loadGoldenToolpath("facing_rect_zigzag");
    const toolpath::Toolpath b = loadGoldenToolpath("contour_rect_outside");
    PostJob top{"top", a, "face mill", 1, machine::IndexAngles{}, "AC"};
    PostJob side{"side", b, "face mill", 2, machine::IndexAngles{kPi / 2.0, 0.0, -kPi / 2.0}, "AC"};
    side.path.toolNumber = top.path.toolNumber; // same tool: no tool change between them
    const NcProgram program = lower({top, side}, "indexed", 3000);
    const std::string gcode = runPost(mill(), program);
    INFO(gcode);
    CHECK_THAT(gcode, ContainsSubstring("G0 G54 A0. C0."));
    const auto turn = gcode.find("A90. C-90.");
    REQUIRE(turn != std::string::npos);
    // The tool goes home in Z before the table turns.
    const auto home = gcode.rfind("G91 G28 Z0.", turn);
    REQUIRE(home != std::string::npos);
    CHECK(gcode.find("G55", home) != std::string::npos);
    // The back-plot still matches (rotary words carry no linear motion).
    CHECK(compareBackplot({a, side.path}, backplot(gcode).path, 1e-3).value_or("") == "");
    // The post API carries the index.
    const NcProgram back = ncProgramFromJson(toJson(program));
    CHECK(runPost(mill(), back) == gcode);
}

TEST_CASE("lathe post: diameters, constant surface speed, threads", "[post][lathe]") {
    const geom2d::Regions part = geom2d::unite(
        geom2d::Regions{{geom2d::Polyline::rectangle(Vec2(-30, 0), Vec2(0, 10)), {}}},
        geom2d::Regions{{geom2d::Polyline::rectangle(Vec2(-60, 0), Vec2(-30, 15)), {}}});
    const geom2d::Regions stock{{geom2d::Polyline::rectangle(Vec2(-60, 0), Vec2(1, 16)), {}}};
    tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
    turn.insert.noseRadius = 0.4;
    tooling::Tool thread = tooling::makeDefaultTool(tooling::ToolType::ThreadingInsert, 3);
    thread.insert.threadPitch = 1.5;
    CancelToken cancel;
    NullProgress progress;
    strategies::Feeds feeds;
    feeds.spindle = 1500.0 / 60.0;
    feeds.cut = 0.2 * feeds.spindle;
    feeds.lead = feeds.cut;
    strategies::TurnRoughParams rp;
    rp.common.surfaceSpeed = 200.0 * 1000.0 / 60.0;
    rp.common.maxSpindle = 3000.0 / 60.0;
    const auto rough = strategies::turnRough(
        {part, stock}, rp, strategies::Context{turn, feeds, {}, 0.002, cancel, progress});
    strategies::TurnThreadParams tp;
    tp.length = 20.0;
    const auto threads = strategies::turnThread(
        {part, stock}, tp, strategies::Context{thread, feeds, {}, 0.002, cancel, progress});
    const std::string gcode =
        postJobs({{"rough", rough, "turning", 1}, {"thread", threads, "threading", 1}},
                 builtinPost("fanuc-generic-lathe"), "shaft", 2000);
    INFO(gcode);
    CHECK_THAT(gcode, ContainsSubstring("G18"));
    CHECK_THAT(gcode, ContainsSubstring("T0101"));
    CHECK_THAT(gcode, ContainsSubstring("T0303"));
    CHECK_THAT(gcode, ContainsSubstring("G50 S3000"));
    CHECK_THAT(gcode, ContainsSubstring("G96 S200 M03"));
    CHECK_THAT(gcode, ContainsSubstring("G97 S1500 M03")); // threads at constant rpm
    CHECK_THAT(gcode, ContainsSubstring("G32"));
    CHECK_THAT(gcode, ContainsSubstring("F1.5"));
    CHECK_THAT(gcode, ContainsSubstring("F0.2")); // feed per revolution
    CHECK(gcode.find(" Y") == std::string::npos);
    // Diameter programming: the first pass below the 16 mm radius stock runs at radius 14.
    CHECK_THAT(gcode, ContainsSubstring("X28."));
    const BackplotResult plotted = backplot(gcode, {true});
    CHECK(plotted.toolChanges == std::vector<int>{1, 3});
    CHECK(compareBackplot({rough, threads}, plotted.path, 2e-3, nullptr, false).value_or("") == "");
    // The post API carries the turning data.
    const NcProgram program = lower({{"rough", rough, "turning", 1}}, "shaft", 2000);
    CHECK(runPost(builtinPost("fanuc-generic-lathe"), ncProgramFromJson(toJson(program))) ==
          runPost(builtinPost("fanuc-generic-lathe"), program));
}

TEST_CASE("a left-hand tap on a mill runs the spindle the other way", "[post][tap]") {
    for (const char* id : {"fanuc-generic-mill", "haas-classic-mill"}) {
        CAPTURE(id);
        const std::string gcode = postJobs({leftHandTap()}, builtinPost(id), "lh", 1000);
        CHECK_THAT(gcode, ContainsSubstring("S600 M04"));
        CHECK_THAT(gcode, ContainsSubstring("G74"));
        CHECK_THAT(gcode, !ContainsSubstring("G84"));
        CHECK_THAT(gcode, !ContainsSubstring("M03"));
        const toolpath::Toolpath back = backplot(gcode).path;
        REQUIRE(back.cycles.size() == 1);
        CHECK(back.cycles[0].type == toolpath::CycleType::TapLeft);
    }
    SECTION("a post that does not say it takes one refuses") {
        LowerOptions options;
        options.cycleTypes = {toolpath::CycleType::Tap};
        CHECK_THROWS_WITH(lower({leftHandTap()}, "lh", 1000, options),
                          ContainsSubstring("left-hand tap"));
    }
}
