// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The JavaScript built-in posts and oxcam:fanuc (DL-0039). The golden programs in
// tests/golden/posts were written by the C++ formatter the posts replaced (byte for byte equal
// before it was removed, DL-0039 step 4); OCAM_UPDATE_GOLDEN=1 rewrites them.

#include "RunPost.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/post/Engine.hpp>
#include <ocam/post/Post.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>

using namespace ocam;
using namespace ocam::post;
using namespace ocam::post::testing;
using Catch::Matchers::ContainsSubstring;

namespace {

toolpath::Toolpath loadGoldenToolpath(const std::string& name) {
    const auto path = std::filesystem::path(OCAM_GOLDEN_DIR) / (name + ".json");
    return toolpath::toolpathFromJson(parseJson(readFileBytes(path), name));
}

/// "line N: <expected> | <actual>" of the first difference, or "" when equal.
std::string firstDifference(const std::string& expected, const std::string& actual) {
    std::istringstream a(expected);
    std::istringstream b(actual);
    std::string la;
    std::string lb;
    for (int line = 1;; ++line) {
        const bool moreA = static_cast<bool>(std::getline(a, la));
        const bool moreB = static_cast<bool>(std::getline(b, lb));
        if (!moreA && !moreB) {
            return expected == actual ? "" : "line endings differ";
        }
        if (!moreA || !moreB || la != lb) {
            return "line " + std::to_string(line) + ": golden '" + (moreA ? la : "<end>") +
                   "' | posted '" + (moreB ? lb : "<end>") + "'";
        }
    }
}

/// The text must equal tests/golden/posts/<name>.nc byte for byte.
void checkGolden(const std::string& name, const std::string& text) {
    const auto path = std::filesystem::path(OCAM_GOLDEN_DIR) / "posts" / (name + ".nc");
    const char* update = std::getenv("OCAM_UPDATE_GOLDEN");
    if (update && std::string(update) == "1") {
        std::filesystem::create_directories(path.parent_path());
        writeFileAtomic(path, text);
        SUCCEED("golden program updated");
        return;
    }
    INFO("golden file " << path.string());
    REQUIRE(std::filesystem::exists(path));
    CHECK(firstDifference(readFileBytes(path), text) == "");
}

std::vector<PostJob> goldenJobs(bool compensated) {
    std::vector<PostJob> jobs;
    int offset = 1;
    for (const char* name :
         {"facing_rect_zigzag", "pocket_rect_island", "contour_rect_outside", "drill_mixed"}) {
        toolpath::Toolpath path = loadGoldenToolpath(name);
        if (compensated && std::string(name) == "contour_rect_outside") {
            path.compensation = toolpath::Compensation::Left;
        }
        jobs.push_back(PostJob{name, path, std::string(name) + " tool", offset++});
    }
    return jobs;
}

} // namespace

TEST_CASE("the built-in posts are listed and declare their machines", "[post][builtin]") {
    CHECK(builtinPostIds() == std::vector<std::string>{"fanuc-generic-mill",
                                                       "fanuc-generic-mill-4axis",
                                                       "fanuc-generic-lathe", "haas-lathe",
                                                       "haas-classic-mill"});
    const PostManifest& m = builtinPost("fanuc-generic-mill").manifest();
    CHECK(m.tcp);
    CHECK(m.verify == "fanuc-mill");
    CHECK(lowerOptionsFor(m).cannedCycles);
    const PostManifest& fourAxis = builtinPost("fanuc-generic-mill-4axis").manifest();
    CHECK_FALSE(fourAxis.tcp);
    CHECK_FALSE(lowerOptionsFor(fourAxis).tcp);
    const PostManifest& lathe = builtinPost("fanuc-generic-lathe").manifest();
    CHECK(lathe.machines == std::vector<std::string>{"lathe2"});
    // Only the tap cycle: plain moves cannot reverse the spindle.
    CHECK(lowerOptionsFor(lathe).cannedCycles);
    CHECK(lowerOptionsFor(lathe).cycleTypes ==
          std::vector{toolpath::CycleType::Tap, toolpath::CycleType::TapLeft});
    CHECK_THROWS_WITH(builtinPost("nope"), ContainsSubstring("no built-in post 'nope'"));
}

TEST_CASE("the mill post writes the golden programs", "[post][builtin][golden]") {
    for (const bool cannedCycles : {true, false}) {
        for (const bool compensated : {false, true}) {
            CAPTURE(cannedCycles, compensated);
            LowerOptions options;
            options.cannedCycles = cannedCycles;
            const NcProgram program = lower(goldenJobs(compensated), "golden", 1000, options);
            checkGolden(std::string("mill-") + (cannedCycles ? "cycles" : "expanded") +
                            (compensated ? "-compensated" : ""),
                        runPost(mill(), program));
        }
    }
}

TEST_CASE("oxcam:fanuc settings change the output as declared", "[post][builtin][golden]") {
    // Every setting away from its default: inch, no line numbers, no trailing points, R arcs
    // split at 90 degrees, dwell in seconds, semicolon comments in mixed case, other templates.
    const PostScript script = millWith(R"js(
    this.units = "in";
    this.lineNumbers = null;
    this.trailingPoint = false;
    this.decimals = { linear: 4, feed: 1, dwell: 2, rotary: 3, feedPerRev: 3 };
    this.arcFormat = "r";
    this.maxArcSweep = 90;
    this.dwellInMilliseconds = false;
    this.commentStyle = "semicolon";
    this.upperComments = false;
    this.header = ["%", "O{program_number} {program_name}"];
    this.safeStart = "G20 G90 G17";
    this.toolChangeBlocks = ["M06 T{tool2}", "M01"];
    this.toolLengthOffset = "G43 H{tool2}";
    this.cutterCompOffset = "D{tool2}";
    this.footer = ["M05", "M30", "", "%"];
    this.coolantCodes = { ...this.coolantCodes, flood: "M7", off: "M9" };
    this.spindleCw = "M3";
    this.spindleStop = "M5";
)js");
    for (const bool cannedCycles : {true, false}) {
        CAPTURE(cannedCycles);
        LowerOptions options;
        options.cannedCycles = cannedCycles;
        const NcProgram program = lower(goldenJobs(true), "Settings test", 77, options);
        checkGolden(std::string("settings-") + (cannedCycles ? "cycles" : "expanded"),
                    runPost(script, program));
    }
}

TEST_CASE("the example post runs: options, header, tool list, preload, feed clamp",
          "[post][builtin][example]") {
    const auto file =
        std::filesystem::path(OCAM_SOURCE_DIR) / "examples" / "posts" / "example-vmc.js";
    CancelToken cancel;
    const PostScript example = PostScript::load(readFileBytes(file), "example-vmc.js", cancel);
    CHECK(example.manifest().id == "example-vmc");
    REQUIRE(example.options().size() == 3);

    std::vector<PostJob> jobs = goldenJobs(false);
    for (std::size_t k = 0; k < jobs.size(); ++k) {
        jobs[k].path.toolNumber = static_cast<int>(k) + 1; // four tools
        jobs[k].operationId = "op" + std::to_string(k);
    }
    NcProgram program = lower(jobs, "Example", 1000, lowerOptionsFor(example.manifest()));
    program.project = "Bracket";
    program.setup = "Setup 1";
    NcMachine machine = ncMachine(machine::genericMill3());
    machine.maxFeed = 10.0; // 600 mm/min: the golden feeds go over it
    program.machine = machine;
    for (NcOperation& op : program.operations) {
        op.options = example.operationOptionValues(op.id == "op1" ? Json{{"highSpeed", "finishing"}}
                                                                  : Json::object());
    }
    const PostResult result =
        example.run(program, example.optionValues({{"preload", true}}), cancel);
    const std::string& text = result.text;
    INFO(text.substr(0, 1500));
    CHECK(text.starts_with("%\nO01000\n"));
    CHECK_THAT(text, ContainsSubstring("(BRACKET - SETUP 1)"));
    CHECK_THAT(text, ContainsSubstring("(POST EXAMPLE-VMC 1.0.0)"));
    CHECK_THAT(text, ContainsSubstring("(T1 "));
    CHECK_THAT(text, ContainsSubstring(" M01\n"));
    CHECK_THAT(text, ContainsSubstring("(HIGH SPEED FINISHING)"));
    CHECK(text.find("(HIGH SPEED ROUGHING)") == std::string::npos);
    // After T1 M06 the post preloads T2.
    const auto t1 = text.find("T1 M06");
    REQUIRE(t1 != std::string::npos);
    CHECK(text.find(" T2\n", t1) != std::string::npos);
    // Feeds over the machine's 600 mm/min are clamped, with a warning.
    CHECK(text.find("F1200.") == std::string::npos);
    CHECK_FALSE(result.warnings.empty());
}

namespace {

/// The golden jobs as four tools, for the Haas-style post: a dwell cycle, through-tool coolant
/// on the first tool, extended work offsets.
NcProgram haasProgram(const PostScript& haas, const Json& smoothFinish) {
    std::vector<PostJob> jobs = goldenJobs(false);
    for (std::size_t k = 0; k < jobs.size(); ++k) {
        jobs[k].path.toolNumber = static_cast<int>(k) + 1;
        jobs[k].operationId = "op" + std::to_string(k);
    }
    jobs[0].path.coolant = tooling::Coolant::ThroughTool;
    jobs[1].workOffset = 7;
    jobs[2].workOffset = 105;
    REQUIRE_FALSE(jobs[3].path.cycles.empty());
    jobs[3].path.cycles[0].type = toolpath::CycleType::Dwell;
    jobs[3].path.cycles[0].dwell = 0.5;
    // Pecks of 0.3 in: exact in either unit, so the expanded pecks do not drift in inches.
    REQUIRE(jobs[3].path.cycles[1].type == toolpath::CycleType::Peck);
    jobs[3].path.cycles[1].peck = 7.62;
    NcProgram program = lower(jobs, "Haas test", 1000, lowerOptionsFor(haas.manifest()));
    for (NcOperation& op : program.operations) {
        op.options = haas.operationOptionValues(op.id == "op1" ? smoothFinish : Json::object());
    }
    return program;
}

std::vector<toolpath::Toolpath> pathsOf(const std::vector<PostJob>& jobs) {
    std::vector<toolpath::Toolpath> paths;
    for (const PostJob& job : jobs) {
        paths.push_back(job.path);
    }
    return paths;
}

} // namespace

TEST_CASE("the Haas-style post writes the control's codes", "[post][builtin][haas]") {
    const PostScript& haas = builtinPost("haas-classic-mill");
    const PostManifest& m = haas.manifest();
    CHECK_FALSE(m.tcp); // machine positions: no tool centre point control
    CHECK(m.machines ==
          std::vector<std::string>{"mill3", "mill4_a", "mill5_table_ac", "mill5_table_bc"});
    const NcProgram program =
        haasProgram(haas, {{"smoothness", "finish"}, {"cornerRounding", 0.05}});
    CancelToken cancel;
    const PostResult result = haas.run(program, haas.optionValues({}), cancel);
    const std::string& text = result.text;
    INFO(text.substr(0, 2500));

    // Program number in five digits, "%" at both ends, no line numbers.
    CHECK(text.starts_with("%\nO01000 (HAAS TEST)\nG00 G17 G21 G40 G49 G80 G90 G94\n"));
    CHECK(text.ends_with("G91 G28 Z0.\nG28 Y0.\nG90\nM30\n%\n"));
    CHECK(text.find("\nN") == std::string::npos);
    // Through-spindle coolant before the spindle speed.
    const auto m88 = text.find("\nM88\n");
    REQUIRE(m88 != std::string::npos);
    CHECK(m88 < text.find(" M03\n"));
    // Optional stop before later tool changes, a safe start after each.
    CHECK(text.find("M01\nT1 M06") == std::string::npos);
    CHECK_THAT(text, ContainsSubstring("M01\nT2 M06"));
    CHECK_THAT(text, ContainsSubstring("T2 M06 (POCKET_RECT_ISLAND TOOL)\nG00 G90 G40 G49\n"));
    // Extended work offsets.
    CHECK_THAT(text, ContainsSubstring("G154 P1 "));
    CHECK_THAT(text, ContainsSubstring("G154 P99 "));
    CHECK(text.find("G54.1") == std::string::npos);
    // Smoothness for the second operation only, then back to the control's.
    const auto finish = text.find("G187 P3 E0.05\n");
    REQUIRE(finish != std::string::npos);
    CHECK(text.find("G187\n", finish) != std::string::npos);
    CHECK(text.find("G187") == finish);
    // Dwell in decimal seconds; the peck cycle states P0. so no dwell carries over.
    CHECK_THAT(text, ContainsSubstring(" P0.5 F"));
    CHECK_THAT(text, ContainsSubstring(" Q7.62 P0. F"));
    // One M code per block.
    std::istringstream lines(text);
    for (std::string line; std::getline(lines, line);) {
        CAPTURE(line);
        CHECK(line.size() <= 80);
        const std::string code = line.substr(0, line.find('('));
        const auto first = code.find('M');
        CHECK((first == std::string::npos || code.find('M', first + 1) == std::string::npos));
    }

    // The program back-plots onto the toolpaths.
    std::vector<PostJob> jobs = goldenJobs(false);
    jobs[3].path.cycles[0].type = toolpath::CycleType::Dwell;
    jobs[3].path.cycles[0].dwell = 0.5;
    jobs[3].path.cycles[1].peck = 7.62;
    const BackplotResult plotted = backplot(text);
    CHECK(plotted.warnings.empty());
    CHECK(compareBackplot(pathsOf(jobs), plotted.path, 1.5e-3).value_or("") == "");
}

TEST_CASE("the Haas-style post's program options", "[post][builtin][haas]") {
    const PostScript& haas = builtinPost("haas-classic-mill");
    const NcProgram program = haasProgram(haas, Json::object());
    CancelToken cancel;
    const std::string text = haas.run(program,
                                      haas.optionValues({{"units", "inch"},
                                                         {"lineNumbers", true},
                                                         {"optionalStop", false},
                                                         {"safeStartAfterToolChange", false},
                                                         {"homeWith", "G53"},
                                                         {"homeYAtEnd", false}}),
                                      cancel)
                                 .text;
    INFO(text.substr(0, 1500));
    CHECK(text.starts_with("%\nO01000 (HAAS TEST)\nN10 G00 G17 G20 G40 G49 G80 G90 G94\n"));
    CHECK(text.find("M01") == std::string::npos);
    CHECK(text.find("G00 G90 G40 G49") == std::string::npos);
    CHECK(text.find("G28") == std::string::npos);
    CHECK_THAT(text, ContainsSubstring(" G00 G53 G49 Z0.\n"));
    CHECK(text.find("G53 Y0.") == std::string::npos);
    CHECK(text.find("G187") == std::string::npos); // the control's smoothness throughout
    const BackplotResult plotted = backplot(text);
    std::vector<PostJob> jobs = goldenJobs(false);
    jobs[3].path.cycles[0].type = toolpath::CycleType::Dwell;
    jobs[3].path.cycles[0].dwell = 0.5;
    jobs[3].path.cycles[1].peck = 7.62;
    // 4 decimals of an inch: end points and arc centres within one and a half units.
    CHECK(compareBackplot(pathsOf(jobs), plotted.path, 4e-3).value_or("") == "");
}

TEST_CASE("the Haas-style post refuses what the control cannot do", "[post][builtin][haas]") {
    const PostScript& haas = builtinPost("haas-classic-mill");
    CancelToken cancel;
    {
        // Mist coolant: no code for it unless the user chooses one.
        std::vector<PostJob> jobs = goldenJobs(false);
        jobs[0].path.coolant = tooling::Coolant::Mist;
        const NcProgram program = lower(jobs, "Mist", 1000, lowerOptionsFor(haas.manifest()));
        CHECK_THROWS_WITH(haas.run(program, haas.optionValues({}), cancel),
                          ContainsSubstring("mist coolant"));
        CHECK_THAT(
            haas.run(program, haas.optionValues({{"mistCoolant", "shower M07"}}), cancel).text,
            ContainsSubstring("\nM07\n"));
    }
    {
        // Work offsets past G154 P99.
        std::vector<PostJob> jobs = goldenJobs(false);
        jobs[0].workOffset = 106;
        const NcProgram program = lower(jobs, "Offsets", 1000, lowerOptionsFor(haas.manifest()));
        CHECK_THROWS_WITH(haas.run(program, haas.optionValues({}), cancel),
                          ContainsSubstring("work offset 106"));
    }
    {
        // Tools the machine does not hold, and program numbers past O99999.
        std::vector<PostJob> jobs = goldenJobs(false);
        jobs[0].path.toolNumber = 30;
        NcProgram program = lower(jobs, "Tools", 1000, lowerOptionsFor(haas.manifest()));
        NcMachine machine = ncMachine(machine::genericMill3());
        machine.toolCapacity = 20;
        program.machine = machine;
        CHECK_THROWS_WITH(haas.run(program, haas.optionValues({}), cancel),
                          ContainsSubstring("the machine holds tools 1 to 20"));
        program.machine.reset();
        program.number = 100000;
        CHECK_THROWS_WITH(haas.run(program, haas.optionValues({}), cancel),
                          ContainsSubstring("does not fit Onnnnn"));
    }
}

TEST_CASE("posting speed of the JavaScript mill post", "[.][post][benchmark]") {
    // A long program: the pocket toolpath repeated (run with: test_post "[benchmark]"). Before
    // the C++ formatter was removed it took 0.9 s per million instructions here (DL-0039).
    toolpath::Toolpath pocket = loadGoldenToolpath("pocket_rect_island");
    const std::size_t copies = 400000 / std::max<std::size_t>(1, pocket.moves.size()) + 1;
    toolpath::Toolpath big = pocket;
    big.moves.clear();
    for (std::size_t c = 0; c < copies; ++c) {
        for (toolpath::Move m : pocket.moves) {
            m.end.z() -= 0.001 * static_cast<double>(c % 100);
            big.moves.push_back(m);
        }
    }
    const NcProgram program = lower({{"big", big, "end mill", 1}}, "speed", 1);
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    const std::string text = runPost(mill(), program);
    const auto t1 = Clock::now();
    // The engine alone: a post whose handlers write nothing.
    CancelToken cancel;
    const PostScript empty = PostScript::load(R"js(
export const manifest = { id: "empty", name: "Empty", version: "1", engine: 1, machines: ["mill3"] };
export const options = [];
export default class {
  programBegin() {} programEnd() {} operationBegin() {} operationEnd() {} comment() {}
  toolChange() {} workOffset() {} spindle() {} coolant() {} toolLength() {} rapid() {}
  linear() {} arc() {} dwell() {} cycle() {} cycleCancel() {} compensationOn() {}
  compensationOff() {} index() {} homeZ() {} tcpOff() {}
}
)js",
                                              "empty.js", cancel);
    const auto t2 = Clock::now();
    (void)runPost(empty, program);
    const auto t3 = Clock::now();
    const double js = std::chrono::duration<double>(t1 - t0).count();
    const double engine = std::chrono::duration<double>(t3 - t2).count();
    const double million = 1e6 / static_cast<double>(program.instrs.size());
    std::cout << program.instrs.size() << " instructions: mill post " << js << " s ("
              << js * million << " s per million), engine alone " << engine << " s\n";
    CHECK_FALSE(text.empty());
}

TEST_CASE("a post declares its work offsets and their codes", "[post][builtin]") {
    // Without a declaration: G54-G59 and G54.1 P1-P48.
    CHECK(describeWorkOffsets(mill().manifest()) == "G54-G59 (1-6), G54.1 P1-P48 (7-54)");
    CHECK(describeWorkOffsets(builtinPost("fanuc-generic-lathe").manifest()) == "G54-G59 (1-6)");
    const PostManifest& haas = builtinPost("haas-classic-mill").manifest();
    CHECK(workOffsetCode(haas, 1) == "G54");
    CHECK(workOffsetCode(haas, 7) == "G154 P1");
    CHECK(workOffsetCode(haas, 105) == "G154 P99");
    CHECK_FALSE(workOffsetCode(haas, 106));
    CHECK(lastWorkOffset(haas) == 105);

    // The Fanuc base writes what the post declares, and refuses the rest.
    const PostScript custom =
        millWith("", R"(workOffsets: [{ from: 1, to: 4, code: "G{}", start: 54 },
                             { from: 5, to: 12, code: "G{}", start: 110 }],)");
    std::vector<PostJob> jobs = goldenJobs(false);
    jobs[3].workOffset = 6;
    CHECK_THAT(postJobs(jobs, custom, "offsets", 1), ContainsSubstring(" G111 "));
    jobs[3].workOffset = 13;
    CHECK_THROWS_WITH(postJobs(jobs, custom, "offsets", 1),
                      ContainsSubstring("work offset 13: Test mill has work offsets 1-4, 5-12"));
    CHECK_THROWS_WITH(millWith("", R"(workOffsets: [{ from: 1, to: 6, code: "G{}", start: 54 },
                                                    { from: 6, to: 9, code: "G{}", start: 110 }],)"),
                      ContainsSubstring("overlap"));
}
