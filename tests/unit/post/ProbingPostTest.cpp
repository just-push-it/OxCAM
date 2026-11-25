// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Probing cycles in the NC program and the generic post (DL-0090). The golden program
// tests/golden/posts/mill-probing.nc is the generic mill post's; OCAM_UPDATE_GOLDEN=1 rewrites it.

#include "RunPost.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/post/Engine.hpp>
#include <ocam/post/Post.hpp>
#include <ocam/strategies/Probing.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <sstream>

using namespace ocam;
using namespace ocam::post;
using namespace ocam::post::testing;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using toolpath::ProbeFeature;

namespace {

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

void checkGolden(const std::string& name, const std::string& text) {
    const auto path = std::filesystem::path(OCAM_GOLDEN_DIR) / "posts" / (name + ".nc");
    const char* update = std::getenv("OCAM_UPDATE_GOLDEN");
    if (update && std::string(update) == "1") {
        writeFileAtomic(path, text);
        SUCCEED("golden program updated");
        return;
    }
    INFO("golden file " << path.string());
    REQUIRE(std::filesystem::exists(path));
    CHECK(firstDifference(readFileBytes(path), text) == "");
}

struct Probing {
    tooling::Tool tool = [] {
        tooling::Tool t = tooling::makeDefaultTool(tooling::ToolType::Probe, 20, 6.0);
        t.mill.triggerOffset = 0.02;
        return t;
    }();
    CancelToken cancel;
    NullProgress progress;

    toolpath::Toolpath cycle(strategies::ProbeParams p) {
        p.approach = 5.0;
        p.overtravel = 4.0;
        p.fastFeed = 1000.0 / 60.0;
        p.measureFeed = 100.0 / 60.0;
        const strategies::Context ctx{tool, {}, {25.0, 5.0, 1.0}, 0.002, cancel, progress};
        return strategies::probe(p, ctx);
    }

    /// Three operations with the probe: set G54 Z from the top, G55 X and Y from a bore, and
    /// check a boss's diameter. With `setOffsets` false, the top and the bore are checks too.
    std::vector<PostJob> jobs(bool setOffsets = true) {
        strategies::ProbeParams top;
        top.feature = ProbeFeature::Surface;
        top.nominal = Vec3(10, 10, 0);
        top.direction = -Vec3::UnitZ();
        top.workOffset = 1;
        strategies::ProbeParams bore;
        bore.feature = ProbeFeature::Bore;
        bore.nominal = Vec3(40, 30, -6);
        bore.size = 20.0;
        bore.workOffset = 2;
        if (!setOffsets) {
            top.workOffset = 0;
            top.tolerance = 0.02;
            bore.workOffset = 0;
            bore.tolerance = 0.01;
        }
        strategies::ProbeParams boss;
        boss.feature = ProbeFeature::Boss;
        boss.nominal = Vec3(80, 30, -4);
        boss.size = 16.0;
        boss.tolerance = 0.05;
        NcTool table;
        table.number = 20;
        table.name = "Probe";
        table.type = "probe";
        table.diameter = 6.0;
        std::vector<PostJob> out;
        for (const auto& [name, params] :
             std::vector<std::pair<const char*, strategies::ProbeParams>>{
                 {"Probe top", top}, {"Probe bore", bore}, {"Check boss", boss}}) {
            PostJob job{name, cycle(params), "Probe", 1};
            job.tool = table;
            out.push_back(std::move(job));
        }
        return out;
    }
};

} // namespace

TEST_CASE("a probing cycle is one instruction with its moves", "[post][probing]") {
    Probing f;
    const NcProgram program = lower(f.jobs(), "probing", 1001);
    std::vector<const NcInstr*> probes;
    for (const NcInstr& i : program.instrs) {
        if (i.op == NcOp::Probe) {
            probes.push_back(&i);
        }
        // A probe does not turn; nothing between its moves is a plain move.
        CHECK(i.op != NcOp::Spindle);
        CHECK(i.op != NcOp::Linear);
    }
    REQUIRE(probes.size() == 3);
    const NcInstr& top = *probes[0];
    CHECK(top.probe.feature == ProbeFeature::Surface);
    CHECK(top.probe.workOffset == 1);
    CHECK(top.probe.axes == "Z");
    REQUIRE(top.probeMoves.size() == 4); // down, touch, back, up
    const NcProbeMove& touch = top.probeMoves[1];
    CHECK(touch.measure);
    // It goes on past the nominal surface by the overtravel, looking for it.
    CHECK_THAT(touch.expected.z(), WithinAbs(-0.02, 1e-9));
    CHECK_THAT(touch.target.z(), WithinAbs(-4.02, 1e-9));
    CHECK_THAT(touch.feed, WithinAbs(100.0 / 60.0, 1e-12));
    CHECK_THAT(top.probeMoves[0].feed, WithinAbs(1000.0 / 60.0, 1e-12));
    CHECK(probes[1]->probe.axes == "XY");
    CHECK(probes[1]->probe.workOffset == 2);
    CHECK(probes[2]->probe.isCheck());
    CHECK(probes[2]->probe.tolerance == 0.05);
    CHECK(std::count_if(probes[1]->probeMoves.begin(), probes[1]->probeMoves.end(),
                        [](const NcProbeMove& m) { return m.measure; }) == 4);

    // The JSON form (the post API) reads back to the same program.
    const Json json = toJson(program);
    CHECK(json.at("version") == kNcProgramJsonVersion);
    const Json* first = nullptr;
    for (const Json& j : json.at("instructions")) {
        if (j.at("op") == "probe") {
            first = &j;
            break;
        }
    }
    REQUIRE(first);
    CHECK(first->at("feature") == "surface");
    CHECK(first->at("operation") == 0);
    CHECK(first->at("check") == false);
    CHECK(first->at("moves").size() == 4);
    CHECK(first->at("moves").at(1).at("kind") == "measure");
    CHECK_THAT(first->at("moves").at(1).at("expected").at("z").get<double>(),
               WithinAbs(-0.02, 1e-9));
    CHECK_THAT(first->at("fast_feed").get<double>(), WithinAbs(1000.0, 1e-9));
    const NcProgram back = ncProgramFromJson(json);
    CHECK(toJson(back) == json);
}

TEST_CASE("the generic mill post writes feature checks as skip moves", "[post][probing][golden]") {
    Probing f;
    std::vector<std::string> warnings;
    const std::string text = postJobs(f.jobs(false), mill(), "probing", 1001, &warnings);
    checkGolden("mill-probing", text);
    CHECK_THAT(text, ContainsSubstring("(PROBE SURFACE: CHECK WITHIN 0.02)"));
    CHECK_THAT(text, ContainsSubstring("(PROBE BORE: CHECK WITHIN 0.01)"));
    CHECK_THAT(text, ContainsSubstring("(PROBE BOSS: CHECK WITHIN 0.05)"));
    CHECK_THAT(text, ContainsSubstring("G31 X10. Y10. Z-4.02 F100"));
    CHECK(text.find("M03") == std::string::npos); // the probe does not turn
    // Plain G-code cannot compare the measurement: said once.
    CHECK(std::count_if(warnings.begin(), warnings.end(), [](const std::string& w) {
              return w.find("feature checks only") != std::string::npos;
          }) == 1);
}

TEST_CASE("the generic post refuses a cycle that sets a work offset", "[post][probing]") {
    // It would probe and then cut on the old offset: refused, naming the operation and what the
    // post needs (the maintainer's decision, ADR-0008).
    Probing f;
    CHECK_THROWS_WITH(postJobs(f.jobs(), mill(), "probing", 1001),
                      ContainsSubstring("Probe top: it sets G54 Z") &&
                          ContainsSubstring("needs a probeEnd that applies the result"));

    // A post that overrides probeEnd posts it, with the base's skip moves.
    CancelToken cancel;
    const PostScript post = PostScript::load(R"js(
import { FanucMill } from "oxcam:fanuc";
import { out } from "oxcam:post";
export const manifest = {
  id: "probe-end", name: "Probe end", version: "1", engine: 1, machines: ["mill3"],
};
export const options = [];
export default class extends FanucMill {
  probeEnd(cycle) { out.block(`G65 P1300 W${cycle.work_offset}`); }
}
)js",
                                             "probe-end.js", cancel);
    std::vector<std::string> warnings;
    const std::string text = postJobs(f.jobs(), post, "probing", 1001, &warnings);
    CHECK(warnings.empty());
    CHECK_THAT(text, ContainsSubstring("G31 X10. Y10. Z-4.02 F100"));
    CHECK_THAT(text, ContainsSubstring("G65 P1300 W1"));
    CHECK_THAT(text, ContainsSubstring("G65 P1300 W2"));
    CHECK(text.find("NOT USED") == std::string::npos);
}

TEST_CASE("a post calls its probe's macros through the hooks", "[post][probing]") {
    // Placeholder program numbers: a real post takes them from its probe maker's manual.
    CancelToken cancel;
    const PostScript post = PostScript::load(R"js(
import { FanucMill } from "oxcam:fanuc";
import { out } from "oxcam:post";
export const manifest = {
  id: "probe-macros", name: "Probe macros", version: "1", engine: 1, machines: ["mill3"],
};
export const options = [];
export default class extends FanucMill {
  probeOn(cycle) { out.block("M1001 (ON)"); }
  probeOff() { out.block("M1002 (OFF)"); }
  probePosition(move, cycle) {
    out.block(`G65 P1100 X${this.length(move.x)} Y${this.length(move.y)} Z${this.length(move.z)}`);
  }
  probeMeasure(move, cycle) {
    out.block(`G65 P1101 X${this.length(move.expected.x)} Y${this.length(move.expected.y)} Z${this.length(move.expected.z)}`);
  }
  probeEnd(cycle) {
    out.block(cycle.check ? `G65 P1200 T${this.length(cycle.tolerance)}`
                          : `G65 P1300 W${cycle.work_offset}`);
  }
}
)js",
                                             "probe-macros.js", cancel);
    Probing f;
    std::vector<std::string> warnings;
    const std::string text = postJobs(f.jobs(), post, "probing", 1001, &warnings);
    CHECK(warnings.empty());
    CHECK(text.find("G31") == std::string::npos);
    CHECK(text.find("NOT USED") == std::string::npos);
    CHECK_THAT(text, ContainsSubstring("G65 P1101 X10. Y10. Z-0.02"));
    CHECK_THAT(text, ContainsSubstring("G65 P1300 W2"));
    CHECK_THAT(text, ContainsSubstring("G65 P1200 T0.05"));
    // On before the first cycle, off once at the end (one tool for all three).
    CHECK(text.find("M1001") < text.find("G65"));
    CHECK(text.find("M1001", text.find("M1001") + 1) == std::string::npos);
    CHECK(text.find("M1002") > text.rfind("G65"));
}
