// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Back boring in the NC program and the G-code (DL-0083): a canned cycle where the post takes
// one, else oriented-spindle moves.

#include "RunPost.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/post/Post.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <string>

using namespace ocam;
using namespace ocam::post;
using namespace ocam::post::testing;
using Catch::Matchers::ContainsSubstring;

namespace {

/// One back bore at (20, 20): from 2 above the plate down to -25, cutting up to -20, shifted
/// 2 mm along -X with the spindle at 90°.
PostJob backBoreJob() {
    toolpath::Toolpath path;
    path.toolNumber = 4;
    path.spindleSpeed = 10.0;
    path.start = Vec3(20.0, 20.0, 15.0);
    toolpath::DrillCycle cycle;
    cycle.type = toolpath::CycleType::BackBore;
    cycle.topZ = -20.0;
    cycle.bottomZ = -25.0;
    cycle.retractZ = 2.0;
    cycle.feedRate = 20.0;
    cycle.dwell = 0.5;
    cycle.shift = Vec2(-2.0, 0.0);
    cycle.orient = degToRad(90.0);
    path.cycles.push_back(cycle);
    toolpath::Move down;
    down.kind = toolpath::MoveKind::Rapid;
    down.feedClass = toolpath::FeedClass::Rapid;
    down.end = Vec3(20.0, 20.0, 2.0);
    path.moves.push_back(down);
    toolpath::Move hole;
    hole.kind = toolpath::MoveKind::Cycle;
    hole.cycle = 0;
    hole.end = Vec3(20.0, 20.0, 2.0);
    hole.feedRate = 20.0;
    path.moves.push_back(hole);
    toolpath::Move up;
    up.kind = toolpath::MoveKind::Rapid;
    up.feedClass = toolpath::FeedClass::Rapid;
    up.end = Vec3(20.0, 20.0, 15.0);
    path.moves.push_back(up);
    PostJob job;
    job.name = "Back bore";
    job.path = path;
    return job;
}

/// Line numbers of the lines containing each text, in order; -1 when one is missing.
std::vector<long> positions(const std::string& gcode, const std::vector<std::string>& texts) {
    std::vector<long> out;
    std::size_t from = 0;
    for (const std::string& t : texts) {
        const std::size_t at = gcode.find(t, from);
        if (at == std::string::npos) {
            out.push_back(-1);
            continue;
        }
        out.push_back(static_cast<long>(std::count(gcode.begin(), gcode.begin() + at, '\n')));
        from = at + t.size();
    }
    return out;
}

} // namespace

TEST_CASE("a back bore is lowered as moves with the spindle oriented", "[post][backbore]") {
    LowerOptions options;
    options.cycleTypes = {toolpath::CycleType::Drill}; // no back bore: it is played out
    const NcProgram program = lower({backBoreJob()}, "bb", 1000, options);
    std::vector<NcOp> ops;
    for (const NcInstr& i : program.instrs) {
        if (i.op == NcOp::SpindleOrient || i.op == NcOp::Spindle || i.op == NcOp::Linear) {
            ops.push_back(i.op);
        }
        CHECK(i.op != NcOp::Cycle);
    }
    // On; oriented (down); on, cut up; oriented (up); on again; off at the end.
    CHECK(ops == std::vector<NcOp>{NcOp::Spindle, NcOp::SpindleOrient, NcOp::Spindle, NcOp::Linear,
                                   NcOp::SpindleOrient, NcOp::Spindle, NcOp::Spindle});
    for (const NcInstr& i : program.instrs) {
        if (i.op == NcOp::SpindleOrient) {
            CHECK(i.angle == degToRad(90.0));
        }
        if (i.op == NcOp::Spindle) {
            CHECK((i.spindle == 10.0 || i.spindle == 0.0)); // as set, or off
        }
        if (i.op == NcOp::Linear) {
            CHECK(i.feed == 20.0); // the cycle's feed, as set
        }
    }
    SECTION("the NC program's JSON round-trips the new op and the cycle's fields") {
        const NcProgram back = ncProgramFromJson(toJson(program));
        REQUIRE(back.instrs.size() == program.instrs.size());
        for (std::size_t k = 0; k < back.instrs.size(); ++k) {
            CHECK(back.instrs[k].op == program.instrs[k].op);
            CHECK(std::abs(back.instrs[k].angle - program.instrs[k].angle) < 1e-12);
        }
        const NcProgram cyc = lower({backBoreJob()}, "bb", 1000, LowerOptions{});
        const NcProgram cycBack = ncProgramFromJson(toJson(cyc));
        bool found = false;
        for (const NcInstr& i : cycBack.instrs) {
            if (i.op == NcOp::Cycle) {
                found = true;
                CHECK(i.cycle.type == toolpath::CycleType::BackBore);
                CHECK(std::abs(i.cycle.shift.x() + 2.0) < 1e-12);
                CHECK(std::abs(i.cycle.orient - degToRad(90.0)) < 1e-12);
            }
        }
        CHECK(found);
    }
}

TEST_CASE("the generic mill post plays a back bore out as moves", "[post][backbore][builtin]") {
    const std::string gcode = postJobs({backBoreJob()}, mill(), "bb", 1000);
    INFO(gcode);
    CHECK_THAT(gcode, !ContainsSubstring("G87"));
    const auto at = positions(gcode, {"S600 M03", "M19 R90.", "X18.", "Z-25.", "X20.", "S600 M03",
                                      "G1 Z-20. F1200", "G4 P500", "M19 R90.", "X18.", "Z2.",
                                      "X20.", "S600 M03", "M05"});
    for (std::size_t k = 0; k < at.size(); ++k) {
        CAPTURE(k);
        CHECK(at[k] >= 0);
        if (k > 0) {
            CHECK(at[k] > at[k - 1]);
        }
    }
    // The program back-plots to the same motion as the toolpath played out.
    const toolpath::Toolpath plotted = backplot(gcode).path;
    const auto difference = compareBackplot({backBoreJob().path}, plotted, 1e-3);
    CHECK_FALSE(difference);
}

TEST_CASE("a post that takes back bores writes the cycle", "[post][backbore]") {
    const PostScript post = millWith("", "cycles: [\"drill\", \"back_bore\"],");
    const std::string gcode = postJobs({backBoreJob()}, post, "bb", 1000);
    INFO(gcode);
    CHECK_THAT(gcode, ContainsSubstring("G98 G87 X20. Y20. Z-20. R-25. Q2. P500 F1200"));
    CHECK_THAT(gcode, !ContainsSubstring("M19"));
    CHECK_THAT(gcode, ContainsSubstring("G80"));
    SECTION("its spindle orientation block is a setting") {
        const PostScript other =
            millWith("this.spindleOrientBlock = \"M19 P{angle}\";", "cycles: [\"drill\"],");
        CHECK_THAT(postJobs({backBoreJob()}, other, "bb", 1000), ContainsSubstring("M19 P90."));
    }
}
