// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The part handed to the sub-spindle (ADR-0007, DL-0092): the spool turned on the main spindle of
// `generic-lathe-sub`, handed over and parted off under grip, its back turned on the sub-spindle.
// The program on the generic Fanuc-style lathe post is checked against
// tests/golden/posts/lathe-sub-transfer.nc (OCAM_UPDATE_GOLDEN=1 rewrites it).

#include "FixtureSpecs.hpp"
#include "TestProject.hpp"

#include <ocam/document/Generate.hpp>
#include <ocam/document/LatheMachine.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/post/Engine.hpp>
#include <ocam/post/Post.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <sstream>

using namespace ocam;
using namespace ocam::document;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace {

struct SubJob {
    SubJob()
        : doc(Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "turret_sub_project.ocam")) {
        mainId = doc.project().setups.front().id;
        subId = doc.project().setups.back().id;
        for (const Operation& op : main().operations) {
            if (op.type == "spindle_transfer") {
                transferId = op.id;
            }
        }
    }
    const Setup& main() const { return *doc.project().findSetup(mainId); }
    const Setup& sub() const { return *doc.project().findSetup(subId); }
    const machine::Machine& machine() const { return machineFor(doc.project(), main()); }
    toolpath::Toolpath generate() {
        return generateToolpath(doc, main(), *doc.project().findOperation(transferId), cancel,
                                progress);
    }
    void set(const char* key, Json value) {
        doc.modify("param",
                   [&](Project& p) { p.findOperation(transferId)->parameters[key] = value; });
    }
    PostOutput post(const Uuid& setup, const Json& options = Json::object()) {
        return postSetup(doc, setup, machine(), post::builtinPost("fanuc-generic-lathe"), cancel,
                         progress, options);
    }
    Document doc;
    Uuid mainId;
    Uuid subId;
    Uuid transferId;
    CancelToken cancel;
    NullProgress progress;
};

/// The machine's codes, made up for the test: the generic post has none of its own.
Json subCodes() {
    return Json{{"program",
                 {{"subSpindleSelect", "M201"},
                  {"mainSpindleSelect", "M202"},
                  {"subChuckOpen", "M211"},
                  {"subChuckClose", "M210"},
                  {"syncOn", "M203"},
                  {"syncAngleOn", "M204 R{angle}"},
                  {"syncOff", "M205"},
                  {"z2Letter", "B"}}}};
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
    std::istringstream a(readFileBytes(path));
    std::istringstream b(text);
    std::string la;
    std::string lb;
    for (int line = 1;; ++line) {
        const bool moreA = static_cast<bool>(std::getline(a, la));
        const bool moreB = static_cast<bool>(std::getline(b, lb));
        INFO("line " << line);
        CHECK(moreA == moreB);
        if (!moreA || !moreB) {
            break;
        }
        CHECK(la == lb);
    }
}

} // namespace

TEST_CASE("a transfer takes the part where both setups put it", "[document][lathe][transfer]") {
    SubJob f;
    REQUIRE(transferTarget(f.doc.project(), f.main()) == &f.sub());
    const toolpath::Toolpath tp = f.generate();
    REQUIRE(tp.transfer);
    const toolpath::SpindleTransfer& t = *tp.transfer;
    const machine::LatheSubSpindle& sub = *f.machine().lathe->subSpindle;
    CHECK(tp.moves.empty());
    CHECK(tp.toolNumber == 0);
    CHECK(t.feed == Approx(500.0 / 60.0));
    CHECK(t.spindle == Approx(800.0 / 60.0));
    CHECK(t.sync == toolpath::SpindleTransfer::Sync::Speed);
    CHECK(t.partOff);
    CHECK(t.workZ2 == Approx(sub.workZ2));
    // The machine's grip while the operation does not say, and the clearance's 2 mm before it.
    CHECK(t.approachZ2 == Approx(t.pickZ2 + sub.grip + 2.0));
    const auto pick = pickOffZ2(f.doc, f.main(), f.sub(), f.machine(), f.cancel);
    REQUIRE(pick);
    CHECK(t.pickZ2 == Approx(*pick));

    SECTION("the sub-spindle setup's simulation takes it at the same Z2") {
        const SetupSimulation sim = simulateSetup(f.doc, f.subId, {}, f.cancel, f.progress);
        REQUIRE_FALSE(sim.hasErrors());
        const OperationSimulation& face = sim.operations.front();
        REQUIRE(face.latheMotion);
        REQUIRE(face.latheMotion->steps.size() >= 2);
        CHECK(face.latheMotion->steps[0].to.z2 == Approx(t.pickZ2));
        CHECK(sim.machineCollisionCount() == 0);
    }
    SECTION("a grip of its own moves where it takes the part, and the sub-spindle's work") {
        f.set("grip", 30.0);
        const toolpath::Toolpath deeper = f.generate();
        // The jaws 10 mm further over the part: Z2 10 mm nearer the main spindle.
        CHECK(deeper.transfer->pickZ2 == Approx(t.pickZ2 - 10.0));
        CHECK(deeper.transfer->approachZ2 == Approx(t.approachZ2));
    }
    SECTION("refused, naming the value") {
        f.set("z2_feed", 0.0);
        CHECK_THROWS_WITH(f.generate(), ContainsSubstring("Z2 feed"));
        f.set("z2_feed", 500.0);
        f.set("rpm", 0);
        CHECK_THROWS_WITH(f.generate(), ContainsSubstring("Speed"));
        f.set("rpm", 6000);
        CHECK_THROWS_WITH(f.generate(), ContainsSubstring("outside the"));
        f.set("rpm", 800);
        f.set("clearance", 100.0);
        f.set("grip", 1000.0);
        CHECK_THROWS_WITH(f.generate(), ContainsSubstring("outside its travel"));
    }
    SECTION("both stopped: no speed asked for") {
        f.set("spindles", "stopped");
        f.set("rpm", 0);
        const toolpath::Toolpath stopped = f.generate();
        CHECK(stopped.transfer->sync == toolpath::SpindleTransfer::Sync::Stopped);
        CHECK(stopped.spindleSpeed == 0.0);
    }
    SECTION("a part-off must follow it, unless the main chuck opens") {
        f.doc.modify("no part-off",
                     [&](Project& p) { p.findSetup(f.mainId)->operations.pop_back(); });
        CHECK_THROWS_WITH(f.generate(), ContainsSubstring("a part-off must follow"));
        f.set("then", "open_main");
        CHECK_NOTHROW(f.generate());
    }
    SECTION("no sub-spindle setup to take it") {
        f.doc.modify("hand-loaded", [&](Project& p) {
            p.findSetup(f.subId)->stock.sourceSetup.reset();
            p.findSetup(f.subId)->stock.kind = StockKind::Box;
        });
        CHECK_THROWS_WITH(f.generate(), ContainsSubstring("no setup on the sub-spindle"));
    }
}

TEST_CASE("the main setup simulates the transfer and parts off under grip",
          "[document][lathe][transfer]") {
    SubJob f;
    const SetupSimulation main = simulateSetup(f.doc, f.mainId, {}, f.cancel, f.progress);
    for (const OperationSimulation& os : main.operations) {
        INFO(os.name << ": " << os.error);
        CHECK(os.error.empty());
    }
    // Parted off: two pieces, and the sub-spindle setup starts from the part's.
    REQUIRE(main.operations.back().latheAfter);
    CHECK(main.operations.back().latheAfter->profile().size() == 2);
    const SetupSimulation sub = simulateSetup(f.doc, f.subId, {}, f.cancel, f.progress);
    REQUIRE_FALSE(sub.hasErrors());
    REQUIRE(sub.latheInitial);
    CHECK(sub.latheInitial->profile().size() == 1);
}

TEST_CASE("the pair posts as one program on the Fanuc-style lathe",
          "[document][lathe][transfer][golden]") {
    SubJob f;
    SECTION("the generic post asks for the machine's codes") {
        Json options{{"program", {{"z2Letter", "B"}}}};
        const PostOutput out = f.post(f.mainId, options);
        REQUIRE_FALSE(out.ok());
        CHECK_THAT(out.errors.front(), ContainsSubstring("Sub-spindle chuck open"));
    }
    SECTION("with them it writes the golden program: the front, the transfer, the part-off, the "
            "back") {
        const PostOutput out = f.post(f.mainId, subCodes());
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        checkGolden("lathe-sub-transfer", out.gcode);
        const auto at = [&](const std::string& s) {
            const auto k = out.gcode.find(s);
            INFO(s);
            CHECK(k != std::string::npos);
            return k;
        };
        CHECK(at("(TRANSFER)") < at("M211"));
        CHECK(at("M211") < at("G0 B"));
        CHECK(at("G0 B") < at("G1 B"));
        CHECK(at("G1 B") < at("M210"));
        CHECK(at("M210") < at("M203"));
        CHECK(at("M203") < at("(PART OFF)"));
        CHECK(at("(PART OFF)") < at("M205"));
        CHECK(at("M205") < at("M201"));
        CHECK(at("M201") < at("(FACE THE BACK)"));
        // Z2 to where it cuts, before the back is turned.
        CHECK(at("G0 B-250.") < at("(FACE THE BACK)"));
    }
    SECTION("the sub-spindle setup alone is refused: its program is the main setup's") {
        const PostOutput out = f.post(f.subId, subCodes());
        REQUIRE_FALSE(out.ok());
        CHECK_THAT(out.errors.front(), ContainsSubstring("post setup 'Main spindle'"));
    }
    SECTION("loaded by hand, the sub-spindle setup posts on its own") {
        f.doc.modify("no transfer", [&](Project& p) {
            auto& ops = p.findSetup(f.mainId)->operations;
            ops.erase(
                std::remove_if(ops.begin(), ops.end(),
                               [](const Operation& o) { return o.type == "spindle_transfer"; }),
                ops.end());
        });
        const PostOutput out = f.post(f.subId, subCodes());
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        CHECK_THAT(out.gcode, ContainsSubstring("M201"));
        CHECK_THAT(out.gcode, !ContainsSubstring("G0 B"));
    }
    SECTION("a post that does not say it writes a sub-spindle is refused") {
        const PostOutput out = postSetup(f.doc, f.mainId, f.machine(),
                                         post::builtinPost("haas-lathe"), f.cancel, f.progress);
        REQUIRE_FALSE(out.ok());
        CHECK_THAT(out.errors.front(), ContainsSubstring("does not write a sub-spindle"));
    }
    SECTION("the NC program's instructions, and its JSON") {
        const LoweredSetup lowered =
            lowerSetup(f.doc, f.mainId, f.machine(), {}, f.cancel, f.progress);
        REQUIRE(lowered.ok());
        std::vector<post::NcOp> ops;
        for (const post::NcInstr& i : lowered.program.instrs) {
            if (i.op == post::NcOp::Chuck || i.op == post::NcOp::SubAxis ||
                i.op == post::NcOp::SpindleSync) {
                ops.push_back(i.op);
            }
        }
        using post::NcOp;
        CHECK(ops == std::vector{NcOp::Chuck, NcOp::SubAxis, NcOp::SubAxis, NcOp::Chuck,
                                 NcOp::SpindleSync, NcOp::SpindleSync, NcOp::SubAxis});
        const post::NcProgram back = post::ncProgramFromJson(post::toJson(lowered.program));
        REQUIRE(back.instrs.size() == lowered.program.instrs.size());
        bool sub = false;
        for (const post::NcOperation& o : back.operations) {
            sub = sub || o.subSpindle;
        }
        CHECK(sub);
    }
}
