// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Bar pull (DL-0091): a puller on the turret takes the bar out of the open chuck for the next
// part. The shaft fixture turned and parted off, then the bar pulled, on the turret lathe; the
// program on the generic Fanuc-style lathe post is checked against
// tests/golden/posts/lathe-bar-pull.nc (OCAM_UPDATE_GOLDEN=1 rewrites it).

#include "FixtureSpecs.hpp"
#include "TestProject.hpp"

#include <ocam/document/Generate.hpp>
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
using toolpath::MachineFunction;
using toolpath::MoveKind;

namespace {

using namespace fixtures::shaft;
constexpr double kLength = kMajorLength + kMinorLength;
constexpr double kPartOffWidth = 3.0;
// The part, the part-off's width and the 1 mm faced off the front: the bar stands out as it did.
constexpr double kPull = kLength + kPartOffWidth + 1.0;

/// The shaft faced, turned and parted off from a bar 1 mm over its front and radius, then the
/// bar pulled by `kPull` with T7.
struct BarJob {
    BarJob() {
        const Uuid modelId = doc.importModel(std::filesystem::path(OCAM_FIXTURE_DIR) / "shaft.step",
                                             cancel, progress);
        setupId = doc.addSetup(modelId);
        tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
        turn.name = "Turn";
        turn.insert.noseRadius = 0.4;
        tooling::Tool part = tooling::makeDefaultTool(tooling::ToolType::PartingInsert, 5);
        part.name = "Part";
        part.insert.width = kPartOffWidth;
        part.insert.noseRadius = 0.1;
        tooling::Tool puller = tooling::makeDefaultTool(tooling::ToolType::BarPuller, 7, 40.0);
        puller.name = "Puller";
        puller.mill.fluteLength = 25.0;
        doc.modify("lathe setup", [&](Project& p) {
            Setup* s = p.findSetup(setupId);
            s->machine = "generic-lathe-turret";
            s->originFrom.reset();
            s->wcs.origin = Vec3(0, 0, kLength);
            s->wcs.zAxis = Vec3::UnitZ();
            s->wcs.xAxis = Vec3::UnitX();
            s->stock.kind = StockKind::Box;
            s->stock.size = StockSize::OffsetFromPart;
            s->stock.offsetSide = 1.0;
            s->stock.offsetTop = 1.0;
            s->stock.offsetBottom = 40.0;
            p.tools.tools = {testing::withPreset(turn), testing::withPreset(part), puller};
            auto add = [&](const char* name, const char* type, const tooling::Tool& tool,
                           Json params = Json::object(), PresetOverrides po = {}) {
                Operation op;
                op.name = name;
                op.type = type;
                op.toolId = tool.id;
                op.presetId = tool.presets.front().id;
                op.parameters = std::move(params);
                op.presetOverrides = po;
                op.heights = defaultHeights(op);
                s->operations.push_back(op);
                return op.id;
            };
            add("Face", "turn_face", p.tools.tools[0]);
            add("Rough", "turn_rough", p.tools.tools[0], Json::object(), testing::stepDown(1.5));
            add("Finish", "turn_finish", p.tools.tools[0]);
            add("Part off", "turn_partoff", p.tools.tools[1]);
            pullId = add("Bar pull", "bar_pull", p.tools.tools[2], Json{{"pull_length", kPull}});
        });
    }
    const Setup& setup() const { return *doc.project().findSetup(setupId); }
    toolpath::Toolpath generate(const Uuid& id) {
        return generateToolpath(doc, setup(), *doc.project().findOperation(id), cancel, progress);
    }
    void setParam(const char* key, Json value) {
        doc.modify("param", [&](Project& p) { p.findOperation(pullId)->parameters[key] = value; });
    }
    PostOutput post(const char* postId, const Json& options = Json::object()) {
        return postSetup(doc, setupId, machineFor(doc.project(), setup()),
                         post::builtinPost(postId), cancel, progress, options);
    }
    Document doc;
    Uuid setupId;
    Uuid pullId;
    CancelToken cancel;
    NullProgress progress;
};

/// Program option values, as postSetup takes them.
const Json kChuckCodes{{"program", {{"chuckOpen", "M11"}, {"chuckClose", "M10"}}}};

/// The text must equal tests/golden/posts/<name>.nc byte for byte.
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

TEST_CASE("a bar pull grips the bar where the part-off left it and pulls it out",
          "[document][lathe][barpull]") {
    BarJob f;
    const toolpath::Toolpath tp = f.generate(f.pullId);
    INFO((tp.warnings.empty() ? std::string() : tp.warnings.front()));
    const tooling::Tool& puller = f.doc.project().tools.tools[2];
    const tooling::CuttingPreset& preset = puller.presets.front();
    CHECK(tp.strategy == "bar_pull");
    CHECK(tp.toolNumber == 7);
    CHECK(tp.spindleSpeed == 0.0); // the chuck opens: the spindle stands still
    CHECK_FALSE(tp.isTurning());
    // The bar's front after the part-off: the part's back, less the part-off's width.
    const double front = -kLength - kPartOffWidth;
    const double grip = puller.mill.fluteLength; // from the puller while not set
    CHECK(tp.start.z() == Approx(front + 2.0));
    std::vector<MachineFunction> functions;
    for (const toolpath::Move& m : tp.moves) {
        CHECK(m.end.x() == Approx(0.0).margin(1e-12)); // on the spindle's axis
        if (m.kind == MoveKind::Dwell) {
            functions.push_back(m.function);
        }
    }
    CHECK(functions == std::vector{MachineFunction::ChuckOpen, MachineFunction::ChuckClose});
    REQUIRE(tp.moves.size() == 5);
    // Onto the bar at the plunge feed, out by the pull at the pull feed, off it again.
    CHECK(tp.moves[0].end.z() == Approx(front - grip));
    CHECK(tp.moves[0].feedRate == Approx(preset.plungeFeed));
    CHECK(tp.moves[2].end.z() == Approx(front - grip + kPull));
    CHECK(tp.moves[2].feedRate == Approx(preset.pullFeed));
    CHECK(tp.moves[4].end.z() == Approx(front + kPull + 2.0));
    CHECK(tp.moves[4].feedRate == Approx(preset.plungeFeed));

    SECTION("the bar feeder's push and a dwell at the chuck, when asked") {
        f.setParam("bar_feeder", true);
        f.setParam("dwell", 0.5);
        f.setParam("grip", 10.0);
        const toolpath::Toolpath fed = f.generate(f.pullId);
        functions.clear();
        for (const toolpath::Move& m : fed.moves) {
            if (m.kind == MoveKind::Dwell) {
                functions.push_back(m.function);
                CHECK(m.dwell == (m.function == MachineFunction::ChuckOpen ||
                                          m.function == MachineFunction::ChuckClose
                                      ? 0.5
                                      : 0.0));
            }
        }
        CHECK(functions == std::vector{MachineFunction::BarFeederOn, MachineFunction::ChuckOpen,
                                       MachineFunction::ChuckClose, MachineFunction::BarFeederOff});
        CHECK(fed.moves[0].end.z() == Approx(front - 10.0));
    }
    SECTION("refused, naming the value: no pull length, a grip deeper than the jaws, a bar too "
            "big, no pull feed, another tool") {
        f.setParam("pull_length", 0.0);
        CHECK_THROWS_WITH(f.generate(f.pullId), ContainsSubstring("Pull length"));
        f.setParam("pull_length", kPull);
        f.setParam("grip", 30.0);
        CHECK_THROWS_WITH(f.generate(f.pullId), ContainsSubstring("jaws of T7 are 25.000 mm"));
        f.setParam("grip", 10.0);
        f.doc.modify("small", [&](Project& p) { p.tools.tools[2].mill.tipDiameter = 25.0; });
        CHECK_THROWS_WITH(f.generate(f.pullId), ContainsSubstring("takes a bar of at most"));
        f.doc.modify("feed", [&](Project& p) {
            p.tools.tools[2].mill.tipDiameter = 40.0;
            p.tools.tools[2].presets[0].pullFeed = 0.0;
        });
        CHECK_THROWS_WITH(f.generate(f.pullId), ContainsSubstring("pull feed"));
        f.doc.modify("insert", [&](Project& p) {
            Operation* op = p.findOperation(f.pullId);
            op->toolId = p.tools.tools[0].id;
            op->presetId = p.tools.tools[0].presets[0].id;
        });
        CHECK_THROWS_WITH(f.generate(f.pullId), ContainsSubstring("needs a bar puller"));
    }
    SECTION("a pull at the cycle's start: what follows works on the bar where it stands") {
        f.doc.modify("first", [&](Project& p) {
            auto& ops = p.findSetup(f.setupId)->operations;
            std::rotate(ops.begin(), ops.end() - 1, ops.end());
            ops.front().parameters["pull_length"] = 10.0;
        });
        const toolpath::Toolpath pull = f.generate(f.pullId);
        CHECK(pull.moves[0].end.z() == Approx(1.0 - grip)); // the bar's front, 1 mm over the part
        const toolpath::Toolpath face = f.generate(f.setup().operations[1].id);
        double highest = -1e9;
        for (const toolpath::Move& m : face.moves) {
            highest = std::max(highest, m.end.z());
        }
        CHECK(highest > 11.0); // faces from 11 mm, where the bar now ends
    }
}

TEST_CASE("the simulation moves the stock with the pull", "[document][lathe][barpull]") {
    BarJob f;
    SimulationOptions options;
    const SetupSimulation sim = simulateSetup(f.doc, f.setupId, options, f.cancel, f.progress);
    REQUIRE(sim.operations.size() == 5);
    for (const OperationSimulation& os : sim.operations) {
        INFO(os.name << ": " << os.error);
        CHECK(os.error.empty());
    }
    const auto front = [](const sim::LatheStock& s) {
        double z = -1e9;
        for (const auto& r : s.heldBar()) {
            z = std::max(z, r.bounds().max.x());
        }
        return z;
    };
    const sim::LatheStock& beforePull = *sim.operations[3].latheAfter;
    const sim::LatheStock& afterPull = *sim.operations[4].latheAfter;
    CHECK(front(beforePull) == Approx(-kLength - kPartOffWidth).margin(1e-6));
    // The next cycle's bar stands where this one's did: nothing to say.
    CHECK(front(afterPull) == Approx(1.0).margin(1e-6));
    CHECK(front(*sim.latheInitial) == Approx(1.0).margin(1e-6));
    for (const std::string& w : sim.warnings) {
        CHECK_THAT(w, !ContainsSubstring("after the bar pull"));
    }
    // The part came off before the pull: it is compared as it was then.
    REQUIRE(sim.comparison);
    CHECK(sim.comparison->gougeRays == 0);
    // Playback: the bar as far as it has come.
    const toolpath::Toolpath& path = sim.operations[4].toolpath;
    const auto at = [&](std::size_t move) {
        return front(*simulatePartialLathe(sim, 4, move));
    };
    CHECK(at(0) == Approx(-kLength - kPartOffWidth).margin(1e-6));
    CHECK(at(3) == Approx(1.0).margin(1e-6));
    CHECK(at(path.moves.size()) == Approx(1.0).margin(1e-6));

    SECTION("a pull that leaves the bar elsewhere than this setup starts it is said") {
        f.setParam("pull_length", kPull - 2.0);
        const SetupSimulation shorter =
            simulateSetup(f.doc, f.setupId, options, f.cancel, f.progress);
        CHECK(
            std::any_of(shorter.warnings.begin(), shorter.warnings.end(), [](const std::string& w) {
                return w.find("2.000 mm shorter") != std::string::npos;
            }));
    }
}

TEST_CASE("bar pull posts on the Fanuc-style lathe", "[document][lathe][barpull][golden]") {
    BarJob f;
    SECTION("the generic post asks for the machine's chuck codes") {
        const PostOutput out = f.post("fanuc-generic-lathe");
        REQUIRE_FALSE(out.ok());
        CHECK_THAT(out.errors.front(), ContainsSubstring("Chuck open"));
    }
    SECTION("with them it writes the golden program, which reads back as the toolpaths") {
        const PostOutput out = f.post("fanuc-generic-lathe", kChuckCodes);
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        checkGolden("lathe-bar-pull", out.gcode);
        // Feeds per minute while the spindle stands still, the chuck codes around the pull.
        const auto at = [&](const std::string& s) {
            return out.gcode.find(s);
        };
        REQUIRE(at("G98") != std::string::npos);
        CHECK(at("M05") < at("M11"));
        CHECK(at("M11") < at("M10"));
        double deviation = 0.0;
        const auto differs = post::compareBackplot(
            out.toolpaths, post::backplot(out.gcode, {true}).path, 2e-3, &deviation, false);
        INFO(differs.value_or(""));
        CHECK_FALSE(differs.has_value());
    }
    SECTION("the part loop: run again, or count parts") {
        Json options = kChuckCodes;
        options["program"]["partLoop"] = "repeat";
        PostOutput out = f.post("fanuc-generic-lathe", options);
        REQUIRE(out.ok());
        CHECK_THAT(out.gcode, ContainsSubstring("M99"));
        CHECK_THAT(out.gcode, !ContainsSubstring("M30"));
        options["program"]["partLoop"] = "count";
        out = f.post("fanuc-generic-lathe", options);
        REQUIRE_FALSE(out.ok());
        CHECK_THAT(out.errors.front(), ContainsSubstring("Part counter"));
        options["program"]["partCount"] = 25;
        options["program"]["partCounter"] = "#500";
        out = f.post("fanuc-generic-lathe", options);
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        for (const char* text :
             {"#500=#500+1", "IF[#500GE25]GOTO9999", "M99", "N9999 #500=0", "M30"}) {
            INFO(text);
            CHECK_THAT(out.gcode, ContainsSubstring(text));
        }
    }
    SECTION("the bar feeder's codes, when it pushes") {
        f.setParam("bar_feeder", true);
        Json options = kChuckCodes;
        PostOutput out = f.post("fanuc-generic-lathe", options);
        REQUIRE_FALSE(out.ok());
        CHECK_THAT(out.errors.front(), ContainsSubstring("Bar feeder on"));
        options["program"]["barFeederOn"] = "M101";
        options["program"]["barFeederOff"] = "M102";
        out = f.post("fanuc-generic-lathe", options);
        REQUIRE(out.ok());
        const auto at = [&](const std::string& s) {
            return out.gcode.find(s);
        };
        CHECK(at("M101") < at("M11"));
        CHECK(at("M10") < at("M102"));
    }
    SECTION("the Haas-style post starts from Haas's chuck codes") {
        const PostOutput out = f.post("haas-lathe");
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        CHECK_THAT(out.gcode, ContainsSubstring("M11"));
        CHECK_THAT(out.gcode, ContainsSubstring("M10"));
    }
    SECTION("the NC program says what the machine does, and lowering will not open a turning "
            "chuck") {
        const PostOutput out = f.post("fanuc-generic-lathe", kChuckCodes);
        REQUIRE(out.ok());
        post::PostJob job;
        job.path = out.toolpaths.back();
        job.name = "Bar pull";
        const post::NcProgram program = post::lower({job}, "O1", 1, {});
        std::vector<std::string> ops;
        for (const post::NcInstr& i : program.instrs) {
            if (i.op == post::NcOp::Chuck) {
                ops.push_back(i.open ? "open" : "close");
            }
        }
        CHECK(ops == std::vector<std::string>{"open", "close"});
        const Json j = post::toJson(program);
        CHECK(j["version"] == post::kNcProgramJsonVersion); // chuck is version 7
        CHECK(post::ncProgramFromJson(j).instrs.size() == program.instrs.size());
        toolpath::Toolpath turning = job.path;
        turning.spindleSpeed = 10.0;
        job.path = turning;
        CHECK_THROWS_WITH(post::lower({job}, "O1", 1, {}),
                          ContainsSubstring("the chuck would open with the spindle turning"));
    }
}
