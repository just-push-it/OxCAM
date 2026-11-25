// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Drilling, tapping and reaming on a lathe's spindle axis (DL-0043).

#include "FixtureSpecs.hpp"
#include "TestProject.hpp"

#include <ocam/document/Generate.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/document/SimulationCache.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/geom2d/Clip.hpp>
#include <ocam/post/Post.hpp>
#include <ocam/sim/Compare.hpp>
#include <ocam/tooling/ToolLibrary.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>

using namespace ocam;
using namespace ocam::document;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
using toolpath::FeedClass;
using toolpath::MoveKind;

namespace {

/// A lathe setup on a fixture along +Z, its origin at the part's front, with a bar 1 mm over the
/// radius and the front and `tools` in the project.
struct LatheFixture {
    LatheFixture(const char* file, double front, std::vector<tooling::Tool> tools,
                 double back = 0.0) {
        const Uuid modelId =
            doc.importModel(std::filesystem::path(OCAM_FIXTURE_DIR) / file, cancel, progress);
        setupId = doc.addSetup(modelId);
        doc.modify("lathe", [&](Project& p) {
            Setup* s = p.findSetup(setupId);
            s->machine = "generic-2axis-lathe";
            s->originFrom.reset();
            s->wcs.origin = Vec3(0, 0, front);
            s->wcs.zAxis = Vec3::UnitZ();
            s->wcs.xAxis = Vec3::UnitX();
            s->stock.kind = StockKind::Box;
            s->stock.size = StockSize::OffsetFromPart;
            s->stock.offsetSide = 1.0;
            s->stock.offsetTop = 1.0;
            s->stock.offsetBottom = back;
            p.tools.tools = std::move(tools);
        });
    }
    Uuid add(const char* type, std::size_t tool, Json params = Json::object(),
             PresetOverrides overrides = {}) {
        Operation op;
        op.name = type;
        op.type = type;
        const tooling::Tool& t = doc.project().tools.tools.at(tool);
        op.toolId = t.id;
        op.presetId = t.presets.front().id;
        op.parameters = std::move(params);
        op.presetOverrides = overrides;
        op.heights = defaultHeights(op);
        const Uuid id = op.id;
        doc.modify("add", [&](Project& p) { p.findSetup(setupId)->operations.push_back(op); });
        return id;
    }
    const Operation& op(const Uuid& id) const { return *doc.project().findOperation(id); }
    toolpath::Toolpath generate(const Uuid& id) {
        return generateToolpath(doc, *doc.project().findSetup(setupId), op(id), cancel, progress);
    }
    /// The tip's lowest point of an operation with one hole.
    double bottom(const Uuid& id) {
        const toolpath::Toolpath tp = generate(id);
        INFO((tp.warnings.empty() ? std::string() : tp.warnings.front()));
        REQUIRE(tp.cycles.size() == 1);
        return tp.cycles[0].bottomZ;
    }
    Document doc;
    Uuid setupId;
    CancelToken cancel;
    NullProgress progress;
};

tooling::Tool drill(int number, double diameter) {
    tooling::Tool t = tooling::makeDefaultTool(tooling::ToolType::Drill, number);
    t.mill.diameter = diameter;
    t.mill.shankDiameter = diameter;
    t.mill.fluteLength = 70.0;
    t.mill.overallLength = 110.0;
    return document::testing::withPreset(t);
}

tooling::Tool insert(int number) {
    tooling::Tool t = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, number);
    t.insert.noseRadius = 0.4;
    return document::testing::withPreset(t);
}

tooling::Tool tap(int number) {
    tooling::Tool t = tooling::makeDefaultTool(tooling::ToolType::Tap, number);
    t.mill.diameter = 10.0;
    t.mill.shankDiameter = 10.0;
    t.mill.fluteLength = 30.0;
    t.mill.overallLength = 80.0;
    t.mill.threadPitch = 1.5;
    return document::testing::withPreset(t);
}

double tipLength(const tooling::Tool& t) {
    return t.mill.diameter / 2.0 / std::tan(t.mill.tipAngle / 2.0);
}

} // namespace

TEST_CASE("a drill on a lathe goes as deep as the part lets it", "[document][lathe][holes]") {
    using namespace fixtures::bored;
    SECTION("a blind bore: its tip stops at the bottom, whatever the depth mode") {
        LatheFixture f("bored.step", kLength, {drill(2, 16.0)});
        const toolpath::Toolpath tp = f.generate(f.add("drill", 0));
        REQUIRE(tp.cycles.size() == 1);
        CHECK(tp.cycles[0].bottomZ == Approx(-kBoreDepth).margin(1e-3));
        CHECK(tp.cycles[0].topZ == Approx(1.0)); // the bar's front
        CHECK(tp.cycles[0].retractZ == Approx(3.0));
        CHECK(tp.surfaceSpeed == 0.0); // a constant spindle speed
        CHECK(tp.spindleSpeed > 0.0);
        CHECK_FALSE(tp.isTurning());
        for (const auto& m : tp.moves) {
            CHECK(m.end.x() == Approx(0.0).margin(1e-12));
            CHECK(m.end.y() == Approx(0.0).margin(1e-12));
        }
        CHECK(std::none_of(tp.warnings.begin(), tp.warnings.end(), [](const std::string& w) {
            return w.find("cuts into the part") != std::string::npos;
        }));
    }
    SECTION("after facing it starts at the faced front") {
        LatheFixture f("bored.step", kLength, {insert(1), drill(2, 16.0)});
        f.add("turn_face", 0);
        const toolpath::Toolpath tp = f.generate(f.add("drill", 1));
        REQUIRE(tp.cycles.size() == 1);
        CHECK(tp.cycles[0].topZ == Approx(0.0).margin(1e-3));
        CHECK(tp.cycles[0].retractZ == Approx(2.0).margin(1e-3));
    }
    SECTION("a drill the bore cannot take is refused") {
        LatheFixture f("bored.step", kLength, {drill(2, 25.0)});
        CHECK_THROWS_WITH(f.generate(f.add("drill", 0)),
                          ContainsSubstring("no hole on the spindle axis"));
    }
    SECTION("the tolerance of hole matching: a drill 0.04 over the bore fits") {
        LatheFixture f("bored.step", kLength, {drill(2, kBoreDiameter + 0.04)});
        CHECK(f.bottom(f.add("drill", 0)) == Approx(-kBoreDepth).margin(1e-3));
    }
    SECTION("a set depth is from the part's front, in the chosen depth mode") {
        LatheFixture f("bored.step", kLength, {drill(2, 16.0)});
        const double tip = tipLength(f.doc.project().tools.tools[0]);
        const Uuid full = f.add("drill", 0, Json{{"hole_depth", 10.0}});
        CHECK(f.bottom(full) == Approx(-10.0 - tip));
        f.doc.modify("tip",
                     [&](Project& p) { p.findOperation(full)->parameters["depth_mode"] = "tip"; });
        CHECK(f.bottom(full) == Approx(-10.0));
    }
    SECTION("a hole already drilled there is nothing to do") {
        LatheFixture f("bored.step", kLength, {drill(2, 16.0)});
        f.add("drill", 0, Json{{"hole_depth", 20.0}});
        const toolpath::Toolpath tp = f.generate(f.add("drill", 0, Json{{"hole_depth", 10.0}}));
        CHECK(tp.cycles.empty());
        CHECK_THAT(tp.warnings.back(), ContainsSubstring("no depth"));
    }
    SECTION("a depth past the bore is drilled and warned of") {
        LatheFixture f("bored.step", kLength, {drill(2, 16.0)});
        const toolpath::Toolpath tp = f.generate(
            f.add("drill", 0, Json{{"hole_depth", kBoreDepth + 5.0}, {"depth_mode", "tip"}}));
        CHECK(tp.cycles[0].bottomZ == Approx(-kBoreDepth - 5.0));
        CHECK(std::any_of(tp.warnings.begin(), tp.warnings.end(), [](const std::string& w) {
            return w.find("cuts into the part") != std::string::npos;
        }));
    }
}

TEST_CASE("a through bore is drilled through", "[document][lathe][holes]") {
    using namespace fixtures::bushing;
    LatheFixture f("bushing.step", kLength, {drill(2, kBoreDiameter)}, 3.0);
    const double tip = tipLength(f.doc.project().tools.tools[0]);
    const toolpath::Toolpath tp = f.generate(f.add("drill", 0));
    REQUIRE(tp.cycles.size() == 1);
    // Full diameter (the default) through the back, plus the breakthrough.
    CHECK(tp.cycles[0].bottomZ == Approx(-kLength - tip - 1.0).margin(1e-3));
    // Deeper than 3 D: pecked, automatically, as in milling.
    CHECK(tp.cycles[0].type == toolpath::CycleType::Peck);
}

TEST_CASE("a part with nothing on the axis needs a depth", "[document][lathe][holes]") {
    using namespace fixtures::shaft;
    tooling::Tool centre = tooling::makeDefaultTool(tooling::ToolType::CenterDrill, 2);
    centre.mill.diameter = 6.3;
    centre.mill.tipDiameter = 2.5;
    centre.mill.tipLength = 3.0;
    centre.mill.fluteLength = 10.0;
    LatheFixture f("shaft.step", kMajorLength + kMinorLength,
                   {document::testing::withPreset(centre)});
    CHECK_THROWS_WITH(f.generate(f.add("drill", 0)),
                      ContainsSubstring("set the operation's depth"));
    // A centre hole cuts the part, on purpose: the simulation shows it, and it is warned of.
    const Uuid id = f.add("drill", 0, Json{{"hole_depth", 5.0}, {"depth_mode", "tip"}});
    const toolpath::Toolpath tp = f.generate(id);
    CHECK(tp.cycles[0].bottomZ == Approx(-5.0));
    CHECK_THAT(tp.warnings.back(), ContainsSubstring("cuts into the part"));
}

TEST_CASE("what a lathe cannot do on its axis is refused", "[document][lathe][holes]") {
    using namespace fixtures::bored;
    LatheFixture f("bored.step", kLength,
                   {drill(2, 16.0),
                    document::testing::withPreset(tooling::makeFlatEndMill(3, 10.0, 20.0, 60.0))});
    SECTION("thread milling, on a machine without live tooling") {
        CHECK_THROWS_WITH(f.generate(f.add("threadmill", 0)), ContainsSubstring("live tooling"));
    }
    SECTION("helical boring") {
        CHECK_THROWS_WITH(f.generate(f.add("bore", 1)), ContainsSubstring("live tooling"));
    }
    SECTION("a mill in a drilling operation") {
        CHECK_THROWS_WITH(f.generate(f.add("drill", 1)), ContainsSubstring("needs a drill"));
    }
    SECTION("selected faces") {
        const Uuid id = f.add("drill", 0);
        f.doc.modify("faces", [&](Project& p) {
            Operation* o = p.findOperation(id);
            ShapeRef ref;
            ref.modelId = *p.findSetup(f.setupId)->modelId;
            o->geometry.push_back(ref);
        });
        CHECK_THROWS_WITH(f.generate(id), ContainsSubstring("selected faces"));
    }
    SECTION("a tap cycle without a tap") {
        CHECK_THROWS_WITH(f.generate(f.add("drill", 0, Json{{"cycle", "tap"}})),
                          ContainsSubstring("Tap operation"));
    }
    SECTION("a move off the axis, in the machine") {
        toolpath::Toolpath tp = f.generate(f.add("drill", 0));
        const machine::Machine& machine =
            machineFor(f.doc.project(), *f.doc.project().findSetup(f.setupId));
        CHECK(machine::solve(machine, tp).errors.empty());
        tp.moves.back().end.x() = 5.0;
        CHECK_FALSE(machine::solve(machine, tp).errors.empty());
    }
}

TEST_CASE("tapping and reaming need a hole first", "[document][lathe][holes]") {
    using namespace fixtures::bushing;
    tooling::Tool reamer = tooling::makeDefaultTool(tooling::ToolType::Reamer, 4);
    reamer.mill.diameter = kBoreDiameter;
    reamer.mill.shankDiameter = kBoreDiameter;
    reamer.mill.fluteLength = 60.0;
    reamer.mill.overallLength = 100.0;
    LatheFixture f("bushing.step", kLength,
                   {drill(2, kBoreDiameter), tap(3), document::testing::withPreset(reamer)}, 3.0);
    SECTION("on solid stock they are refused") {
        CHECK_THROWS_WITH(f.generate(f.add("tap", 1)), ContainsSubstring("drill the hole first"));
        CHECK_THROWS_WITH(f.generate(f.add("bore", 2)), ContainsSubstring("drill the hole first"));
    }
    SECTION("a disabled drill does not count") {
        const Uuid first = f.add("drill", 0);
        f.doc.modify("off", [&](Project& p) { p.findOperation(first)->enabled = false; });
        CHECK_THROWS_WITH(f.generate(f.add("tap", 1)), ContainsSubstring("drill the hole first"));
    }
    SECTION("after the drill they run, to the depth asked for") {
        f.add("drill", 0);
        const toolpath::Toolpath thread = f.generate(f.add("tap", 1, Json{{"thread_depth", 15.0}}));
        REQUIRE(thread.cycles.size() == 1);
        CHECK(thread.cycles[0].type == toolpath::CycleType::Tap);
        CHECK(thread.cycles[0].bottomZ == Approx(-15.0));
        CHECK(thread.cycles[0].pitch == Approx(1.5));
        CHECK(thread.cycles[0].feedRate == Approx(1.5 * thread.spindleSpeed));
        const toolpath::Toolpath ream = f.generate(f.add("bore", 2));
        INFO((ream.warnings.empty() ? std::string() : ream.warnings.front()));
        REQUIRE(ream.cycles.size() == 1);
        CHECK(ream.cycles[0].type == toolpath::CycleType::Bore);
    }
    SECTION("the program posts the tap as a cycle and the drill as moves, and verifies") {
        f.add("drill", 0);
        f.add("tap", 1, Json{{"thread_depth", 15.0}});
        const machine::Machine& machine =
            machineFor(f.doc.project(), *f.doc.project().findSetup(f.setupId));
        const post::PostScript& script = defaultPostFor(machine);
        const PostOutput out = postSetup(f.doc, f.setupId, machine, script, f.cancel, f.progress);
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        INFO(out.gcode);
        CHECK_THAT(out.gcode, ContainsSubstring("G84 X0. Z-15. F1.5"));
        CHECK_THAT(out.gcode, ContainsSubstring("G80"));
        CHECK_THAT(out.gcode, ContainsSubstring("G97 S"));
        CHECK_THAT(out.gcode, !ContainsSubstring("G83"));
        CHECK_THAT(out.gcode, !ContainsSubstring("G96"));
        post::BackplotOptions options;
        options.lathe = true;
        options.machine = &machine;
        const post::BackplotResult plotted = post::backplot(out.gcode, options);
        CHECK(post::compareBackplot(out.toolpaths, plotted.path, 1.5e-3, nullptr, false)
                  .value_or("") == "");
    }
    SECTION("a left-hand tap on the axis: M04, the post's cycle, or refused until it has one") {
        f.add("drill", 0);
        const Uuid thread = f.add("tap", 1, Json{{"thread_depth", 15.0}});
        const Uuid tapId = *f.doc.project().findOperation(thread)->toolId;
        f.doc.modify("left", [&](Project& p) { p.tools.findTool(tapId)->mill.leftHand = true; });
        CHECK(f.generate(thread).cycles[0].type == toolpath::CycleType::TapLeft);
        const machine::Machine& machine =
            machineFor(f.doc.project(), *f.doc.project().findSetup(f.setupId));
        const PostOutput refused =
            postSetup(f.doc, f.setupId, machine, post::builtinPost("fanuc-generic-lathe"), f.cancel,
                      f.progress);
        REQUIRE_FALSE(refused.ok());
        CHECK_THAT(refused.errors.front(), ContainsSubstring("Left-hand tap cycle"));
        const PostOutput out = postSetup(f.doc, f.setupId, machine,
                                         post::builtinPost("haas-lathe"), f.cancel, f.progress);
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        INFO(out.gcode);
        CHECK_THAT(out.gcode, ContainsSubstring("G184 X0. Z-15. F1.5"));
        CHECK_THAT(out.gcode, ContainsSubstring("M04"));
    }
    SECTION("a post without a tap cycle refuses to tap") {
        const Uuid id = f.add("drill", 0);
        (void)id;
        post::PostJob job;
        job.path = f.generate(f.add("tap", 1));
        post::LowerOptions options;
        options.cannedCycles = false;
        CHECK_THROWS_WITH(post::lower({job}, "O1", 1, options), ContainsSubstring("no tap cycle"));
        options.cannedCycles = true;
        options.cycleTypes = {toolpath::CycleType::Drill};
        CHECK_THROWS_WITH(post::lower({job}, "O1", 1, options), ContainsSubstring("no tap cycle"));
        options.cycleTypes = {toolpath::CycleType::Tap};
        CHECK_NOTHROW(post::lower({job}, "O1", 1, options));
    }
    SECTION("the simulation cuts the drill's bore, and a tap removes nothing") {
        f.add("drill", 0);
        f.add("tap", 1);
        SimulationOptions options;
        const SetupSimulation sim = simulateSetup(f.doc, f.setupId, options, f.cancel, f.progress);
        REQUIRE(sim.operations.size() == 2);
        CHECK(sim.operations[0].error == "");
        CHECK(sim.operations[1].error == "");
        CHECK(sim.operations[0].removedVolume > 0.0);
        CHECK(sim.operations[1].removedVolume == Approx(0.0).margin(1e-9));
        CHECK(sim.comparison);
    }
}

TEST_CASE("a part bored from solid bar comes out to size", "[document][lathe][holes]") {
    using namespace fixtures::bored;
    tooling::Tool boring = insert(3);
    LatheFixture f("bored.step", kLength, {insert(1), drill(2, 16.0), boring});
    f.add("turn_face", 0);
    f.add("drill", 1);
    const Uuid outside = f.add("turn_rough", 0, Json::object(), document::testing::stepDown(1.5));
    const Uuid inside =
        f.add("turn_rough", 2, Json{{"side", "inside"}}, document::testing::stepDown(1.0));
    f.add("turn_finish", 0);
    f.add("turn_finish", 2, Json{{"side", "inside"}});
    SECTION("outside passes stay out of the bore, inside passes out of the wall") {
        const auto cuts = [](const toolpath::Toolpath& tp) {
            std::vector<double> radii;
            for (const auto& m : tp.moves) {
                if (m.kind == MoveKind::Linear && m.feedClass == FeedClass::Cut) {
                    radii.push_back(m.end.x());
                }
            }
            return radii;
        };
        // The profile passes round the corners of the bore's mouth, by the nose radius at most.
        const auto out = cuts(f.generate(outside));
        REQUIRE_FALSE(out.empty());
        CHECK(*std::min_element(out.begin(), out.end()) > kShoulderDiameter / 2.0 - 0.5);
        const auto in = cuts(f.generate(inside));
        REQUIRE_FALSE(in.empty());
        CHECK(*std::max_element(in.begin(), in.end()) < kBoreDiameter / 2.0 + 0.5);
        // Down the bottom to the axis, and no further.
        CHECK(*std::min_element(in.begin(), in.end()) > -1e-9);
        // Nothing behind the bottom.
        for (const auto& m : f.generate(inside).moves) {
            CHECK(m.end.z() > -kBoreDepth - 1e-3);
        }
    }
    SECTION("nothing left, nothing gouged") {
        SimulationOptions options;
        const SetupSimulation sim = simulateSetup(f.doc, f.setupId, options, f.cancel, f.progress);
        for (const auto& op : sim.operations) {
            INFO(op.name << ": " << op.error);
            CHECK(op.error.empty());
        }
        REQUIRE(sim.comparison);
        CHECK(sim.comparison->maxGouge < 1e-2);
        CHECK_FALSE(sim.comparison->hasGouges());
        CHECK(sim.comparison->maxExcess < 0.25); // the nose radius in the corners at most
        // The simulation's stock takes the bore down to the bottom of the part's.
        const auto& final = sim.operations.back().latheAfter;
        REQUIRE(final);
        CHECK(final->volume() == Approx(fixtures::bored::kVolume).epsilon(0.01));
    }
    SECTION("an earlier hole changes a later turning operation; an earlier turning one does not") {
        const auto before = f.generate(inside);
        const std::string key =
            toolpathInputKey(f.doc, *f.doc.project().findSetup(f.setupId), f.op(inside));
        f.doc.modify("wider", [&](Project& p) {
            tooling::Tool& t = p.tools.tools[1];
            t.mill.diameter = 12.0;
            t.mill.shankDiameter = 12.0;
        });
        CHECK(toolpathInputKey(f.doc, *f.doc.project().findSetup(f.setupId), f.op(inside)) != key);
        const auto after = *cachedToolpath(f.doc, *f.doc.project().findSetup(f.setupId),
                                           f.op(inside), f.cancel, f.progress);
        CHECK(after.moves.size() > before.moves.size());

        const std::string again =
            toolpathInputKey(f.doc, *f.doc.project().findSetup(f.setupId), f.op(inside));
        f.doc.modify("rename", [&](Project& p) {
            p.findSetup(f.setupId)->operations.front().parameters["stock_to_leave"] = 0.1;
        });
        CHECK(toolpathInputKey(f.doc, *f.doc.project().findSetup(f.setupId), f.op(inside)) ==
              again);
    }
}

TEST_CASE("a boring bar has to fit the bore", "[document][lathe][holes][holder]") {
    using namespace fixtures::bored;
    LatheFixture f("bored.step", kLength, {insert(1), drill(2, 16.0), insert(3)});
    f.add("turn_face", 0);
    f.add("drill", 1);
    const Uuid inside =
        f.add("turn_rough", 2, Json{{"side", "inside"}}, document::testing::stepDown(1.0));
    const auto holdIn = [&](const char* code) {
        f.doc.modify("holder", [&](Project& p) {
            tooling::Holder holder = tooling::makeTurningHolder(*tooling::parseDesignation(code));
            tooling::fitToHolder(p.tools.tools[2], holder);
            p.tools.tools[2].holderId = holder.id;
            p.tools.holders = {holder};
        });
    };
    SECTION("too big for the part's bore") {
        holdIn("S25T-PCLNR 12");
        CHECK_THROWS_WITH(f.generate(inside), ContainsSubstring("the holder of T3"));
        CHECK_THROWS_WITH(f.generate(inside), ContainsSubstring("the part"));
    }
    SECTION("a small one runs to the axis, touching neither the part nor the drilled stock") {
        holdIn("S08H-SCLCR 06"); // reaches the axis at the bottom: far side 9 from the tip
        const toolpath::Toolpath tp = f.generate(inside);
        CHECK_FALSE(tp.moves.empty());
        SimulationOptions options;
        const SetupSimulation sim = simulateSetup(f.doc, f.setupId, options, f.cancel, f.progress);
        REQUIRE(sim.operations.size() == 3);
        CHECK(sim.operations[2].error == "");
        CHECK(sim.operations[2].collisions.empty());
    }
}

TEST_CASE("a solid boring bar", "[document][lathe][solid]") {
    using namespace fixtures::bushing;
    // Haas 03-3010 with a longer reach than the catalogue's, to run the bushing's whole length.
    tooling::Tool bar = tooling::makeSolidBoringBar(3);
    CHECK(bar.validate().empty());
    bar.solid->overallLength = 80.0;
    bar.solid->headLength = 50.0;
    bar.solid->maxOverhang = 60.0;
    LatheFixture f("bushing.step", kLength, {drill(2, 7.0), bar}, 3.0);
    f.add("drill", 0);
    const auto rough = [&](double ap = 0.2) {
        return f.add("turn_rough", 1, Json{{"side", "inside"}}, document::testing::stepDown(ap));
    };
    const auto setBar = [&](const auto& change) {
        f.doc.modify("bar", [&](Project& p) { change(*p.tools.tools[1].solid); });
    };
    SECTION("it runs, with no holder, and comes out to size") {
        const Uuid id = rough();
        CHECK_FALSE(f.generate(id).moves.empty());
        f.add("turn_finish", 1, Json{{"side", "inside"}});
        SimulationOptions options;
        const SetupSimulation sim = simulateSetup(f.doc, f.setupId, options, f.cancel, f.progress);
        for (const auto& op : sim.operations) {
            INFO(op.name << ": " << op.error);
            CHECK(op.error.empty());
            CHECK(op.collisions.empty());
        }
        // Along the part the inside is the part's bore; behind it the drill's point stays.
        const geom2d::Regions along = geom2d::intersect(
            sim.operations.back().latheAfter->profile(),
            {{geom2d::Polyline::rectangle(Vec2(-kLength + 0.5, 0.0), Vec2(0.0, 100.0)), {}}},
            {1e-3});
        REQUIRE(along.size() == 1);
        CHECK(along.front().bounds().min.y() == Approx(kBoreDiameter / 2.0).margin(0.05));
    }
    SECTION("outside it is refused") {
        CHECK_THROWS_WITH(f.generate(f.add("turn_finish", 1)), ContainsSubstring("cuts inside"));
    }
    SECTION("more than its T-MAX is refused, not reduced") {
        CHECK_THROWS_WITH(f.generate(rough(1.0)), ContainsSubstring("maximum 0.500 mm"));
    }
    SECTION("further than its overhang is refused") {
        setBar([](tooling::SolidBar& s) { s.maxOverhang = 30.5; });
        CHECK_THROWS_WITH(f.generate(rough()), ContainsSubstring("overhang"));
    }
    SECTION("a bore smaller than its minimum is refused, by its head across the axis") {
        setBar([](tooling::SolidBar& s) { s.minBore = 9.0; });
        CHECK_THROWS_WITH(f.generate(rough()), ContainsSubstring("the part"));
    }
    SECTION("the file keeps it, and a holder is not for it") {
        tooling::ToolLibrary lib;
        lib.tools = {bar};
        const Json j = tooling::toJson(lib);
        const tooling::ToolLibrary back = tooling::toolLibraryFromJson(j);
        REQUIRE(back.tools.size() == 1);
        CHECK(back.tools[0].solid == bar.solid);
        tooling::Tool held = bar;
        held.holderId = Uuid::generate();
        CHECK_FALSE(held.validate().empty());
    }
}

TEST_CASE("a through drill stops at the chuck", "[document][lathe][holes]") {
    // The shaft-in-a-chuck project's bar and chuck, with a drill aimed through them.
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "chuck_project.ocam");
    const Uuid setupId = doc.project().setups.front().id;
    // Wider than the chuck's through bore.
    const tooling::Tool t = drill(9, fixtures::chuck::kBore + 2.0);
    Operation op;
    op.name = "deep";
    op.type = "drill";
    op.toolId = t.id;
    op.presetId = t.presets.front().id;
    op.parameters = Json{{"hole_depth", 200.0}};
    op.heights = defaultHeights(op);
    doc.modify("a drill", [&](Project& p) {
        tooling::Tool longer = t;
        longer.mill.fluteLength = 250.0;
        longer.mill.overallLength = 300.0;
        p.tools.tools.push_back(longer);
        p.findSetup(setupId)->operations.push_back(op);
    });
    const Setup& setup = *doc.project().findSetup(setupId);
    CHECK_THROWS_WITH(generateToolpath(doc, setup, setup.operations.back(), cancel, progress),
                      ContainsSubstring("of the chuck"));
}

TEST_CASE("threads inside, left hand, multi-start and tapered", "[document][lathe][thread]") {
    using namespace fixtures::bushing;
    tooling::Tool insert = tooling::makeDefaultTool(tooling::ToolType::ThreadingInsert, 3);
    insert.insert.threadPitch = 1.5;
    insert = document::testing::withPreset(insert);
    LatheFixture f("bushing.step", kLength, {drill(2, kBoreDiameter), insert}, 3.0);
    f.add("drill", 0);
    const auto threadMoves = [](const toolpath::Toolpath& tp) {
        std::vector<toolpath::Move> moves;
        for (const auto& m : tp.moves) {
            if (m.pitch > 0.0) {
                moves.push_back(m);
            }
        }
        return moves;
    };
    SECTION("inside: from the bore outwards, towards the chuck, at a constant spindle speed") {
        const auto tp =
            f.generate(f.add("turn_thread", 1, Json{{"side", "inside"}, {"length", 20.0}}));
        CHECK(tp.tipQuadrant == 2);
        CHECK(tp.surfaceSpeed == 0.0);
        const auto cuts = threadMoves(tp);
        REQUIRE(cuts.size() == 7); // 6 passes and a spring pass
        CHECK(cuts.front().end.x() > kBoreDiameter / 2.0);
        CHECK(cuts.back().end.x() == Approx(kBoreDiameter / 2.0 + 0.5413 * 1.5));
        CHECK(cuts.back().end.z() == Approx(-20.0));
        CHECK(cuts.back().pitch == Approx(1.5));
        // Out of the cut, towards the axis.
        double lowest = 1e9;
        for (const auto& m : tp.moves) {
            lowest = std::min(lowest, m.end.x());
        }
        CHECK(lowest < kBoreDiameter / 2.0);
        CHECK(lowest > 0.0);
    }
    SECTION("inside on solid stock is refused") {
        LatheFixture solid("bushing.step", kLength, {insert}, 3.0);
        CHECK_THROWS_WITH(solid.generate(solid.add("turn_thread", 0, Json{{"side", "inside"}})),
                          ContainsSubstring("needs a bore"));
    }
    SECTION("left hand: away from the chuck") {
        const auto tp = f.generate(f.add("turn_thread", 1,
                                         Json{{"major_diameter", kOuterDiameter},
                                              {"length", 20.0},
                                              {"hand", "left"},
                                              {"start_offset", 5.0}}));
        const auto cuts = threadMoves(tp);
        REQUIRE_FALSE(cuts.empty());
        CHECK(cuts.front().end.z() == Approx(-5.0));
        CHECK_THAT(tp.warnings.front(), ContainsSubstring("relief groove"));
        // It starts behind the thread: the run-up is on the back side.
        double back = 1e9;
        for (const auto& m : tp.moves) {
            back = std::min(back, m.end.z());
        }
        CHECK(back == Approx(-25.0 - 3.0 * 1.5));
    }
    SECTION("two starts: the lead is two pitches, the feed follows, the start moves by a pitch") {
        const auto tp = f.generate(
            f.add("turn_thread", 1,
                  Json{{"major_diameter", kOuterDiameter}, {"length", 20.0}, {"starts", 2}}));
        const auto cuts = threadMoves(tp);
        REQUIRE(cuts.size() == 14);
        CHECK(cuts.front().pitch == Approx(3.0));
        CHECK(cuts.front().feedRate == Approx(3.0 * tp.spindleSpeed));
        std::vector<double> starts;
        Vec3 pos = tp.start;
        for (const auto& m : tp.moves) {
            if (m.pitch > 0.0) {
                starts.push_back(pos.z());
            }
            pos = m.end;
        }
        CHECK(starts.front() == Approx(3.0 * 3.0));
        CHECK(starts.back() == Approx(3.0 * 3.0 + 1.5));
    }
    SECTION("tapered: the radius changes along the cut, run-up included") {
        const auto tp = f.generate(
            f.add("turn_thread", 1,
                  Json{{"major_diameter", kOuterDiameter}, {"length", 20.0}, {"taper", 1.0}}));
        const auto cuts = threadMoves(tp);
        REQUIRE_FALSE(cuts.empty());
        Vec3 pos = tp.start;
        for (const auto& m : tp.moves) {
            if (m.pitch > 0.0) {
                // 1 mm of radius in 20 mm: the same slope from the run-up to the end.
                CHECK((m.end.x() - pos.x()) / (pos.z() - m.end.z()) == Approx(1.0 / 20.0));
            }
            pos = m.end;
        }
    }
    SECTION("it posts, tapers included, and verifies") {
        f.add("turn_thread", 1, Json{{"side", "inside"}, {"length", 20.0}, {"starts", 2}});
        f.add("turn_thread", 1,
              Json{{"major_diameter", kOuterDiameter}, {"length", 20.0}, {"taper", 1.0}});
        const machine::Machine& machine =
            machineFor(f.doc.project(), *f.doc.project().findSetup(f.setupId));
        const PostOutput out =
            postSetup(f.doc, f.setupId, machine, defaultPostFor(machine), f.cancel, f.progress);
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        CHECK_THAT(out.gcode, ContainsSubstring("F3."));
        post::BackplotOptions options;
        options.lathe = true;
        options.machine = &machine;
        const post::BackplotResult plotted = post::backplot(out.gcode, options);
        CHECK(post::compareBackplot(out.toolpaths, plotted.path, 1.5e-3, nullptr, false)
                  .value_or("") == "");
    }
    SECTION("the simulation cuts it beside the profile, and the insert is drawn on its side") {
        f.add("turn_thread", 1, Json{{"side", "inside"}, {"length", 20.0}});
        SimulationOptions options;
        const SetupSimulation sim = simulateSetup(f.doc, f.setupId, options, f.cancel, f.progress);
        REQUIRE(sim.operations.size() == 2);
        CHECK(sim.operations[1].error == "");
        CHECK(sim.operations[1].removedVolume > 0.0);
        CHECK(sim::revolvedVolume(sim.operations[1].latheAfter->profile()) ==
              Approx(sim::revolvedVolume(sim.operations[0].latheAfter->profile())));
        geom2d::Aabb2 box = sim.operations[1].insert.bounds();
        CHECK(box.max.y() <= 1e-9); // below the programmed point: mirrored
    }
}

TEST_CASE("threads show in the stock, and read as no gouge", "[document][lathe]") {
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "shaft_project.ocam");
    const Uuid setupId = doc.project().setups.front().id;
    SimulationOptions options;
    const SetupSimulation sim = simulateSetup(doc, setupId, options, cancel, progress);
    REQUIRE(sim.operations.size() == 5);
    const auto& thread = sim.operations[3];
    REQUIRE(thread.error.empty());
    CHECK(sim.operations[2].latheAfter->threads().empty());
    const auto& after = *thread.latheAfter;
    CHECK_FALSE(after.threads().empty());
    CHECK(thread.removedVolume > 10.0);
    // The profile is as the finish left it: the comparison reads the same, with no gouge, however
    // deep the thread goes against a part drawn at its nominal diameter.
    CHECK(sim::revolvedVolume(after.profile()) ==
          Catch::Approx(sim::revolvedVolume(sim.operations[2].latheAfter->profile())));
    const auto compared = latheComparison(sim, after);
    REQUIRE(compared);
    CHECK_FALSE(compared->hasGouges());
    const sim::DisplayMesh shown = latheDisplayMesh(sim, 3, after, true, sim::kStockColour);
    // In the thread's colour where it cut, and nowhere red.
    const sim::Rgb cut = sim::operationColour(3);
    std::size_t coloured = 0, red = 0;
    for (const auto& c : shown.colours) {
        coloured += c == cut ? 1 : 0;
        red += c[0] > 200 && c[1] < 100 && c[2] < 100 ? 1 : 0;
    }
    CHECK(coloured > 1000);
    CHECK(red == 0);
    // The cache keeps it.
    const Setup& setup = *doc.project().findSetup(setupId);
    const auto back = decodeSimulation(encodeSimulation(sim, "key"), "key", doc, setup);
    REQUIRE(back);
    CHECK(back->operations[3].latheAfter->threads() == after.threads());
    CHECK(back->operations[3].latheAfter->volume() == Catch::Approx(after.volume()));
    // Part way through the operation, part of the thread.
    const auto partial =
        simulatePartialLathe(sim, 3, thread.toolpath.moves.size() / 2, nullptr);
    REQUIRE(partial);
    CHECK(partial->threads().size() < after.threads().size());
    CHECK_FALSE(partial->threads().empty());
}

TEST_CASE("a thread goes on with the stock to the next setup", "[document][lathe]") {
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "flip_project.ocam");
    REQUIRE(doc.project().setups.size() == 2);
    tooling::Tool thread = tooling::makeDefaultTool(tooling::ToolType::ThreadingInsert, 7);
    thread.insert.threadPitch = 1.5;
    doc.modify("thread", [&](Project& p) {
        p.tools.tools.push_back(thread);
        Operation op;
        op.name = "Thread";
        op.type = "turn_thread";
        op.toolId = thread.id;
        op.presetId = thread.presets.front().id;
        op.parameters = Json{{"length", 10.0}};
        p.setups.front().operations.push_back(op);
    });
    const Setup first = doc.project().setups.front();
    const Setup second = doc.project().setups.back();
    const auto one = simulateSetup(doc, first.id, {}, cancel, progress);
    REQUIRE(one.operations.back().error.empty());
    REQUIRE_FALSE(one.operations.back().latheAfter->threads().empty());
    const auto two = simulateSetup(doc, second.id, {}, cancel, progress);
    REQUIRE(two.latheInitial);
    CHECK(two.latheInitial->threads().size() == one.operations.back().latheAfter->threads().size());
    CHECK(two.latheInitial->threads().front().turn ==
          one.operations.back().latheAfter->threads().front().turn);
}

namespace {

struct LiveFixture {
    CancelToken cancel;
    NullProgress progress;
    Document doc =
        Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "live_project.ocam");

    const Setup& setup() const { return doc.project().setups.front(); }
    toolpath::Toolpath generate(std::size_t k) {
        return generateToolpath(doc, setup(), setup().operations[k], cancel, progress);
    }
    void machine(const char* id) {
        doc.modify("machine", [&](Project& p) { p.setups.front().machine = id; });
    }
    void drive(std::size_t k, const char* value) {
        doc.modify("drive",
                   [&](Project& p) { p.setups.front().operations[k].parameters["drive"] = value; });
    }
};

} // namespace

TEST_CASE("holes with a live tool", "[document][lathe][live]") {
    using namespace fixtures::livetool;
    LiveFixture f;
    constexpr std::size_t kBolts = 2, kCross = 3, kOffset = 4;
    SECTION("it goes where the holes are") {
        const toolpath::Toolpath bolts = f.generate(kBolts);
        CHECK(bolts.live);
        std::vector<Vec2> at;
        for (const auto& m : bolts.moves) {
            if (m.kind == toolpath::MoveKind::Cycle) {
                at.emplace_back(m.end.x(), m.end.y());
            }
        }
        REQUIRE(at.size() == 4);
        for (const Vec2& p : at) {
            CHECK(p.norm() == Catch::Approx(kBoltCircleRadius));
        }
        // Across the part, the tool's axis along +X in the WCS; through the axis at its height.
        const toolpath::Toolpath cross = f.generate(kCross);
        CHECK(cross.live);
        bool feeds = false;
        for (const auto& m : cross.moves) {
            CHECK((m.axis - Vec3::UnitX()).norm() < 1e-9);
            CHECK(std::abs(m.end.y()) < 1e-6);
            CHECK(m.end.z() == Catch::Approx(kCrossZ - kLength));
            feeds = feeds || m.kind == toolpath::MoveKind::Linear;
        }
        CHECK(feeds);
    }
    SECTION("the choice is the operation's: nothing makes it") {
        // Set to the spindle, a hole off the axis is refused, naming the setting.
        f.doc.modify("faces", [&](Project& p) {
            Operation& op = p.setups.front().operations[kCross];
            op.orientation.reset();
            op.geometry = {ShapeRef{}};
        });
        f.drive(kCross, "spindle");
        CHECK_THROWS_WITH(f.generate(kCross), ContainsSubstring("Tool drive"));
        // A tool orientation with the spindle too.
        LiveFixture g;
        g.drive(kCross, "spindle");
        CHECK_THROWS_WITH(g.generate(kCross), ContainsSubstring("Tool drive"));
    }
    SECTION("a machine without live tooling refuses it, and says what to do") {
        f.machine("generic-2axis-lathe");
        CHECK_THROWS_WITH(f.generate(kBolts), ContainsSubstring("no live tooling"));
        CHECK_THROWS_WITH(f.generate(kBolts), ContainsSubstring("Tool drive"));
    }
    SECTION("without a Y axis what crosses the part's axis still goes, by C alone") {
        f.machine("generic-lathe-live");
        CHECK_NOTHROW(f.generate(kBolts));
        CHECK_NOTHROW(f.generate(kCross));
        CHECK_THROWS_WITH(f.generate(kOffset), ContainsSubstring("no Y axis"));
        f.machine("generic-lathe-live-y");
        CHECK_NOTHROW(f.generate(kOffset));
    }
    SECTION("what a turret cannot reach") {
        const auto tilted = [&](const Vec3& z) {
            f.doc.modify("tilt", [&](Project& p) {
                ToolOrientation o;
                o.frame.zAxis = z;
                o.frame.xAxis = Vec3::UnitY();
                p.setups.front().operations[kCross].orientation = o;
            });
        };
        tilted(Vec3(1, 0, 1).normalized());
        CHECK_THROWS_WITH(f.generate(kCross), ContainsSubstring("off the spindle's"));
        tilted(-Vec3::UnitZ());
        CHECK_THROWS_WITH(f.generate(kCross), ContainsSubstring("from the front"));
    }
    SECTION("a live tool taps") {
        tooling::Tool tap = tooling::makeDefaultTool(tooling::ToolType::Tap, 8, 7.0);
        tap.mill.threadPitch = 1.0;
        f.doc.modify("tap", [&](Project& p) {
            p.tools.tools.push_back(tap);
            Operation op;
            op.name = "Tap";
            op.type = "tap";
            op.toolId = tap.id;
            op.presetId = tap.presets.front().id;
            op.parameters = Json{{"drive", "live"}};
            op.heights = defaultHeights(op);
            p.setups.front().operations.push_back(op);
        });
        CHECK_NOTHROW(f.generate(5));
        SECTION("with no hole to go into it is refused, naming the place") {
            f.doc.modify("no drill", [&](Project& p) {
                p.setups.front().operations[kBolts].enabled = false;
                p.setups.front().operations[kCross].enabled = false;
                p.setups.front().operations[kOffset].enabled = false;
            });
            CHECK_THROWS_WITH(f.generate(5), ContainsSubstring("drill the hole first"));
        }
    }
}

TEST_CASE("a live tool on the spindle's axis", "[document][lathe][live]") {
    LiveFixture f;
    tooling::Tool centre = tooling::makeDefaultTool(tooling::ToolType::Drill, 8);
    centre.mill.diameter = 10.0;
    centre.mill.shankDiameter = 10.0;
    centre.presets = {tooling::defaultPreset(centre)};
    for (const char* drive : {"spindle", "live"}) {
        LiveFixture g;
        g.doc.modify("hole", [&](Project& p) {
            p.tools.tools.push_back(centre);
            Operation op;
            op.name = "On the axis";
            op.type = "drill";
            op.toolId = centre.id;
            op.presetId = centre.presets.front().id;
            op.parameters = Json{{"drive", drive}, {"hole_depth", 10.0}};
            op.heights = defaultHeights(op);
            p.setups.front().operations.push_back(op);
        });
        const toolpath::Toolpath tp = g.generate(5);
        CHECK(tp.live == (std::string(drive) == "live"));
        for (const auto& m : tp.moves) {
            if (m.kind != toolpath::MoveKind::Dwell) {
                CHECK(m.end.head<2>().norm() < 1e-9);
            }
        }
        // Either way the stock takes the same hole.
        const SetupSimulation sim =
            simulateSetup(g.doc, g.setup().id, {}, g.cancel, g.progress);
        REQUIRE(sim.operations.back().error.empty());
        CHECK_FALSE(tp.moves.empty());
        CHECK(sim.operations.back().removedVolume > 0.0);
        CHECK(sim.operations.back().latheAfter->holes().size() ==
              sim.operations[4].latheAfter->holes().size());
    }
}

TEST_CASE("a live tool's speed and the program", "[document][lathe][live]") {
    LiveFixture f;
    const auto post = [&](const char* id) {
        const machine::Machine& m = machineFor(f.doc.project(), f.setup());
        return postSetup(f.doc, f.setup().id, m, post::builtinPost(id), f.cancel, f.progress);
    };
    SECTION("the machine's limit refuses it, and nothing clamps it") {
        machine::Machine fast = machine::genericLatheLiveY();
        fast.id = "slow-live";
        fast.liveMaxSpindle = 100.0 / 60.0;
        f.doc.modify("machine", [&](Project& p) {
            p.machines.push_back(fast);
            p.setups.front().machine = fast.id;
        });
        const auto out = post("haas-lathe");
        REQUIRE_FALSE(out.ok());
        CHECK_THAT(out.errors.front(), ContainsSubstring("live tool speed"));
    }
    SECTION("a generic post asks for the machine's codes") {
        const auto out = post("fanuc-generic-lathe");
        REQUIRE_FALSE(out.ok());
        CHECK_THAT(out.errors.front(), ContainsSubstring("your machine's codes"));
    }
    SECTION("with the codes it posts, and the program goes where the toolpaths do") {
        const auto out = post("haas-lathe");
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        for (const char* word : {"G98", "M154", "M133 P", "M135", "M155", "G99", "C90.", "C180.",
                                 "C270.", "Y8."}) {
            INFO(word);
            CHECK_THAT(out.gcode, ContainsSubstring(word));
        }
        // The turning before it stays per revolution; the live tool's speed is the preset's.
        const toolpath::Toolpath bolts = f.generate(2);
        CHECK_THAT(out.gcode, ContainsSubstring(fmt::format(
                                  "M133 P{:.0f}", bolts.spindleSpeed * 60.0)));
        double deviation = 0.0;
        const auto differs = post::compareBackplot(out.toolpaths, post::backplot(out.gcode, {true}).path,
                                                   2e-3, &deviation, false);
        (void)deviation;
        INFO(differs.value_or(""));
        CHECK_FALSE(differs.has_value());
    }
    SECTION("a tap goes on the control's own cycle") {
        f.doc.modify("taps", [&](Project& p) {
            for (const std::size_t from : {std::size_t{2}, std::size_t{3}}) {
                // Each fits its tap drill hole, Ø6 and Ø8.
                tooling::Tool tap = tooling::makeDefaultTool(
                    tooling::ToolType::Tap, 10 + static_cast<int>(from), from == 2 ? 7.0 : 9.0);
                tap.mill.threadPitch = 1.0;
                p.tools.tools.push_back(tap);
                Operation op = p.setups.front().operations[from];
                op.id = Uuid::generate();
                op.name = from == 2 ? "Tap the face" : "Tap across";
                op.type = "tap";
                op.toolId = tap.id;
                op.presetId = tap.presets.front().id;
                op.parameters = Json{{"drive", "live"}};
                op.geometry.clear();
                p.setups.front().operations.push_back(op);
            }
        });
        const toolpath::Toolpath tapped = f.generate(5);

        const auto out = post("haas-lathe");
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        INFO(out.gcode);
        CHECK_THAT(out.gcode, ContainsSubstring(fmt::format("S{:.0f}", tapped.spindleSpeed * 60.0)));
        CHECK_THAT(out.gcode, ContainsSubstring("G18 G95 C"));
        CHECK_THAT(out.gcode, ContainsSubstring("G19 G195 X"));
        CHECK_THAT(out.gcode, !ContainsSubstring(
                                  fmt::format("M133 P{:.0f}", tapped.spindleSpeed * 60.0)));
        double deviation = 0.0;
        const auto differs = post::compareBackplot(
            out.toolpaths, post::backplot(out.gcode, {true}).path, 2e-3, &deviation, false);
        INFO(differs.value_or(""));
        CHECK_FALSE(differs.has_value());

        // Left-hand taps: the tool says so, the same speed, Haas's reverse cycles.
        f.doc.modify("left hand", [&](Project& p) {
            for (std::size_t k = 5; k < 7; ++k) {
                p.tools.findTool(*p.setups.front().operations[k].toolId)->mill.leftHand = true;
            }
        });
        const auto left = post("haas-lathe");
        INFO((left.errors.empty() ? std::string() : left.errors.front()));
        REQUIRE(left.ok());
        CHECK_THAT(left.gcode, ContainsSubstring("G18 G186 C"));
        CHECK_THAT(left.gcode, ContainsSubstring("G19 G196 X"));
        CHECK_THAT(left.gcode, !ContainsSubstring("G195"));
        CHECK_THAT(left.gcode, ContainsSubstring(fmt::format("S{:.0f}", tapped.spindleSpeed * 60.0)));
        const auto backLeft = post::compareBackplot(
            left.toolpaths, post::backplot(left.gcode, {true}).path, 2e-3, &deviation, false);
        INFO(backLeft.value_or(""));
        CHECK_FALSE(backLeft.has_value());
    }
}

TEST_CASE("a part with holes turns to its envelope", "[document][lathe][live]") {
    using namespace fixtures::livetool;
    LiveFixture f;
    const LatheGeometry g = latheGeometry(f.doc, f.setup(), f.cancel);
    CHECK_FALSE(g.partIsRound);
    // The hole across the plane of the axis leaves no gap in what turning leaves.
    CHECK(geom2d::totalArea(g.part) == Catch::Approx(kLength * kDiameter / 2.0).epsilon(1e-3));
}

TEST_CASE("a live tool's rapids against the stock", "[document][lathe][live]") {
    using namespace fixtures::livetool;
    LiveFixture f;
    SECTION("the fixture's holes go clear") {
        const SetupSimulation sim = simulateSetup(f.doc, f.setup().id, {}, f.cancel, f.progress);
        for (const auto& op : sim.operations) {
            INFO(op.name);
            CHECK(op.collisions.empty());
        }
    }
    SECTION("a rapid across the part is reported") {
        SimulationOptions options;
        toolpath::Toolpath across = f.generate(2);
        toolpath::Move rapid;
        rapid.kind = toolpath::MoveKind::Rapid;
        rapid.feedClass = toolpath::FeedClass::Rapid;
        rapid.end = Vec3(-15.0, 0.0, -10.0);
        across.moves.push_back(rapid);
        rapid.end = Vec3(15.0, 0.0, -10.0);
        across.moves.push_back(rapid);
        const Uuid id = f.setup().operations[2].id;
        options.toolpathFor = [&](const Operation& op) -> std::optional<toolpath::Toolpath> {
            return op.id == id ? std::optional(across) : std::nullopt;
        };
        const SetupSimulation sim =
            simulateSetup(f.doc, f.setup().id, options, f.cancel, f.progress);
        const auto& hits = sim.operations[2].collisions;
        REQUIRE_FALSE(hits.empty());
        CHECK(std::any_of(hits.begin(), hits.end(), [](const sim::Collision& c) {
            return c.kind == sim::CollisionKind::Rapid;
        }));
    }
}

TEST_CASE("a live tool's holes in the stock and against the part", "[document][lathe][live]") {
    using namespace fixtures::livetool;
    LiveFixture f;
    const SetupSimulation sim = simulateSetup(f.doc, f.setup().id, {}, f.cancel, f.progress);
    REQUIRE(sim.operations.size() == 5);
    for (const auto& op : sim.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
    }
    // A milling setup going on from it gets the holes too.
    CHECK(sim.lastStock(true).volume() ==
          Catch::Approx(sim.operations.back().latheAfter->volume()).epsilon(0.01));
    const auto& last = *sim.operations.back().latheAfter;
    CHECK(last.holes().size() == 6);
    CHECK(sim.operations[2].removedVolume ==
          Catch::Approx(4.0 * kPi * 9.0 * kBoltDepth).epsilon(0.1));
    CHECK(sim.operations[3].removedVolume == Catch::Approx(kPi * 16.0 * kDiameter).epsilon(0.05));
    // Where the part has its holes they read as no gouge, and nothing is left to take.
    REQUIRE(sim.comparison);
    CHECK_FALSE(sim.comparison->hasGouges());
    const auto beforeHoles = latheComparison(sim, *sim.operations[1].latheAfter);
    REQUIRE(beforeHoles);
    CHECK(beforeHoles->excessVolume > 0.9 * (4.0 * kPi * 9.0 * kBoltDepth));
    // A hole through material the part has is a gouge, in the sum and in red.
    sim::LatheStock wrong = last;
    wrong.cutHole({Vec3(-5.0, 0.0, -50.0), Vec3::UnitZ(), 40.0, {{0.0, 3.0}, {100.0, 3.0}}});
    const auto gouged = latheComparison(sim, wrong);
    REQUIRE(gouged);
    CHECK(gouged->hasGouges());
    CHECK(gouged->gougeVolume > 0.5 * kPi * 9.0 * 30.0);
    const sim::DisplayMesh shown = latheDisplayMesh(sim, 4, wrong, true, sim::kStockColour);
    std::size_t red = 0;
    for (const auto& c : shown.colours) {
        red += c[0] > 120 && c[1] < 90 && c[2] < 90 ? 1 : 0;
    }
    CHECK(red > 100);
    // Drawn, it holds the volume.
    const sim::DisplayMesh right = latheDisplayMesh(sim, 4, last, true, sim::kStockColour);
    double volume = 0.0;
    for (std::size_t t = 0; t + 2 < right.indices.size(); t += 3) {
        const auto& a = right.positions[right.indices[t]];
        const auto& b = right.positions[right.indices[t + 1]];
        const auto& c = right.positions[right.indices[t + 2]];
        volume += (static_cast<double>(a[0]) * (b[1] * c[2] - b[2] * c[1]) +
                   static_cast<double>(a[1]) * (b[2] * c[0] - b[0] * c[2]) +
                   static_cast<double>(a[2]) * (b[0] * c[1] - b[1] * c[0])) /
                  6.0;
    }
    CHECK(volume == Catch::Approx(last.volume()).epsilon(0.02));
    // Part way through, part of the bolt circle.
    const auto partial = simulatePartialLathe(sim, 2, sim.operations[2].toolpath.moves.size() / 2);
    REQUIRE(partial);
    CHECK(partial->holes().size() < 4);
    CHECK_FALSE(partial->holes().empty());
    // The cache keeps it, frames comparing included.
    const auto back = decodeSimulation(encodeSimulation(sim, "key"), "key", f.doc, f.setup());
    REQUIRE(back);
    CHECK(back->operations[2].holeProfile == sim.operations[2].holeProfile);
    CHECK(back->operations.back().latheAfter->holes() == last.holes());
    REQUIRE(latheComparison(*back, wrong));
    CHECK(latheComparison(*back, wrong)->hasGouges());
}
