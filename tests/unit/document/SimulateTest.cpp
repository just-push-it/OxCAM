// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "FixtureSpecs.hpp"
#include "TestProject.hpp"

#include <ocam/cad/Features.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/document/SimulationCache.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/sim/Compare.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <numbers>

using namespace ocam;
using namespace ocam::document;
using Catch::Approx;

namespace {

ShapeRef faceAtHeight(const Document& doc, const Uuid& modelId, double height) {
    const cad::Model& model = *doc.model(modelId);
    ShapeRef ref;
    for (const auto& f : cad::findPlanarFaces(model)) {
        if (std::abs(f.height - height) < 1e-6) {
            ref = {modelId, cad::SubShapeType::Face, f.face,
                   cad::signatureOf(model, cad::SubShapeType::Face, f.face)};
        }
    }
    REQUIRE(ref.index > 0);
    return ref;
}

/// Volume enclosed by a closed display mesh (divergence theorem).
double enclosedVolume(const sim::DisplayMesh& m) {
    double v = 0.0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        auto at = [&](std::uint32_t i) {
            return Vec3(static_cast<double>(m.positions[i][0]),
                        static_cast<double>(m.positions[i][1]),
                        static_cast<double>(m.positions[i][2]));
        };
        v += at(m.indices[t]).dot(at(m.indices[t + 1]).cross(at(m.indices[t + 2])));
    }
    return v / 6.0;
}

} // namespace

TEST_CASE("automatic simulation resolution", "[document][simulate]") {
    geom3d::Aabb3 b;
    b.extend(Vec3(0, 0, 0));
    b.extend(Vec3(102, 62, 31));
    CHECK(automaticResolution(b) == Approx(0.1));
    b.extend(Vec3(1000, 0, 0));
    CHECK(automaticResolution(b) == Approx(1.0));
    geom3d::Aabb3 tiny;
    tiny.extend(Vec3(0, 0, 0));
    tiny.extend(Vec3(2, 2, 2));
    CHECK(automaticResolution(tiny) == Approx(0.05));
}

TEST_CASE("simulate the block fixture: face, pocket and contour", "[document][simulate]") {
    using namespace fixtures::block_pocket;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid modelId = doc.importModel(
        std::filesystem::path(OCAM_FIXTURE_DIR) / "block_pocket.step", cancel, progress);
    const Uuid setupId = doc.addSetup(modelId);
    const tooling::Tool tool = testing::withPreset(tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    doc.modify("setup", [&](Project& p) {
        p.tools.tools.push_back(tool);
        auto add = [&](const std::string& type, std::vector<ShapeRef> geometry, Json params,
                       PresetOverrides po = {}) {
            Operation op;
            op.name = type;
            op.type = type;
            op.toolId = tool.id;
            op.presetId = tool.presets.front().id;
            op.geometry = std::move(geometry);
            op.parameters = std::move(params);
            op.presetOverrides = po;
            p.findSetup(setupId)->operations.push_back(op);
        };
        add("facing", {}, Json::object());
        add("pocket2d", {faceAtHeight(doc, modelId, kHeight - kPocketDepth)}, Json::object(),
            testing::stepDown(5.0));
        add("contour2d", {faceAtHeight(doc, modelId, 0.0)}, Json::object(),
            testing::stepDown(10.0));
    });

    SimulationOptions options;
    options.resolution = 0.25;
    const SetupSimulation sim = simulateSetup(doc, setupId, options, cancel, progress);
    REQUIRE(sim.operations.size() == 3);
    for (const auto& op : sim.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
        CHECK(op.collisions.empty());
        // The machine's positions along the toolpath, for showing it.
        CHECK(op.rotary.angles.size() == op.toolpath.moves.size());
        CHECK(op.machineCollisions.empty());
    }
    REQUIRE(sim.machine);
    CHECK(sim.machine->id == "generic-3axis-vmc");
    const double stockVolume = (kLength + 2) * (kWidth + 2) * (kHeight + 1);
    CHECK(sim.initialStock.volume() == Approx(stockVolume).epsilon(1e-3));
    // Facing removes the 1 mm on top; the pocket removes its volume.
    CHECK(sim.operations[0].removedVolume == Approx((kLength + 2) * (kWidth + 2)).epsilon(0.01));
    const double pocketVolume = kPocketArea * kPocketDepth;
    CHECK(sim.operations[1].removedVolume == Approx(pocketVolume).epsilon(0.01));
    // The final stock is the part.
    const double partVolume = sim.part.signedVolume();
    CHECK(sim.finalStock().volume() == Approx(partVolume).epsilon(0.005));
    REQUIRE(sim.comparison);
    std::cout << "block fixture: excess rays " << sim.comparison->excessRays << ", max excess "
              << sim.comparison->maxExcess << ", excess volume " << sim.comparison->excessVolume
              << ", gouge rays " << sim.comparison->gougeRays << "\n";
    CHECK_FALSE(sim.comparison->hasGouges());
    CHECK(sim.comparison->excessVolume < 0.001 * stockVolume);

    // Playback: half of the pocket lies between the checkpoints.
    const auto& pocket = sim.operations[1];
    const sim::Stock half = simulatePartial(sim, 1, pocket.toolpath.moves.size() / 2);
    CHECK(half.volume() < sim.stockBefore(1).volume() - 1.0);
    CHECK(half.volume() > pocket.stockAfter.volume() + 1.0);
    CHECK(simulatePartial(sim, 1, pocket.toolpath.moves.size()).volume() ==
          Approx(pocket.stockAfter.volume()));
    // Checkpoints share their unchanged tiles with each other, not copies.
    CHECK(sim.stockBefore(0).volume() == Approx(stockVolume).epsilon(1e-3));

    // The display mesh of the final stock shows the pocket floor.
    const double floorZ = -kPocketDepth - 1.0; // WCS: origin on the stock top, 1 mm above the part
    // The display (DL-0040) puts the pocket floor where it was cut, and encloses the stock's
    // volume.
    const sim::DisplayMesh smooth = sim::buildDisplayMesh(sim.finalStock(), &*sim.comparison);
    CHECK(std::any_of(smooth.positions.begin(), smooth.positions.end(),
                      [&](const auto& v) { return std::abs(v[2] - floorZ) < 1e-3; }));
    CHECK(enclosedVolume(smooth) == Approx(sim.finalStock().volume()).epsilon(0.005));
}

TEST_CASE("contour tabs stand in the simulated stock", "[document][simulate][tabs]") {
    using namespace fixtures::block_pocket;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid modelId = doc.importModel(
        std::filesystem::path(OCAM_FIXTURE_DIR) / "block_pocket.step", cancel, progress);
    const Uuid setupId = doc.addSetup(modelId);
    const tooling::Tool tool = testing::withPreset(tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    // The contour around the block, down to its bottom, with and without four 5 x 3 mm tabs.
    doc.modify("setup", [&](Project& p) {
        p.tools.tools.push_back(tool);
        for (const bool tabs : {false, true}) {
            Operation op;
            op.name = tabs ? "With tabs" : "Without tabs";
            op.type = "contour2d";
            op.toolId = tool.id;
            op.presetId = tool.presets.front().id;
            op.geometry = {faceAtHeight(doc, modelId, 0.0)};
            op.parameters = Json{{"tabs", tabs}, {"tab_width", 5.0}, {"tab_height", 3.0}};
            op.presetOverrides = testing::stepDown(10.0);
            p.findSetup(setupId)->operations.push_back(op);
        }
    });
    SimulationOptions options;
    options.resolution = 0.25;
    const SetupSimulation sim = simulateSetup(doc, setupId, options, cancel, progress);
    REQUIRE(sim.operations.size() == 2);
    for (const auto& op : sim.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
        CHECK(op.collisions.empty());
    }
    // The second contour cuts what the first left: the tabs it keeps were cut by the first, so
    // simulate the tabbed one alone as well.
    const double without = sim.operations[0].removedVolume;
    CHECK(sim.operations[1].removedVolume < 0.01 * without);

    doc.modify("drop the untabbed contour", [&](Project& p) {
        auto& ops = p.findSetup(setupId)->operations;
        ops.erase(ops.begin());
    });
    const SetupSimulation tabbed = simulateSetup(doc, setupId, options, cancel, progress);
    REQUIRE(tabbed.operations.size() == 1);
    CHECK(tabbed.operations[0].error.empty());
    // The stock is 1 mm wider than the block on every side; the tabs stand 3 mm high in that
    // band, each at least 5 mm long (more by the wall, where the tool's edge curves away), so the
    // four leave between 4 x 5 x 1 x 3 = 60 and 4 x 11 x 1 x 3 = 132 mm^3 standing.
    const double left = without - tabbed.operations[0].removedVolume;
    INFO("tabs left " << left << " mm^3");
    CHECK(left > 60.0 * 0.8);
    CHECK(left < 132.0 * 1.2);
    // Where they are, the simulated stock is above the part: excess, not gouges.
    REQUIRE(tabbed.comparison);
    CHECK_FALSE(tabbed.comparison->hasGouges());
    REQUIRE(sim.comparison);
    CHECK(tabbed.comparison->excessVolume > sim.comparison->excessVolume + 0.8 * 60.0);
}

TEST_CASE("simulation reports operations that cannot be generated", "[document][simulate]") {
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid modelId = doc.importModel(
        std::filesystem::path(OCAM_FIXTURE_DIR) / "block_pocket.step", cancel, progress);
    const Uuid setupId = doc.addSetup(modelId);
    doc.modify("op", [&](Project& p) {
        Operation op;
        op.name = "No tool";
        op.type = "facing";
        p.findSetup(setupId)->operations.push_back(op);
    });
    SimulationOptions options;
    options.resolution = 1.0;
    const SetupSimulation sim = simulateSetup(doc, setupId, options, cancel, progress);
    REQUIRE(sim.operations.size() == 1);
    CHECK_FALSE(sim.operations[0].error.empty());
    CHECK(sim.hasErrors());
    CHECK(sim.finalStock().volume() == Approx(sim.initialStock.volume()));
    CHECK_THROWS_AS(simulateSetup(doc, Uuid::generate(), options, cancel, progress), Error);
}

TEST_CASE("rest pocketing clears what a larger tool left", "[document][simulate][rest]") {
    using namespace fixtures::block_pocket;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid modelId = doc.importModel(
        std::filesystem::path(OCAM_FIXTURE_DIR) / "block_pocket.step", cancel, progress);
    const Uuid setupId = doc.addSetup(modelId);
    const tooling::Tool big = testing::withPreset(tooling::makeFlatEndMill(1, 16.0, 25.0, 60.0));
    const tooling::Tool small = testing::withPreset(tooling::makeFlatEndMill(2, 6.0, 15.0, 50.0));
    const tooling::Tool face = testing::withPreset(tooling::makeFlatEndMill(3, 10.0, 25.0, 60.0));
    Uuid restId;
    doc.modify("setup", [&](Project& p) {
        p.tools.tools = {big, small, face};
        auto add = [&](const std::string& name, const std::string& type, const tooling::Tool& tool,
                       std::vector<ShapeRef> geometry, Json params, PresetOverrides po = {}) {
            Operation op;
            op.name = name;
            op.type = type;
            op.toolId = tool.id;
            op.presetId = tool.presets.front().id;
            op.geometry = std::move(geometry);
            op.parameters = std::move(params);
            op.presetOverrides = po;
            p.findSetup(setupId)->operations.push_back(op);
            return op.id;
        };
        const ShapeRef floor = faceAtHeight(doc, modelId, kHeight - kPocketDepth);
        add("Face", "facing", face, {}, Json::object());
        add("Rough", "pocket2d", big, {floor}, Json::object(), testing::stepDown(5.0));
        restId =
            add("Rest", "pocket2d", small, {floor},
                Json{{"rest_machining", true}, {"rest_resolution", 0.2}}, testing::stepDown(5.0));
        add("Profile", "contour2d", face, {faceAtHeight(doc, modelId, 0.0)}, Json::object(),
            testing::stepDown(10.0));
    });
    const Setup& setup = *doc.project().findSetup(setupId);
    const Operation& restOp = *doc.project().findOperation(restId);

    // The rest pass only visits the corners: far shorter than a full pocket with the same tool.
    const auto restPath = generateToolpath(doc, setup, restOp, cancel, progress);
    Operation fullOp = restOp;
    fullOp.parameters["rest_machining"] = false;
    const auto fullPath = generateToolpath(doc, setup, fullOp, cancel, progress);
    const auto restStats = toolpath::computeStats(restPath);
    const auto fullStats = toolpath::computeStats(fullPath);
    INFO("rest " << restStats.cutLength << " mm, full " << fullStats.cutLength << " mm");
    CHECK(restStats.cutLength > 0.0);
    CHECK(restStats.cutLength < 0.4 * fullStats.cutLength);
    // The key follows earlier operations.
    const std::string key = toolpathInputKey(doc, setup, restOp);
    doc.modify("rough stepover", [&](Project& p) {
        p.findSetup(setupId)->operations[1].presetOverrides.radialDepth = 8.0;
    });
    CHECK(toolpathInputKey(doc, *doc.project().findSetup(setupId),
                           *doc.project().findOperation(restId)) != key);

    // Without the rest pass the corners stay; with it the part is clean.
    SimulationOptions options;
    options.resolution = 0.2;
    const SetupSimulation withRest = simulateSetup(doc, setupId, options, cancel, progress);
    for (const auto& op : withRest.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
        CHECK(op.collisions.empty());
    }
    REQUIRE(withRest.comparison);
    CHECK_FALSE(withRest.comparison->hasGouges());
    CHECK(withRest.comparison->maxExcess < 0.05);

    doc.modify("disable rest", [&](Project& p) { p.findOperation(restId)->enabled = false; });
    const SetupSimulation without = simulateSetup(doc, setupId, options, cancel, progress);
    REQUIRE(without.comparison);
    CHECK(without.comparison->excessVolume > 10.0);
    CHECK(without.comparison->maxExcess > 1.0);
}

TEST_CASE("3D roughing, rest roughing and finishing of the dome", "[document][simulate][3d]") {
    using namespace fixtures::dome;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid modelId =
        doc.importModel(std::filesystem::path(OCAM_FIXTURE_DIR) / "dome.step", cancel, progress);
    const Uuid setupId = doc.addSetup(modelId);
    const tooling::Tool rougher =
        testing::withPreset(tooling::makeFlatEndMill(1, 12.0, 40.0, 80.0));
    const tooling::Tool rester = testing::withPreset(tooling::makeFlatEndMill(2, 6.0, 40.0, 80.0));
    const tooling::Tool ball = testing::withPreset(tooling::makeBallEndMill(3, 6.0, 40.0, 80.0));
    Uuid restId;
    doc.modify("ops", [&](Project& p) {
        p.tools.tools = {rougher, rester, ball};
        auto add = [&](const std::string& name, const std::string& type, const tooling::Tool& tool,
                       Json params, PresetOverrides po = {}) {
            Operation op;
            op.name = name;
            op.type = type;
            op.toolId = tool.id;
            op.presetId = tool.presets.front().id;
            op.parameters = std::move(params);
            op.presetOverrides = po;
            p.findSetup(setupId)->operations.push_back(op);
            return op.id;
        };
        // No fine levels on the rough: they would take the dome's stairs at the rest pass's own
        // spacing and leave it nothing (a convex dome has no other place the large tool misses).
        add("Rough", "zlevel_rough", rougher,
            Json{{"stock_to_leave_xy", 0.5}, {"stock_to_leave_z", 0.5}, {"fine_step_down", 0.0}},
            testing::stepDown(3.0));
        restId = add("Rest rough", "zlevel_rough", rester,
                     Json{{"stock_to_leave_xy", 0.5},
                          {"stock_to_leave_z", 0.5},
                          {"rest_machining", true},
                          {"rest_resolution", 0.5}},
                     testing::stepDown(1.0));
        add("Finish", "parallel3d", ball, Json{{"scallop_height", 0.02}});
    });
    SimulationOptions options;
    options.resolution = 0.25;
    const SetupSimulation sim = simulateSetup(doc, setupId, options, cancel, progress);
    REQUIRE(sim.operations.size() == 3);
    for (const auto& op : sim.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
        CHECK(op.collisions.empty());
        CHECK(op.removedVolume > 0.0);
    }
    REQUIRE(sim.comparison);
    std::cout << "dome: rough " << sim.operations[0].removedVolume << ", rest "
              << sim.operations[1].removedVolume << ", finish " << sim.operations[2].removedVolume
              << " mm3; excess " << sim.comparison->excessVolume << " mm3 (max "
              << sim.comparison->maxExcess << "), gouge " << sim.comparison->gougeVolume << "\n";
    CHECK_FALSE(sim.comparison->hasGouges());
    // The rest pass only removes what the large tool left: much less than the roughing.
    CHECK(sim.operations[1].removedVolume < 0.25 * sim.operations[0].removedVolume);
    // The ball finish brings the dome to size; stock remains only on the base's sides, which the
    // parallel pass does not reach.
    CHECK(sim.finalStock().volume() < kVolume + 0.06 * (82.0 * 82.0 * 31.0 - kVolume));
}

TEST_CASE("turning the shaft fixture on a lathe", "[document][simulate][lathe]") {
    using namespace fixtures::shaft;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid modelId =
        doc.importModel(std::filesystem::path(OCAM_FIXTURE_DIR) / "shaft.step", cancel, progress);
    const Uuid setupId = doc.addSetup(modelId);
    tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
    turn.insert.noseRadius = 0.4;
    tooling::Tool part = tooling::makeDefaultTool(tooling::ToolType::PartingInsert, 2);
    part.insert.width = 3.0;
    part.insert.noseRadius = 0.1;
    doc.modify("lathe setup", [&](Project& p) {
        Setup* s = p.findSetup(setupId);
        s->machine = "generic-2axis-lathe";
        s->originFrom.reset();                                   // the origin as set
        s->wcs.origin = Vec3(0, 0, kMajorLength + kMinorLength); // front face centre
        s->wcs.zAxis = Vec3::UnitZ();
        s->wcs.xAxis = Vec3::UnitX();
        s->stock.kind = StockKind::Box;
        s->stock.size = StockSize::OffsetFromPart;
        s->stock.offsetSide = 1.0;
        s->stock.offsetTop = 1.0;
        s->stock.offsetBottom = 5.0;
        p.tools.tools = {turn, part};
        auto add = [&](const char* name, const char* type, const tooling::Tool& tool, Json params,
                       PresetOverrides po = {}) {
            Operation op;
            op.name = name;
            op.type = type;
            op.toolId = tool.id;
            op.presetId = tool.presets.front().id;
            op.parameters = std::move(params);
            op.presetOverrides = po;
            s->operations.push_back(op);
        };
        add("Face", "turn_face", turn, Json::object());
        add("Rough", "turn_rough", turn, Json::object(), testing::stepDown(1.5));
        add("Finish", "turn_finish", turn, Json::object());
        add("Part off", "turn_partoff", part, Json::object());
    });
    SimulationOptions options;
    options.resolution = 0.2;
    const SetupSimulation sim = simulateSetup(doc, setupId, options, cancel, progress);
    REQUIRE(sim.latheInitial);
    REQUIRE(sim.operations.size() == 4);
    for (const auto& op : sim.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
        CHECK(op.removedVolume > 0.0);
    }
    const double partVolume = kVolume;
    // After parting off, the stock is the part plus the piece cut off behind it; the part-side
    // piece matches the part exactly but for the shoulder fillet.
    REQUIRE(sim.comparison);
    CHECK_FALSE(sim.comparison->hasGouges());
    CHECK(sim.comparison->excessVolume > 0.0);
    std::cout << "shaft: final " << sim.operations.back().latheAfter->volume() << " part "
              << partVolume << " excess " << sim.comparison->excessVolume << "\n";
    for (std::size_t i = 0; i < sim.operations.size(); ++i) {
        CHECK(sim.latheBefore(i + 1) == sim.operations[i].latheAfter);
    }
    CHECK(sim.latheBefore(0) == sim.latheInitial);
    // Only the profile: no dexels at all.
    CHECK(sim.initialStock.grid(2).isEmpty());
    CHECK(sim.finalStock().grid(2).isEmpty());
    CHECK_FALSE(sim.partReference);
    // A milling setup going on from it gets the same stock as dexels.
    const sim::Stock asDexels = sim.lastStock(true);
    CHECK(asDexels.volume() == Approx(sim.operations.back().latheAfter->volume()).epsilon(0.01));
    // Playback replays on the axisymmetric stock.
    const auto halfLathe =
        simulatePartialLathe(sim, 1, sim.operations[1].toolpath.moves.size() / 2);
    REQUIRE(halfLathe);
    CHECK(halfLathe->volume() < sim.latheBefore(1)->volume());
    CHECK(halfLathe->volume() > sim.operations[1].latheAfter->volume());

    // The turned stock drawn as its revolved profile (DL-0040): closed around the stock's
    // volume and, after the finish, at size on the part's diameters and faces (green), with no
    // gouge (red) anywhere.
    REQUIRE_FALSE(sim.partProfile.isEmpty());
    const std::size_t finish = 2;
    const sim::DisplayMesh turned =
        latheDisplayMesh(sim, finish, *sim.operations[finish].latheAfter, true, sim::kStockColour);
    REQUIRE(turned.triangleCount() > 0);
    CHECK(enclosedVolume(turned) ==
          Approx(sim.operations[finish].latheAfter->volume()).epsilon(1e-3));
    const sim::Rgb atSize = sim::comparisonColour(0.0, 0.0, 0.02, 2.0);
    std::size_t green = 0, red = 0;
    for (const auto& c : turned.colours) {
        green += c == atSize ? 1 : 0;
        red += c[0] > 200 && c[1] < 100 && c[2] < 100 ? 1 : 0;
    }
    CHECK(red == 0);
    CHECK(green > turned.colours.size() / 2);
    // Milling setups have no lathe stock.
    CHECK(latheDisplayMesh(SetupSimulation{}, 0, *sim.latheInitial, true, sim::kStockColour)
              .indices.empty());

    // A frame cut into the part shows as a gouge, as deep as it goes.
    sim::LatheStock gouged = *sim.operations[2].latheAfter;
    gouged.subtract({{geom2d::Polyline::rectangle(Vec2(-10.0, 9.0), Vec2(-5.0, 20.0)), {}}});
    const auto cut = latheComparison(sim, gouged);
    REQUIRE(cut);
    CHECK(cut->hasGouges());
    CHECK(cut->maxGouge == Approx(1.0).margin(0.05));
    CHECK(cut->gougeVolume > 0.0);
    // It survives the cache, frames comparing included.
    const auto reread = decodeSimulation(encodeSimulation(sim, "key"), "key", doc,
                                         *doc.project().findSetup(setupId));
    REQUIRE(reread);
    CHECK(reread->partRegions.size() == sim.partRegions.size());
    REQUIRE(latheComparison(*reread, gouged));
    CHECK(latheComparison(*reread, gouged)->hasGouges());

    // Posting picks the lathe post and verifies by back-plotting.
    const Setup& setup = *doc.project().findSetup(setupId);
    const machine::Machine& machine = machineFor(doc.project(), setup);
    CHECK(machine.isLathe());
    const auto out = postSetup(doc, setupId, machine, defaultPostFor(machine), cancel, progress);
    INFO((out.errors.empty() ? std::string() : out.errors.front()));
    REQUIRE(out.ok());
    CHECK_THAT(out.gcode, Catch::Matchers::ContainsSubstring("G96"));
    CHECK_THAT(out.gcode, Catch::Matchers::ContainsSubstring("T0202"));
    CHECK(post::compareBackplot(out.toolpaths, post::backplot(out.gcode, {true}).path, 2e-3,
                                nullptr, false)
              .value_or("") == "");
}

TEST_CASE("turning in an ISO holder", "[document][simulate][lathe]") {
    using namespace fixtures::shaft;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid modelId =
        doc.importModel(std::filesystem::path(OCAM_FIXTURE_DIR) / "shaft.step", cancel, progress);
    const Uuid setupId = doc.addSetup(modelId);
    tooling::Holder holder =
        tooling::makeTurningHolder(*tooling::parseDesignation("PCLNR 2525M 12"));
    tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
    turn.insert.noseRadius = 0.4;
    turn.holderId = holder.id;
    tooling::fitToHolder(turn, holder);
    doc.modify("lathe setup", [&](Project& p) {
        Setup* s = p.findSetup(setupId);
        s->machine = "generic-2axis-lathe";
        s->originFrom.reset();
        s->wcs.origin = Vec3(0, 0, kMajorLength + kMinorLength);
        s->wcs.zAxis = Vec3::UnitZ();
        s->wcs.xAxis = Vec3::UnitX();
        s->stock.kind = StockKind::Box;
        s->stock.size = StockSize::OffsetFromPart;
        s->stock.offsetSide = 1.0;
        s->stock.offsetTop = 1.0;
        s->stock.offsetBottom = 5.0;
        p.tools.tools = {turn};
        p.tools.holders = {holder};
        for (const char* type : {"turn_face", "turn_rough", "turn_finish"}) {
            Operation op;
            op.name = type;
            op.type = type;
            op.toolId = turn.id;
            op.presetId = turn.presets.front().id;
            op.parameters = Json::object();
            op.presetOverrides = testing::stepDown(1.5);
            s->operations.push_back(op);
        }
    });
    SimulationOptions options;
    const auto errors = [&] {
        std::vector<std::string> found;
        for (const auto& op : simulateSetup(doc, setupId, options, cancel, progress).operations) {
            found.push_back(op.error);
        }
        return found;
    };
    SECTION("a shank holder cuts outside") {
        const SetupSimulation sim = simulateSetup(doc, setupId, options, cancel, progress);
        for (const auto& op : sim.operations) {
            INFO(op.name << ": " << op.error);
            CHECK(op.error.empty());
        }
        REQUIRE(sim.comparison);
        CHECK_FALSE(sim.comparison->hasGouges());
        // Playback shows the holder, and a saved simulation keeps it.
        for (const auto& op : sim.operations) {
            REQUIRE(op.toolParts.size() == 2);
            CHECK_FALSE(op.toolParts[1].round);
            CHECK(op.toolParts[1].low == Approx(-25.0));
        }
        const auto back = decodeSimulation(encodeSimulation(sim, "key"), "key", doc,
                                           *doc.project().findSetup(setupId));
        REQUIRE(back);
        CHECK(back->operations[0].toolParts == sim.operations[0].toolParts);
        // The designation stays with the project.
        const auto path = std::filesystem::temp_directory_path() / "oxcam_iso_holder_test.ocam";
        doc.save(path);
        const Document reopened = Document::open(path);
        std::filesystem::remove(path);
        REQUIRE(reopened.project().tools.holders.size() == 1);
        REQUIRE(reopened.project().tools.holders[0].iso);
        CHECK(*reopened.project().tools.holders[0].iso == *holder.iso);
        CHECK(reopened.project().tools.tools[0].insert.cuttingEdgeAngle ==
              Approx(turn.insert.cuttingEdgeAngle));
    }
    SECTION("a boring bar does not") {
        doc.modify("boring bar", [&](Project& p) {
            tooling::setTurningHolderCode(p.tools.holders[0],
                                          *tooling::parseDesignation("S25T-PCLNR 12"));
        });
        for (const std::string& error : errors()) {
            CHECK_THAT(error, Catch::Matchers::ContainsSubstring("boring bar"));
        }
    }
    SECTION("nor does a left-hand holder") {
        doc.modify("left hand", [&](Project& p) {
            tooling::setTurningHolderCode(p.tools.holders[0],
                                          *tooling::parseDesignation("PCLNL 2525M 12"));
        });
        for (const std::string& error : errors()) {
            CHECK_THAT(error, Catch::Matchers::ContainsSubstring("left-hand"));
        }
    }
}

TEST_CASE("tube stock is hollow, in milling and on a lathe", "[document][simulate][stock]") {
    CancelToken cancel;
    NullProgress progress;
    SECTION("milling: a tube along WCS Z around the block") {
        using namespace fixtures::block_pocket;
        Document doc;
        const Uuid modelId = doc.importModel(
            std::filesystem::path(OCAM_FIXTURE_DIR) / "block_pocket.step", cancel, progress);
        const Uuid setupId = doc.addSetup(modelId);
        constexpr double kOuter = 130, kInner = 40, kTubeLength = 40;
        doc.modify("tube", [&](Project& p) {
            Stock& st = p.findSetup(setupId)->stock;
            st.kind = StockKind::Tube;
            st.size = StockSize::Fixed;
            st.axis = 2;
            st.diameter = kOuter;
            st.innerDiameter = kInner;
            st.length = kTubeLength;
        });
        const Setup& setup = *doc.project().findSetup(setupId);
        const std::optional<geom3d::Mesh> mesh = stockMeshInWcs(doc, setup);
        REQUIRE(mesh.has_value());
        CHECK(mesh->isClosed());
        const double volume =
            std::numbers::pi / 4.0 * (kOuter * kOuter - kInner * kInner) * kTubeLength;
        CHECK(mesh->signedVolume() == Approx(volume).epsilon(1e-3));
        SimulationOptions options;
        options.resolution = 0.5;
        options.compare = false;
        const SetupSimulation sim = simulateSetup(doc, setupId, options, cancel, progress);
        CHECK(sim.initialStock.volume() == Approx(volume).epsilon(0.01));
        // No material on the axis (the block's centre), material in the wall.
        const Vec3 centre = modelToWcs(*doc.project().findModel(modelId), setup) *
                            Vec3(kLength / 2, kWidth / 2, kHeight / 2);
        const sim::DexelGrid& g = sim.initialStock.grid(2);
        auto rayAt = [&](double x, double y) -> const sim::IntervalList& {
            return g.ray(static_cast<int>(std::floor((x - g.u0()) / g.spacing())),
                         static_cast<int>(std::floor((y - g.v0()) / g.spacing())));
        };
        CHECK(rayAt(centre.x(), centre.y()).empty());
        const sim::IntervalList& wall = rayAt(centre.x() + (kOuter + kInner) / 4, centre.y());
        REQUIRE(wall.size() == 1);
        CHECK(wall.front().hi - wall.front().lo == Approx(kTubeLength).margin(0.01));
        // The smooth display (DL-0040) draws both round walls round: every vertex between the
        // end faces lies on the bore or on the outside diameter.
        const sim::DisplayMesh smooth = sim::buildDisplayMesh(sim.initialStock);
        CHECK(enclosedVolume(smooth) == Approx(volume).epsilon(0.01));
        const double zLo = sim.initialStock.bounds().min.z() + options.resolution;
        const double zHi = sim.initialStock.bounds().max.z() - options.resolution;
        double worst = 0.0;
        std::size_t onBore = 0;
        for (const auto& v : smooth.positions) {
            if (v[2] < zLo + 1e-3 || v[2] > zHi - 1e-3) {
                continue; // on an end face
            }
            const double r = std::hypot(v[0] - centre.x(), v[1] - centre.y());
            const double d = std::min(std::abs(r - kInner / 2), std::abs(r - kOuter / 2));
            worst = std::max(worst, d);
            onBore += std::abs(r - kInner / 2) < 0.1 ? 1 : 0;
        }
        CHECK(onBore > 0);
        CHECK(worst < 0.2 * options.resolution);
    }
    SECTION("turning: the shaft in a tube, no additional stock") {
        using namespace fixtures::shaft;
        Document doc;
        const Uuid modelId = doc.importModel(std::filesystem::path(OCAM_FIXTURE_DIR) / "shaft.step",
                                             cancel, progress);
        const Uuid setupId = doc.addSetup(modelId);
        doc.modify("lathe tube", [&](Project& p) {
            Setup* s = p.findSetup(setupId);
            s->machine = "generic-2axis-lathe";
            s->originFrom.reset();
            s->wcs = Frame{};
            s->stock.kind = StockKind::Tube;
            s->stock.size = StockSize::Part;
            s->stock.innerDiameter = 8.0;
        });
        // Sized on the part's mesh: its largest diameter, its length, on the spindle axis.
        const auto shape = doc.stockShape(setupId);
        REQUIRE(shape.has_value());
        CHECK(shape->diameter == Approx(kMajorDiameter).margin(1e-6));
        CHECK(shape->length == Approx(kMajorLength + kMinorLength).margin(1e-6));
        CHECK(shape->innerDiameter == 8.0);
        const LatheGeometry g = latheGeometry(doc, *doc.project().findSetup(setupId), cancel);
        REQUIRE(g.stock.size() == 1);
        const geom2d::Aabb2 b = g.stock.front().bounds();
        CHECK(b.min.y() == Approx(4.0));
        CHECK(b.max.y() == Approx(kMajorDiameter / 2).margin(1e-6));
        SimulationOptions options;
        options.resolution = 0.25;
        options.compare = false;
        const SetupSimulation sim = simulateSetup(doc, setupId, options, cancel, progress);
        REQUIRE(sim.latheInitial);
        const double volume = std::numbers::pi / 4.0 * (kMajorDiameter * kMajorDiameter - 64.0) *
                              (kMajorLength + kMinorLength);
        CHECK(sim.latheInitial->volume() == Approx(volume).epsilon(0.02));
        // Drawn as its revolved profile, bore included (DL-0040).
        const sim::DisplayMesh turned =
            latheDisplayMesh(sim, 0, *sim.latheInitial, false, sim::kStockColour);
        CHECK(enclosedVolume(turned) == Approx(volume).epsilon(1e-3));
        CHECK(std::any_of(turned.positions.begin(), turned.positions.end(), [](const auto& v) {
            return std::abs(std::hypot(v[0], v[1]) - 4.0) < 1e-3;
        }));
    }
}

TEST_CASE("stock from an earlier setup carries what it cut", "[document][simulate][setups]") {
    using namespace fixtures::block_pocket;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid modelId = doc.importModel(
        std::filesystem::path(OCAM_FIXTURE_DIR) / "block_pocket.step", cancel, progress);
    const Uuid first = doc.addSetup(modelId);
    const Uuid second = doc.addSetup(modelId);
    const tooling::Tool tool = testing::withPreset(tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    Uuid facingId;
    doc.modify("setups", [&](Project& p) {
        p.tools.tools.push_back(tool);
        // The first setup pockets the top.
        Operation pocket;
        pocket.name = "Pocket";
        pocket.type = "pocket2d";
        pocket.toolId = tool.id;
        pocket.presetId = tool.presets.front().id;
        pocket.geometry = {faceAtHeight(doc, modelId, kHeight - kPocketDepth)};
        pocket.presetOverrides = testing::stepDown(5.0);
        p.findSetup(first)->operations.push_back(pocket);
        // The second holds the part upside down, on the first setup's stock.
        Setup& s = *p.findSetup(second);
        s.stock.kind = StockKind::FromSetup;
        s.stock.sourceSetup = first;
        s.wcs.origin = Vec3(-1.0, kWidth + 1.0, -1.0);
        s.wcs.xAxis = Vec3::UnitX();
        s.wcs.zAxis = -Vec3::UnitZ();
        Operation facing;
        facing.name = "Facing";
        facing.type = "facing";
        facing.toolId = tool.id;
        facing.presetId = tool.presets.front().id;
        facingId = facing.id;
        s.operations.push_back(facing);
    });
    SimulationOptions options;
    options.compare = false;
    const SetupSimulation one = simulateSetup(doc, first, options, cancel, progress);
    REQUIRE_FALSE(one.hasErrors());
    const double removedFirst = one.initialStock.volume() - one.finalStock().volume();
    REQUIRE(removedFirst > 0.5 * kPocketArea * kPocketDepth);

    // The second setup starts as the first left it: the pocket is in its stock (now facing
    // down), cut on its own grid from the first setup's toolpath turned over.
    const SetupSimulation two = simulateSetup(doc, second, options, cancel, progress);
    for (const auto& w : two.warnings) {
        INFO(w);
        CHECK(w.find("not carried") == std::string::npos);
    }
    const std::optional<geom3d::Mesh> root = stockMeshInWcs(doc, *doc.project().findSetup(second));
    REQUIRE(root.has_value());
    const sim::Stock uncut = sim::Stock::fromMesh(*root, root->bounds(), two.resolution);
    const double carried = uncut.volume() - two.initialStock.volume();
    CHECK(carried == Approx(removedFirst).epsilon(0.03));
    // Where it was cut: under the pocket's centre, turned over into this WCS, the material ends
    // at the pocket's floor and runs to the block's bottom (now on top).
    {
        const Transform3 toWcs =
            modelToWcs(*doc.project().findModel(modelId), *doc.project().findSetup(second));
        const double cx = kPocketX + 0.5 * kPocketLength;
        const double cy = kPocketY + 0.5 * kPocketWidth;
        const Vec3 floor = toWcs * Vec3(cx, cy, kHeight - kPocketDepth);
        const Vec3 bottom = toWcs * Vec3(cx, cy, 0.0);
        const sim::DexelGrid& g = two.initialStock.grid(2);
        const int i = static_cast<int>(std::floor((floor.x() - g.u0()) / g.spacing()));
        const int j = static_cast<int>(std::floor((floor.y() - g.v0()) / g.spacing()));
        const sim::IntervalList& ray = g.ray(i, j);
        REQUIRE(ray.size() == 1);
        CHECK(ray.front().lo == Approx(floor.z()).margin(0.2));
        CHECK(ray.front().hi == Approx(bottom.z()).margin(0.2));
    }

    // Rest machining in the second setup sees the same stock.
    const Setup& secondSetup = *doc.project().findSetup(second);
    const StockBefore before =
        stockBeforeOperation(doc, secondSetup, facingId, two.resolution, cancel, progress);
    CHECK(before.warnings.empty());
    CHECK(before.stock.volume() == Approx(two.initialStock.volume()).epsilon(0.01));

    // It starts from the first setup's simulation: one given (a cache's) is used, not simulated
    // again, and gives the same stock.
    int asked = 0;
    SimulationOptions given = options;
    const auto firstSimulation = std::make_shared<const SetupSimulation>(one);
    given.sourceSimulation = [&](const Setup& source, CancelToken&, ProgressSink&) {
        ++asked;
        CHECK(source.id == first);
        return firstSimulation;
    };
    const SetupSimulation fromGiven = simulateSetup(doc, second, given, cancel, progress);
    CHECK(asked == 1);
    CHECK(fromGiven.initialStock.volume() == two.initialStock.volume());

    // The second setup's toolpaths and simulation follow changes to the first.
    const std::string key = simulationInputKey(doc, secondSetup, options);
    doc.modify("deeper", [&](Project& p) {
        p.findSetup(first)->operations.front().presetOverrides.axialDepth = 2.0;
    });
    CHECK(simulationInputKey(doc, *doc.project().findSetup(second), options) != key);
}

TEST_CASE("chamfers are not gouges, in their setup and the next", "[document][simulate][compare]") {
    using namespace fixtures::block_pocket;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid modelId = doc.importModel(
        std::filesystem::path(OCAM_FIXTURE_DIR) / "block_pocket.step", cancel, progress);
    const Uuid first = doc.addSetup(modelId);
    const Uuid second = doc.addSetup(modelId);
    const tooling::Tool tool =
        testing::withPreset(tooling::makeChamferMill(1, 10.0, 0.0, degToRad(90.0), 50.0));
    doc.modify("setups", [&](Project& p) {
        p.tools.tools.push_back(tool);
        // A 1 mm chamfer round the top face's edges, which the model does not have.
        Operation chamfer;
        chamfer.name = "Chamfer";
        chamfer.type = "chamfer2d";
        chamfer.toolId = tool.id;
        chamfer.presetId = tool.presets.front().id;
        chamfer.geometry = {faceAtHeight(doc, modelId, kHeight)};
        chamfer.parameters = Json{{"width", 1.0}};
        p.findSetup(first)->operations.push_back(chamfer);
        // The second setup holds the part upside down, on the first setup's stock.
        Setup& s = *p.findSetup(second);
        s.stock.kind = StockKind::FromSetup;
        s.stock.sourceSetup = first;
        s.wcs.origin = Vec3(-1.0, kWidth + 1.0, -1.0);
        s.wcs.xAxis = Vec3::UnitX();
        s.wcs.zAxis = -Vec3::UnitZ();
    });
    SimulationOptions options;
    options.resolution = 0.25;
    const SetupSimulation one = simulateSetup(doc, first, options, cancel, progress);
    REQUIRE_FALSE(one.hasErrors());
    REQUIRE(one.comparison.has_value());
    // The chamfer cut part material: a gouge to a comparison that does not know it is one.
    CHECK(sim::compareToPart(one.finalStock(), one.part, {options.tolerance}).hasGouges());
    CHECK(std::any_of(one.chamfered.begin(), one.chamfered.end(),
                      [](const sim::IntervalList& l) { return !l.empty(); }));
    CHECK_FALSE(one.comparison->hasGouges());

    // Turned over, the chamfers are at the bottom, and still not gouges.
    const SetupSimulation two = simulateSetup(doc, second, options, cancel, progress);
    REQUIRE(two.comparison.has_value());
    CHECK(sim::compareToPart(two.finalStock(), two.part, {options.tolerance}).hasGouges());
    CHECK(std::any_of(two.initialChamfered.begin(), two.initialChamfered.end(),
                      [](const sim::IntervalList& l) { return !l.empty(); }));
    CHECK_FALSE(two.comparison->hasGouges());
}

TEST_CASE("a simulation goes on from the operations that did not change",
          "[document][simulate][resume]") {
    using namespace fixtures::block_pocket;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid modelId = doc.importModel(
        std::filesystem::path(OCAM_FIXTURE_DIR) / "block_pocket.step", cancel, progress);
    const Uuid setupId = doc.addSetup(modelId);
    const tooling::Tool tool = testing::withPreset(tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    doc.modify("setup", [&](Project& p) {
        p.tools.tools.push_back(tool);
        for (const char* type : {"facing", "pocket2d"}) {
            Operation op;
            op.name = type;
            op.type = type;
            op.toolId = tool.id;
            op.presetId = tool.presets.front().id;
            if (std::string(type) == "pocket2d") {
                op.geometry = {faceAtHeight(doc, modelId, kHeight - kPocketDepth)};
                op.presetOverrides = testing::stepDown(5.0);
            }
            p.findSetup(setupId)->operations.push_back(op);
        }
    });
    SimulationOptions options;
    options.resolution = 0.25;
    const auto first = std::make_shared<const SetupSimulation>(
        simulateSetup(doc, setupId, options, cancel, progress));
    REQUIRE(first->operations.size() == 2);
    const std::vector<std::string> keys =
        stockInputKeys(doc, *doc.project().findSetup(setupId), options);
    REQUIRE(keys.size() == 3);
    CHECK(first->initialKey == keys[0]);
    CHECK(first->operations[0].inputKey == keys[1]);
    CHECK(first->operations[1].inputKey == keys[2]);
    // Taken as they are, not simulated again: the same stock (every tile shared), the same
    // operation (its tool shape shared).
    const auto same = [](const sim::Stock& a, const sim::Stock& b) {
        const sim::DexelGrid& ga = a.grid(2);
        const sim::DexelGrid& gb = b.grid(2);
        for (int tj = 0; tj < ga.tileRows(); ++tj) {
            for (int ti = 0; ti < ga.tileColumns(); ++ti) {
                if (ga.tileId(ti, tj) != gb.tileId(ti, tj)) {
                    return false;
                }
            }
        }
        return true;
    };
    const auto taken = [](const OperationSimulation& a, const OperationSimulation& b) {
        return a.tool && a.tool == b.tool;
    };

    // The pocket changes: the facing is taken as it was, the pocket simulated again, and the
    // result is what a simulation from the start gives.
    doc.modify("deeper", [&](Project& p) {
        p.findSetup(setupId)->operations[1].presetOverrides.axialDepth = 2.0;
    });
    SimulationOptions resumed = options;
    resumed.resumeFrom = first;
    const SetupSimulation second = simulateSetup(doc, setupId, resumed, cancel, progress);
    const SetupSimulation fresh = simulateSetup(doc, setupId, options, cancel, progress);
    REQUIRE(second.operations.size() == 2);
    CHECK(same(second.initialStock, first->initialStock));
    CHECK(taken(second.operations[0], first->operations[0]));
    CHECK_FALSE(taken(second.operations[1], first->operations[1]));
    CHECK(second.operations[1].inputKey != first->operations[1].inputKey);
    CHECK(second.operations[1].inputKey == fresh.operations[1].inputKey);
    CHECK(second.finalStock().volume() == fresh.finalStock().volume());
    CHECK(second.operations[1].removedVolume == fresh.operations[1].removedVolume);
    REQUIRE(second.comparison.has_value());
    CHECK(second.comparison->excessVolume == fresh.comparison->excessVolume);

    // The facing changes: nothing after it is the same, so only the initial stock is taken.
    doc.modify("facing", [&](Project& p) {
        p.findSetup(setupId)->operations[0].presetOverrides.axialDepth = 0.5;
    });
    const SetupSimulation third = simulateSetup(doc, setupId, resumed, cancel, progress);
    CHECK(same(third.initialStock, first->initialStock));
    CHECK_FALSE(taken(third.operations[0], first->operations[0]));
    CHECK_FALSE(taken(third.operations[1], first->operations[1]));

    // Another resolution is another initial stock: nothing is taken.
    SimulationOptions coarser = resumed;
    coarser.resolution = 0.5;
    const SetupSimulation fourth = simulateSetup(doc, setupId, coarser, cancel, progress);
    CHECK(fourth.resolution == 0.5);
    CHECK_FALSE(same(fourth.initialStock, first->initialStock));
    CHECK_FALSE(taken(fourth.operations[0], first->operations[0]));
}

TEST_CASE("a setup simulation saved to bytes and read back", "[document][simulate][cache]") {
    using namespace fixtures::block_pocket;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid modelId = doc.importModel(
        std::filesystem::path(OCAM_FIXTURE_DIR) / "block_pocket.step", cancel, progress);
    const Uuid setupId = doc.addSetup(modelId);
    const tooling::Tool tool = testing::withPreset(tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    doc.modify("setup", [&](Project& p) {
        p.tools.tools.push_back(tool);
        for (const char* type : {"facing", "pocket2d"}) {
            Operation op;
            op.name = type;
            op.type = type;
            op.toolId = tool.id;
            op.presetId = tool.presets.front().id;
            if (std::string(type) == "pocket2d") {
                op.geometry = {faceAtHeight(doc, modelId, kHeight - kPocketDepth)};
                op.presetOverrides = testing::stepDown(5.0);
            }
            p.findSetup(setupId)->operations.push_back(op);
        }
    });
    SimulationOptions options;
    options.resolution = 0.25;
    const Setup& setup = *doc.project().findSetup(setupId);
    const SetupSimulation sim = simulateSetup(doc, setupId, options, cancel, progress);
    const std::string key = simulationInputKey(doc, setup, options);
    const std::string bytes = encodeSimulation(sim, key);

    const std::optional<SetupSimulation> back = decodeSimulation(bytes, key, doc, setup);
    REQUIRE(back.has_value());
    CHECK(back->setupId == sim.setupId);
    CHECK(back->resolution == sim.resolution);
    CHECK(back->initialKey == sim.initialKey);
    CHECK(back->stockWarnings == sim.stockWarnings);
    CHECK(back->initialChamfered == sim.initialChamfered);
    CHECK(back->chamfered == sim.chamfered);
    CHECK(back->initialStock.volume() == sim.initialStock.volume());
    REQUIRE(back->operations.size() == sim.operations.size());
    for (std::size_t k = 0; k < sim.operations.size(); ++k) {
        const OperationSimulation& a = sim.operations[k];
        const OperationSimulation& b = back->operations[k];
        CHECK(b.operationId == a.operationId);
        CHECK(b.name == a.name);
        CHECK(b.inputKey == a.inputKey);
        CHECK(b.stockAfter.volume() == a.stockAfter.volume());
        CHECK(b.removedVolume == a.removedVolume);
        CHECK(b.toolpath.moves.size() == a.toolpath.moves.size());
        CHECK_FALSE(toolpath::compareToolpaths(a.toolpath, b.toolpath).has_value());
        CHECK((b.tool != nullptr) == (a.tool != nullptr));
        CHECK(b.rotary.angles.size() == a.rotary.angles.size());
        CHECK(b.error == a.error);
    }
    // Checkpoints share exactly the tiles they shared before (memory as before): after facing
    // and after the pocket, every tile the pocket did not reach.
    const auto shared = [](const SetupSimulation& s) {
        const sim::DexelGrid& a = s.operations[0].stockAfter.grid(2);
        const sim::DexelGrid& b = s.operations[1].stockAfter.grid(2);
        int n = 0;
        for (int tj = 0; tj < a.tileRows(); ++tj) {
            for (int ti = 0; ti < a.tileColumns(); ++ti) {
                n += a.tileId(ti, tj) == b.tileId(ti, tj) ? 1 : 0;
            }
        }
        return n;
    };
    CHECK(shared(*back) == shared(sim));
    CHECK(shared(*back) > 0);
    REQUIRE(back->comparison.has_value());
    CHECK(back->comparison->excessVolume == sim.comparison->excessVolume);
    CHECK(back->comparison->excess == sim.comparison->excess);
    CHECK((back->partReference != nullptr) == (sim.partReference != nullptr));
    CHECK(back->part.triangles.size() == sim.part.triangles.size());
    REQUIRE(back->machine);
    CHECK(back->machine->id == sim.machine->id);

    // Another key, damaged or truncated bytes: simulated again (nothing read).
    CHECK_FALSE(decodeSimulation(bytes, key + "x", doc, setup).has_value());
    std::string damaged = bytes;
    damaged[damaged.size() / 2] ^= 0x5a;
    CHECK_FALSE(decodeSimulation(damaged, key, doc, setup).has_value());
    CHECK_FALSE(
        decodeSimulation(std::string_view(bytes).substr(0, bytes.size() / 3), key, doc, setup)
            .has_value());
}

// Measures the simulation cache on a project of one's own: OCAM_CACHE_BENCH=<project.ocam>.
TEST_CASE("simulation cache on a real project", "[.cachebench]") {
    const char* file = std::getenv("OCAM_CACHE_BENCH");
    if (!file) {
        SKIP("set OCAM_CACHE_BENCH to a project file");
    }
    CancelToken cancel;
    NullProgress progress;
    const Document doc = Document::open(std::filesystem::path(file), cancel, progress);
    for (const Setup& setup : doc.project().setups) {
        SimulationOptions options;
        const auto t0 = std::chrono::steady_clock::now();
        const SetupSimulation sim = simulateSetup(doc, setup.id, options, cancel, progress);
        const auto t1 = std::chrono::steady_clock::now();
        const std::string key = simulationInputKey(doc, setup, options);
        const std::string bytes = encodeSimulation(sim, key);
        const auto t2 = std::chrono::steady_clock::now();
        const auto back = decodeSimulation(bytes, key, doc, setup);
        const auto t3 = std::chrono::steady_clock::now();
        const auto ms = [](auto a, auto b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        std::printf("%s: simulate %.0f ms, encode %.0f ms (%.1f MB), decode %.0f ms, %s\n",
                    setup.name.c_str(), ms(t0, t1), ms(t1, t2), bytes.size() / 1e6, ms(t2, t3),
                    back ? "read back" : "NOT READ");
        CHECK(back.has_value());
    }
}
