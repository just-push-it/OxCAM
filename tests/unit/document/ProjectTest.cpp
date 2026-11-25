// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "TestProject.hpp"

#include <ocam/document/PostSetup.hpp>
#include <ocam/document/ProjectIo.hpp>
#include <ocam/document/Stock.hpp>
#include <ocam/foundation/Error.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

using namespace ocam;
using namespace ocam::document;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("frames normalize and transform", "[document][frame]") {
    const Frame f{{10, 0, 5}, {2, 0, 0.5}, {0, 0, 3}};
    const Frame n = f.normalized();
    CHECK((n.xAxis - Vec3::UnitX()).norm() < 1e-12);
    CHECK((n.zAxis - Vec3::UnitZ()).norm() < 1e-12);
    CHECK((n.yAxis() - Vec3::UnitY()).norm() < 1e-12);
    CHECK(((f.toParent() * Vec3(1, 2, 3)) - Vec3(11, 2, 8)).norm() < 1e-12);

    const Frame rotated{{0, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // 90 degrees about Z
    CHECK(((rotated.toParent() * Vec3(1, 0, 0)) - Vec3(0, 1, 0)).norm() < 1e-12);

    CHECK_THROWS_AS((Frame{{0, 0, 0}, {0, 0, 1}, {0, 0, 1}}.normalized()), Error);
    CHECK_THROWS_AS((Frame{{0, 0, 0}, {1, 0, 0}, {0, 0, 0}}.normalized()), Error);
}

TEST_CASE("work offset names", "[document]") {
    CHECK(workOffsetName(1) == "G54");
    CHECK(workOffsetName(6) == "G59");
    CHECK(workOffsetName(7) == "G54.1 P1");
}

TEST_CASE("project validation finds dangling references", "[document][project]") {
    Project p = testing::sampleProject();
    CHECK(p.validate().empty());

    Project broken = p;
    broken.setups[0].modelId = Uuid::generate();
    broken.setups[0].operations[0].toolId = Uuid::generate();
    broken.setups[0].operations[1].presetId = Uuid::generate();
    broken.setups[1].stock.sourceSetup = broken.setups[1].id; // self reference
    broken.setups[0].workOffset = 0;
    broken.setups.push_back(broken.setups[0]); // duplicate ids
    CHECK(broken.validate().size() >= 6);

    Project badStock = p;
    badStock.setups[0].stock.kind = StockKind::Box;
    badStock.setups[0].stock.size = StockSize::Fixed;
    badStock.setups[0].stock.boxCorners = {Vec3(0, 0, 0), Vec3(10, 0, 10)};
    CHECK(badStock.validate().size() == 1);
    badStock.setups[0].stock.kind = StockKind::Tube;
    badStock.setups[0].stock.diameter = 40;
    badStock.setups[0].stock.length = 20;
    badStock.setups[0].stock.innerDiameter = 40; // no wall
    CHECK(badStock.validate().size() == 1);
    badStock.setups[0].stock.innerDiameter = 30;
    CHECK(badStock.validate().empty());
}

TEST_CASE("fixtures: validation and round-trip", "[document][project][io]") {
    Project p = testing::sampleProject();
    ModelEntry vise;
    vise.name = "vise";
    vise.sha256 = sha256Hex("vise");
    p.models.push_back(vise);
    p.setups[0].fixtures = {vise.id};
    p.setups[0].fixtureClearance = 2.5;
    CHECK(p.validate().empty());

    const Json doc = toJson(p);
    CHECK(doc.at("setups").at(0).at("fixtures").size() == 1);
    CHECK_FALSE(doc.at("setups").at(1).contains("fixtures"));
    const Project back = projectFromJson(doc);
    CHECK(back == p);
    CHECK(back.setups[0].fixtures == std::vector<Uuid>{vise.id});
    CHECK(back.setups[0].fixtureClearance == 2.5);

    Project broken = p;
    broken.setups[0].fixtures.push_back(Uuid::generate());     // missing model
    broken.setups[0].fixtures.push_back(vise.id);              // twice
    broken.setups[0].fixtures.push_back(*p.setups[0].modelId); // its own part
    broken.setups[0].fixtureClearance = -1.0;
    CHECK(broken.validate().size() == 4);
}

TEST_CASE("setup placements: round-trip and older files", "[document][project][io]") {
    Project p = testing::sampleProject();
    SetupPlacement placement;
    placement.carrier = "C";
    placement.target = Vec3(0, 0, 50);
    placement.targetOn = "the centre of a face of 'zero-point plate'";
    placement.workPoint = Vec3(50, 30, 0);
    placement.workPointOn = "the centre of a face of the stock";
    placement.offset = Vec3(1, -2, 0.5);
    placement.rotation = Vec3(0, 0, 90);
    p.setups[0].placement = placement;
    SetupPlacement typed;
    typed.target = Vec3(10, 20, -30); // on the base, the WCS origin there, nothing snapped
    p.setups[1].placement = typed;

    const Json doc = toJson(p);
    projectSchema().validate(doc, "project");
    const Json& first = doc.at("setups").at(0).at("placement");
    CHECK(first.at("carrier") == "C");
    CHECK(first.at("work_point").size() == 3);
    const Json& second = doc.at("setups").at(1).at("placement");
    CHECK_FALSE(second.contains("carrier"));
    CHECK_FALSE(second.contains("work_point"));
    CHECK_FALSE(second.contains("offset"));
    const Project back = projectFromJson(doc);
    CHECK(back == p);
    REQUIRE(back.setups[0].placement);
    CHECK(*back.setups[0].placement == placement);

    // Files without placements (all older ones) load without one: the default placement.
    const Json plain = toJson(testing::sampleProject());
    CHECK_FALSE(plain.at("setups").at(0).contains("placement"));
    CHECK_FALSE(projectFromJson(plain).setups[0].placement);

    // A carrier that is no machine axis is refused by the schema.
    Json bad = doc;
    bad.at("setups").at(0).at("placement")["carrier"] = "W";
    CHECK_THROWS_AS(projectSchema().validate(bad, "project"), Error);
}

TEST_CASE("project lookups", "[document][project]") {
    Project p = testing::sampleProject();
    const Uuid opId = p.setups[0].operations[1].id;
    const Setup* owner = nullptr;
    const Operation* op = std::as_const(p).findOperation(opId, &owner);
    REQUIRE(op != nullptr);
    CHECK(owner == &p.setups[0]);
    CHECK(p.findSetup(p.setups[1].id) == &p.setups[1]);
    CHECK(p.findModel(p.models[0].id) == &p.models[0]);
    CHECK(p.findOperation(Uuid::generate()) == nullptr);
}

TEST_CASE("operations move within and between setups", "[document][project]") {
    Project p = testing::sampleProject();
    Setup& first = p.setups[0];
    Setup& second = p.setups[1];
    REQUIRE(first.operations.size() == 2);
    REQUIRE(second.operations.size() == 1);
    const Uuid facing = first.operations[0].id;
    const Uuid pocket = first.operations[1].id;
    const Uuid drill = second.operations[0].id;
    auto ids = [](const Setup& s) {
        std::vector<Uuid> out;
        for (const auto& op : s.operations) {
            out.push_back(op.id);
        }
        return out;
    };

    // In front of itself or of its successor: no change.
    CHECK_FALSE(p.moveOperation(facing, first.id, 0));
    CHECK_FALSE(p.moveOperation(facing, first.id, 1));
    // Down past the pocket (index counted before the move).
    CHECK(p.moveOperation(facing, first.id, 2));
    CHECK(ids(first) == std::vector<Uuid>{pocket, facing});
    // Back up to the front.
    CHECK(p.moveOperation(facing, first.id, 0));
    CHECK(ids(first) == std::vector<Uuid>{facing, pocket});
    // To another setup, in front of its drill, then past the end (appends).
    CHECK(p.moveOperation(pocket, second.id, 0));
    CHECK(ids(first) == std::vector<Uuid>{facing});
    CHECK(ids(second) == std::vector<Uuid>{pocket, drill});
    CHECK(p.moveOperation(facing, second.id, 99));
    CHECK(first.operations.empty());
    CHECK(ids(second) == std::vector<Uuid>{pocket, drill, facing});
    // Unknown operation or setup.
    CHECK_FALSE(p.moveOperation(Uuid::generate(), second.id, 0));
    CHECK_FALSE(p.moveOperation(drill, Uuid::generate(), 0));
    CHECK(p.validate().empty());
}

TEST_CASE("project JSON round-trip is exact and deterministic", "[document][io]") {
    Project p = testing::sampleProject();
    // Operation heights and customized cutting values travel too.
    Operation& op = p.setups.front().operations.front();
    op.heights.retract = {HeightRef::StockTop, 3.5};
    op.heights.bottom = {HeightRef::Geometry, -0.25};
    op.heights.top = {HeightRef::Picked, 0.1,
                      ShapeRef{p.models.front().id, cad::SubShapeType::Edge, 3, {}}};
    op.presetOverrides.feedPerTooth = 0.07;
    op.presetOverrides.surfaceSpeed = 2500.0;
    // A contour's outline with its own side; the operation's side is not written.
    op.geometry = {ShapeRef{p.models.front().id, cad::SubShapeType::Edge, 4, {}},
                   ShapeRef{p.models.front().id, cad::SubShapeType::Face, 2, {}}};
    op.geometry.front().side = GeometrySide::Inside;
    const Json doc = toJson(p);
    CHECK(doc.at("format") == "oxcam.project");
    const Json& written = doc.at("setups").at(0).at("operations").at(0).at("geometry");
    CHECK(written.at(0).at("side") == "inside");
    CHECK_FALSE(written.at(1).contains("side"));
    const Project back = projectFromJson(doc);
    CHECK(back == p);
    CHECK(dumpJson(toJson(back)) == dumpJson(doc));
}

TEST_CASE("older files' step parameters become preset overrides", "[document][io]") {
    Project p = testing::sampleProject();
    Json doc = toJson(p);
    // The first operation's tool is the 10 mm end mill: a stepover ratio becomes a width.
    Json& ops = doc.at("setups").at(0).at("operations");
    ops.at(0)["parameters"]["step_down"] = 4.0;
    ops.at(0)["parameters"]["stepover_ratio"] = 0.5;
    ops.at(1)["parameters"]["step_down"] = 0.0; // "one level": the preset decides now
    ops.at(1)["parameters"]["stepover"] = 1.5;
    ops.at(1)["preset_overrides"] = Json{{"radial_depth", 2.5}}; // set already: kept
    const Project back = projectFromJson(doc);
    const auto& first = back.setups.front().operations;
    CHECK(first.at(0).presetOverrides.axialDepth == 4.0);
    CHECK(first.at(0).presetOverrides.radialDepth == 5.0);
    CHECK_FALSE(first.at(1).presetOverrides.axialDepth.has_value());
    CHECK(first.at(1).presetOverrides.radialDepth == 2.5);
    for (const Operation& op : first) {
        CHECK_FALSE(op.parameters.contains("step_down"));
        CHECK_FALSE(op.parameters.contains("stepover_ratio"));
        CHECK_FALSE(op.parameters.contains("stepover"));
    }
}

TEST_CASE("older files' plunge feed shares become feed rates", "[document][io]") {
    Project p = testing::sampleProject();
    Json doc = toJson(p);
    // The facing operation's preset: 5000 mm/s on a 10 mm, 3-flute end mill at 0.05 mm/tooth;
    // the operation itself runs at 0.1 mm/tooth.
    Json& ops = doc.at("setups").at(0).at("operations");
    ops.at(0)["preset_overrides"] = Json{{"feed_per_tooth", 0.1}, {"plunge_feed_factor", 0.25}};
    const Project back = projectFromJson(doc);
    const double n = 5000.0 / (kPi * 10.0);
    CHECK_THAT(*back.setups[0].operations[0].presetOverrides.plungeFeed,
               WithinAbs(0.25 * 0.1 * 3 * n, 1e-9));
    // Saved again, the feed rate is what is kept.
    CHECK(toJson(back)
              .at("setups")
              .at(0)
              .at("operations")
              .at(0)
              .at("preset_overrides")
              .contains("plunge_feed"));
}

TEST_CASE("older files keep the lead feed their turning had", "[document][io]") {
    Project p = testing::sampleProject();
    tooling::Tool insert = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 9);
    insert.presets.front().feedPerTooth = 0.25;
    p.tools.tools.push_back(insert);
    const auto add = [&](PresetOverrides po) {
        Operation op;
        op.name = "Finish";
        op.type = "turn_finish";
        op.toolId = insert.id;
        op.presetId = insert.presets.front().id;
        op.presetOverrides = po;
        p.setups[0].operations.push_back(op);
    };
    PresetOverrides own;
    own.feedPerTooth = 0.3;
    add(own);
    add({});
    const std::size_t first = p.setups[0].operations.size() - 2;
    Json doc = toJson(p);
    Json& presetJson = doc.at("tools").at("tools").back().at("presets").at(0);

    SECTION("a file from before it had its own feed led at the feed") {
        presetJson.erase("lead_feed_per_rev");
        const Project back = projectFromJson(doc);
        CHECK_THAT(back.tools.tools.back().presets[0].leadFeedPerRev, WithinAbs(0.25, 1e-12));
        CHECK_THAT(*back.setups[0].operations[first].presetOverrides.leadFeedPerRev,
                   WithinAbs(0.3, 1e-12));
        CHECK_FALSE(back.setups[0].operations[first + 1].presetOverrides.leadFeedPerRev);
    }
    SECTION("a file with it keeps it, 0 included") {
        presetJson["lead_feed_per_rev"] = 0.0;
        doc.at("setups").at(0).at("operations").at(first + 1)["preset_overrides"] =
            Json{{"lead_feed_per_rev", 0.0}};
        const Project back = projectFromJson(doc);
        CHECK(back.tools.tools.back().presets[0].leadFeedPerRev == 0.0);
        CHECK_FALSE(back.setups[0].operations[first].presetOverrides.leadFeedPerRev);
        CHECK(*back.setups[0].operations[first + 1].presetOverrides.leadFeedPerRev == 0.0);
    }
}

TEST_CASE("older files keep the end feed their part-offs had", "[document][io]") {
    Project p = testing::sampleProject();
    tooling::Tool parting = tooling::makeDefaultTool(tooling::ToolType::PartingInsert, 9);
    parting.insert.width = 3.0;
    parting.presets.front().feedPerTooth = 0.06;
    p.tools.tools.push_back(parting);
    const auto add = [&](PresetOverrides po) {
        Operation op;
        op.name = "Part off";
        op.type = "turn_partoff";
        op.toolId = parting.id;
        op.presetId = parting.presets.front().id;
        op.presetOverrides = po;
        p.setups[0].operations.push_back(op);
    };
    PresetOverrides own;
    own.feedPerTooth = 0.08;
    add(own);
    add({});
    const std::size_t first = p.setups[0].operations.size() - 2;
    Json doc = toJson(p);
    const auto partingPreset = [](Json& d) -> Json& {
        return d.at("tools").at("tools").back().at("presets").at(0);
    };

    SECTION("a file from before it had its own feed ran at half") {
        partingPreset(doc).erase("part_off_end_feed");
        const Project back = projectFromJson(doc);
        CHECK_THAT(back.tools.tools.back().presets[0].partOffEndFeed, WithinAbs(0.03, 1e-12));
        CHECK_THAT(*back.setups[0].operations[first].presetOverrides.partOffEndFeed,
                   WithinAbs(0.04, 1e-12));
        CHECK_FALSE(back.setups[0].operations[first + 1].presetOverrides.partOffEndFeed);
    }
    SECTION("a file with it keeps it, 0 included") {
        partingPreset(doc)["part_off_end_feed"] = 0.0;
        doc.at("setups").at(0).at("operations").at(first + 1)["preset_overrides"] =
            Json{{"part_off_end_feed", 0.0}};
        const Project back = projectFromJson(doc);
        CHECK(back.tools.tools.back().presets[0].partOffEndFeed == 0.0);
        CHECK_FALSE(back.setups[0].operations[first].presetOverrides.partOffEndFeed);
        CHECK(*back.setups[0].operations[first + 1].presetOverrides.partOffEndFeed == 0.0);
    }
}

TEST_CASE("older files keep the ramp and plunge feeds their operations had", "[document][io]") {
    Project p = testing::sampleProject();
    Json doc = toJson(p);
    // As saved before ramps had their own feed and plunges were a share: the facing
    // operation's preset plunged at 0.4 of the feed and ramped at the feed; the operation runs at
    // 0.1 mm/tooth instead of the preset's 0.05. The pocket keeps the preset's feeds.
    for (Json& t : doc.at("tools").at("tools")) {
        for (Json& pj : t.at("presets")) {
            pj.erase("ramp_feed");
            pj.erase("lead_feed");
            pj.erase("plunge_feed");
            pj["plunge_feed_factor"] = 0.4;
        }
    }
    Json& ops = doc.at("setups").at(0).at("operations");
    ops.at(0)["preset_overrides"] = Json{{"feed_per_tooth", 0.1}};
    const Project back = projectFromJson(doc);
    const double n = 5000.0 / (kPi * 10.0);
    const double presetFeed = 0.05 * 3 * n;
    const tooling::CuttingPreset& preset = back.tools.tools.front().presets.front();
    CHECK_THAT(preset.rampFeed, WithinAbs(presetFeed, 1e-9));
    CHECK_THAT(preset.leadFeed, WithinAbs(presetFeed, 1e-9));
    CHECK_THAT(preset.plungeFeed, WithinAbs(0.4 * presetFeed, 1e-9));
    const PresetOverrides& own = back.setups[0].operations[0].presetOverrides;
    CHECK_THAT(*own.rampFeed, WithinAbs(0.1 * 3 * n, 1e-9));
    CHECK_THAT(*own.leadFeed, WithinAbs(0.1 * 3 * n, 1e-9));
    CHECK_THAT(*own.plungeFeed, WithinAbs(0.4 * 0.1 * 3 * n, 1e-9));
    const PresetOverrides& pocket = back.setups[0].operations[1].presetOverrides;
    CHECK_FALSE(pocket.rampFeed.has_value());
    CHECK_FALSE(pocket.leadFeed.has_value());
    CHECK_FALSE(pocket.plungeFeed.has_value());
}

TEST_CASE("the trial roughing types load as 3D roughing", "[document][io]") {
    const Json doc = toJson(testing::sampleProject());
    {
        Json d = doc;
        Json& ops = d.at("setups").at(0).at("operations");
        REQUIRE(ops.size() >= 2);
        ops.at(0)["type"] = "zlevel_fronts";
        ops.at(1)["type"] = "zlevel_medial";
        ops.at(1)["parameters"]["open_areas"] = "medial";
        const Project back = projectFromJson(d);
        const auto& first = back.setups.front().operations;
        CHECK(first.at(0).type == "zlevel_rough");
        CHECK(first.at(1).type == "zlevel_rough");
        CHECK(first.at(1).parameters.at("open_areas") == "medial"); // kept
    }
    {
        // The traced clearing's "clearing" choice goes; other operations are untouched.
        Json d = doc;
        Json& ops = d.at("setups").at(0).at("operations");
        const std::string otherType = ops.at(1).at("type").get<std::string>();
        ops.at(0)["type"] = "zlevel_rough";
        ops.at(0)["parameters"]["clearing"] = "offset";
        const Project back = projectFromJson(d);
        const auto& first = back.setups.front().operations;
        CHECK(first.at(0).type == "zlevel_rough");
        CHECK_FALSE(first.at(0).parameters.contains("clearing"));
        CHECK(first.at(1).type == otherType);
    }
}

TEST_CASE("older files' tool multiples become lengths", "[document][io]") {
    Project p = testing::sampleProject();
    Operation& op = p.setups.front().operations.front();
    const tooling::Tool* tool = p.tools.findTool(*op.toolId);
    REQUIRE(tool != nullptr);
    const double d = tool->mill.diameter;
    Json doc = toJson(p);
    Json& params = doc.at("setups").at(0).at("operations").at(0)["parameters"];
    params["helix_radius_ratio"] = 0.5; // of the radius
    params["lead_radius_ratio"] = 0.25;
    params["peck_ratio"] = 2.0;
    params["pitch_ratio"] = 0.4;
    const Project back = projectFromJson(doc);
    const Json& values = back.setups.front().operations.front().parameters;
    CHECK_THAT(values.at("helix_radius").get<double>(), WithinRel(0.25 * d, 1e-12));
    CHECK_THAT(values.at("lead_radius").get<double>(), WithinRel(0.25 * d, 1e-12));
    CHECK_THAT(values.at("peck_depth").get<double>(), WithinRel(2.0 * d, 1e-12));
    CHECK_THAT(values.at("plunge_spacing").get<double>(), WithinRel(0.4 * d, 1e-12));
    CHECK_FALSE(values.contains("helix_radius_ratio"));
    CHECK_FALSE(values.contains("peck_ratio"));
}

TEST_CASE("older contours' and chamfers' side goes onto their outlines", "[document][io]") {
    Project p = testing::sampleProject();
    Operation& op = p.setups.front().operations.front();
    op.type = "chamfer2d";
    op.geometry = {ShapeRef{p.models.front().id, cad::SubShapeType::Edge, 4, {}},
                   ShapeRef{p.models.front().id, cad::SubShapeType::Edge, 5, {}}};
    op.geometry.back().side = GeometrySide::Outside; // its own side stays
    Json doc = toJson(p);
    doc.at("setups").at(0).at("operations").at(0)["parameters"] = Json{{"side", "inside"}};
    const Project loaded = projectFromJson(doc);
    const Operation& back = loaded.setups.front().operations.front();
    CHECK_FALSE(back.parameters.contains("side"));
    CHECK(back.geometry.at(0).side == GeometrySide::Inside);
    CHECK(back.geometry.at(1).side == GeometrySide::Outside);
    // Contours the same way.
    doc.at("setups").at(0).at("operations").at(0)["type"] = "contour2d";
    const Project contour = projectFromJson(doc);
    const Operation& c = contour.setups.front().operations.front();
    CHECK_FALSE(c.parameters.contains("side"));
    CHECK(c.geometry.at(0).side == GeometrySide::Inside);
}

TEST_CASE("invalid project documents are rejected", "[document][io]") {
    const Json good = toJson(testing::sampleProject());
    auto codeOf = [](Json d) {
        try {
            (void)projectFromJson(std::move(d));
        } catch (const Error& e) {
            return e.code();
        }
        return ErrorCode::Internal;
    };
    CHECK(codeOf(good) == ErrorCode::Internal);

    Json noSetups = good;
    noSetups.erase("setups");
    CHECK(codeOf(noSetups) == ErrorCode::Parse);

    Json badStock = good;
    badStock["setups"][0]["stock"]["kind"] = "sphere";
    CHECK(codeOf(badStock) == ErrorCode::Parse);

    Json badTools = good;
    badTools["tools"]["tools"][0]["type"] = "spork";
    CHECK(codeOf(badTools) == ErrorCode::Parse);

    Json badHash = good;
    badHash["models"][0]["sha256"] = "1234";
    CHECK(codeOf(badHash) == ErrorCode::Parse);

    Json future = good;
    future["version"] = kProjectVersion + 1;
    CHECK(codeOf(future) == ErrorCode::Unsupported);
}

TEST_CASE("archive round-trip and integrity checks", "[document][io]") {
    ArchiveContents in;
    in.projectJson = R"({"hello": "world"})";
    const std::string step = "ISO-10303-21; pretend STEP content";
    in.models.emplace(sha256Hex(step), step);

    const std::string bytes = writeArchive(in);
    const ArchiveContents out = readArchive(bytes, "test.ocam");
    CHECK(out.projectJson == in.projectJson);
    CHECK(out.models == in.models);

    CHECK_THROWS_AS(readArchive("definitely not a zip", "x.ocam"), Error);

    ArchiveContents tampered;
    tampered.projectJson = "{}";
    tampered.models.emplace(std::string(64, 'a'), step); // name does not match content
    CHECK_THROWS_AS(readArchive(writeArchive(tampered), "t.ocam"), Error);
}

TEST_CASE("stock resolution", "[document][stock]") {
    Project p = testing::sampleProject();
    const geom3d::Aabb3 modelBox{Vec3(0, 0, 0), Vec3(100, 60, 30)};
    const ModelBoundsFn bounds = [&](const Uuid&, const Mat3& axes) {
        geom3d::Aabb3 b;
        for (int i = 0; i < 8; ++i) {
            b.extend(axes.transpose() * Vec3((i & 1) ? modelBox.max.x() : modelBox.min.x(),
                                             (i & 2) ? modelBox.max.y() : modelBox.min.y(),
                                             (i & 4) ? modelBox.max.z() : modelBox.min.z()));
        }
        return std::optional(b);
    };

    p.setups[0].stock = Stock{};
    p.setups[0].stock.offsetSide = 2.0;
    p.setups[0].stock.offsetTop = 1.5;
    p.setups[0].stock.offsetBottom = 0.5;
    const auto offset = resolveStock(p, p.setups[0], bounds);
    REQUIRE(offset.has_value());
    CHECK((offset->box.min - Vec3(-2, -2, -0.5)).norm() < 1e-12);
    CHECK((offset->box.max - Vec3(102, 62, 31.5)).norm() < 1e-12);

    // Setup 2 takes its stock from setup 1.
    const auto chained = resolveStock(p, p.setups[1], bounds);
    REQUIRE(chained.has_value());
    CHECK(*chained == *offset);

    // Sizes rounded up to a multiple of 25: the extra on both sides in X and Y, on top in Z.
    p.setups[0].stock.roundTo = 25.0;
    const auto rounded = resolveStock(p, p.setups[0], bounds);
    REQUIRE(rounded.has_value());
    CHECK((rounded->box.min - Vec3(-12.5, -7.5, -0.5)).norm() < 1e-9);
    CHECK((rounded->box.max - Vec3(112.5, 67.5, 49.5)).norm() < 1e-9);
    p.setups[0].stock.roundTo = 0.0;

    // Offsets follow the WCS: with the tool axis along project -Z the top offset goes below.
    const Frame wcs = p.setups[0].wcs;
    p.setups[0].wcs.zAxis = -Vec3::UnitZ();
    p.setups[0].wcs.xAxis = Vec3::UnitX();
    const auto flipped = resolveStock(p, p.setups[0], bounds);
    REQUIRE(flipped.has_value());
    CHECK((flipped->bounds().min - Vec3(-2, -2, -1.5)).norm() < 1e-9);
    CHECK((flipped->bounds().max - Vec3(102, 62, 30.5)).norm() < 1e-9);
    p.setups[0].wcs = wcs;

    // A cylinder of an earlier file, placed in the project frame.
    Stock cyl;
    cyl.kind = StockKind::Cylinder;
    cyl.size = StockSize::Fixed;
    cyl.cylinderPlacement = {Vec3(0, 0, 0), Vec3(1, 0, 0)};
    cyl.diameter = 40;
    cyl.length = 120;
    p.setups[0].stock = cyl;
    const auto c = resolveStock(p, p.setups[0], bounds);
    REQUIRE(c.has_value());
    CHECK(c->kind == StockShape::Kind::Cylinder);
    CHECK((c->bounds().min - Vec3(0, -20, -20)).norm() < 1e-12);
    CHECK((c->bounds().max - Vec3(120, 20, 20)).norm() < 1e-12);

    const ModelBoundsFn none = [](const Uuid&, const Mat3&) {
        return std::optional<geom3d::Aabb3>();
    };
    p.setups[0].stock = Stock{};
    CHECK_FALSE(resolveStock(p, p.setups[0], none).has_value());
}

TEST_CASE("stock shapes and sizes", "[document][stock]") {
    Project p = testing::sampleProject();
    Setup& setup = p.setups[0];
    setup.wcs = Frame{Vec3(7, -3, 2), Vec3::UnitX(), Vec3::UnitZ()}; // project axes
    const geom3d::Aabb3 modelBox{Vec3(0, 0, 0), Vec3(100, 60, 30)};  // centre (50, 30, 15)
    const ModelBoundsFn bounds = [&](const Uuid&, const Mat3& axes) {
        geom3d::Aabb3 b;
        for (int i = 0; i < 8; ++i) {
            b.extend(axes.transpose() * Vec3((i & 1) ? modelBox.max.x() : modelBox.min.x(),
                                             (i & 2) ? modelBox.max.y() : modelBox.min.y(),
                                             (i & 4) ? modelBox.max.z() : modelBox.min.z()));
        }
        return std::optional(b);
    };
    // The part's largest radius: 35 about any line (the line asked for is kept).
    Vec3 askedPoint = Vec3::Zero(), askedDirection = Vec3::Zero();
    const ModelRadiusFn radius = [&](const Uuid&, const Vec3& point, const Vec3& direction) {
        askedPoint = point;
        askedDirection = direction;
        return std::optional(35.0);
    };
    auto near = [](const Vec3& a, const Vec3& b) {
        return (a - b).norm() < 1e-9;
    };
    Stock& st = setup.stock;

    SECTION("box") {
        st = Stock{};
        st.size = StockSize::Part; // the part's box, nothing added
        auto b = resolveStock(p, setup, bounds, radius);
        REQUIRE(b);
        CHECK(near(b->box.min, modelBox.min));
        CHECK(near(b->box.max, modelBox.max));
        st.shift = Vec3(1, -2, 3); // Base X, Y, Z: moved from centred
        b = resolveStock(p, setup, bounds, radius);
        CHECK(near(b->box.min, Vec3(1, -2, 3)));
        st.size = StockSize::Fixed;
        st.boxSize = Vec3(120, 80, 40);
        st.shift = Vec3::Zero();
        b = resolveStock(p, setup, bounds, radius);
        CHECK(near(b->box.min, Vec3(-10, -10, -5)));
        CHECK(near(b->box.max, Vec3(110, 70, 35)));
        // Without a model, fixed stock is centred on the project origin.
        Setup bare = setup;
        bare.modelId.reset();
        b = resolveStock(p, bare, bounds, radius);
        CHECK(near(b->box.min, Vec3(-60, -40, -20)));
        bare.stock.size = StockSize::OffsetFromPart;
        CHECK_FALSE(resolveStock(p, bare, bounds, radius));
    }
    SECTION("cylinder, fixed along X, centred on the part") {
        st = Stock{};
        st.kind = StockKind::Cylinder;
        st.size = StockSize::Fixed;
        st.axis = 0;
        st.diameter = 40;
        st.length = 120;
        auto c = resolveStock(p, setup, bounds, radius);
        REQUIRE(c);
        CHECK(c->kind == StockShape::Kind::Cylinder);
        CHECK(near(c->base, Vec3(-10, 30, 15)));
        CHECK(near(c->axis, Vec3::UnitX()));
        CHECK(c->diameter == 40);
        CHECK(c->length == 120);
        st.shift = Vec3(5, 0, -2);
        c = resolveStock(p, setup, bounds, radius);
        CHECK(near(c->base, Vec3(-5, 30, 13)));
    }
    SECTION("cylinder offset from the part, rounded up") {
        st = Stock{};
        st.kind = StockKind::Cylinder;
        st.offsetSide = 2.0;
        st.offsetTop = 1.5;
        st.offsetBottom = 0.5;
        auto c = resolveStock(p, setup, bounds, radius);
        REQUIRE(c);
        // About the line through the part box's centre, along WCS Z.
        CHECK(near(askedPoint, Vec3(50, 30, 15)));
        CHECK(near(askedDirection, Vec3::UnitZ()));
        CHECK(c->diameter == Catch::Approx(74.0));
        CHECK(near(c->base, Vec3(50, 30, -0.5)));
        CHECK(c->length == Catch::Approx(32.0));
        // Along Z the extra length goes to the top; the diameter rounds up too.
        st.roundTo = 25.0;
        c = resolveStock(p, setup, bounds, radius);
        CHECK(c->diameter == Catch::Approx(75.0));
        CHECK(near(c->base, Vec3(50, 30, -0.5)));
        CHECK(c->length == Catch::Approx(50.0));
        // Along X the ends get the end offsets and share the extra.
        st.axis = 0;
        c = resolveStock(p, setup, bounds, radius);
        CHECK(near(askedDirection, Vec3::UnitX()));
        CHECK(c->length == Catch::Approx(125.0));
        CHECK(near(c->base, Vec3(-12, 30, 15)));
        // No radius to measure with: not resolved.
        CHECK_FALSE(resolveStock(p, setup, bounds));
    }
    SECTION("tube with no additional stock") {
        st = Stock{};
        st.kind = StockKind::Tube;
        st.size = StockSize::Part;
        st.axis = 1;
        st.offsetSide = 5.0; // not used: nothing added
        st.innerDiameter = 20.0;
        auto t = resolveStock(p, setup, bounds, radius);
        REQUIRE(t);
        CHECK(t->diameter == Catch::Approx(70.0));
        CHECK(t->innerDiameter == 20.0);
        CHECK(near(t->base, Vec3(50, 0, 15)));
        CHECK(t->length == Catch::Approx(60.0));
        // The bore must be inside the stock.
        st.innerDiameter = 70.0;
        CHECK_FALSE(resolveStock(p, setup, bounds, radius));
        st.innerDiameter = 0.0;
        CHECK_FALSE(resolveStock(p, setup, bounds, radius));
    }
}

TEST_CASE("stock shapes and sizes round-trip; earlier kinds open as the same stock",
          "[document][stock][io]") {
    const Project p = testing::sampleProject();
    // Every shape and size reads back as written.
    std::vector<Stock> stocks;
    for (const StockKind kind : {StockKind::Box, StockKind::Cylinder, StockKind::Tube}) {
        for (const StockSize size :
             {StockSize::Fixed, StockSize::OffsetFromPart, StockSize::Part}) {
            Stock st;
            st.kind = kind;
            st.size = size;
            st.axis = static_cast<int>(stocks.size()) % 3;
            st.boxSize = Vec3(10, 20, 30);
            st.diameter = 40;
            st.length = 50;
            st.innerDiameter = 12;
            st.offsetSide = 1.25;
            st.offsetTop = 2.5;
            st.offsetBottom = 0.75;
            st.roundTo = 5;
            st.shift = Vec3(1, -2, 3);
            stocks.push_back(st);
        }
    }
    Stock legacyBox;
    legacyBox.size = StockSize::Fixed;
    legacyBox.boxCorners = {Vec3(0, 0, 0), Vec3(10, 20, 30)};
    stocks.push_back(legacyBox);
    Stock legacyCylinder;
    legacyCylinder.kind = StockKind::Cylinder;
    legacyCylinder.size = StockSize::Fixed;
    legacyCylinder.cylinderPlacement = {Vec3(1, 2, 3), Vec3(0, 1, 0)};
    stocks.push_back(legacyCylinder);
    for (const Stock& st : stocks) {
        Project q = p;
        q.setups[0].stock = st;
        CHECK(projectFromJson(toJson(q)).setups[0].stock == st);
    }

    // Files of earlier versions: one 'kind', read as the shape and size of the same stock.
    auto read = [&](const Json& stock) {
        Json j = toJson(p);
        j.at("setups").at(0)["stock"] = stock;
        return projectFromJson(j).setups[0].stock;
    };
    const Stock offset = read({{"kind", "offset_from_model"},
                               {"box_min", {0, 0, 0}},
                               {"box_max", {0, 0, 0}},
                               {"cylinder_base", {0, 0, 0}},
                               {"cylinder_axis", {0, 0, 1}},
                               {"cylinder_diameter", 0},
                               {"cylinder_length", 0},
                               {"offset_side", 2.0},
                               {"offset_top", 1.5},
                               {"offset_bottom", 0.5},
                               {"round_to", 25.0}});
    CHECK(offset.kind == StockKind::Box);
    CHECK(offset.size == StockSize::OffsetFromPart);
    CHECK(offset.offsetSide == 2.0);
    CHECK(offset.offsetTop == 1.5);
    CHECK(offset.offsetBottom == 0.5);
    CHECK(offset.roundTo == 25.0);
    CHECK(offset.shift == Vec3::Zero());
    CHECK_FALSE(offset.boxCorners);
    const Stock sized = read({{"kind", "box"},
                              {"box_min", {0, 0, 0}},
                              {"box_max", {0, 0, 0}},
                              {"box_size", {110, 70, 40}},
                              {"box_shift", {2, 0, 4}}});
    CHECK(sized.kind == StockKind::Box);
    CHECK(sized.size == StockSize::Fixed);
    CHECK(sized.boxSize == Vec3(110, 70, 40));
    CHECK(sized.shift == Vec3(2, 0, 4));
    CHECK_FALSE(sized.boxCorners);
    const Stock corners =
        read({{"kind", "box"}, {"box_min", {-1, -2, -3}}, {"box_max", {10, 20, 30}}});
    CHECK(corners.size == StockSize::Fixed);
    REQUIRE(corners.boxCorners);
    CHECK((*corners.boxCorners)[0] == Vec3(-1, -2, -3));
    CHECK((*corners.boxCorners)[1] == Vec3(10, 20, 30));
    const Stock cylinder = read({{"kind", "cylinder"},
                                 {"cylinder_base", {0, 0, 0}},
                                 {"cylinder_axis", {1, 0, 0}},
                                 {"cylinder_diameter", 40},
                                 {"cylinder_length", 120}});
    CHECK(cylinder.kind == StockKind::Cylinder);
    CHECK(cylinder.size == StockSize::Fixed);
    CHECK(cylinder.diameter == 40);
    CHECK(cylinder.length == 120);
    REQUIRE(cylinder.cylinderPlacement);
    CHECK((*cylinder.cylinderPlacement)[1] == Vec3::UnitX());
    const Stock source =
        read({{"kind", "from_setup"}, {"source_setup", p.setups[1].id.toString()}});
    CHECK(source.kind == StockKind::FromSetup);
    CHECK(source.sourceSetup == p.setups[1].id);
    // Written again in the new keys only.
    Project q = p;
    q.setups[0].stock = cylinder;
    const Json written = toJson(q).at("setups").at(0).at("stock");
    CHECK(written.at("shape") == "cylinder");
    CHECK_FALSE(written.contains("kind"));
}

TEST_CASE("custom machines travel with the project", "[document][project][machine]") {
    Project p = testing::sampleProject();
    machine::Machine custom = machine::genericMill4A();
    custom.id = "shop-haas-vf2-trt";
    custom.name = "Shop VF-2 with a rotary table";
    custom.axes.back().min = -720;
    custom.axes.back().max = 720;
    p.machines.push_back(custom);
    p.setups[0].machine = custom.id;
    CHECK(p.validate().empty());
    CHECK(p.findMachine("shop-haas-vf2-trt") != nullptr);
    CHECK(&machineFor(p, p.setups[0]) == &p.machines.front());
    CHECK(machineFor(p, p.setups[1]).id == machine::genericMill3().id);

    const Project back = projectFromJson(toJson(p));
    REQUIRE(back.machines.size() == 1);
    CHECK(back.machines.front() == custom);
    CHECK(back == p);

    Project unknown = p;
    unknown.setups[0].machine = "no-such-machine";
    CHECK(unknown.validate().size() == 1);
    // Nothing stands in for it, and a project naming it still reads and writes as it was.
    CHECK(findMachineFor(unknown, unknown.setups[0]) == nullptr);
    CHECK_THAT(machineProblem(unknown, unknown.setups[0]), ContainsSubstring("no-such-machine"));
    CHECK_THROWS_WITH(machineFor(unknown, unknown.setups[0]), ContainsSubstring("no-such-machine"));
    CHECK(machineProblem(p, p.setups[0]).empty());
    const Project unknownBack = projectFromJson(toJson(unknown));
    CHECK(unknownBack.setups[0].machine == "no-such-machine");
    // No machine named at all is the generic 3-axis VMC, as the setup's properties show it.
    Project none = p;
    none.setups[0].machine.clear();
    CHECK(&machineFor(none, none.setups[0]) == &machine::genericMill3());
    Project duplicate = p;
    duplicate.machines.push_back(custom);
    CHECK_FALSE(duplicate.validate().empty());
}

TEST_CASE("operation tool orientations: round-trip, older files, validation",
          "[document][io][orientation]") {
    Project p = testing::sampleProject();
    // Older files (and operations at the setup's WCS) write no orientation.
    const Json plain = toJson(p);
    CHECK_FALSE(plain.at("setups").at(0).at("operations").at(0).contains("tool_orientation"));
    CHECK_FALSE(projectFromJson(plain).setups.front().operations.front().orientation);

    Operation& op = p.setups.front().operations.front();
    op.orientation = ToolOrientation{Frame{{5, -1, -10}, {1, 0, 0}, {0, -1, 0}}, 3};
    const Json doc = toJson(p);
    const Json& written = doc.at("setups").at(0).at("operations").at(0).at("tool_orientation");
    CHECK(written.at("work_offset") == 3);
    CHECK(written.at("frame").at("z_axis") == Json::array({0.0, -1.0, 0.0}));
    const Project back = projectFromJson(doc);
    CHECK(back == p);
    CHECK(dumpJson(toJson(back)) == dumpJson(doc));
    // The frame maps operation coordinates into the WCS.
    const Transform3 t = operationFrame(back.setups.front().operations.front());
    CHECK((t * Vec3(0, 0, 1) - Vec3(5, -2, -10)).norm() < 1e-12);
    CHECK((t * Vec3(0, 1, 0) - Vec3(5, -1, -9)).norm() < 1e-12);
    CHECK(operationFrame(back.setups.front().operations.back()).isApprox(Transform3::Identity()));
    CHECK(p.validate().empty());

    Project bad = p;
    bad.setups.front().operations.front().orientation->frame.xAxis = Vec3(0, 2, 0); // along Z
    bad.setups.front().operations.front().orientation->workOffset = -1;
    CHECK(bad.validate().size() == 2);
}

TEST_CASE("posts, post choices and option values round-trip", "[document][project][io][post]") {
    Project p = testing::sampleProject();
    p.posts.push_back({"my-vmc.js", "export const manifest = {};"});
    p.setups[0].post = PostChoice{"my-vmc", true};
    p.setups[0].postOptions = Json{{"program", {{"optionalStop", true}}},
                                   {"operations", {{"some-op", {{"smoothing", "finish"}}}}}};
    p.setups[1].post = PostChoice{"fanuc-generic-mill", false};
    const Json doc = toJson(p);
    CHECK(doc.at("posts").at(0).at("file") == "my-vmc.js");
    CHECK(doc.at("setups").at(0).at("post") == Json{{"id", "my-vmc"}, {"source", "project"}});
    CHECK(doc.at("setups").at(1).at("post").at("source") == "builtin");
    const Project back = projectFromJson(doc);
    CHECK(back == p);

    // Projects without posts write none.
    const Json plain = toJson(testing::sampleProject());
    CHECK_FALSE(plain.contains("posts"));
    CHECK_FALSE(plain.at("setups").at(0).contains("post"));
    CHECK_FALSE(plain.at("setups").at(0).contains("post_options"));

    // A machine's default post (machine format version 6).
    machine::Machine m = machine::genericMill3();
    m.id = "my-vmc";
    m.post = "my-vmc";
    const Json mj = machine::toJson(m);
    CHECK(mj.at("version") == machine::kMachineVersion);
    CHECK(mj.at("post") == "my-vmc");
    CHECK(machine::machineFromJson(mj).post == "my-vmc");
    CHECK_FALSE(machine::toJson(machine::genericMill3()).contains("post"));
}
