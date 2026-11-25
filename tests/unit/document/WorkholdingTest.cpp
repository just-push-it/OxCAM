// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "FixtureSpecs.hpp"
#include "TestProject.hpp"

#include <ocam/cad/Assembly.hpp>
#include <ocam/cad/Datum.hpp>
#include <ocam/cad/Features.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/ProjectIo.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/document/Workholding.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Sha256.hpp>
#include <ocam/tooling/Profile.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <map>
#include <set>

using namespace ocam;
using namespace ocam::document;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::Equals;
using Catch::Matchers::StartsWith;
using Catch::Matchers::WithinAbs;

namespace {

const std::filesystem::path kFixtures(OCAM_FIXTURE_DIR);

bool near(const Vec3& a, const Vec3& b, double tol = 1e-6) {
    return (a - b).norm() < tol;
}

/// A version 1 vise document as OpenCAM wrote it: the jaws open along +Y, the fixed jaw's face
/// at Y 20, its top at Z 70, the moving jaw modelled 50 mm away, openings 0 to 150 mm.
Json v1Vise(const std::string& kind = "vise") {
    const bool centring = kind == "self_centring";
    Json bodies = Json::array();
    for (const auto& [name, role] :
         {std::pair{"base", "base"}, std::pair{"fixed", "fixed_jaw"}, std::pair{"moving", "jaw"}}) {
        const std::string r = centring && std::string(role) == "fixed_jaw" ? "opposite_jaw" : role;
        bodies.push_back({{"id", Uuid::generate().toString()},
                          {"name", name},
                          {"sha256", sha256Hex(name)},
                          {"role", r}});
    }
    return {{"format", "opencam.workholding"},
            {"version", 1},
            {"units", "mm"},
            {"id", Uuid::generate().toString()},
            {"name", "Vise"},
            {"description", ""},
            {"kind", kind},
            {"datum",
             {{"origin", Json::array({0, centring ? 45 : 20, 70})},
              {"x_axis", Json::array({0, 1, 0})},
              {"z_axis", Json::array({0, 0, 1})}}},
            {"modelled_opening", 50},
            {"min_opening", 0},
            {"max_opening", 150},
            {"bodies", bodies}};
}

const PosedPart& partOn(const WorkholdingPose& pose, const std::string& member) {
    const auto it = std::find_if(pose.parts.begin(), pose.parts.end(),
                                 [&](const PosedPart& p) { return p.member == member; });
    REQUIRE(it != pose.parts.end());
    return *it;
}

/// A template from one of the generated fixture files.
WorkholdingPackage fixture(const char* file) {
    return loadWorkholdingFile(kFixtures / file);
}

// Stock 102 x 62 x 31 in the WCS, its top at Z 0 (block_pocket's offset stock).
const geom3d::Aabb3 kStock{Vec3(0, 0, -31), Vec3(102, 62, 0)};

} // namespace

TEST_CASE("version 1 vises migrate and hold the stock as before", "[document][workholding][io]") {
    const WorkholdingTemplate w = workholdingFromJson(v1Vise(), "vise");
    CHECK(w.validate().empty());
    REQUIRE(w.members.size() == 1);
    CHECK(w.members[0].name == "Moving jaw");
    CHECK_THAT(w.members[0].minTravel, WithinAbs(-50.0, 1e-9));
    CHECK_THAT(w.members[0].maxTravel, WithinAbs(100.0, 1e-9));
    REQUIRE(w.parts.size() == 3);
    CHECK(w.parts[1].member.empty());
    CHECK(w.parts[2].member == "Moving jaw");
    REQUIRE(w.holdings.size() == 1);
    CHECK(w.holdings[0].kind == HoldingKind::Jaws);

    SetupWorkholding placement;
    placement.templateId = w.id;
    placement.gripDepth = 8.0;
    const WorkholdingPose pose = poseWorkholding(w, placement, kStock);
    CHECK_THAT(pose.opening, WithinAbs(102.0, 1e-9));
    // The old datum (fixed jaw face, jaw top, across the middle) at the stock's -X side.
    CHECK(near(pose.templateToWcs * Vec3(0, 20, 70), Vec3(0, 31, -23)));
    CHECK(near(partOn(pose, "Moving jaw").toWcs * Vec3(0, 70, 70), Vec3(102, 31, -23)));
    CHECK(partOn(pose, "").toWcs.isApprox(pose.templateToWcs));

    // Self-centring: both jaws travel half the change, the stock stays centred.
    const WorkholdingTemplate c = workholdingFromJson(v1Vise("self_centring"), "centring");
    CHECK(c.validate().empty());
    REQUIRE(c.members.size() == 2);
    CHECK(c.members[1].linkedTo == "Moving jaw");
    const WorkholdingPose centred = poseWorkholding(c, placement, kStock);
    CHECK_THAT(centred.travel.at("Moving jaw"), WithinAbs(26.0, 1e-9));
    CHECK_THAT(centred.travel.at("Opposite jaw"), WithinAbs(26.0, 1e-9));
    CHECK_THAT((partOn(centred, "Opposite jaw").toWcs * Vec3(0, 20, 70)).x(), WithinAbs(0.0, 1e-9));
    CHECK_THAT((partOn(centred, "Moving jaw").toWcs * Vec3(0, 70, 70)).x(), WithinAbs(102.0, 1e-9));
}

TEST_CASE("workholding templates: JSON round-trip and validation", "[document][workholding]") {
    WorkholdingTemplate w = workholdingFromJson(v1Vise(), "vise");
    WorkholdingDatum offset;
    offset.name = "Above the jaws";
    offset.kind = DatumKind::Offset;
    offset.from = "Jaw tops";
    offset.along = DatumAxis::Direction;
    offset.distance = 10.0;
    w.datums.push_back(offset);
    WorkholdingDatum middle;
    middle.name = "Centre";
    middle.kind = DatumKind::Midpoint;
    middle.from = "Fixed jaw face";
    middle.to = "Moving jaw face";
    w.datums.push_back(middle);
    w.parts[0].colour = 0x123456;
    w.parts[0].collides = false;
    w.mount = "Centre";
    CHECK(w.validate().empty());
    const Json doc = toJson(w);
    CHECK(doc.at("format") == "oxcam.workholding");
    CHECK(doc.at("version") == kWorkholdingVersion);
    CHECK(workholdingFromJson(doc, "vise") == w);

    SECTION("schema violations are rejected") {
        Json bad = doc;
        bad["holdings"][0]["kind"] = "magnet";
        CHECK_THROWS_AS(workholdingFromJson(bad, "vise"), Error);
        bad = doc;
        bad["datums"][0]["kind"] = "guess";
        CHECK_THROWS_AS(workholdingFromJson(bad, "vise"), Error);
        bad = doc;
        bad["parts"] = Json::array();
        CHECK_THROWS_AS(workholdingFromJson(bad, "vise"), Error);
        bad = doc;
        bad["version"] = kWorkholdingVersion + 1; // newer than this build
        CHECK_THROWS_AS(workholdingFromJson(bad, "vise"), Error);
    }
    SECTION("template problems") {
        WorkholdingTemplate t = w;
        t.parts[2].member = "Clamp";
        CHECK_THAT(t.validate().front(), ContainsSubstring("no member 'Clamp'"));
        t = w;
        t.members[0].maxTravel = -60.0;
        CHECK_THAT(t.validate().front(), ContainsSubstring("largest travel"));
        t = w;
        t.datums.back().to = "Nowhere";
        CHECK_THAT(t.validate().front(), ContainsSubstring("does not exist"));
        t = w;
        t.datums.back().from = "Centre";
        CHECK_THAT(t.validate().front(), ContainsSubstring("from itself"));
        t = w;
        t.holdings[0].seat.clear();
        CHECK_THAT(t.validate().front(), ContainsSubstring("jaw tops"));
        t = w;
        t.holdings.clear();
        CHECK_THAT(t.validate().front(), ContainsSubstring("no stock holding"));
        t = w;
        t.members.push_back({"Follower", Vec3::UnitX(), 0, 10, "Ghost", 1.0});
        CHECK_THAT(t.validate().front(), ContainsSubstring("unknown member"));
    }
}

TEST_CASE("workholding files: archive round-trip and integrity", "[document][workholding]") {
    WorkholdingPackage pkg{workholdingFromJson(v1Vise(), "vise"), {}};
    for (auto& p : pkg.workholding.parts) {
        pkg.files.emplace(p.sha256, p.name); // sha256Hex(name) is the hash of these bytes
    }
    const std::string bytes = writeWorkholdingArchive(pkg);
    const WorkholdingPackage back = readWorkholdingArchive(bytes, "vise.ocamhold");
    CHECK(back.workholding == pkg.workholding);
    CHECK(back.files == pkg.files);

    const auto path = std::filesystem::temp_directory_path() / "oxcam_test_vise.ocamhold";
    saveWorkholdingFile(path, pkg);
    CHECK(loadWorkholdingFile(path).workholding == pkg.workholding);
    std::filesystem::remove(path);

    WorkholdingPackage missing = pkg;
    missing.files.erase(missing.files.begin());
    CHECK_THROWS_AS(writeWorkholdingArchive(missing), Error);
    WorkholdingPackage corrupt = pkg;
    corrupt.files.begin()->second = "tampered";
    CHECK_THROWS_AS(readWorkholdingArchive(writeWorkholdingArchive(corrupt), "x"), Error);
    CHECK_THROWS_AS(readWorkholdingArchive("not a zip", "x"), Error);
}

TEST_CASE("datums from picked geometry and from each other", "[document][workholding]") {
    using namespace fixtures::plate;
    const WorkholdingPackage pkg = fixture("plate.ocamhold");
    const WorkholdingTemplate& w = pkg.workholding;
    CancelToken cancel;
    NullProgress progress;
    const cad::Model plate =
        cad::Model::importStepData(pkg.files.at(w.parts[0].sha256), "plate", cancel, progress);
    const cad::Model pin =
        cad::Model::importStepData(pkg.files.at(w.parts[1].sha256), "pin", cancel, progress);

    // The plate's top: its centre and outward normal, exactly.
    const WorkholdingDatum* top = w.findDatum("Top");
    REQUIRE(top);
    CHECK(top->shape == DatumShape::Plane);
    CHECK(near(top->point, Vec3(kLength / 2, kWidth / 2, kThickness)));
    CHECK(near(top->direction, Vec3::UnitZ()));
    // The front edge along X.
    const WorkholdingDatum* edge = w.findDatum("Front edge");
    REQUIRE(edge);
    CHECK(edge->shape == DatumShape::Line);
    CHECK_THAT(std::abs(edge->direction.x()), WithinAbs(1.0, 1e-9));
    CHECK(near(edge->point, Vec3(kLength / 2, 0, kThickness)));

    // The pin's round face: its axis; a point clicked on the plate's top; a vertex.
    bool foundAxis = false;
    for (int f = 1; f <= cad::subShapeCount(pin, cad::SubShapeType::Face); ++f) {
        const WorkholdingDatum d = pickedDatum("Pin", w.parts[1], pin, cad::SubShapeType::Face, f);
        if (d.shape == DatumShape::Axis) {
            foundAxis = true;
            CHECK_THAT(d.point.x(), WithinAbs(kPinX, 1e-6));
            CHECK_THAT(d.point.y(), WithinAbs(kPinY, 1e-6));
            CHECK_THAT(std::abs(d.direction.z()), WithinAbs(1.0, 1e-9));
        }
    }
    CHECK(foundAxis);
    const WorkholdingDatum onTop =
        pickedDatum("Clicked", w.parts[0], plate, cad::SubShapeType::Face, top->index,
                    Vec3(30, 40, kThickness + 0.3));
    CHECK(onTop.shape == DatumShape::Point);
    CHECK(near(onTop.point, Vec3(30, 40, kThickness)));
    CHECK(near(onTop.direction, Vec3::UnitZ()));
    const WorkholdingDatum corner =
        pickedDatum("Vertex", w.parts[0], plate, cad::SubShapeType::Vertex, 1);
    CHECK(corner.shape == DatumShape::Point);
    CHECK(corner.direction.isZero());

    // Offsets and midpoints follow their datums.
    WorkholdingTemplate t = w;
    WorkholdingDatum up;
    up.name = "Up 5";
    up.kind = DatumKind::Offset;
    up.from = "Top";
    up.along = DatumAxis::Direction;
    up.distance = 5.0;
    t.datums.push_back(up);
    WorkholdingDatum mid;
    mid.name = "Mid";
    mid.kind = DatumKind::Midpoint;
    mid.from = "Top";
    mid.to = "Front edge";
    t.datums.push_back(mid);
    CHECK(near(datumAt(t, "Up 5", {}).point, Vec3(kLength / 2, kWidth / 2, kThickness + 5)));
    CHECK(near(datumAt(t, "Mid", {}).point, Vec3(kLength / 2, kWidth / 4, kThickness)));
    CHECK_THROWS_WITH(datumAt(t, "Nowhere", {}), ContainsSubstring("no datum"));
}

TEST_CASE("a standard vise from one STEP file closes on the stock", "[document][workholding]") {
    using namespace fixtures::vise;
    CancelToken cancel;
    NullProgress progress;
    // Every body of the file a part.
    const auto bodies = cad::importStepAssembly(kFixtures / "vise_assembly.step", cancel, progress);
    REQUIRE(bodies.size() == 3);
    WorkholdingPackage pkg;
    std::vector<std::pair<WorkholdingPart, geom3d::Aabb3>> parts;
    for (const auto& b : bodies) {
        parts.emplace_back(addWorkholdingPart(pkg, b.name, b.model), b.model.info().bounds);
    }
    std::string why;
    const auto w = viseTemplate("Vise", {parts, parts[1].first.id, parts[2].first.id, false},
                                Vec3::UnitY(), Vec3::UnitZ(), kMaxOpening, &why);
    REQUIRE(w);
    CHECK(w->validate().empty());
    CHECK(near(datumAt(*w, "Fixed jaw face", {}).point, Vec3(0, kFixedFace, kJawTop)));
    CHECK(near(datumAt(*w, "Moving jaw face", {}).point,
               Vec3(0, kFixedFace + kModelledOpening, kJawTop)));
    CHECK(near(datumAt(*w, "Bottom centre", {}).point, Vec3(0, kBaseLength / 2 - 10, 0)));
    // Wrong way round, or no fixed jaw.
    CHECK_FALSE(viseTemplate("Vise", {parts, parts[1].first.id, parts[2].first.id, false},
                             -Vec3::UnitY(), Vec3::UnitZ(), kMaxOpening, &why));
    CHECK_THAT(why, ContainsSubstring("side"));
    CHECK_FALSE(viseTemplate("Vise", {parts, Uuid::generate(), parts[2].first.id, false},
                             Vec3::UnitY(), Vec3::UnitZ(), kMaxOpening, &why));
    CHECK_THAT(why, ContainsSubstring("fixed jaw"));

    SetupWorkholding placement;
    placement.templateId = w->id;
    placement.gripDepth = kGripDepth;
    SECTION("jaws along WCS Y, closed on the stock, the fixed jaw on its side") {
        placement.rotation = 90;
        placement.shift = 5.0;
        const WorkholdingPose pose = poseWorkholding(*w, placement, kStock);
        CHECK_THAT(pose.opening, WithinAbs(62.0, 1e-9));
        CHECK_THAT(pose.travel.at("Moving jaw"), WithinAbs(12.0, 1e-9));
        // Template Y (the jaws' opening) is WCS Y; across the jaws is WCS -X, shifted 5 mm.
        CHECK(
            near(pose.templateToWcs * Vec3(0, kFixedFace, kJawTop), Vec3(56, 0, -31 + kGripDepth)));
        CHECK_THAT((partOn(pose, "Moving jaw").toWcs * Vec3(0, 70, 70)).y(), WithinAbs(62, 1e-9));
        CHECK(near(pose.mount, pose.templateToWcs * Vec3(0, kBaseLength / 2 - 10, 0)));
    }
    SECTION("a typed opening") {
        placement.jaws = JawSetting::Opening;
        placement.opening = 120.0;
        WorkholdingPose pose = poseWorkholding(*w, placement, kStock);
        CHECK_THAT((partOn(pose, "Moving jaw").toWcs * Vec3(0, 70, 70)).x(),
                   WithinAbs(120.0, 1e-9));
        CHECK(pose.warnings.empty());
        placement.opening = 90.0; // closer than the stock is wide
        pose = poseWorkholding(*w, placement, kStock);
        REQUIRE(pose.warnings.size() == 1);
        CHECK_THAT(pose.warnings.front(), ContainsSubstring("closer than the stock"));
        placement.opening = 160.0;
        CHECK_THROWS_WITH(poseWorkholding(*w, placement, kStock),
                          StartsWith("the jaws can't reach 160.000 mm: they open "));
        // Posed as far as it goes: the moving jaw at the end of its travel, drawn in error.
        pose = poseWorkholding(*w, placement, kStock, nullptr, std::nullopt, true);
        REQUIRE(pose.outOfReach);
        CHECK(pose.outOfReach->kind == WorkholdingReach::Kind::Open);
        CHECK(pose.outOfReach->wanted == 160.0);
        const WorkholdingMember& moving = w->members.front();
        CHECK(pose.travel.at(moving.name) == Approx(moving.maxTravel));
        CHECK(std::any_of(pose.parts.begin(), pose.parts.end(),
                          [](const PosedPart& part) { return part.outOfReach; }));
        CHECK(std::all_of(pose.parts.begin(), pose.parts.end(), [](const PosedPart& part) {
            return part.outOfReach == !part.member.empty();
        }));
        CHECK_THAT(reachMessage(*pose.outOfReach, LengthUnit::Inch),
                   StartsWith("the jaws can't reach 6.299 in: they open "));
    }
    SECTION("stock wider than the vise opens") {
        WorkholdingTemplate small = *w;
        small.members[0].maxTravel = 30.0;
        CHECK_THROWS_WITH(poseWorkholding(small, placement, kStock),
                          StartsWith("the jaws can't reach 102.000 mm"));
    }
}

TEST_CASE("a self-centring vise centres the stock", "[document][workholding]") {
    using namespace fixtures::selfcentring;
    const WorkholdingTemplate w = fixture("selfcentring.ocamhold").workholding;
    CHECK(w.validate().empty());
    SetupWorkholding placement;
    placement.templateId = w.id;
    placement.rotation = 90;
    placement.gripDepth = 6.0;
    for (const double width : {40.0, 62.0, 110.0}) {
        const geom3d::Aabb3 stock{Vec3(0, 0, -20), Vec3(80, width, 0)};
        const WorkholdingPose pose = poseWorkholding(w, placement, stock);
        // Both jaws travel half the change; the stock's centre is the vise's centre.
        CHECK_THAT(pose.travel.at("Moving jaw"), WithinAbs((width - kModelledOpening) / 2, 1e-9));
        CHECK_THAT(pose.travel.at("Opposite jaw"), WithinAbs((width - kModelledOpening) / 2, 1e-9));
        const Vec3 centre = pose.templateToWcs.inverse() * Vec3(40, width / 2, -20);
        CHECK_THAT(centre.y(), WithinAbs(0.0, 1e-9));
        CHECK_THAT(centre.z(), WithinAbs(kJawTop - 6.0, 1e-9));
        CHECK_THAT((partOn(pose, "Moving jaw").toWcs * Vec3(0, kModelledOpening / 2, kJawTop)).y(),
                   WithinAbs(width, 1e-9));
    }
    // Wider than it opens.
    CHECK_THROWS_WITH(poseWorkholding(w, placement, {Vec3(0, 0, -20), Vec3(80, 130, 0)}),
                      StartsWith("the jaws can't reach 130.000 mm"));
}

TEST_CASE("a dovetail fixture seats the stock's tenon", "[document][workholding]") {
    using namespace fixtures::dovetail;
    const WorkholdingTemplate w = fixture("dovetail.ocamhold").workholding;
    CHECK(w.validate().empty());
    REQUIRE(w.findDatum("Fixed flank"));
    CHECK(w.findDatum("Fixed flank")->kind == DatumKind::Picked);
    SetupWorkholding placement;
    placement.templateId = w.id;
    placement.rotation = 90; // the stock's WCS Y across the dovetail
    const geom3d::Aabb3 stock{Vec3(0, 0, -20), Vec3(60, 40, 0)};

    // As modelled: a 30 mm tenon, nothing moves; the tenon's bottom 5 mm below the seat,
    // centred between the flanks and along the fixture.
    WorkholdingPose pose = poseWorkholding(w, placement, stock);
    CHECK_THAT(pose.opening, WithinAbs(kModelledWidth, 1e-6));
    CHECK_THAT(pose.travel.at("Moving jaw"), WithinAbs(0.0, 1e-6));
    CHECK(near(pose.templateToWcs.inverse() * Vec3(30, 20, -20), Vec3(0, 0, kSeat - kDepth), 1e-6));

    // A 34 mm tenon: the moving flank opens 4 mm, the stock sits 2 mm towards it.
    placement.jaws = JawSetting::Opening;
    placement.opening = 34.0;
    pose = poseWorkholding(w, placement, stock);
    CHECK_THAT(pose.travel.at("Moving jaw"), WithinAbs(4.0, 1e-6));
    CHECK(near(pose.templateToWcs.inverse() * Vec3(30, 20, -20), Vec3(0, 2, kSeat - kDepth), 1e-6));
    placement.opening = 60.0;
    CHECK_THROWS_WITH(poseWorkholding(w, placement, stock),
                      StartsWith("the flanks can't hold a 60.000 mm tenon"));
}

TEST_CASE("a custom fixture seats the stock's chosen point", "[document][workholding]") {
    using namespace fixtures::plate;
    const WorkholdingTemplate w = fixture("plate.ocamhold").workholding;
    CHECK(w.validate().empty());
    SetupWorkholding placement;
    placement.templateId = w.id;
    const geom3d::Aabb3 stock{Vec3(0, 0, -20), Vec3(60, 40, 0)};
    WorkholdingPose pose = poseWorkholding(w, placement, stock);
    // The stock's bottom centre on the plate's top centre, its X along the plate's front edge.
    CHECK(near(pose.templateToWcs.inverse() * Vec3(30, 20, -20),
               Vec3(kLength / 2, kWidth / 2, kThickness)));
    CHECK_THAT(std::abs((pose.templateToWcs.linear() * Vec3::UnitX()).x()), WithinAbs(1.0, 1e-9));
    CHECK(near(pose.mount, pose.templateToWcs * Vec3::Zero()));

    // Another point of the stock: its min corner.
    WorkholdingTemplate corner = w;
    corner.holdings[0].anchor = {0, 0, 0};
    pose = poseWorkholding(corner, placement, stock);
    CHECK(near(pose.templateToWcs.inverse() * Vec3(0, 0, -20),
               Vec3(kLength / 2, kWidth / 2, kThickness)));
}

TEST_CASE("setups with a workholding: validation and project round-trip",
          "[document][workholding][io]") {
    Project p = testing::sampleProject();
    const WorkholdingTemplate w = workholdingFromJson(v1Vise(), "vise");
    p.workholdings.push_back(w);
    SetupWorkholding placement;
    placement.templateId = w.id;
    placement.holding = "Jaws";
    placement.jaws = JawSetting::Opening;
    placement.opening = 70.0;
    placement.rotation = 270;
    placement.shift = -2.5;
    placement.travel = {{"Moving jaw", 3.0}};
    p.setups[0].workholding = placement;
    SetupPlacement onMachine;
    onMachine.target = Vec3(1, 2, 3);
    onMachine.onWorkholding = true;
    p.setups[0].placement = onMachine;
    CHECK(p.validate().empty());
    CHECK(hasFixtures(p.setups[0]));
    CHECK_FALSE(hasFixtures(p.setups[1]));

    const Json doc = toJson(p);
    projectSchema().validate(doc, "project");
    CHECK_FALSE(doc.at("setups").at(0).at("workholding").contains("grip_depth"));
    const Project back = projectFromJson(doc);
    CHECK(back == p);
    CHECK(dumpJson(toJson(back)) == dumpJson(doc));

    // Files without workholding stay as they were.
    const Json plain = toJson(testing::sampleProject());
    CHECK_FALSE(plain.contains("workholdings"));
    CHECK_FALSE(plain.at("setups").at(0).contains("workholding"));

    Project bad = p;
    bad.setups[0].workholding->holding = "Second station";
    CHECK_THAT(bad.validate().front(), ContainsSubstring("no holding"));
    bad = p;
    bad.setups[0].workholding->travel = {{"Clamp", 1.0}};
    CHECK_THAT(bad.validate().front(), ContainsSubstring("no member 'Clamp'"));
    bad = p;
    bad.setups[0].workholding->rotation = 45;
    CHECK_THAT(bad.validate().front(), ContainsSubstring("90"));
    bad = p;
    bad.workholdings.clear();
    CHECK_THAT(bad.validate().front(), ContainsSubstring("missing template"));

    Json badDoc = doc;
    badDoc["setups"][0]["workholding"]["rotation"] = 45;
    CHECK_THROWS_AS(projectFromJson(badDoc), Error);

    removeUnusedWorkholdings(p);
    CHECK(p.workholdings.size() == 1);
    p.setups[0].workholding.reset();
    removeUnusedWorkholdings(p);
    CHECK(p.workholdings.empty());
}

TEST_CASE("the vise template holds the stock and keeps toolpaths clear",
          "[document][workholding][generate]") {
    using namespace fixtures;
    CancelToken cancel;
    NullProgress progress;
    const WorkholdingPackage library = fixture("vise.ocamhold");

    Document doc;
    const Uuid model = doc.importModel(kFixtures / "block_pocket.step", cancel, progress);
    const Uuid setupId = doc.addSetup(model);
    tooling::Tool mill = testing::withPreset(tooling::makeFlatEndMill(1, 10.0, 22.0, 72.0));
    doc.modify("tool", [&](Project& p) { p.tools.tools.push_back(mill); });
    const cad::Model& block = *doc.model(model);
    ShapeRef bottom;
    for (const auto& f : cad::findPlanarFaces(block)) {
        if (std::abs(f.height) < 1e-6) {
            bottom = {model, cad::SubShapeType::Face, f.face,
                      cad::signatureOf(block, cad::SubShapeType::Face, f.face)};
        }
    }
    Operation profile;
    profile.name = "Profile";
    profile.type = "contour2d";
    profile.toolId = mill.id;
    profile.presetId = mill.presets.front().id;
    profile.geometry = {bottom};
    profile.presetOverrides = testing::stepDown(6.0);
    doc.modify("op", [&](Project& p) { p.findSetup(setupId)->operations.push_back(profile); });
    auto setup = [&]() -> const Setup& {
        return *doc.project().findSetup(setupId);
    };
    auto key = [&] {
        return toolpathInputKey(doc, setup(), setup().operations.front());
    };
    auto generate = [&] {
        return generateToolpath(doc, setup(), setup().operations.front(), cancel, progress);
    };
    const toolpath::Toolpath free = generate();
    CHECK(free.warnings.empty());
    const std::string k0 = key();

    doc.useWorkholding(library, importWorkholdingBodies(library, cancel, progress), setupId);
    CHECK(doc.undoLabel() == "Use workholding Test vise");
    REQUIRE(setup().workholding);
    doc.modify("place", [&](Project& p) {
        p.findSetup(setupId)->workholding->rotation = 90;
        p.findSetup(setupId)->workholding->gripDepth = vise::kGripDepth;
    });
    CHECK(doc.project().validate().empty());
    const std::string k1 = key();
    CHECK(k1 != k0);

    const auto bodies = fixtureBodiesInWcs(doc, setup());
    REQUIRE(bodies.size() == 3);
    geom3d::Aabb3 jawBox;
    for (const auto& b : bodies) {
        if (b.name == "Fixed jaw") {
            jawBox = b.mesh.bounds();
        }
    }
    CHECK_THAT(jawBox.max.y(), WithinAbs(0.0, 1e-6));
    CHECK_THAT(jawBox.max.z(), WithinAbs(-31.0 + vise::kGripDepth, 1e-6));
    CHECK_THAT(jawBox.min.x(), WithinAbs(1.0, 1e-6)); // 100 wide jaws on the 102 long stock

    // The profile is trimmed where it would run into the jaws: wherever the 10 mm cutter is over
    // a jaw, its tip stays the clearance above it; the ends are still cut to depth.
    const toolpath::Toolpath held = generate();
    REQUIRE_FALSE(held.warnings.empty());
    CHECK_THAT(held.warnings.front(), ContainsSubstring("workholding"));
    const double jawTop = -31.0 + vise::kGripDepth;
    bool cutBelowTops = false;
    for (const auto& run : toolpath::sampleRuns(held, 0.01)) {
        for (const Vec3& q : run.points) {
            const double dx = std::max({1.0 - q.x(), q.x() - 101.0, 0.0});
            const double dy = std::max({q.y() - 0.0, 0.0});
            const double dyMoving = std::max({62.0 - q.y(), 0.0});
            if (std::min(std::hypot(dx, dy), std::hypot(dx, dyMoving)) < 5.0) {
                CHECK(q.z() >= jawTop + 1.0 - 1e-3);
            }
            cutBelowTops =
                cutBelowTops || (q.z() < jawTop && run.feedClass == toolpath::FeedClass::Cut);
        }
    }
    CHECK(cutBelowTops);

    // A part that does not collide is left out of the clearance.
    doc.modify("no collide", [&](Project& p) {
        for (auto& w : p.workholdings) {
            for (auto& part : w.parts) {
                part.collides = part.name != "Moving jaw";
            }
        }
    });
    CHECK(fixtureBodiesInWcs(doc, setup()).size() == 2);
    CHECK(key() != k1);
    doc.undo();

    // The simulation includes the vise: the trimmed toolpath is clear, the free one hits it.
    SimulationOptions options;
    options.compare = false;
    options.resolution = 1.0;
    const SetupSimulation clear = simulateSetup(doc, setupId, options, cancel, progress);
    CHECK(clear.fixtureCollisionCount() == 0);
    CHECK_FALSE(clear.hasErrors());
    options.toolpathFor = [&](const Operation&) {
        return std::optional(free);
    };
    const SetupSimulation crash = simulateSetup(doc, setupId, options, cancel, progress);
    REQUIRE(crash.fixtureCollisionCount() > 0);
    const auto& c = crash.operations.front().fixtureCollisions.front();
    CHECK((c.body == "Fixed jaw" || c.body == "Moving jaw"));

    // A typed opening out of range: generation refuses, naming the range.
    doc.modify("open", [&](Project& p) {
        p.findSetup(setupId)->workholding->jaws = JawSetting::Opening;
        p.findSetup(setupId)->workholding->opening = 200.0;
    });
    CHECK_THROWS_WITH(generate(), ContainsSubstring("the jaws can't reach 200.000 mm"));

    // Saved and opened again: the parts travel in the project.
    doc.modify("open", [&](Project& p) { p.findSetup(setupId)->workholding->opening = 80.0; });
    const auto ocam = std::filesystem::temp_directory_path() / "oxcam_test_vise.ocam";
    doc.save(ocam);
    const Document opened = Document::open(ocam);
    std::filesystem::remove(ocam);
    CHECK(opened.project() == doc.project());
    CHECK(fixtureBodiesInWcs(opened, *opened.project().findSetup(setupId)).size() == 3);
    CHECK(opened.workholdingPackage(library.workholding.id)->files == library.files);

    // Undo takes the vise off again and drops the template.
    while (doc.project().findSetup(setupId)->workholding) {
        doc.undo();
    }
    CHECK(doc.project().workholdings.empty());
    CHECK(key() == k0);
}

TEST_CASE("the fixture project with a vise template generates", "[document][workholding]") {
    const Document doc = Document::open(kFixtures / "vise_project.ocam");
    REQUIRE(doc.project().workholdings.size() == 1);
    const Setup& setup = doc.project().setups.front();
    REQUIRE(setup.workholding);
    CancelToken cancel;
    NullProgress progress;
    for (const Operation& op : setup.operations) {
        CHECK_NOTHROW(generateToolpath(doc, setup, op, cancel, progress));
    }
}

namespace {

/// A workholding's parts from one of the fixtures' multi-body STEP files, every body a part,
/// with their models read back from their files (as the editor has them).
struct Imported {
    WorkholdingPackage package;
    std::map<std::string, cad::Model> models;
    const cad::Model* modelOf(const WorkholdingPart& p) const {
        const auto it = models.find(p.sha256);
        return it == models.end() ? nullptr : &it->second;
    }
    /// The first face of part `index` whose exact geometry satisfies `wanted`.
    WorkholdingPick face(std::size_t index,
                         const std::function<bool(const cad::DatumGeometry&)>& wanted) const {
        const WorkholdingPart& part = package.workholding.parts.at(index);
        const cad::Model& m = *modelOf(part);
        for (int f = 1; f <= cad::subShapeCount(m, cad::SubShapeType::Face); ++f) {
            if (wanted(cad::datumGeometry(m, cad::SubShapeType::Face, f))) {
                return {part.id, cad::SubShapeType::Face, f, std::nullopt};
            }
        }
        FAIL("no such face");
        return {};
    }
    Uuid id(std::size_t index) const { return package.workholding.parts.at(index).id; }
    PartModelFn modelFn() const {
        return [this](const WorkholdingPart& p) {
            return modelOf(p);
        };
    }
};

Imported importParts(const char* file) {
    CancelToken cancel;
    NullProgress progress;
    Imported out;
    for (const auto& b : cad::importStepAssembly(kFixtures / file, cancel, progress)) {
        const WorkholdingPart part = addWorkholdingPart(out.package, b.name, b.model);
        out.models.emplace(part.sha256,
                           cad::Model::importStepData(out.package.files.at(part.sha256), b.name,
                                                      cancel, progress));
    }
    return out;
}

std::function<bool(const cad::DatumGeometry&)> plane(const Vec3& normal) {
    return [normal](const cad::DatumGeometry& g) {
        return g.shape == cad::DatumGeometry::Shape::Plane && g.direction.isApprox(normal, 1e-6);
    };
}

std::function<bool(const cad::DatumGeometry&)> leaning(double y) {
    return [y](const cad::DatumGeometry& g) {
        return g.shape == cad::DatumGeometry::Shape::Plane && std::abs(g.direction.z()) > 0.2 &&
               g.direction.y() * y > 0.5;
    };
}

} // namespace

TEST_CASE("the setup wizard's answers make a standard vise", "[document][workholding][wizard]") {
    using namespace fixtures::vise;
    const Imported in = importParts("vise_assembly.step");
    WorkholdingAnswers a;
    a.type = FixtureType::Vise;
    a.name = "Wizard vise";
    CHECK(missingAnswers(a).size() == 4);
    CHECK_THROWS_WITH(buildWorkholding(a, in.package.workholding.parts, in.modelFn()),
                      ContainsSubstring("moving jaw"));
    a.moving = {in.id(2)};
    a.faceA = in.face(1, plane(Vec3::UnitY()));  // the fixed jaw's face towards the gap
    a.faceB = in.face(2, plane(-Vec3::UnitY())); // the moving jaw's face towards the gap
    a.top = in.face(1, plane(Vec3::UnitZ()));
    a.minOpening = 0.0;
    a.maxOpening = kMaxOpening;
    a.gripDepth = kGripDepth;
    REQUIRE(missingAnswers(a).empty());
    const WorkholdingTemplate w = buildWorkholding(a, in.package.workholding.parts, in.modelFn());
    CHECK(w.validate().empty());
    CHECK(w.up.isApprox(Vec3::UnitZ()));
    REQUIRE(w.members.size() == 1);
    CHECK(w.members[0].direction.isApprox(Vec3::UnitY()));
    CHECK(w.parts[2].member == "Moving jaw");
    CHECK(w.parts[1].member.empty());
    const StockHolding& h = w.holdings.front();
    // The opening as modelled, its range, and a slider position.
    CHECK_THAT(holdingOpening(w, h, {}), WithinAbs(kModelledOpening, 1e-6));
    const auto range = openingRange(w, h);
    REQUIRE(range);
    CHECK_THAT(range->first, WithinAbs(0.0, 1e-6));
    CHECK_THAT(range->second, WithinAbs(kMaxOpening, 1e-6));
    CHECK_THAT(travelsForOpening(w, h, 80.0).at("Moving jaw"), WithinAbs(30.0, 1e-6));
    // It holds stock: the fixed jaw on the stock's side, the jaw tops the grip depth above its
    // bottom, centred across the jaws.
    SetupWorkholding placement;
    placement.templateId = w.id;
    placement.rotation = 90;
    const WorkholdingPose pose = poseWorkholding(w, placement, kStock);
    CHECK_THAT(pose.opening, WithinAbs(62.0, 1e-9));
    CHECK(near(pose.templateToWcs * Vec3(0, kFixedFace, kJawTop), Vec3(51, 0, -31 + kGripDepth)));
    // The mount: the bottom centre of all the parts.
    CHECK(near(datumAt(w, w.mount, {}).point, Vec3(0, kBaseLength / 2 - 10, 0)));

    // The moving jaw's face on a part that does not move: nothing would open the jaws.
    WorkholdingAnswers wrong = a;
    wrong.moving = {in.id(0)};
    CHECK_THROWS_WITH(buildWorkholding(wrong, in.package.workholding.parts, in.modelFn()),
                      ContainsSubstring("moving part"));
}

TEST_CASE("the setup wizard's self-centring vise moves its jaws together",
          "[document][workholding][wizard]") {
    using namespace fixtures::selfcentring;
    const Imported in = importParts("selfcentring_assembly.step");
    WorkholdingAnswers a;
    a.type = FixtureType::SelfCentringVise;
    a.opposite = {in.id(1)};
    a.moving = {in.id(2)};
    a.faceA = in.face(1, plane(Vec3::UnitY()));
    a.faceB = in.face(2, plane(-Vec3::UnitY()));
    a.top = in.face(2, plane(Vec3::UnitZ()));
    a.minOpening = 20.0;
    a.maxOpening = kMaxOpening;
    const WorkholdingTemplate w = buildWorkholding(a, in.package.workholding.parts, in.modelFn());
    CHECK(w.validate().empty());
    REQUIRE(w.members.size() == 2);
    CHECK(w.members[1].linkedTo == w.members[0].name);
    CHECK(w.members[1].direction.isApprox(-w.members[0].direction));
    const StockHolding& h = w.holdings.front();
    const auto range = openingRange(w, h);
    REQUIRE(range);
    CHECK_THAT(range->first, WithinAbs(20.0, 1e-6));
    CHECK_THAT(range->second, WithinAbs(kMaxOpening, 1e-6));
    // One opening drives both jaws: each travels half the change, the centre stays put.
    const auto travel = travelsForOpening(w, h, 90.0);
    CHECK_THAT(travel.at("Moving jaw"), WithinAbs(20.0, 1e-6));
    CHECK_THAT(travel.at("Opposite jaw"), WithinAbs(20.0, 1e-6));
    CHECK_THAT(holdingOpening(w, h, travel), WithinAbs(90.0, 1e-6));
    SetupWorkholding placement;
    placement.templateId = w.id;
    placement.rotation = 90;
    const geom3d::Aabb3 stock{Vec3(0, 0, -20), Vec3(80, 70, 0)};
    const WorkholdingPose pose = poseWorkholding(w, placement, stock);
    CHECK_THAT((pose.templateToWcs.inverse() * Vec3(40, 35, -20)).y(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("the setup wizard's dovetail and seat", "[document][workholding][wizard]") {
    SECTION("dovetail") {
        using namespace fixtures::dovetail;
        const Imported in = importParts("dovetail_assembly.step");
        WorkholdingAnswers a;
        a.type = FixtureType::Dovetail;
        a.moving = {in.id(2)};
        a.faceA = in.face(1, leaning(1.0));
        a.faceB = in.face(2, leaning(-1.0));
        a.top = in.face(1, plane(Vec3::UnitZ()));
        a.tenonDepth = kDepth;
        a.minOpening = 25.0;
        a.maxOpening = 50.0;
        const WorkholdingTemplate w =
            buildWorkholding(a, in.package.workholding.parts, in.modelFn());
        CHECK(w.validate().empty());
        const StockHolding& h = w.holdings.front();
        CHECK(h.kind == HoldingKind::Dovetail);
        CHECK_THAT(holdingOpening(w, h, {}), WithinAbs(kModelledWidth, 1e-6));
        SetupWorkholding placement;
        placement.templateId = w.id;
        placement.rotation = 90;
        const WorkholdingPose pose =
            poseWorkholding(w, placement, {Vec3(0, 0, -20), Vec3(60, 40, 0)});
        CHECK(near(pose.templateToWcs.inverse() * Vec3(30, 20, -20), Vec3(0, 0, kSeat - kDepth),
                   1e-6));
        // Both flanks fixed: no member, no range.
        WorkholdingAnswers fixedFlanks = a;
        fixedFlanks.moving.clear();
        const WorkholdingTemplate f =
            buildWorkholding(fixedFlanks, in.package.workholding.parts, in.modelFn());
        CHECK(f.members.empty());
        CHECK_FALSE(openingRange(f, f.holdings.front()));
    }
    SECTION("seat") {
        using namespace fixtures::plate;
        const Imported in = importParts("plate_assembly.step");
        WorkholdingAnswers a;
        a.type = FixtureType::Seat;
        a.top = in.face(0, plane(Vec3::UnitZ()));
        a.anchor = {0, 0, 0};
        const WorkholdingTemplate w =
            buildWorkholding(a, in.package.workholding.parts, in.modelFn());
        CHECK(w.validate().empty());
        SetupWorkholding placement;
        placement.templateId = w.id;
        const WorkholdingPose pose =
            poseWorkholding(w, placement, {Vec3(0, 0, -20), Vec3(60, 40, 0)});
        CHECK(near(pose.templateToWcs.inverse() * Vec3(0, 0, -20),
                   Vec3(kLength / 2, kWidth / 2, kThickness)));
    }
}

namespace {

/// The first round face of part `index` with this radius.
WorkholdingPick roundFace(const Imported& in, std::size_t index, double radius) {
    const WorkholdingPart& part = in.package.workholding.parts.at(index);
    const cad::Model& m = *in.modelOf(part);
    for (int f = 1; f <= cad::subShapeCount(m, cad::SubShapeType::Face); ++f) {
        const cad::SubShapeGeometry g = cad::subShapeGeometry(m, cad::SubShapeType::Face, f);
        if (g.kind == cad::SubShapeGeometry::Kind::Cylinder && std::abs(g.radius - radius) < 1e-6) {
            return {part.id, cad::SubShapeType::Face, f, std::nullopt};
        }
    }
    FAIL("no such round face");
    return {};
}

/// A bar of this diameter on WCS Z from Z -105 to 1 (the chuck project's stock, Ø32).
geom3d::Aabb3 bar(double diameter) {
    return {Vec3(-diameter / 2, -diameter / 2, -105), Vec3(diameter / 2, diameter / 2, 1)};
}

} // namespace

TEST_CASE("workholding parts removed and merged", "[document][workholding][wizard]") {
    using namespace fixtures::selfcentring;
    // Base, the first jaw in two bodies, the second jaw, a riser.
    Imported in = importParts("selfcentring_extras.step");
    WorkholdingPackage& pkg = in.package;
    REQUIRE(pkg.workholding.parts.size() == 5);
    const Uuid riser = in.id(4);
    pkg.workholding.datums.push_back(pickedDatum("On the riser", pkg.workholding.parts[4],
                                                 *in.modelOf(pkg.workholding.parts[4]),
                                                 cad::SubShapeType::Face, 1));
    removeWorkholdingPart(pkg, riser);
    CHECK(pkg.workholding.parts.size() == 4);
    CHECK(pkg.files.size() == 4);
    CHECK(pkg.workholding.datums.empty()); // picked on the riser
    CHECK_THROWS(removeWorkholdingPart(pkg, riser));

    // The first jaw's bodies as one part, in the first one's place, with both solids.
    pkg.workholding.parts[1].member = "Jaw";
    const Uuid a = in.id(1), b = in.id(2), second = in.id(3);
    CHECK_THROWS(mergeWorkholdingParts(pkg, {a}, "Jaw -Y", in.modelFn()));
    const WorkholdingPart merged = mergeWorkholdingParts(pkg, {a, b}, "Jaw -Y", in.modelFn());
    REQUIRE(pkg.workholding.parts.size() == 3);
    CHECK(pkg.workholding.parts[1].id == merged.id);
    CHECK(merged.name == "Jaw -Y");
    CHECK(merged.member == "Jaw");
    CHECK(pkg.files.size() == 3);
    CancelToken cancel;
    NullProgress progress;
    const cad::Model m =
        cad::Model::importStepData(pkg.files.at(merged.sha256), "merged", cancel, progress);
    CHECK(m.info().solids == 2);
    CHECK_THAT(m.info().bounds.max.z(), WithinAbs(kJawTop, 1e-6));
    CHECK_THAT(m.info().bounds.min.z(), WithinAbs(kBaseTop, 1e-6));

    // The wizard sets the merged vise up as the one-body jaws.
    in.models.emplace(merged.sha256, m);
    WorkholdingAnswers ans;
    ans.type = FixtureType::SelfCentringVise;
    ans.opposite = {merged.id};
    ans.moving = {second};
    ans.faceA = in.face(1, plane(Vec3::UnitY()));
    ans.faceB = in.face(2, plane(-Vec3::UnitY()));
    ans.top = in.face(2, plane(Vec3::UnitZ()));
    ans.minOpening = 20.0;
    ans.maxOpening = kMaxOpening;
    const WorkholdingTemplate w = buildWorkholding(ans, pkg.workholding.parts, in.modelFn());
    CHECK(w.validate().empty());
    CHECK_THAT(holdingOpening(w, w.holdings.front(), {}), WithinAbs(kModelledOpening, 1e-6));
}

TEST_CASE("the setup wizard's chuck grips a bar, its jaws together",
          "[document][workholding][wizard][chuck]") {
    using namespace fixtures::chuck;
    const Imported in = importParts("chuck_assembly.step");
    WorkholdingAnswers a;
    a.type = FixtureType::Chuck;
    CHECK(missingAnswers(a).size() == 4);
    a.moving = {in.id(1), in.id(2), in.id(3)};
    a.axis = roundFace(in, 0, kBore / 2);
    a.faceA = in.face(1, plane(-Vec3::UnitX())); // the first jaw's face towards the axis
    a.top = in.face(1, plane(Vec3::UnitZ()));
    a.minOpening = kMinDiameter;
    a.maxOpening = kMaxDiameter;
    a.gripDepth = kGripLength;
    REQUIRE(missingAnswers(a).empty());
    const WorkholdingTemplate w = buildWorkholding(a, in.package.workholding.parts, in.modelFn());
    CHECK(w.validate().empty());
    CHECK(w.up.isApprox(Vec3::UnitZ()));
    // Three jaws, each its own member moving radially, the second and third following the first.
    REQUIRE(w.members.size() == 3);
    CHECK(w.members[0].direction.isApprox(Vec3::UnitX(), 1e-9));
    for (std::size_t i = 1; i < 3; ++i) {
        CHECK(w.members[i].linkedTo == w.members[0].name);
        CHECK_THAT(w.members[i].direction.dot(Vec3::UnitX()), WithinAbs(-0.5, 1e-9));
        CHECK_THAT(w.members[i].direction.z(), WithinAbs(0.0, 1e-9));
    }
    CHECK(w.parts[0].member.empty());
    CHECK(w.parts[1].member == w.members[0].name);
    const StockHolding& h = w.holdings.front();
    CHECK(h.kind == HoldingKind::Chuck);
    CHECK_THAT(holdingOpening(w, h, {}), WithinAbs(2 * kGripRadius, 1e-6));
    const auto range = openingRange(w, h);
    REQUIRE(range);
    CHECK_THAT(range->first, WithinAbs(kMinDiameter, 1e-6));
    CHECK_THAT(range->second, WithinAbs(kMaxDiameter, 1e-6));
    for (const auto& [member, travel] : travelsForOpening(w, h, 32.0)) {
        CHECK_THAT(travel, WithinAbs(16.0 - kGripRadius, 1e-6));
    }

    // On a Ø32 bar: centred on WCS Z, the jaw front 15 mm up from the bar's end, the first jaw
    // closed on the bar's side towards +X.
    SetupWorkholding placement;
    placement.templateId = w.id;
    const WorkholdingPose pose = poseWorkholding(w, placement, bar(32.0));
    CHECK_THAT(pose.opening, WithinAbs(32.0, 1e-9));
    CHECK(near(pose.templateToWcs * Vec3(0, 0, kJawFront), Vec3(0, 0, -105 + kGripLength)));
    CHECK(near(partOn(pose, "Jaw 1").toWcs * Vec3(kGripRadius, 0, kJawFront),
               Vec3(16, 0, -105 + kGripLength)));
    // A typed grip length and opening.
    placement.gripDepth = 25.0;
    placement.jaws = JawSetting::Opening;
    placement.opening = 40.0;
    const WorkholdingPose open = poseWorkholding(w, placement, bar(32.0));
    CHECK(near(open.templateToWcs * Vec3(0, 0, kJawFront), Vec3(0, 0, -80)));
    CHECK_THAT(open.travel.at("Jaw 3"), WithinAbs(20.0 - kGripRadius, 1e-6));
    // Larger than it opens.
    placement.jaws = JawSetting::OnStock;
    CHECK_THROWS_WITH(poseWorkholding(w, placement, bar(140.0)),
                      StartsWith("the jaws can't reach \u00d8140.000 mm: they grip \u00d8"));

    // The gripping face must be on a jaw.
    WorkholdingAnswers onBody = a;
    onBody.faceA = in.face(0, plane(Vec3::UnitZ()));
    CHECK_THROWS(buildWorkholding(onBody, in.package.workholding.parts, in.modelFn()));
    // Every jaw: one jaw alone is not a chuck.
    WorkholdingAnswers oneJaw = a;
    oneJaw.moving = {in.id(1)};
    CHECK_THROWS_WITH(buildWorkholding(oneJaw, in.package.workholding.parts, in.modelFn()),
                      ContainsSubstring("every jaw"));
}

TEST_CASE("the setup wizard's collet grips bars a little smaller than its bore",
          "[document][workholding][wizard][chuck]") {
    using namespace fixtures::collet;
    const Imported in = importParts("collet_assembly.step");
    WorkholdingAnswers a;
    a.type = FixtureType::Collet;
    CHECK(missingAnswers(a).size() == 3);
    a.axis = roundFace(in, 1, kBore / 2);
    a.faceA = roundFace(in, 1, kBore / 2); // the bore: a point on it at its radius
    a.top = in.face(1, plane(Vec3::UnitZ()));
    a.gripRange = kGripRange;
    a.gripDepth = kGripLength;
    const WorkholdingTemplate w = buildWorkholding(a, in.package.workholding.parts, in.modelFn());
    CHECK(w.validate().empty());
    CHECK(w.members.empty());
    const StockHolding& h = w.holdings.front();
    CHECK_THAT(holdingOpening(w, h, {}), WithinAbs(kBore, 1e-6));
    CHECK_FALSE(openingRange(w, h));
    SetupWorkholding placement;
    placement.templateId = w.id;
    for (const double d : {20.0, 19.6}) {
        const WorkholdingPose pose = poseWorkholding(w, placement, bar(d));
        CHECK_THAT(pose.opening, WithinAbs(kBore, 1e-6));
        CHECK(near(pose.templateToWcs * Vec3(0, 0, kFront), Vec3(0, 0, -105 + kGripLength)));
    }
    CHECK_THROWS_WITH(poseWorkholding(w, placement, bar(19.4)),
                      StartsWith("the collet can't hold \u00d819.400 mm"));
    CHECK_THROWS_WITH(poseWorkholding(w, placement, bar(20.2)),
                      StartsWith("the collet can't hold \u00d820.200 mm"));
}

TEST_CASE("chuck templates: files and JSON", "[document][workholding][chuck]") {
    for (const char* file : {"chuck.ocamhold", "collet.ocamhold"}) {
        const WorkholdingPackage package = fixture(file);
        const WorkholdingTemplate& w = package.workholding;
        CHECK(w.validate().empty());
        const Json j = toJson(w);
        CHECK(j.at("version") == kWorkholdingVersion);
        CHECK(j.at("holdings").at(0).at("kind") == "chuck");
        CHECK(workholdingFromJson(j, file) == w);
    }
    // A holding without its axis is reported.
    WorkholdingTemplate w = fixture("chuck.ocamhold").workholding;
    w.holdings.front().axis.clear();
    CHECK_FALSE(w.validate().empty());
}

TEST_CASE("jaw mounts and jaw sets: JSON and validation", "[document][workholding][jaws]") {
    // A chuck that takes jaw sets: its own jaws stay, marked as top jaws.
    WorkholdingTemplate chuck = fixture("chuck.ocamhold").workholding;
    WorkholdingDatum seat;
    seat.name = "Jaw seat";
    seat.kind = DatumKind::Typed;
    seat.member = "Jaw 1";
    seat.point = Vec3(40.0, 0.0, 30.0);
    seat.direction = Vec3::UnitZ();
    chuck.datums.push_back(seat);
    chuck.jawMount = JawMount{"Test jaws", "Jaw seat"};
    chuck.parts[1].topJaw = true;
    CHECK(chuck.validate().empty());
    const Json cj = toJson(chuck);
    CHECK(cj.at("jaw_mount") == Json{{"interface", "Test jaws"}, {"seat", "Jaw seat"}});
    CHECK(cj.at("parts").at(1).at("top_jaw") == true);
    CHECK(workholdingFromJson(cj, "chuck") == chuck);
    // An earlier draft's mount fields are read past.
    Json draft = cj;
    draft["jaw_mount"]["pitch"] = 1.5;
    draft["jaw_mount"]["positions"] = Json::array({0, 12});
    CHECK(workholdingFromJson(draft, "draft") == chuck);

    // A jaw set: one jaw, no members or holdings.
    WorkholdingTemplate jaws;
    jaws.name = "Step jaws (test)";
    WorkholdingPart jaw = chuck.parts[1];
    jaw.member.clear();
    jaw.topJaw = false;
    jaws.parts = {jaw};
    const auto typed = [](const char* name, const Vec3& at, const Vec3& dir) {
        WorkholdingDatum d;
        d.name = name;
        d.kind = DatumKind::Typed;
        d.point = at;
        d.direction = dir;
        return d;
    };
    jaws.datums = {typed("Seat", Vec3::Zero(), Vec3::UnitZ()),
                   typed("Grip 1", Vec3(-10, 0, 10), -Vec3::UnitX()),
                   typed("Stop 1", Vec3(-12, 0, 5), Vec3::UnitZ()),
                   typed("Front", Vec3(0, 0, 30), Vec3::UnitZ())};
    // Two ways on, as a vise's holdings: the jaw as made, and turned round on its seat.
    jaws.jawSet =
        JawSet{"test  JAWS",
               JawSet::Kind::Hard,
               "Front",
               {{"OD", "Seat", -Vec3::UnitX(), {{"Step 1", "Grip 1", "Stop 1"}}},
                {"Reversed (ID)", "Seat", Vec3::UnitX(), {{"Step 1", "Grip 1", "Stop 1"}}}}};
    CHECK(jaws.validate().empty());
    const Json jj = toJson(jaws);
    CHECK(jj.at("holdings").empty());
    CHECK(jj.at("jaw_set").at("kind") == "hard");
    CHECK(jj.at("jaw_set").at("options").size() == 2);
    CHECK(jj.at("jaw_set").at("options").at(1).at("name") == "Reversed (ID)");
    CHECK_FALSE(jj.at("jaw_set").contains("seat"));
    CHECK(workholdingFromJson(jj, "jaws") == jaws);
    // An earlier draft's set, its one way on beside its interface: read as one option.
    Json one = jj;
    const Json first = one["jaw_set"]["options"][0];
    one["jaw_set"].erase("options");
    for (const char* key : {"seat", "inward", "steps"}) {
        one["jaw_set"][key] = first.at(key);
    }
    const WorkholdingTemplate oneWay = workholdingFromJson(one, "draft");
    REQUIRE(oneWay.jawSet);
    REQUIRE(oneWay.jawSet->options.size() == 1);
    CHECK(oneWay.jawSet->options.front().name == "Normal");
    CHECK(oneWay.jawSet->options.front().steps == jaws.jawSet->options.front().steps);
    CHECK(oneWay.jawSet->findOption("") == &oneWay.jawSet->options.front());
    CHECK(jaws.jawSet->findOption("Reversed (ID)") == &jaws.jawSet->options[1]);
    CHECK_FALSE(jaws.jawSet->findOption("Upside down"));

    // What is wrong is said.
    WorkholdingTemplate bad = jaws;
    bad.jawSet->kind = JawSet::Kind::Soft;
    bad.jawSet->front.clear();
    bad.jawSet->options[0].steps.push_back({"Step 1", "Grip 1", "Nowhere"});
    bad.jawSet->options[1].name = "OD";
    std::string all;
    for (const std::string& issue : bad.validate()) {
        all += issue + "; ";
    }
    CHECK_THAT(all, ContainsSubstring("bored back from it"));
    CHECK_THAT(all, ContainsSubstring("steps need different"));
    CHECK_THAT(all, ContainsSubstring("its faces must be datums"));
    CHECK_THAT(all, ContainsSubstring("options need different"));
    WorkholdingTemplate noSeat = chuck;
    noSeat.jawMount->seat = "Nowhere";
    CHECK_FALSE(noSeat.validate().empty());

    // Version 3 had neither: a chuck keeps its own jaws.
    Json v3 = toJson(fixture("chuck.ocamhold").workholding);
    v3["version"] = 3;
    const WorkholdingTemplate old = workholdingFromJson(v3, "v3");
    CHECK_FALSE(old.jawMount);
    CHECK(std::none_of(old.parts.begin(), old.parts.end(),
                       [](const WorkholdingPart& p) { return p.topJaw; }));

    // The setup's choices, in the project.
    Project project;
    project.workholdings = {chuck, jaws};
    Setup setup;
    setup.name = "Second op";
    SetupWorkholding sw;
    sw.templateId = chuck.id;
    sw.jawSet = jaws.id;
    sw.jawOption = "Reversed (ID)";
    sw.grip = GripSide::Inside;
    sw.step = "Step 1";
    sw.stopOffset = 4.5;
    sw.softBore = SoftBore{std::nullopt, 0.05, 12.0, 30.0};
    setup.workholding = sw;
    project.setups = {setup};
    const Project back = projectFromJson(toJson(project));
    REQUIRE(back.setups.front().workholding);
    CHECK(*back.setups.front().workholding == sw);
    // An earlier draft's setup fields are read past.
    Json drafted = toJson(project);
    drafted["setups"][0]["workholding"]["jaw_reversed"] = true;
    drafted["setups"][0]["workholding"]["serration"] = 7;
    CHECK(*projectFromJson(drafted).setups.front().workholding == sw);
    // An option the set does not have is said.
    Project wrong = project;
    wrong.setups.front().workholding->jawOption = "Upside down";
    std::string wrongIssues;
    for (const std::string& issue : wrong.validate()) {
        wrongIssues += issue + "; ";
    }
    CHECK_THAT(wrongIssues, ContainsSubstring("has no option 'Upside down'"));
    // A jaw set no setup uses goes; one that a setup uses stays.
    Project used = back;
    removeUnusedWorkholdings(used);
    CHECK(used.workholdings.size() == 2);
}

namespace {

/// The test chuck's project, its chuck taking jaw sets: a seat on master jaw 1's top at radius 50,
/// 1.5 mm pitch, positions 0 to 20; and a jaw set made of one of its jaws (a box X 25..70,
/// Y -10..10, Z 60..100): seated at the middle of its bottom, in towards -X, gripping on its
/// inner face (X 25, so 22.5 in from the seat) and stopping the work 10 above the seat.
struct JawSetup {
    JawSetup() {
        doc = Document::open(kFixtures / "chuck_project.ocam");
        setupId = doc.project().setups.front().id;
        const auto typed = [](const char* name, const Vec3& at, const Vec3& dir,
                              const char* member = "") {
            WorkholdingDatum d;
            d.name = name;
            d.kind = DatumKind::Typed;
            d.member = member;
            d.point = at;
            d.direction = dir;
            return d;
        };
        doc.modify("Jaws", [&](Project& p) {
            WorkholdingTemplate& chuck = p.workholdings.front();
            chuck.datums.push_back(typed("Jaw seat", Vec3(50, 0, 100), Vec3::UnitZ(), "Jaw 1"));
            chuck.jawMount = JawMount{"Test jaws", "Jaw seat"};
            for (WorkholdingPart& part : chuck.parts) {
                part.topJaw = !part.member.empty();
            }
            WorkholdingPart jaw = chuck.parts[1];
            jaw.member.clear();
            jaw.topJaw = false;
            WorkholdingTemplate hard;
            hard.name = "Step jaws (test)";
            hard.parts = {jaw};
            hard.datums = {typed("Seat", Vec3(47.5, 0, 60), Vec3::UnitZ()),
                           typed("Grip", Vec3(25, 0, 80), -Vec3::UnitX()),
                           typed("Stop", Vec3(30, 0, 70), Vec3::UnitZ()),
                           typed("Front", Vec3(47.5, 0, 100), Vec3::UnitZ())};
            // The jaw as made grips outside; turned round on its seat, its step grips inside.
            hard.jawSet =
                JawSet{"test  JAWS",
                       JawSet::Kind::Hard,
                       "Front",
                       {{"OD", "Seat", -Vec3::UnitX(), {{"Inner", "Grip", "Stop"}}},
                        {"Reversed (ID)", "Seat", Vec3::UnitX(), {{"Inner", "Grip", "Stop"}}}}};
            WorkholdingTemplate soft = hard;
            soft.id = Uuid::generate();
            soft.name = "Soft jaws (test)";
            soft.jawSet->kind = JawSet::Kind::Soft;
            soft.jawSet->options = {
                {"Normal", "Seat", -Vec3::UnitX(), {{"Clamp", "Grip", "Stop"}}}};
            hardId = hard.id;
            softId = soft.id;
            p.workholdings.push_back(hard);
            p.workholdings.push_back(soft);
        });
    }
    void choose(const std::function<void(SetupWorkholding&)>& change) {
        doc.modify("Choose", [&](Project& p) { change(*p.findSetup(setupId)->workholding); });
    }
    const Setup& setup() const { return *doc.project().findSetup(setupId); }

    Document doc;
    Uuid setupId;
    Uuid hardId;
    Uuid softId;
};

} // namespace

TEST_CASE("jaw sets on a chuck's master jaws", "[document][workholding][jaws]") {
    JawSetup f;
    REQUIRE(f.doc.project().validate().empty());
    CHECK(jawSetFits(f.doc.project().workholdings.front(),
                     *f.doc.project().findWorkholding(f.hardId)));
    const auto pose = [&] {
        const auto p = setupWorkholdingPose(f.doc, f.setup());
        REQUIRE(p);
        return *p;
    };
    SECTION("hard step jaws grip the stock, seated as modelled") {
        f.choose([&](SetupWorkholding& w) { w.jawSet = f.hardId; });
        const WorkholdingPose p = pose();
        REQUIRE(p.jaws);
        const JawPlacement& j = *p.jaws;
        CHECK(j.option == "OD"); // not chosen: the first
        CHECK(j.step == "Inner");
        CHECK(j.outside);
        CHECK(j.gripDiameter == Approx(p.stockWidth));
        // The grip face's radius: 50 + travel - 22.5 = the stock's radius.
        const double r = 0.5 * p.stockWidth;
        CHECK(50.0 + j.travel - 22.5 == Approx(r));
        CHECK(p.warnings.empty());
        // The chuck's own jaws are out, the set's three are in, one on each master jaw, each
        // with an id of its own (views keep their models by id).
        int jaws = 0;
        std::set<Uuid> ids;
        for (const PosedPart& part : p.parts) {
            if (part.name.rfind("Jaw ", 0) == 0 && part.name.find(": ") != std::string::npos) {
                ++jaws;
                ids.insert(part.id);
                // Its grip face where the stock's surface is.
                const Vec3 grip = part.toWcs * Vec3(25, 0, 80);
                CHECK(grip.head<2>().norm() == Approx(0.5 * p.stockWidth).margin(1e-6));
            } else {
                CHECK(part.member.empty());
            }
        }
        CHECK(jaws == 3);
        CHECK(ids.size() == 3);
        // The stock's held end on the stop face.
        const PosedPart& first =
            *std::find_if(p.parts.begin(), p.parts.end(),
                          [](const PosedPart& x) { return x.name.rfind("Jaw 1:", 0) == 0; });
        CHECK((first.toWcs * Vec3(30, 0, 70)).z() == Approx(j.stopZ));

        // Stock the master jaws cannot reach is refused, as by any workholding, saying what the
        // step grips; shown, the jaws stand as far as they go, in error.
        f.choose([&](SetupWorkholding& w) {
            w.jaws = JawSetting::Opening;
            w.opening = 200.0;
        });
        CHECK_THROWS_WITH(pose(),
                          StartsWith("the jaws can't reach \u00d8200.000 mm: on step 'Inner' they "
                                     "grip \u00d8"));
        const auto shown = shownWorkholdingPose(f.doc, f.setup());
        REQUIRE(shown);
        REQUIRE(shown->outOfReach);
        CHECK(shown->outOfReach->step == "Inner");
        // Over master jaw 1's travel, -23 to 40: radius 27.5 + travel.
        CHECK(shown->outOfReach->from == Approx(2.0 * (27.5 - 23.0)));
        CHECK(shown->outOfReach->to == Approx(2.0 * (27.5 + 40.0)));
        REQUIRE(shown->jaws);
        CHECK(shown->jaws->travel == Approx(40.0));
        int marked = 0;
        for (const PosedPart& part : shown->parts) {
            if (part.name.find(": ") != std::string::npos) {
                marked += part.outOfReach ? 1 : 0;
            }
        }
        CHECK(marked == 3);
        CHECK_THAT(reachMessage(*shown->outOfReach, LengthUnit::Inch),
                   Equals("the jaws can't reach \u00d87.874 in: on step 'Inner' they grip "
                          "\u00d80.354\u20135.315 in"));
    }
    SECTION("the reversed option's step grips an inside diameter") {
        f.choose([&](SetupWorkholding& w) {
            w.jawSet = f.hardId;
            w.jawOption = "Reversed (ID)";
        });
        CHECK_THROWS_WITH(pose(), ContainsSubstring("grips an outside diameter"));
        f.choose([&](SetupWorkholding& w) {
            w.grip = GripSide::Inside;
            w.jaws = JawSetting::Opening;
            w.opening = 120.0;
        });
        const WorkholdingPose p = pose();
        REQUIRE(p.jaws);
        CHECK_FALSE(p.jaws->outside);
        CHECK(p.jaws->option == "Reversed (ID)");
        // Turned round, the grip face stands 22.5 out from the seat.
        CHECK(50.0 + p.jaws->travel + 22.5 == Approx(60.0));
        // A bore the master jaws cannot reach is refused, by their range.
        f.choose([&](SetupWorkholding& w) { w.opening = 10.0; });
        CHECK_THROWS_WITH(pose(), StartsWith("the jaws can't reach \u00d810.000 mm"));
        // An option the set does not have is said.
        f.choose([&](SetupWorkholding& w) { w.jawOption = "Upside down"; });
        CHECK_THROWS_WITH(pose(), ContainsSubstring("has no option 'Upside down'"));
    }
    SECTION("an inside grip on stock with no bore says so") {
        f.choose([&](SetupWorkholding& w) {
            w.jawSet = f.hardId;
            w.jawOption = "Reversed (ID)";
            w.grip = GripSide::Inside;
        });
        CHECK_THROWS_WITH(pose(), ContainsSubstring("needs the bore it grips"));
    }
    SECTION("soft jaws bored to fit, and the operation that bores them") {
        f.choose([&](SetupWorkholding& w) {
            w.jawSet = f.softId;
            w.softBore = SoftBore{std::nullopt, 0.1, 12.0, 30.0};
        });
        const WorkholdingPose p = pose();
        REQUIRE(p.jaws);
        const JawPlacement& j = *p.jaws;
        REQUIRE(j.boreDiameter);
        CHECK(*j.boreDiameter == Approx(p.stockWidth + 0.1));
        // Bored clamping 30 on the step, then closed by half the clearance onto the stock.
        CHECK(50.0 + j.boringTravel - 22.5 == Approx(15.0));
        CHECK(j.travel == Approx(j.boringTravel - 0.05));
        // Each jaw is cut: less of it, and where the bore is, nothing.
        int cut = 0;
        for (const PosedPart& part : p.parts) {
            if (!part.cut) {
                continue;
            }
            ++cut;
            const cad::Model* whole = f.doc.modelData(part.sha256);
            const cad::Model* bored = posedPartModel(f.doc, part);
            REQUIRE(whole);
            REQUIRE(bored);
            const double taken = whole->info().volume - bored->info().volume;
            // The bore takes the segment of its circle beyond the flat face at 15, 12 deep.
            const double r = 0.5 * *j.boreDiameter;
            const double segment = r * r * std::acos(15.0 / r) - 15.0 * std::sqrt(r * r - 225.0);
            CHECK(taken == Approx(12.0 * segment).epsilon(0.03));
            CHECK(posedPartModel(f.doc, part) == bored); // made once
        }
        CHECK(cut == 3);
        // The bore's floor is where the work stops.
        const SoftJawBoring b = softJawBoring(f.doc, f.setup());
        CHECK(b.bore);
        CHECK(b.diameter == Approx(*j.boreDiameter));
        geom2d::Aabb2 before;
        for (const auto& r : b.before) {
            before.extend(r.bounds());
        }
        CHECK(before.min.y() == Approx(15.0).margin(0.6));
        CHECK(geom2d::totalArea(b.after) < geom2d::totalArea(b.before));
        // Generated: a turning operation on the jaws, in a bore, as deep as the bore.
        CancelToken cancel;
        NullProgress progress;
        Operation op;
        op.name = "Bore soft jaws";
        op.type = "turn_soft_jaws";
        op.parameters = Json::object();
        const Operation& turn = *std::find_if(
            f.setup().operations.begin(), f.setup().operations.end(),
            [](const Operation& o) { return o.type == "turn_finish" || o.type == "turn_rough"; });
        op.toolId = turn.toolId;
        op.presetId = turn.presetId;
        tooling::Tool boring = *f.doc.project().tools.findTool(*turn.toolId);
        f.doc.modify("Op", [&](Project& pr) {
            pr.findSetup(f.setupId)->operations.insert(pr.findSetup(f.setupId)->operations.begin(),
                                                       op);
        });
        const toolpath::Toolpath tp = generateToolpath(f.doc, f.setup(), op, cancel, progress);
        CHECK_FALSE(tp.moves.empty());
        CHECK(std::any_of(tp.warnings.begin(), tp.warnings.end(), [](const std::string& w) {
            return w.find("bores the soft jaws") != std::string::npos;
        }));
        // Down to the floor where there is jaw (outside its face at 15), and no further; past
        // the floor's inner edge the nose goes round it, in the air.
        // The programmed point is the imaginary tip, the nose's lowest point here; its centre is a
        // nose radius in from the tip's radius.
        const double rn = boring.insert.noseRadius;
        double deepest = 1e9;
        for (const toolpath::Move& m : tp.moves) {
            if (m.end.x() - rn > 15.0 + 1e-3) {
                deepest = std::min(deepest, m.end.z());
            }
        }
        CHECK(deepest == Approx(j.stopZ).margin(1e-3));
        // The simulation leaves it out of the work and the machine, and says so.
        SimulationOptions options;
        options.compare = false;
        const SetupSimulation sim = simulateSetup(f.doc, f.setupId, options, cancel, progress);
        CHECK(std::any_of(sim.warnings.begin(), sim.warnings.end(), [](const std::string& w) {
            return w.find("bores the soft jaws") != std::string::npos;
        }));
        CHECK(sim.operations.front().removedVolume == 0.0);
    }
    SECTION("no jaw set: the chuck's own jaws, as before") {
        const WorkholdingPose p = pose();
        CHECK_FALSE(p.jaws);
        CHECK(std::count_if(p.parts.begin(), p.parts.end(),
                            [](const PosedPart& x) { return !x.member.empty(); }) == 3);
    }
}

TEST_CASE("a setup's workholding set up apart and taken back", "[document][workholding][jaws]") {
    // The workholding dialog edits a snapshot; OK takes the setup's workholding from it.
    JawSetup f;
    const Uuid chuckId = f.setup().workholding->templateId;
    REQUIRE(f.setup().workholding->jawSet != f.hardId);
    Document apart = f.doc.snapshot();
    apart.modify("Choose", [&](Project& p) {
        SetupWorkholding& w = *p.findSetup(f.setupId)->workholding;
        w.jawSet = f.hardId;
        w.stopOffset = 3.0;
    });
    // A copy updated apart (from a library) comes too, with what other setups choose kept.
    auto package = apart.workholdingPackage(chuckId);
    REQUIRE(package);
    package->workholding.name = "Chuck (updated)";
    apart.updateWorkholding(*package, {});
    CHECK(apart.project().findWorkholding(chuckId)->name == "Chuck (updated)");
    WorkholdingPackage stranger = *package;
    stranger.workholding.id = Uuid::generate();
    CHECK_THROWS_WITH(apart.updateWorkholding(stranger, {}),
                      ContainsSubstring("not in the project"));
    // The document it came from is as it was until then.
    CHECK(f.doc.project().findWorkholding(chuckId)->name != "Chuck (updated)");

    f.doc.takeWorkholding(apart, f.setupId);
    CHECK(f.setup().workholding == apart.project().findSetup(f.setupId)->workholding);
    CHECK(f.setup().workholding->jawSet == f.hardId);
    CHECK(f.doc.project().findWorkholding(chuckId)->name == "Chuck (updated)");
    // Sets no setup uses are dropped; one step undoes it all.
    CHECK_FALSE(f.doc.project().findWorkholding(f.softId));
    CHECK(f.doc.undoLabel() == "Set up workholding");
    REQUIRE(shownWorkholdingPose(f.doc, f.setup()));
    f.doc.undo();
    CHECK(f.setup().workholding->jawSet != f.hardId);
    CHECK(f.doc.project().findWorkholding(chuckId)->name != "Chuck (updated)");
}

TEST_CASE("step jaws turned either way on the master jaws", "[document][workholding][jaws]") {
    // A step jaw laid out as a user's was (placeholder shape, typed datums): 99.5 long from its
    // inner end (X 0) to its back (X 99.5), seated at its middle (X 49.75) on its bottom, its steps
    // rising outwards: step 1's face at X 0 (its work's end on the body face below the seat), step
    // 2's at X 43 (on step 1's tread, Z 28), step 3's at X 60 (on step 2's, Z 41), facing -X. As
    // modelled its steps face the axis (OD); turned round they face out (ID), and its back faces
    // the axis: the way it holds a bar across its full height.
    JawSetup f;
    const auto typed = [](const char* name, const Vec3& at, const Vec3& dir) {
        WorkholdingDatum d;
        d.name = name;
        d.kind = DatumKind::Typed;
        d.point = at;
        d.direction = dir;
        return d;
    };
    const std::vector<JawStep> steps{{"Step 1", "Step 1 face", "Body face"},
                                     {"Step 2", "Step 2 face", "Step 1 tread"},
                                     {"Step 3", "Step 3 face", "Step 2 tread"}};
    Uuid stepped;
    f.doc.modify("Step jaws", [&](Project& p) {
        WorkholdingTemplate jaws = *p.findWorkholding(f.hardId);
        jaws.id = Uuid::generate();
        jaws.name = "Turnable step jaws (test)";
        jaws.datums = {typed("Seat", Vec3(49.75, 0, 0), Vec3::UnitZ()),
                       typed("Step 1 face", Vec3(0, 0, 14), -Vec3::UnitX()),
                       typed("Body face", Vec3(-10, 0, -1), Vec3::UnitZ()),
                       typed("Step 2 face", Vec3(43, 0, 34.5), -Vec3::UnitX()),
                       typed("Step 1 tread", Vec3(21.5, 0, 28), Vec3::UnitZ()),
                       typed("Step 3 face", Vec3(60, 0, 47.5), -Vec3::UnitX()),
                       typed("Step 2 tread", Vec3(51.5, 0, 41), Vec3::UnitZ()),
                       typed("Back face", Vec3(99.5, 0, 30), Vec3::UnitX()),
                       typed("Front", Vec3(80, 0, 54), Vec3::UnitZ())};
        std::vector<JawStep> reversed = steps;
        reversed.insert(reversed.begin(), {"Back", "Back face", "Body face"});
        jaws.jawSet->options = {{"OD", "Seat", -Vec3::UnitX(), steps},
                                {"Reversed", "Seat", Vec3::UnitX(), reversed}};
        stepped = jaws.id;
        p.workholdings.push_back(jaws);
    });
    REQUIRE(f.doc.project().validate().empty());
    const auto pose = [&] {
        const auto p = shownWorkholdingPose(f.doc, f.setup());
        REQUIRE(p);
        return *p;
    };
    // The test chuck's seat is 50 out from the axis at master jaw travel 0 (travel -23 to 40).
    const auto radiusOf = [&](const WorkholdingPose& p, const char* datum) {
        const PosedPart& jaw1 =
            *std::find_if(p.parts.begin(), p.parts.end(),
                          [](const PosedPart& x) { return x.name.rfind("Jaw 1:", 0) == 0; });
        const Vec3 at =
            jaw1.toWcs * datumAt(*f.doc.project().findWorkholding(stepped), datum, {}).point;
        return std::pair{at.head<2>().norm(), at.z()};
    };

    SECTION("as modelled, step 1 grips the bar, its end on the stop") {
        f.choose([&](SetupWorkholding& w) {
            w.jawSet = stepped;
            w.jawOption = "OD";
        });
        const WorkholdingPose p = pose();
        REQUIRE(p.jaws);
        CHECK_FALSE(p.outOfReach);
        CHECK(p.refusal.empty());
        CHECK(p.jaws->step == "Step 1"); // Auto: the first that reaches
        const double r = 0.5 * p.stockWidth;
        CHECK(radiusOf(p, "Step 1 face").first == Approx(r));
        CHECK(radiusOf(p, "Body face").second == Approx(p.jaws->stopZ));
        // Step 2 cannot close to it: said, drawn as far as it goes.
        f.choose([&](SetupWorkholding& w) { w.step = "Step 2"; });
        const WorkholdingPose two = pose();
        REQUIRE(two.outOfReach);
        CHECK(two.outOfReach->step == "Step 2");
        CHECK(two.outOfReach->from == Approx(2.0 * (50.0 - 49.75 + 43.0 - 23.0)));
        CHECK_THROWS_WITH(setupWorkholdingPose(f.doc, f.setup()),
                          StartsWith("the jaws can't reach \u00d8"));
    }
    SECTION("the stock stood out from the stop") {
        f.choose([&](SetupWorkholding& w) {
            w.jawSet = stepped;
            w.jawOption = "OD";
        });
        const double against = pose().jaws->stopZ;
        f.choose([&](SetupWorkholding& w) { w.stopOffset = 12.0; });
        const WorkholdingPose p = pose();
        REQUIRE(p.jaws);
        CHECK_FALSE(p.outOfReach);
        CHECK(p.jaws->step == "Step 1");
        // The jaws 12 further down the stock, still gripping it, its end 12 out from the stop.
        CHECK(p.jaws->stopZ == Approx(against - 12.0));
        CHECK(radiusOf(p, "Body face").second == Approx(p.jaws->stopZ));
        CHECK(radiusOf(p, "Step 1 face").first == Approx(0.5 * p.stockWidth));
        // Never in past the stop.
        f.choose([&](SetupWorkholding& w) { w.stopOffset = -1.0; });
        std::string issues;
        for (const std::string& issue : f.doc.project().validate()) {
            issues += issue;
        }
        CHECK_THAT(issues, ContainsSubstring("cannot go in past the jaws' stop"));
        CHECK_THROWS_WITH(setupWorkholdingPose(f.doc, f.setup()),
                          ContainsSubstring("cannot go in past the jaws' stop"));
    }
    SECTION("Auto takes the step that reaches") {
        // A bar step 1 cannot reach (its face goes out to 40.25 at most), step 2 can.
        f.choose([&](SetupWorkholding& w) {
            w.jawSet = stepped;
            w.jawOption = "OD";
            w.jaws = JawSetting::Opening;
            w.opening = 120.0;
        });
        const WorkholdingPose p = pose();
        REQUIRE(p.jaws);
        CHECK_FALSE(p.outOfReach);
        CHECK(p.jaws->step == "Step 2");
        CHECK(radiusOf(p, "Step 2 face").first == Approx(60.0));
        CHECK(radiusOf(p, "Step 1 tread").second == Approx(p.jaws->stopZ));
    }
    SECTION("turned round, its back grips the bar, its steps a bore") {
        f.choose([&](SetupWorkholding& w) {
            w.jawSet = stepped;
            w.jawOption = "Reversed";
        });
        const WorkholdingPose p = pose();
        REQUIRE(p.jaws);
        CHECK_FALSE(p.outOfReach);
        CHECK(p.jaws->step == "Back");
        CHECK(radiusOf(p, "Back face").first == Approx(0.5 * p.stockWidth));
        // Turned round, the steps face out: step 1's at 50 + 49.75 + travel, a bore.
        f.choose([&](SetupWorkholding& w) {
            w.grip = GripSide::Inside;
            w.jaws = JawSetting::Opening;
            w.opening = 220.0;
        });
        const WorkholdingPose bore = pose();
        REQUIRE(bore.jaws);
        CHECK_FALSE(bore.outOfReach);
        CHECK(bore.jaws->step == "Step 1");
        CHECK(radiusOf(bore, "Step 1 face").first == Approx(110.0));
    }
    SECTION("an entry with no step that grips that way: drawn where it stands, and said") {
        f.doc.modify("No back", [&](Project& p) {
            for (WorkholdingTemplate& t : p.workholdings) {
                if (t.id == stepped) {
                    t.jawSet->options[1].steps.erase(t.jawSet->options[1].steps.begin());
                }
            }
        });
        f.choose([&](SetupWorkholding& w) {
            w.jawSet = stepped;
            w.jawOption = "Reversed";
        });
        CHECK_THROWS_WITH(setupWorkholdingPose(f.doc, f.setup()),
                          ContainsSubstring("has no step that grips an outside diameter (OD)"));
        const WorkholdingPose p = pose();
        CHECK_THAT(p.refusal, ContainsSubstring("Turnable step jaws (test) (Reversed)"));
        REQUIRE(p.jaws);
        int drawn = 0;
        for (const PosedPart& part : p.parts) {
            if (part.name.find(": ") != std::string::npos) {
                CHECK(part.outOfReach);
                ++drawn;
            }
        }
        CHECK(drawn == 3);
    }
}

TEST_CASE("turning keeps clear of the chuck", "[document][workholding][chuck]") {
    using namespace fixtures::chuck;
    Document doc = Document::open(kFixtures / "chuck_project.ocam");
    const Setup& setup = doc.project().setups.front();
    REQUIRE(setup.workholding);
    CancelToken cancel;
    NullProgress progress;
    // The jaw front 10 mm behind the part's back end (Z -80): every operation generates, and
    // none comes nearer the jaws than the clearance.
    const double jawFront = -105 + kGripLength;
    const auto bodies = latheBodies(doc, setup, 0.0);
    REQUIRE(bodies.size() == 4);
    for (const Operation& op : setup.operations) {
        const toolpath::Toolpath tp = generateToolpath(doc, setup, op, cancel, progress);
        if (op.type == "turn_rough" || op.type == "turn_finish") {
            for (const toolpath::Move& m : tp.moves) {
                CHECK(m.end.z() > jawFront + setup.fixtureClearance - 1e-6);
            }
        }
        const tooling::Tool& tool = *doc.project().tools.findTool(*op.toolId);
        const auto side =
            tp.tipQuadrant == 2 ? tooling::TurningSide::Inside : tooling::TurningSide::Outside;
        CHECK(sim::checkTurningFixtures(tp, tooling::turningShape(tool, side).outline, bodies)
                  .empty());
    }
    SimulationOptions options;
    options.compare = false;
    const SetupSimulation sim = simulateSetup(doc, setup.id, options, cancel, progress);
    CHECK_FALSE(sim.hasErrors());
    CHECK(sim.fixtureCollisionCount() == 0);

    // Gripped 27 mm deep the jaws cover the part's back end: roughing and finishing stop in front
    // of them (and say so), parting off there is refused, naming the jaw.
    doc.modify("Grip it deeper",
               [](Project& p) { p.setups.front().workholding->gripDepth = 27.0; });
    const Setup& deep = doc.project().setups.front();
    for (const Operation& op : deep.operations) {
        if (op.type == "turn_rough" || op.type == "turn_finish") {
            const toolpath::Toolpath tp = generateToolpath(doc, deep, op, cancel, progress);
            for (const toolpath::Move& m : tp.moves) {
                CHECK(m.end.z() > -78.0 + deep.fixtureClearance - 1e-6);
            }
            CHECK(std::any_of(tp.warnings.begin(), tp.warnings.end(), [](const std::string& s) {
                return s.find("not turned") != std::string::npos;
            }));
        } else if (op.type == "turn_partoff") {
            CHECK_THROWS_WITH(generateToolpath(doc, deep, op, cancel, progress),
                              ContainsSubstring("comes within") && ContainsSubstring("Jaw"));
        }
    }
}
