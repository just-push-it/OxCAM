// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Back boring and 3D edge breaking on real models (DL-0083): selection, generation and
// simulation.

#include "FixtureSpecs.hpp"
#include "TestProject.hpp"

#include <ocam/cad/EdgeFaces.hpp>
#include <ocam/cad/Features.hpp>
#include <ocam/cad/SubShape.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/foundation/Error.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <filesystem>

using namespace ocam;
using namespace ocam::document;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

struct Fixture {
    Document doc;
    Uuid modelId;
    Uuid setupId;
    CancelToken cancel;
    NullProgress progress;

    explicit Fixture(const char* step) {
        modelId = doc.importModel(std::filesystem::path(OCAM_FIXTURE_DIR) / step, cancel, progress);
        setupId = doc.addSetup(modelId);
    }

    const cad::Model& model() const { return *doc.model(modelId); }

    /// Adds a tool (with its default preset) and returns it.
    tooling::Tool addTool(tooling::Tool tool) {
        tool = testing::withPreset(std::move(tool));
        doc.modify("tool", [&](Project& p) { p.tools.tools.push_back(tool); });
        return tool;
    }

    Uuid addOperation(const std::string& type, const tooling::Tool& tool,
                      std::vector<ShapeRef> geometry, Json params = Json::object()) {
        Operation op;
        op.name = type;
        op.type = type;
        op.toolId = tool.id;
        op.presetId = tool.presets.front().id;
        op.geometry = std::move(geometry);
        op.parameters = std::move(params);
        doc.modify("op", [&](Project& p) { p.findSetup(setupId)->operations.push_back(op); });
        return op.id;
    }

    toolpath::Toolpath generate(const Uuid& opId) {
        const Setup* owner = nullptr;
        const Operation* op = doc.project().findOperation(opId, &owner);
        return generateToolpath(doc, *owner, *op, cancel, progress);
    }

    ShapeRef ref(cad::SubShapeType type, int index) const {
        return {modelId, type, index, cad::signatureOf(model(), type, index)};
    }
};

/// A back-boring bar: a Ø13 head 3 mm tall on a Ø5 neck 40 mm long.
tooling::Tool backBar() {
    tooling::Tool t = tooling::makeDefaultTool(tooling::ToolType::BoringBar, 7, 13.0);
    t.mill.diameter = 13.0;
    t.mill.fluteLength = 3.0;
    t.mill.neckDiameter = 5.0;
    t.mill.neckLength = 40.0;
    t.mill.overallLength = 80.0;
    t.mill.shankDiameter = 0.0;
    t.mill.flutes = 1;
    return t;
}

} // namespace

TEST_CASE("back boring the far side of a through hole", "[document][backbore]") {
    using namespace fixtures::hole_plate;
    Fixture f("hole_plate.step");
    const tooling::Tool drill = f.addTool(tooling::makeFlatEndMill(1, 10.0, 25.0, 60.0));
    const tooling::Tool bar = f.addTool(backBar());
    // The Ø10 through hole.
    std::vector<ShapeRef> hole;
    for (const cad::Hole& h : cad::findHoles(f.model())) {
        if (h.bottom == cad::HoleBottom::Through &&
            std::abs(h.diameter - kThroughDiameter) < 1e-6) {
            hole.push_back(f.ref(cad::SubShapeType::Face, h.faces.front()));
        }
    }
    REQUIRE(hole.size() == 1);
    // The stock has no hole yet: plunge it first.
    f.addOperation("drill", drill, hole);
    const Uuid op = f.addOperation("backbore", bar, hole, Json{{"depth", 3.0}, {"dwell", 0.2}});
    const toolpath::Toolpath tp = f.generate(op);
    CHECK(tp.warnings.empty());
    REQUIRE(tp.cycles.size() == 1);
    const toolpath::DrillCycle& c = tp.cycles.front();
    CHECK(c.type == toolpath::CycleType::BackBore);
    // The WCS sits 1 mm above the plate: the far side at -21, the counterbore up to -18.
    CHECK_THAT(c.topZ + 3.0, WithinAbs(-1.0 - kThickness + 3.0, 1e-6));
    CHECK_THAT(c.shift.x(), WithinAbs(-2.0, 1e-12)); // half the head's overhang, along -X
    // The R plane: the hole operations' feed height, 2 mm above the hole's top.
    CHECK_THAT(c.retractZ, WithinAbs(-1.0 + 2.0, 1e-6));

    SECTION("the simulation cuts the counterbore and nothing else") {
        SimulationOptions options;
        options.resolution = 0.1;
        options.compare = false;
        const SetupSimulation sim = simulateSetup(f.doc, f.setupId, options, f.cancel, f.progress);
        REQUIRE(sim.operations.size() == 2);
        const OperationSimulation& back = sim.operations[1];
        CHECK(back.error.empty());
        CHECK(back.collisions.empty());
        const double ring = kPi * (6.5 * 6.5 - 5.0 * 5.0) * 3.0;
        CHECK_THAT(back.removedVolume, WithinAbs(ring, 0.1 * ring));
    }
    SECTION("a bar that cannot pass is refused") {
        const Uuid tight = f.addOperation("backbore", bar, hole, Json{{"shift", 0.5}});
        CHECK_THROWS_WITH(f.generate(tight), ContainsSubstring("cannot pass through the hole"));
    }
}

TEST_CASE("3D edge breaking on a sloped block with a hole", "[document][edgebreak]") {
    using namespace fixtures::wedge_hole;
    Fixture f("wedge_hole.step");
    const tooling::Tool mill =
        f.addTool(tooling::makeChamferMill(1, 8.0, 0.0, degToRad(90.0), 50.0));
    const cad::Model& model = f.model();
    std::vector<ShapeRef> sloped;   // the front edge of the slope
    std::vector<ShapeRef> rim;      // the hole's edge in the slope
    std::vector<ShapeRef> vertical; // the front left corner
    const Vec3 slope = Vec3(kLength, 0.0, kLow - kHigh).normalized();
    for (int e = 1; e <= cad::subShapeCount(model, cad::SubShapeType::Edge); ++e) {
        const auto g = cad::subShapeGeometry(model, cad::SubShapeType::Edge, e);
        const ShapeRef r = f.ref(cad::SubShapeType::Edge, e);
        if (g.kind == cad::SubShapeGeometry::Kind::Line) {
            if (std::abs(std::abs(g.direction.dot(slope)) - 1.0) < 1e-6 &&
                std::abs(g.point.y()) < 1e-6) {
                sloped.push_back(r);
            } else if (std::abs(std::abs(g.direction.z()) - 1.0) < 1e-6 &&
                       g.point.head<2>().norm() < 1e-6) {
                vertical.push_back(r);
            }
        } else if ((r.signature.centroid.head<2>() - Vec2(kHoleX, kHoleY)).norm() < kHoleDiameter &&
                   r.signature.centroid.z() > kLow) {
            rim.push_back(r);
        }
    }
    REQUIRE(sloped.size() == 1);
    REQUIRE(vertical.size() == 1);
    REQUIRE_FALSE(rim.empty());
    // The rim really is not flat: it is no chamfer2d edge.
    for (const ShapeRef& r : rim) {
        CHECK_FALSE(cad::edgeAcross(model, r.index, Vec3::UnitZ()));
        const cad::EdgeFaces ef = cad::edgeFaces(model, r.index, 0.5);
        CHECK(ef.problem.empty());
        for (const cad::EdgeFacePoint& p : ef.points) {
            // One face is the slope (its normal up and towards +X), the other the hole's wall.
            const bool slopeFirst = p.normal[0].z() > 0.5;
            const Vec3& wall = slopeFirst ? p.normal[1] : p.normal[0];
            CHECK(std::abs(wall.z()) < 1e-6);
            const Vec2 toAxis = Vec2(kHoleX, kHoleY) - p.point.head<2>();
            CHECK(wall.head<2>().dot(toAxis) > 0.0); // the wall's outward normal faces the axis
        }
    }

    std::vector<ShapeRef> edges = sloped;
    edges.insert(edges.end(), rim.begin(), rim.end());
    edges.push_back(vertical.front());
    const Uuid op = f.addOperation("edgebreak3d", mill, edges, Json{{"width", 0.5}});
    const toolpath::Toolpath tp = f.generate(op);
    // The vertical corner is too steep for the cone: skipped and named; the others are cut.
    REQUIRE(tp.warnings.size() == 1);
    CHECK_THAT(tp.warnings.front(),
               ContainsSubstring(fmt::format("edge {}", vertical.front().index)));
    CHECK_THAT(tp.warnings.front(), ContainsSubstring("too steep"));
    // Cuts along the slope (the WCS 1 mm above the top at x = 0, origin 1 mm outside the
    // block) and around the hole, below the rim.
    bool alongSlope = false;
    bool inHole = false;
    for (const toolpath::Move& m : tp.moves) {
        if (m.feedClass != toolpath::FeedClass::Cut) {
            continue;
        }
        const Vec3 onPart = m.end + Vec3(-1.0, -1.0, kHigh + 1.0);
        alongSlope = alongSlope || (onPart.y() < 0.0 && onPart.x() > 10.0 && onPart.x() < 50.0);
        const double r = (onPart.head<2>() - Vec2(kHoleX, kHoleY)).norm();
        if (r < kHoleDiameter / 2.0) {
            inHole = true;
            const double rimZ = kHigh - (kHigh - kLow) * onPart.x() / kLength;
            CHECK(onPart.z() < rimZ);
        }
    }
    CHECK(alongSlope);
    CHECK(inHole);

    SECTION("no edges selected") {
        const Uuid none = f.addOperation("edgebreak3d", mill, {});
        CHECK_THROWS_WITH(f.generate(none), ContainsSubstring("select the edges"));
    }
}
