// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Probing operations in a project (DL-0090): the faces picked on the model give the nominal
// values, and the cycle sets the work offset the operation names.

#include "FixtureSpecs.hpp"

#include <ocam/cad/SubShape.hpp>
#include <ocam/document/Document.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/Probing.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/foundation/Error.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <filesystem>
#include <functional>

using namespace ocam;
using namespace ocam::document;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

struct ProbeProject {
    Document doc;
    Uuid modelId;
    Uuid setupId;
    Uuid probeId;
    CancelToken cancel;
    NullProgress progress;

    explicit ProbeProject(const char* step) {
        modelId = doc.importModel(std::filesystem::path(OCAM_FIXTURE_DIR) / step, cancel, progress);
        setupId = doc.addSetup(modelId);
        tooling::Tool probe = tooling::makeDefaultTool(tooling::ToolType::Probe, 20, 6.0);
        probe.mill.triggerOffset = 0.02;
        probeId = probe.id;
        doc.modify("probe", [&](Project& p) { p.tools.tools.push_back(probe); });
    }

    const Setup& setup() const { return *doc.project().findSetup(setupId); }

    /// The model's face whose geometry matches (model coordinates).
    ShapeRef face(const std::function<bool(const cad::SubShapeGeometry&)>& match) const {
        const cad::Model& model = *doc.model(modelId);
        for (int i = 1; i <= cad::subShapeCount(model, cad::SubShapeType::Face); ++i) {
            if (match(cad::subShapeGeometry(model, cad::SubShapeType::Face, i))) {
                return {modelId, cad::SubShapeType::Face, i,
                        cad::signatureOf(model, cad::SubShapeType::Face, i)};
            }
        }
        FAIL("no such face");
        return {};
    }
    /// The flat face looking along `normal` at `at` along that axis.
    ShapeRef plane(const Vec3& normal, double at) const {
        return face([&](const cad::SubShapeGeometry& g) {
            const int axis = std::abs(normal.x()) > 0.5 ? 0 : std::abs(normal.y()) > 0.5 ? 1 : 2;
            return g.kind == cad::SubShapeGeometry::Kind::Plane &&
                   (g.direction - normal).norm() < 1e-9 && std::abs(g.point[axis] - at) < 1e-6;
        });
    }

    Operation operation(const std::string& type, std::vector<ShapeRef> faces) const {
        Operation op;
        op.name = type;
        op.type = type;
        op.toolId = probeId;
        op.geometry = std::move(faces);
        op.heights = defaultHeights(op);
        return op;
    }

    Uuid add(Operation op) {
        op.parameters = withProbeNominals(doc, setup(), op);
        doc.modify("op", [&](Project& p) { p.findSetup(setupId)->operations.push_back(op); });
        return op.id;
    }

    toolpath::Toolpath generate(const Uuid& id) {
        const Setup* owner = nullptr;
        const Operation* op = doc.project().findOperation(id, &owner);
        return generateToolpath(doc, *owner, *op, cancel, progress);
    }
};

double number(const Json& j, const char* key) {
    return j.at(key).get<double>();
}

} // namespace

TEST_CASE("picked faces give a probing cycle its nominal values", "[document][probing]") {
    using namespace fixtures::block_pocket;
    ProbeProject f("block_pocket.step");
    // The default setup's WCS is at the stock's top corner, 1 mm out: model (x, y, z) is
    // WCS (x + 1, y + 1, z - kHeight - 1).
    REQUIRE((f.setup().wcs.origin - Vec3(-1, -1, kHeight + 1)).norm() < 1e-6);
    const Setup& s = f.setup();

    SECTION("the top, probed down Z") {
        const Json n = probeNominals(
            f.doc, s, f.operation("probe_surface", {f.plane(Vec3::UnitZ(), kHeight)}));
        CHECK(n.at("direction") == "z_minus");
        CHECK_FALSE(n.contains("feature"));
        CHECK_THAT(number(n, "z"), WithinAbs(-1.0, 1e-6));
    }
    SECTION("the -X side, probed along +X") {
        const Json n =
            probeNominals(f.doc, s, f.operation("probe_surface", {f.plane(-Vec3::UnitX(), 0.0)}));
        CHECK(n.at("direction") == "x_plus");
        CHECK_THAT(number(n, "x"), WithinAbs(1.0, 1e-6));
        CHECK_THAT(number(n, "z"), WithinAbs(-1.0 - kHeight / 2.0, 1e-6));
    }
    SECTION("two outside faces are a web") {
        const Json n = probeNominals(f.doc, s,
                                     f.operation("probe_width", {f.plane(-Vec3::UnitX(), 0.0),
                                                                 f.plane(Vec3::UnitX(), kLength)}));
        CHECK(n.at("feature") == "web");
        CHECK(n.at("axis") == "x");
        CHECK_THAT(number(n, "width"), WithinAbs(kLength, 1e-6));
        CHECK_THAT(number(n, "x"), WithinAbs(kLength / 2.0 + 1.0, 1e-6));
    }
    SECTION("two inside faces are a pocket") {
        const Json n = probeNominals(
            f.doc, s,
            f.operation("probe_width", {f.plane(Vec3::UnitY(), kPocketY),
                                        f.plane(-Vec3::UnitY(), kPocketY + kPocketWidth)}));
        CHECK(n.at("feature") == "pocket");
        CHECK(n.at("axis") == "y");
        CHECK_THAT(number(n, "width"), WithinAbs(kPocketWidth, 1e-6));
        CHECK_THAT(number(n, "y"), WithinAbs(kPocketY + kPocketWidth / 2.0 + 1.0, 1e-6));
    }
    SECTION("an outside and an inside corner") {
        const Json outside =
            probeNominals(f.doc, s,
                          f.operation("probe_corner", {f.plane(-Vec3::UnitX(), 0.0),
                                                       f.plane(-Vec3::UnitY(), 0.0)}));
        CHECK(outside.at("feature") == "outside_corner");
        CHECK(outside.at("corner") == "x_minus_y_minus");
        CHECK_THAT(number(outside, "x"), WithinAbs(1.0, 1e-6));
        CHECK_THAT(number(outside, "y"), WithinAbs(1.0, 1e-6));

        const Json inside =
            probeNominals(f.doc, s,
                          f.operation("probe_corner", {f.plane(Vec3::UnitX(), kPocketX),
                                                       f.plane(Vec3::UnitY(), kPocketY)}));
        CHECK(inside.at("feature") == "inside_corner");
        CHECK(inside.at("corner") == "x_minus_y_minus");
        CHECK_THAT(number(inside, "x"), WithinAbs(kPocketX + 1.0, 1e-6));
        CHECK_THAT(number(inside, "y"), WithinAbs(kPocketY + 1.0, 1e-6));
    }
    SECTION("faces that are not what the operation measures") {
        CHECK_THROWS_WITH(
            probeNominals(f.doc, s, f.operation("probe_circle", {f.plane(Vec3::UnitZ(), kHeight)})),
            ContainsSubstring("does not measure"));
        CHECK_THROWS_WITH(
            probeNominals(f.doc, s,
                          f.operation("probe_width", {f.plane(-Vec3::UnitX(), 0.0),
                                                      f.plane(Vec3::UnitZ(), kHeight)})),
            ContainsSubstring("side faces"));
        CHECK_THROWS_WITH(probeNominals(f.doc, s, f.operation("probe_surface", {})),
                          ContainsSubstring("pick the faces"));
    }
}

TEST_CASE("bores and bosses from their cylinders", "[document][probing]") {
    SECTION("a bore") {
        using namespace fixtures::hole_plate;
        ProbeProject f("hole_plate.step");
        const ShapeRef wall = f.face([](const cad::SubShapeGeometry& g) {
            return g.kind == cad::SubShapeGeometry::Kind::Cylinder &&
                   std::abs(2.0 * g.radius - kThroughDiameter) < 1e-6;
        });
        const Json n = probeNominals(f.doc, f.setup(), f.operation("probe_check", {wall}));
        CHECK(n.at("feature") == "bore");
        CHECK_THAT(number(n, "diameter"), WithinAbs(kThroughDiameter, 1e-6));
        CHECK_THAT(number(n, "x"), WithinAbs(kThroughX + 1.0, 1e-6));
        CHECK_THAT(number(n, "y"), WithinAbs(kThroughY + 1.0, 1e-6));
    }
    SECTION("a boss") {
        using namespace fixtures::boss_plate;
        ProbeProject f("boss_plate.step");
        const ShapeRef wall = f.face([](const cad::SubShapeGeometry& g) {
            return g.kind == cad::SubShapeGeometry::Kind::Cylinder &&
                   std::abs(2.0 * g.radius - kBossDiameter) < 1e-6;
        });
        const Json n = probeNominals(f.doc, f.setup(), f.operation("probe_circle", {wall}));
        CHECK(n.at("feature") == "boss");
        CHECK_THAT(number(n, "x"), WithinAbs(kBossX + 1.0, 1e-6));
    }
}

TEST_CASE("a probing operation's toolpath and simulation", "[document][probing]") {
    using namespace fixtures::block_pocket;
    ProbeProject f("block_pocket.step");
    const Uuid corner = f.add(
        f.operation("probe_corner", {f.plane(-Vec3::UnitX(), 0.0), f.plane(-Vec3::UnitY(), 0.0)}));
    const toolpath::Toolpath tp = f.generate(corner);
    CHECK(tp.operationId == corner);
    REQUIRE(tp.probes.size() == 1);
    CHECK(tp.probes[0].feature == toolpath::ProbeFeature::OutsideCorner);
    CHECK(tp.probes[0].workOffset == f.setup().workOffset); // 0 = the setup's own
    CHECK(tp.probes[0].axes == "XY");
    CHECK_THAT(tp.probes[0].nominal.x(), WithinAbs(1.0, 1e-6));
    CHECK(tp.spindleSpeed == 0.0);
    CHECK(tp.warnings.empty());

    // A value typed over the picked faces' is used, and the difference is said.
    f.doc.modify("move corner",
                 [&](Project& p) { p.findOperation(corner)->parameters["x"] = 1.5; });
    const toolpath::Toolpath moved = f.generate(corner);
    CHECK_THAT(moved.probes[0].nominal.x(), WithinAbs(1.5, 1e-12));
    REQUIRE_FALSE(moved.warnings.empty());
    CHECK_THAT(moved.warnings.front(), ContainsSubstring("pick the faces again"));

    // A zero feed is refused by name (ADR-0004).
    f.doc.modify("feed",
                 [&](Project& p) { p.findOperation(corner)->parameters["measure_feed"] = 0.0; });
    CHECK_THROWS_WITH(f.generate(corner), ContainsSubstring("measuring feed is 0"));
    f.doc.modify("feed", [&](Project& p) {
        p.findOperation(corner)->parameters["measure_feed"] = 120.0;
        p.findOperation(corner)->parameters["x"] = 1.0;
    });

    // The simulation shows its moves and cuts nothing: the stock is untouched (1 mm all round),
    // so the stylus meets it before the part's faces and that is reported.
    const SetupSimulation sim = simulateSetup(f.doc, f.setupId, {}, f.cancel, f.progress);
    REQUIRE(sim.operations.size() == 1);
    const OperationSimulation& os = sim.operations.front();
    CHECK(os.removedVolume == 0.0);
    CHECK(os.toolpath.isProbing());
    REQUIRE_FALSE(os.collisions.empty());
    CHECK(os.collisions.front().kind == sim::CollisionKind::Probe);
}

TEST_CASE("a probing operation needs a probe", "[document][probing]") {
    using namespace fixtures::block_pocket;
    ProbeProject f("block_pocket.step");
    const tooling::Tool mill = tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0);
    f.doc.modify("mill", [&](Project& p) { p.tools.tools.push_back(mill); });
    Operation op = f.operation("probe_surface", {f.plane(Vec3::UnitZ(), kHeight)});
    op.toolId = mill.id;
    const Uuid id = f.add(op);
    CHECK_THROWS_WITH(f.generate(id), ContainsSubstring("needs a probe"));
}
