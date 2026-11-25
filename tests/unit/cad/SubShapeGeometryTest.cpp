// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "FixtureSpecs.hpp"

#include <ocam/cad/Model.hpp>
#include <ocam/cad/SubShape.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <filesystem>

using namespace ocam;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using Kind = cad::SubShapeGeometry::Kind;

namespace {

std::filesystem::path fixture(const char* name) {
    return std::filesystem::path(OCAM_FIXTURE_DIR) / name;
}

bool near(const Vec3& a, const Vec3& b, double tolerance = 1e-6) {
    return (a - b).norm() < tolerance;
}

} // namespace

TEST_CASE("the exact geometry of faces, edges and vertices", "[cad][subshape]") {
    using namespace fixtures::hole_plate;
    const cad::Model model = cad::Model::importStep(fixture("hole_plate.step"));
    const geom3d::Aabb3 box = model.info().bounds;
    const double top = box.max.z();

    // The through hole: a cylinder of its radius whose axis point sits at the hole's centre.
    int cylinders = 0, circles = 0, tops = 0, vertices = 0;
    for (int f = 1; f <= cad::subShapeCount(model, cad::SubShapeType::Face); ++f) {
        const cad::SubShapeGeometry g = cad::subShapeGeometry(model, cad::SubShapeType::Face, f);
        if (g.kind == Kind::Cylinder && std::abs(g.radius - kThroughDiameter / 2) < 1e-9) {
            ++cylinders;
            CHECK(g.hasAxis());
            CHECK(std::abs(std::abs(g.direction.z()) - 1.0) < 1e-9);
            CHECK((g.point.head<2>() - Eigen::Vector2d(kThroughX, kThroughY)).norm() < 1e-6);
            // Halfway through the plate.
            CHECK_THAT(g.point.z(), WithinAbs(box.min.z() + kThickness / 2, 1e-6));
            // Its two ends, on the axis: the plate's faces, lowest along the axis first.
            const double s = g.direction.z() > 0.0 ? 1.0 : -1.0;
            CHECK((g.axisStart.head<2>() - Eigen::Vector2d(kThroughX, kThroughY)).norm() < 1e-6);
            CHECK_THAT(g.axisStart.z(), WithinAbs(s > 0.0 ? box.min.z() : top, 1e-6));
            CHECK_THAT(g.axisEnd.z(), WithinAbs(s > 0.0 ? top : box.min.z(), 1e-6));
        }
        if (g.kind == Kind::Plane && std::abs(g.point.z() - top) < 1e-9) {
            ++tops;
            CHECK(near(g.direction, Vec3::UnitZ()));
        }
        if (g.kind == Kind::Plane && std::abs(g.point.z() - box.min.z()) < 1e-9) {
            CHECK(near(g.direction, -Vec3::UnitZ())); // outward
        }
    }
    CHECK(cylinders == 1);
    CHECK(tops == 1);
    // Its rims: circles centred on the axis, square to it.
    for (int e = 1; e <= cad::subShapeCount(model, cad::SubShapeType::Edge); ++e) {
        const cad::SubShapeGeometry g = cad::subShapeGeometry(model, cad::SubShapeType::Edge, e);
        if (g.kind == Kind::Circle && std::abs(g.radius - kThroughDiameter / 2) < 1e-9) {
            ++circles;
            CHECK((g.point.head<2>() - Eigen::Vector2d(kThroughX, kThroughY)).norm() < 1e-6);
            CHECK(std::abs(std::abs(g.direction.z()) - 1.0) < 1e-9);
        }
        if (g.kind == Kind::Line) {
            CHECK_THAT(g.direction.norm(), WithinAbs(1.0, 1e-9));
        }
    }
    CHECK(circles >= 2); // one circle per rim, or arcs where a writer splits it
    // The corners are vertices of the box.
    for (int v = 1; v <= cad::subShapeCount(model, cad::SubShapeType::Vertex); ++v) {
        const cad::SubShapeGeometry g = cad::subShapeGeometry(model, cad::SubShapeType::Vertex, v);
        CHECK(g.kind == Kind::Point);
        if (near(g.point, box.min) || near(g.point, box.max)) {
            ++vertices;
        }
    }
    CHECK(vertices == 2);
    CHECK_THROWS(cad::subShapeGeometry(model, cad::SubShapeType::Face, 0));
}

TEST_CASE("models as STEP data, compounds and moved copies", "[cad][subshape]") {
    const cad::Model plate = cad::Model::importStep(fixture("hole_plate.step"));
    const cad::Model block = cad::Model::importStep(fixture("block_pocket.step"));
    // STEP in memory and back.
    const std::string data = plate.exportStepData();
    CancelToken cancel;
    NullProgress progress;
    const cad::Model back = cad::Model::importStepData(data, "plate", cancel, progress);
    CHECK_THAT(back.info().volume, WithinRel(plate.info().volume, 1e-9));
    CHECK(cad::subShapeCount(back, cad::SubShapeType::Face) ==
          cad::subShapeCount(plate, cad::SubShapeType::Face));
    // Two bodies as one.
    const cad::Model both = cad::compound({plate, cad::Model(), block});
    CHECK(both.info().solids == 2);
    CHECK_THAT(both.info().volume, WithinRel(plate.info().volume + block.info().volume, 1e-9));
    CHECK(&cad::compound({plate}).impl() == &plate.impl());
    // Moved: same faces in the same order, 100 mm along X and turned about Z.
    Transform3 xf = Transform3::Identity();
    xf.rotate(Eigen::AngleAxisd(kPi / 2, Vec3::UnitZ()));
    xf.pretranslate(Vec3(100, 0, 0));
    const cad::Model moved = cad::transformed(plate, xf);
    REQUIRE(cad::subShapeCount(moved, cad::SubShapeType::Face) ==
            cad::subShapeCount(plate, cad::SubShapeType::Face));
    for (int f = 1; f <= cad::subShapeCount(plate, cad::SubShapeType::Face); ++f) {
        const auto a = cad::subShapeGeometry(plate, cad::SubShapeType::Face, f);
        const auto b = cad::subShapeGeometry(moved, cad::SubShapeType::Face, f);
        CHECK(a.kind == b.kind);
        CHECK(near(xf * a.point, b.point, 1e-6));
    }
    // Written out and read back, the moved copy keeps its place.
    const cad::Model reread =
        cad::Model::importStepData(moved.exportStepData(), "moved", cancel, progress);
    CHECK(near(reread.info().bounds.min, moved.info().bounds.min, 1e-6));
}
