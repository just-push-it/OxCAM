// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "FixtureSpecs.hpp"

#include <ocam/cad/Assembly.hpp>
#include <ocam/cad/Datum.hpp>
#include <ocam/foundation/Error.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <filesystem>

using namespace ocam;
using Catch::Matchers::WithinAbs;

TEST_CASE("datum geometry of faces, edges and vertices; STEP in memory; merged models",
          "[cad][datum]") {
    using namespace fixtures::plate;
    CancelToken cancel;
    NullProgress progress;
    const auto bodies = cad::importStepAssembly(
        std::filesystem::path(OCAM_FIXTURE_DIR) / "plate_assembly.step", cancel, progress);
    REQUIRE(bodies.size() == 2);
    const cad::Model& plate = bodies[0].model;
    const cad::Model& pin = bodies[1].model;

    // Written to memory and read back: the same solid, the same faces.
    const std::string bytes = plate.exportStepData();
    const cad::Model again = cad::Model::importStepData(bytes, "plate", cancel, progress);
    CHECK(again.info().faces == plate.info().faces);
    CHECK((again.info().bounds.min - plate.info().bounds.min).norm() < 1e-6);
    CHECK((again.info().bounds.max - plate.info().bounds.max).norm() < 1e-6);

    // Every face of the plate is flat: its centre on the box, its normal outward.
    int planes = 0;
    for (int f = 1; f <= cad::subShapeCount(plate, cad::SubShapeType::Face); ++f) {
        const cad::DatumGeometry g = cad::datumGeometry(plate, cad::SubShapeType::Face, f);
        REQUIRE(g.shape == cad::DatumGeometry::Shape::Plane);
        ++planes;
        const Vec3 centre(kLength / 2, kWidth / 2, kThickness / 2);
        CHECK(g.direction.dot(g.point - centre) > 0.0); // outward
    }
    CHECK(planes == 6);
    // The pin's round face: its axis through the pin's centre; its circular edges: the axis too.
    bool axis = false;
    for (int f = 1; f <= cad::subShapeCount(pin, cad::SubShapeType::Face); ++f) {
        const cad::DatumGeometry g = cad::datumGeometry(pin, cad::SubShapeType::Face, f);
        if (g.shape == cad::DatumGeometry::Shape::Axis) {
            axis = true;
            CHECK_THAT(g.point.x(), WithinAbs(kPinX, 1e-6));
            CHECK_THAT(g.point.y(), WithinAbs(kPinY, 1e-6));
            CHECK_THAT(std::abs(g.direction.z()), WithinAbs(1.0, 1e-9));
        }
    }
    CHECK(axis);
    bool circle = false;
    for (int e = 1; e <= cad::subShapeCount(pin, cad::SubShapeType::Edge); ++e) {
        const cad::DatumGeometry g = cad::datumGeometry(pin, cad::SubShapeType::Edge, e);
        if (g.shape == cad::DatumGeometry::Shape::Axis) {
            circle = true;
            CHECK_THAT(g.point.x(), WithinAbs(kPinX, 1e-6));
            CHECK_THAT(g.point.y(), WithinAbs(kPinY, 1e-6));
        }
    }
    CHECK(circle);
    const cad::DatumGeometry vertex = cad::datumGeometry(plate, cad::SubShapeType::Vertex, 1);
    CHECK(vertex.shape == cad::DatumGeometry::Shape::Point);
    CHECK(vertex.direction.isZero());
    CHECK_THROWS_AS(cad::datumGeometry(plate, cad::SubShapeType::Face, 99), Error);

    // Merged: both solids in one model.
    const cad::Model merged = cad::compound({plate, pin});
    CHECK(merged.info().solids == 2);
    CHECK(merged.info().faces == plate.info().faces + pin.info().faces);
}
