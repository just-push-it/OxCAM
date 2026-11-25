// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "FixtureSpecs.hpp"

#include <ocam/cad/Revolve.hpp>
#include <ocam/foundation/Error.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <filesystem>

using namespace ocam;
using namespace ocam::fixtures;
using Catch::Matchers::WithinAbs;

namespace {

/// Largest radius of an outline at height h (from its segments).
double radiusAt(const geom2d::Polyline& outline, double h) {
    double r = 0.0;
    const auto pts = outline.discretize(0.001);
    for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
        const Vec2& a = pts[k];
        const Vec2& b = pts[k + 1];
        if ((a.y() - h) * (b.y() - h) > 0.0 || a.y() == b.y()) {
            continue;
        }
        r = std::max(r, a.x() + (b.x() - a.x()) * (h - a.y()) / (b.y() - a.y()));
    }
    return r;
}

} // namespace

TEST_CASE("holder outline of a stepped solid", "[cad][holder]") {
    using namespace shaft;
    const cad::Model model =
        cad::Model::importStep(std::filesystem::path(OCAM_FIXTURE_DIR) / "shaft.step");
    const double tol = 0.02;

    // Auto: the Z axis (round across it); the tool end is away from the wide part, at the top.
    const cad::HolderOutline auto_ = cad::holderOutline(model);
    CHECK(auto_.axis == cad::RevolveAxis::Z);
    CHECK(auto_.toolEnd == cad::ToolEnd::High);
    CHECK(auto_.asymmetry < 1e-2); // of the facets
    CHECK_THAT(auto_.fullLength, WithinAbs(kMajorLength + kMinorLength, 1e-6));
    // The wide part reaches the far end: nothing to cut off.
    CHECK_THAT(auto_.length, WithinAbs(kMajorLength + kMinorLength, 1e-6));
    const auto& v = auto_.profile.vertices();
    REQUIRE(v.size() >= 4);
    CHECK(v.front().p == Vec2(0.0, 0.0));
    CHECK(v.back().p.x() == 0.0);
    // Never inside the model, and close to it away from the step.
    for (double h : {1.0, 10.0, 25.0, 28.0}) {
        const double r = radiusAt(auto_.profile, h);
        CHECK(r >= kMinorDiameter / 2.0);
        CHECK(r <= kMinorDiameter / 2.0 + 2.0 * tol);
    }
    for (double h : {31.0, 50.0, 79.0}) {
        const double r = radiusAt(auto_.profile, h);
        CHECK(r >= kMajorDiameter / 2.0);
        CHECK(r <= kMajorDiameter / 2.0 + 2.0 * tol);
    }
    CHECK(v.size() < 20); // straight runs collapse

    // The tool end at the low end: the gauge line is the top of the wide part.
    cad::HolderOutlineOptions options;
    options.toolEnd = cad::ToolEnd::Low;
    const cad::HolderOutline low = cad::holderOutline(model, options);
    CHECK(low.toolEnd == cad::ToolEnd::Low);
    CHECK_THAT(low.suggestedLength, WithinAbs(kMajorLength, 0.2));
    CHECK_THAT(low.length, WithinAbs(low.suggestedLength, 1e-9));
    CHECK(radiusAt(low.profile, 20.0) >= kMajorDiameter / 2.0);

    // An explicit length and axis.
    options.length = 30.0;
    options.axis = cad::RevolveAxis::Z;
    const cad::HolderOutline cut = cad::holderOutline(model, options);
    CHECK_THAT(cut.length, WithinAbs(30.0, 1e-9));
    CHECK_THAT(cut.profile.vertices().back().p.y(), WithinAbs(30.0, 1e-9));

    // Turned about X the model is not round.
    options.axis = cad::RevolveAxis::X;
    options.length = 0.0;
    CHECK(cad::holderOutline(model, options).asymmetry > 0.5);

    CHECK_THROWS_AS(cad::holderOutline(cad::Model{}), Error);
}
