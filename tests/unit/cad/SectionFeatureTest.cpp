// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "FixtureSpecs.hpp"

#include <ocam/cad/Features.hpp>
#include <ocam/cad/Section.hpp>
#include <ocam/cad/SubShape.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <filesystem>

using namespace ocam;
using namespace ocam::fixtures;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

cad::Model load(const char* name) {
    return cad::Model::importStep(std::filesystem::path(OCAM_FIXTURE_DIR) / name);
}

double circleArea(double d) {
    return kPi * d * d / 4.0;
}

bool hasArc(const geom2d::Polyline& p) {
    return std::any_of(p.vertices().begin(), p.vertices().end(),
                       [](const geom2d::Vertex& v) { return v.bulge != 0.0; });
}

} // namespace

TEST_CASE("section through the pocket keeps fillet arcs exact", "[cad][section]") {
    using namespace block_pocket;
    const cad::Model model = load("block_pocket.step");

    const geom2d::Regions pocketLevel = cad::sectionAtZ(model, kHeight - kPocketDepth / 2);
    REQUIRE(pocketLevel.size() == 1);
    REQUIRE(pocketLevel[0].holes.size() == 1);
    CHECK(hasArc(pocketLevel[0].holes[0]));
    CHECK_THAT(pocketLevel[0].holes[0].area(), WithinRel(kPocketArea, 1e-6));
    CHECK_THAT(geom2d::totalArea(pocketLevel), WithinRel(kLength * kWidth - kPocketArea, 1e-6));
    CHECK(pocketLevel[0].outer.isCounterClockwise());

    const geom2d::Regions belowPocket = cad::sectionAtZ(model, 5.0);
    REQUIRE(belowPocket.size() == 1);
    CHECK(belowPocket[0].holes.empty());
    CHECK_THAT(belowPocket[0].area(), WithinRel(kLength * kWidth, 1e-6));

    CHECK(cad::sectionAtZ(model, kHeight + 1.0).empty());
}

TEST_CASE("section through the hole plate", "[cad][section]") {
    using namespace hole_plate;
    const cad::Model model = load("hole_plate.step");

    // z = 18: every hole is cut (counterbore at its large diameter).
    const double expected18 = kSize * kSize - circleArea(kThroughDiameter) -
                              circleArea(kFlatDiameter) - circleArea(kDrillDiameter) -
                              circleArea(kCboreDiameter);
    const geom2d::Regions r18 = cad::sectionAtZ(model, 18.0);
    REQUIRE(r18.size() == 1);
    CHECK(r18[0].holes.size() == 4);
    CHECK_THAT(geom2d::totalArea(r18), WithinRel(expected18, 1e-6));

    // z = 5: only the two through holes remain.
    const double expected5 =
        kSize * kSize - circleArea(kThroughDiameter) - circleArea(kCboreHoleDiameter);
    CHECK_THAT(geom2d::totalArea(cad::sectionAtZ(model, 5.0)), WithinRel(expected5, 1e-6));
}

TEST_CASE("section through the shaft", "[cad][section]") {
    using namespace shaft;
    const cad::Model model = load("shaft.step");
    CHECK_THAT(geom2d::totalArea(cad::sectionAtZ(model, 10.0)),
               WithinRel(circleArea(kMajorDiameter), 1e-6));
    CHECK_THAT(geom2d::totalArea(cad::sectionAtZ(model, kMajorLength + 10.0)),
               WithinRel(circleArea(kMinorDiameter), 1e-6));
}

TEST_CASE("hole recognition: one of each kind", "[cad][features]") {
    using namespace hole_plate;
    const auto holes = cad::findHoles(load("hole_plate.step"));
    REQUIRE(holes.size() == 5);

    // Sorted by entry (x, y, z).
    const auto& through = holes[0];
    CHECK((through.entry - Vec3(kThroughX, kThroughY, kThickness)).norm() < 1e-6);
    CHECK((through.direction - Vec3(0, 0, -1)).norm() < 1e-9);
    CHECK_THAT(through.diameter, WithinRel(kThroughDiameter, 1e-6));
    CHECK_THAT(through.depth, WithinRel(kThickness, 1e-6));
    CHECK(through.bottom == cad::HoleBottom::Through);

    const auto& drilled = holes[1];
    CHECK((drilled.entry - Vec3(kDrillX, kDrillY, kThickness)).norm() < 1e-6);
    CHECK_THAT(drilled.diameter, WithinRel(kDrillDiameter, 1e-6));
    CHECK_THAT(drilled.depth, WithinRel(kDrillDepth, 1e-6));
    CHECK(drilled.bottom == cad::HoleBottom::Cone);
    CHECK_THAT(drilled.tipAngle, WithinRel(kDrillTipAngle, 1e-6));

    const auto& flat = holes[2];
    CHECK((flat.entry - Vec3(kFlatX, kFlatY, kThickness)).norm() < 1e-6);
    CHECK_THAT(flat.diameter, WithinRel(kFlatDiameter, 1e-6));
    CHECK_THAT(flat.depth, WithinRel(kFlatDepth, 1e-6));
    CHECK(flat.bottom == cad::HoleBottom::Flat);

    const auto& cboreHole = holes[3];
    CHECK((cboreHole.entry - Vec3(kCboreX, kCboreY, kThickness - kCboreDepth)).norm() < 1e-6);
    CHECK_THAT(cboreHole.diameter, WithinRel(kCboreHoleDiameter, 1e-6));
    CHECK_THAT(cboreHole.depth, WithinRel(kThickness - kCboreDepth, 1e-6));
    CHECK(cboreHole.bottom == cad::HoleBottom::Through);

    const auto& cbore = holes[4];
    CHECK((cbore.entry - Vec3(kCboreX, kCboreY, kThickness)).norm() < 1e-6);
    CHECK_THAT(cbore.diameter, WithinRel(kCboreDiameter, 1e-6));
    CHECK_THAT(cbore.depth, WithinRel(kCboreDepth, 1e-6));
    CHECK(cbore.bottom == cad::HoleBottom::Flat);
}

TEST_CASE("hole recognition ignores bosses and other axes", "[cad][features]") {
    using namespace boss_plate;
    const cad::Model model = load("boss_plate.step");
    const auto holes = cad::findHoles(model);
    REQUIRE(holes.size() == 1);
    CHECK_THAT(holes[0].diameter, WithinRel(kHoleDiameter, 1e-6));
    CHECK((holes[0].entry - Vec3(kHoleX, kHoleY, kThickness)).norm() < 1e-6);

    CHECK(cad::findHoles(model, Vec3::UnitX()).empty());
    CHECK(cad::findHoles(load("shaft.step")).empty());
}

TEST_CASE("boss recognition: studs and turned diameters", "[cad][features]") {
    {
        using namespace boss_plate;
        const cad::Model model = load("boss_plate.step");
        const auto bosses = cad::findBosses(model);
        REQUIRE(bosses.size() == 1); // the plate's sides are planes; the hole is concave
        CHECK_THAT(bosses[0].diameter, WithinRel(kBossDiameter, 1e-6));
        CHECK_THAT(bosses[0].length, WithinRel(kBossHeight, 1e-6));
        CHECK((bosses[0].entry - Vec3(kBossX, kBossY, kThickness + kBossHeight)).norm() < 1e-6);
        CHECK((bosses[0].direction - Vec3(0, 0, -1)).norm() < 1e-9);
        CHECK_FALSE(bosses[0].faces.empty());
        CHECK(cad::findBosses(model, Vec3::UnitX()).empty());
        // Seen from below, it stands on the plate's far side: its free end is still its top.
        const auto below = cad::findBosses(model, -Vec3::UnitZ());
        REQUIRE(below.size() == 1);
        CHECK((below[0].direction - Vec3(0, 0, -1)).norm() < 1e-9);
    }
    {
        using namespace shaft;
        const auto bosses = cad::findBosses(load("shaft.step"));
        REQUIRE(bosses.size() == 2);
        std::vector<double> diameters{bosses[0].diameter, bosses[1].diameter};
        std::sort(diameters.begin(), diameters.end());
        CHECK_THAT(diameters[0], WithinRel(kMinorDiameter, 1e-6));
        CHECK_THAT(diameters[1], WithinRel(kMajorDiameter, 1e-6));
        for (const cad::Boss& b : bosses) {
            CHECK((b.direction - Vec3(0, 0, -1)).norm() < 1e-9);
        }
    }
    CHECK(cad::findHoles(load("boss_plate.step")).size() == 1); // holes are unchanged
}

TEST_CASE("planar face recognition", "[cad][features]") {
    using namespace block_pocket;
    const auto faces = cad::findPlanarFaces(load("block_pocket.step"));
    REQUIRE(faces.size() == 3);
    CHECK_THAT(faces[0].height, WithinAbs(kHeight, 1e-6));
    CHECK(faces[0].facingUp);
    CHECK_THAT(faces[0].area, WithinRel(kLength * kWidth - kPocketArea, 1e-6));
    CHECK_THAT(faces[1].height, WithinAbs(kHeight - kPocketDepth, 1e-6));
    CHECK(faces[1].facingUp);
    CHECK_THAT(faces[1].area, WithinRel(kPocketArea, 1e-6));
    CHECK_THAT(faces[2].height, WithinAbs(0.0, 1e-6));
    CHECK_FALSE(faces[2].facingUp);
}

TEST_CASE("face planes for WCS alignment", "[cad][features]") {
    using namespace block_pocket;
    const cad::Model model = load("block_pocket.step");
    const auto faces = cad::findPlanarFaces(model);
    const auto bottom = cad::facePlane(model, faces[2].face);
    REQUIRE(bottom);
    CHECK((bottom->normal + Vec3::UnitZ()).norm() < 1e-9);
    CHECK_THAT(bottom->origin.z(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(bottom->origin.x(), WithinAbs(kLength / 2.0, 1e-6));
    // Side faces are found along their own axis.
    const auto sides = cad::findPlanarFaces(model, -Vec3::UnitY());
    REQUIRE_FALSE(sides.empty());
    const auto front = cad::facePlane(model, sides.front().face);
    REQUIRE(front);
    CHECK((front->normal + Vec3::UnitY()).norm() < 1e-9);
}

TEST_CASE("axial profile of a turned shaft", "[cad][section]") {
    using namespace shaft;
    const cad::Model model = load("shaft.step");
    // WCS at the minor end's face centre, Z along the shaft axis.
    Transform3 toWcs = Transform3::Identity();
    toWcs.translation() = Vec3(0, 0, -(kMajorLength + kMinorLength));
    const auto profile = cad::axialProfile(model, toWcs, 1e-4);
    REQUIRE_FALSE(profile.empty());
    CHECK_THAT(
        geom2d::totalArea(profile),
        WithinRel(kMajorDiameter / 2 * kMajorLength + kMinorDiameter / 2 * kMinorLength, 1e-6));
    geom2d::Aabb2 b;
    for (const auto& r : profile) {
        b.extend(r.bounds());
    }
    CHECK_THAT(b.min.x(), WithinAbs(-(kMajorLength + kMinorLength), 1e-6));
    CHECK_THAT(b.max.x(), WithinAbs(0.0, 1e-6));
    CHECK_THAT(b.max.y(), WithinAbs(kMajorDiameter / 2, 1e-6));
    CHECK_THAT(b.min.y(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("edges across an axis, and points inside the part", "[cad][subshape]") {
    using namespace block_pocket;
    const cad::Model model = load("block_pocket.step");
    int across = 0;
    int along = 0;
    for (int i = 1; i <= cad::subShapeCount(model, cad::SubShapeType::Edge); ++i) {
        const auto sig = cad::signatureOf(model, cad::SubShapeType::Edge, i);
        const bool horizontal = cad::edgeAcross(model, i, Vec3::UnitZ());
        // Vertical lines (the block's and the pocket's corners) are not across Z; lines and
        // fillet arcs at one height are. (A circle's direction is its axis: the pocket's fillet
        // arcs point along Z too, but lie at a face's height.)
        const double z = sig.centroid.z();
        const bool atFace = std::abs(z) < 1e-6 || std::abs(z - kHeight) < 1e-6 ||
                            std::abs(z - (kHeight - kPocketDepth)) < 1e-6;
        if (std::abs(std::abs(sig.direction.z()) - 1.0) < 1e-9 && !atFace) {
            CHECK_FALSE(horizontal);
            ++along;
        } else {
            CHECK(horizontal);
            ++across;
        }
    }
    CHECK(along >= 8);   // four block corners, four pocket walls' joins
    CHECK(across >= 16); // top and bottom outlines, the pocket's rim and floor
    CHECK(cad::containsPoint(model, Vec3(5, 5, 5)));
    CHECK_FALSE(cad::containsPoint(model, Vec3(kPocketX + 5, kPocketY + 5, kHeight - 1)));
    CHECK_FALSE(cad::containsPoint(model, Vec3(-1, 5, 5)));
}
