// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/geom2d/Clip.hpp>
#include <ocam/geom2d/PathOps.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

using namespace ocam;
using namespace ocam::geom2d;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

std::size_t arcCount(const Polyline& p) {
    return static_cast<std::size_t>(std::count_if(p.vertices().begin(), p.vertices().end(),
                                                  [](const Vertex& v) { return v.bulge != 0.0; }));
}

} // namespace

TEST_CASE("arc fitting recovers circles and lines", "[geom2d][pathops]") {
    const Polyline circle = Polyline::circle({3, 4}, 10);
    const Polyline fitted = fitArcs(circle.discretize(1e-4), true, 1e-3);
    CHECK(fitted.isClosed());
    CHECK(fitted.vertices().size() <= 4);
    CHECK(arcCount(fitted) == fitted.vertices().size());
    CHECK_THAT(fitted.area(), WithinRel(kPi * 100, 1e-5));

    const Polyline rect =
        fitArcs(Polyline::rectangle({0, 0}, {20, 10}).discretize(1e-3), true, 1e-3);
    CHECK(rect.vertices().size() == 4);
    CHECK(arcCount(rect) == 0);

    // Collinear intermediate points collapse.
    const Polyline line = fitArcs({{0, 0}, {1, 0}, {2, 0}, {3, 0}}, false, 1e-6);
    CHECK(line.vertices().size() == 2);
}

TEST_CASE("arc fitting of a rounded offset keeps corners round", "[geom2d][pathops]") {
    const Regions grown =
        offset(Regions{Region{Polyline::rectangle({0, 0}, {40, 20}), {}}}, 5.0, {1e-4});
    REQUIRE(grown.size() == 1);
    const Polyline fitted = fitArcs(grown[0].outer.discretize(1e-4), true, 2e-3);
    CHECK(arcCount(fitted) >= 4);
    CHECK(fitted.vertices().size() <= 12);
    CHECK_THAT(fitted.area(), WithinRel(40 * 20 + 2 * 5 * (40 + 20) + kPi * 25, 1e-4));
}

TEST_CASE("closest point on lines and arcs", "[geom2d][pathops]") {
    const Polyline circle = Polyline::circle({0, 0}, 5);
    const ClosestPoint cp = closestPoint(circle, {0, 10});
    CHECK_THAT(cp.distance, WithinAbs(5.0, 1e-12));
    CHECK((cp.point - Vec2(0, 5)).norm() < 1e-12);

    const Polyline rect = Polyline::rectangle({0, 0}, {10, 10});
    const ClosestPoint inner = closestPoint(rect, {3, 1});
    CHECK(inner.segment == 0);
    CHECK_THAT(inner.distance, WithinAbs(1.0, 1e-12));
}

TEST_CASE("restarting a loop preserves geometry", "[geom2d][pathops]") {
    const Polyline circle = Polyline::circle({0, 0}, 5);
    const Polyline restarted = restartAtNearest(circle, {0, 8});
    CHECK((restarted.vertices().front().p - Vec2(0, 5)).norm() < 1e-12);
    CHECK_THAT(restarted.signedArea(), WithinRel(circle.signedArea(), 1e-12));
    CHECK_THAT(restarted.length(), WithinRel(circle.length(), 1e-12));

    const Polyline rect = Polyline::rectangle({0, 0}, {10, 10});
    const Polyline atCorner = restartAtNearest(rect, {11, 11});
    CHECK(atCorner.vertices().front().p == Vec2(10, 10));
    CHECK(atCorner.vertices().size() == 4);
    const Polyline mid = restartAtNearest(rect, {5, -2});
    CHECK(mid.vertices().size() == 5);
    CHECK_THAT(mid.signedArea(), WithinRel(100.0, 1e-12));
}

TEST_CASE("segment containment respects holes and concavity", "[geom2d][pathops]") {
    Polyline hole = Polyline::rectangle({4, 4}, {6, 6});
    hole.reverse();
    const Regions plate{Region{Polyline::rectangle({0, 0}, {10, 10}), {hole}}};
    CHECK(segmentInside(plate, {1, 1}, {9, 1}));
    CHECK_FALSE(segmentInside(plate, {1, 5}, {9, 5}));  // crosses the hole
    CHECK_FALSE(segmentInside(plate, {1, 1}, {11, 1})); // leaves the region

    const Regions ell{Region{
        Polyline::fromPoints({{0, 0}, {10, 0}, {10, 2}, {2, 2}, {2, 10}, {0, 10}}, true), {}}};
    CHECK_FALSE(segmentInside(ell, {9, 1}, {1, 9})); // cuts the inner corner
    CHECK(segmentInside(ell, {9, 1}, {1, 1}));
}

TEST_CASE("swept area of a straight path", "[geom2d][pathops]") {
    const Regions swept = sweep({{{0, 0}, {10, 0}}}, 2.0, {1e-4});
    CHECK_THAT(totalArea(swept), WithinRel(10 * 4 + kPi * 4, 1e-3));
}
