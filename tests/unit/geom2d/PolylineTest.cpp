// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/geom2d/Polyline.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

using namespace ocam;
using namespace ocam::geom2d;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// Rounded rectangle [0,w]x[0,h] with corner radius r, counter-clockwise.
Polyline roundedRect(double w, double h, double r) {
    const double b = bulgeFromSweep(kPi / 2.0);
    return Polyline({{{r, 0}, 0.0},
                     {{w - r, 0}, b},
                     {{w, r}, 0.0},
                     {{w, h - r}, b},
                     {{w - r, h}, 0.0},
                     {{r, h}, b},
                     {{0, h - r}, 0.0},
                     {{0, r}, b}},
                    true);
}

} // namespace

TEST_CASE("arc from bulge", "[geom2d][polyline]") {
    // Quarter arc CCW from (1,0) to (0,1) around the origin.
    const auto arc = arcFromBulge({1, 0}, {0, 1}, bulgeFromSweep(kPi / 2.0));
    REQUIRE(arc.has_value());
    CHECK_THAT(arc->radius, WithinAbs(1.0, 1e-12));
    CHECK(arc->center.norm() < 1e-12);
    CHECK_THAT(arc->sweep, WithinAbs(kPi / 2.0, 1e-12));
    CHECK((arc->pointAt(0.5) - Vec2(std::sqrt(0.5), std::sqrt(0.5))).norm() < 1e-12);

    // Negative bulge: clockwise, center on the other side.
    const auto cw = arcFromBulge({1, 0}, {0, 1}, -bulgeFromSweep(kPi / 2.0));
    REQUIRE(cw.has_value());
    CHECK((cw->center - Vec2(1, 1)).norm() < 1e-12);

    CHECK_FALSE(arcFromBulge({0, 0}, {1, 0}, 0.0).has_value());
    CHECK_FALSE(arcFromBulge({1, 1}, {1, 1}, 0.5).has_value());
}

TEST_CASE("circle area, length and bounds are exact", "[geom2d][polyline]") {
    const double r = GENERATE(0.5, 3.0, 125.0);
    const Polyline c = Polyline::circle({2, -1}, r);
    CHECK_THAT(c.signedArea(), WithinRel(kPi * r * r, 1e-12));
    CHECK_THAT(c.length(), WithinRel(2.0 * kPi * r, 1e-12));
    const Aabb2 box = c.bounds();
    CHECK_THAT(box.min.x(), WithinAbs(2 - r, 1e-9));
    CHECK_THAT(box.max.y(), WithinAbs(-1 + r, 1e-9));
    CHECK_THAT(box.min.y(), WithinAbs(-1 - r, 1e-9));
}

TEST_CASE("rounded rectangle area is exact", "[geom2d][polyline]") {
    const Polyline p = roundedRect(40, 20, 5);
    CHECK_THAT(p.signedArea(), WithinRel(40.0 * 20.0 - (4.0 - kPi) * 25.0, 1e-12));
    CHECK_THAT(p.length(), WithinRel(2 * (30.0 + 10.0) + 2 * kPi * 5.0, 1e-12));
    CHECK(p.isCounterClockwise());
}

TEST_CASE("reverse flips orientation and keeps geometry", "[geom2d][polyline]") {
    Polyline p = roundedRect(40, 20, 5);
    const double area = p.signedArea();
    const double len = p.length();
    p.reverse();
    CHECK_THAT(p.signedArea(), WithinRel(-area, 1e-12));
    CHECK_THAT(p.length(), WithinRel(len, 1e-12));
    p.reverse();
    CHECK_THAT(p.signedArea(), WithinRel(area, 1e-12));

    Polyline open({{{0, 0}, 0.0}, {{1, 0}, 0.4}, {{2, 1}, 0.0}}, false);
    const double openLen = open.length();
    open.reverse();
    CHECK(open.vertices().front().p == Vec2(2, 1));
    // Segment (1,0)->(2,1) with bulge 0.4 becomes (2,1)->(1,0), stored on the new first vertex.
    CHECK_THAT(open.vertices()[0].bulge, WithinAbs(-0.4, 1e-15));
    CHECK(open.vertices()[1].bulge == 0.0);
    CHECK_THAT(open.length(), WithinRel(openLen, 1e-12));
}

TEST_CASE("discretize respects the chord tolerance", "[geom2d][polyline]") {
    const double r = 10.0;
    const double tol = GENERATE(1e-1, 1e-3, 1e-5);
    const auto pts = Polyline::circle({0, 0}, r).discretize(tol);
    REQUIRE(pts.size() >= 4);
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const Vec2 mid = 0.5 * (pts[i] + pts[(i + 1) % pts.size()]);
        CHECK(r - mid.norm() <= tol * (1 + 1e-9));
        CHECK(std::abs(pts[i].norm() - r) < 1e-9);
    }
}

TEST_CASE("point containment", "[geom2d][polyline]") {
    const Polyline c = Polyline::circle({0, 0}, 5);
    CHECK(c.contains({0, 0}));
    CHECK(c.contains({4.9, 0}));
    CHECK_FALSE(c.contains({3.6, 3.6}));
    Polyline cw = c;
    cw.reverse();
    CHECK(cw.contains({0, 0}));
    CHECK_FALSE(Polyline::fromPoints({{0, 0}, {1, 0}}, false).contains({0.5, 0}));
}
