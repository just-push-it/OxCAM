// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/geom3d/Bvh.hpp>
#include <ocam/geom3d/Convex.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <random>

using namespace ocam;
using namespace ocam::geom3d;
using Catch::Matchers::WithinAbs;

TEST_CASE("GJK distances between simple solids", "[geom3d][convex]") {
    const Convex unit = Convex::box({Vec3(0, 0, 0), Vec3(1, 1, 1)});

    SECTION("separated boxes") {
        const Convex other = Convex::box({Vec3(3, 0.5, 0.2), Vec3(4, 2, 3)});
        const ConvexDistance d = distance(unit, other);
        CHECK_FALSE(d.intersecting);
        CHECK_THAT(d.distance, WithinAbs(2.0, 1e-9));
        CHECK_THAT(d.pointA.x(), WithinAbs(1.0, 1e-9));
        CHECK_THAT(d.pointB.x(), WithinAbs(3.0, 1e-9));
    }
    SECTION("diagonal corner distance") {
        const Convex other = Convex::box({Vec3(2, 2, 2), Vec3(3, 3, 3)});
        CHECK_THAT(distance(unit, other).distance, WithinAbs(std::sqrt(3.0), 1e-9));
    }
    SECTION("overlapping boxes") {
        const Convex other = Convex::box({Vec3(0.5, 0.5, 0.5), Vec3(3, 3, 3)});
        CHECK(distance(unit, other).intersecting);
        CHECK(intersects(unit, other));
    }
    SECTION("sphere and capsule margins") {
        const Convex s = Convex::sphere(Vec3(0.5, 0.5, 4.0), 1.0);
        CHECK_THAT(distance(unit, s).distance, WithinAbs(2.0, 1e-9));
        CHECK(intersects(unit, s, 2.5));
        CHECK_FALSE(intersects(unit, s, 1.5));
        const Convex c = Convex::capsule(Vec3(-5, 0.5, 2.5), Vec3(5, 0.5, 2.5), 1.0);
        CHECK_THAT(distance(unit, c).distance, WithinAbs(0.5, 1e-9));
        CHECK(intersects(unit, Convex::capsule(Vec3(-5, 0.5, 1.8), Vec3(5, 0.5, 1.8), 1.0)));
    }
    SECTION("cylinder support") {
        // Cylinder along X at height 3 with radius 1: its bottom is 1 above the box.
        const Convex cyl = Convex::cylinder(Vec3(-2, 0.5, 3), Vec3::UnitX(), 1.0, 10.0);
        CHECK_THAT(distance(unit, cyl).distance, WithinAbs(1.0, 1e-6));
        // Tilted 45 degrees from (3, 0.5, 0): the nearest point is the rim of the base disk
        // towards the box, 0.5 / sqrt(2) left of and above the base centre.
        const Vec3 axis = Vec3(1, 0, 1).normalized();
        const Convex tilted = Convex::cylinder(Vec3(3, 0.5, 0), axis, 0.5, 5.0);
        CHECK_THAT(distance(unit, tilted).distance, WithinAbs(2.0 - 0.5 / std::sqrt(2.0), 1e-6));
        const Aabb3 b = cyl.bounds();
        CHECK_THAT(b.min.y(), WithinAbs(-0.5, 1e-12));
        CHECK_THAT(b.max.x(), WithinAbs(8.0, 1e-12));
    }
    SECTION("triangle against capsule") {
        const Convex tri = Convex::triangle(Vec3(0, 0, 0), Vec3(10, 0, 0), Vec3(0, 10, 0));
        CHECK_THAT(distance(tri, Convex::capsule(Vec3(2, 2, 3), Vec3(2, 2, 10), 1.0)).distance,
                   WithinAbs(2.0, 1e-9));
        CHECK_THAT(distance(tri, Convex::sphere(Vec3(8, 8, 0), 0.5)).distance,
                   WithinAbs(std::sqrt(18.0) - 0.5, 1e-9));
    }
}

TEST_CASE("GJK agrees with brute force on random boxes", "[geom3d][convex]") {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> u(-3.0, 3.0);
    for (int trial = 0; trial < 200; ++trial) {
        Aabb3 a, b;
        a.extend(Vec3(u(rng), u(rng), u(rng)));
        a.extend(Vec3(u(rng), u(rng), u(rng)));
        b.extend(Vec3(u(rng), u(rng), u(rng)));
        b.extend(Vec3(u(rng), u(rng), u(rng)));
        // Axis-aligned boxes: the gap per axis gives the exact distance.
        Vec3 gap;
        for (int k = 0; k < 3; ++k) {
            gap[k] = std::max({0.0, b.min[k] - a.max[k], a.min[k] - b.max[k]});
        }
        const ConvexDistance d = distance(Convex::box(a), Convex::box(b));
        CHECK_THAT(d.distance, WithinAbs(gap.norm(), 1e-7));
        CHECK(d.intersecting == (gap.norm() == 0.0));
    }
}

namespace {

Vec3 closestOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    // Ericson 5.1.5.
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const double d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0 && d2 <= 0)
        return a;
    const Vec3 bp = p - b;
    const double d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0 && d4 <= d3)
        return b;
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0)
        return a + d1 / (d1 - d3) * ab;
    const Vec3 cp = p - c;
    const double d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0 && d5 <= d6)
        return c;
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0)
        return a + d2 / (d2 - d6) * ac;
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        return b + (d4 - d3) / ((d4 - d3) + (d5 - d6)) * (c - b);
    const double denom = 1.0 / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

} // namespace

TEST_CASE("GJK capsule-triangle distances agree with brute force", "[geom3d][convex]") {
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    int checked = 0;
    for (int trial = 0; trial < 3000; ++trial) {
        const Vec3 a(u(rng) * 3, u(rng) * 3, u(rng)), b(u(rng) * 3, u(rng) * 3, u(rng)),
            c(u(rng) * 3, u(rng) * 3, u(rng));
        // Segments placed near the triangle, many nearly touching.
        const Vec3 base = (a + b + c) / 3.0 + Vec3(u(rng), u(rng), 1.0 + u(rng));
        const Vec3 dir = Vec3(u(rng), u(rng), u(rng) + (trial % 2 ? 0.2 : -0.2)).normalized();
        const Vec3 s0 = base, s1 = base + dir * 5.0;
        double brute = std::numeric_limits<double>::infinity();
        for (int k = 0; k <= 4000; ++k) {
            const Vec3 p = s0 + (s1 - s0) * (k / 4000.0);
            brute = std::min(brute, (closestOnTriangle(p, a, b, c) - p).norm());
        }
        const double radius = std::max(0.0, brute + 0.01 * u(rng));
        const ConvexDistance d =
            distance(Convex::capsule(s0, s1, radius), Convex::triangle(a, b, c));
        const double expected = std::max(0.0, brute - radius);
        INFO("trial " << trial << " brute " << brute << " radius " << radius);
        CHECK(d.distance <= expected + 1e-9);
        CHECK(d.distance >= expected - 2e-3); // sampling of the brute force
        if (brute - radius > 1e-3) {
            CHECK_FALSE(d.intersecting);
            ++checked;
        }
    }
    CHECK(checked > 500);
}

TEST_CASE("triangle BVH queries and ray casts", "[geom3d][bvh]") {
    // A 20 x 20 grid of unit squares in z = 0 (800 triangles) plus a raised square at z = 5.
    Mesh m;
    for (int j = 0; j <= 20; ++j) {
        for (int i = 0; i <= 20; ++i) {
            m.vertices.emplace_back(i, j, 0.0);
        }
    }
    for (int j = 0; j < 20; ++j) {
        for (int i = 0; i < 20; ++i) {
            const auto v = static_cast<std::uint32_t>(j * 21 + i);
            m.triangles.push_back({v, v + 1, v + 22});
            m.triangles.push_back({v, v + 22, v + 21});
        }
    }
    const auto base = static_cast<std::uint32_t>(m.vertices.size());
    m.vertices.emplace_back(5, 5, 5);
    m.vertices.emplace_back(7, 5, 5);
    m.vertices.emplace_back(7, 7, 5);
    m.triangles.push_back({base, base + 1, base + 2});
    const TriangleBvh bvh(m);

    std::size_t count = 0;
    bvh.query({Vec3(2.5, 2.5, -1), Vec3(4.5, 3.5, 1)}, [&](std::size_t) {
        ++count;
        return true;
    });
    // Squares overlapping x in [2.5, 4.5], y in [2.5, 3.5]: 3 x 2 squares, both triangles of
    // each have overlapping boxes except where a triangle's box misses (all overlap here).
    CHECK(count == 12);

    const auto hit = bvh.raycast(Vec3(6.5, 5.5, 10), Vec3(0, 0, -1));
    REQUIRE(hit);
    CHECK_THAT(hit->t, WithinAbs(5.0, 1e-12));
    CHECK(hit->triangle == m.triangles.size() - 1);
    const auto ground = bvh.raycast(Vec3(1.3, 17.2, 10), Vec3(0, 0, -1));
    REQUIRE(ground);
    CHECK_THAT(ground->point.z(), WithinAbs(0.0, 1e-12));
    CHECK_FALSE(bvh.raycast(Vec3(30, 30, 10), Vec3(0, 0, -1)));
    CHECK_FALSE(bvh.raycast(Vec3(6.5, 5.5, 10), Vec3(0, 0, -1), 4.0));
}

TEST_CASE("GJK regression: capsule above a triangle", "[geom3d][convex]") {
    const Vec3 c(-3, -10, 20.486);
    const Vec3 a = Vec3(-0.396777, -0.418254, 0.817087).normalized();
    const Convex body = Convex::capsule(c + 1e-3 * a, c + 12.0 * a, 3.0 - 2e-3);
    const Convex tri =
        Convex::triangle(Vec3(-4.16667, -8.33333, 17.6973), Vec3(-3.33333, -8.33333, 17.873),
                         Vec3(-3.33333, -7.5, 18.2384));
    double brute = std::numeric_limits<double>::infinity();
    for (int k = 0; k <= 20000; ++k) {
        const Vec3 p = c + 1e-3 * a + (12.0 - 1e-3) * a * (k / 20000.0);
        brute = std::min(brute, (closestOnTriangle(p, Vec3(-4.16667, -8.33333, 17.6973),
                                                   Vec3(-3.33333, -8.33333, 17.873),
                                                   Vec3(-3.33333, -7.5, 18.2384)) -
                                 p)
                                    .norm());
    }
    const ConvexDistance d = distance(body, tri);
    INFO("brute " << brute << " gjk " << d.distance << " intersecting " << d.intersecting);
    CHECK_FALSE(d.intersecting);
    CHECK_THAT(d.distance, WithinAbs(brute - (3.0 - 2e-3), 1e-6));
}
