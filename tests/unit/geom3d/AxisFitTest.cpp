// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/geom3d/AxisFit.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>

using namespace ocam;
using namespace ocam::geom3d;

namespace {

/// A welded, closed frustum (r0 at the base, r1 at the top; a cylinder when equal) of `sides`
/// facets around `axis` through `base`, `length` long, covering `arc` radians of the side only
/// when arc < 2 pi (an open band, no caps).
Mesh frustum(const Vec3& base, const Vec3& axis, double r0, double r1, double length, int sides,
             double arc = 2.0 * kPi) {
    const Vec3 w = axis.normalized();
    const Vec3 u = w.unitOrthogonal();
    const Vec3 v = w.cross(u);
    const bool full = arc >= 2.0 * kPi - 1e-12;
    const int ring = full ? sides : sides + 1;
    Mesh m;
    for (int end = 0; end < 2; ++end) {
        const double r = end ? r1 : r0;
        for (int k = 0; k < ring; ++k) {
            const double t = arc * k / sides;
            m.vertices.push_back(base + end * length * w + r * (std::cos(t) * u + std::sin(t) * v));
        }
    }
    const auto top = static_cast<std::uint32_t>(ring);
    for (int k = 0; k < sides; ++k) {
        const auto a = static_cast<std::uint32_t>(k);
        const auto b = static_cast<std::uint32_t>((k + 1) % ring);
        m.triangles.push_back({a, b, top + b});
        m.triangles.push_back({a, top + b, top + a});
    }
    if (full) {
        const auto c0 = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.push_back(base);
        m.vertices.push_back(base + length * w);
        for (int k = 0; k < sides; ++k) {
            const auto a = static_cast<std::uint32_t>(k);
            const auto b = static_cast<std::uint32_t>((k + 1) % ring);
            m.triangles.push_back({c0, b, a});
            m.triangles.push_back({c0 + 1, top + a, top + b});
        }
    }
    return m;
}

/// Distance of `p` from the line through `q` along unit `d`.
double offLine(const Vec3& p, const Vec3& q, const Vec3& d) {
    const Vec3 x = p - q;
    return (x - x.dot(d) * d).norm();
}

} // namespace

TEST_CASE("the axis of a cylinder picked on a mesh", "[geom3d][axisfit]") {
    const Vec3 base(100, -40, 25);
    const Vec3 axis = Vec3(1, 0.2, -0.1).normalized();
    const Mesh m = frustum(base, axis, 60.0, 60.0, 80.0, 48);
    const auto fit = revolutionAxis(m, 7); // a side facet
    REQUIRE(fit);
    CHECK(std::abs(std::abs(fit->direction.dot(axis)) - 1.0) < 1e-9);
    CHECK(offLine(fit->point, base, axis) < 1e-6);
    CHECK(std::abs((fit->point - base).dot(axis) - 40.0) < 1e-6); // the face's middle
    CHECK(std::abs(fit->radius - 60.0 * std::cos(kPi / 48)) < 0.1);
    CHECK(fit->triangles == 96);
    // A cap is flat: no axis.
    CHECK_FALSE(revolutionAxis(m, 96 + 3));
}

TEST_CASE("the axis of a cone and of a narrow arc", "[geom3d][axisfit]") {
    const Vec3 base(0, 0, 0);
    const Mesh cone = frustum(base, Vec3::UnitX(), 50.0, 20.0, 40.0, 64);
    const auto c = revolutionAxis(cone, 5);
    REQUIRE(c);
    CHECK(std::abs(std::abs(c->direction.x()) - 1.0) < 1e-9);
    CHECK(offLine(c->point, base, Vec3::UnitX()) < 1e-6);
    // 50 degrees of a cylinder about Y.
    const Mesh arc = frustum(Vec3(10, 0, 5), Vec3::UnitY(), 120.0, 120.0, 30.0, 12, 0.87);
    const auto a = revolutionAxis(arc, 4);
    REQUIRE(a);
    CHECK(std::abs(std::abs(a->direction.y()) - 1.0) < 1e-9);
    CHECK(offLine(a->point, Vec3(10, 0, 5), Vec3::UnitY()) < 1e-6);
}

TEST_CASE("the circle through three rim points", "[geom3d][axisfit]") {
    const Vec3 centre(5, -3, 12);
    const double r = 35;
    auto at = [&](double t) -> Vec3 {
        return centre + r * Vec3(0, std::cos(t), std::sin(t));
    };
    const auto c = circleThrough(at(0.1), at(1.7), at(4.0));
    REQUIRE(c);
    CHECK((c->point - centre).norm() < 1e-9);
    CHECK(std::abs(c->radius - r) < 1e-9);
    CHECK((c->direction - Vec3::UnitX()).norm() < 1e-9); // counter-clockwise seen from +X
    CHECK_FALSE(circleThrough(Vec3::Zero(), Vec3(1, 1, 1), Vec3(2, 2, 2)));
}
