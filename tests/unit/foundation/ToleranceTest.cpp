// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/foundation/Tolerance.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace ocam;

TEST_CASE("scalar nearlyEqual: absolute and relative", "[tolerance]") {
    STATIC_REQUIRE(nearlyEqual(1.0, 1.0 + 1e-7, 1e-6));
    STATIC_REQUIRE_FALSE(nearlyEqual(1.0, 1.0 + 1e-5, 1e-6));
    // Relative tolerance dominates for large magnitudes.
    CHECK(nearlyEqual(1e9, 1e9 + 1.0, 1e-6, 1e-8));
    CHECK_FALSE(nearlyEqual(1e9, 1e9 + 100.0, 1e-6, 1e-8));
    CHECK(nearlyEqual(-2.0, -2.0, 0.0));
}

TEST_CASE("vector nearlyEqual uses Euclidean distance", "[tolerance]") {
    const Vec3 a{1, 2, 3};
    CHECK(nearlyEqual(a, Vec3{1, 2, 3 + 5e-7}, 1e-6));
    CHECK_FALSE(nearlyEqual(a, Vec3{1 + 1e-6, 2 + 1e-6, 3}, 1e-6));
    CHECK(nearlyEqual(Vec2{0, 0}, Vec2{3e-7, 4e-7}, 5e-7));
}

TEST_CASE("nearlyParallel", "[tolerance]") {
    const Vec3 z = Vec3::UnitZ();
    CHECK(nearlyParallel(z, z, 1e-9));
    CHECK(nearlyParallel(z, Vec3{1e-10, 0, 1}.normalized(), 1e-9));
    CHECK_FALSE(nearlyParallel(z, -z, 1e-9));
    CHECK_FALSE(nearlyParallel(z, Vec3::UnitX(), 1e-9));
}
