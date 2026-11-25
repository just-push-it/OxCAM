// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/geom2d/Ordering.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <numeric>
#include <random>

using namespace ocam;
using namespace ocam::geom2d;
using Catch::Matchers::WithinAbs;

TEST_CASE("ordering returns a permutation", "[geom2d][ordering]") {
    CHECK(orderPoints({}, {0, 0}).empty());

    std::mt19937 rng(1234);
    std::uniform_real_distribution<double> u(0.0, 100.0);
    std::vector<Vec2> pts(60);
    for (auto& p : pts) {
        p = {u(rng), u(rng)};
    }
    auto order = orderPoints(pts, {0, 0});
    REQUIRE(order.size() == pts.size());
    std::sort(order.begin(), order.end());
    std::vector<std::size_t> expected(pts.size());
    std::iota(expected.begin(), expected.end(), std::size_t{0});
    CHECK(order == expected);
}

TEST_CASE("2-opt never lengthens the nearest-neighbour path", "[geom2d][ordering]") {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> u(0.0, 100.0);
    for (int trial = 0; trial < 20; ++trial) {
        std::vector<Vec2> pts(40);
        for (auto& p : pts) {
            p = {u(rng), u(rng)};
        }
        const auto nnOnly = orderPoints(pts, {0, 0}, 0);
        const auto improved = orderPoints(pts, {0, 0});
        CHECK(pathLength(pts, improved, {0, 0}) <= pathLength(pts, nnOnly, {0, 0}) + 1e-9);
        CHECK(orderPoints(pts, {0, 0}) == improved); // deterministic
    }
}

TEST_CASE("collinear points are visited in order", "[geom2d][ordering]") {
    const std::vector<Vec2> pts{{3, 0}, {1, 0}, {4, 0}, {2, 0}};
    const auto order = orderPoints(pts, {0, 0});
    CHECK(order == std::vector<std::size_t>{1, 3, 0, 2});
    CHECK_THAT(pathLength(pts, order, {0, 0}), WithinAbs(4.0, 1e-12));
}
