// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/geom2d/Clip.hpp>
#include <ocam/geom2d/Region.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace ocam;
using namespace ocam::geom2d;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

Region square(double x0, double y0, double size) {
    return Region{Polyline::rectangle({x0, y0}, {x0 + size, y0 + size}), {}};
}

} // namespace

TEST_CASE("buildRegions nests outers, holes and islands", "[geom2d][region]") {
    std::vector<Polyline> loops;
    loops.push_back(Polyline::circle({50, 50}, 20));          // hole
    loops.push_back(Polyline::rectangle({45, 45}, {55, 55})); // island in the hole
    Polyline outer = Polyline::rectangle({0, 0}, {100, 100});
    outer.reverse(); // orientation is ignored
    loops.push_back(outer);
    loops.push_back(Polyline::rectangle({200, 0}, {210, 10}));      // separate part
    loops.push_back(Polyline::fromPoints({{0, 0}, {1, 1}}, false)); // open: dropped

    const Regions regions = buildRegions(loops);
    REQUIRE(regions.size() == 3);

    // Largest first.
    CHECK(regions[0].holes.size() == 1);
    CHECK_THAT(regions[0].area(), WithinRel(10000.0 - kPi * 400.0, 1e-12));
    CHECK(regions[0].outer.isCounterClockwise());
    CHECK_FALSE(regions[0].holes[0].isCounterClockwise());
    CHECK_THAT(totalArea(regions), WithinRel(10000.0 - kPi * 400.0 + 100.0 + 100.0, 1e-12));

    CHECK(regions[0].contains({10, 10}));
    CHECK_FALSE(regions[0].contains({50, 40}));
    CHECK_FALSE(regions[0].contains({50, 50})); // inside the island, which is its own region
}

TEST_CASE("offset grows and shrinks", "[geom2d][clip]") {
    const Regions sq{square(0, 0, 10)};
    const ClipOptions opts{1e-4};

    const Regions grown = offset(sq, 1.0, opts);
    REQUIRE(grown.size() == 1);
    CHECK_THAT(totalArea(grown), WithinAbs(100.0 + 40.0 + kPi, 2e-3));

    ClipOptions miter = opts;
    miter.join = JoinStyle::Miter;
    CHECK_THAT(totalArea(offset(sq, 1.0, miter)), WithinAbs(144.0, 1e-6));

    const Regions shrunk = offset(sq, -1.0, opts);
    REQUIRE(shrunk.size() == 1);
    CHECK_THAT(totalArea(shrunk), WithinAbs(64.0, 1e-6));

    CHECK(offset(sq, -5.1, opts).empty());
}

TEST_CASE("offset shrinks holes when the region grows", "[geom2d][clip]") {
    Polyline hole = Polyline::circle({50, 50}, 10);
    hole.reverse();
    const Regions plate{Region{Polyline::rectangle({0, 0}, {100, 100}), {hole}}};
    const Regions inset = offset(plate, -2.0, {1e-4});
    REQUIRE(inset.size() == 1);
    REQUIRE(inset[0].holes.size() == 1);
    CHECK_THAT(inset[0].holes[0].area(), WithinRel(kPi * 144.0, 1e-4));
    CHECK_THAT(inset[0].outer.area(), WithinAbs(96.0 * 96.0, 1e-6));
}

TEST_CASE("offset splits a region through a narrow neck", "[geom2d][clip]") {
    // Two 10x10 squares joined by a 2 mm wide bridge.
    const Regions dumbbell = unite(Regions{square(0, 0, 10), square(14, 0, 10)},
                                   Regions{Region{Polyline::rectangle({10, 4}, {14, 6}), {}}});
    REQUIRE(dumbbell.size() == 1);
    CHECK(offset(dumbbell, -1.5, {1e-4}).size() == 2);
}

TEST_CASE("booleans", "[geom2d][clip]") {
    const Regions a{square(0, 0, 10)};
    const Regions b{square(5, 5, 10)};
    CHECK_THAT(totalArea(unite(a, b)), WithinAbs(175.0, 1e-6));
    CHECK_THAT(totalArea(intersect(a, b)), WithinAbs(25.0, 1e-6));
    CHECK_THAT(totalArea(subtract(a, b)), WithinAbs(75.0, 1e-6));

    const Regions punched = subtract(Regions{square(0, 0, 10)}, Regions{square(3, 3, 4)});
    REQUIRE(punched.size() == 1);
    CHECK(punched[0].holes.size() == 1);
    CHECK_THAT(punched[0].area(), WithinAbs(84.0, 1e-6));
}

TEST_CASE("clipping open paths keeps their direction", "[geom2d][clip]") {
    const Regions box{square(0, 0, 10)};
    // A line across the square, right to left: the part inside runs right to left too.
    const auto across = clipPaths({{Vec2(15, 5), Vec2(-5, 5)}}, box, true);
    REQUIRE(across.size() == 1);
    CHECK_THAT(across[0].front().x(), WithinAbs(10.0, 1e-6));
    CHECK_THAT(across[0].back().x(), WithinAbs(0.0, 1e-6));
    // Outside: the two ends, each in the line's direction.
    const auto outside = clipPaths({{Vec2(15, 5), Vec2(-5, 5)}}, box, false);
    REQUIRE(outside.size() == 2);
    for (const auto& piece : outside) {
        CHECK(piece.front().x() > piece.back().x());
    }
    // A closed loop inside comes back whole, still clockwise.
    const std::vector<Vec2> loop{Vec2(2, 2), Vec2(2, 8), Vec2(8, 8), Vec2(8, 2), Vec2(2, 2)};
    const auto kept = clipPaths({loop}, box, true);
    REQUIRE(kept.size() == 1);
    CHECK((kept[0].front() - kept[0].back()).norm() < 1e-6);
    double area = 0.0;
    for (std::size_t i = 0; i + 1 < kept[0].size(); ++i) {
        area += kept[0][i].x() * kept[0][i + 1].y() - kept[0][i + 1].x() * kept[0][i].y();
    }
    CHECK(area < 0.0);
}
