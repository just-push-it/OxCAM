// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/sim/Dexel.hpp>
#include <ocam/sim/FixtureCheck.hpp>
#include <ocam/toolpath/PathBuilder.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>

using namespace ocam;
using namespace ocam::sim;

namespace {

bool has(const std::vector<FixtureCollision>& cs, const std::string& body,
         const std::string& part) {
    return std::any_of(cs.begin(), cs.end(), [&](const FixtureCollision& c) {
        return c.body == body && c.toolPart == part;
    });
}

/// A pass along X at height z, from x = -70 (beyond the jaw) to 40 at y.
toolpath::Toolpath pass(double y, double z) {
    toolpath::Toolpath tp;
    tp.start = Vec3(-70, y, 80);
    toolpath::PathBuilder b(tp);
    b.rapid(Vec3(-70, y, z + 5));
    b.feed(Vec3(-70, y, z), toolpath::FeedClass::Plunge, 5);
    b.feed(Vec3(40, y, z), toolpath::FeedClass::Cut, 10);
    b.rapid(Vec3(40, y, 80));
    return tp;
}

} // namespace

TEST_CASE("the tool assembly against fixtures", "[sim][fixtures]") {
    // A jaw: 100 long in X, Y from 10 to 30, its top at z = 0.
    const std::vector<FixtureBody> jaw{
        {"fixed jaw", boxMesh({Vec3(-50, 10, -30), Vec3(50, 30, 0)})}};
    tooling::Tool mill = tooling::makeFlatEndMill(1, 10.0, 20.0, 60.0);
    mill.stickout = 30.0;
    tooling::Holder chuck = tooling::makeHolder("chuck", {});
    mill.holderId = chuck.id;
    const ToolShape tool = ToolShape::fromTool(mill, &chuck);

    SECTION("a pass beside the jaw, the tool's side 0.5 mm off it") {
        CHECK(checkFixtures(pass(4.5, -5.0), jaw, tool).empty());
        // Within a 1 mm clearance it counts.
        FixtureCheckOptions options;
        options.clearance = 1.0;
        const auto near = checkFixtures(pass(4.5, -5.0), jaw, tool, options);
        REQUIRE(has(near, "fixed jaw", "tool"));
        CHECK_THAT(near.front().distance, Catch::Matchers::WithinAbs(0.5, 0.05));
    }
    SECTION("a pass into the jaw") {
        const auto cs = checkFixtures(pass(8.0, -5.0), jaw, tool);
        REQUIRE(has(cs, "fixed jaw", "tool"));
        // One run of the cut (move 2), not a collision per pose.
        CHECK(std::count_if(cs.begin(), cs.end(),
                            [](const FixtureCollision& c) { return c.toolPart == "tool"; }) == 1);
        CHECK(cs.front().firstMove == 2);
        CHECK(cs.front().distance == 0.0);
    }
    SECTION("the holder reaches down beside the jaw") {
        // The tool's side is 5 mm off the jaw; the holder face, 30 mm above the tip, is below the
        // jaw top and the chuck's nose (radius 16) reaches over the jaw.
        const auto cs = checkFixtures(pass(-5.0, -35.0), jaw, tool);
        CHECK(has(cs, "fixed jaw", "tool holder"));
        CHECK_FALSE(has(cs, "fixed jaw", "tool"));
        CHECK_FALSE(has(cs, "fixed jaw", "tool shank"));
    }
    SECTION("above the jaw") {
        CHECK(checkFixtures(pass(20.0, 1.0), jaw, tool).empty());
        CHECK_FALSE(checkFixtures(pass(20.0, -1.0), jaw, tool).empty());
    }
    SECTION("a tilted tool leaning over the jaw") {
        toolpath::Toolpath tp = pass(0.0, -10.0);
        const Vec3 lean = Vec3(0, 1, 1).normalized(); // the shank leans over the jaw
        for (auto& m : tp.moves) {
            m.axis = lean;
        }
        CHECK_FALSE(checkFixtures(tp, jaw, tool).empty());
        for (auto& m : tp.moves) {
            m.axis = Vec3(0, -1, 1).normalized(); // away from it
        }
        CHECK(checkFixtures(tp, jaw, tool).empty());
    }
    SECTION("no fixtures, nothing to check") {
        CHECK(checkFixtures(pass(8.0, -5.0), {}, tool).empty());
    }
}
