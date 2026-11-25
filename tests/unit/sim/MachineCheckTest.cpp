// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/machine/Model.hpp>
#include <ocam/sim/Dexel.hpp>
#include <ocam/sim/MachineCheck.hpp>
#include <ocam/toolpath/PathBuilder.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

using namespace ocam;
using namespace ocam::sim;

namespace {

bool has(const std::vector<MachineCollision>& cs, const std::string& a, const std::string& b) {
    return std::any_of(cs.begin(), cs.end(),
                       [&](const MachineCollision& c) { return c.first == a && c.second == b; });
}

toolpath::Toolpath pocketDown(double bottom) {
    toolpath::Toolpath tp;
    tp.start = Vec3(0, 0, 60);
    toolpath::PathBuilder b(tp);
    b.rapid(Vec3(0, 0, 25));
    b.feed(Vec3(0, 0, bottom), toolpath::FeedClass::Plunge, 5);
    b.feed(Vec3(5, 0, bottom), toolpath::FeedClass::Cut, 10);
    b.rapid(Vec3(5, 0, 60));
    return tp;
}

} // namespace

TEST_CASE("machine components against the part, stock and tool", "[sim][machine]") {
    const machine::Machine& m5 = machine::genericMill5TableAC();
    // A 40 x 40 x 20 part on the table centre; the WCS at the table centre.
    const geom3d::Aabb3 box{Vec3(-20, -20, 0), Vec3(20, 20, 20)};
    const geom3d::Mesh part = boxMesh(box);
    tooling::Tool tool = tooling::makeFlatEndMill(1, 6.0, 15.0, 40.0);
    tool.stickout = 20.0; // the spindle nose is 20 mm above the tip (no holder)
    const ToolShape shape = ToolShape::fromTool(tool);
    const Transform3 wcs = Transform3::Identity();

    SECTION("clear") {
        const auto cs = checkMachine(m5, pocketDown(5.0), wcs, part, box, shape);
        CHECK(cs.empty());
    }
    SECTION("spindle nose reaches the part") {
        const auto cs = checkMachine(m5, pocketDown(-2.0), wcs, part, box, shape);
        CHECK(has(cs, "spindle nose", "part"));
        CHECK(has(cs, "spindle nose", "stock"));
        // The plunge to -2 is move 2 (index 1).
        CHECK(cs.front().firstMove <= 2);
    }
    SECTION("tool into the table") {
        tooling::Tool longTool = tooling::makeFlatEndMill(2, 6.0, 15.0, 100.0);
        longTool.stickout = 80.0;
        toolpath::Toolpath tp;
        tp.start = Vec3(100, 0, 60);
        toolpath::PathBuilder b(tp);
        b.rapid(Vec3(100, 0, 5));
        b.feed(Vec3(100, 0, -10), toolpath::FeedClass::Plunge, 5);
        const auto cs = checkMachine(m5, tp, wcs, part, box, ToolShape::fromTool(longTool));
        CHECK(has(cs, "rotary table", "tool"));
    }
    SECTION("machines without components are not checked") {
        CHECK(
            checkMachine(machine::genericMill3(), pocketDown(-2.0), wcs, part, box, shape).empty());
    }
}

TEST_CASE("tilted tools bring the head close to the trunnion", "[sim][machine]") {
    const machine::Machine& m5 = machine::genericMill5TableAC();
    tooling::Tool tool = tooling::makeBallEndMill(1, 6.0, 15.0, 60.0);
    tool.stickout = 20.0;
    const ToolShape shape = ToolShape::fromTool(tool);
    auto cut = [&](const Vec3& from, const Vec3& to, const Vec3& axis) {
        toolpath::Toolpath tp;
        tp.start = from - 40.0 * (to - from).normalized();
        toolpath::Move m;
        m.kind = toolpath::MoveKind::Rapid;
        m.feedClass = toolpath::FeedClass::Rapid;
        m.axis = axis.normalized();
        m.end = from;
        tp.moves.push_back(m);
        m.kind = toolpath::MoveKind::Linear;
        m.feedClass = toolpath::FeedClass::Cut;
        m.feedRate = 10;
        m.end = to;
        tp.moves.push_back(m);
        return tp;
    };
    // The top of a tall part on a riser, the tool tilted 30 degrees: well clear of everything.
    const geom3d::Aabb3 tall{Vec3(-20, -20, 100), Vec3(20, 20, 140)};
    const Vec3 tilt30(0, -0.5, std::sqrt(0.75));
    const auto clear = checkMachine(m5, cut(Vec3(-10, 0, 140), Vec3(10, 0, 140), tilt30),
                                    Transform3::Identity(), boxMesh(tall), tall, shape);
    CHECK(clear.empty());
    // The side of a flat part with the tool horizontal, far out along X: the table stands on
    // edge and the spindle housing (220 mm wide) comes down beside a trunnion wall.
    const geom3d::Aabb3 box{Vec3(-20, -20, 0), Vec3(20, 20, 20)};
    const auto edge = checkMachine(m5, cut(Vec3(150, -25, 10), Vec3(150, -20, 10), -Vec3::UnitY()),
                                   Transform3::Identity(), boxMesh(box), box, shape);
    const bool trunnion = std::any_of(edge.begin(), edge.end(), [](const MachineCollision& c) {
        return c.second.find("trunnion") != std::string::npos ||
               c.first.find("trunnion") != std::string::npos;
    });
    CHECK(trunnion);
}

namespace {

machine::ModelPart boxPart(const char* name, const char* carrier, const Vec3& lo, const Vec3& hi) {
    machine::Component c;
    c.min = lo;
    c.max = hi;
    machine::ModelPart p = machine::makePart(name, machine::componentMesh(c));
    p.carrier = carrier;
    return p;
}

toolpath::Toolpath line(const Vec3& from, const Vec3& to) {
    toolpath::Toolpath tp;
    tp.start = from + Vec3(0, 0, 50);
    toolpath::PathBuilder b(tp);
    b.rapid(from);
    b.feed(to, toolpath::FeedClass::Cut, 10);
    return tp;
}

} // namespace

TEST_CASE("machine model parts move with their axes", "[sim][machine][model]") {
    // A trunnion machine of mesh parts: X and Y move the table, Z the head; a fixed column
    // behind the table at Y 400.
    machine::Machine m = machine::genericMill5TableAC();
    m.components.clear();
    m.model = machine::defaultModel(m);
    const double top = machine::kDefaultSpindleHeight;
    m.model.parts = {
        boxPart("column", "", Vec3(-200, 400, -600), Vec3(200, 600, 900)),
        boxPart("head", "Z", Vec3(-100, -100, top), Vec3(100, 400, top + 300)),
        boxPart("cradle", "A", Vec3(-280, -170, -260), Vec3(280, 170, -200)),
        boxPart("table", "C", Vec3(-160, -160, -30), Vec3(160, 160, 0)),
    };
    REQUIRE(m.validate().empty());
    tooling::Tool tool = tooling::makeFlatEndMill(1, 6.0, 15.0, 40.0);
    tool.stickout = 30.0;
    const ToolShape shape = ToolShape::fromTool(tool);
    const geom3d::Aabb3 box{Vec3(-20, -20, 0), Vec3(20, 20, 20)};
    const geom3d::Mesh part = boxMesh(box);
    const Transform3 wcs = Transform3::Identity();

    SECTION("near the table centre nothing touches; the head slides on the column") {
        CHECK(checkMachine(m, line(Vec3(-10, 0, 25), Vec3(10, 0, 25)), wcs, part, box, shape)
                  .empty());
    }
    SECTION("the table travels into the column") {
        // The tool at Y -300 of the table moves the table 300 mm towards the column.
        const auto cs =
            checkMachine(m, line(Vec3(0, -100, 25), Vec3(0, -300, 25)), wcs, part, box, shape);
        CHECK(has(cs, "column", "cradle"));
        CHECK(has(cs, "column", "table"));
        CHECK_FALSE(has(cs, "head", "column"));
    }
    SECTION("the head comes down on the part") {
        const auto cs =
            checkMachine(m, line(Vec3(0, 0, 25), Vec3(0, 0, -15)), wcs, part, box, shape);
        CHECK(has(cs, "head", "part"));
        CHECK(has(cs, "head", "stock"));
    }
    SECTION("hidden parts are still checked") {
        for (auto& p : m.model.parts) {
            p.shown = false;
        }
        const auto cs =
            checkMachine(m, line(Vec3(0, -100, 25), Vec3(0, -300, 25)), wcs, part, box, shape);
        CHECK(has(cs, "column", "cradle"));
    }
    SECTION("parts that do not collide are left out") {
        for (auto& p : m.model.parts) {
            p.collides = false;
        }
        CHECK(checkMachine(m, line(Vec3(0, -100, 25), Vec3(0, -300, 25)), wcs, part, box, shape)
                  .empty());
    }
}

TEST_CASE("fixtures against the machine", "[sim][machine]") {
    // The spindle nose (radius 45) 30 mm above the tip, a clamp 40 mm tall beside the part.
    const machine::Machine& m5 = machine::genericMill5TableAC();
    tooling::Tool tool = tooling::makeFlatEndMill(1, 6.0, 15.0, 40.0);
    tool.stickout = 30.0;
    const ToolShape shape = ToolShape::fromTool(tool);
    const geom3d::Aabb3 box{Vec3(-20, -20, 0), Vec3(20, 20, 20)};
    const std::vector<FixtureBody> clamp{
        {"clamp", boxMesh(geom3d::Aabb3{Vec3(30, -30, 0), Vec3(60, 30, 40)})}};
    const machine::WorkMount wcs =
        machine::tableMount(machine::effectiveModel(m5), Transform3::Identity());
    // Cutting the part's top: the nose passes 5 mm over the clamp.
    CHECK(checkMachine(m5, line(Vec3(-10, 0, 15), Vec3(20, 0, 15)), wcs, boxMesh(box), box, clamp,
                       shape)
              .empty());
    // Deeper, the nose comes down on the clamp.
    const auto cs = checkMachine(m5, line(Vec3(-10, 0, 5), Vec3(20, 0, 5)), wcs, boxMesh(box), box,
                                 clamp, shape);
    CHECK(has(cs, "spindle nose", "clamp"));
    // The tool itself against the clamp is checkFixtures' (FixtureCheck.hpp).
    const auto through = checkMachine(m5, line(Vec3(45, 0, 70), Vec3(45, 0, 35)), wcs, boxMesh(box),
                                      box, clamp, shape);
    CHECK(through.empty());
}
