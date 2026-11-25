// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Back boring and 3D edge breaking in the dexel simulation (DL-0083): what they remove against
// the analytic shapes, and no collisions on the way.

#include <ocam/sim/Dexel.hpp>
#include <ocam/sim/Simulate.hpp>
#include <ocam/sim/ToolShape.hpp>
#include <ocam/strategies/BackBore.hpp>
#include <ocam/strategies/EdgeBreak.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

using namespace ocam;
using namespace ocam::sim;
using toolpath::FeedClass;
using toolpath::Move;
using toolpath::MoveKind;

namespace {

geom3d::Aabb3 box(const Vec3& lo, const Vec3& hi) {
    geom3d::Aabb3 b;
    b.extend(lo);
    b.extend(hi);
    return b;
}

double relError(double value, double expected) {
    return std::abs(value - expected) / expected;
}

/// A plate from z = -20 to 0 around (20, 20), with a Ø10 hole through it at (20, 20).
Stock drilledPlate(double size, double resolution) {
    Stock s =
        Stock::box(box({20 - size / 2, 20 - size / 2, -20}, {20 + size / 2, 20 + size / 2, 0}),
                   resolution, false);
    toolpath::Toolpath plunge;
    plunge.start = {20, 20, 5};
    Move m;
    m.kind = MoveKind::Linear;
    m.feedRate = 10.0;
    m.end = {20, 20, -30};
    plunge.moves.push_back(m);
    simulateToolpath(s, ToolShape::fromTool(tooling::makeFlatEndMill(9, 10.0, 40.0, 80.0)), plunge);
    return s;
}

strategies::HoleTarget throughHole() {
    return {{20.0, 20.0}, 0.0, 20.0, true, 10.0};
}

struct Ctx {
    CancelToken cancel;
    NullProgress progress;
    strategies::Context of(const tooling::Tool& tool) {
        strategies::Feeds feeds;
        feeds.cut = 20.0;
        feeds.plunge = 5.0;
        feeds.lead = 15.0;
        feeds.spindle = 100.0;
        return strategies::Context{tool, feeds, {15.0, 5.0, 1.0}, 0.002, cancel, progress};
    }
};

} // namespace

TEST_CASE("a back bore removes its counterbore and passes the hole", "[sim][backbore]") {
    Ctx c;
    SECTION("oriented: a back-boring bar") {
        tooling::Tool bar = tooling::makeDefaultTool(tooling::ToolType::BoringBar, 1, 13.0);
        bar.mill.diameter = 13.0;
        bar.mill.fluteLength = 3.0;
        bar.mill.neckDiameter = 5.0;
        bar.mill.neckLength = 40.0;
        bar.mill.overallLength = 80.0;
        bar.mill.shankDiameter = 0.0;
        strategies::BackBoreParams p;
        p.depth = 3.0;
        p.shift = 2.0;
        const auto tp = strategies::backBore({throughHole()}, p, c.of(bar));
        Stock s = drilledPlate(20.0, 0.05);
        const double before = s.volume();
        const auto r = simulateToolpath(s, ToolShape::fromTool(bar), tp);
        CHECK(r.collisions.empty()); // the neck passes the hole the shank could not
        CHECK(relError(before - s.volume(), kPi * (6.5 * 6.5 - 25.0) * 3.0) < 0.02);
    }
    SECTION("interpolated: a slot cutter") {
        tooling::Tool cutter = tooling::makeDefaultTool(tooling::ToolType::SlotCutter, 2, 8.0);
        cutter.mill.diameter = 8.0;
        cutter.mill.fluteLength = 2.0;
        cutter.mill.neckDiameter = 2.0;
        cutter.mill.neckLength = 35.0;
        cutter.mill.overallLength = 70.0;
        cutter.mill.shankDiameter = 8.0;
        strategies::BackBoreParams p;
        p.method = strategies::BackBoreMethod::Interpolated;
        p.depth = 2.0;
        p.diameter = 15.0;
        p.helixPitch = 0.5;
        const auto tp = strategies::backBore({throughHole()}, p, c.of(cutter));
        Stock s = drilledPlate(24.0, 0.05);
        const double before = s.volume();
        const auto r = simulateToolpath(s, ToolShape::fromTool(cutter), tp);
        CHECK(r.collisions.empty());
        CHECK(relError(before - s.volume(), kPi * (7.5 * 7.5 - 25.0) * 2.0) < 0.02);
    }
    SECTION("interpolated: a lollipop breaks the far edge") {
        tooling::Tool ball = tooling::makeDefaultTool(tooling::ToolType::Lollipop, 3, 6.0);
        ball.mill.diameter = 6.0;
        ball.mill.neckDiameter = 3.0;
        ball.mill.neckLength = 35.0;
        ball.mill.fluteLength = 6.0;
        ball.mill.overallLength = 70.0;
        ball.mill.shankDiameter = 6.0;
        strategies::BackBoreParams p;
        p.method = strategies::BackBoreMethod::Interpolated;
        p.feature = strategies::BackFeature::Chamfer;
        p.chamferWidth = 0.5;
        const auto tp = strategies::backBore({throughHole()}, p, c.of(ball));
        // Where the ball circles: the envelope of the sphere, integrated over the far edge.
        double tip = 0.0;
        double rho = 0.0;
        for (const Move& m : tp.moves) {
            if (m.feedClass == FeedClass::Cut) {
                tip = m.end.z();
                rho = (m.end.head<2>() - Vec2(20.0, 20.0)).norm();
            }
        }
        double expected = 0.0;
        const int n = 2000;
        for (int k = 0; k < n; ++k) {
            const double z = -20.0 + (k + 0.5) * 1.0 / n;
            const double dz = z - (tip + 3.0);
            const double reach = dz * dz < 9.0 ? rho + std::sqrt(9.0 - dz * dz) : 0.0;
            expected += reach > 5.0 ? kPi * (reach * reach - 25.0) * (1.0 / n) : 0.0;
        }
        Stock s = drilledPlate(16.0, 0.02);
        const double before = s.volume();
        const auto r = simulateToolpath(s, ToolShape::fromTool(ball), tp);
        CHECK(r.collisions.empty());
        CHECK(relError(before - s.volume(), expected) < 0.05);
    }
}

TEST_CASE("a 3D edge break removes its prism along a sloped edge", "[sim][edgebreak]") {
    Ctx c;
    const tooling::Tool mill = tooling::makeChamferMill(1, 8.0, 0.0, degToRad(90.0), 50.0);
    // A wedge: top from z = 12 at x = 0 down to z = 2 at x = 40, y from 0 to 20.
    geom3d::Mesh wedge;
    const std::vector<Vec2> xz{{0, 0}, {40, 0}, {40, 2}, {0, 12}};
    for (const double y : {0.0, 20.0}) {
        for (const Vec2& p : xz) {
            wedge.vertices.emplace_back(p.x(), y, p.y());
        }
    }
    wedge.triangles = {{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {0, 4, 5}, {0, 5, 1},
                       {1, 5, 6}, {1, 6, 2}, {2, 6, 7}, {2, 7, 3}, {3, 7, 4}, {3, 4, 0}};
    if (wedge.signedVolume() < 0.0) {
        for (auto& t : wedge.triangles) {
            std::swap(t[1], t[2]);
        }
    }
    REQUIRE(std::abs(wedge.signedVolume() - 0.5 * (12.0 + 2.0) * 40.0 * 20.0) < 1e-6);
    // The front edge of the slope.
    const Vec3 a(0, 0, 12);
    const Vec3 b(40, 0, 2);
    const Vec3 t = (b - a).normalized();
    const Vec3 nTop = Vec3(0.25, 0.0, 1.0).normalized();
    const Vec3 nFront = -Vec3::UnitY();
    const auto inFace = [&](const Vec3& n, const Vec3& other) {
        Vec3 u = n.cross(t).normalized();
        return u.dot(other) > 0.0 ? Vec3(-u) : u;
    };
    strategies::BreakEdge edge;
    edge.name = "front";
    edge.faces = {0, 0};
    for (int k = 0; k <= 80; ++k) {
        strategies::EdgeSample s;
        s.point = a + (b - a) * (k / 80.0);
        s.tangent = t;
        s.normal = {nTop, nFront};
        s.inFace = {inFace(nTop, nFront), inFace(nFront, nTop)};
        edge.samples.push_back(s);
    }
    strategies::EdgeBreakParams p;
    p.width = 1.0;
    p.tipOffset = 0.3;
    const auto tp = strategies::edgeBreak({edge}, {wedge}, p, c.of(mill));
    REQUIRE(tp.warnings.empty());
    // The break's cross-section: the triangle between the edge and where it meets the faces.
    const auto pose = strategies::breakPose(mill, edge.samples[40], p.width, p.tipOffset);
    REQUIRE(pose);
    const Vec3 ea = pose->faceWidth[0] * edge.samples[40].inFace[0];
    const Vec3 eb = pose->faceWidth[1] * edge.samples[40].inFace[1];
    const double area = 0.5 * ea.cross(eb).norm();
    const double expected = area * (b - a).norm();

    Stock s = Stock::fromMesh(wedge, wedge.bounds(), 0.05, false);
    const double before = s.volume();
    const auto r = simulateToolpath(s, ToolShape::fromTool(mill), tp);
    CHECK(r.collisions.empty());
    CHECK(relError(before - s.volume(), expected) < 0.05);
}
