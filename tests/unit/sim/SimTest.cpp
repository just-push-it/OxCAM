// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/foundation/Error.hpp>
#include <ocam/sim/Compare.hpp>
#include <ocam/sim/Dexel.hpp>
#include <ocam/sim/Rest.hpp>
#include <ocam/sim/Simulate.hpp>
#include <ocam/sim/ToolShape.hpp>
#include <ocam/strategies/Strategies.hpp>
#include <ocam/strategies/Surface.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace ocam;
using namespace ocam::sim;
using Catch::Approx;
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

/// Small helper to write toolpaths by hand.
struct PathWriter {
    toolpath::Toolpath path;
    explicit PathWriter(const Vec3& start) { path.start = start; }
    PathWriter& rapid(const Vec3& p) {
        Move m;
        m.kind = MoveKind::Rapid;
        m.feedClass = FeedClass::Rapid;
        m.end = p;
        path.moves.push_back(m);
        return *this;
    }
    PathWriter& line(const Vec3& p) {
        Move m;
        m.kind = MoveKind::Linear;
        m.end = p;
        m.feedRate = 10.0;
        path.moves.push_back(m);
        return *this;
    }
    PathWriter& arc(const Vec3& p, const Vec3& center, bool cw) {
        Move m;
        m.kind = cw ? MoveKind::ArcCW : MoveKind::ArcCCW;
        m.end = p;
        m.center = center;
        m.feedRate = 10.0;
        path.moves.push_back(m);
        return *this;
    }
};

/// Closed prism over a regular polygon (outward triangles).
geom3d::Mesh prism(const Vec2& c, double r, int n, double z0, double z1) {
    geom3d::Mesh m;
    for (int k = 0; k < n; ++k) {
        const double a = 2.0 * kPi * k / n;
        m.vertices.emplace_back(c.x() + r * std::cos(a), c.y() + r * std::sin(a), z0);
        m.vertices.emplace_back(c.x() + r * std::cos(a), c.y() + r * std::sin(a), z1);
    }
    const auto bc = static_cast<std::uint32_t>(m.vertices.size());
    m.vertices.emplace_back(c.x(), c.y(), z0);
    m.vertices.emplace_back(c.x(), c.y(), z1);
    for (int k = 0; k < n; ++k) {
        const auto b0 = static_cast<std::uint32_t>(2 * k), t0 = b0 + 1;
        const auto b1 = static_cast<std::uint32_t>(2 * ((k + 1) % n)), t1 = b1 + 1;
        m.triangles.push_back({b0, b1, t1});
        m.triangles.push_back({b0, t1, t0});
        m.triangles.push_back({bc, b1, b0});
        m.triangles.push_back({bc + 1, t0, t1});
    }
    return m;
}

ToolShape flat(double d, double flute = 25.0, double overall = 60.0) {
    return ToolShape::fromTool(tooling::makeFlatEndMill(1, d, flute, overall));
}

double relError(double value, double expected) {
    return std::abs(value - expected) / expected;
}

} // namespace

TEST_CASE("interval lists: subtract, intersect, unite, grow", "[sim][dexel]") {
    IntervalList l{{0, 10}};
    CHECK(subtract(l, 2, 4));
    CHECK(l == IntervalList{{0, 2}, {4, 10}});
    CHECK_FALSE(subtract(l, 2.5, 3.5));
    CHECK(subtract(l, 1, 6));
    CHECK(l == IntervalList{{0, 1}, {6, 10}});
    CHECK(subtract(l, -5, 7));
    CHECK(l == IntervalList{{7, 10}});
    CHECK(subtract(l, 9, 20));
    CHECK(l == IntervalList{{7, 9}});
    CHECK(subtract(l, 0, 100));
    CHECK(l.empty());
    CHECK_FALSE(subtract(l, 0, 1));

    const IntervalList a{{0, 4}, {6, 10}};
    const IntervalList b{{2, 7}};
    CHECK(intersect(a, b) == IntervalList{{2, 4}, {6, 7}});
    CHECK(unite(a, b) == IntervalList{{0, 10}});
    CHECK(grow(a, -1.5) == IntervalList{{1.5f, 2.5f}, {7.5f, 8.5f}});
    CHECK(grow(a, 1.0) == IntervalList{{-1, 11}});
    CHECK(totalLength(a) == Approx(8.0));
    CHECK(overlapLength(a, 3, 7) == Approx(2.0));
    CHECK(differenceLength(a, b) == Approx(5.0));
}

TEST_CASE("box stock: exact volume on all three grids", "[sim][dexel]") {
    const Stock s = Stock::box(box({0, 0, 0}, {100, 60, 30}), 0.5);
    CHECK(s.volume() == Approx(180000.0).epsilon(1e-9));
    CHECK(s.grid(0).volume() == Approx(180000.0).epsilon(1e-9));
    CHECK(s.grid(1).volume() == Approx(180000.0).epsilon(1e-9));
    // Faces exactly on ray coordinates: the fill rule counts shared edges once.
    const Stock t = Stock::box(box({0.25, 0.25, 0}, {10.25, 5.25, 3}), 0.5);
    CHECK(t.volume() == Approx(150.0).epsilon(0.12));
}

TEST_CASE("mesh rasterization matches the mesh volume", "[sim][dexel]") {
    const geom3d::Mesh m = prism({3.1, -2.7}, 12.0, 200, -5.0, 17.0);
    const double expected = m.signedVolume();
    const Stock s = Stock::fromMesh(m, m.bounds(), 0.1);
    CHECK(relError(s.volume(), expected) < 0.005);
    CHECK(relError(s.grid(0).volume(), expected) < 0.005);
    CHECK(relError(s.grid(1).volume(), expected) < 0.005);
}

TEST_CASE("snapshots are copy-on-write", "[sim][dexel]") {
    Stock s = Stock::box(box({0, 0, 0}, {50, 50, 10}), 0.5);
    const Stock before = s;
    PathWriter w({10, 10, 20});
    w.line({10, 10, 5}).line({40, 10, 5});
    simulateToolpath(s, flat(6), w.path);
    CHECK(before.volume() == Approx(25000.0).epsilon(1e-9));
    CHECK(s.volume() < before.volume() - 100.0);
}

TEST_CASE("removal matches analytic volumes at 0.1 mm", "[sim][removal]") {
    const auto stockBox = box({0, 0, 0}, {100, 60, 30});
    const double v0 = 100.0 * 60.0 * 30.0;

    SECTION("flat plunge") {
        Stock s = Stock::box(stockBox, 0.1);
        PathWriter w({50, 30, 40});
        w.line({50, 30, 25}).rapid({50, 30, 40});
        simulateToolpath(s, flat(10), w.path);
        const double expected = kPi * 25.0 * 5.0;
        CHECK(relError(v0 - s.volume(), expected) < 0.005);
    }
    SECTION("flat slot") {
        Stock s = Stock::box(stockBox, 0.1);
        PathWriter w({20, 30, 40});
        w.line({20, 30, 27}).line({70, 30, 27}).rapid({70, 30, 40});
        const auto r = simulateToolpath(s, flat(10), w.path);
        CHECK(r.collisions.empty());
        const double expected = 3.0 * (kPi * 25.0 + 50.0 * 10.0);
        CHECK(relError(v0 - s.volume(), expected) < 0.005);
        // Tri-dexel grids agree with the Z grid.
        CHECK(relError(v0 - s.grid(0).volume(), expected) < 0.02);
        CHECK(relError(v0 - s.grid(1).volume(), expected) < 0.02);
    }
    SECTION("ball groove") {
        Stock s = Stock::box(stockBox, 0.1);
        const ToolShape ball = ToolShape::fromTool(tooling::makeBallEndMill(1, 10, 25, 60));
        PathWriter w({30, 30, 40});
        w.line({30, 30, 25}).line({70, 30, 25}).rapid({70, 30, 40});
        simulateToolpath(s, ball, w.path);
        const double expected = 0.5 * kPi * 25.0 * 40.0 + 2.0 / 3.0 * kPi * 125.0;
        CHECK(relError(v0 - s.volume(), expected) < 0.005);
    }
    SECTION("arc slot") {
        Stock s = Stock::box(stockBox, 0.1);
        PathWriter w({30, 30, 40});
        w.line({30, 30, 28}).arc({70, 30, 28}, {50, 30, 28}, true).rapid({70, 30, 40});
        simulateToolpath(s, flat(8), w.path);
        // Half annulus (radii 16..24) plus two half disks at the ends, 2 mm deep.
        const double expected = 2.0 * (0.5 * kPi * (24.0 * 24.0 - 16.0 * 16.0) + kPi * 16.0);
        CHECK(relError(v0 - s.volume(), expected) < 0.005);
    }
    SECTION("pocket from the pocket strategy") {
        const double b = std::tan(kPi / 8.0);
        const geom2d::Polyline outline({{{5, 0}, 0},
                                        {{35, 0}, b},
                                        {{40, 5}, 0},
                                        {{40, 15}, b},
                                        {{35, 20}, 0},
                                        {{5, 20}, b},
                                        {{0, 15}, 0},
                                        {{0, 5}, b}},
                                       true);
        const tooling::Tool tool = tooling::makeFlatEndMill(1, 10, 25, 60);
        CancelToken cancel;
        NullProgress progress;
        strategies::Context ctx{tool, {}, {}, 0.002, cancel, progress};
        ctx.heights.clearance = 15;
        ctx.heights.retract = 5;
        const auto tp = strategies::pocket({{outline, {}}}, 0.0, -10.0, {}, ctx);
        REQUIRE_FALSE(tp.moves.empty());
        Stock s = Stock::box(box({-10, -10, -20}, {50, 30, 0}), 0.1);
        const double before = s.volume();
        const auto r = simulateToolpath(s, ToolShape::fromTool(tool), tp);
        CHECK(r.collisions.empty());
        const double expected = (800.0 - (4.0 - kPi) * 25.0) * 10.0;
        CHECK(relError(before - s.volume(), expected) < 0.005);
    }
}

TEST_CASE("removed volume converges with resolution", "[sim][removal]") {
    const ToolShape ball = ToolShape::fromTool(tooling::makeBallEndMill(1, 8, 25, 60));
    const double expected = 0.5 * kPi * 16.0 * 30.0 + 2.0 / 3.0 * kPi * 64.0;
    double previous = 1.0;
    for (const double h : {0.4, 0.2, 0.1}) {
        Stock s = Stock::box(box({0, 0, 0}, {60, 40, 20}), h, false);
        const double before = s.volume();
        PathWriter w({10.03, 20.07, 30});
        w.line({10.03, 20.07, 16}).line({40.03, 20.07, 16}).rapid({40.03, 20.07, 30});
        simulateToolpath(s, ball, w.path);
        const double err = relError(before - s.volume(), expected);
        INFO("h = " << h << ", error = " << err);
        CHECK(err < previous * 1.2 + 1e-4);
        previous = err;
    }
    CHECK(previous < 0.005);
}

TEST_CASE("collisions: shank, holder and rapid", "[sim][collision]") {
    const auto stockBox = box({0, 0, -40}, {100, 60, 0});
    tooling::Tool tool = tooling::makeFlatEndMill(1, 10, 20, 60);
    tool.stickout = 30.0;
    tooling::Holder holder;
    holder.profile = geom2d::Polyline::fromPoints({{0, 0}, {16, 0}, {16, 40}, {0, 40}}, false);

    SECTION("deep slot hits the shank and the holder") {
        Stock s = Stock::box(stockBox, 0.25);
        PathWriter w({20, 30, 10});
        w.rapid({20, 30, 2}).line({20, 30, -32}).line({60, 30, -32}).rapid({60, 30, 10});
        const auto r = simulateToolpath(s, ToolShape::fromTool(tool, &holder), w.path);
        bool shank = false, hold = false;
        for (const auto& c : r.collisions) {
            shank = shank || c.kind == CollisionKind::Shank;
            hold = hold || c.kind == CollisionKind::Holder;
        }
        CHECK(shank);
        CHECK(hold);
        CHECK(r.collisions.front().firstMove == 1);
    }
    SECTION("within the flute length there is no collision") {
        Stock s = Stock::box(stockBox, 0.25);
        PathWriter w({20, 30, 10});
        w.rapid({20, 30, 2}).line({20, 30, -15}).line({60, 30, -15}).rapid({60, 30, 10});
        const auto r = simulateToolpath(s, ToolShape::fromTool(tool, &holder), w.path);
        CHECK(r.collisions.empty());
    }
    SECTION("rapid through material") {
        Stock s = Stock::box(stockBox, 0.25);
        PathWriter w({-10, 30, -1});
        w.rapid({110, 30, -1});
        const auto r = simulateToolpath(s, ToolShape::fromTool(tool, &holder), w.path);
        REQUIRE(r.collisions.size() == 1);
        CHECK(r.collisions[0].kind == CollisionKind::Rapid);
        CHECK(r.collisions[0].depth == Approx(1.0).margin(0.05));
    }
}

TEST_CASE("drill cycles are simulated", "[sim][removal]") {
    Stock s = Stock::box(box({0, 0, -20}, {40, 40, 0}), 0.1);
    toolpath::Toolpath tp;
    tp.start = {10, 10, 10};
    toolpath::DrillCycle c;
    c.type = toolpath::CycleType::Peck;
    c.retractZ = 2;
    c.bottomZ = -8;
    c.peck = 3;
    c.feedRate = 5;
    tp.cycles.push_back(c);
    for (const Vec2& p : {Vec2(10, 10), Vec2(30, 30)}) {
        Move m;
        m.kind = MoveKind::Cycle;
        m.feedClass = FeedClass::Plunge;
        m.end = {p.x(), p.y(), 2};
        m.cycle = 0;
        tp.moves.push_back(m);
    }
    const double before = s.volume();
    const auto r = simulateToolpath(s, flat(6), tp);
    CHECK(r.collisions.empty());
    CHECK(relError(before - s.volume(), 2.0 * kPi * 9.0 * 8.0) < 0.005);
}

TEST_CASE("part comparison and display mesh", "[sim][compare]") {
    const geom3d::Mesh part = boxMesh(box({0, 0, 0}, {60, 40, 28}));
    Stock s = Stock::box(box({0, 0, 0}, {60, 40, 30}), 0.25);

    Comparison before = compareToPart(s, part);
    CHECK_FALSE(before.hasGouges());
    CHECK(before.maxExcess == Approx(2.0).margin(1e-3));
    CHECK(before.excessVolume > 0.9 * 60 * 40 * 2);

    // Face to the part top with a 20 mm cutter, then gouge with one plunge.
    PathWriter w({-15, -5, 28});
    for (int k = 0; k <= 6; ++k) {
        const double y = -5.0 + 8.0 * k;
        w.line({k % 2 == 0 ? 75.0 : -15.0, y, 28});
        w.line({k % 2 == 0 ? 75.0 : -15.0, y + 8.0, 28});
    }
    simulateToolpath(s, flat(20), w.path);
    const Comparison faced = compareToPart(s, part);
    CHECK_FALSE(faced.hasGouges());
    CHECK(faced.excessRays == 0);

    PathWriter g({30, 20, 40});
    g.line({30, 20, 27}).rapid({30, 20, 40});
    simulateToolpath(s, flat(6), g.path);
    const Comparison gouged = compareToPart(s, part);
    CHECK(gouged.hasGouges());
    // The part prepared once (playback) gives the same comparison.
    const PartReference prepared =
        preparePart(Stock::box(box({0, 0, 0}, {60, 40, 30}), 0.25), part);
    REQUIRE(prepared.matches(s.grid(2)));
    const Comparison again = compareToPart(s, prepared);
    CHECK(again.gouge == gouged.gouge);
    CHECK(again.excess == gouged.excess);
    CHECK(again.gougeVolume == Approx(gouged.gougeVolume));
    CHECK_THROWS_AS(compareToPart(Stock::box(box({0, 0, 0}, {60, 40, 30}), 0.5), prepared), Error);
    CHECK(gouged.maxGouge == Approx(1.0 - gouged.tolerance).margin(0.01));
    CHECK(gouged.gougeVolume == Approx(kPi * 9.0 * (1.0 - gouged.tolerance)).epsilon(0.15));

    const DisplayMesh mesh = buildDisplayMesh(s, &gouged);
    CHECK(mesh.triangleCount() > 1000);
    CHECK(mesh.positions.size() == mesh.colours.size());
    bool red = false;
    for (const Rgb& c : mesh.colours) {
        red = red || (c[0] > 120 && c[1] < 80);
    }
    CHECK(red);
    // Red only where the part material is missing (the plunge's floor and walls), not down the
    // rays to the stock's underside: a chamfer cut on top does not show on the bottom.
    const auto deepest = static_cast<std::size_t>(
        std::max_element(gouged.gouge.begin(), gouged.gouge.end()) - gouged.gouge.begin());
    CHECK(gouged.gougeLo[deepest] == Approx(27.0).margin(0.01));
    CHECK(gouged.gougeHi[deepest] == Approx(28.0 - gouged.tolerance).margin(0.01));
    const auto lowestRed = [](const DisplayMesh& m) {
        float lowest = std::numeric_limits<float>::infinity();
        for (std::size_t v = 0; v < m.positions.size(); ++v) {
            if (m.colours[v][0] > 120 && m.colours[v][1] < 80) {
                lowest = std::min(lowest, m.positions[v][2]);
            }
        }
        return lowest;
    };
    CHECK(lowestRed(mesh) > 26.0f);
    CHECK(lowestRed(mesh) < 28.0f);
    StockDisplay display;
    display.update(s, &prepared, 1, true);
    DisplayMesh all;
    for (const auto& chunk : display.chunks()) {
        if (chunk) {
            all.positions.insert(all.positions.end(), chunk->positions.begin(),
                                 chunk->positions.end());
            all.colours.insert(all.colours.end(), chunk->colours.begin(), chunk->colours.end());
        }
    }
    CHECK(lowestRed(all) > 26.0f);
    CHECK(lowestRed(all) < 28.0f);
    const DisplayMesh plain = buildDisplayMesh(s, nullptr, {50, 2.0});
    CHECK(plain.triangleCount() > 0);
    CHECK(plain.triangleCount() < mesh.triangleCount());

    // Uncut material in the colour the viewer asks for.
    DisplayOptions coloured{50, 2.0};
    coloured.stockColour = {10, 20, 30};
    const DisplayMesh tinted = buildDisplayMesh(s, nullptr, coloured);
    CHECK(std::find(tinted.colours.begin(), tinted.colours.end(), coloured.stockColour) !=
          tinted.colours.end());
    CHECK(std::find(tinted.colours.begin(), tinted.colours.end(), kStockColour) ==
          tinted.colours.end());
}

// Hidden from the default run (and from CTest's discovery): it measures time, so CTest runs it on
// its own (tests/unit/sim/CMakeLists.txt) instead of next to other tests.
TEST_CASE("simulation throughput", "[.][sim][perf]") {
    // A dense zigzag of short moves. OCAM_SIM_BENCH=1 runs the 100k-move benchmark.
    const bool bench = std::getenv("OCAM_SIM_BENCH") != nullptr;
    const int moves = bench ? 100000 : 10000;
    const double h = bench ? 0.1 : 0.2;
    Stock s = Stock::box(box({0, 0, -20}, {100, 100, 0}), h);
    PathWriter w({0, 0, 5});
    w.line({0, 0, -2});
    const int perRow = 100;
    const int rows = moves / perRow;
    for (int r = 0; r < rows; ++r) {
        const double y = 100.0 * r / rows;
        for (int k = 1; k <= perRow; ++k) {
            const double x = r % 2 == 0 ? k : 100 - k;
            w.line({x, y + 100.0 / rows * k / perRow, -2.0 - 0.01 * (r % 5)});
        }
    }
    const auto t0 = std::chrono::steady_clock::now();
    const auto r = simulateToolpath(s, flat(10), w.path);
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::cout << "simulated " << r.movesSimulated << " moves at " << h << " mm in " << seconds
              << " s\n";
    CHECK(r.movesSimulated == w.path.moves.size());
    CHECK(seconds < (bench ? 30.0 : 15.0));
}

TEST_CASE("remaining material regions follow the stock", "[sim][rest]") {
    Stock s = Stock::box(box({0, 0, 0}, {40, 20, 10}), 0.25, false);
    // Untouched: the whole block, one region without holes.
    auto all = materialRegions(s, 5.0, 10.0);
    REQUIRE(all.size() == 1);
    CHECK(all[0].holes.empty());
    CHECK(geom2d::totalArea(all) == Approx(800.0).epsilon(1e-6));
    // A slot to z = 4 across the middle splits the slab above z = 5 into two pieces.
    PathWriter w({20, -10, 20});
    w.line({20, -10, 4}).line({20, 30, 4}).rapid({20, 30, 20});
    simulateToolpath(s, flat(6), w.path);
    auto split = materialRegions(s, 5.0, 10.0);
    CHECK(split.size() == 2);
    CHECK(geom2d::totalArea(split) == Approx(800.0 - 6.0 * 20.0).margin(2 * 0.25 * 20.0));
    // Below the slot floor nothing was removed.
    CHECK(geom2d::totalArea(materialRegions(s, 1.0, 3.0)) == Approx(800.0).epsilon(1e-6));
    // A plunge leaves a hole in the slab.
    PathWriter g({8, 10, 20});
    g.line({8, 10, 2}).rapid({8, 10, 20});
    simulateToolpath(s, flat(4), g.path);
    auto holed = materialRegions(s, 3.0, 4.0);
    REQUIRE(holed.size() == 1);
    CHECK(holed[0].holes.size() == 1);
    CHECK(geom2d::totalArea(holed) == Approx(800.0 - kPi * 4.0).margin(1.0));
    // Empty slab above the stock.
    CHECK(materialRegions(s, 11.0, 20.0).empty());
}

TEST_CASE("chamfer and helical bore remove their analytic volumes", "[sim][removal]") {
    CancelToken cancel;
    NullProgress progress;
    SECTION("45 degree chamfer around a block") {
        const tooling::Tool tool = tooling::makeChamferMill(1, 12.0, 0.0, degToRad(90.0), 60.0);
        strategies::Context ctx{tool, {}, {}, 0.002, cancel, progress};
        strategies::ChamferParams p;
        p.width = 1.0;
        const geom2d::Regions rect{{geom2d::Polyline::rectangle({0, 0}, {40, 20}), {}}};
        const auto tp = strategies::chamfer(rect, 0.0, p, ctx);
        Stock s = Stock::box(box({0, 0, -10}, {40, 20, 0}), 0.05, false);
        const double before = s.volume();
        simulateToolpath(s, ToolShape::fromTool(tool), tp);
        // Triangular prisms along the full edges; at each convex corner the two prisms overlap
        // in a pyramid of volume w^3 / 3.
        const double expected = 120.0 * 0.5 - 4.0 / 3.0;
        CHECK(relError(before - s.volume(), expected) < 0.02);
    }
    SECTION("helical bore of a 20 mm hole with a 10 mm end mill") {
        const tooling::Tool tool = tooling::makeFlatEndMill(1, 10.0, 25.0, 60.0);
        strategies::Context ctx{tool, {}, {}, 0.002, cancel, progress};
        strategies::BoreParams p;
        p.helixPitch = 1.0;
        const auto tp = strategies::bore({{{25, 25}, 0.0, 5.0, false, 20.0}}, p, ctx);
        Stock s = Stock::box(box({0, 0, -20}, {50, 50, 0}), 0.1, false);
        const double before = s.volume();
        simulateToolpath(s, ToolShape::fromTool(tool), tp);
        CHECK(relError(before - s.volume(), kPi * 100.0 * 5.0) < 0.005);
    }
}

TEST_CASE("ball parallel finishing leaves the requested scallop", "[sim][surface]") {
    // A flat plate finished with a ball mill: the cusps between passes are the scallop height.
    const geom3d::Mesh plate = boxMesh(box({0, 0, 0}, {30, 20, 10}));
    const tooling::Tool tool = tooling::makeBallEndMill(1, 6.0, 20.0, 50.0);
    CancelToken cancel;
    NullProgress progress;
    strategies::Context ctx{tool, {}, {}, 0.002, cancel, progress};
    ctx.heights.retract = 15;
    ctx.heights.clearance = 20;
    strategies::ParallelParams p;
    p.scallopHeight = 0.05;
    const auto tp = strategies::parallelFinish(plate, 0.0, p, ctx);
    Stock s = Stock::box(box({0, 0, 0}, {30, 20, 10.5}), 0.02, false);
    simulateToolpath(s, ToolShape::fromTool(tool), tp);
    const Comparison c = compareToPart(s, plate, {0.005});
    CHECK_FALSE(c.hasGouges());
    // Away from the plate's edges the cusp height is the scallop (within a ray spacing).
    double worst = 0.0;
    for (int j = 0; j < c.nv; ++j) {
        for (int i = 0; i < c.nu; ++i) {
            const Vec2 q(s.grid(2).u(i), s.grid(2).v(j));
            if (q.x() > 5 && q.x() < 25 && q.y() > 5 && q.y() < 15) {
                worst = std::max(
                    worst,
                    static_cast<double>(
                        c.excess[static_cast<std::size_t>(j) * static_cast<std::size_t>(c.nu) +
                                 static_cast<std::size_t>(i)]));
            }
        }
    }
    INFO("worst cusp " << worst);
    CHECK(worst <= 0.05 + 0.01);
    CHECK(worst >= 0.03);
}

TEST_CASE("material coloured by the operation that cut it", "[sim][compare]") {
    const Stock before = Stock::box(box({0, 0, 0}, {20, 20, 10}), 0.5);
    Stock after = before;
    PathWriter w({10, 10, 20});
    w.line({10, 10, 5}).rapid({10, 10, 20});
    simulateToolpath(after, flat(6), w.path);
    const std::vector<std::uint8_t> changed = changedRays(before, after);
    const DexelGrid& g = before.grid(2);
    REQUIRE(changed.size() == static_cast<std::size_t>(g.nu()) * static_cast<std::size_t>(g.nv()));
    const auto cut = std::count(changed.begin(), changed.end(), std::uint8_t{1});
    CHECK(cut > 80); // about pi * 3^2 / 0.5^2 = 113 rays
    CHECK(cut < 150);
    CHECK(changedRays(before, Stock::box(box({0, 0, 0}, {20, 20, 10}), 1.0)).empty());

    DisplayOptions options;
    options.cutBy = &changed; // 1 = operation 0
    const DisplayMesh mesh = buildDisplayMesh(after, nullptr, options);
    CHECK(std::find(mesh.colours.begin(), mesh.colours.end(), operationColour(0)) !=
          mesh.colours.end());
    CHECK(std::find(mesh.colours.begin(), mesh.colours.end(), kStockColour) != mesh.colours.end());
    CHECK(operationColour(0) != operationColour(1));
}

TEST_CASE("a surface is at size by where it is along its ray", "[sim][compare]") {
    // A part with a groove all around, and a stock that still fills it: every Z ray has stock left
    // in the middle and none at either end, as when a live tool cuts in from the side.
    geom3d::Mesh part = boxMesh(box({0, 0, 0}, {60, 40, 10}));
    const geom3d::Mesh upper = boxMesh(box({0, 0, 20}, {60, 40, 30}));
    const auto base = static_cast<std::uint32_t>(part.vertices.size());
    part.vertices.insert(part.vertices.end(), upper.vertices.begin(), upper.vertices.end());
    for (auto tri : upper.triangles) {
        for (auto& v : tri) {
            v += base;
        }
        part.triangles.push_back(tri);
    }
    const Stock s = Stock::box(box({0, 0, 0}, {60, 40, 30}), 0.25);
    const PartReference prepared = preparePart(s, part);
    StockDisplay display;
    display.update(s, &prepared, 0, true);
    const Rgb green = comparisonColour(0.0, 0.0, prepared.tolerance, 2.0);
    int atEnds = 0, atEndsGreen = 0, inGroove = 0, inGrooveGreen = 0;
    for (const auto& chunk : display.chunks()) {
        if (!chunk) {
            continue;
        }
        for (std::size_t v = 0; v < chunk->positions.size(); ++v) {
            const auto& p = chunk->positions[v];
            if (p[0] > 0.2f && p[0] < 59.8f && p[1] > 0.2f && p[1] < 39.8f) {
                continue; // the walls only
            }
            const bool isGreen = chunk->colours[v] == green;
            if (p[2] > 2.0f && p[2] < 8.0f) {
                ++atEnds;
                atEndsGreen += isGreen ? 1 : 0;
            } else if (p[2] > 12.0f && p[2] < 18.0f) {
                ++inGroove;
                inGrooveGreen += isGreen ? 1 : 0;
            }
        }
    }
    REQUIRE(atEnds > 0);
    REQUIRE(inGroove > 0);
    CHECK(atEndsGreen == atEnds);
    CHECK(inGrooveGreen == 0);
}

TEST_CASE("incremental stock display", "[sim][compare]") {
    const Stock start = Stock::box(box({0, 0, 0}, {60, 40, 30}), 0.1);
    DisplayOptions options;
    options.maxCells = 800;
    StockDisplay display(options);
    display.update(start, nullptr, 0, true);
    const auto before = display.chunks();
    REQUIRE(before.size() > 100);

    Stock cut = start;
    PathWriter w({10, 10, 40});
    w.line({10, 10, 25}).line({20, 10, 25}).rapid({20, 10, 40});
    simulateToolpath(cut, flat(6), w.path);
    display.update(cut, nullptr, 0, false);
    const auto after = display.chunks();
    REQUIRE(after.size() == before.size());
    std::size_t rebuilt = 0;
    for (std::size_t c = 0; c < after.size(); ++c) {
        rebuilt += after[c] != before[c] ? 1 : 0;
    }
    CHECK(rebuilt > 0);
    CHECK(rebuilt < after.size() / 4); // only around the cut (16 x 6 mm of 60 x 40 mm)

    // The same picture as a full rebuild with the same cut marks.
    const std::vector<std::uint8_t> marks = changedRays(start, cut);
    StockDisplay fresh(options);
    fresh.update(cut, nullptr, 0, true, &marks);
    auto summarize = [](const StockDisplay& d) {
        std::size_t triangles = 0, cutVertices = 0;
        for (const auto& mesh : d.chunks()) {
            if (mesh) {
                triangles += mesh->triangleCount();
                cutVertices += static_cast<std::size_t>(
                    std::count(mesh->colours.begin(), mesh->colours.end(), operationColour(0)));
            }
        }
        return std::pair(triangles, cutVertices);
    };
    CHECK(summarize(display) == summarize(fresh));
    CHECK(summarize(display).second > 0);
}
