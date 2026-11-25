// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "../../../libs/sim/src/Tilted.hpp"

#include <ocam/sim/Simulate.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <random>

using namespace ocam;
using namespace ocam::sim;
using Catch::Matchers::WithinRel;

namespace {

/// A convex function of w that is <= 0 exactly inside the slice.
double outside(const SolidPiece& p, const Vec3& w, const Vec3& u) {
    const double h = w.dot(u);
    double g = std::max(p.h0 - h, h - p.h1);
    if (p.sphere > 0.0) {
        return std::max(g, (w - p.centre * u).norm() - p.sphere);
    }
    const double r = p.r0 + (p.r1 - p.r0) * (h - p.h0) / (p.h1 - p.h0);
    return std::max(g, (w - h * u).norm() - r);
}

toolpath::Move line(const Vec3& end, const Vec3& axis,
                    toolpath::FeedClass fc = toolpath::FeedClass::Cut) {
    toolpath::Move m;
    m.kind =
        fc == toolpath::FeedClass::Rapid ? toolpath::MoveKind::Rapid : toolpath::MoveKind::Linear;
    m.feedClass = fc;
    m.end = end;
    m.axis = axis.normalized();
    m.feedRate = fc == toolpath::FeedClass::Rapid ? 0.0 : 10.0;
    return m;
}

} // namespace

TEST_CASE("swept spans of tilted tool slices agree with brute force", "[sim][tilted]") {
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> u01(0.0, 1.0), um(-1.0, 1.0);
    int hits = 0;
    for (int trial = 0; trial < 400; ++trial) {
        SolidPiece piece;
        if (trial % 2 == 0) {
            piece.h0 = 2.0 * u01(rng);
            piece.h1 = piece.h0 + 0.5 + 4.0 * u01(rng);
            piece.r0 = 0.2 + 3.0 * u01(rng);
            piece.r1 = 0.2 + 3.0 * u01(rng);
        } else {
            piece.sphere = 1.0 + 2.0 * u01(rng);
            piece.centre = piece.sphere;
            piece.h0 = piece.centre - piece.sphere * u01(rng);
            piece.h1 = piece.centre + piece.sphere * (u01(rng) - 0.3);
            if (piece.h1 <= piece.h0 + 0.05) {
                piece.h1 = piece.h0 + 0.05;
            }
        }
        const Vec3 u = Vec3(um(rng), um(rng), um(rng)).normalized();
        const Vec3 p0(um(rng), um(rng), um(rng));
        const Vec3 d = Vec3(um(rng), um(rng), um(rng)) * (trial % 3 == 0 ? 0.0 : 4.0);
        const int axis = trial % 3;
        Vec3 origin(um(rng) * 3.0, um(rng) * 3.0, um(rng) * 3.0);
        origin[axis] = 0.0;
        double lo = 0.0, hi = 0.0;
        const bool got = detail::sweptSpan(piece, p0, d, u, origin, axis, lo, hi);
        // Brute force: sample s along the ray and t along the move.
        double blo = 1e9, bhi = -1e9;
        const Vec3 E = Vec3::Unit(axis);
        for (int is = 0; is <= 1200; ++is) {
            const double s = -12.0 + 24.0 * is / 1200.0;
            // The swept slice is convex: minimize the convex outside measure over t.
            auto g = [&](double t) {
                return outside(piece, origin + s * E - (p0 + t * d), u);
            };
            double a = 0.0, b = 1.0;
            for (int it = 0; it < 60; ++it) {
                const double m1 = a + (b - a) / 3.0, m2 = b - (b - a) / 3.0;
                if (g(m1) < g(m2)) {
                    b = m2;
                } else {
                    a = m1;
                }
            }
            if (std::min({g(0.0), g(1.0), g(0.5 * (a + b))}) <= 0.0) {
                blo = std::min(blo, s);
                bhi = std::max(bhi, s);
            }
        }
        INFO("trial " << trial << " got " << got << " [" << lo << ", " << hi << "] brute [" << blo
                      << ", " << bhi << "]");
        if (blo <= bhi) {
            REQUIRE(got);
            ++hits;
            CHECK(lo <= blo + 1e-6);
            CHECK(hi >= bhi - 1e-6);
            // The sampling is 0.02 along s.
            CHECK(lo >= blo - 0.021);
            CHECK(hi <= bhi + 0.021);
        } else if (got) {
            CHECK(hi - lo < 0.021);
        }
    }
    CHECK(hits > 100);
}

TEST_CASE("a barely tilted tool removes what the 3-axis sweep removes", "[sim][tilted]") {
    const tooling::Tool tool = tooling::makeBallEndMill(1, 6.0, 20.0, 60.0);
    const ToolShape shape = ToolShape::fromTool(tool);
    REQUIRE_FALSE(shape.cutterPieces().empty());
    const geom3d::Aabb3 box{Vec3(0, 0, 0), Vec3(40, 30, 10)};
    auto run = [&](const Vec3& axis) {
        Stock stock = Stock::box(box, 0.25);
        toolpath::Toolpath tp;
        tp.start = Vec3(-5, 10, 20);
        tp.moves.push_back(line(Vec3(-5, 10, 7), axis, toolpath::FeedClass::Rapid));
        tp.moves.push_back(line(Vec3(45, 10, 7), axis));
        tp.moves.push_back(line(Vec3(45, 18, 4), axis));
        tp.moves.push_back(line(Vec3(-5, 22, 6), axis));
        tp.moves.push_back(line(Vec3(-5, 22, 20), axis, toolpath::FeedClass::Rapid));
        const double before = stock.volume();
        const SimulateResult r = simulateToolpath(stock, shape, tp);
        CHECK(r.collisions.empty());
        return before - stock.volume();
    };
    const double flat = run(Vec3::UnitZ());
    const double tilted = run(Vec3(1e-6, 0, 1));
    CHECK_THAT(tilted, WithinRel(flat, 0.005));
}

TEST_CASE("a horizontal end mill cuts an analytic slab", "[sim][tilted]") {
    // Flat end mill D10 with 30 mm flutes lying along +Y, tip at y = -25, sweeping along X at
    // z = 10 through a 40 x 40 x 20 box: it removes z in [5, 15] for y in [-20, 5].
    tooling::Tool tool = tooling::makeFlatEndMill(1, 10.0, 30.0, 70.0);
    const ToolShape shape = ToolShape::fromTool(tool);
    Stock stock = Stock::box({Vec3(0, -20, 0), Vec3(40, 20, 20)}, 0.25);
    const double before = stock.volume();
    toolpath::Toolpath tp;
    tp.start = Vec3(-10, -25, 10);
    tp.moves.push_back(line(Vec3(-10, -25, 10), Vec3::UnitY()));
    tp.moves.push_back(line(Vec3(50, -25, 10), Vec3::UnitY()));
    const SimulateResult r = simulateToolpath(stock, shape, tp);
    CHECK_THAT(before - stock.volume(), WithinRel(40.0 * 10.0 * 25.0, 0.01));
    // The tri-dexel X grid sees the cut too: rays along X at y = 0, z = 10 are empty.
    const DexelGrid& gx = stock.grid(0);
    const auto [i0, i1] = gx.uRange(0.0, 0.0);
    const auto [j0, j1] = gx.vRange(10.0, 10.0);
    CHECK(gx.ray(i0, j0).empty());
    // The shank above the flutes (y > 5) runs through the rest of the box.
    REQUIRE_FALSE(r.collisions.empty());
    for (const Collision& c : r.collisions) {
        CHECK(c.kind == CollisionKind::Shank);
    }
}

TEST_CASE("tilted tools report shank and rapid collisions", "[sim][tilted]") {
    tooling::Tool tool = tooling::makeFlatEndMill(1, 6.0, 10.0, 60.0);
    tool.stickout = 40.0;
    const ToolShape shape = ToolShape::fromTool(tool);
    REQUIRE_FALSE(shape.shankPieces().empty());
    Stock stock = Stock::box({Vec3(0, 0, 0), Vec3(40, 40, 20)}, 0.5);
    // Tilted 60 degrees and plunged deep: the body above the 10 mm flutes enters the stock.
    const Vec3 axis = Vec3(std::sin(1.0472), 0, std::cos(1.0472));
    toolpath::Toolpath tp;
    tp.start = Vec3(20, 20, 40);
    tp.moves.push_back(line(Vec3(20, 20, 30), axis, toolpath::FeedClass::Rapid));
    tp.moves.push_back(line(Vec3(20, 20, 2), axis, toolpath::FeedClass::Plunge));
    tp.moves.push_back(line(Vec3(20, 20, 30), axis, toolpath::FeedClass::Rapid));
    tp.moves.push_back(line(Vec3(5, 5, 10), axis, toolpath::FeedClass::Rapid)); // through stock
    const SimulateResult r = simulateToolpath(stock, shape, tp);
    bool shank = false, rapid = false;
    for (const Collision& c : r.collisions) {
        shank = shank || c.kind == CollisionKind::Shank;
        rapid = rapid || c.kind == CollisionKind::Rapid;
    }
    CHECK(shank);
    CHECK(rapid);
}
