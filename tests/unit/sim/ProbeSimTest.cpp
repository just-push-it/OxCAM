// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Probe moves in the simulation (DL-0090): they never cut, and the stylus meeting material
// where no touch is expected is reported.

#include <ocam/sim/Dexel.hpp>
#include <ocam/sim/Simulate.hpp>
#include <ocam/sim/ToolShape.hpp>
#include <ocam/strategies/Probing.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ocam;
using namespace ocam::sim;
using Catch::Approx;
using toolpath::ProbeFeature;

namespace {

/// A 100 x 60 x 40 block with its top at Z0, and a Ø6 probe.
struct ProbeFixture {
    geom3d::Aabb3 block;
    tooling::Tool tool = tooling::makeDefaultTool(tooling::ToolType::Probe, 20, 6.0);
    CancelToken cancel;
    NullProgress progress;

    ProbeFixture() {
        block.extend(Vec3(0, 0, -40));
        block.extend(Vec3(100, 60, 0));
        tool.mill.triggerOffset = 0.02;
    }

    toolpath::Toolpath cycle(strategies::ProbeParams p) {
        p.workOffset = 1;
        p.approach = 5.0;
        p.overtravel = 4.0;
        p.fastFeed = 20.0;
        p.measureFeed = 2.0;
        const strategies::Context ctx{tool, {}, {25.0, 5.0, 1.0}, 0.002, cancel, progress};
        return strategies::probe(p, ctx);
    }

    SimulateResult simulate(Stock& stock, const toolpath::Toolpath& path) const {
        return simulateToolpath(stock, ToolShape::fromTool(tool, nullptr), path);
    }
};

} // namespace

TEST_CASE("probe moves touch the stock and cut nothing", "[sim][probing]") {
    ProbeFixture f;
    SECTION("the top, down Z") {
        strategies::ProbeParams p;
        p.feature = ProbeFeature::Surface;
        p.nominal = Vec3(50, 30, 0);
        p.direction = -Vec3::UnitZ();
        Stock stock = Stock::box(f.block, 0.25);
        const double before = stock.volume();
        const SimulateResult r = f.simulate(stock, f.cycle(p));
        CHECK(r.collisions.empty());
        CHECK(stock.volume() == Approx(before).epsilon(1e-12));
    }
    SECTION("a web across X, from both sides") {
        strategies::ProbeParams p;
        p.feature = ProbeFeature::Web;
        p.nominal = Vec3(50, 30, -10);
        p.direction = Vec3::UnitX();
        p.size = 100.0;
        Stock stock = Stock::box(f.block, 0.25);
        const double before = stock.volume();
        const SimulateResult r = f.simulate(stock, f.cycle(p));
        CHECK(r.collisions.empty());
        CHECK(stock.volume() == Approx(before).epsilon(1e-12));
    }
}

TEST_CASE("a probe that meets stock it does not expect is reported", "[sim][probing]") {
    ProbeFixture f;
    // The face is taken to be at X10, but the stock is still there from X0: the protected move
    // down to the approach position goes into it.
    strategies::ProbeParams p;
    p.feature = ProbeFeature::Surface;
    p.nominal = Vec3(10, 30, -10);
    p.direction = Vec3::UnitX();
    const toolpath::Toolpath path = f.cycle(p);
    Stock stock = Stock::box(f.block, 0.25);
    const double before = stock.volume();
    const SimulateResult r = f.simulate(stock, path);
    REQUIRE_FALSE(r.collisions.empty());
    CHECK(r.collisions.front().kind == CollisionKind::Probe);
    CHECK(path.moves.at(r.collisions.front().firstMove).kind == toolpath::MoveKind::Probe);
    CHECK(toString(CollisionKind::Probe) == "probe");
    CHECK(stock.volume() == Approx(before).epsilon(1e-12)); // still nothing cut
}
