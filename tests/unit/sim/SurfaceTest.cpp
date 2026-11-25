// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/sim/Compare.hpp>
#include <ocam/sim/Dexel.hpp>
#include <ocam/sim/Lathe.hpp>
#include <ocam/sim/Simulate.hpp>
#include <ocam/sim/Surface.hpp>
#include <ocam/sim/ToolShape.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>

using namespace ocam;
using namespace ocam::sim;
using Catch::Approx;

namespace {

geom3d::Aabb3 box(const Vec3& lo, const Vec3& hi) {
    geom3d::Aabb3 b;
    b.extend(lo);
    b.extend(hi);
    return b;
}

Vec3 at(const DisplayMesh& m, std::uint32_t v) {
    return Vec3(static_cast<double>(m.positions[v][0]), static_cast<double>(m.positions[v][1]),
                static_cast<double>(m.positions[v][2]));
}

/// Volume enclosed by the triangles (divergence theorem).
double meshVolume(const DisplayMesh& m) {
    double v = 0.0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        v += at(m, m.indices[t]).dot(at(m, m.indices[t + 1]).cross(at(m, m.indices[t + 2])));
    }
    return v / 6.0;
}

/// Edges (by vertex position) used by an unequal number of triangles in each direction: zero
/// for a closed, consistently oriented mesh.
std::size_t openEdges(const DisplayMesh& m) {
    using Key = std::tuple<float, float, float>;
    auto key = [&](std::uint32_t v) {
        return Key{m.positions[v][0], m.positions[v][1], m.positions[v][2]};
    };
    std::map<std::pair<Key, Key>, int> count;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        for (int e = 0; e < 3; ++e) {
            const Key a = key(m.indices[t + static_cast<std::size_t>(e)]);
            const Key b = key(m.indices[t + static_cast<std::size_t>((e + 1) % 3)]);
            if (a < b) {
                ++count[{a, b}];
            } else {
                --count[{b, a}];
            }
        }
    }
    return static_cast<std::size_t>(
        std::count_if(count.begin(), count.end(), [](const auto& kv) { return kv.second != 0; }));
}

toolpath::Move line(const Vec3& p, toolpath::MoveKind kind = toolpath::MoveKind::Linear) {
    toolpath::Move m;
    m.kind = kind;
    m.end = p;
    m.feedRate = kind == toolpath::MoveKind::Rapid ? 0.0 : 10.0;
    if (kind == toolpath::MoveKind::Rapid) {
        m.feedClass = toolpath::FeedClass::Rapid;
    }
    return m;
}

} // namespace

TEST_CASE("smooth surface of a box is the box", "[sim][surface]") {
    const Stock s = Stock::box(box({0, 0, 0}, {40, 30, 20}), 0.5);
    const DisplayMesh m = buildSurfaceMesh(s, 1000);
    CHECK(openEdges(m) == 0);
    CHECK(meshVolume(m) == Approx(40.0 * 30.0 * 20.0).epsilon(1e-5));
    // Every vertex on a face of the box, corners included (sharp features).
    double worst = 0.0;
    for (std::uint32_t v = 0; v < m.positions.size(); ++v) {
        const Vec3 p = at(m, v);
        const double d = std::min({std::abs(p.x()), std::abs(p.x() - 40), std::abs(p.y()),
                                   std::abs(p.y() - 30), std::abs(p.z()), std::abs(p.z() - 20)});
        worst = std::max(worst, d);
    }
    CHECK(worst < 1e-4);
    // Flat faces and sharp edges: six normals only.
    for (const auto& n : m.normals) {
        const double big = std::max({std::abs(n[0]), std::abs(n[1]), std::abs(n[2])});
        CHECK(big == Approx(1.0).margin(1e-6));
    }
}

TEST_CASE("smooth surface follows a ball-end groove and a cylinder", "[sim][surface]") {
    const double h = 0.25;
    Stock s = Stock::box(box({0, 0, 0}, {40, 30, 10}), h);
    const ToolShape ball = ToolShape::fromTool(tooling::makeBallEndMill(1, 10, 25, 60));
    toolpath::Toolpath path;
    path.start = Vec3(-10, 15, 20);
    path.moves.push_back(line({-10, 15, 7}));
    path.moves.push_back(line({50, 15, 7}));
    path.moves.push_back(line({50, 15, 20}, toolpath::MoveKind::Rapid));
    simulateToolpath(s, ball, path);
    const DisplayMesh m = buildSurfaceMesh(s, 1000);
    CHECK(openEdges(m) == 0);
    // Groove: a half cylinder of radius 5 about the line y = 15, z = 12; elsewhere planes.
    double worst = 0.0;
    for (std::uint32_t v = 0; v < m.positions.size(); ++v) {
        const Vec3 p = at(m, v);
        double d = std::min({std::abs(p.x()), std::abs(p.x() - 40), std::abs(p.y()),
                             std::abs(p.y() - 30), std::abs(p.z())});
        if (std::abs(p.y() - 15) >= 4.0 - 1e-3) {
            d = std::min(d, std::abs(p.z() - 10));
        }
        if (p.z() > 7 - 1e-3) {
            d = std::min(d, std::abs(std::hypot(p.y() - 15, p.z() - 12) - 5.0));
        }
        worst = std::max(worst, d);
    }
    INFO("worst vertex distance " << worst);
    CHECK(worst < 0.2 * h);

    // A bar from a revolved profile: vertices on the cylinder and the end faces.
    const geom2d::Regions bar = LatheStock::cylinder(0, 30, 12).profile();
    const Stock round = Stock::fromRevolvedProfile(bar, 0.2);
    const DisplayMesh rm = buildSurfaceMesh(round, 1000);
    CHECK(openEdges(rm) == 0);
    double radial = 0.0;
    for (std::uint32_t v = 0; v < rm.positions.size(); ++v) {
        const Vec3 p = at(rm, v);
        double d = std::min(std::abs(p.z()), std::abs(p.z() - 30));
        d = std::min(d, std::abs(std::hypot(p.x(), p.y()) - 12.0));
        radial = std::max(radial, d);
    }
    INFO("worst radial distance " << radial);
    CHECK(radial < 0.1 * 0.2);
    CHECK(meshVolume(rm) == Approx(kPi * 144.0 * 30.0).epsilon(2e-3));
}

TEST_CASE("a stock sampled again on a turned grid keeps its cuts", "[sim][surface]") {
    // A 40 x 30 x 20 block with a 20 x 10 x 5 pocket in its top, on every grid.
    const geom3d::Aabb3 block = box(Vec3(0, 0, 0), Vec3(40, 30, 20));
    Stock source = Stock::box(block, 0.25);
    const Vec3 lo(10, 10, 15), hi(30, 20, 25);
    for (int axis = 0; axis < 3; ++axis) {
        DexelGrid& g = source.grid(axis);
        const auto [i0, i1] = g.uRange(lo[g.uAxis()], hi[g.uAxis()]);
        const auto [j0, j1] = g.vRange(lo[g.vAxis()], hi[g.vAxis()]);
        for (int j = j0; j <= j1; ++j) {
            for (int i = i0; i <= i1; ++i) {
                subtract(g.mutableRay(i, j), lo[axis], hi[axis]);
            }
        }
    }
    // Turned over (half a turn about X) into a grid of the same block: the pocket is underneath.
    Transform3 turn = Transform3::Identity();
    turn.linear() = Eigen::AngleAxisd(kPi, Vec3::UnitX()).toRotationMatrix();
    turn.translation() = Vec3(0, 30, 20);
    const Stock target = Stock::box(block, 0.25);
    const Stock turned = resampleStock(source, turn, target);
    CHECK(turned.volume() == Approx(source.volume()).epsilon(0.005));
    const DexelGrid& z = turned.grid(2);
    const IntervalList& under = z.ray(z.uRange(20, 20.3).first, z.vRange(15, 15.3).first);
    REQUIRE(under.size() == 1);
    CHECK(under.front().lo == Approx(5.0).margin(0.1));
    CHECK(under.front().hi == Approx(20.0).margin(0.1));
    const IntervalList& beside = z.ray(z.uRange(5, 5.3).first, z.vRange(15, 15.3).first);
    REQUIRE(beside.size() == 1);
    CHECK(beside.front().lo == Approx(0.0).margin(0.1));
    // Never more material than the target had: an empty target stays empty.
    const Stock none =
        resampleStock(source, turn, Stock::box(box(Vec3(50, 50, 50), Vec3(60, 60, 60)), 0.25));
    CHECK(none.volume() == Approx(0.0).margin(1e-9));
}

TEST_CASE("revolved mesh of a turned profile", "[sim][surface][lathe]") {
    LatheStock stock = LatheStock::cylinder(0, 50, 15, 4);
    const DisplayMesh m = revolvedMesh(stock.profile(), 0.01);
    CHECK(openEdges(m) == 0);
    CHECK(meshVolume(m) == Approx(stock.volume()).epsilon(1e-3));
}

TEST_CASE("half profile of a turned mesh by its axial section", "[sim][surface][lathe]") {
    // A tube (radii 4 and 12, z 0..30) as a closed mesh: the revolved mesh of its profile.
    const DisplayMesh tube = revolvedMesh(LatheStock::cylinder(0, 30, 12, 4).profile(), 0.001);
    geom3d::Mesh mesh;
    for (const auto& p : tube.positions) {
        mesh.vertices.emplace_back(p[0], p[1], p[2]);
    }
    for (std::size_t t = 0; t + 2 < tube.indices.size(); t += 3) {
        mesh.triangles.push_back({tube.indices[t], tube.indices[t + 1], tube.indices[t + 2]});
    }
    const ProfileDistance d = ProfileDistance::section(mesh);
    REQUIRE_FALSE(d.isEmpty());
    CHECK(d(Vec2(15, 8)) == Approx(-4.0).margin(1e-3));   // in the wall
    CHECK(d(Vec2(15, 2)) == Approx(2.0).margin(1e-3));    // in the bore
    CHECK(d(Vec2(15, 12.5)) == Approx(0.5).margin(1e-3)); // outside the diameter
    CHECK(d(Vec2(31, 8)) == Approx(1.0).margin(1e-3));    // past the end face
}

TEST_CASE("smooth display updates match a full rebuild", "[sim][surface]") {
    const double h = 0.25;
    const Stock before = Stock::box(box({0, 0, 0}, {60, 40, 10}), h);
    Stock after = before;
    const ToolShape ball = ToolShape::fromTool(tooling::makeBallEndMill(1, 6, 25, 60));
    toolpath::Toolpath path;
    path.start = Vec3(10, 10, 20);
    path.moves.push_back(line({10, 10, 8}));
    path.moves.push_back(line({45, 25, 6}));
    path.moves.push_back(line({45, 25, 20}, toolpath::MoveKind::Rapid));
    simulateToolpath(after, ball, path);

    DisplayOptions options;
    options.maxCells = 120;
    StockDisplay display(options);
    display.update(before, nullptr, 0, true);
    display.update(after, nullptr, 0, false);
    StockDisplay fresh(options);
    fresh.update(after, nullptr, 0, true);
    auto summarize = [](const StockDisplay& d) {
        std::size_t tris = 0;
        double volume = 0.0;
        for (const auto& c : d.chunks()) {
            if (c) {
                tris += c->triangleCount();
                volume += meshVolume(*c);
            }
        }
        return std::pair{tris, volume};
    };
    const auto [t1, v1] = summarize(display);
    const auto [t2, v2] = summarize(fresh);
    CHECK(t1 == t2);
    CHECK(v1 == Approx(v2).epsilon(1e-9));
    CHECK(v2 == Approx(after.volume()).epsilon(0.01));
}
