// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "Clearing.hpp"
#include "StrategyTestUtil.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/geom2d/Clip.hpp>
#include <ocam/strategies/Strategies.hpp>
#include <ocam/strategies/Surface.hpp>
#include <ocam/toolpath/Smoothing.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <unordered_map>

using namespace ocam;
using namespace ocam::strategies;
using namespace ocam::strategies::testing;
using Catch::Approx;
using Catch::Matchers::WithinAbs;
using toolpath::FeedClass;
using toolpath::MoveKind;

namespace {

/// Closed mesh of the solid under z = f(x, y) over a rectangle, down to zBottom.
geom3d::Mesh heightMesh(const std::function<double(double, double)>& f, double x0, double x1,
                        double y0, double y1, int n, double zBottom) {
    geom3d::Mesh m;
    auto idx = [&](int i, int j) {
        return static_cast<std::uint32_t>(j * (n + 1) + i);
    };
    for (int j = 0; j <= n; ++j) {
        for (int i = 0; i <= n; ++i) {
            const double x = x0 + (x1 - x0) * i / n, y = y0 + (y1 - y0) * j / n;
            m.vertices.emplace_back(x, y, f(x, y));
        }
    }
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            m.triangles.push_back({idx(i, j), idx(i + 1, j), idx(i + 1, j + 1)});
            m.triangles.push_back({idx(i, j), idx(i + 1, j + 1), idx(i, j + 1)});
        }
    }
    // Sides and bottom: a ring of bottom vertices below the boundary.
    std::vector<std::uint32_t> top, bottom;
    for (int i = 0; i < n; ++i)
        top.push_back(idx(i, 0));
    for (int j = 0; j < n; ++j)
        top.push_back(idx(n, j));
    for (int i = n; i > 0; --i)
        top.push_back(idx(i, n));
    for (int j = n; j > 0; --j)
        top.push_back(idx(0, j));
    for (const auto t : top) {
        const Vec3 v = m.vertices[t];
        bottom.push_back(static_cast<std::uint32_t>(m.vertices.size()));
        m.vertices.emplace_back(v.x(), v.y(), zBottom);
    }
    const auto c = static_cast<std::uint32_t>(m.vertices.size());
    m.vertices.emplace_back(0.5 * (x0 + x1), 0.5 * (y0 + y1), zBottom);
    const std::size_t k = top.size();
    for (std::size_t s = 0; s < k; ++s) {
        const auto a = top[s], b = top[(s + 1) % k], ab = bottom[s], bb = bottom[(s + 1) % k];
        m.triangles.push_back({a, ab, bb});
        m.triangles.push_back({a, bb, b});
        m.triangles.push_back({c, bb, ab});
    }
    return m;
}

/// Distance from xy to the moves before `index` that cut at height z (arcs sampled).
double distanceToCut(const toolpath::Toolpath& tp, std::size_t index, const Vec2& xy, double z) {
    double best = std::numeric_limits<double>::infinity();
    Vec3 from = Vec3::Zero();
    for (std::size_t k = 0; k < index; ++k) {
        const auto& m = tp.moves[k];
        const bool atZ = std::abs(from.z() - z) < 1e-6 && std::abs(m.end.z() - z) < 1e-6;
        if (atZ && m.kind != MoveKind::Rapid) {
            for (int s = 0; s <= 512; ++s) {
                Vec2 q;
                if (m.isArc()) {
                    const Vec2 c = m.center.head<2>();
                    const double a0 = std::atan2(from.y() - c.y(), from.x() - c.x());
                    double a1 = std::atan2(m.end.y() - c.y(), m.end.x() - c.x());
                    if (m.kind == MoveKind::ArcCCW && a1 <= a0) {
                        a1 += 2.0 * kPi;
                    } else if (m.kind == MoveKind::ArcCW && a1 >= a0) {
                        a1 -= 2.0 * kPi;
                    }
                    const double a = a0 + (a1 - a0) * s / 512.0;
                    q = c + (from.head<2>() - c).norm() * Vec2(std::cos(a), std::sin(a));
                } else {
                    q = from.head<2>() + (m.end.head<2>() - from.head<2>()) * (s / 512.0);
                }
                best = std::min(best, (q - xy).norm());
            }
        }
        from = m.end;
    }
    return best;
}

/// True when the tool, where move `index` ends, touches none of `material` that the moves
/// before it (at that height) have not removed: a plunge there goes into air or onto a cut
/// floor.
bool intoClear(const toolpath::Toolpath& tp, std::size_t index, const geom2d::Regions& material,
               const geom2d::Aabb2& extent, double radius) {
    const double z = tp.moves[index].end.z();
    geom2d::Aabb2 box = extent;
    box.min -= Vec2::Constant(3.0 * radius + 5.0);
    box.max += Vec2::Constant(3.0 * radius + 5.0);
    detail::CellGrid grid(box, 0.1);
    grid.fill(material, true, 0.001);
    Vec3 from = tp.start;
    for (std::size_t k = 0; k < index; ++k) {
        const auto& m = tp.moves[k];
        if (m.kind != MoveKind::Rapid && std::abs(m.end.z() - z) < 1e-6) {
            std::vector<Vec2> pts;
            if (m.isArc()) {
                const Vec2 c = m.center.head<2>();
                const double rr = (from.head<2>() - c).norm();
                const double a0 = std::atan2(from.y() - c.y(), from.x() - c.x());
                double a1 = std::atan2(m.end.y() - c.y(), m.end.x() - c.x());
                if (m.kind == MoveKind::ArcCCW) {
                    while (a1 <= a0 + 1e-12) {
                        a1 += 2.0 * kPi;
                    }
                } else {
                    while (a1 >= a0 - 1e-12) {
                        a1 -= 2.0 * kPi;
                    }
                }
                const int n = std::max(2, static_cast<int>(std::abs(a1 - a0) * rr / 0.05));
                for (int s = 0; s <= n; ++s) {
                    const double a = a0 + (a1 - a0) * s / n;
                    pts.push_back(c + rr * Vec2(std::cos(a), std::sin(a)));
                }
            } else {
                const double len = (m.end - from).head<2>().norm();
                const int n = std::max(1, static_cast<int>(len / 0.05));
                for (int s = 0; s <= n; ++s) {
                    pts.push_back(from.head<2>() + (m.end - from).head<2>() * (double(s) / n));
                }
            }
            for (const Vec2& q : pts) {
                grid.clearDisk(q, radius);
            }
        }
        from = m.end;
    }
    return !grid.touches(tp.moves[index].end.head<2>(), radius);
}

/// Largest width of cut (material removed per millimetre of travel, over 2 mm windows) of the
/// cutting moves at height z, replayed on a 0.1 mm grid of a rectangular stock. Each entry
/// (a plunge or ramp) and the `lead` millimetres after it are left out: entries remove material
/// without sideways travel, and the shallow lead into a pass is measured by `leadWidth`.
struct WidthOfCut {
    double steady = 0.0;    ///< Largest width away from entries.
    Vec2 at = Vec2::Zero(); ///< Where it is.
    double total = 0.0;     ///< Travel measured [mm].
};
WidthOfCut widthOfCut(const toolpath::Toolpath& tp, double z, const geom2d::Aabb2& stock,
                      double radius, double lead) {
    const double h = 0.1;
    const double pad = 3.0 * radius + 10.0;
    const Vec2 origin = stock.min - Vec2::Constant(pad);
    const int nx = static_cast<int>((stock.max.x() - stock.min.x() + 2.0 * pad) / h) + 1;
    const int ny = static_cast<int>((stock.max.y() - stock.min.y() + 2.0 * pad) / h) + 1;
    std::vector<char> material(static_cast<std::size_t>(nx) * ny, 0);
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            const Vec2 q = origin + Vec2(i, j) * h;
            material[static_cast<std::size_t>(i) * ny + j] =
                q.x() >= stock.min.x() && q.x() <= stock.max.x() && q.y() >= stock.min.y() &&
                q.y() <= stock.max.y();
        }
    }
    const int rc = static_cast<int>(std::ceil(radius / h)) + 1;
    struct Sample {
        double removed;
        double length;
        bool afterEntry;
        Vec2 at;
    };
    std::vector<Sample> samples;
    double sinceEntry = std::numeric_limits<double>::infinity();
    Vec3 from = tp.start;
    for (const auto& m : tp.moves) {
        const bool atZ = std::abs(from.z() - z) < 1e-6 && std::abs(m.end.z() - z) < 1e-6;
        if (m.feedClass == FeedClass::Plunge || m.feedClass == FeedClass::Ramp) {
            sinceEntry = 0.0;
        }
        if (atZ && m.kind != MoveKind::Rapid) {
            std::vector<Vec2> pts;
            if (m.isArc()) {
                const Vec2 c = m.center.head<2>();
                const double rr = (from.head<2>() - c).norm();
                const double a0 = std::atan2(from.y() - c.y(), from.x() - c.x());
                double a1 = std::atan2(m.end.y() - c.y(), m.end.x() - c.x());
                if (m.kind == MoveKind::ArcCCW && a1 <= a0) {
                    a1 += 2.0 * kPi;
                } else if (m.kind == MoveKind::ArcCW && a1 >= a0) {
                    a1 -= 2.0 * kPi;
                }
                const int n = std::max(2, static_cast<int>(std::abs(a1 - a0) * rr / 0.05));
                for (int s = 0; s <= n; ++s) {
                    const double a = a0 + (a1 - a0) * s / n;
                    pts.push_back(c + rr * Vec2(std::cos(a), std::sin(a)));
                }
            } else {
                const double len = (m.end - from).head<2>().norm();
                const int n = std::max(1, static_cast<int>(len / 0.05));
                for (int s = 0; s <= n; ++s) {
                    pts.push_back(from.head<2>() + (m.end - from).head<2>() * (double(s) / n));
                }
            }
            for (std::size_t k = 1; k < pts.size(); ++k) {
                const double step = (pts[k] - pts[k - 1]).norm();
                const int ci = static_cast<int>(std::lround((pts[k].x() - origin.x()) / h));
                const int cj = static_cast<int>(std::lround((pts[k].y() - origin.y()) / h));
                int removed = 0;
                for (int i = std::max(0, ci - rc); i <= std::min(nx - 1, ci + rc); ++i) {
                    for (int j = std::max(0, cj - rc); j <= std::min(ny - 1, cj + rc); ++j) {
                        char& cell = material[static_cast<std::size_t>(i) * ny + j];
                        if (cell &&
                            (origin + Vec2(i, j) * h - pts[k]).squaredNorm() <= radius * radius) {
                            cell = 0;
                            ++removed;
                        }
                    }
                }
                sinceEntry += step;
                samples.push_back({removed * h * h, step, sinceEntry <= lead + 1.0, pts[k]});
            }
        }
        from = m.end;
    }
    WidthOfCut out;
    for (const Sample& s : samples) {
        out.total += s.length;
    }
    // Sliding 2 mm windows that contain no entry stretch.
    double area = 0.0;
    double length = 0.0;
    std::size_t first = 0;
    for (std::size_t k = 0; k < samples.size(); ++k) {
        if (samples[k].afterEntry) {
            area = length = 0.0;
            first = k + 1;
            continue;
        }
        area += samples[k].removed;
        length += samples[k].length;
        while (length > 2.0 && first < k) {
            area -= samples[first].removed;
            length -= samples[first].length;
            ++first;
        }
        if (length >= 1.5 && area / length > out.steady) {
            out.steady = area / length;
            out.at = samples[k].at;
        }
    }
    return out;
}

/// Largest z - lo(d) over densely sampled points of every triangle near xy (the oracle).
double oracleDrop(const geom3d::Mesh& m, const Cutter& cutter, const Vec2& xy, double floor) {
    double best = floor;
    for (const auto& t : m.triangles) {
        const Vec3 a = m.vertices[t[0]], b = m.vertices[t[1]], c = m.vertices[t[2]];
        for (int u = 0; u <= 24; ++u) {
            for (int v = 0; u + v <= 24; ++v) {
                const Vec3 p = a + (b - a) * (u / 24.0) + (c - a) * (v / 24.0);
                const double d = (p.head<2>() - xy).norm();
                if (d <= cutter.radius()) {
                    best = std::max(best, p.z() - cutter.lo(d));
                }
            }
        }
    }
    return best;
}

auto dome(double R, double c, double base) {
    return [=](double x, double y) {
        const double q = R * R - x * x - y * y;
        return std::max(base, q > 0.0 ? std::sqrt(q) - c : base);
    };
}

} // namespace

TEST_CASE("cutter lower envelopes", "[strategies][surface]") {
    const Cutter flat = Cutter::torus(5, 0), ball = Cutter::torus(5, 5), bull = Cutter::torus(5, 1);
    CHECK(flat.lo(4.9) == 0.0);
    CHECK_THAT(ball.lo(3.0), WithinAbs(5.0 - 4.0, 1e-12));
    CHECK(bull.lo(4.0) == 0.0);
    CHECK_THAT(bull.lo(5.0), WithinAbs(1.0, 1e-12));
    const Cutter cone = Cutter::cone(6, 0.5, degToRad(45.0));
    CHECK_THAT(cone.lo(2.5), WithinAbs(2.0, 1e-12));
    // Facet contacts maximize s r - lo(r).
    for (const Cutter* c : {&flat, &ball, &bull, &cone}) {
        for (const double s : {0.1, 0.5, 1.0, 3.0}) {
            const auto [r, value] = c->facetContact(s);
            double brute = -1e9;
            for (int k = 0; k <= 20000; ++k) {
                const double rr = c->radius() * k / 20000.0;
                brute = std::max(brute, s * rr - c->lo(rr));
            }
            CHECK(brute <= value + 1e-9); // nothing better than the analytic contact
            CHECK(value - brute < 1e-3);  // up to the brute-force sampling step
            CHECK_THAT(s * r - c->lo(r), WithinAbs(value, 1e-9));
        }
    }
    CHECK_THROWS(Cutter(tooling::makeDefaultTool(tooling::ToolType::Lollipop, 1, 6.0)));
    const Cutter grown = ball.offset(0.5);
    CHECK(grown.radius() == 5.5);
    CHECK(grown.cornerRadius() == 5.5);
}

TEST_CASE("drop-cutter on analytic surfaces", "[strategies][surface]") {
    const double tanA = std::tan(degToRad(30.0));
    const auto plane =
        heightMesh([&](double x, double) { return x * tanA; }, -50, 50, -50, 50, 4, -60);
    const Cutter ball = Cutter::torus(4, 4), flat = Cutter::torus(4, 0), bull = Cutter::torus(4, 1);
    const double cosA = std::cos(degToRad(30.0));
    for (const double x : {-10.0, 0.0, 7.5}) {
        CHECK_THAT(DropCutter(plane, ball).drop({x, 3.0}, -100),
                   WithinAbs(x * tanA + 4.0 * (1.0 / cosA - 1.0), 1e-9));
        CHECK_THAT(DropCutter(plane, flat).drop({x, 3.0}, -100),
                   WithinAbs(x * tanA + 4.0 * tanA, 1e-9));
        // Bull nose: the corner torus touches; tip = plane + flat part * tan + corner term.
        const double expected = x * tanA + 3.0 * tanA + 1.0 * (1.0 / cosA - 1.0);
        CHECK_THAT(DropCutter(plane, bull).drop({x, 3.0}, -100), WithinAbs(expected, 1e-9));
    }

    // Ball on a finely tessellated dome: within the chord error of the analytic offset.
    const double Rs = 20.0, Rt = 3.0;
    const auto sphere = heightMesh(dome(Rs, 0.0, 0.0), -30, 30, -30, 30, 240, -5);
    const DropCutter onSphere(sphere, Cutter::torus(Rt, Rt));
    for (const double x : {0.0, 5.0, 10.0, 15.0}) {
        const double expected = std::sqrt((Rs + Rt) * (Rs + Rt) - x * x) - Rt;
        CHECK_THAT(onSphere.drop({x, 0.0}, -100), WithinAbs(expected, 0.03));
    }
    // Away from everything the floor is returned.
    CHECK(onSphere.drop({100.0, 100.0}, -7.0) == -7.0);
}

TEST_CASE("drop-cutter matches a dense-sampling oracle", "[strategies][surface]") {
    const auto part = heightMesh(dome(18.0, 6.0, 0.0), -20, 20, -20, 20, 24, -5);
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> u(-22.0, 22.0);
    for (const Cutter& cutter : {Cutter::torus(3, 3), Cutter::torus(3, 0), Cutter::torus(4, 1.5),
                                 Cutter::cone(5, 0.5, degToRad(45.0))}) {
        const DropCutter dc(part, cutter);
        for (int k = 0; k < 40; ++k) {
            const Vec2 xy(u(rng), u(rng));
            const double tip = dc.drop(xy, -50.0);
            const double oracle = oracleDrop(part, cutter, xy, -50.0);
            INFO("at " << xy.transpose());
            CHECK(oracle <= tip + 1e-7); // no sampled point inside the tool: no gouge
            CHECK(tip - oracle < 0.1);   // and touching (up to the sampling density)
        }
    }
}

TEST_CASE("forbidden regions and waterline around a boss", "[strategies][surface]") {
    // A cylindrical boss of radius 10, 10 high, on nothing.
    const auto boss =
        heightMesh([](double x, double y) { return x * x + y * y <= 100.0 ? 10.0 : 0.0; }, -15, 15,
                   -15, 15, 300, -1);
    Harness h(tooling::makeFlatEndMill(1, 6.0, 20.0, 50.0));
    WaterlineParams p;
    p.topZ = 9.0;
    p.bottomZ = 5.0;
    p.stepDown = 2.0;
    p.resolution = 0.25;
    const auto tp = waterline(boss, p, h.context());
    CHECK(tp.warnings.empty());
    int points = 0;
    for (const Vec3& q : feedPoints(tp)) {
        if (q.z() < 9.0 + 1e-9 && q.z() > 5.0 - 1e-9) {
            // The heightfield boss has a stair-stepped rim (grid 0.1): allow its size.
            CHECK_THAT(q.head<2>().norm(), WithinAbs(13.0, 0.15));
            ++points;
        }
    }
    CHECK(points > 50);
    // Climb: clockwise around the outside of the part.
    double area2 = 0.0;
    const auto paths = cutPathsAt(tp, 7.0);
    REQUIRE_FALSE(paths.empty());
    const auto& loop = paths.front();
    for (std::size_t k = 0; k + 1 < loop.size(); ++k) {
        area2 += loop[k].x() * loop[k + 1].y() - loop[k + 1].x() * loop[k].y();
    }
    CHECK(area2 < 0.0);
}

TEST_CASE("parallel finishing stays on the surface", "[strategies][surface]") {
    const auto part = heightMesh(dome(18.0, 6.0, 0.0), -20, 20, -20, 20, 40, -5);
    Harness h(tooling::makeBallEndMill(1, 6.0, 20.0, 50.0));
    ParallelParams p;
    p.scallopHeight = 0.02;
    const auto tp = parallelFinish(part, 0.0, p, h.context());
    CHECK(tp.warnings.empty());
    const Cutter cutter = Cutter::torus(3, 3);
    // Every feed point lies on the drop-cutter surface (no gouge, no air cut).
    const DropCutter dc(part, cutter);
    std::size_t checked = 0;
    for (const auto& m : tp.moves) {
        if (m.kind == MoveKind::Linear && m.feedClass == FeedClass::Cut && (checked++ % 37) == 0) {
            CHECK_THAT(m.end.z(), WithinAbs(dc.drop(m.end.head<2>(), 0.0), 1e-9));
        }
    }
    CHECK(checked > 100);
    // Stepover from the scallop height: 2 sqrt(2 r h - h^2). One-way lines each start with a
    // plunge, so their y values give the line spacing.
    p.zigzag = false;
    const auto oneWay = parallelFinish(part, 0.0, p, h.context());
    std::vector<double> ys;
    for (const auto& m : oneWay.moves) {
        if (m.feedClass == FeedClass::Plunge) {
            ys.push_back(m.end.y());
        }
    }
    REQUIRE(ys.size() > 10);
    const double expected = 2.0 * std::sqrt(2.0 * 3.0 * 0.02 - 0.02 * 0.02);
    CHECK_THAT(ys[1] - ys[0], WithinAbs(expected, 1e-6));
}

TEST_CASE("Z-level roughing keeps the stock allowance", "[strategies][surface]") {
    const auto boss =
        heightMesh([](double x, double y) { return x * x + y * y <= 100.0 ? 10.0 : 0.0; }, -15, 15,
                   -15, 15, 150, -1);
    Harness h(tooling::makeFlatEndMill(1, 8.0, 25.0, 60.0));
    ZLevelRoughParams p;
    p.stepDown = 2.5;
    p.stockToLeaveXY = 0.5;
    p.stockToLeaveZ = 0.5;
    const geom2d::Regions stock{{geom2d::Polyline::rectangle({-15, -15}, {15, 15}), {}}};
    const auto tp = zLevelRough(boss, stock, 10.0, 0.5, p, h.context());
    REQUIRE_FALSE(tp.moves.empty());
    for (const Vec3& q : feedPoints(tp)) {
        if (q.z() < 10.0 - 1e-9) {
            // Tool centre at least boss radius + tool radius + allowance from the axis (the
            // heightfield rim is stepped by about 0.2).
            CHECK(q.head<2>().norm() >= 10.0 + 4.0 + 0.5 - 0.25);
            CHECK(q.z() >= 0.5 - 1e-9);
        }
    }
    // Four evenly spaced levels from below the top down to 0.5.
    std::set<double> levels;
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::Cut) {
            levels.insert(std::round(m.end.z() * 1000.0) / 1000.0);
        }
    }
    CHECK(levels.size() == 4);
    CHECK(levels.count(0.5) == 1);
}

TEST_CASE("Z-level roughing adds levels at flat areas", "[strategies][surface]") {
    // A boss whose top (10) lies between two step-down levels from the stock top (11): without
    // its own level the boss top would keep almost a whole step instead of the allowance.
    const auto boss =
        heightMesh([](double x, double y) { return x * x + y * y <= 100.0 ? 10.0 : 0.0; }, -15, 15,
                   -15, 15, 150, -1);
    Harness h(tooling::makeFlatEndMill(1, 8.0, 25.0, 60.0));
    ZLevelRoughParams p;
    p.stepDown = 1.0;
    p.stockToLeaveXY = 0.5;
    p.stockToLeaveZ = 0.5;
    const geom2d::Regions stock{{geom2d::Polyline::rectangle({-15, -15}, {15, 15}), {}}};
    auto overBoss = [](const toolpath::Toolpath& tp, double z) {
        for (const Vec3& q : feedPoints(tp)) {
            if (std::abs(q.z() - z) < 1e-6 && q.head<2>().norm() < 5.0) {
                return true;
            }
        }
        return false;
    };
    const auto tp = zLevelRough(boss, stock, 11.0, 0.5, p, h.context());
    CHECK(overBoss(tp, 10.5));
    // The levels end on the flat and are evenly spaced below it, never more than a step apart.
    std::set<double> levels;
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::Cut) {
            levels.insert(std::round(m.end.z() * 1000.0) / 1000.0);
        }
    }
    REQUIRE(levels.size() >= 2);
    CHECK(*levels.rbegin() == 10.5);
    CHECK(*levels.begin() == 0.5);
    double previous = 11.0;
    for (auto it = levels.rbegin(); it != levels.rend(); ++it) {
        CHECK(previous - *it <= 1.0 + 1e-3);
        previous = *it;
    }
    // A whole level: the open stock around the boss is entered in air, without ramps or helixes.
    CHECK(std::none_of(tp.moves.begin(), tp.moves.end(),
                       [](const toolpath::Move& m) { return m.feedClass == FeedClass::Ramp; }));
    p.flatAreaLevels = false;
    const auto plain = zLevelRough(boss, stock, 11.0, 0.5, p, h.context());
    CHECK_FALSE(overBoss(plain, 10.5));
}

TEST_CASE("Z-level roughing enters open sides in air and pockets by a helix",
          "[strategies][surface]") {
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    ZLevelRoughParams p;
    p.stepDown = 2.0;
    p.entry = EntryKind::Helix;
    const geom2d::Regions stock{{geom2d::Polyline::rectangle({-15, -15}, {15, 15}), {}}};

    SECTION("a boss: entries plunge outside the stock or where the level is cut already") {
        const auto boss =
            heightMesh([](double x, double y) { return x * x + y * y <= 64.0 ? 10.0 : 0.0; }, -15,
                       15, -15, 15, 150, -1);
        const auto tp = zLevelRough(boss, stock, 10.0, 1.0, p, h.context());
        // The level's material: the stock around the boss.
        geom2d::Polyline bossOutline = geom2d::Polyline::circle({0, 0}, 8.0);
        bossOutline.reverse();
        const geom2d::Regions material{{stock.front().outer, {bossOutline}}};
        int outside = 0;
        for (std::size_t k = 0; k < tp.moves.size(); ++k) {
            const auto& m = tp.moves[k];
            CHECK(m.feedClass != FeedClass::Ramp);
            if (m.feedClass == FeedClass::Plunge) {
                // Distance from the tool axis to the stock.
                const Vec2 gap(std::max(std::abs(m.end.x()) - 15.0, 0.0),
                               std::max(std::abs(m.end.y()) - 15.0, 0.0));
                if (gap.norm() >= 3.0 - 1e-6) {
                    ++outside; // the whole tool is outside the stock
                } else {
                    INFO("plunge to " << m.end.transpose() << " (move " << k << ")");
                    CHECK(intoClear(tp, k, material, {Vec2(-15, -15), Vec2(15, 15)}, 2.95));
                }
            }
        }
        CHECK(outside > 0);
    }

    SECTION("a closed pocket: a helix inside the pocket, no plunge into its material") {
        const auto block =
            heightMesh([](double x, double y) { return x * x + y * y <= 81.0 ? 2.0 : 10.0; }, -15,
                       15, -15, 15, 150, -1);
        const auto tp = zLevelRough(block, stock, 10.0, 2.0, p, h.context());
        const geom2d::Regions material{{geom2d::Polyline::circle({0, 0}, 9.0), {}}};
        int helixTurns = 0;
        for (std::size_t k = 0; k < tp.moves.size(); ++k) {
            const auto& m = tp.moves[k];
            if (m.isArc() && m.feedClass == FeedClass::Ramp) {
                CHECK(m.center.head<2>().norm() < 9.0 - 3.0);
                ++helixTurns;
            }
            if (m.feedClass == FeedClass::Plunge) {
                INFO("plunge to " << m.end.transpose() << " (move " << k << ")");
                CHECK(intoClear(tp, k, material, {Vec2(-9, -9), Vec2(9, 9)}, 3.0));
            }
        }
        CHECK(helixTurns > 0);
    }
}

TEST_CASE("Z-level rest roughing ignores the part in the in-process stock",
          "[strategies][surface]") {
    // The in-process stock holds the boss itself plus rest material in one corner: only the
    // corner is roughed, the boss's walls are not traced again.
    const auto boss =
        heightMesh([](double x, double y) { return x * x + y * y <= 64.0 ? 10.0 : 0.0; }, -15, 15,
                   -15, 15, 150, -1);
    Harness h(tooling::makeFlatEndMill(1, 4.0, 25.0, 60.0));
    ZLevelRoughParams p;
    p.stepDown = 2.0;
    p.stockToLeaveXY = 0.0;
    p.stockToLeaveZ = 0.0;
    p.material = [](double, double) {
        return geom2d::Regions{{geom2d::Polyline::circle({0, 0}, 8.2), {}},
                               {geom2d::Polyline::rectangle({11, 11}, {15, 15}), {}}};
    };
    p.materialSlack = 0.5;
    const geom2d::Regions stock{{geom2d::Polyline::rectangle({-15, -15}, {15, 15}), {}}};
    const auto tp = zLevelRough(boss, stock, 10.0, 1.0, p, h.context());
    REQUIRE_FALSE(tp.moves.empty());
    for (const Vec3& q : feedPoints(tp)) {
        if (q.z() < 10.0 - 1e-9) {
            CHECK(q.x() > 5.0); // near the corner only
            CHECK(q.y() > 5.0);
        }
    }
}

TEST_CASE("Z-level roughing cuts open areas at most a stepover wide", "[strategies][surface]") {
    // A boss in a square stock: the band around it is open (air outside the stock). Passes move
    // in one stepover at a time from the air, so away from entries the tool cuts about a
    // stepover wide and never a slot (the full diameter, 2.5 stepovers here). Short stretches
    // where a pass meets the round wall reach about 1.4 stepovers; offset passes cannot avoid
    // that (DL-0018).
    const auto boss =
        heightMesh([](double x, double y) { return x * x + y * y <= 64.0 ? 10.0 : 0.0; }, -20, 20,
                   -20, 20, 200, -1);
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    ZLevelRoughParams p;
    p.stepDown = 5.0;
    p.stockToLeaveXY = 0.0;
    p.stockToLeaveZ = 0.0;
    p.flatAreaLevels = false;
    const geom2d::Regions stock{rect(-18, -18, 18, 18)};
    const auto tp = zLevelRough(boss, stock, 10.0, 0.0, p, h.context());
    const double stepover = p.stepoverRatio * 6.0;
    const double lead = stepover / std::tan(20.0 * kPi / 180.0);
    const geom2d::Aabb2 box{Vec2(-18, -18), Vec2(18, 18)};
    for (const double z : {5.0, 0.0}) {
        INFO("z = " << z);
        const WidthOfCut w = widthOfCut(tp, z, box, 3.0, lead);
        INFO("largest at " << w.at.transpose());
        CHECK(w.total > 100.0);
        CHECK(w.steady <= 1.6 * stepover);
    }
}

TEST_CASE("Z-level roughing does not follow loops that cut only air", "[strategies][surface]") {
    // Tool centres may run r + 0.2 r outside the stock for air entries; a loop that far out
    // cuts nothing and is not followed.
    const auto boss = heightMesh(
        [](double x, double y) { return std::abs(x) <= 12.0 && std::abs(y) <= 12.0 ? 10.0 : 0.0; },
        -15, 15, -15, 15, 150, -1);
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    ZLevelRoughParams p;
    p.stepDown = 5.0;
    const geom2d::Regions stock{{geom2d::Polyline::rectangle({-14, -14}, {14, 14}), {}}};
    const auto tp = zLevelRough(boss, stock, 10.0, 0.5, p, h.context());
    REQUIRE_FALSE(tp.moves.empty());
    int plunges = 0;
    for (std::size_t k = 0; k < tp.moves.size(); ++k) {
        const auto& m = tp.moves[k];
        if (m.feedClass == FeedClass::Plunge) {
            ++plunges; // in the air outside the stock, or into the area a path cut there
            INFO("plunge " << k << " of " << tp.moves.size() << " to " << m.end.transpose()
                           << ", next move class "
                           << (k + 1 < tp.moves.size() ? int(tp.moves[k + 1].feedClass) : -1));
            CHECK((std::max(std::abs(m.end.x()), std::abs(m.end.y())) > 14.0 + 2.0 ||
                   distanceToCut(tp, k, m.end.head<2>(), m.end.z()) < 2.95));
        } else if (m.feedClass == FeedClass::Cut && m.end.z() < 10.0) {
            // Every cut reaches the stock: its centre is less than a radius outside it.
            CHECK(std::max(std::abs(m.end.x()), std::abs(m.end.y())) < 14.0 + 3.0);
        }
    }
    CHECK(plunges > 0);
}

TEST_CASE("Z-level roughing takes the stairs off a rounded edge with fine levels",
          "[strategies][surface]") {
    // A round boss 12 in radius, 10 high, its top edge rounded with a radius of 6: the round
    // runs from radius 6 at 10 down to radius 12 at 4, then a vertical wall to the base at 0.
    const auto height = [](double rho) {
        return rho <= 6.0    ? 10.0
               : rho <= 12.0 ? 4.0 + std::sqrt(std::max(0.0, 36.0 - (rho - 6.0) * (rho - 6.0)))
                             : 0.0;
    };
    const auto boss = heightMesh([&](double x, double y) { return height(std::hypot(x, y)); }, -22,
                                 22, -22, 22, 220, -1);
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    const double r = 3.0;
    const geom2d::Regions stock = {rect(-20, -20, 20, 20)};
    ZLevelRoughParams p;
    p.stepDown = 12.0; // one level at the base: the round keeps a stair 6 tall
    p.stockToLeaveXY = 0.3;
    p.stockToLeaveZ = 0.3;
    p.flatAreaLevels = false;
    // The cut levels between the top and the base.
    const auto between = [](const toolpath::Toolpath& tp) {
        std::set<double> zs;
        for (const auto& m : tp.moves) {
            if (m.feedClass == FeedClass::Cut && m.end.z() > 0.5 && m.end.z() < 9.9) {
                zs.insert(std::round(m.end.z() * 1000.0) / 1000.0);
            }
        }
        return zs;
    };
    CHECK(between(zLevelRough(boss, stock, 10.0, 0.0, p, h.context())).empty());

    p.fineStepDown = 1.0;
    const auto tp = zLevelRough(boss, stock, 10.0, 0.0, p, h.context());
    INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
    CHECK(tp.warnings.empty());
    const auto zs = between(tp);
    CHECK(zs.size() >= 5); // fine levels down the round (those above it have nothing)
    // At each fine level the tool comes as close to the round as its allowance lets it: the
    // stair left there is the fine step at most.
    for (const double z : zs) {
        // Where the round is at this height (minus the Z allowance's reach).
        double edge = 6.0;
        while (edge < 12.0 && height(edge) > z - 0.3) {
            edge += 0.01;
        }
        double closest = 1e9;
        for (const Vec3& q : feedPoints(tp)) {
            if (std::abs(q.z() - z) < 1e-6) {
                closest = std::min(closest, q.head<2>().norm());
            }
        }
        INFO("level " << z << ": round at " << edge << ", closest centre " << closest);
        CHECK(closest <= edge + r + 0.3 + 0.6);
        // and no closer than the allowance lets it (the height field's grid is 0.75)
        CHECK(closest >= edge + r - 0.4);
    }
}

TEST_CASE("contour finishing spaces its levels by the cusp height", "[strategies][surface]") {
    // A round boss 12 in radius, 10 high, its top edge rounded with a radius of 6 (from radius 6
    // at 10 to radius 12 at 4), then a vertical wall to the base at 0.
    const auto height = [](double rho) {
        return rho <= 6.0    ? 10.0
               : rho <= 12.0 ? 4.0 + std::sqrt(std::max(0.0, 36.0 - (rho - 6.0) * (rho - 6.0)))
                             : 0.0;
    };
    const auto boss = heightMesh([&](double x, double y) { return height(std::hypot(x, y)); }, -20,
                                 20, -20, 20, 200, -1);
    Harness h(tooling::makeBallEndMill(1, 6.0, 25.0, 60.0));
    const double R = 3.0;
    ContourFinishParams p;
    p.scallopHeight = 0.01;
    p.topZ = 10.0;
    p.bottomZ = 2.5; // the round and a little of the wall below it
    const double smoothing = 0.005;
    const double sMax = 2.0 * std::sqrt(2.0 * R * p.scallopHeight - 1e-4);
    auto tp = contourFinish(boss, p, h.context());
    toolpath::fitArcs(tp, smoothing);
    INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
    CHECK(tp.warnings.empty());
    // Each level's loop is a circle around the axis: its radius from the cut moves' ends.
    std::map<double, std::pair<double, int>> radius; // z -> (sum, count)
    std::size_t arcs = 0;
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::Cut) {
            auto& [sum, count] = radius[std::round(m.end.z() * 1e4) / 1e4];
            sum += m.end.head<2>().norm();
            ++count;
            arcs += m.isArc() ? 1 : 0;
        }
    }
    REQUIRE(radius.size() > 10);
    // Consecutive levels: the passes are at most about the cusp's width apart along the tool
    // centre's surface (s = sqrt(dr^2 + dz^2)). On the vertical wall (below 4) the step is about
    // that width; near the top of the round, where it is shallow, far smaller.
    std::vector<std::pair<double, double>> levels; // (z, loop radius), top down
    for (auto it = radius.rbegin(); it != radius.rend(); ++it) {
        levels.emplace_back(it->first, it->second.first / it->second.second);
    }
    double wallStep = 0.0, topStep = 1e9;
    for (std::size_t k = 1; k < levels.size(); ++k) {
        const auto [z0, r0] = levels[k - 1];
        const auto [z1, r1] = levels[k];
        if (z0 < 9.0) { // below the shallowest walls, where the levels may skip
            INFO("level " << z1 << " radius " << r1);
            CHECK(std::hypot(r1 - r0, z0 - z1) <= 1.15 * sMax);
        }
        if (z0 < 3.9 && z1 > 2.6) {
            wallStep = std::max(wallStep, z0 - z1);
        }
        if (z1 > 8.5) {
            topStep = std::min(topStep, z0 - z1);
        }
    }
    CHECK(wallStep > 0.85 * sMax);
    CHECK(topStep < 0.5 * sMax);
    // Arcs: the circles come out as arcs, a few per level.
    CHECK(arcs >= radius.size());
    // No gouge beyond the smoothing tolerance: within that sideways of every feed point (an arc
    // may stand that far from the points it replaces), a ball 5 um smaller dropped there stays
    // at or below the point.
    const DropCutter dc(boss, Cutter::torus(R - 0.005, R - 0.005));
    for (const Vec3& q : feedPoints(tp)) {
        double lowest = dc.drop(q.head<2>(), -100.0);
        for (int k = 0; k < 8; ++k) {
            const double a = kPi / 4.0 * k;
            lowest = std::min(
                lowest, dc.drop(q.head<2>() + smoothing * Vec2(std::cos(a), std::sin(a)), -100.0));
        }
        CHECK(lowest <= q.z() + 1e-6);
    }
}

TEST_CASE("waterline finishing outputs arcs when smoothing", "[strategies][surface]") {
    const auto boss =
        heightMesh([](double x, double y) { return x * x + y * y <= 100.0 ? 10.0 : 0.0; }, -15, 15,
                   -15, 15, 150, -1);
    Harness h(tooling::makeBallEndMill(1, 6.0, 25.0, 60.0));
    WaterlineParams p;
    p.stepDown = 2.0;
    p.topZ = 10.0;
    p.bottomZ = 1.0;
    const auto plain = waterline(boss, p, h.context());
    auto smooth = plain;
    toolpath::fitArcs(smooth, 0.01);
    const auto count = [](const toolpath::Toolpath& tp, bool arcs) {
        return std::count_if(tp.moves.begin(), tp.moves.end(), [&](const toolpath::Move& m) {
            return m.feedClass == FeedClass::Cut && m.isArc() == arcs;
        });
    };
    CHECK(count(plain, true) == 0);
    CHECK(count(smooth, true) > 0);
    CHECK(smooth.moves.size() * 4 < plain.moves.size());
}

TEST_CASE("flat area finishing finds and covers every flat", "[strategies][surface][flat]") {
    // A boss (top at 10) on a step (at 4) on a base (at 0): three flat levels, walls between.
    const auto part = heightMesh(
        [](double x, double y) {
            const double m = std::max(std::abs(x), std::abs(y));
            return m <= 6.0 ? 10.0 : m <= 11.0 ? 4.0 : 0.0;
        },
        -16, 16, -16, 16, 160, -1);
    const auto levels = flatLevels(part, 0.5 * kPi / 180.0, 0.001, 0.002);
    REQUIRE(levels.size() == 3);
    CHECK_THAT(levels[0].z, WithinAbs(10.0, 1e-9));
    CHECK_THAT(levels[1].z, WithinAbs(4.0, 1e-9));
    CHECK_THAT(levels[2].z, WithinAbs(0.0, 1e-9));
    CHECK_THAT(geom2d::totalArea(levels[0].area), WithinAbs(12.0 * 12.0, 3.0));

    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    const double r = 3.0;
    FlatFinishParams p;
    SECTION("cuts at the flats only, without touching the part") {
        const auto tp = flatFinish(part, 10.0, 0.0, p, h.context());
        REQUIRE_FALSE(tp.moves.empty());
        std::set<double> cutLevels;
        for (const auto& m : tp.moves) {
            if (m.feedClass == FeedClass::Cut) {
                cutLevels.insert(std::round(m.end.z() * 1000.0) / 1000.0);
            }
        }
        CHECK(cutLevels == std::set<double>{0.0, 4.0, 10.0});
        // No gouge beyond the forbidden boundary's tolerance: on these 20:1 walls a micrometre
        // sideways is 0.02 in Z, so the check drops a tool 5 um smaller in radius.
        const DropCutter dc(part, Cutter::torus(r - 0.005, 0.0));
        for (const Vec3& q : feedPoints(tp)) {
            CHECK(dc.drop(q.head<2>(), -100.0) <= q.z() + 1e-6);
        }
        // Points of each flat that the flat bottom can reach are swept by it.
        auto covered = [&](double z, const Vec2& xy) {
            const auto swept = geom2d::sweep(cutPathsAt(tp, z), r, {0.002});
            return std::any_of(swept.begin(), swept.end(),
                               [&](const geom2d::Region& g) { return g.contains(xy, 1e-3); });
        };
        for (const Vec2& xy : {Vec2(0, 0), Vec2(5.8, 5.8), Vec2(-5.8, 0)}) {
            CHECK(covered(10.0, xy));
        }
        for (const Vec2& xy : {Vec2(8.5, 0), Vec2(10.8, 10.8), Vec2(-9, 9), Vec2(0, -6.6)}) {
            CHECK(covered(4.0, xy));
        }
        for (const Vec2& xy : {Vec2(13.5, 0), Vec2(15.8, 15.8), Vec2(-11.6, 3)}) {
            CHECK(covered(0.0, xy));
        }
    }
    SECTION("leaves stock on the walls and the floors") {
        p.stockToLeaveXY = 0.5;
        p.stockToLeaveZ = 0.2;
        const auto tp = flatFinish(part, 10.0, 0.0, p, h.context());
        for (const Vec3& q : feedPoints(tp)) {
            if (std::abs(q.z() - 4.2) < 1e-6) {
                // Clear of the boss by the tool radius and the allowance (grid step 0.2).
                const Vec2 out(std::max(std::abs(q.x()) - 6.0, 0.0),
                               std::max(std::abs(q.y()) - 6.0, 0.0));
                CHECK(out.norm() >= r + 0.5 - 0.25);
            }
        }
        std::set<double> cutLevels;
        for (const auto& m : tp.moves) {
            if (m.feedClass == FeedClass::Cut) {
                cutLevels.insert(std::round(m.end.z() * 1000.0) / 1000.0);
            }
        }
        CHECK(cutLevels == std::set<double>{0.2, 4.2, 10.2});
    }
    SECTION("open flats start in the air beside them") {
        p.stockOutline = {rect(-17, -17, 17, 17)};
        const auto tp = flatFinish(part, 10.0, 0.0, p, h.context());
        // Entries (the first plunge or ramp after a rapid), counted at the height of the cut
        // that follows: plunges beside the flat (out of its own outline by the tool radius)
        // separately from entries from above.
        const std::map<double, double> halfSize{{0.0, 16.0}, {4.0, 11.0}, {10.0, 6.0}};
        std::map<double, int> beside, above;
        std::optional<Vec2> pending; // an entry seen, its level not yet known: where
        bool pendingPlunge = false;
        bool afterRapid = false;
        for (const auto& m : tp.moves) {
            if (afterRapid &&
                (m.feedClass == FeedClass::Plunge || m.feedClass == FeedClass::Ramp)) {
                pending = m.end.head<2>();
                pendingPlunge = m.feedClass == FeedClass::Plunge;
            }
            if (pending && m.feedClass == FeedClass::Cut) {
                const double z = std::round(m.end.z() * 1000.0) / 1000.0;
                const double m2 = std::max(std::abs(pending->x()), std::abs(pending->y()));
                (pendingPlunge && m2 >= halfSize.at(z) + r - 0.3 ? beside : above)[z]++;
                pending.reset();
            }
            afterRapid = m.kind == MoveKind::Rapid;
        }
        // The base and the step are open to the side, and the boss top drops away to the step
        // all round: all three start with a plunge in the air beside them (and are then cut from
        // the outside in, each loop taking at most a stepover); none is entered from above.
        CHECK(beside[0.0] >= 1);
        CHECK(beside[4.0] >= 1);
        CHECK(beside[10.0] >= 1);
        CHECK(above.empty());
        // Still no gouge and every flat covered.
        const DropCutter dc(part, Cutter::torus(r - 0.005, 0.0));
        for (const Vec3& q : feedPoints(tp)) {
            CHECK(dc.drop(q.head<2>(), -100.0) <= q.z() + 1e-6);
        }
        for (const auto& [z, xy] : std::vector<std::pair<double, Vec2>>{
                 {10.0, {5.8, 5.8}}, {4.0, {8.5, 0}}, {4.0, {-9, 9}}, {0.0, {13.5, 0}}}) {
            const auto swept = geom2d::sweep(cutPathsAt(tp, z), r, {0.002});
            CHECK(std::any_of(swept.begin(), swept.end(),
                              [&](const geom2d::Region& g) { return g.contains(xy, 1e-3); }));
        }
    }
    SECTION("heights limit the levels; ball mills are refused") {
        const auto tp = flatFinish(part, 5.0, 1.0, p, h.context());
        std::set<double> cutLevels;
        for (const auto& m : tp.moves) {
            if (m.feedClass == FeedClass::Cut) {
                cutLevels.insert(std::round(m.end.z() * 1000.0) / 1000.0);
            }
        }
        CHECK(cutLevels == std::set<double>{4.0});
        Harness ball(tooling::makeBallEndMill(1, 6.0, 25.0, 60.0));
        const auto none = flatFinish(part, 10.0, 0.0, p, ball.context());
        REQUIRE_FALSE(none.warnings.empty());
        CHECK(none.warnings.front().find("flat bottom") != std::string::npos);
    }
}

TEST_CASE("flat area finishing machines over narrow openings", "[strategies][surface][flat]") {
    // A plate (top at 10) with a slot 6 wide and a bore 16 across through it.
    const auto part = heightMesh(
        [](double x, double y) {
            const bool plate = std::abs(x) <= 20.0 && std::abs(y) <= 20.0;
            const bool slot = std::abs(x) <= 12.0 && std::abs(y - 10.0) <= 3.0;
            const bool bore = x * x + (y + 6.0) * (y + 6.0) <= 64.0;
            return plate && !slot && !bore ? 10.0 : 0.0;
        },
        -26, 26, -26, 26, 260, -1);
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    FlatFinishParams p;
    // Points along the paths at the top, every 0.2.
    const auto pointsAtTop = [](const toolpath::Toolpath& tp) {
        std::vector<Vec2> pts;
        for (const auto& path : cutPathsAt(tp, 10.0)) {
            for (std::size_t k = 0; k + 1 < path.size(); ++k) {
                const Vec2 d = path[k + 1] - path[k];
                const int n = std::max(1, static_cast<int>(std::ceil(d.norm() / 0.2)));
                for (int s = 0; s <= n; ++s) {
                    pts.push_back(path[k] + d * (static_cast<double>(s) / n));
                }
            }
        }
        return pts;
    };
    // Along the slot's middle, and deep in the bore (past the rim the passes may reach).
    const auto overSlot = [](const Vec2& q) {
        return std::abs(q.x()) < 10.0 && std::abs(q.y() - 10.0) < 1.0;
    };
    const auto inBore = [](const Vec2& q) {
        return (q - Vec2(0.0, -6.0)).norm() < 8.0 - 1.2;
    };

    SECTION("off: the passes go around both") {
        const auto pts = pointsAtTop(flatFinish(part, 10.0, 5.0, p, h.context()));
        REQUIRE_FALSE(pts.empty());
        CHECK(std::none_of(pts.begin(), pts.end(), overSlot));
        CHECK(std::none_of(pts.begin(), pts.end(), inBore));
    }
    SECTION("on, at 10: across the slot, around the bore") {
        p.bridgeOpenings = 10.0;
        const auto tp = flatFinish(part, 10.0, 5.0, p, h.context());
        INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
        CHECK(tp.warnings.empty());
        const auto pts = pointsAtTop(tp);
        CHECK(std::count_if(pts.begin(), pts.end(), overSlot) > 20);
        CHECK(std::none_of(pts.begin(), pts.end(), inBore));
        // Still on the flat's height only, and no gouge.
        const DropCutter dc(part, Cutter::torus(3.0 - 0.005, 0.0));
        for (const Vec3& q : feedPoints(tp)) {
            CHECK(dc.drop(q.head<2>(), -100.0) <= q.z() + 1e-6);
        }
    }
    SECTION("on, at 20: across both") {
        p.bridgeOpenings = 20.0;
        const auto pts = pointsAtTop(flatFinish(part, 10.0, 5.0, p, h.context()));
        CHECK(std::count_if(pts.begin(), pts.end(), overSlot) > 20);
        CHECK(std::count_if(pts.begin(), pts.end(), inBore) > 20);
    }
}

TEST_CASE("flat area finishing enters beside a flat clear of its edge's stock",
          "[strategies][surface][flat]") {
    // A round boss with a flat top (radius 8, at 10), a 45 degree chamfer down to 6 at radius
    // 12, then a vertical wall to the base at 0. Roughing leaves stock above the chamfer (it is
    // not a flat): a plunge beside the top must land beyond it, past the wall.
    const auto part = heightMesh(
        [](double x, double y) {
            const double rho = std::hypot(x, y);
            return rho <= 8.0 ? 10.0 : rho <= 12.0 ? 10.0 - (rho - 8.0) : 0.0;
        },
        -24, 24, -24, 24, 240, -1);
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    const double r = 3.0;
    FlatFinishParams p;
    const auto tp = flatFinish(part, 10.0, 5.0, p, h.context());
    INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
    CHECK(tp.warnings.empty());
    int plunges = 0;
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::Plunge) {
            ++plunges;
            const double at = std::hypot(m.end.x(), m.end.y());
            INFO("plunge at " << m.end.transpose());
            CHECK(at >= 12.0 + r - 0.2); // clear of the wall, not over the chamfer
        }
    }
    CHECK(plunges >= 1); // still entered from the side
    CHECK(std::none_of(tp.moves.begin(), tp.moves.end(),
                       [](const toolpath::Move& m) { return m.feedClass == FeedClass::Ramp; }));
}

TEST_CASE("flat area finishing closes bands without clean-up loops",
          "[strategies][surface][flat]") {
    // A ring-shaped flat (top at 10) from radius 10 to 28: the tool centres' band is 19.5 wide,
    // so with the stepover at the full flat diameter (6) the loops a whole stepover in from
    // either side end 7.5 apart, a strip more than the flat radius from both.
    const auto part = heightMesh(
        [](double x, double y) {
            const double rho = std::hypot(x, y);
            return rho >= 10.0 && rho <= 28.0 ? 10.0 : 0.0;
        },
        -34, 34, -34, 34, 340, -1);
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    FlatFinishParams p;
    p.stepover = 6.0;
    const auto tp = flatFinish(part, 10.0, 5.0, p, h.context());
    INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
    CHECK(tp.warnings.empty());
    // Entered once (a plunge in the air beside the ring), no clean-up loops plunged into
    // afterwards.
    CHECK(std::count_if(tp.moves.begin(), tp.moves.end(), [](const toolpath::Move& m) {
              return m.feedClass == FeedClass::Plunge;
          }) == 1);
    // The whole ring is swept by the flat bottom, its middle included.
    const auto swept = geom2d::sweep(cutPathsAt(tp, 10.0), 3.0, {0.002});
    for (double rho = 10.5; rho <= 27.5; rho += 0.5) {
        for (double a = 0.0; a < 6.28; a += 0.7) {
            const Vec2 q(rho * std::cos(a), rho * std::sin(a));
            INFO("at " << q.transpose());
            CHECK(std::any_of(swept.begin(), swept.end(),
                              [&](const geom2d::Region& g) { return g.contains(q, 1e-3); }));
        }
    }
}

TEST_CASE("plunge roughing covers the area with overlapping plunges", "[strategies][plunge]") {
    Harness h(tooling::makeFlatEndMill(1, 10.0, 30.0, 70.0));
    const geom2d::Regions area{{geom2d::Polyline::rectangle({0, 0}, {60, 40}), {}}};
    PlungeParams p;
    const auto tp = plungeRough(area, 0.0, -12.0, p, h.context());
    REQUIRE(tp.warnings.empty());
    std::vector<Vec2> centres;
    int plunges = 0;
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::Plunge) {
            ++plunges;
            CHECK(m.end.z() == Catch::Approx(-12.0));
            // The whole tool stays inside the area.
            CHECK(m.end.x() >= 5.0 - 1e-6);
            CHECK(m.end.x() <= 55.0 + 1e-6);
            CHECK(m.end.y() >= 5.0 - 1e-6);
            CHECK(m.end.y() <= 35.0 + 1e-6);
            centres.push_back(m.end.head<2>());
        } else if (m.kind == MoveKind::Rapid) {
            CHECK(m.end.z() >= -1e-9); // never sideways inside the material
        }
    }
    CHECK(plunges > 20);
    // Everything a round tool can reach is cut, except the cusps plunges a pitch apart leave
    // on the wall (r - sqrt(r^2 - (pitch / 2)^2) deep): every point of the area opened by the
    // tool radius and shrunk by the cusp height lies within a radius of some plunge.
    const double cusp = 5.0 - std::sqrt(25.0 - 2.5 * 2.5);
    const geom2d::Regions reachable = geom2d::offset(
        geom2d::offset(geom2d::offset(area, -5.0, {0.002}), 5.0, {0.002}), -(cusp + 0.05), {0.002});
    int uncovered = 0;
    for (double x = 0.25; x < 60.0; x += 0.5) {
        for (double y = 0.25; y < 40.0; y += 0.5) {
            const Vec2 q(x, y);
            if (!reachable.front().contains(q)) {
                continue;
            }
            const bool cut = std::any_of(centres.begin(), centres.end(), [&](const Vec2& c) {
                return (c - q).norm() <= 5.0 + 1e-6;
            });
            uncovered += cut ? 0 : 1;
        }
    }
    CHECK(uncovered == 0);

    // Too wide a spacing is reported.
    p.stepoverRatio = 1.0;
    p.pitch = 10.0; // the tool diameter
    p.stagger = false;
    CHECK_FALSE(plungeRough(area, 0.0, -12.0, p, h.context()).warnings.empty());
}

TEST_CASE("waterline and contour finishing stay inside their containment",
          "[strategies][surface]") {
    // A cylindrical boss of radius 10, 10 high; tool centres contained to x >= 2.
    const auto boss =
        heightMesh([](double x, double y) { return x * x + y * y <= 100.0 ? 10.0 : 0.0; }, -15, 15,
                   -15, 15, 150, -1);
    Harness h(tooling::makeBallEndMill(1, 6.0, 25.0, 60.0));
    const geom2d::Regions half{{geom2d::Polyline::rectangle(Vec2(2, -30), Vec2(30, 30)), {}}};
    const auto check = [&](const toolpath::Toolpath& all, const toolpath::Toolpath& contained) {
        std::size_t cuts = 0;
        for (const auto& m : contained.moves) {
            if (m.feedClass == FeedClass::Cut) {
                CHECK(m.end.x() >= 2.0 - 0.01);
                ++cuts;
            }
        }
        CHECK(cuts > 10);
        // About 81 of the 360 degrees on either side of the X axis are inside.
        const double ratio =
            toolpath::computeStats(contained).cutLength / toolpath::computeStats(all).cutLength;
        CHECK(ratio > 0.3);
        CHECK(ratio < 0.6);
    };
    SECTION("waterline") {
        WaterlineParams p;
        p.topZ = 8.0;
        p.bottomZ = 4.0;
        p.stepDown = 2.0;
        p.resolution = 0.25;
        const auto all = waterline(boss, p, h.context());
        p.boundary = half;
        check(all, waterline(boss, p, h.context()));
    }
    SECTION("contour finishing") {
        ContourFinishParams p;
        p.scallopHeight = 0.05;
        p.topZ = 8.0;
        p.bottomZ = 5.0;
        p.resolution = 0.25;
        const auto all = contourFinish(boss, p, h.context());
        p.boundary = half;
        const auto contained = contourFinish(boss, p, h.context());
        CHECK(contained.warnings.empty());
        check(all, contained);
    }
}

TEST_CASE("flat area finishing on the selected flats only", "[strategies][surface][flat]") {
    // The boss on a step on a base; only the boss top (its plan view) is selected.
    const auto part = heightMesh(
        [](double x, double y) {
            const double m = std::max(std::abs(x), std::abs(y));
            return m <= 6.0 ? 10.0 : m <= 11.0 ? 4.0 : 0.0;
        },
        -16, 16, -16, 16, 80, -1);
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    FlatFinishParams p;
    p.only = {{geom2d::Polyline::rectangle(Vec2(-6, -6), Vec2(6, 6)), {}}};
    const auto tp = flatFinish(part, 10.0, 0.0, p, h.context());
    std::set<double> cutLevels;
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::Cut) {
            cutLevels.insert(std::round(m.end.z() * 1000.0) / 1000.0);
        }
    }
    CHECK(cutLevels == std::set<double>{10.0});
}

// -- Pencil and steep and shallow finishing (DL-0080) ----------------------------------------

namespace {

/// The surface of a profile in XZ (x increasing) swept along Y from y0 to y1: open, the top
/// surface only, which is all the drop-cutter needs.
geom3d::Mesh sweptProfile(const std::vector<Vec2>& xz, double y0, double y1) {
    geom3d::Mesh m;
    for (const Vec2& q : xz) {
        m.vertices.emplace_back(q.x(), y0, q.y());
        m.vertices.emplace_back(q.x(), y1, q.y());
    }
    for (std::uint32_t k = 0; k + 1 < xz.size(); ++k) {
        const std::uint32_t a = 2 * k, b = 2 * k + 1, c = 2 * k + 2, d = 2 * k + 3;
        m.triangles.push_back({a, c, d});
        m.triangles.push_back({a, d, b});
    }
    return m;
}

/// A channel's profile: tops at `top` outside x in [a, b], vertical walls at a and b, a floor at
/// 0, and fillets of radius rf between the walls and the floor.
std::vector<Vec2> filletChannel(double a, double b, double rf, double top, double margin) {
    std::vector<Vec2> pts{{a - margin, top}, {a, top}, {a, rf}};
    constexpr int kSteps = 24;
    for (int k = 1; k <= kSteps; ++k) {
        const double t = kPi + 0.5 * kPi * k / kSteps;
        pts.emplace_back(a + rf + rf * std::cos(t), rf + rf * std::sin(t));
    }
    for (int k = 0; k <= kSteps; ++k) {
        const double t = 1.5 * kPi + 0.5 * kPi * k / kSteps;
        pts.emplace_back(b - rf + rf * std::cos(t), rf + rf * std::sin(t));
    }
    pts.emplace_back(b, top);
    pts.emplace_back(b + margin, top);
    return pts;
}

/// With OCAM_TOOLPATH_DUMP set, the toolpath as JSON in that folder (for plots).
void dumpToolpath(const std::string& name, const toolpath::Toolpath& tp) {
    if (const char* dir = std::getenv("OCAM_TOOLPATH_DUMP")) {
        std::ofstream(std::filesystem::path(dir) / (name + ".json")) << toolpath::toJson(tp).dump();
    }
}

/// Points of the cutting moves (and links) at most `step` apart.
std::vector<Vec3> densePoints(const toolpath::Toolpath& tp, double step) {
    std::vector<Vec3> out;
    for (const auto& run : toolpath::sampleRuns(tp, 0.0005)) {
        if (run.feedClass != FeedClass::Cut && run.feedClass != FeedClass::Link) {
            continue;
        }
        for (std::size_t k = 0; k + 1 < run.points.size(); ++k) {
            const Vec3& a = run.points[k];
            const Vec3& b = run.points[k + 1];
            const int n = std::max(1, static_cast<int>(std::ceil((b - a).norm() / step)));
            for (int s = 0; s < n; ++s) {
                out.push_back(a + (b - a) * (static_cast<double>(s) / n));
            }
        }
        if (!run.points.empty()) {
            out.push_back(run.points.back());
        }
    }
    return out;
}

/// For a ball of radius r whose tips are `tips`: at each point, the distance from the surface
/// point to the nearest ball (negative: inside one, a gouge). Plan hashing on the centres.
class BallGap {
public:
    BallGap(const std::vector<Vec3>& tips, double r) : r_(r) {
        for (const Vec3& t : tips) {
            const Vec3 c = t + Vec3(0, 0, r);
            cells_[key(cell(c.x()), cell(c.y()))].push_back(c);
        }
    }
    double at(const Vec3& p) const {
        double best = std::numeric_limits<double>::infinity();
        const auto reach = static_cast<std::int64_t>(std::ceil((r_ + 0.3) / kCell));
        const auto ix = cell(p.x()), iy = cell(p.y());
        for (std::int64_t dx = -reach; dx <= reach; ++dx) {
            for (std::int64_t dy = -reach; dy <= reach; ++dy) {
                const auto it = cells_.find(key(ix + dx, iy + dy));
                if (it == cells_.end()) {
                    continue;
                }
                for (const Vec3& c : it->second) {
                    best = std::min(best, (c - p).norm());
                }
            }
        }
        return best - r_;
    }

private:
    static constexpr double kCell = 0.5;
    static std::int64_t cell(double v) { return static_cast<std::int64_t>(std::floor(v / kCell)); }
    static std::int64_t key(std::int64_t x, std::int64_t y) { return (x << 32) ^ (y & 0xffffffff); }
    double r_;
    std::unordered_map<std::int64_t, std::vector<Vec3>> cells_;
};

/// Points along a toolpath's cutting moves, at most 0.25 mm apart, their ends included.
std::vector<Vec3> cutPoints(const toolpath::Toolpath& tp) {
    std::vector<Vec3> pts;
    for (const auto& run : toolpath::sampleRuns(tp, 0.001)) {
        if (run.feedClass == FeedClass::Cut) {
            for (std::size_t k = 0; k + 1 < run.points.size(); ++k) {
                const Vec3& p = run.points[k];
                const Vec3& q = run.points[k + 1];
                const int n = std::max(1, static_cast<int>(std::ceil((q - p).norm() / 0.25)));
                for (int s = 0; s < n; ++s) {
                    pts.push_back(p + (q - p) * (static_cast<double>(s) / n));
                }
            }
            if (!run.points.empty()) {
                pts.push_back(run.points.back());
            }
        }
    }
    return pts;
}

} // namespace

TEST_CASE("pencil passes run along the fillet tangency", "[strategies][surface][pencil]") {
    // A channel 20 wide along Y, its floor edges filleted with a radius of 3.
    constexpr double a = 10.0, b = 30.0, rf = 3.0;
    const auto part = sweptProfile(filletChannel(a, b, rf, 15.0, 10.0), 0.0, 60.0);

    SECTION("a tool smaller than the fillet after a larger one: one pass along each fillet") {
        const double r = 2.0;
        Harness h(tooling::makeBallEndMill(1, 2.0 * r, 20.0, 50.0));
        PencilParams p;
        p.referenceRadius = 6.0;
        const auto tp = pencilFinish(part, p, h.context());
        dumpToolpath("pencil_channel", tp);
        CHECK(tp.warnings.empty());
        // The ball touches each fillet halfway round, between the wall's and the floor's
        // tangency lines: its centre rf - r from the fillet's axis, at 45 degrees.
        const double off = (rf - r) / std::sqrt(2.0);
        double leftMin = 1e9, leftMax = -1e9, rightMin = 1e9, rightMax = -1e9;
        std::size_t checked = 0;
        for (const Vec3& q : cutPoints(tp)) {
            const bool left = q.x() < 0.5 * (a + b);
            double& lo = left ? leftMin : rightMin;
            double& hi = left ? leftMax : rightMax;
            lo = std::min(lo, q.y());
            hi = std::max(hi, q.y());
            if (q.y() < 6.0 || q.y() > 54.0) {
                continue; // the channel's ends
            }
            const Vec2 axis(left ? a + rf : b - rf, rf);
            const Vec2 centre(q.x(), q.z() + r);
            INFO("at " << q.transpose());
            CHECK_THAT((centre - axis).norm(), WithinAbs(rf - r, 0.01));
            CHECK_THAT(std::abs(centre.x() - axis.x()), WithinAbs(off, 0.02));
            CHECK_THAT(axis.y() - centre.y(), WithinAbs(off, 0.02));
            ++checked;
        }
        CHECK(checked > 100);
        // Each pass runs the channel's length.
        CHECK(leftMin < 2.0);
        CHECK(leftMax > 58.0);
        CHECK(rightMin < 2.0);
        CHECK(rightMax > 58.0);
    }

    SECTION("a tool larger than the fillet: where it touches the wall and the floor") {
        const double r = 4.0;
        Harness h(tooling::makeBallEndMill(1, 2.0 * r, 20.0, 50.0));
        PencilParams p; // the reference is the tool itself
        const auto tp = pencilFinish(part, p, h.context());
        CHECK(tp.warnings.empty());
        std::size_t checked = 0;
        for (const Vec3& q : cutPoints(tp)) {
            if (q.y() < 6.0 || q.y() > 54.0) {
                continue;
            }
            const bool left = q.x() < 0.5 * (a + b);
            CHECK_THAT(q.x(), WithinAbs(left ? a + r : b - r, 0.02));
            CHECK_THAT(q.z(), WithinAbs(0.0, 0.01));
            ++checked;
        }
        CHECK(checked > 100);
    }

    SECTION("offset passes fill the band the reference tool left, on the surface") {
        const double r = 2.0, refR = 6.0, stepover = 0.4;
        Harness h(tooling::makeBallEndMill(1, 2.0 * r, 20.0, 50.0));
        PencilParams p;
        p.referenceRadius = refR;
        p.offsetPasses = 20;
        p.stepover = stepover;
        const auto tp = pencilFinish(part, p, h.context());
        dumpToolpath("pencil_channel_offsets", tp);
        CHECK(tp.warnings.empty());
        const DropCutter dc(part, Cutter(h.tool));
        std::set<long> passes;
        for (const Vec3& q : cutPoints(tp)) {
            if (q.y() < 6.0 || q.y() > 54.0 || q.x() > 0.5 * (a + b)) {
                continue;
            }
            // On the surface (the passes are straight along Y), within the band: no further
            // out than where the reference ball touched the floor (x = a + refR) and a
            // stepover, and none up the wall (the centre no nearer the wall than the radius).
            CHECK_THAT(q.z(), WithinAbs(dc.drop(q.head<2>(), -10.0), 1e-6));
            CHECK(q.x() <= a + refR + stepover + 1e-6);
            CHECK(q.x() >= a + r - 1e-6);
            passes.insert(std::lround(q.x() * 100.0));
        }
        // The pencil pass and those beside it: up the fillet towards the wall one or two, out
        // along the floor to the reference tool's contact about ten.
        CHECK(passes.size() >= 8);
        CHECK(passes.size() <= 16);
    }

    SECTION("no corner tighter than the reference tool: no passes") {
        const auto bump =
            heightMesh([](double x, double y) { return 5.0 * std::exp(-(x * x + y * y) / 50.0); },
                       -20, 20, -20, 20, 80, -5);
        Harness h(tooling::makeBallEndMill(1, 4.0, 20.0, 50.0));
        PencilParams p;
        p.referenceRadius = 3.0;
        const auto tp = pencilFinish(bump, p, h.context());
        CHECK(tp.moves.empty());
        CHECK(tp.warnings.size() == 1);
    }

    SECTION("contained: only the corners inside the containment") {
        Harness h(tooling::makeBallEndMill(1, 4.0, 20.0, 50.0));
        PencilParams p;
        p.referenceRadius = 6.0;
        p.boundary = {rect(0.0, 20.0, 20.0, 40.0)}; // the left fillet's middle third
        const auto tp = pencilFinish(part, p, h.context());
        std::size_t checked = 0;
        for (const Vec3& q : cutPoints(tp)) {
            CHECK(q.x() < 20.0);
            CHECK(q.y() >= 20.0 - 1e-6);
            CHECK(q.y() <= 40.0 + 1e-6);
            ++checked;
        }
        CHECK(checked > 40);
    }

    SECTION("the reference tool cannot be smaller than the tool") {
        Harness h(tooling::makeBallEndMill(1, 6.0, 20.0, 50.0));
        PencilParams p;
        p.referenceRadius = 2.0;
        CHECK_THROWS_AS(pencilFinish(part, p, h.context()), Error);
    }
}

TEST_CASE("steep and shallow finishing covers the dome within the scallop height",
          "[strategies][surface][steep]") {
    // A spherical cap 20 high on a plate, 70.5 degrees steep where it meets the plate.
    constexpr double R = 30.0, c = 10.0;
    const auto part = heightMesh(dome(R, c, 0.0), -40, 40, -40, 40, 160, -5);
    const double r = 3.0;
    const double cusp = 0.02;
    Harness h(tooling::makeBallEndMill(1, 2.0 * r, 30.0, 60.0));
    SteepShallowParams p;
    p.thresholdAngle = degToRad(40.0);
    p.overlap = 1.0;
    p.steepScallop = cusp;
    p.shallowScallop = cusp;
    p.topZ = 20.0;
    p.bottomZ = 0.0;
    p.resolution = 0.25;
    const auto tp = steepShallowFinish(part, p, h.context());
    dumpToolpath("steep_shallow_dome", tp);
    CHECK(tp.warnings.empty());

    // Both kinds of passes: runs at one height (steep) and runs that follow the surface.
    std::size_t level = 0, sloped = 0;
    for (const auto& run : toolpath::sampleRuns(tp, 0.001)) {
        if (run.feedClass != FeedClass::Cut || run.points.size() < 3) {
            continue;
        }
        const auto [lo, hi] =
            std::minmax_element(run.points.begin(), run.points.end(),
                                [](const Vec3& u, const Vec3& v) { return u.z() < v.z(); });
        (hi->z() - lo->z() < 1e-6 ? level : sloped)++;
    }
    CHECK(level > 10);
    CHECK(sloped > 10);
    // Every entry is a plunge at feed from the retract height (above the part's top and the
    // feed clearance): no rapid comes down into what the roughing left above a level, and none
    // goes across lower, even between the steep and the shallow passes (this harness's
    // clearance height is below the part's top). Rapids straight up end where they may.
    const double retract = std::max(h.heights.retract, 20.0 + h.heights.feedClearance);
    Vec3 at = tp.start;
    for (const auto& m : tp.moves) {
        const bool up = (m.end.head<2>() - at.head<2>()).norm() < 1e-9 && m.end.z() >= at.z();
        if (m.kind == MoveKind::Rapid && !up) {
            CHECK(m.end.z() >= retract - 1e-6);
        }
        at = m.end;
    }

    // The material left at every vertex of the surface the ball can reach: at most the cusp a
    // parallel pass leaves on its slope (h / cos^2 of the slope) where it is shallow, the
    // constant-Z passes' cusp where it is steep, and nowhere cut into.
    const double rBase = std::sqrt(R * R - c * c);
    const auto slopeAt = [&](const Vec3& q) {
        const double rho = q.head<2>().norm();
        return rho < rBase ? std::asin(std::min(1.0, rho / R)) : 0.0;
    };
    const auto reachable = [&](const Vec3& q) {
        // Not the plate's sides, nor the corner around the cap's foot.
        return std::abs(q.x()) <= 39.0 && std::abs(q.y()) <= 39.0 && q.z() > -1e-9 &&
               std::abs(q.head<2>().norm() - rBase) >= r;
    };
    const BallGap gap(densePoints(tp, 0.05), r);
    double worstShallow = 0.0, worstSteep = 0.0;
    std::size_t checked = 0;
    for (const Vec3& q : part.vertices) {
        if (!reachable(q)) {
            continue;
        }
        const double alpha = slopeAt(q);
        const double g = gap.at(q);
        INFO("at " << q.transpose() << ", slope " << alpha * 180.0 / kPi);
        CHECK(g > -0.003);
        if (alpha < p.thresholdAngle) {
            CHECK(g <= 1.25 * cusp / std::pow(std::cos(alpha), 2) + 0.003);
            worstShallow = std::max(worstShallow, g);
        } else {
            CHECK(g <= 1.25 * cusp + 0.003);
            worstSteep = std::max(worstSteep, g);
        }
        ++checked;
    }
    INFO("worst left: shallow " << worstShallow << ", steep " << worstSteep);
    CHECK(checked > 10000);
    // Parallel passes alone at the same scallop height leave far more on the steep sides.
    ParallelParams pp;
    pp.scallopHeight = cusp;
    const BallGap parallelGap(densePoints(parallelFinish(part, 0.0, pp, h.context()), 0.05), r);
    double parallelSteep = 0.0;
    for (const Vec3& q : part.vertices) {
        if (reachable(q) && slopeAt(q) > p.thresholdAngle) {
            parallelSteep = std::max(parallelSteep, parallelGap.at(q));
        }
    }
    INFO("parallel passes alone leave " << parallelSteep << " on the steep sides");
    CHECK(parallelSteep > 1.25 * cusp + 0.003);
    CHECK(parallelSteep > 2.0 * worstSteep);
    // The shallow areas one at a time: few entries from above.
    std::size_t entries = 0;
    for (const auto& m : tp.moves) {
        entries += m.feedClass == FeedClass::Plunge ? 1 : 0;
    }
    CHECK(entries < 20);
}

TEST_CASE("steep and shallow areas split at the threshold", "[strategies][surface][steep]") {
    const auto part = heightMesh(dome(30.0, 10.0, 0.0), -40, 40, -40, 40, 160, -5);
    const DropCutter dc(part, Cutter::torus(3, 3));
    geom2d::Aabb2 area;
    area.extend(Vec2(-36, -36));
    area.extend(Vec2(36, 36));
    CancelToken cancel;
    const auto shallow = shallowAreas(dc, area, 0.25, -20.0, degToRad(40.0), 0.002, cancel);
    // The ball's centre rides on a sphere of R + r about the cap's centre, so the slope under
    // it is 40 degrees at a radius of (R + r) sin 40. Inside that the top is shallow; then the
    // steep ring down the cap and into its foot; outside, the plate again.
    const double ring = 33.0 * std::sin(degToRad(40.0));
    const auto inShallow = [&](const Vec2& q) {
        return std::any_of(shallow.begin(), shallow.end(),
                           [&](const geom2d::Region& reg) { return reg.contains(q); });
    };
    CHECK(shallow.size() == 2);
    CHECK(inShallow(Vec2(0, 0)));
    CHECK(inShallow(Vec2(ring - 0.5, 0)));
    CHECK_FALSE(inShallow(Vec2(ring + 0.5, 0)));
    CHECK_FALSE(inShallow(Vec2(0, 27.0)));
    CHECK(inShallow(Vec2(34.0, 0)));
}
