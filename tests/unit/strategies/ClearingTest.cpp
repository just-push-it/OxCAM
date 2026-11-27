// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Engagement-limited clearing (libs/strategies/src/Clearing.hpp): the material grid's
// engagement angle, and Z-level roughing keeping it within the limit everywhere.

#include "Clearing.hpp"

#include "MedialAxis.hpp"
#include "StrategyTestUtil.hpp"

#include <ocam/geom3d/Mesh.hpp>
#include <ocam/strategies/Surface.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <functional>
#include <memory>
#include <numbers>
#include <optional>
#include <set>

using namespace ocam;
using namespace ocam::strategies;
using namespace ocam::strategies::detail;
using namespace ocam::strategies::testing;
using Catch::Approx;
using toolpath::FeedClass;
using toolpath::MoveKind;

namespace {

geom2d::Regions box(double x0, double y0, double x1, double y1) {
    return {{geom2d::Polyline::rectangle({x0, y0}, {x1, y1}), {}}};
}

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
    std::vector<std::uint32_t> top, bottom;
    for (int i = 0; i < n; ++i) {
        top.push_back(idx(i, 0));
    }
    for (int j = 0; j < n; ++j) {
        top.push_back(idx(n, j));
    }
    for (int i = n; i > 0; --i) {
        top.push_back(idx(i, n));
    }
    for (int j = n; j > 0; --j) {
        top.push_back(idx(0, j));
    }
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

struct Replay {
    double largest = 0.0;   ///< Largest engagement angle of the cutting moves [rad].
    Vec2 at = Vec2::Zero(); ///< Where.
    std::size_t move = 0;   ///< The move it is on.
    double leftover = 0.0;  ///< Material still on the grid afterwards [mm²].
    double idle = 0.0;      ///< Cutting moves at the level on which the tool meets nothing [mm].
};

/// Replays the cutting moves at height z on a grid of `material` (0.1 mm cells) and reports
/// the largest engagement angle away from entries (plunges, ramps and the lead that follows
/// them), plus what is left of the material within `reach` of the moves.
Replay replay(const toolpath::Toolpath& tp, double z, double thickness,
              const geom2d::Regions& material, const geom2d::Aabb2& extent, double radius) {
    const double h = 0.1;
    geom2d::Aabb2 grown = extent;
    grown.min -= Vec2::Constant(3.0 * radius + 5.0);
    grown.max += Vec2::Constant(3.0 * radius + 5.0);
    CellGrid grid(grown, h);
    grid.fill(material, true, 0.001);
    Replay out;
    Vec3 from = tp.start;
    double sinceEntry = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < tp.moves.size(); ++index) {
        const auto& m = tp.moves[index];
        if (m.feedClass == FeedClass::Plunge || m.feedClass == FeedClass::Ramp ||
            m.feedClass == FeedClass::LeadIn) {
            sinceEntry = 0.0;
        }
        // Moves within the level's slab clear its material (helixes and ramps descend through
        // it); the engagement is measured on those that run at the level.
        const bool atZ = std::abs(from.z() - z) < 1e-6 && std::abs(m.end.z() - z) < 1e-6;
        const bool inSlab = m.end.z() > z - 1e-6 && m.end.z() <= z + thickness + 1e-6;
        if (inSlab && m.kind != MoveKind::Rapid) {
            std::vector<Vec2> pts;
            if (m.isArc()) {
                const Vec2 c = m.center.head<2>();
                const double rr = (from.head<2>() - c).norm();
                const double a0 = std::atan2(from.y() - c.y(), from.x() - c.x());
                double a1 = std::atan2(m.end.y() - c.y(), m.end.x() - c.x());
                if (m.kind == MoveKind::ArcCCW) {
                    while (a1 <= a0 + 1e-12) {
                        a1 += 2.0 * std::numbers::pi;
                    }
                } else {
                    while (a1 >= a0 - 1e-12) {
                        a1 -= 2.0 * std::numbers::pi;
                    }
                }
                const int n = std::max(2, static_cast<int>(std::abs(a1 - a0) * rr / 0.05));
                for (int s = 1; s <= n; ++s) {
                    const double a = a0 + (a1 - a0) * s / n;
                    pts.push_back(c + rr * Vec2(std::cos(a), std::sin(a)));
                }
            } else {
                const double len = (m.end - from).head<2>().norm();
                const int n = std::max(1, static_cast<int>(len / 0.05));
                for (int s = 1; s <= n; ++s) {
                    pts.push_back(from.head<2>() + (m.end - from).head<2>() * (double(s) / n));
                }
            }
            Vec2 prev = from.head<2>();
            for (const Vec2& q : pts) {
                const double step = (q - prev).norm();
                sinceEntry += step;
                if (atZ && step > 1e-9 && m.feedClass == FeedClass::Cut &&
                    sinceEntry > 2.0 * radius) {
                    const double angle = grid.contactAngle(q, radius, (q - prev).normalized());
                    if (angle > out.largest) {
                        out.largest = angle;
                        out.at = q;
                        out.move = index;
                    }
                }
                if (atZ && m.feedClass == FeedClass::Cut && !grid.anyWithin(q, radius)) {
                    out.idle += step;
                }
                grid.clearDisk(q, radius);
                prev = q;
            }
        }
        from = m.end;
    }
    // What is left: cells still set within the extent.
    for (double x = extent.min.x(); x <= extent.max.x(); x += h) {
        for (double y = extent.min.y(); y <= extent.max.y(); y += h) {
            if (grid.at(Vec2(x, y))) {
                out.leftover += h * h;
            }
        }
    }
    return out;
}

} // namespace

TEST_CASE("engagement angle of a straight cut", "[strategies][clearing]") {
    CHECK(engagementAngle(0.0, 3.0) == Approx(0.0));
    CHECK(engagementAngle(3.0, 3.0) == Approx(std::numbers::pi / 2));
    CHECK(engagementAngle(6.0, 3.0) == Approx(std::numbers::pi));
    // Measured on the grid: a tool moving along a material edge, cutting 1.2 mm of it.
    CellGrid material({Vec2(-10, -10), Vec2(40, 10)}, 0.05);
    material.fill(box(-5, 1.8, 40, 10), true, 0.001);
    for (double x = -6.0; x <= 10.0; x += 0.05) {
        material.clearDisk(Vec2(x, 0.0), 3.0);
    }
    CHECK(material.contactAngle(Vec2(10.05, 0.0), 3.0, Vec2(1, 0)) ==
          Approx(engagementAngle(1.2, 3.0)).margin(0.06));
    CHECK(material.touches(Vec2(10.0, 0.0), 3.0));
    CHECK_FALSE(material.touches(Vec2(10.0, -2.0), 3.0));
}

TEST_CASE("links between passes stay as low as is safe", "[strategies][clearing][links]") {
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    const Context ctx = h.context();
    const double r = 3.0;
    const double cell = clearingCell(r);
    // One level at z = 0 of an area 60 x 40; the tool at depth at `from`, going to `to`.
    struct Level {
        LevelClearing in;
        geom2d::Aabb2 grid{Vec2(-34, -24), Vec2(34, 24)};
        CellGrid material{grid, clearingCell(3.0)};
    };
    const auto level = [&](const geom2d::Regions& forbidden, const geom2d::Regions& material) {
        auto l = std::make_unique<Level>();
        const geom2d::Regions area = geom2d::subtract(box(-30, -20, 30, 20), forbidden, {0.001});
        REQUIRE(area.size() == 1);
        l->in.area = area.front();
        l->in.forbidden = forbidden;
        l->in.toolRadius = r;
        l->in.z = 0.0;
        l->in.approachZ = 10.0;
        l->in.retract = 20.0;
        l->material.fill(material, true, 0.001);
        return l;
    };
    const auto move = [&](const Level& l, double maxLink, const Vec2& from, const Vec2& to) {
        toolpath::Toolpath tp;
        tp.start = Vec3(from.x(), from.y(), 0.0);
        toolpath::PathBuilder b(tp);
        const LevelLinks links(l.in, l.material, l.grid, cell, -0.5 * cell, maxLink, ctx);
        const std::vector<Vec2> way = links.moveTo(b, to);
        CHECK((b.position() - Vec3(to.x(), to.y(), 0.0)).norm() < 1e-9);
        return std::make_pair(tp, way);
    };
    const auto highest = [](const toolpath::Toolpath& tp) {
        double z = tp.start.z();
        for (const toolpath::Move& m : tp.moves) {
            z = std::max(z, m.end.z());
        }
        return z;
    };
    const auto onlyLinks = [&](const toolpath::Toolpath& tp) {
        return !tp.moves.empty() &&
               std::all_of(tp.moves.begin(), tp.moves.end(), [&](const toolpath::Move& m) {
                   return m.feedClass == FeedClass::Link && m.feedRate == h.feeds.cut &&
                          std::abs(m.end.z()) < 1e-12;
               });
    };
    // The tool cuts nothing along a way: no material within its radius less a cell.
    const auto cutsNothing = [&](const Level& l, const std::vector<Vec2>& way) {
        for (std::size_t k = 1; k < way.size(); ++k) {
            for (int s = 0; s <= 200; ++s) {
                const Vec2 q = way[k - 1] + (way[k] - way[k - 1]) * (s / 200.0);
                if (l.material.anyWithin(q, r - cell) || !l.in.area.contains(q)) {
                    return false;
                }
            }
        }
        return true;
    };
    // Ends with a feed down from the feed clearance at the plunge feed.
    const auto plungesLastMillimetre = [&](const toolpath::Toolpath& tp) {
        REQUIRE(tp.moves.size() >= 2);
        const toolpath::Move& last = tp.moves.back();
        const toolpath::Move& before = tp.moves[tp.moves.size() - 2];
        return last.feedClass == FeedClass::Plunge && last.feedRate == h.feeds.plunge &&
               before.kind == MoveKind::Rapid && std::abs(before.end.z() - 1.0) < 1e-9;
    };

    SECTION("away from the material the tool stands next to: at depth") {
        // A pass has just run along the edge of what is left (x >= 10): the tool touches it.
        const auto l = level({}, box(10, -20, 30, 20));
        const auto [tp, way] = move(*l, 12.0, Vec2(7.0, 0.0), Vec2(7.0, -10.0));
        CHECK(l->material.touches(Vec2(7.0, 0.0), r));
        CHECK(onlyLinks(tp));
        CHECK(tp.moves.size() == 1);
        CHECK(cutsNothing(*l, way));
    }
    SECTION("around material standing in the way: at depth, when not too long") {
        const auto l = level({}, box(-1, 2, 1, 20));
        const auto [tp, way] = move(*l, 25.0, Vec2(-5.0, 6.0), Vec2(5.0, 6.0));
        CHECK(onlyLinks(tp));
        CHECK(way.size() > 2);
        CHECK(cutsNothing(*l, way));
        double length = 0.0;
        for (std::size_t k = 1; k < way.size(); ++k) {
            length += (way[k] - way[k - 1]).norm();
        }
        CHECK(length <= 25.0);
        CHECK(std::any_of(way.begin(), way.end(), [](const Vec2& q) { return q.y() < 0.0; }));
    }
    SECTION("material in the way, the way around too long: just over the material") {
        const auto l = level({}, box(-1, 2, 1, 20));
        const auto [tp, way] = move(*l, 8.0, Vec2(-5.0, 6.0), Vec2(5.0, 6.0));
        CHECK(way.empty());
        CHECK(highest(tp) == Approx(10.0));
        CHECK(plungesLastMillimetre(tp));
    }
    SECTION("to an entry just off the material: fed down from the approach height") {
        // The level's material stands from the top of its column (here the approach height
        // less the feed clearance) down to the level: a rapid straight down next to it grazed
        // its edge (the dome's fine-level stair, a rapid collision in the simulation).
        const auto l = level({}, box(10, -20, 30, 20));
        const Vec2 entry(10.0 - r - 0.5, 0.0); // clear of the grid, within the feed clearance
        CHECK_FALSE(l->material.touches(entry, r));
        const auto [tp, way] = move(*l, 12.0, Vec2(-25.0, 15.0), entry);
        CHECK(way.empty());
        const toolpath::Move& last = tp.moves.back();
        CHECK(last.feedClass == FeedClass::Plunge);
        CHECK(last.feedRate == h.feeds.plunge);
        CHECK(tp.moves.size() >= 2);
        CHECK(tp.moves[tp.moves.size() - 2].end.z() == Approx(10.0));
        CHECK(std::none_of(tp.moves.begin(), tp.moves.end(), [](const toolpath::Move& m) {
            return m.kind == MoveKind::Rapid && m.end.z() < 10.0 - 1e-9;
        }));
        // A feed clearance further off, the column is clear: rapid down to just off the floor.
        const auto [clear, clearWay] = move(*l, 12.0, Vec2(-25.0, 15.0), Vec2(10.0 - r - 1.5, 0.0));
        CHECK(clearWay.empty());
        CHECK(plungesLastMillimetre(clear));
    }
    SECTION("nothing in the way but too far to feed: just off the floor") {
        const auto l = level({}, box(-1, -20, 1, 10));
        const auto [tp, way] = move(*l, 12.0, Vec2(-25.0, 15.0), Vec2(25.0, 15.0));
        CHECK(way.empty());
        CHECK(highest(tp) == Approx(1.0));
        CHECK(plungesLastMillimetre(tp));
        CHECK(std::none_of(tp.moves.begin(), tp.moves.end(),
                           [](const toolpath::Move& m) { return m.feedClass == FeedClass::Link; }));
    }
    SECTION("the part in the way: over the retract height") {
        const auto l = level(box(-5, -5, 5, 5), {});
        const auto [tp, way] = move(*l, 12.0, Vec2(-10.0, 0.0), Vec2(10.0, 0.0));
        CHECK(way.empty());
        CHECK(highest(tp) == Approx(20.0));
        CHECK(plungesLastMillimetre(tp));
        // The same, with room to go round it at depth.
        const auto [round, roundWay] = move(*l, 30.0, Vec2(-10.0, 0.0), Vec2(10.0, 0.0));
        CHECK(onlyLinks(round));
        CHECK(roundWay.size() > 2);
        CHECK(cutsNothing(*l, roundWay));
    }
    SECTION("round the part where it reaches out of the area, through `travel`") {
        // The part (with the tool's radius) crosses the area's lower edge; below the area is
        // air the tool may pass through.
        const geom2d::Regions forbidden = box(-3, -25, 3, 5);
        auto l = level(forbidden, {});
        const auto [inside, insideWay] = move(*l, 32.0, Vec2(-8.0, -15.0), Vec2(8.0, -15.0));
        CHECK(insideWay.empty()); // round the top it is too long
        CHECK(highest(inside) == Approx(20.0));
        l->in.travel = geom2d::subtract(box(-40, -35, 40, -20), forbidden, {0.001});
        const auto [tp, way] = move(*l, 32.0, Vec2(-8.0, -15.0), Vec2(8.0, -15.0));
        CHECK(onlyLinks(tp));
        CHECK(std::any_of(way.begin(), way.end(), [](const Vec2& q) { return q.y() < -25.0; }));
    }
}

TEST_CASE("Z-level roughing of a large open area stays down between passes",
          "[strategies][clearing][links]") {
    // A round boss in a wide stock: the outside is one open area, cleared from the air in.
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    ZLevelRoughParams p;
    p.stepDown = 10.0;
    p.stockToLeaveXY = 0.5;
    p.stockToLeaveZ = 0.0;
    p.flatAreaLevels = false;
    const auto boss =
        heightMesh([](double x, double y) { return x * x + y * y <= 100.0 ? 10.0 : 0.0; }, -40, 40,
                   -40, 40, 320, -1);
    const geom2d::Regions stock = box(-35, -35, 35, 35);
    const auto tp = zLevelRough(boss, stock, 10.0, 0.0, p, h.context());
    INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
    REQUIRE(tp.warnings.empty());
    const double ae = p.stepoverRatio * 6.0;
    // Lifts at the level: from a move at depth, rapids up and back down to it.
    int lifts = 0;
    int shortHigh = 0; // above the feed clearance, to a point within a link's length
    int links = 0;
    Vec3 from = tp.start;
    std::optional<Vec3> liftedAt;
    double top = 0.0;
    for (const toolpath::Move& m : tp.moves) {
        if (m.kind == MoveKind::Rapid) {
            if (!liftedAt && std::abs(from.z()) < 1e-9) {
                liftedAt = from;
                top = from.z();
            }
            top = std::max(top, m.end.z());
        } else {
            if (liftedAt && std::abs(m.end.z()) < 1e-9) {
                ++lifts;
                if (top > h.heights.feedClearance + 1e-9 &&
                    (m.end - *liftedAt).head<2>().norm() <= 4.0 * ae) {
                    ++shortHigh;
                }
            }
            liftedAt.reset();
            if (m.feedClass == FeedClass::Link) {
                ++links;
                CHECK(m.feedRate == h.feeds.cut);
            }
        }
        from = m.end;
    }
    INFO(lifts << " lifts, " << links << " links at depth");
    CHECK(shortHigh == 0);
    CHECK(links > 0);
    // Every position at the level keeps the allowance off the boss.
    for (const Vec3& q : feedPoints(tp)) {
        if (q.z() < 10.0 - 1e-6) {
            CHECK(q.head<2>().norm() >= 10.0 + 3.0 + 0.5 - 0.01);
        }
    }
}

TEST_CASE("Z-level roughing takes the stock over a face at one fine level, from the air",
          "[strategies][clearing]") {
    // A low round boss (top at 4) in a wide stock, one step-down level at the bottom and fine
    // levels a millimetre apart: the 6 mm over the boss's top is in the step-down level's part
    // zone. The fine level just above the face takes it all, from the top, as an open area
    // entered from the air the step-down level cleared around it (no helix); the levels above
    // it cut nothing there. In rest roughing too, where the in-process stock is the one before
    // the operation and does not know what its step-down level cleared.
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    ZLevelRoughParams p;
    p.stepDown = 10.0;
    p.fineStepDown = 1.0;
    p.stockToLeaveXY = 0.0;
    p.stockToLeaveZ = 0.3;
    p.flatAreaLevels = false;
    const auto boss =
        heightMesh([](double x, double y) { return x * x + y * y <= 225.0 ? 4.0 : 0.0; }, -30, 30,
                   -30, 30, 300, -1);
    const geom2d::Regions stock = box(-28, -28, 28, 28);
    const auto check = [&](const toolpath::Toolpath& tp) {
        INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
        CHECK(tp.warnings.empty());
        std::set<long> overFace; // levels [µm] cutting over the face's middle
        for (const toolpath::Move& m : tp.moves) {
            if (m.kind != MoveKind::Rapid && m.end.head<2>().norm() < 10.0 && m.end.z() < 10.0) {
                overFace.insert(std::lround(m.end.z() * 1000.0));
            }
        }
        REQUIRE(overFace.size() == 1);
        CHECK(*overFace.begin() == 5000); // the fine level just above the face and allowance
        CHECK(std::none_of(tp.moves.begin(), tp.moves.end(),
                           [](const toolpath::Move& m) { return m.feedClass == FeedClass::Ramp; }));
    };
    SECTION("from the stock") {
        check(zLevelRough(boss, stock, 10.0, 0.0, p, h.context()));
    }
    SECTION("rest roughing") {
        ZLevelRoughParams pr = p;
        pr.material = [&](double, double) {
            return stock;
        };
        check(zLevelRough(boss, stock, 10.0, 0.0, pr, h.context()));
    }
}

TEST_CASE("Z-level roughing: no room for a helix, the order of areas", "[strategies][clearing]") {
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    ZLevelRoughParams p;
    p.stepDown = 5.0;
    p.stockToLeaveXY = 0.0;
    p.stockToLeaveZ = 0.0;
    p.flatAreaLevels = false;

    SECTION("a hole too small for a helix is left, not plunged into") {
        const auto plate =
            heightMesh([](double x, double y) { return x * x + y * y <= 16.0 ? -2.0 : 10.0; }, -15,
                       15, -15, 15, 150, -1);
        Harness hs(tooling::makeFlatEndMill(1, 6.35, 25.0, 60.0));
        const geom2d::Regions stock = box(-15, -15, 15, 15);
        const auto tp = zLevelRough(plate, stock, 10.0, 0.0, p, hs.context());
        CHECK(tp.warnings.empty()); // left without a word: the tool was the user's choice
        CHECK(std::none_of(tp.moves.begin(), tp.moves.end(), [](const toolpath::Move& m) {
            return m.feedClass == FeedClass::Plunge || m.feedClass == FeedClass::Cut;
        }));
        // Asked to plunge, it does.
        ZLevelRoughParams pp = p;
        pp.entry = EntryKind::Plunge;
        const auto plunged = zLevelRough(plate, stock, 10.0, 0.0, pp, hs.context());
        CHECK(std::any_of(plunged.moves.begin(), plunged.moves.end(), [](const toolpath::Move& m) {
            return m.feedClass == FeedClass::Plunge;
        }));
    }
    SECTION("two pockets: level by level, or area by area") {
        const auto plate = heightMesh(
            [](double x, double y) {
                return std::abs(y) <= 8.0 && std::abs(std::abs(x) - 15.0) <= 8.0 ? -2.0 : 10.0;
            },
            -30, 30, -15, 15, 300, -1);
        const geom2d::Regions stock = box(-30, -15, 30, 15);
        // The pocket (by the sign of x) and level of each cut, in order.
        const auto sequence = [&](const toolpath::Toolpath& tp) {
            std::vector<std::pair<int, double>> seq;
            for (const toolpath::Move& m : tp.moves) {
                if (m.feedClass == FeedClass::Cut && m.end.z() < 10.0) {
                    const std::pair<int, double> at{m.end.x() < 0.0 ? 0 : 1, m.end.z()};
                    if (seq.empty() || seq.back() != at) {
                        seq.push_back(at);
                    }
                }
            }
            return seq;
        };
        const auto byLevel = sequence(zLevelRough(plate, stock, 10.0, 0.0, p, h.context()));
        REQUIRE(byLevel.size() == 4); // both pockets at 5, then both at 0
        CHECK(byLevel[0].second == 5.0);
        CHECK(byLevel[1].second == 5.0);
        CHECK(byLevel[0].first != byLevel[1].first);
        CHECK(byLevel[2].second == 0.0);
        CHECK(byLevel[3].second == 0.0);
        ZLevelRoughParams pa = p;
        pa.byArea = true;
        const auto byArea = sequence(zLevelRough(plate, stock, 10.0, 0.0, pa, h.context()));
        REQUIRE(byArea.size() == 4); // one pocket at 5 and 0, then the other
        CHECK(byArea[0].first == byArea[1].first);
        CHECK(byArea[0].second == 5.0);
        CHECK(byArea[1].second == 0.0);
        CHECK(byArea[2].first != byArea[0].first);
        CHECK(byArea[2].second == 5.0);
        CHECK(byArea[3].second == 0.0);
    }
}

TEST_CASE("the medial axis of simple areas", "[strategies][clearing][medial]") {
    SECTION("a long rectangle: one branch along the middle at half the width") {
        const MedialGraph g = medialAxis(box(-20, -5, 20, 5), 0.1, 0.3, 0.15, 1e-3);
        REQUIRE_FALSE(g.empty());
        // The longest branch runs along y = 0 with radius 5 (the corner spurs are short).
        const auto longest =
            std::max_element(g.edges.begin(), g.edges.end(),
                             [](const MedialGraph::Edge& a, const MedialGraph::Edge& b) {
                                 return a.length < b.length;
                             });
        CHECK(longest->length > 25.0);
        for (std::size_t i = 0; i < longest->points.size(); ++i) {
            if (std::abs(longest->points[i].x()) < 12.0) {
                CHECK(std::abs(longest->points[i].y()) < 0.11);
                CHECK(longest->radii[i] == Approx(5.0).margin(0.11));
            }
        }
    }
    SECTION("a disc: its centre, at its radius") {
        const MedialGraph g =
            medialAxis({{geom2d::Polyline::circle({3, -2}, 8.0), {}}}, 0.1, 0.3, 0.15, 1e-3);
        double best = 0.0;
        Vec2 at = Vec2::Zero();
        for (const MedialGraph::Node& n : g.nodes) {
            if (n.radius > best) {
                best = n.radius;
                at = n.p;
            }
        }
        for (const MedialGraph::Edge& e : g.edges) {
            for (std::size_t i = 0; i < e.points.size(); ++i) {
                if (e.radii[i] > best) {
                    best = e.radii[i];
                    at = e.points[i];
                }
            }
        }
        CHECK(best == Approx(8.0).margin(0.15));
        CHECK((at - Vec2(3, -2)).norm() < 0.2);
    }
}

TEST_CASE("Z-level roughing keeps the engagement within the limit",
          "[strategies][clearing][medial]") {
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    const double r = 3.0;
    ZLevelRoughParams p;
    p.stepDown = 5.0;
    p.stockToLeaveXY = 0.0;
    p.stockToLeaveZ = 0.0;
    p.flatAreaLevels = false;
    const double ae = p.stepoverRatio * 2.0 * r;
    // The limit the strategy works to, plus the replay's own grid noise (a finer grid, and
    // material drawn by hand: at a corner a sample can read a few degrees more).
    const double limit = engagementAngle(ae + 0.3, r) + 0.15;
    const auto cutArcs = [](const toolpath::Toolpath& tp) {
        return std::count_if(tp.moves.begin(), tp.moves.end(), [](const toolpath::Move& m) {
            return m.feedClass == FeedClass::Cut && m.isArc();
        });
    };

    SECTION("a square pocket, entered by a helix, spiralled out and circled") {
        const auto plate = heightMesh(
            [](double x, double y) {
                return std::abs(x) <= 12.0 && std::abs(y) <= 12.0 ? -2.0 : 10.0;
            },
            -20, 20, -20, 20, 200, -1);
        const geom2d::Regions stock = box(-20, -20, 20, 20);
        const auto tp = zLevelRough(plate, stock, 10.0, 0.0, p, h.context());
        INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
        CHECK(tp.warnings.empty());
        CHECK(std::any_of(tp.moves.begin(), tp.moves.end(), [](const toolpath::Move& m) {
            return m.feedClass == FeedClass::Ramp && m.isArc();
        }));
        CHECK(cutArcs(tp) > 8);
        const geom2d::Regions material = box(-11.65, -11.65, 11.65, 11.65);
        const Replay rp = replay(tp, 5.0, 5.0, material, {Vec2(-12, -12), Vec2(12, 12)}, r);
        INFO("largest at " << rp.at.transpose() << " on move " << rp.move);
        CHECK(rp.largest <= limit);
        CHECK(rp.leftover < 16.0);
    }
    SECTION("a narrow closed slot at 0.8 D: circles along it, joined by tangents") {
        const auto plate = heightMesh(
            [](double x, double y) {
                return std::abs(x) <= 20.0 && std::abs(y) <= 6.35 ? -2.0 : 10.0;
            },
            -26, 26, -14, 14, 260, -1);
        Harness hs(tooling::makeFlatEndMill(1, 6.35, 25.0, 60.0));
        ZLevelRoughParams ps = p;
        ps.stepoverRatio = 0.8;
        const geom2d::Regions stock = box(-26, -14, 26, 14);
        const auto tp = zLevelRough(plate, stock, 10.0, 0.0, ps, hs.context());
        INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
        CHECK(tp.warnings.empty());
        CHECK(cutArcs(tp) > 20);
        const geom2d::Regions material = box(-19.65, -6.0, 19.65, 6.0);
        const Replay rp = replay(tp, 5.0, 5.0, material, {Vec2(-20, -6.35), Vec2(20, 6.35)}, 3.175);
        INFO("largest at " << rp.at.transpose() << " on move " << rp.move << ", left "
                           << rp.leftover << " mm2, idle " << rp.idle << " mm");
        // The backs of the circles, in the slot already cut, are crossed straight (189 mm of
        // cutting moves in air before, 95 after).
        CHECK(rp.idle < 130.0);
        CHECK(rp.largest <= engagementAngle(0.8 * 6.35 + 0.3, 3.175) + 0.15);
        CHECK(rp.leftover < 14.0);
        // The circles (the wall pass's corner arcs are small) are centred on the slot's axis.
        for (const toolpath::Move& m : tp.moves) {
            if (m.feedClass == FeedClass::Cut && m.isArc() && std::abs(m.end.z() - 5.0) < 1e-6 &&
                (m.end - m.center).head<2>().norm() > 2.0) {
                CHECK(std::abs(m.center.y()) < 0.8);
            }
        }
    }
    SECTION("rest roughing: a level under one this operation cut crosses the circles' backs") {
        // The in-process stock is the whole block at every height: what stands over the lower
        // level is what the upper level of this operation did not cut, not the stock before it.
        // (Taken from the stock alone, the slot was all under material, no link at depth was
        // clear, and the lower level's circles went round through the air.)
        const auto plate = heightMesh(
            [](double x, double y) {
                return std::abs(x) <= 20.0 && std::abs(y) <= 6.35 ? -2.0 : 10.0;
            },
            -26, 26, -14, 14, 260, -1);
        Harness hs(tooling::makeFlatEndMill(1, 6.35, 25.0, 60.0));
        ZLevelRoughParams ps = p;
        ps.stepoverRatio = 0.8;
        const geom2d::Regions stock = box(-26, -14, 26, 14);
        // The lower level (z = 0) replayed, as rest roughing and as plain roughing (no in-process
        // stock, so nothing overhead): the same circles, their backs crossed alike.
        const geom2d::Regions material = box(-19.65, -6.0, 19.65, 6.0);
        const auto lower = [&](const toolpath::Toolpath& tp) {
            return replay(tp, 0.0, 5.0, material, {Vec2(-20, -6.35), Vec2(20, 6.35)}, 3.175);
        };
        const Replay plain = lower(zLevelRough(plate, stock, 10.0, 0.0, ps, hs.context()));
        ps.material = [&](double, double) {
            return stock;
        };
        const auto tp = zLevelRough(plate, stock, 10.0, 0.0, ps, hs.context());
        INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
        const Replay rp = lower(tp);
        INFO("largest at " << rp.at.transpose() << " on move " << rp.move << ", left "
                           << rp.leftover << " mm2, idle " << rp.idle << " mm (plain " << plain.idle
                           << " mm)");
        // 319 mm in air with the stock alone overhead, 231 mm after.
        CHECK(rp.idle < plain.idle + 10.0);
        CHECK(rp.largest <= engagementAngle(0.8 * 6.35 + 0.3, 3.175) + 0.15);
    }
    SECTION("the lines across the circles' backs rise by the link lift") {
        const auto plate = heightMesh(
            [](double x, double y) {
                return std::abs(x) <= 20.0 && std::abs(y) <= 6.35 ? -2.0 : 10.0;
            },
            -26, 26, -14, 14, 260, -1);
        Harness hs(tooling::makeFlatEndMill(1, 6.35, 25.0, 60.0));
        ZLevelRoughParams ps = p;
        ps.stepoverRatio = 0.8;
        const geom2d::Regions stock = box(-26, -14, 26, 14);
        const auto atDepthLinks = [](const toolpath::Toolpath& tp, double z) {
            return std::count_if(tp.moves.begin(), tp.moves.end(), [&](const toolpath::Move& m) {
                return m.feedClass == FeedClass::Link && std::abs(m.end.z() - z) < 1e-6;
            });
        };
        const auto flat = zLevelRough(plate, stock, 10.0, 0.0, ps, hs.context());
        CHECK(atDepthLinks(flat, 5.1) == 0);
        ps.linkLift = 0.1;
        const auto tp = zLevelRough(plate, stock, 10.0, 0.0, ps, hs.context());
        INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
        CHECK(tp.warnings.empty());
        // Each crossing at z = 5 goes up to 5.1, and every move up there comes back down to 5:
        // nothing else changes height.
        CHECK(atDepthLinks(tp, 5.1) > 5);
        for (std::size_t i = 0; i + 1 < tp.moves.size(); ++i) {
            const toolpath::Move& m = tp.moves[i];
            if (std::abs(m.end.z() - 5.1) < 1e-6 && m.feedClass == FeedClass::Link) {
                const toolpath::Move& next = tp.moves[i + 1];
                CHECK(next.feedClass == FeedClass::Link);
                CHECK((std::abs(next.end.z() - 5.0) < 1e-6 || std::abs(next.end.z() - 5.1) < 1e-6));
            }
        }
        // The cutting is the same: only the crossings are lifted.
        const geom2d::Regions material = box(-19.65, -6.0, 19.65, 6.0);
        const Replay rp = replay(tp, 5.0, 5.0, material, {Vec2(-20, -6.35), Vec2(20, 6.35)}, 3.175);
        INFO("largest at " << rp.at.transpose() << " on move " << rp.move << ", left "
                           << rp.leftover << " mm2");
        CHECK(rp.largest <= engagementAngle(0.8 * 6.35 + 0.3, 3.175) + 0.15);
        CHECK(rp.leftover < 14.0);
    }
    SECTION("an L-shaped boss in square stock is traced (open area), or circled when asked") {
        const auto boss = heightMesh(
            [](double x, double y) {
                const bool inL = (x >= -10 && x <= 10 && y >= -10 && y <= 0) ||
                                 (x >= -10 && x <= 0 && y >= -10 && y <= 10);
                return inL ? 10.0 : 0.0;
            },
            -20, 20, -20, 20, 200, -1);
        const geom2d::Regions stock = box(-18, -18, 18, 18);
        geom2d::Regions material =
            geom2d::subtract(stock,
                             {{geom2d::Polyline::fromPoints({{-10.35, -10.35},
                                                             {10.35, -10.35},
                                                             {10.35, 0.35},
                                                             {0.35, 0.35},
                                                             {0.35, 10.35},
                                                             {-10.35, 10.35}},
                                                            true),
                               {}}},
                             {0.001});
        {
            const auto tp = zLevelRough(boss, stock, 10.0, 0.0, p, h.context());
            INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
            CHECK(tp.warnings.empty());
            CHECK(std::none_of(tp.moves.begin(), tp.moves.end(), [](const toolpath::Move& m) {
                return m.feedClass == FeedClass::Ramp;
            }));
            const Replay rp = replay(tp, 5.0, 5.0, material, {Vec2(-18, -18), Vec2(18, 18)}, r);
            INFO("largest at " << rp.at.transpose() << " on move " << rp.move);
            CHECK(rp.largest <= limit);
            CHECK(rp.leftover < 6.0);
        }
        {
            ZLevelRoughParams po = p;
            po.medialOpenAreas = true;
            const auto tp = zLevelRough(boss, stock, 10.0, 0.0, po, h.context());
            INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
            CHECK(tp.warnings.empty());
            CHECK(std::any_of(tp.moves.begin(), tp.moves.end(), [](const toolpath::Move& m) {
                return m.feedClass == FeedClass::Ramp && m.isArc();
            }));
            const Replay rp = replay(tp, 5.0, 5.0, material, {Vec2(-18, -18), Vec2(18, 18)}, r);
            INFO("largest at " << rp.at.transpose() << " on move " << rp.move << ", left "
                               << rp.leftover << " mm2, idle " << rp.idle << " mm");
            // Out of material the tool crosses straight to where it meets material again,
            // instead of going round the circles through the air outside the stock (1704 mm
            // of cutting moves in air before, 394 after).
            CHECK(rp.idle < 600.0);
            CHECK(rp.largest <= limit);
            CHECK(rp.leftover < 25.0);
        }
    }
    SECTION("a round boss next to the stock's side: the passes along the side flow round it") {
        // The boss comes within half a millimetre of the stock's top side, so the passes along
        // that side are cut off where its outline bulges into their way and start again past
        // it. The tool flows on round the boss, in the material, rather than leaving it for an
        // entry beside the pass and leading back in where it just was (the maintainer's flange,
        // its lobes next to the stock's sides).
        const auto boss = heightMesh(
            [](double x, double y) {
                return x * x + (y - 12.0) * (y - 12.0) <= 56.25 ? 10.0 : 0.0;
            },
            -36, 36, -26, 26, 360, -1);
        const geom2d::Regions stock = box(-30, -20, 30, 20);
        const geom2d::Regions material =
            geom2d::subtract(stock, {{geom2d::Polyline::circle({0.0, 12.0}, 7.85), {}}}, {0.001});
        const auto tp = zLevelRough(boss, stock, 10.0, 0.0, p, h.context());
        INFO("warnings: " << (tp.warnings.empty() ? "" : tp.warnings.front()));
        CHECK(tp.warnings.empty());
        // Entries (lead-ins) at the level near the boss.
        int leads = 0;
        FeedClass previous = FeedClass::Rapid;
        for (const toolpath::Move& m : tp.moves) {
            if (m.feedClass == FeedClass::LeadIn && previous != FeedClass::LeadIn &&
                std::abs(m.end.z() - 5.0) < 1e-6 &&
                (m.end.head<2>() - Vec2(0.0, 12.0)).norm() < 15.0) {
                ++leads;
            }
            previous = m.feedClass;
        }
        const Replay rp = replay(tp, 5.0, 5.0, material, {Vec2(-30, -20), Vec2(30, 20)}, r);
        INFO(leads << " entries near the boss; largest at " << rp.at.transpose() << " on move "
                   << rp.move << ", left " << rp.leftover << " mm2");
        // 23 entries before, 13 after (the rest where the fronts end at the boss's flanks).
        CHECK(leads < 18);
        CHECK(rp.largest <= limit);
        CHECK(rp.leftover < 25.0);
    }
}
