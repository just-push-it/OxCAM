// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "StrategyTestUtil.hpp"

#include <ocam/strategies/MultiAxis.hpp>
#include <ocam/toolpath/PathBuilder.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <functional>

using namespace ocam;
using namespace ocam::strategies;
using namespace ocam::strategies::testing;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using toolpath::FeedClass;
using toolpath::MoveKind;

namespace {

/// Closed mesh of a shaft about X (x in [x0, x1], radius R) with a rectangular slot of depth
/// `depth` for x in [sx0, sx1] and angles [t0, t1] (radial walls).
geom3d::Mesh slottedShaft(double x0, double x1, double R, double sx0, double sx1, double t0,
                          double t1, double depth, int nt = 180) {
    // Nodes: (coordinate, inside-slot flag); walls are duplicated nodes.
    std::vector<std::pair<double, bool>> xs{{x0, false}, {sx0, false}, {sx0, true},
                                            {sx1, true}, {sx1, false}, {x1, false}};
    std::vector<std::pair<double, bool>> ts;
    for (int k = 0; k < nt; ++k) {
        const double t = 2.0 * kPi * k / nt;
        if (t > t0 && (ts.empty() || ts.back().first <= t0)) {
            ts.emplace_back(t0, false);
            ts.emplace_back(t0, true);
        }
        if (t > t1 && ts.back().first <= t1 && ts.back().second) {
            ts.emplace_back(t1, true);
            ts.emplace_back(t1, false);
        }
        ts.emplace_back(t, t > t0 && t < t1);
    }
    geom3d::Mesh m;
    const auto nx = xs.size(), nn = ts.size();
    for (const auto& [x, xin] : xs) {
        for (const auto& [t, tin] : ts) {
            const double r = (xin && tin) ? R - depth : R;
            m.vertices.emplace_back(x, r * std::sin(t), r * std::cos(t));
        }
    }
    auto idx = [&](std::size_t i, std::size_t j) {
        return static_cast<std::uint32_t>(i * nn + (j % nn));
    };
    for (std::size_t i = 0; i + 1 < nx; ++i) {
        for (std::size_t j = 0; j < nn; ++j) {
            m.triangles.push_back({idx(i, j), idx(i + 1, j), idx(i + 1, j + 1)});
            m.triangles.push_back({idx(i, j), idx(i + 1, j + 1), idx(i, j + 1)});
        }
    }
    for (const std::size_t i : {std::size_t{0}, nx - 1}) {
        const auto c = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.emplace_back(xs[i].first, 0.0, 0.0);
        for (std::size_t j = 0; j < nn; ++j) {
            if (i == 0) {
                m.triangles.push_back({c, idx(i, j + 1), idx(i, j)});
            } else {
                m.triangles.push_back({c, idx(i, j), idx(i, j + 1)});
            }
        }
    }
    if (m.signedVolume() < 0.0) {
        for (auto& t : m.triangles) {
            std::swap(t[1], t[2]);
        }
    }
    return m;
}

/// Closed mesh of a dome z = sqrt(R^2 - x^2 - y^2) on a square base.
geom3d::Mesh dome(double R, int n) {
    geom3d::Mesh m;
    auto f = [&](double x, double y) {
        return std::sqrt(std::max(0.0, R * R - x * x - y * y));
    };
    const double a = R + 5.0;
    auto idx = [&](int i, int j) {
        return static_cast<std::uint32_t>(j * (n + 1) + i);
    };
    for (int j = 0; j <= n; ++j) {
        for (int i = 0; i <= n; ++i) {
            const double x = -a + 2.0 * a * i / n, y = -a + 2.0 * a * j / n;
            m.vertices.emplace_back(x, y, f(x, y));
        }
    }
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            m.triangles.push_back({idx(i, j), idx(i + 1, j), idx(i + 1, j + 1)});
            m.triangles.push_back({idx(i, j), idx(i + 1, j + 1), idx(i, j + 1)});
        }
    }
    return m; // an open surface is enough for drop-cutter
}

} // namespace

TEST_CASE("wrapping maps the unrolled plane onto the cylinder", "[strategies][multiaxis]") {
    toolpath::Toolpath flat;
    flat.start = Vec3(0, 0, 5);
    toolpath::PathBuilder b(flat);
    b.rapid(Vec3(0, 0, 1));
    b.feed(Vec3(0, 0, -2), FeedClass::Plunge, 5);
    b.feed(Vec3(20, kPi * 15.0, -2), FeedClass::Cut, 10); // half a turn at radius 15
    b.arc(Vec3(24, kPi * 15.0 + 4.0, -2), Vec2(20, kPi * 15.0 + 4.0), false, FeedClass::Cut, 10);
    b.retractTo(5);
    const double R = 15.0, tol = 0.005;
    const toolpath::Toolpath w = wrapToolpath(flat, R, tol);
    REQUIRE(toolpath::isMultiAxis(w));
    CHECK((w.start - Vec3(0, 0, 20)).norm() < 1e-12);
    Vec3 prev = w.start;
    for (const auto& m : w.moves) {
        // Radial tool axis through the point.
        const double rho = std::hypot(m.end.y(), m.end.z());
        CHECK((m.axis - Vec3(0, m.end.y() / rho, m.end.z() / rho)).norm() < 1e-9);
        // Chords: the midpoint of each move lies within tol of the cylinder of its radius.
        const double r0 = std::hypot(prev.y(), prev.z());
        const Vec3 mid = 0.5 * (prev + m.end);
        if (std::abs(r0 - rho) < 1e-9) {
            CHECK(rho - std::hypot(mid.y(), mid.z()) <= tol + 1e-9);
        }
        prev = m.end;
    }
    // Half a turn along the cut ends on the bottom of the cylinder at radius 13.
    bool found = false;
    for (const auto& m : w.moves) {
        found = found || (m.end - Vec3(20, 0, -13)).norm() < 1e-9;
    }
    CHECK(found);
    // The arc of radius 4 in the plane stays a curve of length ~2 pi 4 / 4.
    CHECK_THROWS(wrapToolpath(flat, 0.0, tol));
}

TEST_CASE("radial height map and wrapped pocketing of a slot", "[strategies][multiaxis]") {
    const double R = 20.0, depth = 3.0;
    const geom3d::Mesh part = slottedShaft(0, 40, R, 10, 30, 0.3, 1.3, depth);
    REQUIRE(part.signedVolume() > 0.0);
    CancelToken cancel;
    const RadialMap map = radialMap(part, 0, 40, 0.25, 25.0, cancel);
    // Inside the slot the surface is 3 mm lower.
    const int iMid = static_cast<int>(std::lround((20.0 - map.x0) / map.dx));
    const int jIn = static_cast<int>(std::lround(0.8 / (2.0 * kPi) * map.nt));
    const int jOut = static_cast<int>(std::lround(3.0 / (2.0 * kPi) * map.nt));
    // (The floor is faceted: chords of 2 degrees sag by 0.003 mm.)
    CHECK_THAT(map.at(iMid, jIn), WithinAbs(R - depth, 0.005));
    CHECK(map.at(iMid, jOut) > R - 0.01);

    Harness h(tooling::makeFlatEndMill(1, 4.0, 10.0, 50.0));
    h.heights = {R + 5.0, R + 2.0, 1.0};
    RotaryPocketParams p;
    p.stepDown = 1.0;
    const toolpath::Toolpath tp = rotaryPocket(part, R, 0.0, 40.0, p, h.context());
    REQUIRE_FALSE(tp.moves.empty());
    INFO(tp.warnings.size());
    double minRadius = 1e9;
    Vec3 prev = tp.start;
    for (const auto& m : tp.moves) {
        const double rho = std::hypot(m.end.y(), m.end.z());
        CHECK(std::abs(m.axis.x()) < 1e-9);
        if (m.kind != MoveKind::Rapid && rho < R - 1e-6) {
            minRadius = std::min(minRadius, rho);
            // Tool centre inside the slot shrunk by the tool radius (radial walls: the angular
            // margin at radius rho is 2 / rho).
            const double t = std::atan2(m.end.y(), m.end.z());
            CHECK(m.end.x() >= 12.0 - 0.02);
            CHECK(m.end.x() <= 28.0 + 0.02);
            CHECK(t >= 0.3 + 2.0 / rho - 0.02 / rho);
            CHECK(t <= 1.3 - 2.0 / rho + 0.02 / rho);
        }
        // Rapids below the stock radius are radial, or stay over the cleared slot.
        const double rho0 = std::hypot(prev.y(), prev.z());
        const double lateral = std::hypot(m.end.x() - prev.x(), std::atan2(m.end.y(), m.end.z()) -
                                                                    std::atan2(prev.y(), prev.z()));
        if (m.kind == MoveKind::Rapid && std::min(rho, rho0) < R - 1e-6 && lateral > 1e-6) {
            const double t = std::atan2(m.end.y(), m.end.z());
            CHECK(m.end.x() >= 12.0 - 0.02);
            CHECK(m.end.x() <= 28.0 + 0.02);
            CHECK(t >= 0.3 + 1.9 / rho);
            CHECK(t <= 1.3 - 1.9 / rho);
        }
        prev = m.end;
    }
    CHECK_THAT(minRadius, WithinAbs(R - depth, 1e-3));
    CHECK(tp.strategy == "rotary_pocket");
}

TEST_CASE("wrapping refuses a radius with no way around to speak of", "[strategies][multiaxis]") {
    toolpath::Toolpath flat;
    flat.start = Vec3(0.0, 0.0, 0.0);
    toolpath::Move m;
    m.kind = MoveKind::Linear;
    m.end = Vec3(0.0, 1e-3, 0.0);
    flat.moves.push_back(m);
    CHECK_THROWS_WITH(wrapToolpath(flat, 1e-9, 0.002), ContainsSubstring("radius"));
    CHECK_NOTHROW(wrapToolpath(flat, 1.0, 0.002));
}

TEST_CASE("wrapped pocketing refuses a part across the axis, not one that ends on it",
          "[strategies][multiaxis]") {
    const double R = 20.0;
    Harness h(tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    RotaryPocketParams p;
    p.stepDown = 1.0;
    SECTION("a shaft standing on the axis, as turned about Z") {
        geom3d::Mesh part = slottedShaft(0, 60, R, 25, 35, 0.3, 1.3, 3.0);
        for (Vec3& v : part.vertices) {
            v = Vec3(v.z(), v.y(), -v.x());
        }
        const double stockRadius = std::hypot(R, 60.0);
        h.heights = {stockRadius + 5.0, stockRadius + 2.0, 1.0};
        CHECK_THROWS_WITH(rotaryPocket(part, stockRadius, -R, R, p, h.context()),
                          ContainsSubstring("does not go around the WCS X axis"));
    }
    SECTION("a shaft coming nearly to a point goes around it, within the tool's radius") {
        geom3d::Mesh part = slottedShaft(0, 40, R, 10, 30, 0.3, 1.3, 3.0);
        // The last ring in to a small end, far enough along to be a cone.
        const double tip = 1.0;
        for (Vec3& v : part.vertices) {
            if (v.x() == 40.0) {
                v = Vec3(50.0, v.y() * tip / R, v.z() * tip / R);
            }
        }
        h.heights = {R + 5.0, R + 2.0, 1.0};
        p.stepDown = 5.0;
        p.resolution = 1.0;
        CHECK_NOTHROW(rotaryPocket(part, R, 0.0, 50.0, p, h.context()));
    }
    SECTION("a shaft coming to a point stops where the tool is wider than the way around") {
        // Levels this near the axis wrapped one short move into hundreds of millions: 191 GB on a
        // true point, stopped by hand. The test programs' memory cap ends this one if it returns.
        for (const double tip : {0.0, 0.01}) {
            INFO("tip " << tip);
            geom3d::Mesh part = slottedShaft(0, 40, R, 10, 30, 0.3, 1.3, 3.0);
            for (Vec3& v : part.vertices) {
                if (v.x() == 40.0) {
                    v = Vec3(50.0, v.y() * tip / R, v.z() * tip / R);
                }
            }
            h.heights = {R + 5.0, R + 2.0, 1.0};
            p.stepDown = 5.0;
            p.resolution = 1.0;
            const toolpath::Toolpath tp = rotaryPocket(part, R, 0.0, 50.0, p, h.context());
            CHECK_FALSE(tp.moves.empty());
            CHECK(tp.moves.size() < 2000000);
            CHECK(std::all_of(tp.moves.begin(), tp.moves.end(),
                              [](const toolpath::Move& m) { return std::isfinite(m.end.norm()); }));
            CHECK(std::any_of(tp.warnings.begin(), tp.warnings.end(), [](const std::string& w) {
                return w.find("wider than the way around") != std::string::npos;
            }));
        }
    }
    SECTION("a part held off the axis does not come to it") {
        // A shell of a part above the axis: no ray reaches it from below.
        geom3d::Mesh part = slottedShaft(0, 40, R, 10, 30, 0.3, 1.3, 3.0);
        for (Vec3& v : part.vertices) {
            v.z() = 30.0 + 0.25 * v.z();
            v.y() *= 0.25;
        }
        const double stockRadius = std::hypot(5.0, 35.0);
        h.heights = {stockRadius + 5.0, stockRadius + 2.0, 1.0};
        p.stepDown = 5.0;
        p.resolution = 1.0;
        CHECK_NOTHROW(rotaryPocket(part, stockRadius, 0.0, 40.0, p, h.context()));
    }
}

TEST_CASE("wrapped pocketing of a slot across the map's seam reaches both walls",
          "[strategies][multiaxis]") {
    const double R = 20.0, depth = 3.0, half = 0.6;
    // Centred on a mesh edge, which is then turned onto angle 0.
    const double centre = 2.0 * kPi * 60 / 180;
    geom3d::Mesh part = slottedShaft(0, 40, R, 10, 30, centre - half, centre + half, depth);
    for (Vec3& v : part.vertices) {
        const double c = std::cos(centre), s = std::sin(centre);
        v = Vec3(v.x(), c * v.y() - s * v.z(), s * v.y() + c * v.z());
    }
    Harness h(tooling::makeFlatEndMill(1, 4.0, 10.0, 50.0));
    h.heights = {R + 5.0, R + 2.0, 1.0};
    RotaryPocketParams p;
    p.stepDown = 1.0;
    const toolpath::Toolpath tp = rotaryPocket(part, R, 0.0, 40.0, p, h.context());
    double lowest = 1e9, highest = -1e9;
    for (const auto& m : tp.moves) {
        const double rho = std::hypot(m.end.y(), m.end.z());
        if (m.kind != MoveKind::Rapid && rho < R - depth + 1e-3) {
            const double t = std::atan2(m.end.y(), m.end.z());
            lowest = std::min(lowest, t);
            highest = std::max(highest, t);
        }
    }
    const double reach = half - 2.0 / (R - depth);
    CHECK_THAT(lowest, WithinAbs(-reach, 0.02 / R));
    CHECK_THAT(highest, WithinAbs(reach, 0.02 / R));
}

TEST_CASE("tool axis rules", "[strategies][multiaxis]") {
    ToolAxisParams p;
    p.lead = degToRad(10.0);
    const Vec3 n = Vec3::UnitZ(), f = Vec3::UnitX();
    CHECK(
        (toolAxisFor(p, Vec3::Zero(), n, f) - Vec3(std::sin(p.lead), 0, std::cos(p.lead))).norm() <
        1e-12);
    p.lead = 0.0;
    p.tilt = degToRad(10.0);
    // Tilt to the left of +X with the normal up: towards +Y.
    CHECK(toolAxisFor(p, Vec3::Zero(), n, f).y() > 0.17);
    // Limited to the largest tilt from Z.
    p.tilt = 0.0;
    p.maxTilt = degToRad(30.0);
    const Vec3 wall = toolAxisFor(p, Vec3::Zero(), Vec3::UnitX(), Vec3::UnitY());
    CHECK_THAT(std::acos(wall.z()), WithinAbs(degToRad(30.0), 1e-12));
    CHECK(wall.x() > 0.0);
    p.mode = ToolAxisMode::FromPoint;
    p.point = Vec3(0, 0, -100);
    p.maxTilt = degToRad(90.0);
    CHECK((toolAxisFor(p, Vec3(10, 0, 0), n, f) - Vec3(10, 0, 100).normalized()).norm() < 1e-12);
    p.mode = ToolAxisMode::Fixed;
    p.fixedTilt = degToRad(20.0);
    p.fixedAzimuth = degToRad(90.0);
    CHECK(
        (toolAxisFor(p, Vec3::Zero(), n, f) - Vec3(0, std::sin(p.fixedTilt), std::cos(p.fixedTilt)))
            .norm() < 1e-12);
}

TEST_CASE("5-axis ball finishing keeps the 3-axis ball centres", "[strategies][multiaxis]") {
    const geom3d::Mesh part = dome(20.0, 60);
    Harness h(tooling::makeBallEndMill(2, 6.0, 15.0, 60.0));
    h.heights = {40.0, 30.0, 1.0};
    MultiAxisFinishParams p;
    p.pattern.stepover = 2.0;
    // On the dome only: at its foot a tool leaning forward runs into it (reported, see below).
    p.pattern.boundary = {rect(-12, -12, 12, 12)};
    p.axis.mode = ToolAxisMode::LeadTilt;
    p.axis.lead = degToRad(15.0);
    p.axis.maxTilt = degToRad(60.0);
    const toolpath::Toolpath tp5 = multiAxisFinish(part, 0.0, p, h.context());
    const toolpath::Toolpath tp3 = parallelFinish(part, 0.0, p.pattern, h.context());
    REQUIRE(tp5.moves.size() == tp3.moves.size());
    REQUIRE(toolpath::isMultiAxis(tp5));
    const double r = 3.0;
    int tilted = 0;
    for (std::size_t k = 0; k < tp5.moves.size(); ++k) {
        const auto& m5 = tp5.moves[k];
        const auto& m3 = tp3.moves[k];
        CHECK_THAT(m5.axis.norm(), WithinAbs(1.0, 1e-12));
        // Same ball centre.
        CHECK(((m5.end + r * m5.axis) - (m3.end + r * Vec3::UnitZ())).norm() < 1e-9);
        CHECK(std::acos(std::clamp(m5.axis.z(), -1.0, 1.0)) <= degToRad(60.0) + 1e-9);
        if (m5.feedClass == FeedClass::Cut && m3.end.z() > 1.0) {
            // On the dome the normal points from the sphere centre to the ball centre; the axis
            // leans 15 degrees from it unless the tilt limit applies (facets of the mesh turn
            // the normal by up to about a degree).
            const Vec3 n = (m3.end + r * Vec3::UnitZ()).normalized();
            const double angle = std::acos(std::clamp(n.dot(m5.axis), -1.0, 1.0));
            if (std::acos(n.z()) + degToRad(15.0) < degToRad(59.0)) {
                CHECK_THAT(angle, WithinAbs(degToRad(15.0), degToRad(2.5)));
                ++tilted;
            }
        }
    }
    CHECK(tilted > 100);
    // A convex dome: the tilted tool does not touch it elsewhere.
    for (const auto& w : tp5.warnings) {
        INFO(w);
        CHECK(w.find("flutes") == std::string::npos);
    }
    // Over the whole base the leaning tool reaches the dome's foot from the flat around it.
    MultiAxisFinishParams all = p;
    all.pattern.boundary.clear();
    const toolpath::Toolpath foot = multiAxisFinish(part, 0.0, all, h.context());
    bool footWarned = false;
    for (const auto& w : foot.warnings) {
        footWarned = footWarned || w.find("flutes") != std::string::npos;
    }
    CHECK(footWarned);
    CHECK(tp5.strategy == "multiaxis_finish");

    // A flat end mill is refused.
    Harness flat(tooling::makeFlatEndMill(3, 6.0, 15.0, 60.0));
    CHECK_THROWS(multiAxisFinish(part, 0.0, p, flat.context()));
}

TEST_CASE("5-axis finishing reports flutes cutting into the part", "[strategies][multiaxis]") {
    // A step: the tool tilted 45 degrees towards the wall drags its flutes into it.
    auto f = [](double x, double) {
        return x > 10.0 ? 15.0 : 0.0;
    };
    geom3d::Mesh step;
    {
        // Two flat plates and the vertical wall between them.
        auto quad = [&](const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
            const auto base = static_cast<std::uint32_t>(step.vertices.size());
            step.vertices.insert(step.vertices.end(), {a, b, c, d});
            step.triangles.push_back({base, base + 1, base + 2});
            step.triangles.push_back({base, base + 2, base + 3});
        };
        quad({-20, -20, 0}, {10, -20, 0}, {10, 20, 0}, {-20, 20, 0});
        quad({10, -20, 0}, {10, -20, 15}, {10, 20, 15}, {10, 20, 0});
        quad({10, -20, 15}, {40, -20, 15}, {40, 20, 15}, {10, 20, 15});
    }
    (void)f;
    Harness h(tooling::makeBallEndMill(2, 6.0, 20.0, 60.0));
    h.heights = {40.0, 30.0, 1.0};
    MultiAxisFinishParams p;
    p.pattern.stepover = 3.0;
    p.pattern.boundary = {rect(-15, -10, 5, 10)};
    p.axis.mode = ToolAxisMode::Fixed;
    p.axis.fixedTilt = degToRad(45.0);
    p.axis.fixedAzimuth = 0.0; // leaning towards +X, over the wall
    const toolpath::Toolpath tp = multiAxisFinish(step, -1.0, p, h.context());
    bool warned = false;
    for (const auto& w : tp.warnings) {
        warned = warned || w.find("flutes cut into the part") != std::string::npos;
    }
    CHECK(warned);
}
