// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Swarf milling (DL-0085) on analytic ruled walls: a cone, a twisted bilinear patch, drafted
// planar walls meeting at convex and concave corners.

#include "StrategyTestUtil.hpp"

#include <ocam/foundation/Units.hpp>
#include <ocam/strategies/MultiAxis.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <set>

using namespace ocam;
using namespace ocam::strategies;
using namespace ocam::strategies::testing;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using toolpath::FeedClass;

namespace {

constexpr int kSamples = 9;

/// The ruling from `bottom` to `top` of a surface whose outward normal along it is `normal(t)`.
template <class Normal> WallRuling ruling(const Vec3& bottom, const Vec3& top, Normal normal) {
    WallRuling w;
    for (int k = 0; k < kSamples; ++k) {
        const double t = static_cast<double>(k) / (kSamples - 1);
        w.points.push_back((1 - t) * bottom + t * top);
        w.normals.push_back(normal(t).normalized());
    }
    w.direction = (top - bottom).normalized();
    return w;
}

/// The outside of a cone about Z through (0, 0): radius r0 at z0, r1 at z1, n rulings over the
/// angles [a0, a1] (a whole turn closes on itself).
RuledWall cone(double r0, double r1, double z0, double z1, int n, double a0 = 0.0,
               double a1 = 2.0 * kPi) {
    RuledWall wall;
    wall.name = "cone";
    for (int i = 0; i <= n; ++i) {
        const double a = a0 + (a1 - a0) * i / n;
        const Vec3 e(std::cos(a), std::sin(a), 0.0);
        const Vec3 n0 = (z1 - z0) * e + (r0 - r1) * Vec3::UnitZ();
        wall.rulings.push_back(ruling(r0 * e + z0 * Vec3::UnitZ(), r1 * e + z1 * Vec3::UnitZ(),
                                      [&](double) { return n0; }));
    }
    return wall;
}

/// The bilinear patch between the bottom edge b0-b1 and the top edge t0-t1, outward normal on
/// the side of `outside`.
RuledWall bilinear(const Vec3& b0, const Vec3& b1, const Vec3& t0, const Vec3& t1, int n,
                   const Vec3& outside) {
    RuledWall wall;
    wall.name = "twisted";
    for (int i = 0; i <= n; ++i) {
        const double s = static_cast<double>(i) / n;
        const Vec3 bottom = (1 - s) * b0 + s * b1;
        const Vec3 top = (1 - s) * t0 + s * t1;
        wall.rulings.push_back(ruling(bottom, top, [&](double t) {
            const Vec3 across = (1 - t) * (b1 - b0) + t * (t1 - t0);
            Vec3 nn = across.cross(top - bottom);
            return nn.dot(outside) < 0 ? Vec3(-nn) : nn;
        }));
    }
    return wall;
}

/// A drafted planar wall from bottom a to b (z = 0), leaning `draft` towards -normal, height h.
RuledWall drafted(const Vec2& a, const Vec2& b, double h, double draft, int n,
                  const std::string& name) {
    const Vec2 along = (b - a).normalized();
    const Vec2 out2(along.y(), -along.x()); // right of a -> b
    const Vec3 out(out2.x(), out2.y(), 0.0);
    const Vec3 lean = -std::tan(draft) * h * out;
    const Vec3 normal = std::cos(draft) * out + std::sin(draft) * Vec3::UnitZ();
    RuledWall wall;
    wall.name = name;
    for (int i = 0; i <= n; ++i) {
        const Vec2 p = a + (b - a) * (static_cast<double>(i) / n);
        const Vec3 bottom(p.x(), p.y(), 0.0);
        wall.rulings.push_back(
            ruling(bottom, bottom + lean + h * Vec3::UnitZ(), [&](double) { return normal; }));
    }
    return wall;
}

/// Where the first lead in reaches the wall: the start of the cut.
Vec3 leadInEnd(const toolpath::Toolpath& tp) {
    for (const toolpath::Move& m : tp.moves) {
        if (m.feedClass == FeedClass::LeadIn) {
            return m.end;
        }
    }
    FAIL("no lead in");
    return Vec3::Zero();
}

std::vector<const toolpath::Move*> cuts(const toolpath::Toolpath& tp) {
    std::vector<const toolpath::Move*> out;
    for (const toolpath::Move& m : tp.moves) {
        if (m.feedClass == FeedClass::Cut) {
            out.push_back(&m);
        }
    }
    return out;
}

} // namespace

TEST_CASE("swarf on a cone: the flank along every generator", "[strategies][swarf]") {
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    const double r = 3.0;
    const RuledWall wall = cone(18.0, 13.0, 0.0, 20.0, 180);
    const double tilt = std::atan(5.0 / 20.0);
    SwarfParams p;
    const toolpath::Toolpath tp = swarf({}, {wall}, std::nullopt, p, h.context());
    REQUIRE(tp.strategy == "swarf5x");
    REQUIRE(toolpath::isMultiAxis(tp));
    const auto cut = cuts(tp);
    REQUIRE(cut.size() == 180);
    double climb = 0.0;
    Vec3 previous = leadInEnd(tp);
    for (const toolpath::Move* m : cut) {
        const Vec3 e = Vec3(m->end.x(), m->end.y(), 0.0).normalized();
        CHECK_THAT(std::acos(m->axis.z()), WithinAbs(tilt, 1e-9));
        // A tool radius from the generator, parallel to it, the flat end square on its foot.
        const Vec3 foot = 18.0 * e;
        const Vec3 g = (13.0 * e + 20.0 * Vec3::UnitZ() - foot).normalized();
        CHECK(g.dot(m->axis) > 1.0 - 1e-12);
        const Vec3 d = foot - m->end;
        CHECK_THAT((d - d.dot(m->axis) * m->axis).norm(), WithinAbs(r, 1e-9));
        CHECK_THAT(d.dot(m->axis), WithinAbs(0.0, 1e-9));
        climb += (-e).dot((m->end - previous).cross(m->axis));
        previous = m->end;
        CHECK(m->feedRate == h.feeds.cut);
    }
    CHECK(climb > 0.0); // the wall on the right of the feed
    // The loop is closed: it ends where it began, then leads off the wall.
    CHECK((cut.back()->end - leadInEnd(tp)).norm() < 1e-9);
    std::set<FeedClass> classes;
    for (const toolpath::Move& m : tp.moves) {
        classes.insert(m.feedClass);
        if (m.feedClass == FeedClass::LeadIn || m.feedClass == FeedClass::LeadOut) {
            CHECK(m.feedRate == h.feeds.lead);
        }
        if (m.feedClass == FeedClass::Plunge) {
            CHECK(m.feedRate == h.feeds.plunge);
        }
    }
    CHECK(classes.contains(FeedClass::LeadIn));
    CHECK(classes.contains(FeedClass::LeadOut));

    SECTION("conventional runs the other way") {
        p.direction = CutDirection::Conventional;
        const toolpath::Toolpath conv = swarf({}, {wall}, std::nullopt, p, h.context());
        double turn = 0.0;
        const auto c = cuts(conv);
        for (std::size_t k = 1; k < c.size(); ++k) {
            const Vec3 e = Vec3(c[k]->end.x(), c[k]->end.y(), 0.0).normalized();
            turn += (-e).dot((c[k]->end - c[k - 1]->end).cross(c[k]->axis));
        }
        CHECK(turn < 0.0);
    }
    SECTION("depth passes, the deepest last, and a floor to stay above") {
        p.depthPasses = 3;
        p.axisOffset = 0.5;
        const SwarfFloor floor{Vec3(0, 0, -2.0), Vec3::UnitZ()};
        const toolpath::Toolpath deep = swarf({}, {wall}, floor, p, h.context());
        std::vector<double> ends;
        for (const toolpath::Move* m : cuts(deep)) {
            if (ends.empty() || std::abs(ends.back() - m->end.z()) > 1e-6) {
                ends.push_back(m->end.z());
            }
        }
        REQUIRE(ends.size() == 3);
        // The last: the lowest point of the tilted flat end 0.5 above the floor (below the
        // wall's foot, which stands 2 above it).
        CHECK_THAT(ends[2] - r * std::sin(tilt), WithinAbs(-1.5, 1e-9));
        CHECK(ends[0] > ends[1]);
        CHECK(ends[1] > ends[2]);
        // Equal shares of the wall's height above the tool's end, along the axis.
        CHECK_THAT((ends[0] - ends[1]) - (ends[1] - ends[2]), WithinAbs(0.0, 1e-9));
    }
    SECTION("rulings steeper than the largest tilt are refused, not tilted back") {
        p.maxTilt = degToRad(10.0);
        CHECK_THROWS_WITH(swarf({}, {wall}, std::nullopt, p, h.context()),
                          ContainsSubstring("more than the largest tilt"));
    }
    SECTION("only end mills cut with their side") {
        Harness drill(tooling::makeDrill(2, 6.0, degToRad(118.0), 30.0, 60.0));
        CHECK_THROWS_WITH(swarf({}, {wall}, std::nullopt, p, drill.context()),
                          ContainsSubstring("side of the tool"));
    }
}

TEST_CASE("swarf on a twisted ruled wall: the flank error is measured and limited",
          "[strategies][swarf]") {
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    const double r = 3.0;
    // The top edge turned 10 degrees against the bottom one about the patch's middle.
    const double turn = degToRad(10.0);
    const Vec3 b0(0, -14, 0), b1(0, 14, 0);
    const Vec3 t0(14 * std::sin(turn), -14 * std::cos(turn), 20);
    const Vec3 t1(-14 * std::sin(turn), 14 * std::cos(turn), 20);
    const RuledWall wall = bilinear(b0, b1, t0, t1, 60, Vec3::UnitX());
    SwarfParams p;
    p.tolerance = 0.02;
    const toolpath::Toolpath tp = swarf({}, {wall}, std::nullopt, p, h.context());
    REQUIRE_FALSE(tp.warnings.empty());
    CHECK_THAT(tp.warnings.back(), ContainsSubstring("twisted"));
    // Every position: the axis through both ends of a ruling offset by the radius along their
    // normals (the tool's end at the bottom one), the flank within the tolerance between them.
    const auto offAxis = [](const Vec3& q, const toolpath::Move& m) {
        const Vec3 e = q - m.end;
        return (e - e.dot(m.axis) * m.axis).norm();
    };
    double worst = 0.0;
    int matched = 0;
    for (const toolpath::Move* m : cuts(tp)) {
        for (const WallRuling& w : wall.rulings) {
            const Vec3 a = w.points.front() + r * w.normals.front();
            const Vec3 b = w.points.back() + r * w.normals.back();
            if ((a - m->end).norm() > 1e-9 || offAxis(b, *m) > 1e-9) {
                continue;
            }
            ++matched;
            for (const Vec3& q : w.points) {
                worst = std::max(worst, std::abs(offAxis(q, *m) - r));
            }
        }
    }
    CHECK(matched == static_cast<int>(cuts(tp).size()));
    // At the corners of the patch its normal points down at the foot: leads stay level.
    CHECK(wall.rulings.front().normals.front().z() < -0.05);
    for (std::size_t k = 1; k < tp.moves.size(); ++k) {
        const toolpath::Move& m = tp.moves[k];
        if (m.feedClass == FeedClass::LeadIn || m.feedClass == FeedClass::LeadOut) {
            CHECK_THAT(m.end.z(), WithinAbs(tp.moves[k - 1].end.z(), 1e-9));
        }
    }
    CHECK(worst > 0.002); // twisted: not exact between the ends
    CHECK(worst <= 0.02); // but within the tolerance
    p.tolerance = 0.002;
    CHECK_THROWS_WITH(swarf({}, {wall}, std::nullopt, p, h.context()),
                      ContainsSubstring("twist too much"));
}

TEST_CASE("swarf turns about convex corners and refuses inside ones", "[strategies][swarf]") {
    Harness h(tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
    const double r = 3.0;
    // Upright walls: neighbours share the corner's edge as a ruling. (Drafted planes ruled
    // along their steepest direction do not; each is then cut on its own.)
    const double draft = 0.0;
    // A square boss, its walls given in any order and direction.
    std::vector<RuledWall> boss{drafted({-30, 0}, {0, 0}, 15, draft, 20, "front"),
                                drafted({0, 30}, {-30, 30}, 15, draft, 20, "back"),
                                drafted({0, 0}, {0, 30}, 15, draft, 20, "right"),
                                drafted({-30, 30}, {-30, 0}, 15, draft, 20, "left")};
    std::reverse(boss[1].rulings.begin(), boss[1].rulings.end());
    std::reverse(boss[2].rulings.begin(), boss[2].rulings.end());
    SwarfParams p;
    const toolpath::Toolpath tp = swarf({}, boss, std::nullopt, p, h.context());
    // One closed pass round the boss: a single lead in.
    int leads = 0;
    for (const toolpath::Move& m : tp.moves) {
        leads += m.feedClass == FeedClass::LeadIn ? 1 : 0;
    }
    CHECK(leads == 1);
    // Round each corner the tool turns about the corner's edge, a radius from it, with its end on
    // the edge's foot.
    const auto c = cuts(tp);
    int turning = 0;
    for (const toolpath::Move* m : c) {
        for (const Vec2& corner : {Vec2(0, 0), Vec2(0, 30), Vec2(-30, 30), Vec2(-30, 0)}) {
            const Vec3 foot(corner.x(), corner.y(), 0.0);
            const double off = (m->end - foot).norm();
            if (off < r + 1e-6) {
                CHECK_THAT(off, WithinAbs(r, 1e-6));
                ++turning;
            }
        }
    }
    CHECK(turning > 4 * 20); // a quarter turn at each corner, in steps within the tolerance
    CHECK((c.back()->end - leadInEnd(tp)).norm() < 1e-9);

    // A pocket: walls seen from inside meet in inside corners.
    std::vector<RuledWall> pocket{drafted({0, 0}, {-30, 0}, 15, draft, 20, "front"),
                                  drafted({0, 30}, {0, 0}, 15, draft, 20, "right")};
    CHECK_THROWS_WITH(swarf({}, pocket, std::nullopt, p, h.context()),
                      ContainsSubstring("inside corner"));
}
