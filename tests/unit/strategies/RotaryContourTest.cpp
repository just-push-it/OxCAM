// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Wrapped contour (DL-0084): keyways, helical grooves, wrapped outlines and ring grooves on a
// cylinder about X.

#include "StrategyTestUtil.hpp"

#include <ocam/strategies/MultiAxis.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>

using namespace ocam;
using namespace ocam::strategies;
using namespace ocam::strategies::testing;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using toolpath::FeedClass;
using toolpath::MoveKind;

namespace {

constexpr double kR = 20.0; // the stock's radius
constexpr double kD = 6.0;  // the tool's diameter

Vec3 onCylinder(double x, double angle, double radius) {
    return {x, radius * std::sin(angle), radius * std::cos(angle)};
}
double radiusOf(const Vec3& p) {
    return std::hypot(p.y(), p.z());
}
double angleOf(const Vec3& p) {
    return std::atan2(p.y(), p.z());
}

/// A straight line along X at one angle.
WrappedCurve lineAlongX(double x0, double x1, double angle, double radius) {
    WrappedCurve c;
    for (int k = 0; k <= 30; ++k) {
        c.points.push_back(onCylinder(x0 + (x1 - x0) * k / 30.0, angle, radius));
    }
    return c;
}

/// The loop around [x0, x1] x [t0, t1] at `radius` (counter-clockwise in x and angle).
WrappedCurve box(double x0, double x1, double t0, double t1, double radius) {
    WrappedCurve c;
    c.closed = true;
    const int n = 40;
    for (int k = 0; k < n; ++k) {
        c.points.push_back(onCylinder(x0 + (x1 - x0) * k / n, t0, radius));
    }
    for (int k = 0; k < n; ++k) {
        c.points.push_back(onCylinder(x1, t0 + (t1 - t0) * k / n, radius));
    }
    for (int k = 0; k < n; ++k) {
        c.points.push_back(onCylinder(x1 - (x1 - x0) * k / n, t1, radius));
    }
    for (int k = 0; k < n; ++k) {
        c.points.push_back(onCylinder(x0, t1 - (t1 - t0) * k / n, radius));
    }
    return c;
}

/// A circle about X at x.
WrappedCurve ring(double x, double radius) {
    WrappedCurve c;
    c.closed = true;
    for (int k = 0; k < 360; ++k) {
        c.points.push_back(onCylinder(x, 2.0 * kPi * k / 360.0, radius));
    }
    return c;
}

struct Fixture {
    Harness h{tooling::makeFlatEndMill(1, kD, 15.0, 50.0)};
    RotaryContourParams p;
    Fixture() {
        h.heights = {kR + 5.0, kR + 2.0, 1.0};
        p.stepDown = 1.5;
        p.stepover = 2.0;
    }
    toolpath::Toolpath run(const std::vector<WrappedCurve>& curves,
                           const std::vector<WrappedSlot>& slots = {}) {
        return rotaryContour(curves, slots, kR, 0.0, 50.0, p, h.context());
    }
};

/// The points of the cutting moves at a radius (within 1e-6), in order: each run's start and
/// every move's end.
std::vector<Vec3> cutsAt(const toolpath::Toolpath& tp, double radius) {
    std::vector<Vec3> out;
    Vec3 from = tp.start;
    for (const toolpath::Move& m : tp.moves) {
        if (m.kind == MoveKind::Linear && m.feedClass == FeedClass::Cut &&
            std::abs(radiusOf(m.end) - radius) < 1e-6) {
            if (std::abs(radiusOf(from) - radius) < 1e-6 &&
                (out.empty() || (out.back() - from).norm() > 1e-12)) {
                out.push_back(from);
            }
            out.push_back(m.end);
        }
        from = m.end;
    }
    return out;
}

/// Every move's tool axis is radial, and no move goes below `floor`.
void checkRadial(const toolpath::Toolpath& tp, double floor) {
    double deepest = 1e9;
    for (const toolpath::Move& m : tp.moves) {
        CHECK(std::abs(m.axis.x()) < 1e-9);
        CHECK((m.axis - Vec3(0.0, m.end.y(), m.end.z()).normalized()).norm() < 1e-9);
        deepest = std::min(deepest, radiusOf(m.end));
    }
    CHECK_THAT(deepest, WithinAbs(floor, 1e-6));
}

} // namespace

TEST_CASE("wrapped contour: a keyway along the axis", "[strategies][multiaxis][wrap]") {
    Fixture f;
    const double t0 = 0.7; // where it is around the part
    SECTION("on a line on the surface, the depth below it") {
        f.p.depth = 3.0;
        const toolpath::Toolpath tp = f.run({lineAlongX(10.0, 40.0, t0, kR)});
        CHECK(tp.strategy == "rotary_contour");
        checkRadial(tp, kR - 3.0);
        // Levels from the stock, 1.5 apart, down to the floor; at each, the line, at its angle:
        // the arc length the line was unrolled to wraps back to it.
        for (const double level : {kR - 1.5, kR - 3.0}) {
            const auto cuts = cutsAt(tp, level);
            REQUIRE_FALSE(cuts.empty());
            for (const Vec3& q : cuts) {
                CHECK_THAT(angleOf(q), WithinAbs(t0, 1e-9));
                CHECK(q.x() >= 10.0 - 1e-6);
                CHECK(q.x() <= 40.0 + 1e-6);
            }
            CHECK_THAT(cuts.back().x(), WithinAbs(40.0, 1e-6));
        }
        // On the line there is no wall beside it: no leads.
        for (const toolpath::Move& m : tp.moves) {
            CHECK(m.feedClass != FeedClass::LeadIn);
            CHECK(m.feedClass != FeedClass::LeadOut);
        }
    }
    SECTION("its floor, the tool's width: once along its middle") {
        // A slot 6 mm wide along the arc at its floor, 3 mm deep, closed at both ends.
        const double floor = kR - 3.0;
        const double half = (kD / 2.0) / floor;
        WrappedSlot slot;
        slot.faces = {{box(10.0, 40.0, t0 - half, t0 + half, floor)}};
        slot.floorRadius = floor;
        const toolpath::Toolpath tp = f.run({}, {slot});
        INFO((tp.warnings.empty() ? std::string() : tp.warnings.front()));
        checkRadial(tp, floor);
        const auto cuts = cutsAt(tp, floor);
        REQUIRE(cuts.size() >= 2);
        for (const Vec3& q : cuts) {
            CHECK_THAT(angleOf(q), WithinAbs(t0, 1e-3 / floor));
            CHECK(q.x() >= 13.0 - 0.01);
            CHECK(q.x() <= 37.0 + 0.01);
        }
        // From end to end, the ends the tool's radius from the slot's: one pass a level.
        double lo = 1e9, hi = -1e9;
        for (const Vec3& q : cuts) {
            lo = std::min(lo, q.x());
            hi = std::max(hi, q.x());
        }
        CHECK_THAT(lo, WithinAbs(13.0, 0.02));
        CHECK_THAT(hi, WithinAbs(37.0, 0.02));
        std::size_t plunges = 0;
        for (const toolpath::Move& m : tp.moves) {
            plunges += m.feedClass == FeedClass::Plunge ? 1 : 0;
        }
        CHECK(plunges == 2); // two levels, one pass each
    }
    SECTION("a slot open at the part's end runs out past it") {
        const double floor = kR - 3.0;
        const double half = (kD / 2.0) / floor;
        WrappedSlot slot;
        slot.faces = {{box(30.0, 50.0, t0 - half, t0 + half, floor)}};
        slot.floorRadius = floor;
        const auto cuts = cutsAt(f.run({}, {slot}), floor);
        REQUIRE_FALSE(cuts.empty());
        double hi = -1e9;
        for (const Vec3& q : cuts) {
            hi = std::max(hi, q.x());
        }
        CHECK(hi >= 50.0 + kD / 2.0 - 0.02); // the whole tool past the end
    }
    SECTION("a slot narrower than the tool is refused") {
        const double floor = kR - 3.0;
        const double half = (kD / 2.0 - 0.2) / floor;
        WrappedSlot slot;
        slot.faces = {{box(10.0, 40.0, t0 - half, t0 + half, floor)}};
        slot.floorRadius = floor;
        CHECK_THROWS_WITH(f.run({}, {slot}), ContainsSubstring("narrower than the tool"));
    }
}

TEST_CASE("wrapped contour: beside a wall", "[strategies][multiaxis][wrap]") {
    Fixture f;
    const double floor = kR - 3.0;
    // Picked from the higher X: it still runs from the lower.
    WrappedCurve wall = lineAlongX(40.0, 10.0, 0.0, floor);
    SECTION("left, climbing: along +X, the tool at +r/rho") {
        f.p.side = WrapSide::Left;
        const toolpath::Toolpath tp = f.run({wall});
        checkRadial(tp, floor);
        const auto cuts = cutsAt(tp, floor);
        REQUIRE(cuts.size() >= 2);
        for (const Vec3& q : cuts) {
            CHECK_THAT(angleOf(q), WithinAbs((kD / 2.0) / floor, 1e-6));
        }
        CHECK(cuts.back().x() > cuts.front().x());
        CHECK_THAT(cuts.back().x(), WithinAbs(40.0, 1e-6));
    }
    SECTION("right, conventional: along +X too, on the other side") {
        f.p.side = WrapSide::Right;
        f.p.direction = CutDirection::Conventional;
        const auto cuts = cutsAt(f.run({wall}), floor);
        REQUIRE(cuts.size() >= 2);
        for (const Vec3& q : cuts) {
            CHECK_THAT(angleOf(q), WithinAbs(-(kD / 2.0) / floor, 1e-6));
        }
        CHECK(cuts.back().x() > cuts.front().x());
    }
    SECTION("right, climbing: the other way") {
        f.p.side = WrapSide::Right;
        const auto cuts = cutsAt(f.run({wall}), floor);
        REQUIRE(cuts.size() >= 2);
        CHECK(cuts.back().x() < cuts.front().x());
    }
}

TEST_CASE("wrapped contour: a helical groove", "[strategies][multiaxis][wrap]") {
    Fixture f;
    // One and a half turns at 5 mm a turn, from x = 10, on the surface; 2 mm deep.
    const double pitch = 5.0, turns = 1.5;
    WrappedCurve helix;
    const int n = 900;
    for (int k = 0; k <= n; ++k) {
        const double t = 2.0 * kPi * turns * k / n;
        helix.points.push_back(onCylinder(10.0 + pitch * t / (2.0 * kPi), t, kR));
    }
    f.p.depth = 2.0;
    const double floor = kR - 2.0;
    const toolpath::Toolpath tp = f.run({helix});
    checkRadial(tp, floor);
    const auto cuts = cutsAt(tp, floor);
    REQUIRE(cuts.size() > 10);
    // Each point is on the helix: its angle is the helix's at its x.
    double unwrapped = angleOf(cuts.front()), lo = unwrapped, hi = unwrapped;
    for (std::size_t k = 0; k < cuts.size(); ++k) {
        const Vec3& q = cuts[k];
        const double expected = 2.0 * kPi * (q.x() - 10.0) / pitch;
        CHECK_THAT(std::remainder(angleOf(q) - expected, 2.0 * kPi), WithinAbs(0.0, 1e-6));
        if (k > 0) {
            unwrapped += std::remainder(angleOf(q) - angleOf(cuts[k - 1]), 2.0 * kPi);
            lo = std::min(lo, unwrapped);
            hi = std::max(hi, unwrapped);
        }
    }
    // The way around it goes: one and a half turns at the floor.
    CHECK_THAT(hi - lo, WithinAbs(2.0 * kPi * turns, 1e-6));
    // The flat path's length along the arc at the floor is what the helix unrolls to there.
    double length = 0.0;
    for (std::size_t k = 1; k < cuts.size(); ++k) {
        length += (cuts[k] - cuts[k - 1]).norm();
    }
    const double unrolled = std::hypot(pitch * turns, 2.0 * kPi * turns * floor);
    CHECK_THAT(length, WithinAbs(unrolled, 0.05));
}

TEST_CASE("wrapped contour: a closed wrapped outline", "[strategies][multiaxis][wrap]") {
    Fixture f;
    // The foot of a raised pad 3 mm high: 20 mm along X, 1 rad around.
    const double floor = kR - 3.0;
    const WrappedCurve pad = box(10.0, 30.0, -0.5, 0.5, floor);
    f.p.side = WrapSide::Right; // outside
    const toolpath::Toolpath tp = f.run({pad});
    checkRadial(tp, floor);
    // Unrolled at the floor, every point keeps the tool's radius from the pad's outline.
    const auto distance = [&](const Vec3& q) {
        const Vec2 u(q.x(), floor * angleOf(q));
        const double dx = std::max({10.0 - u.x(), 0.0, u.x() - 30.0});
        const double dy = std::max({-0.5 * floor - u.y(), 0.0, u.y() - 0.5 * floor});
        return std::hypot(dx, dy);
    };
    const auto cuts = cutsAt(tp, floor);
    REQUIRE(cuts.size() > 10);
    for (const Vec3& q : cuts) {
        CHECK_THAT(distance(q), WithinAbs(kD / 2.0, 5e-3));
    }
    // Climb milling outside: clockwise seen from outside the part (negative area unrolled).
    double area = 0.0;
    for (std::size_t k = 0; k + 1 < cuts.size(); ++k) {
        const Vec2 a(cuts[k].x(), floor * angleOf(cuts[k]));
        const Vec2 b(cuts[k + 1].x(), floor * angleOf(cuts[k + 1]));
        area += a.x() * b.y() - b.x() * a.y();
    }
    CHECK(area < 0.0);
    // Leads on and off stay clear of the pad.
    bool leads = false;
    for (const toolpath::Move& m : tp.moves) {
        if (m.feedClass == FeedClass::LeadIn || m.feedClass == FeedClass::LeadOut) {
            leads = true;
            CHECK(distance(m.end) >= kD / 2.0 - 5e-3);
        }
    }
    CHECK(leads);
    SECTION("inside, with stock to leave") {
        f.p.side = WrapSide::Left;
        f.p.stockToLeaveXY = 0.5;
        f.p.stockToLeaveZ = 0.25;
        const toolpath::Toolpath in = f.run({pad});
        checkRadial(in, floor + 0.25);
        for (const Vec3& q : cutsAt(in, floor + 0.25)) {
            const Vec2 u(q.x(), (floor + 0.25) * angleOf(q));
            const double inside =
                std::min({u.x() - 10.0, 30.0 - u.x(), u.y() + 0.5 * (floor + 0.25),
                          0.5 * (floor + 0.25) - u.y()});
            CHECK(inside >= kD / 2.0 + 0.5 - 5e-3);
        }
    }
}

TEST_CASE("wrapped contour: a floor in two faces is one slot", "[strategies][multiaxis][wrap]") {
    // A slot 0.8 rad wide around the bottom of the part, its floor split where the angle turns
    // over (as a cylinder's face may come split at its seam): no wall where they meet.
    Fixture f;
    const double floor = kR - 3.0;
    WrappedSlot slot;
    slot.faces = {{box(10.0, 40.0, kPi - 0.4, kPi, floor)},
                  {box(10.0, 40.0, -kPi, -kPi + 0.4, floor)}};
    slot.floorRadius = floor;
    const toolpath::Toolpath tp = f.run({}, {slot});
    checkRadial(tp, floor);
    const auto cuts = cutsAt(tp, floor);
    REQUIRE(cuts.size() > 10);
    double nearest = 1e9, farthest = 0.0;
    for (const Vec3& q : cuts) {
        const double off = std::abs(std::remainder(angleOf(q) - kPi, 2.0 * kPi));
        CHECK(off <= 0.4 - (kD / 2.0) / floor + 1e-3 / floor);
        nearest = std::min(nearest, off);
        farthest = std::max(farthest, off);
    }
    CHECK_THAT(farthest, WithinAbs(0.4 - (kD / 2.0) / floor, 1e-3 / floor));
    CHECK(nearest < (kD / 2.0) / floor - 0.05); // a pass nearer the middle than a wall allows
}

TEST_CASE("wrapped contour: ring grooves all the way around", "[strategies][multiaxis][wrap]") {
    Fixture f;
    const double floor = kR - 2.0;
    const auto around = [&](const std::vector<Vec3>& cuts) {
        double unwrapped = 0.0, lo = 0.0, hi = 0.0;
        for (std::size_t k = 1; k < cuts.size(); ++k) {
            unwrapped += std::remainder(angleOf(cuts[k]) - angleOf(cuts[k - 1]), 2.0 * kPi);
            lo = std::min(lo, unwrapped);
            hi = std::max(hi, unwrapped);
        }
        return hi - lo;
    };
    SECTION("the tool's width: its middle, once around a level") {
        WrappedSlot groove;
        groove.faces = {{ring(20.0, floor), ring(26.0, floor)}};
        groove.floorRadius = floor;
        const toolpath::Toolpath tp = f.run({}, {groove});
        checkRadial(tp, floor);
        const auto cuts = cutsAt(tp, floor);
        REQUIRE(cuts.size() > 10);
        for (const Vec3& q : cuts) {
            CHECK_THAT(q.x(), WithinAbs(23.0, 0.01));
        }
        CHECK_THAT(around(cuts), WithinAbs(2.0 * kPi, 0.01));
    }
    SECTION("wider: both walls, each once around") {
        WrappedSlot groove;
        groove.faces = {{ring(20.0, floor), ring(28.0, floor)}};
        groove.floorRadius = floor;
        const toolpath::Toolpath tp = f.run({}, {groove});
        const auto cuts = cutsAt(tp, floor);
        REQUIRE(cuts.size() > 10);
        bool near = false, far = false;
        for (const Vec3& q : cuts) {
            CHECK(q.x() >= 23.0 - 0.01);
            CHECK(q.x() <= 25.0 + 0.01);
            near = near || std::abs(q.x() - 23.0) < 0.01;
            far = far || std::abs(q.x() - 25.0) < 0.01;
        }
        CHECK(near);
        CHECK(far);
    }
    SECTION("a groove on the line, a circle on the surface") {
        f.p.depth = 2.0;
        const auto cuts = cutsAt(f.run({ring(23.0, kR)}), floor);
        REQUIRE(cuts.size() > 10);
        CHECK_THAT(around(cuts), WithinAbs(2.0 * kPi, 1e-6));
    }
}

TEST_CASE("wrapped contour: entries and refusals", "[strategies][multiaxis][wrap]") {
    Fixture f;
    const double floor = kR - 3.0;
    SECTION("a ramp goes down along the path, no steeper than its angle") {
        f.p.entry = EntryKind::Ramp;
        f.p.rampAngle = 5.0 * kPi / 180.0;
        f.p.depth = 3.0;
        const toolpath::Toolpath tp = f.run({lineAlongX(10.0, 40.0, 0.0, kR)});
        Vec3 from = tp.start;
        bool ramps = false;
        for (const toolpath::Move& m : tp.moves) {
            CHECK(m.feedClass != FeedClass::Plunge);
            if (m.feedClass == FeedClass::Ramp) {
                ramps = true;
                const double down = radiusOf(from) - radiusOf(m.end);
                const double along = std::abs(m.end.x() - from.x());
                CHECK(down <= along * std::tan(f.p.rampAngle) + 1e-9);
                CHECK(m.feedRate == f.h.feeds.ramp);
            }
            from = m.end;
        }
        CHECK(ramps);
        checkRadial(tp, floor);
    }
    SECTION("the feeds are the preset's, as set") {
        f.p.side = WrapSide::Right;
        const toolpath::Toolpath tp = f.run({box(10.0, 30.0, -0.5, 0.5, floor)});
        for (const toolpath::Move& m : tp.moves) {
            if (m.kind == MoveKind::Rapid) {
                continue;
            }
            const double expected = m.feedClass == FeedClass::Cut      ? f.h.feeds.cut
                                    : m.feedClass == FeedClass::Plunge ? f.h.feeds.plunge
                                                                       : f.h.feeds.lead;
            CHECK(m.feedRate == expected);
        }
    }
    SECTION("a curve through the axis is refused") {
        WrappedCurve across;
        across.points = {Vec3(0, -5, 0), Vec3(0, 0, 0), Vec3(0, 5, 0)};
        CHECK_THROWS_WITH(f.run({across}), ContainsSubstring("rotary axis"));
    }
    SECTION("a depth past the axis is refused") {
        f.p.depth = 30.0;
        CHECK_THROWS_WITH(f.run({lineAlongX(10.0, 40.0, 0.0, kR)}), ContainsSubstring("axis"));
    }
    SECTION("a slot wider than one pass needs the preset's stepover") {
        f.p.stepover = 0.0;
        WrappedSlot slot;
        slot.faces = {{box(10.0, 40.0, -10.0 / floor, 10.0 / floor, floor)}};
        slot.floorRadius = floor;
        CHECK_THROWS_WITH(f.run({}, {slot}), ContainsSubstring("radial depth (ae)"));
    }
}
