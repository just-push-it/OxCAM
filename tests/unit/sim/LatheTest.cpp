// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/geom2d/Clip.hpp>
#include <ocam/sim/Lathe.hpp>
#include <ocam/sim/Simulate.hpp>
#include <ocam/sim/Surface.hpp>
#include <ocam/strategies/Turning.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>

using namespace ocam;
using namespace ocam::sim;
using Catch::Approx;
using Catch::Matchers::WithinAbs;
using geom2d::Polyline;
using geom2d::Regions;

namespace {

Regions rect(double z0, double z1, double r0, double r1) {
    return {{Polyline::rectangle(Vec2(z0, r0), Vec2(z1, r1)), {}}};
}

/// Stepped shaft: radius 10 for z in [-30, 0], radius 15 for z in [-80, -30].
Regions steppedShaft() {
    return geom2d::unite(rect(-30, 0, 0, 10), rect(-80, -30, 0, 15));
}

struct Bench {
    tooling::Tool tool;
    CancelToken cancel;
    NullProgress progress;
    strategies::Context context() {
        strategies::Feeds f;
        f.spindle = 1000.0 / 60.0;
        f.cut = 0.2 * f.spindle;
        f.plunge = 0.1 * f.spindle;
        f.lead = f.cut;
        f.partOffEnd = 0.1 * f.spindle;
        return strategies::Context{tool, f, {}, 0.002, cancel, progress};
    }
};

tooling::Tool insert(tooling::ToolType type, int number = 1) {
    tooling::Tool t = tooling::makeDefaultTool(type, number);
    t.insert.noseRadius = 0.8;
    t.insert.shape = "C";
    t.insert.inscribedCircle = 12.7;
    if (type == tooling::ToolType::GroovingInsert || type == tooling::ToolType::PartingInsert) {
        t.insert.width = 3.0;
        t.insert.noseRadius = 0.2;
    }
    if (type == tooling::ToolType::ThreadingInsert) {
        t.insert.threadPitch = 1.5;
    }
    return t;
}

double area(const Regions& r) {
    return geom2d::totalArea(r);
}

/// Part material the stock no longer covers (gouges) and stock outside the part (left over).
std::pair<double, double> deviation(const LatheStock& stock, const Regions& part) {
    const double gouge = area(geom2d::subtract(part, stock.profile(), {1e-4}));
    const double excess = area(geom2d::subtract(stock.profile(), part, {1e-4}));
    return {gouge, excess};
}

void run(LatheStock& stock, const tooling::Tool& tool, const toolpath::Toolpath& tp) {
    const auto side =
        tp.tipQuadrant == 2 ? tooling::TurningSide::Inside : tooling::TurningSide::Outside;
    simulateTurning(stock, tooling::turningShape(tool, side).outline, tp);
}

} // namespace

TEST_CASE("an ISO holder's angles reach the toolpath", "[sim][lathe][turning]") {
    using namespace strategies;
    // A shaft with a shoulder behind a Ø20 neck, front at z = 0.
    const Regions part{{geom2d::Polyline::fromPoints({Vec2(-60, 0), Vec2(-60, 15), Vec2(-30, 15),
                                                      Vec2(-30, 10), Vec2(0, 10), Vec2(0, 0)},
                                                     true),
                        {}}};
    const LatheStock bar = LatheStock::cylinder(-60, 1, 16);
    auto toolIn = [](const char* code) {
        tooling::Tool t = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
        t.insert.noseRadius = 0.4;
        tooling::fitToHolder(t, tooling::makeTurningHolder(*tooling::parseDesignation(code)));
        t.insert.noseRadius = 0.4;
        return t;
    };
    auto generated = [&](const tooling::Tool& t, auto strategy, auto params) {
        CancelToken cancel;
        NullProgress progress;
        const Context ctx{t, {10.0, 5.0, 10.0, 10.0, 1000.0 / 60.0}, {}, 0.01, cancel, progress};
        return strategy(TurnGeometry{part, bar.profile()}, params, ctx);
    };
    SECTION("95 degrees reaches the shoulder") {
        const tooling::Tool t = toolIn("PCLNR 2525M 12");
        TurnFinishParams finish;
        LatheStock stock = bar;
        run(stock, t, generated(t, turnFinish, finish));
        CHECK(deviation(stock, part).first < 0.01);
    }
    SECTION("45 degrees stays off the shoulder and leaves its corner") {
        const tooling::Tool t = toolIn("PSSNR 2525M 12");
        TurnRoughParams rough;
        rough.depthOfCut = 1.0;
        rough.stockToLeave = 0.0;
        LatheStock roughed = bar;
        run(roughed, t, generated(t, turnRough, rough));
        CHECK(deviation(roughed, part).first < 0.01);
        TurnFinishParams finish;
        LatheStock finished = bar;
        run(finished, t, generated(t, turnFinish, finish));
        CHECK(deviation(finished, part).first < 0.01);
        // The 45 degree corner the leading edge cannot reach: about 5 x 5 / 2.
        CHECK(deviation(finished, part).second > 10.0);
        // It cannot face: the side edge would cut into the face.
        CHECK_THROWS_WITH(generated(t, turnFace, TurnFaceParams{}),
                          Catch::Matchers::ContainsSubstring("45.0°"));
    }
    SECTION("a holder for facing does not turn along Z") {
        const tooling::Tool t = toolIn("PSKNR 2525M 12");
        CHECK_NOTHROW(generated(t, turnFace, TurnFaceParams{}));
        CHECK_THROWS_WITH(generated(t, turnFinish, TurnFinishParams{}),
                          Catch::Matchers::ContainsSubstring("end edge"));
    }
}

TEST_CASE("axisymmetric stock volumes and dexel conversion", "[sim][lathe]") {
    LatheStock bar = LatheStock::cylinder(-80, 0, 15);
    CHECK(bar.volume() == Approx(kPi * 225.0 * 80.0).epsilon(1e-9));
    LatheStock tube = LatheStock::cylinder(-50, 0, 20, 8);
    CHECK(tube.volume() == Approx(kPi * (400.0 - 64.0) * 50.0).epsilon(1e-9));
    const Regions shaft = steppedShaft();
    const double expected = kPi * (100.0 * 30.0 + 225.0 * 50.0);
    CHECK(revolvedVolume(shaft) == Approx(expected).epsilon(1e-9));
    const Stock dexel = Stock::fromRevolvedProfile(shaft, 0.1);
    CHECK(dexel.volume() == Approx(expected).epsilon(0.005));
    CHECK(dexel.grid(0).volume() == Approx(expected).epsilon(0.005));
    CHECK(dexel.grid(1).volume() == Approx(expected).epsilon(0.005));
}

TEST_CASE("a turning move cut in parts removes what the whole move does", "[sim][lathe]") {
    // Playback shows a move part by part: each part is the rest of the move from where the
    // previous one got to, cut short.
    const auto outline = tooling::turningShape(insert(tooling::ToolType::TurningInsert)).outline;
    toolpath::Toolpath path;
    path.tipQuadrant = 3;
    path.start = Vec3(14, 0, 2);
    toolpath::Move line;
    line.end = Vec3(14, 0, -30);
    line.feedRate = 1.0;
    toolpath::Move arc = line;
    arc.kind = toolpath::MoveKind::ArcCW;
    arc.plane = toolpath::ArcPlane::ZX;
    arc.center = Vec3(17, 0, -30);
    arc.end = Vec3(17, 0, -33);
    path.moves = {line, arc};
    LatheStock whole = LatheStock::cylinder(-80, 0, 15);
    LatheStock parts = whole;
    simulateTurning(whole, outline, path);
    for (std::size_t k = 0; k < path.moves.size(); ++k) {
        const toolpath::Move& m = path.moves[k];
        double t0 = 0.0;
        for (const double t1 : {0.3, 0.7, 1.0}) {
            toolpath::Toolpath piece;
            piece.start = toolPositionAt(path, k);
            Vec3 axis = toolpath::axisBefore(path, k);
            if (t0 > 0.0) {
                const auto head = toolpath::partialMove(piece.start, axis, m, t0);
                REQUIRE(head);
                piece.start = head->end;
                axis = head->axis;
            }
            const auto rest = toolpath::partialMove(piece.start, axis, m, (t1 - t0) / (1.0 - t0));
            REQUIRE(rest);
            piece.moves.push_back(*rest);
            simulateTurning(parts, outline, piece);
            t0 = t1;
        }
    }
    CHECK(whole.volume() < LatheStock::cylinder(-80, 0, 15).volume() - 1000.0);
    // Up to slivers where the arc is sampled at other points (a few hundredths of a micron).
    CHECK(area(geom2d::subtract(whole.profile(), parts.profile(), {1e-4})) < 1e-2);
    CHECK(area(geom2d::subtract(parts.profile(), whole.profile(), {1e-4})) < 1e-2);
}

TEST_CASE("face, rough and finish turning bring a bar to the part", "[sim][lathe]") {
    const Regions part = steppedShaft();
    const Regions stockProfile = rect(-82, 2, 0, 17);
    Bench b{insert(tooling::ToolType::TurningInsert)};
    const strategies::TurnGeometry g{part, stockProfile};
    LatheStock stock(stockProfile);

    strategies::TurnFaceParams face;
    face.depthOfCut = 1.0;
    const auto faced = strategies::turnFace(g, face, b.context());
    CHECK(faced.warnings.empty());
    run(stock, b.tool, faced);
    // The front is at z = 0 now.
    geom2d::Aabb2 sb;
    for (const auto& r : stock.profile()) {
        sb.extend(r.bounds());
    }
    CHECK_THAT(sb.max.x(), WithinAbs(0.0, 1e-3));

    strategies::TurnRoughParams rough;
    rough.depthOfCut = 2.0;
    rough.stockToLeave = 0.3;
    const auto roughed = strategies::turnRough(g, rough, b.context());
    CHECK(roughed.warnings.empty());
    run(stock, b.tool, roughed);
    auto [gouge, excess] = deviation(stock, part);
    INFO("after roughing: gouge " << gouge << ", excess " << excess);
    CHECK(gouge < 1e-3);
    // The allowance along the profile, plus the 2 mm of bar behind the part (not parted off).
    CHECK(excess < 0.3 * (80.0 + 5.0) + 2.0 * 17.0 + 5.0);

    strategies::TurnFinishParams finish;
    const auto finished = strategies::turnFinish(g, finish, b.context());
    CHECK(finished.warnings.empty());
    run(stock, b.tool, finished);
    std::tie(gouge, excess) = deviation(stock, part);
    INFO("after finishing: gouge " << gouge << ", excess " << excess);
    CHECK(gouge < 1e-3);
    // Only the nose-radius fillet in the inside corner of the shoulder stays: r^2 (1 - pi/4),
    // plus the stock behind the part (not parted off) beyond z = -80.
    const double fillet = 0.8 * 0.8 * (1.0 - kPi / 4.0);
    const double behind = 2.0 * 17.0;
    CHECK(excess < fillet + behind + 0.2);
    CHECK(excess > behind - 2.0); // the nose reaches 0.8 mm behind the part's end
}

TEST_CASE("nose-radius compensation on a taper and a radius", "[sim][lathe]") {
    // A 20 degree taper from r = 6 at the front to r = 12 (6 / tan 20 = 16.48 long), then a
    // cylinder.
    const Regions part = geom2d::unite(
        Regions{{Polyline::fromPoints({Vec2(-16.48, 0), Vec2(0, 0), Vec2(0, 6), Vec2(-16.48, 12)},
                                      true),
                 {}}},
        rect(-60, -16.48, 0, 12));
    const Regions stockProfile = rect(-62, 0, 0, 14);
    Bench b{insert(tooling::ToolType::TurningInsert)};
    b.tool.insert.shape = "D"; // 55 degree: clears the taper's flank
    const strategies::TurnGeometry g{part, stockProfile};
    LatheStock stock(stockProfile);
    strategies::TurnRoughParams rough;
    rough.depthOfCut = 1.5;
    rough.stockToLeave = 0.2;
    run(stock, b.tool, strategies::turnRough(g, rough, b.context()));
    run(stock, b.tool, strategies::turnFinish(g, {}, b.context()));
    const auto [gouge, excess] = deviation(stock, part);
    INFO("gouge " << gouge << ", excess " << excess);
    CHECK(gouge < 1e-3);
    // Behind the part: 2 x 14; on the taper nothing should remain (compensated path).
    CHECK(excess < 2.0 * 14.0 + 0.1);
}

TEST_CASE("in the control, the finish program lies on the taper itself", "[sim][lathe]") {
    // A cone from radius 10 at the front to 15 at Z-10, then a cylinder.
    const Regions part{
        {geom2d::Polyline::fromPoints(
             {Vec2(-40, 0), Vec2(0, 0), Vec2(0, 10), Vec2(-10, 15), Vec2(-40, 15)}, true),
         {}}};
    const Regions stockProfile = rect(-40, 0, 1, 16);
    Bench b{insert(tooling::ToolType::TurningInsert)};
    strategies::TurnFinishParams inControl;
    inControl.inControl = true;
    const auto tp = strategies::turnFinish({part, stockProfile}, inControl, b.context());
    CHECK(tp.compensation == toolpath::Compensation::Right);
    // The sharp profile's corner is on the program; in the computer the nose is offset off it.
    const auto hasCorner = [](const toolpath::Toolpath& path) {
        return std::any_of(path.moves.begin(), path.moves.end(), [](const toolpath::Move& m) {
            return std::abs(m.end.z() + 10.0) < 1e-3 && std::abs(m.end.x() - 15.0) < 1e-3;
        });
    };
    CHECK(hasCorner(tp));
    const auto computed = strategies::turnFinish({part, stockProfile}, {}, b.context());
    CHECK(computed.compensation == toolpath::Compensation::None);
    CHECK_FALSE(hasCorner(computed));
    // Outside mirrored: the side swaps with the direction.
    strategies::TurnFinishParams away = inControl;
    away.awayFromChuck = true;
    away.zRange = strategies::TurnZRange{-5.0, -40.0};
    const auto back = strategies::turnFinish({part, rect(-40, 0, 1, 16)}, away, b.context());
    CHECK(back.compensation == toolpath::Compensation::Left);
    strategies::TurnFinishParams bore = inControl;
    bore.side = tooling::TurningSide::Inside;
    const Regions tube{{geom2d::Polyline::rectangle(Vec2(-40, 5), Vec2(0, 15)), {}}};
    CHECK(strategies::turnFinish({tube, rect(-40, 4, 1, 16)}, bore, b.context()).compensation ==
          toolpath::Compensation::Left);
}

TEST_CASE("grooving clears a recess", "[sim][lathe]") {
    const Regions part = geom2d::subtract(rect(-60, 0, 0, 15), rect(-25, -15, 10, 16));
    const Regions stockProfile = rect(-60, 0, 0, 15);
    Bench b{insert(tooling::ToolType::GroovingInsert)};
    const auto tp = strategies::turnGroove({part, stockProfile}, {}, b.context());
    CHECK(tp.warnings.empty());
    LatheStock stock(stockProfile);
    run(stock, b.tool, tp);
    const auto [gouge, excess] = deviation(stock, part);
    INFO("gouge " << gouge << ", excess " << excess);
    CHECK(gouge < 1e-3);
    CHECK(excess < 0.2); // the insert's small corner radii in the groove corners
    CHECK(stock.volume() == Approx(revolvedVolume(part)).epsilon(1e-3));
}

TEST_CASE("a thread cut beside the profile", "[sim][lathe]") {
    const double lead = 1.5;
    const double depth = 0.92;
    const double radius = 10.0;
    const LatheStock bar = LatheStock::cylinder(-40, 0, radius);
    const tooling::Tool tool = insert(tooling::ToolType::ThreadingInsert);
    const geom2d::Polyline shape = tooling::turningShape(tool).outline;
    const double tip = radius - depth;
    const auto pass = [&](LatheStock& s, double from, double to, double t0 = 0.0, double t1 = 1.0) {
        s.cutThread(shape, Vec2(from, tip), Vec2(to, tip), lead, t0, t1);
    };
    const auto mesh = [](const LatheStock& s) {
        return latheStockMesh(s, 0.005, {}, {});
    };
    const auto meshVolume = [](const DisplayMesh& m) {
        double sum = 0.0;
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
            const auto& a = m.positions[m.indices[t]];
            const auto& b = m.positions[m.indices[t + 1]];
            const auto& c = m.positions[m.indices[t + 2]];
            sum += (static_cast<double>(a[0]) * (b[1] * c[2] - b[2] * c[1]) +
                    static_cast<double>(a[1]) * (b[2] * c[0] - b[0] * c[2]) +
                    static_cast<double>(a[2]) * (b[0] * c[1] - b[1] * c[0])) /
                   6.0;
        }
        return sum;
    };

    LatheStock cut = bar;
    pass(cut, -5.0, -30.0);
    SECTION("the profile stays, the volume goes") {
        CHECK(cut.profile().size() == bar.profile().size());
        CHECK(revolvedVolume(cut.profile()) == Approx(bar.volume()));
        CHECK(cut.volume() < bar.volume());
        // A form's area around the helix, by Pappus: the outline below the surface.
        geom2d::Aabb2 box = shape.bounds();
        (void)box;
        const double removed = bar.volume() - cut.volume();
        const double width = 2.0 * depth * std::tan(degToRad(30.0));
        const double area = 0.5 * width * depth;
        const double pappus = area * 2.0 * kPi * (radius - depth / 3.0) * (25.0 / lead);
        CHECK(removed == Approx(pappus).epsilon(0.2));
    }
    SECTION("drawn, it holds the volume, and is as deep as the pass") {
        const DisplayMesh m = mesh(cut);
        REQUIRE(m.triangleCount() > 1000);
        CHECK(meshVolume(m) == Approx(cut.volume()).epsilon(0.003));
        double lowest = 1e9, highest = 0.0;
        for (const auto& v : m.positions) {
            if (v[2] < -1.0 && v[2] > -39.0) {
                const double r = std::hypot(v[0], v[1]);
                lowest = std::min(lowest, r);
                highest = std::max(highest, r);
            }
        }
        CHECK(lowest == Approx(tip).margin(tool.insert.noseRadius + 0.01));
        CHECK(lowest >= tip - 1e-3);
        CHECK(highest == Approx(radius).margin(1e-3));
    }
    SECTION("the groove follows the helix") {
        // Going towards the chuck with the spindle forward cuts a right-hand thread: along the
        // groove's bottom, z - lead * angle / (2 pi) stays the same, a lead apart.
        const DisplayMesh m = mesh(cut);
        int checked = 0;
        for (const auto& v : m.positions) {
            const double r = std::hypot(v[0], v[1]);
            if (r < tip + 1e-3 && v[2] < -8.0 && v[2] > -27.0) {
                const double angle = std::atan2(v[1], v[0]);
                const double s = (v[2] - lead * angle / (2.0 * kPi) + 5.0) / lead;
                CHECK(std::abs(s - std::round(s)) * lead < 0.3);
                ++checked;
            }
        }
        CHECK(checked > 50);
        CHECK(cut.threads().front().turn > 0.0);
        // The other way round, a left-hand one.
        LatheStock left = bar;
        pass(left, -30.0, -5.0);
        CHECK(left.threads().front().turn < 0.0);
        CHECK(left.volume() == Approx(cut.volume()).epsilon(0.02));
    }
    SECTION("the deepest pass decides, and a shallower one adds nothing") {
        LatheStock two = cut;
        two.cutThread(shape, Vec2(-5.0, tip + 0.4), Vec2(-30.0, tip + 0.4), lead);
        CHECK(two.threads().size() == 2);
        CHECK(two.volume() == Approx(cut.volume()).epsilon(1e-6));
    }
    SECTION("a pass cut part by part is the pass") {
        LatheStock parts = bar;
        pass(parts, -5.0, -30.0, 0.0, 0.25);
        const double quarter = parts.volume();
        CHECK(quarter < bar.volume());
        CHECK(quarter > cut.volume());
        pass(parts, -5.0, -30.0, 0.25, 0.75);
        pass(parts, -5.0, -30.0, 0.75, 1.0);
        CHECK(parts.threads().size() == 1);
        CHECK(parts.volume() == Approx(cut.volume()).epsilon(1e-9));
    }
    SECTION("turning afterwards takes the thread with it") {
        LatheStock turned = cut;
        turned.subtract(rect(-45, 5, 8.0, 20.0));
        CHECK(turned.volume() == Approx(kPi * 64.0 * 40.0).epsilon(1e-3));
    }
    SECTION("a fine pitch is as exact as a coarse one") {
        const double fine = 0.5;
        const double fineDepth = 0.31;
        LatheStock f = bar;
        f.cutThread(shape, Vec2(-5.0, radius - fineDepth), Vec2(-30.0, radius - fineDepth), fine);
        const DisplayMesh m = mesh(f);
        double lowest = 1e9;
        for (const auto& v : m.positions) {
            if (v[2] < -6.0 && v[2] > -29.0) {
                lowest = std::min(lowest, std::hypot(static_cast<double>(v[0]), v[1]));
            }
        }
        CHECK(lowest >= radius - fineDepth - 1e-3);
        CHECK(meshVolume(m) == Approx(f.volume()).epsilon(0.003));
    }
    SECTION("inside a bore") {
        LatheStock tube = LatheStock::cylinder(-40, 0, 20.0, 8.0);
        const tooling::TurningShape inside =
            tooling::turningShape(tool, tooling::TurningSide::Inside);
        tube.cutThread(inside.outline, Vec2(-5.0, 8.8), Vec2(-30.0, 8.8), lead);
        CHECK(tube.volume() < LatheStock::cylinder(-40, 0, 20.0, 8.0).volume());
        const DisplayMesh m = mesh(tube);
        CHECK(meshVolume(m) == Approx(tube.volume()).epsilon(0.003));
        double widest = 0.0;
        for (const auto& v : m.positions) {
            const double r = std::hypot(static_cast<double>(v[0]), v[1]);
            if (r < 12.0) {
                widest = std::max(widest, r);
            }
        }
        CHECK(widest == Approx(8.8).margin(1e-3));
    }
    SECTION("two threads over one surface are one surface") {
        LatheStock crossed = cut;
        pass(crossed, -30.0, -5.0); // a left-hand one over the right-hand one
        const DisplayMesh m = mesh(crossed);
        CHECK(crossed.volume() < cut.volume());
        CHECK(meshVolume(m) == Approx(crossed.volume()).epsilon(0.03));
    }
    SECTION("flipped with its setup it keeps its hand") {
        LatheStock turnedOver(bar.profile());
        Transform3 flip = Transform3::Identity();
        flip.linear() = Eigen::AngleAxisd(kPi, Vec3::UnitX()).toRotationMatrix();
        flip.translation() = Vec3(0, 0, -40);
        turnedOver.takeCuts(cut, flip);
        REQUIRE(turnedOver.threads().size() == 1);
        CHECK(turnedOver.threads().front().turn == cut.threads().front().turn);
        CHECK(turnedOver.threads().front().from.x() == Approx(-35.0));
        CHECK(turnedOver.volume() == Approx(cut.volume()).epsilon(0.01));
    }
}

TEST_CASE("a hole beside the profile", "[sim][lathe]") {
    const double radius = 15.0;
    const LatheStock bar = LatheStock::cylinder(-40, 0, radius);
    const auto meshVolume = [](const DisplayMesh& m) {
        double sum = 0.0;
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
            const auto& a = m.positions[m.indices[t]];
            const auto& b = m.positions[m.indices[t + 1]];
            const auto& c = m.positions[m.indices[t + 2]];
            sum += (static_cast<double>(a[0]) * (b[1] * c[2] - b[2] * c[1]) +
                    static_cast<double>(a[1]) * (b[2] * c[0] - b[0] * c[2]) +
                    static_cast<double>(a[2]) * (b[0] * c[1] - b[1] * c[0])) /
                   6.0;
        }
        return sum;
    };
    const std::vector<Vec2> flat{{0.0, 3.0}, {100.0, 3.0}};
    SECTION("a blind hole in the front face, off the axis") {
        LatheStock s = bar;
        s.cutHole({Vec3(8, 0, -10), Vec3::UnitZ(), 40.0, flat});
        const double expected = bar.volume() - kPi * 9.0 * 10.0;
        CHECK(s.volume() == Approx(expected).epsilon(0.01));
        const DisplayMesh m = latheStockMesh(s, 0.005, {}, {});
        CHECK(meshVolume(m) == Approx(expected).epsilon(0.01));
        // Nothing of the surface is left inside the hole above its bottom.
        for (const auto& v : m.positions) {
            if (v[2] > -9.9f) {
                CHECK(std::hypot(v[0] - 8.0f, v[1]) > 2.99f);
            }
        }
    }
    SECTION("a cross hole, right through") {
        LatheStock s = bar;
        s.cutHole({Vec3(-20, 0, -20), Vec3::UnitX(), 40.0, flat});
        // A cylinder through a cylinder: about the diameter's length of it, a little less.
        const double removed = bar.volume() - s.volume();
        CHECK(removed == Approx(kPi * 9.0 * 30.0).epsilon(0.03));
        const DisplayMesh m = latheStockMesh(s, 0.005, {}, {});
        CHECK(meshVolume(m) == Approx(s.volume()).epsilon(0.01));
    }
    SECTION("the same hole deeper is one hole") {
        LatheStock s = bar;
        s.cutHole({Vec3(8, 0, -5), Vec3::UnitZ(), 35.0, flat});
        s.cutHole({Vec3(8, 0, -10), Vec3::UnitZ(), 40.0, flat});
        s.cutHole({Vec3(8, 0, -7), Vec3::UnitZ(), 37.0, flat});
        REQUIRE(s.holes().size() == 1);
        CHECK(s.holes().front().tip.z() == Approx(-10.0));
    }
    SECTION("no cuts: exactly the revolved profile") {
        const DisplayMesh a = latheStockMesh(bar, 0.005, {}, {});
        const DisplayMesh b = revolvedMesh(bar.profile(), 0.005, ProfileColour{});
        CHECK(a.positions == b.positions);
        CHECK(a.indices == b.indices);
    }
}

TEST_CASE("threading and parting off", "[strategies][lathe]") {
    const Regions part = rect(-40, 0, 0, 10);
    const Regions stockProfile = rect(-50, 0, 0, 10);
    {
        Bench b{insert(tooling::ToolType::ThreadingInsert)};
        strategies::TurnThreadParams p;
        p.length = 20.0;
        p.passes = 5;
        p.springPasses = 1;
        const auto tp = strategies::turnThread({part, stockProfile}, p, b.context());
        CHECK(tp.warnings.empty());
        CHECK(tp.surfaceSpeed == 0.0); // constant rpm for threads
        std::vector<double> radii;
        for (const auto& m : tp.moves) {
            if (m.pitch > 0.0) {
                CHECK(m.pitch == 1.5);
                CHECK_THAT(m.end.z(), WithinAbs(-20.0, 1e-9));
                radii.push_back(m.end.x());
            }
        }
        REQUIRE(radii.size() == 6);
        CHECK_THAT(radii.back(), WithinAbs(10.0 - 0.6134 * 1.5, 1e-9));
        CHECK_THAT(radii[radii.size() - 2], WithinAbs(radii.back(), 1e-12)); // spring pass
        for (std::size_t k = 1; k + 1 < radii.size(); ++k) {
            CHECK(radii[k] < radii[k - 1]);
        }
    }
    {
        Bench b{insert(tooling::ToolType::PartingInsert)};
        const auto tp = strategies::turnPartOff({part, stockProfile}, {}, b.context());
        double lowest = 1e9;
        for (const auto& m : tp.moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                CHECK_THAT(m.end.z(),
                           WithinAbs(-43.0, 1e-9)); // left corner: 3 mm insert behind -40
                lowest = std::min(lowest, m.end.x());
            }
        }
        CHECK_THAT(lowest, WithinAbs(-0.5, 1e-9));
        // The end of the cut runs at its own feed, which is no share of the cutting feed.
        strategies::Context ctx = b.context();
        ctx.feeds.partOffEnd = 0.07 * ctx.feeds.spindle;
        const auto ending = strategies::turnPartOff({part, stockProfile}, {}, ctx);
        std::vector<double> rates;
        for (const auto& m : ending.moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                rates.push_back(m.feedRate);
            }
        }
        REQUIRE(rates.size() == 2);
        CHECK(rates[0] == ctx.feeds.cut);
        CHECK(rates[1] == ctx.feeds.partOffEnd);
        // Without a distance nothing runs at it, so it need not be set.
        ctx.feeds.partOffEnd = 0.0;
        CHECK_THROWS_WITH(strategies::turnPartOff({part, stockProfile}, {}, ctx),
                          Catch::Matchers::ContainsSubstring("the part-off end feed of T1 is 0"));
        strategies::TurnPartOffParams straight;
        straight.slowDistance = 0.0;
        for (const auto& m : strategies::turnPartOff({part, stockProfile}, straight, ctx).moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                CHECK(m.feedRate == ctx.feeds.cut);
            }
        }
        LatheStock stock(stockProfile);
        run(stock, b.tool, tp);
        // The part is separated: nothing joins z > -40 to z < -43 any more.
        const auto pieces = stock.profile();
        CHECK(pieces.size() == 2);
    }
}

TEST_CASE("bodies turning with the spindle: the area they sweep", "[sim][lathe][chuck]") {
    // A jaw: a block from radius 16 to 40 across Y -10..10, Z -90..-50.
    const geom3d::Mesh jaw = boxMesh({Vec3(16, -10, -90), Vec3(40, 10, -50)});
    const Regions area = revolvedEnvelope(jaw, 0.5);
    REQUIRE(area.size() == 1);
    const geom2d::Aabb2 b = area.front().bounds();
    CHECK_THAT(b.min.x(), WithinAbs(-90, 1e-6));
    CHECK_THAT(b.max.x(), WithinAbs(-50, 1e-6));                    // the front, not the next slice
    CHECK_THAT(b.min.y(), WithinAbs(16, 1e-6));                     // its nearest point to the axis
    CHECK_THAT(b.max.y(), WithinAbs(std::hypot(40.0, 10.0), 1e-6)); // its far corners
    CHECK_THAT(area.front().area(), WithinAbs(40 * (std::hypot(40.0, 10.0) - 16), 1e-3));
    // Grown, and a body across the axis reaching it.
    const Regions grown = revolvedEnvelope(jaw, 0.5, 2.0);
    CHECK_THAT(grown.front().bounds().min.y(), WithinAbs(14, 1e-6));
    CHECK_THAT(grown.front().bounds().max.x(), WithinAbs(-48, 1e-6));
    const Regions across = revolvedEnvelope(boxMesh({Vec3(-5, -5, 0), Vec3(5, 5, 10)}), 1.0);
    CHECK_THAT(across.front().bounds().min.y(), WithinAbs(0, 1e-6));
    CHECK_THROWS(revolvedEnvelope(jaw, 0.0));
}

TEST_CASE("turning toolpaths checked against the chuck", "[sim][lathe][chuck]") {
    const std::vector<LatheBody> bodies{
        {"Jaw", revolvedEnvelope(boxMesh({Vec3(16, -10, -90), Vec3(40, 10, -50)}), 0.5)}};
    const Polyline insert = Polyline::rectangle(Vec2(0, 0), Vec2(3, 10)); // behind and above
    toolpath::Toolpath tp;
    tp.start = Vec3(20, 0, 5);
    auto move = [&](toolpath::MoveKind kind, double x, double z) {
        toolpath::Move m;
        m.kind = kind;
        m.end = Vec3(x, 0, z);
        m.feedRate = kind == toolpath::MoveKind::Rapid ? 0.0 : 1.0;
        tp.moves.push_back(m);
    };
    move(toolpath::MoveKind::Rapid, 14, 2);
    move(toolpath::MoveKind::Linear, 14, -45); // stops in front of the jaw
    move(toolpath::MoveKind::Rapid, 20, -45);
    move(toolpath::MoveKind::Rapid, 20, 5);
    CHECK(checkTurningFixtures(tp, insert, bodies).empty());
    // Running on past the jaw front: the insert (reaching 10 up from radius 14) hits it; the
    // moves after it, still in it, are the same collision.
    tp.moves[1].end.z() = -60;
    tp.moves[2].end.z() = -60;
    const auto hits = checkTurningFixtures(tp, insert, bodies);
    REQUIRE(hits.size() == 1);
    CHECK(hits.front().body == "Jaw");
    CHECK(hits.front().toolPart == "insert");
    CHECK(hits.front().firstMove == 1);
    CHECK(hits.front().lastMove == 3);
    CHECK(hits.front().position.z() < -50 + 1e-6);
}

TEST_CASE("a turning insert's rapids against the stock", "[sim][lathe]") {
    const geom2d::Polyline insert = geom2d::Polyline::rectangle(Vec2(0.0, -0.5), Vec2(1.0, 0.0));
    toolpath::Toolpath path;
    path.start = Vec3(30.0, 0.0, 30.0);
    const auto rapidTo = [&](const Vec3& end) {
        toolpath::Move move;
        move.kind = toolpath::MoveKind::Rapid;
        move.feedClass = toolpath::FeedClass::Rapid;
        move.end = end;
        path.moves.push_back(move);
    };
    LatheStock stock = LatheStock::cylinder(-40.0, 0.0, 10.0);
    std::vector<Collision> hits;
    SECTION("clear of the bar") {
        rapidTo(Vec3(30.0, 0.0, -30.0));
        rapidTo(Vec3(10.5, 0.0, -30.0));
        simulateTurning(stock, insert, path, 0, 2, nullptr, &hits);
        CHECK(hits.empty());
    }
    SECTION("through the bar, reported at the move") {
        rapidTo(Vec3(5.0, 0.0, 10.0));
        rapidTo(Vec3(5.0, 0.0, -30.0));
        rapidTo(Vec3(30.0, 0.0, -30.0));
        simulateTurning(stock, insert, path, 0, 3, nullptr, &hits);
        REQUIRE(hits.size() == 1);
        CHECK(hits[0].kind == CollisionKind::Rapid);
        CHECK(hits[0].firstMove == 1);
    }
    SECTION("resting on a wall it has just cut is not one") {
        toolpath::Move cut;
        cut.kind = toolpath::MoveKind::Linear;
        cut.end = Vec3(5.0, 0.0, -20.0);
        cut.feedRate = 1.0;
        rapidTo(Vec3(10.5, 0.0, -20.0));
        path.moves.push_back(cut);
        rapidTo(Vec3(30.0, 0.0, -20.0));
        simulateTurning(stock, insert, path, 0, 3, nullptr, &hits);
        CHECK(hits.empty());
    }
}

TEST_CASE("a turning tool's body against the stock and across the axis", "[sim][lathe]") {
    // A bar's body behind the tip along +Z, 10 wide, its axis 6 below the tip: in a bore of radius
    // 5 it reaches 11 - 5 past the axis (r = -6.. +4 at a tip radius of 5 - 1, say).
    const geom2d::Polyline insert = geom2d::Polyline::rectangle(Vec2(0.0, -0.5), Vec2(1.0, 0.0));
    const std::vector<geom2d::Polyline> body{
        geom2d::Polyline::rectangle(Vec2(2.0, -11.0), Vec2(40.0, -1.0))};
    toolpath::Toolpath path;
    path.start = Vec3(4.0, 0.0, 30.0);
    toolpath::Move move;
    move.kind = toolpath::MoveKind::Linear;
    move.end = Vec3(4.0, 0.0, 0.0);
    move.feedRate = 1.0;
    path.moves.push_back(move);
    SECTION("the far side meets the wall across the axis") {
        const auto hits =
            checkTurningBody(path, insert, body, LatheStock::cylinder(-10.0, 20.0, 30.0, 5.0));
        REQUIRE(hits.size() == 1);
        CHECK(hits[0].kind == CollisionKind::Holder);
        // The same with the bore wide enough for it.
        CHECK(checkTurningBody(path, insert, body, LatheStock::cylinder(-10.0, 20.0, 30.0, 7.5))
                  .empty());
    }
    SECTION("the chuck too, only when asked to look across the axis") {
        const std::vector<LatheBody> chuck{
            {"jaw", {{geom2d::Polyline::rectangle(Vec2(10.0, 6.0), Vec2(12.0, 9.0)), {}}}}};
        CHECK(checkTurningFixtures(path, body[0], chuck).empty());
        const auto hits = checkTurningFixtures(path, body[0], chuck, 200, nullptr, true);
        REQUIRE(hits.size() == 1);
        CHECK(hits[0].body == "jaw");
    }
}

TEST_CASE("a bar pulled out of the chuck (DL-0091)", "[sim][lathe][barpull]") {
    // A Ø40 bar from z -60 to 0, a part parted off in front of it (z 3 to 30).
    LatheStock stock(geom2d::unite(LatheStock::cylinder(-60.0, 0.0, 20.0).profile(),
                                   LatheStock::cylinder(3.0, 30.0, 15.0).profile()));
    REQUIRE(stock.profile().size() == 2);
    SECTION("the held bar is what reaches the back end") {
        const Regions held = stock.heldBar();
        REQUIRE(held.size() == 1);
        CHECK(held[0].bounds().max.x() == Approx(0.0).margin(1e-9));
    }
    SECTION("pulled: it moves out, goes on behind as it was, and the part that came off goes") {
        const LatheStock after = stock.pulled(33.0);
        REQUIRE(after.profile().size() == 1);
        const auto box = after.profile()[0].bounds();
        CHECK(box.min.x() == Approx(-60.0).margin(1e-6)); // still in the chuck at its back
        CHECK(box.max.x() == Approx(33.0).margin(1e-6));
        CHECK(box.max.y() == Approx(20.0).margin(1e-6));
        CHECK(after.volume() == Approx(kPi * 400.0 * 93.0).epsilon(1e-6));
    }
    SECTION("a tube stays a tube behind the pull") {
        const LatheStock tube = LatheStock::cylinder(-50.0, 0.0, 20.0, 8.0);
        const LatheStock after = tube.pulled(10.0);
        CHECK(after.volume() == Approx(kPi * (400.0 - 64.0) * 60.0).epsilon(1e-6));
    }
    SECTION("how far a pull has moved it, move by move") {
        strategies::BarPullParams p;
        p.pullLength = 33.0;
        p.grip = 10.0;
        p.clearance = 2.0;
        tooling::Tool puller = tooling::makeDefaultTool(tooling::ToolType::BarPuller, 7, 40.0);
        CancelToken cancel;
        NullProgress progress;
        strategies::Feeds feeds;
        feeds.plunge = 5.0;
        feeds.pull = 20.0;
        const strategies::Context ctx{puller, feeds, {}, 0.002, cancel, progress};
        const toolpath::Toolpath tp = strategies::barPull({0.0, 20.0}, p, ctx);
        std::size_t pull = 0;
        for (std::size_t k = 0; k < tp.moves.size(); ++k) {
            if (tp.moves[k].feedRate == 20.0) {
                pull = k;
            }
        }
        REQUIRE(pull > 0);
        CHECK(barPulledBy(tp, pull) == Approx(0.0));
        CHECK(barPulledBy(tp, pull, 0.5) == Approx(16.5));
        CHECK(barPulledBy(tp, pull + 1) == Approx(33.0));
        CHECK(barPulledBy(tp, tp.moves.size()) == Approx(33.0)); // the chuck closed: no more
    }
}
