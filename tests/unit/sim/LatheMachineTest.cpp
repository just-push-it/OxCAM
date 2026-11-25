// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The lathe's machine check on made-up moves (DL-0067): each kind of collision at the move it
// happens, and a clean job with none.

#include <ocam/foundation/Units.hpp>
#include <ocam/geom2d/Polyline.hpp>
#include <ocam/machine/Turret.hpp>
#include <ocam/sim/LatheMachine.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

using namespace ocam;
using namespace ocam::sim;
using machine::AxisPosition;
using machine::LatheStep;

namespace {

/// A plain shank across the spindle: X up to 100 from the tip, 25 across, 25 along Z.
TurretTool shankTool(const machine::Machine& m, int station, double reach, double along) {
    TurretTool t;
    t.station = station;
    t.name = "T" + std::to_string(station);
    t.placement = Transform3::Identity();
    t.placement.translation() = machine::tipPoint(*m.lathe, Vec2(reach, along));
    t.solids.push_back(
        {"holder",
         platePrism({{-12.5, 0.0}, {12.5, 0.0}, {12.5, 100.0}, {-12.5, 100.0}}, -25.0, 0.0)});
    return t;
}

/// A bar along Z, sticking out towards the chuck from its tip back into the turret.
TurretTool barTool(const machine::Machine& m, int station, double reach, double along) {
    TurretTool t;
    t.station = station;
    t.name = "T" + std::to_string(station);
    t.placement = Transform3::Identity();
    t.placement.translation() = machine::tipPoint(*m.lathe, Vec2(reach, along));
    t.solids.push_back({"holder", roundPrism(Vec3::Zero(), Vec3::UnitZ(), 12.0, 200.0)});
    return t;
}

constexpr double kFront = 200.0; // where the WCS's Z 0 is on the machine
constexpr double kAlong = -60.0; // a tool across the spindle's tip from the turret's centre

machine::WorkMount spindleMount() {
    machine::WorkMount mount;
    mount.carrier = "C";
    mount.wcs = Eigen::Translation3d(0.0, 0.0, kFront) * Transform3::Identity();
    return mount;
}

/// A bar of 40 mm, 100 mm long, its front at the WCS's Z 0.
geom2d::Regions barArea(double radius = 20.0) {
    return {{geom2d::Polyline::rectangle(Vec2(-100.0, 0.0), Vec2(0.0, radius)), {}}};
}

toolpath::Toolpath tenMillimetreMoves(const Vec3& from, const Vec3& to) {
    toolpath::Toolpath path;
    path.start = from;
    const int n = static_cast<int>(std::ceil((to - from).norm() / 10.0));
    for (int k = 1; k <= n; ++k) {
        toolpath::Move m;
        m.kind = toolpath::MoveKind::Linear;
        m.end = from + (to - from) * (static_cast<double>(k) / n);
        path.moves.push_back(m);
    }
    return path;
}

/// Station `station` working with its tip at `tip` (machine zero), nothing before the first move.
machine::LatheMotion working(const machine::Machine& m, int station, const Vec2& tip) {
    machine::LatheMotion motion;
    motion.station = station;
    motion.tip = machine::tipPoint(*m.lathe, tip);
    motion.held.t = machine::stationAngle(m, station);
    motion.start = motion.held;
    return motion;
}

LatheStockTrack stockOf(const geom2d::Regions& area) {
    LatheStockTrack track;
    track.start = area;
    return track;
}

bool onlyBetween(const std::vector<MachineCollision>& hits, const std::string& during) {
    return std::all_of(hits.begin(), hits.end(),
                       [&](const MachineCollision& c) { return c.during == during; });
}

} // namespace

TEST_CASE("a clean job on the turret reports nothing", "[sim][lathe][machine]") {
    const machine::Machine& m = machine::genericLatheTurret();
    const LatheMachineCheck check(m, spindleMount(),
                                  {shankTool(m, 1, 190.0, kAlong), shankTool(m, 2, 190.0, kAlong),
                                   shankTool(m, 7, 190.0, kAlong), barTool(m, 4, 110.0, -120.0)},
                                  {}, {}, true);
    const auto hits =
        check.check(working(m, 1, Vec2(190.0, kAlong)),
                    tenMillimetreMoves(Vec3(30, 0, 10), Vec3(30, 0, -60)), stockOf(barArea()));
    CHECK(hits.empty());
}

TEST_CASE("a tool not cutting meets the chuck", "[sim][lathe][machine]") {
    const machine::Machine& m = machine::genericLatheTurret();
    // Station 2 stands 30 degrees round, nearer the chuck than the tool cutting: its tip at Y 95,
    // close enough to the spindle's line for a chuck of radius 105 when the tool cutting is at
    // radius 10.
    const double deeper = kAlong - 60.0;
    const LatheMachineCheck check(m, spindleMount(),
                                  {shankTool(m, 1, 190.0, kAlong), shankTool(m, 2, 190.0, deeper)},
                                  {}, {}, true);
    const auto path = tenMillimetreMoves(Vec3(10, 0, 60), Vec3(10, 0, -60));
    const auto hits = check.check(working(m, 1, Vec2(190.0, kAlong)), path, stockOf(barArea(5.0)));
    REQUIRE(hits.size() == 1);
    CHECK(hits[0].first == "T2, holder");
    CHECK(hits[0].second == "Chuck");
    CHECK(hits[0].during.empty());
    // The shank's near side reaches the chuck's front (Z 90, plus the clearance) when station 2's
    // tip is at 103.5: the tool cutting at the machine's Z 163.5 and the WCS's Z -36.5.
    const toolpath::Move& first = path.moves[hits[0].firstMove];
    CHECK(first.end.z() <= -36.5 + 2.0 + 1e-9);
    CHECK(first.end.z() >= -36.5 - 10.0 - 2.0 - 1e-9);
    CHECK(hits[0].lastMove + 1 == path.moves.size());
}

TEST_CASE("the tool cutting is left to the turning checks at the stock", "[sim][lathe][machine]") {
    const machine::Machine& m = machine::genericLatheTurret();
    const LatheMachineCheck check(m, spindleMount(), {shankTool(m, 1, 190.0, kAlong)}, {}, {},
                                  true);
    const auto path = tenMillimetreMoves(Vec3(30, 0, 10), Vec3(10, 0, 10));
    const auto hits = check.check(working(m, 1, Vec2(190.0, kAlong)), path, stockOf(barArea()));
    CHECK(hits.empty());
}

TEST_CASE("the turret and a tool meet the enclosure", "[sim][lathe][machine]") {
    const machine::Machine& m = machine::genericLatheTurret();
    // Going out along +X: the turret's centre at radius + 190, its back at 150 more, and the wall
    // at X 700.
    const auto path = tenMillimetreMoves(Vec3(100, 0, 10), Vec3(400, 0, 10));
    SECTION("the disc") {
        const LatheMachineCheck check(m, spindleMount(), {shankTool(m, 1, 190.0, kAlong)}, {}, {},
                                      true);
        const auto hits = check.check(working(m, 1, Vec2(190.0, kAlong)), path, stockOf(barArea()));
        REQUIRE(hits.size() == 1);
        CHECK(hits[0].first == "Turret");
        CHECK(hits[0].second == "Enclosure, back");
        const toolpath::Move& first = path.moves[hits[0].firstMove];
        CHECK(first.end.x() >= 359.0 - 1e-9);
        CHECK(first.end.x() <= 359.0 + 10.0 + 1e-9);
        CHECK(hits[0].position.x() > 290.0);
    }
    SECTION("the tool standing opposite goes first") {
        const LatheMachineCheck check(
            m, spindleMount(), {shankTool(m, 1, 190.0, kAlong), shankTool(m, 7, 190.0, kAlong)}, {},
            {}, true);
        const auto hits = check.check(working(m, 1, Vec2(190.0, kAlong)), path, stockOf(barArea()));
        REQUIRE_FALSE(hits.empty());
        CHECK(hits[0].first == "T7, holder");
        CHECK(hits[0].second == "Enclosure, back");
        // Its end comes round to centre + 190: the wall at radius 319.
        const toolpath::Move& first = path.moves[hits[0].firstMove];
        CHECK(first.end.x() >= 319.0 - 1e-9);
        CHECK(first.end.x() <= 319.0 + 10.0 + 1e-9);
    }
}

TEST_CASE("an index move swings a tool through the work", "[sim][lathe][machine]") {
    const machine::Machine& m = machine::genericLatheTurret();
    const auto path = tenMillimetreMoves(Vec3(22, 0, 20), Vec3(30, 0, 20));
    // Station 1 cutting at radius 22 of a bar of radius 20; station 7 sticks out 40 mm further, so
    // coming round in place it goes through the bar.
    const std::vector<TurretTool> tools{shankTool(m, 1, 190.0, kAlong),
                                        shankTool(m, 7, 230.0, kAlong)};
    machine::LatheMotion motion = working(m, 7, Vec2(230.0, kAlong));
    motion.start = AxisPosition{};
    motion.start.t = machine::stationAngle(m, 1);
    // Where station 1 stands at radius 22, level with the bar.
    motion.start.linear = Vec3(22.0 + 190.0 - 420.0, 0.0, kFront - 50.0 - (760.0 + kAlong));
    AxisPosition to = motion.start;
    to.t = machine::indexTo(m, motion.start.t, 7);
    motion.steps.push_back({LatheStep::Kind::Index, to, 1.0, false});
    motion.held = to;
    motion.held.linear = Vec3::Zero();
    SECTION("reported as the index, before the first move") {
        const LatheMachineCheck check(m, spindleMount(), tools, {}, {}, true);
        // After the index the toolpath stands clear of the bar.
        const auto hits = check.check(motion, tenMillimetreMoves(Vec3(70, 0, 20), Vec3(80, 0, 20)),
                                      stockOf(barArea()));
        REQUIRE_FALSE(hits.empty());
        CHECK(hits[0].first == "T7, holder");
        CHECK(hits[0].second == "stock");
        CHECK(hits[0].during == std::string(machine::toString(LatheStep::Kind::Index)));
        CHECK(hits[0].firstMove == 0);
        CHECK(onlyBetween(hits, hits[0].during));
    }
    SECTION("a part is told from the stock") {
        const geom2d::Regions part = barArea(18.0);
        const LatheMachineCheck check(m, spindleMount(), tools, {}, part, true);
        const auto hits = check.check(motion, tenMillimetreMoves(Vec3(70, 0, 20), Vec3(80, 0, 20)),
                                      stockOf(barArea()));
        REQUIRE_FALSE(hits.empty());
        CHECK(std::any_of(hits.begin(), hits.end(),
                          [](const MachineCollision& c) { return c.second == "part"; }));
    }
    SECTION("standing back from the bar, it goes clear") {
        motion.start.linear.x() += 60.0;
        motion.steps[0].to.linear.x() += 60.0;
        const LatheMachineCheck check(m, spindleMount(), tools, {}, {}, true);
        const auto hits = check.check(motion, tenMillimetreMoves(Vec3(70, 0, 20), Vec3(80, 0, 20)),
                                      stockOf(barArea()));
        CHECK(hits.empty());
    }
}

TEST_CASE("the turret meets the tailstock", "[sim][lathe][machine]") {
    const machine::Machine& m = machine::genericLatheTurret();
    // The tailstock's body in front at the machine's Z 400: W -360 takes it from 760.
    machine::LatheMotion motion = working(m, 1, Vec2(190.0, kAlong));
    motion.held.w = -360.0;
    motion.start.w = -360.0;
    const std::vector<TurretTool> tools{shankTool(m, 1, 190.0, kAlong)};
    SECTION("going towards it along Z") {
        // At radius 5 the disc's edge is 45 from the centre line, inside the body's radius of 55.
        const auto path = tenMillimetreMoves(Vec3(5, 0, 60), Vec3(5, 0, 200));
        const LatheMachineCheck check(m, spindleMount(), tools, {}, {}, true);
        const auto hits = check.check(motion, path, stockOf(barArea()));
        const auto disc = std::find_if(hits.begin(), hits.end(), [](const MachineCollision& c) {
            return c.first == "Turret";
        });
        REQUIRE(disc != hits.end());
        CHECK(disc->second == "Tailstock body");
        // The disc's back, 110 behind the tip, reaches the body at the machine's Z 400.
        const toolpath::Move& first = path.moves[disc->firstMove];
        CHECK(first.end.z() + kFront + 110.0 >= 399.0 - 1e-9);
        CHECK(first.end.z() + kFront + 110.0 <= 399.0 + 10.0 + 1e-9);
    }
    SECTION("coming in on a turret parked in its way") {
        // Parked at radius 5 at the machine's Z 500: nothing touches until W brings the body in.
        machine::LatheMotion in = working(m, 1, Vec2(190.0, kAlong));
        in.start.linear = Vec3(5.0 + 190.0 - 420.0, 0.0, 300.0 - (760.0 + kAlong));
        AxisPosition to = in.start;
        to.w = -700.0;
        in.steps.push_back({LatheStep::Kind::Tailstock, to, 1.0, false});
        in.held = to;
        in.held.linear = Vec3::Zero();
        const auto path = tenMillimetreMoves(Vec3(5, 0, 100), Vec3(5, 0, 90));
        const LatheMachineCheck check(m, spindleMount(), tools, {}, {}, true);
        const auto hits = check.check(in, path, stockOf(barArea()));
        REQUIRE_FALSE(hits.empty());
        CHECK(hits[0].during == std::string(machine::toString(LatheStep::Kind::Tailstock)));
        CHECK(hits[0].first == "Turret");
        CHECK(hits[0].second == "Tailstock body");
    }
    SECTION("its centre, as the setup has it, against a tool not cutting") {
        // A quill on W, in the WCS plane, standing at Z 120 to 200 of the WCS at W -360.
        LatheWorkBody quill;
        quill.name = "Tailstock quill";
        quill.area = {{geom2d::Polyline::rectangle(Vec2(120.0, 0.0), Vec2(200.0, 110.0)), {}}};
        quill.carrier = "W";
        quill.frame = Eigen::Translation3d(0.0, 0.0, 360.0) * spindleMount().wcs;
        const std::vector<TurretTool> two{shankTool(m, 1, 190.0, kAlong),
                                          shankTool(m, 2, 190.0, kAlong + 100.0)};
        const auto path = tenMillimetreMoves(Vec3(25, 0, 0), Vec3(25, 0, 60));
        const LatheMachineCheck check(m, spindleMount(), two, {quill}, {}, true);
        const auto hits = check.check(motion, path, stockOf(barArea()));
        REQUIRE_FALSE(hits.empty());
        CHECK(hits[0].first == "T2, holder");
        CHECK(hits[0].second == "Tailstock quill");
    }
}

TEST_CASE("the sub-spindle comes in", "[sim][lathe][machine]") {
    const machine::Machine& m = machine::genericLatheSub();
    machine::LatheMotion motion = working(m, 1, Vec2(190.0, kAlong));
    motion.start.linear = Vec3(-50.0, 0.0, 0.0);
    AxisPosition in = motion.start;
    const std::vector<TurretTool> tools{shankTool(m, 1, 190.0, kAlong)};
    const auto path = tenMillimetreMoves(Vec3(200, 0, 200), Vec3(200, 0, 210));
    SECTION("short of the main spindle's chuck it is clear") {
        in.z2 = -800.0; // its chuck's front at 205, the main chuck's at 90
        motion.steps.push_back({LatheStep::Kind::Transfer, in, 1.0, true});
        motion.held = in;
        motion.held.linear = Vec3::Zero();
        const LatheMachineCheck check(m, spindleMount(), tools, {}, {}, true);
        const auto hits = check.check(motion, path, stockOf({}));
        CHECK(hits.empty());
    }
    SECTION("too far, its chuck meets the main one") {
        in.z2 = -950.0; // its chuck's front at 55
        motion.steps.push_back({LatheStep::Kind::Transfer, in, 1.0, true});
        motion.held = in;
        motion.held.linear = Vec3::Zero();
        const LatheMachineCheck check(m, spindleMount(), tools, {}, {}, true);
        const auto hits = check.check(motion, path, stockOf({}));
        REQUIRE_FALSE(hits.empty());
        CHECK(hits[0].first == "Sub-spindle chuck");
        CHECK(hits[0].second == "Chuck");
        CHECK(hits[0].during == std::string(machine::toString(LatheStep::Kind::Transfer)));
    }
}

TEST_CASE("material already gone is not in the way", "[sim][lathe][machine]") {
    const machine::Machine& m = machine::genericLatheTurret();
    // A long bar along Z in station 2's place, level with the work at the turret's 30 degrees.
    const double deeper = kAlong - 40.0;
    const std::vector<TurretTool> tools{shankTool(m, 1, 190.0, kAlong),
                                        shankTool(m, 2, 190.0, deeper)};
    // A flange of radius 110 at the machine's Z 170 to 190: station 2 meets it, unless cut away.
    const geom2d::Regions flange{
        {geom2d::Polyline::rectangle(Vec2(-30.0, 0.0), Vec2(-10.0, 110.0)), {}}};
    const auto path = tenMillimetreMoves(Vec3(30, 0, 80), Vec3(30, 0, 40));
    const LatheMachineCheck check(m, spindleMount(), tools, {}, {}, true);
    SECTION("it is there") {
        const auto hits = check.check(working(m, 1, Vec2(190.0, kAlong)), path, stockOf(flange));
        REQUIRE_FALSE(hits.empty());
        CHECK(hits[0].first == "T2, holder");
        CHECK(hits[0].second == "stock");
    }
    SECTION("it has been cut away by then") {
        LatheStockTrack track;
        track.start = flange;
        const geom2d::Regions gone{
            {geom2d::Polyline::rectangle(Vec2(-30.0, 0.0), Vec2(-10.0, 5.0)), {}}};
        track.at = [&](std::size_t) -> const geom2d::Regions& {
            return gone;
        };
        const auto hits = check.check(working(m, 1, Vec2(190.0, kAlong)), path, track);
        CHECK(hits.empty());
    }
}

TEST_CASE("a machine of meshes, as parts from its CAD are", "[sim][lathe][machine]") {
    // The layout's disc, wall and chuck given up for mesh parts carried by the same axes.
    machine::Machine m = machine::genericLatheTurret();
    machine::MachineModel model = machine::effectiveModel(m);
    const auto meshOf = [](const machine::Component& c) {
        return std::make_shared<const geom3d::Mesh>(machine::componentMesh(c));
    };
    const std::vector<machine::LatheSolid> solids = machine::latheSolids(m);
    for (const char* name : {"Turret", "Chuck", "Enclosure, back"}) {
        const auto solid =
            std::find_if(solids.begin(), solids.end(),
                         [&](const machine::LatheSolid& s) { return s.shape.name == name; });
        REQUIRE(solid != solids.end());
        machine::ModelPart part;
        part.name = std::string(name) + " from CAD";
        part.carrier = solid->carrier;
        part.mesh = "made-up";
        part.geometry = meshOf(solid->shape);
        model.parts.push_back(std::move(part));
    }
    m.model = model;
    m.lathe->turret.diameter = 0.0;
    m.lathe->chuckDiameter = 0.0;
    m.lathe->enclosureMax = Vec3::Zero();
    m.lathe->enclosureMin = Vec3::Zero();
    REQUIRE(m.validate().empty());
    SECTION("the disc against the wall") {
        const LatheMachineCheck check(m, spindleMount(), {shankTool(m, 1, 190.0, kAlong)}, {}, {},
                                      true);
        const auto path = tenMillimetreMoves(Vec3(100, 0, 10), Vec3(400, 0, 10));
        const auto hits = check.check(working(m, 1, Vec2(190.0, kAlong)), path, stockOf(barArea()));
        REQUIRE_FALSE(hits.empty());
        CHECK(hits[0].first == "Turret from CAD");
        CHECK(hits[0].second == "Enclosure, back from CAD");
        const toolpath::Move& first = path.moves[hits[0].firstMove];
        CHECK(first.end.x() >= 359.0 - 1e-9);
        CHECK(first.end.x() <= 359.0 + 10.0 + 1e-9);
    }
    SECTION("the chuck on C is checked by what it sweeps") {
        const double deeper = kAlong - 60.0;
        const LatheMachineCheck check(
            m, spindleMount(), {shankTool(m, 1, 190.0, kAlong), shankTool(m, 2, 190.0, deeper)}, {},
            {}, true);
        const auto path = tenMillimetreMoves(Vec3(10, 0, 60), Vec3(10, 0, -60));
        const auto hits =
            check.check(working(m, 1, Vec2(190.0, kAlong)), path, stockOf(barArea(5.0)));
        REQUIRE_FALSE(hits.empty());
        CHECK(hits[0].first == "T2, holder");
        CHECK(hits[0].second == "Chuck from CAD");
    }
    SECTION("the disc meets a stock by its triangles") {
        const LatheMachineCheck check(m, spindleMount(), {shankTool(m, 1, 190.0, kAlong)}, {}, {},
                                      true);
        // Coming down to where the disc's edge, radius + 40, is on a bar of radius 100.
        const auto path = tenMillimetreMoves(Vec3(120, 0, -60), Vec3(30, 0, -60));
        const auto hits =
            check.check(working(m, 1, Vec2(190.0, kAlong)), path, stockOf(barArea(100.0)));
        REQUIRE_FALSE(hits.empty());
        CHECK(hits[0].first == "Turret from CAD");
        CHECK(hits[0].second == "stock");
    }
}

TEST_CASE("arriving at the first position, the tool is the one about to cut",
          "[sim][lathe][machine]") {
    const machine::Machine& m = machine::genericLatheTurret();
    machine::LatheMotion motion = working(m, 1, Vec2(190.0, kAlong));
    // In along Z, to half a millimetre off the bar's front: nearer than the clearance.
    const Vec3 tipAtZero = motion.tip;
    motion.start.linear = Vec3(10.0 - tipAtZero.x(), 0.0, kFront + 50.0 - tipAtZero.z());
    AxisPosition to = motion.start;
    to.linear.z() = kFront + 0.5 - tipAtZero.z();
    motion.steps.push_back({LatheStep::Kind::Rapid, to, 1.0, false});
    // All of the tool on this side of its tip.
    TurretTool tool = shankTool(m, 1, 190.0, kAlong);
    tool.solids[0].prism =
        platePrism({{0.0, 0.0}, {25.0, 0.0}, {25.0, 100.0}, {0.0, 100.0}}, -25.0, 0.0);
    const LatheMachineCheck check(m, spindleMount(), {tool}, {}, {}, true);
    SECTION("stopping there is no collision") {
        const auto path = tenMillimetreMoves(Vec3(10, 0, 0.5), Vec3(30, 0, 0.5));
        CHECK(check.check(motion, path, stockOf(barArea())).empty());
    }
    SECTION("going in through the bar on the way there is") {
        motion.steps[0].to.linear.z() = kFront - 40.0 - tipAtZero.z();
        const auto path = tenMillimetreMoves(Vec3(10, 0, -40), Vec3(30, 0, -40));
        const auto hits = check.check(motion, path, stockOf(barArea()));
        REQUIRE_FALSE(hits.empty());
        CHECK(hits[0].during == std::string(machine::toString(LatheStep::Kind::Rapid)));
    }
}

TEST_CASE("a prism's mesh is closed and faces out", "[sim][lathe][machine]") {
    const geom3d::Mesh plate = prismMesh(platePrism({{0, 0}, {10, 0}, {10, 5}, {0, 5}}, -2.0, 2.0));
    CHECK(plate.signedVolume() == Catch::Approx(200.0));
    const geom3d::Mesh bar = prismMesh(roundPrism(Vec3::Zero(), Vec3::UnitZ(), 10.0, 100.0));
    CHECK(bar.signedVolume() > kPi * 100.0 * 100.0);
    CHECK(bar.signedVolume() < 1.02 * kPi * 100.0 * 100.0);
}
