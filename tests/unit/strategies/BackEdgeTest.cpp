// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Back boring and back chamfering, and 3D edge breaking (DL-0083).

#include "StrategyTestUtil.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/strategies/BackBore.hpp>
#include <ocam/strategies/EdgeBreak.hpp>
#include <ocam/toolpath/Toolpath.hpp>

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
using toolpath::CycleType;
using toolpath::FeedClass;
using toolpath::MoveKind;

namespace {

/// A back-boring bar: a Ø13 head 3 mm tall on a Ø5 neck 40 mm long.
tooling::Tool backBar(double tipAngle = 0.0) {
    tooling::Tool t = tooling::makeDefaultTool(tooling::ToolType::BoringBar, 1, 13.0);
    t.mill.diameter = 13.0;
    t.mill.fluteLength = 3.0;
    t.mill.neckDiameter = 5.0;
    t.mill.neckLength = 40.0;
    t.mill.overallLength = 80.0;
    t.mill.shankDiameter = 0.0;
    t.mill.tipAngle = tipAngle;
    t.mill.flutes = 1;
    REQUIRE(t.validate().empty());
    return t;
}

tooling::Tool lollipop() {
    tooling::Tool t = tooling::makeDefaultTool(tooling::ToolType::Lollipop, 2, 6.0);
    t.mill.diameter = 6.0;
    t.mill.neckDiameter = 3.0;
    t.mill.neckLength = 35.0;
    t.mill.fluteLength = 6.0;
    t.mill.overallLength = 70.0;
    t.mill.shankDiameter = 6.0;
    REQUIRE(t.validate().empty());
    return t;
}

/// A Ø10 through hole at (20, 20) in a plate from z = -20 to 0.
HoleTarget throughHole() {
    return {{20.0, 20.0}, 0.0, 20.0, true, 10.0};
}

/// The moves of a toolpath with its cycles played out.
std::vector<toolpath::Move> expanded(const toolpath::Toolpath& tp) {
    return toolpath::expandCycles(tp).moves;
}

/// Closed mesh of the prism over an XZ polygon (counter-clockwise seen from -Y) from y0 to y1.
geom3d::Mesh extrudeXZ(const std::vector<Vec2>& poly, double y0, double y1) {
    geom3d::Mesh m;
    const auto n = static_cast<std::uint32_t>(poly.size());
    for (const Vec2& p : poly) {
        m.vertices.emplace_back(p.x(), y0, p.y());
    }
    for (const Vec2& p : poly) {
        m.vertices.emplace_back(p.x(), y1, p.y());
    }
    for (std::uint32_t k = 1; k + 1 < n; ++k) {
        m.triangles.push_back({0, k, k + 1});
        m.triangles.push_back({n, n + k + 1, n + k});
    }
    for (std::uint32_t k = 0; k < n; ++k) {
        const std::uint32_t j = (k + 1) % n;
        m.triangles.push_back({k, n + k, n + j});
        m.triangles.push_back({k, n + j, j});
    }
    if (m.signedVolume() < 0.0) {
        for (auto& t : m.triangles) {
            std::swap(t[1], t[2]);
        }
    }
    return m;
}

/// Points along a straight edge between two planar faces (normals na, nb).
BreakEdge straightEdge(const Vec3& a, const Vec3& b, const Vec3& na, const Vec3& nb,
                       int count = 41) {
    BreakEdge e;
    e.name = "edge 1";
    const Vec3 t = (b - a).normalized();
    const auto inFace = [&](const Vec3& n, const Vec3& other) {
        Vec3 u = n.cross(t).normalized();
        return u.dot(other) > 0.0 ? Vec3(-u) : u;
    };
    for (int k = 0; k < count; ++k) {
        EdgeSample s;
        s.point = a + (b - a) * (static_cast<double>(k) / (count - 1));
        s.tangent = t;
        s.normal = {na.normalized(), nb.normalized()};
        s.inFace = {inFace(na.normalized(), nb.normalized()),
                    inFace(nb.normalized(), na.normalized())};
        e.samples.push_back(s);
    }
    return e;
}

} // namespace

TEST_CASE("back boring with the spindle oriented is a back-boring cycle",
          "[strategies][backbore]") {
    Harness h(backBar());
    BackBoreParams p;
    p.depth = 3.0;
    p.shift = 2.0;
    p.shiftAngle = kPi; // -X
    p.orient = degToRad(90.0);
    p.dwell = 0.5;
    const auto tp = backBore({throughHole()}, p, h.context());
    CHECK(tp.strategy == "backbore");
    CHECK(tp.warnings.empty());
    REQUIRE(tp.cycles.size() == 1);
    const toolpath::DrillCycle& c = tp.cycles.front();
    CHECK(c.type == CycleType::BackBore);
    // The head's top (3 mm above the tip) ends 3 mm above the far side; it starts 2 mm below
    // the far side with all of the head clear of it.
    CHECK_THAT(c.topZ, WithinAbs(-20.0, 1e-6));
    CHECK_THAT(c.bottomZ, WithinAbs(-25.0, 1e-3));
    CHECK_THAT(c.retractZ, WithinAbs(2.0, 1e-9));
    CHECK_THAT(c.shift.x(), WithinAbs(-2.0, 1e-12));
    CHECK_THAT(c.shift.y(), WithinAbs(0.0, 1e-12));
    CHECK_THAT(c.orient, WithinAbs(degToRad(90.0), 1e-12));
    CHECK(c.feedRate == h.feeds.cut); // the preset's feed, as set (ADR-0004)
    CHECK(c.dwell == 0.5);

    SECTION("played out: oriented and shifted through, centred and turning up") {
        std::vector<toolpath::Move> moves;
        for (const auto& m : expanded(tp)) {
            if (m.end.z() < 2.0 - 1e-9 || m.orient) {
                moves.push_back(m);
            }
        }
        // Shift, down, back on centre (all oriented); cut up; dwell; shift, up, back.
        REQUIRE(moves.size() >= 7);
        const Vec3 hole(20.0, 20.0, 0.0);
        CHECK(moves[0].orient);
        CHECK((moves[0].end - Vec3(18.0, 20.0, 2.0)).norm() < 1e-9);
        CHECK(moves[1].orient);
        CHECK((moves[1].end - Vec3(18.0, 20.0, c.bottomZ)).norm() < 1e-9);
        CHECK(moves[2].orient);
        CHECK((moves[2].end - Vec3(20.0, 20.0, c.bottomZ)).norm() < 1e-9);
        CHECK_FALSE(moves[3].orient);
        CHECK(moves[3].kind == MoveKind::Linear);
        CHECK(moves[3].feedRate == h.feeds.cut);
        CHECK((moves[3].end - Vec3(20.0, 20.0, -20.0)).norm() < 1e-9);
        CHECK(moves[4].kind == MoveKind::Dwell);
        CHECK(moves[5].orient);
        CHECK(*moves[5].orient == c.orient);
        CHECK((moves[5].end - Vec3(18.0, 20.0, -20.0)).norm() < 1e-9);
        for (std::size_t k = 0; k < moves.size(); ++k) {
            if (moves[k].kind == MoveKind::Rapid && moves[k].end.z() < 0.0) {
                CHECK(moves[k].orient); // never a turning rapid inside the part
            }
        }
    }
    SECTION("JSON keeps the cycle and the oriented moves") {
        const auto back = toolpath::toolpathFromJson(toolpath::toJson(tp));
        CHECK_FALSE(toolpath::compareToolpaths(tp, back));
        const auto flat = toolpath::expandCycles(tp);
        CHECK_FALSE(
            toolpath::compareToolpaths(flat, toolpath::toolpathFromJson(toolpath::toJson(flat))));
    }
    SECTION("a back chamfer with a back-chamfer cutter") {
        Harness hc(backBar(degToRad(90.0)));
        BackBoreParams pc = p;
        pc.feature = BackFeature::Chamfer;
        pc.chamferWidth = 0.5;
        const auto tc = backBore({throughHole()}, pc, hc.context());
        REQUIRE(tc.cycles.size() == 1);
        // The 90° cone narrows from Ø13 at 3 mm to Ø5: it is Ø11 (5.5 = 5 + 0.5) at 4 mm.
        CHECK_THAT(tc.cycles.front().topZ, WithinAbs(-20.0 - 4.0, 1e-3));
        CHECK_THROWS_WITH(backBore({throughHole()}, pc, h.context()),
                          ContainsSubstring("back-chamfer cutter"));
    }
    SECTION("a blind hole has no far side to reach") {
        HoleTarget blind = throughHole();
        blind.through = false;
        const auto tb = backBore({blind}, p, h.context());
        CHECK(tb.cycles.empty());
        REQUIRE(tb.warnings.size() == 1);
        CHECK_THAT(tb.warnings.front(), ContainsSubstring("not a through hole"));
    }
}

TEST_CASE("back boring refuses a tool that cannot pass the hole", "[strategies][backbore]") {
    Harness h(backBar());
    BackBoreParams p;
    p.depth = 3.0;
    SECTION("the shift leaves the head on the wall") {
        p.shift = 1.0; // the Ø13 head reaches 5.5 mm from the axis of a Ø10 hole
        CHECK_THROWS_WITH(backBore({throughHole()}, p, h.context()),
                          ContainsSubstring("cannot pass through the hole"));
        CHECK_THROWS_WITH(backBore({throughHole()}, p, h.context()),
                          ContainsSubstring("between 1.500 and 2.500"));
    }
    SECTION("the shift puts the neck on the wall") {
        p.shift = 2.6;
        CHECK_THROWS_WITH(backBore({throughHole()}, p, h.context()),
                          ContainsSubstring("cannot pass through the hole"));
    }
    SECTION("the neck is too short for the hole") {
        p.shift = 2.0;
        tooling::Tool shortNeck = backBar();
        shortNeck.mill.neckLength = 15.0;
        Harness hs(shortNeck);
        CHECK_THROWS_WITH(backBore({throughHole()}, p, hs.context()), ContainsSubstring("neck"));
    }
    SECTION("the head is too wide for any shift") {
        p.shift = 2.0;
        HoleTarget small = throughHole();
        small.diameter = 8.5;
        CHECK_THROWS_WITH(backBore({small}, p, h.context()),
                          ContainsSubstring("cannot pass through the hole"));
    }
    SECTION("only a back-boring bar orients") {
        Harness hl(lollipop());
        CHECK_THROWS_WITH(backBore({throughHole()}, p, hl.context()),
                          ContainsSubstring("back-boring bar"));
        p.method = BackBoreMethod::Interpolated;
        HoleTarget small = throughHole();
        small.diameter = 5.0; // the Ø6 ball does not pass
        p.feature = BackFeature::Chamfer;
        CHECK_THROWS_WITH(backBore({small}, p, hl.context()), ContainsSubstring("does not pass"));
    }
}

TEST_CASE("back chamfer by interpolation with a lollipop", "[strategies][backbore]") {
    Harness h(lollipop());
    BackBoreParams p;
    p.method = BackBoreMethod::Interpolated;
    p.feature = BackFeature::Chamfer;
    p.chamferWidth = 0.5;
    p.chamferAngle = degToRad(45.0);
    const auto tp = backBore({throughHole()}, p, h.context());
    CHECK(tp.warnings.empty());
    CHECK(tp.cycles.empty());
    // One full circle at one height, on the hole's axis.
    double tip = 0.0;
    double rho = -1.0;
    int cuts = 0;
    Vec3 at = tp.start;
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::Cut) {
            REQUIRE(m.isArc());
            CHECK((m.center.head<2>() - Vec2(20.0, 20.0)).norm() < 1e-9);
            tip = m.end.z();
            rho = (m.end.head<2>() - Vec2(20.0, 20.0)).norm();
            CHECK_THAT(at.z(), WithinAbs(m.end.z(), 1e-12));
            ++cuts;
        }
        at = m.end;
    }
    CHECK(cuts == 2);
    // The ball (radius 3, centre 3 above the tip) on its circle reaches 5.5 mm from the axis at
    // the far side and 5.0 mm half a millimetre above it: the chamfer's two edges.
    const auto reach = [&](double z) {
        const double dz = z - (tip + 3.0);
        return rho + std::sqrt(9.0 - dz * dz);
    };
    CHECK_THAT(reach(-20.0), WithinAbs(5.5, 1e-3)); // the profile is a polyline within 1e-4
    CHECK_THAT(reach(-19.5), WithinAbs(5.0, 1e-3));
    // Feeds are the preset's, as set: leads at the lead feed, the circle at the cutting feed.
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::LeadIn || m.feedClass == FeedClass::LeadOut) {
            CHECK(m.feedRate == h.feeds.lead);
        }
    }
}

TEST_CASE("back counterbore by interpolation with a slot cutter", "[strategies][backbore]") {
    tooling::Tool cutter = tooling::makeDefaultTool(tooling::ToolType::SlotCutter, 3, 8.0);
    cutter.mill.diameter = 8.0;
    cutter.mill.fluteLength = 2.0;
    cutter.mill.neckDiameter = 2.0;
    cutter.mill.neckLength = 35.0;
    cutter.mill.overallLength = 70.0;
    cutter.mill.shankDiameter = 8.0;
    REQUIRE(cutter.validate().empty());
    Harness h(cutter);
    BackBoreParams p;
    p.method = BackBoreMethod::Interpolated;
    p.feature = BackFeature::Counterbore;
    p.depth = 2.0;
    p.helixPitch = 0.5;
    CHECK_THROWS_WITH(backBore({throughHole()}, p, h.context()),
                      ContainsSubstring("counterbore diameter"));
    p.diameter = 15.0;
    const auto tp = backBore({throughHole()}, p, h.context());
    CHECK(tp.warnings.empty());
    // One circle (radius 3.5) covers 5..7.5 mm: a helix up to the top, then a circle there.
    double top = -1e9;
    double low = 1e9;
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::Cut) {
            CHECK_THAT((m.end.head<2>() - Vec2(20.0, 20.0)).norm(), WithinAbs(3.5, 1e-9));
            top = std::max(top, m.end.z());
            low = std::min(low, m.end.z());
            CHECK(m.feedRate == h.feeds.cut);
        }
    }
    CHECK_THAT(top, WithinAbs(-20.0, 1e-9)); // the head's top 2 mm above the far side
    CHECK(low < -20.0);                      // climbing from below the far side
    SECTION("wider than the neck allows") {
        // The neck (Ø2) stays in the Ø10 hole: the head (Ø8) reaches Ø16 at most.
        p.diameter = 16.5;
        CHECK_THROWS_WITH(backBore({throughHole()}, p, h.context()),
                          ContainsSubstring("the largest is Ø16.000"));
    }
}

TEST_CASE("back boring parameters", "[strategies][backbore][params]") {
    const tooling::Tool bar = backBar();
    // The shift while not set: half the head's overhang over the neck.
    const BackBoreParams d = backBoreParams(Json::object(), bar, nullptr);
    CHECK_THAT(d.shift, WithinAbs(2.0, 1e-12));
    CHECK(d.method == BackBoreMethod::Oriented);
    CHECK_THAT(d.shiftAngle, WithinAbs(kPi, 1e-12));
    const BackBoreParams s = backBoreParams(
        Json{{"shift", 1.75}, {"shift_direction", "y+"}, {"orient_angle", 30.0}}, bar, nullptr);
    CHECK(s.shift == 1.75);
    CHECK_THAT(s.shiftAngle, WithinAbs(kPi / 2.0, 1e-12));
    CHECK_THAT(s.orient, WithinAbs(degToRad(30.0), 1e-12));
    const StrategyInfo* info = findStrategy("backbore");
    REQUIRE(info != nullptr);
    CHECK(info->geometry == GeometryUse::Holes);
    REQUIRE(info->find("shift") != nullptr);
    REQUIRE(info->find("shift")->toolDefault);
    CHECK_THAT(info->find("shift")->toolDefault(bar), WithinAbs(2.0, 1e-12));
}

TEST_CASE("break poses hold the width across the edge", "[strategies][edgebreak]") {
    const tooling::Tool mill90 = tooling::makeChamferMill(1, 8.0, 0.0, degToRad(90.0), 50.0);
    SECTION("a horizontal square edge: a 45° chamfer under the cone") {
        // The top edge of a block's front face (y = 0), along +X.
        const BreakEdge e = straightEdge({0, 0, 0}, {10, 0, 0}, Vec3::UnitZ(), -Vec3::UnitY(), 3);
        const auto pose = breakPose(mill90, e.samples[1], std::sqrt(2.0), 0.5);
        REQUIRE(pose);
        CHECK_THAT(pose->faceWidth[0], WithinAbs(1.0, 1e-9));
        CHECK_THAT(pose->faceWidth[1], WithinAbs(1.0, 1e-9));
        // As a 2D chamfer of width 1 with the tip 0.5 below it: the axis 0.5 out, the tip 1.5
        // down (DL-0014).
        CHECK((pose->tip - Vec3(5.0, -0.5, -1.5)).norm() < 1e-9);
        CHECK((pose->out - Vec3(0, -1, 0)).norm() < 1e-12);
    }
    SECTION("a sloped edge keeps the width") {
        // The front edge of a top face sloping down along +X at 14°.
        const Vec3 nTop = Vec3(0.25, 0.0, 1.0).normalized();
        const BreakEdge e = straightEdge({0, 0, 12}, {40, 0, 2}, nTop, -Vec3::UnitY(), 3);
        const auto pose = breakPose(mill90, e.samples[1], 0.5, 0.3);
        REQUIRE(pose);
        const EdgeSample& s = e.samples[1];
        const Vec3 ea = s.point + pose->faceWidth[0] * s.inFace[0];
        const Vec3 eb = s.point + pose->faceWidth[1] * s.inFace[1];
        CHECK_THAT((ea - eb).norm(), WithinAbs(0.5, 1e-9));
        // The break's plane contains the edge and both of its edges touch the cone.
        CHECK_THAT(pose->planeNormal.dot(s.tangent), WithinAbs(0.0, 1e-9));
        CHECK_THAT(pose->planeNormal.z(), WithinAbs(std::sin(kPi / 4.0), 1e-9));
        CHECK(pose->tip.y() < 0.0); // the tool stands outside the front face
    }
    SECTION("a ball meets both faces the width apart") {
        const tooling::Tool ball = tooling::makeBallEndMill(2, 6.0, 12.0, 50.0);
        const BreakEdge e = straightEdge({0, 0, 0}, {10, 0, 0}, Vec3::UnitZ(), -Vec3::UnitY(), 3);
        const auto pose = breakPose(ball, e.samples[1], 1.0, 0.0);
        REQUIRE(pose);
        const Vec3 centre = pose->tip + Vec3(0, 0, 3.0);
        const Vec3 ea = e.samples[1].point + pose->faceWidth[0] * e.samples[1].inFace[0];
        const Vec3 eb = e.samples[1].point + pose->faceWidth[1] * e.samples[1].inFace[1];
        CHECK_THAT((ea - eb).norm(), WithinAbs(1.0, 1e-9));
        CHECK_THAT((centre - ea).norm(), WithinAbs(3.0, 1e-9));
        CHECK_THAT((centre - eb).norm(), WithinAbs(3.0, 1e-9));
    }
    SECTION("what the tool cannot do with its axis vertical") {
        std::string why;
        // An inside corner: a floor meeting a wall that rises from it.
        const BreakEdge inside =
            straightEdge({0, 0, 0}, {10, 0, 0}, Vec3::UnitZ(), Vec3::UnitY(), 3);
        BreakEdge flipped = inside;
        for (auto& s : flipped.samples) {
            s.inFace = {-Vec3::UnitY(), Vec3::UnitZ()};
        }
        CHECK_FALSE(breakPose(mill90, flipped.samples[1], 0.5, 0.3, &why));
        CHECK(why == "an inside corner");
        // A vertical edge of a block: too steep for the cone.
        const BreakEdge vertical =
            straightEdge({0, 0, 0}, {0, 0, -10}, -Vec3::UnitX(), -Vec3::UnitY(), 3);
        CHECK_FALSE(breakPose(mill90, vertical.samples[1], 0.5, 0.3, &why));
        CHECK_THAT(why, ContainsSubstring("too steep"));
        // An undercut: the lower face turns in under the edge.
        // A knife edge whose lower face looks down: the ball would have to cut above its
        // equator.
        const tooling::Tool ball = tooling::makeBallEndMill(2, 6.0, 12.0, 50.0);
        const BreakEdge under = straightEdge({0, 0, 0}, {10, 0, 0}, Vec3::UnitZ(),
                                             Vec3(0.0, -0.2, -1.0).normalized(), 3);
        CHECK_FALSE(breakPose(ball, under.samples[1], 1.0, 0.0, &why));
        CHECK_THAT(why, ContainsSubstring("equator"));
        CHECK_THROWS_WITH(
            breakPose(tooling::makeFlatEndMill(1, 6, 20, 50), inside.samples[1], 0.5, 0.3),
            ContainsSubstring("chamfer mill"));
    }
}

TEST_CASE("3D edge breaking follows the edge and skips what it cannot reach",
          "[strategies][edgebreak]") {
    Harness h(tooling::makeChamferMill(1, 8.0, 0.0, degToRad(90.0), 50.0));
    // A wedge: top from z = 12 at x = 0 down to z = 2 at x = 40, y from 0 to 20.
    const geom3d::Mesh wedge = extrudeXZ({{0, 0}, {40, 0}, {40, 2}, {0, 12}}, 0.0, 20.0);
    const Vec3 nTop = Vec3(0.25, 0.0, 1.0).normalized();
    BreakEdge front = straightEdge({0, 0, 12}, {40, 0, 2}, nTop, -Vec3::UnitY());
    front.faces = {0, 0}; // the wedge as one mesh: its band is left out wherever it is
    EdgeBreakParams p;
    p.width = 0.5;
    p.tipOffset = 0.3;
    const auto tp = edgeBreak({front}, {wedge}, p, h.context());
    CHECK(tp.strategy == "edgebreak3d");
    CHECK(tp.warnings.empty());
    std::vector<Vec3> cuts;
    for (const auto& m : tp.moves) {
        if (m.feedClass == FeedClass::Cut) {
            cuts.push_back(m.end);
            CHECK(m.feedRate == h.feeds.cut);
            CHECK((m.axis - Vec3::UnitZ()).norm() < 1e-12);
        }
        if (m.feedClass == FeedClass::LeadIn || m.feedClass == FeedClass::LeadOut) {
            CHECK(m.feedRate == h.feeds.lead);
        }
    }
    REQUIRE(cuts.size() == front.samples.size() - 1);
    // Every point is the pose of its sample: down the slope with the edge.
    for (std::size_t k = 0; k < cuts.size(); ++k) {
        const auto pose = breakPose(h.tool, front.samples[k + 1], p.width, p.tipOffset);
        REQUIRE(pose);
        CHECK((cuts[k] - pose->tip).norm() < 1e-9);
    }
    CHECK(cuts.front().z() > cuts.back().z() + 9.0);

    SECTION("a wall next to the edge is not cut: the edge is skipped and listed") {
        // A block standing 1 mm in front of the front face, taller than the wedge.
        const geom3d::Mesh wall = extrudeXZ({{-5, -5}, {45, -5}, {45, 20}, {-5, 20}}, -4.0, -1.0);
        BreakEdge named = front;
        named.name = "edge 7";
        const auto skipped = edgeBreak({named}, {wedge, wall}, p, h.context());
        REQUIRE(skipped.warnings.size() == 1);
        CHECK_THAT(skipped.warnings.front(), ContainsSubstring("edge 7"));
        CHECK_THAT(skipped.warnings.front(), ContainsSubstring("cut past the break"));
        for (const auto& m : skipped.moves) {
            CHECK(m.feedClass != FeedClass::Cut);
        }
    }
    SECTION("unreachable edges are listed, the others cut") {
        const BreakEdge vertical =
            straightEdge({0, 0, 0}, {0, 0, 12}, -Vec3::UnitX(), -Vec3::UnitY());
        BreakEdge seam;
        seam.name = "edge 9";
        seam.unusable = "it bounds one face only (a seam or a free edge)";
        BreakEdge v = vertical;
        v.name = "edge 3";
        const auto mixed = edgeBreak({v, front, seam}, {wedge}, p, h.context());
        REQUIRE(mixed.warnings.size() == 1);
        CHECK_THAT(mixed.warnings.front(), ContainsSubstring("2 of 3 edges skipped"));
        CHECK_THAT(mixed.warnings.front(), ContainsSubstring("edge 3 (the edge is too steep"));
        CHECK_THAT(mixed.warnings.front(), ContainsSubstring("edge 9 (it bounds one face"));
        CHECK(std::count_if(mixed.moves.begin(), mixed.moves.end(), [](const auto& m) {
                  return m.feedClass == FeedClass::Cut;
              }) == static_cast<long>(cuts.size()));
    }
    SECTION("parameters") {
        const EdgeBreakParams d = edgeBreakParams(Json{{"width", 0.4}});
        CHECK(d.width == 0.4);
        CHECK_FALSE(d.leadLength);
        const StrategyInfo* info = findStrategy("edgebreak3d");
        REQUIRE(info != nullptr);
        CHECK(info->geometry == GeometryUse::AnyEdges);
        CHECK_THAT(info->find("lead_length")->toolDefault(h.tool), WithinAbs(2.0, 1e-12));
    }
}
