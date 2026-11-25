// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Probing cycles (DL-0090): where the stylus goes, and what each cycle records.

#include "StrategyTestUtil.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/strategies/Probing.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <vector>

using namespace ocam;
using namespace ocam::strategies;
using namespace ocam::strategies::testing;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using toolpath::FeedClass;
using toolpath::MoveKind;
using toolpath::ProbeFeature;

namespace {

constexpr double kTrigger = 0.02;

/// A Ø6 stylus with a 0.02 mm trigger offset: effective radius 2.98.
tooling::Tool probeTool() {
    tooling::Tool t = tooling::makeDefaultTool(tooling::ToolType::Probe, 20, 6.0);
    t.mill.triggerOffset = kTrigger;
    return t;
}

ProbeParams base(ProbeFeature feature) {
    ProbeParams p;
    p.feature = feature;
    p.workOffset = 1;
    p.approach = 5.0;
    p.overtravel = 4.0;
    p.fastFeed = 1000.0 / 60.0;
    p.measureFeed = 100.0 / 60.0;
    return p;
}

std::vector<toolpath::Move> measures(const toolpath::Toolpath& tp) {
    std::vector<toolpath::Move> out;
    for (const toolpath::Move& m : tp.moves) {
        if (m.kind == MoveKind::Probe && m.measure) {
            out.push_back(m);
        }
    }
    return out;
}

void checkVec(const Vec3& a, const Vec3& b) {
    CHECK_THAT(a.x(), WithinAbs(b.x(), 1e-9));
    CHECK_THAT(a.y(), WithinAbs(b.y(), 1e-9));
    CHECK_THAT(a.z(), WithinAbs(b.z(), 1e-9));
}

/// Moves below the retract height are probe moves only: no rapid goes down there.
void checkProtected(const toolpath::Toolpath& tp, double retract) {
    for (const toolpath::Move& m : tp.moves) {
        if (m.kind == MoveKind::Rapid) {
            CHECK(m.end.z() >= retract - 1e-9);
        } else {
            CHECK(m.kind == MoveKind::Probe);
            CHECK(m.feedClass == FeedClass::Probe);
            CHECK(m.probe == 0);
        }
    }
}

} // namespace

TEST_CASE("the probing operations and their parameters", "[strategies][probing]") {
    for (const char* id :
         {"probe_surface", "probe_width", "probe_circle", "probe_corner", "probe_check"}) {
        INFO(id);
        const StrategyInfo* info = findStrategy(id);
        REQUIRE(info);
        CHECK(isProbingStrategy(id));
        CHECK(info->geometry == GeometryUse::Faces);
        CHECK_FALSE(info->description.empty());
        const ParamDef* fast = info->find("fast_feed");
        REQUIRE(fast);
        CHECK(fast->kind == ParamKind::Feed);
        CHECK((info->find("tolerance") != nullptr) == (std::string_view(id) == "probe_check"));
        CHECK((info->find("work_offset") != nullptr) != (std::string_view(id) == "probe_check"));
    }
    CHECK_FALSE(isProbingStrategy("drill"));

    // Feeds are mm/min in the parameters, mm/s in code; the defaults are the ParamDefs'.
    const ProbeParams surface = probeParams("probe_surface", Json::object());
    CHECK_THAT(surface.fastFeed, WithinAbs(1000.0 / 60.0, 1e-12));
    CHECK_THAT(surface.measureFeed, WithinAbs(100.0 / 60.0, 1e-12));
    CHECK(surface.workOffset == 0); // the setup's own, resolved by the document
    checkVec(surface.direction, -Vec3::UnitZ());
    CHECK(probeAxes(ProbeParams{surface.feature, {}, Vec3::UnitX(), 0, 1}) == "X");

    const ProbeParams width = probeParams(
        "probe_width", Json{{"feature", "pocket"}, {"axis", "y"}, {"width", 30.0}, {"x", 5.0}});
    CHECK(width.feature == ProbeFeature::Pocket);
    checkVec(width.direction, Vec3::UnitY());
    CHECK(width.size == 30.0);
    CHECK(width.nominal.x() == 5.0);

    // Outside a part's -X -Y corner the stylus moves +X and +Y; inside a pocket's, -X and -Y.
    const ProbeParams outside = probeParams("probe_corner", Json{{"corner", "x_minus_y_minus"}});
    checkVec(outside.direction, Vec3(1, 1, 0));
    const ProbeParams inside = probeParams(
        "probe_corner", Json{{"feature", "inside_corner"}, {"corner", "x_plus_y_minus"}});
    checkVec(inside.direction, Vec3(1, -1, 0));

    const ProbeParams check = probeParams(
        "probe_check", Json{{"feature", "bore"}, {"diameter", 12.0}, {"tolerance", 0.01}});
    CHECK(check.workOffset == 0);
    CHECK(check.tolerance == 0.01);
    CHECK(check.size == 12.0);
    CHECK(probeAxes(check).empty());
    CHECK_THROWS_AS(probeParams("drill", Json::object()), Error);
}

TEST_CASE("a surface probed down Z", "[strategies][probing]") {
    Harness h(probeTool());
    ProbeParams p = base(ProbeFeature::Surface);
    p.nominal = Vec3(10, 20, 0);
    p.direction = -Vec3::UnitZ();
    const toolpath::Toolpath tp = probe(p, h.context());
    CHECK(tp.strategy == "probe");
    CHECK(tp.spindleSpeed == 0.0);
    CHECK(tp.coolant == tooling::Coolant::Off);
    REQUIRE(tp.probes.size() == 1);
    const toolpath::ProbeCycle& c = tp.probes[0];
    CHECK(c.feature == ProbeFeature::Surface);
    CHECK(c.axes == "Z");
    CHECK(c.workOffset == 1);
    CHECK(c.stylusDiameter == 6.0);
    CHECK(c.triggerOffset == kTrigger);
    CHECK(c.overtravel == 4.0);
    checkProtected(tp, h.heights.retract);

    // Rapid to the retract height, down to the approach distance, touch, back, up, away.
    const double re = 3.0 - kTrigger;
    REQUIRE(tp.moves.size() == 6);
    checkVec(tp.moves[0].end, {10, 20, 5});
    CHECK(tp.moves[0].kind == MoveKind::Rapid);
    checkVec(tp.moves[1].end, {10, 20, re + 5.0 - 3.0}); // tip = ball centre - radius
    CHECK_FALSE(tp.moves[1].measure);
    CHECK(tp.moves[1].feedRate == p.fastFeed);
    checkVec(tp.moves[2].end, {10, 20, re - 3.0}); // the tip a trigger offset into the face
    CHECK(tp.moves[2].measure);
    CHECK(tp.moves[2].feedRate == p.measureFeed);
    checkVec(tp.moves[3].end, tp.moves[1].end);
    checkVec(tp.moves[4].end, {10, 20, 5});
    checkVec(tp.moves[5].end, {10, 20, 15});
    CHECK(tp.moves[5].kind == MoveKind::Rapid);
}

TEST_CASE("a surface probed along X", "[strategies][probing]") {
    Harness h(probeTool());
    ProbeParams p = base(ProbeFeature::Surface);
    p.nominal = Vec3(0, 10, -4); // a face looking towards -X at X0, touched at Z-4
    p.direction = Vec3::UnitX();
    const toolpath::Toolpath tp = probe(p, h.context());
    const auto m = measures(tp);
    REQUIRE(m.size() == 1);
    checkVec(m[0].end, {-(3.0 - kTrigger), 10, -7});
    CHECK(tp.probes[0].axes == "X");
}

TEST_CASE("a bore is touched four times from its centre", "[strategies][probing]") {
    Harness h(probeTool());
    ProbeParams p = base(ProbeFeature::Bore);
    p.nominal = Vec3(40, 30, -6);
    p.size = 20.0;
    const toolpath::Toolpath tp = probe(p, h.context());
    checkProtected(tp, h.heights.retract);
    CHECK(tp.probes[0].axes == "XY");
    const auto m = measures(tp);
    REQUIRE(m.size() == 4);
    const double reach = 10.0 - (3.0 - kTrigger);
    checkVec(m[0].end, {40 + reach, 30, -9});
    checkVec(m[1].end, {40, 30 + reach, -9});
    checkVec(m[2].end, {40 - reach, 30, -9});
    checkVec(m[3].end, {40, 30 - reach, -9});
    // It stays down in the bore: one rapid in, one out.
    int rapids = 0;
    for (const toolpath::Move& mv : tp.moves) {
        rapids += mv.kind == MoveKind::Rapid ? 1 : 0;
    }
    CHECK(rapids == 2);

    p.size = 2.0 * (5.0 + 2.98) - 0.1; // no room for the stylus and the approach distance
    CHECK_THROWS_WITH(probe(p, h.context()), ContainsSubstring("too small"));
}

TEST_CASE("a boss is touched from outside, over it between touches", "[strategies][probing]") {
    Harness h(probeTool());
    ProbeParams p = base(ProbeFeature::Boss);
    p.nominal = Vec3(0, 0, -5);
    p.size = 20.0;
    p.workOffset = 3;
    const toolpath::Toolpath tp = probe(p, h.context());
    checkProtected(tp, h.heights.retract);
    CHECK(tp.probes[0].workOffset == 3);
    const auto m = measures(tp);
    REQUIRE(m.size() == 4);
    checkVec(m[0].end, {10 + 2.98, 0, -8});
    checkVec(m[2].end, {-10 - 2.98, 0, -8});
    int rapids = 0;
    for (const toolpath::Move& mv : tp.moves) {
        rapids += mv.kind == MoveKind::Rapid ? 1 : 0;
    }
    CHECK(rapids == 2); // in and away: over the boss between touches by protected moves
}

TEST_CASE("a web and a pocket across X and Y", "[strategies][probing]") {
    Harness h(probeTool());
    ProbeParams web = base(ProbeFeature::Web);
    web.nominal = Vec3(50, 30, -10);
    web.direction = Vec3::UnitX();
    web.size = 100.0;
    const auto w = measures(probe(web, h.context()));
    REQUIRE(w.size() == 2);
    checkVec(w[0].end, {-2.98, 30, -13});
    checkVec(w[1].end, {102.98, 30, -13});

    ProbeParams pocket = base(ProbeFeature::Pocket);
    pocket.nominal = Vec3(50, 30, -5);
    pocket.direction = Vec3::UnitY();
    pocket.size = 20.0;
    const toolpath::Toolpath tp = probe(pocket, h.context());
    CHECK(tp.probes[0].axes == "Y");
    const auto q = measures(tp);
    REQUIRE(q.size() == 2);
    checkVec(q[0].end, {50, 20 + 2.98, -8});
    checkVec(q[1].end, {50, 40 - 2.98, -8});
}

TEST_CASE("outside and inside corners", "[strategies][probing]") {
    Harness h(probeTool());
    ProbeParams outside = base(ProbeFeature::OutsideCorner);
    outside.nominal = Vec3(0, 0, -5);
    outside.direction = Vec3(1, 1, 0); // the part's -X -Y corner
    outside.inset = 8.0;
    const auto o = measures(probe(outside, h.context()));
    REQUIRE(o.size() == 2);
    checkVec(o[0].end, {-2.98, 8, -8}); // the X face, 8 mm along it
    checkVec(o[1].end, {8, -2.98, -8}); // the Y face

    ProbeParams inside = base(ProbeFeature::InsideCorner);
    inside.nominal = Vec3(30, 20, -5);
    inside.direction = Vec3(-1, -1, 0); // a pocket's -X -Y corner
    inside.inset = 10.0;
    const toolpath::Toolpath tp = probe(inside, h.context());
    checkProtected(tp, h.heights.retract);
    const auto i = measures(tp);
    REQUIRE(i.size() == 2);
    checkVec(i[0].end, {32.98, 30, -8});
    checkVec(i[1].end, {40, 22.98, -8});
    // In and out at the point inside the corner.
    checkVec(tp.moves[0].end, {40, 30, 5});

    inside.inset = 6.0; // the ball and the approach distance do not fit
    CHECK_THROWS_WITH(probe(inside, h.context()), ContainsSubstring("distance from the corner"));
}

TEST_CASE("probing refuses what it cannot do", "[strategies][probing]") {
    ProbeParams p = base(ProbeFeature::Surface);
    p.direction = -Vec3::UnitZ();
    {
        Harness h(tooling::makeFlatEndMill(1, 6, 15, 50));
        CHECK_THROWS_WITH(probe(p, h.context()), ContainsSubstring("needs a probe"));
    }
    Harness h(probeTool());
    // ADR-0004: a feed left at 0 is refused, by name; nothing is put in its place.
    ProbeParams slow = p;
    slow.measureFeed = 0.0;
    CHECK_THROWS_WITH(probe(slow, h.context()), ContainsSubstring("measuring feed is 0"));
    slow = p;
    slow.fastFeed = 0.0;
    CHECK_THROWS_WITH(probe(slow, h.context()), ContainsSubstring("positioning feed is 0"));
    // Starting to touch above the retract height would need a rapid down to it.
    ProbeParams high = p;
    high.nominal.z() = 10.0;
    CHECK_THROWS_WITH(probe(high, h.context()), ContainsSubstring("retract height"));
}
