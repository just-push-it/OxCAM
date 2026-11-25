// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Probing moves and cycles in the toolpath IR (DL-0090).

#include <ocam/toolpath/PathBuilder.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace ocam;
using namespace ocam::toolpath;
using Catch::Matchers::WithinAbs;

namespace {

/// A bore probed at two points, by hand.
Toolpath probedBore() {
    Toolpath tp;
    tp.strategy = "probe";
    tp.start = Vec3(0, 0, 20);
    ProbeCycle c;
    c.feature = ProbeFeature::Bore;
    c.nominal = Vec3(0, 0, -5);
    c.size = 20.0;
    c.workOffset = 2;
    c.axes = "XY";
    c.approach = 2.0;
    c.overtravel = 3.0;
    c.fastFeed = 20.0;
    c.measureFeed = 2.0;
    c.stylusDiameter = 4.0;
    c.triggerOffset = 0.01;
    tp.probes.push_back(c);
    PathBuilder b(tp);
    b.rapid({0, 0, 5});
    b.probe({0, 0, -7}, 20.0, 0, false);
    b.probe({8.01, 0, -7}, 2.0, 0, true);
    b.probe({0, 0, -7}, 20.0, 0, false);
    b.probe({-8.01, 0, -7}, 2.0, 0, true);
    b.probe({0, 0, -7}, 20.0, 0, false);
    b.probe({0, 0, 5}, 20.0, 0, false);
    b.retractTo(20.0);
    return tp;
}

} // namespace

TEST_CASE("probe moves and cycles have names and kinds of their own", "[toolpath][probing]") {
    CHECK(toString(MoveKind::Probe) == "probe");
    CHECK(toString(FeedClass::Probe) == "probe");
    CHECK(parseFeedClass("probe") == FeedClass::Probe);
    for (ProbeFeature f :
         {ProbeFeature::Surface, ProbeFeature::Web, ProbeFeature::Pocket, ProbeFeature::Bore,
          ProbeFeature::Boss, ProbeFeature::OutsideCorner, ProbeFeature::InsideCorner}) {
        CHECK(parseProbeFeature(toString(f)) == f);
    }
    CHECK_FALSE(parseProbeFeature("hole").has_value());

    const Toolpath tp = probedBore();
    CHECK(tp.isProbing());
    REQUIRE(tp.moves.size() == 8);
    CHECK(tp.moves[1].kind == MoveKind::Probe);
    CHECK(tp.moves[1].feedClass == FeedClass::Probe);
    CHECK_FALSE(tp.moves[1].measure);
    CHECK(tp.moves[2].measure);
    CHECK(tp.moves[2].probe == 0);
    CHECK(tp.probes[0].isCheck() == false);
}

TEST_CASE("probe moves round-trip through JSON and are compared", "[toolpath][probing]") {
    const Toolpath tp = probedBore();
    const Toolpath back = toolpathFromJson(toJson(tp));
    CHECK_FALSE(compareToolpaths(tp, back).has_value());
    CHECK(back.probes == tp.probes);
    CHECK(back.moves[2].measure);
    CHECK(back.moves[2].kind == MoveKind::Probe);
    CHECK(toJson(back) == toJson(tp));

    Toolpath other = tp;
    other.moves[2].measure = false;
    CHECK(compareToolpaths(tp, other).has_value());
    other = tp;
    other.probes[0].workOffset = 3;
    CHECK(compareToolpaths(tp, other).has_value());
    other = tp;
    other.probes[0].measureFeed = 3.0;
    CHECK(compareToolpaths(tp, other).has_value());

    // Toolpaths without probing cycles are written as before.
    Toolpath plain;
    PathBuilder b(plain);
    b.feed({1, 0, 0}, FeedClass::Cut, 1.0);
    CHECK_FALSE(toJson(plain).contains("probes"));
}

TEST_CASE("probe moves are timed, sampled and moved like feed moves", "[toolpath][probing]") {
    const Toolpath tp = probedBore();
    const std::vector<double> times = moveDurations(tp, {});
    CHECK_THAT(times[1], WithinAbs(12.0 / 20.0, 1e-12));
    CHECK_THAT(times[2], WithinAbs(8.01 / 2.0, 1e-12));
    const Stats stats = computeStats(tp);
    CHECK_THAT(stats.bounds.min.x(), WithinAbs(-8.01, 1e-12));
    CHECK_THAT(stats.cutLength, WithinAbs(12.0 + 4 * 8.01 + 12.0, 1e-9));

    bool probeRun = false;
    for (const PathRun& run : sampleRuns(tp)) {
        probeRun = probeRun || run.feedClass == FeedClass::Probe;
    }
    CHECK(probeRun);

    // Into another frame: the cycle's nominal position and direction go with the moves.
    Toolpath along = tp;
    along.probes[0].feature = ProbeFeature::Surface;
    along.probes[0].direction = Vec3::UnitX();
    Transform3 t = Transform3::Identity();
    t.translate(Vec3(10, 0, 1));
    t.rotate(Eigen::AngleAxisd(kPi / 2.0, Vec3::UnitZ()));
    const Toolpath moved = transformed(along, t);
    CHECK_THAT(moved.probes[0].nominal.x(), WithinAbs(10.0, 1e-12));
    CHECK_THAT(moved.probes[0].nominal.z(), WithinAbs(-4.0, 1e-12));
    CHECK_THAT(moved.probes[0].direction.y(), WithinAbs(1.0, 1e-12));
    CHECK(moved.moves[2].kind == MoveKind::Probe);
    CHECK_THAT(moved.moves[2].end.y(), WithinAbs(8.01, 1e-12));
}
