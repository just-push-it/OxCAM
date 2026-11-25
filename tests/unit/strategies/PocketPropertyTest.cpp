// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Property tests of the pocket strategy on random regions:
//   1. no gouge: every feed point at depth lies in the allowed tool-centre area;
//   2. full coverage: all material a round tool can reach is swept at every level;
//   3. ramps never exceed the ramp angle.
// Runs 300 random regions by default; set OCAM_PROPERTY_ITERATIONS (e.g. 10000) for longer runs.

#include "StrategyTestUtil.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdlib>
#include <random>

using namespace ocam;
using namespace ocam::strategies;
using namespace ocam::strategies::testing;

namespace {

int iterations() {
    if (const char* env = std::getenv("OCAM_PROPERTY_ITERATIONS")) {
        return std::max(1, std::atoi(env));
    }
    return 300;
}

// Random star-shaped polygon around the origin, optionally with a circular island.
geom2d::Region randomRegion(std::mt19937& rng) {
    std::uniform_int_distribution<int> vertexCount(5, 14);
    std::uniform_real_distribution<double> radius(20.0, 60.0);
    std::uniform_real_distribution<double> jitter(-0.3, 0.3);
    const int n = vertexCount(rng);
    std::vector<Vec2> pts;
    double minR = 1e9;
    for (int i = 0; i < n; ++i) {
        const double a = 2.0 * kPi * (i + 0.5 + jitter(rng)) / n;
        const double r = radius(rng);
        minR = std::min(minR, r);
        pts.emplace_back(r * std::cos(a), r * std::sin(a));
    }
    geom2d::Region region{geom2d::Polyline::fromPoints(pts, true), {}};
    if (std::uniform_int_distribution<int>(0, 2)(rng) == 0 && minR > 18.0) {
        geom2d::Polyline island =
            geom2d::Polyline::circle({0, 0}, std::uniform_real_distribution<double>(2.0, 6.0)(rng));
        island.reverse();
        region.holes.push_back(island);
    }
    return region;
}

} // namespace

TEST_CASE("pocket properties on random regions", "[strategies][pocket][property]") {
    std::mt19937 rng(20261018);
    const double diameters[] = {4.0, 6.0, 10.0};
    const int count = iterations();
    int failures = 0;
    for (int it = 0; it < count && failures < 5; ++it) {
        const geom2d::Regions area{randomRegion(rng)};
        const double d = diameters[std::uniform_int_distribution<int>(0, 2)(rng)];
        Harness h(tooling::makeFlatEndMill(1, d, 3 * d, 60));
        PocketParams p;
        p.stepoverRatio = std::uniform_real_distribution<double>(0.2, 0.9)(rng);
        p.stockToLeaveXY = std::uniform_real_distribution<double>(0.0, 0.5)(rng);
        p.entry = static_cast<EntryKind>(std::uniform_int_distribution<int>(0, 2)(rng));
        p.rampAngle = degToRad(std::uniform_real_distribution<double>(1.0, 10.0)(rng));
        const double z = -2.0;

        INFO("iteration " << it << ", D " << d << ", stepover " << p.stepoverRatio << ", stock "
                          << p.stockToLeaveXY);
        const auto tp = pocket(area, 0.0, z, p, h.context());

        const geom2d::ClipOptions clip{0.002};
        const auto allowed = geom2d::offset(area, -(d / 2 + p.stockToLeaveXY), clip);
        if (allowed.empty()) {
            CHECK(tp.moves.empty());
            continue;
        }
        const auto allowedGrown = geom2d::offset(allowed, 0.01, clip);
        const auto reachable = geom2d::offset(allowed, d / 2, clip);

        bool ok = true;
        // 1. No gouge.
        for (const Vec3& q : feedPoints(tp)) {
            if (q.z() < -1e-9 &&
                std::none_of(allowedGrown.begin(), allowedGrown.end(),
                             [&](const auto& r) { return r.contains(q.head<2>()); })) {
                ok = false;
                FAIL_CHECK("feed point outside the allowed area: " << q.transpose());
                break;
            }
        }
        // 2. Coverage.
        const auto left =
            geom2d::subtract(reachable, geom2d::sweep(cutPathsAt(tp, z), d / 2, clip), clip);
        const double uncovered =
            geom2d::totalArea(geom2d::offset(geom2d::offset(left, -0.02, clip), 0.02, clip));
        if (uncovered > 0.01) {
            ok = false;
            FAIL_CHECK("uncut material: " << uncovered << " mm^2");
        }
        // 3. Ramp angle.
        Vec3 pos = tp.start;
        for (const auto& m : tp.moves) {
            if (m.feedClass == toolpath::FeedClass::Ramp) {
                double run = (m.end - pos).head<2>().norm();
                if (m.isArc()) {
                    run = (pos.head<2>() - m.center.head<2>()).norm() *
                          std::abs(toolpath::arcSweep(pos, m));
                }
                if (pos.z() - m.end.z() > run * std::tan(p.rampAngle) + 1e-6) {
                    ok = false;
                    FAIL_CHECK("ramp steeper than the ramp angle");
                    break;
                }
            }
            pos = m.end;
        }
        if (!ok) {
            ++failures;
        }
    }
}
