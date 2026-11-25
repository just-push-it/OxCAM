// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Swarf milling (DL-0085) and Z-level roughing in a tilted orientation (DL-0036) on the swarf
// fixture: a base plate with a tapered round boss and a twisted square boss.

#include "FixtureSpecs.hpp"

#include <ocam/document/Document.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <set>

using namespace ocam;
using namespace ocam::document;
using namespace ocam::fixtures::swarf;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double kToolRadius = 3.0; // T2, Ø6 flat end mill
const Vec3 kModelToWcs(-kBaseX / 2, -kBaseY / 2, 0.0);

struct SwarfProject {
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "swarf_project.ocam");
    CancelToken cancel;
    NullProgress progress;

    const Setup& setup() const { return doc.project().setups.front(); }
    const Operation& op(const std::string& name) const {
        for (const Operation& o : setup().operations) {
            if (o.name == name) {
                return o;
            }
        }
        FAIL("no operation " << name);
        throw;
    }
    toolpath::Toolpath generate(const Operation& o) {
        return generateToolpath(doc, setup(), o, cancel, progress);
    }
    toolpath::Toolpath generate(const std::string& name, const Json& changes) {
        Operation o = op(name);
        for (const auto& [key, value] : changes.items()) {
            o.parameters[key] = value;
        }
        return generate(o);
    }
};

double distanceToLine(const Vec3& q, const Vec3& a, const Vec3& u) {
    const Vec3 d = q - a;
    return (d - d.dot(u) * u).norm();
}

double distanceToSegment(const Vec3& q, const Vec3& a, const Vec3& b) {
    const Vec3 ab = b - a;
    const double t = std::clamp((q - a).dot(ab) / ab.squaredNorm(), 0.0, 1.0);
    return (q - (a + t * ab)).norm();
}

/// The twisted boss's corners in the WCS: bottom (z = base top) and top (turned).
std::array<Vec3, 4> twistCorners(double z, double turn) {
    const double h = kTwistSide / 2;
    const std::array<std::pair<double, double>, 4> c{{{-h, -h}, {h, -h}, {h, h}, {-h, h}}};
    std::array<Vec3, 4> out;
    for (int i = 0; i < 4; ++i) {
        const auto [x, y] = c[static_cast<std::size_t>(i)];
        out[static_cast<std::size_t>(i)] =
            Vec3(kTwistX + x * std::cos(turn) - y * std::sin(turn),
                 kTwistY + x * std::sin(turn) + y * std::cos(turn), z) +
            kModelToWcs;
    }
    return out;
}

} // namespace

TEST_CASE("swarf lays the side of the tool along the tapered boss's cone",
          "[document][swarf][multiaxis]") {
    SwarfProject f;
    const toolpath::Toolpath tp = f.generate(f.op("Swarf taper"));
    REQUIRE(toolpath::isMultiAxis(tp));
    const Vec3 c = Vec3(kConeX, kConeY, 0.0) + kModelToWcs;
    const double slope = (kConeBottomRadius - kConeTopRadius) / kBossHeight;
    const double tilt = std::atan(slope);
    std::set<long long> heights; // tool end heights of the cuts, by pass
    int cuts = 0;
    double turned = 0.0; // the wall on the right of the feed (climb), summed
    Vec3 previous = tp.start;
    for (const toolpath::Move& m : tp.moves) {
        if (m.feedClass == toolpath::FeedClass::Cut) {
            ++cuts;
            CAPTURE(m.end.x(), m.end.y(), m.end.z());
            // The axis leans in towards the boss by the cone's half-angle.
            CHECK_THAT(std::acos(m.axis.z()), WithinAbs(tilt, 1e-3));
            const Vec3 radial = Vec3(m.end.x() - c.x(), m.end.y() - c.y(), 0.0).normalized();
            CHECK(m.axis.dot(radial) < -0.2);
            // Parallel to the cone's generator there, a tool radius away from it.
            const Vec3 bottom = c + Vec3(0, 0, kBaseHeight) + kConeBottomRadius * radial;
            const Vec3 top = c + Vec3(0, 0, kTop) + kConeTopRadius * radial;
            const Vec3 g = (top - bottom).normalized();
            CHECK(std::abs(g.dot(m.axis)) > std::cos(1e-3));
            CHECK_THAT(distanceToLine(bottom, m.end, m.axis), WithinAbs(kToolRadius, 2e-3));
            heights.insert(std::llround(m.end.z() * 10.0));
            const Vec3 feed = m.end - previous;
            turned += -radial.dot(feed.cross(m.axis));
        }
        previous = m.end;
    }
    CHECK(cuts > 200);
    CHECK(turned > 0.0); // climb: the boss on the right
    // Two depth passes: the last with the tool's lowest point 0.1 above the base plate (the
    // floor), the first half the wall higher along the axis.
    REQUIRE(heights.size() == 2);
    const double lowEnd = kBaseHeight + 0.1 + kToolRadius * std::sin(tilt);
    CHECK_THAT(*heights.begin() / 10.0, WithinAbs(lowEnd, 0.06));
    // Speeds and feeds exactly as the preset gives them (ADR-0004): one feed per class.
    std::set<std::pair<int, long long>> feeds;
    for (const toolpath::Move& m : tp.moves) {
        if (m.kind == toolpath::MoveKind::Linear) {
            feeds.insert({static_cast<int>(m.feedClass), std::llround(m.feedRate * 1e6)});
        }
    }
    std::set<int> classes;
    for (const auto& [cls, rate] : feeds) {
        CHECK(classes.insert(cls).second);
        CHECK(rate > 0);
    }
}

TEST_CASE("swarf on the twisted boss stays within the tolerance of its ruled sides",
          "[document][swarf][multiaxis]") {
    SwarfProject f;
    const toolpath::Toolpath tp = f.generate(f.op("Swarf twist"));
    REQUIRE(toolpath::isMultiAxis(tp));
    REQUIRE_FALSE(tp.warnings.empty());
    CHECK_THAT(tp.warnings.back(), ContainsSubstring("twisted"));
    // The four sides, each the bilinear surface between a bottom and a top edge.
    const auto b = twistCorners(kBaseHeight, 0.0);
    const auto t = twistCorners(kTop, kTwistAngle);
    std::vector<Vec3> wall;
    for (int i = 0; i < 4; ++i) {
        const auto j = static_cast<std::size_t>(i);
        const auto k = static_cast<std::size_t>((i + 1) % 4);
        for (int a = 0; a <= 140; ++a) {
            for (int h = 0; h <= 40; ++h) {
                const double s = a / 140.0, v = h / 40.0;
                wall.push_back((1 - v) * ((1 - s) * b[j] + s * b[k]) +
                               v * ((1 - s) * t[j] + s * t[k]));
            }
        }
    }
    const double tolerance = 0.02;
    int cuts = 0;
    double closest = 1e9, furthest = 0.0;
    for (const toolpath::Move& m : tp.moves) {
        if (m.feedClass != toolpath::FeedClass::Cut) {
            continue;
        }
        ++cuts;
        CAPTURE(m.end.x(), m.end.y(), m.end.z());
        double nearest = 1e9;
        for (const Vec3& q : wall) {
            nearest = std::min(nearest, distanceToSegment(q, m.end, m.end + 25.0 * m.axis));
        }
        // The flank never cuts into a side by more than the tolerance and touches it.
        CHECK(nearest > kToolRadius - tolerance - 1e-3);
        CHECK(nearest < kToolRadius + tolerance + 0.01);
        closest = std::min(closest, nearest);
        furthest = std::max(furthest, nearest);
    }
    CHECK(cuts > 40);
    // Ruled but twisted: the flank strays from the sides, by about r (1 - cos(twist / 2)).
    CHECK(closest < kToolRadius - 0.002);

    SECTION("a tolerance tighter than the twist refuses the walls") {
        CHECK_THROWS_WITH(f.generate("Swarf twist", Json{{"tolerance", 0.005}}),
                          ContainsSubstring("twist too much"));
    }
}

TEST_CASE("swarf refuses what the side of the tool cannot cut as set",
          "[document][swarf][multiaxis]") {
    SwarfProject f;
    SECTION("a wall leaning further than the largest tilt") {
        CHECK_THROWS_WITH(f.generate("Swarf taper", Json{{"max_tilt", 10.0}}),
                          ContainsSubstring("more than the largest tilt"));
    }
    SECTION("a floor and no wall") {
        Operation o = f.op("Swarf taper");
        o.geometry.erase(o.geometry.begin());
        CHECK_THROWS_WITH(f.generate(o), ContainsSubstring("taken as the floor"));
    }
    SECTION("without a floor the end follows the wall's bottom edge; past it is reported") {
        Operation o = f.op("Swarf taper");
        o.geometry.pop_back(); // no floor
        o.parameters["depth_passes"] = 1;
        o.parameters["axis_offset"] = 0.0;
        // A flat end square to the axis has its lowest point on the wall's bottom edge.
        const toolpath::Toolpath tp = f.generate(o);
        const double tilt = std::atan((kConeBottomRadius - kConeTopRadius) / kBossHeight);
        for (const toolpath::Move& m : tp.moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                CHECK_THAT(m.end.z() - kToolRadius * std::sin(tilt), WithinAbs(kBaseHeight, 1e-6));
            }
        }
        for (const std::string& w : tp.warnings) {
            CHECK_THAT(w, !ContainsSubstring("cuts into the part"));
        }
        // 1 mm past the edge, into the base plate: reported, not avoided.
        o.parameters["axis_offset"] = -1.0;
        const toolpath::Toolpath deeper = f.generate(o);
        bool reported = false;
        for (const std::string& w : deeper.warnings) {
            reported = reported || w.find("cuts into the part") != std::string::npos;
        }
        CHECK(reported);
    }
}

TEST_CASE("Z-level roughing with the tool tilted, resting on the earlier roughing",
          "[document][swarf][orientation]") {
    SwarfProject f;
    const Operation& o = f.op("Tilted rest rough");
    REQUIRE(o.orientation);
    const Vec3 axis = operationFrame(o).linear().col(2);
    CHECK_THAT(std::acos(axis.z()), WithinAbs(20.0 * kPi / 180.0, 1e-9));

    const toolpath::Toolpath inFrame =
        generateToolpathInFrame(f.doc, f.setup(), o, f.cancel, f.progress);
    const toolpath::Toolpath inWcs = toolpathInWcs(inFrame, o);
    REQUIRE_FALSE(inFrame.moves.empty());
    double restCut = 0.0;
    Vec3 at = inFrame.start;
    for (std::size_t k = 0; k < inFrame.moves.size(); ++k) {
        // A 3-axis toolpath in its frame, the frame's Z axis in the WCS.
        CHECK((inFrame.moves[k].axis - Vec3::UnitZ()).norm() < 1e-12);
        CHECK((inWcs.moves[k].axis - axis).norm() < 1e-9);
        if (inFrame.moves[k].feedClass == toolpath::FeedClass::Cut) {
            restCut += (inFrame.moves[k].end - at).norm();
        }
        at = inFrame.moves[k].end;
    }
    CHECK(restCut > 0.0);
    // The 3-axis roughing cleared the plate around the bosses: what is left for the tilted tool
    // is next to the bosses' walls (the stairs of the 3 mm levels, and what the 10 mm tool could
    // not reach), not out on the plate.
    const Vec3 cone = Vec3(kConeX, kConeY, 0.0) + kModelToWcs;
    const Vec3 twist = Vec3(kTwistX, kTwistY, 0.0) + kModelToWcs;
    double furthest = 0.0;
    at = inWcs.start;
    for (const toolpath::Move& m : inWcs.moves) {
        if (m.feedClass == toolpath::FeedClass::Cut) {
            const double toCone = (m.end - cone).head<2>().norm() - kConeBottomRadius;
            const double toTwist = (m.end - twist).head<2>().cwiseAbs().maxCoeff() - kTwistSide / 2;
            furthest = std::max(furthest, std::min(toCone, toTwist));
        }
        at = m.end;
    }
    CHECK(furthest < 8.0); // a tool diameter and a little from the walls
}
