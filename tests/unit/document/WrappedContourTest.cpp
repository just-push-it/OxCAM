// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A wrapped contour (DL-0084) on a lathe with a live tool and on a 4th axis: what it follows,
// the simulation, and the programs the posts write for it.

#include "FixtureSpecs.hpp"

#include <ocam/cad/SubShape.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/post/Engine.hpp>
#include <ocam/post/Post.hpp>
#include <ocam/sim/Compare.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <fmt/format.h>

#include <cmath>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

using namespace ocam;
using namespace ocam::document;
using Catch::Matchers::ContainsSubstring;

namespace {

/// A fixture project whose operation `index` becomes a wrapped contour.
struct Wrapping {
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    std::size_t index;

    Wrapping(const char* file, std::size_t op)
        : doc(Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / file)), index(op) {}

    const Setup& setup() const { return doc.project().setups.front(); }
    const cad::Model& model() const { return *doc.model(*setup().modelId); }
    Transform3 toWcs() const {
        return modelToWcs(*doc.project().findModel(*setup().modelId), setup());
    }
    /// The first face or edge whose geometry (in the WCS) `accepts`.
    ShapeRef find(
        cad::SubShapeType type,
        const std::function<bool(const cad::SubShapeGeometry&, const Transform3&)>& accepts) const {
        for (int i = 1; i <= cad::subShapeCount(model(), type); ++i) {
            if (accepts(cad::subShapeGeometry(model(), type, i), toWcs())) {
                return {*setup().modelId, type, i, cad::signatureOf(model(), type, i)};
            }
        }
        FAIL("no such face or edge");
        return {};
    }
    /// The cylindrical faces of radius `radius`: a floor may come in more than one.
    std::vector<ShapeRef> cylinder(double radius) const {
        std::vector<ShapeRef> out;
        for (int i = 1; i <= cad::subShapeCount(model(), cad::SubShapeType::Face); ++i) {
            const cad::SubShapeGeometry g =
                cad::subShapeGeometry(model(), cad::SubShapeType::Face, i);
            if (g.kind == cad::SubShapeGeometry::Kind::Cylinder &&
                std::abs(g.radius - radius) < 1e-6) {
                out.push_back({*setup().modelId, cad::SubShapeType::Face, i,
                               cad::signatureOf(model(), cad::SubShapeType::Face, i)});
            }
        }
        REQUIRE_FALSE(out.empty());
        return out;
    }
    void contour(std::vector<ShapeRef> geometry, Json parameters) {
        doc.modify("Wrapped contour", [&](Project& p) {
            Operation& o = p.setups.front().operations[index];
            o.type = "rotary_contour";
            o.name = "Wrapped contour";
            o.geometry = std::move(geometry);
            o.parameters = std::move(parameters);
        });
    }
    toolpath::Toolpath generate() {
        return generateToolpath(doc, setup(), setup().operations[index], cancel, progress);
    }
    PostOutput post(const post::PostScript& script, const Json& options = Json::object()) {
        const machine::Machine& m = machineFor(doc.project(), setup());
        return postSetup(doc, setup().id, m, script, cancel, progress, options);
    }
};

void matchesBackplot(const PostOutput& out, const machine::Machine& machine, bool lathe) {
    post::BackplotOptions options;
    options.lathe = lathe;
    options.machine = &machine;
    const auto differs = post::compareBackplot(
        out.toolpaths, post::backplot(out.gcode, options).path, 2e-3, nullptr, !lathe);
    INFO(differs.value_or(""));
    CHECK_FALSE(differs.has_value());
}

} // namespace

TEST_CASE("a wrapped contour around a lathe's diameter", "[document][lathe][live][mill][wrap]") {
    using namespace fixtures::wrapped;
    Wrapping f("wrapped_project.ocam", 3); // the slot's wrapped pocket, now a contour
    const double floor = kDiameter / 2.0 - kSlotDepth;
    const double r = 3.0; // the mill's radius
    // WCS Z is 0 at the part's front: z from the back is WCS z + the length.
    const auto fromBack = [](const Vec3& p) {
        return p.z() + kLength;
    };
    const auto floorPoints = [&](const toolpath::Toolpath& tp) {
        std::vector<Vec3> out;
        for (const toolpath::Move& m : tp.moves) {
            if (m.kind == toolpath::MoveKind::Linear && m.feedClass == toolpath::FeedClass::Cut &&
                std::abs(m.end.head<2>().norm() - floor) < 1e-6) {
                out.push_back(m.end);
            }
        }
        return out;
    };

    SECTION("the slot's floor: inside its walls, to the floor, the tool radial") {
        f.contour(f.cylinder(floor), Json::object());
        const toolpath::Toolpath tp = f.generate();
        CHECK(tp.strategy == "rotary_contour");
        CHECK(tp.live);
        CHECK(tp.liveAxes == toolpath::LiveAxes::Wrap);
        REQUIRE_FALSE(tp.moves.empty());
        double deepest = 1e9;
        for (const toolpath::Move& m : tp.moves) {
            deepest = std::min(deepest, m.end.head<2>().norm());
            CHECK((m.axis - Vec3(m.end.x(), m.end.y(), 0.0).normalized()).norm() < 1e-6);
        }
        CHECK(deepest == Catch::Approx(floor).margin(1e-6));
        // The walls the tool's radius away along the arc at the floor, all four reached.
        const double reach = kSlotAngle / 2.0 - r / floor;
        double lo = 1e9, hi = -1e9, front = -1e9, back = 1e9;
        const auto points = floorPoints(tp);
        REQUIRE_FALSE(points.empty());
        for (const Vec3& p : points) {
            const double t = std::atan2(p.y(), p.x());
            CHECK(std::abs(t) <= reach + 1e-3 / floor);
            CHECK(fromBack(p) >= kSlotZ0 + r - 1e-3);
            CHECK(fromBack(p) <= kSlotZ1 - r + 1e-3);
            lo = std::min(lo, t);
            hi = std::max(hi, t);
            back = std::min(back, fromBack(p));
            front = std::max(front, fromBack(p));
        }
        CHECK(lo == Catch::Approx(-reach).margin(1e-3 / floor));
        CHECK(hi == Catch::Approx(reach).margin(1e-3 / floor));
        CHECK(back == Catch::Approx(kSlotZ0 + r).margin(1e-3));
        CHECK(front == Catch::Approx(kSlotZ1 - r).margin(1e-3));
    }

    SECTION("a keyway along Z: on the foot of a wall, C held") {
        const double half = kSlotAngle / 2.0;
        const ShapeRef foot = f.find(cad::SubShapeType::Edge,
                                     [&](const cad::SubShapeGeometry& g, const Transform3& toWcs) {
                                         const Vec3 p = toWcs * g.point;
                                         const Vec3 d = toWcs.linear() * g.direction;
                                         return g.kind == cad::SubShapeGeometry::Kind::Line &&
                                                std::abs(std::abs(d.z()) - 1.0) < 1e-9 &&
                                                std::abs(p.head<2>().norm() - floor) < 1e-6 &&
                                                std::abs(std::atan2(p.y(), p.x()) - half) < 1e-6;
                                     });
        f.contour({foot}, Json{{"side", "on"}});
        const toolpath::Toolpath tp = f.generate();
        const auto points = floorPoints(tp);
        REQUIRE_FALSE(points.empty());
        for (const Vec3& p : points) {
            CHECK(std::atan2(p.y(), p.x()) == Catch::Approx(half).margin(1e-9));
            CHECK(fromBack(p) >= kSlotZ0 - 1e-6);
            CHECK(fromBack(p) <= kSlotZ1 + 1e-6);
        }
        // From the lower Z, the chuck's side, along Z only.
        CHECK(fromBack(points.back()) == Catch::Approx(kSlotZ1).margin(1e-6));
    }

    SECTION("the simulation cuts the slot, and nothing else") {
        f.contour(f.cylinder(floor), Json::object());
        const SetupSimulation sim = simulateSetup(f.doc, f.setup().id, {}, f.cancel, f.progress);
        REQUIRE(sim.operations.size() == 4);
        for (const auto& op : sim.operations) {
            INFO(op.name << ": " << op.error);
            CHECK(op.error.empty());
            CHECK(op.collisions.empty());
        }
        const double R = kDiameter / 2.0;
        const double slot = (R * R - floor * floor) / 2.0 * kSlotAngle * (kSlotZ1 - kSlotZ0);
        // Less only the corners a round tool leaves.
        CHECK(sim.operations[3].removedVolume == Catch::Approx(slot).epsilon(0.05));
        REQUIRE(sim.comparison);
        CHECK_FALSE(sim.comparison->hasGouges());
    }

    SECTION("Haas and Fanuc: cylindrical interpolation on each level's radius, the feed as set") {
        f.contour(f.cylinder(floor), Json::object());
        const toolpath::Toolpath tp = f.generate();
        const machine::Machine& m = machineFor(f.doc.project(), f.setup());
        {
            const PostOutput out = f.post(post::builtinPost("haas-lathe"));
            INFO((out.errors.empty() ? std::string() : out.errors.front()));
            REQUIRE(out.ok());
            INFO(out.gcode);
            CHECK_THAT(out.gcode, ContainsSubstring(fmt::format("G107 Y0 C0 R{:.0f}.", floor)));
            for (const toolpath::Move& mv : tp.moves) {
                if (mv.kind == toolpath::MoveKind::Linear &&
                    mv.feedClass == toolpath::FeedClass::Cut) {
                    CHECK_THAT(out.gcode,
                               ContainsSubstring(fmt::format("F{:.0f}", mv.feedRate * 60.0)));
                    break;
                }
            }
            matchesBackplot(out, m, true);
        }
        {
            const Json options{{"program",
                                {{"liveToolOn", "M133 P{rpm}"},
                                 {"liveToolOff", "M135"},
                                 {"cAxisOn", "M154"},
                                 {"cAxisOff", "M155"}}}};
            const PostOutput out = f.post(post::builtinPost("fanuc-generic-lathe"), options);
            INFO((out.errors.empty() ? std::string() : out.errors.front()));
            REQUIRE(out.ok());
            INFO(out.gcode);
            CHECK_THAT(out.gcode, ContainsSubstring(fmt::format("G7.1 C{:.0f}.", floor)));
            CHECK_THAT(out.gcode, ContainsSubstring("G7.1 C0"));
            matchesBackplot(out, m, true);
        }
    }

    SECTION("without live tooling it is refused") {
        f.contour(f.cylinder(floor), Json::object());
        f.doc.modify("m", [&](Project& p) { p.setups.front().machine = "generic-2axis-lathe"; });
        CHECK_THROWS_WITH(f.generate(), ContainsSubstring("no live tooling"));
    }
    SECTION("nothing selected is refused, saying what to select") {
        f.contour({}, Json::object());
        CHECK_THROWS_WITH(f.generate(), ContainsSubstring("select the edges to follow"));
    }
}

TEST_CASE("a wrapped contour on a 4th axis", "[document][mill][wrap]") {
    using namespace fixtures::rotary;
    Wrapping f("rotary_project.ocam", 0); // the wrapped grooves, now contours
    const double R = kDiameter / 2.0;
    const double ringFloor = R - kRingDepth, grooveFloor = R - kGrooveDepth;
    std::vector<ShapeRef> floors = f.cylinder(ringFloor);
    for (const ShapeRef& g : f.cylinder(grooveFloor)) {
        floors.push_back(g);
    }
    f.contour(floors, Json::object());
    const toolpath::Toolpath tp = f.generate();
    REQUIRE_FALSE(tp.moves.empty());
    double deepest = 1e9;
    bool ringNear = false, ringFar = false;
    for (const toolpath::Move& m : tp.moves) {
        const double rho = std::hypot(m.end.y(), m.end.z());
        deepest = std::min(deepest, rho);
        CHECK(std::abs(m.axis.x()) < 1e-9); // radial about X
        if (m.kind == toolpath::MoveKind::Linear && m.feedClass == toolpath::FeedClass::Cut &&
            std::abs(rho - ringFloor) < 1e-6) {
            // The ring groove, 8 wide for a 6 mm mill: a pass by each wall, all the way round.
            CHECK(m.end.x() >= kRingX0 + 3.0 - 1e-3);
            CHECK(m.end.x() <= kRingX1 - 3.0 + 1e-3);
            ringNear = ringNear || std::abs(m.end.x() - (kRingX0 + 3.0)) < 1e-3;
            ringFar = ringFar || std::abs(m.end.x() - (kRingX1 - 3.0)) < 1e-3;
        }
    }
    CHECK(deepest == Catch::Approx(grooveFloor).margin(1e-6));
    CHECK(ringNear);
    CHECK(ringFar);

    SECTION("the 4th-axis post: A words, inverse time where A turns, the back-plot agrees") {
        const machine::Machine& m = machineFor(f.doc.project(), f.setup());
        const PostOutput out = f.post(defaultPostFor(m));
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        CHECK_THAT(out.gcode, ContainsSubstring(" A"));
        CHECK_THAT(out.gcode, ContainsSubstring("G93"));
        matchesBackplot(out, m, false);
    }
    SECTION("a machine with no rotary axis refuses it") {
        f.doc.modify("m", [&](Project& p) { p.setups.front().machine = "generic-3axis-vmc"; });
        CHECK_THROWS_WITH(f.generate(), ContainsSubstring("no rotary axis"));
    }
}
