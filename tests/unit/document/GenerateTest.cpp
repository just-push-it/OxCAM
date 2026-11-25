// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "FixtureSpecs.hpp"
#include "TestProject.hpp"

#include <ocam/cad/Features.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/ProjectIo.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/document/ToolpathCache.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/sim/Dexel.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <set>
#include <thread>

using namespace ocam;
using namespace ocam::document;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

struct Fixture {
    Document doc;
    Uuid modelId;
    Uuid setupId;
    Uuid toolId;
    CancelToken cancel;
    NullProgress progress;

    Uuid presetId;

    Fixture(const char* step, tooling::Tool tool) {
        tool = testing::withPreset(std::move(tool));
        modelId = doc.importModel(std::filesystem::path(OCAM_FIXTURE_DIR) / step, cancel, progress);
        setupId = doc.addSetup(modelId);
        toolId = tool.id;
        presetId = tool.presets.front().id;
        doc.modify("tool", [&](Project& p) { p.tools.tools.push_back(tool); });
    }

    const Setup& setup() const { return *doc.project().findSetup(setupId); }

    Uuid addOperation(const std::string& type, std::vector<ShapeRef> geometry = {},
                      Json params = Json::object(), PresetOverrides overrides = {}) {
        Operation op;
        op.name = type;
        op.type = type;
        op.toolId = toolId;
        op.presetId = presetId;
        op.geometry = std::move(geometry);
        op.parameters = std::move(params);
        op.presetOverrides = overrides;
        doc.modify("op", [&](Project& p) { p.findSetup(setupId)->operations.push_back(op); });
        return op.id;
    }

    toolpath::Toolpath generate(const Uuid& opId) {
        const Setup* owner = nullptr;
        const Operation* op = doc.project().findOperation(opId, &owner);
        return generateToolpath(doc, *owner, *op, cancel, progress);
    }

    ShapeRef faceAtHeight(double height) const {
        const cad::Model& model = *doc.model(modelId);
        ShapeRef ref;
        for (const auto& f : cad::findPlanarFaces(model)) {
            if (std::abs(f.height - height) < 1e-6) {
                ref = {modelId, cad::SubShapeType::Face, f.face,
                       cad::signatureOf(model, cad::SubShapeType::Face, f.face)};
            }
        }
        REQUIRE(ref.index > 0);
        return ref;
    }
};

} // namespace

TEST_CASE("generate facing, pocket and contour on the block fixture", "[document][generate]") {
    using namespace fixtures::block_pocket;
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    // Default setup: stock 1 mm proud on sides and top; WCS origin at the stock's top corner.
    const Vec3 wcsOrigin = f.setup().wcs.origin;
    CHECK((wcsOrigin - Vec3(-1, -1, kHeight + 1)).norm() < 1e-6);

    SECTION("facing removes the 1 mm on top") {
        const auto tp = f.generate(f.addOperation("facing"));
        const auto stats = toolpath::computeStats(tp);
        CHECK_THAT(stats.bounds.min.z(), WithinAbs(-1.0, 1e-6));
        CHECK(tp.warnings.empty());
        CHECK(tp.operationId != Uuid());
    }
    SECTION("pocket on the selected floor face") {
        const auto tp =
            f.generate(f.addOperation("pocket2d", {f.faceAtHeight(kHeight - kPocketDepth)}));
        const auto stats = toolpath::computeStats(tp);
        const double floorZ = (kHeight - kPocketDepth) - (kHeight + 1);
        CHECK_THAT(stats.bounds.min.z(), WithinAbs(floorZ, 1e-6));
        // Tool centre stays one radius inside the pocket walls (pocket at x 30..70 in the model).
        double minX = 1e9, maxX = -1e9;
        for (const auto& run : toolpath::sampleRuns(tp)) {
            for (const Vec3& q : run.points) {
                if (q.z() < -1e-6) {
                    minX = std::min(minX, q.x());
                    maxX = std::max(maxX, q.x());
                }
            }
        }
        CHECK(minX >= kPocketX + 1 + 3 - 1e-3);
        CHECK(maxX <= kPocketX + kPocketLength + 1 - 3 + 1e-3);
    }
    SECTION("flat area finishing cuts the top and the pocket floor") {
        const auto tp = f.generate(f.addOperation("flat3d"));
        CHECK(tp.warnings.empty());
        std::set<double> levels;
        for (const auto& m : tp.moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                levels.insert(std::round(m.end.z() * 1000.0) / 1000.0);
            }
        }
        const double floorZ = (kHeight - kPocketDepth) - (kHeight + 1);
        CHECK(levels == std::set<double>{floorZ, -1.0});
        // On the floor the tool stays inside the pocket.
        for (const auto& run : toolpath::sampleRuns(tp)) {
            for (const Vec3& q : run.points) {
                if (q.z() < floorZ + 1e-6) {
                    CHECK(q.x() >= kPocketX + 1 + 3 - 0.3);
                    CHECK(q.x() <= kPocketX + kPocketLength + 1 - 3 + 0.3);
                }
            }
        }
    }
    SECTION("outside contour around the bottom face") {
        const auto tp = f.generate(f.addOperation("contour2d", {f.faceAtHeight(0.0)},
                                                  Json::object(), testing::stepDown(10.0)));
        const auto stats = toolpath::computeStats(tp);
        CHECK_THAT(stats.bounds.min.z(), WithinAbs(-(kHeight + 1), 1e-6));
        CHECK(stats.bounds.max.x() > kLength + 1 + 3 - 1e-3); // outside the part
    }
    SECTION("errors are reported clearly") {
        CHECK_THROWS_WITH(f.generate(f.addOperation("pocket2d")),
                          ContainsSubstring("select the face"));
        CHECK_THROWS_WITH(f.generate(f.addOperation("pocket2d", {f.faceAtHeight(kHeight)},
                                                    Json{{"stock_to_leave_xy", 500}})),
                          ContainsSubstring("stock_to_leave_xy"));
        const Uuid noTool = f.addOperation("facing");
        f.doc.modify("clear tool", [&](Project& p) { p.findOperation(noTool)->toolId.reset(); });
        CHECK_THROWS_WITH(f.generate(noTool), ContainsSubstring("choose a tool"));
        CHECK_THROWS_WITH(f.generate(f.addOperation("laser")),
                          ContainsSubstring("unknown operation type"));
    }
}

TEST_CASE("generate drilling from recognized holes", "[document][generate]") {
    using namespace fixtures::hole_plate;
    Fixture f("hole_plate.step",
              tooling::makeDrill(1, kThroughDiameter, degToRad(118.0), 40.0, 90.0));
    const auto tp = f.generate(f.addOperation("drill"));
    REQUIRE(tp.cycles.size() == 1);
    // The Ø10 through hole at (20, 20); WCS origin at (-1, -1, 21).
    const auto cycleMove =
        std::find_if(tp.moves.begin(), tp.moves.end(),
                     [](const toolpath::Move& m) { return m.kind == toolpath::MoveKind::Cycle; });
    REQUIRE(cycleMove != tp.moves.end());
    CHECK_THAT(cycleMove->end.x(), WithinAbs(kThroughX + 1, 1e-6));
    CHECK_THAT(cycleMove->end.y(), WithinAbs(kThroughY + 1, 1e-6));
    CHECK_THAT(tp.cycles[0].topZ, WithinAbs(-1.0, 1e-6));
    CHECK(tp.cycles[0].bottomZ < -1.0 - kThickness - 1.0); // breakthrough plus tip

    // Selecting the drill-point hole's wall drills that hole instead, whatever its size.
    const cad::Model& model = *f.doc.model(f.modelId);
    for (const auto& hole : cad::findHoles(model)) {
        if (hole.bottom == cad::HoleBottom::Cone) {
            const ShapeRef wall{
                f.modelId, cad::SubShapeType::Face, hole.faces.front(),
                cad::signatureOf(model, cad::SubShapeType::Face, hole.faces.front())};
            const auto picked = f.generate(f.addOperation("drill", {wall}));
            REQUIRE(picked.cycles.size() == 1);
            CHECK_THAT(picked.cycles[0].topZ, WithinAbs(-1.0, 1e-6));
            // By default the R plane is 2 mm above the hole's own top.
            CHECK_THAT(picked.cycles[0].retractZ, WithinAbs(-1.0 + 2.0, 1e-6));
        }
    }

    // A feed height is the cycles' R plane.
    const Uuid withFeed = f.addOperation("drill");
    f.doc.modify("feed", [&](Project& p) {
        p.findOperation(withFeed)->heights.feed = {HeightRef::StockTop, 4.0};
    });
    const auto rPlane = f.generate(withFeed);
    REQUIRE(rPlane.cycles.size() == 1);
    CHECK_THAT(rPlane.cycles[0].retractZ, WithinAbs(4.0, 1e-9));

    // The simulation plays the cycles as moves: the drill feeds down to the hole bottom.
    SimulationOptions options;
    options.resolution = 0.5;
    options.compare = false;
    const SetupSimulation sim = simulateSetup(f.doc, f.setupId, options, f.cancel, f.progress);
    REQUIRE_FALSE(sim.operations.empty());
    const auto& moves = sim.operations.front().toolpath.moves;
    CHECK(std::none_of(moves.begin(), moves.end(), [](const toolpath::Move& m) {
        return m.kind == toolpath::MoveKind::Cycle;
    }));
    CHECK(std::any_of(moves.begin(), moves.end(), [&](const toolpath::Move& m) {
        return m.kind == toolpath::MoveKind::Linear && m.end.z() < -1.0 - kThickness;
    }));
}

namespace {

/// Material of a ray between z `lo` and `hi`.
double materialIn(const sim::IntervalList& ray, double lo, double hi) {
    double sum = 0.0;
    for (const sim::Interval& in : ray) {
        sum += std::max(0.0, std::min(static_cast<double>(in.hi), hi) -
                                 std::max(static_cast<double>(in.lo), lo));
    }
    return sum;
}

} // namespace

TEST_CASE("thread milling around a boss", "[document][generate][threadmill]") {
    using namespace fixtures::boss_plate;
    tooling::Tool mill = tooling::makeDefaultTool(tooling::ToolType::ThreadMill, 1, 6.0);
    mill.mill.threadPitch = 1.5;
    mill.mill.fluteLength = 20.0;
    Fixture f("boss_plate.step", mill);
    constexpr double kThreadLength = 10.0, kPitch = 1.5;
    const double depth = 0.6134 * kPitch;
    const double radius = kBossDiameter / 2.0 - depth + 3.0;

    // The helix of each: its centre, radius and z range; right hand, climb: clockwise, down.
    struct Helix {
        Vec2 centre = Vec2::Zero();
        double lo = 1e9, hi = -1e9;
        int arcs = 0;
    };
    const auto helixOf = [&](const toolpath::Toolpath& tp) {
        Helix h;
        Vec3 pos = tp.start;
        for (const auto& m : tp.moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                REQUIRE(m.kind == toolpath::MoveKind::ArcCW);
                CHECK_THAT((m.end.head<2>() - m.center.head<2>()).norm(), WithinAbs(radius, 1e-9));
                CHECK(m.end.z() < pos.z());
                h.centre = m.center.head<2>();
                h.lo = std::min({h.lo, pos.z(), m.end.z()});
                h.hi = std::max({h.hi, pos.z(), m.end.z()});
                ++h.arcs;
            }
            pos = m.end;
        }
        return h;
    };

    // Found by its diameter, the major diameter's.
    const Uuid found = f.addOperation("threadmill", {},
                                      Json{{"side", "external"},
                                           {"major_diameter", kBossDiameter},
                                           {"thread_depth", kThreadLength}});
    const toolpath::Toolpath tp = f.generate(found);
    INFO((tp.warnings.empty() ? std::string() : tp.warnings.front()));
    CHECK(tp.warnings.empty());
    const Helix h = helixOf(tp);
    CHECK(h.arcs == 2 * 7); // ceil(10 / 1.5) turns
    CHECK_THAT(h.hi - h.lo, WithinAbs(7 * kPitch, 1e-9));

    // A selected boss face: the major diameter is the boss's own.
    const cad::Model& model = *f.doc.model(f.modelId);
    const auto bosses = cad::findBosses(model);
    REQUIRE(bosses.size() == 1);
    const int face = bosses.front().faces.front();
    const ShapeRef ref{f.modelId, cad::SubShapeType::Face, face,
                       cad::signatureOf(model, cad::SubShapeType::Face, face)};
    const Helix selected =
        helixOf(f.generate(f.addOperation("threadmill", {ref}, Json{{"side", "external"}})));
    CHECK((selected.centre - h.centre).norm() < 1e-9);
    // Its whole length, down to the plate.
    CHECK_THAT(selected.lo, WithinAbs(h.lo - (kBossHeight - kThreadLength), 1e-9));

    // Internal thread milling does not take a boss's face, nor external a hole's.
    const auto internal = f.generate(f.addOperation("threadmill", {ref}));
    CHECK(helixOf(internal).arcs == 0);
    CHECK_THAT(internal.warnings.front(), ContainsSubstring("belongs to a hole"));
    const auto none = f.generate(f.addOperation("threadmill", {}, Json{{"side", "external"}}));
    CHECK_THAT(none.warnings.front(), ContainsSubstring("select the boss faces"));

    // The simulation shows the cut: the thread's groove between the major and minor diameters,
    // down the thread's length, and nothing inside the minor diameter.
    SimulationOptions options;
    options.resolution = 0.2;
    options.triDexel = false;
    options.compare = false;
    options.lastOperation = found;
    const SetupSimulation sim = simulateSetup(f.doc, f.setupId, options, f.cancel, f.progress);
    REQUIRE(sim.operations.size() == 1);
    CHECK(sim.operations[0].error.empty());
    const sim::DexelGrid& before = sim.stockBefore(0).grid(2);
    const sim::DexelGrid& after = sim.operations[0].stockAfter.grid(2);
    const double top = h.lo + kThreadLength; // the boss's top
    const auto ray = [&](const sim::DexelGrid& g, double r,
                         double& at) -> const sim::IntervalList& {
        const Vec2 p = h.centre + Vec2(r, 0.0);
        const int i = static_cast<int>(std::floor((p.x() - g.u0()) / g.spacing()));
        const int j = static_cast<int>(std::floor((p.y() - g.v0()) / g.spacing()));
        at = (Vec2(g.u0() + (i + 0.5) * g.spacing(), g.v0() + (j + 0.5) * g.spacing()) - h.centre)
                 .norm();
        return g.ray(i, j);
    };
    double at = 0.0;
    const double inThread = kBossDiameter / 2.0 - 0.4;
    const double had = materialIn(ray(before, inThread, at), h.lo + 0.5, top - 0.5);
    CHECK(at > kBossDiameter / 2.0 - depth + 0.1);
    CHECK(at < kBossDiameter / 2.0 - 0.1);
    CHECK_THAT(had, WithinAbs(kThreadLength - 1.0, 0.01));
    CHECK(materialIn(ray(after, inThread, at), h.lo + 0.5, top - 0.5) < 0.8 * had);
    const double core = kBossDiameter / 2.0 - depth - 0.5;
    CHECK_THAT(materialIn(ray(after, core, at), h.lo + 0.5, top - 0.5),
               WithinAbs(materialIn(ray(before, core, at), h.lo + 0.5, top - 0.5), 1e-6));
    CHECK(sim.operations[0].removedVolume > 0.0);
}

TEST_CASE("toolpath input keys track relevant changes only", "[document][generate]") {
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    const Uuid op = f.addOperation("facing");
    auto key = [&] {
        const Setup* owner = nullptr;
        const Operation* o = f.doc.project().findOperation(op, &owner);
        return toolpathInputKey(f.doc, *owner, *o);
    };
    const std::string k0 = key();
    f.addOperation("drill"); // another operation: no effect
    CHECK(key() == k0);
    f.doc.modify("param",
                 [&](Project& p) { p.findOperation(op)->presetOverrides.radialDepth = 2.0; });
    const std::string k1 = key();
    CHECK(k1 != k0);
    f.doc.modify("tool", [&](Project& p) { p.tools.tools[0].mill.diameter = 8.0; });
    CHECK(key() != k1);
}

TEST_CASE("feeds are output exactly as set, zero is refused", "[document][generate][feeds]") {
    using namespace fixtures::block_pocket;
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 10.0, 22.0, 72.0));
    const Uuid op = f.addOperation("plunge_rough", {f.faceAtHeight(kHeight - kPocketDepth)});
    const tooling::Tool& tool = f.doc.project().tools.tools.front();
    const tooling::CuttingPreset& preset = tool.presets.front();
    const double cut = preset.feedRate(tool.mill.diameter, tool.mill.flutes);

    // Every feed move runs at a value the preset shows: the feed rate or the plunge feed.
    const toolpath::Toolpath tp = f.generate(op);
    bool plunged = false;
    for (const auto& m : tp.moves) {
        if (m.kind == toolpath::MoveKind::Rapid) {
            continue;
        }
        if (m.feedClass == toolpath::FeedClass::Plunge) {
            plunged = true;
            CHECK(m.feedRate == preset.plungeFeed);
        } else {
            CHECK(m.feedRate == cut);
        }
    }
    CHECK(plunged);
    CHECK(tp.spindleSpeed == preset.spindleSpeed(tool.mill.diameter));

    // A plunge feed of 0 is not replaced by anything: the toolpath is refused, naming it.
    f.doc.modify("no plunge feed",
                 [](Project& p) { p.tools.tools.front().presets.front().plungeFeed = 0.0; });
    CHECK_THROWS_WITH(f.generate(op), ContainsSubstring("plunge feed"));
    // Set for this operation only, it is used as set.
    f.doc.modify("plunge here",
                 [&](Project& p) { p.findOperation(op)->presetOverrides.plungeFeed = 2.5; });
    for (const auto& m : f.generate(op).moves) {
        if (m.feedClass == toolpath::FeedClass::Plunge && m.kind != toolpath::MoveKind::Rapid) {
            CHECK(m.feedRate == 2.5);
        }
    }
}

TEST_CASE("ramps and helixes feed at the ramp feed", "[document][generate][feeds]") {
    using namespace fixtures::block_pocket;
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    f.doc.modify("ramp feed",
                 [](Project& p) { p.tools.tools.front().presets.front().rampFeed = 7.0; });
    // The pocket enters each level by a helix.
    const Uuid op = f.addOperation("pocket2d", {f.faceAtHeight(kHeight - kPocketDepth)});
    bool ramped = false;
    for (const auto& m : f.generate(op).moves) {
        if (m.feedClass == toolpath::FeedClass::Ramp) {
            ramped = true;
            CHECK(m.feedRate == 7.0);
        }
    }
    CHECK(ramped);
    // The operation's own value.
    f.doc.modify("own", [&](Project& p) { p.findOperation(op)->presetOverrides.rampFeed = 3.0; });
    for (const auto& m : f.generate(op).moves) {
        if (m.feedClass == toolpath::FeedClass::Ramp) {
            CHECK(m.feedRate == 3.0);
        }
    }
    // Zero is refused, naming the ramp feed.
    f.doc.modify("none", [&](Project& p) {
        p.findOperation(op)->presetOverrides.rampFeed.reset();
        p.tools.tools.front().presets.front().rampFeed = 0.0;
    });
    CHECK_THROWS_WITH(f.generate(op), ContainsSubstring("ramp feed"));
}

TEST_CASE("lead moves feed at the lead feed", "[document][generate][feeds]") {
    using namespace fixtures::block_pocket;
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    f.doc.modify("lead feed",
                 [](Project& p) { p.tools.tools.front().presets.front().leadFeed = 6.0; });
    // A contour along the pocket's walls leads on and off them.
    ShapeRef floor = f.faceAtHeight(kHeight - kPocketDepth);
    floor.side = GeometrySide::Inside;
    const Uuid op = f.addOperation("contour2d", {floor});
    const auto leads = [&] {
        std::vector<double> rates;
        for (const auto& m : f.generate(op).moves) {
            if (m.feedClass == toolpath::FeedClass::LeadIn ||
                m.feedClass == toolpath::FeedClass::LeadOut) {
                rates.push_back(m.feedRate);
            }
        }
        return rates;
    };
    const auto preset = leads();
    REQUIRE_FALSE(preset.empty());
    CHECK(std::all_of(preset.begin(), preset.end(), [](double v) { return v == 6.0; }));
    // The operation's own value.
    f.doc.modify("own", [&](Project& p) { p.findOperation(op)->presetOverrides.leadFeed = 2.5; });
    const auto own = leads();
    CHECK(std::all_of(own.begin(), own.end(), [](double v) { return v == 2.5; }));
    // Zero is refused, naming the lead feed.
    f.doc.modify("none", [&](Project& p) {
        p.findOperation(op)->presetOverrides.leadFeed.reset();
        p.tools.tools.front().presets.front().leadFeed = 0.0;
    });
    CHECK_THROWS_WITH(f.generate(op), ContainsSubstring("lead feed"));
}

TEST_CASE("toolpaths keep clear of the setup's fixtures", "[document][generate][fixtures]") {
    using namespace fixtures;
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 10.0, 22.0, 72.0));
    const Uuid op = f.addOperation("facing");
    const toolpath::Toolpath before = f.generate(op);
    CHECK(before.warnings.empty());

    // A stepped pin (the shaft: Ø30 to 50 mm, then Ø20 to 80) standing through the block.
    const Uuid pin = f.doc.addImportedFixture(
        importModelFile(std::filesystem::path(OCAM_FIXTURE_DIR) / "shaft.step", f.cancel,
                        f.progress),
        f.setupId);
    CHECK(f.setup().fixtures == std::vector<Uuid>{pin});
    CHECK(f.doc.undoLabel() == "Import fixture shaft");
    f.doc.modify("place", [&](Project& p) {
        for (auto& m : p.models) {
            if (m.id == pin) {
                m.placement.origin = Vec3(50, 30, 0);
            }
        }
    });
    auto key = [&] {
        const Setup* owner = nullptr;
        const Operation* o = f.doc.project().findOperation(op, &owner);
        return toolpathInputKey(f.doc, *owner, *o);
    };
    const std::string k0 = key();

    const toolpath::Toolpath tp = f.generate(op);
    REQUIRE_FALSE(tp.warnings.empty());
    CHECK_THAT(tp.warnings.front(), ContainsSubstring("fixtures"));
    // WCS: the pin's axis at (51, 31); its wide part ends at z = 50 - 31 = 19, its top at 49.
    const Vec2 axis(51, 31);
    bool cutNearby = false;
    for (const auto& run : toolpath::sampleRuns(tp, 0.01)) {
        for (std::size_t k = 0; k + 1 < run.points.size(); ++k) {
            for (int i = 0; i <= 20; ++i) {
                const Vec3 q = run.points[k] + (run.points[k + 1] - run.points[k]) * (i / 20.0);
                const double d = (q.head<2>() - axis).norm();
                if (d < 14.9) {
                    CHECK(q.z() >= 50.0 - 1e-3);
                } else if (d < 19.9) {
                    CHECK(q.z() >= 20.0 - 1e-3);
                }
                cutNearby = cutNearby || (d < 25.0 && run.feedClass == toolpath::FeedClass::Cut);
            }
        }
    }
    CHECK(cutNearby); // the face around the pin is still cut

    // The clearance and the holder are inputs now.
    f.doc.modify("clearance", [&](Project& p) { p.findSetup(f.setupId)->fixtureClearance = 2.0; });
    const std::string k1 = key();
    CHECK(k1 != k0);
    tooling::Holder holder = tooling::makeHolder("chuck", {});
    f.doc.modify("holder", [&](Project& p) {
        p.tools.holders.push_back(holder);
        p.tools.tools[0].holderId = holder.id;
    });
    CHECK(key() != k1);

    // Removing the model removes it from the setup.
    f.doc.removeModel(pin);
    CHECK(f.setup().fixtures.empty());
    CHECK(f.doc.project().validate().empty());
    CHECK(f.generate(op).warnings.empty());
}

TEST_CASE("generate tapping, thread milling, boring and chamfers", "[document][generate]") {
    using namespace fixtures::hole_plate;
    SECTION("tapping finds the tap drill holes") {
        tooling::Tool tap = tooling::makeDefaultTool(tooling::ToolType::Tap, 1, 10.0);
        tap.mill.threadPitch = 1.5;
        Fixture f("hole_plate.step", tap);
        const auto tp = f.generate(f.addOperation("tap"));
        REQUIRE_FALSE(tp.cycles.empty());
        for (const auto& c : tp.cycles) {
            CHECK(c.type == toolpath::CycleType::Tap);
        }
    }
    SECTION("thread milling with a major diameter") {
        tooling::Tool mill = tooling::makeDefaultTool(tooling::ToolType::ThreadMill, 1, 6.0);
        mill.mill.threadPitch = 1.5;
        Fixture f("hole_plate.step", mill);
        const auto none = f.generate(f.addOperation("threadmill"));
        CHECK_FALSE(none.warnings.empty()); // needs a selection or a major diameter
        const auto tp =
            f.generate(f.addOperation("threadmill", {}, Json{{"major_diameter", 10.0}}));
        CHECK(std::any_of(tp.moves.begin(), tp.moves.end(), [](const auto& m) {
            return m.kind == toolpath::MoveKind::ArcCCW && m.feedClass == toolpath::FeedClass::Cut;
        }));
    }
    SECTION("helical boring of holes larger than the tool") {
        Fixture f("hole_plate.step", tooling::makeFlatEndMill(1, 6.0, 25.0, 60.0));
        const auto tp = f.generate(f.addOperation("bore"));
        CHECK(std::count_if(tp.moves.begin(), tp.moves.end(),
                            [](const auto& m) { return m.isArc(); }) > 10);
    }
    SECTION("chamfer around the top face") {
        Fixture f("hole_plate.step", tooling::makeChamferMill(1, 10.0, 0.0, degToRad(90.0), 50.0));
        const auto tp = f.generate(
            f.addOperation("chamfer2d", {f.faceAtHeight(kThickness)}, Json{{"width", 0.5}}));
        CHECK(tp.strategy == "chamfer2d");
        const auto stats = toolpath::computeStats(tp);
        // Top face at WCS -1; tip 0.5 below the 0.5 mm chamfer.
        CHECK_THAT(stats.bounds.min.z(), WithinAbs(-2.0, 1e-6));
    }
    SECTION("chamfer selected edges, each height at its own") {
        Fixture f("hole_plate.step", tooling::makeChamferMill(1, 10.0, 0.0, degToRad(90.0), 50.0));
        const cad::Model& model = *f.doc.model(f.modelId);
        std::vector<ShapeRef> holeTop;
        std::vector<ShapeRef> outerEdges; // around the top and the bottom of the plate
        for (int i = 1; i <= cad::subShapeCount(model, cad::SubShapeType::Edge); ++i) {
            const auto sig = cad::signatureOf(model, cad::SubShapeType::Edge, i);
            const ShapeRef ref{f.modelId, cad::SubShapeType::Edge, i, sig};
            const bool horizontal = std::abs(sig.direction.z()) < 1e-9;
            if (std::abs(sig.centroid.z() - kThickness) < 1e-6 &&
                (sig.centroid.head<2>() - Vec2(kThroughX, kThroughY)).norm() <
                    0.5 * kThroughDiameter + 1e-6) {
                holeTop.push_back(ref);
            } else if (horizontal && std::abs(sig.measure - kSize) < 1e-6 &&
                       (std::abs(sig.centroid.z()) < 1e-6 ||
                        std::abs(sig.centroid.z() - kThickness) < 1e-6)) {
                outerEdges.push_back(ref);
            }
        }
        REQUIRE_FALSE(holeTop.empty());
        REQUIRE(outerEdges.size() == 8);
        const auto cutLevels = [](const toolpath::Toolpath& tp) {
            std::vector<double> z; // in cutting order
            for (const auto& m : tp.moves) {
                const double h = std::round(m.end.z() * 1000.0) / 1000.0;
                if (m.feedClass == toolpath::FeedClass::Cut && (z.empty() || z.back() != h)) {
                    z.push_back(h);
                }
            }
            return z;
        };
        // The through hole's top edge: the side it opens to is the hole, so the tool stays
        // within it, its tip 0.5 below the 0.5 mm chamfer at the top (WCS -1).
        const auto hole = f.generate(f.addOperation("chamfer2d", holeTop, Json{{"width", 0.5}}));
        CHECK(cutLevels(hole) == std::vector<double>{-2.0});
        for (const auto& m : hole.moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                // WCS origin at the stock corner, 1 mm outside the plate.
                const Vec2 centre(kThroughX + 1.0, kThroughY + 1.0);
                CHECK((m.end.head<2>() - centre).norm() < 0.5 * kThroughDiameter);
            }
        }
        // The plate's top and bottom outlines, from outside: the top first, then the bottom
        // (WCS -21), both in one toolpath.
        const auto outer =
            f.generate(f.addOperation("chamfer2d", outerEdges, Json{{"width", 0.5}}));
        CHECK(cutLevels(outer) == std::vector<double>{-2.0, -22.0});

        // Each outline on its own side: the hole's edge inside, the plate's top outside, at
        // the same height in one operation (automatic for the hole, set for the top).
        std::vector<ShapeRef> mixed = holeTop;
        for (const ShapeRef& r : outerEdges) {
            if (std::abs(r.signature.centroid.z() - kThickness) < 1e-6) {
                mixed.push_back(r);
                mixed.back().side = GeometrySide::Outside;
            }
        }
        const auto both = f.generate(f.addOperation("chamfer2d", mixed, Json{{"width", 0.5}}));
        CHECK(cutLevels(both).front() == -2.0);
        bool inHole = false;
        bool outsidePlate = false;
        const Vec2 centre(kThroughX + 1.0, kThroughY + 1.0);
        for (const auto& m : both.moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                inHole |= (m.end.head<2>() - centre).norm() < 0.5 * kThroughDiameter;
                outsidePlate |= m.end.x() > kSize + 1.0 || m.end.x() < 1.0;
            }
        }
        CHECK(inHole);
        CHECK(outsidePlate);
    }
}

TEST_CASE("3D operations on the dome fixture", "[document][generate][3d]") {
    using namespace fixtures::dome;
    Fixture f("dome.step", tooling::makeBallEndMill(1, 6.0, 30.0, 60.0));
    // WCS at the stock top (1 mm above the dome's top).
    SECTION("parallel finishing follows the dome") {
        const auto tp =
            f.generate(f.addOperation("parallel3d", {}, Json{{"scallop_height", 0.05}}));
        CHECK(tp.warnings.empty());
        // Along the moves (with smoothing an arc in the line's vertical plane goes over the top
        // between its ends).
        double top = -1e9;
        for (const auto& run : toolpath::sampleRuns(tp, 0.001)) {
            if (run.feedClass == toolpath::FeedClass::Cut) {
                for (const Vec3& q : run.points) {
                    top = std::max(top, q.z());
                }
            }
        }
        CHECK_THAT(top, WithinAbs(-1.0, 0.02)); // the dome top, 1 mm under the stock top
    }
    SECTION("waterline and roughing generate") {
        const auto wl =
            f.generate(f.addOperation("waterline3d", {}, Json::object(), testing::stepDown(2.0)));
        CHECK(wl.warnings.empty());
        CHECK(wl.moves.size() > 50);
        const auto rough =
            f.generate(f.addOperation("zlevel_rough", {}, Json::object(), testing::stepDown(4.0)));
        CHECK(rough.moves.size() > 50);
    }
}

TEST_CASE("pencil finishing in the filleted pocket's corners", "[document][generate][3d]") {
    using namespace fixtures::fillet_pocket;
    const double r = 2.0;
    Fixture f("fillet_pocket.step", tooling::makeBallEndMill(1, 2.0 * r, 20.0, 60.0));
    // Default setup: WCS origin at the stock's top corner, 1 mm proud of the part.
    const Vec3 origin = f.setup().wcs.origin;
    // After a 10 mm ball: one pass all round the pocket, touching the floor fillet halfway.
    const auto tp = f.generate(f.addOperation("pencil3d", {}, Json{{"reference_diameter", 10.0}}));
    CHECK(tp.warnings.empty());
    const double off = (kFloorRadius - r) / std::sqrt(2.0);
    std::size_t plunges = 0, checked = 0;
    for (const auto& m : tp.moves) {
        plunges += m.feedClass == toolpath::FeedClass::Plunge ? 1 : 0;
    }
    for (const auto& run : toolpath::sampleRuns(tp, 0.001)) {
        if (run.feedClass != toolpath::FeedClass::Cut) {
            continue;
        }
        for (const Vec3& q : run.points) {
            const Vec3 at = q + origin; // model coordinates
            // The straight part of the left wall's fillet.
            if (at.x() > kPocketX + 5.0 || at.y() < kPocketY + kCornerRadius ||
                at.y() > kPocketY + kPocketWidth - kCornerRadius) {
                continue;
            }
            CHECK_THAT(at.x(), WithinAbs(kPocketX + kFloorRadius - off, 0.03));
            CHECK_THAT(at.z() + r, WithinAbs(kFloorZ + kFloorRadius - off, 0.03));
            ++checked;
        }
    }
    CHECK(checked >= 2);
    CHECK(plunges == 1); // one closed loop
}

TEST_CASE("steep and shallow finishing on the dome", "[document][generate][3d]") {
    using namespace fixtures::dome;
    Fixture f("dome.step", tooling::makeBallEndMill(1, 6.0, 30.0, 60.0));
    const auto tp = f.generate(f.addOperation(
        "steep_shallow3d", {},
        Json{{"threshold_angle", 40.0}, {"steep_scallop", 0.05}, {"shallow_scallop", 0.05}}));
    CHECK(tp.warnings.empty());
    std::size_t level = 0, sloped = 0;
    double top = -1e9;
    for (const auto& run : toolpath::sampleRuns(tp, 0.001)) {
        if (run.feedClass != toolpath::FeedClass::Cut || run.points.size() < 3) {
            continue;
        }
        double lo = 1e9, hi = -1e9;
        for (const Vec3& q : run.points) {
            lo = std::min(lo, q.z());
            hi = std::max(hi, q.z());
        }
        top = std::max(top, hi);
        (hi - lo < 1e-6 ? level : sloped)++;
    }
    CHECK(level > 5);                       // constant-Z passes on the steep sides
    CHECK(sloped > 5);                      // parallel passes over the top and the plate
    CHECK_THAT(top, WithinAbs(-1.0, 0.02)); // the dome top, 1 mm under the stock top
    // Without the preset's ap a step down for the steep areas is missing: refused, not guessed.
    CHECK_THROWS(
        f.generate(f.addOperation("steep_shallow3d", {}, Json{{"shallow_scallop", 0.05}}, [] {
            PresetOverrides o;
            o.axialDepth = 0.0;
            return o;
        }())));
}

TEST_CASE("3+2 setup posts with table indexing", "[document][generate][index]") {
    using namespace fixtures::block_pocket;
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    // Second setup on the front face (-Y) of the block, on the 5-axis trunnion machine.
    const Uuid front = f.doc.addSetup(f.modelId);
    f.doc.modify("front setup", [&](Project& p) {
        Setup* s = p.findSetup(front);
        s->machine = "generic-5axis-trunnion-ac";
        s->workOffset = 2;
        s->originFrom.reset(); // the origin as set
        s->wcs.origin = Vec3(-1, -1, 0);
        s->wcs.zAxis = -Vec3::UnitY();
        s->wcs.xAxis = Vec3::UnitX();
        s->stock = p.findSetup(f.setupId)->stock;
        Operation op;
        op.name = "front face";
        op.type = "facing";
        op.toolId = f.toolId;
        op.presetId = f.presetId;
        s->operations.push_back(op);
    });
    CancelToken cancel;
    NullProgress progress;
    const Setup& setup = *f.doc.project().findSetup(front);
    const auto out = postSetup(f.doc, front, machineFor(f.doc.project(), setup),
                               post::builtinPost("fanuc-generic-mill"), cancel, progress);
    INFO((out.errors.empty() ? std::string() : out.errors.front()));
    REQUIRE(out.ok());
    CHECK_THAT(out.gcode, ContainsSubstring("G55"));
    // WCS Z = -Y: the table tilts A by -90 degrees (the smallest tilt, then the smallest C).
    CHECK_THAT(out.gcode, ContainsSubstring("A-90. C0."));
    const auto plotted = post::backplot(out.gcode);
    CHECK(post::compareBackplot(out.toolpaths, plotted.path, 2e-3).value_or("") == "");
}

TEST_CASE("operation heights and per-operation cutting values", "[document][generate]") {
    using namespace fixtures::block_pocket;
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    // WCS Z0 is the stock top, 1 mm above the part top.
    const double floorZ = (kHeight - kPocketDepth) - (kHeight + 1);
    const Uuid id = f.addOperation("pocket2d", {f.faceAtHeight(kHeight - kPocketDepth)});
    auto change = [&](const std::function<void(Operation&)>& edit) {
        f.doc.modify("edit", [&](Project& p) { edit(*p.findOperation(id)); });
    };
    auto op = [&]() -> const Operation& {
        return *f.doc.project().findOperation(id);
    };

    SECTION("defaults: 0.2\" above the stock, feed from the retract height") {
        // A Default setting (files of earlier versions) is the type's default.
        CHECK(op().heights.isDefault());
        CHECK(effectiveHeights(op()) == defaultHeights(op()));
        const OperationHeightValues hv = operationHeights(f.doc, f.setup(), op());
        CHECK_THAT(hv.retract, WithinAbs(0.0 + kDefaultHeightAllowance, 1e-9)); // stock top
        CHECK_THAT(hv.clearance, WithinAbs(2.0 * kDefaultHeightAllowance, 1e-9));
        CHECK_THAT(*hv.feed, WithinAbs(hv.retract, 1e-9));
        CHECK_THAT(*hv.top, WithinAbs(0.0, 1e-9));       // roughing: the stock top
        CHECK_THAT(*hv.bottom, WithinAbs(floorZ, 1e-6)); // the selected floor
        CHECK(hv.warnings.empty());

        // Finishing starts at the part top; facing stops there; holes feed above each hole.
        Operation other;
        other.type = "contour2d";
        CHECK(defaultHeights(other).top.from == HeightRef::PartTop);
        CHECK(defaultHeights(other).bottom.from == HeightRef::Geometry);
        other.type = "waterline3d";
        CHECK(defaultHeights(other).bottom.from == HeightRef::PartBottom);
        other.type = "facing";
        CHECK(defaultHeights(other).top.from == HeightRef::StockTop);
        CHECK(defaultHeights(other).bottom.from == HeightRef::PartTop);
        other.type = "drill";
        CHECK(defaultHeights(other).feed == HeightSetting{HeightRef::HoleTop, 2.0, std::nullopt});
        other.parameters = Json{{"retract_clearance", 3.0}}; // earlier versions' R plane
        CHECK(defaultHeights(other).feed == HeightSetting{HeightRef::HoleTop, 3.0, std::nullopt});
    }

    SECTION("the feed height applies where the tool comes down into the top") {
        const auto tp = f.generate(id);
        Vec3 at = tp.start;
        int entries = 0;
        for (const toolpath::Move& m : tp.moves) {
            if (m.kind == toolpath::MoveKind::Linear &&
                m.feedClass == toolpath::FeedClass::Plunge &&
                std::abs(at.z() - kDefaultHeightAllowance) < 1e-6 &&
                (m.end.head<2>() - at.head<2>()).norm() < 1e-9) {
                ++entries;
                CHECK(m.end.z() >= -1e-6); // into the top level only, not the lower ones
            }
            at = m.end;
        }
        CHECK(entries >= 1);
    }

    SECTION("clearance and retract cannot be measured from each other") {
        change([](Operation& o) {
            o.heights.clearance = {HeightRef::Retract, 5.0};
            o.heights.retract = {HeightRef::Clearance, -2.0};
        });
        CHECK_THROWS_AS(operationHeights(f.doc, f.setup(), op()), Error);
    }

    SECTION("references and offsets") {
        change([](Operation& o) {
            o.heights.clearance = {HeightRef::StockTop, 20.0};
            o.heights.retract = {HeightRef::PartTop, 3.0};
            o.heights.top = {HeightRef::PartTop, 0.0};
            o.heights.bottom = {HeightRef::Geometry, 1.0}; // leave 1 mm on the floor
            o.heights.feed = {HeightRef::TopHeight, 0.5};
        });
        const OperationHeightValues hv = operationHeights(f.doc, f.setup(), op());
        CHECK_THAT(hv.clearance, WithinAbs(20.0, 1e-9));
        CHECK_THAT(hv.retract, WithinAbs(-1.0 + 3.0, 1e-9));
        CHECK_THAT(*hv.top, WithinAbs(-1.0, 1e-9));
        CHECK_THAT(*hv.bottom, WithinAbs(floorZ + 1.0, 1e-6));
        CHECK_THAT(*hv.feed, WithinAbs(-0.5, 1e-9));
        CHECK(hv.warnings.empty());

        const auto tp = f.generate(id);
        const auto stats = toolpath::computeStats(tp);
        CHECK_THAT(stats.bounds.min.z(), WithinAbs(floorZ + 1.0, 1e-6));
        CHECK_THAT(stats.bounds.max.z(), WithinAbs(20.0, 1e-6)); // up to the clearance
    }

    SECTION("unsafe heights are raised with a warning; unusable references fail") {
        change([](Operation& o) { o.heights.retract = {HeightRef::Origin, -5.0}; });
        const OperationHeightValues hv = operationHeights(f.doc, f.setup(), op());
        CHECK(hv.retract > 0.0);
        REQUIRE_FALSE(hv.warnings.empty());
        CHECK_FALSE(f.generate(id).warnings.empty());
        change([](Operation& o) {
            o.heights = {};
            o.heights.bottom = {HeightRef::Geometry, 0.0};
            o.geometry.clear();
        });
        CHECK_THROWS_AS(operationHeights(f.doc, f.setup(), op()), Error);
    }

    SECTION("a height from a picked face") {
        change([&](Operation& o) {
            o.heights.bottom = {HeightRef::Picked, 0.5, f.faceAtHeight(kHeight - kPocketDepth)};
        });
        const OperationHeightValues hv = operationHeights(f.doc, f.setup(), op());
        CHECK_THAT(*hv.bottom, WithinAbs(floorZ + 0.5, 1e-6));
        change([](Operation& o) { o.heights.bottom.shape.reset(); });
        CHECK_THROWS_AS(operationHeights(f.doc, f.setup(), op()), Error);
    }

    SECTION("plunge roughing the pocket") {
        const Uuid plunge =
            f.addOperation("plunge_rough", {f.faceAtHeight(kHeight - kPocketDepth)});
        const auto tp = f.generate(plunge);
        CHECK(tp.warnings.empty());
        const auto stats = toolpath::computeStats(tp);
        CHECK_THAT(stats.bounds.min.z(), WithinAbs(floorZ, 1e-6));
        CHECK(std::count_if(tp.moves.begin(), tp.moves.end(), [](const toolpath::Move& m) {
                  return m.feedClass == toolpath::FeedClass::Plunge;
              }) > 5);
    }

    SECTION("a customized feed per tooth changes the feed, not the tool's preset") {
        double cutBefore = 0.0;
        for (const auto& m : f.generate(id).moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                cutBefore = m.feedRate;
            }
        }
        const double fz = f.doc.project().tools.findTool(f.toolId)->presets.front().feedPerTooth;
        change([fz](Operation& o) { o.presetOverrides.feedPerTooth = 2.0 * fz; });
        double cutAfter = 0.0;
        for (const auto& m : f.generate(id).moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                cutAfter = m.feedRate;
            }
        }
        CHECK_THAT(cutAfter, WithinAbs(2.0 * cutBefore, 1e-6 * cutBefore));
        CHECK(f.doc.project().tools.findTool(f.toolId)->presets.front().feedPerTooth == fz);
    }
}

TEST_CASE("contour outlines each on their own side", "[document][generate]") {
    using namespace fixtures::block_pocket;
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    const cad::Model& model = *f.doc.model(f.modelId);
    // The block's bottom outline (the operation's side: outside) and the pocket's rim (inside).
    std::vector<ShapeRef> geometry;
    for (int i = 1; i <= cad::subShapeCount(model, cad::SubShapeType::Edge); ++i) {
        const auto sig = cad::signatureOf(model, cad::SubShapeType::Edge, i);
        ShapeRef ref{f.modelId, cad::SubShapeType::Edge, i, sig};
        const Vec3& c = sig.centroid;
        const bool horizontal = std::abs(sig.direction.z()) < 1e-9;
        if (std::abs(c.z()) < 1e-6 && horizontal && sig.measure > kWidth - 1e-6) {
            geometry.push_back(ref);
        } else if (std::abs(c.z() - kHeight) < 1e-6 && c.x() > kPocketX - 1e-6 &&
                   c.x() < kPocketX + kPocketLength + 1e-6 && c.y() > kPocketY - 1e-6 &&
                   c.y() < kPocketY + kPocketWidth + 1e-6) {
            ref.side = GeometrySide::Inside;
            geometry.push_back(ref);
        }
    }
    REQUIRE(geometry.size() > 4);
    const auto tp =
        f.generate(f.addOperation("contour2d", geometry, Json::object(), testing::stepDown(40.0)));
    // WCS origin 1 mm outside the block's corner.
    bool inPocket = false;
    bool outsideBlock = false;
    for (const auto& m : tp.moves) {
        if (m.feedClass != toolpath::FeedClass::Cut) {
            continue;
        }
        const Vec2 p = m.end.head<2>() - Vec2(1.0, 1.0);
        inPocket |= p.x() > kPocketX && p.x() < kPocketX + kPocketLength && p.y() > kPocketY &&
                    p.y() < kPocketY + kPocketWidth;
        outsideBlock |= p.x() > kLength || p.x() < 0.0 || p.y() > kWidth || p.y() < 0.0;
    }
    CHECK(inPocket);
    CHECK(outsideBlock);
}

TEST_CASE("outlines open to the side away from the part", "[document][generate]") {
    using namespace fixtures::block_pocket;
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    const cad::Model& model = *f.doc.model(f.modelId);
    const auto edgeAt = [&](const auto& test) {
        for (int i = 1; i <= cad::subShapeCount(model, cad::SubShapeType::Edge); ++i) {
            const auto sig = cad::signatureOf(model, cad::SubShapeType::Edge, i);
            if (std::abs(sig.direction.z()) < 1e-9 && test(sig.centroid)) {
                return ShapeRef{f.modelId, cad::SubShapeType::Edge, i, sig};
            }
        }
        FAIL("no such edge");
        return ShapeRef{};
    };
    const auto side = [&](const ShapeRef& r) {
        return automaticSide(f.doc, f.setup(), r);
    };
    // The block's top and bottom outlines: outside; the pocket's rim: inside.
    const ShapeRef topOuter = edgeAt(
        [&](const Vec3& c) { return std::abs(c.z() - kHeight) < 1e-6 && std::abs(c.y()) < 1e-6; });
    const ShapeRef bottomOuter =
        edgeAt([&](const Vec3& c) { return std::abs(c.z()) < 1e-6 && std::abs(c.y()) < 1e-6; });
    const ShapeRef rim = edgeAt([&](const Vec3& c) {
        return std::abs(c.z() - kHeight) < 1e-6 && std::abs(c.y() - kPocketY) < 1e-6;
    });
    CHECK(side(topOuter) == GeometrySide::Outside);
    CHECK(side(bottomOuter) == GeometrySide::Outside);
    CHECK(side(rim) == GeometrySide::Inside);
    // Faces: the top face opens outward, the pocket's floor inward (its walls rise around it).
    CHECK(side(f.faceAtHeight(kHeight)) == GeometrySide::Outside);
    CHECK(side(f.faceAtHeight(kHeight - kPocketDepth)) == GeometrySide::Inside);
}

TEST_CASE("contour from edge loops and simulating up to an operation", "[document][generate]") {
    using namespace fixtures::block_pocket;
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    const cad::Model& model = *f.doc.model(f.modelId);
    // The four edges around the bottom of the block.
    std::vector<ShapeRef> bottomEdges;
    for (int i = 1; i <= cad::subShapeCount(model, cad::SubShapeType::Edge); ++i) {
        const auto sig = cad::signatureOf(model, cad::SubShapeType::Edge, i);
        if (std::abs(sig.centroid.z()) < 1e-6 && std::abs(sig.direction.z()) < 1e-9 &&
            sig.measure > kWidth - 1e-6) {
            bottomEdges.push_back({f.modelId, cad::SubShapeType::Edge, i, sig});
        }
    }
    REQUIRE(bottomEdges.size() == 4);
    const Uuid contour =
        f.addOperation("contour2d", bottomEdges, Json::object(), testing::stepDown(10.0));
    const auto tp = f.generate(contour);
    const auto stats = toolpath::computeStats(tp);
    CHECK_THAT(stats.bounds.min.z(), WithinAbs(-(kHeight + 1), 1e-6));
    CHECK(stats.bounds.max.x() > kLength + 1 + 3 - 1e-3); // outside the part

    // One edge of the outline finds the whole outline on the face across the tool axis.
    std::vector<int> loop = cad::edgeLoopOf(model, bottomEdges.front().index, Vec3::UnitZ());
    std::sort(loop.begin(), loop.end());
    std::vector<int> expected;
    for (const auto& e : bottomEdges) {
        expected.push_back(e.index);
    }
    std::sort(expected.begin(), expected.end());
    CHECK(loop == expected);

    // A start point at the block's corner vertex: the contour goes in next to that corner.
    int corner = 0;
    for (int i = 1; i <= cad::subShapeCount(model, cad::SubShapeType::Vertex); ++i) {
        if (cad::signatureOf(model, cad::SubShapeType::Vertex, i).centroid.norm() < 1e-9) {
            corner = i;
        }
    }
    REQUIRE(corner > 0);
    f.doc.modify("start", [&](Project& p) {
        p.findOperation(contour)->startPoint =
            ShapeRef{f.modelId, cad::SubShapeType::Vertex, corner,
                     cad::signatureOf(model, cad::SubShapeType::Vertex, corner)};
    });
    const auto started = f.generate(contour);
    const auto leadIn = std::find_if(started.moves.begin(), started.moves.end(), [](const auto& m) {
        return m.feedClass == toolpath::FeedClass::LeadIn;
    });
    REQUIRE(leadIn != started.moves.end());
    // The corner is at WCS (1, 1): the lead-in ends on the offset loop within a few mm of it.
    CHECK((leadIn->end.head<2>() - Vec2(1.0, 1.0)).norm() < 3.0 + 3.0 + 1e-3);

    // Edges that do not close a loop are refused.
    const Uuid open = f.addOperation(
        "contour2d", std::vector<ShapeRef>(bottomEdges.begin(), bottomEdges.begin() + 3));
    CHECK_THROWS_WITH(f.generate(open), ContainsSubstring("closed loops"));

    // A preview simulation stops after the chosen operation.
    const Uuid facing = f.addOperation("facing");
    SimulationOptions options;
    options.resolution = 1.0;
    options.compare = false;
    options.lastOperation = contour;
    const SetupSimulation upTo = simulateSetup(f.doc, f.setupId, options, f.cancel, f.progress);
    REQUIRE(upTo.operations.size() == 1);
    CHECK(upTo.operations.front().operationId == contour);
    SimulationOptions all = options;
    all.lastOperation.reset();
    CHECK(simulationInputKey(f.doc, f.setup(), options) !=
          simulationInputKey(f.doc, f.setup(), all));
    (void)facing;
}

TEST_CASE("smoothing is applied to every operation that has it", "[document][generate]") {
    const auto arcs = [](const toolpath::Toolpath& tp) {
        return std::count_if(tp.moves.begin(), tp.moves.end(),
                             [](const toolpath::Move& m) { return m.isArc(); });
    };
    SECTION("a pocket's own arcs become lines when it is off") {
        Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
        const ShapeRef floor = f.faceAtHeight(20.0);
        const auto on = f.generate(f.addOperation("pocket2d", {floor}));
        const auto off =
            f.generate(f.addOperation("pocket2d", {floor}, Json{{"smoothing", false}}));
        CHECK(arcs(on) > 0);
        CHECK(arcs(off) == 0);
        CHECK(off.moves.size() > on.moves.size());
    }
    SECTION("3D passes become arcs in their vertical planes when it is on") {
        Fixture f("dome.step", tooling::makeBallEndMill(1, 6.0, 30.0, 60.0));
        const auto on = f.generate(f.addOperation("parallel3d", {}, Json{{"scallop_height", 0.1}}));
        const auto off = f.generate(
            f.addOperation("parallel3d", {}, Json{{"scallop_height", 0.1}, {"smoothing", false}}));
        CHECK(std::any_of(on.moves.begin(), on.moves.end(), [](const toolpath::Move& m) {
            return m.isArc() && m.plane != toolpath::ArcPlane::XY;
        }));
        CHECK(arcs(off) == 0);
    }
}

TEST_CASE("flat area finishing on selected faces", "[document][generate][flat]") {
    // WCS at the stock top, 1 mm above the block: its top at -1, the pocket floor at -11.
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    const auto levels = [](const toolpath::Toolpath& tp) {
        std::set<double> z;
        for (const auto& m : tp.moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                z.insert(std::round(m.end.z() * 1000.0) / 1000.0);
            }
        }
        return z;
    };
    CHECK(levels(f.generate(f.addOperation("flat3d"))) == std::set<double>{-11.0, -1.0});
    CHECK(levels(f.generate(f.addOperation("flat3d", {f.faceAtHeight(20.0)}))) ==
          std::set<double>{-11.0});
}

TEST_CASE("generated toolpaths are cached and saved with the project", "[document][cache]") {
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    const Uuid facing = f.addOperation("facing");
    const Uuid pocket = f.addOperation("pocket2d", {f.faceAtHeight(20.0)});
    const auto cached = [&](const Document& doc, const Uuid& id) {
        const Setup* owner = nullptr;
        const Operation* op = doc.project().findOperation(id, &owner);
        return cachedToolpath(doc, *owner, *op, f.cancel, f.progress);
    };
    const auto key = [](const Document& doc, const Uuid& id) {
        const Setup* owner = nullptr;
        const Operation* op = doc.project().findOperation(id, &owner);
        return toolpathInputKey(doc, *owner, *op);
    };
    const auto first = cached(f.doc, facing);
    CHECK(cached(f.doc, facing) == first); // not generated again
    const Document snapshot = f.doc.snapshot();
    CHECK(snapshot.toolpathCache().find(facing, key(snapshot, facing)) == first); // shared
    const auto pocketPath = cached(f.doc, pocket);

    const auto file = std::filesystem::temp_directory_path() / "oxcam_toolpath_cache.ocam";
    f.doc.save(file);
    Document reopened = Document::open(file);
    CHECK(reopened.toolpathCache().size() == 2);
    const auto restored = reopened.toolpathCache().find(facing, key(reopened, facing));
    REQUIRE(restored != nullptr);
    CHECK_FALSE(toolpath::compareToolpaths(*first, *restored).has_value());
    CHECK(restored->toolId == first->toolId);
    CHECK(cached(reopened, pocket) == reopened.toolpathCache().find(pocket, key(reopened, pocket)));

    // Changed inputs: generated again; only the current toolpaths are saved.
    reopened.modify(
        "ae", [&](Project& p) { p.findOperation(facing)->presetOverrides.radialDepth = 2.0; });
    CHECK(reopened.toolpathCache().find(facing, key(reopened, facing)) == nullptr);
    CHECK(cached(reopened, facing) != nullptr);
    CHECK(reopened.toolpathCache().size() == 3);
    reopened.save(file);
    CHECK(Document::open(file).toolpathCache().size() == 2);

    // A damaged cache, or one of another format version, is generated again.
    ArchiveContents archive = readArchive(readFileBytes(file), "cache test");
    CHECK_FALSE(archive.toolpathCache.empty());
    Json saved = parseJson(archive.toolpathCache, "cache");
    saved["version"] = kToolpathCacheVersion + 1;
    ToolpathCache other;
    CHECK(other.load(saved) == 0);
    archive.toolpathCache = "{ damaged";
    writeFileAtomic(file, writeArchive(archive));
    CHECK(Document::open(file).toolpathCache().size() == 0);
    std::filesystem::remove(file);
}

TEST_CASE("posting uses the cached toolpaths, generated once", "[document][cache]") {
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    const Uuid facing = f.addOperation("facing");
    const Setup& setup = f.setup();
    const Operation& op = setup.operations.front();
    // Two threads asking at once: one generates, the other waits for it.
    std::shared_ptr<const toolpath::Toolpath> a;
    std::shared_ptr<const toolpath::Toolpath> b;
    {
        std::thread other([&] {
            CancelToken cancel;
            NullProgress progress;
            b = cachedToolpath(f.doc, setup, op, cancel, progress);
        });
        a = cachedToolpath(f.doc, setup, op, f.cancel, f.progress);
        other.join();
    }
    REQUIRE(a != nullptr);
    CHECK(a == b);
    CHECK(f.doc.toolpathCache().size() == 1);

    // The post takes the toolpath from the cache: a marked one shows up in the program.
    toolpath::Toolpath marked = *a;
    toolpath::Move m;
    m.kind = toolpath::MoveKind::Linear;
    m.end = marked.moves.back().end + Vec3(12.3456, 0.0, 0.0);
    m.feedRate = 10.0;
    marked.moves.push_back(m);
    f.doc.toolpathCache().store(facing, toolpathInputKey(f.doc, setup, op),
                                std::make_shared<const toolpath::Toolpath>(marked));
    const machine::Machine& machine = machineFor(f.doc.project(), setup);
    const PostOutput out =
        postSetup(f.doc, setup.id, machine, defaultPostFor(machine), f.cancel, f.progress);
    REQUIRE(out.ok());
    REQUIRE(out.toolpaths.size() == 1);
    CHECK(out.toolpaths.front().moves.size() == marked.moves.size());
}

TEST_CASE("operations with tool orientations of their own in one setup",
          "[document][generate][orientation]") {
    using namespace fixtures::block_pocket;
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    // Default setup: WCS at the stock's top corner (-1, -1, 31), stock 1 mm proud.
    f.doc.modify("trunnion", [&](Project& p) {
        p.findSetup(f.setupId)->machine = "generic-5axis-trunnion-ac";
    });
    auto orient = [&](const Uuid& id, const Vec3& z, const Vec3& x, int workOffset = 0) {
        f.doc.modify("orient", [&](Project& p) {
            p.findOperation(id)->orientation =
                ToolOrientation{Frame{Vec3::Zero(), x, z}, workOffset};
        });
    };
    auto post = [&] {
        CancelToken cancel;
        NullProgress progress;
        return postSetup(f.doc, f.setupId, machineFor(f.doc.project(), f.setup()),
                         post::builtinPost("fanuc-generic-mill"), cancel, progress);
    };
    const Uuid top = f.addOperation("facing");
    const Uuid front = f.addOperation("facing");
    orient(front, -Vec3::UnitY(), Vec3::UnitX());
    const Uuid right = f.addOperation("facing");
    orient(right, Vec3::UnitX(), Vec3::UnitY());

    SECTION("each operation is generated in its own frame and shown tilted in the WCS") {
        const auto tp = f.generate(front);
        std::size_t cuts = 0;
        for (const auto& m : tp.moves) {
            CHECK((m.axis - Vec3(0, -1, 0)).norm() < 1e-9);
            if (m.feedClass == toolpath::FeedClass::Cut) {
                ++cuts;
                // The part's front face (project y = 0) is WCS y = 1.
                CHECK_THAT(m.end.y(), WithinAbs(1.0, 1e-6));
            }
        }
        CHECK(cuts > 0);
        // Rapids stay in front of the part.
        CHECK(tp.start.y() < -5.0);
        // In its own frame the tool is along Z; facing cuts 1 mm below the stock (z = 0).
        const Operation& op = *f.doc.project().findOperation(front);
        const auto inFrame = generateToolpathInFrame(f.doc, f.setup(), op, f.cancel, f.progress);
        CHECK_FALSE(toolpath::isMultiAxis(inFrame));
        CHECK_THAT(toolpath::computeStats(inFrame).bounds.min.z(), WithinAbs(-1.0, 1e-6));
        // Heights are measured along the operation's Z: the retract height is above the stock.
        const OperationHeightValues hv = operationHeights(f.doc, f.setup(), op);
        CHECK_THAT(hv.retract, WithinAbs(kDefaultHeightAllowance, 1e-9));
        // The setup's operations are unchanged.
        CHECK_FALSE(toolpath::isMultiAxis(f.generate(top)));
    }
    SECTION("the post indexes the table for each operation and homes Z before turning it") {
        const auto out = post();
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        const machine::Machine& m = machineFor(f.doc.project(), f.setup());
        const auto rightIndex = machine::indexSolution(m, Vec3::UnitX());
        REQUIRE(rightIndex);
        CHECK_THAT(out.gcode, ContainsSubstring("A0. C0."));
        CHECK_THAT(out.gcode, ContainsSubstring("A-90. C0."));
        CHECK_THAT(out.gcode,
                   ContainsSubstring(fmt::format("A{:.0f}. C{:.0f}.", radToDeg(rightIndex->a),
                                                 radToDeg(rightIndex->c))));
        // One tool: Z goes home before each of the two turns of the table, and at the end.
        std::size_t homes = 0;
        for (std::size_t at = out.gcode.find("G28"); at != std::string::npos;
             at = out.gcode.find("G28", at + 1)) {
            ++homes;
        }
        CHECK(homes == 3);
        CHECK(out.gcode.find("G55") == std::string::npos);
        // The program back-plotted through the machine: positions and tool axes match.
        post::BackplotOptions options;
        options.machine = &m;
        const auto plotted = post::backplot(out.gcode, options);
        CHECK(post::compareBackplot(out.toolpaths, plotted.path, 2e-3).value_or("") == "");
        // The front face (project y = 0) is 31 mm below the setup's work offset (the stock
        // top) once the table tilts it up: the facing cuts there.
        const auto& frontPath = out.toolpaths[1];
        double lowest = 1e9;
        for (const auto& mv : frontPath.moves) {
            lowest = std::min(lowest, mv.end.z());
        }
        CHECK_THAT(lowest, WithinAbs(-31.0, 1e-6));
    }
    SECTION("a work offset of its own is measured at the orientation's origin") {
        f.doc.modify("own offset", [&](Project& p) {
            p.findOperation(front)->orientation->workOffset = 2;
            p.findOperation(front)->orientation->frame.origin = Vec3(51, 0, -16);
        });
        const auto out = post();
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        CHECK_THAT(out.gcode, ContainsSubstring("G55"));
        // The front face is 1 mm behind the origin (on the stock's front).
        double lowest = 1e9;
        for (const auto& mv : out.toolpaths[1].moves) {
            lowest = std::min(lowest, mv.end.z());
        }
        CHECK_THAT(lowest, WithinAbs(-1.0, 1e-6));
        // The setup's own offset cannot be measured somewhere else too.
        f.doc.modify("clash",
                     [&](Project& p) { p.findOperation(front)->orientation->workOffset = 1; });
        const auto clash = post();
        REQUIRE_FALSE(clash.ok());
        CHECK_THAT(clash.errors.front(), ContainsSubstring("G54"));
    }
    SECTION("orientations the machine cannot reach are refused, naming why") {
        orient(right, -Vec3::UnitZ(), Vec3::UnitX()); // from below: beyond A +-120
        CHECK_THROWS_WITH(f.generate(right), ContainsSubstring("cannot turn the tool orientation"));
        const auto out = post();
        CHECK_FALSE(out.ok());
        f.doc.modify("3-axis", [&](Project& p) { p.findSetup(f.setupId)->machine.clear(); });
        CHECK_THROWS_WITH(f.generate(front), ContainsSubstring("has no rotary axes"));
        // Turning about the tool axis only works without rotary axes.
        orient(right, Vec3::UnitZ(), Vec3::UnitY());
        orient(front, Vec3::UnitZ(), Vec3::UnitX());
        const auto flat = post();
        INFO((flat.errors.empty() ? std::string() : flat.errors.front()));
        REQUIRE(flat.ok());
        CHECK(flat.gcode.find(" A") == std::string::npos);
    }
    SECTION("the simulation cuts each face along its tool axis") {
        f.doc.modify("only front", [&](Project& p) { p.findOperation(right)->enabled = false; });
        SimulationOptions options;
        options.resolution = 0.5;
        options.compare = false;
        const auto sim = simulateSetup(f.doc, f.setupId, options, f.cancel, f.progress);
        REQUIRE(sim.operations.size() == 2);
        CHECK(sim.operations[1].error.empty());
        // The front skin of the stock: 102 x 30 x 1 mm under the faced top.
        CHECK_THAT(sim.operations[1].removedVolume, WithinRel(102.0 * 30.0, 0.05));
        CHECK(sim.operations[1].collisions.empty());
    }
}

TEST_CASE("rest machining in an operation's own frame", "[document][generate][orientation]") {
    using namespace fixtures::block_pocket;
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    const tooling::Tool big = testing::withPreset(tooling::makeFlatEndMill(2, 16.0, 25.0, 60.0));
    f.doc.modify("big tool", [&](Project& p) { p.tools.tools.push_back(big); });
    const ShapeRef floor = f.faceAtHeight(kHeight - kPocketDepth);
    f.doc.modify("rough", [&](Project& p) {
        Operation op;
        op.name = "rough";
        op.type = "pocket2d";
        op.toolId = big.id;
        op.presetId = big.presets.front().id;
        op.geometry = {floor};
        p.findSetup(f.setupId)->operations.push_back(op);
    });
    const Json rest{{"rest_machining", true}, {"rest_resolution", 0.25}};
    const Uuid plain = f.addOperation("pocket2d", {floor}, rest);
    const Uuid turned = f.addOperation("pocket2d", {floor}, rest);
    // The same tool axis, its X along the WCS Y and its origin elsewhere: the rest stock is
    // built in this frame.
    f.doc.modify("orient", [&](Project& p) {
        p.findOperation(turned)->orientation =
            ToolOrientation{Frame{Vec3(7, 3, -2), Vec3::UnitY(), Vec3::UnitZ()}, 0};
    });
    const Uuid full = f.addOperation("pocket2d", {floor});
    const auto plainPath = f.generate(plain);
    const double plainLength = toolpath::computeStats(plainPath).cutLength;
    // Each after the rough only.
    f.doc.modify("only one rest", [&](Project& p) { p.findOperation(plain)->enabled = false; });
    const auto turnedPath = f.generate(turned);
    const double turnedLength = toolpath::computeStats(turnedPath).cutLength;
    const double fullLength = toolpath::computeStats(f.generate(full)).cutLength;
    CHECK(plainLength > 0.0);
    CHECK(plainLength < 0.5 * fullLength);
    CHECK_THAT(turnedLength, WithinRel(plainLength, 0.15));
    // Mapped back, it cuts where the plain rest operation does (the pocket's corners).
    const auto box = toolpath::computeStats(turnedPath).bounds;
    const auto plainBox = toolpath::computeStats(plainPath).bounds;
    CHECK((box.min - plainBox.min).norm() < 1.0);
    CHECK((box.max - plainBox.max).norm() < 1.0);
}

TEST_CASE("lowerSetup gives the NC program its project, tools, operations and machine",
          "[document][post]") {
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    const Uuid face = f.addOperation("facing");
    f.doc.modify("comment", [&](Project& p) { p.findOperation(face)->comment = "skim the top"; });
    const machine::Machine& machine = machineFor(f.doc.project(), f.setup());
    const LoweredSetup lowered =
        lowerSetup(f.doc, f.setupId, machine, post::LowerOptions{}, f.cancel, f.progress);
    REQUIRE(lowered.ok());
    const post::NcProgram& program = lowered.program;
    CHECK(program.project == f.doc.project().name);
    CHECK(program.setup == f.setup().name);
    // "2026-10-25T14:03:07Z"
    REQUIRE(program.time.size() == 20);
    CHECK(program.time[10] == 'T');
    CHECK(program.time.back() == 'Z');
    REQUIRE(program.machine);
    CHECK(program.machine->id == machine.id);
    CHECK(program.machine->kinematics == "mill3");
    REQUIRE(program.tools.size() == 1);
    CHECK(program.tools[0].number == 1);
    CHECK(program.tools[0].type == "flat_end_mill");
    CHECK(program.tools[0].diameter == 6.0);
    CHECK(program.tools[0].gaugeLength > 0.0);
    REQUIRE(program.operations.size() == 1);
    CHECK(program.operations[0].name == "facing");
    CHECK(program.operations[0].comment == "skim the top");
    CHECK(program.operations[0].strategy == "facing");
    CHECK(program.operations[0].tool == 1);
    CHECK(program.operations[0].orientation == post::NcOrientation::ThreeAxis);
    CHECK(program.operations[0].boundsMax.z() > program.operations[0].boundsMin.z());
}

TEST_CASE("a post is refused for machines it does not declare", "[document][post]") {
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    (void)f.addOperation("facing");
    const machine::Machine& machine = machineFor(f.doc.project(), f.setup());
    const PostOutput out = postSetup(
        f.doc, f.setupId, machine, post::builtinPost("fanuc-generic-lathe"), f.cancel, f.progress);
    REQUIRE_FALSE(out.ok());
    CHECK_THAT(out.errors.front(), Catch::Matchers::ContainsSubstring("is not for mill3 machines"));
    CHECK(out.gcode.empty());
}

namespace {

/// A post the project carries: a FanucMill with a program option and a per-operation option,
/// which it writes as comments.
const char* const kProjectPost = R"js(
import { out } from "oxcam:post";
import { FanucMill } from "oxcam:fanuc";
export const manifest = { id: "shop-vmc", name: "Shop VMC", version: "1.0.0", engine: 1,
                          machines: ["mill3"] };
export const options = [
  { id: "optionalStop", label: "Optional stop", type: "bool", default: false },
  { id: "smoothing", label: "Smoothing", type: "choice", default: "rough",
    choices: ["rough", "finish"], scope: "operation" },
];
export default class extends FanucMill {
  operationBegin(op) {
    super.operationBegin(op);
    out.comment(`SMOOTHING ${op.options.smoothing}`);
    if (this.options.optionalStop) out.block("M01");
  }
}
)js";

} // namespace

TEST_CASE("the post a setup is posted with, and its option values", "[document][post]") {
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    const Uuid face = f.addOperation("facing");
    const Uuid pocket = f.addOperation("facing");
    auto postName = [&] {
        return document::postFor(f.doc.project(), f.setup(), machineFor(f.doc.project(), f.setup()))
            .manifest()
            .id;
    };
    // Nothing chosen: the built-in post for the machine's kinematics.
    CHECK(postName() == "fanuc-generic-mill");

    // The machine's default post: a built-in or one the project carries.
    machine::Machine shop = machine::genericMill3();
    shop.id = "shop-vmc-machine";
    shop.name = "Shop VMC";
    shop.post = "fanuc-generic-mill-4axis";
    f.doc.modify("machine", [&](Project& p) {
        p.machines.push_back(shop);
        p.findSetup(f.setupId)->machine = shop.id;
    });
    CHECK(postName() == "fanuc-generic-mill-4axis");
    f.doc.modify("project post", [&](Project& p) {
        p.posts.push_back({"shop-vmc.js", kProjectPost});
        p.machines.back().post = "shop-vmc";
    });
    CHECK(postName() == "shop-vmc");

    // The setup's choice wins.
    f.doc.modify("choose", [&](Project& p) {
        p.findSetup(f.setupId)->post = PostChoice{"fanuc-generic-mill", false};
    });
    CHECK(postName() == "fanuc-generic-mill");
    f.doc.modify("choose",
                 [&](Project& p) { p.findSetup(f.setupId)->post = PostChoice{"shop-vmc", true}; });
    CHECK(postName() == "shop-vmc");

    // Option values: the program's, and each operation's (the others take the default).
    f.doc.modify("options", [&](Project& p) {
        p.findSetup(f.setupId)->postOptions =
            Json{{"program", {{"optionalStop", true}}},
                 {"operations", {{pocket.toString(), {{"smoothing", "finish"}}}}}};
    });
    // A copy: the edits below replace the project the machine lives in.
    const machine::Machine machine = machineFor(f.doc.project(), f.setup());
    const PostOutput out = postSetup(f.doc, f.setupId, machine, f.cancel, f.progress);
    REQUIRE(out.ok());
    const auto first = out.gcode.find("(SMOOTHING ROUGH)");
    const auto second = out.gcode.find("(SMOOTHING FINISH)");
    CHECK(first != std::string::npos);
    CHECK(second != std::string::npos);
    CHECK(first < second);
    CHECK(out.gcode.find(" M01") != std::string::npos);
    (void)face;

    // A wrong value is an error naming the option.
    f.doc.modify("bad option", [&](Project& p) {
        p.findSetup(f.setupId)->postOptions["program"]["optionalStop"] = "yes";
    });
    const PostOutput bad = postSetup(f.doc, f.setupId, machine, f.cancel, f.progress);
    REQUIRE_FALSE(bad.ok());
    CHECK_THAT(bad.errors.front(), Catch::Matchers::ContainsSubstring("optionalStop"));

    // A chosen project post that is gone.
    f.doc.modify("remove post", [&](Project& p) { p.posts.clear(); });
    const PostOutput gone = postSetup(f.doc, f.setupId, machine, f.cancel, f.progress);
    REQUIRE_FALSE(gone.ok());
    CHECK_THAT(gone.errors.front(), Catch::Matchers::ContainsSubstring("not in the project"));
}

TEST_CASE("a machine that does not exist is refused, not taken for another",
          "[document][machine][refusal]") {
    Fixture f("block_pocket.step", tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
    const Uuid face = f.addOperation("facing");
    // A machine of the project's, for a toolpath to be kept under before it goes.
    machine::Machine own = machine::genericMill3();
    own.id = "our-mill";
    f.doc.modify("machine", [&](Project& p) {
        p.machines.push_back(own);
        p.findSetup(f.setupId)->machine = own.id;
    });
    const auto kept = cachedToolpath(f.doc, f.setup(), *f.doc.project().findOperation(face),
                                     f.cancel, f.progress);
    REQUIRE_FALSE(kept->moves.empty());
    f.doc.modify("the machine goes", [&](Project& p) { p.machines.clear(); });
    // After that edit: a change replaces the project, and what pointed into it with it.
    const Operation& op = *f.doc.project().findOperation(face);

    const auto namesIt = [&](const std::string& text) {
        CHECK_THAT(text, ContainsSubstring("'our-mill'"));
        CHECK_THAT(text, ContainsSubstring("under Machine in the setup's properties"));
    };
    SECTION("generating, with the toolpath kept from before as well") {
        CHECK_THROWS_WITH(f.generate(face), ContainsSubstring("'our-mill'"));
        CHECK_THROWS_WITH(cachedToolpath(f.doc, f.setup(), op, f.cancel, f.progress),
                          ContainsSubstring("under Machine in the setup's properties"));
    }
    SECTION("simulating") {
        CHECK_THROWS_WITH(simulateSetup(f.doc, f.setupId, {}, f.cancel, f.progress),
                          ContainsSubstring("'our-mill'"));
    }
    SECTION("posting, whichever machine and post are passed") {
        const PostOutput out = postSetup(f.doc, f.setupId, machine::genericMill3(),
                                         post::builtinPost("fanuc-generic-mill"), f.cancel,
                                         f.progress, Json::object());
        REQUIRE_FALSE(out.ok());
        CHECK(out.gcode.empty());
        namesIt(out.errors.front());
    }
    SECTION("what is shown of the setup goes on without it") {
        CHECK_FALSE(toolpathInputKey(f.doc, f.setup(), op).empty());
        CHECK_FALSE(simulationInputKey(f.doc, f.setup(), {}).empty());
        CHECK(latheSupports(f.doc, f.setup()).empty());
        namesIt(machineProblem(f.doc.project(), f.setup()));
    }
    SECTION("a project that names it opens, and choosing a machine puts it right") {
        const auto path = std::filesystem::temp_directory_path() / "oxcam_unknown_machine.ocam";
        f.doc.save(path);
        Document opened = Document::open(path);
        std::filesystem::remove(path);
        const Uuid setupId = opened.project().setups.front().id;
        CHECK(opened.project().setups.front().machine == "our-mill");
        CHECK_FALSE(opened.project().validate().empty());
        CHECK(findMachineFor(opened.project(), opened.project().setups.front()) == nullptr);
        opened.modify("choose",
                      [&](Project& p) { p.findSetup(setupId)->machine = "generic-3axis-vmc"; });
        const Setup& fixed = *opened.project().findSetup(setupId);
        CHECK_FALSE(generateToolpath(opened, fixed, fixed.operations.front(), f.cancel, f.progress)
                        .moves.empty());
    }
}

TEST_CASE("a 2D trace along selected edges, open and closed", "[document][generate][trace]") {
    using namespace fixtures::hole_plate;
    Fixture f("hole_plate.step", tooling::makeBallEndMill(1, 2.0, 10.0, 50.0));
    const cad::Model& model = *f.doc.model(f.modelId);
    // Two edges of the plate's top that meet at its (kSize, 0) corner: an open L. And the through
    // hole's top edges: a closed circle.
    std::vector<ShapeRef> ell;
    std::vector<ShapeRef> hole;
    for (int i = 1; i <= cad::subShapeCount(model, cad::SubShapeType::Edge); ++i) {
        const auto sig = cad::signatureOf(model, cad::SubShapeType::Edge, i);
        const ShapeRef ref{f.modelId, cad::SubShapeType::Edge, i, sig};
        if (std::abs(sig.centroid.z() - kThickness) > 1e-6) {
            continue;
        }
        const Vec2 c = sig.centroid.head<2>();
        if ((std::abs(c.y()) < 1e-6 || std::abs(c.x() - kSize) < 1e-6) &&
            std::abs(sig.measure - kSize) < 1e-6) {
            ell.push_back(ref);
        } else if ((c - Vec2(kThroughX, kThroughY)).norm() < 0.5 * kThroughDiameter + 1e-6) {
            hole.push_back(ref);
        }
    }
    REQUIRE(ell.size() == 2);
    REQUIRE_FALSE(hole.empty());
    // WCS: the stock corner 1 mm outside the plate, its top 1 mm above (the plate's top at -1).
    const auto cutPoints = [](const toolpath::Toolpath& tp) {
        std::vector<Vec3> pts;
        for (const auto& run : toolpath::sampleRuns(tp, 0.001)) {
            if (run.feedClass == toolpath::FeedClass::Cut) {
                pts.insert(pts.end(), run.points.begin(), run.points.end());
            }
        }
        return pts;
    };
    SECTION("the open L: on its edges, at the depth below them, in levels of the step down") {
        const auto tp = f.generate(
            f.addOperation("trace2d", ell, Json{{"depth", 0.6}}, testing::stepDown(0.25)));
        CHECK(tp.strategy == "trace2d");
        CHECK(tp.warnings.empty());
        std::set<long long> levels;
        for (const Vec3& q : cutPoints(tp)) {
            levels.insert(std::llround(q.z() * 1000.0));
            // On the plate's front edge (y = 0) or its right one (x = kSize).
            const bool front = std::abs(q.y() - 1.0) < 1e-6 && q.x() > 1.0 - 1e-6;
            const bool right = std::abs(q.x() - (kSize + 1.0)) < 1e-6 && q.y() > 1.0 - 1e-6;
            CHECK((front || right));
        }
        CHECK(levels == std::set<long long>{-1200, -1400, -1600});
    }
    SECTION("the hole's top edge: a closed loop beside it, inside, the tool's side on it") {
        const auto tp = f.generate(f.addOperation(
            "trace2d", hole, Json{{"depth", 0.2}, {"side", "left"}}, testing::stepDown(1.0)));
        const Vec2 centre(kThroughX + 1.0, kThroughY + 1.0);
        for (const Vec3& q : cutPoints(tp)) {
            CHECK_THAT((q.head<2>() - centre).norm(),
                       WithinAbs(0.5 * kThroughDiameter - 1.0, 0.01));
            CHECK_THAT(q.z(), WithinAbs(-1.2, 1e-6));
        }
    }
    SECTION("the way down is fed from the feed height, at the operation's plunge feed") {
        const auto tp = f.generate(
            f.addOperation("trace2d", ell, Json{{"depth", 0.3}}, testing::stepDown(1.0)));
        for (const auto& m : tp.moves) {
            if (m.kind == toolpath::MoveKind::Rapid) {
                CHECK(m.end.z() >= -1.0 + 1e-6); // never below the edges' height by a rapid
            }
        }
        // A plunge feed of 0 is not replaced: the toolpath is refused, naming it (ADR-0004).
        f.doc.modify("no plunge feed",
                     [](Project& p) { p.tools.tools.front().presets.front().plungeFeed = 0.0; });
        const auto op =
            f.addOperation("trace2d", ell, Json{{"depth", 0.3}}, testing::stepDown(1.0));
        CHECK_THROWS_WITH(f.generate(op), ContainsSubstring("plunge feed"));
    }
    SECTION("faces are not edges") {
        CHECK_THROWS_WITH(f.generate(f.addOperation("trace2d", {f.faceAtHeight(kThickness)})),
                          ContainsSubstring("edges"));
        CHECK_THROWS_WITH(f.generate(f.addOperation("trace2d")), ContainsSubstring("edges"));
    }
    SECTION("the simulation shows the groove, and does not take it for a gouge") {
        f.addOperation("trace2d", ell, Json{{"depth", 0.5}}, testing::stepDown(1.0));
        SimulationOptions options;
        options.resolution = 0.25;
        const SetupSimulation sim = simulateSetup(f.doc, f.setupId, options, f.cancel, f.progress);
        REQUIRE(sim.operations.size() == 1);
        INFO(sim.operations.front().error);
        CHECK(sim.operations.front().error.empty());
        CHECK(sim.operations.front().collisions.empty());
        // Through the 1 mm of stock above the plate and 0.5 into it, along 160 mm: a ball's
        // groove, about half of it over the plate.
        CHECK(sim.operations.front().removedVolume > 100.0);
        REQUIRE(sim.comparison);
        CHECK_FALSE(sim.comparison->hasGouges());
    }
}

TEST_CASE("the trace fixture project simulates cleanly", "[document][simulate][trace]") {
    // An open chain at the block's top corner and a closed ring beside its pocket, after facing.
    CancelToken cancel;
    NullProgress progress;
    const Document doc =
        Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "trace_project.ocam");
    const Setup& setup = doc.project().setups.front();
    SimulationOptions options;
    options.resolution = 0.25;
    const SetupSimulation sim = simulateSetup(doc, setup.id, options, cancel, progress);
    REQUIRE(sim.operations.size() == 3);
    for (const auto& op : sim.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
        CHECK(op.collisions.empty());
        CHECK(op.removedVolume > 1.0);
    }
    REQUIRE(sim.comparison);
    CHECK_FALSE(sim.comparison->hasGouges());
}
