// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Milling, thread milling and helical boring with a live tool on a lathe (DL-0066).

#include "FixtureSpecs.hpp"
#include "TestProject.hpp"

#include <ocam/cad/Features.hpp>
#include <ocam/cad/SubShape.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/document/SimulationCache.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/post/Post.hpp>
#include <ocam/sim/Compare.hpp>
#include <ocam/sim/Simulate.hpp>
#include <ocam/tooling/Tool.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

using namespace ocam;
using namespace ocam::document;
using Catch::Matchers::ContainsSubstring;

namespace {

constexpr std::size_t kPocket = 2, kBore = 3, kThreadMill = 5, kFlat = 8, kFinish = 9;

struct MillTurn {
    CancelToken cancel;
    NullProgress progress;
    Document doc =
        Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "millturn_project.ocam");

    const Setup& setup() const { return doc.project().setups.front(); }
    toolpath::Toolpath generate(std::size_t k) {
        return generateToolpath(doc, setup(), setup().operations[k], cancel, progress);
    }
    void machine(const char* id) {
        doc.modify("machine", [&](Project& p) { p.setups.front().machine = id; });
    }
    void axes(std::size_t k, const char* value) {
        doc.modify("axes",
                   [&](Project& p) { p.setups.front().operations[k].parameters["live_axes"] = value; });
    }
    PostOutput post(const char* id, const Json& options = Json::object()) {
        const machine::Machine& m = machineFor(doc.project(), setup());
        return postSetup(doc, setup().id, m, post::builtinPost(id), cancel, progress, options);
    }
    static void matchesBackplot(const PostOutput& out) {
        double deviation = 0.0;
        const auto differs = post::compareBackplot(
            out.toolpaths, post::backplot(out.gcode, {true}).path, 2e-3, &deviation, false);
        INFO(differs.value_or(""));
        CHECK_FALSE(differs.has_value());
    }
};

} // namespace

TEST_CASE("what a live tool mills", "[document][lathe][live][mill]") {
    MillTurn f;
    SECTION("the operations say how they go") {
        const toolpath::Toolpath pocket = f.generate(kPocket);
        CHECK(pocket.live);
        CHECK(pocket.liveAxes == toolpath::LiveAxes::Polar);
        const toolpath::Toolpath flat = f.generate(kFlat);
        CHECK(flat.live);
        CHECK(flat.liveAxes == toolpath::LiveAxes::HeldC);
        CHECK(f.generate(kBore).liveAxes == toolpath::LiveAxes::Polar);
        CHECK(f.generate(kThreadMill).liveAxes == toolpath::LiveAxes::Polar);
    }
    SECTION("polar interpolation is for the face") {
        f.axes(kFlat, "polar");
        CHECK_THROWS_WITH(f.generate(kFlat), ContainsSubstring("Live tool axes"));
    }
    SECTION("without a Y axis, what leaves the spindle's centre is refused, naming the choice") {
        f.machine("generic-lathe-live");
        CHECK_THROWS_WITH(f.generate(kFlat), ContainsSubstring("no Y axis"));
        CHECK_NOTHROW(f.generate(kPocket));
        f.axes(kPocket, "y");
        CHECK_THROWS_WITH(f.generate(kPocket), ContainsSubstring("polar interpolation"));
    }
    SECTION("without live tooling, nothing mills") {
        f.machine("generic-2axis-lathe");
        CHECK_THROWS_WITH(f.generate(kPocket), ContainsSubstring("no live tooling"));
        CHECK_THROWS_WITH(f.generate(kThreadMill), ContainsSubstring("no live tooling"));
    }
}

TEST_CASE("milling a turned stock", "[document][lathe][live][mill]") {
    using namespace fixtures::millturn;
    MillTurn f;
    const SetupSimulation sim = simulateSetup(f.doc, f.setup().id, {}, f.cancel, f.progress);
    REQUIRE(sim.operations.size() == 10);
    for (const auto& op : sim.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
        CHECK(op.collisions.empty());
    }
    REQUIRE(sim.milledFrom);
    CHECK(*sim.milledFrom == kPocket);
    CHECK(sim.playsOnProfile(0));
    CHECK(sim.playsOnProfile(kPocket - 1));
    CHECK_FALSE(sim.playsOnProfile(kPocket));
    SECTION("each takes what it should") {
        CHECK(sim.operations[kPocket].removedVolume ==
              Catch::Approx(kPocketSide * kPocketSide * kPocketDepth).epsilon(0.15));
        CHECK(sim.operations[kBore].removedVolume ==
              Catch::Approx(kPi * 25.0 * kBoreDepth).epsilon(0.1));
        CHECK(sim.operations[kFlat].removedVolume > 100.0);
    }
    SECTION("the turning before it is on the profile, and dexels agree") {
        const double exact = sim.operations[1].latheAfter->volume();
        CHECK(sim.operations[1].stockAfter.volume() == Catch::Approx(exact).epsilon(0.01));
    }
    SECTION("the stock against the part, in dexels, reads no gouge") {
        REQUIRE(sim.comparison);
        CHECK_FALSE(sim.comparison->hasGouges());
        CHECK(sim.partReference);
    }
    SECTION("turning after the milling keeps the milling") {
        const sim::Stock& before = sim.operations[kFlat].stockAfter;
        const sim::Stock& after = sim.operations[kFinish].stockAfter;
        CHECK(after.volume() <= before.volume() + 1e-6);
        CHECK(after.volume() > 0.9 * before.volume());
        CHECK(sim.lastStock(true).volume() == Catch::Approx(after.volume()));
    }
    SECTION("the cache keeps it") {
        const std::string bytes = encodeSimulation(sim, "key");
        const auto back = decodeSimulation(bytes, "key", f.doc, f.setup());
        REQUIRE(back);
        REQUIRE(back->milledFrom);
        CHECK(*back->milledFrom == kPocket);
        CHECK(back->finalStock().volume() == Catch::Approx(sim.finalStock().volume()));
    }
}

TEST_CASE("an external thread milled on the turned diameter with a live tool",
          "[document][lathe][live][mill][threadmill]") {
    using namespace fixtures::millturn;
    MillTurn f;
    constexpr double kLengthOfThread = 10.0;
    // After the finish turning, the thread mill goes around the front of the bar, C turning or
    // held.
    const Operation& internal = f.setup().operations[kThreadMill];
    REQUIRE(internal.type == "threadmill");
    Operation op;
    op.name = "External thread";
    op.type = "threadmill";
    op.toolId = internal.toolId;
    op.presetId = internal.presetId;
    op.parameters = Json{
        {"side", "external"}, {"major_diameter", kDiameter}, {"thread_depth", kLengthOfThread}};
    op.heights = defaultHeights(op);
    f.doc.modify("thread", [&](Project& p) { p.setups.front().operations.push_back(op); });
    const std::size_t k = f.setup().operations.size() - 1;
    const tooling::Tool& tool = *f.doc.project().tools.findTool(*op.toolId);
    const double helix = kDiameter / 2.0 - 0.6134 * kThreadPitch + tool.mill.diameter / 2.0;
    const auto checkHelix = [&](const toolpath::Toolpath& tp) {
        int arcs = 0;
        for (const auto& m : tp.moves) {
            if (m.feedClass == toolpath::FeedClass::Cut) {
                REQUIRE(m.isArc());
                CHECK(m.center.head<2>().norm() < 1e-6); // around the spindle's axis
                CHECK((m.end.head<2>() - m.center.head<2>()).norm() == Catch::Approx(helix));
                ++arcs;
            }
        }
        CHECK(arcs == 2 * 7); // ceil(10 / 1.5) turns
    };

    SECTION("polar interpolation, C turning") {
        const toolpath::Toolpath tp = f.generate(k);
        CHECK(tp.live);
        CHECK(tp.liveAxes == toolpath::LiveAxes::Polar);
        checkHelix(tp);
    }
    SECTION("the Y axis, C held still") {
        f.axes(k, "y");
        const toolpath::Toolpath tp = f.generate(k);
        CHECK(tp.liveAxes == toolpath::LiveAxes::HeldC);
        checkHelix(tp);
        // Without a Y axis it would leave Y 0: refused, naming the choice.
        f.machine("generic-lathe-live");
        CHECK_THROWS_WITH(f.generate(k), ContainsSubstring("no Y axis"));
    }
    SECTION("the simulation shows the cut, and it is no gouge") {
        const SetupSimulation sim = simulateSetup(f.doc, f.setup().id, {}, f.cancel, f.progress);
        REQUIRE(sim.operations.size() == k + 1);
        const auto& thread = sim.operations[k];
        INFO(thread.error);
        CHECK(thread.error.empty());
        CHECK(thread.collisions.empty());
        // At least the root's ring minus the flat: between the major and minor diameters.
        const double minor = kDiameter - 2.0 * 0.6134 * kThreadPitch;
        const double ring = kPi / 4.0 * (kDiameter * kDiameter - minor * minor) * kLengthOfThread;
        CHECK(thread.removedVolume > 0.3 * ring);
        CHECK(thread.removedVolume < 1.2 * ring);
        REQUIRE(sim.comparison);
        CHECK_FALSE(sim.comparison->hasGouges());
    }
    SECTION("in the program, both ways, and the back-plot agrees") {
        for (const char* axes : {"polar", "y"}) {
            INFO(axes);
            f.axes(k, axes);
            const auto out = f.post("haas-lathe");
            INFO((out.errors.empty() ? std::string() : out.errors.front()));
            REQUIRE(out.ok());
            CHECK_THAT(out.gcode, ContainsSubstring("(EXTERNAL THREAD)"));
            MillTurn::matchesBackplot(out);
        }
    }
}

TEST_CASE("an external thread on a stud across the part, C held still",
          "[document][lathe][live][mill][threadmill]") {
    using namespace fixtures::stud_bar;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid model = doc.importModel(std::filesystem::path(OCAM_FIXTURE_DIR) / "stud_bar.step",
                                       cancel, progress);
    const Uuid setupId = doc.addSetup(model);
    tooling::Tool mill = tooling::makeDefaultTool(tooling::ToolType::ThreadMill, 1, 6.0);
    mill.mill.threadPitch = 1.5;
    mill.mill.shankDiameter = 4.0;
    mill.mill.fluteLength = 20.0;
    mill.mill.overallLength = 60.0;
    mill.presets = {tooling::defaultPreset(mill)};
    mill.presets.front().surfaceSpeed = 1000.0; // within the live tool's speed
    Operation op;
    op.name = "Stud thread";
    op.type = "threadmill";
    op.toolId = mill.id;
    op.presetId = mill.presets.front().id;
    op.parameters = Json{{"side", "external"}, {"live_axes", "y"}};
    // The stud's face picked: it stands along +X.
    {
        const cad::Model& part = *doc.model(model);
        const auto bosses = cad::findBosses(part, Vec3::UnitX());
        REQUIRE(bosses.size() == 1);
        CHECK(bosses.front().diameter == Catch::Approx(kStudDiameter));
        const int face = bosses.front().faces.front();
        op.geometry = {{model, cad::SubShapeType::Face, face,
                        cad::signatureOf(part, cad::SubShapeType::Face, face)}};
    }
    // From outside along +X, onto the stud's end (WCS z 0 is the bar's front).
    op.orientation = ToolOrientation{
        Frame{Vec3(kFlatX, 0.0, kStudZ - kLength), Vec3::UnitY(), Vec3::UnitX()}, 0};
    op.heights = defaultHeights(op);
    doc.modify("setup", [&](Project& p) {
        Setup* s = p.findSetup(setupId);
        s->machine = "generic-lathe-live-y";
        s->originFrom.reset();
        s->wcs.origin = Vec3(0, 0, kLength);
        s->wcs.zAxis = Vec3::UnitZ();
        s->wcs.xAxis = Vec3::UnitX();
        s->stock.kind = StockKind::Box;
        s->stock.size = StockSize::OffsetFromPart;
        s->stock.offsetSide = 1.0;
        s->stock.offsetTop = 1.0;
        s->stock.offsetBottom = 0.0;
        p.tools.tools = {mill};
        s->operations.push_back(op);
    });
    const auto generate = [&] {
        const Setup& s = *doc.project().findSetup(setupId);
        return generateToolpath(doc, s, s.operations.front(), cancel, progress);
    };
    const toolpath::Toolpath tp = generate();
    INFO((tp.warnings.empty() ? std::string() : tp.warnings.front()));
    CHECK(tp.live);
    CHECK(tp.liveAxes == toolpath::LiveAxes::HeldC);
    // The stud's own diameter is the major; the helix goes around its axis. Across the part the
    // toolpath is in the setup's frame, its arcs lines (DL-0066): the points tell.
    const double helix = kStudDiameter / 2.0 - 0.6134 * 1.5 + 3.0;
    const Vec3 axisPoint(0.0, 0.0, kStudZ - kLength);
    const auto fromAxis = [&](const Vec3& p) {
        Vec3 d = p - axisPoint;
        d.x() = 0.0;
        return d;
    };
    double turned = 0.0; // about +X, the tool's direction
    double xMin = 1e9, xMax = -1e9, xFirst = 0.0, xLast = 0.0;
    Vec3 pos = tp.start;
    bool first = true;
    for (const auto& m : tp.moves) {
        if (m.feedClass == toolpath::FeedClass::Cut) {
            CHECK(fromAxis(m.end).norm() == Catch::Approx(helix).margin(1e-6));
            const Vec3 a = fromAxis(pos), b = fromAxis(m.end);
            turned += std::atan2(a.cross(b).x(), a.dot(b));
            xFirst = first ? pos.x() : xFirst;
            first = false;
            xLast = m.end.x();
            xMin = std::min({xMin, pos.x(), m.end.x()});
            xMax = std::max({xMax, pos.x(), m.end.x()});
        }
        pos = m.end;
    }
    // Right hand, climb: clockwise seen from the tool, six turns (ceil(8 / 1.5)), going down
    // from a whole number of pitches above the stud's root to its root at the flat.
    CHECK(turned == Catch::Approx(-6.0 * 2.0 * kPi).margin(1e-6));
    CHECK(xMin == Catch::Approx(kFlatX).margin(1e-6));
    CHECK(xMax == Catch::Approx(kFlatX + 6 * 1.5).margin(1e-6));
    CHECK(xFirst > xLast);
    // The program, and its back-plot agrees.
    const Setup& s = *doc.project().findSetup(setupId);
    const machine::Machine& m = machineFor(doc.project(), s);
    const auto out = postSetup(doc, setupId, m, post::builtinPost("haas-lathe"), cancel, progress,
                               Json::object());
    INFO((out.errors.empty() ? std::string() : out.errors.front()));
    REQUIRE(out.ok());
    MillTurn::matchesBackplot(out);
    // Polar interpolation is for the face only.
    doc.modify("polar", [&](Project& p) {
        p.findSetup(setupId)->operations.front().parameters["live_axes"] = "polar";
    });
    CHECK_THROWS_WITH(generate(), ContainsSubstring("Live tool axes"));
}

TEST_CASE("a live tap after milling, which only the simulation can check", "[document][lathe][live][mill]") {
    MillTurn f;
    constexpr std::size_t kTapDrill = 6, kTap = 7;
    REQUIRE(f.setup().operations[kTap].type == "tap");
    SECTION("the drill before it leaves it a hole") {
        const SetupSimulation sim = simulateSetup(f.doc, f.setup().id, {}, f.cancel, f.progress);
        CHECK(sim.operations[kTap].collisions.empty());
    }
    SECTION("without the drill, the tap reads as going into solid") {
        f.doc.modify("no drill",
                     [&](Project& p) { p.setups.front().operations[kTapDrill].enabled = false; });
        CHECK_NOTHROW(f.generate(kTap));
        const SetupSimulation sim = simulateSetup(f.doc, f.setup().id, {}, f.cancel, f.progress);
        const auto& op = sim.operations[kTap - 1];
        REQUIRE(op.operationId == f.setup().operations[kTap].id);
        const bool flagged =
            std::any_of(op.collisions.begin(), op.collisions.end(), [](const sim::Collision& c) {
                return c.kind == sim::CollisionKind::NoHole && c.depth > 1.0;
            });
        CHECK(flagged);
    }
}

TEST_CASE("a live tool's milling in the program", "[document][lathe][live][mill]") {
    MillTurn f;
    SECTION("Haas: polar interpolation in radius, and the back-plot agrees") {
        const auto out = f.post("haas-lathe");
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        for (const char* word : {"G112", "G113", "G17", "G18", "M154", "M155", "M133 P", "M135",
                                 "G18 G95 C"}) {
            INFO(word);
            CHECK_THAT(out.gcode, ContainsSubstring(word));
        }
        MillTurn::matchesBackplot(out);
    }
    SECTION("Y axis with C held: X a diameter, G17, the C on the page") {
        f.axes(kPocket, "y");
        f.doc.modify("c", [&](Project& p) {
            p.setups.front().operations[kPocket].parameters["live_c"] = 90.0;
        });
        const auto out = f.post("haas-lathe");
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        const auto pocket = out.gcode.find("(POCKET)");
        const auto counterbore = out.gcode.find("(COUNTERBORE)");
        REQUIRE(pocket != std::string::npos);
        const std::string program = out.gcode.substr(pocket, counterbore - pocket);
        CHECK_THAT(program, ContainsSubstring("C90."));
        CHECK_THAT(program, ContainsSubstring("G17"));
        CHECK_THAT(program, !ContainsSubstring("G112"));
        MillTurn::matchesBackplot(out);
    }
    SECTION("Fanuc's polar interpolation, with a builder's codes: X a diameter, C for Y") {
        const Json options{{"program",
                            {{"liveToolOn", "M133 P{rpm}"},
                             {"liveToolOff", "M135"},
                             {"cAxisOn", "M154"},
                             {"cAxisOff", "M155"},
                             {"liveTapAxial", "M29 S{rpm}|G84 Z{z} F{lead}"},
                             {"liveTapRadial", "M29 S{rpm}|G88 X{x} F{lead}"}}}};
        const auto out = f.post("fanuc-generic-lathe", options);
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        CHECK_THAT(out.gcode, ContainsSubstring("G1 G12.1"));
        CHECK_THAT(out.gcode, ContainsSubstring("G13.1"));
        CHECK_THAT(out.gcode, !ContainsSubstring("G112"));
        CHECK_THAT(out.gcode, ContainsSubstring("M29 S"));
        MillTurn::matchesBackplot(out);
    }
    SECTION("a generic post asks for the machine's codes") {
        const auto out = f.post("fanuc-generic-lathe");
        REQUIRE_FALSE(out.ok());
        CHECK_THAT(out.errors.front(), ContainsSubstring("your machine's codes"));
    }
    SECTION("feeds leave as the operation sets them") {
        const toolpath::Toolpath pocket = f.generate(kPocket);
        const auto out = f.post("haas-lathe");
        REQUIRE(out.ok());
        double feed = 0.0;
        for (const auto& m : pocket.moves) {
            if (m.kind == toolpath::MoveKind::Linear && m.feedClass == toolpath::FeedClass::Cut) {
                feed = m.feedRate;
                break;
            }
        }
        REQUIRE(feed > 0.0);
        CHECK_THAT(out.gcode, ContainsSubstring(fmt::format("F{:.0f}.", feed * 60.0)));
    }
}

TEST_CASE("another lathe setup goes on from the milling", "[document][lathe][live][mill]") {
    using namespace fixtures::millturn;
    MillTurn f;
    const Uuid firstId = f.setup().id;
    const Operation turning = f.setup().operations[kFinish];
    f.doc.modify("second setup", [&](Project& p) {
        Setup two = p.setups.front();
        two.id = Uuid::generate();
        two.name = "Goes on";
        two.operations = {turning};
        two.operations.front().id = Uuid::generate();
        two.stock.kind = StockKind::FromSetup;
        two.stock.sourceSetup = firstId;
        p.setups.push_back(two);
    });
    const Setup& second = f.doc.project().setups.back();
    const SetupSimulation first = simulateSetup(f.doc, firstId, {}, f.cancel, f.progress);
    const SetupSimulation sim = simulateSetup(f.doc, second.id, {}, f.cancel, f.progress);
    REQUIRE(sim.operations.size() == 1);
    CHECK(sim.operations[0].error.empty());
    REQUIRE(sim.milledFrom);
    CHECK(*sim.milledFrom == 0);
    CHECK_FALSE(sim.playsOnProfile(0));
    SECTION("it starts from what the milling left, the flat and pocket included") {
        const double left = first.finalStock().volume();
        CHECK(sim.initialStock.volume() == Catch::Approx(left).epsilon(0.005));
        const double profile = sim.latheInitial->volume();
        CHECK(sim.initialStock.volume() < profile - 100.0);
    }
    SECTION("turning clips it and the milling stays") {
        CHECK(sim.finalStock().volume() <= sim.initialStock.volume() + 1e-6);
        CHECK(sim.finalStock().volume() > 0.9 * sim.initialStock.volume());
    }
    SECTION("nothing warns of leaving the milling out") {
        for (const std::string& w : sim.warnings) {
            CHECK_THAT(w, !ContainsSubstring("leaves out"));
        }
    }
    SECTION("the comparison reads no gouge") {
        REQUIRE(sim.comparison);
        CHECK_FALSE(sim.comparison->hasGouges());
    }
    SECTION("the cache keeps it") {
        const std::string bytes = encodeSimulation(sim, "key");
        const auto back = decodeSimulation(bytes, "key", f.doc, second);
        REQUIRE(back);
        CHECK(back->milledFrom == sim.milledFrom);
        CHECK(back->initialStock.volume() == Catch::Approx(sim.initialStock.volume()));
    }
    SECTION("a setup after it starts from the same stock") {
        CHECK(sim.lastStock(true).volume() == Catch::Approx(sim.finalStock().volume()));
    }
}

TEST_CASE("a live tool traces edges on a lathe's face", "[document][lathe][live][mill][trace]") {
    using namespace fixtures::millturn;
    MillTurn f;
    // Two edges of the pocket's mouth on the face, meeting at a corner: an open chain. The
    // pocket's own tool and preset, after it.
    const cad::Model& model = *f.doc.model(*f.setup().modelId);
    std::vector<ShapeRef> ell;
    for (int i = 1; i <= cad::subShapeCount(model, cad::SubShapeType::Edge); ++i) {
        const auto sig = cad::signatureOf(model, cad::SubShapeType::Edge, i);
        const Vec3 c = sig.centroid;
        if (std::abs(c.z() - kLength) < 1e-6 && std::abs(sig.measure - kPocketSide) < 1e-6 &&
            (std::abs(c.y() + kPocketSide / 2.0) < 1e-6 ||
             std::abs(c.x() - kPocketSide / 2.0) < 1e-6)) {
            ell.push_back({*f.setup().modelId, cad::SubShapeType::Edge, i, sig});
        }
    }
    REQUIRE(ell.size() == 2);
    Operation trace = f.setup().operations[kPocket];
    trace.id = Uuid::generate();
    trace.name = "Trace";
    trace.type = "trace2d";
    trace.geometry = ell;
    trace.parameters = Json{{"depth", 0.5}};
    trace.heights = {};
    f.doc.modify("trace", [&](Project& p) {
        auto& ops = p.setups.front().operations;
        ops.insert(ops.begin() + static_cast<std::ptrdiff_t>(kPocket) + 1, trace);
    });
    constexpr std::size_t kTrace = kPocket + 1;
    REQUIRE(f.setup().operations[kTrace].type == "trace2d");
    SECTION("polar interpolation, on the edges, the depth below the face") {
        const toolpath::Toolpath tp = f.generate(kTrace);
        CHECK(tp.live);
        CHECK(tp.liveAxes == toolpath::LiveAxes::Polar);
        bool cut = false;
        for (const auto& m : tp.moves) {
            if (m.kind == toolpath::MoveKind::Linear && m.feedClass == toolpath::FeedClass::Cut) {
                cut = true;
                CHECK(m.end.z() == Catch::Approx(-0.5).margin(1e-6)); // the WCS is on the face
                const bool front = std::abs(m.end.y() + kPocketSide / 2.0) < 1e-6;
                const bool side = std::abs(m.end.x() - kPocketSide / 2.0) < 1e-6;
                CHECK((front || side));
            }
        }
        CHECK(cut);
    }
    SECTION("without live tooling it is refused") {
        f.machine("generic-2axis-lathe");
        CHECK_THROWS_WITH(f.generate(kTrace), ContainsSubstring("no live tooling"));
    }
    SECTION("the simulation shows it cutting, and nothing hits") {
        const SetupSimulation sim = simulateSetup(f.doc, f.setup().id, {}, f.cancel, f.progress);
        REQUIRE(sim.operations.size() == 11);
        const auto& op = sim.operations[kTrace];
        INFO(op.error);
        CHECK(op.error.empty());
        CHECK(op.collisions.empty());
        CHECK(op.removedVolume > 1.0);
        // The groove is meant: not a gouge.
        REQUIRE(sim.comparison);
        CHECK_FALSE(sim.comparison->hasGouges());
    }
    SECTION("Haas: in the program under polar interpolation, and the back-plot agrees") {
        const auto out = f.post("haas-lathe");
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        const auto at = out.gcode.find("(TRACE)");
        const auto next = out.gcode.find("(COUNTERBORE)");
        REQUIRE(at != std::string::npos);
        REQUIRE(next > at);
        const std::string program = out.gcode.substr(at, next - at);
        CHECK_THAT(program, ContainsSubstring("G112"));
        CHECK_THAT(program, ContainsSubstring("G113"));
        MillTurn::matchesBackplot(out);
    }
}

namespace {

struct Wrapped {
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "wrapped_project.ocam");
    static constexpr std::size_t kSlot = 3;
    const Setup& setup() const { return doc.project().setups.front(); }
    toolpath::Toolpath generate() {
        return generateToolpath(doc, setup(), setup().operations[kSlot], cancel, progress);
    }
    PostOutput post(const char* id, const Json& options = Json::object()) {
        const machine::Machine& m = machineFor(doc.project(), setup());
        return postSetup(doc, setup().id, m, post::builtinPost(id), cancel, progress, options);
    }
};

} // namespace

TEST_CASE("wrapping a pocket around a lathe's diameter", "[document][lathe][live][mill][wrap]") {
    using namespace fixtures::wrapped;
    Wrapped f;
    const toolpath::Toolpath tp = f.generate();
    SECTION("the toolpath stays on the slot: the big diameter's floor, not the neck") {
        CHECK(tp.live);
        CHECK(tp.liveAxes == toolpath::LiveAxes::Wrap);
        REQUIRE_FALSE(tp.moves.empty());
        const double floor = kDiameter / 2.0 - kSlotDepth;
        double deepest = 1e9;
        double lowest = 1e9, highest = -1e9, nearest = 1e9;
        // WCS Z is 0 at the part's front: the slot from the back.
        for (const toolpath::Move& m : tp.moves) {
            if (m.kind == toolpath::MoveKind::Linear) {
                deepest = std::min(deepest, m.end.head<2>().norm());
                if (m.end.head<2>().norm() < floor + 1e-3) {
                    const double t = std::atan2(m.end.y(), m.end.x());
                    lowest = std::min(lowest, t);
                    highest = std::max(highest, t);
                    nearest = std::min(nearest, std::abs(t));
                }
                const double z = m.end.z() + kLength;
                CHECK(z >= kSlotZ0 - 3.0);
                CHECK(z <= kSlotZ1 + 3.0);
                // The tool's axis is radial.
                CHECK((m.axis - Vec3(m.end.x(), m.end.y(), 0.0).normalized()).norm() < 1e-6);
            }
        }
        CHECK(deepest == Catch::Approx(floor).margin(1e-3));
        // One pass from wall to wall, the tool's radius from each, and across the middle: the slot
        // is centred where the mesh has an edge and the map a column.
        const double reach = kSlotAngle / 2.0 - 3.0 / floor;
        CHECK(lowest == Catch::Approx(-reach).margin(1e-3));
        CHECK(highest == Catch::Approx(reach).margin(1e-3));
        CHECK(nearest < 0.1);
    }
    SECTION("the machine without live tooling refuses") {
        f.doc.modify("m", [&](Project& p) { p.setups.front().machine = "generic-2axis-lathe"; });
        CHECK_THROWS_WITH(f.generate(), ContainsSubstring("no live tooling"));
    }
    SECTION("a mill with no rotary axis refuses") {
        f.doc.modify("m", [&](Project& p) { p.setups.front().machine = "generic-3axis-vmc"; });
        CHECK_THROWS_WITH(f.generate(), ContainsSubstring("no rotary axis"));
    }
    SECTION("a 4th axis takes it with the shaft on its axis, and refuses it across") {
        f.doc.modify("m", [&](Project& p) {
            Setup& s = p.setups.front();
            s.machine = "generic-4axis-vmc";
            s.post.reset();
            std::erase_if(s.operations,
                          [](const Operation& o) { return o.type != "rotary_pocket"; });
        });
        const auto slot = [&] {
            return generateToolpath(f.doc, f.setup(), f.setup().operations.front(), f.cancel,
                                    f.progress);
        };
        // As it is on the lathe, standing on Z with WCS X across it.
        CHECK_THROWS_WITH(slot(), ContainsSubstring("does not go around the WCS X axis"));
        f.doc.modify("m", [&](Project& p) {
            Setup& s = p.setups.front();
            s.wcs.origin = Vec3::Zero();
            s.wcs.xAxis = Vec3::UnitZ();
            s.wcs.zAxis = -Vec3::UnitX();
            SetupPlacement placement;
            placement.carrier = "A";
            placement.rotation = Vec3(0.0, 90.0, 0.0);
            s.placement = placement;
        });
        const toolpath::Toolpath on = slot();
        REQUIRE_FALSE(on.moves.empty());
        for (const toolpath::Move& m : on.moves) {
            CHECK(std::abs(m.axis.x()) < 1e-9); // radial about WCS X
        }
        const machine::Machine& m = machineFor(f.doc.project(), f.setup());
        const PostOutput out = postSetup(f.doc, f.setup().id, m, f.cancel, f.progress);
        REQUIRE(out.ok());
        CHECK_THAT(out.gcode, ContainsSubstring(" A"));
    }
    SECTION("the simulation cuts the slot and nothing else") {
        const SetupSimulation sim = simulateSetup(f.doc, f.setup().id, {}, f.cancel, f.progress);
        REQUIRE(sim.operations.size() == 4);
        for (const auto& op : sim.operations) {
            INFO(op.name << ": " << op.error);
            CHECK(op.error.empty());
            CHECK(op.collisions.empty());
        }
        const double r = kDiameter / 2.0;
        const double slot = (r * r - (r - kSlotDepth) * (r - kSlotDepth)) / 2.0 * kSlotAngle *
                            (kSlotZ1 - kSlotZ0);
        // Less only the corners a round tool leaves.
        CHECK(sim.operations[Wrapped::kSlot].removedVolume == Catch::Approx(slot).epsilon(0.05));
        REQUIRE(sim.comparison);
        CHECK_FALSE(sim.comparison->hasGouges());
    }
    SECTION("only what is left to cut shows a colour, part way and finished") {
        // At size is by where a surface is along its Z ray (DL-0075): the rays through the slot
        // run along the diameter and the shoulder, which stay green.
        const SetupSimulation sim = simulateSetup(f.doc, f.setup().id, {}, f.cancel, f.progress);
        REQUIRE(sim.partReference);
        const auto& op = sim.operations[Wrapped::kSlot];
        sim::Stock partWay = sim.stockBefore(Wrapped::kSlot);
        sim::SimulateOptions moves;
        moves.checkCollisions = false;
        sim::simulateMoves(partWay, *op.tool, op.toolpath, 0, 320, moves);
        std::vector<const sim::Stock*> starts;
        for (std::size_t j = 0; j <= Wrapped::kSlot; ++j) {
            starts.push_back(&sim.stockBefore(j));
        }
        const sim::Rgb green = sim::comparisonColour(0.0, 0.0, sim.partReference->tolerance, 2.0);
        const float front = static_cast<float>(-kLength);
        for (const sim::Stock* stock : std::array<const sim::Stock*, 2>{&partWay, &op.stockAfter}) {
            sim::DisplayOptions options;
            options.maxCells = 800; // as the panel has it
            sim::StockDisplay display(options);
            display.update(*stock, sim.partReference.get(), static_cast<int>(Wrapped::kSlot), true,
                           nullptr, &starts);
            int bySlot = 0, elsewhere = 0;
            for (const auto& chunk : display.chunks()) {
                if (!chunk) {
                    continue;
                }
                for (std::size_t v = 0; v < chunk->positions.size(); ++v) {
                    if (chunk->colours[v] == green) {
                        continue;
                    }
                    const auto& p = chunk->positions[v];
                    const double z = p[2] - front;
                    const double r = std::hypot(p[0], p[1]);
                    const bool slot = z > kSlotZ0 - 0.3 && z < kSlotZ1 + 0.3 &&
                                      r > kDiameter / 2.0 - kSlotDepth - 0.3 &&
                                      std::abs(std::atan2(p[1], p[0])) < kSlotAngle / 2.0 + 0.05;
                    // The insert's nose leaves its radius in the corner at the shoulder.
                    const bool nose = std::abs(z - (kLength - kNeckLength)) < 0.6 &&
                                      r < kNeckDiameter / 2.0 + 0.6;
                    if (slot) {
                        ++bySlot;
                    } else if (!nose) {
                        ++elsewhere;
                    }
                }
            }
            CHECK(bySlot > 0);
            CHECK(elsewhere == 0);
        }
    }
    SECTION("Haas: G107 on the floor's radius, Y a length, the feed on the page, the back-plot agrees") {
        const PostOutput out = f.post("haas-lathe");
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        INFO(out.gcode);
        CHECK_THAT(out.gcode, ContainsSubstring("G19"));
        CHECK_THAT(out.gcode, ContainsSubstring(fmt::format("G107 Y0 C0 R{:.0f}.", kDiameter / 2.0 - kSlotDepth)));
        CHECK_THAT(out.gcode, ContainsSubstring("M154"));
        CHECK_THAT(out.gcode, ContainsSubstring("M133 P"));
        for (const toolpath::Move& m : tp.moves) {
            if (m.kind == toolpath::MoveKind::Linear && m.feedClass == toolpath::FeedClass::Cut) {
                CHECK_THAT(out.gcode, ContainsSubstring(fmt::format("F{:.0f}", m.feedRate * 60.0)));
                break;
            }
        }
        double deviation = 0.0;
        const auto differs = post::compareBackplot(
            out.toolpaths, post::backplot(out.gcode, {true}).path, 2e-3, &deviation, false);
        INFO(differs.value_or(""));
        CHECK_FALSE(differs.has_value());
    }
    SECTION("Fanuc: G7.1 with the radius, C in degrees, the back-plot agrees") {
        const Json options{{"program",
                            {{"liveToolOn", "M133 P{rpm}"},
                             {"liveToolOff", "M135"},
                             {"cAxisOn", "M154"},
                             {"cAxisOff", "M155"}}}};
        const PostOutput out = f.post("fanuc-generic-lathe", options);
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        INFO(out.gcode);
        CHECK_THAT(out.gcode, ContainsSubstring(fmt::format("G7.1 C{:.0f}.", kDiameter / 2.0 - kSlotDepth)));
        CHECK_THAT(out.gcode, ContainsSubstring("G7.1 C0"));
        CHECK_THAT(out.gcode, !ContainsSubstring("G107"));
        double deviation = 0.0;
        const auto differs = post::compareBackplot(
            out.toolpaths, post::backplot(out.gcode, {true}).path, 2e-3, &deviation, false);
        INFO(differs.value_or(""));
        CHECK_FALSE(differs.has_value());
    }
    SECTION("empty codes are refused, naming them") {
        const Json options{{"program", {{"wrapOn", ""}}}};
        const PostOutput out = f.post("haas-lathe", options);
        REQUIRE_FALSE(out.ok());
        CHECK_THAT(out.errors.front(), ContainsSubstring("Cylindrical interpolation on"));
    }
}
