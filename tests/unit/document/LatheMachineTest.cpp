// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A lathe setup on a machine with a turret, chuck, tailstock and enclosure (DL-0067): the shaft
// fixture turned, every tool on the turret. Timings: test_document "[lathemachinebench]".

#include "FixtureSpecs.hpp"
#include "TestProject.hpp"

#include <ocam/document/Generate.hpp>
#include <ocam/document/LatheFitting.hpp>
#include <ocam/document/LatheMachine.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/ProjectIo.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/document/SimulationCache.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/machine/Turret.hpp>
#include <ocam/post/Engine.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>

using namespace ocam;
using namespace ocam::document;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace {

/// The shaft faced, roughed and finished by T1 and parted off by T5, on the machine named, with
/// its bar long enough to stand clear of the machine's chuck.
struct ShaftOnTurret {
    explicit ShaftOnTurret(const char* machineId = "generic-lathe-turret") {
        using namespace fixtures::shaft;
        const Uuid modelId = doc.importModel(std::filesystem::path(OCAM_FIXTURE_DIR) / "shaft.step",
                                             cancel, progress);
        setupId = doc.addSetup(modelId);
        tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
        turn.name = "Turn";
        turn.insert.noseRadius = 0.4;
        tooling::Tool part = tooling::makeDefaultTool(tooling::ToolType::PartingInsert, 5);
        part.name = "Part";
        part.insert.width = 3.0;
        part.insert.noseRadius = 0.1;
        doc.modify("lathe setup", [&](Project& p) {
            Setup* s = p.findSetup(setupId);
            s->machine = machineId;
            s->originFrom.reset();
            s->wcs.origin = Vec3(0, 0, kMajorLength + kMinorLength);
            s->wcs.zAxis = Vec3::UnitZ();
            s->wcs.xAxis = Vec3::UnitX();
            s->stock.kind = StockKind::Box;
            s->stock.size = StockSize::OffsetFromPart;
            s->stock.offsetSide = 1.0;
            s->stock.offsetTop = 1.0;
            s->stock.offsetBottom = 40.0;
            p.tools.tools = {testing::withPreset(turn), testing::withPreset(part)};
            auto add = [&](const char* name, const char* type, const tooling::Tool& tool,
                           PresetOverrides po = {}) {
                Operation op;
                op.name = name;
                op.type = type;
                op.toolId = tool.id;
                op.presetId = tool.presets.front().id;
                op.parameters = Json::object();
                op.presetOverrides = po;
                s->operations.push_back(op);
            };
            add("Face", "turn_face", p.tools.tools[0]);
            add("Rough", "turn_rough", p.tools.tools[0], testing::stepDown(1.5));
            add("Finish", "turn_finish", p.tools.tools[0]);
            add("Part off", "turn_partoff", p.tools.tools[1]);
        });
    }

    /// A tool that rides along at `station`.
    Uuid mount(tooling::Tool tool, int station, std::optional<Vec2> tip = std::nullopt) {
        tool.number = station;
        const Uuid id = tool.id;
        doc.modify("mount", [&](Project& p) {
            p.tools.tools.push_back(tool);
            TurretEntry entry;
            entry.tool = id;
            entry.tip = tip;
            p.findSetup(setupId)->turret.push_back(entry);
        });
        return id;
    }

    /// Moves a tool used by the job to where it would hit something.
    void setTip(const Uuid& tool, const Vec2& tip) {
        doc.modify("tip", [&](Project& p) {
            auto& turret = p.findSetup(setupId)->turret;
            const auto it = std::find_if(turret.begin(), turret.end(),
                                         [&](const TurretEntry& e) { return e.tool == tool; });
            if (it == turret.end()) {
                turret.push_back(TurretEntry{tool, std::nullopt, tip, std::nullopt});
            } else {
                it->tip = tip;
            }
        });
    }

    /// A copy of the machine in the project, changed.
    void withMachine(const std::function<void(machine::Machine&)>& change) {
        doc.modify("machine", [&](Project& p) {
            machine::Machine m = machineFor(p, *p.findSetup(setupId));
            m.id += "-changed";
            change(m);
            p.findSetup(setupId)->machine = m.id;
            p.machines.push_back(std::move(m));
        });
    }

    SetupSimulation simulate() {
        SimulationOptions options;
        return simulateSetup(doc, setupId, options, cancel, progress);
    }

    Document doc;
    Uuid setupId;
    CancelToken cancel;
    NullProgress progress;
};

tooling::Tool longDrill(double length = 120.0) {
    tooling::Tool t = tooling::makeDefaultTool(tooling::ToolType::Drill, 3);
    t.name = "Drill";
    t.mill.diameter = 12.0;
    t.mill.shankDiameter = 12.0;
    t.mill.fluteLength = 70.0;
    t.mill.overallLength = length;
    return testing::withPreset(t);
}

const OperationSimulation& named(const SetupSimulation& sim, const char* name) {
    const auto it = std::find_if(sim.operations.begin(), sim.operations.end(),
                                 [&](const OperationSimulation& o) { return o.name == name; });
    REQUIRE(it != sim.operations.end());
    return *it;
}

/// Everything reported on the machine in the simulation, as a line each, for the failure message.
std::string report(const SetupSimulation& sim) {
    std::string text;
    for (const OperationSimulation& op : sim.operations) {
        for (const sim::MachineCollision& c : op.machineCollisions) {
            text += op.name + ": " + c.first + " / " + c.second + " moves " +
                    std::to_string(c.firstMove) + "-" + std::to_string(c.lastMove) +
                    (c.during.empty() ? "" : " during " + c.during) + "\n";
        }
        if (!op.error.empty()) {
            text += op.name + ": " + op.error + "\n";
        }
    }
    return text;
}

} // namespace

TEST_CASE("a clean job on the turret: the machine moves and reports nothing",
          "[document][lathe][machine]") {
    ShaftOnTurret f;
    f.mount(longDrill(), 3);
    const SetupSimulation sim = f.simulate();
    INFO(report(sim));
    REQUIRE_FALSE(sim.hasErrors());
    CHECK(sim.machineCollisionCount() == 0);
    REQUIRE(sim.machine);
    REQUIRE(sim.turretTools.size() == 3);
    CHECK(sim.turretTools[0].station == 1);
    CHECK(sim.turretTools[1].station == 3);
    CHECK(sim.turretTools[2].station == 5);
    CHECK(sim.turretTools[1].name == "T03 Drill");

    const OperationSimulation& face = named(sim, "Face");
    REQUIRE(face.latheMotion);
    REQUIRE(face.mount);
    CHECK(face.mount->carrier == "C");
    CHECK(face.latheMotion->station == 1);
    // The bar's back (41 mm past the part's 80) in the chuck by its grip: at the machine's Z 65.
    CHECK(face.mount->wcs.translation().z() - 80.0 - 40.0 == Approx(65.0));

    // No tool change from face to rough; to part off, an index before anything else moves.
    const OperationSimulation& rough = named(sim, "Rough");
    REQUIRE(rough.latheMotion);
    for (const machine::LatheStep& step : rough.latheMotion->steps) {
        CHECK(step.kind == machine::LatheStep::Kind::Rapid);
    }
    const OperationSimulation& part = named(sim, "Part off");
    REQUIRE(part.latheMotion);
    REQUIRE(part.latheMotion->steps.size() == 3);
    CHECK(part.latheMotion->station == 5);
    CHECK(part.latheMotion->steps[0].kind == machine::LatheStep::Kind::Index);
    CHECK(part.latheMotion->steps[0].to.t == Approx(2.0 * kPi * 4.0 / 12.0));
    CHECK(part.latheMotion->steps[0].to.linear.isApprox(part.latheMotion->start.linear));
    // Along Z, then in on X.
    CHECK(part.latheMotion->steps[1].to.linear.x() == part.latheMotion->start.linear.x());
    CHECK(part.latheMotion->steps[2].to.linear.z() == part.latheMotion->steps[1].to.linear.z());

    // The axes put the tool on the program's point.
    const machine::MachineModel model = machine::effectiveModel(*sim.machine);
    const Vec3 tip = rough.toolpath.moves.back().end;
    const auto axes =
        machine::latheAxes(model, *rough.latheMotion, *rough.mount, rough.toolpath, tip);
    REQUIRE(axes);
    CHECK((rough.latheMotion->tip + axes->linear - (*rough.mount).wcs * tip).norm() < 1e-9);
}

TEST_CASE("a job with no layout simulates as before", "[document][lathe][machine]") {
    ShaftOnTurret f("generic-2axis-lathe");
    const SetupSimulation sim = f.simulate();
    REQUIRE_FALSE(sim.hasErrors());
    CHECK_FALSE(sim.machine);
    CHECK(sim.turretTools.empty());
    for (const OperationSimulation& op : sim.operations) {
        CHECK_FALSE(op.latheMotion);
        CHECK_FALSE(op.mount);
    }
}

TEST_CASE("a tool riding along meets the chuck at the operation and move it happens",
          "[document][lathe][machine]") {
    ShaftOnTurret f;
    // A drill in station 2, 30 degrees round, its tip 40 mm nearer the chuck than the tool
    // cutting's: within the chuck's radius there, it reaches the chuck well down the shaft.
    f.mount(longDrill(), 2, Vec2(170.0, -100.0));
    const SetupSimulation sim = f.simulate();
    INFO(report(sim));
    const OperationSimulation& rough = named(sim, "Rough");
    const auto hit =
        std::find_if(rough.machineCollisions.begin(), rough.machineCollisions.end(),
                     [](const sim::MachineCollision& c) { return c.second == "Chuck"; });
    REQUIRE(hit != rough.machineCollisions.end());
    CHECK_THAT(hit->first, ContainsSubstring("T02 Drill"));
    CHECK(hit->during.empty());
    CHECK(hit->firstMove > 0);
    CHECK(hit->lastMove < rough.toolpath.moves.size());
    // The chuck's front (90, and 1 of clearance) with the drill's tip there has the tool cutting at
    // the machine's Z 131: the WCS's Z -54 with the WCS's Z 0 at 185.
    CHECK(rough.toolpath.moves[hit->firstMove].end.z() < -50.0);
    CHECK(named(sim, "Face").machineCollisions.empty());
}

TEST_CASE("an index reports with the operation it brings the tool to",
          "[document][lathe][machine]") {
    ShaftOnTurret f;
    const SetupSimulation clean = f.simulate();
    REQUIRE(clean.machineCollisionCount() == 0);
    // Parting off's tool reaching far out: as it comes round where facing left the turret (the tool
    // cutting's tip close to the front), it swings through the bar.
    const Uuid part = f.doc.project().tools.tools[1].id;
    f.setTip(part, Vec2(380.0, -60.0));
    const SetupSimulation sim = f.simulate();
    INFO(report(sim));
    const OperationSimulation& partOff = named(sim, "Part off");
    REQUIRE_FALSE(partOff.machineCollisions.empty());
    const auto hit =
        std::find_if(partOff.machineCollisions.begin(), partOff.machineCollisions.end(),
                     [](const sim::MachineCollision& c) {
                         return c.during == machine::toString(machine::LatheStep::Kind::Index);
                     });
    REQUIRE(hit != partOff.machineCollisions.end());
    CHECK_THAT(hit->first, ContainsSubstring("T05 Part"));
    CHECK(hit->firstMove == 0);
}

TEST_CASE("the enclosure is checked for the turret and the tools", "[document][lathe][machine]") {
    ShaftOnTurret f;
    SECTION("the turret at home against a wall brought in") {
        f.withMachine([](machine::Machine& m) { m.lathe->enclosureMax.x() = 560.0; });
        const SetupSimulation sim = f.simulate();
        INFO(report(sim));
        const OperationSimulation& face = named(sim, "Face");
        REQUIRE_FALSE(face.machineCollisions.empty());
        CHECK(face.machineCollisions[0].first == "Turret");
        CHECK(face.machineCollisions[0].second == "Enclosure, back");
        CHECK(face.machineCollisions[0].during ==
              machine::toString(machine::LatheStep::Kind::Rapid));
    }
    SECTION("a long tool across the turret reaches the roof") {
        // Station 4 stands a quarter of a turn round, its tip up at Y 340 of a roof at Y 320.
        f.mount(longDrill(), 4, Vec2(340.0, -60.0));
        const SetupSimulation sim = f.simulate();
        INFO(report(sim));
        const auto all = named(sim, "Finish").machineCollisions;
        CHECK(std::any_of(all.begin(), all.end(), [](const sim::MachineCollision& c) {
            return c.first.find("T04 Drill") != std::string::npos && c.second == "Enclosure, roof";
        }));
    }
}

TEST_CASE("the tailstock comes in and stays in the way", "[document][lathe][machine]") {
    ShaftOnTurret f;
    LatheTailstock centre;
    centre.tipZ = 1.0;
    centre.centreDiameter = 12.0;
    centre.quillDiameter = 16.0;
    centre.quillLength = 200.0;
    // Facing and parting off cannot run under a tailstock; rough and finish can.
    f.doc.modify("no facing", [&](Project& p) {
        auto& ops = p.findSetup(f.setupId)->operations;
        ops.erase(ops.begin());
        ops.pop_back();
        p.findSetup(f.setupId)->tailstock = centre;
    });
    SECTION("a centre far enough out: it moves, the setup's quill on W, and nothing is hit") {
        const SetupSimulation sim = f.simulate();
        INFO(report(sim));
        REQUIRE_FALSE(sim.hasErrors());
        const OperationSimulation& rough = named(sim, "Rough");
        REQUIRE(rough.latheMotion);
        REQUIRE_FALSE(rough.latheMotion->steps.empty());
        CHECK(rough.latheMotion->steps[0].kind == machine::LatheStep::Kind::Tailstock);
        CHECK(rough.latheMotion->steps[0].to.w < -100.0);
        CHECK(rough.latheMotion->held.w == rough.latheMotion->steps[0].to.w);
        CHECK(sim.machineCollisionCount() == 0);
    }
    SECTION("its quill is the model a view moves with W") {
        const auto supports = latheSupports(f.doc, *f.doc.project().findSetup(f.setupId));
        REQUIRE_FALSE(supports.empty());
        CHECK(supports.front().id == tailstockSupportId());
    }
    SECTION("a quill out further than the machine's goes: said, not shortened") {
        const auto quillSaid = [](const SetupSimulation& sim) {
            return std::any_of(sim.warnings.begin(), sim.warnings.end(), [](const auto& w) {
                return w.find("Quill length") != std::string::npos;
            });
        };
        // A machine that does not say how far its quill goes says nothing.
        CHECK_FALSE(quillSaid(f.simulate()));
        f.withMachine([](machine::Machine& m) { m.lathe->tailstock->quillTravel = 100.0; });
        const Setup& setup = *f.doc.project().findSetup(f.setupId);
        const machine::Machine onMachine = machineFor(f.doc.project(), setup);
        const auto why = quillBeyondTravel(setup, onMachine);
        REQUIRE(why);
        CHECK_THAT(*why, ContainsSubstring("out 200 mm"));
        CHECK_THAT(*why, ContainsSubstring("100 mm at most"));
        CHECK_THAT(*why, ContainsSubstring("'" + onMachine.name + "'"));
        CHECK_THAT(*why, ContainsSubstring("Quill length under the setup's Tailstock"));
        const SetupSimulation sim = f.simulate();
        CHECK(quillSaid(sim));
        // As the setup has it: the quill is still 200 mm out in the setup.
        CHECK(f.doc.project().findSetup(f.setupId)->tailstock->quillLength == 200.0);
        // Within the travel, or at it: nothing.
        f.doc.modify("in",
                     [&](Project& p) { p.findSetup(f.setupId)->tailstock->quillLength = 100.0; });
        CHECK_FALSE(quillBeyondTravel(*f.doc.project().findSetup(f.setupId), onMachine));
        // No tailstock in the setup: nothing.
        Setup without = *f.doc.project().findSetup(f.setupId);
        without.tailstock.reset();
        CHECK_FALSE(quillBeyondTravel(without, onMachine));
    }
    SECTION("a short quill: the disc comes to the body") {
        f.doc.modify("short",
                     [&](Project& p) { p.findSetup(f.setupId)->tailstock->quillLength = 40.0; });
        const SetupSimulation sim = f.simulate();
        INFO(report(sim));
        CHECK(std::any_of(sim.operations.begin(), sim.operations.end(), [](const auto& op) {
            return std::any_of(op.machineCollisions.begin(), op.machineCollisions.end(),
                               [](const sim::MachineCollision& c) {
                                   return c.first == "Turret" && c.second == "Tailstock body";
                               });
        }));
    }
}

TEST_CASE("a turret's load: stations, and two tools at one", "[document][lathe][machine]") {
    ShaftOnTurret f;
    const machine::Machine& m = machineFor(f.doc.project(), *f.doc.project().findSetup(f.setupId));
    TurretLoad load = turretLoad(f.doc, *f.doc.project().findSetup(f.setupId), m);
    REQUIRE(load.stations.size() == 2);
    CHECK(load.problems.empty());
    CHECK(load.stations[0].station == 1);
    CHECK(load.stations[0].used);
    CHECK(load.stations[0].across);
    CHECK_FALSE(load.stations[0].tipSet);
    CHECK(load.stations[0].tip.x() == Approx(m.lathe->turret.radialReach));

    SECTION("a tool that rides along") {
        f.mount(longDrill(), 3);
        load = turretLoad(f.doc, *f.doc.project().findSetup(f.setupId), m);
        REQUIRE(load.stations.size() == 3);
        CHECK(load.stations[1].station == 3);
        CHECK_FALSE(load.stations[1].used);
        CHECK_FALSE(load.stations[1].across);
        CHECK(load.stations[1].tip.x() == Approx(m.lathe->turret.axialPitch));
    }
    SECTION("two tools at one station, and one beyond the turret") {
        f.mount(longDrill(), 5);
        tooling::Tool beyond = longDrill();
        beyond.id = Uuid::generate();
        f.mount(beyond, 13);
        load = turretLoad(f.doc, *f.doc.project().findSetup(f.setupId), m);
        REQUIRE(load.problems.size() == 2);
        std::string all;
        for (const std::string& p : load.problems) {
            all += p + "\n";
        }
        CHECK_THAT(all, ContainsSubstring("both at station 5"));
        CHECK_THAT(all, ContainsSubstring("stations 1 to 12"));
        const SetupSimulation sim = f.simulate();
        std::string warned;
        for (const std::string& w : sim.warnings) {
            warned += w + "\n";
        }
        CHECK_THAT(warned, ContainsSubstring("both at station 5"));
    }
    SECTION("moving a tool to another station is the number the program calls") {
        f.doc.modify("station", [&](Project& p) { p.tools.tools[1].number = 9; });
        load = turretLoad(f.doc, *f.doc.project().findSetup(f.setupId), m);
        REQUIRE(load.stations.size() == 2);
        CHECK(load.stations[1].station == 9);
    }
}

TEST_CASE("the machine's simulation in the project and the cache", "[document][lathe][machine]") {
    ShaftOnTurret f;
    const Uuid drill = f.mount(longDrill(), 3, Vec2(100.0, -150.0));
    SECTION("the setup keeps the turret") {
        const Json saved = toJson(f.doc.project());
        const Project back = projectFromJson(saved);
        const Setup* s = back.findSetup(f.setupId);
        REQUIRE(s->turret.size() == 1);
        CHECK(s->turret[0].tool == drill);
        REQUIRE(s->turret[0].tip);
        CHECK(*s->turret[0].tip == Vec2(100.0, -150.0));
        CHECK_FALSE(s->turret[0].across);
    }
    SECTION("a simulation keeps the turret's tools and motion") {
        const SetupSimulation sim = f.simulate();
        const Setup& setup = *f.doc.project().findSetup(f.setupId);
        const std::string key = simulationInputKey(f.doc, setup, {});
        const std::string bytes = encodeSimulation(sim, key);
        const auto back = decodeSimulation(bytes, key, f.doc, setup);
        REQUIRE(back);
        REQUIRE(back->turretTools.size() == sim.turretTools.size());
        CHECK(back->turretTools[1].name == sim.turretTools[1].name);
        CHECK(back->turretTools[1].solids.size() == sim.turretTools[1].solids.size());
        REQUIRE(back->operations.size() == sim.operations.size());
        for (std::size_t i = 0; i < sim.operations.size(); ++i) {
            CHECK(back->operations[i].latheMotion == sim.operations[i].latheMotion);
        }
    }
    SECTION("a tool riding along is in the simulation's key") {
        const std::string before =
            simulationInputKey(f.doc, *f.doc.project().findSetup(f.setupId), {});
        f.doc.modify("longer",
                     [&](Project& p) { p.tools.findTool(drill)->mill.overallLength = 200.0; });
        CHECK(simulationInputKey(f.doc, *f.doc.project().findSetup(f.setupId), {}) != before);
    }
}

TEST_CASE("the fixture projects on the turret", "[document][lathe][machine]") {
    CancelToken cancel;
    NullProgress progress;
    const auto open = [](const char* file) {
        return Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / file);
    };
    SECTION("clean, with a full turret") {
        const Document doc = open("turret_project.ocam");
        const SetupSimulation sim =
            simulateSetup(doc, doc.project().setups.front().id, {}, cancel, progress);
        INFO(report(sim));
        REQUIRE_FALSE(sim.hasErrors());
        CHECK(sim.machineCollisionCount() == 0);
        CHECK(sim.turretTools.size() == 5);
    }
    SECTION("with a tool reaching for the chuck, and one swinging through the work") {
        const Document doc = open("turret_collisions_project.ocam");
        const SetupSimulation sim =
            simulateSetup(doc, doc.project().setups.front().id, {}, cancel, progress);
        const std::string text = report(sim);
        INFO(text);
        CHECK_THAT(text, ContainsSubstring("T02 Boring bar 20, holder / Chuck"));
        CHECK_THAT(text, ContainsSubstring("T05 Part off 3 mm, insert / part moves 0-0 during the "
                                           "turret's index"));
    }
    SECTION("on a tailstock") {
        const Document doc = open("turret_tailstock_project.ocam");
        const SetupSimulation sim =
            simulateSetup(doc, doc.project().setups.front().id, {}, cancel, progress);
        INFO(report(sim));
        REQUIRE_FALSE(sim.hasErrors());
        CHECK(sim.machineCollisionCount() == 0);
        REQUIRE(sim.operations.front().latheMotion);
        CHECK(sim.operations.front().latheMotion->held.w < 0.0);
    }
    SECTION("taken over by the sub-spindle") {
        const Document doc = open("turret_sub_project.ocam");
        const Setup& second = doc.project().setups.back();
        const SetupSimulation sim = simulateSetup(doc, second.id, {}, cancel, progress);
        INFO(report(sim));
        for (const std::string& w : sim.warnings) {
            UNSCOPED_INFO(w);
        }
        REQUIRE_FALSE(sim.hasErrors());
        const OperationSimulation& face = sim.operations.front();
        REQUIRE(face.mount);
        CHECK(face.mount->carrier == "C2");
        REQUIRE(face.latheMotion);
        // The work's Z runs towards the main spindle.
        CHECK(face.mount->wcs.linear().col(2).isApprox(-Vec3::UnitZ()));
        // In to take it, where the part on the two spindles is the same, and back to cut.
        REQUIRE(face.latheMotion->steps.size() >= 2);
        const machine::LatheStep& in = face.latheMotion->steps[0];
        const machine::LatheStep& back = face.latheMotion->steps[1];
        CHECK(in.kind == machine::LatheStep::Kind::Transfer);
        CHECK(in.workStays);
        CHECK(back.kind == machine::LatheStep::Kind::Transfer);
        CHECK_FALSE(back.workStays);
        const machine::Machine& m = *sim.machine;
        CHECK(back.to.z2 == Approx(m.lathe->subSpindle->workZ2));
        const Setup& first = doc.project().setups.front();
        const SetupSimulation main = simulateSetup(doc, first.id, {}, cancel, progress);
        REQUIRE(main.operations.front().mount);
        const Transform3 mainFromProject =
            main.operations.front().mount->wcs * first.wcs.toParent().inverse();
        const Transform3 subFromProject = Eigen::Translation3d(0.0, 0.0, in.to.z2) *
                                          face.mount->wcs * second.wcs.toParent().inverse();
        CHECK((mainFromProject.matrix() - subFromProject.matrix()).cwiseAbs().maxCoeff() < 1e-9);
        // The sub-spindle's chuck stops short of the main one.
        CHECK(m.lathe->subSpindle->nose + in.to.z2 - m.lathe->subSpindle->chuckLength >
              m.lathe->chuckLength);
        // Facing the back: the tool reaches away from the turret's other side.
        for (const sim::TurretTool& t : sim.turretTools) {
            CHECK(t.placement.linear().col(2).isApprox(-Vec3::UnitZ()));
        }
        CHECK(sim.machineCollisionCount() == 0);
        // Its program is the main setup's, which runs both (ADR-0007).
        const PostOutput posted = postSetup(doc, second.id, m, defaultPostFor(m), cancel, progress);
        REQUIRE_FALSE(posted.errors.empty());
        CHECK_THAT(posted.errors.front(), ContainsSubstring("by a transfer from setup"));
    }
}

TEST_CASE("a setup's own chuck and steady rest on the machine", "[document][lathe][machine]") {
    CancelToken cancel;
    NullProgress progress;
    SECTION("its chuck stands in for the machine's, its back at the nose") {
        Document doc =
            Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "chuck_project.ocam");
        const Uuid id = doc.project().setups.front().id;
        doc.modify("On the turret",
                   [&](Project& p) { p.findSetup(id)->machine = "generic-lathe-turret"; });
        const SetupSimulation sim = simulateSetup(doc, id, {}, cancel, progress);
        INFO(report(sim));
        REQUIRE_FALSE(sim.hasErrors());
        CHECK_FALSE(sim.machineChuck);
        REQUIRE(sim.operations.front().mount);
        double back = 1e9;
        for (const sim::LatheBody& b : latheBodies(doc, *doc.project().findSetup(id), 0.0)) {
            for (const geom2d::Region& r : b.area) {
                back = std::min(back, r.bounds().min.x());
            }
        }
        CHECK(sim.operations.front().mount->wcs.translation().z() + back ==
              Approx(0.0).margin(1e-6));
        SECTION("on the machine's workholding mount: a back plate in front of the nose") {
            doc.modify("Back plate", [&](Project& p) {
                machine::Machine m = machine::genericLatheTurret();
                m.id += "-back-plate";
                m.lathe->workholdingMount = 45.0;
                p.findSetup(id)->machine = m.id;
                p.machines.push_back(std::move(m));
            });
            const Setup& setup = *doc.project().findSetup(id);
            const machine::WorkMount on =
                latheWorkMount(doc, setup, machineFor(doc.project(), setup), {});
            CHECK(on.wcs.translation().z() + back == Approx(45.0));
        }
    }
    SECTION("the machine's own chuck stands on the mount, and the work in it") {
        machine::Machine m = machine::genericLatheTurret();
        const machine::Machine plain = m;
        m.lathe->workholdingMount = 45.0;
        const auto chuckBase = [](const machine::Machine& on) {
            for (const machine::LatheSolid& s : machine::latheSolids(on)) {
                if (s.shape.name == machine::kLatheChuckPart) {
                    return s.shape.base.z();
                }
            }
            return -1.0;
        };
        CHECK(chuckBase(plain) == Approx(0.0));
        CHECK(chuckBase(m) == Approx(45.0));
        ShaftOnTurret f;
        const Setup& setup = *f.doc.project().findSetup(f.setupId);
        const geom2d::Regions bar{
            {geom2d::Polyline::rectangle(Vec2(-50.0, 0.0), Vec2(100.0, 20.0)), {}}};
        CHECK(latheWorkMount(f.doc, setup, m, bar).wcs.translation().z() ==
              Approx(latheWorkMount(f.doc, setup, plain, bar).wcs.translation().z() + 45.0));
    }
    SECTION("a steady rest in the way of a tool riding along") {
        ShaftOnTurret f;
        // Facing and parting off are not for a job with a rest; the tool in station 2 stands
        // at Y 95, 30 mm nearer the chuck than the tool cutting: a rest of radius 150 reaches it.
        LatheSteadyRest rest;
        rest.z = -30.0;
        rest.width = 20.0;
        rest.outerDiameter = 300.0;
        f.doc.modify("Rest", [&](Project& p) {
            p.findSetup(f.setupId)->steadyRest = rest;
            auto& ops = p.findSetup(f.setupId)->operations;
            ops.pop_back();
        });
        tooling::Tool shank = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 2);
        shank.name = "Beside";
        f.mount(testing::withPreset(shank), 2, Vec2(190.0, -100.0));
        const SetupSimulation sim = f.simulate();
        const std::string text = report(sim);
        INFO(text);
        CHECK_THAT(text, ContainsSubstring("T02 Beside, insert / Steady rest"));
    }
    SECTION("a live tool on the main spindle of a lathe with a sub-spindle") {
        Document doc =
            Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "turret_live_project.ocam");
        const Uuid id = doc.project().setups.front().id;
        doc.modify("With a sub-spindle", [&](Project& p) {
            Setup* s = p.findSetup(id);
            s->machine = "generic-lathe-sub";
            SetupPlacement main;
            main.carrier = "C";
            s->placement = main;
        });
        const Setup& setup = *doc.project().findSetup(id);
        const machine::Machine& m = machineFor(doc.project(), setup);
        // The refusal, if any, is for the post's codes: not that a turned C leaves the work behind.
        const PostOutput posted =
            postSetup(doc, id, m, post::builtinPost("haas-lathe"), cancel, progress);
        for (const std::string& e : posted.errors) {
            CHECK_THAT(e, !ContainsSubstring("does not sit on it"));
        }
    }
    SECTION("live tools: the machine turns the work, the tool stays in its plane") {
        const Document doc =
            Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "turret_live_project.ocam");
        for (const Setup& setup : doc.project().setups) {
            const SetupSimulation sim = simulateSetup(doc, setup.id, {}, cancel, progress);
            INFO(report(sim));
            const machine::MachineModel model = machine::effectiveModel(*sim.machine);
            bool turned = false;
            for (const OperationSimulation& op : sim.operations) {
                if (!op.error.empty() || !op.toolpath.live || !op.latheMotion) {
                    continue;
                }
                for (const toolpath::Move& m : op.toolpath.moves) {
                    const auto axes =
                        machine::latheAxes(model, *op.latheMotion, *op.mount, op.toolpath, m.end);
                    REQUIRE(axes);
                    turned = turned || std::abs(axes->rotary.c) > 1e-6;
                    // Through the C it says, the work comes to the tool.
                    const Vec3 work =
                        machine::poseAt(model, *axes).carried(model, "C") * (*op.mount).wcs * m.end;
                    CHECK((op.latheMotion->tip + axes->linear - work).norm() < 1e-6);
                }
            }
            CHECK(turned);
        }
    }
}

TEST_CASE("a turret that indexes at home goes there in the machine and the program",
          "[document][lathe][machine]") {
    ShaftOnTurret f;
    f.withMachine([](machine::Machine& m) { m.lathe->turret.indexAtHome = true; });
    const SetupSimulation sim = f.simulate();
    INFO(report(sim));
    const OperationSimulation& part = named(sim, "Part off");
    REQUIRE(part.latheMotion);
    REQUIRE(part.latheMotion->steps.size() == 5);
    CHECK(part.latheMotion->steps[0].kind == machine::LatheStep::Kind::Rapid);
    CHECK(part.latheMotion->steps[0].to.linear.x() == 0.0);
    CHECK(part.latheMotion->steps[1].to.linear.isZero());
    CHECK(part.latheMotion->steps[2].kind == machine::LatheStep::Kind::Index);
    const machine::Machine& m = machineFor(f.doc.project(), *f.doc.project().findSetup(f.setupId));
    const PostOutput posted =
        postSetup(f.doc, f.setupId, m, defaultPostFor(m), f.cancel, f.progress);
    REQUIRE(posted.errors.empty());
    const auto toolChange = posted.gcode.find("T0505");
    REQUIRE(toolChange != std::string::npos);
    const auto u = posted.gcode.rfind("G28 U0.", toolChange);
    const auto w = posted.gcode.rfind("G28 W0.", toolChange);
    REQUIRE(u != std::string::npos);
    REQUIRE(w != std::string::npos);
    CHECK(u < w);
    // Not before the first tool.
    CHECK(posted.gcode.find("G28") > posted.gcode.find("T0101"));
}

// What the moves between operations meet, on every lathe fixture put on the turret machine:
//   test_document "[approachsurvey]"
// And in the lathe plane, with the tool's tip alone: whether the first leg of each approach between
// operations of one tool goes through the stock.
TEST_CASE("the approach on every lathe fixture", "[.][approachsurvey]") {
    CancelToken cancel;
    NullProgress progress;
    for (const char* file :
         {"shaft_project.ocam", "bored_project.ocam", "holders_project.ocam", "flip_project.ocam",
          "threads_project.ocam", "grooved_project.ocam", "ranges_project.ocam",
          "backturn_project.ocam", "nose_project.ocam", "centres_project.ocam",
          "chuck_project.ocam", "live_project.ocam", "millturn_project.ocam", "turret_project.ocam",
          "turret_tailstock_project.ocam", "turret_sub_project.ocam"}) {
        Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / file);
        doc.modify("On the turret", [&](Project& p) {
            for (Setup& s : p.setups) {
                if (s.machine.rfind("generic-lathe", 0) == 0 ||
                    s.machine == "generic-2axis-lathe") {
                    s.machine = s.machine == "generic-lathe-sub"
                                    ? s.machine
                                    : std::string("generic-lathe-turret");
                }
            }
        });
        for (const Setup& setup : doc.project().setups) {
            if (!machineFor(doc.project(), setup).lathe) {
                continue;
            }
            const SetupSimulation sim = simulateSetup(doc, setup.id, {}, cancel, progress);
            std::printf("%s / %s\n", file, setup.name.c_str());
            Vec3 at(0, 0, 0);
            bool have = false;
            for (std::size_t i = 0; i < sim.operations.size(); ++i) {
                const OperationSimulation& op = sim.operations[i];
                if (!op.error.empty() || op.toolpath.moves.empty()) {
                    continue;
                }
                const Vec3 start = op.toolpath.start;
                // The tip's way, as if the same tool went on: is the leg in the stock?
                const auto before = sim.latheBefore(i);
                auto cuts = [&](const Vec2& a, const Vec2& b) {
                    if (!before) {
                        return false;
                    }
                    for (int k = 1; k < 200; ++k) {
                        const Vec2 q = a + (b - a) * (k / 200.0);
                        for (const geom2d::Region& r : before->profile()) {
                            if (r.contains(q)) {
                                return true;
                            }
                        }
                    }
                    return false;
                };
                const double rNow = std::hypot(at.x(), at.y());
                const double rTo = std::hypot(start.x(), start.y());
                const bool zFirst = have && (cuts(Vec2(at.z(), rNow), Vec2(start.z(), rNow)) ||
                                             cuts(Vec2(start.z(), rNow), Vec2(start.z(), rTo)));
                const bool xFirst = have && (cuts(Vec2(at.z(), rNow), Vec2(at.z(), rTo)) ||
                                             cuts(Vec2(at.z(), rTo), Vec2(start.z(), rTo)));
                const bool out = have && machine::approachOutFirst(at, start);
                std::printf("  %-22s q%d r%7.2f z%8.2f -> r%7.2f z%8.2f  tip: Z first %s, X first "
                            "%s; goes %s%s\n",
                            op.name.c_str(), op.toolpath.tipQuadrant, rNow, at.z(), rTo, start.z(),
                            zFirst ? "CUTS" : "clear", xFirst ? "CUTS" : "clear",
                            out ? "out first" : "Z first", (out ? xFirst : zFirst) ? " CUTS" : "");
                for (const sim::MachineCollision& c : op.machineCollisions) {
                    if (!c.during.empty()) {
                        std::printf("    %s / %s in %s\n", c.first.c_str(), c.second.c_str(),
                                    c.during.c_str());
                    }
                }
                for (const toolpath::Move& m : op.toolpath.moves) {
                    if (m.kind != toolpath::MoveKind::Dwell) {
                        at = m.end;
                    }
                }
                have = true;
            }
        }
    }
}

TEST_CASE("a lathe's approach: along Z, then in on X, in the machine and the program",
          "[document][lathe][machine]") {
    CancelToken cancel;
    NullProgress progress;
    const auto open = [](const char* file) {
        return Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / file);
    };
    SECTION("the program: Z before X, for the first tool and after a change") {
        const Document doc = open("turret_project.ocam");
        const Setup& setup = doc.project().setups.front();
        const machine::Machine& m = machineFor(doc.project(), setup);
        const PostOutput posted = postSetup(doc, setup.id, m, defaultPostFor(m), cancel, progress);
        REQUIRE(posted.errors.empty());
        for (const char* word : {"T0101", "T0505"}) {
            const std::size_t at = posted.gcode.find(word);
            REQUIRE(at != std::string::npos);
            const std::size_t z = posted.gcode.find(" Z", at);
            const std::size_t x = posted.gcode.find(" X", at);
            CAPTURE(word);
            CHECK(z < x);
            // The Z block has no X on it, and the spindle starts between them.
            const std::size_t lineEnd = posted.gcode.find('\n', z);
            CHECK(x > lineEnd);
        }
        CHECK(posted.gcode.find("G96", posted.gcode.find("T0505")) >
              posted.gcode.find(" Z", posted.gcode.find("T0505")));
    }
    SECTION("a tailstock on a machine with its turret's home beyond it") {
        const Document doc = open("turret_tailstock_project.ocam");
        const Setup& setup = doc.project().setups.front();
        CHECK(setup.machine == "generic-lathe-turret");
        const SetupSimulation sim = simulateSetup(doc, setup.id, {}, cancel, progress);
        INFO(report(sim));
        REQUIRE_FALSE(sim.hasErrors());
        CHECK(sim.machineCollisionCount() == 0);
        // The turret's home is past the tailstock's front, whichever the setup's centre: X first
        // would have the tool go in, then along Z through it.
        const machine::Machine& m = *sim.machine;
        REQUIRE(sim.operations.front().latheMotion);
        const double front = m.lathe->tailstock->z + sim.operations.front().latheMotion->held.w;
        CHECK(m.lathe->turret.centre.z() - 60.0 > front);
    }
    SECTION("a cross hole after a hole on the axis: out on X first, in both") {
        Document doc = open("live_project.ocam");
        const Uuid id = doc.project().setups.front().id;
        doc.modify("On the turret",
                   [&](Project& p) { p.findSetup(id)->machine = "generic-lathe-turret"; });
        const Setup& setup = *doc.project().findSetup(id);
        const SetupSimulation sim = simulateSetup(doc, id, {}, cancel, progress);
        INFO(report(sim));
        const OperationSimulation& cross = named(sim, "Cross hole");
        REQUIRE(cross.latheMotion);
        const auto& steps = cross.latheMotion->steps;
        REQUIRE(steps.size() >= 2);
        const machine::LatheStep& first = steps[steps.size() - 2];
        CHECK(first.kind == machine::LatheStep::Kind::Rapid);
        CHECK(first.to.linear.z() == cross.latheMotion->start.linear.z());
        // No tool in the part on the way, as Z first had.
        for (const sim::MachineCollision& c : cross.machineCollisions) {
            if (c.during == machine::toString(machine::LatheStep::Kind::Rapid)) {
                CHECK(c.second != "part");
                CHECK(c.second != "stock");
            }
        }
        // The program has the Z after the X there.
        const machine::Machine& m = machineFor(doc.project(), setup);
        const PostOutput posted =
            postSetup(doc, id, m, post::builtinPost("haas-lathe"), cancel, progress);
        REQUIRE(posted.errors.empty());
        const std::size_t tool = posted.gcode.find("T0303");
        REQUIRE(tool != std::string::npos);
        const std::size_t x = posted.gcode.find(" X", tool);
        const std::size_t z = posted.gcode.find(" Z", tool);
        CHECK(x < z);
    }
    SECTION(
        "from home, nothing but Z goes first; a turret at home goes in X, then Z, to get there") {
        ShaftOnTurret f;
        f.withMachine([](machine::Machine& m) { m.lathe->turret.indexAtHome = true; });
        const SetupSimulation sim = f.simulate();
        const OperationSimulation& face = named(sim, "Face");
        REQUIRE(face.latheMotion);
        REQUIRE(face.latheMotion->steps.size() == 2);
        CHECK(face.latheMotion->steps[0].to.linear.x() == 0.0);
        CHECK(face.latheMotion->steps[0].to.linear.z() < 0.0);
        const OperationSimulation& part = named(sim, "Part off");
        REQUIRE(part.latheMotion->steps.size() == 5);
        CHECK(part.latheMotion->steps[3].to.linear.x() == 0.0); // Z, still at home in X
    }
}

TEST_CASE("tools mount through the block on their station", "[document][lathe][machine][fitting]") {
    const FittingPackage& across = builtinFittings()[0];
    const FittingPackage& along = builtinFittings()[1];
    ShaftOnTurret f;
    const machine::Machine& m = machine::genericLatheTurret();
    const machine::LatheTurret& t = m.lathe->turret;
    const auto setup = [&]() -> const Setup& {
        return *f.doc.project().findSetup(f.setupId);
    };
    const auto station = [&](int number) {
        const TurretLoad load = turretLoad(f.doc, setup(), m);
        for (const TurretStation& s : load.stations) {
            if (s.station == number) {
                return s;
            }
        }
        FAIL("no such station");
        return TurretStation{};
    };
    SECTION("no blocks: as before") {
        const TurretStation s = station(1);
        CHECK_FALSE(s.block);
        CHECK(s.tip.x() == Approx(t.radialReach));
    }
    SECTION("a block across: its seat on the station, the tip out of its mouth") {
        f.doc.useFitting(across);
        f.doc.modify("Block", [&](Project& p) {
            p.findSetup(f.setupId)->turretBlocks = {{1, across.fitting.id}};
        });
        const TurretStation s = station(1);
        REQUIRE(s.block);
        CHECK(s.across);
        CHECK(s.overhang == Approx(30.0));
        // From the index line: to the seat, the block's 60, the overhang.
        CHECK(s.tip.x() == Approx(t.blockRadius + 60.0 + 30.0));
        CHECK(s.tipPoint.y() == Approx(0.0).margin(1e-9));
        // The block's +X, where its slot is, goes towards the work: -Z of the machine.
        const Transform3 on = blockOnStation(across.fitting, *m.lathe, false);
        CHECK(
            (on * across.fitting.seat.point - (t.centre + Vec3(-t.blockRadius, 0.0, t.blockAlong)))
                .norm() < 1e-9);
        CHECK((on.linear() * Vec3::UnitX() + Vec3::UnitZ()).norm() < 1e-9);
        CHECK((on.linear() * Vec3::UnitZ() + Vec3::UnitX()).norm() < 1e-9);
        // The tool sits with its top at the height of the tip, in the slot.
        CHECK((on * Vec3(0, 12.5, 0)).y() == Approx(-12.5));
        // So the turret holds it face down: its shank above the tip's height, in the slot, as the
        // block editor's example tool has it, not below in the block (the maintainer's OD holder:
        // drawn on the side where its wedge goes).
        CHECK((on.linear() * blockUp(across.fitting)).y() == Approx(-1.0));
        CHECK(s.faceDown);
        const std::vector<sim::TurretTool> tools =
            turretTools(f.doc, turretLoad(f.doc, setup(), m), machine::WorkMount{});
        const auto held = std::find_if(tools.begin(), tools.end(),
                                       [](const sim::TurretTool& tool) { return tool.station == 1; });
        REQUIRE(held != tools.end());
        double low = 1e9;
        double high = -1e9;
        int inserts = 0;
        for (const sim::TurretTool::Solid& solid : held->solids) {
            // The insert (this tool has no holder): its body on the shank's side of its rake face.
            if (solid.part != "insert") {
                continue;
            }
            ++inserts;
            for (const auto* face : {&solid.prism.lower, &solid.prism.upper}) {
                for (const Vec3& p : *face) {
                    low = std::min(low, (held->placement * p).y());
                    high = std::max(high, (held->placement * p).y());
                }
            }
        }
        REQUIRE(inserts > 0);
        CHECK(low == Approx(s.tipPoint.y()).margin(1e-6));
        CHECK(high > s.tipPoint.y() + 1.0);
        SECTION("a setup's own overhang, and a tip it typed") {
            f.doc.modify("Further out", [&](Project& p) {
                TurretEntry e;
                e.tool = p.tools.tools[0].id;
                e.overhang = 50.0;
                p.findSetup(f.setupId)->turret.push_back(e);
            });
            CHECK(station(1).tip.x() == Approx(t.blockRadius + 60.0 + 50.0));
            f.setTip(f.doc.project().tools.tools[0].id, Vec2(222.0, -88.0));
            const TurretStation typed = station(1);
            CHECK(typed.tipSet);
            CHECK(typed.tip == Vec2(222.0, -88.0));
            CHECK(typed.tipPoint.isApprox(machine::tipPoint(*m.lathe, Vec2(222.0, -88.0))));
        }
    }
    SECTION("a tool's body stands back from its insert, so their faces never coincide") {
        // The maintainer: "Z fighting on stick tools and inserts". The insert as it is, the body
        // in from its outline all round: here, face down in a block, above the rake face.
        f.doc.useFitting(across);
        const tooling::Holder holder =
            tooling::makeTurningHolder(*tooling::parseDesignation("PCLNR 2525M 12"));
        f.doc.modify("Holder and block", [&](Project& p) {
            p.tools.holders.push_back(holder);
            p.tools.tools[0].holderId = holder.id;
            p.findSetup(f.setupId)->turretBlocks = {{1, across.fitting.id}};
        });
        const TurretStation s = station(1);
        const std::vector<sim::TurretTool> tools =
            turretTools(f.doc, turretLoad(f.doc, setup(), m), machine::WorkMount{});
        const auto held = std::find_if(tools.begin(), tools.end(),
                                       [](const sim::TurretTool& tool) { return tool.station == 1; });
        REQUIRE(held != tools.end());
        double insertLow = 1e9;
        double bodyLow = 1e9;
        int bodies = 0;
        for (const sim::TurretTool::Solid& solid : held->solids) {
            const bool insert = solid.part == "insert";
            bodies += insert ? 0 : 1;
            for (const auto* face : {&solid.prism.lower, &solid.prism.upper}) {
                for (const Vec3& p : *face) {
                    const double y = (held->placement * p).y() - s.tipPoint.y();
                    (insert ? insertLow : bodyLow) = std::min(insert ? insertLow : bodyLow, y);
                }
            }
        }
        REQUIRE(bodies > 0);
        CHECK(insertLow == Approx(0.0).margin(1e-6)); // the rake face at the tip's height
        CHECK(bodyLow == Approx(0.05).margin(1e-6));  // the body's face behind it
    }
    SECTION("a block along: the tool out along Z, towards the work, on its pitch") {
        f.doc.useFitting(along);
        tooling::Tool drill = longDrill();
        f.mount(drill, 3);
        f.doc.modify("Block", [&](Project& p) {
            p.findSetup(f.setupId)->turretBlocks = {{3, along.fitting.id}};
        });
        const TurretStation s = station(3);
        CHECK_FALSE(s.across);
        // 35 up from the seat towards the spindle.
        CHECK(s.tip.x() == Approx(t.blockRadius + 35.0));
        // 40 out of the block's middle and the overhang, towards the chuck.
        CHECK(s.tip.y() == Approx(t.blockAlong - 40.0 - 60.0));
        // Its bore is 32 mm, the drill's shank 12: the largest it takes, so the drill goes in
        // through a collet or sleeve. Not a problem, and not said (the maintainer: milling units
        // are "basically just collet holders").
        const TurretLoad load = turretLoad(f.doc, setup(), m);
        CHECK(std::none_of(load.problems.begin(), load.problems.end(),
                           [](const std::string& p) { return p.find("32.0") != p.npos; }));
        // A shank larger than the bore does not go in.
        FittingPackage small = along;
        small.fitting.roundBar = 10.0;
        f.doc.useFitting(small);
        const TurretLoad tight = turretLoad(f.doc, setup(), m);
        REQUIRE_FALSE(tight.problems.empty());
        CHECK_THAT(tight.problems.front(),
                   ContainsSubstring("12.0 mm round shank, larger than the 10.0 mm bore"));
    }
    SECTION("a round shank in a block for square ones is said, a size not stated is not") {
        FittingPackage square = across;
        square.fitting.id = Uuid::generate();
        square.fitting.squareShank.reset();
        f.doc.useFitting(square);
        f.mount(longDrill(), 3);
        f.doc.modify("Block", [&](Project& p) {
            p.findSetup(f.setupId)->turretBlocks = {{3, square.fitting.id}};
        });
        const auto said = [&](std::string_view text) {
            const TurretLoad load = turretLoad(f.doc, setup(), m);
            return std::any_of(load.problems.begin(), load.problems.end(),
                               [&](const std::string& p) { return p.find(text) != p.npos; });
        };
        CHECK(said("takes only square shanks"));
        // Both, with no round size stated: nothing to say about the drill's.
        square.fitting.takes = LatheFitting::Takes::Both;
        f.doc.useFitting(square);
        CHECK_FALSE(said("takes only"));
        CHECK_FALSE(said("takes 32"));
    }
    SECTION("a block or centre the machine does not take is said, and used") {
        f.doc.useFitting(across);
        f.doc.useFitting(builtinFittings()[2]);
        f.doc.modify("Block", [&](Project& p) {
            Setup& s = *p.findSetup(f.setupId);
            s.turretBlocks = {{1, across.fitting.id}};
            s.tailstock = LatheTailstock{};
            s.tailstock->tipZ = 120.0;
            s.tailstock->centre = builtinFittings()[2].fitting.id;
        });
        const auto problems = [&](const machine::Machine& machine) {
            return turretLoad(f.doc, setup(), machine).problems;
        };
        const auto mentions = [](const std::vector<std::string>& all, std::string_view text) {
            return std::any_of(all.begin(), all.end(),
                               [&](const std::string& p) { return p.find(text) != p.npos; });
        };
        // The generic lathe takes the generic examples.
        CHECK_FALSE(mentions(problems(m), "mounts by"));
        CHECK_FALSE(mentions(problems(m), "quill takes"));
        machine::Machine vdi = m;
        vdi.lathe->turret.blockMounts = {"VDI 40", "VDI 30"};
        vdi.lathe->tailstock->quillInterface = "MT 4";
        const auto said = problems(vdi);
        for (const std::string& one : said) {
            UNSCOPED_INFO(one);
        }
        CHECK(mentions(said, "station 1: block 'Block for a 25 mm shank across (generic)' mounts "
                             "by 'Generic bolt-on (placeholder)', and machine"));
        CHECK(mentions(said, "takes 'VDI 40' or 'VDI 30'"));
        CHECK(mentions(said, "quill takes 'MT 4'"));
        // Reported, not refused: the tool still sits in the block.
        CHECK(station(1).block == across.fitting.id);
        // Names compare without case or spacing.
        vdi.lathe->turret.blockMounts = {"generic BOLT-ON  (placeholder)"};
        CHECK_FALSE(mentions(problems(vdi), "mounts by"));
        // A machine that does not say takes anything.
        vdi.lathe->turret.blockMounts.clear();
        vdi.lathe->tailstock->quillInterface.clear();
        CHECK(problems(vdi) == problems(m));
        // A block that does not say, on a machine that does.
        FittingPackage silent = across;
        silent.fitting.mountType.clear();
        f.doc.useFitting(silent);
        vdi.lathe->turret.blockMounts = {"VDI 40"};
        CHECK(mentions(problems(vdi), "does not say how it mounts"));
    }
    SECTION("a shank in a block for tools along is said") {
        f.doc.useFitting(along);
        f.doc.modify("Block", [&](Project& p) {
            p.findSetup(f.setupId)->turretBlocks = {{1, along.fitting.id}};
        });
        const TurretLoad load = turretLoad(f.doc, setup(), m);
        REQUIRE_FALSE(load.problems.empty());
        CHECK_THAT(load.problems.front(), ContainsSubstring("holds tools along it"));
    }
    SECTION("the block rides along, and its collisions are reported") {
        f.doc.useFitting(across);
        // On station 2, next to the tool cutting, with no tool in it: near the chuck by its reach.
        machine::Machine low = m;
        f.withMachine([](machine::Machine& changed) { changed.lathe->turret.blockAlong = -140.0; });
        f.doc.modify("Block", [&](Project& p) {
            p.findSetup(f.setupId)->turretBlocks = {{2, across.fitting.id}};
        });
        const SetupSimulation sim = f.simulate();
        const std::string text = report(sim);
        INFO(text);
        REQUIRE(sim.turretBlocks.size() == 1);
        CHECK(sim.turretBlocks[0].station == 2);
        CHECK_THAT(text, ContainsSubstring("Station 2 block"));
        CHECK_THAT(text, ContainsSubstring("/ Chuck"));
    }
    SECTION("a station beyond the turret") {
        f.doc.useFitting(across);
        f.doc.modify("Block", [&](Project& p) {
            p.findSetup(f.setupId)->turretBlocks = {{13, across.fitting.id}};
        });
        const TurretLoad load = turretLoad(f.doc, setup(), m);
        REQUIRE_FALSE(load.problems.empty());
        CHECK_THAT(load.problems.back(), ContainsSubstring("stations 1 to 12"));
    }
}

TEST_CASE("the built-in project with blocks and a live centre",
          "[document][lathe][machine][fitting]") {
    CancelToken cancel;
    NullProgress progress;
    const Document doc =
        Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "turret_blocks_project.ocam");
    const Setup& setup = doc.project().setups.front();
    SECTION("it simulates clean, and the blocks and centre came back from the file") {
        CHECK(doc.project().fittings.size() == 3);
        REQUIRE(setup.tailstock);
        REQUIRE(setup.tailstock->centre);
        const auto supports = latheSupports(doc, setup);
        REQUIRE(supports.size() == 2);
        CHECK(supports[1].name == "Tailstock centre");
        CHECK(supports[1].id == tailstockCentreSupportId());
        const SetupSimulation sim = simulateSetup(doc, setup.id, {}, cancel, progress);
        std::string warned;
        for (const std::string& w : sim.warnings) {
            warned += w + "\n";
        }
        INFO(report(sim));
        INFO(warned);
        REQUIRE_FALSE(sim.hasErrors());
        CHECK(sim.turretBlocks.size() == 4);
        CHECK(sim.machineCollisionCount() == 0);
        // Tips where the blocks put them: T1 in a block across.
        const OperationSimulation& rough = named(sim, "Rough");
        REQUIRE(rough.latheMotion);
        CHECK(machine::genericLatheTurret().lathe->turret.centre.x() - rough.latheMotion->tip.x() ==
              Approx(150.0 + 60.0 + 30.0));
    }
    SECTION("the centre's tip is at the setup's point, on the axis, and the quill behind it") {
        const auto bodies = latheBodies(doc, setup, 0.0);
        const auto centre = std::find_if(bodies.begin(), bodies.end(), [](const sim::LatheBody& b) {
            return b.name == "Tailstock centre";
        });
        REQUIRE(centre != bodies.end());
        geom2d::Aabb2 box;
        for (const geom2d::Region& r : centre->area) {
            box.extend(r.bounds());
        }
        CHECK(box.min.x() == Approx(setup.tailstock->tipZ).margin(0.6));
        CHECK(box.max.x() == Approx(setup.tailstock->tipZ + 99.0).margin(0.6));
        CHECK(box.max.y() == Approx(22.5).margin(0.6));
    }
    SECTION("the tailstock's body stands behind the quill behind the centre") {
        const SetupSimulation sim = simulateSetup(doc, setup.id, {}, cancel, progress);
        const OperationSimulation& first = sim.operations.front();
        REQUIRE(first.latheMotion);
        const double back =
            (first.mount->wcs *
             Vec3(0, 0, setup.tailstock->tipZ + 99.0 + setup.tailstock->quillLength))
                .z();
        CHECK(first.latheMotion->held.w == Approx(back - 760.0));
    }
    SECTION("the cached simulation has the blocks again") {
        const SetupSimulation sim = simulateSetup(doc, setup.id, {}, cancel, progress);
        const std::string key = simulationInputKey(doc, setup, {});
        const auto back = decodeSimulation(encodeSimulation(sim, key), key, doc, setup);
        REQUIRE(back);
        REQUIRE(back->turretBlocks.size() == sim.turretBlocks.size());
        CHECK(back->turretBlocks[0].mesh->triangles.size() ==
              sim.turretBlocks[0].mesh->triangles.size());
    }
}

TEST_CASE("a fitting's file, library and validation", "[document][lathe][fitting]") {
    const auto folder = std::filesystem::temp_directory_path() / "oxcam-fitting-test";
    std::filesystem::remove_all(folder);
    FittingPackage p = builtinFittings()[1];
    SECTION("written and read back") {
        p.fitting.name = "Mine";
        p.fitting.id = Uuid::generate();
        saveFitting(folder, p);
        const auto loaded = loadFittings(folder);
        REQUIRE(loaded.size() == 1);
        CHECK(loaded[0].fitting == p.fitting);
        CHECK(*loaded[0].file.bytes == *p.file.bytes);
        CHECK(removeFittingFile(folder, p.fitting.id));
        CHECK(loadFittings(folder).empty());
        CHECK_FALSE(std::filesystem::exists(folder / (p.file.sha256 + ".step")));
    }
    SECTION("a changed model is said") {
        p.fitting.id = Uuid::generate();
        saveFitting(folder, p);
        writeFileAtomic(folder / (p.file.sha256 + ".step"), "not the same");
        std::vector<std::string> problems;
        CHECK(loadFittings(folder, &problems).empty());
        REQUIRE(problems.size() == 1);
        CHECK_THAT(problems[0], ContainsSubstring("changed"));
    }
    SECTION("version 1: a size of 0 was 'any', now a size not stated") {
        const auto v1 = [&](double shank, double bore) {
            Json doc = toJson(p.fitting);
            doc["version"] = 1;
            for (const char* key : {"takes", "square_shank", "round_bar", "mount_type"}) {
                doc.erase(key);
            }
            doc["shank"] = shank;
            doc["bore"] = bore;
            return fittingFromJson(doc, "v1");
        };
        const LatheFitting square = v1(25.0, 0.0);
        CHECK(square.takes == LatheFitting::Takes::Square);
        CHECK(square.squareShank == 25.0);
        CHECK_FALSE(square.roundBar);
        CHECK(square.mountType.empty());
        const LatheFitting round = v1(0.0, 32.0);
        CHECK(round.takes == LatheFitting::Takes::Round);
        CHECK(round.roundBar == 32.0);
        const LatheFitting any = v1(0.0, 0.0);
        CHECK(any.takes == LatheFitting::Takes::Both);
        CHECK_FALSE(any.squareShank);
        CHECK_FALSE(any.roundBar);
        CHECK(v1(20.0, 16.0).takes == LatheFitting::Takes::Both);
        // A centre had sizes it did not use.
        Json centre = toJson(builtinFittings()[2].fitting);
        centre["version"] = 1;
        centre.erase("mount_type");
        centre["shank"] = 0.0;
        centre["bore"] = 0.0;
        CHECK_NOTHROW(fittingFromJson(centre, "v1 centre"));
        // Written back: what it takes, and only the sizes stated.
        const Json now = toJson(any);
        CHECK(now.at("version") == kFittingVersion);
        CHECK(now.at("takes") == "both");
        CHECK_FALSE(now.contains("square_shank"));
        CHECK_FALSE(now.contains("shank"));
    }
    SECTION("problems") {
        LatheFitting f = p.fitting;
        CHECK(f.validate().empty());
        f.roundBar = 0.0; // a size stated must be one
        REQUIRE_FALSE(f.validate().empty());
        CHECK_THAT(f.validate().front(), ContainsSubstring("more than 0"));
        f = p.fitting;
        f.mount.direction = Vec3(1.0, 0.0, 1.0); // neither with the seat nor square to it
        REQUIRE_FALSE(f.validate().empty());
        CHECK_THAT(f.validate().front(), ContainsSubstring("square"));
        f = p.fitting;
        f.mount.direction = -f.seat.direction;
        REQUIRE_FALSE(f.validate().empty());
        CHECK_THAT(f.validate().front(), ContainsSubstring("into the turret"));
        f = p.fitting;
        f.overhang = 0.0;
        CHECK_FALSE(f.validate().empty());
        f = p.fitting;
        f.name.clear();
        CHECK_FALSE(f.validate().empty());
    }
    SECTION("built-ins are the shapes they say, and their sizes in range") {
        for (const FittingPackage& b : builtinFittings()) {
            CAPTURE(b.fitting.name);
            CHECK(b.fitting.validate().empty());
            const auto mesh = stepMesh(b.file, 0.2, b.fitting.name);
            CHECK(mesh->signedVolume() > 1000.0);
            CHECK(b.fitting.description.find("placeholder") != std::string::npos);
            CHECK(b.fitting.mountType.find("placeholder") != std::string::npos);
        }
    }
    SECTION("the example tool in a block") {
        const LatheFitting& across = builtinFittings()[0].fitting;
        const ExampleTool square = exampleTool(across);
        CHECK_FALSE(square.round);
        CHECK(square.sizeStated);
        CHECK(square.size == 25.0);
        // Out of the mouth by the overhang, at the tip's height, on the side towards the work.
        CHECK(square.tip.isApprox(across.mount.point + Vec3::UnitZ() * across.overhang +
                                  towardsWork(across) * 12.5));
        // The shank under the tip's height, inside the block's slot.
        for (const Vec3& v : square.shank.front().lower) {
            CHECK(v.y() <= 1e-9);
        }
        LatheFitting unsized = across;
        unsized.squareShank.reset();
        CHECK_FALSE(exampleTool(unsized).sizeStated);
        const LatheFitting& along = builtinFittings()[1].fitting;
        const ExampleTool bar = exampleTool(along);
        CHECK(bar.round);
        CHECK(bar.size == 32.0);
        // Along the block's tool mount, beyond it by the overhang.
        CHECK((bar.tip - along.mount.point).dot(along.mount.direction) == Approx(along.overhang));
    }
    SECTION("turn about the seat, and an orientation that needs it") {
        LatheFitting f = builtinFittings()[0].fitting;
        const Vec3 before = towardsWork(f);
        CHECK(before.isApprox(Vec3::UnitX()));
        f.seatTurn = 90.0;
        CHECK(towardsWork(f).isApprox(Vec3::UnitY()));
    }
    std::filesystem::remove_all(folder);
}

TEST_CASE("a fitting in the project and removal", "[document][lathe][fitting]") {
    ShaftOnTurret f;
    const FittingPackage& block = builtinFittings()[0];
    f.doc.useFitting(block);
    f.doc.modify("Block", [&](Project& p) {
        p.findSetup(f.setupId)->turretBlocks = {{4, block.fitting.id}};
    });
    SECTION("the project keeps it, with its file") {
        const auto file = std::filesystem::temp_directory_path() / "oxcam-fitting-project.ocam";
        f.doc.save(file);
        const Document back = Document::open(file);
        REQUIRE(back.project().findFitting(block.fitting.id));
        CHECK(*back.project().findFitting(block.fitting.id) == block.fitting);
        REQUIRE(back.modelFile(block.fitting.sha256));
        CHECK(back.project().setups.front().turretBlocks.front().station == 4);
        std::filesystem::remove(file);
    }
    SECTION("changing a block or its place changes the simulation's key") {
        const Setup& setup = *f.doc.project().findSetup(f.setupId);
        const std::string before = simulationInputKey(f.doc, setup, {});
        f.doc.modify("Move", [&](Project& p) { p.fittings.front().overhang = 44.0; });
        CHECK(simulationInputKey(f.doc, *f.doc.project().findSetup(f.setupId), {}) != before);
    }
    SECTION("no setup using it, it goes") {
        f.doc.modify("Take off", [&](Project& p) {
            p.findSetup(f.setupId)->turretBlocks.clear();
            removeUnusedFittings(p);
        });
        CHECK(f.doc.project().fittings.empty());
    }
    SECTION("a block that is not there is refused by the project's check") {
        Project p = f.doc.project();
        p.fittings.clear();
        std::string all;
        for (const std::string& issue : p.validate()) {
            all += issue;
        }
        CHECK_THAT(all, ContainsSubstring("station 4"));
    }
}

TEST_CASE("the kinds of block a station takes", "[document][lathe][fitting]") {
    machine::Machine mine = machine::genericLatheTurret();
    mine.lathe->turret.blockMounts = {"Bolt-on"};
    mine.lathe->turret.stationMounts = {{{2, 4}, {"Milling unit"}}};
    LatheFitting turning = builtinFittings()[0].fitting;
    turning.mountType = "bolt-ON";
    LatheFitting mill = builtinFittings()[1].fitting;
    mill.mountType = "Milling unit";
    CHECK_FALSE(mountMismatch(turning, mine, 1));
    REQUIRE(mountMismatch(turning, mine, 2));
    CHECK_THAT(*mountMismatch(turning, mine, 2), ContainsSubstring("station 2 of machine"));
    CHECK_THAT(*mountMismatch(turning, mine, 2), ContainsSubstring("'Milling unit'"));
    CHECK_FALSE(mountMismatch(mill, mine, 4));
    CHECK(mountMismatch(mill, mine, 3));
    // On no station in particular: it fits when one takes it.
    CHECK_FALSE(mountMismatch(turning, mine));
    CHECK_FALSE(mountMismatch(mill, mine));
    LatheFitting other = turning;
    other.mountType = "VDI 40";
    REQUIRE(mountMismatch(other, mine));
    CHECK_THAT(*mountMismatch(other, mine), ContainsSubstring("'Bolt-on' or 'Milling unit'"));
    // A station of a turret that names none for the rest takes anything there.
    mine.lathe->turret.blockMounts.clear();
    CHECK_FALSE(mountMismatch(other, mine, 1));
    CHECK(mountMismatch(other, mine, 2));
    CHECK_FALSE(mountMismatch(other, mine));
}

TEST_CASE("a turret whose stations stand sideways", "[document][lathe][machine][fitting]") {
    // Station faces shifted sideways, the turret's centre off the spindle's plane: the seat and a
    // tip without a block on the spindle's plane all the same.
    ShaftOnTurret f;
    machine::Machine m = machine::genericLatheTurret();
    m.lathe->turret.centre.y() = 75.0;
    m.lathe->turret.blockAcross = -75.0;
    const machine::LatheTurret& t = m.lathe->turret;
    CHECK(machine::tipPoint(*m.lathe, Vec2(200.0, -10.0)).y() == Approx(0.0));
    const FittingPackage& across = builtinFittings()[0];
    const Transform3 on = blockOnStation(across.fitting, *m.lathe, false);
    CHECK((on * across.fitting.seat.point -
           Vec3(t.centre.x() - t.blockRadius, 0.0, t.centre.z() + t.blockAlong))
              .norm() < 1e-9);
    f.doc.useFitting(across);
    f.doc.modify("Block", [&](Project& p) {
        p.findSetup(f.setupId)->turretBlocks = {{1, across.fitting.id}};
    });
    const TurretLoad load = turretLoad(f.doc, *f.doc.project().findSetup(f.setupId), m);
    REQUIRE_FALSE(load.stations.empty());
    for (const TurretStation& s : load.stations) {
        CHECK(s.tipPoint.y() == Approx(0.0).margin(1e-9));
    }
    for (const std::string& problem : load.problems) {
        CHECK_THAT(problem, !ContainsSubstring("off the spindle's centre height"));
    }
    // Written only when it is not 0, and read back.
    CHECK(machine::machineFromJson(machine::toJson(m)) == m);
    CHECK_FALSE(machine::toJson(machine::genericLatheTurret())
                    .at("lathe")
                    .at("turret")
                    .contains("block_across"));
}

TEST_CASE("a machine check's cost", "[.][lathemachinebench]") {
    using Clock = std::chrono::steady_clock;
    auto msSince = [](Clock::time_point t) {
        return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
    };
    for (const int tools : {0, 1, 4, 11}) {
        ShaftOnTurret f;
        for (int k = 0; k < tools; ++k) {
            tooling::Tool t = longDrill();
            t.id = Uuid::generate();
            t.name = "Drill " + std::to_string(k);
            const int station = k + 2 >= 5 ? k + 3 : k + 2;
            f.mount(t, station);
        }
        double best = 1e9;
        std::size_t hits = 0;
        for (int run = 0; run < 5; ++run) {
            const auto t0 = Clock::now();
            const SetupSimulation sim = f.simulate();
            best = std::min(best, msSince(t0));
            hits = sim.machineCollisionCount();
        }
        std::printf("%2d tools riding along: simulate %.1f ms (%zu reported)\n", tools, best, hits);
    }
    {
        // A block on every station.
        ShaftOnTurret f;
        const FittingPackage& block = builtinFittings()[0];
        f.doc.useFitting(block);
        f.doc.modify("Blocks", [&](Project& p) {
            for (int k = 1; k <= 12; ++k) {
                p.findSetup(f.setupId)->turretBlocks.push_back({k, block.fitting.id});
            }
        });
        double best = 1e9;
        std::size_t hits = 0;
        for (int run = 0; run < 5; ++run) {
            const auto t0 = Clock::now();
            const SetupSimulation sim = f.simulate();
            best = std::min(best, msSince(t0));
            hits = sim.machineCollisionCount();
        }
        std::printf("12 blocks: simulate %.1f ms (%zu reported)\n", best, hits);
    }
    {
        ShaftOnTurret f("generic-2axis-lathe");
        double best = 1e9;
        for (int run = 0; run < 5; ++run) {
            const auto t0 = Clock::now();
            (void)f.simulate();
            best = std::min(best, msSince(t0));
        }
        std::printf("no layout: simulate %.1f ms\n", best);
    }
}
