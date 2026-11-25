// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Timings of a turning simulation on the shaft fixture, per stage (DL-0021). Hidden (timing); run
// with
//   test_document "[turnbench]"

#include "FixtureSpecs.hpp"
#include "TestProject.hpp"

#include <ocam/document/Document.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/sim/Compare.hpp>
#include <ocam/sim/Lathe.hpp>
#include <ocam/sim/Simulate.hpp>
#include <ocam/sim/Surface.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace ocam;
using namespace ocam::document;

namespace {

using Clock = std::chrono::steady_clock;

double msSince(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

/// The shaft fixture faced, roughed, finished and parted off.
Uuid shaftSetup(Document& doc, double depthOfCut, double extraBar = 0.0) {
    using namespace fixtures::shaft;
    CancelToken cancel;
    NullProgress progress;
    const Uuid modelId =
        doc.importModel(std::filesystem::path(OCAM_FIXTURE_DIR) / "shaft.step", cancel, progress);
    const Uuid setupId = doc.addSetup(modelId);
    tooling::Tool turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
    turn.insert.noseRadius = 0.4;
    tooling::Tool part = tooling::makeDefaultTool(tooling::ToolType::PartingInsert, 2);
    part.insert.width = 3.0;
    part.insert.noseRadius = 0.1;
    doc.modify("lathe setup", [&](Project& p) {
        Setup* s = p.findSetup(setupId);
        s->machine = "generic-2axis-lathe";
        s->originFrom.reset();
        s->wcs.origin = Vec3(0, 0, kMajorLength + kMinorLength);
        s->wcs.zAxis = Vec3::UnitZ();
        s->wcs.xAxis = Vec3::UnitX();
        s->stock.kind = StockKind::Box;
        s->stock.size = StockSize::OffsetFromPart;
        s->stock.offsetSide = 1.0;
        s->stock.offsetTop = 1.0;
        s->stock.offsetBottom = 5.0 + extraBar;
        p.tools.tools = {turn, part};
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
        add("Face", "turn_face", turn);
        add("Rough", "turn_rough", turn, testing::stepDown(depthOfCut));
        add("Finish", "turn_finish", turn);
        add("Part off", "turn_partoff", part);
    });
    return setupId;
}

} // namespace

// The moves and stock of roughing and finishing the shaft in ISO holders, as CSV for a plot:
//   OXCAM_TURN_PLOT=<file> test_document "[turnplot]"
TEST_CASE("turning toolpaths per holder", "[.][turnplot]") {
    const char* file = std::getenv("OXCAM_TURN_PLOT");
    REQUIRE(file);
    std::ofstream out(file);
    out << "holder,what,kind,z0,x0,z1,x1\n";
    CancelToken cancel;
    NullProgress progress;
    for (const char* code : {"PCLNR 2525M 12", "PDJNR 2525M 15", "PSSNR 2525M 12"}) {
        Document doc;
        const Uuid setupId = shaftSetup(doc, 1.5);
        doc.modify("in the holder", [&](Project& p) {
            Setup* s = p.findSetup(setupId);
            tooling::Holder holder = tooling::makeTurningHolder(*tooling::parseDesignation(code));
            tooling::fitToHolder(p.tools.tools[0], holder);
            p.tools.tools[0].holderId = holder.id;
            p.tools.holders = {holder};
            s->operations.erase(s->operations.begin()); // a 45 degree holder cannot face
            s->operations.pop_back();                   // parting off is another tool
            s->stock.offsetTop = 0.0;
            s->stock.offsetBottom = 0.0;
        });
        SimulationOptions options;
        const SetupSimulation sim = simulateSetup(doc, setupId, options, cancel, progress);
        auto loops = [&](const char* what, const geom2d::Regions& regions) {
            for (const auto& r : regions) {
                const auto pts = r.outer.discretize(1e-3);
                for (std::size_t i = 0; i < pts.size(); ++i) {
                    const Vec2& a = pts[i];
                    const Vec2& b = pts[(i + 1) % pts.size()];
                    out << code << ',' << what << ",line," << a.x() << ',' << a.y() << ',' << b.x()
                        << ',' << b.y() << '\n';
                }
            }
        };
        loops("part", sim.partRegions);
        for (const auto& op : sim.operations) {
            REQUIRE(op.error.empty());
            Vec3 pos = op.toolpath.start;
            for (const auto& m : op.toolpath.moves) {
                out << code << ',' << op.name << ','
                    << (m.kind == toolpath::MoveKind::Rapid ? "rapid" : "feed") << ',' << pos.z()
                    << ',' << pos.x() << ',' << m.end.z() << ',' << m.end.x() << '\n';
                pos = m.end;
            }
        }
        loops("stock", sim.operations.back().latheAfter->profile());
        loops("insert", sim.operations.back().insert.discretize(1e-3).empty()
                            ? geom2d::Regions{}
                            : geom2d::Regions{{sim.operations.back().insert, {}}});
    }
}

// A turning project's operations, per setup, as CSV for a plot:
//   OXCAM_TURN_PLOT=<file> OXCAM_TURN_PROJECT=<fixture project> test_document "[turnplotproject]"
TEST_CASE("toolpaths of a turning project", "[.][turnplotproject]") {
    const char* file = std::getenv("OXCAM_TURN_PLOT");
    const char* project = std::getenv("OXCAM_TURN_PROJECT");
    REQUIRE(file);
    REQUIRE(project);
    std::ofstream out(file);
    out << "op,what,kind,z0,x0,z1,x1\n";
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / project);
    SimulationOptions options;
    auto loops = [&](const std::string& op, const char* what, const geom2d::Regions& regions) {
        for (const auto& r : regions) {
            std::vector<geom2d::Polyline> lines{r.outer};
            lines.insert(lines.end(), r.holes.begin(), r.holes.end());
            for (const auto& line : lines) {
                const auto pts = line.discretize(1e-3);
                for (std::size_t i = 0; i < pts.size(); ++i) {
                    const Vec2& a = pts[i];
                    const Vec2& b = pts[(i + 1) % pts.size()];
                    out << op << ',' << what << ",line," << a.x() << ',' << a.y() << ',' << b.x()
                        << ',' << b.y() << '\n';
                }
            }
        }
    };
    for (const Setup& setup : doc.project().setups) {
        const SetupSimulation sim = simulateSetup(doc, setup.id, options, cancel, progress);
        for (const auto& op : sim.operations) {
            INFO(op.name << ": " << op.error);
            REQUIRE(op.error.empty());
            const std::string name = setup.name + ": " + op.name;
            loops(name, "part", sim.partRegions);
            Vec3 pos = op.toolpath.start;
            for (const auto& m : op.toolpath.moves) {
                if (m.kind == toolpath::MoveKind::Dwell) {
                    continue;
                }
                out << name << ",move," << (m.kind == toolpath::MoveKind::Rapid ? "rapid" : "feed")
                    << ',' << pos.z() << ',' << pos.x() << ',' << m.end.z() << ',' << m.end.x()
                    << '\n';
                pos = m.end;
            }
            loops(name, "stock", op.latheAfter->profile());
        }
        if (sim.comparison) {
            std::printf("%s: max excess %.4f, max gouge %.4f, removed:", setup.name.c_str(),
                        sim.comparison->maxExcess, sim.comparison->maxGouge);
        }
        for (const auto& op : sim.operations) {
            std::printf(" %s %.0f", op.name.c_str(), op.removedVolume);
        }
        std::printf("\n");
    }
}

TEST_CASE("turning simulation stages", "[.][turnbench]") {
    CancelToken cancel;
    NullProgress progress;
    for (const double extraBar : {0.0, 300.0}) {
        Document doc;
        const Uuid setupId = shaftSetup(doc, 0.5, extraBar);
        auto t0 = Clock::now();
        const SetupSimulation sim = simulateSetup(doc, setupId, {}, cancel, progress);
        std::printf("bar %.0f mm longer: simulate %.1f ms\n", extraBar, msSince(t0));
        REQUIRE(sim.latheInitial);
        sim::LatheStock lathe = *sim.latheInitial;
        for (const auto& op : sim.operations) {
            REQUIRE(op.error.empty());
            t0 = Clock::now();
            (void)sim::checkTurningFixtures(op.toolpath, op.insert, {}, 200, nullptr);
            const double check = msSince(t0);
            t0 = Clock::now();
            sim::simulateTurning(lathe, op.insert, op.toolpath);
            std::printf("  %-9s %6zu moves: cut %.1f ms, check %.1f ms\n", op.name.c_str(),
                        op.toolpath.moves.size(), msSince(t0), check);
        }
        t0 = Clock::now();
        const auto mesh =
            latheDisplayMesh(sim, 2, *sim.operations[2].latheAfter, true, sim::kStockColour);
        std::printf("  display mesh: %.1f ms, %zu triangles\n", msSince(t0), mesh.triangleCount());
        // Frames through roughing, each continuing from the last as the panel does.
        const auto& rough = sim.operations[1];
        const int frames = 60;
        sim::LatheStock frame = *sim.latheBefore(1);
        double cutMs = 0.0, compareMs = 0.0, meshMs = 0.0;
        std::size_t from = 0;
        for (int f = 1; f <= frames; ++f) {
            const std::size_t to =
                rough.toolpath.moves.size() * static_cast<std::size_t>(f) / frames;
            t0 = Clock::now();
            sim::simulateTurning(frame, rough.insert, rough.toolpath, from, to);
            cutMs += msSince(t0);
            t0 = Clock::now();
            (void)latheComparison(sim, frame);
            compareMs += msSince(t0);
            t0 = Clock::now();
            (void)latheDisplayMesh(sim, 1, frame, true, sim::kStockColour);
            meshMs += msSince(t0);
            from = to;
        }
        std::printf("  frame: cut %.2f, compare %.2f, display mesh %.2f ms\n", cutMs / frames,
                    compareMs / frames, meshMs / frames);
    }
}

TEST_CASE("turning simulation with threads", "[.][turnbench]") {
    CancelToken cancel;
    NullProgress progress;
    for (const char* file : {"shaft_project.ocam", "threads_project.ocam"}) {
        Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / file);
        const Uuid setupId = doc.project().setups.front().id;
        auto t0 = Clock::now();
        const SetupSimulation sim = simulateSetup(doc, setupId, {}, cancel, progress);
        std::printf("%s: simulate %.1f ms, %zu thread passes\n", file, msSince(t0),
                    sim.operations.back().latheAfter->threads().size());
        for (std::size_t k = 0; k < sim.operations.size(); ++k) {
            const auto& op = sim.operations[k];
            if (op.toolpath.strategy != "turn_thread" || !op.latheAfter) {
                continue;
            }
            t0 = Clock::now();
            const auto mesh = latheDisplayMesh(sim, k, *op.latheAfter, true, sim::kStockColour);
            const double meshMs = msSince(t0);
            t0 = Clock::now();
            const double volume = op.latheAfter->volume();
            std::printf("  %-12s display mesh %.1f ms, %zu triangles; volume %.1f mm3 in %.2f ms, "
                        "removed %.1f mm3\n",
                        op.name.c_str(), meshMs, mesh.triangleCount(), volume, msSince(t0),
                        op.removedVolume);
        }
    }
    // How far the mesh goes with the thread: Ø20, a turn of the form at a time.
    const tooling::Tool tool = [] {
        tooling::Tool t = tooling::makeDefaultTool(tooling::ToolType::ThreadingInsert, 1);
        t.insert.noseRadius = 0.05;
        return t;
    }();
    const geom2d::Polyline shape = tooling::turningShape(tool).outline;
    for (const auto& [pitch, length] : std::vector<std::pair<double, double>>{
             {0.25, 10.0}, {0.5, 20.0}, {1.5, 20.0}, {1.5, 100.0}, {0.5, 200.0}}) {
        sim::LatheStock stock = sim::LatheStock::cylinder(-length - 10.0, 0.0, 10.0);
        const double depth = 0.6134 * pitch;
        for (int pass = 1; pass <= 6; ++pass) {
            const double d = depth * std::sqrt(pass / 6.0);
            stock.cutThread(shape, Vec2(-1.0, 10.0 - d), Vec2(-length - 1.0, 10.0 - d), pitch);
        }
        auto t0 = Clock::now();
        const auto mesh = sim::latheStockMesh(stock, 0.005, {}, {});
        const double meshMs = msSince(t0);
        double lowest = 1e9;
        for (const auto& v : mesh.positions) {
            if (static_cast<double>(v[2]) < -2.0 && static_cast<double>(v[2]) > -length) {
                lowest = std::min(lowest,
                                  std::hypot(static_cast<double>(v[0]), static_cast<double>(v[1])));
            }
        }
        std::printf("pitch %.2f x %.0f mm: mesh %.1f ms, %zu triangles, root found at %.4f "
                    "(expected %.4f)\n",
                    pitch, length, meshMs, mesh.triangleCount(), lowest, 10.0 - depth);
    }
}

TEST_CASE("turning simulation with a chuck", "[.][turnbench]") {
    CancelToken cancel;
    NullProgress progress;
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "chuck_project.ocam");
    const Setup& setup = doc.project().setups.front();
    auto t0 = Clock::now();
    const auto bodies = latheBodies(doc, setup, 0.0);
    std::printf("chuck: %zu bodies %.0f ms\n", bodies.size(), msSince(t0));
    t0 = Clock::now();
    const SetupSimulation sim = simulateSetup(doc, setup.id, {}, cancel, progress);
    std::printf("chuck: simulateSetup (%.3f mm rays, %zu operations): %.0f ms\n", sim.resolution,
                sim.operations.size(), msSince(t0));
    for (const auto& op : sim.operations) {
        t0 = Clock::now();
        (void)sim::checkTurningFixtures(op.toolpath, op.insert, bodies, 200, nullptr);
        std::printf("  %-12s %6zu moves: fixture check %.1f ms\n", op.name.c_str(),
                    op.toolpath.moves.size(), msSince(t0));
    }
}
