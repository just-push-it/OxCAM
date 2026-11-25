// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Projects on lathes with a layout (DL-0067): the shaft with a turret full of tools, the same with
// a tailstock, and a part taken over by the sub-spindle.

#include "FixtureSpecs.hpp"

#include <ocam/cad/Revolve.hpp>
#include <ocam/cad/occt/ShapeAccess.hpp>
#include <ocam/document/Document.hpp>
#include <ocam/document/LatheFitting.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Progress.hpp>
#include <ocam/machine/Machine.hpp>
#include <ocam/tooling/ToolLibrary.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <cstdio>
#include <filesystem>
#include <fstream>

using namespace ocam;

namespace {

tooling::Tool preset(tooling::Tool tool) {
    if (tool.presets.empty()) {
        tool.presets.push_back(tooling::defaultPreset(tool));
    }
    return tool;
}

void addOperation(document::Setup& s, const char* name, const char* type, const tooling::Tool& tool,
                  std::optional<double> stepDown = std::nullopt) {
    document::Operation o;
    o.name = name;
    o.type = type;
    o.toolId = tool.id;
    o.presetId = tool.presets.front().id;
    o.parameters = Json::object();
    if (stepDown) {
        o.presetOverrides.axialDepth = *stepDown;
    }
    o.heights = document::defaultHeights(o);
    s.operations.push_back(o);
}

/// Tools as a shop would leave them in a 12-station turret: an ISO shank and parting tool the job
/// uses, a boring bar and a drill along the spindle and a threading tool along for the ride.
struct TurretTools {
    tooling::Tool turn, part, bar, drill, thread;
    TurretTools(document::Project& p) {
        tooling::Holder shank =
            tooling::makeTurningHolder(*tooling::parseDesignation("PCLNR 2525M 12"));
        tooling::Holder boring =
            tooling::makeTurningHolder(*tooling::parseDesignation("S20R-SCLCR 09"));
        turn = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 1);
        turn.name = "CNMG 120404";
        turn.insert.noseRadius = 0.4;
        tooling::fitToHolder(turn, shank);
        turn.holderId = shank.id;
        part = tooling::makeDefaultTool(tooling::ToolType::PartingInsert, 5);
        part.name = "Part off 3 mm";
        part.insert.width = 3.0;
        part.insert.noseRadius = 0.1;
        bar = tooling::makeDefaultTool(tooling::ToolType::TurningInsert, 7);
        bar.name = "Boring bar 20";
        bar.insert.noseRadius = 0.4;
        tooling::fitToHolder(bar, boring);
        bar.holderId = boring.id;
        drill = tooling::makeDefaultTool(tooling::ToolType::Drill, 8);
        drill.name = "Drill 16";
        drill.mill.diameter = 16.0;
        drill.mill.shankDiameter = 16.0;
        drill.mill.fluteLength = 90.0;
        drill.mill.overallLength = 140.0;
        thread = tooling::makeDefaultTool(tooling::ToolType::ThreadingInsert, 10);
        thread.name = "Thread 60";
        thread.insert.noseRadius = 0.1;
        turn = preset(turn);
        part = preset(part);
        bar = preset(bar);
        drill = preset(drill);
        thread = preset(thread);
        p.tools.tools = {turn, part, bar, drill, thread};
        p.tools.holders = {shank, boring};
    }
};

void lathe(document::Setup& s, const char* name, const char* machine, double z, double direction) {
    s.name = name;
    s.machine = machine;
    s.originFrom.reset();
    s.wcs.origin = Vec3(0, 0, z);
    s.wcs.zAxis = direction * Vec3::UnitZ();
    s.wcs.xAxis = Vec3::UnitX();
}

} // namespace

// The built-in fittings' models (data/fittings holds what this wrote once, so their hashes stay the
// same). Plain shapes of ours, sizes placeholders; the points they go with are in
// document::builtinFittings.
TopoDS_Shape boxShape(const Vec3& lo, const Vec3& hi) {
    return BRepPrimAPI_MakeBox(gp_Pnt(lo.x(), lo.y(), lo.z()), gp_Pnt(hi.x(), hi.y(), hi.z()))
        .Shape();
}

void writeFittingModels(const std::filesystem::path& out) {
    const auto write = [&](const char* name, const cad::Model& model) {
        std::ofstream(out / name, std::ios::binary) << model.exportStepData();
    };
    // Across: a block 80 x 80 x 60, a slot 25 wide on its +X side for a shank whose top is at Y 0.
    write("fitting_radial_block.step",
          cad::modelFromShape(BRepAlgoAPI_Cut(boxShape(Vec3(-40, -40, 0), Vec3(40, 40, 60)),
                                              boxShape(Vec3(15, 0, -1), Vec3(41, 25, 61)))
                                  .Shape()));
    // Along: a block with a 32 mm bore along +X, 35 up from its seat.
    write("fitting_axial_block.step",
          cad::modelFromShape(
              BRepAlgoAPI_Cut(
                  boxShape(Vec3(-40, -40, 0), Vec3(40, 40, 70)),
                  BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(-41, 0, 35), gp_Dir(1, 0, 0)), 16.0, 82.0)
                      .Shape())
                  .Shape()));
    // A live centre: a body 45 across and 60 long, a 60 degree point.
    write("fitting_live_centre.step",
          cad::revolvedAboutZ({{0.0, 0.0}, {0.0, 22.5}, {60.0, 22.5}, {99.0, 0.0}}));
}

void writeLatheMachineFixtures(const std::filesystem::path& out, CancelToken& cancel,
                               ProgressSink& progress) {
    const double front = ocam::fixtures::shaft::kMajorLength + ocam::fixtures::shaft::kMinorLength;

    // The shaft from a long bar, with the turret full.
    {
        document::Document doc;
        const Uuid model = doc.importModel(out / "shaft.step", cancel, progress);
        const Uuid setup = doc.addSetup(model);
        doc.modify("Add the operations", [&](document::Project& p) {
            p.name = "on a lathe's turret";
            const TurretTools tools(p);
            document::Setup& s = *p.findSetup(setup);
            lathe(s, "On the spindle", "generic-lathe-turret", front, 1.0);
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 40.0;
            addOperation(s, "Face", "turn_face", tools.turn);
            addOperation(s, "Rough", "turn_rough", tools.turn, 1.5);
            addOperation(s, "Finish", "turn_finish", tools.turn);
            addOperation(s, "Part off", "turn_partoff", tools.part);
            for (const tooling::Tool* t : {&tools.bar, &tools.drill, &tools.thread}) {
                s.turret.push_back({t->id, std::nullopt, std::nullopt, std::nullopt});
            }
        });
        doc.save(out / "turret_project.ocam");
        std::printf("wrote %s\n", (out / "turret_project.ocam").string().c_str());

        // The same with something in the way: a tool riding along in station 2 reaching for the
        // chuck, and the parting tool swinging through the work when it indexes.
        doc.modify("In the way", [&](document::Project& p) {
            p.name = "on a lathe's turret, with collisions";
            document::Setup& s = *p.findSetup(setup);
            s.turret[0].tip = Vec2(170.0, -100.0);
            s.turret.push_back(
                {p.tools.tools[1].id, std::nullopt, Vec2(210.0, -80.0), std::nullopt});
            p.tools.tools[2].number = 2;
        });
        doc.save(out / "turret_collisions_project.ocam");
        std::printf("wrote %s\n", (out / "turret_collisions_project.ocam").string().c_str());
    }

    // Tooling blocks on the stations and a live centre, from the built-in examples.
    {
        document::Document doc;
        const Uuid model = doc.importModel(out / "shaft.step", cancel, progress);
        const Uuid setup = doc.addSetup(model);
        for (const document::FittingPackage& f : document::builtinFittings()) {
            doc.useFitting(f);
        }
        doc.modify("Add the operations", [&](document::Project& p) {
            p.name = "with blocks on the turret";
            const TurretTools tools(p);
            document::Setup& s = *p.findSetup(setup);
            lathe(s, "On blocks", "generic-lathe-turret", front, 1.0);
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 40.0;
            s.tailstock = document::LatheTailstock{};
            s.tailstock->tipZ = 1.0;
            s.tailstock->quillDiameter = 40.0;
            s.tailstock->quillLength = 120.0;
            s.tailstock->centre = document::builtinFittings()[2].fitting.id;
            addOperation(s, "Rough", "turn_rough", tools.turn, 1.5);
            addOperation(s, "Finish", "turn_finish", tools.turn);
            const Uuid across = document::builtinFittings()[0].fitting.id;
            const Uuid along = document::builtinFittings()[1].fitting.id;
            s.turretBlocks = {{1, across}, {5, across}, {7, along}, {8, along}};
            for (const tooling::Tool* t : {&tools.part, &tools.bar, &tools.drill}) {
                s.turret.push_back({t->id, std::nullopt, std::nullopt, std::nullopt});
            }
        });
        doc.save(out / "turret_blocks_project.ocam");
        std::printf("wrote %s\n", (out / "turret_blocks_project.ocam").string().c_str());
    }

    // A turret with something in every station, for how playback takes it.
    {
        document::Document doc;
        const Uuid model = doc.importModel(out / "shaft.step", cancel, progress);
        const Uuid setup = doc.addSetup(model);
        doc.modify("Add the operations", [&](document::Project& p) {
            p.name = "on a full turret";
            const TurretTools tools(p);
            document::Setup& s = *p.findSetup(setup);
            lathe(s, "On the spindle", "generic-lathe-turret", front, 1.0);
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 40.0;
            addOperation(s, "Face", "turn_face", tools.turn);
            addOperation(s, "Rough", "turn_rough", tools.turn, 1.5);
            addOperation(s, "Finish", "turn_finish", tools.turn);
            addOperation(s, "Part off", "turn_partoff", tools.part);
            for (const tooling::Tool* t : {&tools.bar, &tools.drill, &tools.thread}) {
                s.turret.push_back({t->id, std::nullopt, std::nullopt, std::nullopt});
            }
            for (const int station : {2, 3, 4, 6, 9, 11, 12}) {
                tooling::Tool extra = tools.thread;
                extra.id = Uuid::generate();
                extra.number = station;
                extra.name = "Thread " + std::to_string(station);
                p.tools.tools.push_back(extra);
                s.turret.push_back({extra.id, std::nullopt, std::nullopt, std::nullopt});
            }
        });
        doc.save(out / "turret_full_project.ocam");
        std::printf("wrote %s\n", (out / "turret_full_project.ocam").string().c_str());
    }

    // The live tools' project on the lathe with the turret.
    {
        document::Document doc = document::Document::open(out / "live_project.ocam");
        doc.modify("On the turret", [&](document::Project& p) {
            for (document::Setup& s : p.setups) {
                s.machine = "generic-lathe-turret";
            }
        });
        doc.save(out / "turret_live_project.ocam");
        std::printf("wrote %s\n", (out / "turret_live_project.ocam").string().c_str());
    }

    // On a tailstock.
    {
        document::Document doc;
        const Uuid model = doc.importModel(out / "shaft.step", cancel, progress);
        const Uuid setup = doc.addSetup(model);
        doc.modify("Add the operations", [&](document::Project& p) {
            p.name = "on a lathe's turret and tailstock";
            const TurretTools tools(p);
            document::Setup& s = *p.findSetup(setup);
            lathe(s, "On the tailstock", "generic-lathe-turret", front, 1.0);
            s.stock.kind = document::StockKind::Box;
            s.stock.size = document::StockSize::OffsetFromPart;
            s.stock.offsetSide = 1.0;
            s.stock.offsetTop = 1.0;
            s.stock.offsetBottom = 40.0;
            s.tailstock = document::LatheTailstock{};
            s.tailstock->tipZ = 1.0;
            s.tailstock->centreDiameter = 12.0;
            s.tailstock->quillDiameter = 16.0;
            s.tailstock->quillLength = 200.0;
            addOperation(s, "Rough", "turn_rough", tools.turn, 1.5);
            addOperation(s, "Finish", "turn_finish", tools.turn);
            s.turret.push_back({tools.drill.id, std::nullopt, std::nullopt, std::nullopt});
        });
        doc.save(out / "turret_tailstock_project.ocam");
        std::printf("wrote %s\n", (out / "turret_tailstock_project.ocam").string().c_str());
    }

    // The spool turned on the main spindle, then handed to the sub-spindle and parted off.
    {
        document::Document doc;
        const Uuid model = doc.importModel(out / "spool.step", cancel, progress);
        const Uuid first = doc.addSetup(model);
        const Uuid second = doc.addSetup(model);
        doc.modify("Add the operations", [&](document::Project& p) {
            p.name = "taken over by the sub-spindle";
            const TurretTools tools(p);
            document::Setup& one = *p.findSetup(first);
            lathe(one, "Main spindle", "generic-lathe-sub", ocam::fixtures::spool::kLength, 1.0);
            one.stock.kind = document::StockKind::Box;
            one.stock.size = document::StockSize::OffsetFromPart;
            one.stock.offsetSide = 1.0;
            one.stock.offsetTop = 1.0;
            one.stock.offsetBottom = 30.0;
            addOperation(one, "Face", "turn_face", tools.turn);
            addOperation(one, "Rough", "turn_rough", tools.turn, 1.5);
            addOperation(one, "Finish", "turn_finish", tools.turn);
            // Handed to the sub-spindle and parted off under its grip (ADR-0007).
            document::Operation transfer;
            transfer.name = "Transfer";
            transfer.type = "spindle_transfer";
            transfer.parameters = Json{{"z2_feed", 500.0}, {"rpm", 800}};
            transfer.heights = document::defaultHeights(transfer);
            one.operations.push_back(transfer);
            addOperation(one, "Part off", "turn_partoff", tools.part);
            one.operations.back().parameters["z_offset"] = 1.0; // 1 mm to face on the back
            document::Setup& two = *p.findSetup(second);
            lathe(two, "Sub-spindle", "generic-lathe-sub", 0.0, -1.0);
            two.stock.kind = document::StockKind::FromSetup;
            two.stock.sourceSetup = first;
            document::SetupPlacement placement;
            placement.carrier = "C2";
            two.placement = placement;
            addOperation(two, "Face the back", "turn_face", tools.turn);
            addOperation(two, "Rough the back", "turn_rough", tools.turn, 1.5);
            addOperation(two, "Finish the back", "turn_finish", tools.turn);
        });
        doc.save(out / "turret_sub_project.ocam");
        std::printf("wrote %s\n", (out / "turret_sub_project.ocam").string().c_str());
    }
}
