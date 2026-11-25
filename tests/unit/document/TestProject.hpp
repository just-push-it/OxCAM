// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <ocam/document/Project.hpp>
#include <ocam/foundation/Sha256.hpp>

namespace ocam::document::testing {

/// Speeds and feeds come only from presets: a tool with its default preset added when it has
/// none (tooling::defaultPreset).
inline tooling::Tool withPreset(tooling::Tool tool) {
    if (tool.presets.empty()) {
        tool.presets.push_back(tooling::defaultPreset(tool));
    }
    return tool;
}

/// The step down and stepover of every strategy are the preset's ap and ae; these change them
/// for one operation.
inline PresetOverrides stepDown(double ap) {
    PresetOverrides o;
    o.axialDepth = ap;
    return o;
}
inline PresetOverrides stepover(double ae) {
    PresetOverrides o;
    o.radialDepth = ae;
    return o;
}

/// A project exercising every serialized field: one model, two tools with presets, two setups
/// (the second taking its stock from the first), operations with geometry and parameters.
inline Project sampleProject() {
    Project p;
    p.name = "Bracket";
    p.displayUnit = LengthUnit::Inch;
    p.notes = "Test project";

    ModelEntry model;
    model.name = "bracket";
    model.sourcePath = "C:/parts/bracket.step";
    model.sha256 = sha256Hex("bracket");
    model.placement = Frame{{1, 2, 3}, {0, 1, 0}, {0, 0, 1}};
    p.models.push_back(model);

    tooling::Tool endMill = tooling::makeFlatEndMill(1, 10.0, 22.0, 72.0);
    tooling::CuttingPreset preset;
    preset.name = "Aluminium";
    preset.surfaceSpeed = 5000.0;
    preset.feedPerTooth = 0.05;
    endMill.presets.push_back(preset);
    p.tools.tools.push_back(endMill);
    p.tools.tools.push_back(tooling::makeDrill(2, 8.5, degToRad(140.0), 43.0, 103.0));

    Setup first;
    first.name = "Op 10";
    first.modelId = model.id;
    first.wcs = Frame{{0, 0, 30}, {1, 0, 0}, {0, 0, 1}};
    first.workOffset = 1;
    first.stock.offsetSide = 2.0;
    first.machine = "generic-3axis";

    Operation facing;
    facing.name = "Face top";
    facing.type = "facing";
    facing.toolId = endMill.id;
    facing.presetId = preset.id;
    facing.parameters = Json{{"angle", 45.0}, {"passes", Json::array({1, 2})}};
    first.operations.push_back(facing);

    Operation pocket;
    pocket.name = "Pocket";
    pocket.type = "pocket2d";
    pocket.enabled = false;
    pocket.toolId = endMill.id;
    pocket.presetId = preset.id;
    pocket.comment = "rough only";
    ShapeRef floor;
    floor.modelId = model.id;
    floor.index = 7;
    floor.signature = {{50, 30, 20}, {0, 0, 1}, 778.5};
    pocket.geometry.push_back(floor);
    first.operations.push_back(pocket);

    Setup second;
    second.name = "Op 20";
    second.modelId = model.id;
    second.wcs = Frame{{0, 60, 0}, {1, 0, 0}, {0, 0, -1}};
    second.workOffset = 2;
    second.stock.kind = StockKind::FromSetup;
    second.stock.sourceSetup = first.id;
    Operation drill;
    drill.name = "Drill";
    drill.type = "drill";
    drill.toolId = p.tools.tools[1].id;
    second.operations.push_back(drill);

    p.setups = {first, second};
    return p;
}

} // namespace ocam::document::testing
