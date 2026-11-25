// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "ocam/tooling/Tool.hpp"

#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Units.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace ocam::tooling {

/// A named collection of tools and holders: either a user library file (*.ocamtools) or the
/// snapshot of tools embedded in a project.
struct ToolLibrary {
    Uuid id = Uuid::generate();
    std::string name;
    std::vector<Tool> tools;
    std::vector<Holder> holders;

    const Tool* findTool(const Uuid& toolId) const;
    Tool* findTool(const Uuid& toolId);
    const Tool* findToolByNumber(int number) const;
    const Holder* findHolder(const Uuid& holderId) const;

    /// Smallest positive tool number not yet used.
    int nextFreeNumber() const;

    /// Library-level problems: invalid tools or holders, duplicate ids or tool numbers,
    /// references to missing holders. Empty = valid.
    std::vector<std::string> validate() const;

    bool operator==(const ToolLibrary&) const = default;
};

/// Which tools a tool list shows. Empty fields match every tool.
struct ToolFilter {
    /// Case-insensitive: part of the name, vendor or part number, or the tool number ("12" or
    /// "T12").
    std::string text;
    std::optional<ToolFamily> family;
    std::optional<ToolType> type;
    double minDiameter = 0.0; ///< [mm]; 0 = no lower limit.
    double maxDiameter = 0.0; ///< [mm]; 0 = no upper limit.

    /// True when no field narrows the list.
    bool empty() const noexcept;
    /// True when `tool` passes every set field. With a diameter limit, tools without a
    /// diameter (turning inserts) do not.
    bool matches(const Tool& tool) const;
};

/// File format identity: "oxcam.toollib", current version.
inline constexpr int kToolLibraryVersion = 1;
const FormatSpec& toolLibraryFormat();

/// The JSON Schema for tool library documents (schemas/toollib.v1.schema.json).
const SchemaValidator& toolLibrarySchema();

/// Serializes with lengths in `fileUnit`, angles in degrees, surface speed in m/min (mm files)
/// or ft/min (inch files).
Json toJson(const ToolLibrary& library, LengthUnit fileUnit = LengthUnit::Millimeter);

/// Migrates, validates against the schema and converts. Throws ocam::Error (Parse/Unsupported).
ToolLibrary toolLibraryFromJson(Json doc, std::string_view sourceName = "tool library");

ToolLibrary loadToolLibrary(const std::filesystem::path& path);
void saveToolLibrary(const ToolLibrary& library, const std::filesystem::path& path,
                     LengthUnit fileUnit = LengthUnit::Millimeter);

} // namespace ocam::tooling
