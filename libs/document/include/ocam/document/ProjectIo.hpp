// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// project.json <-> Project, and the .ocam archive container:
//   project.json             the Project (format "oxcam.project")
//   models/<sha256>.step     embedded model files, named by content hash (parts, fixtures and
//                            the bodies of the workholding templates)
//   machines/<sha256>.step   STEP files of the project machines' model parts (DL-0037)
//   machines/<sha256>.stl    meshes of the machines' parts that have no STEP file
//   cache/toolpaths.json     generated toolpaths (ToolpathCache); optional, never required
// Other entries are ignored when reading.

#pragma once

#include "ocam/document/Project.hpp"

#include <map>
#include <string>
#include <string_view>

namespace ocam::document {

inline constexpr int kProjectVersion = 1;
const FormatSpec& projectFormat();
const SchemaValidator& projectSchema();

Json toJson(const Project& project);
/// Migrates, validates (project and embedded tool library schemas) and converts.
/// Throws ocam::Error (Parse / Unsupported).
Project projectFromJson(Json doc, std::string_view sourceName = "project.json");

struct ArchiveContents {
    std::string projectJson;
    std::map<std::string, std::string> models; ///< sha256 -> STEP file bytes
    /// Machine part files: "<sha256>.step" or "<sha256>.stl" -> bytes (machine::partFileName).
    std::map<std::string, std::string> machineFiles;
    std::string toolpathCache; ///< cache/toolpaths.json; empty = none
};

/// Builds the zip archive bytes.
std::string writeArchive(const ArchiveContents& contents);

/// Reads archive bytes. Verifies that each model's and machine part file's content matches its
/// hash.
/// Throws Error(Parse) for corrupt or incomplete archives.
ArchiveContents readArchive(std::string_view bytes, std::string_view sourceName);

} // namespace ocam::document
