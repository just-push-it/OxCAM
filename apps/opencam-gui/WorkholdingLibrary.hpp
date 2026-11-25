// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The user's workholding library: one template file (*.ocamhold, see
// ocam/document/Workholding.hpp) per vise, named after its id, in a folder of the application
// data directory. Setups use a copy stored in the project (Project::workholdings), so projects
// stay portable.

#pragma once

#include <ocam/document/Workholding.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace ocam::gui {

class WorkholdingLibrary {
public:
    /// The library folder (created on demand). Settings key "workholdingLibraryFolder"
    /// overrides the default <app data>/workholding.
    static std::filesystem::path folder();
    /// Uses `folder` instead of the settings' or the default one (test hooks); empty clears it.
    static void setFolderOverride(std::filesystem::path folder);

    struct Entry {
        document::WorkholdingPackage package;
        std::filesystem::path file;
    };
    /// Every valid template file in the folder, by name. Unreadable files are reported in
    /// `problems` (one sentence each) and skipped.
    static std::vector<Entry> load(std::vector<std::string>* problems = nullptr);

    /// Writes <folder>/<id>.ocamhold (replacing the template of the same id). Throws ocam::Error
    /// for invalid templates or write failures.
    static std::filesystem::path save(const document::WorkholdingPackage& package);

    /// Deletes the template's file; false when there is none.
    static bool remove(const Uuid& id);
};

} // namespace ocam::gui
