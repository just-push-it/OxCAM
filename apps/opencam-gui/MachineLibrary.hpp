// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The user's machine library: one oxcam.machine JSON file per machine (named after its id) in
// a folder of the application data directory, with its model's part meshes in <id>.meshes. Setups
// use a copy stored in the project, so projects stay portable; see Project::machines.

#pragma once

#include <ocam/machine/Machine.hpp>

#include <QString>
#include <filesystem>
#include <string>
#include <vector>

namespace ocam::gui {

class MachineLibrary {
public:
    /// The library folder (created on demand). Settings key "machineLibraryFolder" overrides
    /// the default <app data>/machines.
    static std::filesystem::path folder();

    struct Entry {
        machine::Machine machine;
        std::filesystem::path file;
    };
    /// Every valid machine file in the folder, by name. Unreadable files are reported in
    /// `problems` (one sentence each) and skipped.
    static std::vector<Entry> load(std::vector<std::string>* problems = nullptr);

    /// Writes <folder>/<id>.json (replacing a file of the same id), its revision one more than
    /// the file's when the machine differs from it (the file's when not; a new file: one more
    /// than the machine's). Throws ocam::Error for invalid machines or write failures.
    static std::filesystem::path save(const machine::Machine& machine);

    /// Deletes the machine's file; false when there is none.
    static bool remove(const std::string& id);

    /// True when the id is a valid machine id that no built-in machine uses.
    static bool isUsableId(const std::string& id);
};

} // namespace ocam::gui
