// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The user's tool libraries: any number of .ocamtools files (for example one per machine or
// process), one of them the default. New libraries go to a folder of the application data
// directory; existing files elsewhere can be added. Projects keep copies of the tools they use
// (Project::tools), so they stay portable.

#pragma once

#include <ocam/tooling/ToolLibrary.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace ocam::gui {

class ToolLibraries {
public:
    /// Where new libraries are created (<app data>/tool-libraries, created on demand).
    static std::filesystem::path folder();

    /// The library files, in the user's order (settings "toolLibraries"; the single library of
    /// earlier versions, "toolLibraryPath", is taken over).
    static std::vector<std::filesystem::path> files();
    static void setFiles(const std::vector<std::filesystem::path>& files);
    /// Adds a file to the list (no-op when listed); the first library becomes the default.
    static void add(const std::filesystem::path& file);
    /// Takes a file off the list (the file itself is kept).
    static void remove(const std::filesystem::path& file);

    /// The default library: the one the tool library command opens and whose tools come first
    /// in tool pickers. nullopt when there are no libraries.
    static std::optional<std::filesystem::path> defaultFile();
    static void setDefaultFile(const std::optional<std::filesystem::path>& file);

    struct Entry {
        std::filesystem::path file;
        tooling::ToolLibrary library;
        bool isDefault = false;
    };
    /// Every readable library, the default first. Files are re-read only when they changed.
    /// Unreadable files are reported in `problems` (one sentence each) and skipped.
    static std::vector<Entry> load(std::vector<std::string>* problems = nullptr);

    /// Creates an empty library called `name` in folder() (a free file name derived from the
    /// name), adds it to the list and returns its file. Throws ocam::Error on write failures.
    static std::filesystem::path create(const std::string& name);
};

} // namespace ocam::gui
