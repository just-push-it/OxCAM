// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The user's post processor library: JavaScript post files (*.js) in a folder of the
// application data directory (ADR-0006, DL-0039). Setups use a copy stored in the project, so
// projects stay portable; see Project::posts.

#pragma once

#include <ocam/foundation/Json.hpp>
#include <ocam/post/Engine.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace ocam::gui {

class PostLibrary {
public:
    /// The library folder (created on demand). Settings key "postLibraryFolder" overrides the
    /// default <app data>/posts.
    static std::filesystem::path folder();

    struct Entry {
        post::PostScript post;
        std::string source; ///< The file's text, as a project copies it.
        std::filesystem::path file;
    };
    /// Every post file in the folder that loads, by name. Files that do not load are reported in
    /// `problems` (one sentence each) and skipped.
    static std::vector<Entry> load(std::vector<std::string>* problems = nullptr);

    /// The user's own defaults for a post's program options ({id: value}), saved in the settings
    /// under "postDefaults/<post id>": the Post Process dialog starts from them, below the
    /// setup's values. Empty when none.
    static Json userDefaults(const std::string& postId);
    static void setUserDefaults(const std::string& postId, const Json& values);
};

} // namespace ocam::gui
