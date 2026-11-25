// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The posts a setup can be posted with (the setup page, the Post Process dialog) and editors of
// post options. A setup chooses a built-in post or one the project carries; posts from the
// user's library are copied into the project when used (DL-0039).

#pragma once

#include "PostLibrary.hpp"

#include <ocam/document/Project.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/machine/Machine.hpp>
#include <ocam/post/Engine.hpp>

#include <QString>
#include <QStringList>
#include <functional>
#include <optional>
#include <string>
#include <vector>

class QFormLayout;
class QWidget;

namespace ocam::gui {

struct PostEntry {
    enum class Kind {
        Machine,   ///< the machine's default post (the setup chooses none)
        BuiltIn,   ///< a built-in post
        InProject, ///< a post the project carries
        InLibrary, ///< a post of the user's library, copied into the project when used
    };
    Kind kind = Kind::Machine;
    std::string id; ///< The post's id (Machine: the post the machine's default resolves to).
    QString label;
    /// InLibrary, and Machine when the machine's default is a library post the project does not
    /// carry yet: the file to copy.
    std::optional<PostLibrary::Entry> library;
};

/// The choices for a setup on `machine`: the machine's default first, then the built-in posts,
/// the project's posts and the library's posts that write for the machine's kinematics (and the
/// setup's own choice even when it does not). Posts that do not load are reported in `problems`.
std::vector<PostEntry> postEntries(const document::Project& project, const document::Setup& setup,
                                   const machine::Machine& machine, QStringList* problems);

/// The heading over a kind's entries in a list (empty for the machine's default, which needs
/// none): where they come from, said once rather than before each name.
QString postKindHeading(PostEntry::Kind kind);

/// The index of the setup's choice in `entries` (0, the machine's default, when it chooses none).
int currentPostEntry(const std::vector<PostEntry>& entries, const document::Setup& setup);

/// What a setup stores for an entry: none for the machine's default.
std::optional<document::PostChoice> postChoiceOf(const PostEntry& entry);

/// The post an entry stands for. Throws as document::postFor.
post::PostScript entryPost(const PostEntry& entry, const document::Project& project,
                           const machine::Machine& machine);

/// Adds a post to the project, replacing a copy with the same id.
void addProjectPost(document::Project& project, const document::ProjectPost& copy);

/// A project copy of a library post.
document::ProjectPost projectCopy(const PostLibrary::Entry& entry);

/// Adds the rows that choose a setup's work offset to `form`: "Work offset", the post's ranges by
/// name ("Standard", "Extended"), and the number in the range's code ("G154 P" [2]), as far as
/// the machine has work offsets. `current` shows even beyond them (marked). With
/// `setupsChoice`, the ranges start with the setup's own work offset (0; operations). `set`
/// receives the work offset chosen.
void addWorkOffsetRows(QFormLayout* form, const document::Project& project,
                       const document::Setup& setup, int current, bool setupsChoice,
                       std::function<void(int)> set);

/// The value of a post option in `values` ({id: value}), else the option's default.
Json postOptionValue(const Json& values, const post::PostOption& option);

/// An editor of one post option showing `value`; `set` receives each new value.
QWidget* postOptionEditor(const post::PostOption& option, const Json& value,
                          std::function<void(const Json&)> set);

} // namespace ocam::gui
