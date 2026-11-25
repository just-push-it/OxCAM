// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The post engine: post processors are JavaScript modules run by an embedded, sandboxed
// QuickJS-ng (ADR-0006).
//
//   PostScript::load() : compiles a post, reads its manifest and option definitions
//   PostScript::run()  : runs the post's handlers over an NC program in a fresh runtime
//
// A post can only produce text and messages: it has no file, network, process, clock or random
// access, and can import only oxcam: modules. See docs/design-log/0039.

#pragma once

#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/post/Post.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ocam::post {

/// The engine API version. A post's manifest names the version it needs (`engine`); posts that
/// need a newer one are refused.
inline constexpr int kPostEngineVersion = 1;

/// What a post declares about itself and its control (the post's `manifest` export). Lowering
/// reads the declarations; see DL-0039 for what each one does.
/// Work offsets a post writes: numbers `from` to `to` (1 = the first work offset of a setup) as
/// `code` with "{}" replaced by `start` + (n - from), e.g. {1, 6, "G{}", 54}: G54 to G59.
struct WorkOffsetRange {
    int from = 1;
    int to = 1;
    std::string code;
    int start = 1;
    /// How the user chooses the range ("Standard", "Extended"); named so when the post does not.
    std::string name;

    bool operator==(const WorkOffsetRange&) const = default;
};

struct PostManifest {
    std::string id;
    std::string name;
    std::string version;
    int engine = kPostEngineVersion;
    std::vector<std::string> machines; ///< Machine kinematics ids (machine::toString(Kinematics)).
    std::string extension;
    double maxArcSweep = 0.0;           ///< [rad]
    std::vector<std::string> arcPlanes; ///< "xy", "zx", "yz"
    bool helicalArcs = true;
    double tolerance = 0.0;          ///< [mm] arcs the post cannot take become lines within it
    std::vector<std::string> cycles; ///< Canned cycle types the control takes.
    bool tcp = true; ///< Multi-axis: tool centre point control, else machine positions.
    std::optional<std::string> verify; ///< Back-plot dialect, or none.
    std::string encoding;
    /// It writes a sub-spindle's work and the transfer to it (ADR-0007).
    bool subSpindle = false;
    /// The work offsets the control has and their codes (manifest `workOffsets`); without the
    /// declaration G54-G59 and G54.1 P1-P48 (1-54).
    std::vector<WorkOffsetRange> workOffsets;
};

/// An option the user sets when posting (the post's `options` export).
struct PostOption {
    enum class Type { Bool, Number, Choice, Text };
    enum class Scope { Program, Operation };

    std::string id;
    std::string label;
    Type type = Type::Bool;
    Scope scope = Scope::Program;
    Json defaultValue;
    std::vector<std::string> choices; ///< Choice options.
    std::optional<double> min;        ///< Number options.
    std::optional<double> max;
};

struct EngineLimits {
    std::size_t memory = std::size_t{1} << 30; ///< [bytes] heap of one run
    /// [bytes] JavaScript stack. Below the 1 MiB default thread stack of Windows, so deep
    /// recursion ends in a RangeError before the native stack runs out.
    std::size_t stack = std::size_t{512} << 10;
};

struct PostResult {
    std::string text;
    std::vector<std::string> warnings;
};

/// A compiled post: its source, manifest and options. Cheap to copy and safe to share between
/// threads; every load() and run() evaluates the module in a runtime of its own.
class PostScript {
public:
    /// Compiles `source` (an ES module) and reads its exports. `name` is the file name shown in
    /// messages ("my-post.js"). Throws Error(Parse) for syntax errors, failing top-level code,
    /// and a missing or invalid manifest, options or default export; OperationCancelled when
    /// `cancel` is set.
    static PostScript load(std::string source, std::string name, CancelToken& cancel,
                           const EngineLimits& limits = {});

    const std::string& name() const noexcept { return name_; }
    const PostManifest& manifest() const noexcept { return manifest_; }
    const std::vector<PostOption>& options() const noexcept { return options_; }

    /// The values of the program options: `values` (option id -> value) checked against the
    /// definitions, with the definitions' defaults for the options it leaves out. Throws
    /// Error(InvalidArgument) for unknown options and values of the wrong type or range.
    Json optionValues(const Json& values) const;
    /// The same for the per-operation options of one operation.
    Json operationOptionValues(const Json& values) const;

    /// Runs the post over `program` and returns the text. Throws Error(InvalidArgument) naming
    /// the post's file, line and message when the post fails or calls error(), and
    /// OperationCancelled when `cancel` is set.
    PostResult run(const NcProgram& program, const Json& optionValues, CancelToken& cancel) const;

private:
    Json checkedOptions(const Json& values, PostOption::Scope scope) const;

    std::string source_;
    std::string name_;
    EngineLimits limits_;
    PostManifest manifest_;
    std::vector<PostOption> options_;
};

/// How OxCAM lowers a program for a post (its declarations): the canned cycle types it takes,
/// tool centre point control or machine positions, the arc planes it takes, helical arcs, the
/// longest arc sweep and the tolerance of arcs made into lines.
LowerOptions lowerOptionsFor(const PostManifest& manifest);

/// The code a post writes for work offset `n` (WorkOffsetRange), or nullopt when it has none.
std::optional<std::string> workOffsetCode(const PostManifest& manifest, int n);
/// The highest work offset number the post declares.
int lastWorkOffset(const PostManifest& manifest);
/// The post's work offsets in words: "G54-G59 (1-6), G54.1 P1-P48 (7-54)".
std::string describeWorkOffsets(const PostManifest& manifest);

/// The built-in posts (data/posts/*.js, embedded), loaded on first use. Throws
/// Error(InvalidArgument) for an unknown id.
const PostScript& builtinPost(std::string_view id);
/// The ids of the built-in posts.
std::vector<std::string> builtinPostIds();

} // namespace ocam::post
