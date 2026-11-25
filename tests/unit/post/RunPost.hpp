// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Posting in tests: the built-in JavaScript posts, and FanucMill with changed settings.

#pragma once

#include <ocam/post/Engine.hpp>
#include <ocam/post/Post.hpp>

#include <string>
#include <vector>

namespace ocam::post::testing {

/// Runs a post over a program. `warnings` receives the program's and the post's warnings.
inline std::string runPost(const PostScript& post, const NcProgram& program,
                           std::vector<std::string>* warnings = nullptr) {
    CancelToken cancel;
    PostResult result = post.run(program, post.optionValues({}), cancel);
    if (warnings) {
        warnings->insert(warnings->end(), program.warnings.begin(), program.warnings.end());
        warnings->insert(warnings->end(), result.warnings.begin(), result.warnings.end());
    }
    return result.text;
}

/// Lowers jobs as the post declares and runs it.
inline std::string postJobs(const std::vector<PostJob>& jobs, const PostScript& post,
                            std::string name, int number,
                            std::vector<std::string>* warnings = nullptr) {
    return runPost(post, lower(jobs, std::move(name), number, lowerOptionsFor(post.manifest())),
                   warnings);
}

inline const PostScript& mill() {
    return builtinPost("fanuc-generic-mill");
}

/// A FanucMill post whose constructor runs `settings` (JavaScript statements), with `manifest`
/// added to its manifest (for example `cycles: []`).
inline PostScript millWith(const std::string& settings, const std::string& manifest = "") {
    CancelToken cancel;
    return PostScript::load(R"js(
import { FanucMill } from "oxcam:fanuc";
export const manifest = {
  id: "test-mill", name: "Test mill", version: "1", engine: 1, machines: ["mill3"],
  )js" + manifest + R"js(
};
export const options = [];
export default class extends FanucMill {
  constructor(context) {
    super(context);
    )js" + settings + R"js(
  }
}
)js",
                            "test-mill.js", cancel);
}

/// The mill post with 4 decimals for lengths and 2 for feeds (back-plots within 1e-4 mm).
inline const PostScript& fineMill() {
    static const PostScript post =
        millWith("this.decimals = { ...this.decimals, linear: 4, feed: 2 };");
    return post;
}

} // namespace ocam::post::testing
