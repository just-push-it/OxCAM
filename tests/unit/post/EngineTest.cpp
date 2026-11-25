// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The JavaScript post engine (DL-0039): loading, manifests and options, running handlers, the
// sandbox, limits, cancellation and error reporting.

#include <ocam/foundation/Error.hpp>
#include <ocam/post/Engine.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <fmt/format.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <string>
#include <thread>

using namespace ocam;
using namespace ocam::post;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;
using Catch::Matchers::WithinAbs;

namespace {

constexpr const char* kManifest = R"js(
export const manifest = {
  id: "test-post", name: "Test post", version: "1.0.0", engine: 1, machines: ["mill3"],
};
)js";

/// A post with the test manifest, no options, and `body` (which defines the default export).
std::string postSource(const std::string& body, const std::string& options = "[]") {
    return std::string(kManifest) + "export const options = " + options + ";\n" + body;
}

/// A post class that writes one line per handler call: "<handler> <op fields>".
const std::string kEchoPost = postSource(R"js(
import { out } from "oxcam:post";
export default class {
  constructor(context) { this.program = context.program; this.options = context.options; }
  programBegin(p) { out.line(`begin ${p.name} ${p.number}`); }
  programEnd(p) { out.line("end"); }
  comment(i) { out.line(`comment ${i.text}`); }
  toolChange(i) { out.line(`toolChange ${i.tool} ${i.tool_name}`); }
  rapid(i) { out.line(`rapid ${i.x} ${i.y} ${i.z}`); }
  linear(i) { out.line(`linear ${i.x} ${i.y} ${i.z} ${i.feed}`); }
}
)js");

PostScript load(const std::string& source, const EngineLimits& limits = {}) {
    CancelToken cancel;
    return PostScript::load(source, "post.js", cancel, limits);
}

NcProgram smallProgram() {
    NcProgram program;
    program.name = "PART SETUP 1";
    program.number = 1000;
    NcInstr comment;
    comment.op = NcOp::Comment;
    comment.text = "Face";
    NcInstr tool;
    tool.op = NcOp::ToolChange;
    tool.tool = 3;
    tool.toolName = "D10 end mill";
    NcInstr rapid;
    rapid.op = NcOp::Rapid;
    rapid.target = Vec3(10.0, 20.0, std::numeric_limits<double>::quiet_NaN());
    NcInstr linear;
    linear.op = NcOp::Linear;
    linear.target = Vec3(10.0, 20.0, -1.5);
    linear.feed = 10.0; // mm/s = 600 mm/min
    program.instrs = {comment, tool, rapid, linear};
    return program;
}

std::string runText(const PostScript& script, const NcProgram& program = smallProgram(),
                    const Json& options = Json::object()) {
    CancelToken cancel;
    return script.run(program, script.optionValues(options), cancel).text;
}

/// The message of the Error that `f` throws (fails when it throws none).
template <class F> std::string errorOf(F&& f) {
    try {
        f();
    } catch (const Error& e) {
        return e.what();
    }
    FAIL("no error thrown");
    return {};
}

} // namespace

TEST_CASE("a post loads with its manifest and the declarations' defaults") {
    const PostScript script = load(kEchoPost);
    const PostManifest& m = script.manifest();
    CHECK(script.name() == "post.js");
    CHECK(m.id == "test-post");
    CHECK(m.name == "Test post");
    CHECK(m.version == "1.0.0");
    CHECK(m.engine == 1);
    CHECK(m.machines == std::vector<std::string>{"mill3"});
    CHECK(m.extension == "nc");
    CHECK_THAT(m.maxArcSweep, WithinAbs(kPi, 1e-12));
    CHECK(m.arcPlanes == std::vector<std::string>{"xy", "zx", "yz"});
    CHECK(m.helicalArcs);
    CHECK(m.cycles.size() == 7);
    CHECK(m.tcp);
    CHECK_FALSE(m.verify);
    CHECK(m.encoding == "ascii");
    CHECK(script.options().empty());
}

TEST_CASE("manifest declarations are read") {
    const PostScript script = load(R"js(
export const manifest = {
  id: "lathe-post", name: "Lathe", version: "2.1", engine: 1, machines: ["lathe2"],
  extension: "tap", arcs: { maxSweep: 90, planes: ["zx"], helical: false }, tolerance: 0.01,
  cycles: ["drill"], multiAxis: "machine", verify: "fanuc-lathe",
};
export const options = [];
export default class {}
)js");
    const PostManifest& m = script.manifest();
    CHECK(m.extension == "tap");
    CHECK_THAT(m.maxArcSweep, WithinAbs(kPi / 2.0, 1e-12));
    CHECK(m.arcPlanes == std::vector<std::string>{"zx"});
    CHECK_FALSE(m.helicalArcs);
    CHECK_THAT(m.tolerance, WithinAbs(0.01, 1e-12));
    CHECK(m.cycles == std::vector<std::string>{"drill"});
    CHECK_FALSE(m.tcp);
    CHECK(m.verify == "fanuc-lathe");
}

TEST_CASE("a post runs its handlers over the program in order") {
    const PostScript script = load(kEchoPost);
    CHECK(runText(script) == "begin PART SETUP 1 1000\n"
                             "comment Face\n"
                             "toolChange 3 D10 end mill\n"
                             "rapid 10 20 null\n"
                             "linear 10 20 -1.5 600\n"
                             "end\n");
}

TEST_CASE("posting twice gives the same text") {
    const PostScript script = load(kEchoPost);
    CHECK(runText(script) == runText(script));
}

TEST_CASE("a run starts from a fresh module, not from what the load or an earlier run left") {
    const PostScript script = load(postSource(R"js(
import { out } from "oxcam:post";
let runs = 0;
runs += 1;
export default class {
  programBegin() { out.line(`evaluations ${runs}`); }
  programEnd() {}
  comment() {} toolChange() {} rapid() {} linear() {}
}
)js"));
    CHECK(runText(script) == "evaluations 1\n");
    CHECK(runText(script) == "evaluations 1\n");
}

TEST_CASE("warnings are collected") {
    const PostScript script = load(postSource(R"js(
import { warning } from "oxcam:post";
export default class {
  programBegin() { warning("check the tool table"); }
  programEnd() {}
  comment() {} toolChange() {} rapid() {} linear() {}
}
)js"));
    CancelToken cancel;
    const PostResult result = script.run(smallProgram(), script.optionValues({}), cancel);
    CHECK(result.warnings == std::vector<std::string>{"check the tool table"});
}

TEST_CASE("options: definitions, defaults and values") {
    const PostScript script = load(postSource(R"js(
import { out } from "oxcam:post";
export default class {
  constructor(context) { this.options = context.options; }
  programBegin() { out.line(JSON.stringify(this.options)); }
  programEnd() {}
  comment() {} toolChange() {} rapid() {} linear() {}
}
)js",
                                              R"js([
  { id: "optionalStop", label: "Optional stop", type: "bool", default: false },
  { id: "safeZ", label: "Safe Z", type: "number", default: 50, min: 0, max: 500 },
  { id: "arcs", label: "Arcs", type: "choice", default: "ijk", choices: ["ijk", "r"] },
  { id: "header", label: "Header", type: "text", default: "" },
  { id: "smoothing", label: "Smoothing", type: "bool", default: true, scope: "operation" },
])js"));
    REQUIRE(script.options().size() == 5);
    CHECK(script.options()[1].type == PostOption::Type::Number);
    CHECK(script.options()[1].max == 500.0);
    CHECK(script.options()[4].scope == PostOption::Scope::Operation);

    const Json defaults = script.optionValues({});
    CHECK(defaults ==
          Json{{"optionalStop", false}, {"safeZ", 50}, {"arcs", "ijk"}, {"header", ""}});
    CHECK(script.optionValues({{"arcs", "r"}, {"safeZ", 20.5}})["arcs"] == "r");
    CHECK_THAT(runText(script, smallProgram(), {{"optionalStop", true}}),
               StartsWith(R"({"arcs":"ijk","header":"","optionalStop":true,"safeZ":50})"));

    CHECK_THAT(errorOf([&] { (void)script.optionValues({{"nope", 1}}); }),
               ContainsSubstring("has no option 'nope'"));
    CHECK_THAT(errorOf([&] { (void)script.optionValues({{"safeZ", "high"}}); }),
               ContainsSubstring("option 'safeZ' must be a number"));
    CHECK_THAT(errorOf([&] { (void)script.optionValues({{"safeZ", 900}}); }),
               ContainsSubstring("must be at most 500"));
    CHECK_THAT(errorOf([&] { (void)script.optionValues({{"arcs", "polar"}}); }),
               ContainsSubstring("must be one of: ijk, r"));
    CHECK_THAT(errorOf([&] { (void)script.optionValues({{"smoothing", false}}); }),
               ContainsSubstring("is set per operation"));
}

TEST_CASE("invalid option definitions are refused when the post loads") {
    auto withOptions = [](const std::string& options) {
        return errorOf([&] { (void)load(postSource("export default class {}", options)); });
    };
    CHECK_THAT(withOptions(R"([{ id: "a", label: "A", type: "bool", default: 3 }])"),
               ContainsSubstring("option 'a': the default must be true or false"));
    CHECK_THAT(withOptions(R"([{ id: "a", label: "A", type: "choice", default: "x" }])"),
               ContainsSubstring("needs 'choices'"));
    CHECK_THAT(withOptions(R"([{ id: "a", label: "A", type: "number", default: 1, min: 5 }])"),
               ContainsSubstring("the default must be at least 5"));
    CHECK_THAT(withOptions(R"([{ id: "a", label: "A", type: "bool", default: true },
                                { id: "a", label: "B", type: "bool", default: true }])"),
               ContainsSubstring("defined twice"));
    CHECK_THAT(withOptions(R"([{ id: "a", label: "A", type: "bool", default: true, colour: 1 }])"),
               ContainsSubstring("post.js"));
}

TEST_CASE("invalid manifests and exports are refused when the post loads") {
    CHECK_THAT(
        errorOf([] { (void)load(R"js(export const options = []; export default class {})js"); }),
        ContainsSubstring("post.js: the post does not export 'manifest'"));
    CHECK_THAT(errorOf([] { (void)load(std::string(kManifest) + "export default class {}"); }),
               ContainsSubstring("does not export 'options'"));
    CHECK_THAT(errorOf([] { (void)load(postSource("export default 42;")); }),
               ContainsSubstring("the post's default export must be a class"));
    CHECK_THAT(errorOf([] { (void)load(postSource("")); }),
               ContainsSubstring("the post's default export must be a class"));

    // Unknown declarations are errors, so a misspelled one cannot silently do nothing.
    const std::string misspelled = R"js(
export const manifest = { id: "p", name: "P", version: "1", engine: 1, machines: ["mill3"],
                          arcs: { maxSweeep: 90 } };
export const options = [];
export default class {}
)js";
    CHECK_THAT(errorOf([&] { (void)load(misspelled); }), ContainsSubstring("post.js"));

    const std::string newer = R"js(
export const manifest = { id: "p", name: "P", version: "1", engine: 7, machines: ["mill3"] };
export const options = [];
export default class {}
)js";
    try {
        (void)load(newer);
        FAIL("a post for a newer engine loaded");
    } catch (const Error& e) {
        CHECK(e.code() == ErrorCode::Unsupported);
        CHECK_THAT(e.what(), ContainsSubstring("needs post engine version 7; this OxCAM has "
                                               "version 1"));
    }
}

TEST_CASE("errors name the post's file, line and message") {
    // Syntax errors.
    try {
        (void)load(postSource("export default class {\n  programBegin( {\n}\n"));
        FAIL("a syntax error loaded");
    } catch (const Error& e) {
        CHECK(e.code() == ErrorCode::Parse);
        CHECK_THAT(e.what(), StartsWith("post.js:"));
        CHECK_THAT(e.what(), ContainsSubstring("SyntaxError"));
    }

    // An exception thrown by a handler: line 8 of the source below is the throw.
    const std::string throwing = postSource(R"js(export default class {
  programBegin() {}
  programEnd() {}
  comment() {}
  toolChange(i) {
    this.fail(i);
  }
  fail(i) { throw new RangeError(`no tool ${i.tool}`); }
  rapid() {} linear() {}
}
)js");
    const int throwLine =
        1 + static_cast<int>(std::count(
                throwing.begin(),
                throwing.begin() + static_cast<std::ptrdiff_t>(throwing.find("throw new")), '\n'));
    try {
        (void)runText(load(throwing));
        FAIL("the handler's exception was lost");
    } catch (const Error& e) {
        CHECK(e.code() == ErrorCode::InvalidArgument);
        CHECK_THAT(e.what(), StartsWith(fmt::format("post.js:{}:", throwLine)));
        CHECK_THAT(e.what(), ContainsSubstring("RangeError: no tool 3"));
    }

    // error() stops posting with the post's own message.
    const PostScript stopping = load(postSource(R"js(
import { error } from "oxcam:post";
export default class {
  programBegin() {} programEnd() {} comment() {} rapid() {} linear() {}
  toolChange(i) { error(`tool ${i.tool} is not in the magazine`); }
}
)js"));
    CHECK(errorOf([&] { (void)runText(stopping); }) == "post.js: tool 3 is not in the magazine");

    // Values that are not Error objects.
    CHECK_THAT(errorOf([] { (void)load(postSource("throw 'plain';\nexport default class {}")); }),
               ContainsSubstring("post.js: uncaught plain"));
}

TEST_CASE("a missing or async handler is an error naming it") {
    const PostScript missing = load(postSource(R"js(
export default class { programBegin() {} programEnd() {} comment() {} rapid() {} linear() {} }
)js"));
    CHECK(errorOf([&] { (void)runText(missing); }) ==
          "post.js: the post has no handler 'toolChange'");

    const PostScript async = load(postSource(R"js(
export default class {
  programBegin() {} programEnd() {} toolChange() {} rapid() {} linear() {}
  async comment() {}
}
)js"));
    CHECK_THAT(errorOf([&] { (void)runText(async); }),
               ContainsSubstring("handler 'comment' is async"));
}

TEST_CASE("sandbox: posts can import only oxcam: modules") {
    for (const char* module : {"std", "os", "./helpers.js", "../lib.js", "C:/posts/lib.js",
                               "https://example.com/lib.js"}) {
        CAPTURE(module);
        const std::string source = postSource(fmt::format("import * as m from \"{}\";\n"
                                                          "export default class {{}}",
                                                          module));
        CHECK_THAT(errorOf([&] { (void)load(source); }),
                   ContainsSubstring("posts can import only oxcam: modules"));
    }
    CHECK_THAT(errorOf([] {
                   (void)load(postSource("import * as m from \"oxcam:nothing\";\n"
                                         "export default class {}"));
               }),
               ContainsSubstring("there is no module 'oxcam:nothing'"));
    CHECK_THAT(
        errorOf([] { (void)load(postSource("await import('./x.js');\nexport default class {}")); }),
        ContainsSubstring("posts can import only oxcam: modules"));
}

TEST_CASE("sandbox: no clock, no randomness, no host objects") {
    const PostScript script = load(postSource(R"js(
import { out } from "oxcam:post";
export default class {
  programBegin() {
    out.line([typeof Date, typeof performance, typeof WeakRef, typeof std, typeof os,
              typeof require, typeof process, typeof print, typeof console].join(" "));
    try { Math.random(); out.line("random"); } catch (e) { out.line(e.message); }
  }
  programEnd() {} comment() {} toolChange() {} rapid() {} linear() {}
}
)js"));
    CHECK(runText(script) == "undefined undefined undefined undefined undefined undefined "
                             "undefined undefined undefined\n"
                             "Math.random is not available in posts: the same program must give "
                             "the same output\n");
}

TEST_CASE("sandbox: the memory limit stops a post that allocates without bound") {
    EngineLimits limits;
    limits.memory = std::size_t{32} << 20;
    CHECK_THAT(errorOf([&] {
                   (void)load(postSource("const a = []; for (;;) a.push(new Array(1000).fill(1));\n"
                                         "export default class {}"),
                              limits);
               }),
               ContainsSubstring("out of memory"));
}

TEST_CASE("sandbox: endless recursion ends in a RangeError, not a crash") {
    CHECK_THAT(errorOf([] {
                   (void)load(postSource("function f(n) { return f(n + 1) + 1; }\nf(0);\n"
                                         "export default class {}"));
               }),
               ContainsSubstring("RangeError"));
}

TEST_CASE("cancelling stops a post, even one that never returns") {
    const std::string endless = postSource("for (;;) {}\nexport default class {}");
    CancelToken cancel;
    std::thread canceller([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        cancel.requestCancel();
    });
    CHECK_THROWS_AS(PostScript::load(endless, "post.js", cancel), OperationCancelled);
    canceller.join();

    const PostScript looping = load(postSource(R"js(
export default class {
  programBegin() {} programEnd() {} comment() {} rapid() {} linear() {}
  toolChange() { for (;;) {} }
}
)js"));
    CancelToken runCancel;
    std::thread runCanceller([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        runCancel.requestCancel();
    });
    CHECK_THROWS_AS(looping.run(smallProgram(), looping.optionValues({}), runCancel),
                    OperationCancelled);
    runCanceller.join();
}

TEST_CASE("out.line refuses text with line breaks") {
    const PostScript script = load(postSource(R"js(
import { out } from "oxcam:post";
export default class {
  programBegin() { out.line("G0 X0\nM30"); }
  programEnd() {} comment() {} toolChange() {} rapid() {} linear() {}
}
)js"));
    CHECK_THAT(errorOf([&] { (void)runText(script); }),
               ContainsSubstring("out.line: the text has a line break"));
}
