// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The oxcam:post output library (DL-0039): number formats, modal words and codes, blocks, line
// numbers, comments, buffers and the ASCII check.

#include <ocam/foundation/Error.hpp>
#include <ocam/post/Engine.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <string>

using namespace ocam;
using namespace ocam::post;
using Catch::Matchers::ContainsSubstring;

namespace {

/// A post whose programBegin runs `code` with the whole library imported.
std::string postWith(const std::string& code) {
    return R"js(
import { number, word, code, out, warning, error } from "oxcam:post";
export const manifest = {
  id: "library-test", name: "Library test", version: "1", engine: 1, machines: ["mill3"],
};
export const options = [];
export default class {
  programBegin() {
)js" + code +
           R"js(
  }
  programEnd() {}
}
)js";
}

/// The text the post writes when `code` runs (on an empty program).
std::string written(const std::string& code) {
    CancelToken cancel;
    const PostScript script = PostScript::load(postWith(code), "post.js", cancel);
    return script.run(NcProgram{}, script.optionValues({}), cancel).text;
}

/// The message of the error `code` stops posting with.
std::string failure(const std::string& code) {
    try {
        (void)written(code);
    } catch (const Error& e) {
        return e.what();
    }
    FAIL("posting did not fail");
    return {};
}

} // namespace

TEST_CASE("number formats round, trim and place the point like the C++ formatter") {
    struct Case {
        const char* spec;
        const char* value;
        const char* text;
    };
    const Case cases[] = {
        {"{ decimals: 3 }", "10", "10."},
        {"{ decimals: 3 }", "10.5", "10.5"},
        {"{ decimals: 3 }", "-7.25", "-7.25"},
        {"{ decimals: 3 }", "0.0004", "0."},
        {"{ decimals: 3 }", "-0.0004", "0."}, // never "-0"
        {"{ decimals: 2 }", "0.125", "0.13"}, // half away from zero
        {"{ decimals: 0 }", "2.5", "3."},
        {"{ decimals: 0 }", "-2.5", "-3."},
        {"{ decimals: 0 }", "600", "600."},
        {"{ decimals: 0, point: 'needed' }", "600", "600"},
        {"{ decimals: 3, point: 'needed' }", "10", "10"},
        {"{ decimals: 3, point: 'needed' }", "10.25", "10.25"},
        {"{ decimals: 3, zeros: 'keep' }", "10.5", "10.500"},
        {"{ decimals: 4, zeros: 'keep', point: 'needed' }", "2", "2.0000"},
        {"{ decimals: 4, scale: 1 / 25.4 }", "25.4", "1."},
        {"{ decimals: 3, scale: 2 }", "12.5", "25."}, // lathe diameters
    };
    for (const Case& c : cases) {
        CAPTURE(c.spec, c.value);
        CHECK(written(std::string("out.line(number(") + c.spec + ").format(" + c.value + "));") ==
              std::string(c.text) + "\n");
    }
}

TEST_CASE("number formats refuse invalid settings and values") {
    CHECK_THAT(failure("number({});"),
               ContainsSubstring("'decimals' must be a whole number from 0 to 9"));
    CHECK_THAT(failure("number({ decimals: 10 });"), ContainsSubstring("from 0 to 9"));
    CHECK_THAT(failure("number({ decimals: 1.5 });"), ContainsSubstring("from 0 to 9"));
    CHECK_THAT(failure("number({ decimals: 3, point: 'never' });"),
               ContainsSubstring("'point' must be \"always\" or \"needed\""));
    CHECK_THAT(failure("number({ decimals: 3, zeros: 'pad' });"),
               ContainsSubstring("'zeros' must be \"trim\" or \"keep\""));
    CHECK_THAT(failure("number({ decimals: 3, scale: 0 });"),
               ContainsSubstring("'scale' must not be 0"));
    CHECK_THAT(failure("number({ decimal: 3 });"),
               ContainsSubstring("number: unknown setting 'decimal'"));
    CHECK_THAT(failure("number(3);"), ContainsSubstring("expected settings"));
    CHECK_THAT(failure("number({ decimals: 3 }).format('1');"),
               ContainsSubstring("format: expected a number"));
    CHECK_THAT(failure("number({ decimals: 3 }).format(NaN);"),
               ContainsSubstring("format: the number is not finite"));
    CHECK_THAT(failure("number({ decimals: 3 }).format.call({}, 1);"),
               ContainsSubstring("TypeError"));
}

TEST_CASE("words are written only when their text changes") {
    CHECK(written(R"js(
const x = word("X", number({ decimals: 3 }));
out.line(String(x.last) + " " + x.letter);
out.line("[" + x.of(1) + "]");
out.line("[" + x.of(1.0001) + "]");  // the same text: suppressed
out.line("[" + x.of(null) + "]");    // unchanged in the NC program: nothing, state kept
out.line("[" + x.of(undefined) + "]");
out.line("[" + x.of(1.001) + "] " + x.last);
x.reset();
out.line("[" + x.of(1.001) + "]");
const f = word("F", number({ decimals: 0 }), { modal: false });
out.line(f.of(600) + " " + f.of(600));
const q = word("Q", number({ decimals: 3 }), {});
out.line(q.of(2) + " " + q.of(2));
)js") == "null X\n"
         "[X1.]\n"
         "[]\n"
         "[]\n"
         "[]\n"
         "[X1.001] 1.001\n"
         "[X1.001]\n"
         "F600. F600.\n"
         "Q2. \n");

    CHECK_THAT(failure("word('X', 3);"),
               ContainsSubstring("the second argument must be a format from number()"));
    CHECK_THAT(failure("word('X Y', number({ decimals: 3 }));"),
               ContainsSubstring("letters without spaces"));
    CHECK_THAT(failure("word('X', number({ decimals: 3 }), { modl: false });"),
               ContainsSubstring("unknown setting 'modl'"));
    CHECK_THAT(failure("word('X', number({ decimals: 3 }), { modal: 0 });"),
               ContainsSubstring("'modal' must be true or false"));
    CHECK_THAT(failure("word('X', number({ decimals: 3 })).of('1');"),
               ContainsSubstring("X.of: expected a number"));
}

TEST_CASE("code groups are written only when the code changes") {
    CHECK(written(R"js(
const motion = code();
out.line(String(motion.last));
out.line(motion.of("G1") + "|" + motion.of("G1") + "|" + motion.of("G0") + "|" + motion.last);
motion.reset();
out.line(motion.of("G0") + "|" + motion.of(""));
)js") == "null\n"
         "G1||G0|G0\n"
         "G0|\n");
    CHECK_THAT(failure("code().of(1);"), ContainsSubstring("code.of: expected a code"));
}

TEST_CASE("blocks skip empty words and number the lines") {
    CHECK(written(R"js(
out.block("G0", "", null, undefined, "X1.");
out.block("", null);                 // nothing to write: no block, no number used
out.numbering({ start: 10, step: 10 });
out.block("G1", "Z-1.", "F600.");
out.block();
out.block("M30");
out.numbering({ start: 1, step: 2, max: 5 });
for (const w of ["A", "B", "C", "D"]) out.block(w);  // wraps from 5 back to 1
out.separator = "";
out.block("G0", "X1.", "Y2.");
out.skippable("M01");
out.separator = " ";
out.numbering(null);
out.block("M30");
out.skippable("M00");
out.line("%");
)js") == "G0 X1.\n"
         "N10 G1 Z-1. F600.\n"
         "N20 M30\n"
         "N1 A\n"
         "N3 B\n"
         "N5 C\n"
         "N1 D\n"
         "N3G0X1.Y2.\n"
         "/N5M01\n"
         "M30\n"
         "/M00\n"
         "%\n");

    CHECK(written("out.numbering({}); out.block('A'); out.numbering(false); out.block('B');") ==
          "N10 A\nB\n");
    CHECK_THAT(failure("out.block('G0', 1.5);"),
               ContainsSubstring("out.block: word 2 is not text; format numbers"));
    CHECK_THAT(failure("out.block('G0\\nM30');"), ContainsSubstring("word 1 has a line break"));
    CHECK_THAT(failure("out.numbering({ start: 10, step: 0 });"),
               ContainsSubstring("out.numbering:"));
    CHECK_THAT(failure("out.numbering({ start: 100, max: 50 });"),
               ContainsSubstring("out.numbering:"));
    CHECK_THAT(failure("out.numbering({ begin: 1 });"), ContainsSubstring("unknown setting"));
    CHECK_THAT(failure("out.separator = '\\n';"), ContainsSubstring("line break"));
}

TEST_CASE("comments are cleaned like the C++ formatter and never numbered") {
    CHECK(written(R"js(
out.numbering({ start: 10 });
out.comment("Ø10 end mill (rough); 90° chamfer");
out.upperComments = true;
out.comment("ø6 drill");
out.block("M30");
out.commentStyle = "semicolon";
out.comment("face (top)");
out.line(out.commentText("tool: Ø8"));
out.line(out.commentStyle + " " + out.upperComments);
out.commentStyle = "none";
out.comment("not written");
out.block("M30");
)js") == "(D10 end mill rough 90DEG chamfer)\n"
         "(D6 DRILL)\n"
         "N10 M30\n"
         "; FACE TOP\n"
         "TOOL: D8\n"
         "semicolon true\n"
         "N20 M30\n");
    CHECK_THAT(failure("out.commentStyle = 'braces';"),
               ContainsSubstring("out.commentStyle must be"));
    CHECK_THAT(failure("out.upperComments = 1;"),
               ContainsSubstring("out.upperComments must be true or false"));
}

TEST_CASE("buffers collect blocks to write later") {
    CHECK(written(R"js(
out.numbering({ start: 10 });
const sub = out.buffer();
out.block("G0", "X0.");
out.into(sub, () => {
  out.block("G1", "X10.");
  out.comment("in the buffer");
});
out.block("M99");
out.line("(" + JSON.stringify(sub.text) + ")");
out.append(sub);
)js") == "N10 G0 X0.\n"
         "N30 M99\n"
         "(\"N20 G1 X10.\\n(in the buffer)\\n\")\n"
         "N20 G1 X10.\n"
         "(in the buffer)\n");

    // Nested buffers, and the output goes back where it was when the function throws.
    CHECK(written(R"js(
const outer = out.buffer();
const inner = out.buffer();
out.into(outer, () => {
  out.line("outer 1");
  out.into(inner, () => out.line("inner"));
  out.line("outer 2");
});
try {
  out.into(inner, () => { throw new Error("stop"); });
} catch (e) {}
out.line("main");
out.append(outer);
out.append(inner);
)js") == "main\n"
         "outer 1\n"
         "outer 2\n"
         "inner\n");

    CHECK_THAT(failure("const b = out.buffer(); out.into(b, () => out.append(b));"),
               ContainsSubstring("a buffer cannot be appended to itself"));
    CHECK_THAT(failure("out.into({}, () => {});"), ContainsSubstring("TypeError"));
    CHECK_THAT(failure("out.into(out.buffer(), 3);"),
               ContainsSubstring("out.into: expected a buffer and a function"));
}

TEST_CASE("the output must be printable ASCII") {
    CHECK_THAT(failure("out.line('%'); out.line('G0 X1. (\\u00e9)');"),
               ContainsSubstring("post.js: line 2 of the output has a character that is not "
                                 "printable ASCII (byte 0xC3)"));
    CHECK_THAT(failure("out.line('G0\\u0001');"), ContainsSubstring("byte 0x01"));
    CHECK(written("out.line('G0\\tX1.');") == "G0\tX1.\n");
    // Comments are cleaned before they are written, so they always pass.
    CHECK(written("out.comment('é Ø');") == "( D)\n");
}

TEST_CASE("warnings and errors from the library module") {
    CancelToken cancel;
    const PostScript script =
        PostScript::load(postWith("warning('first'); warning('second');"), "post.js", cancel);
    CHECK(script.run(NcProgram{}, script.optionValues({}), cancel).warnings ==
          std::vector<std::string>{"first", "second"});
    CHECK(failure("error('spindle too fast');") == "post.js: spindle too fast");
    CHECK(failure("error();") == "post.js: the post stopped");
}
