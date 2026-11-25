// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The oxcam:post module (DL-0039). Everything a post writes goes through these functions, which
// keep the output rules (number formats, modal suppression, line numbers, comment characters) in
// one place, written in C++ for speed. The rules follow the C++ formatter of DL-0011 so the
// generic posts can reproduce its output exactly.

#include "OutputLibrary.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <initializer_list>
#include <memory>
#include <new>
#include <string_view>

namespace ocam::post::detail {

std::optional<std::string> toText(JSContext* ctx, JSValueConst value) {
    std::size_t length = 0;
    const char* text = JS_ToCStringLen(ctx, &length, value);
    if (!text) {
        return std::nullopt;
    }
    std::string out(text, length);
    JS_FreeCString(ctx, text);
    return out;
}

std::string formatNumber(double value, const NumberSpec& spec) {
    static constexpr double kPowers[] = {1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9};
    const double factor = kPowers[std::clamp(spec.decimals, 0, 9)];
    double rounded = std::round(value * spec.scale * factor) / factor;
    if (rounded == 0.0) {
        rounded = 0.0; // no "-0"
    }
    // Formatted in place: most numbers fit the string's own buffer and allocate nothing.
    char buffer[400];
    char* end = fmt::format_to_n(buffer, sizeof buffer - 1, "{:.{}f}", rounded, spec.decimals).out;
    if (std::find(buffer, end, '.') != end) {
        if (!spec.keepZeros) {
            while (end[-1] == '0') {
                --end;
            }
        }
        if (end[-1] == '.' && !spec.point) {
            --end;
        }
    } else if (spec.point) {
        *end++ = '.';
    }
    return std::string(buffer, end);
}

std::optional<std::string> asciiProblem(const std::string& text) {
    int line = 1;
    for (const char c : text) {
        const auto byte = static_cast<unsigned char>(c);
        if (c == '\n') {
            ++line;
        } else if (byte >= 0x7F || (byte < 0x20 && c != '\t')) {
            return fmt::format("line {} of the output has a character that is not printable "
                               "ASCII (byte 0x{:02X})",
                               line, byte);
        }
    }
    return std::nullopt;
}

namespace {

struct WordState {
    std::string letter;
    NumberSpec spec;
    bool modal = true;
    std::optional<std::string> last;
};

struct CodeState {
    std::optional<std::string> last;
};

struct BufferState {
    std::string text;
};

// -- Helpers ------------------------------------------------------------------------------

/// Runs a native function body: no C++ exception may unwind through QuickJS.
template <class F> JSValue guarded(JSContext* ctx, F&& body) {
    try {
        return body();
    } catch (const std::bad_alloc&) {
        return JS_ThrowOutOfMemory(ctx);
    } catch (const std::exception& e) {
        return JS_ThrowInternalError(ctx, "%s", e.what());
    }
}

JSValue typeError(JSContext* ctx, const std::string& message) {
    return JS_ThrowTypeError(ctx, "%s", message.c_str());
}

template <class T> void finalize(JSRuntime*, JSValueConst value) {
    JSClassID id = 0;
    delete static_cast<T*>(JS_GetAnyOpaque(value, &id));
}

template <class T> T* self(JSContext* ctx, JSValueConst value, JSClassID id) {
    return static_cast<T*>(JS_GetOpaque2(ctx, value, id));
}

/// A new object of a library class that owns `state`.
template <class T> JSValue newObject(JSContext* ctx, JSClassID id, std::unique_ptr<T> state) {
    const JSValue object = JS_NewObjectClass(ctx, id);
    if (JS_IsException(object)) {
        return object;
    }
    JS_SetOpaque(object, state.release());
    return object;
}

bool hasLineBreak(std::string_view text) {
    return text.find_first_of("\r\n") != std::string_view::npos;
}

/// The own keys of a settings object must be among `allowed`, so a misspelled setting is an
/// error rather than ignored.
bool checkKeys(JSContext* ctx, JSValueConst object, const char* what,
               std::initializer_list<std::string_view> allowed) {
    JSPropertyEnum* keys = nullptr;
    std::uint32_t count = 0;
    if (JS_GetOwnPropertyNames(ctx, &keys, &count, object, JS_GPN_STRING_MASK | JS_GPN_ENUM_ONLY) <
        0) {
        return false;
    }
    bool ok = true;
    for (std::uint32_t i = 0; i < count && ok; ++i) {
        const char* key = JS_AtomToCString(ctx, keys[i].atom);
        if (!key) {
            ok = false;
        } else {
            if (std::ranges::find(allowed, std::string_view(key)) == allowed.end()) {
                typeError(ctx, fmt::format("{}: unknown setting '{}'", what, key));
                ok = false;
            }
            JS_FreeCString(ctx, key);
        }
    }
    JS_FreePropertyEnum(ctx, keys, count);
    return ok;
}

/// An optional setting of a settings object: false on error (exception pending).
bool readNumber(JSContext* ctx, JSValueConst object, const char* what, const char* key,
                std::optional<double>& out) {
    const JSValue value = JS_GetPropertyStr(ctx, object, key);
    if (JS_IsException(value)) {
        return false;
    }
    if (JS_IsUndefined(value)) {
        return true;
    }
    double number = 0.0;
    if (!JS_IsNumber(value) || JS_ToFloat64(ctx, &number, value) < 0 || !std::isfinite(number)) {
        JS_FreeValue(ctx, value);
        typeError(ctx, fmt::format("{}: '{}' must be a number", what, key));
        return false;
    }
    out = number;
    return true;
}

bool readText(JSContext* ctx, JSValueConst object, const char* what, const char* key,
              std::optional<std::string>& out) {
    const JSValue value = JS_GetPropertyStr(ctx, object, key);
    if (JS_IsException(value)) {
        return false;
    }
    if (JS_IsUndefined(value)) {
        return true;
    }
    if (!JS_IsString(value)) {
        JS_FreeValue(ctx, value);
        typeError(ctx, fmt::format("{}: '{}' must be text", what, key));
        return false;
    }
    out = toText(ctx, value);
    JS_FreeValue(ctx, value);
    return out.has_value();
}

bool readBool(JSContext* ctx, JSValueConst object, const char* what, const char* key,
              std::optional<bool>& out) {
    const JSValue value = JS_GetPropertyStr(ctx, object, key);
    if (JS_IsException(value)) {
        return false;
    }
    if (JS_IsUndefined(value)) {
        return true;
    }
    if (!JS_IsBool(value)) {
        JS_FreeValue(ctx, value);
        typeError(ctx, fmt::format("{}: '{}' must be true or false", what, key));
        return false;
    }
    out = JS_ToBool(ctx, value) > 0;
    return true;
}

bool isWhole(double value) {
    return std::floor(value) == value;
}

/// A finite number argument; nullopt with a TypeError pending otherwise.
std::optional<double> numberArgument(JSContext* ctx, JSValueConst value, const std::string& what) {
    double number = 0.0;
    if (!JS_IsNumber(value) || JS_ToFloat64(ctx, &number, value) < 0) {
        typeError(ctx, fmt::format("{}: expected a number", what));
        return std::nullopt;
    }
    if (!std::isfinite(number)) {
        typeError(ctx, fmt::format("{}: the number is not finite", what));
        return std::nullopt;
    }
    return number;
}

/// Comment text the way controls accept it: plain ASCII, without ( ) ; or line breaks.
std::string cleanComment(std::string text, bool upper) {
    auto replaceAll = [&](std::string_view from, std::string_view to) {
        for (std::size_t pos = 0; (pos = text.find(from, pos)) != std::string::npos;
             pos += to.size()) {
            text.replace(pos, from.size(), to);
        }
    };
    replaceAll("\xC3\x98", "D"); // diameter sign (capital O with stroke)
    replaceAll("\xC3\xB8", "D");
    replaceAll("\xC2\xB0", "DEG"); // degree sign
    std::erase_if(text, [](char c) {
        return c == '(' || c == ')' || c == ';' || c == '\n' || c == '\r' ||
               static_cast<unsigned char>(c) >= 0x80;
    });
    if (upper) {
        std::ranges::transform(text, text.begin(),
                               [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    }
    return text;
}

// -- number() -------------------------------------------------------------------------------

JSValue jsNumber(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        constexpr const char* what = "number";
        if (argc < 1 || !JS_IsObject(argv[0])) {
            return typeError(ctx, "number: expected settings such as { decimals: 3 }");
        }
        std::optional<double> decimals;
        std::optional<double> scale;
        std::optional<std::string> point;
        std::optional<std::string> zeros;
        if (!checkKeys(ctx, argv[0], what, {"decimals", "point", "zeros", "scale"}) ||
            !readNumber(ctx, argv[0], what, "decimals", decimals) ||
            !readNumber(ctx, argv[0], what, "scale", scale) ||
            !readText(ctx, argv[0], what, "point", point) ||
            !readText(ctx, argv[0], what, "zeros", zeros)) {
            return JS_EXCEPTION;
        }
        if (!decimals || !isWhole(*decimals) || *decimals < 0.0 || *decimals > 9.0) {
            return typeError(ctx, "number: 'decimals' must be a whole number from 0 to 9");
        }
        if (point && *point != "always" && *point != "needed") {
            return typeError(ctx, "number: 'point' must be \"always\" or \"needed\"");
        }
        if (zeros && *zeros != "trim" && *zeros != "keep") {
            return typeError(ctx, "number: 'zeros' must be \"trim\" or \"keep\"");
        }
        if (scale && *scale == 0.0) {
            return typeError(ctx, "number: 'scale' must not be 0");
        }
        auto spec = std::make_unique<NumberSpec>();
        spec->decimals = static_cast<int>(*decimals);
        spec->point = !point || *point == "always";
        spec->keepZeros = zeros && *zeros == "keep";
        spec->scale = scale.value_or(1.0);
        return newObject(ctx, outputOf(ctx).formatClass, std::move(spec));
    });
}

JSValue jsFormat(JSContext* ctx, JSValueConst thisValue, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        const NumberSpec* spec = self<NumberSpec>(ctx, thisValue, outputOf(ctx).formatClass);
        if (!spec) {
            return JS_EXCEPTION;
        }
        const auto value = numberArgument(ctx, argc > 0 ? argv[0] : JS_UNDEFINED, "format");
        if (!value) {
            return JS_EXCEPTION;
        }
        const std::string text = formatNumber(*value, *spec);
        return JS_NewStringLen(ctx, text.data(), text.size());
    });
}

// -- word() ---------------------------------------------------------------------------------

JSValue jsWord(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        Output& output = outputOf(ctx);
        if (argc < 2 || !JS_IsString(argv[0])) {
            return typeError(ctx, "word: expected a letter and a number format, such as "
                                  "word(\"X\", number({ decimals: 3 }))");
        }
        auto state = std::make_unique<WordState>();
        const auto letter = toText(ctx, argv[0]);
        if (!letter) {
            return JS_EXCEPTION;
        }
        if (letter->empty() ||
            std::ranges::any_of(*letter, [](unsigned char c) { return std::isspace(c) != 0; })) {
            return typeError(ctx, "word: the address must be letters without spaces, such as "
                                  "\"X\" or \"Q\"");
        }
        state->letter = *letter;
        const auto* spec =
            static_cast<const NumberSpec*>(JS_GetOpaque(argv[1], output.formatClass));
        if (!spec) {
            return typeError(ctx, fmt::format("word \"{}\": the second argument must be a "
                                              "format from number()",
                                              *letter));
        }
        state->spec = *spec;
        if (argc > 2 && !JS_IsUndefined(argv[2])) {
            const std::string what = fmt::format("word \"{}\"", *letter);
            if (!JS_IsObject(argv[2])) {
                return typeError(ctx, what + ": expected settings such as { modal: false }");
            }
            std::optional<bool> modal;
            if (!checkKeys(ctx, argv[2], what.c_str(), {"modal"}) ||
                !readBool(ctx, argv[2], what.c_str(), "modal", modal)) {
                return JS_EXCEPTION;
            }
            state->modal = modal.value_or(true);
        }
        return newObject(ctx, output.wordClass, std::move(state));
    });
}

JSValue jsWordOf(JSContext* ctx, JSValueConst thisValue, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        WordState* word = self<WordState>(ctx, thisValue, outputOf(ctx).wordClass);
        if (!word) {
            return JS_EXCEPTION;
        }
        const JSValueConst arg = argc > 0 ? argv[0] : JS_UNDEFINED;
        if (JS_IsUndefined(arg) || JS_IsNull(arg)) {
            return JS_NewString(ctx, ""); // the NC program's "unchanged"
        }
        const auto value = numberArgument(ctx, arg, word->letter + ".of");
        if (!value) {
            return JS_EXCEPTION;
        }
        std::string text = formatNumber(*value, word->spec);
        if (word->modal && word->last == text) {
            return JS_NewString(ctx, "");
        }
        word->last = text;
        text.insert(0, word->letter);
        return JS_NewStringLen(ctx, text.data(), text.size());
    });
}

JSValue jsWordReset(JSContext* ctx, JSValueConst thisValue, int, JSValueConst*) {
    WordState* word = self<WordState>(ctx, thisValue, outputOf(ctx).wordClass);
    if (!word) {
        return JS_EXCEPTION;
    }
    word->last.reset();
    return JS_UNDEFINED;
}

JSValue jsWordLast(JSContext* ctx, JSValueConst thisValue, int, JSValueConst*) {
    const WordState* word = self<WordState>(ctx, thisValue, outputOf(ctx).wordClass);
    if (!word) {
        return JS_EXCEPTION;
    }
    return word->last ? JS_NewStringLen(ctx, word->last->data(), word->last->size()) : JS_NULL;
}

JSValue jsWordLetter(JSContext* ctx, JSValueConst thisValue, int, JSValueConst*) {
    const WordState* word = self<WordState>(ctx, thisValue, outputOf(ctx).wordClass);
    if (!word) {
        return JS_EXCEPTION;
    }
    return JS_NewStringLen(ctx, word->letter.data(), word->letter.size());
}

// -- code() ---------------------------------------------------------------------------------

JSValue jsCode(JSContext* ctx, JSValueConst, int, JSValueConst*) {
    return guarded(ctx, [&]() -> JSValue {
        return newObject(ctx, outputOf(ctx).codeClass, std::make_unique<CodeState>());
    });
}

JSValue jsCodeOf(JSContext* ctx, JSValueConst thisValue, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        CodeState* code = self<CodeState>(ctx, thisValue, outputOf(ctx).codeClass);
        if (!code) {
            return JS_EXCEPTION;
        }
        if (argc < 1 || !JS_IsString(argv[0])) {
            return typeError(ctx, "code.of: expected a code such as \"G1\"");
        }
        const auto text = toText(ctx, argv[0]);
        if (!text) {
            return JS_EXCEPTION;
        }
        if (text->empty() || code->last == *text) {
            return JS_NewString(ctx, "");
        }
        code->last = *text;
        return JS_NewStringLen(ctx, text->data(), text->size());
    });
}

JSValue jsCodeReset(JSContext* ctx, JSValueConst thisValue, int, JSValueConst*) {
    CodeState* code = self<CodeState>(ctx, thisValue, outputOf(ctx).codeClass);
    if (!code) {
        return JS_EXCEPTION;
    }
    code->last.reset();
    return JS_UNDEFINED;
}

JSValue jsCodeLast(JSContext* ctx, JSValueConst thisValue, int, JSValueConst*) {
    const CodeState* code = self<CodeState>(ctx, thisValue, outputOf(ctx).codeClass);
    if (!code) {
        return JS_EXCEPTION;
    }
    return code->last ? JS_NewStringLen(ctx, code->last->data(), code->last->size()) : JS_NULL;
}

// -- out --------------------------------------------------------------------------------------

JSValue writeBlock(JSContext* ctx, int argc, JSValueConst* argv, const char* what, bool skippable) {
    return guarded(ctx, [&]() -> JSValue {
        Output& output = outputOf(ctx);
        std::string words;
        bool any = false;
        for (int i = 0; i < argc; ++i) {
            if (JS_IsUndefined(argv[i]) || JS_IsNull(argv[i])) {
                continue;
            }
            if (!JS_IsString(argv[i])) {
                return typeError(ctx, fmt::format("{}: word {} is not text; format numbers with "
                                                  "number() or word()",
                                                  what, i + 1));
            }
            const auto text = toText(ctx, argv[i]);
            if (!text) {
                return JS_EXCEPTION;
            }
            if (text->empty()) {
                continue;
            }
            if (hasLineBreak(*text)) {
                return typeError(ctx, fmt::format("{}: word {} has a line break", what, i + 1));
            }
            if (any) {
                words += output.separator;
            }
            words += *text;
            any = true;
        }
        if (!any) {
            return JS_UNDEFINED; // nothing to write: no block, no line number
        }
        std::string& sink = output.sink();
        if (skippable) {
            sink += '/';
        }
        if (output.numbered) {
            sink += fmt::format("N{}", output.next);
            sink += output.separator;
            output.next += output.step;
            if (output.next > output.max) {
                output.next = output.start;
            }
        }
        sink += words;
        sink += '\n';
        return JS_UNDEFINED;
    });
}

JSValue jsBlock(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    return writeBlock(ctx, argc, argv, "out.block", false);
}

JSValue jsSkippable(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    return writeBlock(ctx, argc, argv, "out.skippable", true);
}

JSValue jsLine(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        if (argc < 1 || !JS_IsString(argv[0])) {
            return typeError(ctx, "out.line: expected text");
        }
        const auto line = toText(ctx, argv[0]);
        if (!line) {
            return JS_EXCEPTION;
        }
        if (hasLineBreak(*line)) {
            return typeError(ctx, "out.line: the text has a line break");
        }
        std::string& sink = outputOf(ctx).sink();
        sink += *line;
        sink += '\n';
        return JS_UNDEFINED;
    });
}

JSValue jsComment(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        Output& output = outputOf(ctx);
        const auto text = argc > 0 ? toText(ctx, argv[0]) : std::optional<std::string>("");
        if (!text) {
            return JS_EXCEPTION;
        }
        const std::string clean = cleanComment(*text, output.upperComments);
        switch (output.comments) {
        case CommentStyle::Parentheses:
            output.sink() += "(" + clean + ")\n";
            break;
        case CommentStyle::Semicolon:
            output.sink() += "; " + clean + "\n";
            break;
        case CommentStyle::None:
            break;
        }
        return JS_UNDEFINED;
    });
}

JSValue jsCommentText(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        const auto text = argc > 0 ? toText(ctx, argv[0]) : std::optional<std::string>("");
        if (!text) {
            return JS_EXCEPTION;
        }
        const std::string clean = cleanComment(*text, outputOf(ctx).upperComments);
        return JS_NewStringLen(ctx, clean.data(), clean.size());
    });
}

JSValue jsNumbering(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        constexpr const char* what = "out.numbering";
        Output& output = outputOf(ctx);
        const JSValueConst arg = argc > 0 ? argv[0] : JS_UNDEFINED;
        if (JS_IsUndefined(arg) || JS_IsNull(arg) || (JS_IsBool(arg) && !JS_ToBool(ctx, arg))) {
            output.numbered = false;
            return JS_UNDEFINED;
        }
        if (!JS_IsObject(arg)) {
            return typeError(ctx, "out.numbering: expected settings such as { start: 10, step: "
                                  "10, max: 99999 }, or null to turn numbering off");
        }
        std::optional<double> start;
        std::optional<double> step;
        std::optional<double> max;
        if (!checkKeys(ctx, arg, what, {"start", "step", "max"}) ||
            !readNumber(ctx, arg, what, "start", start) ||
            !readNumber(ctx, arg, what, "step", step) || !readNumber(ctx, arg, what, "max", max)) {
            return JS_EXCEPTION;
        }
        // The usual numbering when a setting is left out: N10, N20, ... up to N99999.
        const double s = start.value_or(10.0);
        const double i = step.value_or(10.0);
        const double m = max.value_or(99999.0);
        constexpr double kLargest = 999999999.0;
        if (!isWhole(s) || !isWhole(i) || !isWhole(m) || s < 0.0 || i < 1.0 || m < s ||
            m > kLargest) {
            return typeError(ctx, "out.numbering: 'start', 'step' and 'max' must be whole "
                                  "numbers with 0 <= start <= max <= 999999999 and step >= 1");
        }
        output.numbered = true;
        output.start = static_cast<std::int64_t>(s);
        output.step = static_cast<std::int64_t>(i);
        output.max = static_cast<std::int64_t>(m);
        output.next = output.start;
        return JS_UNDEFINED;
    });
}

JSValue jsBuffer(JSContext* ctx, JSValueConst, int, JSValueConst*) {
    return guarded(ctx, [&]() -> JSValue {
        return newObject(ctx, outputOf(ctx).bufferClass, std::make_unique<BufferState>());
    });
}

JSValue jsBufferText(JSContext* ctx, JSValueConst thisValue, int, JSValueConst*) {
    const BufferState* buffer = self<BufferState>(ctx, thisValue, outputOf(ctx).bufferClass);
    if (!buffer) {
        return JS_EXCEPTION;
    }
    return JS_NewStringLen(ctx, buffer->text.data(), buffer->text.size());
}

JSValue jsInto(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    Output& output = outputOf(ctx);
    BufferState* buffer =
        self<BufferState>(ctx, argc > 0 ? argv[0] : JS_UNDEFINED, output.bufferClass);
    if (!buffer) {
        return JS_EXCEPTION;
    }
    if (argc < 2 || !JS_IsFunction(ctx, argv[1])) {
        return JS_ThrowTypeError(ctx, "out.into: expected a buffer and a function");
    }
    // The buffer stays alive while the function runs: argv holds it.
    std::string* saved = output.target;
    output.target = &buffer->text;
    const JSValue result = JS_Call(ctx, argv[1], JS_UNDEFINED, 0, nullptr);
    output.target = saved;
    return result;
}

JSValue jsAppend(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        Output& output = outputOf(ctx);
        const BufferState* buffer =
            self<BufferState>(ctx, argc > 0 ? argv[0] : JS_UNDEFINED, output.bufferClass);
        if (!buffer) {
            return JS_EXCEPTION;
        }
        if (&buffer->text == &output.sink()) {
            return typeError(ctx, "out.append: a buffer cannot be appended to itself");
        }
        output.sink() += buffer->text;
        return JS_UNDEFINED;
    });
}

JSValue jsGetSeparator(JSContext* ctx, JSValueConst, int, JSValueConst*) {
    const std::string& separator = outputOf(ctx).separator;
    return JS_NewStringLen(ctx, separator.data(), separator.size());
}

JSValue jsSetSeparator(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        if (argc < 1 || !JS_IsString(argv[0])) {
            return typeError(ctx, "out.separator must be text");
        }
        auto text = toText(ctx, argv[0]);
        if (!text) {
            return JS_EXCEPTION;
        }
        if (hasLineBreak(*text)) {
            return typeError(ctx, "out.separator must not have a line break");
        }
        outputOf(ctx).separator = std::move(*text);
        return JS_UNDEFINED;
    });
}

JSValue jsGetCommentStyle(JSContext* ctx, JSValueConst, int, JSValueConst*) {
    switch (outputOf(ctx).comments) {
    case CommentStyle::Semicolon:
        return JS_NewString(ctx, "semicolon");
    case CommentStyle::None:
        return JS_NewString(ctx, "none");
    case CommentStyle::Parentheses:
        break;
    }
    return JS_NewString(ctx, "parentheses");
}

JSValue jsSetCommentStyle(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        const auto style =
            argc > 0 && JS_IsString(argv[0]) ? toText(ctx, argv[0]) : std::optional<std::string>();
        Output& output = outputOf(ctx);
        if (style == "parentheses") {
            output.comments = CommentStyle::Parentheses;
        } else if (style == "semicolon") {
            output.comments = CommentStyle::Semicolon;
        } else if (style == "none") {
            output.comments = CommentStyle::None;
        } else {
            return typeError(ctx, "out.commentStyle must be \"parentheses\", \"semicolon\" or "
                                  "\"none\"");
        }
        return JS_UNDEFINED;
    });
}

JSValue jsGetUpperComments(JSContext* ctx, JSValueConst, int, JSValueConst*) {
    return JS_NewBool(ctx, outputOf(ctx).upperComments);
}

JSValue jsSetUpperComments(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    if (argc < 1 || !JS_IsBool(argv[0])) {
        return JS_ThrowTypeError(ctx, "out.upperComments must be true or false");
    }
    outputOf(ctx).upperComments = JS_ToBool(ctx, argv[0]) > 0;
    return JS_UNDEFINED;
}

// -- warning() and error() ------------------------------------------------------------------

JSValue jsWarning(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        const auto message = argc > 0 ? toText(ctx, argv[0]) : std::optional<std::string>("");
        if (!message) {
            return JS_EXCEPTION;
        }
        outputOf(ctx).warnings.push_back(*message);
        return JS_UNDEFINED;
    });
}

JSValue jsError(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
    return guarded(ctx, [&]() -> JSValue {
        const auto message =
            argc > 0 ? toText(ctx, argv[0]) : std::optional<std::string>("the post stopped");
        if (!message) {
            return JS_EXCEPTION;
        }
        const JSValue error = JS_NewError(ctx);
        if (JS_IsException(error)) {
            return error;
        }
        JS_SetPropertyStr(ctx, error, "message",
                          JS_NewStringLen(ctx, message->data(), message->size()));
        JS_SetPropertyStr(ctx, error, kPostErrorTag, JS_TRUE);
        return JS_Throw(ctx, error);
    });
}

// -- Registration ---------------------------------------------------------------------------

struct Method {
    const char* name;
    JSCFunction* function;
    int length;
};

struct Accessor {
    const char* name;
    JSCFunction* getter;
    JSCFunction* setter; ///< nullptr: read-only
};

bool defineMethods(JSContext* ctx, JSValueConst object, std::initializer_list<Method> methods) {
    for (const Method& m : methods) {
        if (JS_SetPropertyStr(ctx, object, m.name,
                              JS_NewCFunction(ctx, m.function, m.name, m.length)) < 0) {
            return false;
        }
    }
    return true;
}

bool defineAccessors(JSContext* ctx, JSValueConst object,
                     std::initializer_list<Accessor> accessors) {
    for (const Accessor& a : accessors) {
        const JSAtom atom = JS_NewAtom(ctx, a.name);
        if (atom == JS_ATOM_NULL) {
            return false;
        }
        const JSValue getter = JS_NewCFunction(ctx, a.getter, a.name, 0);
        const JSValue setter = a.setter ? JS_NewCFunction(ctx, a.setter, a.name, 1) : JS_UNDEFINED;
        const int status = JS_DefinePropertyGetSet(ctx, object, atom, getter, setter,
                                                   JS_PROP_CONFIGURABLE | JS_PROP_ENUMERABLE);
        JS_FreeAtom(ctx, atom);
        if (status < 0) {
            return false;
        }
    }
    return true;
}

template <class T>
bool defineClass(JSContext* ctx, JSClassID& id, const char* name,
                 std::initializer_list<Method> methods, std::initializer_list<Accessor> accessors) {
    JSRuntime* rt = JS_GetRuntime(ctx);
    JS_NewClassID(rt, &id);
    JSClassDef def{};
    def.class_name = name;
    def.finalizer = &finalize<T>;
    if (JS_NewClass(rt, id, &def) < 0) {
        return false;
    }
    const JSValue proto = JS_NewObject(ctx);
    if (JS_IsException(proto)) {
        return false;
    }
    if (!defineMethods(ctx, proto, methods) || !defineAccessors(ctx, proto, accessors)) {
        JS_FreeValue(ctx, proto);
        return false;
    }
    JS_SetClassProto(ctx, id, proto);
    return true;
}

int initPostModule(JSContext* ctx, JSModuleDef* module) {
    const JSValue out = outObject(ctx);
    if (JS_IsException(out) || JS_SetModuleExport(ctx, module, "out", out) < 0) {
        return -1;
    }
    const Method functions[] = {{"number", &jsNumber, 1},
                                {"word", &jsWord, 3},
                                {"code", &jsCode, 0},
                                {"warning", &jsWarning, 1},
                                {"error", &jsError, 1}};
    for (const Method& f : functions) {
        if (JS_SetModuleExport(ctx, module, f.name,
                               JS_NewCFunction(ctx, f.function, f.name, f.length)) < 0) {
            return -1;
        }
    }
    return 0;
}

} // namespace

bool registerOutputLibrary(JSContext* ctx, Output& output) {
    return defineClass<NumberSpec>(ctx, output.formatClass, "NumberFormat",
                                   {{"format", &jsFormat, 1}}, {}) &&
           defineClass<WordState>(
               ctx, output.wordClass, "Word", {{"of", &jsWordOf, 1}, {"reset", &jsWordReset, 0}},
               {{"last", &jsWordLast, nullptr}, {"letter", &jsWordLetter, nullptr}}) &&
           defineClass<CodeState>(ctx, output.codeClass, "Code",
                                  {{"of", &jsCodeOf, 1}, {"reset", &jsCodeReset, 0}},
                                  {{"last", &jsCodeLast, nullptr}}) &&
           defineClass<BufferState>(ctx, output.bufferClass, "Buffer", {},
                                    {{"text", &jsBufferText, nullptr}});
}

void releaseOutputLibrary(JSContext* ctx, Output& output) {
    JS_FreeValue(ctx, output.out);
    output.out = JS_UNDEFINED;
}

JSValue outObject(JSContext* ctx) {
    Output& output = outputOf(ctx);
    if (JS_IsUndefined(output.out)) {
        const JSValue object = JS_NewObject(ctx);
        if (JS_IsException(object)) {
            return object;
        }
        if (!defineMethods(ctx, object,
                           {{"block", &jsBlock, 0},
                            {"skippable", &jsSkippable, 0},
                            {"line", &jsLine, 1},
                            {"comment", &jsComment, 1},
                            {"commentText", &jsCommentText, 1},
                            {"numbering", &jsNumbering, 1},
                            {"buffer", &jsBuffer, 0},
                            {"into", &jsInto, 2},
                            {"append", &jsAppend, 1}}) ||
            !defineAccessors(ctx, object,
                             {{"separator", &jsGetSeparator, &jsSetSeparator},
                              {"commentStyle", &jsGetCommentStyle, &jsSetCommentStyle},
                              {"upperComments", &jsGetUpperComments, &jsSetUpperComments}})) {
            JS_FreeValue(ctx, object);
            return JS_EXCEPTION;
        }
        output.out = object;
    }
    return JS_DupValue(ctx, output.out);
}

JSModuleDef* createPostModule(JSContext* ctx) {
    JSModuleDef* module = JS_NewCModule(ctx, "oxcam:post", &initPostModule);
    if (!module) {
        return nullptr;
    }
    for (const char* name : {"out", "number", "word", "code", "warning", "error"}) {
        if (JS_AddModuleExport(ctx, module, name) < 0) {
            return nullptr;
        }
    }
    return module;
}

} // namespace ocam::post::detail
