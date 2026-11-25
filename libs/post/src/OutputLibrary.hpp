// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The oxcam:post module of the post engine (DL-0039): number formats, modal words and codes, the
// block, comment and line writers, line numbering, buffers, warning() and error(). Internal to
// libs/post; Engine.cpp creates one Output per run.

#pragma once

#include <quickjs.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ocam::post::detail {

/// Property that marks the errors a post raises with error(): reported without a location.
inline constexpr const char* kPostErrorTag = "oxcamPostError";

/// Number format of number(): the C++ formatter's rules (DL-0011), so a post can reproduce it.
struct NumberSpec {
    int decimals = 0;
    bool point = true;      ///< whole numbers end in "." ("10.")
    bool keepZeros = false; ///< "10.500" instead of "10.5"
    double scale = 1.0;     ///< applied before rounding
};

/// Rounds half away from zero to the decimals, never writes "-0".
std::string formatNumber(double value, const NumberSpec& spec);

enum class CommentStyle { Parentheses, Semicolon, None };

/// What a run has written and the output settings. Lives in the run's sandbox (not movable: the
/// output target may point into a buffer).
struct Output {
    std::string text;
    std::vector<std::string> warnings;
    /// Where blocks go: a buffer's text while out.into() runs, else `text`.
    std::string* target = nullptr;

    bool numbered = false;
    std::int64_t next = 0;
    std::int64_t start = 10;
    std::int64_t step = 10;
    std::int64_t max = 99999;

    std::string separator = " ";
    CommentStyle comments = CommentStyle::Parentheses;
    bool upperComments = false;

    JSClassID formatClass = 0;
    JSClassID wordClass = 0;
    JSClassID codeClass = 0;
    JSClassID bufferClass = 0;
    JSValue out = JS_UNDEFINED; ///< The `out` object, created on first use.

    std::string& sink() { return target ? *target : text; }
};

/// The run's Output (defined by the engine, which keeps it in the context's opaque data).
Output& outputOf(JSContext* ctx);

/// Registers the library's classes in a new context. False on failure (exception pending).
bool registerOutputLibrary(JSContext* ctx, Output& output);
/// Frees what the library holds in the context; call before the context is freed.
void releaseOutputLibrary(JSContext* ctx, Output& output);

/// The oxcam:post module.
JSModuleDef* createPostModule(JSContext* ctx);
/// A new reference to the `out` object (JS_EXCEPTION on failure).
JSValue outObject(JSContext* ctx);

/// Why `text` is not plain printable ASCII lines ("line 3: ..."), or nullopt.
std::optional<std::string> asciiProblem(const std::string& text);

std::optional<std::string> toText(JSContext* ctx, JSValueConst value);

} // namespace ocam::post::detail
