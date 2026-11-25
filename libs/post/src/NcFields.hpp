// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The fields of the NC program's JSON form (the post API, DL-0015 and DL-0039), written once:
// post::toJson builds JSON from them and the post engine builds JavaScript objects from them
// directly, so `--ir` files show exactly what a post receives. Internal to libs/post.

#pragma once

#include <ocam/foundation/Json.hpp>
#include <ocam/post/Post.hpp>

#include <string_view>

namespace ocam::post::detail {

/// Receives the fields of one instruction, in order. Keys are string literals.
class FieldSink {
public:
    virtual void number(const char* key, double value) = 0;
    virtual void integer(const char* key, int value) = 0;
    virtual void boolean(const char* key, bool value) = 0;
    virtual void text(const char* key, std::string_view value) = 0;
    virtual void null(const char* key) = 0;
    /// Fields up to endObject() belong to an object under `key`; in an array (beginArray), `key`
    /// is null and the object is the array's next element.
    virtual void beginObject(const char* key) = 0;
    virtual void endObject() = 0;
    /// An array under `key`, of the objects begun up to endArray().
    virtual void beginArray(const char* key) = 0;
    virtual void endArray() = 0;

protected:
    ~FieldSink() = default;
};

/// The fields of an instruction in its JSON form, "op" first.
void instructionFields(const NcInstr& instruction, FieldSink& sink);

/// The JSON form of a program without its instructions.
Json programHeaderJson(const NcProgram& program);

} // namespace ocam::post::detail
