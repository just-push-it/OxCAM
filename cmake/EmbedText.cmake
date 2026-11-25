# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Script mode: generates a C++ source that embeds a text file as a std::string_view.
#   cmake -DINPUT=<file> -DOUTPUT=<file.cpp> -DSYMBOL=<name> -DNAMESPACE=<ns> -P EmbedText.cmake
# Bytes are written as a hex array, which avoids MSVC's string-literal length limit.

cmake_minimum_required(VERSION 3.25)

file(READ "${INPUT}" hex HEX)
string(LENGTH "${hex}" hexLength)
math(EXPR size "${hexLength} / 2")
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "'\\\\x\\1'," bytes "${hex}")
# Break the initializer into lines of 16 bytes for readable diffs of generated files.
string(REGEX REPLACE "(('\\\\x[0-9a-f][0-9a-f]',){16})" "\\1\n    " bytes "${bytes}")

file(WRITE "${OUTPUT}" "// Generated from ${INPUT}. Do not edit.
#include <string_view>

namespace ${NAMESPACE} {
namespace {
constexpr char kData[${size} + 1] = {
    ${bytes}'\\0'};
} // namespace

extern const std::string_view ${SYMBOL}{kData, ${size}};
} // namespace ${NAMESPACE}
")
