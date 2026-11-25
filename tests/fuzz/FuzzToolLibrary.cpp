// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// libFuzzer target: tool library JSON loader. Any exception other than ocam::Error escapes and is
// reported as a crash.

#include <ocam/foundation/Error.hpp>
#include <ocam/tooling/ToolLibrary.hpp>

#include <cstddef>
#include <cstdint>
#include <string_view>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
    try {
        const std::string_view text(reinterpret_cast<const char*>(data), size);
        (void)ocam::tooling::toolLibraryFromJson(ocam::parseJson(text, "fuzz"), "fuzz");
    } catch (const ocam::Error&) {
        // rejected input: fine
    }
    return 0;
}
