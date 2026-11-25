// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// libFuzzer target: .ocam archive container reader.

#include <ocam/document/ProjectIo.hpp>
#include <ocam/foundation/Error.hpp>

#include <cstddef>
#include <cstdint>
#include <string_view>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
    try {
        const std::string_view bytes(reinterpret_cast<const char*>(data), size);
        (void)ocam::document::readArchive(bytes, "fuzz.ocam");
    } catch (const ocam::Error&) {
    }
    return 0;
}
