// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cctype>
#include <charconv>
#include <cmath>
#include <locale>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>

namespace ocam {

/// The whole of `text`, with no surrounding whitespace, as a finite double.
/// libc++ ships `std::from_chars` for integers only (Xcode 16 on the macOS CI runner),
/// so that library parses with the C locale instead.
inline bool parseFiniteDouble(std::string_view text, double& value) {
    if (text.empty() || std::isspace(static_cast<unsigned char>(text.front()))) {
        return false;
    }
#if defined(_LIBCPP_VERSION)
    std::istringstream in{std::string(text)};
    in.imbue(std::locale::classic());
    in >> value;
    return static_cast<bool>(in) && in.eof() && std::isfinite(value);
#else
    const auto res = std::from_chars(text.data(), text.data() + text.size(), value);
    return res.ec == std::errc() && res.ptr == text.data() + text.size() && std::isfinite(value);
#endif
}

} // namespace ocam
