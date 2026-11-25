// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/foundation/Units.hpp"

#include <algorithm>
#include <array>
#include <cctype>

namespace ocam {
namespace {

bool equalsIgnoreCase(std::string_view a, std::string_view b) noexcept {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](char x, char y) {
               return std::tolower(static_cast<unsigned char>(x)) ==
                      std::tolower(static_cast<unsigned char>(y));
           });
}

} // namespace

std::optional<LengthUnit> parseLengthUnit(std::string_view text) noexcept {
    constexpr std::array<std::string_view, 5> mm{"mm", "millimeter", "millimeters", "millimetre",
                                                 "millimetres"};
    constexpr std::array<std::string_view, 3> in{"in", "inch", "inches"};

    if (std::any_of(mm.begin(), mm.end(), [&](auto s) { return equalsIgnoreCase(text, s); })) {
        return LengthUnit::Millimeter;
    }
    if (std::any_of(in.begin(), in.end(), [&](auto s) { return equalsIgnoreCase(text, s); })) {
        return LengthUnit::Inch;
    }
    return std::nullopt;
}

std::string_view toString(LengthUnit unit) noexcept {
    return unit == LengthUnit::Inch ? "in" : "mm";
}

} // namespace ocam
