// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Internal units are millimetres, radians and seconds everywhere in the libraries. Convert only
// at I/O and UI boundaries, using the helpers below.

#pragma once

#include "ocam/foundation/Math.hpp"

#include <optional>
#include <string_view>

namespace ocam {

enum class LengthUnit { Millimeter, Inch };

inline constexpr double kMmPerInch = 25.4;
inline constexpr double kSecondsPerMinute = 60.0;

/// Converts a length expressed in `unit` to millimetres.
constexpr double toMm(double value, LengthUnit unit) noexcept {
    return unit == LengthUnit::Inch ? value * kMmPerInch : value;
}

/// Converts a length in millimetres to `unit`.
constexpr double fromMm(double mm, LengthUnit unit) noexcept {
    return unit == LengthUnit::Inch ? mm / kMmPerInch : mm;
}

/// Converts a feed rate in `unit` per minute (the usual G-code convention) to mm/s.
constexpr double feedPerMinToMmPerSec(double value, LengthUnit unit) noexcept {
    return toMm(value, unit) / kSecondsPerMinute;
}

/// Converts a feed rate in mm/s to `unit` per minute.
constexpr double mmPerSecToFeedPerMin(double mmPerSec, LengthUnit unit) noexcept {
    return fromMm(mmPerSec * kSecondsPerMinute, unit);
}

constexpr double degToRad(double degrees) noexcept {
    return degrees * kPi / 180.0;
}
constexpr double radToDeg(double radians) noexcept {
    return radians * 180.0 / kPi;
}

/// Parses "mm", "millimeter(s)", "in", "inch(es)" (case-insensitive).
std::optional<LengthUnit> parseLengthUnit(std::string_view text) noexcept;

/// Canonical short name: "mm" or "in".
std::string_view toString(LengthUnit unit) noexcept;

} // namespace ocam
