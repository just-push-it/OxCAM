// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "ocam/foundation/Math.hpp"

#include <algorithm>
#include <cmath>

namespace ocam {

/// Geometric tolerances in internal units (mm, rad).
struct Tolerance {
    double linear = 1e-6;  ///< Points closer than this are coincident [mm].
    double angular = 1e-9; ///< Directions closer than this are parallel [rad].
};

/// True when |a - b| <= max(absTol, relTol * max(|a|, |b|)).
constexpr bool nearlyEqual(double a, double b, double absTol, double relTol = 0.0) noexcept {
    const double diff = a > b ? a - b : b - a;
    const double scale = std::max(a < 0 ? -a : a, b < 0 ? -b : b);
    return diff <= std::max(absTol, relTol * scale);
}

/// True when the points are within `tol` of each other (Euclidean distance).
inline bool nearlyEqual(const Vec3& a, const Vec3& b, double tol) noexcept {
    return (a - b).squaredNorm() <= tol * tol;
}

inline bool nearlyEqual(const Vec2& a, const Vec2& b, double tol) noexcept {
    return (a - b).squaredNorm() <= tol * tol;
}

/// True when unit vectors `a` and `b` point the same way within `angTol` radians.
inline bool nearlyParallel(const Vec3& a, const Vec3& b, double angTol) noexcept {
    return std::atan2(a.cross(b).norm(), a.dot(b)) <= angTol;
}

} // namespace ocam
