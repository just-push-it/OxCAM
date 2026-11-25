// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <ocam/foundation/Math.hpp>

#include <limits>

namespace ocam::geom2d {

/// Axis-aligned 2D box. Default-constructed boxes are empty.
struct Aabb2 {
    Vec2 min{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    Vec2 max{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};

    bool isEmpty() const noexcept { return min.x() > max.x() || min.y() > max.y(); }

    void extend(const Vec2& p) noexcept {
        min = min.cwiseMin(p);
        max = max.cwiseMax(p);
    }

    void extend(const Aabb2& other) noexcept {
        if (!other.isEmpty()) {
            extend(other.min);
            extend(other.max);
        }
    }

    Vec2 size() const noexcept { return isEmpty() ? Vec2::Zero() : Vec2(max - min); }
    Vec2 center() const noexcept { return (min + max) * 0.5; }

    bool operator==(const Aabb2&) const = default;

    bool contains(const Vec2& p, double tol = 0.0) const noexcept {
        return p.x() >= min.x() - tol && p.x() <= max.x() + tol && p.y() >= min.y() - tol &&
               p.y() <= max.y() + tol;
    }
};

} // namespace ocam::geom2d
