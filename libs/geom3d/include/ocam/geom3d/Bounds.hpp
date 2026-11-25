// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <ocam/foundation/Math.hpp>

#include <limits>

namespace ocam::geom3d {

/// Axis-aligned 3D box. Default-constructed boxes are empty.
struct Aabb3 {
    Vec3 min = Vec3::Constant(std::numeric_limits<double>::infinity());
    Vec3 max = Vec3::Constant(-std::numeric_limits<double>::infinity());

    bool isEmpty() const noexcept {
        return min.x() > max.x() || min.y() > max.y() || min.z() > max.z();
    }

    void extend(const Vec3& p) noexcept {
        min = min.cwiseMin(p);
        max = max.cwiseMax(p);
    }

    void extend(const Aabb3& other) noexcept {
        if (!other.isEmpty()) {
            extend(other.min);
            extend(other.max);
        }
    }

    Vec3 size() const noexcept { return isEmpty() ? Vec3::Zero() : Vec3(max - min); }
    Vec3 center() const noexcept { return (min + max) * 0.5; }

    bool operator==(const Aabb3&) const = default;

    bool contains(const Vec3& p, double tol = 0.0) const noexcept {
        return (p.array() >= min.array() - tol).all() && (p.array() <= max.array() + tol).all();
    }
};

} // namespace ocam::geom3d
