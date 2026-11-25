// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Convex solids given by their support mappings, and the distance between two of them by the
// Gilbert-Johnson-Keerthi algorithm (phase 9: machine collision checks and tool clearance).
// See docs/design-log/0025.

#pragma once

#include "ocam/geom3d/Bounds.hpp"

#include <vector>

namespace ocam::geom3d {

/// A convex solid: the convex hull of points, or a finite cylinder, optionally rounded by
/// `margin` (the Minkowski sum with a ball of that radius: segments become capsules).
class Convex {
public:
    Convex() = default;

    static Convex points(std::vector<Vec3> pts, double margin = 0.0);
    static Convex box(const Aabb3& box);
    /// Box with the given centre, axes (columns, orthonormal) and half extents.
    static Convex orientedBox(const Vec3& centre, const Mat3& axes, const Vec3& halfExtents);
    /// Solid cylinder from `base` along the unit `axis` for `length`.
    static Convex cylinder(const Vec3& base, const Vec3& axis, double radius, double length);
    /// Capsule: all points within `radius` of the segment a-b.
    static Convex capsule(const Vec3& a, const Vec3& b, double radius);
    static Convex triangle(const Vec3& a, const Vec3& b, const Vec3& c);
    static Convex sphere(const Vec3& centre, double radius);

    /// Farthest point of the solid without its margin in direction d.
    Vec3 support(const Vec3& d) const;
    double margin() const noexcept { return margin_; }
    Aabb3 bounds() const;
    /// The solid moved by a rigid transform.
    Convex transformed(const Transform3& xf) const;
    bool isEmpty() const noexcept { return kind_ == Kind::Points && points_.empty(); }

private:
    enum class Kind { Points, Cylinder };
    Kind kind_ = Kind::Points;
    std::vector<Vec3> points_;
    Vec3 base_ = Vec3::Zero();
    Vec3 axis_ = Vec3::UnitZ();
    double radius_ = 0.0;
    double length_ = 0.0;
    double margin_ = 0.0;
};

struct ConvexDistance {
    double distance = 0.0; ///< 0 when the solids overlap.
    bool intersecting = false;
    Vec3 pointA = Vec3::Zero(); ///< Closest points (a common point when overlapping).
    Vec3 pointB = Vec3::Zero();
};

/// Euclidean distance between two convex solids.
ConvexDistance distance(const Convex& a, const Convex& b);

/// True when the solids come closer than `clearance` (overlap for 0).
bool intersects(const Convex& a, const Convex& b, double clearance = 0.0);

} // namespace ocam::geom3d
