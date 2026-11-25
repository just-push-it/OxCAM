// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Contacts between machine parts, tools and work, shared by the mill's and the lathe's machine
// checks (DL-0037, DL-0067): convex solids against triangle hierarchies and hierarchies against
// each other, within a clearance.

#pragma once

#include <ocam/geom3d/Bvh.hpp>
#include <ocam/geom3d/Convex.hpp>

#include <array>
#include <limits>
#include <optional>

namespace ocam::sim::detail {

/// Closest contact found between two colliders within `clearance`, in the model frame.
struct Contact {
    double distance = 0.0;
    Vec3 point = Vec3::Zero();
};

/// True when the plane of triangle (a, b, c) keeps all of `points` farther than `clearance` on
/// one side: a cheap exact rejection before GJK (flat faces of machine parts are fans of long
/// triangles whose boxes all reach the same spots).
inline bool planeSeparates(const Vec3& a, const Vec3& b, const Vec3& c,
                           const std::array<Vec3, 3>& points, double clearance) {
    const Vec3 n = (b - a).cross(c - a);
    const double length = n.norm();
    if (length < 1e-12) {
        return false;
    }
    double lo = std::numeric_limits<double>::infinity();
    double hi = -lo;
    for (const Vec3& p : points) {
        const double s = n.dot(p - a) / length;
        lo = std::min(lo, s);
        hi = std::max(hi, s);
    }
    return lo > clearance || hi < -clearance;
}

/// The same for a convex solid, through its support points.
inline bool planeSeparates(const Vec3& a, const Vec3& b, const Vec3& c, const geom3d::Convex& solid,
                           double clearance) {
    Vec3 n = (b - a).cross(c - a);
    if (n.norm() < 1e-12) {
        return false;
    }
    n.normalize();
    const double hi = n.dot(solid.support(n) - a) + solid.margin();
    const double lo = n.dot(solid.support(-n) - a) - solid.margin();
    return lo > clearance || hi < -clearance;
}

inline std::optional<Contact> convexToMesh(const geom3d::Convex& c, const geom3d::TriangleBvh& mesh,
                                           const Transform3& meshToWorld, double clearance) {
    const Transform3 toMesh = meshToWorld.inverse();
    const geom3d::Convex local = c.transformed(toMesh);
    geom3d::Aabb3 box = local.bounds();
    box.min -= Vec3::Constant(clearance);
    box.max += Vec3::Constant(clearance);
    std::optional<Contact> hit;
    mesh.query(box, [&](std::size_t t) {
        const auto [a, b, d] = mesh.triangle(t);
        if (planeSeparates(a, b, d, local, clearance)) {
            return true;
        }
        const geom3d::ConvexDistance r = geom3d::distance(local, geom3d::Convex::triangle(a, b, d));
        if (r.intersecting || r.distance < clearance) {
            hit = Contact{r.distance, meshToWorld * r.pointA};
            return false;
        }
        return true;
    });
    return hit;
}

inline std::optional<Contact> meshToMesh(const geom3d::TriangleBvh& a, const Transform3& aToWorld,
                                         const geom3d::TriangleBvh& b, const Transform3& bToWorld,
                                         double clearance) {
    const Transform3 bToA = aToWorld.inverse() * bToWorld;
    std::optional<Contact> hit;
    a.queryPairs(b, bToA, clearance, [&](std::size_t i, std::size_t j) {
        const std::array<Vec3, 3> mine = a.triangle(i);
        const std::array<Vec3, 3> theirs = b.triangle(j);
        const std::array<Vec3, 3> moved{bToA * theirs[0], bToA * theirs[1], bToA * theirs[2]};
        if (planeSeparates(mine[0], mine[1], mine[2], moved, clearance) ||
            planeSeparates(moved[0], moved[1], moved[2], mine, clearance)) {
            return true;
        }
        const geom3d::ConvexDistance d =
            geom3d::distance(geom3d::Convex::triangle(mine[0], mine[1], mine[2]),
                             geom3d::Convex::triangle(moved[0], moved[1], moved[2]));
        if (d.intersecting || d.distance < clearance) {
            hit = Contact{d.distance, aToWorld * d.pointA};
            return false;
        }
        return true;
    });
    return hit;
}

} // namespace ocam::sim::detail
