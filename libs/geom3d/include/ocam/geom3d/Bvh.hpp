// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Bounding volume hierarchy over the triangles of a mesh: box queries and ray casting.
// Median split on the longest axis of the centroid bounds (phase 9). See docs/design-log/0025.

#pragma once

#include "ocam/geom3d/Mesh.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace ocam::geom3d {

struct RayHit {
    double t = 0.0; ///< Distance along the (unit) ray direction.
    std::size_t triangle = 0;
    Vec3 point = Vec3::Zero();
};

class TriangleBvh {
public:
    TriangleBvh() = default;
    /// Copies the mesh. Degenerate triangles are kept (they never report ray hits).
    explicit TriangleBvh(Mesh mesh);

    const Mesh& mesh() const noexcept { return mesh_; }
    bool isEmpty() const noexcept { return nodes_.empty(); }
    Aabb3 bounds() const noexcept { return nodes_.empty() ? Aabb3{} : nodes_.front().box; }

    /// Calls `visit(triangle)` for every triangle whose bounding box overlaps `box`; stops early
    /// when `visit` returns false. Returns false when stopped.
    bool query(const Aabb3& box, const std::function<bool(std::size_t)>& visit) const;

    /// Calls `visit(mine, theirs)` for every pair of a triangle of this hierarchy and one of
    /// `other`, placed in this hierarchy's frame by `otherToThis`, whose bounding boxes come
    /// within `margin` of each other; stops early when `visit` returns false. Returns false when
    /// stopped. Used for mesh-to-mesh collision checks (DL-0037).
    bool queryPairs(const TriangleBvh& other, const Transform3& otherToThis, double margin,
                    const std::function<bool(std::size_t, std::size_t)>& visit) const;

    /// Nearest intersection of the ray origin + t * direction (direction normalized here) with
    /// t in [0, tMax].
    std::optional<RayHit> raycast(const Vec3& origin, const Vec3& direction,
                                  double tMax = std::numeric_limits<double>::infinity()) const;

    /// The three corners of a triangle.
    std::array<Vec3, 3> triangle(std::size_t t) const;

private:
    struct Node {
        Aabb3 box;
        std::uint32_t first = 0; ///< Leaf: first index into order_; inner: left child.
        std::uint32_t count = 0; ///< Leaf: triangle count; 0 for inner nodes.
        std::uint32_t right = 0; ///< Inner: right child.
    };
    std::uint32_t build(std::uint32_t first, std::uint32_t count);

    Mesh mesh_;
    std::vector<Aabb3> boxes_;
    std::vector<Vec3> centroids_;
    std::vector<std::uint32_t> order_;
    std::vector<Node> nodes_;
};

/// Bounding box of a box moved by a rigid transform.
Aabb3 transformed(const Aabb3& box, const Transform3& xf) noexcept;

/// True when the two boxes overlap (touching counts).
bool overlaps(const Aabb3& a, const Aabb3& b) noexcept;

} // namespace ocam::geom3d
