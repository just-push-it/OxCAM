// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "ocam/geom3d/Bounds.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace ocam::geom3d {

/// Indexed triangle mesh. Triangles are counter-clockwise seen from outside (outward normals).
/// Used for part comparison, simulation oracles and later for 3D toolpath kernels.
struct Mesh {
    using Triangle = std::array<std::uint32_t, 3>;

    std::vector<Vec3> vertices;
    std::vector<Triangle> triangles;

    bool isEmpty() const noexcept { return triangles.empty(); }
    Aabb3 bounds() const noexcept;
    double surfaceArea() const noexcept;

    /// Signed volume by the divergence theorem. Equals the enclosed volume for closed,
    /// outward-oriented meshes.
    double signedVolume() const noexcept;

    /// True when every undirected edge is used by exactly two triangles in opposite directions
    /// (closed, consistently oriented 2-manifold). Requires welded vertices.
    bool isClosed() const;

    Vec3 triangleNormal(std::size_t t) const noexcept; ///< Unit normal (zero if degenerate).

    /// Appends another mesh, offsetting its indices.
    void append(const Mesh& other);

    void transform(const Transform3& xf);
};

/// Merges vertices closer than `tolerance` and drops triangles that become degenerate.
/// Deterministic: the first vertex of each cluster (in input order) is kept.
Mesh weldVertices(const Mesh& mesh, double tolerance);

} // namespace ocam::geom3d
