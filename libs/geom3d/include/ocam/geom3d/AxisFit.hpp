// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Axes of round shapes found on triangle meshes (DL-0037: a machine's rotary axis picked from
// its rotary table): the axis of a cylindrical or conical face around a picked triangle, and
// the circle through three points of a rim.

#pragma once

#include "ocam/geom3d/Mesh.hpp"

#include <optional>

namespace ocam::geom3d {

/// A line: the axis of a round face or circle.
struct AxisLine {
    Vec3 point = Vec3::Zero();      ///< On the axis (the face's or circle's centre).
    Vec3 direction = Vec3::UnitZ(); ///< Unit vector.
    double radius = 0.0;            ///< Mean radius of the face or circle [mm].
    std::size_t triangles = 0;      ///< Triangles of the face (0 for a circle).
};

/// The axis of the smooth surface of revolution (cylinder, cone, torus band) around triangle
/// `seed`: the region of triangles reached across edges that bend less than `maxBend` [rad].
/// Its normals all meet the axis: the direction is the one the normals vary least along (the
/// smallest principal direction of their spread) and the point the least-squares meeting point
/// of the normal lines through the facets. Nullopt for flat regions (the normals barely vary)
/// or welded-vertex-less meshes. The point lies at the face's mean height along the axis.
std::optional<AxisLine> revolutionAxis(const Mesh& mesh, std::size_t seed, double maxBend = 0.6);

/// A flat face: the area-weighted centre of the coplanar region around a triangle (edges that
/// bend less than `maxBend` [rad]) and its normal.
struct FlatFace {
    Vec3 centre = Vec3::Zero();
    Vec3 normal = Vec3::UnitZ();
    double area = 0.0;
};
std::optional<FlatFace> flatFace(const Mesh& mesh, std::size_t seed, double maxBend = 0.02);

/// The triangle nearest to `p` (within `tolerance`), e.g. under a point picked on the mesh.
std::optional<std::size_t> nearestTriangle(const Mesh& mesh, const Vec3& p, double tolerance);

/// The circle through three points (centre, plane normal by the right-hand rule a -> b -> c,
/// radius); nullopt when they are (nearly) collinear.
std::optional<AxisLine> circleThrough(const Vec3& a, const Vec3& b, const Vec3& c);

} // namespace ocam::geom3d
