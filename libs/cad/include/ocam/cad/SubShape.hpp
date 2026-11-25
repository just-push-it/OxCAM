// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Persistent references to faces and edges. Index-based references break when a model is
// re-imported or edited upstream, so each reference also stores a geometric signature that is
// used to find the same sub-shape again.

#pragma once

#include "ocam/cad/Model.hpp"

#include <ocam/foundation/Math.hpp>

#include <optional>
#include <vector>

namespace ocam::cad {

enum class SubShapeType { Face, Edge, Vertex };

struct ShapeSignature {
    Vec3 centroid = Vec3::Zero(); ///< Centre of mass of the face or edge [mm].
    Vec3 direction =
        Vec3::Zero();     ///< Plane normal, cylinder/cone axis or line direction; zero otherwise.
    double measure = 0.0; ///< Area (faces) or length (edges).

    bool operator==(const ShapeSignature&) const = default;
};

/// The exact geometry of a face, edge or vertex, for picking on the B-rep (DL-0037).
struct SubShapeGeometry {
    enum class Kind { Plane, Cylinder, Cone, Sphere, Surface, Line, Circle, Curve, Point };
    Kind kind = Kind::Point;
    /// Plane and other surfaces: the face's area centroid. Cylinder, cone: the point of the axis
    /// nearest the face's centroid. Sphere: its centre. Line: the edge's middle. Circle (or arc):
    /// its centre. Other curves: the edge's centre of mass. Point: the vertex.
    Vec3 point = Vec3::Zero();
    /// Plane: the outward normal. Cylinder, cone: the axis (unit, either sense). Line: its
    /// direction. Circle: the normal of its plane (its axis). Zero otherwise.
    Vec3 direction = Vec3::Zero();
    double radius = 0.0; ///< Cylinder, sphere, circle (arc); cone: at `point`.
    /// Cylinder, cone: the axis's points level with the face's two ends (a bore's mouth and its
    /// bottom), lowest along `direction` first. Other faces: `point`; edges and vertices: zero.
    Vec3 axisStart = Vec3::Zero();
    Vec3 axisEnd = Vec3::Zero();
    /// Cylinders, cones and circles have an axis (`point`, `direction`).
    bool hasAxis() const noexcept {
        return kind == Kind::Cylinder || kind == Kind::Cone || kind == Kind::Circle;
    }
};

/// The geometry of the sub-shape at a 1-based index (model coordinates). Throws
/// Error(InvalidArgument) if out of range.
SubShapeGeometry subShapeGeometry(const Model& model, SubShapeType type, int index);

/// Number of distinct faces or edges (indices are 1-based, in TopExp map order).
int subShapeCount(const Model& model, SubShapeType type);

/// Signature of the sub-shape at a 1-based index. Throws Error(InvalidArgument) if out of range.
ShapeSignature signatureOf(const Model& model, SubShapeType type, int index);

/// Finds the sub-shape matching `signature`: `hintIndex` if it still matches, otherwise the best
/// match within tolerance (centroid distance plus relative measure difference). Returns nullopt
/// when nothing matches.
std::optional<int> matchSubShape(const Model& model, SubShapeType type,
                                 const ShapeSignature& signature, int hintIndex,
                                 double tolerance = 1e-4);

/// The edges (1-based) of the outline containing edge `edgeIndex` on a planar face
/// perpendicular to `axis` (model coordinates): one click on a pocket's or a part's outline
/// gives all of it. Just {edgeIndex} when no such face has the edge.
std::vector<int> edgeLoopOf(const Model& model, int edgeIndex, const Vec3& axis);

/// True when the edge (1-based) lies across `axis` (model coordinates): all of it at one height
/// along the axis, within `tolerance` (a horizontal line, circle or curve for a vertical axis).
bool edgeAcross(const Model& model, int edgeIndex, const Vec3& axis, double tolerance = 1e-4);

/// True when the point (model coordinates) lies inside one of the model's solids, by more than
/// `tolerance` from its boundary.
bool containsPoint(const Model& model, const Vec3& point, double tolerance = 1e-6);

} // namespace ocam::cad
