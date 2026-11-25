// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Picked faces, edges and vertices as datums (DL-0035: the datums of a workholding template are
// picked on its STEP parts): their exact geometry (subShapeGeometry, SubShape.hpp) seen as a
// plane, an axis, a line or a point.

#pragma once

#include "ocam/cad/SubShape.hpp"

#include <optional>
#include <vector>

namespace ocam::cad {

/// What a picked sub-shape is, as a datum.
struct DatumGeometry {
    enum class Shape {
        Plane, ///< A flat face: `point` its centre, `direction` its outward normal.
        Axis,  ///< A cylindrical or conical face, or a circular edge: `point` on the axis (the
               ///< face's or circle's centre), `direction` the axis.
        Line,  ///< A straight edge: `point` its middle, `direction` along it.
        Point, ///< A vertex or another face or edge: `point` it (its centre), `direction` the
               ///< face's outward normal there when on a face, else zero.
    };
    Shape shape = Shape::Point;
    Vec3 point = Vec3::Zero();
    Vec3 direction = Vec3::Zero();
};

/// The datum geometry of the face, edge or vertex at a 1-based index. Throws
/// Error(InvalidArgument) if out of range.
DatumGeometry datumGeometry(const Model& model, SubShapeType type, int index);

/// A point clicked on a face: the point of the face nearest `near` and the face's outward normal
/// there (shape Point). Throws Error(InvalidArgument) if the index is out of range.
DatumGeometry pointOnFace(const Model& model, int faceIndex, const Vec3& near);

} // namespace ocam::cad
