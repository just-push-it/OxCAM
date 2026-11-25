// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// An edge sampled with the two faces that meet along it: the input of 3D edge breaking
// (DL-0083).

#pragma once

#include "ocam/cad/Model.hpp"

#include <ocam/foundation/Math.hpp>

#include <array>
#include <string>
#include <vector>

namespace ocam::cad {

/// A point of an edge (model coordinates).
struct EdgeFacePoint {
    Vec3 point = Vec3::Zero();
    Vec3 tangent = Vec3::UnitX(); ///< Along the edge (unit), the same sense throughout.
    std::array<Vec3, 2> normal{Vec3::Zero(), Vec3::Zero()}; ///< Each face's outward normal.
    /// Each face's direction away from the edge, in the face and across the edge (unit).
    std::array<Vec3, 2> inFace{Vec3::Zero(), Vec3::Zero()};
};

struct EdgeFaces {
    std::vector<EdgeFacePoint> points;
    std::array<int, 2> faces{0, 0}; ///< The two faces (1-based indices).
    bool closed = false;            ///< The edge ends where it starts (a circle): no repeat.
    /// Not empty: why the edge has no two faces to break between (a free edge, a seam, ...).
    std::string problem;
};

/// Points along the edge (1-based index) at most `spacing` apart, evenly by length, with the two
/// faces that share it. Throws Error(InvalidArgument) for an index out of range.
EdgeFaces edgeFaces(const Model& model, int edgeIndex, double spacing);

} // namespace ocam::cad
