// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Edges and face boundaries as chains of points in 3D, for work that follows them off a plane:
// curves wrapped around a cylinder (DL-0084).

#pragma once

#include "ocam/cad/Model.hpp"

#include <ocam/foundation/Math.hpp>

#include <vector>

namespace ocam::cad {

/// Points along edges chained end to start, in a frame.
struct Chain3 {
    std::vector<Vec3> points; ///< In order; a closed chain does not repeat its first point.
    bool closed = false;      ///< The last point joins the first.
};

/// The edges (1-based indices, any order and direction) chained end to start in 3D, in the frame
/// `toFrame` maps model coordinates to, each discretized within `curveTolerance`. Edges that do not
/// meet another come back as open chains of their own. Throws Error(InvalidArgument) for an index
/// out of range.
std::vector<Chain3> edgeChains3d(const Model& model, const std::vector<int>& edgeIndices,
                                 const Transform3& toFrame, double curveTolerance = 1e-3);

/// The boundary of a face (1-based index) as closed chains in the frame. Seam edges, where a
/// face that goes all the way around a periodic surface meets itself, are not boundary: a band
/// around a cylinder comes back as its two rims. Throws Error(InvalidArgument) for an index out
/// of range.
std::vector<Chain3> faceChains(const Model& model, int faceIndex, const Transform3& toFrame,
                               double curveTolerance = 1e-3);

/// The least distance of a face (1-based index) from the frame's X axis, exactly: a flat floor's
/// is at its middle, not at its edges.
double faceDistanceFromXAxis(const Model& model, int faceIndex, const Transform3& toFrame);

} // namespace ocam::cad
