// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "ocam/cad/Model.hpp"

#include <ocam/foundation/Math.hpp>
#include <ocam/geom2d/Region.hpp>

namespace ocam::cad {

/// Cross-section of the model's solids with the plane Z = z, as regions in XY.
///
/// Lines and circular arcs whose axis is parallel to Z are kept exact (arcs become bulge
/// segments). Other curves are discretized within `curveTolerance` [mm]. Sectioning exactly at
/// the height of a horizontal face is ill-defined; callers should offset z slightly.
geom2d::Regions sectionAtZ(const Model& model, double z, double curveTolerance = 1e-3);

/// Half profile for turning: the section of the model (placed by `toFrame`) with the frame's XZ
/// plane, restricted to x >= 0, as regions in (u = frame z, v = frame x = radius). For a solid of
/// revolution about the frame's Z axis this is its generating profile. Straight segments only.
geom2d::Regions axialProfile(const Model& model, const Transform3& toFrame,
                             double curveTolerance = 1e-3);
/// True when every face of the model is the same all round the frame's Z axis.
bool turnsAboutZ(const Model& model, const Transform3& toFrame);
/// What turning leaves of the model: its axialProfile when it is the same all round (turnsAboutZ),
/// otherwise those of several planes around the axis together. A cross hole, a flat or a keyway
/// lies in only some of them, so it does not make turning leave a gap for it.
geom2d::Regions axialEnvelope(const Model& model, const Transform3& toFrame, double curveTolerance);

struct FaceOutline {
    geom2d::Regions regions; ///< Outer boundary and inner loops (holes) in frame XY.
    double z = 0.0;          ///< Height of the face plane in the frame.
};

/// Outline of a planar face (1-based index) expressed in another frame: `toFrame` maps model
/// coordinates to that frame (for example model -> project -> setup WCS). The face must be
/// perpendicular to the frame's Z axis, otherwise Error(InvalidArgument). Arcs stay exact.
FaceOutline faceOutline(const Model& model, int faceIndex, const Transform3& toFrame,
                        double curveTolerance = 1e-3);

/// Closed loops chained from edges (1-based indices, any order and direction) in another frame,
/// flattened onto the frame's XY plane; `z` is their lowest point. Throws
/// Error(InvalidArgument) when the edges do not chain into closed loops.
FaceOutline edgeLoops(const Model& model, const std::vector<int>& edgeIndices,
                      const Transform3& toFrame, double curveTolerance = 1e-3);

/// A chain of edges flattened onto a frame's XY plane.
struct EdgeChain {
    geom2d::Polyline path; ///< Closed when the chain ends where it starts.
    double z = 0.0;        ///< Its lowest point along the frame's Z.
};

/// The edges (1-based indices, any order and direction) chained end to end in space, open or
/// closed, in another frame and flattened onto its XY plane: one chain per run of edges that
/// meet (where three meet, a chain goes on along one of them). Lines and arcs about the frame's
/// Z stay exact; other curves are discretized within `curveTolerance`. Edges along the frame's
/// Z alone give no chain.
std::vector<EdgeChain> edgeChains(const Model& model, const std::vector<int>& edgeIndices,
                                  const Transform3& toFrame, double curveTolerance = 1e-3);

} // namespace ocam::cad
