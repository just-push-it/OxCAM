// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "ocam/cad/Model.hpp"

#include <ocam/geom3d/Mesh.hpp>

#include <vector>

namespace ocam::cad {

struct TessellationOptions {
    double linearDeflection = 0.01;  ///< Max chord distance from the surface [mm].
    double angularDeflection = 0.35; ///< Max angle between adjacent facet normals [rad].
};

/// Triangulates every face and returns a single welded, outward-oriented mesh. For a valid
/// closed solid the result is closed (Mesh::isClosed()).
geom3d::Mesh tessellate(const Model& model, const TessellationOptions& options = {});

/// Each face's triangles on their own: element k is face k + 1 (as sub-shape references number
/// them), welded within the face.
std::vector<geom3d::Mesh> tessellateEachFace(const Model& model,
                                             const TessellationOptions& options = {});

/// Triangulates only the given faces (1-based indices, as sub-shape references number them).
geom3d::Mesh tessellateFaces(const Model& model, const std::vector<int>& faceIndices,
                             const TessellationOptions& options = {});

} // namespace ocam::cad
