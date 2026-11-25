// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// STL triangle files: reading (binary and ASCII) and writing (binary). Machine model parts are
// stored as binary STL next to the machine file and in project archives (DL-0037).

#pragma once

#include "ocam/geom3d/Mesh.hpp"

#include <string>
#include <string_view>

namespace ocam::geom3d {

/// Reads binary or ASCII STL content. Vertices closer than `weldTolerance` are merged (STL
/// stores each triangle's corners separately). Throws Error(Parse) naming `sourceName` for
/// truncated or malformed content and for files without triangles.
Mesh readStl(std::string_view bytes, std::string_view sourceName, double weldTolerance = 1e-6);

/// Binary STL of the mesh (facet normals from the corners, a fixed header), so the same mesh
/// always gives the same bytes. Coordinates are stored as 32-bit floats.
std::string writeStl(const Mesh& mesh);

} // namespace ocam::geom3d
