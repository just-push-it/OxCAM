// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Fixture collision checks (DL-0035): the tool assembly (cutting part, shank, holder) moved
// along a toolpath against the solids that hold the part (vise bodies, clamps, plates). Checks
// run on sampled poses of the tool, tilted along the toolpath's axis, using convex slices of the
// tool and the fixtures' triangles (GJK).

#pragma once

#include "ocam/sim/ToolShape.hpp"

#include <ocam/foundation/Cancel.hpp>
#include <ocam/geom3d/Mesh.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <string>
#include <vector>

namespace ocam::sim {

/// A named solid holding the part, as a triangle mesh in the WCS.
struct FixtureBody {
    std::string name;
    geom3d::Mesh mesh;
};

struct FixtureCollision {
    std::size_t firstMove = 0; ///< Index into Toolpath::moves.
    std::size_t lastMove = 0;
    std::string body;             ///< FixtureBody::name.
    std::string toolPart;         ///< "tool", "tool shank" or "tool holder".
    Vec3 position = Vec3::Zero(); ///< A point of the contact on the body (WCS).
    double distance = 0.0;        ///< Smallest distance found (0 = overlap) [mm].
};

struct FixtureCheckOptions {
    double clearance = 0.0; ///< Distances below this count [mm]; 0 = touching only.
    double step = 1.0;      ///< Largest tip travel between checked poses [mm].
    std::size_t maxCollisions = 200;
};

/// Checks a toolpath (CL, WCS; canned cycles expanded) against the bodies. Consecutive poses
/// of the same tool part touching the same body form one collision.
std::vector<FixtureCollision> checkFixtures(const toolpath::Toolpath& path,
                                            const std::vector<FixtureBody>& bodies,
                                            const ToolShape& tool,
                                            const FixtureCheckOptions& options = {},
                                            CancelToken* cancel = nullptr);

} // namespace ocam::sim
