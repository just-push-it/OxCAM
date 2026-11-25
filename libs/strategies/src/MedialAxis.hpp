// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// An approximate medial axis transform of planar regions on a grid: the distance transform's
// ridges, thinned to chains, joined at nodes, spurs pruned. Enough for placing maximal inscribed
// circles along it (DL-0031); not an exact Voronoi diagram.

#pragma once

#include <ocam/geom2d/Region.hpp>

#include <vector>

namespace ocam::strategies::detail {

struct MedialGraph {
    struct Node {
        Vec2 p = Vec2::Zero();
        double radius = 0.0;    ///< Inscribed radius at the node.
        std::vector<int> edges; ///< Indices into `edges`.
    };
    struct Edge {
        int a = -1;                ///< Node at points.front().
        int b = -1;                ///< Node at points.back().
        std::vector<Vec2> points;  ///< Along the branch, from node a to node b.
        std::vector<double> radii; ///< Inscribed radius at each point.
        double length = 0.0;
    };
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    bool empty() const noexcept { return edges.empty(); }
};

/// The medial axis of `area` on a grid of `cell` (the inscribed radius is read to about half a
/// cell): branches as chains of points with their radius, joined at nodes; spurs shorter than
/// `prune` whose end lies at the boundary are removed, and points where the inscribed radius
/// is under `minRadius` are dropped (a corner the circle cannot reach into).
MedialGraph medialAxis(const geom2d::Regions& area, double cell, double prune, double minRadius,
                       double chordTolerance);

} // namespace ocam::strategies::detail
