// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "ocam/geom2d/Polyline.hpp"

#include <vector>

namespace ocam::geom2d {

/// A connected planar area: one counter-clockwise outer boundary and zero or more clockwise
/// holes. This is the neutral input to 2.5D toolpath strategies.
struct Region {
    Polyline outer;
    std::vector<Polyline> holes;

    double area() const noexcept; ///< Outer area minus hole areas (exact for arcs).
    Aabb2 bounds() const noexcept { return outer.bounds(); }
    bool contains(const Vec2& p, double chordTolerance = 1e-4) const;
};

using Regions = std::vector<Region>;

double totalArea(const Regions& regions) noexcept;

/// Organizes non-intersecting closed loops (for example, the wires of a planar section) into
/// regions by nesting depth: depth 0 is an outer boundary, depth 1 a hole in it, depth 2 an
/// island inside the hole (a new outer), and so on. Orientation of the inputs is ignored; the
/// output follows the Region convention. Arcs are preserved. Open or degenerate loops are dropped.
Regions buildRegions(std::vector<Polyline> loops, double chordTolerance = 1e-4);

} // namespace ocam::geom2d
