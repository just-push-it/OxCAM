// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Operations on paths used by toolpath strategies: arc fitting, re-starting loops, nearest
// points, and segment containment tests.

#pragma once

#include "ocam/geom2d/Region.hpp"

#include <vector>

namespace ocam::geom2d {

/// Replaces runs of points that lie on a line or circular arc (within `tolerance`) by single
/// line or arc segments. Arcs sweep at most 180 degrees. Used to turn polygonal offsets back
/// into G2/G3-friendly geometry. See docs/design-log/0010.
Polyline fitArcs(const std::vector<Vec2>& points, bool closed, double tolerance);

struct ClosestPoint {
    std::size_t segment = 0; ///< Segment index (from vertex `segment` to the next).
    double t = 0.0;          ///< Parameter along the segment in [0, 1] (arc length fraction).
    Vec2 point = Vec2::Zero();
    double distance = 0.0;
};

/// Nearest point on a polyline (exact for arcs). The polyline must have at least one segment.
ClosestPoint closestPoint(const Polyline& path, const Vec2& p);

/// Returns a closed polyline that starts at the point of `loop` nearest to `p` (splitting the
/// segment there if needed) and traverses the same geometry in the same direction.
Polyline restartAtNearest(const Polyline& loop, const Vec2& p);

/// A point strictly inside the region (its centroid when that is inside, otherwise the middle of
/// the widest interior span on a horizontal line through the region).
Vec2 interiorPoint(const Region& region, double chordTolerance = 1e-3);

/// True when the straight segment a-b stays inside the regions: both ends and the midpoint are
/// inside, and it does not cross any boundary (boundaries discretized at `chordTolerance`).
bool segmentInside(const Regions& regions, const Vec2& a, const Vec2& b,
                   double chordTolerance = 1e-3);

} // namespace ocam::geom2d
