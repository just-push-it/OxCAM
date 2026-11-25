// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Arc-aware polylines. Each vertex carries a "bulge" describing the segment to the next vertex:
// bulge = tan(sweep / 4), where sweep is the signed included angle of a circular arc (positive =
// counter-clockwise). A bulge of 0 is a straight line. This keeps true arcs exact through
// sectioning and area computation; discretize() produces polygons only where needed.

#pragma once

#include "ocam/geom2d/Bounds.hpp"

#include <ocam/foundation/Math.hpp>

#include <optional>
#include <vector>

namespace ocam::geom2d {

struct Vertex {
    Vec2 p;
    double bulge = 0.0; ///< tan(sweep/4) of the segment from this vertex to the next.

    bool operator==(const Vertex&) const = default;
};

/// Geometry of one circular-arc segment.
struct Arc {
    Vec2 center;
    double radius = 0.0;
    double startAngle = 0.0; ///< Angle of the start point around `center` [rad].
    double sweep = 0.0;      ///< Signed included angle, positive = CCW [rad].

    Vec2 pointAt(double t) const; ///< t in [0, 1] along the arc.
};

/// Returns the arc for a segment, or nullopt when the segment is straight (|bulge| ~ 0) or
/// degenerate (coincident endpoints).
std::optional<Arc> arcFromBulge(const Vec2& p0, const Vec2& p1, double bulge) noexcept;

/// Bulge for an arc from p0 to p1 with the given signed sweep.
double bulgeFromSweep(double sweep) noexcept;

class Polyline {
public:
    Polyline() = default;
    Polyline(std::vector<Vertex> vertices, bool closed)
        : vertices_(std::move(vertices)), closed_(closed) {}

    /// Straight-segment polyline from points.
    static Polyline fromPoints(const std::vector<Vec2>& points, bool closed);

    /// Closed circle as two half arcs, counter-clockwise.
    static Polyline circle(const Vec2& center, double radius);

    /// Closed axis-aligned rectangle, counter-clockwise.
    static Polyline rectangle(const Vec2& min, const Vec2& max);

    const std::vector<Vertex>& vertices() const noexcept { return vertices_; }
    std::vector<Vertex>& vertices() noexcept { return vertices_; }
    bool isClosed() const noexcept { return closed_; }
    bool isEmpty() const noexcept { return vertices_.empty(); }
    std::size_t segmentCount() const noexcept;

    /// Signed enclosed area (positive = CCW), exact for arc segments. 0 for open polylines.
    double signedArea() const noexcept;
    double area() const noexcept;
    bool isCounterClockwise() const noexcept { return signedArea() > 0.0; }

    double length() const noexcept;
    Aabb2 bounds() const noexcept; ///< Exact, including arc extremes.

    /// Reverses direction in place (and negates bulges).
    void reverse();

    /// Points approximating the polyline so no point of a true arc deviates more than
    /// `chordTolerance` from the chords. A closed polyline does not repeat its first point.
    std::vector<Vec2> discretize(double chordTolerance) const;

    /// Winding-number containment test against the discretized boundary (closed only).
    bool contains(const Vec2& point, double chordTolerance = 1e-4) const;

    bool operator==(const Polyline&) const = default;

private:
    std::vector<Vertex> vertices_;
    bool closed_ = false;
};

} // namespace ocam::geom2d
