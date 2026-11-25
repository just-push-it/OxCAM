// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Helpers shared by the strategy implementations.

#pragma once

#include "ocam/strategies/Common.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/geom2d/Region.hpp>
#include <ocam/toolpath/PathBuilder.hpp>

#include <fmt/format.h>

#include <cmath>
#include <vector>

namespace ocam::strategies::detail {

/// Rotating milling tools only; returns the radius.
inline double requireMillingTool(const tooling::Tool& tool) {
    if (tooling::isTurningTool(tool.type)) {
        throw Error(ErrorCode::InvalidArgument, "a turning tool cannot be used for milling");
    }
    if (tool.mill.diameter <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("tool T{} has no diameter", tool.number));
    }
    return tool.mill.diameter / 2.0;
}

inline Vec2 rotate(const Vec2& p, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {c * p.x() - s * p.y(), s * p.x() + c * p.y()};
}

/// x-intervals where the horizontal line y crosses into the regions (even-odd over all loops).
inline std::vector<std::pair<double, double>>
scanIntervals(const std::vector<std::vector<Vec2>>& loops, double y) {
    std::vector<double> xs;
    for (const auto& pts : loops) {
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const Vec2& p = pts[i];
            const Vec2& q = pts[(i + 1) % pts.size()];
            if ((p.y() <= y) != (q.y() <= y)) {
                xs.push_back(p.x() + (y - p.y()) * (q.x() - p.x()) / (q.y() - p.y()));
            }
        }
    }
    std::sort(xs.begin(), xs.end());
    std::vector<std::pair<double, double>> out;
    for (std::size_t i = 0; i + 1 < xs.size(); i += 2) {
        if (xs[i + 1] - xs[i] > 1e-9) {
            out.emplace_back(xs[i], xs[i + 1]);
        }
    }
    return out;
}

/// All boundary loops of the regions, discretized and transformed by `xf`.
template <typename Xf>
std::vector<std::vector<Vec2>> discretizedLoops(const geom2d::Regions& regions, double tol, Xf xf) {
    std::vector<std::vector<Vec2>> loops;
    auto add = [&](const geom2d::Polyline& loop) {
        auto pts = loop.discretize(tol);
        for (auto& p : pts) {
            p = xf(p);
        }
        loops.push_back(std::move(pts));
    };
    for (const auto& r : regions) {
        add(r.outer);
        for (const auto& h : r.holes) {
            add(h);
        }
    }
    return loops;
}

inline bool anyContains(const geom2d::Regions& regions, const Vec2& p, double tol) {
    return std::any_of(regions.begin(), regions.end(),
                       [&](const geom2d::Region& r) { return r.contains(p, tol); });
}

/// XY paths traced by feed moves at height z among moves[first, last), starting from `from`.
/// Used for residual (uncut material) detection.
std::vector<std::vector<Vec2>> pathsAtDepth(const toolpath::Toolpath& tp, std::size_t first,
                                            const Vec3& from, double z, double chordTolerance);

/// Unit tangent at the start of a polyline's first segment.
Vec2 startTangent(const geom2d::Polyline& loop);

/// Midpoint of the longest straight segment (or the first vertex when there is none).
Vec2 preferredStart(const geom2d::Polyline& loop);

/// The parts of a pass inside the tool-centre containment `boundary`, in the pass's direction:
/// the pass itself when the boundary is empty or holds all of it, else open pieces (each longer
/// than `minLength`).
std::vector<geom2d::Polyline> containedPieces(const geom2d::Polyline& pass,
                                              const geom2d::Regions& boundary, double tol,
                                              double minLength = 0.0);

/// `area` cut down to the bounds of the containment `boundary` grown by `margin` (unchanged when
/// the boundary is empty): no surface sampling where nothing is cut.
geom2d::Aabb2 containedArea(const geom2d::Aabb2& area, const geom2d::Regions& boundary,
                            double margin);

/// Douglas-Peucker simplification of a 3D polyline to `tol` (keeps both ends).
std::vector<Vec3> simplify3d(const std::vector<Vec3>& pts, double tol);

} // namespace ocam::strategies::detail
