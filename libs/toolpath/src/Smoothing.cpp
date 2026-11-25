// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Arc fitting of finished toolpaths: see Smoothing.hpp and DL-0032.

#include "ocam/toolpath/Smoothing.hpp"

#include <ocam/geom2d/PathOps.hpp>
#include <ocam/geom2d/Polyline.hpp>

#include <array>
#include <cmath>

namespace ocam::toolpath {
namespace {

bool eligible(const Move& m) {
    return m.kind == MoveKind::Linear && m.pitch == 0.0 &&
           (m.axis - Vec3::UnitZ()).squaredNorm() < 1e-18;
}

bool sameRun(const Move& a, const Move& b) {
    return a.feedClass == b.feedClass && a.feedRate == b.feedRate;
}

/// How many points from `k` on keep one coordinate (x, y or z) constant.
std::size_t constantRun(const std::vector<Vec3>& pts, std::size_t k, int axis) {
    std::size_t n = 1;
    while (k + n < pts.size() && std::abs(pts[k + n][axis] - pts[k][axis]) < 1e-9) {
        ++n;
    }
    return n;
}

/// Moves from pts[0] through pts[n - 1] in `plane`, fitted with arcs, appended to `out`.
std::size_t fitRun(const std::vector<Vec3>& pts, ArcPlane plane, const Move& like, double tolerance,
                   std::vector<Move>& out) {
    std::vector<Vec2> uv;
    uv.reserve(pts.size());
    for (const Vec3& p : pts) {
        uv.push_back(planeCoords(p, plane));
    }
    const double normal = planeNormal(pts.front(), plane);
    const geom2d::Polyline fitted = geom2d::fitArcs(uv, false, tolerance);
    const auto& v = fitted.vertices();
    std::size_t arcs = 0;
    for (std::size_t i = 0; i + 1 < v.size(); ++i) {
        Move m = like;
        m.end = fromPlaneCoords(v[i + 1].p, normal, plane);
        const auto arc = geom2d::arcFromBulge(v[i].p, v[i + 1].p, v[i].bulge);
        if (arc) {
            m.kind = arc->sweep < 0.0 ? MoveKind::ArcCW : MoveKind::ArcCCW;
            m.center = fromPlaneCoords(arc->center, normal, plane);
            m.plane = plane;
            ++arcs;
        } else {
            m.kind = MoveKind::Linear;
        }
        out.push_back(m);
    }
    return arcs;
}

} // namespace

SmoothingResult fitArcs(Toolpath& path, double tolerance) {
    SmoothingResult result;
    result.movesBefore = path.moves.size();
    if (tolerance <= 0.0) {
        result.movesAfter = path.moves.size();
        return result;
    }
    std::vector<Move> out;
    out.reserve(path.moves.size());
    Vec3 pos = path.start;
    const std::size_t n = path.moves.size();
    for (std::size_t i = 0; i < n;) {
        const Move& first = path.moves[i];
        if (!eligible(first)) {
            out.push_back(first);
            if (first.kind != MoveKind::Dwell) {
                pos = first.end;
            }
            ++i;
            continue;
        }
        // The run: consecutive straight moves alike, from the current position.
        std::vector<Vec3> pts{pos};
        std::size_t j = i;
        while (j < n && eligible(path.moves[j]) && sameRun(path.moves[j], first)) {
            pts.push_back(path.moves[j].end);
            ++j;
        }
        // Split into pieces lying in one plane each (the longest that fits from each point).
        for (std::size_t k = 0; k + 1 < pts.size();) {
            const std::array<std::size_t, 3> runs{constantRun(pts, k, 2), constantRun(pts, k, 1),
                                                  constantRun(pts, k, 0)};
            const std::array<ArcPlane, 3> planes{ArcPlane::XY, ArcPlane::ZX, ArcPlane::YZ};
            std::size_t best = 0;
            for (std::size_t q = 1; q < 3; ++q) {
                if (runs[q] > runs[best]) {
                    best = q;
                }
            }
            const std::size_t len = runs[best];
            if (len < 3) {
                Move m = first;
                m.end = pts[k + 1];
                out.push_back(m);
                ++k;
                continue;
            }
            const std::vector<Vec3> piece(pts.begin() + static_cast<std::ptrdiff_t>(k),
                                          pts.begin() + static_cast<std::ptrdiff_t>(k + len));
            result.arcs += fitRun(piece, planes[best], first, tolerance, out);
            k += len - 1;
        }
        pos = pts.back();
        i = j;
    }
    path.moves = std::move(out);
    result.movesAfter = path.moves.size();
    return result;
}

std::size_t linearizeArcs(Toolpath& path, double chordTolerance) {
    std::vector<Move> out;
    out.reserve(path.moves.size());
    Vec3 pos = path.start;
    std::size_t replaced = 0;
    for (const Move& m : path.moves) {
        if (m.isArc()) {
            const std::vector<Vec3> pts = sampleArc(pos, m, chordTolerance);
            for (std::size_t k = 1; k < pts.size(); ++k) {
                Move line = m;
                line.kind = MoveKind::Linear;
                line.plane = ArcPlane::XY;
                line.center = Vec3::Zero();
                line.end = pts[k];
                out.push_back(line);
            }
            ++replaced;
        } else {
            out.push_back(m);
        }
        if (m.kind != MoveKind::Dwell) {
            pos = m.end;
        }
    }
    path.moves = std::move(out);
    return replaced;
}

} // namespace ocam::toolpath
