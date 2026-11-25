// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "Internal.hpp"

#include <ocam/geom2d/Clip.hpp>

namespace ocam::strategies::detail {

std::vector<std::vector<Vec2>> pathsAtDepth(const toolpath::Toolpath& tp, std::size_t first,
                                            const Vec3& from, double z, double chordTolerance) {
    std::vector<std::vector<Vec2>> paths;
    std::vector<Vec2> current;
    Vec3 pos = from;
    auto flush = [&] {
        if (current.size() >= 1) {
            paths.push_back(std::move(current));
        }
        current.clear();
    };
    for (std::size_t i = first; i < tp.moves.size(); ++i) {
        const toolpath::Move& m = tp.moves[i];
        const bool atDepth = std::abs(pos.z() - z) < 1e-7 && std::abs(m.end.z() - z) < 1e-7;
        if (m.kind == toolpath::MoveKind::Rapid || m.kind == toolpath::MoveKind::Dwell ||
            m.kind == toolpath::MoveKind::Cycle || !atDepth) {
            flush();
        } else {
            if (current.empty()) {
                current.push_back(pos.head<2>());
            }
            if (m.isArc()) {
                const auto pts = toolpath::sampleArc(pos, m, chordTolerance);
                for (std::size_t k = 1; k < pts.size(); ++k) {
                    current.push_back(pts[k].head<2>());
                }
            } else {
                current.push_back(m.end.head<2>());
            }
        }
        if (m.kind != toolpath::MoveKind::Dwell) {
            pos = m.end;
        }
    }
    flush();
    return paths;
}

Vec2 startTangent(const geom2d::Polyline& loop) {
    const auto& v = loop.vertices();
    if (v.size() < 2) {
        return Vec2::UnitX();
    }
    const Vec2 chord = v[1].p - v[0].p;
    const Vec2 dir = chord.norm() > 0 ? Vec2(chord.normalized()) : Vec2::UnitX();
    if (auto arc = geom2d::arcFromBulge(v[0].p, v[1].p, v[0].bulge)) {
        return rotate(dir, -arc->sweep / 2.0);
    }
    return dir;
}

Vec2 preferredStart(const geom2d::Polyline& loop) {
    const auto& v = loop.vertices();
    double best = -1.0;
    Vec2 point = v.empty() ? Vec2::Zero() : v[0].p;
    for (std::size_t i = 0; i < loop.segmentCount(); ++i) {
        const Vec2& a = v[i].p;
        const Vec2& b = v[(i + 1) % v.size()].p;
        if (v[i].bulge == 0.0 && (b - a).norm() > best) {
            best = (b - a).norm();
            point = 0.5 * (a + b);
        }
    }
    return point;
}

std::vector<geom2d::Polyline> containedPieces(const geom2d::Polyline& pass,
                                              const geom2d::Regions& boundary, double tol,
                                              double minLength) {
    if (boundary.empty()) {
        return {pass};
    }
    std::vector<Vec2> pts = pass.discretize(tol);
    if (pts.size() < 2) {
        return {};
    }
    if (pass.isClosed()) {
        pts.push_back(pts.front());
    }
    const auto pieces = geom2d::clipPaths({pts}, boundary, true, geom2d::ClipOptions{tol});
    if (pass.isClosed() && pieces.size() == 1 && pieces.front().size() >= pts.size() &&
        (pieces.front().front() - pieces.front().back()).norm() <= tol) {
        return {pass}; // not cut
    }
    std::vector<geom2d::Polyline> out;
    for (const auto& piece : pieces) {
        if (piece.size() < 2) {
            continue;
        }
        geom2d::Polyline line = geom2d::Polyline::fromPoints(piece, false);
        if (line.length() > std::max(minLength, tol)) {
            out.push_back(std::move(line));
        }
    }
    return out;
}

geom2d::Aabb2 containedArea(const geom2d::Aabb2& area, const geom2d::Regions& boundary,
                            double margin) {
    if (boundary.empty()) {
        return area;
    }
    geom2d::Aabb2 b;
    for (const auto& region : boundary) {
        b.extend(region.outer.bounds());
    }
    geom2d::Aabb2 out;
    out.min = area.min.cwiseMax(b.min - Vec2::Constant(margin));
    out.max = area.max.cwiseMin(b.max + Vec2::Constant(margin));
    return out.isEmpty() ? area : out;
}

/// Douglas-Peucker simplification of a 3D polyline (keeps both ends).
std::vector<Vec3> simplify3d(const std::vector<Vec3>& pts, double tol) {
    if (pts.size() < 3) {
        return pts;
    }
    std::vector<char> keep(pts.size(), 0);
    keep.front() = keep.back() = 1;
    std::vector<std::pair<std::size_t, std::size_t>> stack{{0, pts.size() - 1}};
    while (!stack.empty()) {
        const auto [a, b] = stack.back();
        stack.pop_back();
        const Vec3 d = pts[b] - pts[a];
        const double len = d.norm();
        double worst = -1.0;
        std::size_t idx = a;
        for (std::size_t k = a + 1; k < b; ++k) {
            const Vec3 v = pts[k] - pts[a];
            const double dist = len > 1e-12 ? v.cross(d).norm() / len : v.norm();
            if (dist > worst) {
                worst = dist;
                idx = k;
            }
        }
        if (worst > tol) {
            keep[idx] = 1;
            stack.emplace_back(a, idx);
            stack.emplace_back(idx, b);
        }
    }
    std::vector<Vec3> out;
    for (std::size_t k = 0; k < pts.size(); ++k) {
        if (keep[k]) {
            out.push_back(pts[k]);
        }
    }
    return out;
}

} // namespace ocam::strategies::detail
