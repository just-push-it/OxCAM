// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/geom2d/Clip.hpp"

#include <clipper2/clipper.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ocam::geom2d {
namespace {

namespace c2 = Clipper2Lib;

// Fixed-point precision used by Clipper2: 1e-6 mm resolution.
constexpr int kDecimals = 6;

c2::PathD toPath(const Polyline& loop, double chordTolerance) {
    c2::PathD path;
    for (const Vec2& p : loop.discretize(chordTolerance)) {
        path.emplace_back(p.x(), p.y());
    }
    return path;
}

c2::PathsD toPaths(const Regions& regions, double chordTolerance) {
    c2::PathsD paths;
    for (const auto& r : regions) {
        paths.push_back(toPath(r.outer, chordTolerance));
        for (const auto& h : r.holes) {
            paths.push_back(toPath(h, chordTolerance));
        }
    }
    return paths;
}

Polyline fromPath(const c2::PathD& path) {
    std::vector<Vec2> pts;
    pts.reserve(path.size());
    for (const auto& p : path) {
        pts.emplace_back(p.x, p.y);
    }
    return Polyline::fromPoints(pts, true);
}

// Walks a PolyTree: each outer polygon becomes a region, its direct children are holes, and
// grandchildren (islands) start new regions.
void collect(const c2::PolyPathD& node, Regions& out) {
    for (std::size_t i = 0; i < node.Count(); ++i) {
        const c2::PolyPathD& outer = *node.Child(i);
        Region region{fromPath(outer.Polygon()), {}};
        if (!region.outer.isCounterClockwise()) {
            region.outer.reverse();
        }
        for (std::size_t j = 0; j < outer.Count(); ++j) {
            const c2::PolyPathD& hole = *outer.Child(j);
            Polyline h = fromPath(hole.Polygon());
            if (h.isCounterClockwise()) {
                h.reverse();
            }
            region.holes.push_back(std::move(h));
            collect(hole, out);
        }
        out.push_back(std::move(region));
    }
}

Regions toRegions(const c2::PolyTreeD& tree) {
    Regions out;
    collect(tree, out);
    return out;
}

c2::JoinType toJoin(JoinStyle j) {
    switch (j) {
    case JoinStyle::Miter:
        return c2::JoinType::Miter;
    case JoinStyle::Square:
        return c2::JoinType::Square;
    case JoinStyle::Round:
        break;
    }
    return c2::JoinType::Round;
}

Regions boolean(c2::ClipType type, const Regions& a, const Regions& b, const ClipOptions& o) {
    c2::PolyTreeD tree;
    c2::BooleanOp(type, c2::FillRule::NonZero, toPaths(a, o.chordTolerance),
                  toPaths(b, o.chordTolerance), tree, kDecimals);
    return toRegions(tree);
}

} // namespace

Regions offset(const Regions& regions, double delta, const ClipOptions& options) {
    const c2::PathsD inflated = c2::InflatePaths(
        toPaths(regions, options.chordTolerance), delta, toJoin(options.join), c2::EndType::Polygon,
        options.miterLimit, kDecimals, options.chordTolerance);
    // Rebuild nesting (and merge anything that grew together).
    c2::PolyTreeD tree;
    c2::BooleanOp(c2::ClipType::Union, c2::FillRule::NonZero, inflated, {}, tree, kDecimals);
    return toRegions(tree);
}

Regions sweep(const std::vector<std::vector<Vec2>>& paths, double radius,
              const ClipOptions& options) {
    c2::PathsD open;
    for (const auto& path : paths) {
        c2::PathD p;
        for (const Vec2& v : path) {
            p.emplace_back(v.x(), v.y());
        }
        if (p.size() == 1) {
            p.push_back(p.front()); // a single point sweeps a disk
        }
        if (!p.empty()) {
            open.push_back(std::move(p));
        }
    }
    const c2::PathsD inflated =
        c2::InflatePaths(open, radius, c2::JoinType::Round, c2::EndType::Round, options.miterLimit,
                         kDecimals, options.chordTolerance);
    c2::PolyTreeD tree;
    c2::BooleanOp(c2::ClipType::Union, c2::FillRule::NonZero, inflated, {}, tree, kDecimals);
    return toRegions(tree);
}

std::vector<std::vector<Vec2>> clipPaths(const std::vector<std::vector<Vec2>>& paths,
                                         const Regions& regions, bool inside,
                                         const ClipOptions& options) {
    c2::PathsD subjects;
    for (const auto& path : paths) {
        c2::PathD p;
        for (const Vec2& v : path) {
            p.emplace_back(v.x(), v.y());
        }
        if (p.size() >= 2) {
            subjects.push_back(std::move(p));
        }
    }
    if (subjects.empty()) {
        return {};
    }
    c2::ClipperD clipper(kDecimals);
    clipper.AddOpenSubject(subjects);
    clipper.AddClip(toPaths(regions, options.chordTolerance));
    c2::PathsD closed;
    c2::PathsD open;
    clipper.Execute(inside ? c2::ClipType::Intersection : c2::ClipType::Difference,
                    c2::FillRule::NonZero, closed, open);
    // Clipper does not promise the direction of open results: compare each piece with the
    // nearest segment of the inputs and turn it around when it runs the other way.
    std::vector<std::vector<Vec2>> out;
    for (const c2::PathD& piece : open) {
        std::vector<Vec2> pts;
        for (const auto& q : piece) {
            pts.emplace_back(q.x, q.y);
        }
        if (pts.size() < 2) {
            continue;
        }
        std::size_t k = 0;
        while (k + 2 < pts.size() && (pts[k + 1] - pts[k]).squaredNorm() < 1e-12) {
            ++k;
        }
        const Vec2 mid = 0.5 * (pts[k] + pts[k + 1]);
        const Vec2 dir = pts[k + 1] - pts[k];
        double best = std::numeric_limits<double>::infinity();
        double sense = 1.0;
        for (const auto& path : paths) {
            for (std::size_t i = 0; i + 1 < path.size(); ++i) {
                const Vec2 a = path[i];
                const Vec2 ab = path[i + 1] - a;
                const double len2 = ab.squaredNorm();
                if (len2 < 1e-18) {
                    continue;
                }
                const double t = std::clamp((mid - a).dot(ab) / len2, 0.0, 1.0);
                const double d = (a + t * ab - mid).squaredNorm();
                if (d < best) {
                    best = d;
                    sense = dir.dot(ab);
                }
            }
        }
        if (sense < 0.0) {
            std::reverse(pts.begin(), pts.end());
        }
        out.push_back(std::move(pts));
    }
    return out;
}

Regions unite(const Regions& a, const Regions& b, const ClipOptions& options) {
    return boolean(c2::ClipType::Union, a, b, options);
}

Regions intersect(const Regions& a, const Regions& b, const ClipOptions& options) {
    return boolean(c2::ClipType::Intersection, a, b, options);
}

Regions subtract(const Regions& a, const Regions& b, const ClipOptions& options) {
    return boolean(c2::ClipType::Difference, a, b, options);
}

} // namespace ocam::geom2d
