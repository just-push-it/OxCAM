// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/geom2d/PathOps.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace ocam::geom2d {
namespace {

double cross(const Vec2& a, const Vec2& b) {
    return a.x() * b.y() - a.y() * b.x();
}

double wrapAngle(double a) {
    while (a > kPi) {
        a -= 2.0 * kPi;
    }
    while (a <= -kPi) {
        a += 2.0 * kPi;
    }
    return a;
}

// Circle through three points, or nullopt if (nearly) collinear.
std::optional<std::pair<Vec2, double>> circleThrough(const Vec2& a, const Vec2& b, const Vec2& c) {
    const double d = 2.0 * cross(b - a, c - a);
    if (std::abs(d) < 1e-12) {
        return std::nullopt;
    }
    const double ab = (b - a).squaredNorm();
    const double ac = (c - a).squaredNorm();
    const Vec2 ba = b - a;
    const Vec2 ca = c - a;
    const Vec2 center = a + Vec2(ca.y() * ab - ba.y() * ac, ba.x() * ac - ca.x() * ab) / d;
    return std::make_pair(center, (a - center).norm());
}

double distanceToSegment(const Vec2& p, const Vec2& a, const Vec2& b, double* tOut = nullptr) {
    const Vec2 ab = b - a;
    const double len2 = ab.squaredNorm();
    const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    if (tOut) {
        *tOut = t;
    }
    return (a + ab * t - p).norm();
}

// Do points[i..j] lie on the segment points[i]-points[j] within tol?
bool fitsLine(const std::vector<Vec2>& pts, std::size_t i, std::size_t j, double tol) {
    for (std::size_t k = i + 1; k < j; ++k) {
        if (distanceToSegment(pts[k], pts[i], pts[j]) > tol) {
            return false;
        }
    }
    return true;
}

// Do points[i..j] lie on one circular arc of at most 180 degrees within tol? Returns its sweep.
std::optional<double> fitsArc(const std::vector<Vec2>& pts, std::size_t i, std::size_t j,
                              double tol) {
    if (j < i + 2) {
        return std::nullopt;
    }
    const auto circle = circleThrough(pts[i], pts[(i + j) / 2], pts[j]);
    if (!circle) {
        return std::nullopt;
    }
    const auto& [c, r] = *circle;
    if (r > 1e5) {
        return std::nullopt; // effectively straight
    }
    double sweep = 0.0;
    double prevAngle = std::atan2(pts[i].y() - c.y(), pts[i].x() - c.x());
    for (std::size_t k = i + 1; k <= j; ++k) {
        if (std::abs((pts[k] - c).norm() - r) > tol) {
            return std::nullopt;
        }
        const double a = std::atan2(pts[k].y() - c.y(), pts[k].x() - c.x());
        const double step = wrapAngle(a - prevAngle);
        if (sweep != 0.0 && step * sweep < 0.0) {
            return std::nullopt; // direction reversal
        }
        // The chord between consecutive points must stay close to the arc too.
        const double sagitta = r * (1.0 - std::cos(step / 2.0));
        if (sagitta > tol) {
            return std::nullopt;
        }
        sweep += step;
        prevAngle = a;
    }
    if (std::abs(sweep) > kPi + 1e-9) {
        return std::nullopt;
    }
    return sweep;
}

} // namespace

Polyline fitArcs(const std::vector<Vec2>& input, bool closed, double tolerance) {
    std::vector<Vec2> pts;
    pts.reserve(input.size() + 1);
    for (const Vec2& p : input) {
        if (pts.empty() || (p - pts.back()).norm() > 1e-12) {
            pts.push_back(p);
        }
    }
    if (closed && pts.size() > 1 && (pts.front() - pts.back()).norm() <= 1e-12) {
        pts.pop_back();
    }
    if (pts.size() < 2) {
        return Polyline::fromPoints(pts, closed);
    }
    if (closed) {
        pts.push_back(pts.front()); // fit the closing segment like any other
    }

    std::vector<Vertex> out;
    std::size_t i = 0;
    const std::size_t last = pts.size() - 1;
    while (i < last) {
        // Longest line from i.
        std::size_t lineEnd = i + 1;
        while (lineEnd < last && fitsLine(pts, i, lineEnd + 1, tolerance)) {
            ++lineEnd;
        }
        // Longest arc from i.
        std::size_t arcEnd = i;
        double arcSweep = 0.0;
        for (std::size_t j = i + 2; j <= last; ++j) {
            if (auto sweep = fitsArc(pts, i, j, tolerance)) {
                arcEnd = j;
                arcSweep = *sweep;
            } else if (j > arcEnd + 2) {
                break; // allow a couple of misses (noisy middles), then give up
            }
        }
        if (arcEnd > lineEnd) {
            out.push_back({pts[i], bulgeFromSweep(arcSweep)});
            i = arcEnd;
        } else {
            out.push_back({pts[i], 0.0});
            i = lineEnd;
        }
    }
    if (!closed) {
        out.push_back({pts.back(), 0.0});
    }
    return {std::move(out), closed};
}

ClosestPoint closestPoint(const Polyline& path, const Vec2& p) {
    ClosestPoint best;
    best.distance = std::numeric_limits<double>::infinity();
    const auto& v = path.vertices();
    const std::size_t n = v.size();
    for (std::size_t i = 0; i < path.segmentCount(); ++i) {
        const Vec2& a = v[i].p;
        const Vec2& b = v[(i + 1) % n].p;
        ClosestPoint cand;
        cand.segment = i;
        if (auto arc = arcFromBulge(a, b, v[i].bulge)) {
            // Project onto the circle, then clamp to the arc's angular range.
            const double angle = std::atan2(p.y() - arc->center.y(), p.x() - arc->center.x());
            double rel = wrapAngle(angle - arc->startAngle);
            if (arc->sweep > 0 && rel < 0) {
                rel += 2.0 * kPi;
            } else if (arc->sweep < 0 && rel > 0) {
                rel -= 2.0 * kPi;
            }
            double t = rel / arc->sweep;
            if (t < 0.0 || t > 1.0) {
                // Outside the arc: nearest endpoint.
                t = (p - a).norm() <= (p - b).norm() ? 0.0 : 1.0;
            }
            cand.t = t;
            cand.point = arc->pointAt(t);
        } else {
            double t = 0.0;
            distanceToSegment(p, a, b, &t);
            cand.t = t;
            cand.point = a + (b - a) * t;
        }
        cand.distance = (cand.point - p).norm();
        if (cand.distance < best.distance) {
            best = cand;
        }
    }
    return best;
}

Polyline restartAtNearest(const Polyline& loop, const Vec2& p) {
    const auto& v = loop.vertices();
    if (!loop.isClosed() || v.size() < 2) {
        return loop;
    }
    const ClosestPoint cp = closestPoint(loop, p);
    const std::size_t n = v.size();
    std::size_t startVertex = cp.segment;
    std::vector<Vertex> out;

    if (cp.t > 1e-9 && cp.t < 1.0 - 1e-9) {
        // Split segment cp.segment at cp.t: new start vertex is the split point.
        const Vertex& a = v[cp.segment];
        const Vec2& b = v[(cp.segment + 1) % n].p;
        double firstBulge = 0.0;
        double secondBulge = 0.0;
        if (auto arc = arcFromBulge(a.p, b, a.bulge)) {
            firstBulge = bulgeFromSweep(arc->sweep * cp.t);
            secondBulge = bulgeFromSweep(arc->sweep * (1.0 - cp.t));
        }
        out.push_back({cp.point, secondBulge});
        for (std::size_t k = 1; k < n; ++k) {
            out.push_back(v[(cp.segment + k) % n]);
        }
        out.push_back({a.p, firstBulge});
        return {std::move(out), true};
    }
    if (cp.t >= 1.0 - 1e-9) {
        startVertex = (cp.segment + 1) % n;
    }
    for (std::size_t k = 0; k < n; ++k) {
        out.push_back(v[(startVertex + k) % n]);
    }
    return {std::move(out), true};
}

Vec2 interiorPoint(const Region& region, double chordTolerance) {
    const auto outer = region.outer.discretize(chordTolerance);
    // Polygon centroid of the outer boundary.
    double a2 = 0.0;
    Vec2 c = Vec2::Zero();
    for (std::size_t i = 0; i < outer.size(); ++i) {
        const Vec2& p = outer[i];
        const Vec2& q = outer[(i + 1) % outer.size()];
        const double w = cross(p, q);
        a2 += w;
        c += (p + q) * w;
    }
    if (std::abs(a2) > 1e-15) {
        c /= 3.0 * a2;
        if (region.contains(c, chordTolerance)) {
            return c;
        }
    }
    // Widest interior span on the horizontal line through the bounding-box centre.
    const Aabb2 box = region.bounds();
    double bestWidth = -1.0;
    Vec2 best = box.center();
    for (double frac : {0.5, 0.37, 0.63, 0.25, 0.75, 0.13, 0.87}) {
        const double y = box.min.y() + (box.max.y() - box.min.y()) * frac;
        std::vector<double> xs;
        auto collect = [&](const Polyline& loop) {
            const auto pts = loop.discretize(chordTolerance);
            for (std::size_t i = 0; i < pts.size(); ++i) {
                const Vec2& p = pts[i];
                const Vec2& q = pts[(i + 1) % pts.size()];
                if ((p.y() <= y) != (q.y() <= y)) {
                    xs.push_back(p.x() + (y - p.y()) * (q.x() - p.x()) / (q.y() - p.y()));
                }
            }
        };
        collect(region.outer);
        for (const auto& h : region.holes) {
            collect(h);
        }
        std::sort(xs.begin(), xs.end());
        for (std::size_t i = 0; i + 1 < xs.size(); i += 2) {
            if (xs[i + 1] - xs[i] > bestWidth) {
                bestWidth = xs[i + 1] - xs[i];
                best = Vec2(0.5 * (xs[i] + xs[i + 1]), y);
            }
        }
        if (bestWidth > 0.0) {
            break;
        }
    }
    return best;
}

bool segmentInside(const Regions& regions, const Vec2& a, const Vec2& b, double chordTolerance) {
    const Vec2 mid = 0.5 * (a + b);
    const auto inside = [&](const Vec2& p) {
        return std::any_of(regions.begin(), regions.end(),
                           [&](const Region& r) { return r.contains(p, chordTolerance); });
    };
    if (!inside(a) || !inside(b) || !inside(mid)) {
        return false;
    }
    const Vec2 d = b - a;
    auto crosses = [&](const Polyline& loop) {
        const auto pts = loop.discretize(chordTolerance);
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const Vec2& p = pts[i];
            const Vec2& q = pts[(i + 1) % pts.size()];
            const Vec2 e = q - p;
            const double denom = cross(d, e);
            if (std::abs(denom) < 1e-15) {
                continue;
            }
            const double t = cross(p - a, e) / denom;
            const double u = cross(p - a, d) / denom;
            if (t > 1e-9 && t < 1.0 - 1e-9 && u >= 0.0 && u <= 1.0) {
                return true;
            }
        }
        return false;
    };
    for (const Region& r : regions) {
        if (crosses(r.outer) || std::any_of(r.holes.begin(), r.holes.end(),
                                            [&](const Polyline& h) { return crosses(h); })) {
            return false;
        }
    }
    return true;
}

} // namespace ocam::geom2d
