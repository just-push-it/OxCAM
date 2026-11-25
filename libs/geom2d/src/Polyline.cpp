// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/geom2d/Polyline.hpp"

#include <algorithm>
#include <cmath>

namespace ocam::geom2d {
namespace {

constexpr double kStraightBulge = 1e-12;

double cross(const Vec2& a, const Vec2& b) noexcept {
    return a.x() * b.y() - a.y() * b.x();
}

// Signed area between an arc and its chord: r^2/2 * (theta - sin theta), sign of theta.
double arcSegmentArea(const Arc& arc) noexcept {
    return 0.5 * arc.radius * arc.radius * (arc.sweep - std::sin(arc.sweep));
}

// True when angle `a` lies on the arc starting at `start` sweeping `sweep`.
bool angleOnArc(double a, double start, double sweep) noexcept {
    const double twoPi = 2.0 * kPi;
    double d = sweep >= 0.0 ? a - start : start - a;
    d = std::fmod(d, twoPi);
    if (d < 0.0) {
        d += twoPi;
    }
    return d <= std::abs(sweep);
}

} // namespace

Vec2 Arc::pointAt(double t) const {
    const double a = startAngle + sweep * t;
    return center + radius * Vec2(std::cos(a), std::sin(a));
}

std::optional<Arc> arcFromBulge(const Vec2& p0, const Vec2& p1, double bulge) noexcept {
    if (std::abs(bulge) < kStraightBulge) {
        return std::nullopt;
    }
    const Vec2 chord = p1 - p0;
    const double c = chord.norm();
    if (c <= 0.0) {
        return std::nullopt;
    }
    const Vec2 leftNormal(-chord.y() / c, chord.x() / c);
    const Vec2 mid = 0.5 * (p0 + p1);

    Arc arc;
    arc.sweep = 4.0 * std::atan(bulge);
    arc.radius = c * (1.0 + bulge * bulge) / (4.0 * std::abs(bulge));
    arc.center = mid + leftNormal * (c * (1.0 - bulge * bulge) / (4.0 * bulge));
    arc.startAngle = std::atan2(p0.y() - arc.center.y(), p0.x() - arc.center.x());
    return arc;
}

double bulgeFromSweep(double sweep) noexcept {
    return std::tan(sweep / 4.0);
}

Polyline Polyline::fromPoints(const std::vector<Vec2>& points, bool closed) {
    std::vector<Vertex> v;
    v.reserve(points.size());
    for (const auto& p : points) {
        v.push_back({p, 0.0});
    }
    return {std::move(v), closed};
}

Polyline Polyline::circle(const Vec2& center, double radius) {
    return {{{center + Vec2(radius, 0.0), 1.0}, {center - Vec2(radius, 0.0), 1.0}}, true};
}

Polyline Polyline::rectangle(const Vec2& min, const Vec2& max) {
    return fromPoints({min, {max.x(), min.y()}, max, {min.x(), max.y()}}, true);
}

std::size_t Polyline::segmentCount() const noexcept {
    if (vertices_.size() < 2) {
        return 0;
    }
    return closed_ ? vertices_.size() : vertices_.size() - 1;
}

double Polyline::signedArea() const noexcept {
    if (!closed_ || vertices_.size() < 2) {
        return 0.0;
    }
    double twiceArea = 0.0;
    double arcs = 0.0;
    const std::size_t n = vertices_.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vertex& a = vertices_[i];
        const Vec2& b = vertices_[(i + 1) % n].p;
        twiceArea += cross(a.p, b);
        if (auto arc = arcFromBulge(a.p, b, a.bulge)) {
            arcs += arcSegmentArea(*arc);
        }
    }
    return 0.5 * twiceArea + arcs;
}

double Polyline::area() const noexcept {
    return std::abs(signedArea());
}

double Polyline::length() const noexcept {
    double len = 0.0;
    const std::size_t n = vertices_.size();
    for (std::size_t i = 0; i < segmentCount(); ++i) {
        const Vertex& a = vertices_[i];
        const Vec2& b = vertices_[(i + 1) % n].p;
        if (auto arc = arcFromBulge(a.p, b, a.bulge)) {
            len += arc->radius * std::abs(arc->sweep);
        } else {
            len += (b - a.p).norm();
        }
    }
    return len;
}

Aabb2 Polyline::bounds() const noexcept {
    Aabb2 box;
    const std::size_t n = vertices_.size();
    for (const auto& v : vertices_) {
        box.extend(v.p);
    }
    for (std::size_t i = 0; i < segmentCount(); ++i) {
        const Vertex& a = vertices_[i];
        const Vec2& b = vertices_[(i + 1) % n].p;
        if (auto arc = arcFromBulge(a.p, b, a.bulge)) {
            for (int q = 0; q < 4; ++q) {
                const double angle = q * kPi / 2.0;
                if (angleOnArc(angle, arc->startAngle, arc->sweep)) {
                    box.extend(arc->center + arc->radius * Vec2(std::cos(angle), std::sin(angle)));
                }
            }
        }
    }
    return box;
}

void Polyline::reverse() {
    const std::size_t n = vertices_.size();
    if (n < 2) {
        return;
    }
    // Segment i (vertex i -> i+1) becomes a segment starting at vertex i+1 with negated bulge.
    std::vector<Vertex> out(n);
    if (closed_) {
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t src = (n - i) % n;       // vertex order: 0, n-1, ..., 1
            const std::size_t seg = (src + n - 1) % n; // segment ending at src
            out[i] = {vertices_[src].p, -vertices_[seg].bulge};
        }
    } else {
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t src = n - 1 - i;
            out[i].p = vertices_[src].p;
            out[i].bulge = src > 0 ? -vertices_[src - 1].bulge : 0.0;
        }
    }
    vertices_ = std::move(out);
}

std::vector<Vec2> Polyline::discretize(double chordTolerance) const {
    std::vector<Vec2> pts;
    const std::size_t n = vertices_.size();
    if (n == 0) {
        return pts;
    }
    const double tol = std::max(chordTolerance, 1e-9);
    for (std::size_t i = 0; i < segmentCount(); ++i) {
        const Vertex& a = vertices_[i];
        const Vec2& b = vertices_[(i + 1) % n].p;
        pts.push_back(a.p);
        if (auto arc = arcFromBulge(a.p, b, a.bulge)) {
            // Max sagitta of a chord spanning angle phi: r (1 - cos(phi/2)) <= tol.
            const double ratio = std::clamp(1.0 - tol / arc->radius, -1.0, 1.0);
            const double maxStep = std::max(2.0 * std::acos(ratio), 1e-6);
            const int steps =
                std::max(1, static_cast<int>(std::ceil(std::abs(arc->sweep) / maxStep)));
            for (int s = 1; s < steps; ++s) {
                pts.push_back(arc->pointAt(static_cast<double>(s) / steps));
            }
        }
    }
    if (!closed_) {
        pts.push_back(vertices_.back().p);
    }
    return pts;
}

bool Polyline::contains(const Vec2& point, double chordTolerance) const {
    if (!closed_) {
        return false;
    }
    const auto pts = discretize(chordTolerance);
    int winding = 0;
    const std::size_t n = pts.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2& a = pts[i];
        const Vec2& b = pts[(i + 1) % n];
        const double side = cross(b - a, point - a);
        if (a.y() <= point.y()) {
            if (b.y() > point.y() && side > 0.0) {
                ++winding;
            }
        } else if (b.y() <= point.y() && side < 0.0) {
            --winding;
        }
    }
    return winding != 0;
}

} // namespace ocam::geom2d
