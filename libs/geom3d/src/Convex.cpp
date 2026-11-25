// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GJK distance: E. G. Gilbert, D. W. Johnson, S. S. Keerthi, "A fast procedure for computing the
// distance between complex objects in three-dimensional space", IEEE Journal of Robotics and
// Automation 4(2), 1988. The closest point of the simplex to the origin is found by Voronoi
// region tests (C. Ericson, Real-Time Collision Detection, 2005, sections 5.1 and 9.5).

#include "ocam/geom3d/Convex.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace ocam::geom3d {

Convex Convex::points(std::vector<Vec3> pts, double margin) {
    Convex c;
    c.kind_ = Kind::Points;
    c.points_ = std::move(pts);
    c.margin_ = margin;
    return c;
}

Convex Convex::box(const Aabb3& b) {
    std::vector<Vec3> pts;
    pts.reserve(8);
    for (int i = 0; i < 8; ++i) {
        pts.emplace_back((i & 1) ? b.max.x() : b.min.x(), (i & 2) ? b.max.y() : b.min.y(),
                         (i & 4) ? b.max.z() : b.min.z());
    }
    return points(std::move(pts));
}

Convex Convex::orientedBox(const Vec3& centre, const Mat3& axes, const Vec3& h) {
    std::vector<Vec3> pts;
    pts.reserve(8);
    for (int i = 0; i < 8; ++i) {
        pts.push_back(centre + axes.col(0) * ((i & 1) ? h.x() : -h.x()) +
                      axes.col(1) * ((i & 2) ? h.y() : -h.y()) +
                      axes.col(2) * ((i & 4) ? h.z() : -h.z()));
    }
    return points(std::move(pts));
}

Convex Convex::cylinder(const Vec3& base, const Vec3& axis, double radius, double length) {
    Convex c;
    c.kind_ = Kind::Cylinder;
    c.base_ = base;
    c.axis_ = axis.normalized();
    c.radius_ = radius;
    c.length_ = length;
    return c;
}

Convex Convex::capsule(const Vec3& a, const Vec3& b, double radius) {
    return points({a, b}, radius);
}

Convex Convex::triangle(const Vec3& a, const Vec3& b, const Vec3& c) {
    return points({a, b, c});
}

Convex Convex::sphere(const Vec3& centre, double radius) {
    return points({centre}, radius);
}

Vec3 Convex::support(const Vec3& d) const {
    if (kind_ == Kind::Cylinder) {
        const double along = d.dot(axis_);
        Vec3 p = base_ + (along > 0.0 ? length_ : 0.0) * axis_;
        const Vec3 perp = d - along * axis_;
        const double n = perp.norm();
        if (n > 1e-12) {
            p += perp * (radius_ / n);
        }
        return p;
    }
    std::size_t best = 0;
    double bestDot = -std::numeric_limits<double>::infinity();
    for (std::size_t k = 0; k < points_.size(); ++k) {
        const double v = points_[k].dot(d);
        if (v > bestDot) {
            bestDot = v;
            best = k;
        }
    }
    return points_.empty() ? Vec3::Zero() : points_[best];
}

Aabb3 Convex::bounds() const {
    Aabb3 b;
    if (kind_ == Kind::Cylinder) {
        // Extent of a disk of radius r with normal a along axis i: r * sqrt(1 - a_i^2).
        const Vec3 e(radius_ * std::sqrt(std::max(0.0, 1.0 - axis_.x() * axis_.x())),
                     radius_ * std::sqrt(std::max(0.0, 1.0 - axis_.y() * axis_.y())),
                     radius_ * std::sqrt(std::max(0.0, 1.0 - axis_.z() * axis_.z())));
        for (const Vec3& c : {base_, Vec3(base_ + length_ * axis_)}) {
            b.extend(c - e);
            b.extend(c + e);
        }
    } else {
        for (const Vec3& p : points_) {
            b.extend(p);
        }
    }
    if (!b.isEmpty() && margin_ > 0.0) {
        b.min -= Vec3::Constant(margin_);
        b.max += Vec3::Constant(margin_);
    }
    return b;
}

Convex Convex::transformed(const Transform3& xf) const {
    Convex c = *this;
    for (Vec3& p : c.points_) {
        p = xf * p;
    }
    c.base_ = xf * base_;
    c.axis_ = (xf.linear() * axis_).normalized();
    return c;
}

namespace {

struct Vertex {
    Vec3 w; ///< a - b
    Vec3 a;
    Vec3 b;
};

struct Simplex {
    std::array<Vertex, 4> v;
    std::array<double, 4> bary{};
    int n = 0;
};

/// Keeps vertices `idx` (in order) with their barycentric weights.
void keep(Simplex& s, std::initializer_list<int> idx, std::initializer_list<double> weights) {
    std::array<Vertex, 4> v{};
    std::array<double, 4> w{};
    int k = 0;
    auto wi = weights.begin();
    for (const int i : idx) {
        v[static_cast<std::size_t>(k)] = s.v[static_cast<std::size_t>(i)];
        w[static_cast<std::size_t>(k)] = *wi++;
        ++k;
    }
    s.v = v;
    s.bary = w;
    s.n = k;
}

/// Closest point to the origin of triangle (i, j, k) of the simplex: barycentric weights and
/// the vertices of the feature it lies on (Ericson, ClosestPtPointTriangle with p = 0).
struct TriResult {
    std::array<int, 3> idx{};
    std::array<double, 3> w{};
    int n = 0;
    Vec3 point = Vec3::Zero();
};

TriResult closestOnTriangle(const Simplex& s, int i, int j, int k) {
    const Vec3& a = s.v[static_cast<std::size_t>(i)].w;
    const Vec3& b = s.v[static_cast<std::size_t>(j)].w;
    const Vec3& c = s.v[static_cast<std::size_t>(k)].w;
    const Vec3 ab = b - a, ac = c - a, ap = -a;
    TriResult r;
    const double d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        r.idx = {i};
        r.w = {1.0};
        r.n = 1;
        r.point = a;
        return r;
    }
    const Vec3 bp = -b;
    const double d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) {
        r.idx = {j};
        r.w = {1.0};
        r.n = 1;
        r.point = b;
        return r;
    }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double t = d1 / (d1 - d3);
        r.idx = {i, j};
        r.w = {1.0 - t, t};
        r.n = 2;
        r.point = a + t * ab;
        return r;
    }
    const Vec3 cp = -c;
    const double d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) {
        r.idx = {k};
        r.w = {1.0};
        r.n = 1;
        r.point = c;
        return r;
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double t = d2 / (d2 - d6);
        r.idx = {i, k};
        r.w = {1.0 - t, t};
        r.n = 2;
        r.point = a + t * ac;
        return r;
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const double t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        r.idx = {j, k};
        r.w = {1.0 - t, t};
        r.n = 2;
        r.point = b + t * (c - b);
        return r;
    }
    const double denom = 1.0 / (va + vb + vc);
    const double v = vb * denom, w = vc * denom;
    r.idx = {i, j, k};
    r.w = {1.0 - v - w, v, w};
    r.n = 3;
    r.point = a + ab * v + ac * w;
    return r;
}

void apply(Simplex& s, const TriResult& r) {
    if (r.n == 1) {
        keep(s, {r.idx[0]}, {r.w[0]});
    } else if (r.n == 2) {
        keep(s, {r.idx[0], r.idx[1]}, {r.w[0], r.w[1]});
    } else {
        keep(s, {r.idx[0], r.idx[1], r.idx[2]}, {r.w[0], r.w[1], r.w[2]});
    }
}

/// Reduces the simplex to the feature closest to the origin; returns that point. Returns
/// false when a tetrahedron contains the origin.
bool solve(Simplex& s, Vec3& closest) {
    switch (s.n) {
    case 1:
        s.bary[0] = 1.0;
        closest = s.v[0].w;
        return true;
    case 2: {
        const Vec3 a = s.v[0].w; // a copy: keep() below overwrites the simplex
        const Vec3 ab = s.v[1].w - a;
        const double len2 = ab.squaredNorm();
        const double t = len2 > 0.0 ? std::clamp(-a.dot(ab) / len2, 0.0, 1.0) : 0.0;
        if (t <= 0.0) {
            keep(s, {0}, {1.0});
        } else if (t >= 1.0) {
            keep(s, {1}, {1.0});
        } else {
            s.bary[0] = 1.0 - t;
            s.bary[1] = t;
        }
        closest = a + t * ab;
        return true;
    }
    case 3: {
        const TriResult r = closestOnTriangle(s, 0, 1, 2);
        apply(s, r);
        closest = r.point;
        return true;
    }
    default: {
        // Faces of the tetrahedron with the opposite vertex: the origin is outside a face when
        // it lies on the other side from that vertex.
        const std::array<std::array<int, 4>, 4> faces{
            {{0, 1, 2, 3}, {0, 1, 3, 2}, {0, 2, 3, 1}, {1, 2, 3, 0}}};
        // A flat tetrahedron (for example a parallelogram of two segment ends and two triangle
        // corners) has no reliable inside: its hull is the union of its four triangles.
        const Vec3& p0 = s.v[0].w;
        const Vec3 e1 = s.v[1].w - p0, e2 = s.v[2].w - p0, e3 = s.v[3].w - p0;
        const double scale = std::max({e1.norm(), e2.norm(), e3.norm(), 1e-300});
        const bool flat = std::abs(e1.dot(e2.cross(e3))) <= 1e-10 * scale * scale * scale;
        bool outside = false;
        double best = std::numeric_limits<double>::infinity();
        TriResult bestR;
        for (const auto& f : faces) {
            const Vec3& a = s.v[static_cast<std::size_t>(f[0])].w;
            const Vec3& b = s.v[static_cast<std::size_t>(f[1])].w;
            const Vec3& c = s.v[static_cast<std::size_t>(f[2])].w;
            const Vec3& d = s.v[static_cast<std::size_t>(f[3])].w;
            const Vec3 n = (b - a).cross(c - a);
            const double sideOrigin = n.dot(-a);
            const double sideD = n.dot(d - a);
            if (flat || sideOrigin * sideD < 0.0) {
                outside = true;
                const TriResult r = closestOnTriangle(s, f[0], f[1], f[2]);
                const double dist = r.point.squaredNorm();
                if (dist < best) {
                    best = dist;
                    bestR = r;
                }
            }
        }
        if (!outside || (flat && best < 1e-24)) {
            return false;
        }
        apply(s, bestR);
        closest = bestR.point;
        return true;
    }
    }
}

Vertex supportVertex(const Convex& a, const Convex& b, const Vec3& d) {
    Vertex v;
    v.a = a.support(d);
    v.b = b.support(-d);
    v.w = v.a - v.b;
    return v;
}

} // namespace

ConvexDistance distance(const Convex& a, const Convex& b) {
    ConvexDistance out;
    if (a.isEmpty() || b.isEmpty()) {
        out.distance = std::numeric_limits<double>::infinity();
        return out;
    }
    Simplex s;
    Vec3 d = a.bounds().center() - b.bounds().center();
    if (d.squaredNorm() < 1e-24) {
        d = Vec3::UnitX();
    }
    s.v[0] = supportVertex(a, b, -d);
    s.n = 1;
    s.bary[0] = 1.0;
    Vec3 v = s.v[0].w;
    bool core = false; // the solids without margins overlap
    for (int iter = 0; iter < 128; ++iter) {
        const double vv = v.squaredNorm();
        if (vv < 1e-24) {
            core = true;
            break;
        }
        const Vertex w = supportVertex(a, b, -v);
        // Converged: the new support point does not get closer to the origin.
        if (vv - v.dot(w.w) <= 1e-12 * vv + 1e-24) {
            break;
        }
        bool duplicate = false;
        for (int k = 0; k < s.n; ++k) {
            duplicate = duplicate || (s.v[static_cast<std::size_t>(k)].w - w.w).squaredNorm() <
                                         1e-24 * std::max(1.0, vv);
        }
        if (duplicate) {
            break;
        }
        s.v[static_cast<std::size_t>(s.n++)] = w;
        Vec3 next;
        if (!solve(s, next)) {
            core = true;
            break;
        }
        if (next.squaredNorm() >= vv) {
            v = next; // no progress (numerical): stop with the current estimate
            break;
        }
        v = next;
    }
    Vec3 pa = Vec3::Zero(), pb = Vec3::Zero();
    for (int k = 0; k < s.n; ++k) {
        pa += s.bary[static_cast<std::size_t>(k)] * s.v[static_cast<std::size_t>(k)].a;
        pb += s.bary[static_cast<std::size_t>(k)] * s.v[static_cast<std::size_t>(k)].b;
    }
    const double margins = a.margin() + b.margin();
    if (core) {
        out.intersecting = true;
        out.distance = 0.0;
        out.pointA = out.pointB = 0.5 * (pa + pb);
        return out;
    }
    const double coreDist = v.norm();
    const Vec3 dir = coreDist > 0.0 ? Vec3((pb - pa) / coreDist) : Vec3::UnitX();
    if (coreDist <= margins) {
        out.intersecting = true;
        out.distance = 0.0;
        out.pointA = out.pointB = pa + dir * (a.margin() / std::max(margins, 1e-300) * coreDist);
        return out;
    }
    out.distance = coreDist - margins;
    out.pointA = pa + dir * a.margin();
    out.pointB = pb - dir * b.margin();
    return out;
}

bool intersects(const Convex& a, const Convex& b, double clearance) {
    const ConvexDistance d = distance(a, b);
    return d.intersecting || d.distance < clearance;
}

} // namespace ocam::geom3d
