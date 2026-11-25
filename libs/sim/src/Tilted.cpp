// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "Tilted.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace ocam::sim::detail {
namespace {

/// c_s s + c_t t + c_0 <= 0
struct Line {
    double cs = 0.0, ct = 0.0, c0 = 0.0;
    double at(double s, double t) const noexcept { return cs * s + ct * t + c0; }
};

/// Q(s, t) = Ass s^2 + 2 Ast s t + Att t^2 + 2 Bs s + 2 Bt t + C
struct Quadric {
    double ass = 0.0, ast = 0.0, att = 0.0, bs = 0.0, bt = 0.0, c = 0.0;
    double at(double s, double t) const noexcept {
        return ass * s * s + 2.0 * ast * s * t + att * t * t + 2.0 * bs * s + 2.0 * bt * t + c;
    }
};

/// Points where the line meets Q = 0 (up to two).
int lineQuadric(const Line& l, const Quadric& q, std::array<Vec2, 2>& out) {
    Vec2 p, d;
    if (std::abs(l.ct) >= std::abs(l.cs)) {
        if (std::abs(l.ct) < 1e-300) {
            return 0;
        }
        p = Vec2(0.0, -l.c0 / l.ct);
        d = Vec2(1.0, -l.cs / l.ct);
    } else {
        p = Vec2(-l.c0 / l.cs, 0.0);
        d = Vec2(-l.ct / l.cs, 1.0);
    }
    const double qa = q.ass * d.x() * d.x() + 2.0 * q.ast * d.x() * d.y() + q.att * d.y() * d.y();
    const double ax = q.ass * p.x() + q.ast * p.y() + q.bs;
    const double ay = q.ast * p.x() + q.att * p.y() + q.bt;
    const double qb = d.x() * ax + d.y() * ay;
    const double qc = q.at(p.x(), p.y());
    if (std::abs(qa) < 1e-14) {
        if (std::abs(qb) < 1e-14) {
            return 0;
        }
        out[0] = p - qc / (2.0 * qb) * d;
        return 1;
    }
    double disc = qb * qb - qa * qc;
    if (disc < 0.0) {
        if (disc < -1e-12 * std::max(1.0, qb * qb)) {
            return 0;
        }
        disc = 0.0;
    }
    const double sq = std::sqrt(disc);
    out[0] = p + ((-qb - sq) / qa) * d;
    out[1] = p + ((-qb + sq) / qa) * d;
    return 2;
}

double radiusAt(const SolidPiece& p, double h) {
    if (p.sphere > 0.0) {
        const double dh = h - p.centre;
        return std::sqrt(std::max(0.0, p.sphere * p.sphere - dh * dh));
    }
    return p.h1 > p.h0 ? p.r0 + (p.r1 - p.r0) * (h - p.h0) / (p.h1 - p.h0) : p.r0;
}

} // namespace

bool sweptSpan(const SolidPiece& piece, const Vec3& p0, const Vec3& d, const Vec3& u,
               const Vec3& origin, int axis, double& lo, double& hi) {
    const Vec3 E = Vec3::Unit(axis);
    const Vec3 w0 = origin - p0;
    const Vec3 e = -d;
    const double hw = w0.dot(u), hE = E.dot(u), he = e.dot(u);
    double alpha = 0.0, beta = 0.0, gamma = 0.0;
    if (piece.sphere > 0.0) {
        beta = piece.centre;
        gamma = piece.sphere * piece.sphere - piece.centre * piece.centre;
    } else {
        const double b = piece.h1 > piece.h0 ? (piece.r1 - piece.r0) / (piece.h1 - piece.h0) : 0.0;
        const double a = piece.r0 - b * piece.h0;
        alpha = 1.0 + b * b;
        beta = a * b;
        gamma = a * a;
    }
    Quadric q;
    q.ass = 1.0 - alpha * hE * hE;
    q.ast = E.dot(e) - alpha * hE * he;
    q.att = e.dot(e) - alpha * he * he;
    q.bs = w0.dot(E) - alpha * hw * hE - beta * hE;
    q.bt = w0.dot(e) - alpha * hw * he - beta * he;
    q.c = w0.dot(w0) - alpha * hw * hw - 2.0 * beta * hw - gamma;

    const std::array<Line, 4> lines{Line{-hE, -he, piece.h0 - hw}, Line{hE, he, hw - piece.h1},
                                    Line{0.0, -1.0, 0.0}, Line{0.0, 1.0, -1.0}};
    constexpr double epsQ = 1e-7;
    constexpr double epsL = 1e-9;
    lo = std::numeric_limits<double>::infinity();
    hi = -lo;
    auto consider = [&](const Vec2& st, bool onQuadric) {
        if (!std::isfinite(st.x()) || !std::isfinite(st.y())) {
            return;
        }
        if (!onQuadric && q.at(st.x(), st.y()) > epsQ) {
            return;
        }
        for (const Line& l : lines) {
            if (l.at(st.x(), st.y()) > epsL) {
                return;
            }
        }
        lo = std::min(lo, st.x());
        hi = std::max(hi, st.x());
    };
    std::array<Vec2, 2> pts;
    // Extremes of s on the quadric itself (where its tangent runs along s).
    if (std::abs(q.ast) + std::abs(q.att) > 1e-15) {
        const int n = lineQuadric(Line{q.ast, q.att, q.bt}, q, pts);
        for (int k = 0; k < n; ++k) {
            consider(pts[static_cast<std::size_t>(k)], true);
        }
    }
    for (const Line& l : lines) {
        const int n = lineQuadric(l, q, pts);
        for (int k = 0; k < n; ++k) {
            consider(pts[static_cast<std::size_t>(k)], true);
        }
    }
    // Corners of the height and parameter limits.
    for (int a = 0; a < 2; ++a) {
        for (int b = 2; b < 4; ++b) {
            const Line& l1 = lines[static_cast<std::size_t>(a)];
            const Line& l2 = lines[static_cast<std::size_t>(b)];
            const double det = l1.cs * l2.ct - l1.ct * l2.cs;
            if (std::abs(det) < 1e-300) {
                continue;
            }
            const double s = (-l1.c0 * l2.ct + l1.ct * l2.c0) / det;
            const double t = (-l1.cs * l2.c0 + l1.c0 * l2.cs) / det;
            consider(Vec2(s, t), false);
        }
    }
    return lo <= hi;
}

geom3d::Aabb3 piecesBounds(const std::vector<SolidPiece>& pieces, const Vec3& p, const Vec3& u) {
    geom3d::Aabb3 box;
    const Vec3 ext(std::sqrt(std::max(0.0, 1.0 - u.x() * u.x())),
                   std::sqrt(std::max(0.0, 1.0 - u.y() * u.y())),
                   std::sqrt(std::max(0.0, 1.0 - u.z() * u.z())));
    auto disk = [&](double h, double r) {
        const Vec3 c = p + h * u;
        box.extend(c - r * ext);
        box.extend(c + r * ext);
    };
    for (const SolidPiece& s : pieces) {
        disk(s.h0, radiusAt(s, s.h0));
        disk(s.h1, radiusAt(s, s.h1));
        if (s.sphere > 0.0 && s.centre > s.h0 && s.centre < s.h1) {
            disk(s.centre, s.sphere);
        }
    }
    return box;
}

std::vector<SolidPiece> shrink(const std::vector<SolidPiece>& pieces, double amount) {
    std::vector<SolidPiece> out;
    for (SolidPiece s : pieces) {
        if (s.sphere > 0.0) {
            s.sphere = std::max(0.0, s.sphere - amount);
            if (s.sphere <= 0.0) {
                continue;
            }
        } else {
            s.r0 = std::max(0.0, s.r0 - amount);
            s.r1 = std::max(0.0, s.r1 - amount);
            if (s.r0 <= 0.0 && s.r1 <= 0.0) {
                continue;
            }
        }
        out.push_back(s);
    }
    return out;
}

} // namespace ocam::sim::detail
