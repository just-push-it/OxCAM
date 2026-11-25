// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Marching squares: W. E. Lorensen, H. E. Cline, "Marching cubes: a high resolution 3D surface
// construction algorithm", SIGGRAPH 1987 (the 2D case of the same table).

#include "ocam/geom2d/Contour.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

namespace ocam::geom2d {

Regions isoRegions(const ScalarGrid& g, double level) {
    return isoRegions(g, level, {}, 0.0);
}

Regions isoRegions(const ScalarGrid& g, double level, const std::function<bool(const Vec2&)>& exact,
                   double tolerance) {
    const int nx = g.nx, ny = g.ny;
    if (nx <= 0 || ny <= 0 || g.values.size() < static_cast<std::size_t>(nx) * ny) {
        return {};
    }
    auto value = [&](int i, int j) {
        if (i < 0 || j < 0 || i >= nx || j >= ny) {
            return -std::numeric_limits<double>::infinity();
        }
        return g.at(i, j) - level;
    };
    auto inside = [&](int i, int j) {
        return value(i, j) > 0.0;
    };
    // Edge keys over the padded lattice (-1..n): horizontal edges even, vertical odd.
    const std::int64_t stride = nx + 2;
    auto hKey = [&](int i, int j) {
        return ((static_cast<std::int64_t>(j) + 1) * stride + (i + 1)) * 2;
    };
    auto vKey = [&](int i, int j) {
        return ((static_cast<std::int64_t>(j) + 1) * stride + (i + 1)) * 2 + 1;
    };
    std::unordered_map<std::int64_t, Vec2> crossings;
    auto crossing = [&](std::int64_t key, int ia, int ja, int ib, int jb) {
        if (auto it = crossings.find(key); it != crossings.end()) {
            return;
        }
        const double va = value(ia, ja), vb = value(ib, jb);
        auto clampedPoint = [&](int i, int j) {
            // Padding points sit half a cell outside the grid.
            const double ci = std::clamp(static_cast<double>(i), -0.5, nx - 0.5);
            const double cj = std::clamp(static_cast<double>(j), -0.5, ny - 0.5);
            return Vec2(g.origin + Vec2(ci * g.spacing.x(), cj * g.spacing.y()));
        };
        const Vec2 pa = clampedPoint(ia, ja), pb = clampedPoint(ib, jb);
        const bool padded = !std::isfinite(va) || !std::isfinite(vb);
        if (exact && !padded) {
            Vec2 pin = va > 0.0 ? pa : pb;
            Vec2 pout = va > 0.0 ? pb : pa;
            for (int it = 0; it < 60 && (pin - pout).norm() > tolerance; ++it) {
                const Vec2 m = 0.5 * (pin + pout);
                if (exact(m)) {
                    pin = m;
                } else {
                    pout = m;
                }
            }
            crossings.emplace(key, pout); // on the outside
            return;
        }
        double t = 0.5;
        if (std::isfinite(va) && std::isfinite(vb) && va != vb) {
            t = std::clamp(va / (va - vb), 0.0, 1.0);
        } else if (!std::isfinite(va)) {
            t = 0.0; // the padding side: the boundary sits at the padding point
        } else if (!std::isfinite(vb)) {
            t = 1.0;
        }
        crossings.emplace(key, pa + t * (pb - pa));
    };

    struct Segment {
        std::int64_t a, b;
    };
    std::vector<Segment> segments;
    for (int j = -1; j < ny; ++j) {
        for (int i = -1; i < nx; ++i) {
            const int code = (inside(i, j) ? 1 : 0) | (inside(i + 1, j) ? 2 : 0) |
                             (inside(i + 1, j + 1) ? 4 : 0) | (inside(i, j + 1) ? 8 : 0);
            if (code == 0 || code == 15) {
                continue;
            }
            const std::int64_t B = hKey(i, j), T = hKey(i, j + 1), L = vKey(i, j),
                               R = vKey(i + 1, j);
            crossing(B, i, j, i + 1, j);
            crossing(T, i, j + 1, i + 1, j + 1);
            crossing(L, i, j, i, j + 1);
            crossing(R, i + 1, j, i + 1, j + 1);
            // Segments run with the inside on their right (buildRegions fixes orientation).
            auto add = [&](std::int64_t a, std::int64_t b) {
                segments.push_back({a, b});
            };
            const bool centre =
                (code == 5 || code == 10) &&
                (value(i, j) + value(i + 1, j) + value(i + 1, j + 1) + value(i, j + 1)) > 0.0;
            switch (code) {
            case 1:
                add(L, B);
                break;
            case 2:
                add(B, R);
                break;
            case 3:
                add(L, R);
                break;
            case 4:
                add(R, T);
                break;
            case 5:
                if (centre) {
                    add(L, T);
                    add(R, B);
                } else {
                    add(L, B);
                    add(R, T);
                }
                break;
            case 6:
                add(B, T);
                break;
            case 7:
                add(L, T);
                break;
            case 8:
                add(T, L);
                break;
            case 9:
                add(T, B);
                break;
            case 10:
                if (centre) {
                    add(B, L);
                    add(T, R);
                } else {
                    add(B, R);
                    add(T, L);
                }
                break;
            case 11:
                add(T, R);
                break;
            case 12:
                add(R, L);
                break;
            case 13:
                add(R, B);
                break;
            case 14:
                add(B, L);
                break;
            default:
                break;
            }
        }
    }

    // Chain directed segments into loops.
    std::unordered_map<std::int64_t, std::vector<std::size_t>> byStart;
    for (std::size_t s = 0; s < segments.size(); ++s) {
        byStart[segments[s].a].push_back(s);
    }
    std::vector<char> used(segments.size(), 0);
    std::vector<Polyline> loops;
    for (std::size_t s0 = 0; s0 < segments.size(); ++s0) {
        if (used[s0]) {
            continue;
        }
        std::vector<Vec2> pts;
        std::size_t s = s0;
        for (std::size_t guard = 0; guard <= segments.size(); ++guard) {
            used[s] = 1;
            pts.push_back(crossings.at(segments[s].a));
            std::size_t next = segments.size();
            for (const std::size_t cand : byStart[segments[s].b]) {
                if (!used[cand]) {
                    next = cand;
                    break;
                }
            }
            if (next == segments.size()) {
                break;
            }
            s = next;
        }
        // Drop repeated points (crossings at grid points).
        std::vector<Vec2> clean;
        for (const Vec2& p : pts) {
            if (clean.empty() || (p - clean.back()).norm() > 1e-12) {
                clean.push_back(p);
            }
        }
        while (clean.size() > 1 && (clean.front() - clean.back()).norm() <= 1e-12) {
            clean.pop_back();
        }
        if (exact && clean.size() >= 3) {
            // Where a chord between two boundary points cuts into the outside (a corner of the
            // outside within the cell), add the boundary point found across its midpoint.
            const double h = std::min(g.spacing.x(), g.spacing.y());
            std::vector<Vec2> refined;
            std::function<void(const Vec2&, const Vec2&, int)> refine =
                [&](const Vec2& a, const Vec2& b, int depth) {
                    const Vec2 m = 0.5 * (a + b);
                    if (depth >= 6 || (b - a).norm() < 4.0 * tolerance || exact(m)) {
                        refined.push_back(b);
                        return;
                    }
                    const Vec2 d = b - a;
                    // Inside lies to the right of the chord (see the segment table).
                    const Vec2 toInside = Vec2(d.y(), -d.x()).normalized();
                    for (int k = 1; k <= 16; ++k) {
                        const Vec2 q = m + toInside * (k * h / 8.0);
                        if (exact(q)) {
                            Vec2 pin = q, pout = m;
                            for (int it = 0; it < 60 && (pin - pout).norm() > tolerance; ++it) {
                                const Vec2 mm = 0.5 * (pin + pout);
                                if (exact(mm)) {
                                    pin = mm;
                                } else {
                                    pout = mm;
                                }
                            }
                            refine(a, pout, depth + 1);
                            refine(pout, b, depth + 1);
                            return;
                        }
                    }
                    refined.push_back(b);
                };
            for (std::size_t k = 0; k < clean.size(); ++k) {
                refine(clean[k], clean[(k + 1) % clean.size()], 0);
            }
            clean.swap(refined);
        }
        if (clean.size() >= 3) {
            loops.push_back(Polyline::fromPoints(clean, true));
        }
    }
    return buildRegions(std::move(loops));
}

} // namespace ocam::geom2d
