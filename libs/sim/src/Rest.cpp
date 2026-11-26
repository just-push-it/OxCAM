// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Boundary tracing of a binary cell image: every edge between a filled and an empty cell is
// directed with the filled cell on its left, and edges are chained into loops. At a vertex shared
// by two diagonal cells the trace turns left, which keeps the two cells in separate loops. Outer
// loops come out counter-clockwise and holes clockwise.

#include "ocam/sim/Rest.hpp"

#include <ocam/geom2d/Clip.hpp>

#include <algorithm>
#include <array>
#include <cstdint>

namespace ocam::sim {

namespace {

/// Boundary of the filled cells of the stock's Z grid.
geom2d::Regions traceCells(const DexelGrid& g, const std::vector<std::uint8_t>& filled) {
    const int nu = g.nu();
    const int nv = g.nv();
    auto cellIndex = [&](int i, int j) {
        return static_cast<std::size_t>(j) * static_cast<std::size_t>(nu) +
               static_cast<std::size_t>(i);
    };
    auto isFilled = [&](int i, int j) {
        return i >= 0 && j >= 0 && i < nu && j < nv && filled[cellIndex(i, j)] != 0;
    };

    // Directed boundary edges, stored per start vertex as a bit per direction:
    // 0 = +x, 1 = +y, 2 = -x, 3 = -y.
    const int vu = nu + 1;
    auto vertexIndex = [&](int i, int j) {
        return static_cast<std::size_t>(j) * static_cast<std::size_t>(vu) +
               static_cast<std::size_t>(i);
    };
    std::vector<std::uint8_t> edges(static_cast<std::size_t>(vu) * static_cast<std::size_t>(nv + 1),
                                    0);
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            if (!isFilled(i, j)) {
                continue;
            }
            if (!isFilled(i, j - 1)) {
                edges[vertexIndex(i, j)] |= 1u << 0; // bottom, +x
            }
            if (!isFilled(i + 1, j)) {
                edges[vertexIndex(i + 1, j)] |= 1u << 1; // right, +y
            }
            if (!isFilled(i, j + 1)) {
                edges[vertexIndex(i + 1, j + 1)] |= 1u << 2; // top, -x
            }
            if (!isFilled(i - 1, j)) {
                edges[vertexIndex(i, j + 1)] |= 1u << 3; // left, -y
            }
        }
    }

    constexpr std::array<int, 4> dx{1, 0, -1, 0};
    constexpr std::array<int, 4> dy{0, 1, 0, -1};
    const double h = g.spacing();
    auto point = [&](int i, int j) {
        return Vec2(g.u0() + i * h, g.v0() + j * h);
    };

    std::vector<geom2d::Polyline> outers;
    std::vector<geom2d::Polyline> holes;
    for (int j0 = 0; j0 <= nv; ++j0) {
        for (int i0 = 0; i0 <= nu; ++i0) {
            while (edges[vertexIndex(i0, j0)] != 0) {
                int dir = 0;
                while ((edges[vertexIndex(i0, j0)] & (1u << dir)) == 0) {
                    ++dir;
                }
                std::vector<Vec2> pts;
                int i = i0, j = j0;
                int previous = -1;
                double area2 = 0.0;
                for (;;) {
                    edges[vertexIndex(i, j)] &= static_cast<std::uint8_t>(~(1u << dir));
                    if (dir != previous) {
                        pts.push_back(point(i, j));
                    }
                    const int ni = i + dx[static_cast<std::size_t>(dir)];
                    const int nj = j + dy[static_cast<std::size_t>(dir)];
                    area2 += static_cast<double>(i) * nj - static_cast<double>(ni) * j;
                    previous = dir;
                    i = ni;
                    j = nj;
                    const std::uint8_t out = edges[vertexIndex(i, j)];
                    if ((i == i0 && j == j0) || out == 0) {
                        break; // back at the start
                    }
                    // Prefer a left turn, then straight on, then a right turn.
                    int next = -1;
                    for (const int turn : {1, 0, 3}) {
                        const int d = (dir + turn) % 4;
                        if (out & (1u << d)) {
                            next = d;
                            break;
                        }
                    }
                    if (next < 0) {
                        break;
                    }
                    dir = next;
                }
                if (pts.size() >= 4) {
                    (area2 > 0.0 ? outers : holes)
                        .push_back(geom2d::Polyline::fromPoints(pts, true));
                }
            }
        }
    }

    // Holes go into the smallest outer containing them; Clipper then resolves vertex contacts.
    geom2d::Regions regions;
    std::vector<double> areas;
    for (auto& outer : outers) {
        areas.push_back(outer.area());
        regions.push_back({std::move(outer), {}});
    }
    for (auto& hole : holes) {
        const auto& v = hole.vertices();
        const Vec2 probe = 0.5 * (v[0].p + v[1].p); // edge midpoints are never on another loop
        int best = -1;
        for (std::size_t k = 0; k < regions.size(); ++k) {
            if ((best < 0 || areas[k] < areas[static_cast<std::size_t>(best)]) &&
                regions[k].outer.contains(probe, 1e-9)) {
                best = static_cast<int>(k);
            }
        }
        if (best >= 0) {
            regions[static_cast<std::size_t>(best)].holes.push_back(std::move(hole));
        }
    }
    return geom2d::unite(regions, {}, {h * 1e-3});
}

} // namespace

geom2d::Regions materialRegions(const Stock& stock, double zLow, double zHigh, double minLength) {
    const DexelGrid& g = stock.grid(2);
    if (g.isEmpty() || zHigh <= zLow) {
        return {};
    }
    std::vector<std::uint8_t> filled(
        static_cast<std::size_t>(g.nu()) * static_cast<std::size_t>(g.nv()), 0);
    bool any = false;
    for (int j = 0; j < g.nv(); ++j) {
        for (int i = 0; i < g.nu(); ++i) {
            const IntervalList& ray = g.ray(i, j);
            if (!ray.empty() && static_cast<double>(ray.back().hi) > zLow &&
                overlapLength(ray, zLow, zHigh) > minLength) {
                filled[static_cast<std::size_t>(j) * static_cast<std::size_t>(g.nu()) +
                       static_cast<std::size_t>(i)] = 1;
                any = true;
            }
        }
    }
    return any ? traceCells(g, filled) : geom2d::Regions{};
}

geom2d::Regions excessRegions(const Stock& stock, const Comparison& comparison, double minExcess) {
    const DexelGrid& g = stock.grid(2);
    if (g.isEmpty() || comparison.nu != g.nu() || comparison.nv != g.nv()) {
        return {};
    }
    std::vector<std::uint8_t> filled(comparison.excess.size(), 0);
    bool any = false;
    for (std::size_t k = 0; k < filled.size(); ++k) {
        if (static_cast<double>(comparison.excess[k]) > minExcess) {
            filled[k] = 1;
            any = true;
        }
    }
    return any ? traceCells(g, filled) : geom2d::Regions{};
}

} // namespace ocam::sim
