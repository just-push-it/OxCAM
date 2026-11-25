// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/sim/Surface.hpp"

#include <tbb/parallel_for.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>

namespace ocam::sim {

namespace {

/// Rays per axis of the stock's lattice (the Z count also without X and Y grids).
struct FineCounts {
    int nx = 0, ny = 0, nz = 0;
};

FineCounts fineCounts(const Stock& stock) {
    const DexelGrid& g = stock.grid(2);
    FineCounts c{g.nu(), g.nv(), 0};
    if (stock.triDexel() && !stock.grid(0).isEmpty()) {
        c.nz = stock.grid(0).nv();
    } else {
        c.nz = std::max(1, static_cast<int>(std::ceil(stock.bounds().size().z() / g.spacing())));
    }
    return c;
}

/// The end of type `hi` (an interval's top) or not (its bottom) nearest to t, within maxDist.
bool nearestEnd(const IntervalList& r, double t, bool hi, double maxDist, double& out) {
    if (r.empty()) {
        return false;
    }
    const auto it = std::lower_bound(r.begin(), r.end(), t, [](const Interval& in, double v) {
        return static_cast<double>(in.hi) < v;
    });
    const auto k = static_cast<std::ptrdiff_t>(it - r.begin());
    const auto n = static_cast<std::ptrdiff_t>(r.size());
    bool found = false;
    double best = maxDist;
    for (std::ptrdiff_t m = std::max<std::ptrdiff_t>(0, k - 1); m <= std::min(n - 1, k + 1); ++m) {
        const Interval& in = r[static_cast<std::size_t>(m)];
        const double e = hi ? in.hi : in.lo;
        const double d = std::abs(e - t);
        if (d <= best) {
            best = d;
            out = e;
            found = true;
        }
    }
    return found;
}

/// Slope of the boundary through the crossing at t on ray (p, q) of `g`, along the grid's u
/// (dir 0) or v (dir 1) axis, from the matching crossings on the neighbouring rays. Where the
/// two sides disagree (an edge between the rays) the gentler side is taken, so a crossing next
/// to a sharp edge gets the normal of its own face.
bool slopeAt(const DexelGrid& g, int p, int q, double t, bool hi, int dir, double& s) {
    const double h = g.spacing();
    const double reach = 2.0 * h;
    bool hb = false, hf = false;
    double sb = 0.0, sf = 0.0, e = 0.0;
    const int dp = dir == 0 ? 1 : 0;
    const int dq = dir == 1 ? 1 : 0;
    if (p - dp >= 0 && q - dq >= 0 && nearestEnd(g.ray(p - dp, q - dq), t, hi, reach, e)) {
        sb = (t - e) / h;
        hb = true;
    }
    if (p + dp < g.nu() && q + dq < g.nv() && nearestEnd(g.ray(p + dp, q + dq), t, hi, reach, e)) {
        sf = (e - t) / h;
        hf = true;
    }
    if (hb && hf) {
        s = std::abs(sb - sf) <= 0.5 ? 0.5 * (sb + sf) : (std::abs(sb) < std::abs(sf) ? sb : sf);
    } else if (hb) {
        s = sb;
    } else if (hf) {
        s = sf;
    } else {
        return false;
    }
    return true;
}

/// Outward normal of the boundary at the crossing t on ray (p, q) of `g` (`hi`: material ends
/// there going along the ray). False when a neighbouring ray has no matching crossing.
bool crossingNormal(const DexelGrid& g, int p, int q, double t, bool hi, Vec3& n) {
    double du = 0.0, dv = 0.0;
    if (!slopeAt(g, p, q, t, hi, 0, du) || !slopeAt(g, p, q, t, hi, 1, dv)) {
        return false;
    }
    Vec3 v = Vec3::Zero();
    v[g.axis()] = 1.0;
    v[g.uAxis()] = -du;
    v[g.vAxis()] = -dv;
    n = (hi ? v : Vec3(-v)).normalized();
    return true;
}

/// Material flags of the lattice points along Z, one bit per k, for a block of (i, j) columns.
struct Columns {
    int i0 = 0, j0 = 0, w = 0, h = 0, words = 0;
    std::vector<std::uint64_t> bits;

    const std::uint64_t* at(int i, int j) const {
        return bits.data() + (static_cast<std::size_t>(j - j0) * static_cast<std::size_t>(w) +
                              static_cast<std::size_t>(i - i0)) *
                                 static_cast<std::size_t>(words);
    }
    std::uint64_t* at(int i, int j) {
        return bits.data() + (static_cast<std::size_t>(j - j0) * static_cast<std::size_t>(w) +
                              static_cast<std::size_t>(i - i0)) *
                                 static_cast<std::size_t>(words);
    }
    static bool bit(const std::uint64_t* c, int k) {
        return ((c[static_cast<std::size_t>(k) >> 6] >> (static_cast<unsigned>(k) & 63u)) & 1u) !=
               0;
    }
};

/// Calls fn(k) for every set bit k < limit of the words.
template <class Fn> void forBits(const std::uint64_t* w, int words, int limit, Fn&& fn) {
    for (int q = 0; q < words; ++q) {
        std::uint64_t b = w[q];
        while (b) {
            const int k = q * 64 + std::countr_zero(b);
            if (k >= limit) {
                return;
            }
            fn(k);
            b &= b - 1;
        }
    }
}

struct Accumulator {
    Mat3 ata = Mat3::Zero();
    Vec3 atb = Vec3::Zero();
    Vec3 mass = Vec3::Zero();
    int count = 0;
    int normals = 0;
};

struct Quad {
    std::array<std::int32_t, 4> cube{}; ///< Accumulator indices, counter-clockwise from outside.
    bool owned = false;
};

struct Triangle {
    std::array<std::int32_t, 3> cube{};
    Vec3 normal = Vec3::Zero(); ///< Twice the area times the unit normal.
    Vec3 unit = Vec3::Zero();
    bool owned = false;
};

} // namespace

SurfaceLattice surfaceLattice(const Stock& stock, int maxCells) {
    SurfaceLattice l;
    const DexelGrid& g = stock.grid(2);
    if (g.isEmpty()) {
        return l;
    }
    const FineCounts c = fineCounts(stock);
    const double h = g.spacing();
    const int s =
        std::max(1, static_cast<int>(std::ceil(static_cast<double>(std::max({c.nx, c.ny, c.nz})) /
                                               static_cast<double>(std::max(1, maxCells)))));
    l.stride = s;
    // The points on rays, plus one past the last ray (no material there).
    l.ni = (c.nx - 1) / s + 2;
    l.nj = (c.ny - 1) / s + 2;
    l.nk = (c.nz - 1) / s + 2;
    l.origin = Vec3(g.u(0), g.v(0), stock.bounds().min.z() + 0.5 * h);
    l.step = s * h;
    return l;
}

void contourStock(const Stock& stock, const SurfaceLattice& L, int oi0, int oi1, int oj0, int oj1,
                  const SurfaceColour& colour, const SurfaceOptions& options, DisplayMesh& out) {
    if (L.isEmpty()) {
        return;
    }
    oi0 = std::max(oi0, 0);
    oj0 = std::max(oj0, 0);
    oi1 = std::min(oi1, L.ni - 1);
    oj1 = std::min(oj1, L.nj - 1);
    if (oi0 > oi1 || oj0 > oj1) {
        return;
    }
    const DexelGrid& gz = stock.grid(2);
    const DexelGrid& gx = stock.grid(0);
    const DexelGrid& gy = stock.grid(1);
    const bool sides = stock.triDexel() && !gx.isEmpty() && !gy.isEmpty();
    const FineCounts fc = fineCounts(stock);
    const int s = L.stride;
    const double h = gz.spacing();
    const int nk = L.nk;

    // Ranges (lattice point indices, clamped): owned faces O = [oi0, oi1]; faces kept for the
    // shading normals E = O +- 1; cubes with vertices C = [O - 2, O + 1]; edges feeding those
    // cubes A = O +- 2; material columns [O - 2, O + 3].
    auto clampI = [&](int v) {
        return std::clamp(v, 0, L.ni - 1);
    };
    auto clampJ = [&](int v) {
        return std::clamp(v, 0, L.nj - 1);
    };
    const int ci0 = std::max(oi0 - 2, 0), ci1 = std::min(oi1 + 1, L.ni - 2);
    const int cj0 = std::max(oj0 - 2, 0), cj1 = std::min(oj1 + 1, L.nj - 2);
    const int ai0 = clampI(oi0 - 2), ai1 = clampI(oi1 + 2);
    const int aj0 = clampJ(oj0 - 2), aj1 = clampJ(oj1 + 2);
    const int ei0 = clampI(oi0 - 1), ei1 = clampI(oi1 + 1);
    const int ej0 = clampJ(oj0 - 1), ej1 = clampJ(oj1 + 1);

    // Scratch space is kept per thread: chunks are rebuilt many times during playback.
    thread_local Columns cols;
    cols.i0 = ai0;
    cols.j0 = aj0;
    cols.w = clampI(oi1 + 3) - ai0 + 1;
    cols.h = clampJ(oj1 + 3) - aj0 + 1;
    cols.words = (nk + 63) / 64;
    cols.bits.assign(static_cast<std::size_t>(cols.w) * static_cast<std::size_t>(cols.h) *
                         static_cast<std::size_t>(cols.words),
                     0);
    const double z0 = L.origin.z();
    const int kLast = (fc.nz - 1) / s; // last lattice k on a ray
    for (int j = cols.j0; j < cols.j0 + cols.h; ++j) {
        for (int i = cols.i0; i < cols.i0 + cols.w; ++i) {
            if (i * s >= fc.nx || j * s >= fc.ny) {
                continue;
            }
            std::uint64_t* c = cols.at(i, j);
            for (const Interval& in : gz.ray(i * s, j * s)) {
                // Lattice points with lo <= z < hi.
                int k0 = std::max(0, static_cast<int>(std::ceil((in.lo - z0) / L.step)) - 1);
                while (k0 <= kLast && z0 + k0 * L.step < in.lo) {
                    ++k0;
                }
                for (int k = k0; k <= kLast && z0 + k * L.step < in.hi; ++k) {
                    c[static_cast<std::size_t>(k) >> 6] |= std::uint64_t{1}
                                                           << (static_cast<unsigned>(k) & 63u);
                }
            }
        }
    }

    // Accumulators of the cubes in C, found through a dense index.
    const int cw = ci1 - ci0 + 1, ch = cj1 - cj0 + 1;
    if (cw <= 0 || ch <= 0) {
        return;
    }
    // All -1 between calls: the entries used are reset at the end.
    thread_local std::vector<std::int32_t> slot;
    const std::size_t slots = static_cast<std::size_t>(cw) * static_cast<std::size_t>(ch) *
                              static_cast<std::size_t>(nk - 1);
    if (slot.size() < slots) {
        slot.assign(slots, -1);
    }
    thread_local std::vector<Accumulator> acc;
    thread_local std::vector<std::array<int, 3>> accCube;
    acc.clear();
    accCube.clear();
    auto cubeSlot = [&](int i, int j, int k) -> std::int32_t* {
        if (i < ci0 || i > ci1 || j < cj0 || j > cj1 || k < 0 || k > nk - 2) {
            return nullptr;
        }
        return &slot[(static_cast<std::size_t>(k) * static_cast<std::size_t>(ch) +
                      static_cast<std::size_t>(j - cj0)) *
                         static_cast<std::size_t>(cw) +
                     static_cast<std::size_t>(i - ci0)];
    };
    auto cubeOf = [&](int i, int j, int k) -> std::int32_t {
        std::int32_t* p = cubeSlot(i, j, k);
        if (!p) {
            return -1;
        }
        if (*p < 0) {
            *p = static_cast<std::int32_t>(acc.size());
            acc.emplace_back();
            accCube.push_back({i, j, k});
        }
        return *p;
    };
    auto point = [&](int i, int j, int k) {
        return Vec3(L.origin.x() + i * L.step, L.origin.y() + j * L.step,
                    L.origin.z() + k * L.step);
    };

    thread_local std::vector<Quad> quads;
    quads.clear();
    // One crossed edge from lattice point (i, j, k) along `axis`; `inLow`: material at the low
    // end. Adds its crossing to the cubes around it and, inside E, its quad.
    auto edge = [&](int axis, int i, int j, int k, bool inLow) {
        const Vec3 a = point(i, j, k);
        Vec3 p = a;
        const double lo = a[axis];
        const double hiEnd = lo + L.step;
        const bool leaving = inLow; // the crossing is the top of an interval along the axis
        Vec3 n = Vec3::Zero();
        bool hasNormal = false;
        double t = lo + 0.5 * L.step;
        const DexelGrid* g = nullptr;
        int rp = 0, rq = 0;
        if (axis == 2) {
            g = &gz;
            rp = i * s;
            rq = j * s;
        } else if (sides && axis == 0) {
            g = &gx;
            rp = j * s;
            rq = k * s;
        } else if (sides && axis == 1) {
            g = &gy;
            rp = k * s;
            rq = i * s;
        }
        if (g && rp < g->nu() && rq < g->nv()) {
            double e = 0.0;
            if (nearestEnd(g->ray(rp, rq), t, leaving, 0.5 * L.step + 0.5 * h, e)) {
                t = std::clamp(e, lo, hiEnd);
                hasNormal = crossingNormal(*g, rp, rq, e, leaving, n);
            }
        }
        p[axis] = t;
        // The four cubes around the edge, counter-clockwise seen from +axis.
        std::array<std::array<int, 3>, 4> around{};
        const int b = (axis + 1) % 3, c = (axis + 2) % 3;
        const std::array<int, 3> base{i, j, k};
        const int db[4] = {-1, 0, 0, -1};
        const int dc[4] = {-1, -1, 0, 0};
        for (int q = 0; q < 4; ++q) {
            std::array<int, 3> cube = base;
            cube[static_cast<std::size_t>(b)] += db[q];
            cube[static_cast<std::size_t>(c)] += dc[q];
            around[static_cast<std::size_t>(q)] = cube;
        }
        Quad quad;
        bool complete = true;
        for (int q = 0; q < 4; ++q) {
            const auto& cube = around[static_cast<std::size_t>(q)];
            const std::int32_t idx = cubeOf(cube[0], cube[1], cube[2]);
            quad.cube[static_cast<std::size_t>(q)] = idx;
            if (idx < 0) {
                complete = false;
                continue;
            }
            Accumulator& ac = acc[static_cast<std::size_t>(idx)];
            ac.mass += p;
            ++ac.count;
            if (hasNormal) {
                ac.ata += n * n.transpose();
                ac.atb += n * n.dot(p);
                ++ac.normals;
            }
        }
        if (complete && i >= ei0 && i <= ei1 && j >= ej0 && j <= ej1) {
            if (!inLow) {
                std::swap(quad.cube[1], quad.cube[3]); // outward is -axis
            }
            quad.owned = i >= oi0 && i <= oi1 && j >= oj0 && j <= oj1;
            quads.push_back(quad);
        }
    };

    thread_local std::vector<std::uint64_t> diff;
    diff.resize(static_cast<std::size_t>(cols.words));
    for (int j = aj0; j <= aj1; ++j) {
        for (int i = ai0; i <= ai1; ++i) {
            const std::uint64_t* c = cols.at(i, j);
            // Along Z: bit k differs from bit k + 1.
            for (int q = 0; q < cols.words; ++q) {
                const std::uint64_t next =
                    (c[q] >> 1) | (q + 1 < cols.words ? (c[q + 1] << 63) : std::uint64_t{0});
                diff[static_cast<std::size_t>(q)] = c[q] ^ next;
            }
            forBits(diff.data(), cols.words, nk - 1,
                    [&](int k) { edge(2, i, j, k, Columns::bit(c, k)); });
            if (i + 1 < L.ni) {
                const std::uint64_t* d = cols.at(i + 1, j);
                for (int q = 0; q < cols.words; ++q) {
                    diff[static_cast<std::size_t>(q)] = c[q] ^ d[q];
                }
                forBits(diff.data(), cols.words, nk,
                        [&](int k) { edge(0, i, j, k, Columns::bit(c, k)); });
            }
            if (j + 1 < L.nj) {
                const std::uint64_t* d = cols.at(i, j + 1);
                for (int q = 0; q < cols.words; ++q) {
                    diff[static_cast<std::size_t>(q)] = c[q] ^ d[q];
                }
                forBits(diff.data(), cols.words, nk,
                        [&](int k) { edge(1, i, j, k, Columns::bit(c, k)); });
            }
        }
    }

    // Vertex positions: the point nearest the crossings' tangent planes (a quadratic error
    // function solved about the crossings' mean, small eigenvalues dropped), kept in its cube.
    thread_local std::vector<Vec3> pos;
    pos.resize(acc.size());
    for (const auto& cube : accCube) {
        *cubeSlot(cube[0], cube[1], cube[2]) = -1; // leave the index clean for the next call
    }
    for (std::size_t a = 0; a < acc.size(); ++a) {
        const Accumulator& ac = acc[a];
        const Vec3 m = ac.mass / std::max(1, ac.count);
        Vec3 x = m;
        if (options.sharpFeatures && ac.normals > 0) {
            Eigen::SelfAdjointEigenSolver<Mat3> es;
            es.computeDirect(ac.ata);
            const Vec3 r = ac.atb - ac.ata * m;
            Vec3 y = Vec3::Zero();
            for (int e = 0; e < 3; ++e) {
                const double ev = es.eigenvalues()[e];
                if (ev > 0.1) {
                    const Vec3 v = es.eigenvectors().col(e);
                    y += v * (v.dot(r) / ev);
                }
            }
            x = m + y;
            const auto& cube = accCube[a];
            const Vec3 cmin = point(cube[0], cube[1], cube[2]);
            const Vec3 cmax = cmin + Vec3::Constant(L.step);
            const double slack = 0.25 * L.step;
            if ((x.array() < cmin.array() - slack).any() ||
                (x.array() > cmax.array() + slack).any()) {
                x = m;
            } else {
                x = x.cwiseMax(cmin).cwiseMin(cmax);
            }
        }
        pos[a] = x;
    }

    // Triangles (shorter diagonal), with the cubes' incident triangles for shading.
    thread_local std::vector<Triangle> tris;
    tris.clear();
    for (const Quad& q : quads) {
        const Vec3& v0 = pos[static_cast<std::size_t>(q.cube[0])];
        const Vec3& v1 = pos[static_cast<std::size_t>(q.cube[1])];
        const Vec3& v2 = pos[static_cast<std::size_t>(q.cube[2])];
        const Vec3& v3 = pos[static_cast<std::size_t>(q.cube[3])];
        std::array<std::array<int, 3>, 2> split{};
        if ((v2 - v0).squaredNorm() <= (v3 - v1).squaredNorm()) {
            split = {{{0, 1, 2}, {0, 2, 3}}};
        } else {
            split = {{{0, 1, 3}, {1, 2, 3}}};
        }
        for (const auto& sp : split) {
            Triangle t;
            for (int c = 0; c < 3; ++c) {
                t.cube[static_cast<std::size_t>(c)] =
                    q.cube[static_cast<std::size_t>(sp[static_cast<std::size_t>(c)])];
            }
            const Vec3& a = pos[static_cast<std::size_t>(t.cube[0])];
            const Vec3& b = pos[static_cast<std::size_t>(t.cube[1])];
            const Vec3& c = pos[static_cast<std::size_t>(t.cube[2])];
            t.normal = (b - a).cross(c - a);
            if (t.normal.squaredNorm() < 1e-24) {
                continue; // degenerate (two vertices met)
            }
            t.unit = t.normal.normalized();
            t.owned = q.owned;
            tris.push_back(t);
        }
    }
    thread_local std::vector<std::int32_t> first;
    first.assign(acc.size() + 1, 0);
    for (const Triangle& t : tris) {
        for (const auto c : t.cube) {
            ++first[static_cast<std::size_t>(c) + 1];
        }
    }
    for (std::size_t a = 0; a < acc.size(); ++a) {
        first[a + 1] += first[a];
    }
    thread_local std::vector<std::int32_t> incident;
    incident.resize(static_cast<std::size_t>(first.back()));
    {
        thread_local std::vector<std::int32_t> fill;
        fill.assign(first.begin(), first.end() - 1);
        for (std::size_t t = 0; t < tris.size(); ++t) {
            for (const auto c : tris[t].cube) {
                incident[static_cast<std::size_t>(fill[static_cast<std::size_t>(c)]++)] =
                    static_cast<std::int32_t>(t);
            }
        }
    }

    // Output vertices: one per cube and group of faces within the crease angle of each other.
    const double cosCrease = std::cos(options.creaseAngle);
    thread_local std::vector<std::int32_t> head; // first output vertex of a cube
    thread_local std::vector<std::int32_t> next; // per output vertex: the cube's next one
    head.assign(acc.size(), -1);
    next.clear();
    const std::uint32_t base = static_cast<std::uint32_t>(out.positions.size());
    auto vertexFor = [&](std::int32_t cube, const Vec3& n) -> std::uint32_t {
        for (std::int32_t v = head[static_cast<std::size_t>(cube)]; v >= 0;
             v = next[static_cast<std::size_t>(v)]) {
            const auto& vn = out.normals[base + static_cast<std::uint32_t>(v)];
            if (vn[0] * n.x() + vn[1] * n.y() + vn[2] * n.z() > 0.9999) {
                return base + static_cast<std::uint32_t>(v);
            }
        }
        const Vec3& p = pos[static_cast<std::size_t>(cube)];
        const auto v = static_cast<std::int32_t>(next.size());
        next.push_back(head[static_cast<std::size_t>(cube)]);
        head[static_cast<std::size_t>(cube)] = v;
        out.positions.push_back(
            {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())});
        out.normals.push_back(
            {static_cast<float>(n.x()), static_cast<float>(n.y()), static_cast<float>(n.z())});
        out.colours.push_back(colour ? colour(p, n) : kStockColour);
        return base + static_cast<std::uint32_t>(v);
    };
    for (const Triangle& t : tris) {
        if (!t.owned) {
            continue;
        }
        const Vec3& own = t.unit;
        for (const auto c : t.cube) {
            Vec3 sum = Vec3::Zero();
            for (std::int32_t m = first[static_cast<std::size_t>(c)];
                 m < first[static_cast<std::size_t>(c) + 1]; ++m) {
                const Triangle& o =
                    tris[static_cast<std::size_t>(incident[static_cast<std::size_t>(m)])];
                if (o.unit.dot(own) >= cosCrease) {
                    sum += o.normal;
                }
            }
            const Vec3 n = sum.squaredNorm() > 0.0 ? sum.normalized() : own;
            out.indices.push_back(vertexFor(c, n));
        }
    }
}

CutColours::CutColours(const Stock& stock, std::vector<const Stock*> starts, const Rgb& uncut)
    : stock_(&stock), starts_(std::move(starts)), uncut_(uncut) {}

std::optional<Rgb> CutColours::operator()(const Vec3& p, const Vec3& n) const {
    int axis = 2;
    if (stock_->triDexel()) {
        n.cwiseAbs().maxCoeff(&axis);
    }
    const DexelGrid& g = stock_->grid(axis);
    if (g.isEmpty()) {
        return std::nullopt;
    }
    const double h = g.spacing();
    const int i =
        std::clamp(static_cast<int>(std::lround((p[g.uAxis()] - g.u0()) / h - 0.5)), 0, g.nu() - 1);
    const int j =
        std::clamp(static_cast<int>(std::lround((p[g.vAxis()] - g.v0()) / h - 0.5)), 0, g.nv() - 1);
    const bool hi = n[axis] > 0.0;
    double e = 0.0;
    if (!nearestEnd(g.ray(i, j), p[axis], hi, 2.0 * h, e)) {
        return std::nullopt;
    }
    for (std::size_t k = starts_.size(); k-- > 0;) {
        const DexelGrid& before = starts_[k]->grid(axis);
        double f = 0.0;
        if (before.nu() != g.nu() || before.nv() != g.nv() ||
            !nearestEnd(before.ray(i, j), e, hi, 1e-5, f)) {
            return operationColour(static_cast<int>(k));
        }
    }
    return uncut_;
}

DisplayMesh buildSurfaceMesh(const Stock& stock, int maxCells, const SurfaceColour& colour,
                             const SurfaceOptions& options) {
    DisplayMesh mesh;
    const SurfaceLattice L = surfaceLattice(stock, maxCells);
    if (L.isEmpty()) {
        return mesh;
    }
    constexpr int kBlock = 32;
    const int bu = (L.ni + kBlock - 1) / kBlock;
    const int bv = (L.nj + kBlock - 1) / kBlock;
    std::vector<DisplayMesh> parts(static_cast<std::size_t>(bu) * static_cast<std::size_t>(bv));
    tbb::parallel_for(std::size_t{0}, parts.size(), [&](std::size_t b) {
        const int x = static_cast<int>(b % static_cast<std::size_t>(bu));
        const int y = static_cast<int>(b / static_cast<std::size_t>(bu));
        contourStock(stock, L, x * kBlock, (x + 1) * kBlock - 1, y * kBlock, (y + 1) * kBlock - 1,
                     colour, options, parts[b]);
    });
    for (const DisplayMesh& p : parts) {
        const auto offset = static_cast<std::uint32_t>(mesh.positions.size());
        mesh.positions.insert(mesh.positions.end(), p.positions.begin(), p.positions.end());
        mesh.normals.insert(mesh.normals.end(), p.normals.begin(), p.normals.end());
        mesh.colours.insert(mesh.colours.end(), p.colours.begin(), p.colours.end());
        for (const std::uint32_t idx : p.indices) {
            mesh.indices.push_back(idx + offset);
        }
    }
    return mesh;
}

Stock resampleStock(const Stock& source, const Transform3& sourceToTarget, const Stock& target) {
    const DisplayMesh surface =
        buildSurfaceMesh(source, std::numeric_limits<int>::max(), {}, SurfaceOptions{});
    geom3d::Mesh mesh;
    mesh.vertices.reserve(surface.positions.size());
    for (const auto& p : surface.positions) {
        mesh.vertices.push_back(sourceToTarget * Vec3(static_cast<double>(p[0]),
                                                      static_cast<double>(p[1]),
                                                      static_cast<double>(p[2])));
    }
    mesh.triangles.reserve(surface.triangleCount());
    for (std::size_t t = 0; t + 2 < surface.indices.size(); t += 3) {
        mesh.triangles.push_back(
            {surface.indices[t], surface.indices[t + 1], surface.indices[t + 2]});
    }
    Stock out = target;
    for (int axis = 0; axis < 3; ++axis) {
        if (!target.triDexel() && axis != 2) {
            continue;
        }
        const DexelGrid& kept = target.grid(axis);
        if (kept.isEmpty()) {
            continue;
        }
        // An empty grid on the same lattice: rays the surface misses hold no material.
        DexelGrid filled(axis, kept.u0(), kept.v0(), kept.spacing(), kept.nu(), kept.nv());
        filled.fillFromMesh(mesh);
        DexelGrid& g = out.grid(axis);
        g.detach(0, g.nu() - 1, 0, g.nv() - 1);
        tbb::parallel_for(0, g.nv(), [&](int j) {
            for (int i = 0; i < g.nu(); ++i) {
                g.mutableRay(i, j) = intersect(kept.ray(i, j), filled.ray(i, j));
            }
        });
    }
    return out;
}

DisplayMesh revolvedMesh(const geom2d::Regions& profile, double chordTolerance, const Rgb& colour,
                         double creaseAngle) {
    DisplayMesh mesh = revolvedMesh(profile, chordTolerance, ProfileColour{}, creaseAngle);
    std::fill(mesh.colours.begin(), mesh.colours.end(), colour);
    return mesh;
}

ProfileDistance::ProfileDistance(const geom2d::Regions& regions, double chordTolerance) {
    auto add = [&](const geom2d::Polyline& loop) {
        const std::vector<Vec2> pts = loop.discretize(chordTolerance);
        for (std::size_t k = 0; k < pts.size(); ++k) {
            segments_.emplace_back(pts[k], pts[(k + 1) % pts.size()]);
        }
    };
    for (const auto& r : regions) {
        add(r.outer);
        for (const auto& hole : r.holes) {
            add(hole);
        }
    }
}

ProfileDistance ProfileDistance::section(const geom3d::Mesh& mesh) {
    // The plane y = delta, a hair off the axis plane, so vertices on the seam of a revolved
    // face (y = 0) never lie exactly on it. The section's closed loops on both sides of the axis
    // are kept: the even-odd test needs them closed, and the half profile is the v >= 0 side.
    constexpr double delta = 1e-6;
    ProfileDistance out;
    for (const auto& t : mesh.triangles) {
        const Vec3* v[3] = {&mesh.vertices[t[0]], &mesh.vertices[t[1]], &mesh.vertices[t[2]]};
        Vec2 hit[2];
        int count = 0;
        for (int e = 0; e < 3 && count < 2; ++e) {
            const Vec3& a = *v[e];
            const Vec3& b = *v[(e + 1) % 3];
            const double da = a.y() - delta, db = b.y() - delta;
            if ((da < 0.0) != (db < 0.0)) {
                const Vec3 x = a + (b - a) * (da / (da - db));
                hit[count++] = Vec2(x.z(), x.x());
            }
        }
        if (count == 2) {
            out.segments_.emplace_back(hit[0], hit[1]);
        }
    }
    return out;
}

double ProfileDistance::operator()(const Vec2& p) const {
    double best = std::numeric_limits<double>::infinity();
    bool inside = false;
    for (const auto& [a, b] : segments_) {
        const Vec2 ab = b - a;
        const double len2 = ab.squaredNorm();
        const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
        best = std::min(best, (p - (a + t * ab)).squaredNorm());
        if ((a.y() > p.y()) != (b.y() > p.y()) &&
            p.x() < a.x() + (p.y() - a.y()) * ab.x() / ab.y()) {
            inside = !inside;
        }
    }
    return inside ? -std::sqrt(best) : std::sqrt(best);
}

DisplayMesh revolvedMesh(const geom2d::Regions& profile, double chordTolerance,
                         const ProfileColour& colour, double creaseAngle, double colourStep) {
    std::vector<std::vector<Vec2>> loops;
    for (const auto& r : profile) {
        loops.push_back(r.outer.discretize(chordTolerance));
        for (const auto& hole : r.holes) {
            loops.push_back(hole.discretize(chordTolerance));
        }
    }
    return revolvedLoops(loops, chordTolerance, colour, creaseAngle, colourStep, {});
}

DisplayMesh revolvedLoops(const std::vector<std::vector<Vec2>>& loops, double chordTolerance,
                          const ProfileColour& colour, double creaseAngle, double colourStep,
                          const std::function<bool(const Vec2&, const Vec2&)>& skip) {
    DisplayMesh mesh;
    double rmax = 0.0;
    for (const auto& loop : loops) {
        for (const Vec2& p : loop) {
            rmax = std::max(rmax, p.y());
        }
    }
    if (rmax <= 0.0) {
        return mesh;
    }
    const double tol = std::clamp(chordTolerance, 1e-4, 0.5 * rmax);
    const int segments =
        std::clamp(static_cast<int>(std::ceil(kPi / std::acos(1.0 - tol / rmax))), 24, 2048);
    std::vector<double> cs(static_cast<std::size_t>(segments) + 1),
        sn(static_cast<std::size_t>(segments) + 1);
    for (int k = 0; k <= segments; ++k) {
        const double a = 2.0 * kPi * (k % segments) / segments; // the seam closes exactly
        cs[static_cast<std::size_t>(k)] = std::cos(a);
        sn[static_cast<std::size_t>(k)] = std::sin(a);
    }
    const double cosCrease = std::cos(creaseAngle);
    for (const auto& loop : loops) {
        const std::size_t n = loop.size();
        if (n < 3) {
            continue;
        }
        // Outward normals of the profile's edges (outer loops counter-clockwise, holes
        // clockwise: material on the left).
        std::vector<Vec2> en(n);
        for (std::size_t e = 0; e < n; ++e) {
            const Vec2 d = loop[(e + 1) % n] - loop[e];
            const double len = d.norm();
            en[e] = len > 1e-12 ? Vec2(d.y() / len, -d.x() / len) : Vec2::Zero();
        }
        for (std::size_t e = 0; e < n; ++e) {
            const Vec2& p = loop[e];
            const Vec2& q = loop[(e + 1) % n];
            if (en[e].squaredNorm() == 0.0 || (p.y() <= 1e-9 && q.y() <= 1e-9)) {
                continue; // degenerate or on the axis
            }
            if (skip && skip(p, q)) {
                continue;
            }
            // Smooth across profile corners flatter than the crease angle.
            auto endNormal = [&](std::size_t other) {
                const Vec2 o = en[other];
                return o.dot(en[e]) >= cosCrease ? Vec2((o + en[e]).normalized()) : en[e];
            };
            const Vec2 np = endNormal((e + n - 1) % n);
            const Vec2 nq = endNormal((e + 1) % n);
            // Coloured profiles are drawn in pieces of at most colourStep, each in the colour of
            // its middle: a colour changes where the profile's state does, not blended along a
            // long edge.
            const double len = (q - p).norm();
            const int pieces =
                colour ? std::clamp(static_cast<int>(std::ceil(len / colourStep)), 1, 10000) : 1;
            for (int piece = 0; piece < pieces; ++piece) {
                const double t0 = static_cast<double>(piece) / pieces;
                const double t1 = static_cast<double>(piece + 1) / pieces;
                const Vec2 a = p + (q - p) * t0, b = p + (q - p) * t1;
                const Vec2 na = (np + (nq - np) * t0).normalized();
                const Vec2 nb = (np + (nq - np) * t1).normalized();
                const Rgb rgb =
                    colour ? colour(0.5 * (a + b), (na + nb).normalized()) : kStockColour;
                const auto first = static_cast<std::uint32_t>(mesh.positions.size());
                for (int k = 0; k <= segments; ++k) {
                    const double c = cs[static_cast<std::size_t>(k)];
                    const double si = sn[static_cast<std::size_t>(k)];
                    for (const auto& [pt, nn] : {std::pair{a, na}, std::pair{b, nb}}) {
                        mesh.positions.push_back({static_cast<float>(pt.y() * c),
                                                  static_cast<float>(pt.y() * si),
                                                  static_cast<float>(pt.x())});
                        mesh.normals.push_back({static_cast<float>(nn.y() * c),
                                                static_cast<float>(nn.y() * si),
                                                static_cast<float>(nn.x())});
                        mesh.colours.push_back(rgb);
                    }
                }
                for (int k = 0; k < segments; ++k) {
                    const std::uint32_t v = first + 2u * static_cast<std::uint32_t>(k);
                    // v: a at angle k, v + 1: b at k, v + 2: a at k + 1, v + 3: b at k + 1.
                    for (const std::uint32_t t : {v, v + 1, v + 3, v, v + 3, v + 2}) {
                        mesh.indices.push_back(t);
                    }
                }
            }
        }
    }
    return mesh;
}

} // namespace ocam::sim
