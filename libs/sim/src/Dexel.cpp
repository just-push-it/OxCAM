// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/sim/Dexel.hpp"

#include <ocam/foundation/Error.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace ocam::sim {

bool subtract(IntervalList& list, double lo, double hi) {
    if (hi <= lo || list.empty()) {
        return false;
    }
    const auto flo = static_cast<float>(lo);
    const auto fhi = static_cast<float>(hi);
    if (fhi <= list.front().lo || flo >= list.back().hi) {
        return false;
    }
    const std::size_t n = list.size();
    std::size_t k = 0;
    while (k < n && list[k].hi <= flo) {
        ++k;
    }
    if (k == n || list[k].lo >= fhi) {
        return false;
    }
    std::size_t e = k;
    while (e < n && list[e].lo < fhi) {
        ++e;
    }
    // Intervals [k, e) overlap the removed range; keep the parts outside it.
    Interval keep[2];
    std::size_t count = 0;
    if (list[k].lo < flo) {
        keep[count++] = {list[k].lo, flo};
    }
    if (list[e - 1].hi > fhi) {
        keep[count++] = {fhi, list[e - 1].hi};
    }
    const std::size_t removed = e - k;
    if (count <= removed) {
        for (std::size_t c = 0; c < count; ++c) {
            list[k + c] = keep[c];
        }
        list.erase(list.begin() + static_cast<std::ptrdiff_t>(k + count),
                   list.begin() + static_cast<std::ptrdiff_t>(e));
    } else {
        // One interval split in two.
        list[k] = keep[0];
        list.insert(list.begin() + static_cast<std::ptrdiff_t>(k + 1), keep[1]);
    }
    return true;
}

double totalLength(const IntervalList& list) {
    double sum = 0.0;
    for (const Interval& in : list) {
        sum += static_cast<double>(in.hi) - in.lo;
    }
    return sum;
}

double overlapLength(const IntervalList& list, double lo, double hi) {
    double sum = 0.0;
    for (const Interval& in : list) {
        const double a = std::max<double>(in.lo, lo);
        const double b = std::min<double>(in.hi, hi);
        if (b > a) {
            sum += b - a;
        }
    }
    return sum;
}

double topOf(const IntervalList& list) {
    return list.empty() ? -std::numeric_limits<double>::infinity()
                        : static_cast<double>(list.back().hi);
}

double differenceLength(const IntervalList& a, const IntervalList& b) {
    double sum = 0.0;
    for (const Interval& in : a) {
        sum += static_cast<double>(in.hi) - in.lo - overlapLength(b, in.lo, in.hi);
    }
    return sum;
}

IntervalList intersect(const IntervalList& a, const IntervalList& b) {
    IntervalList out;
    std::size_t i = 0, j = 0;
    while (i < a.size() && j < b.size()) {
        const float lo = std::max(a[i].lo, b[j].lo);
        const float hi = std::min(a[i].hi, b[j].hi);
        if (hi > lo) {
            out.push_back({lo, hi});
        }
        if (a[i].hi < b[j].hi) {
            ++i;
        } else {
            ++j;
        }
    }
    return out;
}

IntervalList unite(const IntervalList& a, const IntervalList& b) {
    IntervalList all;
    all.reserve(a.size() + b.size());
    std::merge(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(all),
               [](const Interval& x, const Interval& y) { return x.lo < y.lo; });
    IntervalList out;
    for (const Interval& in : all) {
        if (!out.empty() && in.lo <= out.back().hi) {
            out.back().hi = std::max(out.back().hi, in.hi);
        } else {
            out.push_back(in);
        }
    }
    return out;
}

IntervalList grow(const IntervalList& list, double amount) {
    IntervalList out;
    const auto d = static_cast<float>(amount);
    for (const Interval& in : list) {
        const Interval g{in.lo - d, in.hi + d};
        if (g.hi <= g.lo) {
            continue;
        }
        if (!out.empty() && g.lo <= out.back().hi) {
            out.back().hi = std::max(out.back().hi, g.hi);
        } else {
            out.push_back(g);
        }
    }
    return out;
}

// -- DexelGrid ------------------------------------------------------------------------------

DexelGrid::DexelGrid(int axis, double u0, double v0, double spacing, int nu, int nv)
    : axis_(axis), u0_(u0), v0_(v0), h_(spacing), nu_(nu), nv_(nv) {
    tilesU_ = (nu + kTile - 1) / kTile;
    tilesV_ = (nv + kTile - 1) / kTile;
    tiles_.resize(static_cast<std::size_t>(tilesU_) * static_cast<std::size_t>(tilesV_));
    for (auto& t : tiles_) {
        t = std::make_shared<Tile>();
    }
}

std::pair<int, int> DexelGrid::uRange(double a, double b) const noexcept {
    const int lo = std::max(0, static_cast<int>(std::ceil((a - u0_) / h_ - 0.5)));
    const int hi = std::min(nu_ - 1, static_cast<int>(std::floor((b - u0_) / h_ - 0.5)));
    return {lo, hi};
}

std::pair<int, int> DexelGrid::vRange(double a, double b) const noexcept {
    const int lo = std::max(0, static_cast<int>(std::ceil((a - v0_) / h_ - 0.5)));
    const int hi = std::min(nv_ - 1, static_cast<int>(std::floor((b - v0_) / h_ - 0.5)));
    return {lo, hi};
}

const IntervalList& DexelGrid::ray(int i, int j) const {
    return tiles_[tileIndex(i, j)]->rays[local(i, j)];
}

IntervalList& DexelGrid::mutableRay(int i, int j) {
    auto& tile = tiles_[tileIndex(i, j)];
    if (tile.use_count() > 1) {
        tile = std::make_shared<Tile>(*tile);
    }
    return tile->rays[local(i, j)];
}

void DexelGrid::detach(int i0, int i1, int j0, int j1) {
    if (i0 > i1 || j0 > j1) {
        return;
    }
    for (int tj = j0 / kTile; tj <= j1 / kTile; ++tj) {
        for (int ti = i0 / kTile; ti <= i1 / kTile; ++ti) {
            auto& tile = tiles_[static_cast<std::size_t>(tj) * static_cast<std::size_t>(tilesU_) +
                                static_cast<std::size_t>(ti)];
            if (tile.use_count() > 1) {
                tile = std::make_shared<Tile>(*tile);
            }
        }
    }
}

double DexelGrid::volume() const {
    double sum = 0.0;
    for (const auto& tile : tiles_) {
        for (const auto& r : tile->rays) {
            sum += totalLength(r);
        }
    }
    return sum * h_ * h_;
}

void DexelGrid::fillFromMesh(const geom3d::Mesh& mesh) {
    const int ua = uAxis();
    const int va = vAxis();
    std::vector<std::vector<std::pair<double, int>>> hits(static_cast<std::size_t>(nu_) *
                                                          static_cast<std::size_t>(nv_));

    for (const auto& tri : mesh.triangles) {
        const Vec3& p0 = mesh.vertices[tri[0]];
        const Vec3& p1 = mesh.vertices[tri[1]];
        const Vec3& p2 = mesh.vertices[tri[2]];
        const Vec2 a(p0[ua], p0[va]);
        Vec2 b(p1[ua], p1[va]);
        Vec2 c(p2[ua], p2[va]);
        Vec3 wb = p1;
        Vec3 wc = p2;
        const double area = (b - a).x() * (c - a).y() - (b - a).y() * (c - a).x();
        if (std::abs(area) < 1e-18) {
            continue; // parallel to the rays
        }
        // Orientation along the ray: entering (normal against the ray direction) or leaving.
        const int sign = area > 0 ? 1 : -1; // projected CCW = normal along +axis = leaving
        if (area < 0) {
            std::swap(b, c);
            std::swap(wb, wc);
        }
        // Edge functions are evaluated with the endpoints in a canonical order so the two
        // triangles sharing an edge get exactly opposite values (no double or missed hits).
        auto edge = [](const Vec2& p, const Vec2& q, const Vec2& r) {
            const bool swap = q.x() < p.x() || (q.x() == p.x() && q.y() < p.y());
            const Vec2& s0 = swap ? q : p;
            const Vec2& s1 = swap ? p : q;
            const double w =
                (s1.x() - s0.x()) * (r.y() - s0.y()) - (s1.y() - s0.y()) * (r.x() - s0.x());
            return swap ? -w : w;
        };
        // Top-left rule: include points exactly on top or left edges only.
        auto topLeft = [](const Vec2& p, const Vec2& q) {
            const Vec2 d = q - p;
            return (d.y() == 0.0 && d.x() < 0.0) || d.y() > 0.0;
        };
        const bool tl0 = topLeft(b, c), tl1 = topLeft(c, a), tl2 = topLeft(a, b);
        const double umin = std::min({a.x(), b.x(), c.x()}), umax = std::max({a.x(), b.x(), c.x()});
        const double vmin = std::min({a.y(), b.y(), c.y()}), vmax = std::max({a.y(), b.y(), c.y()});
        const auto [i0, i1] = uRange(umin, umax);
        const auto [j0, j1] = vRange(vmin, vmax);
        for (int j = j0; j <= j1; ++j) {
            for (int i = i0; i <= i1; ++i) {
                const Vec2 p(u(i), v(j));
                const double w0 = edge(b, c, p), w1 = edge(c, a, p), w2 = edge(a, b, p);
                if ((w0 > 0 || (w0 == 0 && tl0)) && (w1 > 0 || (w1 == 0 && tl1)) &&
                    (w2 > 0 || (w2 == 0 && tl2))) {
                    const double t =
                        (w0 * p0[axis_] + w1 * wb[axis_] + w2 * wc[axis_]) / (w0 + w1 + w2);
                    hits[static_cast<std::size_t>(j) * static_cast<std::size_t>(nu_) +
                         static_cast<std::size_t>(i)]
                        .emplace_back(t, sign);
                }
            }
        }
    }

    for (int j = 0; j < nv_; ++j) {
        for (int i = 0; i < nu_; ++i) {
            auto& h = hits[static_cast<std::size_t>(j) * static_cast<std::size_t>(nu_) +
                           static_cast<std::size_t>(i)];
            if (h.empty()) {
                continue;
            }
            std::sort(h.begin(), h.end());
            IntervalList& list = mutableRay(i, j);
            list.clear();
            int depth = 0;
            double start = 0.0;
            for (const auto& [t, s] : h) {
                // Entering faces (s < 0) open material, leaving faces (s > 0) close it.
                if (s < 0) {
                    if (depth++ == 0) {
                        start = t;
                    }
                } else if (depth > 0 && --depth == 0 && t > start) {
                    list.push_back({static_cast<float>(start), static_cast<float>(t)});
                }
            }
        }
    }
}

// -- Stock ------------------------------------------------------------------------------------

Stock Stock::fromMesh(const geom3d::Mesh& mesh, const geom3d::Aabb3& boundsIn, double resolution,
                      bool triDexel) {
    if (resolution <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "simulation resolution must be positive");
    }
    Stock s;
    s.resolution_ = resolution;
    s.triDexel_ = triDexel;
    s.bounds_ = boundsIn;
    s.bounds_.min -= Vec3::Constant(resolution);
    s.bounds_.max += Vec3::Constant(resolution);
    const Vec3 size = s.bounds_.size();
    const double cells = (size.x() / resolution) * (size.y() / resolution);
    if (cells > 4e7) {
        throw Error(ErrorCode::InvalidArgument,
                    "simulation resolution is too fine for this stock size");
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (!triDexel && axis != 2) {
            continue;
        }
        const int ua = (axis + 1) % 3;
        const int va = (axis + 2) % 3;
        const int nu = std::max(1, static_cast<int>(std::ceil(size[ua] / resolution)));
        const int nv = std::max(1, static_cast<int>(std::ceil(size[va] / resolution)));
        s.grids_[static_cast<std::size_t>(axis)] =
            DexelGrid(axis, s.bounds_.min[ua], s.bounds_.min[va], resolution, nu, nv);
        s.grids_[static_cast<std::size_t>(axis)].fillFromMesh(mesh);
    }
    return s;
}

Stock Stock::box(const geom3d::Aabb3& b, double resolution, bool triDexel) {
    return fromMesh(boxMesh(b), b, resolution, triDexel);
}

geom3d::Mesh boxMesh(const geom3d::Aabb3& b) {
    geom3d::Mesh m;
    for (int i = 0; i < 8; ++i) {
        m.vertices.emplace_back((i & 1) ? b.max.x() : b.min.x(), (i & 2) ? b.max.y() : b.min.y(),
                                (i & 4) ? b.max.z() : b.min.z());
    }
    // Faces: -Z, +Z, -Y, +Y, -X, +X (outward, counter-clockwise from outside).
    const std::uint32_t quads[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4},
                                       {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
    for (const auto& q : quads) {
        m.triangles.push_back({q[0], q[1], q[2]});
        m.triangles.push_back({q[0], q[2], q[3]});
    }
    return m;
}

} // namespace ocam::sim
