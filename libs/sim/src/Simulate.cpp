// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Swept-volume removal on the dexel grids. Each move is split into pieces: straight segments
// (horizontal, vertical, or with a small height change) and horizontal arcs. For a ray along Z
// at horizontal distance d from a piece, the removed range is the tool's height span at d,
// widened by the piece's height range; this is exact for horizontal and vertical motion and
// over-cuts sloped motion by at most the split height (a quarter cell). Rays along X and Y see
// the tool as a stack of disks: the cross-section of the swept volume at height z is a capsule
// whose radius is the largest tool radius over the heights the tip passes. Removal commutes,
// so pending pieces are applied in parallel batches; a batch is flushed before any collision
// check so checks always see the stock in program order.
// Moves with a tilted tool (phase 9) are split so the axis turns little within each piece, and
// each piece is swept exactly on all three grids from the tool's convex slices (Tilted.hpp).
// See docs/design-log/0012 and 0026.

#include "ocam/sim/Simulate.hpp"

#include "Tilted.hpp"

#include <ocam/foundation/Error.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <tuple>

namespace ocam::sim {

std::string_view toString(CollisionKind kind) noexcept {
    switch (kind) {
    case CollisionKind::Holder:
        return "holder";
    case CollisionKind::Shank:
        return "shank";
    case CollisionKind::Rapid:
        return "rapid";
    case CollisionKind::NoHole:
        return "no hole";
    case CollisionKind::Probe:
        return "probe";
    }
    return "unknown";
}

namespace {

using toolpath::Move;
using toolpath::MoveKind;

struct ZPiece {
    Vec2 a = Vec2::Zero();
    Vec2 b = Vec2::Zero();
    double za = 0.0; ///< Lowest tip height.
    double zb = 0.0; ///< Highest tip height.
    bool arc = false;
    Vec2 center = Vec2::Zero();
    double radius = 0.0;
    double a0 = 0.0;
    double sweep = 0.0;
    Vec2 lo = Vec2::Zero(); ///< XY bounds of the path itself.
    Vec2 hi = Vec2::Zero();
    bool sloped = false; ///< Straight piece whose height changes (swept exactly, see flushZ).
    double z0 = 0.0;     ///< Tip height at a.
    double z1 = 0.0;     ///< Tip height at b.
};

struct LinePiece {
    Vec3 a = Vec3::Zero();
    Vec3 b = Vec3::Zero();
};

/// A straight piece of a move with a tilted tool: tip from p0 to p1 with the axis u.
struct TiltPiece {
    Vec3 p0 = Vec3::Zero();
    Vec3 p1 = Vec3::Zero();
    Vec3 u = Vec3::UnitZ();
};

/// Ray origin (coordinate 0 along the grid's axis) of ray (i, j).
Vec3 rayOrigin(const DexelGrid& g, int i, int j) {
    Vec3 o = Vec3::Zero();
    o[g.uAxis()] = g.u(i);
    o[g.vAxis()] = g.v(j);
    return o;
}

double distToSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const double len2 = ab.squaredNorm();
    if (len2 < 1e-24) {
        return (p - a).norm();
    }
    const double t = std::clamp((p - a).dot(ab) / len2, 0.0, 1.0);
    return (p - (a + t * ab)).norm();
}

double distToPiece(const Vec2& p, const ZPiece& s) {
    if (!s.arc) {
        return distToSegment(p, s.a, s.b);
    }
    const Vec2 v = p - s.center;
    const double r = v.norm();
    double rel = std::atan2(v.y(), v.x()) - s.a0;
    if (s.sweep < 0.0) {
        rel = -rel;
    }
    rel = std::fmod(rel, 2.0 * kPi);
    if (rel < 0.0) {
        rel += 2.0 * kPi;
    }
    if (rel <= std::abs(s.sweep)) {
        return std::abs(r - s.radius);
    }
    return std::min((p - s.a).norm(), (p - s.b).norm());
}

/// Range along the line `cross = c` (in (along, cross) coordinates) inside the capsule of
/// radius `rho` around segment a-b. Returns false when the line misses the capsule.
bool capsuleSpan(const Vec2& a, const Vec2& b, double rho, double c, double& lo, double& hi) {
    lo = std::numeric_limits<double>::infinity();
    hi = -lo;
    for (const Vec2* p : {&a, &b}) {
        const double dy = c - p->y();
        if (std::abs(dy) <= rho) {
            const double w = std::sqrt(rho * rho - dy * dy);
            lo = std::min(lo, p->x() - w);
            hi = std::max(hi, p->x() + w);
        }
    }
    const Vec2 dir = b - a;
    const double len = dir.norm();
    if (len > 1e-12 && std::abs(dir.y()) > 1e-12) {
        const Vec2 n(-dir.y() / len, dir.x() / len);
        for (const double side : {-1.0, 1.0}) {
            const Vec2 o = a + side * rho * n;
            const double t = (c - o.y()) / dir.y();
            if (t >= 0.0 && t <= 1.0) {
                const double x = o.x() + t * dir.x();
                lo = std::min(lo, x);
                hi = std::max(hi, x);
            }
        }
    }
    return lo <= hi;
}

struct IndexRange {
    int i0 = 0, i1 = -1, j0 = 0, j1 = -1;
    bool empty() const noexcept { return i0 > i1 || j0 > j1; }
};

/// Applies pending pieces to the grids in parallel.
class Remover {
public:
    Remover(Stock& stock, const ToolShape& tool) : stock_(stock), tool_(tool) {}

    void add(const ZPiece& p) { zPieces_.push_back(p); }
    void add(const LinePiece& p) {
        if (stock_.triDexel()) {
            linePieces_.push_back(p);
        }
    }
    void add(const TiltPiece& p) { tiltPieces_.push_back(p); }
    std::size_t pending() const noexcept {
        return zPieces_.size() + linePieces_.size() + tiltPieces_.size();
    }

    void flush() {
        if (!zPieces_.empty()) {
            flushZ();
        }
        if (!linePieces_.empty()) {
            flushSide(0);
            flushSide(1);
        }
        if (!tiltPieces_.empty()) {
            for (int axis = 0; axis < 3; ++axis) {
                if (axis == 2 || stock_.triDexel()) {
                    flushTilted(axis);
                }
            }
        }
        zPieces_.clear();
        linePieces_.clear();
        tiltPieces_.clear();
    }

private:
    template <class Fn>
    void forEachBinned(DexelGrid& g, const std::vector<IndexRange>& ranges, Fn&& fn) {
        bins_.assign(static_cast<std::size_t>(g.tileRows()), {});
        for (std::size_t p = 0; p < ranges.size(); ++p) {
            const IndexRange& r = ranges[p];
            if (r.empty()) {
                continue;
            }
            g.detach(r.i0, r.i1, r.j0, r.j1);
            for (int t = r.j0 / DexelGrid::kTile; t <= r.j1 / DexelGrid::kTile; ++t) {
                bins_[static_cast<std::size_t>(t)].push_back(static_cast<std::uint32_t>(p));
            }
        }
        rows_.clear();
        for (std::size_t t = 0; t < bins_.size(); ++t) {
            if (!bins_[t].empty()) {
                rows_.push_back(static_cast<int>(t));
            }
        }
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, rows_.size()),
                          [&](const tbb::blocked_range<std::size_t>& br) {
                              for (std::size_t k = br.begin(); k != br.end(); ++k) {
                                  const int t = rows_[k];
                                  const int rowLo = t * DexelGrid::kTile;
                                  const int rowHi = rowLo + DexelGrid::kTile - 1;
                                  for (const std::uint32_t p : bins_[static_cast<std::size_t>(t)]) {
                                      const IndexRange& r = ranges[p];
                                      for (int j = std::max(r.j0, rowLo);
                                           j <= std::min(r.j1, rowHi); ++j) {
                                          fn(p, j);
                                      }
                                  }
                              }
                          });
    }

    void flushZ() {
        DexelGrid& g = stock_.grid(2);
        const double R = tool_.cutRadius();
        ranges_.resize(zPieces_.size());
        for (std::size_t p = 0; p < zPieces_.size(); ++p) {
            const ZPiece& s = zPieces_[p];
            IndexRange& r = ranges_[p];
            std::tie(r.i0, r.i1) = g.uRange(s.lo.x() - R, s.hi.x() + R);
            std::tie(r.j0, r.j1) = g.vRange(s.lo.y() - R, s.hi.y() + R);
        }
        forEachBinned(g, ranges_, [&](std::uint32_t p, int j) {
            const ZPiece& s = zPieces_[p];
            const IndexRange& r = ranges_[p];
            const double y = g.v(j);
            int i0 = r.i0, i1 = r.i1;
            if (!s.arc) {
                double xlo = 0.0, xhi = 0.0;
                if (!capsuleSpan(s.a, s.b, R, y, xlo, xhi)) {
                    return;
                }
                const auto [a, b] = g.uRange(xlo, xhi);
                i0 = std::max(i0, a);
                i1 = std::min(i1, b);
            }
            for (int i = i0; i <= i1; ++i) {
                const IntervalList& ray = g.ray(i, j);
                if (ray.empty() || ray.back().hi <= s.za) {
                    continue;
                }
                const Vec2 pt(g.u(i), y);
                const double d = distToPiece(pt, s);
                if (d > R) {
                    continue;
                }
                const HeightSpans spans = tool_.spansAt(d);
                if (spans.count == 0) {
                    continue;
                }
                if (s.sloped && spans.count == 1) {
                    // Lowest point of the tool swept along the slope: z(t) + lo(d(t)) is convex
                    // in t (lo convex and non-decreasing, d convex), so a golden-section search
                    // over the part of the move within reach finds it.
                    if (ray.back().hi <= s.za + spans.spans[0].lo) {
                        continue;
                    }
                    const Vec2 e = s.b - s.a;
                    const Vec2 w = s.a - pt;
                    const double l2 = e.squaredNorm();
                    const double B = 2.0 * w.dot(e);
                    const double C = w.squaredNorm() - R * R;
                    const double disc = std::max(0.0, B * B - 4.0 * l2 * C);
                    double t0 = std::max(0.0, (-B - std::sqrt(disc)) / (2.0 * l2));
                    double t1 = std::min(1.0, (-B + std::sqrt(disc)) / (2.0 * l2));
                    auto f = [&](double t) {
                        const HeightSpans at = tool_.spansAt((w + t * e).norm());
                        return at.count > 0 ? s.z0 + t * (s.z1 - s.z0) + at.spans[0].lo
                                            : std::numeric_limits<double>::infinity();
                    };
                    double low = std::min(f(t0), f(t1));
                    constexpr double phi = 0.6180339887498949;
                    double x1 = t1 - phi * (t1 - t0), x2 = t0 + phi * (t1 - t0);
                    double f1 = f(x1), f2 = f(x2);
                    for (int it = 0; it < 32 && t1 - t0 > 1e-9; ++it) {
                        if (f1 > f2) {
                            t0 = x1;
                            x1 = x2;
                            f1 = f2;
                            x2 = t0 + phi * (t1 - t0);
                            f2 = f(x2);
                        } else {
                            t1 = x2;
                            x2 = x1;
                            f2 = f1;
                            x1 = t1 - phi * (t1 - t0);
                            f1 = f(x1);
                        }
                    }
                    low = std::min({low, f1, f2});
                    subtract(g.mutableRay(i, j), low, s.zb + spans.spans[0].hi);
                    continue;
                }
                IntervalList& mray = g.mutableRay(i, j);
                for (int c = 0; c < spans.count; ++c) {
                    const Interval& sp = spans.spans[static_cast<std::size_t>(c)];
                    subtract(mray, s.za + sp.lo, s.zb + sp.hi);
                }
            }
        });
    }

    /// Tilted pieces on the grid of rays along `axis`.
    void flushTilted(int axis) {
        DexelGrid& g = stock_.grid(axis);
        if (g.isEmpty()) {
            return;
        }
        const auto& pieces = tool_.cutterPieces();
        const std::size_t n = tiltPieces_.size();
        ranges_.resize(n);
        along_.resize(n);
        for (std::size_t p = 0; p < n; ++p) {
            const TiltPiece& s = tiltPieces_[p];
            geom3d::Aabb3 b = detail::piecesBounds(pieces, s.p0, s.u);
            b.extend(detail::piecesBounds(pieces, s.p1, s.u));
            IndexRange& r = ranges_[p];
            std::tie(r.i0, r.i1) = g.uRange(b.min[g.uAxis()], b.max[g.uAxis()]);
            std::tie(r.j0, r.j1) = g.vRange(b.min[g.vAxis()], b.max[g.vAxis()]);
            along_[p] = {b.min[axis], b.max[axis]};
        }
        forEachBinned(g, ranges_, [&](std::uint32_t p, int j) {
            const TiltPiece& s = tiltPieces_[p];
            const IndexRange& r = ranges_[p];
            const Vec3 d = s.p1 - s.p0;
            for (int i = r.i0; i <= r.i1; ++i) {
                const IntervalList& ray = g.ray(i, j);
                if (ray.empty() || ray.back().hi < along_[p].first ||
                    ray.front().lo > along_[p].second) {
                    continue;
                }
                const Vec3 origin = rayOrigin(g, i, j);
                for (const SolidPiece& piece : pieces) {
                    double lo = 0.0, hi = 0.0;
                    if (detail::sweptSpan(piece, s.p0, d, s.u, origin, axis, lo, hi)) {
                        subtract(g.mutableRay(i, j), lo, hi);
                    }
                }
            }
        });
    }

    /// Rays along X (axis 0: u = Y, v = Z) or Y (axis 1: u = Z, v = X).
    void flushSide(int axis) {
        DexelGrid& g = stock_.grid(axis);
        if (g.isEmpty()) {
            return;
        }
        const int cross = 1 - axis;
        const bool zIsV = axis == 0;
        const double R = tool_.cutRadius();
        const double L = tool_.cuttingLength();
        const std::size_t n = linePieces_.size();
        ranges_.resize(n);
        zBase_.resize(n);
        rho_.resize(n);
        for (std::size_t p = 0; p < n; ++p) {
            const LinePiece& s = linePieces_[p];
            const double za = std::min(s.a.z(), s.b.z());
            const double zb = std::max(s.a.z(), s.b.z());
            const double c0 = std::min(s.a[cross], s.b[cross]) - R;
            const double c1 = std::max(s.a[cross], s.b[cross]) + R;
            IndexRange& r = ranges_[p];
            const auto [z0, z1] = zIsV ? g.vRange(za, zb + L) : g.uRange(za, zb + L);
            const auto [k0, k1] = zIsV ? g.uRange(c0, c1) : g.vRange(c0, c1);
            if (zIsV) {
                r = {k0, k1, z0, z1};
            } else {
                r = {z0, z1, k0, k1};
            }
            // Capsule radius for each Z index of the piece.
            zBase_[p] = z0;
            auto& rho = rho_[p];
            rho.clear();
            for (int k = z0; k <= z1; ++k) {
                const double z = zIsV ? g.v(k) : g.u(k);
                rho.push_back(static_cast<float>(tool_.maxRadiusIn(z - zb, z - za)));
            }
        }
        forEachBinned(g, ranges_, [&](std::uint32_t p, int j) {
            const LinePiece& s = linePieces_[p];
            const IndexRange& r = ranges_[p];
            const Vec2 a(s.a[axis], s.a[cross]);
            const Vec2 b(s.b[axis], s.b[cross]);
            const auto& rho = rho_[p];
            for (int i = r.i0; i <= r.i1; ++i) {
                const int zk = zIsV ? j : i;
                const double rr = rho[static_cast<std::size_t>(zk - zBase_[p])];
                if (rr <= 0.0) {
                    if (zIsV) {
                        return; // the whole row is at this height
                    }
                    continue;
                }
                const IntervalList& ray = g.ray(i, j);
                if (ray.empty()) {
                    continue;
                }
                const double c = zIsV ? g.u(i) : g.v(j);
                double lo = 0.0, hi = 0.0;
                if (capsuleSpan(a, b, rr, c, lo, hi)) {
                    subtract(g.mutableRay(i, j), lo, hi);
                }
            }
        });
    }

    Stock& stock_;
    const ToolShape& tool_;
    std::vector<ZPiece> zPieces_;
    std::vector<LinePiece> linePieces_;
    std::vector<TiltPiece> tiltPieces_;
    std::vector<std::pair<double, double>> along_;
    std::vector<IndexRange> ranges_;
    std::vector<int> zBase_;
    std::vector<std::vector<float>> rho_;
    std::vector<std::vector<std::uint32_t>> bins_;
    std::vector<int> rows_;
};

struct Hit {
    bool hit = false;
    Vec3 position = Vec3::Zero();
    double depth = 0.0;
};

/// Scans the Z rays within `radius` of a piece; `overlap(ray, d)` returns the material length
/// in conflict. Stops at the first conflict longer than `tol`.
template <class Fn>
Hit scanZ(const Stock& stock, const ZPiece& s, double radius, double tol, Fn&& overlap) {
    const DexelGrid& g = stock.grid(2);
    const auto [i0, i1] = g.uRange(s.lo.x() - radius, s.hi.x() + radius);
    const auto [j0, j1] = g.vRange(s.lo.y() - radius, s.hi.y() + radius);
    for (int j = j0; j <= j1; ++j) {
        for (int i = i0; i <= i1; ++i) {
            const IntervalList& ray = g.ray(i, j);
            if (ray.empty()) {
                continue;
            }
            const Vec2 p(g.u(i), g.v(j));
            const double d = distToPiece(p, s);
            if (d > radius) {
                continue;
            }
            const double len = overlap(ray, d);
            if (len > tol) {
                return {true, Vec3(p.x(), p.y(), std::min(topOf(ray), s.zb + radius)), len};
            }
        }
    }
    return {};
}

/// Scans the rays near a tilted piece for material inside the solid that `pieces` sweep;
/// stops at the first overlap longer than `tol`.
Hit scanTilted(const Stock& stock, const std::vector<SolidPiece>& pieces, const TiltPiece& s,
               double tol) {
    if (pieces.empty()) {
        return {};
    }
    geom3d::Aabb3 b = detail::piecesBounds(pieces, s.p0, s.u);
    b.extend(detail::piecesBounds(pieces, s.p1, s.u));
    const Vec3 d = s.p1 - s.p0;
    for (int axis = 2; axis >= 0; --axis) {
        if (axis != 2 && !stock.triDexel()) {
            continue;
        }
        const DexelGrid& g = stock.grid(axis);
        if (g.isEmpty()) {
            continue;
        }
        const auto [i0, i1] = g.uRange(b.min[g.uAxis()], b.max[g.uAxis()]);
        const auto [j0, j1] = g.vRange(b.min[g.vAxis()], b.max[g.vAxis()]);
        for (int j = j0; j <= j1; ++j) {
            for (int i = i0; i <= i1; ++i) {
                const IntervalList& ray = g.ray(i, j);
                if (ray.empty() || ray.back().hi < b.min[axis] || ray.front().lo > b.max[axis]) {
                    continue;
                }
                const Vec3 origin = rayOrigin(g, i, j);
                for (const SolidPiece& piece : pieces) {
                    double lo = 0.0, hi = 0.0;
                    if (!detail::sweptSpan(piece, s.p0, d, s.u, origin, axis, lo, hi)) {
                        continue;
                    }
                    const double len = overlapLength(ray, lo, hi);
                    if (len > tol) {
                        Vec3 at = origin;
                        at[axis] = 0.5 * (lo + hi);
                        return {true, at, len};
                    }
                }
            }
        }
    }
    return {};
}

class Simulation {
public:
    Simulation(Stock& stock, const ToolShape& tool, const SimulateOptions& options)
        : stock_(stock), tool_(tool), options_(options), remover_(stock, tool) {
        h_ = stock.resolution();
        arcTol_ = options.arcTolerance > 0.0 ? options.arcTolerance : h_ / 8.0;
        topZ_ = stock.bounds().max.z();
        // Collision checks ignore grazing contact, like the 3-axis checks (0.1 cell).
        shankCheck_ = detail::shrink(tool.shankPieces(), 0.1 * h_);
        holderCheck_ = detail::shrink(tool.holderPieces(), 0.1 * h_);
    }

    /// A move from p0 with the tool axis turning from a0 to the move's axis.
    void motion(const Vec3& p0, const Vec3& a0, const Move& m, std::size_t index) {
        if ((a0 - Vec3::UnitZ()).squaredNorm() > 1e-18 ||
            (m.axis - Vec3::UnitZ()).squaredNorm() > 1e-18) {
            tiltedMotion(p0, a0, m, index);
            return;
        }
        motion(p0, m, index);
    }

    void motion(const Vec3& p0, const Move& m, std::size_t index) {
        if (m.orient) {
            // The spindle stands still: the tool cuts nothing, and its head is off to one side of
            // the axis, which a turning tool's swept shape cannot show. The strategy checked that
            // it passes (DL-0083).
            return;
        }
        zPieces_.clear();
        linePieces_.clear();
        if (m.isArc()) {
            arcPieces(p0, m);
        } else {
            linePiecesFor(p0, m.end);
        }
        if (zPieces_.empty()) {
            return;
        }
        double zmin = std::numeric_limits<double>::infinity();
        for (const ZPiece& s : zPieces_) {
            zmin = std::min(zmin, s.za);
        }
        const bool check = options_.checkCollisions;
        const double L = tool_.cuttingLength();
        const double vis = tool_.visibleLength();
        const bool shank = check && tool_.shankRadius() > 0.0 && vis > L && zmin + L < topZ_;
        const bool holder = check && !tool_.holder().empty() && zmin + vis < topZ_;
        if (m.kind == MoveKind::Probe) {
            // A probe never removes material (DL-0090): it is checked, stylus, body and holder.
            if (check && zmin < topZ_) {
                remover_.flush();
                probeCheck(index);
                shankAndHolder(shank, holder, index);
            }
            return;
        }
        if (check && m.kind == MoveKind::Rapid && zmin < topZ_) {
            remover_.flush();
            for (const ZPiece& s : zPieces_) {
                const Hit hit = scanZ(stock_, s, tool_.cutRadius(), options_.collisionTolerance,
                                      [&](const IntervalList& ray, double d) {
                                          const HeightSpans spans = tool_.spansAt(d);
                                          double len = 0.0;
                                          for (int c = 0; c < spans.count; ++c) {
                                              const Interval& sp =
                                                  spans.spans[static_cast<std::size_t>(c)];
                                              len += overlapLength(ray, s.za + sp.lo, s.zb + sp.hi);
                                          }
                                          return len;
                                      });
                if (hit.hit) {
                    record(CollisionKind::Rapid, index, hit);
                    break;
                }
            }
        }
        for (const ZPiece& s : zPieces_) {
            remover_.add(s);
        }
        for (const LinePiece& s : linePieces_) {
            remover_.add(s);
        }
        if (shank || holder) {
            remover_.flush();
            shankAndHolder(shank, holder, index);
        } else if (remover_.pending() > 4096) {
            remover_.flush();
        }
    }

    void finish() { remover_.flush(); }
    std::vector<Collision> takeCollisions() { return std::move(collisions_); }

private:
    /// A probe move: the stylus meeting material, beyond what a touch may (its trigger travel),
    /// is reported. The stylus is checked shrunk by that allowance, sideways and along Z.
    void probeCheck(std::size_t index) {
        const double allow = options_.probeAllowance;
        const double tol = options_.collisionTolerance;
        for (const ZPiece& s : zPieces_) {
            const Hit hit =
                scanZ(stock_, s, std::max(0.0, tool_.cutRadius() - allow), tol,
                      [&](const IntervalList& ray, double d) {
                          const HeightSpans spans = tool_.spansAt(d + allow);
                          double len = 0.0;
                          for (int c = 0; c < spans.count; ++c) {
                              const Interval& sp = spans.spans[static_cast<std::size_t>(c)];
                              len += overlapLength(ray, s.za + sp.lo + allow, s.zb + sp.hi - allow);
                          }
                          return len;
                      });
            if (hit.hit) {
                record(CollisionKind::Probe, index, hit);
                return;
            }
        }
    }

    /// The shank and holder against the stock, for the current pieces (removals flushed).
    void shankAndHolder(bool shank, bool holder, std::size_t index) {
        if (!shank && !holder) {
            return;
        }
        const double L = tool_.cuttingLength();
        const double vis = tool_.visibleLength();
        const double eps = 0.1 * h_;
        const double tol = options_.collisionTolerance;
        for (const ZPiece& s : zPieces_) {
            if (shank) {
                // Band by band: a narrow neck passes where the shank above it would not (DL-0083).
                const double rs = tool_.shankRadius() - eps;
                const Hit hit = scanZ(stock_, s, rs, tol, [&](const IntervalList& ray, double d) {
                    double len = 0.0;
                    for (const HolderBand& band : tool_.shankBands()) {
                        if (band.radius - eps > d) {
                            len = std::max(len, overlapLength(ray, s.za + std::max(band.h0, L),
                                                              s.zb + std::min(band.h1, vis)));
                        }
                    }
                    return len;
                });
                if (hit.hit) {
                    record(CollisionKind::Shank, index, hit);
                    break;
                }
            }
            if (holder) {
                const Hit hit = scanZ(
                    stock_, s, tool_.holderRadius(), tol, [&](const IntervalList& ray, double d) {
                        double len = 0.0;
                        for (const HolderBand& band : tool_.holder()) {
                            if (band.radius > d + eps) {
                                len = std::max(len,
                                               overlapLength(ray, s.za + band.h0, s.zb + band.h1));
                            }
                        }
                        return len;
                    });
                if (hit.hit) {
                    record(CollisionKind::Holder, index, hit);
                    break;
                }
            }
        }
    }

    void tiltedMotion(const Vec3& p0, const Vec3& a0, const Move& m, std::size_t index) {
        std::vector<Vec3> pts{p0, m.end};
        if (m.isArc()) {
            pts = toolpath::sampleArc(p0, m, arcTol_);
        }
        const Vec3 a1 = m.axis.normalized();
        const double turn = std::acos(std::clamp(a0.normalized().dot(a1), -1.0, 1.0));
        // Split so the far end of the cutting part moves at most half a cell by the turning.
        const double reach = tool_.cuttingLength() + tool_.cutRadius();
        const int splits =
            std::max(1, static_cast<int>(std::min(1e5, std::ceil(turn * reach / (0.5 * h_)))));
        tilt_.clear();
        const std::size_t segs = pts.size() - 1;
        for (std::size_t k = 0; k < segs; ++k) {
            const int n = std::max(1, static_cast<int>(std::ceil(static_cast<double>(splits) /
                                                                 static_cast<double>(segs))));
            for (int j = 0; j < n; ++j) {
                const double ta = (k + static_cast<double>(j) / n) / segs;
                const double tb = (k + static_cast<double>(j + 1) / n) / segs;
                const Vec3 q0 = pts[k] + (pts[k + 1] - pts[k]) * (static_cast<double>(j) / n);
                const Vec3 q1 = pts[k] + (pts[k + 1] - pts[k]) * (static_cast<double>(j + 1) / n);
                tilt_.push_back({q0, q1, toolpath::slerp(a0.normalized(), a1, 0.5 * (ta + tb))});
            }
        }
        if (turn > 1e-12) {
            // The pieces turn the tool in steps; also place it at its exact final pose, so the
            // next move starts from material this pose has removed.
            tilt_.push_back({pts.back(), pts.back(), a1});
        }
        double zmin = std::numeric_limits<double>::infinity();
        for (const TiltPiece& s : tilt_) {
            zmin = std::min({zmin, s.p0.z() - tool_.cutRadius(), s.p1.z() - tool_.cutRadius()});
        }
        const bool check = options_.checkCollisions;
        const double tol = options_.collisionTolerance;
        const bool probe = m.kind == MoveKind::Probe;
        if (probe && check && zmin < topZ_) {
            remover_.flush();
            const std::vector<SolidPiece> stylus =
                detail::shrink(tool_.cutterPieces(), options_.probeAllowance);
            for (const TiltPiece& s : tilt_) {
                const Hit hit = scanTilted(stock_, stylus, s, tol);
                if (hit.hit) {
                    record(CollisionKind::Probe, index, hit);
                    break;
                }
            }
        }
        if (check && m.kind == MoveKind::Rapid && zmin < topZ_) {
            remover_.flush();
            for (const TiltPiece& s : tilt_) {
                const Hit hit = scanTilted(stock_, tool_.cutterPieces(), s, tol);
                if (hit.hit) {
                    record(CollisionKind::Rapid, index, hit);
                    break;
                }
            }
        }
        if (!probe) {
            for (const TiltPiece& s : tilt_) {
                remover_.add(s); // a probe never removes material
            }
        }
        auto below = [&](const std::vector<SolidPiece>& pieces) {
            if (pieces.empty()) {
                return false;
            }
            for (const TiltPiece& s : tilt_) {
                if (std::min(detail::piecesBounds(pieces, s.p0, s.u).min.z(),
                             detail::piecesBounds(pieces, s.p1, s.u).min.z()) < topZ_) {
                    return true;
                }
            }
            return false;
        };
        const bool shank = check && below(shankCheck_);
        const bool holder = check && below(holderCheck_);
        if (shank || holder) {
            remover_.flush();
            for (const TiltPiece& s : tilt_) {
                if (shank) {
                    const Hit hit = scanTilted(stock_, shankCheck_, s, tol);
                    if (hit.hit) {
                        record(CollisionKind::Shank, index, hit);
                        break;
                    }
                }
                if (holder) {
                    const Hit hit = scanTilted(stock_, holderCheck_, s, tol);
                    if (hit.hit) {
                        record(CollisionKind::Holder, index, hit);
                        break;
                    }
                }
            }
        } else if (remover_.pending() > 4096) {
            remover_.flush();
        }
    }

    void record(CollisionKind kind, std::size_t index, const Hit& hit) {
        for (auto it = collisions_.rbegin(); it != collisions_.rend() && it->lastMove + 1 >= index;
             ++it) {
            if (it->kind == kind) {
                it->lastMove = std::max(it->lastMove, index);
                it->depth = std::max(it->depth, hit.depth);
                return;
            }
        }
        if (collisions_.size() < options_.maxCollisions) {
            collisions_.push_back({kind, index, index, hit.position, hit.depth});
        }
    }

    void linePiecesFor(const Vec3& p, const Vec3& q) {
        const double len = (q.head<2>() - p.head<2>()).norm();
        const double dz = std::abs(q.z() - p.z());
        if (len < 1e-12 && dz < 1e-12) {
            return;
        }
        // X/Y rays (and Z rays of multi-span tools) see sloped moves as steps of at most a
        // quarter cell; Z rays of single-span tools sweep the whole slope exactly.
        const double step = h_ / 4.0;
        const int n = (len > 1e-9 && dz > step)
                          ? static_cast<int>(std::min(100000.0, std::ceil(dz / step)))
                          : 1;
        const bool exact = n > 1 && tool_.singleSpan();
        for (int k = 0; k < n; ++k) {
            const Vec3 a = p + (q - p) * (static_cast<double>(k) / n);
            const Vec3 b = p + (q - p) * (static_cast<double>(k + 1) / n);
            if (!exact) {
                zPieces_.push_back(zPiece(a, b));
            }
            linePieces_.push_back({a, b});
        }
        if (exact) {
            zPieces_.push_back(zPiece(p, q));
        }
    }

    static ZPiece zPiece(const Vec3& a, const Vec3& b) {
        ZPiece s;
        s.a = a.head<2>();
        s.b = b.head<2>();
        s.z0 = a.z();
        s.z1 = b.z();
        s.za = std::min(a.z(), b.z());
        s.zb = std::max(a.z(), b.z());
        s.lo = s.a.cwiseMin(s.b);
        s.hi = s.a.cwiseMax(s.b);
        s.sloped = (s.b - s.a).squaredNorm() > 1e-18 && s.zb - s.za > 1e-12;
        return s;
    }

    void arcPieces(const Vec3& p0, const Move& m) {
        // Arcs in a vertical plane (ZX, YZ) as short lines, finely: their chords dip below a
        // convex arc (a finishing pass over a dome's top) by the sampling tolerance, and at the
        // XY arcs' tolerance (an eighth of a ray spacing) that read as a gouge.
        if (m.plane != toolpath::ArcPlane::XY) {
            const std::vector<Vec3> pts = toolpath::sampleArc(p0, m, std::min(arcTol_, 1e-3));
            for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
                linePiecesFor(pts[k], pts[k + 1]);
            }
            return;
        }
        const std::vector<Vec3> pts = toolpath::sampleArc(p0, m, arcTol_);
        // Helices as short lines.
        const bool helical = std::abs(m.end.z() - p0.z()) > 1e-9;
        if (helical) {
            for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
                linePiecesFor(pts[k], pts[k + 1]);
            }
            return;
        }
        ZPiece s;
        s.arc = true;
        s.a = p0.head<2>();
        s.b = m.end.head<2>();
        s.za = s.zb = p0.z();
        s.center = m.center.head<2>();
        s.radius = (s.a - s.center).norm();
        s.a0 = std::atan2(s.a.y() - s.center.y(), s.a.x() - s.center.x());
        s.sweep = toolpath::arcSweep(p0, m);
        s.lo = s.a.cwiseMin(s.b);
        s.hi = s.a.cwiseMax(s.b);
        for (const Vec3& p : pts) {
            s.lo = s.lo.cwiseMin(p.head<2>());
            s.hi = s.hi.cwiseMax(p.head<2>());
        }
        s.lo -= Vec2::Constant(arcTol_);
        s.hi += Vec2::Constant(arcTol_);
        zPieces_.push_back(s);
        for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
            linePieces_.push_back({pts[k], pts[k + 1]});
        }
    }

    Stock& stock_;
    const ToolShape& tool_;
    SimulateOptions options_;
    Remover remover_;
    double h_ = 1.0;
    double arcTol_ = 0.1;
    double topZ_ = 0.0;
    std::vector<ZPiece> zPieces_;
    std::vector<LinePiece> linePieces_;
    std::vector<TiltPiece> tilt_;
    std::vector<SolidPiece> shankCheck_;
    std::vector<SolidPiece> holderCheck_;
    std::vector<Collision> collisions_;
};

} // namespace

std::vector<Collision> tapsWithoutAHole(const Stock& stock, const toolpath::Toolpath& path,
                                        double coreRadius, CancelToken* cancel) {
    std::vector<Collision> out;
    if (coreRadius <= 0.0) {
        return out;
    }
    tooling::Tool core = tooling::makeFlatEndMill(1, 2.0 * coreRadius, 1e4, 2e4);
    const ToolShape shape = ToolShape::fromTool(core);
    SimulateOptions options;
    options.checkCollisions = false;
    const double area = kPi * coreRadius * coreRadius;
    Stock probe = stock;
    double volume = probe.volume();
    Vec3 pos = path.start;
    for (std::size_t k = 0; k < path.moves.size(); ++k) {
        const Move& m = path.moves[k];
        if (m.kind == MoveKind::Dwell) {
            continue;
        }
        const double in = (pos - m.end).dot(m.axis);
        if (m.kind == MoveKind::Linear && in > 1e-6) {
            toolpath::Toolpath stroke;
            stroke.start = pos;
            stroke.moves.push_back(m);
            simulateMoves(probe, shape, stroke, 0, 1, options, cancel);
            const double left = probe.volume();
            const double solid = (volume - left) / area;
            if (solid > std::max(0.5 * stock.resolution(), tooling::tapPointAllowance(coreRadius))) {
                out.push_back({CollisionKind::NoHole, k, k, m.end, solid});
            }
            volume = left;
        }
        pos = m.end;
    }
    return out;
}

SimulateResult simulateToolpath(Stock& stock, const ToolShape& tool, const toolpath::Toolpath& path,
                                const SimulateOptions& options, CancelToken* cancel,
                                ProgressSink* progress, std::size_t moveLimit) {
    return simulateMoves(stock, tool, path, 0, moveLimit, options, cancel, progress);
}

SimulateResult simulateMoves(Stock& stock, const ToolShape& tool, const toolpath::Toolpath& path,
                             std::size_t begin, std::size_t end, const SimulateOptions& options,
                             CancelToken* cancel, ProgressSink* progress) {
    if (tool.cutRadius() <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the tool has no cutting radius");
    }
    SimulateResult result;
    Simulation sim(stock, tool, options);
    const std::size_t n = std::min(end, path.moves.size());
    Vec3 pos = toolPositionAt(path, begin);
    Vec3 axis = toolpath::axisBefore(path, begin);
    for (std::size_t k = begin; k < n; ++k) {
        if (cancel && (k & 255u) == 0) {
            cancel->throwIfCancelled();
        }
        if (progress && (k & 1023u) == 0) {
            progress->report(static_cast<double>(k - begin) /
                                 static_cast<double>(std::max<std::size_t>(n - begin, 1)),
                             "Simulating");
        }
        const Move& m = path.moves[k];
        if (m.kind == MoveKind::Dwell) {
            continue;
        }
        if (m.kind == MoveKind::Cycle) {
            toolpath::Toolpath mini;
            mini.start = pos;
            mini.cycles.push_back(path.cycles.at(static_cast<std::size_t>(m.cycle)));
            Move cm = m;
            cm.cycle = 0;
            mini.moves.push_back(cm);
            Vec3 q = pos;
            for (const Move& em : toolpath::expandCycles(mini).moves) {
                if (em.kind == MoveKind::Dwell) {
                    continue;
                }
                sim.motion(q, em, k);
                q = em.end;
            }
            if ((q - m.end).norm() > 1e-9) {
                Move back;
                back.kind = MoveKind::Rapid;
                back.feedClass = toolpath::FeedClass::Rapid;
                back.end = m.end;
                sim.motion(q, back, k);
            }
        } else {
            sim.motion(pos, axis, m, k);
        }
        pos = m.end;
        axis = m.axis;
    }
    sim.finish();
    result.movesSimulated = n > begin ? n - begin : 0;
    result.collisions = sim.takeCollisions();
    if (progress) {
        progress->report(1.0, "Simulated");
    }
    return result;
}

Vec3 toolPositionAt(const toolpath::Toolpath& path, std::size_t moveIndex) {
    Vec3 pos = path.start;
    const std::size_t n = std::min(moveIndex, path.moves.size());
    for (std::size_t k = 0; k < n; ++k) {
        if (path.moves[k].kind != MoveKind::Dwell) {
            pos = path.moves[k].end;
        }
    }
    return pos;
}

} // namespace ocam::sim
