// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pencil finishing: passes along the concave corners and fillets a larger (reference) tool could
// not reach into. The corners are the creases of the reference tool's cutter-location surface,
// where it touches two faces at once (bitangent), found with the drop-cutter (DL-0016). Each
// pass point is where the tool's contact normal halves the reference tool's two contact normals.
// See docs/design-log/0080.

#include "Internal.hpp"
#include "ocam/strategies/Surface.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <unordered_map>

namespace ocam::strategies {
namespace {

using toolpath::FeedClass;

/// Sharpest concave bend of a cutter-location section.
struct Bend {
    double t = 0.0;     ///< Where, along the section.
    double turn = 0.0;  ///< How much the section's direction turns there [rad].
    double left = 0.0;  ///< Slope angle of the section before the bend [rad].
    double right = 0.0; ///< And after it.
};

/// Along p + t u for t in [t0, t1]: the sharpest concave bend of the cutter-location section,
/// from n + 1 exact drops. The turn is read between the chords on either side of each chord, so
/// a bend between two samples counts whole. Nothing where a drop lands on the floor (no part).
std::optional<Bend> sharpestBend(const DropCutter& dc, const Vec2& p, const Vec2& u, double t0,
                                 double t1, int n, double floor) {
    const double dt = (t1 - t0) / n;
    std::vector<double> z(static_cast<std::size_t>(n) + 1);
    for (int k = 0; k <= n; ++k) {
        z[k] = dc.drop(p + (t0 + k * dt) * u, floor);
        if (z[k] <= floor + 1e-9) {
            return std::nullopt;
        }
    }
    std::vector<double> a(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k) {
        a[k] = std::atan((z[k + 1] - z[k]) / dt);
    }
    Bend best;
    best.turn = -1.0;
    for (int k = 1; k + 1 < n; ++k) {
        const double turn = a[k + 1] - a[k - 1];
        if (turn > best.turn) {
            best.turn = turn;
            // Where the chords before and after meet, within the middle chord.
            const double sa = std::tan(a[k - 1]);
            const double sb = std::tan(a[k + 1]);
            const double ta = t0 + (k - 1) * dt;
            const double tb = t0 + (k + 1) * dt;
            double t = t0 + (k + 0.5) * dt;
            if (std::abs(sa - sb) > 1e-12) {
                t = (z[k + 1] - z[k - 1] + sa * ta - sb * tb) / (sa - sb);
            }
            best.t = std::clamp(t, t0 + k * dt, t0 + (k + 1) * dt);
            best.left = a[k - 1];
            best.right = a[k + 1];
        }
    }
    if (best.turn <= 0.0) {
        return std::nullopt;
    }
    return best;
}

/// Plan points joined into chains: each step to the nearest point not yet used within `link`
/// that does not turn back on itself; points close beside a chain are absorbed into it.
std::vector<PencilCorner> chainPoints(const std::vector<Vec2>& pts, double link) {
    const double cell = link;
    auto key = [&](const Vec2& p) {
        const auto ix = static_cast<std::int64_t>(std::floor(p.x() / cell));
        const auto iy = static_cast<std::int64_t>(std::floor(p.y() / cell));
        return (ix << 32) ^ (iy & 0xffffffff);
    };
    std::unordered_map<std::int64_t, std::vector<std::size_t>> grid;
    for (std::size_t k = 0; k < pts.size(); ++k) {
        grid[key(pts[k])].push_back(k);
    }
    auto near = [&](const Vec2& p, double radius, auto&& visit) {
        const auto ix = static_cast<std::int64_t>(std::floor(p.x() / cell));
        const auto iy = static_cast<std::int64_t>(std::floor(p.y() / cell));
        const int reach = static_cast<int>(std::ceil(radius / cell));
        for (std::int64_t dx = -reach; dx <= reach; ++dx) {
            for (std::int64_t dy = -reach; dy <= reach; ++dy) {
                const auto it = grid.find(((ix + dx) << 32) ^ ((iy + dy) & 0xffffffff));
                if (it == grid.end()) {
                    continue;
                }
                for (const std::size_t q : it->second) {
                    if ((pts[q] - p).norm() <= radius) {
                        visit(q);
                    }
                }
            }
        }
    };
    std::vector<char> used(pts.size(), 0);
    const double maxTurnCos = std::cos(100.0 * kPi / 180.0);
    auto extend = [&](std::vector<std::size_t>& chain) {
        for (;;) {
            const Vec2& here = pts[chain.back()];
            std::optional<Vec2> dir;
            if (chain.size() >= 2) {
                const Vec2 d =
                    here - pts[chain[chain.size() - std::min<std::size_t>(chain.size(), 3)]];
                if (d.norm() > 1e-9) {
                    dir = d.normalized();
                }
            }
            std::size_t best = pts.size();
            double bestScore = std::numeric_limits<double>::infinity();
            near(here, link, [&](std::size_t q) {
                if (used[q]) {
                    return;
                }
                const Vec2 d = pts[q] - here;
                const double len = d.norm();
                double score = len;
                if (dir && len > 1e-9) {
                    const double c = dir->dot(d / len);
                    if (c < maxTurnCos) {
                        return;
                    }
                    score *= 2.0 - c;
                }
                if (score < bestScore) {
                    bestScore = score;
                    best = q;
                }
            });
            if (best == pts.size()) {
                return;
            }
            used[best] = 1;
            chain.push_back(best);
        }
    };
    std::vector<PencilCorner> out;
    for (std::size_t seed = 0; seed < pts.size(); ++seed) {
        if (used[seed]) {
            continue;
        }
        used[seed] = 1;
        std::vector<std::size_t> chain{seed};
        extend(chain);
        std::reverse(chain.begin(), chain.end());
        extend(chain);
        // Points beside the chain (found from both sides of a slanted crease) belong to it.
        for (const std::size_t k : chain) {
            near(pts[k], 0.6 * link, [&](std::size_t q) { used[q] = 1; });
        }
        PencilCorner c;
        for (const std::size_t k : chain) {
            c.points.push_back(pts[k]);
        }
        double length = 0.0;
        for (std::size_t k = 1; k < c.points.size(); ++k) {
            length += (c.points[k] - c.points[k - 1]).norm();
        }
        if (c.points.size() < 3 || length < 2.0 * link) {
            continue;
        }
        c.closed = c.points.size() > 4 && (c.points.front() - c.points.back()).norm() <= link;
        // A light smoothing (the points come from two grid directions).
        if (c.points.size() >= 3) {
            std::vector<Vec2> smooth = c.points;
            const std::size_t n = c.points.size();
            for (std::size_t k = 0; k < n; ++k) {
                if (!c.closed && (k == 0 || k + 1 == n)) {
                    continue;
                }
                const Vec2& a = c.points[(k + n - 1) % n];
                const Vec2& b = c.points[(k + 1) % n];
                smooth[k] = 0.25 * a + 0.5 * c.points[k] + 0.25 * b;
            }
            c.points = std::move(smooth);
        }
        out.push_back(std::move(c));
    }
    return out;
}

/// A 3D pass with its points in order.
struct Piece {
    std::vector<Vec3> points;
    bool closed = false;
};

} // namespace

std::vector<PencilCorner> pencilCorners(const DropCutter& reference, const geom2d::Aabb2& area,
                                        double spacing, double floor, double cornerAngle,
                                        CancelToken& cancel) {
    const HeightField field = computeHeightField(reference, area, spacing, floor, cancel);
    const double h = field.spacing;
    const double gridTurn = 0.5 * cornerAngle;
    auto onPart = [&](int i, int j) {
        return field.at(i, j) > floor + 1e-9;
    };
    // Candidates along both grid directions: a concave turn of the section over a grid point,
    // refined on the exact drop-cutter within a cell either side.
    std::vector<std::vector<Vec2>> rows(static_cast<std::size_t>(field.ny));
    tbb::parallel_for(
        tbb::blocked_range<int>(1, std::max(1, field.ny - 1)),
        [&](const tbb::blocked_range<int>& range) {
            for (int j = range.begin(); j != range.end(); ++j) {
                if (cancel.isCancelled()) {
                    return;
                }
                for (int i = 1; i + 1 < field.nx; ++i) {
                    if (!onPart(i, j)) {
                        continue;
                    }
                    for (const bool alongX : {true, false}) {
                        const int pi = alongX ? i - 1 : i, pj = alongX ? j : j - 1;
                        const int ni = alongX ? i + 1 : i, nj = alongX ? j : j + 1;
                        if (!onPart(pi, pj) || !onPart(ni, nj)) {
                            continue;
                        }
                        const double before = std::atan((field.at(i, j) - field.at(pi, pj)) / h);
                        const double after = std::atan((field.at(ni, nj) - field.at(i, j)) / h);
                        if (after - before < gridTurn) {
                            continue;
                        }
                        const Vec2 u = alongX ? Vec2(1.0, 0.0) : Vec2(0.0, 1.0);
                        const Vec2 p = field.point(i, j);
                        const auto bend = sharpestBend(reference, p, u, -h, h, 32, floor);
                        if (bend && bend->turn >= gridTurn) {
                            rows[static_cast<std::size_t>(j)].push_back(p + bend->t * u);
                        }
                    }
                }
            }
        });
    cancel.throwIfCancelled();
    // One point per crease crossing: merge the two directions' finds.
    std::vector<Vec2> pts;
    {
        const double merge = 0.35 * h;
        std::unordered_map<std::int64_t, std::vector<std::size_t>> grid;
        auto key = [&](std::int64_t ix, std::int64_t iy) {
            return (ix << 32) ^ (iy & 0xffffffff);
        };
        for (const auto& row : rows) {
            for (const Vec2& p : row) {
                const auto ix = static_cast<std::int64_t>(std::floor(p.x() / merge));
                const auto iy = static_cast<std::int64_t>(std::floor(p.y() / merge));
                bool dup = false;
                for (std::int64_t dx = -1; dx <= 1 && !dup; ++dx) {
                    for (std::int64_t dy = -1; dy <= 1 && !dup; ++dy) {
                        const auto it = grid.find(key(ix + dx, iy + dy));
                        if (it == grid.end()) {
                            continue;
                        }
                        for (const std::size_t q : it->second) {
                            if ((pts[q] - p).norm() < merge) {
                                dup = true;
                                break;
                            }
                        }
                    }
                }
                if (!dup) {
                    grid[key(ix, iy)].push_back(pts.size());
                    pts.push_back(p);
                }
            }
        }
    }
    return chainPoints(pts, 1.6 * h);
}

toolpath::Toolpath pencilFinish(const geom3d::Mesh& part, const PencilParams& p,
                                const Context& ctx) {
    const double r = detail::requireMillingTool(ctx.tool);
    if (part.isEmpty()) {
        throw Error(ErrorCode::InvalidArgument, "pencil finishing needs the part's surface");
    }
    const Cutter real(ctx.tool);
    if (real.kind() != Cutter::Kind::Torus) {
        throw Error(ErrorCode::InvalidArgument,
                    "pencil finishing needs a ball, bull-nose or flat end mill");
    }
    const double refR = p.referenceRadius > 0.0 ? p.referenceRadius : r;
    if (refR < r - 1e-9) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("the reference tool ({:.3f} mm) is smaller than the tool "
                                "({:.3f} mm): it must be the larger tool that cut before",
                                2.0 * refR, 2.0 * r));
    }
    if (p.offsetPasses > 0 && p.stepover <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the offset passes need a stepover");
    }
    const double allowance = p.stockToLeave;
    const DropCutter dc(part, real.offset(allowance));
    const DropCutter ref(part, real.offset(allowance + (refR - r)));
    const double refRadius = ref.cutter().radius();
    const geom3d::Aabb3 bounds = part.bounds();
    const double h = p.resolution > 0.0 ? p.resolution : std::clamp(r / 4.0, 0.05, 0.5);
    const double floor = bounds.min.z() - 10.0;
    const double tol = std::max(ctx.chordTolerance, 1e-4);
    geom2d::Aabb2 area;
    area.extend(bounds.min.head<2>() - Vec2::Constant(refRadius + 3.0 * h));
    area.extend(bounds.max.head<2>() + Vec2::Constant(refRadius + 3.0 * h));
    area = detail::containedArea(area, p.boundary, refRadius + 3.0 * h);

    toolpath::Toolpath tp = beginToolpath(ctx, "pencil3d", area.center());
    ProgressRange findRange(ctx.progress, 0.0, 0.6);
    findRange.report(0.0, "Finding corners");
    const std::vector<PencilCorner> corners =
        pencilCorners(ref, area, h, floor, p.cornerAngle, ctx.cancel);
    findRange.report(1.0, "Finding corners");

    // The tool's tip height (with the allowance) and its section slope.
    const auto tipAt = [&](const Vec2& xy) {
        return dc.drop(xy, floor - allowance) + allowance;
    };
    const double eps = std::clamp(h / 16.0, 1e-3, 0.02);
    const double steep = std::tan(80.0 * kPi / 180.0);
    const int sides = std::max(0, p.offsetPasses);
    const auto slots = static_cast<std::size_t>(2 * sides + 1);

    // Per corner: for each pass (index -sides..sides, stored at index + sides) the point at each
    // corner point, if there is one.
    struct CornerPasses {
        std::vector<std::vector<std::optional<Vec3>>> pass;
        bool closed = false;
    };
    std::vector<CornerPasses> all(corners.size());
    tbb::parallel_for(std::size_t{0}, corners.size(), [&](std::size_t c) {
        if (ctx.cancel.isCancelled()) {
            return;
        }
        const PencilCorner& corner = corners[c];
        const std::size_t n = corner.points.size();
        CornerPasses& out = all[c];
        out.closed = corner.closed;
        out.pass.assign(slots, std::vector<std::optional<Vec3>>(n));
        for (std::size_t k = 0; k < n; ++k) {
            const auto at = [&](std::ptrdiff_t m) {
                if (corner.closed) {
                    return corner.points[static_cast<std::size_t>(
                        (m % static_cast<std::ptrdiff_t>(n) + static_cast<std::ptrdiff_t>(n)) %
                        static_cast<std::ptrdiff_t>(n))];
                }
                return corner.points[static_cast<std::size_t>(
                    std::clamp<std::ptrdiff_t>(m, 0, static_cast<std::ptrdiff_t>(n) - 1))];
            };
            const auto km = static_cast<std::ptrdiff_t>(k);
            const Vec2 tangent = at(km + 2) - at(km - 2);
            if (tangent.norm() < 1e-9) {
                continue;
            }
            const Vec2 t = tangent.normalized();
            const Vec2 u(-t.y(), t.x());
            // The crease across the corner, and the reference tool's contact slopes either side.
            const auto bend = sharpestBend(ref, corner.points[k], u, -h, h, 32, floor);
            if (!bend || bend->turn < p.cornerAngle) {
                continue;
            }
            const Vec2 crease = corner.points[k] + bend->t * u;
            if (ref.drop(crease, floor) < bounds.min.z() - 1e-6) {
                continue; // under the part's bottom edge: the tool going off the part
            }
            // Where the tool's section slope halves the reference tool's two contact normals.
            const double target = std::tan(0.5 * (bend->left + bend->right));
            const auto slopeAt = [&](double s) -> std::optional<double> {
                const Vec2 a = crease + (s - eps) * u;
                const Vec2 b = crease + (s + eps) * u;
                const double za = dc.drop(a, floor);
                const double zb = dc.drop(b, floor);
                if (za <= floor + 1e-9 || zb <= floor + 1e-9) {
                    return std::nullopt;
                }
                return (zb - za) / (2.0 * eps);
            };
            const auto g0 = slopeAt(0.0);
            if (!g0) {
                continue;
            }
            double lo = 0.0;
            double hi = 0.0;
            bool found = std::abs(*g0 - target) < 1e-12;
            if (!found) {
                // The section turns up across the corner: the slope grows with t.
                const double dir = *g0 < target ? 1.0 : -1.0;
                const double step = 0.25 * h;
                double s = 0.0;
                while (std::abs(s) <= refRadius + h) {
                    const double next = s + dir * step;
                    const auto g = slopeAt(next);
                    if (!g) {
                        break;
                    }
                    if ((*g - target) * dir >= 0.0) {
                        lo = std::min(s, next);
                        hi = std::max(s, next);
                        found = true;
                        break;
                    }
                    s = next;
                }
                if (!found) {
                    continue;
                }
                for (int it = 0; it < 40 && hi - lo > 1e-5; ++it) {
                    const double mid = 0.5 * (lo + hi);
                    const auto g = slopeAt(mid);
                    if (!g) {
                        break;
                    }
                    if (*g < target) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
            }
            const double sStar = 0.5 * (lo + hi);
            const Vec2 q = crease + sStar * u;
            if (!detail::anyContains(p.boundary, q, tol) && !p.boundary.empty()) {
                continue;
            }
            out.pass[static_cast<std::size_t>(sides)][k] = Vec3(q.x(), q.y(), tipAt(q));
            if (sides == 0) {
                continue;
            }
            // Offset passes, a stepover apart along the tool's section, out to where the
            // reference tool touched (its contact lies R sin(angle) from the crease), and not up
            // walls steeper than 80 degrees (those are constant-Z work).
            for (const int side : {-1, 1}) {
                const double contact = refRadius * std::sin(side < 0 ? bend->left : bend->right);
                const double dt = std::min(p.stepover / 6.0, 0.25 * h);
                double s = sStar;
                Vec3 prev(q.x(), q.y(), tipAt(q));
                double along = 0.0;
                int next = 1;
                for (int guard = 0; guard < 100000 && next <= sides; ++guard) {
                    const double sn = s + side * dt;
                    const Vec2 xy = crease + sn * u;
                    const double zt = dc.drop(xy, floor);
                    if (zt <= floor + 1e-9) {
                        break;
                    }
                    const Vec3 cur(xy.x(), xy.y(), zt + allowance);
                    const double rise = std::abs(cur.z() - prev.z()) / dt;
                    if (rise > steep) {
                        break;
                    }
                    const double seg = (cur - prev).norm();
                    if (along + seg >= next * p.stepover) {
                        const double f = (next * p.stepover - along) / std::max(seg, 1e-12);
                        const Vec2 at2 = (prev + f * (cur - prev)).head<2>();
                        // The tool's contact across the section, against the reference's.
                        const double slope = (cur.z() - prev.z()) / (side * dt);
                        const double tc = sn + r * std::sin(std::atan(slope));
                        if (side * (tc - contact) > 0.5 * p.stepover) {
                            break;
                        }
                        if (p.boundary.empty() || detail::anyContains(p.boundary, at2, tol)) {
                            out.pass[static_cast<std::size_t>(sides + side * next)][k] =
                                Vec3(at2.x(), at2.y(), tipAt(at2));
                        }
                        ++next;
                    }
                    along += seg;
                    prev = cur;
                    s = sn;
                }
            }
        }
    });
    ctx.cancel.throwIfCancelled();
    ctx.progress.report(0.9, "Pencil passes");

    // Pieces: runs of the points a pass has, split where a gap is wider than a few cells.
    const double gap = 3.0 * h;
    struct Group {
        std::vector<std::vector<Piece>> byPass; ///< Index -sides..sides.
    };
    std::vector<Group> groups;
    for (const CornerPasses& cp : all) {
        Group g;
        g.byPass.resize(slots);
        bool any = false;
        for (std::size_t s = 0; s < slots; ++s) {
            const auto& pts = cp.pass[s];
            std::vector<std::vector<Vec3>> runs;
            std::vector<Vec3> run;
            for (const auto& q : pts) {
                if (!q) {
                    continue;
                }
                if (!run.empty() && (run.back().head<2>() - q->head<2>()).norm() > gap) {
                    runs.push_back(std::move(run));
                    run.clear();
                }
                run.push_back(*q);
            }
            if (!run.empty()) {
                runs.push_back(std::move(run));
            }
            bool closed = false;
            if (cp.closed && !runs.empty()) {
                const bool complete = std::all_of(pts.begin(), pts.end(),
                                                  [](const auto& q) { return q.has_value(); });
                if (complete && runs.size() == 1) {
                    closed = true;
                } else if (runs.size() > 1 &&
                           (runs.back().back().head<2>() - runs.front().front().head<2>()).norm() <=
                               gap) {
                    // The loop's seam falls inside a run: join its two ends.
                    runs.back().insert(runs.back().end(), runs.front().begin(), runs.front().end());
                    runs.erase(runs.begin());
                }
            }
            for (auto& rr : runs) {
                if (rr.size() < 2) {
                    continue;
                }
                double len = 0.0;
                for (std::size_t k = 1; k < rr.size(); ++k) {
                    len += (rr[k] - rr[k - 1]).norm();
                }
                if (len < 2.0 * h) {
                    continue;
                }
                Piece piece;
                piece.closed = closed;
                if (closed) {
                    rr.push_back(rr.front());
                }
                piece.points = detail::simplify3d(rr, tol);
                if (closed && piece.points.size() > 1) {
                    piece.points.pop_back();
                }
                g.byPass[s].push_back(std::move(piece));
                any = true;
            }
        }
        if (any) {
            groups.push_back(std::move(g));
        }
    }
    if (groups.empty()) {
        tp.warnings.push_back(
            "no corners the reference tool cannot reach into: nothing for pencil finishing");
        return tp;
    }

    // Cutting: corner by corner (the nearest next), each swept across its passes from the
    // outermost on the nearer side to the outermost on the other; a pass next to the last one
    // is reached along the surface, anything else by a retract and a plunge from above.
    toolpath::PathBuilder b(tp);
    const double retract =
        std::max(ctx.heights.retract, bounds.max.z() + allowance + ctx.heights.feedClearance);
    const double linkMax = std::max(2.5 * p.stepover, 2.0 * h);
    const auto nearestEnd = [](const Piece& piece, const Vec2& from) {
        double best = std::numeric_limits<double>::infinity();
        for (const Vec3& q : piece.points) {
            best = std::min(best, (q.head<2>() - from).norm());
        }
        return best;
    };
    const auto endsDistance = [](const Piece& piece, const Vec2& from) {
        if (piece.closed) {
            double best = std::numeric_limits<double>::infinity();
            for (const Vec3& q : piece.points) {
                best = std::min(best, (q.head<2>() - from).norm());
            }
            return best;
        }
        return std::min((piece.points.front().head<2>() - from).norm(),
                        (piece.points.back().head<2>() - from).norm());
    };
    // The piece oriented (or a loop restarted) to begin nearest `from`.
    const auto oriented = [](Piece piece, const Vec2& from) {
        if (piece.closed) {
            std::size_t best = 0;
            double bestD = std::numeric_limits<double>::infinity();
            for (std::size_t k = 0; k < piece.points.size(); ++k) {
                const double d = (piece.points[k].head<2>() - from).norm();
                if (d < bestD) {
                    bestD = d;
                    best = k;
                }
            }
            std::rotate(piece.points.begin(),
                        piece.points.begin() + static_cast<std::ptrdiff_t>(best),
                        piece.points.end());
            piece.points.push_back(piece.points.front());
        } else if ((piece.points.back().head<2>() - from).norm() <
                   (piece.points.front().head<2>() - from).norm()) {
            std::reverse(piece.points.begin(), piece.points.end());
        }
        return piece;
    };
    std::vector<char> doneGroup(groups.size(), 0);
    bool first = true;
    for (std::size_t count = 0; count < groups.size(); ++count) {
        ctx.cancel.throwIfCancelled();
        const Vec2 here = b.position().head<2>();
        std::size_t gi = groups.size();
        double gBest = std::numeric_limits<double>::infinity();
        bool fromLow = true;
        for (std::size_t g = 0; g < groups.size(); ++g) {
            if (doneGroup[g]) {
                continue;
            }
            // Its outermost passes on either side.
            const auto& passes = groups[g].byPass;
            std::size_t lowS = 0, highS = slots - 1;
            while (lowS < slots && passes[lowS].empty()) {
                ++lowS;
            }
            while (highS > 0 && passes[highS].empty()) {
                --highS;
            }
            for (const auto& piece : passes[lowS]) {
                if (const double d = nearestEnd(piece, here); d < gBest) {
                    gBest = d;
                    gi = g;
                    fromLow = true;
                }
            }
            for (const auto& piece : passes[highS]) {
                if (const double d = nearestEnd(piece, here); d < gBest) {
                    gBest = d;
                    gi = g;
                    fromLow = false;
                }
            }
        }
        if (gi == groups.size()) {
            break;
        }
        doneGroup[gi] = 1;
        const auto& passes = groups[gi].byPass;
        for (std::size_t step = 0; step < slots; ++step) {
            const std::size_t s = fromLow ? step : slots - 1 - step;
            std::vector<char> done(passes[s].size(), 0);
            for (std::size_t m = 0; m < passes[s].size(); ++m) {
                const Vec2 at = b.position().head<2>();
                std::size_t best = passes[s].size();
                double bestD = std::numeric_limits<double>::infinity();
                for (std::size_t k = 0; k < passes[s].size(); ++k) {
                    if (!done[k]) {
                        if (const double d = endsDistance(passes[s][k], at); d < bestD) {
                            bestD = d;
                            best = k;
                        }
                    }
                }
                done[best] = 1;
                const Piece piece = oriented(passes[s][best], at);
                const Vec3& start = piece.points.front();
                const double distance = (start.head<2>() - at).norm();
                if (!first && distance <= linkMax) {
                    // Over to the next pass along the surface (dropped: it cannot gouge).
                    const Vec2 from = at;
                    const int steps =
                        std::max(1, static_cast<int>(std::ceil(distance / (0.5 * h))));
                    std::vector<Vec3> link;
                    for (int k = 1; k < steps; ++k) {
                        const Vec2 xy = from + (start.head<2>() - from) * (double(k) / steps);
                        link.emplace_back(xy.x(), xy.y(), tipAt(xy));
                    }
                    link.push_back(start);
                    for (const Vec3& q : detail::simplify3d(link, tol)) {
                        b.feed(q, FeedClass::Link, ctx.feeds.cut);
                    }
                } else {
                    // From above: the reference tool may have left a lot in the corner.
                    b.retractTo(retract);
                    b.rapidAbove(start.head<2>(), retract, retract);
                    b.feed(start, FeedClass::Plunge, ctx.feeds.plunge);
                }
                for (std::size_t k = 1; k < piece.points.size(); ++k) {
                    b.feed(piece.points[k], FeedClass::Cut, ctx.feeds.cut);
                }
                first = false;
            }
        }
    }
    b.retractTo(ctx.heights.clearance);
    ctx.progress.report(1.0, "Pencil passes");
    return tp;
}

PencilParams pencilParams(const Json& values, const tooling::Tool& tool,
                          const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("pencil3d"), values);
    PencilParams p;
    // Unset, the reference tool is the operation's own (the ParamDef's tool default).
    p.referenceRadius = r.isSet("reference_diameter") ? 0.5 * r.number("reference_diameter")
                                                      : 0.5 * tool.mill.diameter;
    p.cornerAngle = r.number("corner_angle");
    p.offsetPasses = r.integer("offset_passes");
    if (p.offsetPasses > 0) {
        // The stepover between the offset passes: from the scallop height, else the preset's ae.
        const double scallop = r.number("scallop_height");
        if (scallop > 0.0) {
            const Cutter c(tool);
            const double rho = c.cornerRadius();
            const double hh = std::min(scallop, rho);
            p.stepover = rho > 0.0 ? 2.0 * std::sqrt(2.0 * rho * hh - hh * hh) : 0.0;
            if (p.stepover <= 0.0) {
                throw Error(ErrorCode::InvalidArgument,
                            "a scallop height sets the offset passes' stepover only for a tool "
                            "with a corner radius (ball or bull nose): set the preset's ae");
            }
        } else {
            p.stepover = presetStepover(preset);
        }
    }
    p.stockToLeave = r.number("stock_to_leave");
    p.resolution = r.number("resolution");
    return p;
}

} // namespace ocam::strategies
