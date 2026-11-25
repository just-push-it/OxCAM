// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// 3D finishing on drop-cutter: parallel (raster) lines and waterline contours. Stock to leave
// uses an offset tool (radius and corner radius grown by the allowance, tip lowered by it), so
// the real tool stays the allowance away from the part. See docs/design-log/0017.

#include "ocam/strategies/Surface.hpp"

#include "Internal.hpp"

#include <ocam/geom2d/PathOps.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <string_view>

namespace ocam::strategies {
namespace {

using toolpath::FeedClass;

std::vector<Vec3> simplify(const std::vector<Vec3>& pts, double tol) {
    return detail::simplify3d(pts, tol);
}

double scallopStepover(const Cutter& c, double h) {
    const double R = c.radius();
    double s = 1.8 * R;
    if (c.kind() == Cutter::Kind::Torus && c.cornerRadius() > 0.0 && h > 0.0) {
        const double rc = c.cornerRadius();
        const double hh = std::min(h, rc);
        s = std::min(s, 2.0 * (R - rc) + 2.0 * std::sqrt(2.0 * rc * hh - hh * hh));
    }
    return std::max(s, 1e-3);
}

} // namespace

toolpath::Toolpath parallelFinish(const geom3d::Mesh& part, double floorZ, const ParallelParams& p,
                                  const Context& ctx) {
    const double r = detail::requireMillingTool(ctx.tool);
    if (part.isEmpty()) {
        throw Error(ErrorCode::InvalidArgument, "parallel finishing needs the part's surface");
    }
    const Cutter real(ctx.tool);
    const double allowance = p.stockToLeave;
    const DropCutter dc(part, real.offset(allowance));
    const double stepover = p.stepover > 0.0 ? p.stepover : scallopStepover(real, p.scallopHeight);
    const double step = p.sampleStep > 0.0 ? p.sampleStep : std::clamp(r / 8.0, 0.02, 0.5);
    const double tol = std::max(ctx.chordTolerance, 1e-4);
    const geom3d::Aabb3 bounds = part.bounds();

    geom2d::Regions boundary = p.boundary;
    if (boundary.empty()) {
        boundary = {{geom2d::Polyline::rectangle(bounds.min.head<2>(), bounds.max.head<2>()), {}}};
    }
    // Work in raster coordinates (u along the lines, v across them).
    auto toRaster = [&](const Vec2& q) {
        return detail::rotate(q, -p.angle);
    };
    auto fromRaster = [&](const Vec2& q) {
        return detail::rotate(q, p.angle);
    };
    const auto loops = detail::discretizedLoops(boundary, tol, toRaster);
    double vmin = std::numeric_limits<double>::infinity(), vmax = -vmin;
    for (const auto& loop : loops) {
        for (const Vec2& q : loop) {
            vmin = std::min(vmin, q.y());
            vmax = std::max(vmax, q.y());
        }
    }
    toolpath::Toolpath tp = beginToolpath(ctx, "parallel3d", fromRaster(Vec2(0.0, vmin)));
    if (!(vmax >= vmin)) {
        tp.warnings.push_back("the boundary is empty");
        return tp;
    }
    const int lines = std::max(1, static_cast<int>(std::ceil((vmax - vmin) / stepover - 1e-9)) + 1);

    // Runs per line, computed in parallel.
    std::vector<std::vector<std::vector<Vec3>>> runs(static_cast<std::size_t>(lines));
    tbb::parallel_for(tbb::blocked_range<int>(0, lines), [&](const tbb::blocked_range<int>& range) {
        for (int k = range.begin(); k != range.end(); ++k) {
            if (ctx.cancel.isCancelled()) {
                return;
            }
            const double v = std::min(vmin + k * stepover, vmax);
            for (const auto& [ua, ub] : detail::scanIntervals(loops, v)) {
                std::vector<Vec3> pts;
                const int n = std::max(1, static_cast<int>(std::ceil((ub - ua) / step)));
                for (int s = 0; s <= n; ++s) {
                    const Vec2 xy = fromRaster(Vec2(ua + (ub - ua) * s / n, v));
                    const double z = dc.drop(xy, floorZ - allowance) + allowance;
                    pts.emplace_back(xy.x(), xy.y(), z);
                }
                runs[static_cast<std::size_t>(k)].push_back(simplify(pts, tol));
            }
        }
    });
    ctx.cancel.throwIfCancelled();

    toolpath::PathBuilder b(tp);
    const double retract =
        std::max(ctx.heights.retract, bounds.max.z() + allowance + ctx.heights.feedClearance);
    bool first = true;
    // A link from `from` to `to` that stays inside the boundary.
    const auto linkInside = [&](const Vec2& from, const Vec2& to) {
        if (p.boundary.empty()) {
            return true;
        }
        const double len = (to - from).norm();
        const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
        for (int t = 1; t < n; ++t) {
            if (!detail::anyContains(p.boundary, from + (to - from) * (double(t) / n), tol)) {
                return false;
            }
        }
        return true;
    };
    // `sameArea`: the next line of the same area (byArea), which may be linked further along the
    // surface while the link stays inside the boundary.
    const auto cutRun = [&](const std::vector<Vec3>& run, bool sameArea) {
        const Vec3& s = run.front();
        const double gap = (b.position().head<2>() - s.head<2>()).norm();
        const bool near =
            gap <= 2.5 * stepover ||
            (sameArea && gap <= 20.0 * stepover && linkInside(b.position().head<2>(), s.head<2>()));
        if (!first && p.zigzag && near) {
            // Step over along the surface: the dropped connecting line cannot gouge.
            const Vec2 from = b.position().head<2>();
            const int n = std::max(1, static_cast<int>(std::ceil(gap / step)));
            std::vector<Vec3> link;
            for (int t = 1; t <= n; ++t) {
                const Vec2 xy = from + (s.head<2>() - from) * (static_cast<double>(t) / n);
                link.emplace_back(xy.x(), xy.y(), dc.drop(xy, floorZ - allowance) + allowance);
            }
            for (const Vec3& q : simplify(link, tol)) {
                b.feed(q, FeedClass::Cut, ctx.feeds.cut);
            }
        } else {
            // Plunge at feed from the retract height: roughing may have left steps above
            // the surface.
            b.retractTo(retract);
            b.rapidAbove(s.head<2>(), retract, retract);
            b.feed(s, FeedClass::Plunge, ctx.feeds.plunge);
        }
        for (std::size_t q = 1; q < run.size(); ++q) {
            b.feed(run[q], FeedClass::Cut, ctx.feeds.cut);
        }
        first = false;
    };
    if (!p.byArea) {
        for (int k = 0; k < lines; ++k) {
            auto& lineRuns = runs[static_cast<std::size_t>(k)];
            const bool reverse = p.zigzag && (k % 2 == 1);
            if (reverse) {
                std::reverse(lineRuns.begin(), lineRuns.end());
            }
            for (auto& run : lineRuns) {
                if (run.empty()) {
                    continue;
                }
                if (reverse) {
                    std::reverse(run.begin(), run.end());
                }
                cutRun(run, false);
            }
            ctx.progress.report(static_cast<double>(k + 1) / lines, "Parallel finishing");
        }
        b.retractTo(ctx.heights.clearance);
        return tp;
    }

    // By area: the runs split into areas a line meets once each (a run continues the area of
    // the run on the line before that it overlaps, when each overlaps only the other), and each
    // area cut through all its lines before the next, the nearest first.
    struct Area {
        std::vector<std::pair<std::size_t, std::size_t>> runs; ///< (line, run), line by line.
    };
    std::vector<Area> areas;
    {
        const auto span = [&](const std::vector<Vec3>& run) {
            const double a = toRaster(run.front().head<2>()).x();
            const double c = toRaster(run.back().head<2>()).x();
            return std::pair{std::min(a, c), std::max(a, c)};
        };
        const auto overlaps = [](const std::pair<double, double>& x,
                                 const std::pair<double, double>& y) {
            return x.first <= y.second + 1e-9 && y.first <= x.second + 1e-9;
        };
        std::vector<std::pair<double, double>> before;
        std::vector<std::size_t> beforeArea;
        for (std::size_t k = 0; k < runs.size(); ++k) {
            std::vector<std::pair<double, double>> now;
            std::vector<std::size_t> nowRun;
            for (std::size_t j = 0; j < runs[k].size(); ++j) {
                if (!runs[k][j].empty()) {
                    now.push_back(span(runs[k][j]));
                    nowRun.push_back(j);
                }
            }
            std::vector<std::size_t> nowArea(now.size());
            for (std::size_t j = 0; j < now.size(); ++j) {
                std::size_t match = before.size();
                int count = 0;
                for (std::size_t i = 0; i < before.size(); ++i) {
                    if (overlaps(now[j], before[i])) {
                        match = i;
                        ++count;
                    }
                }
                bool continues = count == 1;
                if (continues) {
                    int back = 0;
                    for (const auto& other : now) {
                        back += overlaps(other, before[match]) ? 1 : 0;
                    }
                    continues = back == 1;
                }
                if (continues) {
                    nowArea[j] = beforeArea[match];
                } else {
                    nowArea[j] = areas.size();
                    areas.emplace_back();
                }
                areas[nowArea[j]].runs.emplace_back(k, nowRun[j]);
            }
            before = std::move(now);
            beforeArea = std::move(nowArea);
        }
    }
    std::vector<char> areaDone(areas.size(), 0);
    for (std::size_t count = 0; count < areas.size(); ++count) {
        ctx.cancel.throwIfCancelled();
        // The nearest area, entered at an end of its first or last line.
        const Vec2 here = b.position().head<2>();
        std::size_t best = areas.size();
        bool fromLast = false;
        bool backwards = false;
        double bestD = std::numeric_limits<double>::infinity();
        for (std::size_t a = 0; a < areas.size(); ++a) {
            if (areaDone[a]) {
                continue;
            }
            for (const bool last : {false, true}) {
                const auto [k, j] = last ? areas[a].runs.back() : areas[a].runs.front();
                const auto& run = runs[k][j];
                for (const bool rev : {false, true}) {
                    const double d = ((rev ? run.back() : run.front()).head<2>() - here).norm();
                    if (d < bestD) {
                        bestD = d;
                        best = a;
                        fromLast = last;
                        backwards = rev;
                    }
                }
            }
        }
        areaDone[best] = 1;
        auto order = areas[best].runs;
        if (fromLast) {
            std::reverse(order.begin(), order.end());
        }
        for (std::size_t m = 0; m < order.size(); ++m) {
            auto run = runs[order[m].first][order[m].second];
            const bool rev = p.zigzag ? (backwards != (m % 2 == 1)) : false;
            if (rev) {
                std::reverse(run.begin(), run.end());
            }
            cutRun(run, m > 0);
        }
        ctx.progress.report(std::min(1.0, static_cast<double>(count + 1) / areas.size()),
                            "Parallel finishing");
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

toolpath::Toolpath waterline(const geom3d::Mesh& part, const WaterlineParams& p,
                             const Context& ctx) {
    const double r = detail::requireMillingTool(ctx.tool);
    if (part.isEmpty()) {
        throw Error(ErrorCode::InvalidArgument, "waterline finishing needs the part's surface");
    }
    if (p.stepDown <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the waterline step down must be positive");
    }
    const double allowance = p.stockToLeave;
    const DropCutter dc(part, Cutter(ctx.tool).offset(allowance));
    const geom3d::Aabb3 bounds = part.bounds();
    const double res = p.resolution > 0.0 ? p.resolution : std::clamp(r / 6.0, 0.05, 1.0);
    const double floor = bounds.min.z() - 10.0;
    geom2d::Aabb2 area;
    area.extend(bounds.min.head<2>() - Vec2::Constant(r + allowance + 3.0 * res));
    area.extend(bounds.max.head<2>() + Vec2::Constant(r + allowance + 3.0 * res));
    area = detail::containedArea(area, p.boundary, 3.0 * res);
    const HeightField field = computeHeightField(dc, area, res, floor, ctx.cancel);

    toolpath::Toolpath tp = beginToolpath(ctx, "waterline3d", area.center());
    const auto levels = depthLevels(p.topZ, p.bottomZ, p.stepDown);
    if (levels.empty()) {
        tp.warnings.push_back("nothing to cut: the top is at or below the bottom");
        return tp;
    }
    toolpath::PathBuilder b(tp);
    const double retract =
        std::max(ctx.heights.retract, bounds.max.z() + allowance + ctx.heights.feedClearance);
    const double tol = std::max(ctx.chordTolerance, 1e-3);
    bool any = false;
    for (std::size_t li = 0; li < levels.size(); ++li) {
        ctx.cancel.throwIfCancelled();
        const double z = levels[li];
        // The offset tool's tip sits `allowance` below the real one.
        const geom2d::Regions forbidden = forbiddenRegions(dc, field, z - allowance, floor, tol);
        std::vector<geom2d::Polyline> loops;
        for (const auto& region : forbidden) {
            loops.push_back(region.outer);
            for (const auto& hole : region.holes) {
                loops.push_back(hole);
            }
        }
        for (geom2d::Polyline& loop : loops) {
            if (loop.length() < 4.0 * res) {
                continue;
            }
            // Regions keep the forbidden area (the part) on the left; climb milling (M03)
            // needs it on the right.
            if (p.direction == CutDirection::Climb) {
                loop.reverse();
            }
            for (geom2d::Polyline& piece : detail::containedPieces(loop, p.boundary, tol)) {
                const geom2d::Polyline path =
                    piece.isClosed() ? geom2d::restartAtNearest(piece, b.position().head<2>())
                                     : piece;
                const Vec2 s = path.vertices().front().p;
                b.retractTo(retract);
                b.rapidAbove(s, retract, retract);
                b.feed(Vec3(s.x(), s.y(), z), FeedClass::Plunge, ctx.feeds.plunge);
                b.follow(path, z, FeedClass::Cut, ctx.feeds.cut);
                any = true;
            }
        }
        ctx.progress.report(static_cast<double>(li + 1) / levels.size(), "Waterline");
    }
    if (!any) {
        tp.warnings.push_back("no part surface between the top and bottom heights");
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

toolpath::Toolpath contourFinish(const geom3d::Mesh& part, const ContourFinishParams& p,
                                 const Context& ctx) {
    const double r = detail::requireMillingTool(ctx.tool);
    if (part.isEmpty()) {
        throw Error(ErrorCode::InvalidArgument, "contour finishing needs the part's surface");
    }
    if (p.scallopHeight <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the cusp height must be positive");
    }
    const double allowance = p.stockToLeave;
    const Cutter real(ctx.tool);
    const DropCutter dc(part, real.offset(allowance));
    const geom3d::Aabb3 bounds = part.bounds();
    const double res = p.resolution > 0.0 ? p.resolution : std::clamp(r / 6.0, 0.05, 1.0);
    const double floor = bounds.min.z() - 10.0;
    geom2d::Aabb2 area;
    area.extend(bounds.min.head<2>() - Vec2::Constant(r + allowance + 3.0 * res));
    area.extend(bounds.max.head<2>() + Vec2::Constant(r + allowance + 3.0 * res));
    area = detail::containedArea(area, p.boundary, 3.0 * res);
    const HeightField field = computeHeightField(dc, area, res, floor, ctx.cancel);
    toolpath::Toolpath tp = beginToolpath(ctx, "contour3d", area.center());
    const double tol = std::max(ctx.chordTolerance, 1e-3);

    // The spacing of the levels along the surface that leaves the cusp height: across the
    // cutting radius in contact on a wall (a ball's radius, a bull nose's corner), the passes
    // s apart leave h = rho - sqrt(rho^2 - s^2 / 4). A flat end mill leaves stairs instead: a
    // level dz below the last one leaves dz cos(slope) of material normal to the surface.
    const double h = p.scallopHeight;
    const double rho = real.kind() == Cutter::Kind::Torus ? real.cornerRadius() : 0.0;
    const double sMax = rho > h ? 2.0 * std::sqrt(2.0 * rho * h - h * h) : 0.0;
    const double dzMax = p.maxStepDown > 0.0 ? p.maxStepDown
                         : sMax > 0.0        ? sMax
                                             : std::max(10.0 * h, 0.5 * r);
    const double dzMin = std::clamp(0.25 * h, 0.002, dzMax);
    const double minSlope = std::clamp(p.minSlope, 0.0, degToRad(89.0));

    const auto loopsAt = [&](double z, double tolerance, bool reversed = false) {
        // The offset tool's tip sits `allowance` below the real one.
        std::vector<geom2d::Polyline> loops;
        for (const auto& region : forbiddenRegions(dc, field, z - allowance, floor, tolerance)) {
            loops.push_back(region.outer);
            for (const auto& hole : region.holes) {
                loops.push_back(hole);
            }
        }
        std::erase_if(loops, [&](const geom2d::Polyline& l) { return l.length() < 4.0 * res; });
        if (reversed) {
            for (auto& loop : loops) {
                loop.reverse();
            }
        }
        if (p.boundary.empty()) {
            return loops;
        }
        // Contained: the pieces inside the boundary only.
        std::vector<geom2d::Polyline> pieces;
        for (const auto& loop : loops) {
            for (auto& piece : detail::containedPieces(loop, p.boundary, tol, 2.0 * res)) {
                pieces.push_back(std::move(piece));
            }
        }
        return pieces;
    };
    // The step down the loops of a level allow: at points along them (about every surface
    // resolution) the slope of the tool's contact surface, read with the drop cutter a little to
    // either side across the loop; walls shallower than the minimum slope are left out (the
    // flats' and the parallel finishing's). None steep enough: the largest step.
    const auto stepFor = [&](const std::vector<geom2d::Polyline>& loops) {
        double step = dzMax;
        const double eps = 0.5 * res;
        for (const auto& loop : loops) {
            const std::vector<Vec2> pts = loop.discretize(tol);
            const std::size_t n = pts.size();
            if (n < 3) {
                continue;
            }
            // Open pieces (contained passes) end where they end.
            const bool closed = loop.isClosed();
            const auto next = [&](std::size_t k) {
                return closed ? (k + 1) % n : std::min(k + 1, n - 1);
            };
            const auto prev = [&](std::size_t k) {
                return closed ? (k + n - 1) % n : (k == 0 ? 0 : k - 1);
            };
            double since = res;
            for (std::size_t k = 0; k < n; ++k) {
                const Vec2& a = pts[k];
                const Vec2& b = pts[next(k)];
                since += (b - a).norm();
                if (since < res) {
                    continue;
                }
                since = 0.0;
                const Vec2 t = (pts[next(k)] - pts[prev(k)]);
                if (t.norm() < 1e-12) {
                    continue;
                }
                const Vec2 normal = Vec2(-t.y(), t.x()).normalized();
                const double rise =
                    std::abs(dc.drop(a + eps * normal, floor) - dc.drop(a - eps * normal, floor));
                const double slope = std::atan2(rise, 2.0 * eps);
                if (slope < minSlope) {
                    continue;
                }
                const double allowed =
                    sMax > 0.0 ? sMax * std::sin(slope) : h / std::max(std::cos(slope), 1e-6);
                step = std::min(step, allowed);
            }
        }
        return std::clamp(step, dzMin, dzMax);
    };

    // The levels, from the top down: each the step its own loops and the next level's allow (a
    // concave fillet flattens on the way down), the first where the part begins below the top,
    // the last at the bottom.
    const double coarse = std::max(tol, 0.02);
    std::vector<double> levels;
    {
        double z = p.topZ - dzMin;
        double lastEmpty = std::numeric_limits<double>::quiet_NaN();
        // The trial level's loops, kept for when the next level lands on it.
        double cachedZ = std::numeric_limits<double>::quiet_NaN();
        std::vector<geom2d::Polyline> cached;
        for (int guard = 0; guard < 200000 && z >= p.bottomZ - 1e-9; ++guard) {
            ctx.cancel.throwIfCancelled();
            std::vector<geom2d::Polyline> loops =
                z == cachedZ ? std::move(cached) : loopsAt(z, coarse);
            cachedZ = std::numeric_limits<double>::quiet_NaN();
            if (loops.empty()) {
                lastEmpty = z;
                z -= std::min(dzMax, 1.0);
                continue;
            }
            if (!std::isnan(lastEmpty)) {
                // The part begins between the last empty level and this one: its top edge.
                double lo = z;
                double hi = lastEmpty;
                while (hi - lo > dzMin) {
                    const double mid = 0.5 * (lo + hi);
                    if (loopsAt(mid, coarse).empty()) {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                z = lo;
                loops = loopsAt(z, coarse);
                lastEmpty = std::numeric_limits<double>::quiet_NaN();
            }
            levels.push_back(z);
            if (z <= p.bottomZ + 1e-9) {
                break;
            }
            double step = stepFor(loops);
            const double next = z - step;
            if (next > p.bottomZ) {
                auto below = loopsAt(next, coarse);
                if (!below.empty()) {
                    const double belowStep = stepFor(below);
                    if (belowStep >= step) {
                        cached = std::move(below);
                        cachedZ = next;
                    }
                    step = std::min(step, belowStep);
                }
            }
            z = std::max(z - step, p.bottomZ);
        }
    }
    if (levels.empty()) {
        tp.warnings.push_back("no part surface between the top and bottom heights");
        return tp;
    }

    // Cutting: each level's loops in the cutting direction, the nearest first; down to the next
    // level by a short link at depth (kept on the surface) when it starts close by, else lifted.
    toolpath::PathBuilder b(tp);
    const double retract =
        std::max(ctx.heights.retract, bounds.max.z() + allowance + ctx.heights.feedClearance);
    const double linkMax = std::max({2.0 * dzMax, 4.0 * sMax, r});
    bool atDepth = false;
    for (std::size_t li = 0; li < levels.size(); ++li) {
        ctx.cancel.throwIfCancelled();
        const double z = levels[li];
        // Regions keep the part on the left; climb milling (M03) needs it on the right.
        std::vector<geom2d::Polyline> loops = loopsAt(z, tol, p.direction == CutDirection::Climb);
        std::vector<bool> done(loops.size(), false);
        for (std::size_t count = 0; count < loops.size(); ++count) {
            const Vec2 here = b.position().head<2>();
            std::size_t best = loops.size();
            double bestDistance = std::numeric_limits<double>::infinity();
            geom2d::Polyline bestPath;
            for (std::size_t k = 0; k < loops.size(); ++k) {
                if (done[k]) {
                    continue;
                }
                geom2d::Polyline path =
                    loops[k].isClosed() ? geom2d::restartAtNearest(loops[k], here) : loops[k];
                const double d = (path.vertices().front().p - here).norm();
                if (d < bestDistance) {
                    bestDistance = d;
                    best = k;
                    bestPath = std::move(path);
                }
            }
            if (best == loops.size()) {
                break;
            }
            done[best] = true;
            const Vec2 s = bestPath.vertices().front().p;
            if (atDepth && bestDistance <= linkMax) {
                // Down to the next pass over the surface: never below the tool's contact height.
                const Vec3 from = b.position();
                const int n = std::max(1, static_cast<int>(std::ceil(bestDistance / res)));
                for (int t = 1; t <= n; ++t) {
                    const double u = static_cast<double>(t) / n;
                    const Vec2 xy = from.head<2>() + (s - from.head<2>()) * u;
                    const double zl = from.z() + (z - from.z()) * u;
                    const double surface = t == n ? z : dc.drop(xy, floor) + allowance;
                    b.feed(Vec3(xy.x(), xy.y(), std::max(zl, surface)), FeedClass::Link,
                           ctx.feeds.cut);
                }
            } else {
                b.retractTo(retract);
                b.rapidAbove(s, retract, std::min(retract, z + ctx.heights.feedClearance));
                b.feed(Vec3(s.x(), s.y(), z), FeedClass::Plunge, ctx.feeds.plunge);
            }
            b.follow(bestPath, z, FeedClass::Cut, ctx.feeds.cut);
            atDepth = true;
        }
        ctx.progress.report(static_cast<double>(li + 1) / levels.size(), "Contour finishing");
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

// -- Parameter objects ------------------------------------------------------------------------

ParallelParams parallelParams(const Json& values, const tooling::Tool& /*tool*/,
                              const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("parallel3d"), values);
    ParallelParams p;
    // The stepover is the preset's radial depth unless a scallop height sets it.
    p.scallopHeight = r.number("scallop_height");
    p.stepover = p.scallopHeight > 0.0 ? 0.0 : presetStepover(preset);
    p.angle = r.number("angle");
    p.sampleStep = r.number("sample_step");
    p.stockToLeave = r.number("stock_to_leave");
    p.zigzag = r.choice("pattern") != "one_way";
    return p;
}

WaterlineParams waterlineParams(const Json& values, const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("waterline3d"), values);
    WaterlineParams p;
    p.stepDown = presetStepDown(preset);
    p.stockToLeave = r.number("stock_to_leave");
    p.resolution = r.number("resolution");
    p.direction =
        r.choice("direction") == "conventional" ? CutDirection::Conventional : CutDirection::Climb;
    return p;
}

ContourFinishParams contourFinishParams(const Json& values) {
    const ParamReader r(*findStrategy("contour3d"), values);
    ContourFinishParams p;
    p.scallopHeight = r.number("scallop_height");
    p.maxStepDown = r.number("max_step_down");
    p.minSlope = r.number("min_slope");
    p.stockToLeave = r.number("stock_to_leave");
    p.resolution = r.number("resolution");
    p.direction =
        r.choice("direction") == "conventional" ? CutDirection::Conventional : CutDirection::Climb;
    return p;
}

namespace {

ZLevelRoughParams zLevelParams(const char* strategy, const Json& values, const tooling::Tool& tool,
                               const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy(strategy), values);
    ZLevelRoughParams p;
    p.medialOpenAreas = r.choice("open_areas") == "medial";
    p.fineStepDown = r.number("fine_step_down");
    p.stepDown = presetStepDown(preset);
    p.stepoverRatio = presetStepoverRatio(preset, tool);
    p.stockToLeaveXY = r.number("stock_to_leave_xy");
    p.stockToLeaveZ = r.number("stock_to_leave_z");
    p.resolution = r.number("resolution");
    const std::string entry = r.choice("entry");
    p.entry = entry == "ramp"     ? EntryKind::Ramp
              : entry == "plunge" ? EntryKind::Plunge
                                  : EntryKind::Helix;
    p.rampAngle = r.number("ramp_angle");
    if (!r.isSet("ramp_angle") && preset && preset->rampAngle > 0.0) {
        p.rampAngle = preset->rampAngle;
    }
    if (r.isSet("helix_radius")) {
        p.helixRadius = r.number("helix_radius");
    }
    p.direction =
        r.choice("direction") == "conventional" ? CutDirection::Conventional : CutDirection::Climb;
    p.flatAreaLevels = r.flag("flat_levels");
    p.byArea = r.choice("order") == "areas";
    return p;
}

} // namespace

ZLevelRoughParams zLevelRoughParams(const Json& values, const tooling::Tool& tool,
                                    const tooling::CuttingPreset* preset) {
    return zLevelParams("zlevel_rough", values, tool, preset);
}

FlatFinishParams flatFinishParams(const Json& values, const tooling::Tool& /*tool*/,
                                  const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("flat3d"), values);
    FlatFinishParams p;
    p.stepover = presetStepover(preset);
    p.stockToLeaveXY = r.number("stock_to_leave_xy");
    p.stockToLeaveZ = r.number("stock_to_leave_z");
    p.maxSlope = r.number("max_slope");
    p.resolution = r.number("resolution");
    const std::string entry = r.choice("entry");
    p.entry = entry == "helix"    ? EntryKind::Helix
              : entry == "plunge" ? EntryKind::Plunge
                                  : EntryKind::Ramp;
    p.rampAngle = r.number("ramp_angle");
    if (!r.isSet("ramp_angle") && preset && preset->rampAngle > 0.0) {
        p.rampAngle = preset->rampAngle;
    }
    p.direction =
        r.choice("direction") == "conventional" ? CutDirection::Conventional : CutDirection::Climb;
    p.bridgeOpenings = r.flag("bridge_openings") ? r.number("bridge_max") : 0.0;
    return p;
}

} // namespace ocam::strategies
