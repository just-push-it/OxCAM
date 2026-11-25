// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Wrapping maps the unrolled plane onto the cylinder by arc length (the classic rotary-axis
// wrap). Five-axis ball finishing keeps the ball centres of the 3-axis drop-cutter path, which
// fixes the contact whatever the axis, and turns the axis by a fixed rule (lead and tilt to the
// surface normal as in Vickers and Quan 1989 and Jensen and Anderson 1993, or a fixed
// direction / point). See docs/design-log/0024.

#include "ocam/strategies/MultiAxis.hpp"

#include "Internal.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/geom2d/Contour.hpp>
#include <ocam/geom3d/Bvh.hpp>
#include <ocam/geom3d/Convex.hpp>

#include <fmt/format.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <map>
#include <mutex>

namespace ocam::strategies {

using toolpath::FeedClass;
using toolpath::Move;
using toolpath::MoveKind;

// -- Wrapping -----------------------------------------------------------------------------------

std::pair<Vec3, Vec3> wrapPoint(const Vec3& flat, double radius) {
    const double t = flat.y() / radius;
    const double rho = radius + flat.z();
    const Vec3 dir(0.0, std::sin(t), std::cos(t));
    return {Vec3(flat.x(), rho * dir.y(), rho * dir.z()), dir};
}

toolpath::Toolpath wrapToolpath(const toolpath::Toolpath& flat, double radius,
                                double chordTolerance) {
    const double tol = std::max(chordTolerance, 1e-5);
    // Smaller, a turn is within the tolerance, and the moves' splits are without limit.
    if (radius <= tol) {
        throw Error(ErrorCode::InvalidArgument, "the wrap radius must be above the tolerance");
    }
    toolpath::Toolpath out = flat;
    out.moves.clear();
    out.cycles.clear();
    out.compensation = toolpath::Compensation::None;
    out.start = wrapPoint(flat.start, radius).first;

    Vec3 pos = flat.start;
    auto piece = [&](const Move& tmpl, const Vec3& a, const Vec3& b) {
        // Angular step whose chord stays within the tolerance at the outer radius.
        const double rho = std::max(1e-6, radius + std::max(a.z(), b.z()));
        const double dt = tol < rho ? 2.0 * std::acos(1.0 - tol / rho) : kPi / 4.0;
        const double sweep = std::abs(b.y() - a.y()) / radius;
        const int n = std::max(1, static_cast<int>(std::ceil(sweep / std::max(dt, 1e-6))));
        for (int k = 1; k <= n; ++k) {
            const Vec3 q = a + (b - a) * (static_cast<double>(k) / n);
            Move m = tmpl;
            m.kind = tmpl.kind == MoveKind::Rapid ? MoveKind::Rapid : MoveKind::Linear;
            std::tie(m.end, m.axis) = wrapPoint(q, radius);
            m.center = Vec3::Zero();
            out.moves.push_back(m);
        }
    };
    for (const Move& m : flat.moves) {
        switch (m.kind) {
        case MoveKind::Cycle:
            throw Error(ErrorCode::Unsupported, "canned cycles cannot be wrapped");
        case MoveKind::Probe:
            throw Error(ErrorCode::Unsupported, "probing moves cannot be wrapped");
        case MoveKind::Dwell: {
            Move d = m;
            std::tie(d.end, d.axis) = wrapPoint(pos, radius);
            out.moves.push_back(d);
            continue;
        }
        case MoveKind::ArcCW:
        case MoveKind::ArcCCW: {
            const std::vector<Vec3> pts = toolpath::sampleArc(pos, m, tol / 2.0);
            for (std::size_t k = 1; k < pts.size(); ++k) {
                piece(m, pts[k - 1], pts[k]);
            }
            break;
        }
        case MoveKind::Rapid:
        case MoveKind::Linear:
            piece(m, pos, m.end);
            break;
        }
        pos = m.end;
    }
    return out;
}

// -- Radial height map ----------------------------------------------------------------------------

RadialMap radialMap(const geom3d::Mesh& part, double xMin, double xMax, double spacing,
                    double outerRadius, CancelToken& cancel) {
    RadialMap map;
    map.dx = spacing;
    map.nx = std::max(2, static_cast<int>(std::ceil((xMax - xMin) / spacing)) + 1);
    map.x0 = 0.5 * (xMin + xMax) - 0.5 * (map.nx - 1) * spacing;
    map.nt = std::max(16, static_cast<int>(std::ceil(2.0 * kPi * outerRadius / spacing)));
    map.radius.assign(static_cast<std::size_t>(map.nx) * static_cast<std::size_t>(map.nt), 0.0);
    const geom3d::TriangleBvh bvh(part);
    tbb::parallel_for(tbb::blocked_range<int>(0, map.nx), [&](const tbb::blocked_range<int>& r) {
        for (int i = r.begin(); i != r.end(); ++i) {
            if (cancel.isCancelled()) {
                return;
            }
            const double x = map.x0 + i * spacing;
            for (int j = 0; j < map.nt; ++j) {
                const double t = map.angle(j);
                const Vec3 dir(0.0, std::sin(t), std::cos(t));
                const Vec3 origin = Vec3(x, 0.0, 0.0) + outerRadius * dir;
                if (const auto hit = bvh.raycast(origin, -dir, outerRadius)) {
                    map.radius[static_cast<std::size_t>(i) * static_cast<std::size_t>(map.nt) +
                               static_cast<std::size_t>(j)] = outerRadius - hit->t;
                }
            }
        }
    });
    cancel.throwIfCancelled();
    return map;
}

// -- Wrapped pocketing ---------------------------------------------------------------------------

toolpath::Toolpath rotaryPocket(const geom3d::Mesh& part, double stockRadius, double xMin,
                                double xMax, const RotaryPocketParams& p, const Context& ctx) {
    const double r = detail::requireMillingTool(ctx.tool);
    if (part.isEmpty()) {
        throw Error(ErrorCode::InvalidArgument, "wrapped pocketing needs the part's surface");
    }
    if (stockRadius <= 0.0 || xMax <= xMin) {
        throw Error(ErrorCode::InvalidArgument, "wrapped pocketing needs a stock about WCS X");
    }
    const double h = p.resolution > 0.0 ? p.resolution : std::clamp(r / 4.0, 0.05, 0.5);
    const double tol = std::max(ctx.chordTolerance, 1e-3);
    const geom3d::Aabb3 pb = part.bounds();
    double partRadius = 0.0;
    for (const Vec3& v : part.vertices) {
        partRadius = std::max(partRadius, std::hypot(v.y(), v.z()));
    }
    // Extend past the machined length so features open at the ends are cut through.
    const double margin = r + p.stockToLeaveXY + 2.0 * h;
    const double outer = std::max(stockRadius, partRadius) + 1.0;
    const RadialMap map = radialMap(part, xMin - margin, xMax + margin, h, outer, ctx.cancel);
    const double partX0 = std::max(xMin, pb.min.x()), partX1 = std::min(xMax, pb.max.x());
    const geom3d::TriangleBvh bvh(part);
    // The part's radius at (x, angle) by ray cast; nullopt where the ray misses inside the part's
    // length (holes), 0 beyond its ends.
    auto exactPart = [&](double x, double t) -> std::optional<double> {
        if (x < partX0 - 1e-9 || x > partX1 + 1e-9) {
            return 0.0;
        }
        const Vec3 dir(0.0, std::sin(t), std::cos(t));
        if (const auto hit = bvh.raycast(Vec3(x, 0.0, 0.0) + outer * dir, -dir, outer)) {
            return outer - hit->t;
        }
        return std::nullopt;
    };

    toolpath::Toolpath tp = beginToolpath(ctx, "rotary_pocket", Vec2::Zero());
    const double clearR = std::max(ctx.heights.clearance, stockRadius + 2.0);
    const double retractR = std::max(ctx.heights.retract, stockRadius + 1.0);

    // Free space: where the part lies below a level (beyond the part's ends everything is free;
    // within them, rays that miss are through holes and are left alone).
    auto partAt = [&](int i, int j) -> std::optional<double> {
        const double x = map.x0 + i * map.dx;
        if (x < partX0 - 1e-9 || x > partX1 + 1e-9) {
            return 0.0;
        }
        const double v = map.at(i, j);
        if (v <= 0.0) {
            return std::nullopt;
        }
        return v;
    };

    // Levels: evenly spaced down to the deepest point, plus every flat floor. Recesses shallower
    // than the surface tolerance are the facets of the tessellated cylinder, not features.
    const double surfaceTol = 0.02;
    double deepest = std::numeric_limits<double>::infinity();
    std::map<long long, int> floors;
    int cells = 0;
    int cellsInLength = 0, cellsOnPart = 0;
    // The stock's radius along the map.
    std::vector<double> stockAt(static_cast<std::size_t>(map.nx), stockRadius);
    if (p.stockRadiusAt) {
        for (int i = 0; i < map.nx; ++i) {
            stockAt[static_cast<std::size_t>(i)] = p.stockRadiusAt(map.x0 + i * map.dx);
        }
    }
    for (int i = 0; i < map.nx; ++i) {
        const double x = map.x0 + i * map.dx;
        if (x < partX0 || x > partX1) {
            continue;
        }
        const double top = stockAt[static_cast<std::size_t>(i)];
        for (int j = 0; j < map.nt; ++j) {
            const double v = map.at(i, j);
            ++cellsInLength;
            cellsOnPart += v > 0.0 ? 1 : 0;
            if (v > 0.0 && v < top - surfaceTol) {
                deepest = std::min(deepest, v);
                ++floors[std::llround(v * 1000.0)];
                ++cells;
            }
        }
    }
    if (!std::isfinite(deepest)) {
        tp.warnings.push_back("the part has no recess below the stock radius");
        tp.start =
            wrapPoint(Vec3(0.5 * (xMin + xMax), 0.0, clearR - stockRadius), stockRadius).first;
        return tp;
    }
    // A part lying across the axis: its surface comes to the axis without going around it, as a
    // round end or a point does and a part held off the axis does not come to it. Everything out
    // to its far corners would be a recess, in slivers at every level, for hours (DL-0076).
    if (!p.stockRadiusAt && deepest < r && cellsOnPart < 0.9 * cellsInLength) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("the part does not go around the WCS X axis (the rotary axis), and "
                                "its surface comes {:.3f} mm from it: set the setup's WCS with X "
                                "along the part's axis",
                                deepest));
    }
    std::vector<double> levels =
        depthLevels(stockRadius, deepest + p.stockToLeaveZ, std::max(p.stepDown, 0.01));
    const int floorCells = std::max(20, cells / 500);
    for (const auto& [key, count] : floors) {
        const double f = key / 1000.0 + p.stockToLeaveZ;
        if (count >= floorCells && f < stockRadius - surfaceTol) {
            levels.push_back(f);
        }
    }
    std::sort(levels.begin(), levels.end(), std::greater<>());
    levels.erase(std::unique(levels.begin(), levels.end(),
                             [](double a, double b) { return std::abs(a - b) < 0.01; }),
                 levels.end());

    const double dTheta = 2.0 * kPi / map.nt;
    const double grow = r + p.stockToLeaveXY + 2.0 * h; // the tool's centre and its way out
    bool started = false;
    double previous = stockRadius;
    for (std::size_t li = 0; li < levels.size(); ++li) {
        ctx.cancel.throwIfCancelled();
        const double rho = levels[li];
        // With the tool and its way out wider than a turn at this radius there is no plane to
        // unroll, and what wraps from it splits without limit as the radius falls (DL-0076).
        if (2.0 * kPi * rho < grow) {
            tp.warnings.push_back(fmt::format(
                "the levels from radius {:.3f} mm in are not cut: the tool and its way out, "
                "{:.3f} mm, are wider than the way around the part there",
                rho, grow));
            break;
        }
        // The part may rise to here. A floor at the level reads exactly that at its mesh's edges,
        // which is no wall.
        const double cut = rho - p.stockToLeaveZ + 1e-6;
        // With a stock of many radii: where there is something to take at this level, and as far
        // around it as the tool's centre and its way out need. The rest of the free space is air.
        std::vector<char> reach;
        const auto cell = [&](int i, int j) {
            return static_cast<std::size_t>(i) * static_cast<std::size_t>(map.nt) +
                   static_cast<std::size_t>(j);
        };
        if (p.stockRadiusAt) {
            reach.assign(static_cast<std::size_t>(map.nx) * static_cast<std::size_t>(map.nt), 0);
            for (int i = 0; i < map.nx; ++i) {
                const double x = map.x0 + i * map.dx;
                if (x < partX0 || x > partX1 || stockAt[static_cast<std::size_t>(i)] <= rho + 1e-3) {
                    continue;
                }
                for (int j = 0; j < map.nt; ++j) {
                    const double v = map.at(i, j);
                    reach[cell(i, j)] = v > 0.0 && cut - v > 0.0;
                }
            }
            const int kx = static_cast<int>(std::ceil(grow / map.dx));
            const int kt = static_cast<int>(std::ceil(grow / (rho * (2.0 * kPi / map.nt))));
            // Along the angle (around, twice over for the wrap), then along x.
            for (int i = 0; i < map.nx; ++i) {
                int since = kt + 1;
                for (int k = 0; k < 2 * map.nt; ++k) {
                    const int j = k % map.nt;
                    since = reach[cell(i, j)] == 1 ? 0 : since + 1;
                    if (since <= kt && reach[cell(i, j)] == 0) {
                        reach[cell(i, j)] = 2;
                    }
                }
                since = kt + 1;
                for (int k = 2 * map.nt - 1; k >= 0; --k) {
                    const int j = k % map.nt;
                    since = reach[cell(i, j)] == 1 ? 0 : since + 1;
                    if (since <= kt && reach[cell(i, j)] == 0) {
                        reach[cell(i, j)] = 2;
                    }
                }
            }
            for (char& c : reach) {
                c = c != 0 ? 1 : 0;
            }
            for (int j = 0; j < map.nt; ++j) {
                int since = kx + 1;
                for (int i = 0; i < map.nx; ++i) {
                    since = reach[cell(i, j)] == 1 ? 0 : since + 1;
                    if (since <= kx && reach[cell(i, j)] == 0) {
                        reach[cell(i, j)] = 2;
                    }
                }
                since = kx + 1;
                for (int i = map.nx - 1; i >= 0; --i) {
                    since = reach[cell(i, j)] == 1 ? 0 : since + 1;
                    if (since <= kx && reach[cell(i, j)] == 0) {
                        reach[cell(i, j)] = 2;
                    }
                }
            }
        }
        auto freeValue = [&](int i, int j) {
            if (!reach.empty() && reach[cell(i, j)] == 0) {
                return -1.0;
            }
            const auto v = partAt(i, j);
            return v ? cut - *v : -1.0;
        };
        // Seam: the angle with the least free space; extend across it when it is not empty.
        int seam = 0, seamFree = std::numeric_limits<int>::max();
        for (int j = 0; j < map.nt; ++j) {
            int count = 0;
            for (int i = 0; i < map.nx; ++i) {
                count += freeValue(i, j) > 0.0 ? 1 : 0;
            }
            if (count < seamFree) {
                seamFree = count;
                seam = j;
            }
        }
        const double du = rho * dTheta;
        const int extra = seamFree > 0 ? static_cast<int>(std::ceil(grow / du)) : 0;
        geom2d::ScalarGrid grid;
        grid.nx = map.nx;
        grid.ny = map.nt + 2 * extra + 1; // back to the seam: a wall may lie just before it
        grid.spacing = Vec2(map.dx, du);
        grid.origin = Vec2(map.x0, rho * (map.angle(seam) - extra * dTheta));
        grid.values.resize(static_cast<std::size_t>(grid.nx) * static_cast<std::size_t>(grid.ny));
        bool any = false;
        for (int k = 0; k < grid.ny; ++k) {
            const int j = ((seam - extra + k) % map.nt + map.nt) % map.nt;
            for (int i = 0; i < grid.nx; ++i) {
                const double v = freeValue(i, j);
                grid.values[static_cast<std::size_t>(k) * static_cast<std::size_t>(grid.nx) +
                            static_cast<std::size_t>(i)] = v;
                any = any || v > 0.0;
            }
        }
        if (!any) {
            previous = rho;
            continue;
        }
        auto exactFree = [&](const Vec2& q) {
            if (!reach.empty()) {
                // Out of reach is a boundary in the air: where the grid put it does for that.
                const int i =
                    std::clamp(static_cast<int>(std::lround((q.x() - map.x0) / map.dx)), 0, map.nx - 1);
                const int j =
                    ((static_cast<int>(std::lround(q.y() / rho / dTheta - map.angle(0) / dTheta)) %
                      map.nt) +
                     map.nt) %
                    map.nt;
                if (reach[cell(i, j)] == 0) {
                    return false;
                }
            }
            const auto v = exactPart(q.x(), q.y() / rho);
            return v && cut - *v > 0.0;
        };
        const geom2d::Regions regions =
            geom2d::isoRegions(grid, 0.0, exactFree, std::max(1e-4, 0.25 * tol));
        if (regions.empty()) {
            previous = rho;
            continue;
        }
        Heights flatHeights;
        flatHeights.clearance = clearR - rho;
        flatHeights.retract = retractR - rho;
        flatHeights.feedClearance = ctx.heights.feedClearance;
        const Context flatCtx{ctx.tool, ctx.feeds, flatHeights, tol, ctx.cancel, ctx.progress};
        PocketParams pp;
        pp.stepoverRatio = p.stepoverRatio;
        pp.stepDown = 0.0;
        pp.stockToLeaveXY = p.stockToLeaveXY;
        pp.stockToLeaveZ = 0.0;
        pp.entry = p.entry;
        pp.rampAngle = p.rampAngle;
        pp.direction = p.direction;
        toolpath::Toolpath flat;
        try {
            flat = pocket(regions, previous - rho, 0.0, pp, flatCtx);
        } catch (const Error& e) {
            if (e.code() == ErrorCode::Cancelled) {
                throw;
            }
            tp.warnings.push_back(fmt::format("level at radius {:.3f}: {}", rho, e.what()));
            previous = rho;
            continue;
        }
        for (const auto& w : flat.warnings) {
            tp.warnings.push_back(fmt::format("radius {:.3f}: {}", rho, w));
        }
        previous = rho;
        if (flat.moves.empty()) {
            continue;
        }
        appendWrapped(tp, wrapToolpath(flat, rho, tol), clearR, tol, started);
        ctx.progress.report(static_cast<double>(li + 1) / static_cast<double>(levels.size()),
                            "Wrapped pocketing");
    }
    if (!started) {
        tp.warnings.push_back("no level of the recesses is wide enough for the tool");
        tp.start =
            wrapPoint(Vec3(0.5 * (xMin + xMax), 0.0, clearR - stockRadius), stockRadius).first;
    }
    tp.strategy = "rotary_pocket";
    return tp;
}

void appendWrapped(toolpath::Toolpath& tp, const toolpath::Toolpath& wrapped, double clearR,
                   double tol, bool& started) {
    if (!started) {
        tp.start = wrapped.start;
        started = true;
    } else {
        // Around the clearance cylinder to the next start.
        const Vec3 a = tp.moves.empty() ? tp.start : tp.moves.back().end;
        const Vec3 b = wrapped.start;
        const double ta = std::atan2(a.y(), a.z());
        double tb = std::atan2(b.y(), b.z());
        while (tb - ta > kPi) {
            tb -= 2.0 * kPi;
        }
        while (tb - ta < -kPi) {
            tb += 2.0 * kPi;
        }
        toolpath::Toolpath link;
        link.start = Vec3(a.x(), clearR * ta, 0.0);
        Move m;
        m.kind = MoveKind::Rapid;
        m.feedClass = FeedClass::Rapid;
        m.end = Vec3(b.x(), clearR * tb, 0.0);
        link.moves.push_back(m);
        const toolpath::Toolpath around = wrapToolpath(link, clearR, tol * 10.0);
        tp.moves.insert(tp.moves.end(), around.moves.begin(), around.moves.end());
    }
    tp.moves.insert(tp.moves.end(), wrapped.moves.begin(), wrapped.moves.end());
}

// -- Five-axis ball finishing ------------------------------------------------------------------

Vec3 toolAxisFor(const ToolAxisParams& p, const Vec3& centre, const Vec3& normal,
                 const Vec3& feed) {
    Vec3 a = Vec3::UnitZ();
    switch (p.mode) {
    case ToolAxisMode::LeadTilt: {
        Vec3 left = normal.cross(feed);
        if (left.norm() < 1e-9) {
            left = normal.unitOrthogonal();
        }
        left.normalize();
        const Vec3 forward = left.cross(normal).normalized();
        a = std::cos(p.tilt) * (std::cos(p.lead) * normal + std::sin(p.lead) * forward) +
            std::sin(p.tilt) * left;
        break;
    }
    case ToolAxisMode::Fixed:
        a = Vec3(std::sin(p.fixedTilt) * std::cos(p.fixedAzimuth),
                 std::sin(p.fixedTilt) * std::sin(p.fixedAzimuth), std::cos(p.fixedTilt));
        break;
    case ToolAxisMode::FromPoint:
        a = centre - p.point;
        break;
    case ToolAxisMode::ToPoint:
        a = p.point - centre;
        break;
    }
    if (a.norm() < 1e-12) {
        return Vec3::UnitZ();
    }
    a.normalize();
    // Limit the angle from WCS Z.
    const double angle = std::acos(std::clamp(a.z(), -1.0, 1.0));
    if (angle > p.maxTilt) {
        Vec2 hz(a.x(), a.y());
        hz = hz.norm() > 1e-12 ? Vec2(hz.normalized()) : Vec2(1.0, 0.0);
        a = Vec3(std::sin(p.maxTilt) * hz.x(), std::sin(p.maxTilt) * hz.y(), std::cos(p.maxTilt));
    }
    return a;
}

toolpath::Toolpath multiAxisFinish(const geom3d::Mesh& part, double floorZ,
                                   const MultiAxisFinishParams& p, const Context& ctx) {
    const double r = detail::requireMillingTool(ctx.tool);
    const tooling::Tool& tool = ctx.tool;
    if (tool.type != tooling::ToolType::BallEndMill &&
        std::abs(tool.mill.cornerRadius - r) > 1e-6) {
        throw Error(ErrorCode::Unsupported, "5-axis finishing needs a ball end mill");
    }
    ProgressRange patternRange(ctx.progress, 0.0, 0.6);
    const Context patternCtx{ctx.tool,           ctx.feeds,  ctx.heights,
                             ctx.chordTolerance, ctx.cancel, patternRange};
    const toolpath::Toolpath tp3 = parallelFinish(part, floorZ, p.pattern, patternCtx);
    toolpath::Toolpath tp = tp3;
    tp.strategy = "multiaxis_finish";
    if (tp3.moves.empty()) {
        return tp;
    }

    const double allowance = p.pattern.stockToLeave;
    const DropCutter dc(part, Cutter(tool).offset(allowance));
    auto dropAt = [&](const Vec2& xy) {
        return dc.drop(xy, floorZ - allowance) + allowance;
    };
    const double fd = std::clamp(0.05 * r, 0.005, 0.1);
    const std::size_t n = tp3.moves.size();
    std::vector<char> onSurface(n, 0);
    std::vector<Vec3> normals(n, Vec3::UnitZ());
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, n), [&](const tbb::blocked_range<std::size_t>& range) {
            for (std::size_t k = range.begin(); k != range.end(); ++k) {
                const Move& m = tp3.moves[k];
                if (m.kind != MoveKind::Linear) {
                    continue;
                }
                const Vec2 xy = m.end.head<2>();
                if (std::abs(dropAt(xy) - m.end.z()) > 1e-6) {
                    continue;
                }
                onSurface[k] = 1;
                const double gx =
                    (dropAt(xy + Vec2(fd, 0)) - dropAt(xy - Vec2(fd, 0))) / (2.0 * fd);
                const double gy =
                    (dropAt(xy + Vec2(0, fd)) - dropAt(xy - Vec2(0, fd))) / (2.0 * fd);
                normals[k] = Vec3(-gx, -gy, 1.0).normalized();
            }
        });
    ctx.cancel.throwIfCancelled();

    // Axes at surface points from the feed direction there (the cut arriving and leaving,
    // averaged; at the start of a cut the one leaving, at its end the one arriving); elsewhere
    // the axis of the next surface point (approach) or of the previous one (retract).
    std::vector<std::optional<Vec3>> axes(n);
    Vec3 lastFeed = Vec3::UnitX();
    auto cutting = [&](std::size_t k) {
        return k < n && tp3.moves[k].kind == MoveKind::Linear &&
               tp3.moves[k].feedClass == FeedClass::Cut;
    };
    for (std::size_t k = 0; k < n; ++k) {
        if (!onSurface[k]) {
            continue;
        }
        const Move& m = tp3.moves[k];
        const Vec3 from = k == 0 ? tp3.start : tp3.moves[k - 1].end;
        Vec3 in = cutting(k) ? Vec3(m.end - from) : Vec3::Zero();
        Vec3 out = cutting(k + 1) ? Vec3(tp3.moves[k + 1].end - m.end) : Vec3::Zero();
        in = in.norm() > 1e-9 ? Vec3(in.normalized()) : Vec3::Zero();
        out = out.norm() > 1e-9 ? Vec3(out.normalized()) : Vec3::Zero();
        Vec3 d = in + out;
        if (d.norm() < 1e-9) {
            d = in.norm() > 0.0 ? in : out;
        }
        const Vec3 f = d - d.dot(normals[k]) * normals[k];
        if (f.norm() > 1e-9) {
            lastFeed = f.normalized();
        }
        const Vec3 centre = m.end + r * Vec3::UnitZ();
        axes[k] = toolAxisFor(p.axis, centre, normals[k], lastFeed);
    }
    std::vector<Vec3> resolved(n, Vec3::UnitZ());
    {
        std::optional<Vec3> next;
        std::vector<std::optional<Vec3>> nextAxis(n);
        for (std::size_t k = n; k-- > 0;) {
            if (axes[k]) {
                next = axes[k];
            }
            nextAxis[k] = next;
        }
        std::optional<Vec3> last;
        Vec3 from = tp3.start;
        for (std::size_t k = 0; k < n; ++k) {
            const Move& m = tp3.moves[k];
            if (axes[k]) {
                resolved[k] = *axes[k];
                last = axes[k];
            } else {
                // A move straight up from the surface is a retract: keep the axis.
                const Vec3 d = m.end - from;
                const bool retract = last && d.head<2>().norm() < 1e-9 && d.z() > 0.0;
                resolved[k] = retract ? *last : nextAxis[k].value_or(last.value_or(Vec3::UnitZ()));
            }
            from = m.end;
        }
    }
    for (std::size_t k = 0; k < n; ++k) {
        tp.moves[k].axis = resolved[k];
        tp.moves[k].end = tp3.moves[k].end + r * (Vec3::UnitZ() - resolved[k]);
    }
    tp.start = tp3.start + r * (Vec3::UnitZ() - resolved.front());

    // Clearance of the tilted flutes and shank (above the ball centre) to the part.
    const geom3d::TriangleBvh bvh(part);
    const double flute = std::max(0.0, tool.mill.fluteLength - r);
    const double visible = tooling::visibleLength(tool) - r;
    const double shankR = (tool.mill.shankDiameter > 0.0 ? tool.mill.shankDiameter : 2.0 * r) / 2.0;
    std::atomic<int> fluteHits{0}, shankHits{0};
    std::mutex firstMutex;
    std::optional<std::pair<std::size_t, Vec3>> firstFlute, firstShank;
    auto touches = [&](const geom3d::Convex& body, double clearance) {
        bool hit = false;
        geom3d::Aabb3 box = body.bounds();
        box.min -= Vec3::Constant(clearance);
        box.max += Vec3::Constant(clearance);
        bvh.query(box, [&](std::size_t t) {
            const auto [a, b, c] = bvh.triangle(t);
            if (geom3d::intersects(body, geom3d::Convex::triangle(a, b, c), clearance)) {
                hit = true;
                return false;
            }
            return true;
        });
        return hit;
    };
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, n), [&](const tbb::blocked_range<std::size_t>& range) {
            for (std::size_t k = range.begin(); k != range.end(); ++k) {
                if (!onSurface[k]) {
                    continue;
                }
                const Vec3 centre = tp3.moves[k].end + r * Vec3::UnitZ();
                const Vec3 a = resolved[k];
                // Flutes: a capsule a hair thinner than the ball, so the contact
                // on the ball itself does not count.
                if (flute > 1e-6 && touches(geom3d::Convex::capsule(centre + 1e-3 * a,
                                                                    centre + flute * a, r - 2e-3),
                                            0.0)) {
                    ++fluteHits;
                    std::lock_guard lock(firstMutex);
                    if (!firstFlute || k < firstFlute->first) {
                        firstFlute = {k, centre};
                    }
                }
                if (visible > flute + 1e-6 &&
                    touches(
                        geom3d::Convex::cylinder(centre + flute * a, a, shankR, visible - flute),
                        p.clearance)) {
                    ++shankHits;
                    std::lock_guard lock(firstMutex);
                    if (!firstShank || k < firstShank->first) {
                        firstShank = {k, centre};
                    }
                }
            }
        });
    if (fluteHits > 0) {
        tp.warnings.push_back(fmt::format(
            "the tilted flutes cut into the part at {} points (first at {:.3f}, {:.3f}, {:.3f}); "
            "reduce the tilt or change the axis rule",
            fluteHits.load(), firstFlute->second.x(), firstFlute->second.y(),
            firstFlute->second.z()));
    }
    if (shankHits > 0) {
        tp.warnings.push_back(fmt::format(
            "the tool shank comes within {:.2f} mm of the part at {} points (first at {:.3f}, "
            "{:.3f}, {:.3f})",
            p.clearance, shankHits.load(), firstShank->second.x(), firstShank->second.y(),
            firstShank->second.z()));
    }
    ctx.progress.report(1.0, "5-axis finishing");
    return tp;
}

// -- Parameter objects ------------------------------------------------------------------------

RotaryPocketParams rotaryPocketParams(const Json& values, const tooling::Tool& tool,
                                      const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("rotary_pocket"), values);
    RotaryPocketParams p;
    p.stepDown = presetStepDown(preset);
    p.stepoverRatio = presetStepoverRatio(preset, tool);
    p.stockToLeaveXY = r.number("stock_to_leave_xy");
    p.stockToLeaveZ = r.number("stock_to_leave_z");
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
    return p;
}

MultiAxisFinishParams multiAxisFinishParams(const Json& values, const tooling::Tool& /*tool*/,
                                            const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("multiaxis_finish"), values);
    MultiAxisFinishParams p;
    // The stepover is the preset's radial depth unless a scallop height sets it.
    p.pattern.scallopHeight = r.number("scallop_height");
    p.pattern.stepover = p.pattern.scallopHeight > 0.0 ? 0.0 : presetStepover(preset);
    p.pattern.angle = r.number("angle");
    p.pattern.sampleStep = r.number("sample_step");
    p.pattern.stockToLeave = r.number("stock_to_leave");
    p.pattern.zigzag = r.choice("pattern") != "one_way";
    const std::string mode = r.choice("axis_mode");
    p.axis.mode = mode == "fixed"        ? ToolAxisMode::Fixed
                  : mode == "from_point" ? ToolAxisMode::FromPoint
                  : mode == "to_point"   ? ToolAxisMode::ToPoint
                                         : ToolAxisMode::LeadTilt;
    p.axis.lead = r.number("lead_angle");
    p.axis.tilt = r.number("tilt_angle");
    p.axis.fixedTilt = r.number("fixed_tilt");
    p.axis.fixedAzimuth = r.number("fixed_azimuth");
    p.axis.point = Vec3(r.number("point_x"), r.number("point_y"), r.number("point_z"));
    p.axis.maxTilt = r.number("max_tilt");
    p.clearance = r.number("clearance");
    return p;
}

} // namespace ocam::strategies
