// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Plunge roughing (DL-0027): material is removed by straight plunges of the tool along its
// axis at overlapping positions, rows staggered by half a pitch, with plunges along the wall
// last. Between plunges the tool hops just above the stock top.

#include "Internal.hpp"
#include "ocam/strategies/Strategies.hpp"

#include <ocam/geom2d/Clip.hpp>
#include <ocam/geom2d/PathOps.hpp>
#include <ocam/toolpath/PathBuilder.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ocam::strategies {

using geom2d::Region;
using geom2d::Regions;
using toolpath::FeedClass;

toolpath::Toolpath plungeRough(const Regions& area, double topZ, double bottomZ,
                               const PlungeParams& p, const Context& ctx) {
    using namespace detail;
    const double r = requireMillingTool(ctx.tool);
    const double d = 2.0 * r;
    const double pitch = p.pitch.value_or(kPlungePitchShare * d);
    if (p.stepoverRatio <= 0.0 || pitch <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "invalid stepover or pitch");
    }
    const double tol = ctx.chordTolerance;
    const double rowSpacing = p.stepoverRatio * d;
    const double floorZ = bottomZ + p.stockToLeaveZ;

    // Tool centres keep the tool (plus the allowance) inside the area.
    const Regions centres = geom2d::offset(area, -(r + p.stockToLeaveXY), {tol});
    geom2d::Aabb2 bounds;
    for (const Region& region : centres) {
        bounds.extend(region.bounds());
    }
    toolpath::Toolpath tp =
        beginToolpath(ctx, "plunge_rough", centres.empty() ? Vec2::Zero() : bounds.center());
    if (centres.empty()) {
        tp.warnings.push_back("the tool does not fit in the area");
        return tp;
    }
    if (floorZ >= topZ - 1e-9) {
        tp.warnings.push_back("nothing to cut: the bottom is at or above the top");
        return tp;
    }
    // Neighbouring plunges must overlap enough to leave no islands between them: the cell
    // diagonal (half a pitch along the row with the stagger) must stay within the diameter.
    const double along = p.stagger ? 0.5 * pitch : pitch;
    if (along * along + rowSpacing * rowSpacing > d * d * (1.0 - 1e-9)) {
        tp.warnings.push_back("stepover and pitch leave material between the plunges");
    }

    // Plunges along the wall: every pitch along each centre loop.
    std::vector<std::vector<Vec2>> wallRuns;
    if (p.wall) {
        for (const Region& region : centres) {
            std::vector<const geom2d::Polyline*> loops{&region.outer};
            for (const auto& hole : region.holes) {
                loops.push_back(&hole);
            }
            for (const geom2d::Polyline* loop : loops) {
                const std::vector<Vec2> pts = loop->discretize(tol);
                if (pts.empty()) {
                    continue;
                }
                std::vector<Vec2> run{pts.front()};
                double carried = 0.0;
                for (std::size_t k = 0; k < pts.size(); ++k) {
                    const Vec2 a = pts[k];
                    const Vec2 b = pts[(k + 1) % pts.size()];
                    const double len = (b - a).norm();
                    double s = pitch - carried;
                    while (s <= len - 1e-9) {
                        run.push_back(a + (b - a) * (s / len));
                        s += pitch;
                    }
                    carried = len - (s - pitch);
                }
                // The last plunge must not sit on top of the first.
                if (run.size() > 1 && (run.back() - run.front()).norm() < 0.5 * pitch) {
                    run.pop_back();
                }
                wallRuns.push_back(std::move(run));
            }
        }
    }
    auto distanceToWall = [&](const Vec2& q) {
        double best = std::numeric_limits<double>::infinity();
        for (const Region& region : centres) {
            best = std::min(best, geom2d::closestPoint(region.outer, q).distance);
            for (const auto& hole : region.holes) {
                best = std::min(best, geom2d::closestPoint(hole, q).distance);
            }
        }
        return best;
    };

    // Interior rows, zigzag, staggered by half a pitch; plunges the wall plunges already cover
    // are left out.
    std::vector<std::vector<Vec2>> rows;
    const int rowCount = static_cast<int>(std::floor(bounds.size().y() / rowSpacing)) + 1;
    const double y0 = bounds.min.y() + 0.5 * (bounds.size().y() - (rowCount - 1) * rowSpacing);
    for (int j = 0; j < rowCount; ++j) {
        ctx.cancel.throwIfCancelled();
        const double y = y0 + j * rowSpacing;
        const double shift = p.stagger && (j % 2 == 1) ? 0.5 * pitch : 0.0;
        std::vector<Vec2> row;
        for (double x = bounds.min.x() + shift; x <= bounds.max.x() + 1e-9; x += pitch) {
            const Vec2 q(x, y);
            const bool inside = std::any_of(centres.begin(), centres.end(),
                                            [&](const Region& c) { return c.contains(q, tol); });
            if (inside && (!p.wall || distanceToWall(q) > 0.5 * std::min(pitch, rowSpacing))) {
                row.push_back(q);
            }
        }
        if (j % 2 == 1) {
            std::reverse(row.begin(), row.end());
        }
        if (!row.empty()) {
            rows.push_back(std::move(row));
        }
    }

    toolpath::PathBuilder b(tp);
    const double hop = topZ + ctx.heights.feedClearance;
    const double retract = std::max(ctx.heights.retract, hop);
    bool first = true;
    auto plunge = [&](const Vec2& q) {
        if (first) {
            b.rapidAbove(q, retract, hop);
            first = false;
        } else {
            b.rapid(Vec3(q.x(), q.y(), hop)); // just above the stock
        }
        b.feed(Vec3(q.x(), q.y(), floorZ), FeedClass::Plunge, ctx.feeds.plunge);
        b.rapid(Vec3(q.x(), q.y(), hop)); // straight back out of the hole
    };
    std::size_t count = 0;
    for (const auto& row : rows) {
        for (const Vec2& q : row) {
            plunge(q);
            ++count;
        }
        ctx.progress.report(0.5 * static_cast<double>(&row - rows.data() + 1) / rows.size(),
                            "Plunge roughing");
    }
    for (const auto& run : wallRuns) {
        for (const Vec2& q : run) {
            plunge(q);
            ++count;
        }
    }
    if (count == 0) {
        tp.warnings.push_back("no plunge positions in the area");
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

} // namespace ocam::strategies
