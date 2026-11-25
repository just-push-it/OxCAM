// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Facing: parallel raster passes over the stock outline. See docs/design-log/0006.

#include "Internal.hpp"
#include "ocam/strategies/Strategies.hpp"

#include <ocam/geom2d/Clip.hpp>

#include <algorithm>
#include <cmath>

namespace ocam::strategies {

toolpath::Toolpath facing(const geom2d::Regions& area, double topZ, double finalZ,
                          const FacingParams& p, const Context& ctx) {
    using namespace detail;
    const double r = requireMillingTool(ctx.tool);
    const double d = 2.0 * r;
    if (area.empty()) {
        throw Error(ErrorCode::InvalidArgument, "facing needs a stock outline");
    }
    if (p.stepoverRatio <= 0.0 || p.stepoverRatio > 1.0) {
        throw Error(ErrorCode::InvalidArgument, "facing stepover must be between 0 and 1 × D");
    }
    const double tol = ctx.chordTolerance;
    const geom2d::ClipOptions clip{tol};

    // Tool centre travels over the outline grown by the radius plus overrun; the tool touches
    // material only when its centre is inside the outline grown by the radius.
    const geom2d::Regions centreArea = geom2d::offset(area, r + p.overrun, clip);
    const geom2d::Regions touchArea = geom2d::offset(area, r - 1e-6, clip);

    // Work in a frame where passes run along +X'.
    const auto toLocal = [&](const Vec2& q) {
        return rotate(q, -p.angle);
    };
    const auto toWorld = [&](const Vec2& q) {
        return rotate(q, p.angle);
    };
    const auto areaLoops = discretizedLoops(area, tol, toLocal);
    const auto centreLoops = discretizedLoops(centreArea, tol, toLocal);
    double yMin = std::numeric_limits<double>::infinity();
    double yMax = -std::numeric_limits<double>::infinity();
    for (const auto& loop : areaLoops) {
        for (const Vec2& q : loop) {
            yMin = std::min(yMin, q.y());
            yMax = std::max(yMax, q.y());
        }
    }

    // Pass positions: tool edges overlap the outline edges by a small margin.
    const double margin = std::min(0.1 * d, 1.0);
    const double first = yMin + r - margin;
    const double last = yMax - r + margin;
    std::vector<double> lines;
    if (last <= first) {
        lines.push_back(0.5 * (yMin + yMax));
    } else {
        const int n =
            std::max(1, static_cast<int>(std::ceil((last - first) / (p.stepoverRatio * d) - 1e-9)));
        for (int i = 0; i <= n; ++i) {
            lines.push_back(first + (last - first) * i / n);
        }
    }

    // Passes per line (a line may cross a non-convex outline more than once).
    struct Pass {
        Vec2 a;
        Vec2 b;
    };
    std::vector<std::vector<Pass>> passes;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const bool forward = p.pattern == FacingPattern::OneWay || i % 2 == 0;
        auto intervals = scanIntervals(centreLoops, lines[i]);
        if (!forward) {
            std::reverse(intervals.begin(), intervals.end());
        }
        std::vector<Pass> row;
        for (const auto& [x0, x1] : intervals) {
            const Vec2 a(forward ? x0 : x1, lines[i]);
            const Vec2 b(forward ? x1 : x0, lines[i]);
            row.push_back({toWorld(a), toWorld(b)});
        }
        passes.push_back(std::move(row));
    }

    const double zFinal = finalZ + p.stockToLeave;
    const auto levels = depthLevels(topZ, zFinal, p.stepDown);
    const Vec2 startXY = (!passes.empty() && !passes.front().empty())
                             ? passes.front().front().a
                             : area.front().bounds().center();
    toolpath::Toolpath tp = beginToolpath(ctx, "facing", startXY);
    if (levels.empty()) {
        tp.warnings.push_back("nothing to face: the stock top is at or below the facing height");
        return tp;
    }

    toolpath::PathBuilder b(tp);
    const double hop = std::max(ctx.heights.retract, topZ + ctx.heights.feedClearance);
    // A link at depth is allowed only where the tool cannot touch material.
    auto linkIsClear = [&](const Vec2& from, const Vec2& to) {
        for (int k = 0; k <= 16; ++k) {
            if (anyContains(touchArea, from + (to - from) * (k / 16.0), tol)) {
                return false;
            }
        }
        return true;
    };

    for (std::size_t li = 0; li < levels.size(); ++li) {
        ctx.cancel.throwIfCancelled();
        const double z = levels[li];
        const double approachZ = (li == 0 ? topZ : levels[li - 1]) + ctx.heights.feedClearance;
        bool down = false;
        for (const auto& row : passes) {
            for (const Pass& pass : row) {
                if (down && linkIsClear(b.position().head<2>(), pass.a)) {
                    b.feed(Vec3(pass.a.x(), pass.a.y(), z), toolpath::FeedClass::Link,
                           ctx.feeds.cut);
                } else {
                    b.rapidAbove(pass.a, hop, approachZ);
                    b.feed(Vec3(pass.a.x(), pass.a.y(), z), toolpath::FeedClass::Plunge,
                           ctx.feeds.plunge);
                }
                b.feed(Vec3(pass.b.x(), pass.b.y(), z), toolpath::FeedClass::Cut, ctx.feeds.cut);
                down = true;
            }
        }
        b.retractTo(hop);
        ctx.progress.report(static_cast<double>(li + 1) / levels.size(), "Facing");
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

} // namespace ocam::strategies
