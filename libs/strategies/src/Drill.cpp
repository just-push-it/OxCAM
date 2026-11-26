// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Drilling with canned cycles. See docs/design-log/0009.

#include "Internal.hpp"
#include "ocam/strategies/Strategies.hpp"

#include <ocam/geom2d/Ordering.hpp>

#include <algorithm>
#include <cmath>

namespace ocam::strategies {

toolpath::Toolpath drill(const std::vector<HoleTarget>& holes, const DrillParams& p,
                         const Context& ctx) {
    using namespace detail;
    using tooling::ToolType;
    const double r = requireMillingTool(ctx.tool);
    const double d = 2.0 * r;
    const ToolType type = ctx.tool.type;

    std::vector<Vec2> points;
    for (const auto& h : holes) {
        points.push_back(h.position);
    }
    const auto order = geom2d::orderPoints(points, Vec2::Zero());
    toolpath::Toolpath tp =
        beginToolpath(ctx, "drill", holes.empty() ? Vec2::Zero() : points[order.front()]);
    if (holes.empty()) {
        tp.warnings.push_back("no holes to drill");
        return tp;
    }

    const bool pointed =
        (type == ToolType::Drill || type == ToolType::SpotDrill || type == ToolType::CenterDrill) &&
        ctx.tool.mill.tipAngle > 0.0 && ctx.tool.mill.tipAngle < kPi;
    const double tipLength = pointed ? r / std::tan(ctx.tool.mill.tipAngle / 2.0) : 0.0;

    toolpath::CycleType defaultType = toolpath::CycleType::Drill;
    if (type == ToolType::Tap) {
        defaultType =
            ctx.tool.mill.leftHand ? toolpath::CycleType::TapLeft : toolpath::CycleType::Tap;
    } else if (type == ToolType::Reamer || type == ToolType::BoringBar) {
        defaultType = toolpath::CycleType::Bore;
    } else if (p.dwell > 0.0) {
        defaultType = toolpath::CycleType::Dwell;
    }

    double feed = ctx.feeds.cut;
    if (type == ToolType::Tap) {
        if (ctx.tool.mill.threadPitch <= 0.0) {
            throw Error(ErrorCode::InvalidArgument, "tapping needs the tap's thread pitch");
        }
        feed = ctx.tool.mill.threadPitch * ctx.feeds.spindle;
    }

    toolpath::PathBuilder b(tp);
    bool warnedFlute = false;
    double previousTop = holes[order.front()].topZ;
    for (std::size_t idx : order) {
        ctx.cancel.throwIfCancelled();
        const HoleTarget& h = holes[idx];
        if (h.depth <= 0.0) {
            tp.warnings.push_back(fmt::format("hole at ({:.3f}, {:.3f}) has no depth; skipped",
                                              h.position.x(), h.position.y()));
            continue;
        }
        const double extra = (p.depthMode == DepthMode::FullDiameter ? tipLength : 0.0) +
                             (h.through ? p.breakthrough : 0.0);
        toolpath::DrillCycle c;
        c.topZ = h.topZ;
        c.bottomZ = h.topZ - h.depth - extra;
        c.retractZ = p.rPlaneZ ? std::max(*p.rPlaneZ, h.topZ) : h.topZ + p.retractClearance;
        c.feedRate = feed;
        c.dwell = p.dwell;
        c.pitch = type == ToolType::Tap ? ctx.tool.mill.threadPitch : 0.0;
        c.type = p.cycle.value_or(h.depth > 3.0 * d && defaultType == toolpath::CycleType::Drill
                                      ? toolpath::CycleType::Peck
                                      : defaultType);
        if (c.type == toolpath::CycleType::Tap && type == ToolType::Tap && ctx.tool.mill.leftHand) {
            c.type = toolpath::CycleType::TapLeft;
        }
        if (c.type == toolpath::CycleType::Peck || c.type == toolpath::CycleType::ChipBreak) {
            c.peck = p.peckDepth.value_or(kPeckShare * d);
        }
        if (h.topZ - c.bottomZ > ctx.tool.mill.fluteLength && !warnedFlute) {
            tp.warnings.push_back("a hole is deeper than the tool's flute length");
            warnedFlute = true;
        }
        // Holes at different heights: travel between them at clearance height.
        if (std::abs(h.topZ - previousTop) > 1e-9) {
            b.retractTo(ctx.heights.clearance);
        }
        b.retractTo(c.retractZ);
        if ((b.position().head<2>() - h.position).norm() > 1e-9) {
            b.rapid(Vec3(h.position.x(), h.position.y(), b.position().z()));
        }
        b.cycle(h.position, c);
        previousTop = h.topZ;
        ctx.progress.report(0.5, "Drilling");
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

} // namespace ocam::strategies
