// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Hole-making beyond drilling: rigid tapping, helical thread milling and boring (a boring cycle
// for reamers and boring bars, helical interpolation for end mills). See docs/design-log/0014.

#include "Internal.hpp"
#include "ocam/strategies/Strategies.hpp"

#include <ocam/geom2d/Ordering.hpp>

#include <algorithm>
#include <cmath>

namespace ocam::strategies {
namespace {

using toolpath::FeedClass;

std::vector<std::size_t> holeOrder(const std::vector<HoleTarget>& holes) {
    std::vector<Vec2> points;
    for (const auto& h : holes) {
        points.push_back(h.position);
    }
    return geom2d::orderPoints(points, Vec2::Zero());
}

/// Moves to above `xy` at the R plane of a hole, travelling at clearance height between holes
/// at different heights.
void approachHole(toolpath::PathBuilder& b, const Context& ctx, const Vec2& xy, double rPlane,
                  double& previousTop, double top) {
    if (std::abs(top - previousTop) > 1e-9) {
        b.retractTo(ctx.heights.clearance);
    }
    b.retractTo(rPlane);
    if ((b.position().head<2>() - xy).norm() > 1e-9) {
        b.rapid(Vec3(xy.x(), xy.y(), b.position().z()));
    }
    if (b.position().z() > rPlane + 1e-9) {
        b.rapid(Vec3(xy.x(), xy.y(), rPlane));
    }
    previousTop = top;
}

} // namespace

toolpath::Toolpath tap(const std::vector<HoleTarget>& holes, const TapParams& p,
                       const Context& ctx) {
    detail::requireMillingTool(ctx.tool);
    if (ctx.tool.type != tooling::ToolType::Tap) {
        throw Error(ErrorCode::InvalidArgument, "tapping needs a tap");
    }
    std::vector<HoleTarget> targets = holes;
    if (p.threadDepth > 0.0) {
        for (auto& h : targets) {
            h.through = h.through && p.threadDepth >= h.depth;
            h.depth = std::min(h.depth, p.threadDepth);
        }
    }
    DrillParams dp;
    dp.cycle = ctx.tool.mill.leftHand ? toolpath::CycleType::TapLeft : toolpath::CycleType::Tap;
    dp.depthMode = DepthMode::Tip;
    dp.retractClearance = p.retractClearance;
    dp.rPlaneZ = p.rPlaneZ;
    dp.breakthrough = p.breakthrough;
    toolpath::Toolpath tp = drill(targets, dp, ctx);
    tp.strategy = "tap";
    tp.spindleReverse = ctx.tool.mill.leftHand;
    return tp;
}

ThreadMillGeometry threadMillGeometry(const ThreadMillParams& p, double pitch, double diameter,
                                      double r) {
    ThreadMillGeometry g;
    // ISO 68-1 basic profile; the depths are those of turning (DL-0047): 0.6134 P for an
    // external thread's root, 0.5413 P (5/8 H) for an internal one's.
    g.threadDepth = p.threadDepth > 0.0 ? p.threadDepth : (p.external ? 0.6134 : 0.5413) * pitch;
    if (p.majorDiameter > 0.0) {
        g.major = p.majorDiameter;
    } else if (diameter > 0.0) {
        // A stud is turned to its major diameter; a hole is drilled to the minor, major - 1.0825 P.
        g.major = p.external ? diameter : diameter + 1.0825 * pitch;
    }
    if (g.major <= 0.0) {
        return g;
    }
    const int passes = std::max(1, p.radialPasses);
    for (int k = 1; k <= passes; ++k) {
        const double in = g.threadDepth * k / passes; // into the material, from its surface
        g.passRadii.push_back(p.external ? g.major / 2.0 - in + r
                                         : g.major / 2.0 - g.threadDepth + in - r);
    }
    if (p.springPass) {
        g.passRadii.push_back(g.passRadii.back());
    }
    // With M03 the tool climbs going counter-clockwise inside a hole and clockwise around a boss.
    // A right-hand thread advances towards the tool going counter-clockwise (a right-hand helix).
    const bool climb = p.direction == CutDirection::Climb;
    g.counterClockwise = p.external ? !climb : climb;
    g.upward = g.counterClockwise == p.rightHand;
    return g;
}

toolpath::Toolpath threadMill(const std::vector<HoleTarget>& holes, const ThreadMillParams& p,
                              const Context& ctx) {
    const double r = detail::requireMillingTool(ctx.tool);
    if (ctx.tool.type != tooling::ToolType::ThreadMill) {
        throw Error(ErrorCode::InvalidArgument, "thread milling needs a thread mill");
    }
    const double pitch = p.pitch > 0.0 ? p.pitch : ctx.tool.mill.threadPitch;
    if (pitch <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    "set the thread pitch (on the operation or the thread mill)");
    }
    const double lead = p.leadRadius.value_or(kThreadLeadShare * 2.0 * r);
    const auto order = holeOrder(holes);
    toolpath::Toolpath tp = beginToolpath(
        ctx, "threadmill", holes.empty() ? Vec2::Zero() : holes[order.front()].position);
    if (holes.empty()) {
        tp.warnings.push_back(p.external ? "no bosses to thread" : "no holes to thread");
        return tp;
    }
    toolpath::PathBuilder b(tp);
    double previousTop = holes[order.front()].topZ;
    for (std::size_t idx : order) {
        ctx.cancel.throwIfCancelled();
        const HoleTarget& h = holes[idx];
        const ThreadMillGeometry g = threadMillGeometry(p, pitch, h.diameter, r);
        const auto at = [&] {
            return fmt::format("({:.3f}, {:.3f})", h.position.x(), h.position.y());
        };
        if (!p.external && h.diameter > 0.0 && 2.0 * r >= h.diameter) {
            tp.warnings.push_back(
                fmt::format("thread mill does not fit the hole at {}; skipped", at()));
            continue;
        }
        if (g.passRadii.empty() ||
            *std::min_element(g.passRadii.begin(), g.passRadii.end()) < 0.05) {
            tp.warnings.push_back(
                fmt::format("thread at {} is too small for the tool; skipped", at()));
            continue;
        }
        if (p.external && h.diameter > g.major + 1e-6) {
            tp.warnings.push_back(fmt::format(
                "the boss at {} is Ø{:.3f}, larger than the major diameter Ø{:.3f}; skipped", at(),
                h.diameter, g.major));
            continue;
        }
        if (p.external && lead <= g.threadDepth) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("the lead radius ({:.3f} mm) must be larger than the thread "
                                    "depth ({:.3f} mm), so the tool starts outside the major "
                                    "diameter: set the operation's lead radius",
                                    lead, g.threadDepth));
        }
        const double length = p.depth > 0.0 ? std::min(p.depth, h.depth) : h.depth;
        if (length <= 0.0) {
            continue;
        }
        // A multi-form tool covers the whole length in one revolution; a single point one goes
        // one pitch per revolution over the length.
        if (p.multiForm && ctx.tool.mill.fluteLength < length - 1e-9) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("the thread mill's flute length ({:.3f} mm) is shorter than "
                                    "the thread ({:.3f} mm): a multi-form tool cuts the whole "
                                    "length in one turn. Choose single point, or shorten the "
                                    "thread",
                                    ctx.tool.mill.fluteLength, length));
        }
        const int turns = p.multiForm ? 1 : static_cast<int>(std::ceil(length / pitch - 1e-9));
        const double bottom = h.topZ - length;
        const double top = bottom + turns * pitch;
        const double zStart = g.upward ? bottom : top;
        const double rise = (g.upward ? 1.0 : -1.0) * pitch / 2.0; // each half turn
        const bool cw = !g.counterClockwise;
        // Feeds as set: the cut feed for the tool centre on the helix, the lead feed on the leads.
        const double feed = ctx.feeds.cut;
        const Vec2 c = h.position;
        const double rPlane =
            p.rPlaneZ ? std::max(*p.rPlaneZ, h.topZ) : h.topZ + p.retractClearance;
        for (const double rh : g.passRadii) {
            const Vec2 start = c + Vec2(rh, 0.0); // on the helix, at angle 0
            Vec2 leadCentre, entry, exit;
            bool leadCw = cw;
            if (p.external) {
                // A quarter of a circle outside the helix, turning the other way: tangent to it
                // at angle 0, from a point outside the major diameter and back out to one.
                const double side = g.counterClockwise ? 1.0 : -1.0;
                leadCentre = c + Vec2(rh + lead, 0.0);
                entry = c + Vec2(rh + lead, -side * lead);
                exit = c + Vec2(rh + lead, side * lead);
                leadCw = !cw;
            } else {
                // A semicircle from the hole's axis.
                leadCentre = c + Vec2(rh / 2.0, 0.0);
                entry = c;
                exit = c;
            }
            approachHole(b, ctx, entry, rPlane, previousTop, h.topZ);
            b.feed(Vec3(entry.x(), entry.y(), zStart), FeedClass::Plunge, ctx.feeds.plunge);
            b.arc(Vec3(start.x(), start.y(), zStart), leadCentre, leadCw, FeedClass::LeadIn,
                  ctx.feeds.lead);
            double z = zStart;
            for (int t = 0; t < turns; ++t) {
                const Vec2 opposite = c - Vec2(rh, 0.0);
                b.arc(Vec3(opposite.x(), opposite.y(), z + rise), c, cw, FeedClass::Cut, feed);
                b.arc(Vec3(start.x(), start.y(), z + 2.0 * rise), c, cw, FeedClass::Cut, feed);
                z += 2.0 * rise;
            }
            b.arc(Vec3(exit.x(), exit.y(), z), leadCentre, leadCw, FeedClass::LeadOut,
                  ctx.feeds.lead);
            b.retractTo(rPlane);
        }
        ctx.progress.report(0.5, "Thread milling");
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

toolpath::Toolpath bore(const std::vector<HoleTarget>& holes, const BoreParams& p,
                        const Context& ctx) {
    using tooling::ToolType;
    const double r = detail::requireMillingTool(ctx.tool);
    if (ctx.tool.type == ToolType::Reamer || ctx.tool.type == ToolType::BoringBar) {
        DrillParams dp;
        dp.cycle = toolpath::CycleType::Bore;
        dp.depthMode = DepthMode::Tip;
        dp.retractClearance = p.retractClearance;
        dp.rPlaneZ = p.rPlaneZ;
        dp.breakthrough = p.breakthrough;
        toolpath::Toolpath tp = drill(holes, dp, ctx);
        tp.strategy = "bore";
        return tp;
    }
    if (p.helixPitch <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the helix pitch must be positive");
    }
    const auto order = holeOrder(holes);
    toolpath::Toolpath tp =
        beginToolpath(ctx, "bore", holes.empty() ? Vec2::Zero() : holes[order.front()].position);
    if (holes.empty()) {
        tp.warnings.push_back("no holes to bore");
        return tp;
    }
    toolpath::PathBuilder b(tp);
    const bool clockwise = p.direction == CutDirection::Conventional; // climb: CCW inside a hole
    double previousTop = holes[order.front()].topZ;
    bool warnedFlute = false;
    for (std::size_t idx : order) {
        ctx.cancel.throwIfCancelled();
        const HoleTarget& h = holes[idx];
        const double rh = h.diameter / 2.0 - p.stockToLeave - r;
        if (rh < 0.02 * r) {
            tp.warnings.push_back(
                fmt::format("hole at ({:.3f}, {:.3f}) is not larger than the tool; skipped",
                            h.position.x(), h.position.y()));
            continue;
        }
        const double bottom = h.topZ - h.depth - (h.through ? p.breakthrough : 0.0);
        if (h.topZ - bottom > ctx.tool.mill.fluteLength && !warnedFlute) {
            tp.warnings.push_back("a hole is deeper than the tool's flute length");
            warnedFlute = true;
        }
        const Vec2 c = h.position;
        const Vec2 start = c + Vec2(rh, 0.0);
        const double rPlane =
            p.rPlaneZ ? std::max(*p.rPlaneZ, h.topZ) : h.topZ + p.retractClearance;
        approachHole(b, ctx, start, rPlane, previousTop, h.topZ);
        b.feed(Vec3(start.x(), start.y(), h.topZ + ctx.heights.feedClearance), FeedClass::Plunge,
               ctx.feeds.plunge);
        // Helical descent, then a full circle at the bottom (PathBuilder::helix).
        b.helix(c, rh, bottom, p.helixPitch, clockwise, ctx.feeds.cut);
        b.feed(Vec3(c.x(), c.y(), bottom), FeedClass::Link, ctx.feeds.cut);
        b.retractTo(rPlane);
        ctx.progress.report(0.5, "Boring");
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

} // namespace ocam::strategies
