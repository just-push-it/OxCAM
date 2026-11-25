// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Steep and shallow finishing: the part's surface split by the slope under the tool into steep
// areas, finished with constant-Z passes (waterline, or contour finishing by cusp height), and
// shallow ones, finished with parallel passes, the two overlapping by a band. It adds no kernel:
// the split only sets the containment of the existing ones (DL-0017, DL-0033). See
// docs/design-log/0080.

#include "Internal.hpp"
#include "ocam/strategies/Surface.hpp"

#include <ocam/geom2d/Clip.hpp>
#include <ocam/geom2d/Contour.hpp>

#include <algorithm>
#include <cmath>

namespace ocam::strategies {

geom2d::Regions shallowAreas(const DropCutter& dc, const geom2d::Aabb2& area, double spacing,
                             double floor, double thresholdAngle, double chordTolerance,
                             CancelToken& cancel) {
    const HeightField field = computeHeightField(dc, area, spacing, floor, cancel);
    const double h = field.spacing;
    const double limit = std::tan(std::clamp(thresholdAngle, 0.0, degToRad(89.9)));
    auto onPart = [&](int i, int j) {
        return i >= 0 && j >= 0 && i < field.nx && j < field.ny && field.at(i, j) > floor + 1e-9;
    };
    // Shallow where the cutter-location surface (the part's slope where the tool touches it) is
    // flatter than the threshold: the field tan(threshold) - slope, positive there, traced by
    // marching squares. Off the part, and next to where the tool falls off it, it is not.
    geom2d::ScalarGrid grid;
    grid.origin = field.origin;
    grid.spacing = Vec2::Constant(h);
    grid.nx = field.nx;
    grid.ny = field.ny;
    grid.values.assign(field.z.size(), -1.0);
    for (int j = 0; j < field.ny; ++j) {
        cancel.throwIfCancelled();
        for (int i = 0; i < field.nx; ++i) {
            if (onPart(i, j) && onPart(i - 1, j) && onPart(i + 1, j) && onPart(i, j - 1) &&
                onPart(i, j + 1)) {
                const double gx = (field.at(i + 1, j) - field.at(i - 1, j)) / (2.0 * h);
                const double gy = (field.at(i, j + 1) - field.at(i, j - 1)) / (2.0 * h);
                grid.values[static_cast<std::size_t>(j) * static_cast<std::size_t>(field.nx) +
                            static_cast<std::size_t>(i)] = limit - std::hypot(gx, gy);
            }
        }
    }
    const geom2d::ClipOptions clip{chordTolerance};
    geom2d::Regions shallow = geom2d::isoRegions(grid, 0.0);
    // Drop specks a cell or two across, and nicks as narrow.
    shallow = geom2d::offset(shallow, -h, clip);
    shallow = geom2d::offset(shallow, 2.0 * h, clip);
    shallow = geom2d::offset(shallow, -h, clip);
    return shallow;
}

toolpath::Toolpath steepShallowFinish(const geom3d::Mesh& part, const SteepShallowParams& p,
                                      const Context& ctx) {
    const double r = detail::requireMillingTool(ctx.tool);
    if (part.isEmpty()) {
        throw Error(ErrorCode::InvalidArgument,
                    "steep and shallow finishing needs the part's surface");
    }
    if (p.steepScallop <= 0.0 && p.stepDown <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the steep areas' step down must be positive");
    }
    if (p.overlap < 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the overlap cannot be negative");
    }
    const double allowance = p.stockToLeave;
    const DropCutter dc(part, Cutter(ctx.tool).offset(allowance));
    const geom3d::Aabb3 bounds = part.bounds();
    const double res = p.resolution > 0.0 ? p.resolution : std::clamp(r / 6.0, 0.05, 1.0);
    const double floor = bounds.min.z() - 10.0;
    const double tol = std::max(ctx.chordTolerance, 1e-3);
    const geom2d::ClipOptions clip{tol};
    geom2d::Aabb2 area;
    area.extend(bounds.min.head<2>() - Vec2::Constant(r + allowance + 3.0 * res));
    area.extend(bounds.max.head<2>() + Vec2::Constant(r + allowance + 3.0 * res));
    area = detail::containedArea(area, p.boundary, 3.0 * res);

    toolpath::Toolpath tp = beginToolpath(ctx, "steep_shallow3d", area.center());
    ProgressRange splitRange(ctx.progress, 0.0, 0.2);
    splitRange.report(0.0, "Steep and shallow areas");
    const geom2d::Regions shallow =
        shallowAreas(dc, area, res, floor, p.thresholdAngle, tol, ctx.cancel);
    splitRange.report(1.0, "Steep and shallow areas");

    // Each kind of pass reaches half the overlap past the line between the areas.
    const double half = 0.5 * p.overlap;
    const geom2d::Regions all{{geom2d::Polyline::rectangle(area.min - Vec2::Constant(1.0),
                                                           area.max + Vec2::Constant(1.0)),
                               {}}};
    // Parallel finishing's own area: the tool centre over the part's plan bounds.
    const geom2d::Regions partRect{
        {geom2d::Polyline::rectangle(bounds.min.head<2>(), bounds.max.head<2>()), {}}};
    geom2d::Regions shallowZone = geom2d::intersect(
        half > 0.0 ? geom2d::offset(shallow, half, clip) : shallow, partRect, clip);
    geom2d::Regions steepZone =
        geom2d::subtract(all, half > 0.0 ? geom2d::offset(shallow, -half, clip) : shallow, clip);
    if (!p.boundary.empty()) {
        shallowZone = geom2d::intersect(shallowZone, p.boundary, clip);
        steepZone = geom2d::intersect(steepZone, p.boundary, clip);
    }

    // The steep areas: waterline levels a step down apart, or contour finishing's cusp-spaced
    // levels; the shallow ones: parallel passes. Same tool, same feeds (ADR-0004).
    const auto steepPasses = [&](ProgressSink& progress) {
        const Context sub{ctx.tool,           ctx.feeds,  ctx.heights,
                          ctx.chordTolerance, ctx.cancel, progress};
        if (steepZone.empty()) {
            return toolpath::Toolpath{};
        }
        if (p.steepScallop > 0.0) {
            ContourFinishParams cp;
            cp.scallopHeight = p.steepScallop;
            cp.minSlope = std::clamp(p.thresholdAngle, 0.0, degToRad(89.0));
            cp.stockToLeave = allowance;
            cp.resolution = res;
            cp.topZ = p.topZ;
            cp.bottomZ = p.bottomZ;
            cp.direction = p.direction;
            cp.boundary = steepZone;
            toolpath::Toolpath steep = contourFinish(part, cp, sub);
            // Contour finishing comes down by rapid to just above each level it enters. Here
            // a level's pieces start part way down the part with the roughing's stairs still
            // above them (the shallow areas come later or not at all), so every entry is a
            // plunge at the plunge feed from the retract height, as parallel and waterline
            // finishing enter: the rapid down before each plunge stops at the retract height.
            const double retract = std::max(ctx.heights.retract,
                                            bounds.max.z() + allowance + ctx.heights.feedClearance);
            std::vector<toolpath::Move> moves;
            Vec3 at = steep.start;
            for (std::size_t k = 0; k < steep.moves.size(); ++k) {
                toolpath::Move m = steep.moves[k];
                const bool down = m.kind == toolpath::MoveKind::Rapid &&
                                  (m.end.head<2>() - at.head<2>()).norm() < 1e-9 &&
                                  m.end.z() < at.z();
                if (down && k + 1 < steep.moves.size() &&
                    steep.moves[k + 1].feedClass == toolpath::FeedClass::Plunge) {
                    if (at.z() <= retract + 1e-9) {
                        continue; // already at the retract height: the plunge starts here
                    }
                    m.end.z() = std::max(m.end.z(), retract);
                }
                moves.push_back(m);
                at = m.end;
            }
            steep.moves = std::move(moves);
            return steep;
        }
        WaterlineParams wp;
        wp.stepDown = p.stepDown;
        wp.stockToLeave = allowance;
        wp.resolution = res;
        wp.topZ = p.topZ;
        wp.bottomZ = p.bottomZ;
        wp.direction = p.direction;
        wp.boundary = steepZone;
        return waterline(part, wp, sub);
    };
    const auto shallowPasses = [&](ProgressSink& progress) {
        const Context sub{ctx.tool,           ctx.feeds,  ctx.heights,
                          ctx.chordTolerance, ctx.cancel, progress};
        if (shallowZone.empty()) {
            return toolpath::Toolpath{};
        }
        ParallelParams pp;
        pp.stepover = p.stepover;
        pp.scallopHeight = p.shallowScallop;
        pp.angle = p.angle;
        pp.sampleStep = p.sampleStep;
        pp.stockToLeave = allowance;
        pp.zigzag = p.zigzag;
        pp.byArea = true; // the shallow areas one by one, not line by line across them
        pp.boundary = shallowZone;
        return parallelFinish(part, bounds.min.z(), pp, sub);
    };

    const auto hasCuts = [](const toolpath::Toolpath& t) {
        return std::any_of(t.moves.begin(), t.moves.end(), [](const toolpath::Move& m) {
            return m.feedClass == toolpath::FeedClass::Cut;
        });
    };
    bool any = false;
    const auto append = [&](toolpath::Toolpath piece) {
        if (!hasCuts(piece)) {
            return;
        }
        if (!any) {
            tp.start = piece.start;
        } else {
            // Across to where the next passes begin, at the clearance height or the passes'
            // retract height (over the part's top), whichever is higher.
            const double travel =
                std::max({ctx.heights.clearance, ctx.heights.retract,
                          bounds.max.z() + allowance + ctx.heights.feedClearance});
            Vec3 at = tp.moves.empty() ? tp.start : tp.moves.back().end;
            if (at.z() < travel) {
                at.z() = travel;
                tp.moves.push_back({toolpath::MoveKind::Rapid, toolpath::FeedClass::Rapid, at});
            }
            const Vec3 to(piece.start.x(), piece.start.y(), at.z());
            if ((to - at).norm() > 1e-9) {
                tp.moves.push_back({toolpath::MoveKind::Rapid, toolpath::FeedClass::Rapid, to});
            }
        }
        tp.moves.insert(tp.moves.end(), piece.moves.begin(), piece.moves.end());
        tp.warnings.insert(tp.warnings.end(), piece.warnings.begin(), piece.warnings.end());
        any = true;
    };
    ProgressRange firstRange(ctx.progress, 0.2, 0.6);
    ProgressRange secondRange(ctx.progress, 0.6, 1.0);
    if (p.steepFirst) {
        append(steepPasses(firstRange));
        append(shallowPasses(secondRange));
    } else {
        append(shallowPasses(firstRange));
        append(steepPasses(secondRange));
    }
    if (!any) {
        tp.warnings.push_back("no part surface to finish between the top and bottom heights");
    }
    return tp;
}

SteepShallowParams steepShallowParams(const Json& values, const tooling::Tool& /*tool*/,
                                      const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("steep_shallow3d"), values);
    SteepShallowParams p;
    p.thresholdAngle = r.number("threshold_angle");
    p.overlap = r.number("overlap");
    // Steep areas: the levels the preset's ap apart unless a cusp height spaces them.
    p.steepScallop = r.number("steep_scallop");
    if (p.steepScallop <= 0.0) {
        p.stepDown = presetStepDown(preset);
    }
    // Shallow areas: the stepover is the preset's ae unless a scallop height sets it.
    p.shallowScallop = r.number("shallow_scallop");
    p.stepover = p.shallowScallop > 0.0 ? 0.0 : presetStepover(preset);
    p.angle = r.number("angle");
    p.sampleStep = r.number("sample_step");
    p.zigzag = r.choice("pattern") != "one_way";
    p.stockToLeave = r.number("stock_to_leave");
    p.resolution = r.number("resolution");
    p.direction =
        r.choice("direction") == "conventional" ? CutDirection::Conventional : CutDirection::Climb;
    p.steepFirst = r.choice("order") != "shallow_first";
    return p;
}

} // namespace ocam::strategies
