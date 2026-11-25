// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Edge chamfer with a pointed tool: the cone of the tool cuts the chamfer while the tool follows
// an offset of the edge at a fixed depth. See docs/design-log/0014.

#include "Internal.hpp"
#include "ocam/strategies/Strategies.hpp"

#include <cmath>

namespace ocam::strategies {

ChamferGeometry chamferGeometry(const tooling::Tool& tool, double width, double tipOffset) {
    using tooling::ToolType;
    const double r = detail::requireMillingTool(tool);
    const ToolType t = tool.type;
    const bool pointed = t == ToolType::Chamfer || t == ToolType::Engraver ||
                         t == ToolType::SpotDrill || t == ToolType::Drill ||
                         t == ToolType::CenterDrill;
    if (!pointed || tool.mill.tipAngle <= 0.0 || tool.mill.tipAngle >= kPi) {
        throw Error(ErrorCode::InvalidArgument,
                    "chamfering needs a pointed tool (chamfer mill, engraver or spot drill) with a "
                    "point angle");
    }
    if (width <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the chamfer width must be positive");
    }
    const double half = tool.mill.tipAngle / 2.0;
    const double tipRadius =
        t == ToolType::Chamfer || t == ToolType::Engraver ? tool.mill.tipDiameter / 2.0 : 0.0;
    ChamferGeometry g;
    g.halfAngle = half;
    g.depth = width / std::tan(half);
    g.centreOffset = tipRadius + tipOffset * std::tan(half);
    g.tipBelowEdge = g.depth + tipOffset;
    if (g.centreOffset + width > r + 1e-9) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("the chamfer is too wide for T{}: its cone reaches {:.3f} mm from "
                                "the axis, the tool is {:.3f} mm",
                                tool.number, g.centreOffset + width, r));
    }
    return g;
}

toolpath::Toolpath chamfer(const geom2d::Regions& shape, double edgeZ, const ChamferParams& p,
                           const Context& ctx) {
    const double r = detail::requireMillingTool(ctx.tool);
    const ChamferGeometry g = chamferGeometry(ctx.tool, p.width, p.tipOffset);
    // The contour kernel places the tool centre at r + stock-to-leave from the edge.
    ContourParams cp;
    cp.side = p.side;
    cp.stockToLeaveXY = g.centreOffset - r;
    cp.stepDown = 0.0;
    cp.passes = 1;
    cp.leadRadius = p.leadRadius.value_or(kChamferLeadShare * 2.0 * r);
    cp.direction = p.direction;
    toolpath::Toolpath tp = contour(shape, edgeZ, edgeZ - g.tipBelowEdge, cp, ctx);
    tp.strategy = "chamfer2d";
    return tp;
}

} // namespace ocam::strategies
