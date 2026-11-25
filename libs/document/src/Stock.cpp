// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/document/Stock.hpp"

#include <ocam/foundation/Error.hpp>

#include <algorithm>
#include <cmath>

namespace ocam::document {

Transform3 StockShape::boxToProject() const {
    Transform3 t = Transform3::Identity();
    t.linear() = axes;
    return t;
}

std::array<Vec3, 8> StockShape::corners() const {
    std::array<Vec3, 8> out;
    for (int i = 0; i < 8; ++i) {
        out[static_cast<std::size_t>(i)] =
            axes * Vec3((i & 1) ? box.max.x() : box.min.x(), (i & 2) ? box.max.y() : box.min.y(),
                        (i & 4) ? box.max.z() : box.min.z());
    }
    return out;
}

geom3d::Aabb3 StockShape::bounds() const {
    if (kind == Kind::Box) {
        geom3d::Aabb3 b;
        for (const Vec3& c : corners()) {
            b.extend(c);
        }
        return b;
    }
    // Bounds of a disc of radius r perpendicular to a: extent along each world axis i is
    // r * sqrt(1 - a_i^2).
    const Vec3 a = axis.normalized();
    const double r = diameter / 2.0;
    const Vec3 extent(r * std::sqrt(std::max(0.0, 1.0 - a.x() * a.x())),
                      r * std::sqrt(std::max(0.0, 1.0 - a.y() * a.y())),
                      r * std::sqrt(std::max(0.0, 1.0 - a.z() * a.z())));
    geom3d::Aabb3 b;
    for (const Vec3& c : {base, Vec3(base + a * length)}) {
        b.extend(c - extent);
        b.extend(c + extent);
    }
    return b;
}

namespace {

/// `size` rounded up to a multiple of `step` (sizes already on one, to rounding noise, stay).
double roundUp(double size, double step) {
    return std::max(size, std::ceil(size / step - 1e-9) * step);
}

} // namespace

std::optional<StockShape> resolveStock(const Project& project, const Setup& setup,
                                       const ModelBoundsFn& modelBounds,
                                       const ModelRadiusFn& modelRadius) {
    const Stock& s = setup.stock;
    StockShape shape;
    if (s.kind == StockKind::FromSetup) {
        // Only earlier setups are valid sources, which also prevents infinite recursion.
        for (const auto& other : project.setups) {
            if (other.id == setup.id) {
                return std::nullopt;
            }
            if (s.sourceSetup && other.id == *s.sourceSetup) {
                return resolveStock(project, other, modelBounds, modelRadius);
            }
        }
        return std::nullopt;
    }
    const bool fixed = s.size == StockSize::Fixed;
    if (fixed && s.kind == StockKind::Box && s.boxCorners) {
        shape.box.extend((*s.boxCorners)[0]); // project frame (files of earlier versions)
        shape.box.extend((*s.boxCorners)[1]);
        return shape;
    }
    if (fixed && s.kind != StockKind::Box && s.cylinderPlacement) {
        // Project frame (files of earlier versions).
        if ((*s.cylinderPlacement)[1].isZero()) {
            return std::nullopt;
        }
        shape.kind = StockShape::Kind::Cylinder;
        shape.base = (*s.cylinderPlacement)[0];
        shape.axis = (*s.cylinderPlacement)[1].normalized();
        shape.diameter = s.diameter;
        shape.length = s.length;
    } else {
        // Along the WCS axes, so sizes, offsets and the shift are in the setup's directions.
        Frame wcs;
        try {
            wcs = setup.wcs.normalized();
        } catch (const Error&) {
            return std::nullopt;
        }
        Mat3 axes;
        axes.col(0) = wcs.xAxis;
        axes.col(1) = wcs.yAxis();
        axes.col(2) = wcs.zAxis;
        // The part's box in WCS directions; fixed stock without a model is on the origin.
        std::optional<geom3d::Aabb3> part;
        if (setup.modelId) {
            part = modelBounds(*setup.modelId, axes);
            if (!part || part->isEmpty()) {
                return std::nullopt;
            }
        } else if (!fixed) {
            return std::nullopt;
        }
        const Vec3 centre = part ? part->center() : Vec3::Zero();
        const bool offset = s.size == StockSize::OffsetFromPart;
        const double side = offset ? s.offsetSide : 0.0;
        const double top = offset ? s.offsetTop : 0.0;
        const double bottom = offset ? s.offsetBottom : 0.0;
        const double step = offset ? s.roundTo : 0.0;
        if (s.kind == StockKind::Box) {
            shape.axes = axes;
            Vec3 lo, hi;
            if (fixed) {
                lo = centre - 0.5 * s.boxSize;
                hi = centre + 0.5 * s.boxSize;
            } else {
                lo = Vec3(part->min.x() - side, part->min.y() - side, part->min.z() - bottom);
                hi = Vec3(part->max.x() + side, part->max.y() + side, part->max.z() + top);
                for (int a = 0; a < 3 && step > 0.0; ++a) {
                    const double extra = roundUp(hi[a] - lo[a], step) - (hi[a] - lo[a]);
                    if (a < 2) {
                        lo[a] -= extra / 2.0;
                        hi[a] += extra / 2.0;
                    } else {
                        hi[a] += extra; // the bottom stays on the model
                    }
                }
            }
            shape.box.extend(lo + s.shift);
            shape.box.extend(hi + s.shift);
            return shape;
        }
        if (s.axis < 0 || s.axis > 2) {
            return std::nullopt;
        }
        const int k = s.axis;
        shape.kind = StockShape::Kind::Cylinder;
        double a0 = 0.0, a1 = 0.0; // the ends along the axis, WCS coordinates
        if (fixed) {
            shape.diameter = s.diameter;
            a0 = centre[k] - 0.5 * s.length;
            a1 = a0 + s.length;
        } else {
            // The axis through the centre of the part's box; the part's largest radius about it.
            const auto radius = modelRadius
                                    ? modelRadius(*setup.modelId, axes * centre, axes.col(k))
                                    : std::nullopt;
            if (!radius) {
                return std::nullopt;
            }
            shape.diameter = 2.0 * (*radius + side);
            a0 = part->min[k] - bottom;
            a1 = part->max[k] + top;
            if (step > 0.0) {
                shape.diameter = roundUp(shape.diameter, step);
                const double extra = roundUp(a1 - a0, step) - (a1 - a0);
                if (k == 2) {
                    a1 += extra; // the bottom stays on the model
                } else {
                    a0 -= extra / 2.0;
                    a1 += extra / 2.0;
                }
            }
        }
        Vec3 base = centre;
        base[k] = a0;
        shape.base = axes * (base + s.shift);
        shape.axis = axes.col(k);
        shape.length = a1 - a0;
    }
    if (s.kind == StockKind::Tube) {
        if (!(s.innerDiameter > 0.0 && s.innerDiameter < shape.diameter)) {
            return std::nullopt; // a bore must be inside the stock
        }
        shape.innerDiameter = s.innerDiameter;
    }
    return shape;
}

} // namespace ocam::document
