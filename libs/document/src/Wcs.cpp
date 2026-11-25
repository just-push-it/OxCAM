// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/document/Wcs.hpp"

#include "ocam/document/Document.hpp"
#include "ocam/document/Workholding.hpp"

#include <ocam/cad/Datum.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Units.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>

namespace ocam::document {
namespace {

Error userError(const std::string& message) {
    return Error(ErrorCode::InvalidArgument, message);
}

Vec3 pickedDirection(const AxisPick& pick) {
    return pick.flip ? Vec3(-pick.direction) : pick.direction;
}

/// `v` made square to the unit `z`; nullopt when (nearly) parallel to it.
std::optional<Vec3> squareTo(const Vec3& v, const Vec3& z) {
    const Vec3 s = v - v.dot(z) * z;
    if (s.norm() < 1e-6) {
        return std::nullopt;
    }
    return Vec3(s.normalized());
}

/// The setup's WCS axes as the columns of a matrix (x, y, z), orthonormal.
Mat3 axesOf(const Setup& setup) {
    const Frame f = setup.wcs.normalized();
    Mat3 m;
    m << f.xAxis, f.yAxis(), f.zAxis;
    return m;
}

/// The face, edge or vertex a reference names, found again on its model by its signature.
std::pair<const cad::Model*, int> resolve(const Document& doc, const ShapeRef& shape) {
    const cad::Model* model = doc.model(shape.modelId);
    if (!model) {
        throw userError("the picked model is not loaded");
    }
    const auto index = cad::matchSubShape(*model, shape.type, shape.signature, shape.index);
    if (!index) {
        throw userError("the picked face, edge or vertex is no longer on the model: pick it again");
    }
    return {model, *index};
}

} // namespace

Frame orientedAxes(const WcsOrientation& o, const Frame& current) {
    Frame f = current.normalized();
    if (o.mode == OrientationMode::Default) {
        f.xAxis = Vec3::UnitX();
        f.zAxis = Vec3::UnitZ();
        return f;
    }
    const Vec3 z = o.z ? Vec3(pickedDirection(*o.z).normalized()) : f.zAxis;
    const bool alongY = o.mode == OrientationMode::YZ;
    // The other axis: the pick's, else the current one, made square to Z.
    const Vec3 wanted = o.first ? pickedDirection(*o.first) : alongY ? f.yAxis() : f.xAxis;
    std::optional<Vec3> other = squareTo(wanted, z);
    if (!other) {
        if (o.first && o.z) {
            throw userError(fmt::format("the {} and Z picks are parallel: pick a face or edge "
                                        "across Z",
                                        alongY ? "Y" : "X"));
        }
        // A new Z along the old axis: any axis square to it.
        other = squareTo(alongY ? Vec3(-f.zAxis) : f.zAxis, z);
        if (!other) {
            other = Vec3(z.unitOrthogonal());
        }
    }
    f.zAxis = z;
    f.xAxis = alongY ? Vec3(other->cross(z)) : *other;
    return f.normalized();
}

void setOrientation(Setup& setup, const WcsOrientation& orientation, bool onWorkholding) {
    const Frame before = setup.wcs.normalized();
    const Frame after = orientedAxes(orientation, setup.wcs);
    if (onWorkholding && setup.workholding) {
        if ((after.zAxis - before.zAxis).norm() > 1e-6) {
            throw userError("the workholding stands on the WCS Z axis: pick Z on the part or a "
                            "fixture model");
        }
        // The turn about Z from the old X to the new one, in quarter turns.
        const double turn = radToDeg(std::atan2(before.xAxis.cross(after.xAxis).dot(before.zAxis),
                                                before.xAxis.dot(after.xAxis)));
        const double quarters = std::round(turn / 90.0);
        if (std::abs(turn - 90.0 * quarters) > 1e-4) {
            throw userError("pick an edge or face of the workholding along or across its jaws");
        }
        const int rotation = setup.workholding->rotation - static_cast<int>(quarters) * 90;
        setup.workholding->rotation = ((rotation % 360) + 360) % 360;
    }
    setup.wcs.xAxis = after.xAxis;
    setup.wcs.zAxis = after.zAxis;
    setup.orientation = orientation;
}

Vec3 axisOfShape(const Document& doc, const ShapeRef& shape) {
    const auto [model, index] = resolve(doc, shape);
    const ModelEntry* entry = doc.project().findModel(shape.modelId);
    const cad::DatumGeometry g = cad::datumGeometry(*model, shape.type, index);
    if (g.shape == cad::DatumGeometry::Shape::Point || g.direction.norm() < 1e-9) {
        throw userError("pick a flat or round face, or a straight or circular edge");
    }
    return (entry->placement.toParent().linear() * g.direction).normalized();
}

Vec3 pointOfShape(const Document& doc, const ShapeRef& shape) {
    const auto [model, index] = resolve(doc, shape);
    const ModelEntry* entry = doc.project().findModel(shape.modelId);
    return entry->placement.toParent() * cad::datumGeometry(*model, shape.type, index).point;
}

geom3d::Aabb3 originBox(const Document& doc, const Setup& setup, OriginFrom from) {
    const Mat3 axes = axesOf(setup);
    geom3d::Aabb3 box;
    auto add = [&](const Vec3& projectPoint) {
        box.extend(axes.transpose() * projectPoint);
    };
    switch (from) {
    case OriginFrom::StockBox: {
        const auto stock = doc.stockShape(setup.id);
        if (!stock) {
            throw userError("the setup's stock cannot be resolved");
        }
        if (stock->kind == StockShape::Kind::Box) {
            for (const Vec3& c : stock->corners()) {
                add(c);
            }
        } else {
            // A cylinder's exact extent along each axis.
            const Vec3 a = stock->axis.normalized();
            Vec3 lo, hi;
            for (int k = 0; k < 3; ++k) {
                const Vec3 u = axes.col(k);
                const double c = a.dot(u);
                const double r = 0.5 * stock->diameter * std::sqrt(std::max(0.0, 1.0 - c * c));
                const double s0 = stock->base.dot(u);
                const double s1 = s0 + stock->length * c;
                lo[k] = std::min(s0, s1) - r;
                hi[k] = std::max(s0, s1) + r;
            }
            box.extend(lo);
            box.extend(hi);
        }
        break;
    }
    case OriginFrom::PartBox: {
        const auto b = setup.modelId ? doc.modelBounds(*setup.modelId, axes) : std::nullopt;
        if (!b || b->isEmpty()) {
            throw userError("the setup has no part model");
        }
        box = *b;
        break;
    }
    case OriginFrom::FixtureBox: {
        for (const Uuid& id : setup.fixtures) {
            if (const auto b = doc.modelBounds(id, axes); b && !b->isEmpty()) {
                box.extend(*b);
            }
        }
        if (const auto pose = setupWorkholdingPose(doc, setup)) {
            const Transform3 wcsToProject = setup.wcs.toParent();
            for (const PosedPart& part : pose->parts) {
                const cad::Model* m = posedPartModel(doc, part);
                if (!m) {
                    continue;
                }
                const geom3d::Aabb3 local = m->info().bounds;
                if (local.isEmpty()) {
                    continue;
                }
                for (int c = 0; c < 8; ++c) {
                    add(wcsToProject * part.toWcs *
                        Vec3((c & 1) ? local.max.x() : local.min.x(),
                             (c & 2) ? local.max.y() : local.min.y(),
                             (c & 4) ? local.max.z() : local.min.z()));
                }
            }
        }
        if (box.isEmpty()) {
            throw userError("the setup has no fixtures or workholding to measure from");
        }
        break;
    }
    case OriginFrom::Point:
        throw userError("a picked point has no box");
    }
    return box;
}

Vec3 originReference(const Document& doc, const Setup& setup, const WcsOrigin& origin) {
    if (origin.from == OriginFrom::Point) {
        if (origin.point) {
            return pointOfShape(doc, *origin.point);
        }
        if (origin.workholdingPoint) {
            return workholdingPoint(doc, setup, *origin.workholdingPoint).point;
        }
        throw userError("pick the point the WCS origin is at");
    }
    const geom3d::Aabb3 box = originBox(doc, setup, origin.from);
    const Mat3 axes = axesOf(setup);
    Vec3 along;
    for (int k = 0; k < 3; ++k) {
        const int at = origin.box[static_cast<std::size_t>(k)];
        along[k] = at <= 0 ? box.min[k] : at >= 2 ? box.max[k] : 0.5 * (box.min[k] + box.max[k]);
    }
    return axes * along;
}

std::optional<Vec3> resolveWcsOrigin(const Document& doc, const Setup& setup) {
    if (!setup.originFrom) {
        return std::nullopt;
    }
    return originReference(doc, setup, *setup.originFrom) +
           axesOf(setup) * setup.originFrom->offset;
}

} // namespace ocam::document
