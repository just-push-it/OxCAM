// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The model is tessellated and cut into thin slabs across the axis. In each slab the largest
// distance from the axis is found exactly on the facets clipped to the slab (the distance is
// convex, so the largest value is at a corner of the clipped facet). The radius at each slab
// boundary is the larger of its two slabs, so the outline drawn straight between boundaries
// contains every slab. A radial-error Douglas-Peucker pass then thins the points, and the
// tolerance is added back so nothing of the model pokes out.

#include "ocam/cad/Revolve.hpp"

#include "ocam/cad/Tessellation.hpp"
#include "ocam/cad/occt/ShapeAccess.hpp"

#include <ocam/foundation/Error.hpp>

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <algorithm>
#include <cmath>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <vector>

namespace ocam::cad {
namespace {

/// The part of a polygon (3D points) with coordinate `axis` in [lo, hi].
std::vector<Vec3> clipToSlab(std::vector<Vec3> poly, int axis, double lo, double hi) {
    auto clip = [&](double bound, bool keepAbove) {
        std::vector<Vec3> out;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const Vec3& a = poly[i];
            const Vec3& b = poly[(i + 1) % poly.size()];
            const double da = keepAbove ? a[axis] - bound : bound - a[axis];
            const double db = keepAbove ? b[axis] - bound : bound - b[axis];
            if (da >= 0.0) {
                out.push_back(a);
            }
            if ((da >= 0.0) != (db >= 0.0)) {
                out.push_back(a + (b - a) * (da / (da - db)));
            }
        }
        poly = std::move(out);
    };
    clip(lo, true);
    if (!poly.empty()) {
        clip(hi, false);
    }
    return poly;
}

/// Douglas-Peucker on a function r(h) (heights strictly increasing): keeps the points needed
/// so the straight line between kept points is within `tol` of every dropped point in r.
void simplify(const std::vector<Vec2>& pts, std::size_t first, std::size_t last, double tol,
              std::vector<bool>& keep) {
    if (last <= first + 1) {
        return;
    }
    const Vec2& a = pts[first];
    const Vec2& b = pts[last];
    double worst = -1.0;
    std::size_t at = first;
    for (std::size_t k = first + 1; k < last; ++k) {
        const double t = (pts[k].y() - a.y()) / (b.y() - a.y());
        const double err = std::abs(pts[k].x() - (a.x() + t * (b.x() - a.x())));
        if (err > worst) {
            worst = err;
            at = k;
        }
    }
    if (worst > tol) {
        keep[at] = true;
        simplify(pts, first, at, tol, keep);
        simplify(pts, at, last, tol, keep);
    }
}

} // namespace

HolderOutline holderOutline(const Model& model, const HolderOutlineOptions& options) {
    if (model.isNull()) {
        throw Error(ErrorCode::InvalidArgument, "no holder model");
    }
    const double tol = std::max(options.tolerance, 1e-4);
    const geom3d::Mesh mesh = tessellate(model, {0.5 * tol, 0.2});
    if (mesh.triangles.empty()) {
        throw Error(ErrorCode::InvalidArgument, "the holder model has no faces");
    }
    const geom3d::Aabb3 box = mesh.bounds();
    const Vec3 size = box.size();
    const Vec3 centre = box.center();

    HolderOutline out;
    auto asymmetryOf = [&](int a) {
        const double p = size[(a + 1) % 3];
        const double q = size[(a + 2) % 3];
        return std::abs(p - q) / std::max({p, q, 1e-9});
    };
    int axis = 2;
    switch (options.axis) {
    case RevolveAxis::X:
        axis = 0;
        break;
    case RevolveAxis::Y:
        axis = 1;
        break;
    case RevolveAxis::Z:
        axis = 2;
        break;
    case RevolveAxis::Auto:
        for (int a = 0; a < 3; ++a) {
            const double da = asymmetryOf(a);
            const double best = asymmetryOf(axis);
            if (da < best - 1e-3 || (da < best + 1e-3 && size[a] > size[axis])) {
                axis = a;
            }
        }
        break;
    }
    out.axis = axis == 0 ? RevolveAxis::X : axis == 1 ? RevolveAxis::Y : RevolveAxis::Z;
    out.asymmetry = asymmetryOf(axis);
    const int u = (axis + 1) % 3;
    const int v = (axis + 2) % 3;
    const double lo = box.min[axis];
    const double L = size[axis];
    out.fullLength = L;
    if (L <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the holder model has no length along its axis");
    }

    // Largest distance from the axis per slab, measured from the low end.
    const int slabs =
        std::clamp(static_cast<int>(std::ceil(L / std::max(0.1, L / 600.0))), 8, 1200);
    const double dh = L / slabs;
    std::vector<double> radius(static_cast<std::size_t>(slabs), 0.0);
    auto radial = [&](const Vec3& p) {
        return std::hypot(p[u] - centre[u], p[v] - centre[v]);
    };
    for (const auto& t : mesh.triangles) {
        const std::vector<Vec3> tri{mesh.vertices[t[0]], mesh.vertices[t[1]], mesh.vertices[t[2]]};
        double h0 = tri[0][axis];
        double h1 = h0;
        for (const Vec3& p : tri) {
            h0 = std::min(h0, p[axis]);
            h1 = std::max(h1, p[axis]);
        }
        const int k0 = std::clamp(static_cast<int>((h0 - lo) / dh), 0, slabs - 1);
        const int k1 = std::clamp(static_cast<int>((h1 - lo) / dh), 0, slabs - 1);
        for (int k = k0; k <= k1; ++k) {
            const auto piece =
                k0 == k1 ? tri : clipToSlab(tri, axis, lo + k * dh, lo + (k + 1) * dh);
            double& r = radius[static_cast<std::size_t>(k)];
            for (const Vec3& p : piece) {
                r = std::max(r, radial(p));
            }
        }
    }

    // The flange: the widest slabs. The tool comes out of the end farther from it, and its top
    // (towards the spindle) is the usual gauge line.
    const double widest = *std::max_element(radius.begin(), radius.end());
    const double flangeBand = std::max(0.5, 0.02 * widest);
    double flangeSum = 0.0;
    int flangeCount = 0;
    int flangeLow = slabs;
    int flangeHigh = -1;
    for (int k = 0; k < slabs; ++k) {
        if (radius[static_cast<std::size_t>(k)] >= widest - flangeBand) {
            flangeSum += (k + 0.5) * dh;
            ++flangeCount;
            flangeLow = std::min(flangeLow, k);
            flangeHigh = std::max(flangeHigh, k);
        }
    }
    const double flangeCentre = flangeSum / std::max(flangeCount, 1);
    const bool toolAtLow =
        options.toolEnd == ToolEnd::Auto ? flangeCentre > 0.5 * L : options.toolEnd == ToolEnd::Low;
    out.toolEnd = toolAtLow ? ToolEnd::Low : ToolEnd::High;
    out.suggestedLength = toolAtLow ? (flangeHigh + 1) * dh : L - flangeLow * dh;
    out.length = options.length > 0.0 ? std::min(options.length, L) : out.suggestedLength;

    // Radii at the slab boundaries, heights from the tool end.
    std::vector<Vec2> pts; // (radius, height)
    for (int b = 0; b <= slabs; ++b) {
        const double below = b > 0 ? radius[static_cast<std::size_t>(b - 1)] : 0.0;
        const double above = b < slabs ? radius[static_cast<std::size_t>(b)] : 0.0;
        const double r = b == 0 ? above : b == slabs ? below : std::max(below, above);
        pts.emplace_back(r, toolAtLow ? b * dh : L - b * dh);
    }
    if (!toolAtLow) {
        std::reverse(pts.begin(), pts.end());
    }
    // Cut at the gauge line.
    std::vector<Vec2> kept;
    for (std::size_t k = 0; k < pts.size(); ++k) {
        if (pts[k].y() < out.length - 1e-9) {
            kept.push_back(pts[k]);
        } else {
            const Vec2& a = pts[k - 1];
            const Vec2& b = pts[k];
            const double t = (out.length - a.y()) / (b.y() - a.y());
            kept.emplace_back(a.x() + t * (b.x() - a.x()), out.length);
            break;
        }
    }

    std::vector<bool> keep(kept.size(), false);
    keep.front() = true;
    keep.back() = true;
    simplify(kept, 0, kept.size() - 1, tol, keep);
    std::vector<Vec2> outline{{0.0, 0.0}};
    for (std::size_t k = 0; k < kept.size(); ++k) {
        if (keep[k]) {
            outline.emplace_back(kept[k].x() + tol, kept[k].y());
        }
    }
    outline.emplace_back(0.0, out.length);
    out.profile = geom2d::Polyline::fromPoints(outline, false);
    return out;
}

Model revolvedAboutZ(const std::vector<Vec2>& profile) {
    BRepBuilderAPI_MakePolygon polygon;
    for (const Vec2& q : profile) {
        polygon.Add(gp_Pnt(q.y(), 0.0, q.x()));
    }
    polygon.Close();
    BRepBuilderAPI_MakeFace face(polygon.Wire(), true);
    BRepPrimAPI_MakeRevol solid(face.Shape(), gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)));
    return modelFromShape(solid.Shape());
}

} // namespace ocam::cad
