// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/cad/Features.hpp"

#include "ModelImpl.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepTools.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <algorithm>
#include <cmath>
#include <optional>

namespace ocam::cad {
namespace {

Vec3 toVec(const gp_Pnt& p) {
    return {p.X(), p.Y(), p.Z()};
}
Vec3 toVec(const gp_Dir& d) {
    return {d.X(), d.Y(), d.Z()};
}

// Outward (material-side-out) unit normal of a face at the middle of its UV bounds.
std::optional<std::pair<Vec3, Vec3>> midPointAndNormal(const TopoDS_Face& face,
                                                       const BRepAdaptor_Surface& surf) {
    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    BRepLProp_SLProps props(surf, 0.5 * (u0 + u1), 0.5 * (v0 + v1), 1, 1e-9);
    if (!props.IsNormalDefined()) {
        return std::nullopt;
    }
    gp_Dir n = props.Normal();
    if (face.Orientation() == TopAbs_REVERSED) {
        n.Reverse();
    }
    return std::make_pair(toVec(props.Value()), toVec(n));
}

// Extent of a face along `axis`, and its radial distances from the line (center, axis) at the
// extreme positions, sampled on the UV-bounds grid.
struct AxialExtent {
    double lo = 0, hi = 0;
    double radiusAtLo = 0, radiusAtHi = 0;
};

AxialExtent axialExtent(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                        const Vec3& center, const Vec3& axis) {
    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    AxialExtent e{std::numeric_limits<double>::infinity(),
                  -std::numeric_limits<double>::infinity()};
    constexpr int kSteps = 4;
    for (int i = 0; i <= kSteps; ++i) {
        for (int j = 0; j <= kSteps; ++j) {
            const Vec3 p =
                toVec(surf.Value(u0 + (u1 - u0) * i / kSteps, v0 + (v1 - v0) * j / kSteps));
            const double t = (p - center).dot(axis);
            const double r = ((p - center) - t * axis).norm();
            if (t < e.lo) {
                e.lo = t;
                e.radiusAtLo = r;
            }
            if (t > e.hi) {
                e.hi = t;
                e.radiusAtHi = r;
            }
        }
    }
    return e;
}

struct CylinderGroup {
    Vec3 center;   // axis line point in the plane through the origin perpendicular to `axis`
    double radius; // [mm]
    double lo, hi; // extent along axis
    std::vector<int> faces;
};

/// Cylinders parallel to `axis`, grouped by axis line and radius: concave ones (holes) or convex
/// ones (bosses).
std::vector<CylinderGroup> cylinderGroups(const ShapeIndexMap& faces, const Vec3& axis, bool convex,
                                          double cosTol, double lin) {
    auto projectLine = [&](const Vec3& p) {
        return Vec3(p - p.dot(axis) * axis);
    };
    std::vector<CylinderGroup> groups;
    for (int i = 1; i <= faces.Extent(); ++i) {
        const TopoDS_Face& face = TopoDS::Face(faces(i));
        BRepAdaptor_Surface surf(face, true);
        if (surf.GetType() != GeomAbs_Cylinder) {
            continue;
        }
        const gp_Cylinder cyl = surf.Cylinder();
        const Vec3 dir = toVec(cyl.Axis().Direction());
        if (std::abs(dir.dot(axis)) < cosTol) {
            continue;
        }
        const Vec3 loc = toVec(cyl.Axis().Location());
        const auto pn = midPointAndNormal(face, surf);
        if (!pn) {
            continue;
        }
        const auto& [p, n] = *pn;
        const Vec3 radial = projectLine(p) - projectLine(loc);
        // A normal pointing away from the axis is a boss's; towards it, a hole's.
        if ((n.dot(radial) >= 0.0) != convex) {
            continue;
        }
        const Vec3 center = projectLine(loc);
        const AxialExtent ext = axialExtent(face, surf, center, axis);
        const double radius = cyl.Radius();

        auto it = std::find_if(groups.begin(), groups.end(), [&](const CylinderGroup& g) {
            return (g.center - center).norm() <= lin && std::abs(g.radius - radius) <= lin;
        });
        if (it == groups.end()) {
            groups.push_back({center, radius, ext.lo, ext.hi, {i}});
        } else {
            it->lo = std::min(it->lo, ext.lo);
            it->hi = std::max(it->hi, ext.hi);
            it->faces.push_back(i);
        }
    }
    return groups;
}

} // namespace

std::vector<Hole> findHoles(const Model& model, const Vec3& axisIn, const Tolerance& tol) {
    const Vec3 axis = axisIn.normalized();
    const double cosTol = std::cos(tol.angular);
    const double lin = std::max(tol.linear, 1e-5);

    ShapeIndexMap faces;
    TopExp::MapShapes(model.impl().shape, TopAbs_FACE, faces);

    auto projectLine = [&](const Vec3& p) {
        return Vec3(p - p.dot(axis) * axis);
    };

    // 1. Concave cylinders parallel to the axis, grouped by axis line and radius.
    const std::vector<CylinderGroup> groups = cylinderGroups(faces, axis, false, cosTol, lin);

    // 2. For each group, look for a bottom closing either end.
    struct Closure {
        HoleBottom kind = HoleBottom::Through;
        double tipAngle = 0;
    };
    auto closureAt = [&](const CylinderGroup& g, double endT, bool atHighEnd) -> Closure {
        // Direction pointing from this end back into the hole.
        const Vec3 intoHole = atHighEnd ? Vec3(-axis) : axis;
        for (int i = 1; i <= faces.Extent(); ++i) {
            const TopoDS_Face& face = TopoDS::Face(faces(i));
            BRepAdaptor_Surface surf(face, true);
            if (surf.GetType() == GeomAbs_Cone) {
                const gp_Cone cone = surf.Cone();
                const Vec3 dir = toVec(cone.Axis().Direction());
                if (std::abs(dir.dot(axis)) < cosTol ||
                    (projectLine(toVec(cone.Axis().Location())) - g.center).norm() > lin) {
                    continue;
                }
                const AxialExtent e = axialExtent(face, surf, g.center, axis);
                // Wide end meets the cylinder at endT; narrow end is a point beyond it.
                const bool meetsLow = !atHighEnd && std::abs(e.hi - endT) <= lin &&
                                      std::abs(e.radiusAtHi - g.radius) <= lin &&
                                      e.radiusAtLo <= lin;
                const bool meetsHigh = atHighEnd && std::abs(e.lo - endT) <= lin &&
                                       std::abs(e.radiusAtLo - g.radius) <= lin &&
                                       e.radiusAtHi <= lin;
                if (meetsLow || meetsHigh) {
                    return {HoleBottom::Cone, 2.0 * std::abs(cone.SemiAngle())};
                }
            } else if (surf.GetType() == GeomAbs_Plane) {
                const auto pn = midPointAndNormal(face, surf);
                if (!pn || pn->second.dot(intoHole) < cosTol) {
                    continue;
                }
                const AxialExtent e = axialExtent(face, surf, g.center, axis);
                if (std::abs(e.lo - endT) > lin || std::abs(e.hi - endT) > lin) {
                    continue;
                }
                // The face must fit inside the hole's circle, which rules out faces that continue
                // outward (such as the top face around a hole). The UV bounds of a face are its
                // bounding square in the plane, whose corners reach r*sqrt(2) for a disc of
                // radius r centred on the axis.
                double maxR = 0;
                double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
                BRepTools::UVBounds(face, u0, u1, v0, v1);
                for (double u : {u0, u1}) {
                    for (double v : {v0, v1}) {
                        maxR = std::max(maxR,
                                        (projectLine(toVec(surf.Value(u, v))) - g.center).norm());
                    }
                }
                if (maxR <= g.radius * std::sqrt(2.0) + lin) {
                    return {HoleBottom::Flat, 0.0};
                }
            }
        }
        return {};
    };

    std::vector<Hole> holes;
    for (const auto& g : groups) {
        const Closure low = closureAt(g, g.lo, false);
        const Closure high = closureAt(g, g.hi, true);
        if (low.kind != HoleBottom::Through && high.kind != HoleBottom::Through) {
            continue; // closed at both ends: an internal void, not a machinable hole
        }
        Hole h;
        h.diameter = 2.0 * g.radius;
        h.depth = g.hi - g.lo;
        h.faces = g.faces;
        if (high.kind != HoleBottom::Through) {
            h.entry = g.center + g.lo * axis;
            h.direction = axis;
            h.bottom = high.kind;
            h.tipAngle = high.tipAngle;
        } else {
            h.entry = g.center + g.hi * axis;
            h.direction = -axis;
            h.bottom = low.kind;
            h.tipAngle = low.tipAngle;
        }
        holes.push_back(std::move(h));
    }

    std::sort(holes.begin(), holes.end(), [](const Hole& a, const Hole& b) {
        return std::tie(a.entry.x(), a.entry.y(), a.entry.z()) <
               std::tie(b.entry.x(), b.entry.y(), b.entry.z());
    });
    return holes;
}

std::vector<Boss> findBosses(const Model& model, const Vec3& axisIn, const Tolerance& tol) {
    const Vec3 axis = axisIn.normalized();
    const double cosTol = std::cos(tol.angular);
    const double lin = std::max(tol.linear, 1e-5);

    ShapeIndexMap faces;
    TopExp::MapShapes(model.impl().shape, TopAbs_FACE, faces);
    auto projectLine = [&](const Vec3& p) {
        return Vec3(p - p.dot(axis) * axis);
    };

    // A shoulder facing -axis at the high end: the part goes on outward above the cylinder, so
    // the boss hangs from it and its free end is the low one.
    auto hangsAt = [&](const CylinderGroup& g) {
        for (int i = 1; i <= faces.Extent(); ++i) {
            const TopoDS_Face& face = TopoDS::Face(faces(i));
            BRepAdaptor_Surface surf(face, true);
            if (surf.GetType() != GeomAbs_Plane) {
                continue;
            }
            const auto pn = midPointAndNormal(face, surf);
            if (!pn || pn->second.dot(-axis) < cosTol) {
                continue;
            }
            const AxialExtent e = axialExtent(face, surf, g.center, axis);
            if (std::abs(e.lo - g.hi) > lin || std::abs(e.hi - g.hi) > lin) {
                continue;
            }
            // Reaching past the circle's bounding square: not a cap on the cylinder's end.
            double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
            BRepTools::UVBounds(face, u0, u1, v0, v1);
            for (double u : {u0, u1}) {
                for (double v : {v0, v1}) {
                    if ((projectLine(toVec(surf.Value(u, v))) - g.center).norm() >
                        g.radius * std::sqrt(2.0) + lin) {
                        return true;
                    }
                }
            }
        }
        return false;
    };

    std::vector<Boss> bosses;
    for (const CylinderGroup& g : cylinderGroups(faces, axis, true, cosTol, lin)) {
        Boss b;
        b.diameter = 2.0 * g.radius;
        b.length = g.hi - g.lo;
        b.faces = g.faces;
        if (hangsAt(g)) {
            b.entry = g.center + g.lo * axis;
            b.direction = axis;
        } else {
            b.entry = g.center + g.hi * axis;
            b.direction = -axis;
        }
        bosses.push_back(std::move(b));
    }
    std::sort(bosses.begin(), bosses.end(), [](const Boss& a, const Boss& b) {
        return std::tie(a.entry.x(), a.entry.y(), a.entry.z()) <
               std::tie(b.entry.x(), b.entry.y(), b.entry.z());
    });
    return bosses;
}

std::optional<FacePlane> facePlane(const Model& model, int faceIndex) {
    ShapeIndexMap faces;
    TopExp::MapShapes(model.impl().shape, TopAbs_FACE, faces);
    if (faceIndex < 1 || faceIndex > faces.Extent()) {
        return std::nullopt;
    }
    const TopoDS_Face& face = TopoDS::Face(faces(faceIndex));
    BRepAdaptor_Surface surf(face, true);
    if (surf.GetType() != GeomAbs_Plane) {
        return std::nullopt;
    }
    Vec3 n = toVec(surf.Plane().Axis().Direction());
    if (face.Orientation() == TopAbs_REVERSED) {
        n = -n;
    }
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    return FacePlane{toVec(props.CentreOfMass()), n.normalized()};
}

std::vector<PlanarFace> findPlanarFaces(const Model& model, const Vec3& axisIn,
                                        const Tolerance& tol) {
    const Vec3 axis = axisIn.normalized();
    const double cosTol = std::cos(tol.angular);

    ShapeIndexMap faces;
    TopExp::MapShapes(model.impl().shape, TopAbs_FACE, faces);

    std::vector<PlanarFace> result;
    for (int i = 1; i <= faces.Extent(); ++i) {
        const TopoDS_Face& face = TopoDS::Face(faces(i));
        BRepAdaptor_Surface surf(face, true);
        if (surf.GetType() != GeomAbs_Plane) {
            continue;
        }
        const gp_Pln pln = surf.Plane();
        Vec3 n = toVec(pln.Axis().Direction());
        if (face.Orientation() == TopAbs_REVERSED) {
            n = -n;
        }
        const double c = n.dot(axis);
        if (std::abs(c) < cosTol) {
            continue;
        }
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        result.push_back({i, toVec(pln.Location()).dot(axis), props.Mass(), c > 0.0});
    }
    std::sort(result.begin(), result.end(), [](const PlanarFace& a, const PlanarFace& b) {
        return a.height != b.height ? a.height > b.height : a.face < b.face;
    });
    return result;
}

} // namespace ocam::cad
