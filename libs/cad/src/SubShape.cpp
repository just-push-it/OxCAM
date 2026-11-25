// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/cad/SubShape.hpp"

#include "ModelImpl.hpp"
#include "ocam/cad/occt/ShapeAccess.hpp"

#include <ocam/foundation/Error.hpp>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <algorithm>
#include <cmath>
#include <limits>

namespace ocam::cad {
namespace {

ShapeIndexMap mapOf(const Model& model, SubShapeType type) {
    ShapeIndexMap map;
    TopExp::MapShapes(model.impl().shape,
                      type == SubShapeType::Face   ? TopAbs_FACE
                      : type == SubShapeType::Edge ? TopAbs_EDGE
                                                   : TopAbs_VERTEX,
                      map);
    return map;
}

Vec3 toVec(const gp_Dir& d) {
    return {d.X(), d.Y(), d.Z()};
}

// Direction vectors are compared up to sign (an axis has no preferred sense).
bool directionsMatch(const Vec3& a, const Vec3& b, double tol) {
    if (a.isZero() || b.isZero()) {
        return a.isZero() && b.isZero();
    }
    return std::abs(std::abs(a.dot(b)) - 1.0) <= tol;
}

ShapeSignature signatureOfShape(const TopoDS_Shape& shape, SubShapeType type) {
    ShapeSignature sig;
    if (type == SubShapeType::Vertex) {
        const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(shape));
        sig.centroid = {p.X(), p.Y(), p.Z()};
        return sig;
    }
    GProp_GProps props;
    if (type == SubShapeType::Face) {
        const TopoDS_Face& face = TopoDS::Face(shape);
        BRepGProp::SurfaceProperties(face, props);
        BRepAdaptor_Surface surf(face, true);
        switch (surf.GetType()) {
        case GeomAbs_Plane:
            sig.direction = toVec(surf.Plane().Axis().Direction());
            break;
        case GeomAbs_Cylinder:
            sig.direction = toVec(surf.Cylinder().Axis().Direction());
            break;
        case GeomAbs_Cone:
            sig.direction = toVec(surf.Cone().Axis().Direction());
            break;
        default:
            break;
        }
    } else {
        const TopoDS_Edge& edge = TopoDS::Edge(shape);
        BRepGProp::LinearProperties(edge, props);
        BRepAdaptor_Curve curve(edge);
        if (curve.GetType() == GeomAbs_Line) {
            sig.direction = toVec(curve.Line().Direction());
        } else if (curve.GetType() == GeomAbs_Circle) {
            sig.direction = toVec(curve.Circle().Axis().Direction());
        }
    }
    const gp_Pnt c = props.CentreOfMass();
    sig.centroid = {c.X(), c.Y(), c.Z()};
    sig.measure = props.Mass();
    return sig;
}

// Lower is better; infinity when outside tolerance.
double mismatch(const ShapeSignature& a, const ShapeSignature& b, double tol) {
    const double distance = (a.centroid - b.centroid).norm();
    const double scale = std::max({std::abs(a.measure), std::abs(b.measure), 1.0});
    const double measureDiff = std::abs(a.measure - b.measure) / scale;
    if (distance > tol * std::max(1.0, std::sqrt(scale)) || measureDiff > tol ||
        !directionsMatch(a.direction, b.direction, tol)) {
        return std::numeric_limits<double>::infinity();
    }
    return distance + measureDiff;
}

} // namespace

SubShapeGeometry subShapeGeometry(const Model& model, SubShapeType type, int index) {
    const ShapeIndexMap map = mapOf(model, type);
    if (index < 1 || index > map.Extent()) {
        throw Error(ErrorCode::InvalidArgument, "sub-shape index out of range");
    }
    using Kind = SubShapeGeometry::Kind;
    SubShapeGeometry g;
    const TopoDS_Shape& shape = map(index);
    auto pt = [](const gp_Pnt& p) {
        return Vec3(p.X(), p.Y(), p.Z());
    };
    if (type == SubShapeType::Vertex) {
        g.point = pt(BRep_Tool::Pnt(TopoDS::Vertex(shape)));
        return g;
    }
    GProp_GProps props;
    if (type == SubShapeType::Face) {
        const TopoDS_Face& face = TopoDS::Face(shape);
        BRepGProp::SurfaceProperties(face, props);
        const Vec3 centroid = pt(props.CentreOfMass());
        g.point = centroid;
        BRepAdaptor_Surface surf(face, true);
        auto onAxis = [&](const gp_Ax1& axis) {
            const Vec3 o = pt(axis.Location());
            const Vec3 d = toVec(axis.Direction());
            g.direction = d;
            g.point = o + d.dot(centroid - o) * d;
        };
        switch (surf.GetType()) {
        case GeomAbs_Plane: {
            g.kind = Kind::Plane;
            Vec3 n = toVec(surf.Plane().Axis().Direction());
            g.direction = face.Orientation() == TopAbs_REVERSED ? Vec3(-n) : n;
            break;
        }
        case GeomAbs_Cylinder: {
            g.kind = Kind::Cylinder;
            const gp_Ax1 axis = surf.Cylinder().Axis();
            onAxis(axis);
            g.radius = surf.Cylinder().Radius();
            // V runs along the axis from its location.
            double u0 = 0.0, u1 = 0.0, v0 = 0.0, v1 = 0.0;
            BRepTools::UVBounds(face, u0, u1, v0, v1);
            const Vec3 o = pt(axis.Location());
            g.axisStart = o + g.direction * std::min(v0, v1);
            g.axisEnd = o + g.direction * std::max(v0, v1);
            break;
        }
        case GeomAbs_Cone: {
            g.kind = Kind::Cone;
            const gp_Cone cone = surf.Cone();
            onAxis(cone.Axis());
            // The radius where the axis point lies along the cone.
            const Vec3 apex = pt(cone.Apex());
            g.radius = (g.point - apex).norm() * std::tan(std::abs(cone.SemiAngle()));
            // V runs along a generatrix: cos(semi-angle) of it along the axis.
            double u0 = 0.0, u1 = 0.0, v0 = 0.0, v1 = 0.0;
            BRepTools::UVBounds(face, u0, u1, v0, v1);
            const Vec3 o = pt(cone.Axis().Location());
            const double c = std::cos(cone.SemiAngle());
            g.axisStart = o + g.direction * std::min(v0 * c, v1 * c);
            g.axisEnd = o + g.direction * std::max(v0 * c, v1 * c);
            break;
        }
        case GeomAbs_Sphere:
            g.kind = Kind::Sphere;
            g.point = pt(surf.Sphere().Location());
            g.radius = surf.Sphere().Radius();
            break;
        default:
            g.kind = Kind::Surface;
            break;
        }
        if (g.kind != Kind::Cylinder && g.kind != Kind::Cone) {
            g.axisStart = g.axisEnd = g.point;
        }
        return g;
    }
    const TopoDS_Edge& edge = TopoDS::Edge(shape);
    BRepGProp::LinearProperties(edge, props);
    g.point = pt(props.CentreOfMass());
    g.kind = Kind::Curve;
    if (BRep_Tool::Degenerated(edge)) {
        return g;
    }
    BRepAdaptor_Curve curve(edge);
    if (curve.GetType() == GeomAbs_Line) {
        g.kind = Kind::Line;
        g.direction = toVec(curve.Line().Direction());
        g.point = pt(curve.Value(0.5 * (curve.FirstParameter() + curve.LastParameter())));
    } else if (curve.GetType() == GeomAbs_Circle) {
        g.kind = Kind::Circle;
        g.point = pt(curve.Circle().Location());
        g.direction = toVec(curve.Circle().Axis().Direction());
        g.radius = curve.Circle().Radius();
    }
    return g;
}

int subShapeIndex(const Model& model, const TopoDS_Shape& subShape) {
    if (subShape.IsNull()) {
        return 0;
    }
    const TopAbs_ShapeEnum kind = subShape.ShapeType();
    if (kind != TopAbs_FACE && kind != TopAbs_EDGE && kind != TopAbs_VERTEX) {
        return 0;
    }
    ShapeIndexMap map;
    TopExp::MapShapes(model.impl().shape, kind, map);
    for (int i = 1; i <= map.Extent(); ++i) {
        if (map(i).TShape() == subShape.TShape()) {
            return i;
        }
    }
    return 0;
}

int subShapeCount(const Model& model, SubShapeType type) {
    return mapOf(model, type).Extent();
}

ShapeSignature signatureOf(const Model& model, SubShapeType type, int index) {
    const ShapeIndexMap map = mapOf(model, type);
    if (index < 1 || index > map.Extent()) {
        throw Error(ErrorCode::InvalidArgument, "sub-shape index out of range");
    }
    return signatureOfShape(map(index), type);
}

std::optional<int> matchSubShape(const Model& model, SubShapeType type,
                                 const ShapeSignature& signature, int hintIndex, double tolerance) {
    const ShapeIndexMap map = mapOf(model, type);
    if (hintIndex >= 1 && hintIndex <= map.Extent() &&
        std::isfinite(mismatch(signatureOfShape(map(hintIndex), type), signature, tolerance))) {
        return hintIndex;
    }
    std::optional<int> best;
    double bestScore = std::numeric_limits<double>::infinity();
    for (int i = 1; i <= map.Extent(); ++i) {
        const double score = mismatch(signatureOfShape(map(i), type), signature, tolerance);
        if (score < bestScore) {
            bestScore = score;
            best = i;
        }
    }
    return best;
}

std::vector<int> edgeLoopOf(const Model& model, int edgeIndex, const Vec3& axis) {
    const ShapeIndexMap edges = mapOf(model, SubShapeType::Edge);
    if (edgeIndex < 1 || edgeIndex > edges.Extent() || axis.isZero()) {
        return {edgeIndex};
    }
    const TopoDS_Shape& target = edges(edgeIndex);
    const Vec3 a = axis.normalized();
    for (TopExp_Explorer fx(model.impl().shape, TopAbs_FACE); fx.More(); fx.Next()) {
        const TopoDS_Face& face = TopoDS::Face(fx.Current());
        BRepAdaptor_Surface surf(face, true);
        if (surf.GetType() != GeomAbs_Plane ||
            std::abs(std::abs(toVec(surf.Plane().Axis().Direction()).dot(a)) - 1.0) > 1e-6) {
            continue;
        }
        for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
            std::vector<int> loop;
            bool contains = false;
            for (TopExp_Explorer ex(wx.Current(), TopAbs_EDGE); ex.More(); ex.Next()) {
                const int index = edges.FindIndex(ex.Current());
                if (index > 0 && std::find(loop.begin(), loop.end(), index) == loop.end()) {
                    loop.push_back(index);
                }
                contains = contains || ex.Current().IsSame(target);
            }
            if (contains) {
                return loop;
            }
        }
    }
    return {edgeIndex};
}

bool edgeAcross(const Model& model, int edgeIndex, const Vec3& axisIn, double tolerance) {
    const ShapeIndexMap edges = mapOf(model, SubShapeType::Edge);
    if (edgeIndex < 1 || edgeIndex > edges.Extent()) {
        return false;
    }
    const TopoDS_Edge& edge = TopoDS::Edge(edges(edgeIndex));
    if (BRep_Tool::Degenerated(edge)) {
        return false;
    }
    const Vec3 axis = axisIn.normalized();
    const BRepAdaptor_Curve curve(edge);
    const double t0 = curve.FirstParameter();
    const double t1 = curve.LastParameter();
    double lo = std::numeric_limits<double>::infinity();
    double hi = -lo;
    constexpr int kSamples = 16;
    for (int k = 0; k <= kSamples; ++k) {
        const gp_Pnt p = curve.Value(t0 + (t1 - t0) * k / kSamples);
        const double h = axis.dot(Vec3(p.X(), p.Y(), p.Z()));
        lo = std::min(lo, h);
        hi = std::max(hi, h);
    }
    return hi - lo <= tolerance;
}

bool containsPoint(const Model& model, const Vec3& point, double tolerance) {
    const gp_Pnt p(point.x(), point.y(), point.z());
    for (TopExp_Explorer ex(model.impl().shape, TopAbs_SOLID); ex.More(); ex.Next()) {
        BRepClass3d_SolidClassifier classifier(ex.Current(), p, tolerance);
        if (classifier.State() == TopAbs_IN) {
            return true;
        }
    }
    return false;
}

} // namespace ocam::cad
