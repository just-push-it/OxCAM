// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/cad/Datum.hpp"

#include "ModelImpl.hpp"

#include <ocam/foundation/Error.hpp>

#include <BRepGProp_Face.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

namespace ocam::cad {
namespace {

Vec3 toVec(const gp_XYZ& v) {
    return {v.X(), v.Y(), v.Z()};
}

TopoDS_Shape subShape(const Model& model, SubShapeType type, int index) {
    ShapeIndexMap map;
    TopExp::MapShapes(model.impl().shape,
                      type == SubShapeType::Face   ? TopAbs_FACE
                      : type == SubShapeType::Edge ? TopAbs_EDGE
                                                   : TopAbs_VERTEX,
                      map);
    if (index < 1 || index > map.Extent()) {
        throw Error(ErrorCode::InvalidArgument, "sub-shape index out of range");
    }
    return map.FindKey(index);
}

/// The outward normal of a face at surface parameters (u, v).
Vec3 normalAt(const TopoDS_Face& face, double u, double v) {
    BRepGProp_Face prop(face);
    gp_Pnt p;
    gp_Vec n;
    prop.Normal(u, v, p, n); // oriented by the face's orientation
    return n.Magnitude() > 1e-12 ? Vec3(toVec(n.XYZ()).normalized()) : Vec3::Zero();
}

} // namespace

DatumGeometry datumGeometry(const Model& model, SubShapeType type, int index) {
    // The exact geometry, as a datum sees it.
    using Kind = SubShapeGeometry::Kind;
    const SubShapeGeometry g = subShapeGeometry(model, type, index);
    DatumGeometry out;
    out.point = g.point;
    switch (g.kind) {
    case Kind::Plane:
        out.shape = DatumGeometry::Shape::Plane;
        out.direction = g.direction;
        break;
    case Kind::Cylinder:
    case Kind::Cone:
    case Kind::Circle:
        out.shape = DatumGeometry::Shape::Axis;
        out.direction = g.direction;
        break;
    case Kind::Line:
        out.shape = DatumGeometry::Shape::Line;
        out.direction = g.direction;
        break;
    case Kind::Sphere:
    case Kind::Surface: {
        // Other faces: their centre of mass and the normal of the face point nearest it.
        out.point = signatureOf(model, type, index).centroid;
        out.direction = pointOnFace(model, index, out.point).direction;
        break;
    }
    case Kind::Curve:
    case Kind::Point:
        break;
    }
    return out;
}

DatumGeometry pointOnFace(const Model& model, int faceIndex, const Vec3& near) {
    const TopoDS_Face face = TopoDS::Face(subShape(model, SubShapeType::Face, faceIndex));
    DatumGeometry out;
    out.point = near;
    const occ::handle<Geom_Surface> surface = BRep_Tool::Surface(face);
    GeomAPI_ProjectPointOnSurf project(gp_Pnt(near.x(), near.y(), near.z()), surface);
    if (project.NbPoints() > 0) {
        double u = 0.0;
        double v = 0.0;
        project.LowerDistanceParameters(u, v);
        out.point = toVec(project.NearestPoint().XYZ());
        out.direction = normalAt(face, u, v);
    }
    return out;
}

} // namespace ocam::cad
