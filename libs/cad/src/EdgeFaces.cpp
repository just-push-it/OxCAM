// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/cad/EdgeFaces.hpp"

#include "ModelImpl.hpp"

#include <ocam/foundation/Error.hpp>

#include <fmt/format.h>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <Geom2d_Curve.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <algorithm>
#include <cmath>
#include <gp_Pnt2d.hxx>

namespace ocam::cad {
namespace {

Vec3 toVec(const gp_Vec& v) {
    return {v.X(), v.Y(), v.Z()};
}

/// The face's outward normal and its parameters at the edge's parameter t.
struct OnFace {
    Vec3 normal = Vec3::Zero();
    gp_Pnt2d uv;
    Vec3 du = Vec3::Zero();
    Vec3 dv = Vec3::Zero();
};

std::optional<OnFace> onFace(const TopoDS_Edge& edge, const TopoDS_Face& face,
                             const BRepAdaptor_Surface& surface, double t) {
    double f0 = 0.0;
    double f1 = 0.0;
    const Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, f0, f1);
    if (pcurve.IsNull()) {
        return std::nullopt;
    }
    OnFace out;
    out.uv = pcurve->Value(t);
    gp_Pnt p;
    gp_Vec du;
    gp_Vec dv;
    surface.D1(out.uv.X(), out.uv.Y(), p, du, dv);
    out.du = toVec(du);
    out.dv = toVec(dv);
    Vec3 n = out.du.cross(out.dv);
    if (n.norm() < 1e-12) {
        return std::nullopt; // a singular point (a cone's apex)
    }
    n.normalize();
    out.normal = face.Orientation() == TopAbs_REVERSED ? Vec3(-n) : n;
    return out;
}

/// True when a small step from the face point along `dir` (a 3D direction) stays on the face.
bool stepsIntoFace(const TopoDS_Face& face, const OnFace& at, const Vec3& dir) {
    // The step in the face's parameters that goes along dir (least squares on du, dv).
    const double a11 = at.du.dot(at.du);
    const double a12 = at.du.dot(at.dv);
    const double a22 = at.dv.dot(at.dv);
    const double b1 = at.du.dot(dir);
    const double b2 = at.dv.dot(dir);
    const double det = a11 * a22 - a12 * a12;
    if (std::abs(det) < 1e-18) {
        return false;
    }
    const double su = (a22 * b1 - a12 * b2) / det;
    const double sv = (a11 * b2 - a12 * b1) / det;
    const double len3d = (at.du * su + at.dv * sv).norm();
    if (len3d < 1e-12) {
        return false;
    }
    for (const double step : {0.01, 0.05, 0.2}) {
        const double k = step / len3d;
        const gp_Pnt2d q(at.uv.X() + su * k, at.uv.Y() + sv * k);
        const BRepClass_FaceClassifier classifier(face, q, 1e-7);
        if (classifier.State() == TopAbs_IN) {
            return true;
        }
        if (classifier.State() == TopAbs_OUT) {
            return false;
        }
    }
    return false;
}

} // namespace

EdgeFaces edgeFaces(const Model& model, int edgeIndex, double spacing) {
    const TopoDS_Shape& shape = model.impl().shape;
    ShapeIndexMap edges;
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    if (edgeIndex < 1 || edgeIndex > edges.Extent()) {
        throw Error(ErrorCode::InvalidArgument, fmt::format("no edge {}", edgeIndex));
    }
    ShapeIndexMap faceMap;
    TopExp::MapShapes(shape, TopAbs_FACE, faceMap);
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>,
                               TopTools_ShapeMapHasher>
        ancestors;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, ancestors);

    EdgeFaces out;
    const TopoDS_Edge& edge = TopoDS::Edge(edges(edgeIndex));
    if (BRep_Tool::Degenerated(edge)) {
        out.problem = "a degenerate edge";
        return out;
    }
    std::vector<TopoDS_Face> faces;
    if (const int at = ancestors.FindIndex(edge); at > 0) {
        for (const TopoDS_Shape& f : ancestors(at)) {
            if (std::none_of(faces.begin(), faces.end(),
                             [&](const TopoDS_Face& g) { return g.IsSame(f); })) {
                faces.push_back(TopoDS::Face(f));
            }
        }
    }
    if (faces.size() != 2) {
        out.problem = faces.size() == 1 ? "it bounds one face only (a seam or a free edge)"
                                        : fmt::format("it is shared by {} faces", faces.size());
        return out;
    }
    for (int k = 0; k < 2; ++k) {
        out.faces[static_cast<std::size_t>(k)] =
            faceMap.FindIndex(faces[static_cast<std::size_t>(k)]);
    }
    TopoDS_Vertex v0;
    TopoDS_Vertex v1;
    TopExp::Vertices(edge, v0, v1);
    out.closed = !v0.IsNull() && v0.IsSame(v1);

    const BRepAdaptor_Curve curve(edge);
    const double length = GCPnts_AbscissaPoint::Length(curve);
    const int segments =
        std::max(out.closed ? 8 : 1, static_cast<int>(std::ceil(length / std::max(spacing, 1e-3))));
    const GCPnts_UniformAbscissa uniform(curve, segments + 1);
    std::vector<double> params;
    if (uniform.IsDone() && uniform.NbPoints() == segments + 1) {
        for (int k = 1; k <= uniform.NbPoints(); ++k) {
            params.push_back(uniform.Parameter(k));
        }
    } else {
        const double t0 = curve.FirstParameter();
        const double t1 = curve.LastParameter();
        for (int k = 0; k <= segments; ++k) {
            params.push_back(t0 + (t1 - t0) * k / segments);
        }
    }
    if (out.closed) {
        params.pop_back(); // the first point again
    }
    const std::array<BRepAdaptor_Surface, 2> surfaces{BRepAdaptor_Surface(faces[0]),
                                                      BRepAdaptor_Surface(faces[1])};
    // Which way each face lies from the edge: found at the middle, kept continuous along it.
    std::array<double, 2> sense{0.0, 0.0};
    const std::size_t mid = params.size() / 2;
    for (std::size_t k = 0; k < 2; ++k) {
        gp_Pnt p;
        gp_Vec d;
        curve.D1(params[mid], p, d);
        const auto at = onFace(edge, faces[k], surfaces[k], params[mid]);
        if (!at || d.Magnitude() < 1e-12) {
            out.problem = "its faces have no normal on it";
            return out;
        }
        const Vec3 t = toVec(d).normalized();
        const Vec3 across = at->normal.cross(t).normalized();
        sense[k] = stepsIntoFace(faces[k], *at, across)    ? 1.0
                   : stepsIntoFace(faces[k], *at, -across) ? -1.0
                                                           : 0.0;
        if (sense[k] == 0.0) {
            out.problem = "which side its faces lie on cannot be told";
            return out;
        }
    }
    std::array<Vec3, 2> previous{Vec3::Zero(), Vec3::Zero()};
    for (const double t : params) {
        gp_Pnt p;
        gp_Vec d;
        curve.D1(t, p, d);
        if (d.Magnitude() < 1e-12) {
            continue;
        }
        EdgeFacePoint e;
        e.point = Vec3(p.X(), p.Y(), p.Z());
        e.tangent = toVec(d).normalized();
        bool ok = true;
        for (std::size_t k = 0; k < 2; ++k) {
            const auto at = onFace(edge, faces[k], surfaces[k], t);
            if (!at) {
                ok = false;
                break;
            }
            e.normal[k] = at->normal;
            Vec3 across = (sense[k] * at->normal.cross(e.tangent)).normalized();
            if (!previous[k].isZero() && across.dot(previous[k]) < 0.0) {
                across = -across;
            }
            e.inFace[k] = across;
            previous[k] = across;
        }
        if (ok) {
            out.points.push_back(e);
        }
    }
    if (out.points.size() < 2) {
        out.problem = "its faces have no normal on it";
    }
    return out;
}

} // namespace ocam::cad
