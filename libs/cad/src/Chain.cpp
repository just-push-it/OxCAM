// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/cad/Chain.hpp"

#include "ModelImpl.hpp"

#include <ocam/foundation/Error.hpp>

#include <BRepAdaptor_Curve.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <algorithm>
#include <cmath>
#include <gp_Trsf.hxx>
#include <limits>

namespace ocam::cad {
namespace {

gp_Trsf trsfOf(const Transform3& toFrame) {
    const auto& m = toFrame.matrix();
    gp_Trsf trsf;
    trsf.SetValues(m(0, 0), m(0, 1), m(0, 2), m(0, 3), m(1, 0), m(1, 1), m(1, 2), m(1, 3), m(2, 0),
                   m(2, 1), m(2, 2), m(2, 3));
    return trsf;
}

Vec3 vec(const gp_Pnt& p) {
    return {p.X(), p.Y(), p.Z()};
}

/// The points of an edge in its own direction (as oriented), first and last included.
std::vector<Vec3> edgePoints(const TopoDS_Edge& edge, double tol) {
    BRepAdaptor_Curve curve(edge);
    const bool reversed = edge.Orientation() == TopAbs_REVERSED;
    const double first = curve.FirstParameter();
    const double last = curve.LastParameter();
    std::vector<Vec3> pts;
    GCPnts_QuasiUniformDeflection sampler(curve, tol, first, last);
    if (sampler.IsDone() && sampler.NbPoints() >= 2) {
        for (int i = 1; i <= sampler.NbPoints(); ++i) {
            pts.push_back(vec(sampler.Value(i)));
        }
    } else {
        pts = {vec(curve.Value(first)), vec(curve.Value(last))};
    }
    if (reversed) {
        std::reverse(pts.begin(), pts.end());
    }
    return pts;
}

/// Chains pieces (each a run of points) end to start, reversing them as needed.
std::vector<Chain3> chain(std::vector<std::vector<Vec3>> pieces, double joinTol) {
    std::vector<Chain3> out;
    std::vector<bool> used(pieces.size(), false);
    const auto near = [&](const Vec3& a, const Vec3& b) {
        return (a - b).norm() <= joinTol;
    };
    for (std::size_t first = 0; first < pieces.size(); ++first) {
        if (used[first] || pieces[first].size() < 2) {
            continue;
        }
        used[first] = true;
        std::vector<Vec3> pts = pieces[first];
        // Forward from the end, then backward from the start.
        for (const bool forward : {true, false}) {
            bool found = true;
            while (found && !(pts.size() > 2 && near(pts.front(), pts.back()))) {
                found = false;
                const Vec3 end = forward ? pts.back() : pts.front();
                for (std::size_t k = 0; k < pieces.size() && !found; ++k) {
                    if (used[k] || pieces[k].size() < 2) {
                        continue;
                    }
                    std::vector<Vec3> next = pieces[k];
                    if (near(forward ? next.back() : next.front(), end)) {
                        std::reverse(next.begin(), next.end());
                    } else if (!near(forward ? next.front() : next.back(), end)) {
                        continue;
                    }
                    if (forward) {
                        pts.insert(pts.end(), next.begin() + 1, next.end());
                    } else {
                        pts.insert(pts.begin(), next.begin(), next.end() - 1);
                    }
                    used[k] = true;
                    found = true;
                }
            }
        }
        Chain3 c;
        c.closed = pts.size() > 2 && near(pts.front(), pts.back());
        if (c.closed) {
            pts.pop_back();
        }
        c.points = std::move(pts);
        out.push_back(std::move(c));
    }
    return out;
}

} // namespace

std::vector<Chain3> edgeChains3d(const Model& model, const std::vector<int>& edgeIndices,
                                 const Transform3& toFrame, double curveTolerance) {
    ShapeIndexMap edges;
    TopExp::MapShapes(model.impl().shape, TopAbs_EDGE, edges);
    const gp_Trsf trsf = trsfOf(toFrame);
    std::vector<std::vector<Vec3>> pieces;
    for (const int index : edgeIndices) {
        if (index < 1 || index > edges.Extent()) {
            throw Error(ErrorCode::InvalidArgument, "edge index out of range");
        }
        const TopoDS_Edge edge =
            TopoDS::Edge(BRepBuilderAPI_Transform(edges(index), trsf, true).Shape());
        if (BRep_Tool::Degenerated(edge)) {
            continue;
        }
        pieces.push_back(edgePoints(edge, curveTolerance));
    }
    return chain(std::move(pieces), std::max(1e-4, 10.0 * curveTolerance));
}

std::vector<Chain3> faceChains(const Model& model, int faceIndex, const Transform3& toFrame,
                               double curveTolerance) {
    ShapeIndexMap faces;
    TopExp::MapShapes(model.impl().shape, TopAbs_FACE, faces);
    if (faceIndex < 1 || faceIndex > faces.Extent()) {
        throw Error(ErrorCode::InvalidArgument, "face index out of range");
    }
    const TopoDS_Face face =
        TopoDS::Face(BRepBuilderAPI_Transform(faces(faceIndex), trsfOf(toFrame), true).Shape());
    std::vector<std::vector<Vec3>> pieces;
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        // A seam lies on the face twice: where it meets itself, not where it ends.
        if (BRep_Tool::Degenerated(edge) || BRep_Tool::IsClosed(edge, face)) {
            continue;
        }
        pieces.push_back(edgePoints(edge, curveTolerance));
    }
    std::vector<Chain3> out = chain(std::move(pieces), std::max(1e-4, 10.0 * curveTolerance));
    std::erase_if(out, [](const Chain3& c) { return !c.closed; });
    return out;
}

double faceDistanceFromXAxis(const Model& model, int faceIndex, const Transform3& toFrame) {
    ShapeIndexMap faces;
    TopExp::MapShapes(model.impl().shape, TopAbs_FACE, faces);
    if (faceIndex < 1 || faceIndex > faces.Extent()) {
        throw Error(ErrorCode::InvalidArgument, "face index out of range");
    }
    const TopoDS_Face face =
        TopoDS::Face(BRepBuilderAPI_Transform(faces(faceIndex), trsfOf(toFrame), true).Shape());
    Bnd_Box box;
    BRepBndLib::Add(face, box);
    double x0 = 0.0, y0 = 0.0, z0 = 0.0, x1 = 0.0, y1 = 0.0, z1 = 0.0;
    box.Get(x0, y0, z0, x1, y1, z1);
    // The axis over the face's length and well past it.
    const double margin = 1.0 + (x1 - x0);
    const TopoDS_Edge axis =
        BRepBuilderAPI_MakeEdge(gp_Pnt(x0 - margin, 0.0, 0.0), gp_Pnt(x1 + margin, 0.0, 0.0));
    BRepExtrema_DistShapeShape dist(face, axis);
    if (!dist.IsDone()) {
        throw Error(ErrorCode::Internal, "the distance of a face from the axis failed");
    }
    return dist.Value();
}

} // namespace ocam::cad
