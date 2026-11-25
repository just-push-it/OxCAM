// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/cad/Section.hpp"

#include "ModelImpl.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/geom2d/Clip.hpp>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <NCollection_HSequence.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <algorithm>
#include <cmath>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>
#include <limits>

namespace ocam::cad {
namespace {

Vec2 xy(const gp_Pnt& p) {
    return {p.X(), p.Y()};
}

// Appends one edge, in traversal order, to `out`. Each appended vertex starts a segment; the
// edge's end point is not appended (it is the next edge's start).
void appendEdge(const TopoDS_Edge& edge, double tol, std::vector<geom2d::Vertex>& out) {
    BRepAdaptor_Curve curve(edge);
    const bool reversed = edge.Orientation() == TopAbs_REVERSED;
    const double first = curve.FirstParameter();
    const double last = curve.LastParameter();
    const double start = reversed ? last : first;
    const double end = reversed ? first : last;

    switch (curve.GetType()) {
    case GeomAbs_Line:
        out.push_back({xy(curve.Value(start)), 0.0});
        return;
    case GeomAbs_Circle: {
        const gp_Circ circ = curve.Circle();
        const gp_Dir axis = circ.Axis().Direction();
        if (std::abs(std::abs(axis.Z()) - 1.0) < 1e-9) {
            // Parameter increases counter-clockwise about the circle axis.
            const double sweep = (end - start) * (axis.Z() > 0.0 ? 1.0 : -1.0);
            // Split so every piece sweeps at most half a turn (bulge magnitude <= 1).
            const int pieces =
                std::max(1, static_cast<int>(std::ceil(std::abs(sweep) / kPi - 1e-9)));
            for (int k = 0; k < pieces; ++k) {
                const double t = start + (end - start) * k / pieces;
                out.push_back({xy(curve.Value(t)), geom2d::bulgeFromSweep(sweep / pieces)});
            }
            return;
        }
        break; // tilted circle: discretize
    }
    default:
        break;
    }

    GCPnts_QuasiUniformDeflection sampler(curve, tol, first, last);
    if (!sampler.IsDone() || sampler.NbPoints() < 2) {
        out.push_back({xy(curve.Value(start)), 0.0});
        return;
    }
    const int n = sampler.NbPoints();
    for (int i = 0; i < n - 1; ++i) {
        const int idx = reversed ? n - i : i + 1;
        out.push_back({xy(sampler.Value(idx)), 0.0});
    }
}

} // namespace

namespace {

geom2d::Regions sectionShapeAtZ(const TopoDS_Shape& shape, double z, double curveTolerance) {
    const gp_Pln plane(gp_Pnt(0.0, 0.0, z), gp_Dir(0.0, 0.0, 1.0));
    BRepAlgoAPI_Section section(shape, plane, false);
    section.ComputePCurveOn1(false);
    section.Approximation(false);
    section.Build();
    if (!section.IsDone()) {
        throw Error(ErrorCode::Internal, "planar section failed");
    }

    occ::handle<NCollection_HSequence<TopoDS_Shape>> edges =
        new NCollection_HSequence<TopoDS_Shape>();
    for (TopExp_Explorer ex(section.Shape(), TopAbs_EDGE); ex.More(); ex.Next()) {
        if (!BRep_Tool::Degenerated(TopoDS::Edge(ex.Current()))) {
            edges->Append(ex.Current());
        }
    }
    if (edges->IsEmpty()) {
        return {};
    }
    const occ::handle<NCollection_HSequence<TopoDS_Shape>> wires =
        ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edges, 1e-5, false);

    std::vector<geom2d::Polyline> loops;
    for (int i = 1; i <= wires->Length(); ++i) {
        const TopoDS_Wire& wire = TopoDS::Wire(wires->Value(i));
        std::vector<geom2d::Vertex> vertices;
        for (BRepTools_WireExplorer wex(wire); wex.More(); wex.Next()) {
            appendEdge(wex.Current(), curveTolerance, vertices);
        }
        if (vertices.size() < 2) {
            continue;
        }
        loops.emplace_back(std::move(vertices), BRep_Tool::IsClosed(wire));
    }
    return geom2d::buildRegions(std::move(loops), curveTolerance);
}

} // namespace

geom2d::Regions sectionAtZ(const Model& model, double z, double curveTolerance) {
    return sectionShapeAtZ(model.impl().shape, z, curveTolerance);
}

geom2d::Regions axialProfile(const Model& model, const Transform3& toFrame, double curveTolerance) {
    // Map the frame's XZ plane onto the XY plane: (x, y, z) -> (z, x, y), so the section at
    // Z = 0 comes out directly in (axial, radial) coordinates.
    Transform3 cyclic = Transform3::Identity();
    cyclic.linear() << 0, 0, 1, 1, 0, 0, 0, 1, 0;
    const Transform3 t = cyclic * toFrame;
    const auto& m = t.matrix();
    gp_Trsf trsf;
    trsf.SetValues(m(0, 0), m(0, 1), m(0, 2), m(0, 3), m(1, 0), m(1, 1), m(1, 2), m(1, 3), m(2, 0),
                   m(2, 1), m(2, 2), m(2, 3));
    const TopoDS_Shape moved = BRepBuilderAPI_Transform(model.impl().shape, trsf, false).Shape();
    const geom2d::Regions full = sectionShapeAtZ(moved, 0.0, curveTolerance);
    if (full.empty()) {
        return {};
    }
    geom2d::Aabb2 box;
    for (const auto& r : full) {
        box.extend(r.bounds());
    }
    const double pad = 1.0 + box.size().norm();
    const geom2d::Regions upper{
        {geom2d::Polyline::rectangle(Vec2(box.min.x() - pad, 0.0),
                                     Vec2(box.max.x() + pad, box.max.y() + pad)),
         {}}};
    return geom2d::intersect(full, upper, {curveTolerance});
}

bool turnsAboutZ(const Model& model, const Transform3& toFrame) {
    const auto& m = toFrame.matrix();
    gp_Trsf trsf;
    trsf.SetValues(m(0, 0), m(0, 1), m(0, 2), m(0, 3), m(1, 0), m(1, 1), m(1, 2), m(1, 3), m(2, 0),
                   m(2, 1), m(2, 2), m(2, 3));
    const TopoDS_Shape moved = BRepBuilderAPI_Transform(model.impl().shape, trsf, false).Shape();
    const gp_Ax1 spindle(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    const auto onAxis = [&](const gp_Ax1& axis) {
        return axis.Direction().IsParallel(spindle.Direction(), 1e-6) &&
               gp_Lin(spindle).Distance(axis.Location()) < 1e-6;
    };
    for (TopExp_Explorer faces(moved, TopAbs_FACE); faces.More(); faces.Next()) {
        const BRepAdaptor_Surface s(TopoDS::Face(faces.Current()));
        switch (s.GetType()) {
        case GeomAbs_Plane:
            if (!s.Plane().Axis().Direction().IsParallel(spindle.Direction(), 1e-6)) {
                return false;
            }
            break;
        case GeomAbs_Cylinder:
            if (!onAxis(s.Cylinder().Axis())) {
                return false;
            }
            break;
        case GeomAbs_Cone:
            if (!onAxis(s.Cone().Axis())) {
                return false;
            }
            break;
        case GeomAbs_Torus:
            if (!onAxis(s.Torus().Axis())) {
                return false;
            }
            break;
        case GeomAbs_Sphere:
            if (gp_Lin(spindle).Distance(s.Sphere().Location()) > 1e-6) {
                return false;
            }
            break;
        case GeomAbs_SurfaceOfRevolution:
            if (!onAxis(s.AxeOfRevolution())) {
                return false;
            }
            break;
        default:
            return false;
        }
    }
    return true;
}

geom2d::Regions axialEnvelope(const Model& model, const Transform3& toFrame,
                              double curveTolerance) {
    geom2d::Regions all = axialProfile(model, toFrame, curveTolerance);
    if (turnsAboutZ(model, toFrame)) {
        return all;
    }
    // Off the planes a part's features usually lie in, and round enough to miss most of each.
    for (int k = 0; k < 8; ++k) {
        const double angle = degToRad(17.0 + 45.0 * k);
        const Transform3 turned =
            Transform3(Eigen::AngleAxisd(angle, Vec3::UnitZ()).toRotationMatrix()) * toFrame;
        all = geom2d::unite(all, axialProfile(model, turned, curveTolerance), {curveTolerance});
    }
    // A hole across the axis leaves a gap around it in every plane. Turning does not reach in there,
    // so what is closed off from outside (open only to the axis) is filled.
    geom2d::Aabb2 box;
    for (const auto& r : all) {
        box.extend(r.bounds());
    }
    const double pad = 1.0;
    const geom2d::Regions around{{geom2d::Polyline::rectangle(
                                      Vec2(box.min.x() - pad, 0.0),
                                      Vec2(box.max.x() + pad, box.max.y() + pad)),
                                  {}}};
    geom2d::Regions closed;
    for (const geom2d::Region& gap : geom2d::subtract(around, all, {curveTolerance})) {
        const geom2d::Aabb2 b = gap.bounds();
        if (b.min.x() > box.min.x() - 0.5 * pad && b.max.x() < box.max.x() + 0.5 * pad &&
            b.max.y() < box.max.y() + 0.5 * pad) {
            closed.push_back(gap);
        }
    }
    return closed.empty() ? all : geom2d::unite(all, closed, {curveTolerance});
}

FaceOutline edgeLoops(const Model& model, const std::vector<int>& edgeIndices,
                      const Transform3& toFrame, double curveTolerance) {
    ShapeIndexMap edges;
    TopExp::MapShapes(model.impl().shape, TopAbs_EDGE, edges);
    const auto& m = toFrame.matrix();
    gp_Trsf trsf;
    trsf.SetValues(m(0, 0), m(0, 1), m(0, 2), m(0, 3), m(1, 0), m(1, 1), m(1, 2), m(1, 3), m(2, 0),
                   m(2, 1), m(2, 2), m(2, 3));
    struct Piece {
        TopoDS_Edge edge;
        gp_Pnt start;
        gp_Pnt end;
    };
    std::vector<Piece> pieces;
    double z = std::numeric_limits<double>::infinity();
    for (const int index : edgeIndices) {
        if (index < 1 || index > edges.Extent()) {
            throw Error(ErrorCode::InvalidArgument, "edge index out of range");
        }
        const TopoDS_Edge edge =
            TopoDS::Edge(BRepBuilderAPI_Transform(edges(index), trsf, true).Shape());
        Piece piece{edge, BRep_Tool::Pnt(TopExp::FirstVertex(edge, true)),
                    BRep_Tool::Pnt(TopExp::LastVertex(edge, true))};
        z = std::min({z, piece.start.Z(), piece.end.Z()});
        pieces.push_back(piece);
    }
    if (pieces.empty()) {
        throw Error(ErrorCode::InvalidArgument, "no edges");
    }
    // Chain the edges end to start (reversing them as needed) into closed loops, in plan view.
    const double joinTol = std::max(1e-4, 10.0 * curveTolerance);
    auto near = [&](const gp_Pnt& a, const gp_Pnt& b) {
        return std::hypot(a.X() - b.X(), a.Y() - b.Y()) <= joinTol;
    };
    std::vector<geom2d::Polyline> loops;
    std::vector<bool> used(pieces.size(), false);
    for (std::size_t first = 0; first < pieces.size(); ++first) {
        if (used[first]) {
            continue;
        }
        used[first] = true;
        std::vector<geom2d::Vertex> vertices;
        appendEdge(pieces[first].edge, curveTolerance, vertices);
        const gp_Pnt loopStart = pieces[first].start;
        gp_Pnt end = pieces[first].end;
        while (!near(end, loopStart)) {
            bool found = false;
            for (std::size_t k = 0; k < pieces.size() && !found; ++k) {
                if (used[k]) {
                    continue;
                }
                if (near(pieces[k].start, end)) {
                    appendEdge(pieces[k].edge, curveTolerance, vertices);
                    end = pieces[k].end;
                    found = true;
                } else if (near(pieces[k].end, end)) {
                    appendEdge(TopoDS::Edge(pieces[k].edge.Reversed()), curveTolerance, vertices);
                    end = pieces[k].start;
                    found = true;
                }
                if (found) {
                    used[k] = true;
                }
            }
            if (!found) {
                throw Error(ErrorCode::InvalidArgument,
                            "the selected edges do not form closed loops");
            }
        }
        if (vertices.size() >= 2) {
            loops.emplace_back(std::move(vertices), true);
        }
    }
    return {geom2d::buildRegions(std::move(loops), curveTolerance), z};
}

std::vector<EdgeChain> edgeChains(const Model& model, const std::vector<int>& edgeIndices,
                                  const Transform3& toFrame, double curveTolerance) {
    ShapeIndexMap edges;
    TopExp::MapShapes(model.impl().shape, TopAbs_EDGE, edges);
    const auto& m = toFrame.matrix();
    gp_Trsf trsf;
    trsf.SetValues(m(0, 0), m(0, 1), m(0, 2), m(0, 3), m(1, 0), m(1, 1), m(1, 2), m(1, 3), m(2, 0),
                   m(2, 1), m(2, 2), m(2, 3));
    struct Piece {
        TopoDS_Edge edge;
        gp_Pnt start;
        gp_Pnt end;
        double low = 0.0; ///< Its lowest point along the frame's Z.
    };
    std::vector<Piece> pieces;
    for (const int index : edgeIndices) {
        if (index < 1 || index > edges.Extent()) {
            throw Error(ErrorCode::InvalidArgument, "edge index out of range");
        }
        const TopoDS_Edge edge =
            TopoDS::Edge(BRepBuilderAPI_Transform(edges(index), trsf, true).Shape());
        Piece piece{edge, BRep_Tool::Pnt(TopExp::FirstVertex(edge, true)),
                    BRep_Tool::Pnt(TopExp::LastVertex(edge, true))};
        piece.low = std::min(piece.start.Z(), piece.end.Z());
        BRepAdaptor_Curve curve(edge);
        GCPnts_QuasiUniformDeflection sampler(curve, curveTolerance);
        if (sampler.IsDone()) {
            for (int i = 1; i <= sampler.NbPoints(); ++i) {
                piece.low = std::min(piece.low, sampler.Value(i).Z());
            }
        }
        pieces.push_back(piece);
    }
    // Chained end to end in space (edges above one another stay apart), each chain as far as it
    // goes both ways from the edge it starts with.
    const double joinTol = std::max(1e-4, 10.0 * curveTolerance);
    const auto near = [&](const gp_Pnt& a, const gp_Pnt& b) {
        return a.Distance(b) <= joinTol;
    };
    std::vector<bool> used(pieces.size(), false);
    std::vector<EdgeChain> out;
    for (std::size_t first = 0; first < pieces.size(); ++first) {
        if (used[first]) {
            continue;
        }
        used[first] = true;
        std::vector<std::pair<std::size_t, bool>> chain{{first, false}}; // (piece, reversed)
        gp_Pnt head = pieces[first].start;
        gp_Pnt tail = pieces[first].end;
        // A piece that goes on from `at`, and its far end; reversed when it runs into `at`.
        const auto extend = [&](const gp_Pnt& at, bool forward) {
            for (std::size_t k = 0; k < pieces.size(); ++k) {
                if (used[k]) {
                    continue;
                }
                const bool fromStart = near(pieces[k].start, at);
                if (!fromStart && !near(pieces[k].end, at)) {
                    continue;
                }
                used[k] = true;
                // Forward, a piece starting at `at` keeps its direction; backward, one ending
                // there does.
                const bool reversed = forward ? !fromStart : fromStart;
                if (forward) {
                    chain.emplace_back(k, reversed);
                    tail = fromStart ? pieces[k].end : pieces[k].start;
                } else {
                    chain.emplace(chain.begin(), k, reversed);
                    head = fromStart ? pieces[k].end : pieces[k].start;
                }
                return true;
            }
            return false;
        };
        while (!near(tail, head) && extend(tail, true)) {
        }
        while (!near(tail, head) && extend(head, false)) {
        }
        const bool closed = near(tail, head);
        std::vector<geom2d::Vertex> vertices;
        double low = std::numeric_limits<double>::infinity();
        for (const auto& [k, reversed] : chain) {
            appendEdge(reversed ? TopoDS::Edge(pieces[k].edge.Reversed()) : pieces[k].edge,
                       curveTolerance, vertices);
            low = std::min(low, pieces[k].low);
        }
        if (!closed) {
            vertices.push_back({xy(tail), 0.0});
        }
        // Edges along the frame's Z leave points on top of one another: one is enough.
        std::vector<geom2d::Vertex> plan;
        for (const geom2d::Vertex& v : vertices) {
            if (!plan.empty() && (v.p - plan.back().p).norm() <= 1e-9) {
                plan.back().bulge = v.bulge;
                continue;
            }
            plan.push_back(v);
        }
        if (closed && plan.size() > 1 && (plan.back().p - plan.front().p).norm() <= 1e-9) {
            plan.pop_back();
        }
        if (plan.size() < 2) {
            continue; // nothing across the frame's Z
        }
        out.push_back({geom2d::Polyline(std::move(plan), closed), low});
    }
    return out;
}

FaceOutline faceOutline(const Model& model, int faceIndex, const Transform3& toFrame,
                        double curveTolerance) {
    ShapeIndexMap faces;
    TopExp::MapShapes(model.impl().shape, TopAbs_FACE, faces);
    if (faceIndex < 1 || faceIndex > faces.Extent()) {
        throw Error(ErrorCode::InvalidArgument, "face index out of range");
    }
    const auto& m = toFrame.matrix();
    gp_Trsf trsf;
    trsf.SetValues(m(0, 0), m(0, 1), m(0, 2), m(0, 3), m(1, 0), m(1, 1), m(1, 2), m(1, 3), m(2, 0),
                   m(2, 1), m(2, 2), m(2, 3));
    const TopoDS_Face face =
        TopoDS::Face(BRepBuilderAPI_Transform(faces(faceIndex), trsf, true).Shape());

    BRepAdaptor_Surface surf(face, true);
    if (surf.GetType() != GeomAbs_Plane) {
        throw Error(ErrorCode::InvalidArgument, "the selected face is not planar");
    }
    const gp_Pln pln = surf.Plane();
    if (std::abs(std::abs(pln.Axis().Direction().Z()) - 1.0) > 1e-9) {
        throw Error(ErrorCode::InvalidArgument,
                    "the selected face is not perpendicular to the tool axis");
    }

    std::vector<geom2d::Polyline> loops;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire& wire = TopoDS::Wire(wx.Current());
        std::vector<geom2d::Vertex> vertices;
        for (BRepTools_WireExplorer wex(wire, face); wex.More(); wex.Next()) {
            appendEdge(wex.Current(), curveTolerance, vertices);
        }
        if (vertices.size() >= 2) {
            loops.emplace_back(std::move(vertices), true);
        }
    }
    return {geom2d::buildRegions(std::move(loops), curveTolerance), pln.Location().Z()};
}

} // namespace ocam::cad
