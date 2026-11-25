// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "SimPrs.hpp"

#include <Graphic3d_ArrayOfPoints.hxx>
#include <Graphic3d_ArrayOfSegments.hxx>
#include <Graphic3d_ArrayOfTriangles.hxx>
#include <Graphic3d_AspectFillArea3d.hxx>
#include <Graphic3d_AspectLine3d.hxx>
#include <Graphic3d_AspectMarker3d.hxx>
#include <Graphic3d_Group.hxx>
#include <Graphic3d_MaterialAspect.hxx>
#include <Poly_Triangulation.hxx>
#include <Prs3d_Presentation.hxx>
#include <Select3D_SensitiveTriangulation.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <SelectMgr_Selection.hxx>
#include <cmath>

namespace ocam::gui {
namespace {

occ::handle<Graphic3d_AspectFillArea3d> fillAspect(const Quantity_Color& colour,
                                                   float transparency = 0.0f) {
    Graphic3d_MaterialAspect material(Graphic3d_NameOfMaterial_Plastified);
    material.SetColor(colour);
    material.SetTransparency(transparency);
    occ::handle<Graphic3d_AspectFillArea3d> aspect = new Graphic3d_AspectFillArea3d();
    aspect->SetInteriorStyle(Aspect_IS_SOLID);
    aspect->SetInteriorColor(colour);
    aspect->SetFrontMaterial(material);
    aspect->SetBackMaterial(material);
    aspect->SetDistinguishOff();
    return aspect;
}

/// Surface of revolution of a (radius, height) polyline around +Z.
void addRevolved(Graphic3d_ArrayOfTriangles& tris, const std::vector<Vec2>& rh, int segments) {
    for (std::size_t k = 0; k + 1 < rh.size(); ++k) {
        const Vec2 a = rh[k], b = rh[k + 1];
        const Vec2 d = b - a;
        if (d.norm() < 1e-9) {
            continue;
        }
        const Vec2 n = Vec2(d.y(), -d.x()).normalized(); // outward in (r, h)
        for (int s = 0; s < segments; ++s) {
            const double t0 = 2.0 * kPi * s / segments, t1 = 2.0 * kPi * (s + 1) / segments;
            const double c0 = std::cos(t0), s0 = std::sin(t0), c1 = std::cos(t1), s1 = std::sin(t1);
            const gp_Pnt p00(a.x() * c0, a.x() * s0, a.y()), p01(a.x() * c1, a.x() * s1, a.y());
            const gp_Pnt p10(b.x() * c0, b.x() * s0, b.y()), p11(b.x() * c1, b.x() * s1, b.y());
            const gp_Dir n0(n.x() * c0, n.x() * s0, n.y()), n1(n.x() * c1, n.x() * s1, n.y());
            const int i00 = tris.AddVertex(p00, n0), i01 = tris.AddVertex(p01, n1);
            const int i10 = tris.AddVertex(p10, n0), i11 = tris.AddVertex(p11, n1);
            tris.AddEdges(i00, i01, i11);
            tris.AddEdges(i00, i11, i10);
        }
    }
}

int revolvedVertexCount(std::size_t points, int segments) {
    return static_cast<int>(points) * segments * 4;
}

} // namespace

StockMeshPrs::StockMeshPrs(std::shared_ptr<const sim::DisplayMesh> mesh,
                           const Transform3& wcsToProject, double transparency)
    : mesh_(std::move(mesh)), toProject_(wcsToProject), transparency_(transparency) {}

void StockMeshPrs::Compute(const occ::handle<PrsMgr_PresentationManager>& /*manager*/,
                           const occ::handle<Prs3d_Presentation>& prs, const int /*mode*/) {
    if (!mesh_ || mesh_->indices.empty()) {
        return;
    }
    const auto& m = *mesh_;
    occ::handle<Graphic3d_ArrayOfTriangles> tris = new Graphic3d_ArrayOfTriangles(
        static_cast<int>(m.positions.size()), static_cast<int>(m.indices.size()),
        Graphic3d_ArrayFlags_VertexNormal | Graphic3d_ArrayFlags_VertexColor);
    const Mat3 rot = toProject_.linear();
    for (std::size_t k = 0; k < m.positions.size(); ++k) {
        const Vec3 p = toProject_ * Vec3(m.positions[k][0], m.positions[k][1], m.positions[k][2]);
        const Vec3 n = rot * Vec3(m.normals[k][0], m.normals[k][1], m.normals[k][2]);
        const int v = tris->AddVertex(gp_Pnt(p.x(), p.y(), p.z()), gp_Dir(n.x(), n.y(), n.z()));
        const auto& c = m.colours[k];
        tris->SetVertexColor(
            v, Quantity_Color(c[0] / 255.0, c[1] / 255.0, c[2] / 255.0, Quantity_TOC_sRGB));
    }
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
        tris->AddEdges(static_cast<int>(m.indices[k]) + 1, static_cast<int>(m.indices[k + 1]) + 1,
                       static_cast<int>(m.indices[k + 2]) + 1);
    }
    occ::handle<Graphic3d_Group> group = prs->NewGroup();
    group->SetGroupPrimitivesAspect(fillAspect(Quantity_Color(1.0, 1.0, 1.0, Quantity_TOC_sRGB),
                                               static_cast<float>(transparency_)));
    group->AddPrimitiveArray(tris);
}

MachinePartPrs::MachinePartPrs(std::shared_ptr<const geom3d::Mesh> mesh, std::uint32_t rgb,
                               double transparency)
    : mesh_(std::move(mesh)), rgb_(rgb), transparency_(transparency) {}

void MachinePartPrs::Compute(const occ::handle<PrsMgr_PresentationManager>& /*manager*/,
                             const occ::handle<Prs3d_Presentation>& prs, const int /*mode*/) {
    if (!mesh_ || mesh_->isEmpty()) {
        return;
    }
    // Each triangle with its own normal: machine parts are mostly flat faces.
    const auto& m = *mesh_;
    const int corners = static_cast<int>(m.triangles.size() * 3);
    occ::handle<Graphic3d_ArrayOfTriangles> tris =
        new Graphic3d_ArrayOfTriangles(corners, 0, Graphic3d_ArrayFlags_VertexNormal);
    for (std::size_t t = 0; t < m.triangles.size(); ++t) {
        const Vec3 n = m.triangleNormal(t);
        if (n.squaredNorm() < 0.5) {
            continue; // degenerate
        }
        const gp_Dir normal(n.x(), n.y(), n.z());
        for (const std::uint32_t v : m.triangles[t]) {
            const Vec3& p = m.vertices[v];
            tris->AddVertex(gp_Pnt(p.x(), p.y(), p.z()), normal);
        }
    }
    const Quantity_Color colour(((rgb_ >> 16) & 0xFF) / 255.0, ((rgb_ >> 8) & 0xFF) / 255.0,
                                (rgb_ & 0xFF) / 255.0, Quantity_TOC_sRGB);
    occ::handle<Graphic3d_Group> group = prs->NewGroup();
    group->SetGroupPrimitivesAspect(fillAspect(colour, static_cast<float>(transparency_)));
    group->AddPrimitiveArray(tris);
}

void MachinePartPrs::ComputeSelection(const occ::handle<SelectMgr_Selection>& selection,
                                      const int mode) {
    if (mode != 0 || !mesh_ || mesh_->isEmpty()) {
        return;
    }
    const auto& m = *mesh_;
    occ::handle<Poly_Triangulation> triangulation = new Poly_Triangulation(
        static_cast<int>(m.vertices.size()), static_cast<int>(m.triangles.size()), false);
    for (std::size_t v = 0; v < m.vertices.size(); ++v) {
        const Vec3& p = m.vertices[v];
        triangulation->SetNode(static_cast<int>(v) + 1, gp_Pnt(p.x(), p.y(), p.z()));
    }
    for (std::size_t t = 0; t < m.triangles.size(); ++t) {
        const auto& tri = m.triangles[t];
        triangulation->SetTriangle(static_cast<int>(t) + 1,
                                   Poly_Triangle(static_cast<int>(tri[0]) + 1,
                                                 static_cast<int>(tri[1]) + 1,
                                                 static_cast<int>(tri[2]) + 1));
    }
    occ::handle<SelectMgr_EntityOwner> owner = new SelectMgr_EntityOwner(this);
    selection->Add(
        new Select3D_SensitiveTriangulation(owner, triangulation, TopLoc_Location(), true));
}

ToolBodyPrs::ToolBodyPrs(std::shared_ptr<const sim::ToolShape> tool) : tool_(std::move(tool)) {}

void ToolBodyPrs::Compute(const occ::handle<PrsMgr_PresentationManager>& /*manager*/,
                          const occ::handle<Prs3d_Presentation>& prs, const int /*mode*/) {
    if (!tool_) {
        return;
    }
    constexpr int kSegments = 48;
    const std::vector<Vec2> pts = tool_->profile().outline.discretize(0.02);
    const double L = tool_->cuttingLength();
    const double top = std::min(tool_->visibleLength(), tool_->profile().length);
    std::vector<Vec2> cutting, shank;
    for (std::size_t k = 0; k < pts.size(); ++k) {
        const Vec2& p = pts[k];
        if (p.y() <= L + 1e-9) {
            cutting.push_back(p);
        }
        if (p.y() >= L - 1e-9 && p.y() <= top + 1e-9) {
            shank.push_back(p);
        }
        // Split segments crossing the cutting length or the holder face.
        if (k + 1 < pts.size()) {
            const Vec2& q = pts[k + 1];
            for (const double h : {L, top}) {
                if ((p.y() - h) * (q.y() - h) < 0.0) {
                    const Vec2 x = p + (q - p) * ((h - p.y()) / (q.y() - p.y()));
                    if (h == L) {
                        cutting.push_back(x);
                        shank.push_back(x);
                    } else {
                        shank.push_back(x);
                    }
                }
            }
        }
    }
    for (const auto& [part, colour] :
         {std::pair{&cutting, Quantity_Color(0.95, 0.75, 0.25, Quantity_TOC_sRGB)},
          std::pair{&shank, Quantity_Color(0.70, 0.72, 0.75, Quantity_TOC_sRGB)}}) {
        if (part->size() < 2) {
            continue;
        }
        occ::handle<Graphic3d_ArrayOfTriangles> tris = new Graphic3d_ArrayOfTriangles(
            revolvedVertexCount(part->size(), kSegments),
            revolvedVertexCount(part->size(), kSegments) * 2, Graphic3d_ArrayFlags_VertexNormal);
        addRevolved(*tris, *part, kSegments);
        occ::handle<Graphic3d_Group> group = prs->NewGroup();
        group->SetGroupPrimitivesAspect(fillAspect(colour));
        group->AddPrimitiveArray(tris);
    }
    if (!tool_->holder().empty()) {
        std::vector<Vec2> outline;
        for (const auto& band : tool_->holder()) {
            outline.emplace_back(0.0, band.h0);
            outline.emplace_back(band.radius, band.h0);
            outline.emplace_back(band.radius, band.h1);
            outline.emplace_back(0.0, band.h1);
        }
        occ::handle<Graphic3d_ArrayOfTriangles> tris = new Graphic3d_ArrayOfTriangles(
            revolvedVertexCount(outline.size(), kSegments),
            revolvedVertexCount(outline.size(), kSegments) * 2, Graphic3d_ArrayFlags_VertexNormal);
        for (std::size_t k = 0; k + 3 < outline.size(); k += 4) {
            addRevolved(*tris, {outline[k], outline[k + 1], outline[k + 2], outline[k + 3]},
                        kSegments);
        }
        occ::handle<Graphic3d_Group> group = prs->NewGroup();
        group->SetGroupPrimitivesAspect(
            fillAspect(Quantity_Color(0.45, 0.48, 0.55, Quantity_TOC_sRGB), 0.6f));
        group->AddPrimitiveArray(tris);
    }
}

namespace {

/// A convex outline in the lathe plane (u, v) as a plate from y = low to y = high on the XZ plane.
occ::handle<Graphic3d_ArrayOfTriangles> lathePlate(const std::vector<Vec2>& pts, double low,
                                                   double high) {
    double area = 0.0;
    for (std::size_t k = 0; k < pts.size(); ++k) {
        const Vec2& a = pts[k];
        const Vec2& b = pts[(k + 1) % pts.size()];
        area += a.x() * b.y() - b.x() * a.y();
    }
    // Lathe plane (u, v) -> WCS (x = v, y, z = u).
    auto at = [](const Vec2& p, double y) {
        return gp_Pnt(p.y(), y, p.x());
    };
    const double turn = area >= 0.0 ? 1.0 : -1.0; // counter-clockwise in (u, v)
    const int n = static_cast<int>(pts.size());
    occ::handle<Graphic3d_ArrayOfTriangles> tris = new Graphic3d_ArrayOfTriangles(
        2 * n + 4 * n, 6 * (n - 2) + 6 * n, Graphic3d_ArrayFlags_VertexNormal);
    // The two faces (the outline is convex: a fan).
    for (const double y : {high, low}) {
        const gp_Dir normal(0.0, y == high ? 1.0 : -1.0, 0.0);
        const int first = tris->VertexNumber() + 1;
        for (const Vec2& p : pts) {
            tris->AddVertex(at(p, y), normal);
        }
        for (int k = 1; k + 1 < n; ++k) {
            tris->AddEdges(first, first + k, first + k + 1);
        }
    }
    // The sides, each edge with its own outward normal.
    for (int k = 0; k < n; ++k) {
        const Vec2& a = pts[static_cast<std::size_t>(k)];
        const Vec2& b = pts[static_cast<std::size_t>((k + 1) % n)];
        const Vec2 d = b - a;
        if (d.norm() < 1e-12) {
            continue;
        }
        const Vec2 out = turn * Vec2(d.y(), -d.x()).normalized(); // (u, v)
        const gp_Dir normal(out.y(), 0.0, out.x());
        const int i0 = tris->AddVertex(at(a, high), normal);
        const int i1 = tris->AddVertex(at(b, high), normal);
        const int i2 = tris->AddVertex(at(b, low), normal);
        const int i3 = tris->AddVertex(at(a, low), normal);
        tris->AddEdges(i0, i1, i2);
        tris->AddEdges(i0, i2, i3);
    }
    return tris;
}

/// A bar along Z, from u0 to u1, its axis at x = v and y = 0.
occ::handle<Graphic3d_ArrayOfTriangles> latheBar(double u0, double u1, double v, double radius) {
    constexpr int kSegments = 32;
    occ::handle<Graphic3d_ArrayOfTriangles> tris = new Graphic3d_ArrayOfTriangles(
        4 * kSegments + 2 * (kSegments + 1), 6 * kSegments + 2 * 3 * (kSegments - 1),
        Graphic3d_ArrayFlags_VertexNormal);
    auto rim = [&](int k) {
        const double a = 2.0 * kPi * k / kSegments;
        return std::pair(radius * std::cos(a), radius * std::sin(a));
    };
    for (int k = 0; k < kSegments; ++k) {
        const auto [x0, y0] = rim(k);
        const auto [x1, y1] = rim(k + 1);
        const int i0 = tris->AddVertex(gp_Pnt(v + x0, y0, u0), gp_Dir(x0, y0, 0.0));
        const int i1 = tris->AddVertex(gp_Pnt(v + x1, y1, u0), gp_Dir(x1, y1, 0.0));
        const int i2 = tris->AddVertex(gp_Pnt(v + x1, y1, u1), gp_Dir(x1, y1, 0.0));
        const int i3 = tris->AddVertex(gp_Pnt(v + x0, y0, u1), gp_Dir(x0, y0, 0.0));
        tris->AddEdges(i0, i1, i2);
        tris->AddEdges(i0, i2, i3);
    }
    for (const double u : {u0, u1}) {
        const gp_Dir normal(0.0, 0.0, u == u1 ? 1.0 : -1.0);
        const int first = tris->VertexNumber() + 1;
        for (int k = 0; k <= kSegments; ++k) {
            const auto [x, y] = rim(k);
            tris->AddVertex(gp_Pnt(v + x, y, u), normal);
        }
        for (int k = 1; k + 1 < kSegments; ++k) {
            tris->AddEdges(first, first + k, first + k + 1);
        }
    }
    return tris;
}

} // namespace

InsertPrs::InsertPrs(std::shared_ptr<const geom2d::Polyline> outline, double thickness,
                     std::shared_ptr<const std::vector<tooling::TurningToolPart>> parts)
    : outline_(std::move(outline)), thickness_(thickness), parts_(std::move(parts)) {}

void InsertPrs::Compute(const occ::handle<PrsMgr_PresentationManager>& /*manager*/,
                        const occ::handle<Prs3d_Presentation>& prs, const int /*mode*/) {
    if (!outline_) {
        return;
    }
    std::vector<Vec2> pts = outline_->discretize(0.01);
    if (pts.size() > 1 && (pts.front() - pts.back()).norm() < 1e-9) {
        pts.pop_back();
    }
    if (pts.size() < 3) {
        return;
    }
    geom2d::Aabb2 box;
    for (const Vec2& p : pts) {
        box.extend(p);
    }
    const double half =
        0.5 * (thickness_ > 0.0 ? thickness_ : 0.25 * std::max(box.size().maxCoeff(), 1.0));
    occ::handle<Graphic3d_Group> group = prs->NewGroup();
    group->SetGroupPrimitivesAspect(
        fillAspect(Quantity_Color(0.95, 0.75, 0.25, Quantity_TOC_sRGB)));
    group->AddPrimitiveArray(lathePlate(pts, -half, half));
    if (!parts_) {
        return;
    }
    for (const tooling::TurningToolPart& part : *parts_) {
        std::vector<Vec2> outline = part.outline.discretize(0.01);
        if (outline.size() > 1 && (outline.front() - outline.back()).norm() < 1e-9) {
            outline.pop_back();
        }
        if (outline.size() < 3) {
            continue;
        }
        geom2d::Aabb2 bounds;
        for (const Vec2& p : outline) {
            bounds.extend(p);
        }
        occ::handle<Graphic3d_Group> piece = prs->NewGroup();
        piece->SetGroupPrimitivesAspect(
            fillAspect(Quantity_Color(0.45, 0.48, 0.55, Quantity_TOC_sRGB), 0.35f));
        piece->AddPrimitiveArray(part.round ? latheBar(bounds.min.x(), bounds.max.x(),
                                                       0.5 * (bounds.min.y() + bounds.max.y()),
                                                       0.5 * (bounds.max.y() - bounds.min.y()))
                                            : lathePlate(outline, part.low, part.high));
    }
}

MarkersPrs::MarkersPrs(std::vector<Vec3> points) : points_(std::move(points)) {}

LinesPrs::LinesPrs(std::vector<std::pair<Vec3, Vec3>> segments, std::uint32_t rgb, double width)
    : segments_(std::move(segments)), rgb_(rgb), width_(width) {}

void LinesPrs::Compute(const occ::handle<PrsMgr_PresentationManager>& /*manager*/,
                       const occ::handle<Prs3d_Presentation>& prs, const int /*mode*/) {
    if (segments_.empty()) {
        return;
    }
    occ::handle<Graphic3d_ArrayOfSegments> array =
        new Graphic3d_ArrayOfSegments(static_cast<int>(segments_.size() * 2));
    for (const auto& [a, b] : segments_) {
        array->AddVertex(a.x(), a.y(), a.z());
        array->AddVertex(b.x(), b.y(), b.z());
    }
    const Quantity_Color colour(((rgb_ >> 16) & 0xFF) / 255.0, ((rgb_ >> 8) & 0xFF) / 255.0,
                                (rgb_ & 0xFF) / 255.0, Quantity_TOC_sRGB);
    occ::handle<Graphic3d_Group> group = prs->NewGroup();
    group->SetGroupPrimitivesAspect(
        new Graphic3d_AspectLine3d(colour, Aspect_TOL_DASH, static_cast<float>(width_)));
    group->AddPrimitiveArray(array);
}

void MarkersPrs::Compute(const occ::handle<PrsMgr_PresentationManager>& /*manager*/,
                         const occ::handle<Prs3d_Presentation>& prs, const int /*mode*/) {
    if (points_.empty()) {
        return;
    }
    occ::handle<Graphic3d_ArrayOfPoints> array =
        new Graphic3d_ArrayOfPoints(static_cast<int>(points_.size()));
    for (const Vec3& p : points_) {
        array->AddVertex(p.x(), p.y(), p.z());
    }
    occ::handle<Graphic3d_Group> group = prs->NewGroup();
    group->SetGroupPrimitivesAspect(new Graphic3d_AspectMarker3d(
        Aspect_TOM_BALL, Quantity_Color(0.95, 0.10, 0.10, Quantity_TOC_sRGB), 4.0));
    group->AddPrimitiveArray(array);
}

} // namespace ocam::gui
