// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/cad/Tessellation.hpp"

#include "ModelImpl.hpp"

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <algorithm>

namespace ocam::cad {

namespace {

/// The faces' triangles (all faces when `only` is empty), from a meshed geometry copy: OCCT
/// stores triangulations on the shape itself, and the copy keeps the shared model untouched, so
/// worker threads can tessellate while the viewer displays it.
geom3d::Mesh meshFaces(const Model& model, const std::vector<int>& only,
                       const TessellationOptions& options,
                       std::vector<geom3d::Mesh>* each = nullptr) {
    const TopoDS_Shape shape = BRepBuilderAPI_Copy(model.impl().shape, true, false).Shape();
    BRepMesh_IncrementalMesh mesher(shape, options.linearDeflection, false,
                                    options.angularDeflection, true);
    // The copy keeps the topology, so its face map numbers faces as the model's does.
    ShapeIndexMap faces;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);

    geom3d::Mesh mesh;
    for (int index = 1; index <= faces.Extent(); ++index) {
        if (!only.empty() && std::find(only.begin(), only.end(), index) == only.end()) {
            continue;
        }
        const TopoDS_Face& face = TopoDS::Face(faces(index));
        TopLoc_Location loc;
        const occ::handle<Poly_Triangulation>& tri = BRep_Tool::Triangulation(face, loc);
        if (each) {
            // Each face on its own: the mesh is filled per face and moved out after it.
            each->resize(static_cast<std::size_t>(faces.Extent()));
            mesh = {};
        }
        if (tri.IsNull()) {
            continue;
        }
        const gp_Trsf trsf = loc.Transformation();
        const auto base = static_cast<std::uint32_t>(mesh.vertices.size());
        for (int i = 1; i <= tri->NbNodes(); ++i) {
            const gp_Pnt p = tri->Node(i).Transformed(trsf);
            mesh.vertices.emplace_back(p.X(), p.Y(), p.Z());
        }
        const bool reversed = face.Orientation() == TopAbs_REVERSED;
        for (int i = 1; i <= tri->NbTriangles(); ++i) {
            int n1 = 0, n2 = 0, n3 = 0;
            tri->Triangle(i).Get(n1, n2, n3);
            if (reversed) {
                std::swap(n2, n3);
            }
            mesh.triangles.push_back({base + static_cast<std::uint32_t>(n1 - 1),
                                      base + static_cast<std::uint32_t>(n2 - 1),
                                      base + static_cast<std::uint32_t>(n3 - 1)});
        }
        if (each) {
            (*each)[static_cast<std::size_t>(index - 1)] = geom3d::weldVertices(mesh, 1e-5);
        }
    }
    if (each) {
        return {};
    }
    // Faces are meshed separately; shared edge nodes coincide up to evaluation noise.
    return geom3d::weldVertices(mesh, 1e-5);
}

} // namespace

geom3d::Mesh tessellate(const Model& model, const TessellationOptions& options) {
    return meshFaces(model, {}, options);
}

std::vector<geom3d::Mesh> tessellateEachFace(const Model& model,
                                             const TessellationOptions& options) {
    std::vector<geom3d::Mesh> each;
    meshFaces(model, {}, options, &each);
    return each;
}

geom3d::Mesh tessellateFaces(const Model& model, const std::vector<int>& faceIndices,
                             const TessellationOptions& options) {
    if (faceIndices.empty()) {
        return {};
    }
    return meshFaces(model, faceIndices, options);
}

} // namespace ocam::cad
