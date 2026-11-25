// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "BreakEdges.hpp"

#include "ocam/document/Generate.hpp"

#include <ocam/cad/EdgeFaces.hpp>
#include <ocam/cad/SubShape.hpp>
#include <ocam/cad/Tessellation.hpp>
#include <ocam/foundation/Error.hpp>

#include <fmt/format.h>

#include <algorithm>

namespace ocam::document {

BreakEdgesInput breakEdgesInput(const Document& doc, const Setup& setup, const Operation& op,
                                double spacing, double deflection) {
    const Project& project = doc.project();
    BreakEdgesInput in;
    std::vector<Uuid> models;
    std::vector<std::size_t> firstFace; // per model: where its faces start in `in.faces`
    for (const ShapeRef& ref : op.geometry) {
        if (ref.type != cad::SubShapeType::Edge) {
            continue;
        }
        const ModelEntry* entry = project.findModel(ref.modelId);
        const cad::Model* model = doc.model(ref.modelId);
        if (!entry || !model) {
            throw Error(ErrorCode::InvalidArgument,
                        "the operation refers to a model that is no longer in the project");
        }
        const auto index = cad::matchSubShape(*model, ref.type, ref.signature, ref.index);
        if (!index) {
            throw Error(ErrorCode::InvalidArgument,
                        "a selected edge no longer exists in the model; select it again");
        }
        auto known = std::find(models.begin(), models.end(), ref.modelId);
        if (known == models.end()) {
            models.push_back(ref.modelId);
            firstFace.push_back(in.faces.size());
            for (geom3d::Mesh& mesh : cad::tessellateEachFace(*model, {deflection, 0.2})) {
                mesh.transform(modelToWcs(*entry, setup));
                in.faces.push_back(std::move(mesh));
            }
            known = models.end() - 1;
        }
        const std::size_t offset = firstFace[static_cast<std::size_t>(known - models.begin())];
        const Transform3 toWcs = modelToWcs(*entry, setup);
        const Mat3 turn = toWcs.linear();
        const cad::EdgeFaces faces = cad::edgeFaces(*model, *index, spacing);
        strategies::BreakEdge edge;
        edge.name = fmt::format("edge {}", *index);
        edge.closed = faces.closed;
        edge.unusable = faces.problem;
        if (faces.problem.empty()) {
            for (std::size_t k = 0; k < 2; ++k) {
                edge.faces[k] = static_cast<int>(offset) + faces.faces[k] - 1;
            }
        }
        for (const cad::EdgeFacePoint& p : faces.points) {
            strategies::EdgeSample s;
            s.point = toWcs * p.point;
            s.tangent = (turn * p.tangent).normalized();
            for (std::size_t k = 0; k < 2; ++k) {
                s.normal[k] = (turn * p.normal[k]).normalized();
                s.inFace[k] = (turn * p.inFace[k]).normalized();
            }
            edge.samples.push_back(s);
        }
        in.edges.push_back(std::move(edge));
    }
    if (in.edges.empty()) {
        throw Error(ErrorCode::InvalidArgument, "select the edges to break");
    }
    return in;
}

} // namespace ocam::document
