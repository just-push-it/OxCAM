// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Probing operations in a project. See docs/design-log/0090.

#include "ocam/document/Probing.hpp"

#include "ocam/document/Generate.hpp"
#include "ocam/document/PostSetup.hpp"

#include <ocam/cad/SubShape.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/strategies/Probing.hpp>

#include <fmt/format.h>

#include <cmath>
#include <string>
#include <vector>

namespace ocam::document {
namespace {

Error userError(const std::string& message) {
    return Error(ErrorCode::InvalidArgument, message);
}

/// A picked face in the WCS.
struct PickedFace {
    cad::SubShapeGeometry::Kind kind = cad::SubShapeGeometry::Kind::Plane;
    Vec3 point = Vec3::Zero();     ///< Plane: its centroid. Cylinder: on its axis, mid-length.
    Vec3 direction = Vec3::Zero(); ///< Plane: its outward normal. Cylinder: its axis.
    double radius = 0.0;
    bool solidOnAxis = false; ///< Cylinder: the part is on its axis (a boss), not a hole.
};

constexpr double kSquare = 1e-3; // how far from an axis a face's normal may point

/// The WCS axis `v` lies along (0 X, 1 Y, 2 Z), or -1.
int axisOf(const Vec3& v) {
    for (int a = 0; a < 3; ++a) {
        if (std::abs(std::abs(v[a]) - 1.0) < kSquare) {
            return a;
        }
    }
    return -1;
}

std::vector<PickedFace> pickedFaces(const Document& doc, const Setup& setup, const Operation& op) {
    std::vector<PickedFace> faces;
    for (const ShapeRef& ref : op.geometry) {
        if (ref.type != cad::SubShapeType::Face) {
            throw userError("probing measures faces: remove the selected edges");
        }
        const ModelEntry* entry = doc.project().findModel(ref.modelId);
        const cad::Model* model = doc.model(ref.modelId);
        if (!entry || !model) {
            throw userError("the operation refers to a model that is no longer in the project");
        }
        const auto index = cad::matchSubShape(*model, ref.type, ref.signature, ref.index);
        if (!index) {
            throw userError("a picked face no longer exists in the model; pick it again");
        }
        const cad::SubShapeGeometry g = cad::subShapeGeometry(*model, ref.type, *index);
        const Transform3 toWcs = modelToWcs(*entry, setup);
        PickedFace f;
        f.kind = g.kind;
        if (g.kind == cad::SubShapeGeometry::Kind::Plane) {
            f.point = toWcs * g.point;
            f.direction = (toWcs.linear() * g.direction).normalized();
        } else if (g.kind == cad::SubShapeGeometry::Kind::Cylinder) {
            const Vec3 middle = 0.5 * (g.axisStart + g.axisEnd);
            f.point = toWcs * middle;
            f.direction = (toWcs.linear() * g.direction).normalized();
            f.radius = g.radius;
            f.solidOnAxis = cad::containsPoint(*model, middle, 1e-6);
        } else {
            throw userError("probing measures flat and cylindrical faces only");
        }
        faces.push_back(f);
    }
    return faces;
}

std::string directionName(int axis, double sign) {
    static const char* const names[3][2] = {
        {"x_minus", "x_plus"}, {"y_minus", "y_plus"}, {"z_minus", "z_minus"}};
    return names[axis][sign > 0.0 ? 1 : 0];
}

/// What the picked faces are, as parameter values.
Json nominalsOf(const std::vector<PickedFace>& faces) {
    using Kind = cad::SubShapeGeometry::Kind;
    const char* const kWhat =
        "pick one flat face (a surface), two parallel faces (a web or pocket), two faces at a "
        "right angle (a corner) or one cylindrical face (a bore or boss), square to the WCS";
    if (faces.size() == 1 && faces[0].kind == Kind::Cylinder) {
        const PickedFace& c = faces[0];
        if (axisOf(c.direction) != 2) {
            throw userError("a bore or boss is probed along the tool axis: pick one whose axis "
                            "is the WCS Z");
        }
        return {{"feature", c.solidOnAxis ? "boss" : "bore"},
                {"x", c.point.x()},
                {"y", c.point.y()},
                {"z", c.point.z()},
                {"diameter", 2.0 * c.radius}};
    }
    for (const PickedFace& f : faces) {
        if (f.kind != Kind::Plane || axisOf(f.direction) < 0) {
            throw userError(kWhat);
        }
    }
    if (faces.size() == 1) {
        const PickedFace& f = faces[0];
        const int axis = axisOf(f.direction);
        if (axis == 2 && f.direction.z() < 0.0) {
            throw userError("the probe cannot touch a face that looks down");
        }
        // The stylus moves against the face's outward normal.
        return {{"feature", "surface"},
                {"direction", directionName(axis, -f.direction[axis])},
                {"x", f.point.x()},
                {"y", f.point.y()},
                {"z", f.point.z()}};
    }
    if (faces.size() != 2) {
        throw userError(kWhat);
    }
    const PickedFace& a = faces[0];
    const PickedFace& b = faces[1];
    const int axisA = axisOf(a.direction);
    const int axisB = axisOf(b.direction);
    if (axisA == 2 || axisB == 2) {
        throw userError("a web, pocket or corner is measured on side faces: pick faces square "
                        "to X or Y");
    }
    if (axisA == axisB) {
        if (a.direction.dot(b.direction) > 0.0) {
            throw userError("the two faces look the same way: pick the two sides of a web or "
                            "pocket");
        }
        const double width = std::abs(a.point[axisA] - b.point[axisA]);
        // A web's faces look away from each other, a pocket's at each other.
        const bool web = a.direction.dot(a.point - b.point) > 0.0;
        const Vec3 centre = 0.5 * (a.point + b.point);
        return {{"feature", web ? "web" : "pocket"},
                {"axis", axisA == 0 ? "x" : "y"},
                {"x", centre.x()},
                {"y", centre.y()},
                {"z", centre.z()},
                {"width", width}};
    }
    // A corner: the face square to X and the one square to Y.
    const PickedFace& fx = axisA == 0 ? a : b;
    const PickedFace& fy = axisA == 0 ? b : a;
    const double cx = fx.point.x();
    const double cy = fy.point.y();
    const double nx = fx.direction.x() < 0.0 ? -1.0 : 1.0;
    const double ny = fy.direction.y() < 0.0 ? -1.0 : 1.0;
    // Outside a part's corner the X face runs from the corner away from where the Y face looks;
    // inside a pocket's, towards it.
    const bool outside = (fx.point.y() - cy) * ny < 0.0;
    const double sx = outside ? nx : -nx;
    const double sy = outside ? ny : -ny;
    return {{"feature", outside ? "outside_corner" : "inside_corner"},
            {"corner",
             fmt::format("x_{}_y_{}", sx > 0.0 ? "plus" : "minus", sy > 0.0 ? "plus" : "minus")},
            {"x", cx},
            {"y", cy},
            {"z", 0.5 * (fx.point.z() + fy.point.z())}};
}

/// Whether the operation type measures that feature.
bool measures(std::string_view type, const std::string& feature) {
    if (type == "probe_surface") {
        return feature == "surface";
    }
    if (type == "probe_width") {
        return feature == "web" || feature == "pocket";
    }
    if (type == "probe_circle") {
        return feature == "bore" || feature == "boss";
    }
    if (type == "probe_corner") {
        return feature == "outside_corner" || feature == "inside_corner";
    }
    return feature != "outside_corner" && feature != "inside_corner"; // probe_check
}

} // namespace

Json probeNominals(const Document& doc, const Setup& setup, const Operation& op) {
    if (!strategies::isProbingStrategy(op.type)) {
        throw userError(fmt::format("'{}' is not a probing operation", op.type));
    }
    if (op.geometry.empty()) {
        throw userError("pick the faces to measure");
    }
    Json values = nominalsOf(pickedFaces(doc, setup, op));
    const std::string feature = values.at("feature").get<std::string>();
    if (!measures(op.type, feature)) {
        const strategies::StrategyInfo* info = strategies::findStrategy(op.type);
        throw userError(fmt::format("the picked faces are a {}, which a {} does not measure",
                                    std::string(feature == "outside_corner"  ? "outside corner"
                                                : feature == "inside_corner" ? "inside corner"
                                                                             : feature),
                                    info ? info->label : op.type));
    }
    if (op.type == "probe_surface") {
        values.erase("feature"); // the operation type says it
    }
    return values;
}

Json withProbeNominals(const Document& doc, const Setup& setup, const Operation& op) {
    Json params = op.parameters.is_object() ? op.parameters : Json::object();
    const Json nominals = probeNominals(doc, setup, op); // outlives the loop over it
    for (const auto& [key, value] : nominals.items()) {
        params[key] = value;
    }
    return params;
}

toolpath::Toolpath generateProbing(const Document& doc, const Setup& setup, const Operation& op,
                                   const tooling::Tool& tool, CancelToken& cancel,
                                   ProgressSink& progress) {
    const Project& project = doc.project();
    if (machineFor(project, setup).isLathe()) {
        throw userError("probing needs a milling machine");
    }
    if (op.orientation) {
        throw userError("probing cycles run along the setup's Z axis: clear the operation's tool "
                        "orientation");
    }
    if (tool.type != tooling::ToolType::Probe) {
        throw userError(fmt::format("probing needs a probe: T{} is a {}; choose a probe as the "
                                    "operation's tool",
                                    tool.number, tooling::displayName(tool.type)));
    }
    strategies::ProbeParams params = strategies::probeParams(op.type, op.parameters);
    if (!params.workOffset && op.type != "probe_check") {
        params.workOffset = setup.workOffset; // "the setup's own", as the parameter says
    }
    const OperationHeightValues hv = operationHeights(doc, setup, op);
    strategies::Heights heights;
    heights.clearance = hv.clearance;
    heights.retract = hv.retract;
    strategies::Feeds feeds; // a probe has no spindle speed or cutting feeds
    feeds.coolant = tooling::Coolant::Off;
    const strategies::Context ctx{tool, feeds, heights, 0.002, cancel, progress};
    toolpath::Toolpath tp = strategies::probe(params, ctx);

    // The picked faces gave the nominal values; say so when they no longer match.
    if (!op.geometry.empty()) {
        const Json fromFaces = probeNominals(doc, setup, op);
        const strategies::ParamReader reader(*strategies::findStrategy(op.type), op.parameters);
        for (const auto& [key, value] : fromFaces.items()) {
            const bool differs = value.is_number()
                                     ? std::abs(value.get<double>() - reader.number(key)) > 1e-4
                                     : value.get<std::string>() != reader.choice(key);
            if (differs) {
                tp.warnings.push_back(fmt::format(
                    "the picked faces give {} = {}, the operation has {}: pick the faces again "
                    "to use the model's",
                    key,
                    value.is_number() ? fmt::format("{:.4f}", value.get<double>())
                                      : value.get<std::string>(),
                    value.is_number() ? fmt::format("{:.4f}", reader.number(key))
                                      : reader.choice(key)));
            }
        }
    }
    tp.warnings.insert(tp.warnings.begin(), hv.warnings.begin(), hv.warnings.end());
    tp.operationId = op.id;
    return tp;
}

} // namespace ocam::document
