// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "ocam/document/Project.hpp"

#include <ocam/geom3d/Bounds.hpp>

#include <array>
#include <functional>
#include <optional>

namespace ocam::document {

/// Initial stock as concrete geometry: a box or a cylinder (with a bore: a tube) in the project
/// frame.
struct StockShape {
    enum class Kind { Box, Cylinder } kind = Kind::Box;
    /// Box stock, in the coordinates of `axes`: a project point p has box coordinates
    /// axes^T p.
    geom3d::Aabb3 box;
    /// Box: its edge directions (orthonormal columns) in the project frame. Identity for boxes
    /// given in project coordinates; the setup's WCS axes for stock offset from the model.
    Mat3 axes = Mat3::Identity();
    Vec3 base = Vec3::Zero();  ///< Cylinder: centre of the bottom face.
    Vec3 axis = Vec3::UnitZ(); ///< Cylinder: unit axis.
    double diameter = 0.0;
    double length = 0.0;
    double innerDiameter = 0.0; ///< Cylinder: the bore's diameter (a tube); 0 = solid.

    /// Project-frame bounds.
    geom3d::Aabb3 bounds() const;
    /// Box: rotation from box coordinates to the project frame.
    Transform3 boxToProject() const;
    /// Box: the eight corners in the project frame.
    std::array<Vec3, 8> corners() const;
    bool operator==(const StockShape&) const = default;
};

/// Returns the bounds of a model in the coordinates of `axes` (orthonormal columns in the
/// project frame; a project point p has coordinates axes^T p), or nullopt when the model is not
/// available.
using ModelBoundsFn =
    std::function<std::optional<geom3d::Aabb3>(const Uuid& modelId, const Mat3& axes)>;

/// Returns the largest distance of a model from the line through `point` along the unit
/// `direction` (project frame), or nullopt when the model is not available.
using ModelRadiusFn = std::function<std::optional<double>(const Uuid& modelId, const Vec3& point,
                                                          const Vec3& direction)>;

/// Resolves a setup's stock definition (DL-0038): a box along the setup's WCS axes, or a
/// cylinder or tube along one of them, centred on the part's box (offsets and rounding in WCS
/// directions) and moved by the stock's shift. Stock sized from the part needs `modelRadius`
/// for a cylinder or tube. FromSetup resolves the source setup's initial stock (the simulation
/// carries what that setup cut). Returns nullopt when the definition cannot be resolved (missing
/// model, invalid reference, a bore not inside the stock).
std::optional<StockShape> resolveStock(const Project& project, const Setup& setup,
                                       const ModelBoundsFn& modelBounds,
                                       const ModelRadiusFn& modelRadius = {});

} // namespace ocam::document
