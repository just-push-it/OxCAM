// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// 3D edge breaking (DL-0083): a chamfer mill or ball end mill follows selected edges of any
// shape, keeping the tool axis +Z, with the break's width and its contact held across each edge
// and the tool checked by drop-cutter (DL-0016) against everything but the break.

#pragma once

#include "ocam/strategies/Surface.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace ocam::strategies {

/// A point of an edge with the two faces that meet there (WCS).
struct EdgeSample {
    Vec3 point = Vec3::Zero();
    Vec3 tangent = Vec3::UnitX(); ///< Along the edge (unit).
    /// Each face's outward normal at the point (unit).
    std::array<Vec3, 2> normal{Vec3::UnitZ(), Vec3::UnitY()};
    /// Each face's direction away from the edge, in the face and across the edge (unit).
    std::array<Vec3, 2> inFace{-Vec3::UnitY(), -Vec3::UnitZ()};
};

/// An edge to break, as points along it.
struct BreakEdge {
    std::string name; ///< For warnings ("edge 12").
    std::vector<EdgeSample> samples;
    std::array<int, 2> faces{-1, -1}; ///< Its two faces: indices into the faces' meshes.
    bool closed = false;              ///< The last sample joins the first (a circle).
    /// Not empty: why the edge cannot be broken (it has no second face, say); it is skipped and
    /// listed.
    std::string unusable;
};

struct EdgeBreakParams {
    double width = 0.3;               ///< Width of the break from face to face [mm].
    double tipOffset = 0.3;           ///< Chamfer mills: tip below the break's lower edge [mm].
    std::optional<double> leadLength; ///< [mm], 0 = none; unset: kChamferLeadShare D.
    double gougeTolerance = 0.01;     ///< How far the tool may reach past the break [mm].
};

/// Where the tool stands to break an edge at one of its points.
struct BreakPose {
    Vec3 tip = Vec3::Zero(); ///< Tool tip (WCS), the axis +Z.
    /// The break meets each face this far from the edge, across it [mm].
    std::array<double, 2> faceWidth{0.0, 0.0};
    Vec3 out = Vec3::Zero(); ///< Horizontal direction away from the material (unit, or zero).
    Vec3 planeNormal = Vec3::UnitZ(); ///< The break's normal at its middle (out of the part).
    Vec3 middle = Vec3::Zero();       ///< The middle of the break across the edge.
};

/// The tool's position for the break at one point, or nullopt with `why` set when the tool
/// cannot make it there keeping its axis +Z (an inside corner, too steep an edge for the cone,
/// faces turning under a ball's equator, a break too wide for the tool).
std::optional<BreakPose> breakPose(const tooling::Tool& tool, const EdgeSample& sample,
                                   double width, double tipOffset, std::string* why = nullptr);

/// Breaks each edge in one pass, from its first point to its last, with short horizontal leads
/// where they are clear. Edges whose break cannot be made, or where the tool would cut the part
/// beyond the break, are skipped and listed in a warning. `faces` is the part, a mesh per face:
/// the edge's own two faces are checked against the curved band the break takes from them,
/// the others by drop-cutter against what lies outside the break's prism.
toolpath::Toolpath edgeBreak(const std::vector<BreakEdge>& edges,
                             const std::vector<geom3d::Mesh>& faces, const EdgeBreakParams& params,
                             const Context& ctx);

EdgeBreakParams edgeBreakParams(const Json& values);

namespace detail {
StrategyInfo edgeBreakStrategy();
} // namespace detail

} // namespace ocam::strategies
