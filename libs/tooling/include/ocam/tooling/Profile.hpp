// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Canonical revolved profile of a rotating tool. Simulation, gouge and collision checks consume
// only this profile, so a new tool type needs a profile generator and nothing else.

#pragma once

#include "ocam/tooling/Tool.hpp"

#include <ocam/geom2d/Polyline.hpp>

#include <string_view>
#include <vector>

namespace ocam::tooling {

struct ToolProfile {
    /// Open polyline in the (radius, height) half-plane: starts on the axis at the tip (or at
    /// radius > 0 for tools with a flat tip), runs outward and upward, and ends on the axis at
    /// the top of the shank. Arcs are exact (bulges).
    geom2d::Polyline outline;
    double cuttingLength = 0.0; ///< Material can be removed below this height.
    double maxRadius = 0.0;     ///< Largest radius of the cutting part.
    double length = 0.0;        ///< Overall length.
};

/// Builds the profile of a rotating tool. Throws ocam::Error(InvalidArgument) when the geometry
/// is invalid (see Tool::validate) and Error(Unsupported) for turning tools.
ToolProfile toolProfile(const Tool& tool);

/// Radius of the profile at height z (largest radius at that height), 0 outside [0, length].
double profileRadiusAt(const ToolProfile& profile, double z);

// -- Turning ------------------------------------------------------------------------------------

/// Where a turning tool cuts. A face tool (grooving inserts only) plunges along -Z into a front
/// face, its programmed point on the radially inner corner of its width.
enum class TurningSide { Outside, Inside, Face };
/// The side a toolpath's tip quadrant says it cuts on (nose-radius convention: 2 inside, 8 along
/// -Z, the rest outside).
TurningSide turningSideOf(int tipQuadrant) noexcept;
/// Whether a tip quadrant says the passes run away from the chuck, with the tool mirrored in Z
/// (quadrants 1 inside and 4 outside).
bool turningAwayFromChuck(int tipQuadrant) noexcept;
/// An outline mirrored in Z (its x), still counter-clockwise.
geom2d::Polyline mirroredInZ(const geom2d::Polyline& outline);

/// An insert shape letter of ISO 1832 (the same letters as ISO 5608).
struct InsertShape {
    char letter = 'C';
    double includedAngle = 0.0; ///< The smaller corner angle [degrees]; 0 for round inserts.
    const char* name = "";      ///< In English, for the "Tooling" translations.
};
const std::vector<InsertShape>& insertShapes();
/// The shape of an insert's shape text (its first letter, either case); nullptr when unknown.
const InsertShape* findInsertShape(std::string_view shape);

/// The cutting end of a turning insert in the lathe plane (u = axial Z, v = radius), as a closed
/// convex polygon relative to the programmed point. Outside (OD) tools point towards -Z/-X with
/// the programmed point at the imaginary (sharp) tip, i.e. the nose centre at (r_n, r_n); inside
/// tools are mirrored in the radius. Grooving and parting inserts are a rectangle of the insert
/// width whose programmed point is the left (-Z) bottom corner; threading inserts are a 60 degree
/// form with the programmed point at the tip. See docs/design-log/0020.
struct TurningShape {
    geom2d::Polyline outline;
    double noseRadius = 0.0;
    double width = 0.0;         ///< Grooving and parting inserts.
    double leadAngle = 0.0;     ///< Angle of the leading (side) edge from +Z [rad] (OD tools).
    double trailingAngle = 0.0; ///< Angle of the trailing (end) edge from +Z [rad]: the steepest
                                ///< descent towards -Z the insert clears.
};

/// Throws Error(InvalidArgument) for tools that are not turning inserts.
TurningShape turningShape(const Tool& tool, TurningSide side = TurningSide::Outside);

/// The silhouette of a drill, spot or centre drill, tap, reamer or boring bar standing still on a
/// lathe's spindle axis, over its cutting length: closed and convex like an insert's outline,
/// relative to the tool's tip, x along the axis towards the shank and y the radius on both sides
/// of the axis. A centre drill's shoulder between pilot and countersink is bridged.
geom2d::Polyline holeToolOutline(const Tool& tool);
/// One side of a hole tool: (distance from the tip, radius), from the tip back.
std::vector<Vec2> holeToolSide(const Tool& tool);

} // namespace ocam::tooling
