// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Obstacle avoidance for finished 3-axis toolpaths (fixtures: vises, clamps, plates). The tool,
// its shank and its holder keep a clearance from a triangle mesh: feed moves that would come
// closer are left out, the tool lifts over the obstacle and plunges back where the cut can go
// on, and rapids that would pass through it are raised over it. See docs/design-log/0034.

#pragma once

#include "ocam/strategies/Common.hpp"

#include <ocam/geom3d/Mesh.hpp>

#include <memory>
#include <string>

namespace ocam::strategies {

/// The lowest tool tip height at an XY position that keeps a rotating tool and its holder a
/// clearance away from an obstacle mesh (tool axis +Z). The tool assembly is a stack of solids
/// that each reach upward without end: the cutting part (the exact lower surface of flat, ball
/// and bull-nose end mills; a cylinder of the largest cutting radius otherwise), then a cylinder
/// wherever the shank or the holder grows wider, all grown by the clearance. Thread-safe.
class ObstacleField {
public:
    /// `holder` may be null (the tool's own length only). Throws Error(InvalidArgument) for
    /// turning tools and tools whose geometry is invalid.
    ObstacleField(const geom3d::Mesh& obstacles, const tooling::Tool& tool,
                  const tooling::Holder* holder, double clearance);
    ~ObstacleField();
    ObstacleField(ObstacleField&&) noexcept;
    ObstacleField& operator=(ObstacleField&&) noexcept;

    /// Lowest safe tip height at `xy`; -infinity where nothing is within reach.
    double safeHeight(const Vec2& xy) const;
    /// No tip position above this height comes near the obstacles.
    double ceiling() const noexcept;
    /// Largest radius of the assembly with the clearance: obstacles farther away in XY than
    /// this never matter.
    double reach() const noexcept;
    /// Bounds of the obstacle mesh.
    const geom3d::Aabb3& bounds() const noexcept;
    /// Radius of the cutting part plus the clearance.
    double cutterReach() const noexcept;
    /// Number of solids the assembly is modelled with.
    std::size_t pieceCount() const noexcept;
    bool isEmpty() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

struct AvoidanceParams {
    double retract = 5.0;    ///< Lowest level of the links over the obstacles (WCS Z).
    double plungeFeed = 5.0; ///< Feed back down to the cut after a lift [mm/s].
    double chordTolerance = 0.002;
    std::string name = "fixtures"; ///< What the obstacles are, for the warnings.
    double clearance = 0.0;        ///< The field's clearance, for the warnings [mm].
};

struct AvoidanceResult {
    std::size_t trimmedSections = 0; ///< Stretches of feed moves left out.
    double trimmedLength = 0.0;      ///< Their length [mm].
    std::size_t raisedRapids = 0;    ///< Rapids routed over the obstacles.
    std::size_t skippedHoles = 0;    ///< Drilling cycles left out.

    bool changed() const noexcept {
        return trimmedSections > 0 || raisedRapids > 0 || skippedHoles > 0;
    }
};

/// Keeps a 3-axis toolpath (WCS, tool axis +Z) clear of the field's obstacles:
///  - feed moves (lines and arcs; arcs that need trimming become lines) are cut where the tool
///    would come too close; the tool rapids up to max(retract, what the obstacle needs), across
///    and down again (rapid to the retract height, then at `plungeFeed`) to where the cut can
///    go on;
///  - rapids that would come too close go up, across and down over the obstacle instead;
///  - drilling cycles whose hole the tool cannot reach are left out.
/// Warnings name what was changed. Throws Error(InvalidArgument) for multi-axis toolpaths and
/// OperationCancelled when cancelled.
AvoidanceResult avoidObstacles(toolpath::Toolpath& path, const ObstacleField& field,
                               const AvoidanceParams& params, CancelToken& cancel);

} // namespace ocam::strategies
