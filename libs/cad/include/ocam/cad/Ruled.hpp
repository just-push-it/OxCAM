// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The rulings of a face, from its exact surface, for swarf (flank) milling: the straight lines
// the side of a tool can lie along. See docs/design-log/0080.

#pragma once

#include "ocam/cad/Model.hpp"

#include <ocam/foundation/Math.hpp>

#include <string>
#include <vector>

namespace ocam::cad {

/// One ruling of a face: points along it, the first at its bottom (lowest along `up`), with the
/// face's outward unit normals there.
struct FaceRuling {
    std::vector<Vec3> points;
    std::vector<Vec3> normals;
    Vec3 direction = Vec3::UnitZ(); ///< Unit, bottom to top: the line's own direction.
};

struct FaceRulings {
    /// Across the face, in order; empty when the face is not ruled within the tolerance (see
    /// `problem`).
    std::vector<FaceRuling> rulings;
    /// How far the face strays from the straight lines between its rulings' ends [mm]: 0 on a
    /// plane, a cone or a ruled spline surface; the bow of a curved wall otherwise.
    double straightness = 0.0;
    bool planar = false;
    /// Why the face has no rulings (not ruled, a hole across its rulings, horizontal), else empty.
    std::string problem;
};

struct RulingOptions {
    Vec3 up = Vec3::UnitZ(); ///< In the frame `toFrame` maps to: the bottom of a ruling is lower.
    /// The face may stray this far from straight rulings (the swarf tolerance) [mm].
    double tolerance = 0.01;
    /// Rulings are close enough that points offset along their normals by this distance (the
    /// tool radius) stay within `chordTolerance` of the straight moves between them.
    double offset = 0.0;
    double chordTolerance = 0.002;
    double maxSpacing = 2.0; ///< Largest distance between neighbouring rulings [mm].
    int samples = 9;         ///< Points along each ruling (at least 2).
};

/// The rulings of a face (1-based index), placed by `toFrame`. A plane is ruled along its
/// steepest direction along `up`; other surfaces along the parameter lines that are straight
/// within the tolerance (the ones nearer `up` when both are). Rulings are clipped to the face.
FaceRulings faceRulings(const Model& model, int faceIndex, const Transform3& toFrame,
                        const RulingOptions& options);

} // namespace ocam::cad
