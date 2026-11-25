// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Polygon offsetting and boolean operations on Regions (backed by Clipper2). Inputs are
// discretized within `chordTolerance`; outputs are straight-segment polylines. Arc fitting of the
// results is a separate step (see the design log for biarc fitting, phase 3).

#pragma once

#include "ocam/geom2d/Region.hpp"

namespace ocam::geom2d {

enum class JoinStyle { Round, Miter, Square };

struct ClipOptions {
    double chordTolerance = 1e-3; ///< Arc discretization and round-join tolerance [mm].
    JoinStyle join = JoinStyle::Round;
    double miterLimit = 2.0; ///< Multiple of |delta|, for JoinStyle::Miter.
};

/// Grows (delta > 0) or shrinks (delta < 0) regions. Holes shrink when regions grow. Regions
/// that vanish are removed; regions that split produce several outputs.
Regions offset(const Regions& regions, double delta, const ClipOptions& options = {});

/// Area swept by a disk of `radius` moving along open point paths (round ends). Used to verify
/// toolpath coverage: material minus the swept area is what a path leaves behind.
Regions sweep(const std::vector<std::vector<Vec2>>& paths, double radius,
              const ClipOptions& options = {});

/// The parts of open point paths inside (`inside`) or outside the regions, each in the
/// direction of the path it came from. A closed loop is passed with its first point repeated
/// at the end; a loop that is not cut comes back the same way.
std::vector<std::vector<Vec2>> clipPaths(const std::vector<std::vector<Vec2>>& paths,
                                         const Regions& regions, bool inside,
                                         const ClipOptions& options = {});

Regions unite(const Regions& a, const Regions& b, const ClipOptions& options = {});
Regions intersect(const Regions& a, const Regions& b, const ClipOptions& options = {});
Regions subtract(const Regions& a, const Regions& b, const ClipOptions& options = {});

} // namespace ocam::geom2d
