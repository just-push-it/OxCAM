// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Smoothing of finished toolpaths: runs of short straight moves that lie in one of the arc
// planes (XY, ZX, YZ) become arcs and longer lines, within a tolerance. Any strategy can apply it
// to its output; the controller then gets G2/G3 instead of a stream of short G1 moves, which
// leaves a faceted surface on curved areas. See docs/design-log/0032-arc-smoothing.md.

#pragma once

#include <ocam/toolpath/Toolpath.hpp>

#include <cstddef>

namespace ocam::toolpath {

struct SmoothingResult {
    std::size_t arcs = 0;        ///< Arc moves produced.
    std::size_t movesBefore = 0; ///< Moves in the path before.
    std::size_t movesAfter = 0;  ///< And after.
};

/// Replaces runs of consecutive straight feed moves (the same feed class and rate, 3-axis, not
/// threading) lying in the XY, ZX or YZ plane by arcs (at most 180 degrees each) and lines whose
/// points stay within `tolerance` of the original ones. Moves of other kinds, and runs in no such
/// plane (a raster at an oblique angle), are kept as they are.
SmoothingResult fitArcs(Toolpath& path, double tolerance);

/// Replaces every arc move by straight moves within `chordTolerance` of it (for controls without
/// circular interpolation, or when smoothing is turned off). Returns the number of arcs replaced.
std::size_t linearizeArcs(Toolpath& path, double chordTolerance);

} // namespace ocam::toolpath
