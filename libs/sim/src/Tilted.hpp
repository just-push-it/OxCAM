// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Swept volumes of a tilted tool (phase 9). Each convex slice of the tool (SolidPiece) moving
// along a straight segment with a fixed axis sweeps a convex solid. In the plane of (s, t) - the
// position s along a dexel ray and the move parameter t - the points of that solid on the ray
// form a convex region bounded by one quadric and four lines (the slice's height limits and
// t in [0, 1]); its extent in s is the removed interval, found exactly from the candidate
// extreme points. See docs/design-log/0026.

#pragma once

#include "ocam/sim/ToolShape.hpp"

#include <ocam/geom3d/Bounds.hpp>

#include <vector>

namespace ocam::sim::detail {

/// The interval [lo, hi] of s where the ray `origin + s * e_axis` meets the solid swept by
/// `piece` (tip moving from p0 to p0 + d, tool axis u, unit). False when it misses.
bool sweptSpan(const SolidPiece& piece, const Vec3& p0, const Vec3& d, const Vec3& u,
               const Vec3& origin, int axis, double& lo, double& hi);

/// Bounding box of the pieces with the tip at p and axis u.
geom3d::Aabb3 piecesBounds(const std::vector<SolidPiece>& pieces, const Vec3& p, const Vec3& u);

/// The pieces with every radius reduced by `amount` (collision checks ignore grazing contact).
std::vector<SolidPiece> shrink(const std::vector<SolidPiece>& pieces, double amount);

} // namespace ocam::sim::detail
