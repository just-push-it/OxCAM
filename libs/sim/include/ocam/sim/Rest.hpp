// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Remaining material of an in-process stock as 2D regions, for rest machining (DL-0013).

#pragma once

#include "ocam/sim/Compare.hpp"
#include "ocam/sim/Dexel.hpp"

#include <ocam/geom2d/Region.hpp>

namespace ocam::sim {

/// XY outline of the vertical (Z) rays that hold more than `minLength` of material within
/// (zLow, zHigh]. Each ray stands for the square cell around it, so outlines follow cell edges
/// (staircase polygons on the grid, side = resolution). Diagonally touching cells form separate
/// pieces. Coordinates are the stock's (WCS).
geom2d::Regions materialRegions(const Stock& stock, double zLow, double zHigh,
                                double minLength = 1e-3);

/// XY outline of the vertical rays where a comparison found more than `minExcess` of stock
/// outside the part (3D rest machining).
geom2d::Regions excessRegions(const Stock& stock, const Comparison& comparison, double minExcess);

} // namespace ocam::sim
