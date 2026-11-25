// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Iso-contours of a scalar field sampled on a regular grid (marching squares with linear
// interpolation), as regions.

#pragma once

#include "ocam/geom2d/Region.hpp"

#include <functional>
#include <vector>

namespace ocam::geom2d {

/// A scalar field sampled at origin + (i * spacing.x, j * spacing.y), i < nx, j < ny; value
/// (i, j) is values[j * nx + i].
struct ScalarGrid {
    Vec2 origin = Vec2::Zero();
    Vec2 spacing = Vec2::Ones();
    int nx = 0;
    int ny = 0;
    std::vector<double> values;

    double at(int i, int j) const {
        return values[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) +
                      static_cast<std::size_t>(i)];
    }
    Vec2 point(int i, int j) const { return origin + Vec2(i * spacing.x(), j * spacing.y()); }
};

/// The regions where the field exceeds `level` (outside the grid counts as below it). Saddle
/// cells are resolved by the cell's mean value. Outer boundaries counter-clockwise, holes
/// clockwise.
Regions isoRegions(const ScalarGrid& grid, double level);

/// As isoRegions, with crossings located on an exact predicate (`inside(p)`: the field exceeds
/// the level at p) by bisection to `tolerance`, on the outside of the boundary; chords that cut
/// into the outside get the boundary point found across them. For fields that jump (walls), where
/// linear interpolation misplaces the boundary by up to half a cell.
Regions isoRegions(const ScalarGrid& grid, double level,
                   const std::function<bool(const Vec2&)>& inside, double tolerance);

} // namespace ocam::geom2d
