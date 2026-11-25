// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <ocam/foundation/Math.hpp>

#include <cstddef>
#include <vector>

namespace ocam::geom2d {

/// Orders points for short rapid travel (for example, drill positions): nearest-neighbour
/// construction from `start`, then 2-opt improvement of the open path. Deterministic for a given
/// input. Returns indices into `points`.
std::vector<std::size_t> orderPoints(const std::vector<Vec2>& points, const Vec2& start,
                                     int maxTwoOptPasses = 50);

/// Length of the open path start -> points[order[0]] -> points[order[1]] -> ...
double pathLength(const std::vector<Vec2>& points, const std::vector<std::size_t>& order,
                  const Vec2& start);

} // namespace ocam::geom2d
