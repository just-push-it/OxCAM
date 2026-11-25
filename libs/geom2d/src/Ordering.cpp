// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/geom2d/Ordering.hpp"

#include <algorithm>
#include <limits>

namespace ocam::geom2d {

std::vector<std::size_t> orderPoints(const std::vector<Vec2>& points, const Vec2& start,
                                     int maxTwoOptPasses) {
    const std::size_t n = points.size();
    std::vector<std::size_t> order;
    order.reserve(n);
    if (n == 0) {
        return order;
    }

    // Nearest neighbour. Ties resolve to the lowest index, keeping results deterministic.
    std::vector<bool> used(n, false);
    Vec2 current = start;
    for (std::size_t k = 0; k < n; ++k) {
        std::size_t best = n;
        double bestDist = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < n; ++i) {
            if (!used[i]) {
                const double d = (points[i] - current).squaredNorm();
                if (d < bestDist) {
                    bestDist = d;
                    best = i;
                }
            }
        }
        used[best] = true;
        order.push_back(best);
        current = points[best];
    }

    // 2-opt on the open path with a fixed start: reversing order[i..j] replaces edges
    // (prev(i), i) and (j, next(j)); the last point has no outgoing edge.
    auto at = [&](std::ptrdiff_t k) -> const Vec2& {
        return k < 0 ? start : points[order[static_cast<std::size_t>(k)]];
    };
    const auto last = static_cast<std::ptrdiff_t>(n) - 1;
    for (int pass = 0; pass < maxTwoOptPasses; ++pass) {
        bool improved = false;
        for (std::ptrdiff_t i = 0; i < last; ++i) {
            for (std::ptrdiff_t j = i + 1; j <= last; ++j) {
                const Vec2& a = at(i - 1);
                const Vec2& b = at(i);
                const Vec2& c = at(j);
                double before = (b - a).norm();
                double after = (c - a).norm();
                if (j < last) {
                    const Vec2& d = at(j + 1);
                    before += (d - c).norm();
                    after += (d - b).norm();
                }
                if (after < before - 1e-12) {
                    std::reverse(order.begin() + i, order.begin() + j + 1);
                    improved = true;
                }
            }
        }
        if (!improved) {
            break;
        }
    }
    return order;
}

double pathLength(const std::vector<Vec2>& points, const std::vector<std::size_t>& order,
                  const Vec2& start) {
    double len = 0.0;
    Vec2 current = start;
    for (std::size_t i : order) {
        len += (points[i] - current).norm();
        current = points[i];
    }
    return len;
}

} // namespace ocam::geom2d
