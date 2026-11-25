// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/geom2d/Region.hpp"

#include <algorithm>
#include <numeric>

namespace ocam::geom2d {

double Region::area() const noexcept {
    double a = outer.area();
    for (const auto& h : holes) {
        a -= h.area();
    }
    return a;
}

bool Region::contains(const Vec2& p, double chordTolerance) const {
    if (!outer.contains(p, chordTolerance)) {
        return false;
    }
    return std::none_of(holes.begin(), holes.end(),
                        [&](const Polyline& h) { return h.contains(p, chordTolerance); });
}

double totalArea(const Regions& regions) noexcept {
    double a = 0.0;
    for (const auto& r : regions) {
        a += r.area();
    }
    return a;
}

Regions buildRegions(std::vector<Polyline> loops, double chordTolerance) {
    std::erase_if(loops, [](const Polyline& l) {
        return !l.isClosed() || l.vertices().size() < 2 || l.area() <= 0.0;
    });

    // Largest first: a loop's container is always earlier in this order.
    std::vector<std::size_t> order(loops.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::vector<double> areas(loops.size());
    for (std::size_t i = 0; i < loops.size(); ++i) {
        areas[i] = loops[i].area();
    }
    std::stable_sort(order.begin(), order.end(),
                     [&](std::size_t a, std::size_t b) { return areas[a] > areas[b]; });

    // Sample point strictly on each loop: the midpoint of its first segment. Loops of a valid
    // section do not touch, so this point is inside exactly the loops that contain the loop.
    auto samplePoint = [](const Polyline& l) {
        const auto& v = l.vertices();
        if (auto arc = arcFromBulge(v[0].p, v[1 % v.size()].p, v[0].bulge)) {
            return arc->pointAt(0.5);
        }
        return Vec2(0.5 * (v[0].p + v[1 % v.size()].p));
    };

    std::vector<int> depth(loops.size(), 0);
    std::vector<std::ptrdiff_t> parent(loops.size(), -1);
    for (std::size_t k = 0; k < order.size(); ++k) {
        const std::size_t i = order[k];
        const Vec2 sample = samplePoint(loops[i]);
        // The innermost container is the smallest earlier loop that contains the sample.
        for (std::size_t m = k; m-- > 0;) {
            const std::size_t j = order[m];
            if (loops[j].bounds().contains(sample) && loops[j].contains(sample, chordTolerance)) {
                parent[i] = static_cast<std::ptrdiff_t>(j);
                depth[i] = depth[j] + 1;
                break;
            }
        }
    }

    Regions regions;
    std::vector<std::ptrdiff_t> regionOf(loops.size(), -1);
    for (std::size_t i : order) {
        if (depth[i] % 2 == 0) {
            if (!loops[i].isCounterClockwise()) {
                loops[i].reverse();
            }
            regionOf[i] = static_cast<std::ptrdiff_t>(regions.size());
            regions.push_back(Region{std::move(loops[i]), {}});
        }
    }
    for (std::size_t i : order) {
        if (depth[i] % 2 == 1) {
            if (loops[i].isCounterClockwise()) {
                loops[i].reverse();
            }
            const auto owner = regionOf[static_cast<std::size_t>(parent[i])];
            regions[static_cast<std::size_t>(owner)].holes.push_back(std::move(loops[i]));
        }
    }
    return regions;
}

} // namespace ocam::geom2d
