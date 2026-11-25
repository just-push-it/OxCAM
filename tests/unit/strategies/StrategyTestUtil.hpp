// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <ocam/foundation/Json.hpp>
#include <ocam/geom2d/Clip.hpp>
#include <ocam/strategies/Strategies.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <filesystem>
#include <memory>

namespace ocam::strategies::testing {

/// Owns the cancel token and progress sink a Context refers to.
struct Harness {
    tooling::Tool tool;
    CancelToken cancel;
    NullProgress progress;
    Feeds feeds;
    Heights heights{15.0, 5.0, 1.0};

    explicit Harness(tooling::Tool t) : tool(std::move(t)) {
        feeds.cut = 20.0;
        feeds.plunge = 5.0;
        feeds.ramp = 10.0;
        feeds.lead = 15.0;
        feeds.spindle = 150.0;
    }
    Context context() { return Context{tool, feeds, heights, 0.002, cancel, progress}; }
};

/// Feed moves at height z, as XY paths (arcs sampled).
std::vector<std::vector<Vec2>> cutPathsAt(const toolpath::Toolpath& tp, double z,
                                          double tol = 0.002);

/// Every sampled tool-tip position of feed moves (cycles expanded).
std::vector<Vec3> feedPoints(const toolpath::Toolpath& tp, double tol = 0.002);

/// Compares against tests/golden/<name>.json; with OCAM_UPDATE_GOLDEN=1 rewrites the file.
void checkGolden(const std::string& name, const toolpath::Toolpath& tp);

inline geom2d::Region rect(double x0, double y0, double x1, double y1) {
    return {geom2d::Polyline::rectangle({x0, y0}, {x1, y1}), {}};
}

} // namespace ocam::strategies::testing
