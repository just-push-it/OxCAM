// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Back boring and back chamfering (DL-0083): machining the far side of a through hole from the
// near side, either with a back-boring bar that passes the hole off centre with its spindle
// oriented (a G87-style cycle), or with a tool small enough to pass the hole that interpolates
// circles on the far edge.

#pragma once

#include "ocam/strategies/Strategies.hpp"

namespace ocam::strategies {

enum class BackBoreMethod {
    Oriented,     ///< Back-boring bar: oriented spindle, shifted through, cut upward (cycle).
    Interpolated, ///< A tool that passes the hole: circles (a helix up) on the far side.
};

enum class BackFeature {
    Counterbore, ///< A counterbore or spot face on the far side.
    Chamfer,     ///< A chamfer on the far edge.
};

struct BackBoreParams {
    BackBoreMethod method = BackBoreMethod::Oriented;
    BackFeature feature = BackFeature::Counterbore;
    double depth = 2.0;        ///< Counterbore: its depth above the far side [mm].
    double diameter = 0.0;     ///< Interpolated counterbore: its diameter [mm]; 0 = not set.
    double chamferWidth = 0.5; ///< Chamfer: its width across the far side [mm].
    /// Interpolated chamfer: its angle from the hole's axis [rad] (the oriented method cuts the
    /// cutter's own angle).
    double chamferAngle = kPi / 4.0;
    double shift = 0.0;      ///< Oriented: the tool's shift off the hole's axis [mm].
    double shiftAngle = kPi; ///< Oriented: the shift's direction in the WCS XY plane [rad].
    double orient = 0.0;     ///< Oriented: the spindle's orientation angle [rad].
    /// How far below the far side the tool's head comes before it moves out or starts [mm].
    double clearanceBelow = 2.0;
    double dwell = 0.0;      ///< Oriented: at the top of the cut [s].
    double helixPitch = 1.0; ///< Interpolated counterbore: rise per turn [mm].
    double retractClearance = 2.0;
    std::optional<double> rPlaneZ; ///< See DrillParams.
    CutDirection direction = CutDirection::Climb;
};

/// Machines the far side of each through hole. Refuses (Error(InvalidArgument)) when the tool
/// cannot pass a hole or reach through it; holes that are not through holes are skipped with a
/// warning.
toolpath::Toolpath backBore(const std::vector<HoleTarget>& holes, const BackBoreParams& params,
                            const Context& ctx);

/// The operation's parameters (the shift from the tool where unset; the preset gives nothing
/// beyond the feeds).
BackBoreParams backBoreParams(const Json& values, const tooling::Tool& tool,
                              const tooling::CuttingPreset* preset);

/// The radius of the part of a back-boring tool that passes through the hole: its neck, or
/// without one its shank (the tool's own diameter when it has neither).
double backBoreNeckRadius(const tooling::Tool& tool);

namespace detail {
StrategyInfo backBoreStrategy();
} // namespace detail

} // namespace ocam::strategies
