// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Probing with a spindle probe (DL-0090): cycles that measure a surface, a web or pocket, a bore
// or boss, or a corner, and set a work offset from it or check it against the model. Each cycle
// is one toolpath::ProbeCycle with its protected positioning and measuring moves
// (MoveKind::Probe); what the control does with the measurement is the post's (docs/posts).

#pragma once

#include "ocam/strategies/Common.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace ocam::strategies {

/// One probing cycle as an operation sets it. Nominal values are in the WCS; see
/// toolpath::ProbeCycle for what `nominal`, `direction` and `size` mean for each feature.
struct ProbeParams {
    toolpath::ProbeFeature feature = toolpath::ProbeFeature::Surface;
    Vec3 nominal = Vec3::Zero();
    Vec3 direction = Vec3::Zero();
    double size = 0.0;
    /// The work offset the measurement sets (1 = G54, ...); 0 for a check. The document layer
    /// resolves the operation's "the setup's own" to the setup's number.
    int workOffset = 0;
    double tolerance = 0.0;   ///< Checks [mm].
    double approach = 0.0;    ///< [mm]
    double overtravel = 0.0;  ///< [mm]
    double inset = 0.0;       ///< Corners: from the corner to each touch, along its face [mm].
    double fastFeed = 0.0;    ///< [mm/s]
    double measureFeed = 0.0; ///< [mm/s]
};

/// True for the probing operation types ("probe_surface", ...).
bool isProbingStrategy(std::string_view id) noexcept;

/// The probing operation types, for strategies().
std::vector<StrategyInfo> probingStrategies();

/// Reads a probing operation's parameters. `workOffset` is the operation's own ("work_offset"),
/// 0 meaning the setup's; the caller resolves that.
ProbeParams probeParams(std::string_view strategy, const Json& values);

/// What a cycle sets in its work offset: "X", "Y", "Z" or "XY" (empty for a check).
std::string probeAxes(const ProbeParams& params);

/// The toolpath of one probing cycle with the probe `ctx.tool`: rapids above the retract height
/// only, protected positioning moves below it, and one measuring move per touch. Throws
/// Error(InvalidArgument) for a tool that is not a probe, a feed of 0 (naming it), or a feature
/// the stylus does not fit.
toolpath::Toolpath probe(const ProbeParams& params, const Context& ctx);

} // namespace ocam::strategies
