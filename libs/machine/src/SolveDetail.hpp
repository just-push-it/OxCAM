// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "ocam/machine/Machine.hpp"

namespace ocam::machine::detail {

/// Checks the motion span of `result.path` against the linear axis travel, and its spindle speed
/// and feed rates against the machine limits, adding errors to `result`. It never changes a
/// speed or feed (ADR-0004).
void applyLimits(const Machine& machine, SolveResult& result);

} // namespace ocam::machine::detail
