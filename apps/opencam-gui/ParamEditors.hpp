// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Editors for strategy parameters, built from their descriptors. The operation page and the
// operation defaults of the preferences use the same editors.

#pragma once

#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/strategies/Common.hpp>

#include <functional>

class QWidget;

namespace ocam::gui {

/// An editor for one parameter showing `current` (the descriptor's default when null, or the
/// value it takes from `tool`; "From the tool" without one), in `unit` for lengths and surface
/// speeds. `changed` receives the new value as stored in the parameter object (mm, degrees,
/// m/min).
QWidget* createParamEditor(const strategies::ParamDef& def, const Json* current, LengthUnit unit,
                           std::function<void(const Json&)> changed,
                           const tooling::Tool* tool = nullptr);

} // namespace ocam::gui
