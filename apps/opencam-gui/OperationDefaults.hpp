// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The user's starting values for new operations of each type (Edit > Preferences > Operation
// Defaults). Only the parameters the user changed are kept: the others stay unset, so they
// keep following the strategy's defaults and the tool's cutting preset.

#pragma once

#include <ocam/foundation/Json.hpp>

#include <string_view>

namespace ocam::gui {

/// The parameters new operations of `type` start with (settings "operationDefaults/<type>", a
/// JSON object). Values the strategy no longer knows or accepts are left out.
Json operationDefaults(std::string_view type);
/// Keeps `values` (an object; empty removes the type's defaults).
void setOperationDefaults(std::string_view type, const Json& values);

/// Moves earlier versions' settings into the operation defaults: the general "smoothing on in
/// new operations" choice becomes the smoothing default of each type that has smoothing.
void migrateOperationDefaults();

/// Whether `value` suits the parameter `key` of strategy `type` (kind and range).
bool validParameterValue(std::string_view type, const std::string& key, const Json& value);

} // namespace ocam::gui
