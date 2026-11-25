// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A setup simulation as bytes, for the local simulation cache (ADR-0003): reopening a project
// shows its in-process stock without simulating it again. The bytes hold everything playback,
// the in-process stock, collisions and the comparison show, under the simulation's input key
// (simulationInputKey), compressed. They are never part of a project: the cache lives on the
// user's machine, and any change to the inputs or to the program misses it.

#pragma once

#include "ocam/document/Simulate.hpp"

#include <optional>
#include <string>
#include <string_view>

namespace ocam::document {

/// The format, and the simulation's results: raise it whenever simulations of the same inputs
/// come out differently (a fix in the removal, the checks or the comparison), so simulations
/// cached by earlier builds are not shown (as kToolpathRevision for toolpaths).
inline constexpr int kSimulationCacheVersion = 14;

/// The simulation's bytes under `key` (its simulationInputKey).
std::string encodeSimulation(const SetupSimulation& simulation, const std::string& key);

/// A simulation of `setup` in `doc` from bytes written by encodeSimulation: nullopt when they
/// were written under another key or format version, or are damaged. What is cheap to derive
/// comes from the document: the machine, the tools' shapes and a turned part's half profile.
std::optional<SetupSimulation> decodeSimulation(std::string_view bytes, const std::string& key,
                                                const Document& doc, const Setup& setup);

} // namespace ocam::document
