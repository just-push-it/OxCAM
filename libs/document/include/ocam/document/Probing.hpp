// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Probing operations in a project (DL-0090): the nominal values that the faces picked on the
// model give a probing cycle, and its toolpath.

#pragma once

#include "ocam/document/Document.hpp"

#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Progress.hpp>
#include <ocam/toolpath/Toolpath.hpp>

namespace ocam::document {

/// The parameter values that the operation's picked faces give it, in the setup's WCS: the
/// feature ("feature": surface, web, pocket, bore, boss, outside or inside corner), its
/// direction ("direction", "axis" or "corner"), "x", "y", "z", and "width" or "diameter". One
/// planar face is a surface, two parallel ones a web or pocket, two square to each other a
/// corner, one cylindrical face a bore or boss. Throws ocam::Error with a user-facing message
/// when the faces are none of those, or not what the operation type measures.
Json probeNominals(const Document& doc, const Setup& setup, const Operation& op);

/// `op.parameters` with probeNominals() applied: what picking faces on a probing operation
/// sets. Values the faces do not give are kept.
Json withProbeNominals(const Document& doc, const Setup& setup, const Operation& op);

/// The toolpath of a probing operation with its probe `tool`, in the setup's WCS. Its speeds
/// and feeds are the operation's (no cutting preset). Throws ocam::Error with a user-facing
/// message when it cannot be generated.
toolpath::Toolpath generateProbing(const Document& doc, const Setup& setup, const Operation& op,
                                   const tooling::Tool& tool, CancelToken& cancel,
                                   ProgressSink& progress);

} // namespace ocam::document
