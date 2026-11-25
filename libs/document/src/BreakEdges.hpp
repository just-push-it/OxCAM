// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The input of 3D edge breaking (DL-0083): an operation's selected edges, sampled with their two
// faces, and the models they belong to as one mesh, in the setup's WCS.

#pragma once

#include "ocam/document/Document.hpp"

#include <ocam/geom3d/Mesh.hpp>
#include <ocam/strategies/EdgeBreak.hpp>

#include <vector>

namespace ocam::document {

struct BreakEdgesInput {
    std::vector<strategies::BreakEdge> edges;
    /// Every face of every model an edge is on, in the WCS (BreakEdge::faces index them).
    std::vector<geom3d::Mesh> faces;
};

/// Throws Error(InvalidArgument) when nothing is selected or a selection no longer exists.
BreakEdgesInput breakEdgesInput(const Document& doc, const Setup& setup, const Operation& op,
                                double spacing, double deflection);

} // namespace ocam::document
