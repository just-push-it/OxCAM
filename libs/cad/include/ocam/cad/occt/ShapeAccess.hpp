// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Escape hatch to the underlying OpenCASCADE shape. Including this header requires linking
// OpenCASCADE, so only the GUI viewer and test tooling may use it.

#pragma once

#include "ocam/cad/Model.hpp"

#include <TopoDS_Shape.hxx>

namespace ocam::cad {

const TopoDS_Shape& occtShape(const Model& model);

/// Wraps an existing shape (assumed to be in millimetres) as a Model.
Model modelFromShape(const TopoDS_Shape& shape);

/// The 1-based index (as cad::SubShape numbers them) of a face, edge or vertex of the model,
/// matched on its underlying geometry (a picked sub-shape may carry the presentation's
/// location); 0 when it is not one of the model's.
int subShapeIndex(const Model& model, const TopoDS_Shape& subShape);

} // namespace ocam::cad
