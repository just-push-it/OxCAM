// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A setup's work coordinate system from what the setup page chooses (DL-0038): its axes from the
// part's own or from picked faces and edges (WcsOrientation), its origin at a point of the
// stock's, the part's or the fixtures' box, or at a picked point, moved by an offset along the
// axes (WcsOrigin). Document::modify keeps the origin there after every change.

#pragma once

#include "ocam/document/Project.hpp"

#include <ocam/geom3d/Bounds.hpp>

#include <optional>

namespace ocam::document {

class Document;

/// The WCS axes of a part orientation, with `current`'s origin: the project's axes for the
/// part's own; else Z from its pick and X (or Y) from the other, made square to Z, each flipped
/// as asked. A pick not made yet keeps the current axis (the other made square to it). Throws
/// Error(InvalidArgument) with a message for the user when the two picks are parallel.
Frame orientedAxes(const WcsOrientation& orientation, const Frame& current);

/// Sets the setup's part orientation and its WCS axes (orientedAxes). `onWorkholding`: the pick
/// that changed is on the workholding, whose directions follow the WCS (it is placed along its
/// axes): the axes turn about Z by the quarter turns the pick asks, and the workholding turns
/// back by as much (SetupWorkholding::rotation), so it stays where it is around the part. Throws
/// Error(InvalidArgument) with a message for the user when such a pick would tilt Z or turn by
/// other than quarter turns, and like orientedAxes.
void setOrientation(Setup& setup, const WcsOrientation& orientation, bool onWorkholding);

/// The direction a face or edge of the part or a fixture model gives an axis, project frame:
/// a flat face's outward normal, a round face's or a circular edge's axis, a straight edge's
/// direction. Throws Error(InvalidArgument) for other faces and edges, vertices, or a model
/// that is not loaded.
Vec3 axisOfShape(const Document& doc, const ShapeRef& shape);

/// The point a face, edge or vertex of the part or a fixture model gives an origin, project
/// frame: a flat face's centre, a round face's axis (nearest its centre), a circle's centre, a
/// straight edge's middle, a vertex. Throws Error(InvalidArgument) when the model is not loaded
/// or the face or edge is no longer found.
Vec3 pointOfShape(const Document& doc, const ShapeRef& shape);

/// A reference box of the setup along its WCS axes (Origin from): per axis, the extent of the
/// stock, the part, or the fixtures (fixture models and the placed workholding together),
/// measured along the axis from the project origin. Throws Error(InvalidArgument) with a message
/// for the user when there is no such box (no model, no fixture, a stock that cannot be
/// resolved) or `from` is Point.
geom3d::Aabb3 originBox(const Document& doc, const Setup& setup, OriginFrom from);

/// The point an origin is measured from (its offset not added), project frame. Throws like
/// originBox, pointOfShape and workholdingPoint, or Error(InvalidArgument) when a Point origin has
/// no point.
Vec3 originReference(const Document& doc, const Setup& setup, const WcsOrigin& origin);

/// The setup's WCS origin from Setup::originFrom (the reference plus the offset along the WCS
/// axes), project frame; nullopt without one. Throws like originReference.
std::optional<Vec3> resolveWcsOrigin(const Document& doc, const Setup& setup);

} // namespace ocam::document
