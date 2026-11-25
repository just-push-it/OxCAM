// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Tool holder outlines from solid models (vendor STEP files): the smallest solid of revolution
// about the holder's axis that contains the model, as a (radius, height) outline measured from
// the tool end. See docs/design-log/0034.

#pragma once

#include "ocam/cad/Model.hpp"

#include <ocam/geom2d/Polyline.hpp>

namespace ocam::cad {

/// The model axis the holder turns about.
enum class RevolveAxis { Auto, X, Y, Z };
/// Which end of the model along the axis the tool comes out of.
enum class ToolEnd { Auto, Low, High };

struct HolderOutlineOptions {
    /// Auto: the model axis across which the model is roundest (its two other extents most
    /// nearly equal), the longest of those that are.
    RevolveAxis axis = RevolveAxis::Auto;
    /// Auto: the end farther from the flange (the widest part).
    ToolEnd toolEnd = ToolEnd::Auto;
    /// Height (from the tool end) where the outline stops: the gauge line. Everything above it
    /// sits in the spindle. 0 = the suggested gauge line.
    double length = 0.0;
    /// The outline lies outside the model by at most this much [mm].
    double tolerance = 0.02;
};

struct HolderOutline {
    /// Open polyline from the axis at height 0 (the tool end) to the axis at `length`, radii
    /// never smaller than the model's at any height.
    geom2d::Polyline profile;
    RevolveAxis axis = RevolveAxis::Z; ///< As used (never Auto).
    ToolEnd toolEnd = ToolEnd::Low;    ///< As used (never Auto).
    double fullLength = 0.0;           ///< The model's extent along the axis.
    double suggestedLength = 0.0;      ///< The top of the flange: the usual gauge line.
    double length = 0.0;               ///< Where the outline stops.
    /// How far the model is from round about the axis: the difference of its two other
    /// extents over the larger (0 = round).
    double asymmetry = 0.0;
};

/// The revolved outline of a holder model. The axis passes through the centre of the model's
/// bounding box. Throws Error(InvalidArgument) for a null or empty model.
HolderOutline holderOutline(const Model& model, const HolderOutlineOptions& options = {});

/// The solid a closed (z, radius) polygon sweeps about the Z axis; points on the axis are allowed.
Model revolvedAboutZ(const std::vector<Vec2>& profile);

} // namespace ocam::cad
