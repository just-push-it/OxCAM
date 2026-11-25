// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Turns a project operation into a toolpath: resolves its tool, cutting preset, stock and
// geometry references into the setup's WCS, then runs the strategy.

#pragma once

#include "ocam/document/Document.hpp"

#include <ocam/geom2d/Region.hpp>
#include <ocam/sim/FixtureCheck.hpp>
#include <ocam/sim/Lathe.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <memory>
#include <string>

namespace ocam::document {

/// Generates the toolpath of `op` (in `setup`), in the setup's WCS. Throws ocam::Error with a
/// user-facing message when the operation cannot be generated (no tool, no geometry, unsuitable
/// faces, a tool orientation the setup's machine cannot reach, ...), and OperationCancelled
/// when cancelled. Thread-safe as long as the Document is not modified.
toolpath::Toolpath generateToolpath(const Document& doc, const Setup& setup, const Operation& op,
                                    CancelToken& cancel, ProgressSink& progress);

/// The setup as an operation sees it: with a tool orientation of its own (Operation::
/// orientation) its WCS is the operation's frame, else the setup itself. The operation's
/// geometry, stock, heights and toolpath are measured in this frame.
Setup operationSetup(const Setup& setup, const Operation& op);

/// The toolpath of `op` as generated, in its own frame (operationSetup's WCS; with the tool
/// along its Z axis, arcs and canned cycles as the post needs them for 3+2). Equals
/// generateToolpath without a tool orientation.
toolpath::Toolpath generateToolpathInFrame(const Document& doc, const Setup& setup,
                                           const Operation& op, CancelToken& cancel,
                                           ProgressSink& progress);

/// A toolpath in the operation's frame mapped into the setup's WCS (tilted tool axis; arcs out
/// of the WCS planes become lines and canned cycles are expanded).
toolpath::Toolpath toolpathInWcs(const toolpath::Toolpath& inFrame, const Operation& op);

/// The heights an operation uses, in the setup's WCS [mm] (see OperationHeights). Unset
/// feed, top and bottom mean the strategy's own: feed moves starting 1 mm above the uncut
/// material (an R plane above each hole), the stock top, the selected floor and so on.
struct OperationHeightValues {
    double clearance = 0.0;
    double retract = 0.0;
    /// Where feed moves into the operation's top start; unset for hole operations whose feed
    /// height is measured from each hole (holeFeed).
    std::optional<double> feed;
    /// Hole operations: the feed height (R plane) above each hole's top [mm].
    std::optional<double> holeFeed;
    std::optional<double> top;         ///< Unset where the operation type uses no top height.
    std::optional<double> bottom;      ///< Unset where the operation type uses no bottom height.
    std::vector<std::string> warnings; ///< Heights that were raised to stay safe.
};

/// Resolves the operation's height settings, in the operation's frame (operationSetup). Throws
/// ocam::Error with a user-facing message when a reference is not available (no model for "top of
/// part", no selected faces, ...).
OperationHeightValues operationHeights(const Document& doc, const Setup& setup,
                                       const Operation& op);

/// Where a chamfer operation is meant to cut the part (DL-0014): within its width of the
/// selected edges, from their height down by the chamfer's depth. The part comparison does not
/// count the part material a chamfer removes there as a gouge: chamfers are seldom modelled.
/// A trace (DL-0088) is the same: an engraving's groove within the tool's reach of its edges,
/// down by its depth.
struct ChamferZone {
    geom2d::Regions outline; ///< The selected edges at one height (setup WCS).
    double z = 0.0;          ///< Their height.
    double width = 0.0;      ///< Horizontal chamfer width [mm].
    double depth = 0.0;      ///< Vertical chamfer size [mm].
    /// When set, the area itself (a trace's reach about its edges), not `outline` +- `width`.
    geom2d::Regions band;
};
/// The zones of a chamfer or trace operation; empty for other operations, for one with a tool
/// orientation of its own, for a trace following the surface, and for one that cannot be
/// generated.
std::vector<ChamferZone> chamferZones(const Document& doc, const Setup& setup, const Operation& op);

/// Bumped whenever the strategies generate different toolpaths from the same inputs (fixes,
/// new defaults): toolpaths cached by an earlier revision are then generated again.
// 2: default heights, feed height on entry only; 3: chamfer outlines on the side they open to;
// 4: contour outlines too; 5: turning keeps clear of the chuck; 6: rest machining on stock
// carried from earlier setups; 7: 3D roughing links stay down where the way is clear; 8: 3D
// roughing fine levels take each column at the deepest level reaching it, rest sees its own cuts;
// 9: 3D roughing circles cross straight where they would cut only air; 10: 3D roughing passes
// along a wall flow on round the part where it bulges into their way; 11: turning roughs only
// the material its side reaches, sees earlier holes and ends a blind bore on the axis; 12: lathe
// setups start from what an earlier setup left, and rough passes end where the material does;
// 17: 3D roughing entries just off the material are fed down from the approach height
inline constexpr int kToolpathRevision = 17;

/// A key that changes whenever anything affecting the operation's toolpath changes (operation,
/// setup, tool, models, program version and kToolpathRevision). Used to cache generated
/// toolpaths.
std::string toolpathInputKey(const Document& doc, const Setup& setup, const Operation& op);

/// The toolpath of `op` (in the setup's WCS) from the document's toolpath cache when it holds
/// one for the current inputs, else generated and stored there. Throws like generateToolpath
/// (failures are not cached).
std::shared_ptr<const toolpath::Toolpath> cachedToolpath(const Document& doc, const Setup& setup,
                                                         const Operation& op, CancelToken& cancel,
                                                         ProgressSink& progress);

/// Like cachedToolpath, in the operation's own frame (generateToolpathInFrame); the cache keeps
/// toolpaths this way.
std::shared_ptr<const toolpath::Toolpath>
cachedToolpathInFrame(const Document& doc, const Setup& setup, const Operation& op,
                      CancelToken& cancel, ProgressSink& progress);

/// The side a contour or chamfer along a selected face's outline, or along the closed loop a
/// selected edge belongs to, runs on: the side the outline opens to, where the part is not (just
/// inside and just outside the outline, a little below and above its height). Default when that
/// cannot be told (the edge closes no loop, or the part is on both sides or neither).
GeometrySide automaticSide(const Document& doc, const Setup& setup, const ShapeRef& ref);

/// The setup's fixtures and its placed workholding bodies (DL-0034, DL-0035) as coarse meshes in
/// the WCS, named after their models and bodies. Throws Error(InvalidArgument) with a user-facing
/// message when a fixture is missing or the workholding cannot be placed.
std::vector<sim::FixtureBody> fixtureBodiesInWcs(const Document& doc, const Setup& setup);

/// A lathe setup's tailstock and steady rest as solids, in its WCS (DL-0052).
struct LatheSupport {
    std::string name;
    Uuid id; ///< Stable, for the view.
    cad::Model model;
};
std::vector<LatheSupport> latheSupports(const Document& doc, const Setup& setup);

/// The setup's fixtures and workholding bodies as the areas they sweep turning with the spindle
/// (sim::revolvedEnvelope), grown by `grow` [mm]. Throws like fixtureBodiesInWcs.
std::vector<sim::LatheBody> latheBodies(const Document& doc, const Setup& setup, double grow);

/// Soft jaws bored to fit (DL-0069), in the setup's lathe plane (WCS Z, radius): the jaws where
/// they stand while bored (clamping the ring or plug), before and after the bore, and the bore.
/// Throws Error(InvalidArgument) when the setup's jaws are not soft jaws bored to fit.
struct SoftJawBoring {
    geom2d::Regions before;
    geom2d::Regions after;
    bool bore = true; ///< A bore into the jaws (an outside grip); else a step on their outside.
    double diameter = 0.0;
    double depth = 0.0;
    double clampDiameter = 0.0;
    double floor = 0.0; ///< The bore's floor (where the work stops), WCS Z.
};
SoftJawBoring softJawBoring(const Document& doc, const Setup& setup);

/// Transform from model coordinates to the setup WCS.
Transform3 modelToWcs(const ModelEntry& model, const Setup& setup);

/// Turning geometry of a lathe setup in the lathe plane (u = WCS Z = spindle axis, v = radius):
/// the part's half profile (cad::axialProfile) and the stock: a bar (cylinder stock along the WCS
/// Z axis, or the part's envelope grown by the stock offsets), or what the earlier lathe setup it
/// comes from left (DL-0044). Throws Error(InvalidArgument) when the setup has no model, the stock
/// is not a bar on the spindle axis, or an earlier setup's stock cannot be carried over.
/// On a lathe, is this hole operation's Tool drive a live tool? (Elsewhere the setting means
/// nothing.)
bool liveDrive(const Operation& op);
/// On a lathe: a live tool going around what it cuts rather than straight into a hole (milling,
/// thread milling, helical boring), on the stock as dexels (DL-0066).
bool liveGoesAround(const Operation& op, const tooling::Tool& tool);
/// `op` is enabled, and is a live tool going around what it cuts.
bool liveMills(const Project& project, const Operation& op);
/// The stock `setup` starts from has been milled by a live tool, in a lathe setup on the way.
bool stockIsMilled(const Project& project, const Setup& setup);

struct LatheGeometry {
    geom2d::Regions part;
    bool partIsRound = true; ///< The part is the same all round the spindle (cad::turnsAboutZ).
    geom2d::Regions stock;
    /// What an earlier setup this stock comes from cut beside its profile: threads and holes.
    sim::LatheStock carriedCuts;
    std::vector<std::string> warnings; ///< What the stock from an earlier setup leaves out.
};
LatheGeometry latheGeometry(const Document& doc, const Setup& setup, CancelToken& cancel);

/// The outline that cuts in a lathe simulation: a turning tool's insert on the toolpath's side, or
/// a hole tool's silhouette (tooling::holeToolOutline).
geom2d::Polyline latheCutter(const tooling::Tool& tool, const toolpath::Toolpath& path);

/// What a lathe operation's tool really does: its toolpath, or with nose compensation in the
/// control (whose program is the profile) the same operation with it in the computer, which is the
/// path the control makes of it.
toolpath::Toolpath latheCutPath(const Document& doc, const Setup& setup, const Operation& op,
                                const toolpath::Toolpath& path, CancelToken& cancel,
                                ProgressSink& progress);

/// A lathe toolpath as the simulation sweeps it: cycles as plain moves, and a tap's moves with its
/// pitch (threads remove nothing, sim::simulateTurning).
toolpath::Toolpath latheMoves(const toolpath::Toolpath& path, const tooling::Tool& tool);

} // namespace ocam::document
