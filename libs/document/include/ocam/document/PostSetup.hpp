// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Setup -> G-code: generate every enabled operation, solve against the machine, post process.

#pragma once

#include "ocam/document/Document.hpp"

#include <ocam/machine/Kinematics.hpp>
#include <ocam/machine/Machine.hpp>
#include <ocam/machine/Model.hpp>
#include <ocam/post/Engine.hpp>
#include <ocam/post/Post.hpp>

#include <string>
#include <vector>

namespace ocam::document {

struct PostOutput {
    std::string gcode; ///< Empty when there are errors.
    std::vector<std::string> warnings;
    std::vector<std::string> errors; ///< Operations that failed to generate or solve.
    std::vector<toolpath::Toolpath>
        toolpaths;         ///< Solved toolpaths in program order (for verification).
    std::string extension; ///< The file extension the post declares ("nc").

    bool ok() const noexcept { return errors.empty(); }
};

/// The machine a setup refers to: a project machine or a built-in by id (no id at all: the
/// generic 3-axis VMC), or nullptr when neither has its id. For showing a setup, which has to go
/// on working without one; what reaches the output goes through machineFor.
const machine::Machine* findMachineFor(const Project& project, const Setup& setup);

/// Why the setup has no machine, naming it and where to choose one; empty when it has.
std::string machineProblem(const Project& project, const Setup& setup);

/// The setup's machine (findMachineFor). Throws Error(InvalidArgument) with machineProblem when
/// it has none: an unknown machine is never taken for another.
const machine::Machine& machineFor(const Project& project, const Setup& setup);

/// Where a setup without a placement sits on the machine, as setups sat before placements
/// existed and as the setup's properties show it: the work on the end of the table chain
/// (machine::tableEnd), the project origin at the table centre (machine::tableCentre for the
/// model origin), the project axes along the machine's.
SetupPlacement defaultPlacement(const machine::Machine& machine);

/// The setup's placement, or the default.
SetupPlacement placementOf(const Setup& setup, const machine::Machine& machine);

/// The setup with its placement's work point taken from its workholding when the placement
/// sits on it (SetupPlacement::onWorkholding: the workholding's mount datum, placed around the
/// stock, goes to the target), for the functions below. Unchanged otherwise. Throws
/// Error(InvalidArgument) when the workholding cannot be placed.
Setup withPlacedWorkholding(const Document& doc, const Setup& setup);

/// The project frame at machine zero (project -> model frame) for a placement.
Transform3 projectOnMachine(const Setup& setup, const SetupPlacement& placement);

/// The setup's work on the machine: the placement's carrier and the WCS at machine zero.
machine::WorkMount workMount(const Setup& setup, const machine::Machine& machine);

/// The setup's WCS in solve's table frame (machine::wcsInTable of workMount): what solve, the
/// post and the machine simulation turn and offset.
Transform3 wcsInTable(const Setup& setup, const machine::Machine& machine);

/// Why the setup's work cannot sit where its placement puts it (machine::mountProblem), or empty.
std::string placementProblem(const Setup& setup, const machine::Machine& machine);

/// The built-in post that fits a machine: the generic lathe post for lathes, the 4-axis post for
/// 4-axis mills, the generic mill post otherwise.
const post::PostScript& defaultPostFor(const machine::Machine& machine);

/// The post a setup is posted with: its choice (Setup::post: a built-in post or one the project
/// carries), else the machine's default (Machine::post, looked up in the project's posts, then
/// the built-in posts), else defaultPostFor. Posts the project carries are compiled from their
/// source. Throws Error(InvalidArgument) when a chosen post does not exist, Error(Parse) or
/// Error(Unsupported) when a project post does not load.
post::PostScript postFor(const Project& project, const Setup& setup,
                         const machine::Machine& machine);

/// The posts the project carries, compiled (once per source), in Project::posts order. Throws
/// as postFor.
std::vector<post::PostScript> projectPosts(const Project& project);

/// The option values a setup is posted with by `post`: the setup's (Setup::postOptions, when
/// `post` is the setup's post) over the machine's (Machine::postOptions, when `post` is the
/// machine's default post), in the shape postSetup takes.
Json postOptionsFor(const Project& project, const Setup& setup, const machine::Machine& machine,
                    const post::PostScript& post);

/// The highest work offset a setup can use with a post on a machine: the last the post declares
/// (post::lastWorkOffset), at most the machine's count (Machine::workOffsets; 0: no limit).
int lastWorkOffset(const post::PostManifest& post, const machine::Machine& machine);

/// How a work offset of the setup is written by its post ("G154 P1"); the generic name
/// (workOffsetName(int)) when its post cannot be resolved.
std::string workOffsetName(const Project& project, const Setup& setup, int workOffset);

/// How an operation with a tool orientation of its own (Operation::orientation) is posted on a
/// machine: its frame in the table frame (wcsInTable; on machines without rotary axes, the
/// setup's WCS), the rotary positions that turn its Z axis up the spindle
/// (machine::indexSolution) and where its work offset is measured (ToolOrientation::workOffset).
struct OperationIndex {
    Transform3 frameInTable = Transform3::Identity();
    machine::IndexAngles angles;
    machine::WorkOrigin origin;
    int workOffset = 1; ///< The work offset the operation is posted with.
};

/// Throws Error(InvalidArgument) with a user-facing message when the machine cannot reach the
/// orientation (or, with the setup's work offset, the setup's own 3+2 index).
OperationIndex operationIndex(const Setup& setup, const Operation& op,
                              const machine::Machine& machine);

/// A setup lowered to the NC intermediate representation (before formatting): the input of
/// built-in and external post processors (see post::toJson(NcProgram)).
struct LoweredSetup {
    post::NcProgram program; ///< Empty when there are errors.
    std::vector<std::string> warnings;
    std::vector<std::string> errors;
    std::vector<toolpath::Toolpath> toolpaths; ///< Solved toolpaths in program order.
    bool ok() const noexcept { return errors.empty(); }
};

/// Generates, solves and lowers all enabled operations of a setup, in order.
LoweredSetup lowerSetup(const Document& doc, const Uuid& setupId, const machine::Machine& machine,
                        const post::LowerOptions& options, CancelToken& cancel,
                        ProgressSink& progress);

/// Posts all enabled operations of a setup, in order, with a JavaScript post (DL-0039) and the
/// values of its options, given as Setup::postOptions is: {"program": {id: value},
/// "operations": {operation id: {id: value}}}; the post's defaults for the rest. No partial
/// program is produced: if any operation fails, or the post refuses or fails, `errors` lists why
/// and `gcode` stays empty. Throws OperationCancelled when cancelled.
PostOutput postSetup(const Document& doc, const Uuid& setupId, const machine::Machine& machine,
                     const post::PostScript& post, CancelToken& cancel, ProgressSink& progress,
                     const Json& options = Json::object());

/// The same with the setup's own post (postFor) and option values (postOptionsFor). Work
/// offsets the post does not declare or beyond the machine's count are errors.
PostOutput postSetup(const Document& doc, const Uuid& setupId, const machine::Machine& machine,
                     CancelToken& cancel, ProgressSink& progress);

} // namespace ocam::document
