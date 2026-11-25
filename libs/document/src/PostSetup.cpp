// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/document/PostSetup.hpp"

#include "ocam/document/Generate.hpp"
#include "ocam/document/LatheMachine.hpp"
#include "ocam/document/Workholding.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/machine/Kinematics.hpp>
#include <ocam/machine/Model.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <chrono>
#include <iterator>
#include <map>
#include <mutex>

namespace ocam::document {
namespace {

Error userError(const std::string& message) {
    return Error(ErrorCode::InvalidArgument, message);
}

/// A tool as the post's tool table shows it.
post::NcTool ncTool(const tooling::Tool& tool, const tooling::Holder* holder) {
    post::NcTool t;
    t.number = tool.number;
    t.name = tool.name;
    t.comment = tool.comment;
    t.type = std::string(tooling::toString(tool.type));
    if (tooling::isTurningTool(tool.type)) {
        t.insertShape = tool.insert.shape;
        t.noseRadius = tool.insert.noseRadius;
        t.insertOrientation = tool.insert.orientation;
        t.threadPitch = tool.insert.threadPitch;
    } else {
        t.diameter = tool.mill.diameter;
        t.cornerRadius = tool.mill.cornerRadius;
        t.flutes = tool.mill.flutes;
        t.threadPitch = tool.mill.threadPitch;
    }
    t.gaugeLength = tooling::gaugeLength(tool, holder);
    return t;
}

/// Now as ISO 8601 UTC, to the second: "2026-10-25T14:03:07Z".
std::string utcNow() {
    using namespace std::chrono;
    const auto now = floor<seconds>(system_clock::now());
    const auto day = floor<days>(now);
    const year_month_day date{day};
    const hh_mm_ss time{now - day};
    return fmt::format("{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z", static_cast<int>(date.year()),
                       static_cast<unsigned>(date.month()), static_cast<unsigned>(date.day()),
                       time.hours().count(), time.minutes().count(), time.seconds().count());
}

/// Table rotary axes that do not carry the setup's work must stay at 0.
void checkStill(const Setup& setup, const machine::Machine& machine,
                const machine::IndexAngles& q) {
    if (machine.isLathe()) {
        return; // C is the spindle holding the work, or a spindle standing still
    }
    const machine::MachineModel model = machine::effectiveModel(machine);
    const SetupPlacement placement = placementOf(setup, machine);
    for (const std::string& axis : machine::rotariesNotCarrying(model, placement.carrier)) {
        const double value = axis == "A" ? q.a : axis == "B" ? q.b : q.c;
        if (std::abs(value) > 1e-9) {
            throw userError(fmt::format(
                "it turns the {} axis to {:.3f}°, but the setup's work does not sit on it (it is "
                "placed on {}): place the work on the {} axis or keep the operation along the "
                "setup's Z axis",
                axis, radToDeg(value),
                placement.carrier.empty() ? std::string("the machine base")
                                          : "the " + placement.carrier + " axis",
                axis));
        }
    }
}

bool sameOrigin(const machine::WorkOrigin& a, const machine::WorkOrigin& b) {
    return (a.point - b.point).norm() < 1e-6 && std::abs(a.angles.a - b.angles.a) < 1e-9 &&
           std::abs(a.angles.b - b.angles.b) < 1e-9 && std::abs(a.angles.c - b.angles.c) < 1e-9;
}

} // namespace

OperationIndex operationIndex(const Setup& setup, const Operation& op,
                              const machine::Machine& machine) {
    OperationIndex out;
    if (!op.orientation) {
        throw userError("internal: the operation has no tool orientation of its own");
    }
    const Transform3 frame = operationFrame(op);
    const bool ownOffset = op.orientation->workOffset > 0;
    out.workOffset = ownOffset ? op.orientation->workOffset : setup.workOffset;
    if (!machine.hasRotary()) {
        // Without rotary axes the setup's WCS is the machine's: only turns about its Z.
        if ((frame.linear().col(2) - Vec3::UnitZ()).norm() > 1e-9) {
            throw userError(fmt::format(
                "the tool orientation tilts the tool away from the setup's Z axis, but the "
                "machine '{}' has no rotary axes: use a setup of its own (a separate fixturing) "
                "or a 4- or 5-axis machine",
                machine.name));
        }
        out.frameInTable = frame;
        if (ownOffset) {
            out.origin.point = frame.translation();
        }
        return out;
    }
    const Transform3 wcs = wcsInTable(setup, machine);
    out.frameInTable = wcs * frame;
    const Vec3 z = out.frameInTable.linear().col(2);
    const auto angles = machine::indexSolution(machine, z);
    if (!angles) {
        throw userError(fmt::format(
            "the machine '{}' cannot turn the tool orientation ({:.4f}, {:.4f}, {:.4f}) (project "
            "frame) up the spindle within its rotary axis limits",
            machine.name, z.x(), z.y(), z.z()));
    }
    out.angles = *angles;
    if (ownOffset) {
        out.origin = {out.frameInTable.translation(), *angles};
        return out;
    }
    // The setup's work offset: measured at its WCS origin with the table at the setup's index.
    machine::IndexAngles setupIndex;
    if (machine.canIndex()) {
        const auto s = machine::indexAngles(machine, wcs.linear().col(2));
        if (!s) {
            throw userError("the setup's own tool axis cannot be reached within the rotary axis "
                            "limits, so its work offset cannot be measured: give the operation a "
                            "work offset of its own");
        }
        setupIndex = *s;
    }
    out.origin = {wcs.translation(), setupIndex};
    return out;
}

SetupPlacement defaultPlacement(const machine::Machine& machine) {
    const machine::MachineModel model = machine::effectiveModel(machine);
    SetupPlacement p;
    p.carrier = machine::tableEnd(model);
    p.target = machine::tableCentre(model, Vec3::Zero());
    p.workPoint = Vec3::Zero();
    return p;
}

SetupPlacement placementOf(const Setup& setup, const machine::Machine& machine) {
    return setup.placement ? *setup.placement : defaultPlacement(machine);
}

Setup withPlacedWorkholding(const Document& doc, const Setup& setup) {
    if (!setup.placement || !setup.placement->onWorkholding) {
        return setup;
    }
    Setup out = setup;
    try {
        const auto pose = setupWorkholdingPose(doc, setup);
        if (!pose) {
            throw userError("the setup has no workholding");
        }
        out.placement->workPoint = setup.wcs.toParent() * pose->mount;
    } catch (const Error& e) {
        throw userError(
            fmt::format("the setup sits on the machine by its workholding: {}", e.what()));
    }
    out.placement->onWorkholding = false;
    return out;
}

Transform3 projectOnMachine(const Setup& setup, const SetupPlacement& placement) {
    const Vec3 r = placement.rotation;
    Transform3 turn = Transform3::Identity();
    turn.linear() = (Eigen::AngleAxisd(degToRad(r.z()), Vec3::UnitZ()) *
                     Eigen::AngleAxisd(degToRad(r.y()), Vec3::UnitY()) *
                     Eigen::AngleAxisd(degToRad(r.x()), Vec3::UnitX()))
                        .toRotationMatrix();
    const Vec3 workPoint = placement.workPoint.value_or(setup.wcs.origin);
    return Eigen::Translation3d(placement.target + placement.offset) * turn *
           Eigen::Translation3d(-workPoint);
}

machine::WorkMount workMount(const Setup& setup, const machine::Machine& machine) {
    const SetupPlacement placement = placementOf(setup, machine);
    return {placement.carrier, projectOnMachine(setup, placement) * setup.wcs.toParent()};
}

Transform3 wcsInTable(const Setup& setup, const machine::Machine& machine) {
    return machine::wcsInTable(machine::effectiveModel(machine), workMount(setup, machine));
}

std::string placementProblem(const Setup& setup, const machine::Machine& machine) {
    if (machine.isLathe()) {
        return {};
    }
    const std::string problem = machine::mountProblem(machine::effectiveModel(machine),
                                                      placementOf(setup, machine).carrier);
    return problem.empty() ? problem : "the setup's placement on the machine: " + problem;
}

const machine::Machine* findMachineFor(const Project& project, const Setup& setup) {
    if (const machine::Machine* m = project.findMachine(setup.machine)) {
        return m;
    }
    return machine::builtinMachine(setup.machine);
}

std::string machineProblem(const Project& project, const Setup& setup) {
    if (findMachineFor(project, setup)) {
        return {};
    }
    return fmt::format("setup '{}': machine '{}' is neither a built-in machine nor in the project: "
                       "choose one under Machine in the setup's properties",
                       setup.name, setup.machine);
}

const machine::Machine& machineFor(const Project& project, const Setup& setup) {
    if (const machine::Machine* m = findMachineFor(project, setup)) {
        return *m;
    }
    throw userError(machineProblem(project, setup));
}

const post::PostScript& defaultPostFor(const machine::Machine& machine) {
    if (machine.isLathe()) {
        return post::builtinPost("fanuc-generic-lathe");
    }
    if (machine.kinematics == machine::Kinematics::Mill4A) {
        return post::builtinPost("fanuc-generic-mill-4axis");
    }
    return post::builtinPost("fanuc-generic-mill");
}

std::vector<post::PostScript> projectPosts(const Project& project) {
    // Compiled once per source: the tree and the setup page ask for names often.
    static std::mutex mutex;
    static std::map<std::pair<std::string, std::string>, post::PostScript> compiled;
    CancelToken cancel;
    std::vector<post::PostScript> posts;
    for (const ProjectPost& p : project.posts) {
        const std::string name = p.file.empty() ? "project post" : p.file;
        const std::lock_guard lock(mutex);
        auto it = compiled.find({name, p.source});
        if (it == compiled.end()) {
            if (compiled.size() > 64) {
                compiled.clear();
            }
            it = compiled
                     .emplace(std::pair{name, p.source},
                              post::PostScript::load(p.source, name, cancel))
                     .first;
        }
        posts.push_back(it->second);
    }
    return posts;
}

int lastWorkOffset(const post::PostManifest& post, const machine::Machine& machine) {
    const int last = post::lastWorkOffset(post);
    return machine.workOffsets > 0 ? std::min(last, machine.workOffsets) : last;
}

std::string workOffsetName(const Project& project, const Setup& setup, int workOffset) {
    try {
        const machine::Machine& machine = machineFor(project, setup);
        const post::PostScript post = postFor(project, setup, machine);
        if (const auto code = post::workOffsetCode(post.manifest(), workOffset)) {
            return *code;
        }
        return fmt::format("{} (not on {})", workOffset, post.manifest().id);
    } catch (const Error&) {
        return workOffsetName(workOffset); // the post is missing: the setup page says why
    }
}

Json postOptionsFor(const Project& project, const Setup& setup, const machine::Machine& machine,
                    const post::PostScript& post) {
    // Values are for one post: the setup's for its post, the machine's for its default post.
    auto postsWith = [&](const Setup& s) {
        try {
            return postFor(project, s, machine).manifest().id == post.manifest().id;
        } catch (const Error&) {
            return false; // the post is missing: its values have no post
        }
    };
    Json options =
        setup.postOptions.is_object() && postsWith(setup) ? setup.postOptions : Json::object();
    Json program = Json::object();
    Setup plain = setup;
    plain.post.reset();
    if (machine.postOptions.is_object() && !machine.postOptions.empty() && postsWith(plain)) {
        program = machine.postOptions;
    }
    if (options.contains("program") && options.at("program").is_object()) {
        for (const auto& [id, value] : options.at("program").items()) {
            program[id] = value;
        }
    }
    options["program"] = std::move(program);
    return options;
}

post::PostScript postFor(const Project& project, const Setup& setup,
                         const machine::Machine& machine) {
    auto inProject = [&](const std::string& id) -> std::optional<post::PostScript> {
        for (post::PostScript& p : projectPosts(project)) {
            if (p.manifest().id == id) {
                return std::move(p);
            }
        }
        return std::nullopt;
    };
    if (setup.post) {
        if (!setup.post->inProject) {
            return post::builtinPost(setup.post->id);
        }
        if (auto p = inProject(setup.post->id)) {
            return std::move(*p);
        }
        throw userError(fmt::format("the setup's post '{}' is not in the project any more: "
                                    "choose another post",
                                    setup.post->id));
    }
    if (!machine.post.empty()) {
        if (auto p = inProject(machine.post)) {
            return std::move(*p);
        }
        const std::vector<std::string> builtins = post::builtinPostIds();
        if (std::ranges::find(builtins, machine.post) != builtins.end()) {
            return post::builtinPost(machine.post);
        }
        throw userError(fmt::format("the machine's post '{}' is neither a built-in post nor in "
                                    "the project: choose a post for the setup",
                                    machine.post));
    }
    return defaultPostFor(machine);
}

LoweredSetup lowerSetup(const Document& doc, const Uuid& setupId, const machine::Machine& machine,
                        const post::LowerOptions& options, CancelToken& cancel,
                        ProgressSink& progress) {
    LoweredSetup out;
    const Project& project = doc.project();
    const Setup* setup = project.findSetup(setupId);
    if (!setup) {
        throw Error(ErrorCode::InvalidArgument, "unknown setup");
    }
    std::size_t setupIndex = 0;
    while (setupIndex < project.setups.size() && project.setups[setupIndex].id != setupId) {
        ++setupIndex;
    }

    // A machine passed in does not stand for the setup's: its toolpaths are generated for that.
    if (const std::string problem = machineProblem(project, *setup); !problem.empty()) {
        out.errors.push_back(problem);
        return out;
    }
    if (const std::string problem = placementProblem(*setup, machine); !problem.empty()) {
        out.errors.push_back(problem);
        return out;
    }
    // A sub-spindle setup that a transfer hands its part to goes in the program of the setup it
    // comes from, which runs both as one cycle (ADR-0007).
    {
        const Setup* from = nullptr;
        if (transferInto(project, *setup, &from) && from) {
            out.errors.push_back(fmt::format(
                "setup '{}' takes its part by a transfer from setup '{}': post setup '{}', whose "
                "program runs both",
                setup->name, from->name, from->name));
            return out;
        }
    }
    std::vector<const Setup*> inProgram{setup};
    if (transferOperation(*setup)) {
        if (const Setup* target = transferTarget(project, *setup)) {
            if (const std::string problem = machineProblem(project, *target); !problem.empty()) {
                out.errors.push_back(problem);
                return out;
            }
            inProgram.push_back(target);
        }
    }
    const Setup* const programSetup = setup;
    std::vector<post::PostJob> jobs;
    for (const Setup* inSetup : inProgram) {
        Setup placed;
        try {
            placed = withPlacedWorkholding(doc, *inSetup);
        } catch (const Error& e) {
            out.errors.push_back(e.what());
            return out;
        }
        setup = &placed;
        const Transform3 wcs = wcsInTable(*setup, machine);
        // Work offsets of operations with an orientation of their own, and where each is measured.
        std::map<int, std::pair<machine::WorkOrigin, std::string>> ownOffsets;
        const auto count = static_cast<double>(std::max<std::size_t>(1, setup->operations.size()));
        for (std::size_t i = 0; i < setup->operations.size(); ++i) {
            const Operation& op = setup->operations[i];
            if (!op.enabled) {
                continue;
            }
            cancel.throwIfCancelled();
            try {
                ProgressRange range(progress, i / count, (i + 1) / count);
                // The toolpath shown for the operation, when its inputs are unchanged. With a tool
                // orientation of its own it is posted from its own frame (3+2: arcs and cycles
                // kept).
                toolpath::Toolpath tp = *cachedToolpathInFrame(doc, *setup, op, cancel, range);
                std::optional<OperationIndex> positional;
                if (op.orientation) {
                    if (toolpath::isMultiAxis(tp) || machine.isLathe()) {
                        tp = toolpathInWcs(tp, op);
                    } else {
                        positional = operationIndex(*setup, op, machine);
                    }
                }
                for (const auto& w : tp.warnings) {
                    out.warnings.push_back(fmt::format("{}: {}", op.name, w));
                }
                if (tp.transfer) {
                    // The transfer: Z2 and the spindles, no tool (ADR-0007).
                    post::PostJob job;
                    job.name = op.name;
                    job.path = tp;
                    job.workOffset = setup->workOffset;
                    job.comment = op.comment;
                    job.operationId = op.id.toString();
                    jobs.push_back(std::move(job));
                    continue;
                }
                if (tp.moves.empty()) {
                    out.warnings.push_back(fmt::format("{}: no toolpath; skipped", op.name));
                    continue;
                }
                // The WCS in the table frame (where the setup's placement puts it): 3+2 indexing on
                // table-table machines, simultaneous motion for multi-axis toolpaths.
                const tooling::Tool* tool = project.tools.findTool(*op.toolId);
                const tooling::Holder* holder =
                    tool && tool->holderId ? project.tools.findHolder(*tool->holderId) : nullptr;
                machine::SolveOptions solveOptions;
                solveOptions.tcp = options.tcp;
                solveOptions.toolLength = tool ? tooling::gaugeLength(*tool, holder) : 0.0;
                machine::SolveResult solved =
                    positional ? machine::solveIndexed(machine, tp, positional->frameInTable,
                                                       positional->angles, positional->origin,
                                                       solveOptions)
                               : machine::solve(machine, tp, wcs, solveOptions);
                if (solved.index) {
                    checkStill(*setup, machine, *solved.index);
                }
                if (solved.rotary) {
                    checkStill(*setup, machine, solved.rotary->start);
                    for (const machine::IndexAngles& q : solved.rotary->angles) {
                        checkStill(*setup, machine, q);
                    }
                }
                const int workOffset = positional ? positional->workOffset : setup->workOffset;
                if (positional && op.orientation->workOffset > 0) {
                    // A work offset of its own is measured at its frame's origin at its index: it
                    // cannot be the setup's, nor another operation's measured elsewhere.
                    if (workOffset == setup->workOffset) {
                        throw userError(fmt::format("its tool orientation has a work offset of its "
                                                    "own, {}, which the setup "
                                                    "uses too: choose another work offset",
                                                    workOffsetName(workOffset)));
                    }
                    auto [it, added] =
                        ownOffsets.try_emplace(workOffset, positional->origin, op.name);
                    if (!added && !sameOrigin(it->second.first, positional->origin)) {
                        throw userError(fmt::format(
                            "its work offset {} is measured at another origin or index than "
                            "operation '{}' measures it: choose another work offset",
                            workOffsetName(workOffset), it->second.second));
                    }
                }
                for (const auto& w : solved.warnings) {
                    out.warnings.push_back(fmt::format("{}: {}", op.name, w));
                }
                for (const auto& e : solved.errors) {
                    out.errors.push_back(fmt::format("{}: {}", op.name, e));
                }
                post::PostJob job{op.name,      solved.path,  tool ? tool->name : std::string(),
                                  workOffset,   solved.index, {},
                                  solved.rotary};
                if (!machine.hasRotary()) {
                    job.index.reset(); // positional on a 3-axis machine: nothing to index
                }
                if (!solved.rotary && machine.hasRotary() && !job.index) {
                    // Toolpaths along the spindle on machines that cannot index: rotary axes at 0.
                    job.index = machine::IndexAngles{};
                }
                if (job.index) {
                    job.indexAxes = machine.rotaryAxes();
                }
                job.comment = op.comment;
                job.operationId = op.id.toString();
                job.subSpindle = onSubSpindle(*setup);
                if (tool) {
                    job.tool = ncTool(*tool, holder);
                }
                jobs.push_back(std::move(job));
                out.toolpaths.push_back(std::move(solved.path));
            } catch (const OperationCancelled&) {
                throw;
            } catch (const Error& e) {
                out.errors.push_back(fmt::format("{}: {}", op.name, e.what()));
            }
        }
    }
    setup = programSetup;
    if (jobs.empty() && out.errors.empty()) {
        out.errors.emplace_back("the setup has no enabled operations with toolpaths");
    }
    if (!out.errors.empty()) {
        out.toolpaths.clear();
        return out;
    }
    const std::string programName = fmt::format("{} {}", project.name, setup->name);
    // What the machine says of its moves, wherever a setup is lowered (DL-0067).
    post::LowerOptions lowerOptions = options;
    lowerOptions.indexAtHome = machine.lathe && machine.lathe->turret.indexAtHome;
    lowerOptions.latheApproach = machine.isLathe();
    out.program = post::lower(jobs, programName, 1000 + static_cast<int>(setupIndex), lowerOptions);
    out.program.project = project.name;
    out.program.setup = setup->name;
    out.program.time = utcNow();
    out.program.machine = post::ncMachine(machine);
    return out;
}

PostOutput postSetup(const Document& doc, const Uuid& setupId, const machine::Machine& machine,
                     const post::PostScript& post, CancelToken& cancel, ProgressSink& progress,
                     const Json& options) {
    PostOutput out;
    out.extension = post.manifest().extension;
    // A post says which machines it writes for.
    const std::string kinematics(machine::toString(machine.kinematics));
    if (std::ranges::find(post.manifest().machines, kinematics) == post.manifest().machines.end()) {
        out.errors.push_back(fmt::format("the post '{}' is not for {} machines ({}): it writes "
                                         "for {}",
                                         post.manifest().name, kinematics, machine.name,
                                         fmt::join(post.manifest().machines, ", ")));
        return out;
    }
    LoweredSetup lowered =
        lowerSetup(doc, setupId, machine, post::lowerOptionsFor(post.manifest()), cancel, progress);
    out.warnings = std::move(lowered.warnings);
    out.errors = std::move(lowered.errors);
    if (!out.errors.empty()) {
        return out;
    }
    out.warnings.insert(out.warnings.end(), lowered.program.warnings.begin(),
                        lowered.program.warnings.end());
    // A sub-spindle's work and the transfer to it: only a post that says it writes them (ADR-0007).
    if (!post.manifest().subSpindle &&
        std::ranges::any_of(lowered.program.instrs, [](const post::NcInstr& i) {
            return i.op == post::NcOp::SubAxis || i.op == post::NcOp::SpindleSync || i.sub;
        })) {
        out.errors.push_back(fmt::format("the post '{}' does not write a sub-spindle's work or the "
                                         "transfer to it: choose a post that does (its manifest "
                                         "says subSpindle)",
                                         post.manifest().name));
        return out;
    }
    // Work offsets the post declares and the machine has.
    for (const post::NcOperation& op : lowered.program.operations) {
        if (!post::workOffsetCode(post.manifest(), op.workOffset)) {
            out.errors.push_back(fmt::format("{}: work offset {}: {} has {}", op.name,
                                             op.workOffset, post.manifest().name,
                                             post::describeWorkOffsets(post.manifest())));
        } else if (machine.workOffsets > 0 && op.workOffset > machine.workOffsets) {
            out.errors.push_back(fmt::format("{}: work offset {}: the machine has {} (machine "
                                             "library: Work offsets)",
                                             op.name, op.workOffset, machine.workOffsets));
        }
    }
    if (!out.errors.empty()) {
        return out;
    }
    try {
        // Option values: the program's, and each operation's by its id.
        const Json none = Json::object();
        const Json& program =
            options.is_object() && options.contains("program") ? options.at("program") : none;
        const Json& operations =
            options.is_object() && options.contains("operations") ? options.at("operations") : none;
        for (post::NcOperation& op : lowered.program.operations) {
            op.options = post.operationOptionValues(
                operations.is_object() && operations.contains(op.id) ? operations.at(op.id) : none);
        }
        post::PostResult result = post.run(lowered.program, post.optionValues(program), cancel);
        out.gcode = std::move(result.text);
        out.warnings.insert(out.warnings.end(), std::make_move_iterator(result.warnings.begin()),
                            std::make_move_iterator(result.warnings.end()));
    } catch (const OperationCancelled&) {
        throw;
    } catch (const Error& e) {
        // The post refused the program or failed: its message names the post and the reason.
        out.errors.emplace_back(e.what());
        return out;
    }
    out.toolpaths = std::move(lowered.toolpaths);
    progress.report(1.0, "Posted");
    return out;
}

PostOutput postSetup(const Document& doc, const Uuid& setupId, const machine::Machine& machine,
                     CancelToken& cancel, ProgressSink& progress) {
    const Setup* setup = doc.project().findSetup(setupId);
    if (!setup) {
        throw Error(ErrorCode::InvalidArgument, "unknown setup");
    }
    std::optional<post::PostScript> post;
    try {
        post = postFor(doc.project(), *setup, machine);
    } catch (const OperationCancelled&) {
        throw;
    } catch (const Error& e) {
        PostOutput out;
        out.errors.emplace_back(e.what());
        return out;
    }
    return postSetup(doc, setupId, machine, *post, cancel, progress,
                     postOptionsFor(doc.project(), *setup, machine, *post));
}

} // namespace ocam::document
