// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Toolpaths -> NC IR.

#include "ocam/post/Post.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/machine/Turret.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>

namespace ocam::post {
namespace {

using toolpath::MoveKind;

NcInstr instr(NcOp op) {
    NcInstr i;
    i.op = op;
    return i;
}

/// Box around a toolpath's motion: its start, every end point, the extremes of its arcs, and
/// the bottom and R plane of its holes.
void motionBounds(const toolpath::Toolpath& path, Vec3& lo, Vec3& hi) {
    lo = hi = path.start;
    auto add = [&](const Vec3& p) {
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    };
    Vec3 pos = path.start;
    for (const toolpath::Move& m : path.moves) {
        if (m.kind == MoveKind::Dwell) {
            continue;
        }
        if (m.isArc()) {
            // The quarter points the arc passes through (as seen in its plane).
            const Vec2 c = toolpath::planeCoords(m.center, m.plane);
            const Vec2 s = toolpath::planeCoords(pos, m.plane);
            const Vec2 e = toolpath::planeCoords(m.end, m.plane);
            const double r = (s - c).norm();
            const double a0 = std::atan2(s.y() - c.y(), s.x() - c.x());
            double sweep = std::atan2(e.y() - c.y(), e.x() - c.x()) - a0;
            const bool cw = m.kind == MoveKind::ArcCW;
            sweep = std::fmod(sweep, 2.0 * kPi);
            if (!cw && sweep <= 1e-12) {
                sweep += 2.0 * kPi;
            } else if (cw && sweep >= -1e-12) {
                sweep -= 2.0 * kPi;
            }
            const double n0 = toolpath::planeNormal(pos, m.plane);
            const double n1 = toolpath::planeNormal(m.end, m.plane);
            for (int k = -8; k <= 8; ++k) {
                const double a = k * kPi / 2.0;
                const double t = (a - a0) / sweep;
                if (t > 0.0 && t < 1.0) {
                    add(toolpath::fromPlaneCoords(c + r * Vec2(std::cos(a), std::sin(a)),
                                                  n0 + (n1 - n0) * t, m.plane));
                }
            }
        }
        if (m.kind == MoveKind::Cycle && m.cycle >= 0 &&
            static_cast<std::size_t>(m.cycle) < path.cycles.size()) {
            const toolpath::DrillCycle& c = path.cycles[static_cast<std::size_t>(m.cycle)];
            add(Vec3(m.end.x(), m.end.y(), c.bottomZ));
            add(Vec3(m.end.x(), m.end.y(), c.retractZ));
        }
        if (m.kind == MoveKind::Probe && m.measure && m.probe >= 0 &&
            static_cast<std::size_t>(m.probe) < path.probes.size() &&
            (m.end - pos).norm() > 1e-12) {
            // A measuring move goes on past the surface by the overtravel, looking for it.
            add(m.end + (m.end - pos).normalized() *
                            path.probes[static_cast<std::size_t>(m.probe)].overtravel);
        }
        add(m.end);
        pos = m.end;
    }
}

} // namespace

NcMachine ncMachine(const machine::Machine& machine) {
    NcMachine m;
    m.id = machine.id;
    m.name = machine.name;
    m.kinematics = std::string(machine::toString(machine.kinematics));
    m.axes = machine.axes;
    m.rotaryAxes = machine.rotaryAxes();
    m.minSpindle = machine.minSpindle;
    m.maxSpindle = machine.maxSpindle;
    m.maxFeed = machine.maxFeed;
    m.rapidRate = machine.rapidRate;
    m.toolCapacity = machine.toolCapacity;
    m.workOffsets = machine.workOffsets;
    return m;
}

NcProgram lower(const std::vector<PostJob>& jobs, std::string programName, int programNumber,
                const LowerOptions& options) {
    NcProgram program;
    program.name = std::move(programName);
    program.number = programNumber;

    int currentTool = -1;
    int currentOffset = -1;
    std::optional<Vec3>
        lastEnd; // where the last toolpath left the tool, in its program coordinates
    double currentSpindle = 0.0;
    tooling::Coolant currentCoolant = tooling::Coolant::Off;
    bool cycleActive = false;
    std::optional<machine::IndexAngles> currentIndex; // rotary positions, when known
    bool positioned = false;
    bool needToolLength = false; // tool centre point control ended: re-apply the offset

    auto stopCycle = [&] {
        if (cycleActive) {
            program.instrs.push_back(instr(NcOp::CycleCancel));
            cycleActive = false;
        }
    };
    auto setCoolant = [&](tooling::Coolant c) {
        if (c != currentCoolant) {
            NcInstr i = instr(NcOp::Coolant);
            i.coolant = c;
            program.instrs.push_back(i);
            currentCoolant = c;
        }
    };
    double currentLive = 0.0;
    bool currentTapping = false;
    std::optional<toolpath::ArcPlane> currentMilling;
    auto setLive = [&](double s, bool tapping = false,
                       std::optional<toolpath::ArcPlane> milling = std::nullopt) {
        if (s != currentLive || tapping != currentTapping || milling != currentMilling) {
            // A plane's codes go with the tool: off first if only they change.
            if (currentLive > 0.0 && s > 0.0) {
                NcInstr off = instr(NcOp::LiveTool);
                program.instrs.push_back(off);
            }
            NcInstr i = instr(NcOp::LiveTool);
            i.spindle = s;
            i.tapping = tapping;
            i.milling = milling.has_value();
            i.plane = milling.value_or(toolpath::ArcPlane::XY);
            program.instrs.push_back(i);
            currentLive = s;
            currentTapping = tapping;
            currentMilling = milling;
        }
    };
    double currentCss = 0.0;
    bool currentReverse = false;
    bool onSub = false; // the work is on the sub-spindle (ADR-0007)
    auto setSpindle = [&](double s, double css = 0.0, double maxSpindle = 0.0,
                          bool reverse = false) {
        if (s != currentSpindle || css != currentCss || reverse != currentReverse) {
            NcInstr i = instr(NcOp::Spindle);
            i.sub = onSub;
            i.spindle = s;
            i.reverse = reverse;
            currentReverse = reverse;
            i.surfaceSpeed = s > 0.0 ? css : 0.0;
            i.maxSpindle = maxSpindle;
            program.instrs.push_back(i);
            currentSpindle = s;
            currentCss = i.surfaceSpeed;
        }
    };

    std::optional<toolpath::SpindleTransfer> handedOver; // the last transfer, for its Z2
    bool synchronised = false;
    for (const PostJob& job : jobs) {
        // The transfer to the sub-spindle (ADR-0007): Z2 and the spindles, no tool.
        if (const auto& t = job.path.transfer) {
            NcOperation operation;
            operation.id = job.operationId;
            operation.name = job.name;
            operation.comment = job.comment;
            operation.strategy = job.path.strategy;
            operation.workOffset = job.workOffset;
            const int operationIndex = static_cast<int>(program.operations.size());
            program.operations.push_back(std::move(operation));
            NcInstr begin = instr(NcOp::OperationBegin);
            begin.operation = operationIndex;
            program.instrs.push_back(begin);
            stopCycle();
            setCoolant(tooling::Coolant::Off);
            setLive(0.0);
            const auto chuck = [&](bool open) {
                NcInstr c = instr(NcOp::Chuck);
                c.sub = true;
                c.open = open;
                program.instrs.push_back(c);
            };
            const auto z2 = [&](double to, double feed) {
                NcInstr a = instr(NcOp::SubAxis);
                a.target.z() = to;
                a.feed = feed;
                program.instrs.push_back(a);
            };
            chuck(true);
            z2(t->approachZ2, 0.0);
            z2(t->pickZ2, t->feed);
            chuck(false);
            if (t->dwell > 0.0) {
                NcInstr d = instr(NcOp::Dwell);
                d.dwell = t->dwell;
                program.instrs.push_back(d);
            }
            if (t->sync == toolpath::SpindleTransfer::Sync::Stopped) {
                setSpindle(0.0);
            } else {
                NcInstr s = instr(NcOp::SpindleSync);
                s.polarOn = true;
                if (t->sync == toolpath::SpindleTransfer::Sync::Angle) {
                    s.syncAngle = t->angle;
                }
                program.instrs.push_back(s);
                synchronised = true;
                setSpindle(t->spindle);
            }
            if (!t->partOff) {
                NcInstr c = instr(NcOp::Chuck); // the part is free: the main chuck lets go
                c.open = true;
                program.instrs.push_back(c);
            }
            handedOver = *t;
            NcInstr end = instr(NcOp::OperationEnd);
            end.operation = operationIndex;
            program.instrs.push_back(end);
            continue;
        }
        // The first of the sub-spindle's work: the spindles stop and part, and Z2 takes the part
        // to where it is cut.
        if (job.subSpindle && !onSub) {
            stopCycle();
            setCoolant(tooling::Coolant::Off);
            setLive(0.0);
            setSpindle(0.0);
            if (synchronised) {
                program.instrs.push_back(instr(NcOp::SpindleSync)); // off
                synchronised = false;
            }
            if (handedOver) {
                NcInstr a = instr(NcOp::SubAxis);
                a.target.z() = handedOver->workZ2;
                program.instrs.push_back(a);
            }
            onSub = true;
            currentSpindle = 0.0; // the sub-spindle's, from here
            currentCss = 0.0;
            lastEnd.reset();
        }
        // Plain moves cannot reverse the spindle: expanded, a tap would strip its thread.
        for (const toolpath::CycleType type :
             {toolpath::CycleType::Tap, toolpath::CycleType::TapLeft}) {
            const bool takes =
                options.cannedCycles &&
                (options.cycleTypes.empty() ||
                 std::ranges::find(options.cycleTypes, type) != options.cycleTypes.end());
            if (!takes && std::ranges::any_of(job.path.cycles, [&](const toolpath::DrillCycle& c) {
                    return c.type == type;
                })) {
                throw Error(ErrorCode::InvalidArgument,
                            fmt::format("this post takes no {} cycle, and plain moves cannot "
                                        "reverse the spindle at the bottom of the hole: add \"{}\" "
                                        "to the cycles in its manifest, or choose another post",
                                        type == toolpath::CycleType::Tap ? "tap" : "left-hand tap",
                                        toolpath::toString(type)));
            }
        }
        // Cycles the post does not take become plain moves.
        const toolpath::Toolpath path = !options.cannedCycles ? toolpath::expandCycles(job.path)
                                        : options.cycleTypes.empty()
                                            ? job.path
                                            : toolpath::expandCycles(job.path, options.cycleTypes);
        if (path.moves.empty()) {
            continue;
        }
        // Rotary positions the job starts at: its 3+2 index or the start of its multi-axis
        // motion.
        std::optional<machine::IndexAngles> startRotary = job.index;
        std::string rotaryAxes = job.indexAxes;
        if (job.rotary) {
            startRotary = job.rotary->start;
            rotaryAxes = job.rotary->axes;
        }

        NcOperation operation;
        operation.id = job.operationId;
        operation.name = job.name;
        operation.comment = job.comment;
        operation.strategy = path.strategy;
        operation.tool = path.toolNumber;
        operation.coolant = path.coolant;
        operation.workOffset = job.workOffset;
        operation.subSpindle = job.subSpindle;
        operation.orientation = job.rotary  ? NcOrientation::Simultaneous
                                : job.index ? NcOrientation::Indexed
                                            : NcOrientation::ThreeAxis;
        operation.index = job.index.value_or(machine::IndexAngles{});
        motionBounds(path, operation.boundsMin, operation.boundsMax);
        const int operationIndex = static_cast<int>(program.operations.size());
        program.operations.push_back(std::move(operation));
        if (std::ranges::none_of(program.tools,
                                 [&](const NcTool& t) { return t.number == path.toolNumber; })) {
            NcTool tool = job.tool.value_or(NcTool{});
            tool.number = path.toolNumber;
            if (!job.tool) {
                tool.name = job.toolName;
            }
            program.tools.push_back(std::move(tool));
        }
        NcInstr begin = instr(NcOp::OperationBegin);
        begin.operation = operationIndex;
        program.instrs.push_back(begin);

        const bool toolChange = path.toolNumber != currentTool;
        if (toolChange) {
            setCoolant(tooling::Coolant::Off);
            setLive(0.0);
            setSpindle(0.0);
            if (options.indexAtHome && currentTool >= 0) {
                stopCycle();
                program.instrs.push_back(instr(NcOp::HomeZ)); // a turret clear of it all (DL-0067)
            }
            NcInstr tc = instr(NcOp::ToolChange);
            tc.tool = path.toolNumber;
            tc.toolName = job.toolName;
            program.instrs.push_back(tc);
            currentTool = path.toolNumber;
        }
        // A live tool: the part stands still, its spindle a C axis. Back to turning, the other way.
        if (path.live) {
            setSpindle(0.0);
            currentIndex.reset(); // the C axis is engaged afresh
            std::optional<toolpath::ArcPlane> milling;
            if (path.liveAxes == toolpath::LiveAxes::HeldC) {
                milling = path.liveAcross ? toolpath::ArcPlane::YZ : toolpath::ArcPlane::XY;
            }
            setLive(path.spindleSpeed, path.strategy == "tap", milling);
        } else {
            setLive(0.0);
        }
        const bool reindex = startRotary && startRotary != currentIndex;
        if (reindex && positioned && !toolChange && !path.live) {
            stopCycle();
            program.instrs.push_back(instr(NcOp::HomeZ)); // clear the part before turning it
        }
        if (toolChange || job.workOffset != currentOffset) {
            NcInstr wo = instr(NcOp::WorkOffset);
            wo.workOffset = job.workOffset;
            program.instrs.push_back(wo);
            currentOffset = job.workOffset;
        }
        if (reindex) {
            NcInstr ix = instr(NcOp::Index);
            ix.rotary = Vec3(startRotary->a, startRotary->b, startRotary->c);
            ix.text = rotaryAxes.empty() ? "AC" : rotaryAxes;
            program.instrs.push_back(ix);
            currentIndex = startRotary;
        }
        positioned = true;

        // Position above the start, start the spindle, apply tool length at the start height. A
        // lathe goes along Z first, then in on X (DL-0067).
        const double kUnset = std::numeric_limits<double>::quiet_NaN();
        const bool homeFirst = toolChange && options.indexAtHome;
        const bool zFirst =
            options.latheApproach &&
            !(lastEnd && !homeFirst && machine::approachOutFirst(*lastEnd, path.start));
        for (const toolpath::Move& m : path.moves) {
            if (m.kind != toolpath::MoveKind::Dwell) {
                lastEnd = m.end;
            }
        }
        const auto placeXY = [&] {
            NcInstr xy = instr(NcOp::Rapid);
            xy.target = Vec3(path.start.x(), path.start.y(), kUnset);
            program.instrs.push_back(xy);
        };
        const auto spindleOn = [&] {
            if (!path.live) {
                setSpindle(path.spindleSpeed, path.surfaceSpeed, path.maxSpindle,
                           path.spindleReverse);
            }
        };
        const bool tcp = job.rotary && job.rotary->tcp;
        const auto placeZ = [&] {
            const Vec3 target = zFirst ? Vec3(kUnset, kUnset, path.start.z()) : path.start;
            if (toolChange || tcp || needToolLength) {
                NcInstr tl = instr(NcOp::ToolLength);
                tl.tool = path.toolNumber;
                tl.target = target;
                tl.tcp = tcp;
                program.instrs.push_back(tl);
                needToolLength = false;
            } else {
                NcInstr z = instr(NcOp::Rapid);
                z.target = target;
                program.instrs.push_back(z);
            }
        };
        if (zFirst) {
            placeZ();
            spindleOn();
            placeXY();
        } else {
            placeXY();
            spindleOn();
            placeZ();
        }
        setCoolant(path.coolant);

        const bool compensated = path.compensation != toolpath::Compensation::None;
        bool compActive = false;
        Vec3 pos = path.start;
        machine::IndexAngles rot = startRotary.value_or(machine::IndexAngles{});
        // Polar interpolation, around each run of feed moves: rapids go in X and C without it.
        const bool polar = path.liveAxes == toolpath::LiveAxes::Polar;
        bool polarOn = false;
        auto setPolar = [&](bool on) {
            if (on == polarOn) {
                return;
            }
            NcInstr i = instr(NcOp::Polar);
            i.polarOn = on;
            if (on) {
                pos = Vec3(pos.x() * std::cos(rot.c), pos.x() * std::sin(rot.c), pos.z());
                i.target = pos;
            }
            program.instrs.push_back(i);
            polarOn = on;
        };
        // Cylindrical interpolation, around each run of feed moves that turns C: on the deepest
        // radius of the run, the floor of its level, where the feed is the one on the page.
        const bool wrap = path.liveAxes == toolpath::LiveAxes::Wrap;
        bool wrapOn = false;
        auto setWrap = [&](bool on, std::size_t from = 0) {
            if (on == wrapOn) {
                return;
            }
            NcInstr i = instr(NcOp::Wrap);
            i.polarOn = on;
            if (on) {
                double radius = pos.x();
                double before = job.rotary && from > 0 ? job.rotary->angles[from - 1].c : rot.c;
                bool turns = false;
                for (std::size_t k = from; k < path.moves.size(); ++k) {
                    if (path.moves[k].kind == MoveKind::Rapid) {
                        break;
                    }
                    if (path.moves[k].kind != MoveKind::Dwell) {
                        radius = std::min(radius, path.moves[k].end.x());
                        turns = turns || std::abs(job.rotary->angles[k].c - before) > 1e-12;
                    }
                }
                if (!turns) {
                    return; // X and Z only: plain moves
                }
                i.wrapRadius = radius;
            }
            program.instrs.push_back(i);
            wrapOn = on;
        };
        std::optional<double> oriented; // the spindle stands at this angle (DL-0083)
        for (std::size_t mi = 0; mi < path.moves.size(); ++mi) {
            const toolpath::Move& m = path.moves[mi];
            // A tool passing through a hole off centre: the spindle stops at its angle first, and
            // starts again for the next move that is not.
            if (m.kind != MoveKind::Dwell && m.orient != oriented) {
                stopCycle();
                if (m.orient) {
                    NcInstr o = instr(NcOp::SpindleOrient);
                    o.angle = *m.orient;
                    program.instrs.push_back(o);
                    currentSpindle = 0.0;
                } else {
                    spindleOn();
                }
                oriented = m.orient;
            }
            // Multi-axis toolpaths position the rotary axes with every move.
            auto withRotary = [&](NcInstr& i) {
                if (!job.rotary || mi >= job.rotary->angles.size()) {
                    return;
                }
                const machine::IndexAngles& next = job.rotary->angles[mi];
                i.axes = job.rotary->axes;
                i.rotary = Vec3(next.a, next.b, next.c);
                const bool turns = next != rot;
                if (i.op == NcOp::Linear && !job.rotary->tcp && turns &&
                    mi < job.rotary->times.size() && job.rotary->times[mi] > 0.0) {
                    i.inverseTime = 60.0 / job.rotary->times[mi];
                }
                rot = next;
            };
            // A feed move to `target` with this move's feed (also the lines an arc becomes):
            // cutter compensation starts on the first one that moves in XY.
            auto linear = [&](const Vec3& target, bool rotary) {
                const bool movesXY = (target.head<2>() - pos.head<2>()).norm() > 1e-9;
                if (compensated && !compActive && movesXY) {
                    NcInstr on = instr(NcOp::CompensationOn);
                    on.side = path.compensation;
                    on.tool = path.toolNumber;
                    program.instrs.push_back(on);
                    compActive = true;
                }
                NcInstr i = instr(NcOp::Linear);
                i.target = target;
                i.feed = m.feedRate;
                i.feedClass = m.feedClass;
                i.pitch = m.pitch;
                if (rotary) {
                    withRotary(i);
                }
                program.instrs.push_back(i);
            };
            if (m.kind != MoveKind::Cycle) {
                stopCycle();
            }
            if (polar && m.kind != MoveKind::Dwell) {
                setPolar(m.kind != MoveKind::Rapid);
            }
            if (wrap && m.kind != MoveKind::Dwell) {
                setWrap(m.kind != MoveKind::Rapid, mi);
            }
            switch (m.kind) {
            case MoveKind::Rapid: {
                if (compActive) {
                    program.instrs.push_back(instr(NcOp::CompensationOff));
                    compActive = false;
                }
                NcInstr i = instr(NcOp::Rapid);
                i.target = m.end;
                withRotary(i);
                program.instrs.push_back(i);
                break;
            }
            case MoveKind::Linear: {
                if (path.live && m.pitch > 0.0) {
                    // In and out are one block, on a control's tapping cycle.
                    NcInstr i = instr(NcOp::LiveTap);
                    i.target = m.end;
                    i.spindle = path.spindleSpeed;
                    i.pitch = m.pitch;
                    i.radial = std::abs(m.end.x() - pos.x()) > std::abs(m.end.z() - pos.z());
                    i.reverse = path.spindleReverse;
                    i.rPlane = i.radial ? pos.x() : pos.z();
                    withRotary(i);
                    program.instrs.push_back(i);
                    cycleActive = true;
                    if (mi + 1 < path.moves.size()) {
                        ++mi; // and back out
                    }
                    continue;
                }
                linear(m.end, true);
                break;
            }
            case MoveKind::ArcCW:
            case MoveKind::ArcCCW: {
                const bool planeTaken =
                    std::ranges::find(options.arcPlanes, m.plane) != options.arcPlanes.end();
                const bool helical = std::abs(toolpath::planeNormal(m.end, m.plane) -
                                              toolpath::planeNormal(pos, m.plane)) > 1e-9;
                if (!planeTaken || (helical && !options.helicalArcs)) {
                    // The post cannot take this arc: lines within the tolerance.
                    const std::vector<Vec3> points = toolpath::sampleArc(pos, m, options.tolerance);
                    for (std::size_t k = 1; k < points.size(); ++k) {
                        linear(points[k], false);
                        pos = points[k];
                    }
                    break;
                }
                if (compensated && !compActive) {
                    program.warnings.push_back(fmt::format(
                        "{}: cutter compensation must start on a straight move; add a lead-in line",
                        job.name));
                }
                // Arcs longer than the post takes are split into equal parts (the same split the
                // C++ formatter made).
                const Vec2 c = toolpath::planeCoords(m.center, m.plane);
                const Vec2 s = toolpath::planeCoords(pos, m.plane);
                const Vec2 e = toolpath::planeCoords(m.end, m.plane);
                const double r = (s - c).norm();
                const double a0 = std::atan2(s.y() - c.y(), s.x() - c.x());
                const double a1 = std::atan2(e.y() - c.y(), e.x() - c.x());
                double sweep = std::fmod(a1 - a0, 2.0 * kPi);
                const bool cw = m.kind == MoveKind::ArcCW;
                if (!cw && sweep <= 1e-12) {
                    sweep += 2.0 * kPi;
                } else if (cw && sweep >= -1e-12) {
                    sweep -= 2.0 * kPi;
                }
                const int parts = std::max(
                    1, static_cast<int>(std::ceil(std::abs(sweep) / options.maxArcSweep - 1e-6)));
                const double n0 = toolpath::planeNormal(pos, m.plane);
                const double n1 = toolpath::planeNormal(m.end, m.plane);
                for (int k = 1; k <= parts; ++k) {
                    const double t = static_cast<double>(k) / parts;
                    const double a = a0 + sweep * t;
                    NcInstr i = instr(NcOp::Arc);
                    i.target =
                        k == parts
                            ? m.end
                            : toolpath::fromPlaneCoords(c + r * Vec2(std::cos(a), std::sin(a)),
                                                        n0 + (n1 - n0) * t, m.plane);
                    i.center = m.center;
                    i.clockwise = cw;
                    i.plane = m.plane;
                    i.feed = m.feedRate;
                    i.feedClass = m.feedClass;
                    program.instrs.push_back(i);
                }
                break;
            }
            case MoveKind::Dwell: {
                // What the machine does standing here, then the dwell (DL-0091).
                if (m.function != toolpath::MachineFunction::None) {
                    const bool chuck = m.function == toolpath::MachineFunction::ChuckOpen ||
                                       m.function == toolpath::MachineFunction::ChuckClose;
                    if (m.function == toolpath::MachineFunction::ChuckOpen &&
                        (currentSpindle != 0.0 || currentLive != 0.0)) {
                        throw Error(ErrorCode::InvalidArgument,
                                    fmt::format("{}: the chuck would open with the spindle "
                                                "turning",
                                                job.name));
                    }
                    NcInstr f = instr(chuck ? NcOp::Chuck : NcOp::BarFeeder);
                    f.open = m.function == toolpath::MachineFunction::ChuckOpen;
                    f.polarOn = m.function == toolpath::MachineFunction::BarFeederOn;
                    program.instrs.push_back(f);
                    if (m.dwell <= 0.0) {
                        break;
                    }
                }
                NcInstr i = instr(NcOp::Dwell);
                i.dwell = m.dwell;
                program.instrs.push_back(i);
                break;
            }
            case MoveKind::Probe: {
                // A probing cycle is one instruction with all its moves (DL-0090).
                NcInstr i = instr(NcOp::Probe);
                i.probe = path.probes.at(static_cast<std::size_t>(m.probe));
                i.operation = operationIndex;
                std::size_t k = mi;
                for (; k < path.moves.size() && path.moves[k].kind == MoveKind::Probe &&
                       path.moves[k].probe == m.probe;
                     ++k) {
                    const toolpath::Move& pm = path.moves[k];
                    NcProbeMove move;
                    move.measure = pm.measure;
                    move.feed = pm.feedRate;
                    move.target = pm.end;
                    if (pm.measure) {
                        move.expected = pm.end;
                        if ((pm.end - pos).norm() > 1e-12) {
                            move.target = pm.end + (pm.end - pos).normalized() * i.probe.overtravel;
                        }
                    }
                    i.probeMoves.push_back(move);
                    pos = pm.end;
                }
                program.instrs.push_back(i);
                mi = k - 1;
                continue;
            }
            case MoveKind::Cycle: {
                NcInstr i = instr(NcOp::Cycle);
                i.target = m.end;
                i.cycle = path.cycles.at(static_cast<std::size_t>(m.cycle));
                program.instrs.push_back(i);
                cycleActive = true;
                break;
            }
            }
            if (m.kind != MoveKind::Dwell) {
                pos = m.end;
            }
        }
        stopCycle();
        if (oriented) {
            spindleOn();
        }
        setPolar(false);
        setWrap(false);
        if (compActive) {
            program.instrs.push_back(instr(NcOp::CompensationOff));
        }
        if (job.rotary && !job.rotary->angles.empty()) {
            currentIndex = job.rotary->angles.back();
        }
        if (tcp) {
            program.instrs.push_back(instr(NcOp::TcpOff));
            needToolLength = true;
        }
        NcInstr end = instr(NcOp::OperationEnd);
        end.operation = operationIndex;
        program.instrs.push_back(end);
    }
    stopCycle();
    setCoolant(tooling::Coolant::Off);
    setLive(0.0);
    setSpindle(0.0);
    return program;
}

} // namespace ocam::post
