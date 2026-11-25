// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Inverse kinematics of the supported rotary configurations follows from their rotation
// sequences (closed form, two branches each); see docs/design-log/0023.

#include "ocam/machine/Kinematics.hpp"

#include "SolveDetail.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <functional>

namespace ocam::machine {
namespace {

using toolpath::Move;
using toolpath::MoveKind;

Mat3 rotZ(double t) {
    return Eigen::AngleAxisd(t, Vec3::UnitZ()).toRotationMatrix();
}

double degrees(double rad) {
    return rad * 180.0 / kPi;
}

/// Limits of a rotary axis [rad]; an axis the machine does not list is fixed at 0.
struct Range {
    double lo = 0.0;
    double hi = 0.0;
    bool contains(double v) const noexcept { return v >= lo - 1e-9 && v <= hi + 1e-9; }
};

Range rangeOf(const Machine& m, const char* name) {
    if (const Axis* a = m.axis(name)) {
        return {a->min * kPi / 180.0, a->max * kPi / 180.0};
    }
    return {};
}

/// The value r + 2 pi k within `range` nearest to `target`, if any.
std::optional<double> unwrapNear(double r, double target, const Range& range) {
    std::optional<double> best;
    const double k0 = std::round((target - r) / (2.0 * kPi));
    for (int dk = -2; dk <= 2; ++dk) {
        const double v = r + 2.0 * kPi * (k0 + dk);
        if (range.contains(v) && (!best || std::abs(v - target) < std::abs(*best - target))) {
            best = v;
        }
    }
    return best;
}

double& rotating(IndexAngles& s, Kinematics k) {
    return k == Kinematics::Mill4A ? s.a : s.c;
}
double rotating(const IndexAngles& s, Kinematics k) {
    return k == Kinematics::Mill4A ? s.a : s.c;
}

double jointStep(const IndexAngles& x, const IndexAngles& y) {
    return std::max({std::abs(x.a - y.a), std::abs(x.b - y.b), std::abs(x.c - y.c)});
}

/// The rapid moves that run at or above the toolpath's start height (its clearance height),
/// where the rotary table may turn whole turns back (planRotary's `unwind`).
std::vector<char> clearRapids(const Vec3& start, const std::vector<Move>& moves) {
    std::vector<char> out(moves.size(), 0);
    Vec3 at = start;
    for (std::size_t k = 0; k < moves.size(); ++k) {
        out[k] = moves[k].kind == MoveKind::Rapid && at.z() >= start.z() - 1e-6 &&
                 moves[k].end.z() >= start.z() - 1e-6;
        if (moves[k].kind != MoveKind::Dwell) {
            at = moves[k].end;
        }
    }
    return out;
}

} // namespace

Vec3 toolAxis(const Machine& machine, const IndexAngles& angles) {
    return tableRotation(machine, angles).transpose() *
           (headRotation(machine, angles) * Vec3::UnitZ());
}

std::vector<IndexAngles> axisSolutions(const Machine& machine, const Vec3& axis,
                                       const IndexAngles* near) {
    std::vector<IndexAngles> out;
    if (!machine.hasRotary()) {
        if ((axis.normalized() - Vec3::UnitZ()).norm() < 1e-9) {
            out.emplace_back();
        }
        return out;
    }
    const Vec3 u = axis.normalized();
    const Kinematics k = machine.kinematics;
    const std::vector<std::string> names = [&] {
        std::vector<std::string> v;
        for (const char c : machine.rotaryAxes()) {
            v.emplace_back(1, c);
        }
        return v;
    }();
    const Range rot = rangeOf(machine, names.back().c_str());
    const Range tilt = names.size() > 1 ? rangeOf(machine, names.front().c_str()) : Range{};
    const double target = near ? rotating(*near, k) : 0.0;

    // Base solutions: (tilt, rotation) before unwrapping.
    std::vector<std::pair<double, double>> base;
    if (k == Kinematics::Mill4A) {
        if (std::abs(u.x()) < 1e-7) {
            base.emplace_back(0.0, std::atan2(u.y(), u.z()));
        }
    } else {
        const double horizontal = std::hypot(u.x(), u.y());
        const bool singular = horizontal < 1e-9;
        double c0 = 0.0;
        switch (k) {
        case Kinematics::Mill5TableAC:
            c0 = std::atan2(u.x(), u.y());
            break;
        case Kinematics::Mill5TableBC:
        case Kinematics::Mill5HeadTableBC:
            c0 = std::atan2(-u.y(), u.x());
            break;
        default: // Mill5HeadBC
            c0 = std::atan2(u.y(), u.x());
            break;
        }
        if (singular) {
            c0 = target; // the rotating axis does not change the tool axis: keep it
        }
        for (const double c : {c0, c0 + kPi}) {
            const bool head = k == Kinematics::Mill5HeadBC;
            const Vec3 w = head ? Vec3(rotZ(-c) * u) : Vec3(rotZ(c) * u);
            double t = 0.0;
            switch (k) {
            case Kinematics::Mill5TableAC:
                t = std::atan2(w.y(), w.z());
                break;
            case Kinematics::Mill5TableBC:
                t = std::atan2(-w.x(), w.z());
                break;
            default: // the head tilts towards +X of its turned frame
                t = std::atan2(w.x(), w.z());
                break;
            }
            base.emplace_back(t, c);
            if (singular && std::abs(u.z() - 1.0) < 1e-12) {
                break; // straight up: the second branch only flips C
            }
        }
    }
    for (const auto& [t, r] : base) {
        if (!tilt.contains(t)) {
            continue;
        }
        const auto unwrapped = unwrapNear(r, target, rot);
        if (!unwrapped) {
            continue;
        }
        IndexAngles s;
        rotating(s, k) = *unwrapped;
        if (k == Kinematics::Mill5TableAC) {
            s.a = t;
        } else if (k != Kinematics::Mill4A) {
            s.b = t;
        }
        if ((toolAxis(machine, s) - u).norm() > 1e-7) {
            continue;
        }
        const bool duplicate = std::any_of(
            out.begin(), out.end(), [&](const IndexAngles& o) { return jointStep(o, s) < 1e-12; });
        if (!duplicate) {
            out.push_back(s);
        }
    }
    return out;
}

Vec3 machinePosition(const Machine& machine, const IndexAngles& angles, const Vec3& tip,
                     double toolLength) {
    const Vec3 d = headRotation(machine, angles) * Vec3::UnitZ();
    return tableRotation(machine, angles) * tip +
           (machine.pivotLength + toolLength) * (d - Vec3::UnitZ());
}

Vec3 tipFromMachine(const Machine& machine, const IndexAngles& angles, const Vec3& position,
                    double toolLength) {
    const Vec3 d = headRotation(machine, angles) * Vec3::UnitZ();
    return tableRotation(machine, angles).transpose() *
           (position - (machine.pivotLength + toolLength) * (d - Vec3::UnitZ()));
}

RotaryPlan planRotary(const Machine& machine, const std::vector<Vec3>& axisInTable,
                      const std::vector<char>& unwind) {
    RotaryPlan best;
    const std::string rotaries = machine.rotaryAxes();
    const char rotatingName[2] = {rotaries.empty() ? '\0' : rotaries.back(), '\0'};
    const Range rot = rangeOf(machine, rotatingName);
    if (axisInTable.empty()) {
        return best;
    }
    std::vector<IndexAngles> starts = axisSolutions(machine, axisInTable.front());
    if (starts.empty()) {
        best.errors.push_back(
            fmt::format("move 1: the tool axis ({:.4f}, {:.4f}, {:.4f}) cannot be reached within "
                        "the rotary axis limits",
                        axisInTable.front().x(), axisInTable.front().y(), axisInTable.front().z()));
        best.angles.assign(axisInTable.size(), IndexAngles{});
        return best;
    }
    double bestTravel = std::numeric_limits<double>::infinity();
    bool first = true;
    for (const IndexAngles& start : starts) {
        RotaryPlan plan;
        plan.start = start;
        IndexAngles prev = start;
        double travel = 0.0;
        for (std::size_t k = 0; k < axisInTable.size(); ++k) {
            const std::vector<IndexAngles> sols = axisSolutions(machine, axisInTable[k], &prev);
            if (sols.empty()) {
                if (plan.errors.size() < 5) {
                    plan.errors.push_back(fmt::format(
                        "move {}: the tool axis ({:.4f}, {:.4f}, {:.4f}) cannot be reached within "
                        "the rotary axis limits",
                        k + 1, axisInTable[k].x(), axisInTable[k].y(), axisInTable[k].z()));
                }
                plan.angles.push_back(prev);
                continue;
            }
            IndexAngles next = *std::min_element(sols.begin(), sols.end(),
                                                 [&](const IndexAngles& x, const IndexAngles& y) {
                                                     return jointStep(prev, x) < jointStep(prev, y);
                                                 });
            if (k < unwind.size() && unwind[k]) {
                // Clear of the part: whole turns back towards zero, the tool axis unchanged.
                double& c = rotating(next, machine.kinematics);
                while (c > kPi + 1e-9 && rot.contains(c - 2.0 * kPi)) {
                    c -= 2.0 * kPi;
                }
                while (c < -kPi - 1e-9 && rot.contains(c + 2.0 * kPi)) {
                    c += 2.0 * kPi;
                }
            }
            travel += jointStep(prev, next);
            plan.angles.push_back(next);
            prev = next;
        }
        const bool better =
            first || plan.errors.size() < best.errors.size() ||
            (plan.errors.size() == best.errors.size() && travel < bestTravel - 1e-9);
        if (better) {
            best = std::move(plan);
            bestTravel = travel;
            first = false;
        }
    }
    return best;
}

std::optional<IndexAngles> indexSolution(const Machine& machine, const Vec3& axis) {
    if (machine.canIndex()) {
        return indexAngles(machine, axis);
    }
    const IndexAngles zero;
    const std::vector<IndexAngles> sols = axisSolutions(machine, axis, &zero);
    if (sols.empty()) {
        return std::nullopt;
    }
    return *std::min_element(
        sols.begin(), sols.end(), [](const IndexAngles& x, const IndexAngles& y) {
            if (std::abs(std::abs(x.b) - std::abs(y.b)) > 1e-9) {
                return std::abs(x.b) < std::abs(y.b);
            }
            return std::abs(x.a) + std::abs(x.c) < std::abs(y.a) + std::abs(y.c);
        });
}

SolveResult solveIndexed(const Machine& machine, const toolpath::Toolpath& path,
                         const Transform3& frameInTable, const IndexAngles& angles,
                         const WorkOrigin& origin, const SolveOptions& options) {
    SolveResult result;
    result.path = path;
    if (machine.isLathe()) {
        result.errors.push_back("a tool orientation of its own needs a milling machine");
        return result;
    }
    if (toolpath::isMultiAxis(path)) {
        result.errors.push_back("internal: a positional toolpath must keep the tool along its "
                                "frame's Z axis");
        return result;
    }
    const Vec3 u = toolAxis(machine, angles);
    if ((frameInTable.linear().col(2).normalized() - u).norm() > 1e-7) {
        result.errors.push_back(
            "internal: the rotary positions do not turn the tool orientation up the spindle");
        return result;
    }
    const Mat3 T = tableRotation(machine, angles);
    const Vec3 d = headRotation(machine, angles) * Vec3::UnitZ();
    const bool tiltedHead = (d - Vec3::UnitZ()).norm() > 1e-9;
    if (tiltedHead) {
        for (const toolpath::DrillCycle& c : path.cycles) {
            if (toolpath::isTap(c.type)) {
                result.errors.push_back("tapping needs the tool along the machine Z axis; this "
                                        "orientation tilts the head");
                return result;
            }
        }
    }
    // Machine position of a tip p (frame): T (F p) + (pivot + L)(d - Z), less the work offset's.
    Transform3 toProgram = Transform3::Identity();
    toProgram.linear() = T * frameInTable.linear();
    toProgram.translation() = T * frameInTable.translation() +
                              (machine.pivotLength + options.toolLength) * (d - Vec3::UnitZ()) -
                              tableRotation(machine, origin.angles) * origin.point;
    result.path = toolpath::transformed(path, toProgram, std::max(1e-4, options.tolerance / 2.0));
    for (Move& m : result.path.moves) {
        m.axis = u;
    }
    result.index = angles;
    detail::applyLimits(machine, result);
    return result;
}

namespace {

/// A live tool going around what it cuts with C held still (DL-0066): the WCS turned by C, so X, Y
/// and Z are what is left. On the face C is the operation's; across the part, the tool's direction.
SolveResult solveLiveHeld(const Machine& machine, const toolpath::Toolpath& path) {
    SolveResult result;
    toolpath::Toolpath out = toolpath::expandCycles(path);
    const Vec3 toolAxis = out.moves.empty() ? Vec3::UnitZ() : out.moves.front().axis;
    out.liveAcross = std::abs(toolAxis.z()) <= 1.0 - 1e-9;
    const double c = out.liveAcross ? std::atan2(toolAxis.y(), toolAxis.x()) : path.liveC;
    const double cosC = std::cos(c);
    const double sinC = std::sin(c);
    const auto turned = [&](const Vec3& p) {
        return Vec3(cosC * p.x() + sinC * p.y(), -sinC * p.x() + cosC * p.y(), p.z());
    };
    IndexAngles q;
    q.c = c < 0.0 ? c + 2.0 * kPi : c;
    RotaryMotion motion;
    motion.axes = "C";
    motion.tcp = false;
    motion.start = q;
    out.start = turned(path.start);
    double furthestY = std::abs(out.start.y());
    for (toolpath::Move& m : out.moves) {
        if (m.kind != toolpath::MoveKind::Dwell) {
            m.end = turned(m.end);
            furthestY = std::max(furthestY, std::abs(m.end.y()));
        }
        if (m.isArc()) {
            m.center = turned(m.center);
        }
        m.axis = Vec3(std::cos(q.c), std::sin(q.c), 0.0);
        motion.angles.push_back(q);
    }
    if (furthestY > 1e-6 && !machine.hasYAxis()) {
        result.errors.push_back(fmt::format(
            "the toolpath goes {:.3f} mm off the spindle's centre in Y, and machine '{}' has no Y "
            "axis: use a machine with one, or polar interpolation on the part's face",
            furthestY, machine.name));
    }
    result.path = std::move(out);
    result.rotary = std::move(motion);
    detail::applyLimits(machine, result);
    return result;
}

/// A live tool around the part by cylindrical interpolation (DL-0074): X as the radius, Z, and C
/// going on past a turn, as the control has to take it. A run of pieces on one straight line
/// unrolled comes back as the one line it was, since the control interpolates there.
SolveResult solveLiveWrap(const Machine& machine, const toolpath::Toolpath& path) {
    SolveResult result;
    toolpath::Toolpath out = path;
    out.moves.clear();
    RotaryMotion motion;
    motion.axes = "C";
    motion.tcp = false;
    double c = std::atan2(path.start.y(), path.start.x());
    const auto angles = [&] {
        IndexAngles q;
        q.c = c;
        return q;
    };
    const auto unrolled = [&](const Vec3& p) {
        const double next = std::atan2(p.y(), p.x());
        c += std::remainder(next - c, 2.0 * kPi);
        return Vec3(p.head<2>().norm(), 0.0, p.z());
    };
    out.start = unrolled(path.start);
    motion.start = angles();
    Vec3 from = out.start;
    double fromC = c;
    for (const toolpath::Move& m : path.moves) {
        toolpath::Move next = m;
        if (m.kind == toolpath::MoveKind::Dwell) {
            next.end = from;
            next.axis = Vec3(std::cos(c), std::sin(c), 0.0);
            out.moves.push_back(next);
            motion.angles.push_back(angles());
            continue;
        }
        next.end = unrolled(m.end);
        next.axis = Vec3(std::cos(c), std::sin(c), 0.0);
        // In (radius, angle, z) at a length scale that makes them comparable.
        const Vec3 step(next.end.x() - from.x(), (c - fromC) * next.end.x(),
                        next.end.z() - from.z());
        if (!out.moves.empty()) {
            const toolpath::Move& last = out.moves.back();
            const double lastC = motion.angles.back().c;
            const double beforeC = motion.angles.size() > 1
                                       ? motion.angles[motion.angles.size() - 2].c
                                       : motion.start.c;
            const Vec3 lastFrom =
                out.moves.size() > 1 ? out.moves[out.moves.size() - 2].end : out.start;
            const Vec3 lastStep(last.end.x() - lastFrom.x(), (lastC - beforeC) * next.end.x(),
                                last.end.z() - lastFrom.z());
            const bool sameLine =
                last.kind == next.kind && last.feedRate == next.feedRate &&
                last.feedClass == next.feedClass && step.norm() > 1e-12 &&
                lastStep.norm() > 1e-12 && step.dot(lastStep) > 0.0 &&
                step.cross(lastStep).norm() <= 1e-9 * step.norm() * lastStep.norm();
            if (sameLine) {
                out.moves.back().end = next.end;
                out.moves.back().axis = next.axis;
                motion.angles.back() = angles();
                from = next.end;
                fromC = c;
                continue;
            }
        }
        out.moves.push_back(next);
        motion.angles.push_back(angles());
        from = next.end;
        fromC = c;
    }
    result.path = std::move(out);
    result.rotary = std::move(motion);
    detail::applyLimits(machine, result);
    return result;
}

/// A live tool on the part's face by polar interpolation (DL-0066): rapids in X and C, with the
/// control's polar mode off, and feed moves in the WCS's X and Y for the control to turn into X and
/// C. A helix becomes lines, as controls differ on moving Z with an arc in that mode.
SolveResult solveLivePolar(const Machine& machine, const toolpath::Toolpath& path,
                           double tolerance) {
    SolveResult result;
    const toolpath::Toolpath src = toolpath::expandCycles(path);
    toolpath::Toolpath out = src;
    out.moves.clear();
    RotaryMotion motion;
    motion.axes = "C";
    motion.tcp = false;
    double c = 0.0;
    const auto angles = [&] {
        IndexAngles q;
        q.c = c < 0.0 ? c + 2.0 * kPi : c;
        return q;
    };
    const auto polarOf = [&](const Vec3& p) {
        const double r = p.head<2>().norm();
        if (r > 1e-9) {
            c = std::atan2(p.y(), p.x());
        }
        return Vec3(r, 0.0, p.z());
    };
    out.start = polarOf(src.start);
    motion.start = angles();
    const auto add = [&](toolpath::Move m) {
        const IndexAngles q = angles();
        m.axis = Vec3(std::cos(q.c), std::sin(q.c), 0.0);
        out.moves.push_back(m);
        motion.angles.push_back(q);
    };
    Vec3 pos = src.start;
    for (const toolpath::Move& m : src.moves) {
        if (m.kind == toolpath::MoveKind::Rapid) {
            toolpath::Move rapid = m;
            rapid.end = polarOf(m.end);
            add(rapid);
        } else if (m.isArc() && std::abs(toolpath::planeNormal(m.end, m.plane) -
                                         toolpath::planeNormal(pos, m.plane)) > 1e-9) {
            const std::vector<Vec3> points = toolpath::sampleArc(pos, m, tolerance);
            for (std::size_t k = 1; k < points.size(); ++k) {
                toolpath::Move line = m;
                line.kind = toolpath::MoveKind::Linear;
                line.plane = toolpath::ArcPlane::XY;
                line.center = Vec3::Zero();
                line.end = points[k];
                add(line);
            }
        } else {
            add(m);
        }
        if (m.kind != toolpath::MoveKind::Dwell) {
            pos = m.end;
        }
    }
    // Against the machine's travels and limits the feed moves stand where X and C take them.
    SolveResult limits;
    limits.path = out;
    for (toolpath::Move& m : limits.path.moves) {
        if (m.kind != toolpath::MoveKind::Rapid && m.kind != toolpath::MoveKind::Dwell) {
            m.kind = toolpath::MoveKind::Linear;
            m.end = Vec3(m.end.head<2>().norm(), 0.0, m.end.z());
        }
    }
    detail::applyLimits(machine, limits);
    result.errors = limits.errors;
    result.warnings = limits.warnings;
    result.path = std::move(out);
    result.rotary = std::move(motion);
    return result;
}

/// A live tool's toolpath on a lathe, from the WCS to the machine (DL-0063): the part turned by C
/// until what the tool goes for is in the plane of X, then X (radius) and Z, and Y where the
/// tool's axis does not cross the spindle's. Along Z, C turns the hole onto +X and X is its radius.
/// Across the part, C turns the tool's axis onto +X and Y is what is left. Each move's `axis` holds
/// (cos C, sin C, 0), for checking the program's C words against (post::compareBackplot).
SolveResult solveLiveLathe(const Machine& machine, const toolpath::Toolpath& path) {
    SolveResult result;
    toolpath::Toolpath out = toolpath::expandCycles(path);
    RotaryMotion motion;
    motion.axes = "C";
    motion.tcp = false;
    double c = 0.0;
    const auto place = [&](const Vec3& p, const Vec3& axis) {
        if (std::abs(axis.z()) > 1.0 - 1e-9) {
            const double r = p.head<2>().norm();
            if (r > 1e-9) {
                c = std::atan2(p.y(), p.x());
            }
            return Vec3(r, 0.0, p.z());
        }
        c = std::atan2(axis.y(), axis.x());
        return Vec3(std::cos(c) * p.x() + std::sin(c) * p.y(),
                    -std::sin(c) * p.x() + std::cos(c) * p.y(), p.z());
    };
    const auto angles = [&] {
        IndexAngles q;
        q.c = c < 0.0 ? c + 2.0 * kPi : c;
        return q;
    };
    const Vec3 firstAxis = out.moves.empty() ? Vec3::UnitZ() : out.moves.front().axis;
    out.start = place(path.start, firstAxis);
    motion.start = angles();
    double furthestY = 0.0;
    for (toolpath::Move& m : out.moves) {
        if (m.kind != toolpath::MoveKind::Dwell) {
            m.end = place(m.end, m.axis);
            furthestY = std::max(furthestY, std::abs(m.end.y()));
        }
        const IndexAngles q = angles();
        m.axis = Vec3(std::cos(q.c), std::sin(q.c), 0.0);
        motion.angles.push_back(q);
    }
    if (furthestY > 1e-6 && !machine.hasYAxis()) {
        result.errors.push_back(fmt::format(
            "the toolpath goes {:.3f} mm off the spindle's centre in Y, and machine '{}' has no Y "
            "axis: use a machine with one, or a hole across the part that crosses its axis",
            furthestY, machine.name));
    }
    result.path = std::move(out);
    result.rotary = std::move(motion);
    detail::applyLimits(machine, result);
    return result;
}

} // namespace

SolveResult solve(const Machine& machine, const toolpath::Toolpath& path,
                  const Transform3& wcsInTable, const SolveOptions& options) {
    if (machine.isLathe() && path.liveAxes == toolpath::LiveAxes::Polar) {
        return solveLivePolar(machine, path, std::max(1e-4, options.tolerance / 2.0));
    }
    if (machine.isLathe() && path.liveAxes == toolpath::LiveAxes::HeldC) {
        return solveLiveHeld(machine, path);
    }
    if (machine.isLathe() && path.liveAxes == toolpath::LiveAxes::Wrap) {
        return solveLiveWrap(machine, path);
    }
    if (machine.isLathe() && path.live) {
        return solveLiveLathe(machine, path);
    }
    if (!toolpath::isMultiAxis(path)) {
        return solve(machine, path, Mat3(wcsInTable.linear()));
    }
    SolveResult result;
    result.path = path;
    if (!machine.hasRotary()) {
        result.errors.push_back("the toolpath tilts the tool; this machine has no rotary axes");
        return result;
    }
    if (machine.isLathe()) {
        result.errors.push_back("multi-axis milling needs a milling machine");
        return result;
    }
    const bool hasCycles = std::any_of(path.moves.begin(), path.moves.end(),
                                       [](const Move& m) { return m.kind == MoveKind::Cycle; });
    if (hasCycles) {
        result.errors.push_back("canned cycles are not supported in multi-axis toolpaths");
        return result;
    }

    // Straight moves only: arcs are split, turning their axis from the previous move's.
    std::vector<Move> moves;
    {
        Vec3 pos = path.start;
        Vec3 axis = toolpath::axisBefore(path, 0);
        for (const Move& m : path.moves) {
            if (m.isArc()) {
                const std::vector<Vec3> pts =
                    toolpath::sampleArc(pos, m, std::max(1e-4, options.tolerance / 2.0));
                for (std::size_t i = 1; i < pts.size(); ++i) {
                    Move lm = m;
                    lm.kind = MoveKind::Linear;
                    lm.end = pts[i];
                    lm.axis = toolpath::slerp(
                        axis, m.axis, static_cast<double>(i) / static_cast<double>(pts.size() - 1));
                    moves.push_back(lm);
                }
            } else {
                moves.push_back(m);
            }
            if (m.kind != MoveKind::Dwell) {
                pos = m.end;
                axis = m.axis;
            }
        }
    }

    const Mat3 R = wcsInTable.linear();
    std::vector<Vec3> axes;
    axes.reserve(moves.size());
    for (const Move& m : moves) {
        axes.push_back((R * m.axis).normalized());
    }
    const RotaryPlan plan = planRotary(machine, axes, clearRapids(path.start, moves));
    if (!plan.errors.empty()) {
        result.errors = plan.errors;
        return result;
    }

    RotaryMotion motion;
    motion.axes = machine.rotaryAxes();
    motion.tcp = options.tcp;
    motion.start = plan.start;
    toolpath::Toolpath& out = result.path;
    out.moves.clear();
    out.cycles.clear();

    // Reference point of machine positions: the table centre when the table turns, else the
    // WCS origin.
    const Vec3 ref = machine.hasTableRotary() ? Vec3::Zero() : Vec3(wcsInTable.translation());
    const double L = options.toolLength;
    auto position = [&](const Vec3& pWcs, const IndexAngles& ang) -> Vec3 {
        if (options.tcp) {
            return R * pWcs;
        }
        return machinePosition(machine, ang, wcsInTable * pWcs - ref, L);
    };
    out.start = position(path.start, plan.start);

    auto emit = [&](const Move& tmpl, const Vec3& pWcs, const Vec3& uWcs, const IndexAngles& ang,
                    double time) {
        Move m = tmpl;
        const Vec3 from = out.moves.empty() ? out.start : out.moves.back().end;
        const IndexAngles& before = motion.angles.empty() ? motion.start : motion.angles.back();
        m.end = position(pWcs, ang);
        m.axis = (R * uWcs).normalized();
        if (!options.tcp && m.kind == MoveKind::Linear && time > 0.0 &&
            jointStep(before, ang) > 1e-12) {
            // Posted with inverse-time feeds: keep the equivalent feed along the programmed
            // positions (what a back-plot of the program sees).
            m.feedRate = (m.end - from).norm() / time;
        }
        out.moves.push_back(m);
        motion.angles.push_back(ang);
        motion.times.push_back(time);
    };

    // Splits a move until the machine's linear interpolation of its axes keeps the tip within
    // the tolerance (no tool centre point control).
    std::function<void(const Move&, const Vec3&, const Vec3&, const IndexAngles&, const Vec3&,
                       const Vec3&, const IndexAngles&, double, int)>
        segment = [&](const Move& tmpl, const Vec3& p0, const Vec3& u0, const IndexAngles& a0,
                      const Vec3& p1, const Vec3& u1, const IndexAngles& a1, double time,
                      int depth) {
            if (!options.tcp && depth < 12 && tmpl.kind == MoveKind::Linear &&
                jointStep(a0, a1) > 1e-12) {
                const IndexAngles am{(a0.a + a1.a) / 2.0, (a0.b + a1.b) / 2.0, (a0.c + a1.c) / 2.0};
                const Vec3 xm = 0.5 * (position(p0, a0) + position(p1, a1));
                const Vec3 tip = tipFromMachine(machine, am, xm, L) + ref;
                const Vec3 pm = 0.5 * (p0 + p1);
                if ((tip - wcsInTable * pm).norm() > options.tolerance) {
                    const Vec3 um = toolpath::slerp(u0, u1, 0.5);
                    const auto sols = axisSolutions(machine, R * um, &a0);
                    if (!sols.empty()) {
                        const IndexAngles mid =
                            *std::min_element(sols.begin(), sols.end(),
                                              [&](const IndexAngles& x, const IndexAngles& y) {
                                                  return jointStep(a0, x) + jointStep(x, a1) <
                                                         jointStep(a0, y) + jointStep(y, a1);
                                              });
                        segment(tmpl, p0, u0, a0, pm, um, mid, time / 2.0, depth + 1);
                        segment(tmpl, pm, um, mid, p1, u1, a1, time / 2.0, depth + 1);
                        return;
                    }
                }
            }
            emit(tmpl, p1, u1, a1, time);
        };

    Vec3 p = path.start;
    Vec3 u = moves.empty() ? Vec3::UnitZ() : moves.front().axis;
    IndexAngles a = plan.start;
    int swings = 0;
    std::size_t firstSwing = 0;
    for (std::size_t k = 0; k < moves.size(); ++k) {
        const Move& m = moves[k];
        const IndexAngles& next = plan.angles[k];
        if (m.kind == MoveKind::Dwell) {
            emit(m, p, u, a, 0.0);
            continue;
        }
        const double length = (m.end - p).norm();
        const double time =
            m.kind != MoveKind::Rapid && m.feedRate > 0.0 ? length / m.feedRate : 0.0;
        if (m.kind != MoveKind::Rapid && jointStep(a, next) > options.maxRotaryStep) {
            if (swings++ == 0) {
                firstSwing = k;
            }
        }
        segment(m, p, u, a, m.end, m.axis, next, time, 0);
        p = m.end;
        u = m.axis;
        a = next;
    }
    if (swings > 0) {
        result.warnings.push_back(fmt::format(
            "the rotary axes swing by more than {:.0f} degrees within {} feed move(s), first at "
            "move {} (near a singular tool axis); check the motion",
            degrees(options.maxRotaryStep), swings, firstSwing + 1));
    }
    result.rotary = std::move(motion);
    detail::applyLimits(machine, result);
    return result;
}

RotaryTrack rotaryTrack(const Machine& machine, const toolpath::Toolpath& path,
                        const Transform3& wcsInTable, const IndexAngles* index) {
    RotaryTrack track;
    if (index) {
        track.start = *index;
        track.angles.assign(path.moves.size(), *index);
        return track;
    }
    const Mat3 r = wcsInTable.linear();
    if (toolpath::isMultiAxis(path) && machine.hasRotary()) {
        std::vector<Vec3> axes;
        axes.reserve(path.moves.size());
        for (const auto& m : path.moves) {
            axes.push_back((r * m.axis).normalized());
        }
        RotaryPlan plan = planRotary(machine, axes, clearRapids(path.start, path.moves));
        track.start = plan.start;
        track.angles = std::move(plan.angles);
        track.errors = std::move(plan.errors);
        track.angles.resize(path.moves.size(), track.start);
        return track;
    }
    if (machine.canIndex()) {
        if (const auto setupIndex = indexAngles(machine, r.col(2))) {
            track.start = *setupIndex;
        }
    }
    track.angles.assign(path.moves.size(), track.start);
    return track;
}

Transform3 mountedWcs(const Machine& machine, const toolpath::Toolpath& path,
                      const Transform3& wcsInTable) {
    if (machine.canIndex() || (toolpath::isMultiAxis(path) && machine.hasRotary())) {
        return wcsInTable;
    }
    Transform3 square = Transform3::Identity();
    square.translation() = wcsInTable.translation();
    return square;
}

} // namespace ocam::machine
