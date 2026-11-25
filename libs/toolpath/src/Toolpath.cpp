// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/toolpath/Toolpath.hpp"

#include <ocam/foundation/Error.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>

namespace ocam::toolpath {
namespace {

constexpr std::array<std::pair<FeedClass, std::string_view>, 8> kFeedClasses{{
    {FeedClass::Rapid, "rapid"},
    {FeedClass::Cut, "cut"},
    {FeedClass::Plunge, "plunge"},
    {FeedClass::Ramp, "ramp"},
    {FeedClass::LeadIn, "lead_in"},
    {FeedClass::LeadOut, "lead_out"},
    {FeedClass::Link, "link"},
    {FeedClass::Probe, "probe"},
}};

constexpr std::array<std::pair<ProbeFeature, std::string_view>, 7> kProbeFeatures{{
    {ProbeFeature::Surface, "surface"},
    {ProbeFeature::Web, "web"},
    {ProbeFeature::Pocket, "pocket"},
    {ProbeFeature::Bore, "bore"},
    {ProbeFeature::Boss, "boss"},
    {ProbeFeature::OutsideCorner, "outside_corner"},
    {ProbeFeature::InsideCorner, "inside_corner"},
}};

constexpr std::array<std::pair<CycleType, std::string_view>, 8> kCycleTypes{{
    {CycleType::Drill, "drill"},
    {CycleType::Dwell, "dwell"},
    {CycleType::Peck, "peck"},
    {CycleType::ChipBreak, "chip_break"},
    {CycleType::Tap, "tap"},
    {CycleType::Bore, "bore"},
    {CycleType::TapLeft, "tap_left"},
    {CycleType::BackBore, "back_bore"},
}};

constexpr double kTwoPi = 2.0 * kPi;
constexpr double kPeckClearance = 0.5; // re-entry gap above the previous peck depth [mm]

double round6(double v) {
    const double r = std::round(v * 1e6) / 1e6;
    return r == 0.0 ? 0.0 : r; // no negative zero
}

Json vec3(const Vec3& v) {
    return Json::array({round6(v.x()), round6(v.y()), round6(v.z())});
}

Vec3 vec3From(const Json& j) {
    return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()};
}

double moveLength(const Vec3& from, const Move& m) {
    if (m.isArc()) {
        const double r = (planeCoords(from, m.plane) - planeCoords(m.center, m.plane)).norm();
        const double arcLen = r * std::abs(arcSweep(from, m));
        const double dz = planeNormal(m.end, m.plane) - planeNormal(from, m.plane);
        return std::sqrt(arcLen * arcLen + dz * dz);
    }
    if (m.kind == MoveKind::Dwell) {
        return 0.0;
    }
    return (m.end - from).norm();
}

} // namespace

Vec2 planeCoords(const Vec3& p, ArcPlane plane) noexcept {
    switch (plane) {
    case ArcPlane::ZX:
        return {p.z(), p.x()};
    case ArcPlane::YZ:
        return {p.y(), p.z()};
    case ArcPlane::XY:
        break;
    }
    return {p.x(), p.y()};
}

double planeNormal(const Vec3& p, ArcPlane plane) noexcept {
    switch (plane) {
    case ArcPlane::ZX:
        return p.y();
    case ArcPlane::YZ:
        return p.x();
    case ArcPlane::XY:
        break;
    }
    return p.z();
}

Vec3 fromPlaneCoords(const Vec2& uv, double normal, ArcPlane plane) noexcept {
    switch (plane) {
    case ArcPlane::ZX:
        return {uv.y(), normal, uv.x()};
    case ArcPlane::YZ:
        return {normal, uv.x(), uv.y()};
    case ArcPlane::XY:
        break;
    }
    return {uv.x(), uv.y(), normal};
}

std::string_view toString(MoveKind kind) noexcept {
    switch (kind) {
    case MoveKind::Rapid:
        return "rapid";
    case MoveKind::Linear:
        return "linear";
    case MoveKind::ArcCW:
        return "arc_cw";
    case MoveKind::ArcCCW:
        return "arc_ccw";
    case MoveKind::Dwell:
        return "dwell";
    case MoveKind::Cycle:
        return "cycle";
    case MoveKind::Probe:
        return "probe";
    }
    return "unknown";
}

std::string_view toString(ProbeFeature feature) noexcept {
    for (const auto& [f, name] : kProbeFeatures) {
        if (f == feature) {
            return name;
        }
    }
    return "surface";
}

std::optional<ProbeFeature> parseProbeFeature(std::string_view text) noexcept {
    for (const auto& [f, name] : kProbeFeatures) {
        if (name == text) {
            return f;
        }
    }
    return std::nullopt;
}

namespace {

constexpr std::array<std::pair<MachineFunction, std::string_view>, 5> kFunctions{{
    {MachineFunction::None, "none"},
    {MachineFunction::ChuckOpen, "chuck_open"},
    {MachineFunction::ChuckClose, "chuck_close"},
    {MachineFunction::BarFeederOn, "bar_feeder_on"},
    {MachineFunction::BarFeederOff, "bar_feeder_off"},
}};

} // namespace

std::string_view toString(MachineFunction function) noexcept {
    for (const auto& [f, name] : kFunctions) {
        if (f == function) {
            return name;
        }
    }
    return "none";
}

std::optional<MachineFunction> parseMachineFunction(std::string_view text) noexcept {
    for (const auto& [f, name] : kFunctions) {
        if (name == text) {
            return f;
        }
    }
    return std::nullopt;
}

std::string_view toString(FeedClass feedClass) noexcept {
    for (const auto& [c, name] : kFeedClasses) {
        if (c == feedClass) {
            return name;
        }
    }
    return "cut";
}

std::optional<FeedClass> parseFeedClass(std::string_view text) noexcept {
    for (const auto& [c, name] : kFeedClasses) {
        if (name == text) {
            return c;
        }
    }
    return std::nullopt;
}

std::string_view toString(CycleType type) noexcept {
    for (const auto& [c, name] : kCycleTypes) {
        if (c == type) {
            return name;
        }
    }
    return "drill";
}

std::optional<CycleType> parseCycleType(std::string_view text) noexcept {
    for (const auto& [c, name] : kCycleTypes) {
        if (name == text) {
            return c;
        }
    }
    return std::nullopt;
}

double arcSweep(const Vec3& start, const Move& arc) {
    const Vec2 c = planeCoords(arc.center, arc.plane);
    const Vec2 s = planeCoords(start, arc.plane);
    const Vec2 e = planeCoords(arc.end, arc.plane);
    const double a0 = std::atan2(s.y() - c.y(), s.x() - c.x());
    const double a1 = std::atan2(e.y() - c.y(), e.x() - c.x());
    double sweep = std::fmod(a1 - a0, kTwoPi);
    if (arc.kind == MoveKind::ArcCCW) {
        if (sweep <= 1e-12) {
            sweep += kTwoPi;
        }
    } else if (sweep >= -1e-12) {
        sweep -= kTwoPi;
    }
    return sweep;
}

std::vector<Vec3> sampleArc(const Vec3& start, const Move& arc, double chordTolerance) {
    const Vec2 c = planeCoords(arc.center, arc.plane);
    const Vec2 s = planeCoords(start, arc.plane);
    const double n0 = planeNormal(start, arc.plane);
    const double n1 = planeNormal(arc.end, arc.plane);
    const double r = (s - c).norm();
    const double sweep = arcSweep(start, arc);
    const double a0 = std::atan2(s.y() - c.y(), s.x() - c.x());
    const double ratio =
        std::clamp(1.0 - std::max(chordTolerance, 1e-9) / std::max(r, 1e-12), -1.0, 1.0);
    const double maxStep = std::max(2.0 * std::acos(ratio), 1e-4);
    const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(sweep) / maxStep)));
    std::vector<Vec3> pts;
    pts.reserve(static_cast<std::size_t>(steps) + 1);
    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / steps;
        if (i == steps) {
            pts.push_back(arc.end);
        } else {
            const double a = a0 + sweep * t;
            pts.push_back(fromPlaneCoords(c + r * Vec2(std::cos(a), std::sin(a)),
                                          n0 + (n1 - n0) * t, arc.plane));
        }
    }
    return pts;
}

std::vector<double> moveDurations(const Toolpath& path, const MachineLimits& limits) {
    std::vector<double> out;
    out.reserve(path.moves.size());
    Vec3 pos = path.start;
    for (const Move& m : path.moves) {
        double time = 0.0;
        if (m.kind == MoveKind::Dwell) {
            time = m.dwell;
        } else if (m.kind == MoveKind::Cycle) {
            Toolpath mini;
            mini.start = pos;
            mini.cycles.push_back(path.cycles.at(static_cast<std::size_t>(m.cycle)));
            Move cm = m;
            cm.cycle = 0;
            mini.moves.push_back(cm);
            time = computeStats(mini, limits).totalTime;
            pos = m.end;
        } else {
            const double len = moveLength(pos, m);
            const bool rapid = m.kind == MoveKind::Rapid || m.feedRate <= 0.0;
            time = len / (rapid ? limits.rapidRate : m.feedRate);
            pos = m.end;
        }
        out.push_back(time);
    }
    return out;
}

std::optional<Move> partialMove(const Vec3& start, const Vec3& startAxis, const Move& m, double t) {
    if (t >= 1.0) {
        return m;
    }
    if (t <= 0.0 || m.kind == MoveKind::Cycle || m.kind == MoveKind::Dwell) {
        return std::nullopt;
    }
    Move part = m;
    if (m.isArc()) {
        const Vec2 c = planeCoords(m.center, m.plane);
        const Vec2 s = planeCoords(start, m.plane);
        const double r = (s - c).norm();
        const double a = std::atan2(s.y() - c.y(), s.x() - c.x()) + arcSweep(start, m) * t;
        const double n0 = planeNormal(start, m.plane);
        part.end = fromPlaneCoords(c + r * Vec2(std::cos(a), std::sin(a)),
                                   n0 + (planeNormal(m.end, m.plane) - n0) * t, m.plane);
    } else {
        part.end = start + (m.end - start) * t;
    }
    part.axis = slerp(startAxis, m.axis, t);
    return part;
}

bool isMultiAxis(const Toolpath& path) noexcept {
    return std::any_of(path.moves.begin(), path.moves.end(), [](const Move& m) {
        return (m.axis - Vec3::UnitZ()).squaredNorm() > 1e-18;
    });
}

Vec3 axisBefore(const Toolpath& path, std::size_t moveIndex) {
    if (path.moves.empty()) {
        return Vec3::UnitZ();
    }
    return moveIndex == 0 ? path.moves.front().axis
                          : path.moves[std::min(moveIndex, path.moves.size()) - 1].axis;
}

Vec3 slerp(const Vec3& a, const Vec3& b, double t) {
    const double c = std::clamp(a.dot(b), -1.0, 1.0);
    const double angle = std::acos(c);
    if (angle < 1e-9) {
        return (a + t * (b - a)).normalized();
    }
    if (kPi - angle < 1e-9) {
        // Opposite directions: rotate about any perpendicular axis.
        const Vec3 axis = a.unitOrthogonal();
        return Eigen::AngleAxisd(t * angle, axis) * a;
    }
    const double s = std::sin(angle);
    return (std::sin((1.0 - t) * angle) / s * a + std::sin(t * angle) / s * b).normalized();
}

Toolpath expandCycles(const Toolpath& path, std::span<const CycleType> keep) {
    Toolpath out = path;
    out.moves.clear();
    out.cycles.clear();
    std::map<int, int> kept; // cycle index in `path` -> in `out`
    Vec3 pos = path.start;
    Vec3 axis = Vec3::UnitZ();
    auto emit = [&](MoveKind kind, FeedClass fc, const Vec3& p, double rate,
                    std::optional<double> orient = std::nullopt) {
        if ((p - pos).norm() < 1e-12) {
            return;
        }
        Move m;
        m.kind = kind;
        m.feedClass = fc;
        m.end = p;
        m.axis = axis;
        m.feedRate = rate;
        m.orient = orient;
        out.moves.push_back(m);
        pos = p;
    };
    for (const Move& m : path.moves) {
        if (m.kind != MoveKind::Cycle) {
            out.moves.push_back(m);
            if (m.kind != MoveKind::Dwell) {
                pos = m.end;
            }
            continue;
        }
        const DrillCycle& c = path.cycles.at(static_cast<std::size_t>(m.cycle));
        const double x = m.end.x();
        const double y = m.end.y();
        const double r = c.retractZ;
        axis = m.axis;
        if (std::ranges::find(keep, c.type) != keep.end()) {
            auto [it, added] = kept.try_emplace(m.cycle, static_cast<int>(out.cycles.size()));
            if (added) {
                out.cycles.push_back(c);
            }
            Move hole = m;
            hole.cycle = it->second;
            out.moves.push_back(hole);
            pos = Vec3(x, y, r); // the control returns to the R plane (G99)
            continue;
        }
        auto at = [&](double z) {
            return Vec3(x, y, z);
        };

        emit(MoveKind::Rapid, FeedClass::Rapid, at(std::max(pos.z(), r)), 0.0);
        emit(MoveKind::Rapid, FeedClass::Rapid, at(r), 0.0);
        switch (c.type) {
        case CycleType::Drill:
            emit(MoveKind::Linear, FeedClass::Plunge, at(c.bottomZ), c.feedRate);
            emit(MoveKind::Rapid, FeedClass::Rapid, at(r), 0.0);
            break;
        case CycleType::Dwell: {
            emit(MoveKind::Linear, FeedClass::Plunge, at(c.bottomZ), c.feedRate);
            Move d;
            d.kind = MoveKind::Dwell;
            d.feedClass = FeedClass::Plunge;
            d.end = pos;
            d.dwell = c.dwell;
            out.moves.push_back(d);
            emit(MoveKind::Rapid, FeedClass::Rapid, at(r), 0.0);
            break;
        }
        case CycleType::Peck:
        case CycleType::ChipBreak: {
            // Like the control (G83/G73), pecks are measured from the R plane.
            const double peck = c.peck > 0.0 ? c.peck : (r - c.bottomZ);
            double reached = r;
            int guard = 0;
            while (reached > c.bottomZ + 1e-9 && guard++ < 100000) {
                const double next = std::max(c.bottomZ, reached - peck);
                if (reached < r) {
                    if (c.type == CycleType::Peck) {
                        emit(MoveKind::Rapid, FeedClass::Rapid,
                             at(std::min(r, reached + kPeckClearance)), 0.0);
                    } else {
                        emit(MoveKind::Rapid, FeedClass::Rapid, at(reached + c.chipBreakRetract),
                             0.0);
                    }
                }
                emit(MoveKind::Linear, FeedClass::Plunge, at(next), c.feedRate);
                reached = next;
                if (c.type == CycleType::Peck && reached > c.bottomZ + 1e-9) {
                    emit(MoveKind::Rapid, FeedClass::Rapid, at(r), 0.0);
                }
            }
            emit(MoveKind::Rapid, FeedClass::Rapid, at(r), 0.0);
            break;
        }
        case CycleType::Tap:
        case CycleType::TapLeft:
        case CycleType::Bore:
            emit(MoveKind::Linear, FeedClass::Plunge, at(c.bottomZ), c.feedRate);
            emit(MoveKind::Linear, FeedClass::Plunge, at(r), c.feedRate);
            // A live tool's tap stays one: the program taps it on the control's own cycle.
            if (path.live && isTap(c.type)) {
                out.moves[out.moves.size() - 2].pitch = c.pitch;
                out.moves.back().pitch = c.pitch;
            }
            break;
        case CycleType::BackBore: {
            // The spindle stands oriented while the tool is off the axis (DL-0083).
            const Vec3 off(c.shift.x(), c.shift.y(), 0.0);
            const auto still = [&](const Vec3& p) {
                emit(MoveKind::Rapid, FeedClass::Rapid, p, 0.0, c.orient);
            };
            still(at(r) + off);
            still(at(c.bottomZ) + off);
            still(at(c.bottomZ));
            emit(MoveKind::Linear, FeedClass::Cut, at(c.topZ), c.feedRate);
            if (c.dwell > 0.0) {
                Move d;
                d.kind = MoveKind::Dwell;
                d.feedClass = FeedClass::Cut;
                d.end = pos;
                d.dwell = c.dwell;
                out.moves.push_back(d);
            }
            still(at(c.topZ) + off);
            still(at(r) + off);
            still(at(r));
            break;
        }
        }
    }
    return out;
}

Toolpath transformed(const Toolpath& path, const Transform3& t, double chordTolerance) {
    const Mat3 R = t.linear();
    const bool keepsZ = (R.col(2) - Vec3::UnitZ()).norm() < 1e-12;
    const bool turns = (R - Mat3::Identity()).norm() > 1e-12;
    const Toolpath src = keepsZ ? path : expandCycles(path);
    Toolpath out = src;
    out.moves.clear();
    out.start = t * src.start;
    Vec3 pos = src.start;
    Vec3 axis = axisBefore(src, 0);
    for (const Move& m : src.moves) {
        const bool keepArc = !turns || (keepsZ && m.plane == ArcPlane::XY);
        if (m.isArc() && !keepArc) {
            const std::vector<Vec3> pts = sampleArc(pos, m, chordTolerance);
            for (std::size_t k = 1; k < pts.size(); ++k) {
                Move line = m;
                line.kind = MoveKind::Linear;
                line.plane = ArcPlane::XY;
                line.end = t * pts[k];
                line.center = Vec3::Zero();
                line.axis =
                    (R * slerp(axis, m.axis,
                               static_cast<double>(k) / static_cast<double>(pts.size() - 1)))
                        .normalized();
                out.moves.push_back(line);
            }
        } else {
            Move moved = m;
            moved.end = t * m.end;
            moved.center = t * m.center;
            moved.axis = (R * m.axis).normalized();
            out.moves.push_back(moved);
        }
        if (m.kind != MoveKind::Dwell) {
            pos = m.end;
            axis = m.axis;
        }
    }
    for (ProbeCycle& c : out.probes) {
        c.nominal = t * c.nominal;
        c.direction = R * c.direction;
    }
    if (keepsZ) {
        const double dz = t.translation().z();
        for (DrillCycle& c : out.cycles) {
            c.topZ += dz;
            c.bottomZ += dz;
            c.retractZ += dz;
            c.shift = R.topLeftCorner<2, 2>() * c.shift;
        }
    }
    return out;
}

Stats computeStats(const Toolpath& path, const MachineLimits& limits) {
    const Toolpath flat = expandCycles(path);
    Stats s;
    Vec3 pos = flat.start;
    s.bounds.extend(pos);
    for (const Move& m : flat.moves) {
        ++s.moveCount;
        if (m.kind == MoveKind::Dwell) {
            s.feedTime += m.dwell;
            continue;
        }
        const double len = moveLength(pos, m);
        if (m.kind == MoveKind::Rapid) {
            s.rapidLength += len;
            s.rapidTime += len / limits.rapidRate;
        } else {
            s.cutLength += len;
            s.feedTime += m.feedRate > 0.0 ? len / m.feedRate : len / limits.rapidRate;
        }
        if (m.isArc()) {
            for (const Vec3& p : sampleArc(pos, m, 0.01)) {
                s.bounds.extend(p);
            }
        } else {
            s.bounds.extend(m.end);
        }
        pos = m.end;
    }
    s.totalTime = s.feedTime + s.rapidTime;
    return s;
}

std::vector<PathRun> sampleRuns(const Toolpath& path, double chordTolerance) {
    const Toolpath flat = expandCycles(path);
    std::vector<PathRun> runs;
    Vec3 pos = flat.start;
    for (const Move& m : flat.moves) {
        if (m.kind == MoveKind::Dwell) {
            continue;
        }
        if (runs.empty() || runs.back().feedClass != m.feedClass) {
            runs.push_back({m.feedClass, {pos}});
        }
        auto& pts = runs.back().points;
        if (m.isArc()) {
            const auto arcPts = sampleArc(pos, m, chordTolerance);
            pts.insert(pts.end(), arcPts.begin() + 1, arcPts.end());
        } else {
            pts.push_back(m.end);
        }
        pos = m.end;
    }
    return runs;
}

Json toJson(const Toolpath& path) {
    Json moves = Json::array();
    for (const Move& m : path.moves) {
        Json j;
        switch (m.kind) {
        case MoveKind::Rapid:
            j = {{"k", "R"}, {"p", vec3(m.end)}};
            break;
        case MoveKind::Linear:
            j = {{"k", "L"},
                 {"p", vec3(m.end)},
                 {"f", std::string(toString(m.feedClass))},
                 {"F", round6(m.feedRate * 60.0)}};
            if (m.pitch > 0.0) {
                j["pitch"] = round6(m.pitch);
            }
            break;
        case MoveKind::ArcCW:
        case MoveKind::ArcCCW:
            j = {{"k", m.kind == MoveKind::ArcCW ? "CW" : "CCW"},
                 {"p", vec3(m.end)},
                 {"c", vec3(m.center)},
                 {"f", std::string(toString(m.feedClass))},
                 {"F", round6(m.feedRate * 60.0)}};
            if (m.plane != ArcPlane::XY) {
                j["pl"] = m.plane == ArcPlane::ZX ? "ZX" : "YZ";
            }
            break;
        case MoveKind::Dwell:
            j = {{"k", "D"},
                 {"p", vec3(m.end)},
                 {"f", std::string(toString(m.feedClass))},
                 {"t", round6(m.dwell)}};
            if (m.function != MachineFunction::None) {
                j["fn"] = std::string(toString(m.function));
            }
            break;
        case MoveKind::Cycle:
            j = {{"k", "CY"},
                 {"p", vec3(m.end)},
                 {"f", std::string(toString(m.feedClass))},
                 {"i", m.cycle},
                 {"F", round6(m.feedRate * 60.0)}};
            break;
        case MoveKind::Probe:
            j = {{"k", "P"},
                 {"p", vec3(m.end)},
                 {"f", std::string(toString(m.feedClass))},
                 {"i", m.probe},
                 {"F", round6(m.feedRate * 60.0)}};
            if (m.measure) {
                j["measure"] = true;
            }
            break;
        }
        if (m.orient) {
            j["o"] = round6(*m.orient);
        }
        if ((m.axis - Vec3::UnitZ()).norm() > 1e-12) {
            j["a"] = {std::round(m.axis.x() * 1e9) / 1e9, std::round(m.axis.y() * 1e9) / 1e9,
                      std::round(m.axis.z() * 1e9) / 1e9};
        }
        moves.push_back(std::move(j));
    }
    Json cycles = Json::array();
    for (const DrillCycle& c : path.cycles) {
        Json cj{{"type", std::string(toString(c.type))},
                {"top", round6(c.topZ)},
                {"bottom", round6(c.bottomZ)},
                {"retract", round6(c.retractZ)},
                {"peck", round6(c.peck)},
                {"chip_break_retract", round6(c.chipBreakRetract)},
                {"dwell", round6(c.dwell)},
                {"F", round6(c.feedRate * 60.0)},
                {"pitch", round6(c.pitch)}};
        if (c.type == CycleType::BackBore) {
            cj["shift"] = {round6(c.shift.x()), round6(c.shift.y())};
            cj["orient"] = round6(c.orient);
        }
        cycles.push_back(std::move(cj));
    }
    Json probes = Json::array();
    for (const ProbeCycle& c : path.probes) {
        probes.push_back({{"feature", std::string(toString(c.feature))},
                          {"nominal", vec3(c.nominal)},
                          {"direction", vec3(c.direction)},
                          {"size", round6(c.size)},
                          {"work_offset", c.workOffset},
                          {"axes", c.axes},
                          {"tolerance", round6(c.tolerance)},
                          {"approach", round6(c.approach)},
                          {"overtravel", round6(c.overtravel)},
                          {"fast_F", round6(c.fastFeed * 60.0)},
                          {"measure_F", round6(c.measureFeed * 60.0)},
                          {"stylus_diameter", round6(c.stylusDiameter)},
                          {"trigger_offset", round6(c.triggerOffset)}});
    }
    Json doc{{"operation", path.operationId.toString()},
             {"strategy", path.strategy},
             {"tool_number", path.toolNumber},
             {"tool_diameter", round6(path.toolDiameter)},
             {"spindle_rpm", round6(path.spindleSpeed * 60.0)},
             {"coolant", std::string(tooling::toString(path.coolant))},
             {"compensation", path.compensation == Compensation::Left    ? "left"
                              : path.compensation == Compensation::Right ? "right"
                                                                         : "none"},
             {"start", vec3(path.start)},
             {"moves", std::move(moves)},
             {"cycles", std::move(cycles)},
             {"warnings", path.warnings}};
    if (path.toolId) {
        doc["tool"] = path.toolId->toString();
    }
    if (!path.probes.empty()) {
        doc["probes"] = std::move(probes);
    }
    if (path.liveAxes != LiveAxes::None) {
        doc["live_axes"] = path.liveAxes == LiveAxes::Polar  ? "polar"
                           : path.liveAxes == LiveAxes::Wrap ? "wrap"
                                                             : "held_c";
        doc["live_c"] = round6(path.liveC);
    }
    if (path.live) {
        doc["live"] = true;
    }
    if (const auto& t = path.transfer) {
        doc["transfer"] = {{"approach_z2", round6(t->approachZ2)},
                           {"pick_z2", round6(t->pickZ2)},
                           {"work_z2", round6(t->workZ2)},
                           {"F", round6(t->feed * 60.0)},
                           {"sync", t->sync == SpindleTransfer::Sync::Angle     ? "angle"
                                    : t->sync == SpindleTransfer::Sync::Stopped ? "stopped"
                                                                                : "speed"},
                           {"angle_deg", round6(radToDeg(t->angle))},
                           {"rpm", round6(t->spindle * 60.0)},
                           {"part_off", t->partOff},
                           {"dwell", round6(t->dwell)}};
    }
    if (path.isTurning()) {
        doc["turning"] = {{"tip_quadrant", path.tipQuadrant},
                          {"surface_speed", round6(path.surfaceSpeed * 0.06)}, // m/min
                          {"max_rpm", round6(path.maxSpindle * 60.0)}};
    }
    return doc;
}

Toolpath toolpathFromJson(const Json& doc) {
    try {
        Toolpath p;
        if (const auto id = Uuid::parse(doc.value("operation", ""))) {
            p.operationId = *id;
        }
        p.strategy = doc.value("strategy", "");
        p.toolNumber = doc.value("tool_number", 0);
        p.toolDiameter = doc.value("tool_diameter", 0.0);
        p.spindleSpeed = doc.value("spindle_rpm", 0.0) / 60.0;
        p.coolant =
            tooling::parseCoolant(doc.value("coolant", "flood")).value_or(tooling::Coolant::Flood);
        p.live = doc.value("live", false);
        const std::string axes = doc.value("live_axes", "");
        p.liveAxes = axes == "polar"    ? LiveAxes::Polar
                     : axes == "wrap"   ? LiveAxes::Wrap
                     : axes == "held_c" ? LiveAxes::HeldC
                                        : LiveAxes::None;
        p.liveC = doc.value("live_c", 0.0);
        const std::string comp = doc.value("compensation", "none");
        p.compensation = comp == "left"    ? Compensation::Left
                         : comp == "right" ? Compensation::Right
                                           : Compensation::None;
        p.start = vec3From(doc.at("start"));
        for (const Json& j : doc.at("moves")) {
            Move m;
            const std::string k = j.at("k").get<std::string>();
            if (k == "R") {
                m.kind = MoveKind::Rapid;
                m.feedClass = FeedClass::Rapid;
            } else if (k == "L") {
                m.kind = MoveKind::Linear;
            } else if (k == "CW" || k == "CCW") {
                m.kind = k == "CW" ? MoveKind::ArcCW : MoveKind::ArcCCW;
                m.center = vec3From(j.at("c"));
                const std::string pl = j.value("pl", "XY");
                m.plane = pl == "ZX" ? ArcPlane::ZX : pl == "YZ" ? ArcPlane::YZ : ArcPlane::XY;
            } else if (k == "D") {
                m.kind = MoveKind::Dwell;
                m.dwell = j.at("t").get<double>();
                const std::string fn = j.value("fn", "none");
                const auto function = parseMachineFunction(fn);
                if (!function) {
                    throw Error(ErrorCode::Parse, "unknown machine function " + fn);
                }
                m.function = *function;
            } else if (k == "CY") {
                m.kind = MoveKind::Cycle;
                m.cycle = j.at("i").get<int>();
            } else if (k == "P") {
                m.kind = MoveKind::Probe;
                m.probe = j.at("i").get<int>();
                m.measure = j.value("measure", false);
            } else {
                throw Error(ErrorCode::Parse, "unknown move kind " + k);
            }
            if (j.contains("p")) {
                m.end = vec3From(j.at("p"));
            }
            if (j.contains("f")) {
                m.feedClass = parseFeedClass(j.at("f").get<std::string>()).value_or(FeedClass::Cut);
            }
            m.feedRate = j.value("F", 0.0) / 60.0;
            m.pitch = j.value("pitch", 0.0);
            if (const auto o = j.find("o"); o != j.end()) {
                m.orient = o->get<double>();
            }
            if (const auto a = j.find("a"); a != j.end()) {
                m.axis = vec3From(*a).normalized();
            }
            p.moves.push_back(m);
        }
        for (const Json& j : doc.at("cycles")) {
            DrillCycle c;
            c.type = parseCycleType(j.at("type").get<std::string>()).value_or(CycleType::Drill);
            c.topZ = j.at("top").get<double>();
            c.bottomZ = j.at("bottom").get<double>();
            c.retractZ = j.at("retract").get<double>();
            c.peck = j.value("peck", 0.0);
            c.chipBreakRetract = j.value("chip_break_retract", 0.5);
            c.dwell = j.value("dwell", 0.0);
            c.feedRate = j.value("F", 0.0) / 60.0;
            c.pitch = j.value("pitch", 0.0);
            if (const auto s = j.find("shift"); s != j.end()) {
                c.shift = Vec2(s->at(0).get<double>(), s->at(1).get<double>());
            }
            c.orient = j.value("orient", 0.0);
            p.cycles.push_back(c);
        }
        for (const Json& j : doc.value("probes", Json::array())) {
            ProbeCycle c;
            c.feature = parseProbeFeature(j.at("feature").get<std::string>())
                            .value_or(ProbeFeature::Surface);
            c.nominal = vec3From(j.at("nominal"));
            c.direction = vec3From(j.at("direction"));
            c.size = j.value("size", 0.0);
            c.workOffset = j.value("work_offset", 0);
            c.axes = j.value("axes", "");
            c.tolerance = j.value("tolerance", 0.0);
            c.approach = j.value("approach", 0.0);
            c.overtravel = j.value("overtravel", 0.0);
            c.fastFeed = j.value("fast_F", 0.0) / 60.0;
            c.measureFeed = j.value("measure_F", 0.0) / 60.0;
            c.stylusDiameter = j.value("stylus_diameter", 0.0);
            c.triggerOffset = j.value("trigger_offset", 0.0);
            p.probes.push_back(c);
        }
        for (const Json& w : doc.value("warnings", Json::array())) {
            p.warnings.push_back(w.get<std::string>());
        }
        if (const auto it = doc.find("tool"); it != doc.end()) {
            p.toolId = Uuid::parse(it->get<std::string>());
        }
        if (const auto it = doc.find("transfer"); it != doc.end()) {
            SpindleTransfer t;
            t.approachZ2 = it->at("approach_z2").get<double>();
            t.pickZ2 = it->at("pick_z2").get<double>();
            t.workZ2 = it->at("work_z2").get<double>();
            t.feed = it->at("F").get<double>() / 60.0;
            const std::string sync = it->value("sync", "speed");
            t.sync = sync == "angle"     ? SpindleTransfer::Sync::Angle
                     : sync == "stopped" ? SpindleTransfer::Sync::Stopped
                                         : SpindleTransfer::Sync::Speed;
            t.angle = degToRad(it->value("angle_deg", 0.0));
            t.spindle = it->value("rpm", 0.0) / 60.0;
            t.partOff = it->value("part_off", true);
            t.dwell = it->value("dwell", 0.0);
            p.transfer = t;
        }
        if (const auto it = doc.find("turning"); it != doc.end()) {
            p.tipQuadrant = it->value("tip_quadrant", 3);
            p.surfaceSpeed = it->value("surface_speed", 0.0) / 0.06;
            p.maxSpindle = it->value("max_rpm", 0.0) / 60.0;
        }
        return p;
    } catch (const Json::exception& e) {
        throw Error(ErrorCode::Parse, fmt::format("invalid toolpath JSON: {}", e.what()));
    }
}

std::optional<std::string> compareToolpaths(const Toolpath& expected, const Toolpath& actual,
                                            double tol) {
    auto near = [tol](double a, double b) {
        return std::abs(a - b) <= tol * std::max(1.0, std::abs(a));
    };
    auto nearV = [&](const Vec3& a, const Vec3& b) {
        return near(a.x(), b.x()) && near(a.y(), b.y()) && near(a.z(), b.z());
    };
    auto fmtV = [](const Vec3& v) {
        return fmt::format("({:.6f}, {:.6f}, {:.6f})", v.x(), v.y(), v.z());
    };

    if (expected.compensation != actual.compensation) {
        return std::string("compensation differs");
    }
    if (expected.transfer.has_value() != actual.transfer.has_value() ||
        (expected.transfer && (!near(expected.transfer->pickZ2, actual.transfer->pickZ2) ||
                               !near(expected.transfer->approachZ2, actual.transfer->approachZ2) ||
                               !near(expected.transfer->feed, actual.transfer->feed) ||
                               expected.transfer->sync != actual.transfer->sync ||
                               expected.transfer->partOff != actual.transfer->partOff))) {
        return std::string("transfer differs");
    }
    if (!nearV(expected.start, actual.start)) {
        return "start differs: " + fmtV(expected.start) + " vs " + fmtV(actual.start);
    }
    const std::size_t n = std::min(expected.moves.size(), actual.moves.size());
    for (std::size_t i = 0; i < n; ++i) {
        const Move& e = expected.moves[i];
        const Move& a = actual.moves[i];
        if (e.kind != a.kind || e.feedClass != a.feedClass) {
            return fmt::format("move {}: kind/class {} {} vs {} {}", i, toString(e.kind),
                               toString(e.feedClass), toString(a.kind), toString(a.feedClass));
        }
        if (!nearV(e.end, a.end)) {
            return fmt::format("move {}: end {} vs {}", i, fmtV(e.end), fmtV(a.end));
        }
        if (!nearV(e.axis, a.axis)) {
            return fmt::format("move {}: tool axis {} vs {}", i, fmtV(e.axis), fmtV(a.axis));
        }
        if (e.isArc() && e.plane != a.plane) {
            return fmt::format("move {}: arc plane differs", i);
        }
        if (e.isArc() && !nearV(e.center, a.center)) {
            return fmt::format("move {}: centre {} vs {}", i, fmtV(e.center), fmtV(a.center));
        }
        if (!near(e.feedRate, a.feedRate) || !near(e.dwell, a.dwell) || e.cycle != a.cycle) {
            return fmt::format("move {}: feed/dwell/cycle differ", i);
        }
        if (e.orient.has_value() != a.orient.has_value() ||
            (e.orient && !near(*e.orient, *a.orient))) {
            return fmt::format("move {}: spindle orientation differs", i);
        }
        if (e.probe != a.probe || e.measure != a.measure) {
            return fmt::format("move {}: probing cycle or measuring differ", i);
        }
        if (e.function != a.function) {
            return fmt::format("move {}: machine function {} vs {}", i, toString(e.function),
                               toString(a.function));
        }
    }
    if (expected.moves.size() != actual.moves.size()) {
        return fmt::format("move count {} vs {}", expected.moves.size(), actual.moves.size());
    }
    if (expected.cycles.size() != actual.cycles.size()) {
        return fmt::format("cycle count {} vs {}", expected.cycles.size(), actual.cycles.size());
    }
    for (std::size_t i = 0; i < expected.cycles.size(); ++i) {
        const DrillCycle& e = expected.cycles[i];
        const DrillCycle& a = actual.cycles[i];
        if (e.type != a.type || !near(e.topZ, a.topZ) || !near(e.bottomZ, a.bottomZ) ||
            !near(e.retractZ, a.retractZ) || !near(e.peck, a.peck) ||
            !near(e.feedRate, a.feedRate) || !near(e.dwell, a.dwell) || !near(e.pitch, a.pitch) ||
            !near(e.shift.x(), a.shift.x()) || !near(e.shift.y(), a.shift.y()) ||
            !near(e.orient, a.orient)) {
            return fmt::format("cycle {} differs", i);
        }
    }
    if (expected.probes.size() != actual.probes.size()) {
        return fmt::format("probing cycle count {} vs {}", expected.probes.size(),
                           actual.probes.size());
    }
    for (std::size_t i = 0; i < expected.probes.size(); ++i) {
        const ProbeCycle& e = expected.probes[i];
        const ProbeCycle& a = actual.probes[i];
        if (e.feature != a.feature || !nearV(e.nominal, a.nominal) ||
            !nearV(e.direction, a.direction) || !near(e.size, a.size) ||
            e.workOffset != a.workOffset || e.axes != a.axes || !near(e.tolerance, a.tolerance) ||
            !near(e.approach, a.approach) || !near(e.overtravel, a.overtravel) ||
            !near(e.fastFeed, a.fastFeed) || !near(e.measureFeed, a.measureFeed) ||
            !near(e.stylusDiameter, a.stylusDiameter) || !near(e.triggerOffset, a.triggerOffset)) {
            return fmt::format("probing cycle {} differs", i);
        }
    }
    return std::nullopt;
}

} // namespace ocam::toolpath
