// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A lathe on its kinematic tree (DL-0067). The tree is the machine model's (Model.hpp): Z and X
// (and Y) carry the turret, T turns it, C turns the main spindle, W slides the tailstock, Z2 and C2
// the sub-spindle. A tool's number is its station, station 1 at T 0. Stations are evenly spaced,
// and T = stationAngle(k) brings station k to the working position, facing the spindle.
//
// Tools stay on the turret. What differs from a mill is that where the axes stand depends on which
// tool is cutting: the linear axes put that tool's programmed point on the program's, and every
// other tool goes with it.

#pragma once

#include "ocam/machine/Model.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ocam::machine {

/// The part names of a layout's solids that the simulation treats on their own.
inline constexpr std::string_view kLatheChuckPart = "Chuck";

/// A solid of a lathe's layout: a box or a cylinder of the layout, in the model frame at machine
/// zero, on the axis that carries it.
struct LatheSolid {
    Component shape; ///< `attach` is not used.
    std::string carrier;
    bool shown = true; ///< An enclosure's door and roof start hidden, to see in.
    std::uint32_t colour = 0xB4B8BE;
    bool enclosure = false; ///< A wall of the layout's enclosure (ModelPart::enclosure).
};

/// The solids a machine's layout gives (none without one).
std::vector<LatheSolid> latheSolids(const Machine& machine);

/// The turret's position that has `station` (1 = T 0) working [rad], evenly spaced over the
/// machine's stations.
double stationAngle(const Machine& machine, int station);

/// Where the turret goes from `from` to work `station`: the shortest way round, a tie going
/// forward.
double indexTo(const Machine& machine, double from, int station);

/// The motion of the index line by angle `t` (model frame at machine zero), about the turret's
/// centre.
Transform3 turretTurn(const LatheTurret& turret, double t);

/// How a tool sits in its station. A tool across the spindle has its shank along X on the disc's
/// face, its tip at radialReach from the index line; a tool along the spindle has its centre line
/// at axialPitch and its tip at axialReach beyond the face, on the side the work is. `centreLine`
/// is how far the tool's own centre line lies from its tip (X for one along the spindle, Z for one
/// across).
struct StationMount {
    bool across = false;
    double centreLine = 0.0;
    bool towardsPlusZ = false; ///< The work is on +Z of the turret: a sub-spindle's.
};

/// The tip of a tool in the working position, relative to the turret's centre, in the layout's
/// numbers: (from the index line towards the spindle, along Z).
Vec2 defaultTipFromCentre(const LatheLayout& layout, const StationMount& mount);

/// The point of the model frame (machine zero) a tip `fromCentre` away from the turret's centre is
/// at, in the working position: `blockAcross` sideways from the centre, as the station's seat is.
Vec3 tipPoint(const LatheLayout& layout, const Vec2& fromCentre);

/// A move of the machine between two operations, in the order it runs. Tool changes are in here:
/// what the toolpaths do not say.
struct LatheStep {
    enum class Kind {
        Rapid,     ///< The linear axes, to or from an operation.
        Index,     ///< The turret turns.
        Tailstock, ///< W goes to where the setup has the centre.
        Transfer,  ///< Z2 goes to take the part, and back to where it cuts.
    };
    Kind kind = Kind::Rapid;
    AxisPosition to;
    double duration = 0.0; ///< [s], for playback.
    /// The part stays where the step ends: the sub-spindle coming to take it from the main one.
    bool workStays = false;
};

std::string_view toString(LatheStep::Kind kind) noexcept;

/// How an operation runs on a lathe with a layout.
struct LatheMotion {
    int station = 0;
    Vec3 tip = Vec3::Zero(); ///< The tool's programmed point, working, at machine zero.
    /// T, W and Z2 while it cuts; the linear axes and C depend on the tool's position.
    AxisPosition held;
    AxisPosition start;           ///< Where the machine stands before `steps`.
    std::vector<LatheStep> steps; ///< Up to the toolpath's first position, tool change included.

    bool operator==(const LatheMotion&) const = default;
};

inline bool operator==(const LatheStep& a, const LatheStep& b) {
    return a.kind == b.kind && a.to == b.to && a.duration == b.duration &&
           a.workStays == b.workStays;
}

/// How a lathe gets from where the last toolpath ended to the next one's start (both program
/// positions, Z along the spindle): along Z, then in on X. The exception is a start further out and
/// deeper than the tool stands, as after a hole near the axis with a cross hole next: along Z first
/// would go through the part, so it goes out on X first. The post and the simulation both go by
/// this.
bool approachOutFirst(const Vec3& from, const Vec3& to);

/// The axes `fraction` (0 to 1) through a step from `from`.
AxisPosition lerpAxes(const AxisPosition& from, const AxisPosition& to, double fraction);

/// The angle of the main spindle's C for a toolpath's tip (WCS) [rad]: 0 when the spindle turns the
/// part, else as the control turns it so the tip stands in the X-Z plane (or at Y of a held C).
double latheSpindleAngle(const toolpath::Toolpath& path, const Vec3& tip);

/// The axes for a tip of the toolpath (WCS), the work carried as `mount` has it. T, W and Z2 are
/// `motion.held`. Nullopt when the model does not move the tool relative to the work.
std::optional<AxisPosition> latheAxes(const MachineModel& model, const LatheMotion& motion,
                                      const WorkMount& mount, const toolpath::Toolpath& path,
                                      const Vec3& tipWcs);

} // namespace ocam::machine
