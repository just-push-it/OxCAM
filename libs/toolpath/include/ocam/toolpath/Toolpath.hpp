// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Neutral cutter-location (CL) intermediate representation. Strategies produce it; machine
// kinematics and post processors consume it (phase 4); the simulator replays it (phase 5).
//
// Coordinates are the tool tip in the setup's work coordinate system [mm]; feed rates are mm/s;
// arcs lie in the XY plane of the WCS (G17), where they may climb or descend (helices), or in the
// ZX or YZ plane (G18, G19; finishing passes in vertical planes, never helical). Multi-axis
// toolpaths (phase 9) give each move's tool axis (a unit vector from the tip towards the spindle,
// in the WCS); they use straight moves only.

#pragma once

#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Math.hpp>
#include <ocam/foundation/Uuid.hpp>
#include <ocam/geom3d/Bounds.hpp>
#include <ocam/tooling/Tool.hpp>

#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace ocam::toolpath {

enum class MoveKind {
    Rapid,  ///< Positioning at rapid traverse (G0).
    Linear, ///< Straight feed move (G1).
    ArcCW,  ///< Clockwise arc seen from +Z (G2), optionally helical.
    ArcCCW, ///< Counter-clockwise arc (G3), optionally helical.
    Dwell,  ///< Pause at the current position for `dwell` seconds (G4).
    Cycle,  ///< Canned drilling cycle at `end` (x, y); ends at the cycle's retract plane.
    /// A spindle probe's straight move at `feedRate` (DL-0090), part of the probing cycle
    /// `probe`: a protected positioning move, which stops if the stylus touches anything, or
    /// (`measure`) a measuring move, which stops where it touches the surface it measures. `end`
    /// is where it stops nominally: for a measuring move, where the stylus meets the model's
    /// surface. Probe moves never cut: the simulation reports material they meet.
    Probe,
};

/// The plane an arc lies in. (u, v, normal) is right-handed: XY (x, y, z), ZX (z, x, y), YZ
/// (y, z, x); clockwise is seen from the normal's positive side, as in G-code (G17/G18/G19).
enum class ArcPlane { XY, ZX, YZ };

/// A point's coordinates in an arc plane (u, v), and along its normal.
Vec2 planeCoords(const Vec3& p, ArcPlane plane) noexcept;
double planeNormal(const Vec3& p, ArcPlane plane) noexcept;
/// The point with plane coordinates `uv` and normal coordinate `normal`.
Vec3 fromPlaneCoords(const Vec2& uv, double normal, ArcPlane plane) noexcept;

/// What a move is for: drives feed rates, display colours and simulation checks. Probe: a
/// spindle probe's move (MoveKind::Probe).
enum class FeedClass { Rapid, Cut, Plunge, Ramp, LeadIn, LeadOut, Link, Probe };

/// Something the machine does standing still, carried by a Dwell move (DL-0091): the machine does
/// it where the tool stands, then dwells the move's seconds (0: none). The codes are the post's.
enum class MachineFunction {
    None,
    ChuckOpen,    ///< The main spindle's chuck opens.
    ChuckClose,   ///< It closes.
    BarFeederOn,  ///< The bar feeder pushes the bar.
    BarFeederOff, ///< It stops pushing.
};

std::string_view toString(MoveKind kind) noexcept;
std::string_view toString(FeedClass feedClass) noexcept;
std::optional<FeedClass> parseFeedClass(std::string_view text) noexcept;
/// "chuck_open", "chuck_close", "bar_feeder_on", "bar_feeder_off"; "none".
std::string_view toString(MachineFunction function) noexcept;
std::optional<MachineFunction> parseMachineFunction(std::string_view text) noexcept;

struct Move {
    MoveKind kind = MoveKind::Linear;
    FeedClass feedClass = FeedClass::Cut;
    Vec3 end = Vec3::Zero();
    Vec3 center = Vec3::Zero();    ///< Arcs: centre (on the start's level along the normal).
    ArcPlane plane = ArcPlane::XY; ///< Arcs: the plane they lie in.
    Vec3 axis = Vec3::UnitZ();     ///< Tool axis at the end of the move (3-axis: always +Z).
    double feedRate = 0.0;         ///< mm/s; 0 for rapids.
    double dwell = 0.0;            ///< Dwell: seconds.
    int cycle = -1;                ///< Cycle: index into Toolpath::cycles.
    /// Linear moves synchronized with the spindle (thread cutting, G32): lead per revolution
    /// [mm]; 0 for ordinary moves.
    double pitch = 0.0;
    /// Set: the spindle stands still, oriented at this angle [rad], for the whole move, so the
    /// tool cuts nothing (a back-boring bar passing through its hole off centre, DL-0083).
    std::optional<double> orient;
    int probe = -1;       ///< Probe: index into Toolpath::probes.
    bool measure = false; ///< Probe: a measuring move, else a protected positioning move.
    /// Dwell: what the machine does before the dwell's seconds (DL-0091).
    MachineFunction function = MachineFunction::None;

    bool isArc() const noexcept { return kind == MoveKind::ArcCW || kind == MoveKind::ArcCCW; }
    bool operator==(const Move&) const = default;
};

enum class CycleType {
    Drill,     ///< Feed to depth, rapid out (G81).
    Dwell,     ///< Feed to depth, dwell, rapid out (G82).
    Peck,      ///< Peck with full retract (G83).
    ChipBreak, ///< Peck with short retract (G73).
    Tap,       ///< Synchronized tapping (G84).
    Bore,      ///< Feed in, feed out (G85).
    TapLeft,   ///< Synchronized tapping of a left-hand thread (G74).
    /// Back boring (G87 style, DL-0083): from the initial plane (retractZ) the spindle orients
    /// and the tool shifts off the hole's axis by `shift`, goes down through the hole to
    /// bottomZ, shifts back, starts its spindle and feeds up to topZ (the end of the cut, above
    /// bottomZ), dwells, orients, shifts and goes back up to retractZ, where it shifts back.
    BackBore,
};

constexpr bool isTap(CycleType type) noexcept {
    return type == CycleType::Tap || type == CycleType::TapLeft;
}

std::string_view toString(CycleType type) noexcept;
std::optional<CycleType> parseCycleType(std::string_view text) noexcept;

struct DrillCycle {
    CycleType type = CycleType::Drill;
    double topZ = 0.0;    ///< Hole top (WCS Z). BackBore: where the cut ends.
    double bottomZ = 0.0; ///< Final depth (WCS Z). BackBore: where the cut starts.
    /// R plane: rapid level between holes (WCS Z). BackBore: the initial plane above the hole
    /// that the cycle starts from and returns to.
    double retractZ = 0.0;
    double peck = 0.0; ///< Peck depth for Peck / ChipBreak.
    double chipBreakRetract = 0.5;
    double dwell = 0.0;    ///< Seconds at the bottom (Dwell cycle).
    double feedRate = 0.0; ///< mm/s
    double pitch = 0.0;    ///< Tap pitch [mm/rev].
    /// BackBore: the tool's offset from the hole's axis while the spindle stands oriented, in
    /// the program's X and Y [mm]; it points away from the cutting edge.
    Vec2 shift = Vec2::Zero();
    double orient = 0.0; ///< BackBore: the spindle's orientation angle [rad].

    bool operator==(const DrillCycle&) const = default;
};

/// What a probing cycle measures (DL-0090).
enum class ProbeFeature {
    Surface,       ///< One face, along X, Y or Z.
    Web,           ///< Two outside faces, across X or Y: width and centre.
    Pocket,        ///< Two inside faces, across X or Y: width and centre.
    Bore,          ///< A hole: diameter and centre.
    Boss,          ///< A cylinder from outside: diameter and centre.
    OutsideCorner, ///< Two outside faces at a right angle: the corner in X and Y.
    InsideCorner,  ///< Two inside faces at a right angle: the corner in X and Y.
};

std::string_view toString(ProbeFeature feature) noexcept;
std::optional<ProbeFeature> parseProbeFeature(std::string_view text) noexcept;

/// One probing cycle: what it measures, against which nominal values, and what the control does
/// with the result. Its moves (MoveKind::Probe) are in Toolpath::moves.
struct ProbeCycle {
    ProbeFeature feature = ProbeFeature::Surface;
    /// Surface: the point on the face the stylus touches. Web, pocket, bore, boss: the centre.
    /// Corners: the corner. Z of an XY feature: where the stylus ball's centre is while it
    /// measures (WCS) [mm].
    Vec3 nominal = Vec3::Zero();
    /// Surface: the direction the stylus moves to touch (+-X, +-Y or -Z). Web and pocket: the
    /// axis measured across (+X or +Y). Corners: the directions the stylus moves to touch each
    /// face, as (+-1, +-1, 0). Bore and boss: zero.
    Vec3 direction = Vec3::Zero();
    double size = 0.0; ///< Web and pocket: the width; bore and boss: the diameter [mm].
    /// The work offset the result sets (1 = G54, ...); 0 = none: a check against the nominal
    /// values within `tolerance`.
    int workOffset = 0;
    /// What it sets in that work offset: "X", "Y", "Z" or "XY".
    std::string axes;
    double tolerance = 0.0;  ///< Check: how far the measured values may be off [mm].
    double approach = 0.0;   ///< From the stylus to the surface where each measuring move starts.
    double overtravel = 0.0; ///< How far past the surface a measuring move goes before it gives up.
    double fastFeed = 0.0;   ///< Protected positioning moves [mm/s].
    double measureFeed = 0.0;    ///< Measuring moves [mm/s].
    double stylusDiameter = 0.0; ///< The probe's ball [mm].
    double triggerOffset = 0.0;  ///< The probe's effective trigger offset [mm].

    bool isCheck() const noexcept { return workOffset == 0; }
    bool operator==(const ProbeCycle&) const = default;
};

/// Cutter compensation requested from the control (G41/G42). With None the path is the tool
/// centre; otherwise it is the nominal contour and the control offsets it by the tool radius.
enum class Compensation { None, Left, Right };

/// How a live tool on a lathe goes around what it cuts (DL-0066). None: it goes to holes, C
/// turning each to the tool.
enum class LiveAxes {
    None,
    Polar, ///< C and X, the control turning XY into them (polar interpolation).
    HeldC, ///< X, Y and Z, with C held still.
    Wrap,  ///< X, Z and C, the control mapping feed moves onto a cylinder (DL-0074).
};

/// The part handed from the main spindle to the sub-spindle (ADR-0007): Z2's moves and what the
/// spindles do, with no tool and no tool moves. Z2 in the machine's own (the lathe layout's,
/// 0 where the sub-spindle's nose is at its `nose`).
struct SpindleTransfer {
    enum class Sync {
        Speed,   ///< Both turn at `spindle`, the sub-spindle following the main.
        Angle,   ///< The same, the sub-spindle at `angle` to the main.
        Stopped, ///< Both stand still.
    };
    double approachZ2 = 0.0; ///< Where Z2's rapid stops, its jaws clear of the part.
    double pickZ2 = 0.0;     ///< Z2 with the part in its jaws.
    double workZ2 = 0.0;     ///< Where Z2 takes the part to, to cut it.
    double feed = 0.0;       ///< Z2's feed from approachZ2 to pickZ2 [mm/s].
    Sync sync = Sync::Speed;
    double angle = 0.0;   ///< Sync::Angle [rad].
    double spindle = 0.0; ///< Synchronised: the spindles' speed [rev/s].
    bool partOff = true;  ///< A part-off follows under grip; else the main chuck opens.
    double dwell = 0.0;   ///< After the sub-spindle's chuck closes [s].

    bool operator==(const SpindleTransfer&) const = default;
};

struct Toolpath {
    Uuid operationId;
    std::string strategy;
    std::optional<Uuid> toolId;
    int toolNumber = 0;
    double toolDiameter = 0.0;
    double spindleSpeed = 0.0; ///< rev/s (positive = clockwise, M03).
    bool spindleReverse =
        false; ///< Counter-clockwise (M04): a left-hand tap. The speed stays as it is.
    tooling::Coolant coolant = tooling::Coolant::Flood;
    Compensation compensation = Compensation::None;
    Vec3 start = Vec3::Zero(); ///< Tool position before the first move (at clearance height).
    std::vector<Move> moves;
    std::vector<DrillCycle> cycles;
    std::vector<ProbeCycle> probes; ///< Probing cycles (MoveKind::Probe moves index them).
    std::vector<std::string> warnings;

    // Turning (phase 8). Coordinates are then (x = radius, y = 0, z = axial position) in the
    // WCS, whose Z axis is the spindle axis; see docs/design-log/0020.
    /// Constant surface speed [mm/s] (0 = constant spindle speed); spindleSpeed is then the
    /// nominal speed that feed rates were computed with.
    double surfaceSpeed = 0.0;
    double maxSpindle = 0.0; ///< Spindle limit under constant surface speed [rev/s].
    /// Turning insert tip quadrant: 0 = milling, 3 = outside (OD) tools, 2 = inside (ID) tools.
    int tipQuadrant = 0;
    /// A lathe's driven tool (DL-0063): `spindleSpeed` is the tool's, and the part stands still
    /// at the C angle its holes need. Moves are in the WCS, the tool axis on each.
    bool live = false;
    LiveAxes liveAxes = LiveAxes::None;
    double liveC = 0.0;      ///< HeldC, on the part's face: where C is held [rad].
    bool liveAcross = false; ///< HeldC, once solved: the tool is across the part (Y and Z move).
    /// A transfer to the sub-spindle (ADR-0007): no moves, no tool.
    std::optional<SpindleTransfer> transfer;

    bool isEmpty() const noexcept { return moves.empty(); }
    bool isTurning() const noexcept { return tipQuadrant != 0; }
    /// A spindle probe's toolpath: it has probing cycles.
    bool isProbing() const noexcept { return !probes.empty(); }
    bool operator==(const Toolpath&) const = default;
};

struct MachineLimits {
    double rapidRate = 10000.0 / 60.0; ///< Rapid traverse [mm/s].
};

struct Stats {
    double cutLength = 0.0;   ///< All feed moves [mm].
    double rapidLength = 0.0; ///< [mm]
    double feedTime = 0.0;    ///< [s]
    double rapidTime = 0.0;   ///< [s]
    double totalTime = 0.0;   ///< [s]
    std::size_t moveCount = 0;
    geom3d::Aabb3 bounds; ///< Of all tool tip positions (cycles expanded).
};

/// Length and time estimate. Feed moves use their programmed rate; acceleration is ignored.
Stats computeStats(const Toolpath& path, const MachineLimits& limits = {});

/// Duration of each move [s] as computeStats counts it: feed moves at their programmed rate,
/// rapids at the rapid rate, dwells their time, cycles expanded.
std::vector<double> moveDurations(const Toolpath& path, const MachineLimits& limits = {});

/// Move `m`, starting at `start` with tool axis `startAxis`, cut short at fraction `t` of its
/// length: lines, arcs and helices end part-way and the axis turns part-way. t >= 1 returns the
/// whole move. Cycles and dwells are not split: nullopt (no motion yet) for t < 1.
std::optional<Move> partialMove(const Vec3& start, const Vec3& startAxis, const Move& m, double t);

/// True when any move tilts the tool away from +Z (a multi-axis toolpath, phase 9).
bool isMultiAxis(const Toolpath& path) noexcept;

/// Tool axis before move `moveIndex`: the axis of the previous move, or of the first move at the
/// start (the tool is oriented before it approaches). Along a move the axis turns from this to
/// the move's own axis (spherical interpolation, see slerp).
Vec3 axisBefore(const Toolpath& path, std::size_t moveIndex);

/// Spherical linear interpolation between unit vectors a (t = 0) and b (t = 1).
Vec3 slerp(const Vec3& a, const Vec3& b, double t);

/// Replaces Cycle moves by explicit rapid/feed/dwell moves (G99 behaviour: return to the R
/// plane after each hole), except cycles of the types in `keep`.
Toolpath expandCycles(const Toolpath& path, std::span<const CycleType> keep = {});

/// The toolpath in another frame: positions, arc centres and tool axes mapped by the rigid
/// transform `t` (old frame -> new frame). Feeds are unchanged (lengths are). Arcs stay arcs
/// where their plane is still an arc plane (any arc under a translation, XY arcs under a turn
/// about Z), else they become lines within `chordTolerance`; canned cycles stay cycles where
/// Z still maps to Z, else they are expanded (expandCycles).
Toolpath transformed(const Toolpath& path, const Transform3& t, double chordTolerance = 1e-3);

/// Points along the path for display and checks, grouped into runs of the same feed class.
/// Arcs are sampled within `chordTolerance`. Cycles are expanded.
struct PathRun {
    FeedClass feedClass = FeedClass::Cut;
    std::vector<Vec3> points;
};
std::vector<PathRun> sampleRuns(const Toolpath& path, double chordTolerance = 0.01);

/// Samples one arc move from `start` (inclusive) to its end (inclusive).
std::vector<Vec3> sampleArc(const Vec3& start, const Move& arc, double chordTolerance);

/// Signed sweep of an arc move from `start` [rad]: positive CCW (in its plane), in (-2pi, 2pi].
/// A move whose end coincides with its start in its plane is a full circle.
double arcSweep(const Vec3& start, const Move& arc);

// -- Serialization (golden files, debugging) ----------------------------------------------

/// Compact JSON with numbers rounded to 1e-6 mm (feeds in mm/min).
Json toJson(const Toolpath& path);
Toolpath toolpathFromJson(const Json& doc);

/// Compares two toolpaths move by move with a numeric tolerance. Returns a description of the
/// first difference, or nullopt when equivalent.
std::optional<std::string> compareToolpaths(const Toolpath& expected, const Toolpath& actual,
                                            double tolerance = 1e-5);

} // namespace ocam::toolpath
