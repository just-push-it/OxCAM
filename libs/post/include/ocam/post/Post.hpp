// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Post processing: solved toolpaths -> NC intermediate representation -> G-code text, and the
// reverse direction (back-plotting G-code into a toolpath) for verification.
//
//   lower()    : jobs (toolpath + tool + work offset) -> NcProgram (dialect-neutral program
//                structure: tool changes, spindle, coolant, motion, cycles, compensation)
//   backplot() : G-code text -> Toolpath (Fanuc-style subset)
//
// The text is written by a JavaScript post (Engine.hpp). See docs/design-log/0011 and 0039.

#pragma once

#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/machine/Machine.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ocam::post {

// -- NC intermediate representation ---------------------------------------------------------

enum class NcOp {
    Comment,
    ToolChange, ///< tool, toolName
    WorkOffset, ///< workOffset (merged into the next positioning block)
    Spindle,    ///< spindle [rev/s]; 0 = stop
    /// The spindle stops and holds at `angle` (NC program version 5): a back-boring bar about to
    /// pass through its hole off centre (DL-0083). The next Spindle op starts it again.
    SpindleOrient,
    LiveTool,   ///< spindle [rev/s]: a lathe's driven tool on, the spindle a C axis; 0 = both off
    Coolant,    ///< coolant (Off = stop)
    ToolLength, ///< rapid to target.z with tool length compensation
    Rapid,
    Linear,
    /// A live tool's rigid tap (NC program version 4), from where the tool stands to target and
    /// back: spindle, pitch, rPlane, radial, and C in rotary.
    LiveTap,
    /// Cylindrical interpolation on (polarOn, wrapRadius) or off (NC program version 4): while on,
    /// feed moves are X (the radius), Z and C, with the control keeping the feed on that cylinder.
    Wrap,
    /// Polar interpolation on (polarOn, target = where the tool stands in the WCS's X and Y) or
    /// off (NC program version 4): while on, feed moves are in those X and Y.
    Polar,
    Arc, ///< centre (absolute), clockwise
    Dwell,
    Cycle, ///< canned cycle at target.xy with `cycle`
    CycleCancel,
    CompensationOn,  ///< side (merged into the next linear block)
    CompensationOff, ///< merged into the next block
    Index,           ///< rotary positioning of a 3+2 setup: rotary (a, b, c) [rad], text = axes
    HomeZ,           ///< retract Z to the machine reference (before indexing)
    TcpOff,          ///< ends tool centre point control (after a multi-axis toolpath)
    /// A probing cycle (NC program version 6, DL-0090): `probe`, its moves in `probeMoves`.
    Probe,
    OperationBegin, ///< operation = index into NcProgram::operations (NC program version 2)
    OperationEnd,   ///< operation = index into NcProgram::operations
    /// The main spindle's chuck opens (`open`) or closes (NC program version 7, DL-0091).
    Chuck,
    /// The bar feeder pushes (polarOn) or stops (NC program version 7, DL-0091).
    BarFeeder,
    /// Z2 to target.z, in the machine's Z2, at `feed` or at rapid when it is 0 (NC program
    /// version 7, ADR-0007).
    SubAxis,
    /// The spindles synchronised (polarOn), at the angle `syncAngle` when it has one, or no longer
    /// (NC program version 7, ADR-0007).
    SpindleSync,
};

/// One move of a probing cycle (NC program version 6).
struct NcProbeMove {
    /// A measuring move, which stops where the stylus touches; else a protected positioning
    /// move, which stops if it touches anything.
    bool measure = false;
    /// Where the move goes (the tool tip, program coordinates). A measuring move goes on past
    /// the nominal surface by the cycle's overtravel, looking for it.
    Vec3 target = Vec3::Zero();
    /// Measuring moves: where the tool tip stops on the model's nominal surface.
    Vec3 expected = Vec3::Zero();
    double feed = 0.0; ///< [mm/s]

    bool operator==(const NcProbeMove&) const = default;
};

struct NcInstr {
    NcOp op = NcOp::Comment;
    Vec3 target = Vec3::Zero();
    Vec3 center = Vec3::Zero();
    bool clockwise = false;
    toolpath::ArcPlane plane = toolpath::ArcPlane::XY; ///< Arc: G17, G18 or G19.
    double feed = 0.0;                                 ///< [mm/s]
    int tool = 0;
    std::string toolName;
    double spindle = 0.0; ///< [rev/s]
    tooling::Coolant coolant = tooling::Coolant::Off;
    int workOffset = 1;
    toolpath::DrillCycle cycle;
    toolpath::Compensation side = toolpath::Compensation::None;
    double dwell = 0.0; ///< [s]
    std::string text;
    Vec3 rotary = Vec3::Zero(); ///< Index: A, B, C [rad]
    double pitch = 0.0;         ///< Linear: thread lead per revolution (G32) [mm]
    bool tapping = false;       ///< LiveTool: on for a rigid tap, which the tap's block starts.
    bool milling = false;       ///< LiveTool: on with C held still; `plane` is what it mills in.
    double rPlane = 0.0;        ///< LiveTap: where it starts along its direction [mm]
    double angle = 0.0;         ///< SpindleOrient: the spindle's angle [rad]
    bool radial = false;        ///< LiveTap: along X, across the part, rather than along Z.
    bool reverse = false;       ///< Spindle: counter-clockwise. LiveTap: a left-hand thread.
    bool polarOn = false;       ///< Polar, Wrap, BarFeeder: on, or off.
    bool open = false;          ///< Chuck: opens, or closes.
    bool sub = false;           ///< Spindle, Chuck: the sub-spindle's (ADR-0007).
    std::optional<double> syncAngle; ///< SpindleSync: the sub-spindle's to the main [rad].
    double wrapRadius = 0.0;         ///< Wrap: the cylinder [mm].
    double surfaceSpeed = 0.0;       ///< Spindle: constant surface speed [mm/s]
    double maxSpindle = 0.0;         ///< Spindle: limit under constant surface speed [rev/s]
    /// Rapid / Linear of a multi-axis toolpath: the rotary axis letters to position ("A", "AC",
    /// "BC") at `rotary`.
    std::string axes;
    /// Linear without tool centre point control: inverse-time feed (G93), 1 / move time [1/min].
    double inverseTime = 0.0;
    bool tcp = false; ///< ToolLength: start tool centre point control.
    /// OperationBegin / OperationEnd, Probe: index into NcProgram::operations.
    int operation = -1;
    /// Linear / Arc: what the move is for (cut, plunge, ramp, lead-in, lead-out, link).
    toolpath::FeedClass feedClass = toolpath::FeedClass::Cut;
    toolpath::ProbeCycle probe;          ///< Probe: what it measures and sets.
    std::vector<NcProbeMove> probeMoves; ///< Probe: its moves, in order.
};

/// A tool the program uses, for the post's tool table (NC program version 2).
struct NcTool {
    int number = 0;
    std::string name;
    std::string comment;
    std::string type;      ///< Tool type id ("flat_end_mill", ...); empty when unknown.
    double diameter = 0.0; ///< [mm]
    double cornerRadius = 0.0;
    int flutes = 0;
    double threadPitch = 0.0;
    double gaugeLength = 0.0; ///< Tool and holder, from the tip to the gauge line [mm].
    /// Turning tools: ISO shape letter, nose radius [mm] and tip orientation (1-9); the shape is
    /// empty for other tools.
    std::string insertShape;
    double noseRadius = 0.0;
    int insertOrientation = 0;
};

enum class NcOrientation {
    ThreeAxis,    ///< along the spindle, no rotary positioning
    Indexed,      ///< the rotary axes at a fixed position (3+2): NcOperation::index
    Simultaneous, ///< the rotary axes move with the cut
};

/// An operation of the program (NC program version 2).
struct NcOperation {
    std::string id; ///< The operation's id in the project (empty outside a project).
    std::string name;
    std::string comment;
    std::string strategy; ///< Operation type id ("pocket2d", ...).
    int tool = 0;
    tooling::Coolant coolant = tooling::Coolant::Off;
    int workOffset = 1;
    NcOrientation orientation = NcOrientation::ThreeAxis;
    machine::IndexAngles index; ///< Indexed: the rotary positions [rad].
    /// Box around its motion in program coordinates (including arc extremes) [mm].
    Vec3 boundsMin = Vec3::Zero();
    Vec3 boundsMax = Vec3::Zero();
    /// The values of the post's per-operation options (set when posting; an object).
    Json options = Json::object();
    /// The spindle the work is on: the main one, or the sub-spindle (NC program version 7,
    /// ADR-0007).
    bool subSpindle = false;
};

/// The machine the program is for, as a post sees it (NC program version 2).
struct NcMachine {
    std::string id;
    std::string name;
    std::string kinematics; ///< machine::toString(Kinematics), e.g. "mill3"
    std::vector<machine::Axis> axes;
    std::string rotaryAxes;  ///< "A", "AC", "BC" or empty
    double minSpindle = 0.0; ///< [rev/s]
    double maxSpindle = 0.0; ///< [rev/s]
    double maxFeed = 0.0;    ///< [mm/s]
    double rapidRate = 0.0;  ///< [mm/s]
    int toolCapacity = 0;
    int workOffsets = 0;
};

NcMachine ncMachine(const machine::Machine& machine);

struct NcProgram {
    std::string name;
    int number = 1000;
    std::string project; ///< Version 2: project and setup names.
    std::string setup;
    std::string time; ///< Version 2: when the program was made, ISO 8601 UTC (may be empty).
    std::vector<NcTool> tools;           ///< Version 2: in order of first use.
    std::vector<NcOperation> operations; ///< Version 2: in program order.
    std::optional<NcMachine> machine;    ///< Version 2.
    std::vector<NcInstr> instrs;
    std::vector<std::string> warnings;
};

std::string_view toString(NcOp op) noexcept;
std::optional<NcOp> parseNcOp(std::string_view text) noexcept;

/// The post API: a versioned JSON form of an NcProgram, exactly what a post receives. Lengths
/// in mm, feeds in mm/min, spindle speed in rev/min, dwell in s, angles in degrees; coordinates
/// a block leaves unchanged are null. See docs/design-log/0015 and 0039.
/// Version 2 added operation_begin / operation_end, feed_class, and the program's project,
/// setup, time, tools, operations and machine. Version 5 added spindle_orient and the back_bore
/// cycle (shift_x, shift_y, orient); version 6 the `probe` instruction (DL-0090); version 7
/// chuck, bar_feeder, sub_axis, spindle_sync and the spindle a speed is for (DL-0091, ADR-0007).
/// Version 1 documents are still read.
inline constexpr int kNcProgramJsonVersion = 7;
Json toJson(const NcProgram& program);
/// Throws Error(Parse) for malformed documents.
NcProgram ncProgramFromJson(const Json& doc);

struct PostJob {
    std::string name;        ///< Operation name (comment).
    toolpath::Toolpath path; ///< Solved toolpath (machine axes).
    std::string toolName;
    int workOffset = 1; ///< 1 = G54
    /// Rotary positions for an indexed (3+2) setup, and the machine's rotary axis letters
    /// ("AC", "BC"). Jobs without an index leave the rotary axes where they are.
    std::optional<machine::IndexAngles> index;
    std::string indexAxes;
    /// Rotary motion of a multi-axis toolpath (machine::solve), one position per move.
    std::optional<machine::RotaryMotion> rotary;
    std::string comment;     ///< The operation's comment.
    std::string operationId; ///< The operation's id in the project.
    /// The tool for the program's tool table; unset: its number and `toolName` only.
    std::optional<NcTool> tool;
    /// The work is on the sub-spindle (ADR-0007): its spindle instructions are the sub-spindle's.
    bool subSpindle = false;
};

/// How a program is reshaped for a post (its manifest's declarations, DL-0039;
/// post::lowerOptionsFor).
struct LowerOptions {
    /// A lathe whose turret indexes at home: go there before each tool change after the first.
    bool indexAtHome = false;
    /// A lathe: to each toolpath's start along Z, then in on X (machine::approachOutFirst says
    /// when not). A mill stays over the start first and comes down.
    bool latheApproach = false;
    bool cannedCycles = true; ///< false: cycles are expanded into explicit moves
    /// Multi-axis toolpaths are solved for tool centre point control (see machine::SolveOptions;
    /// used by document::lowerSetup).
    bool tcp = true;
    /// With cannedCycles: the cycle types the post takes; others are expanded. Empty: all.
    std::vector<toolpath::CycleType> cycleTypes;
    /// Arc planes the post takes; arcs in other planes become lines within `tolerance`.
    std::vector<toolpath::ArcPlane> arcPlanes{toolpath::ArcPlane::XY, toolpath::ArcPlane::ZX,
                                              toolpath::ArcPlane::YZ};
    bool helicalArcs = true;        ///< false: helical arcs become lines within `tolerance`
    double maxArcSweep = 2.0 * kPi; ///< [rad] longer arcs are split into equal parts
    double tolerance = 0.002;       ///< [mm] chord error of arcs made into lines
};

NcProgram lower(const std::vector<PostJob>& jobs, std::string programName, int programNumber,
                const LowerOptions& options = {});

// -- Back-plotting --------------------------------------------------------------------------

struct BackplotResult {
    toolpath::Toolpath path;           ///< Motion in mm (WCS), canned cycles as Cycle moves.
    std::vector<std::string> warnings; ///< Unsupported words, missing feed, ...
    std::vector<int> toolChanges;      ///< Tool numbers in order.
};

/// Parses a Fanuc-style G-code subset: G0-G4, G17, G20/G21, G28 and G53 blocks (machine
/// positions, not modelled), G40-G43/G49, G54-G59 and G154, G73/G80-G85, G90/G91, G93/G94,
/// G98/G99, G187 (no motion); F S T
/// M words; N numbers; ( ) and ; comments. A dwell P with a decimal point is in seconds, without
/// one in milliseconds (Haas-style; Fanuc-style posts write whole milliseconds). Other G codes are
/// ignored with a warning. Throws Error(Parse) on malformed blocks.
struct BackplotOptions {
    /// Lathe programs: X words are diameters, T words change tools without M06, G98/G99 select
    /// feed per minute/revolution, G32 cuts threads.
    bool lathe = false;
    /// Multi-axis programs: A/B/C words give each move's tool axis (in the table frame) through
    /// this machine's kinematics. Inverse-time feeds (G93) become feeds along the programmed
    /// X Y Z positions.
    const machine::Machine* machine = nullptr;
};
BackplotResult backplot(std::string_view gcode, const BackplotOptions& options = {});

/// Compares back-plotted motion against the CL toolpaths it was posted from (in order).
/// Moves before each toolpath's start position (tool changes, safe moves) are skipped; moves
/// shorter than `tolerance` are ignored on both sides. Returns the first difference, or
/// nullopt. `maxDeviation` receives the largest endpoint / centre deviation found. A rapid may be
/// a linear feed move in the program (its feed is not compared), not the other way. Feeds are
/// compared unless `compareFeeds` is false (lathe programs at constant surface speed give feeds
/// per revolution at an rpm the program does not state). Tool axes are compared for multi-axis
/// toolpaths (within 1e-4).
std::optional<std::string> compareBackplot(const std::vector<toolpath::Toolpath>& expected,
                                           const toolpath::Toolpath& plotted, double tolerance,
                                           double* maxDeviation = nullptr,
                                           bool compareFeeds = true);

} // namespace ocam::post
