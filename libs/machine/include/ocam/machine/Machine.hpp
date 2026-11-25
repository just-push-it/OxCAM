// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Machine definitions and the CL -> machine-axis step ("solve"). 3-axis mills: machine axes
// equal the WCS tool-tip coordinates (the control applies the work offset). 5-axis table-table
// machines (phase 7) index the part for 3+2 setups; see docs/design-log/0019. Multi-axis
// kinematics, simultaneous motion and machine components for collision checks are phase 9
// (Kinematics.hpp, docs/design-log/0023 and 0025).
// Lathes, mill-turn and Swiss machines extend `Kinematics` and the axis list (phases 8-11).

#pragma once

#include <ocam/foundation/Json.hpp>
#include <ocam/geom3d/Convex.hpp>
#include <ocam/geom3d/Mesh.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ocam::machine {

enum class Kinematics {
    Mill3,            ///< X Y Z
    Mill5TableAC,     ///< trunnion: A tilts about X, C rotates the table about Z
    Mill5TableBC,     ///< trunnion: B tilts about Y, C rotates the table about Z
    Lathe2,           ///< 2-axis lathe: X (radius) and Z along the spindle axis (phase 8)
    Mill4A,           ///< X Y Z plus a rotary table A about X (4th axis, phase 9)
    Mill5HeadTableBC, ///< B tilts the spindle head about Y, C rotates the table about Z
    Mill5HeadBC,      ///< fork head: C turns the head about Z, B tilts it about the turned Y
};

std::string_view toString(Kinematics k) noexcept;

struct Axis {
    std::string name; ///< "X", "Y", "Z", ...
    double min = 0.0; ///< Travel limits: mm for X Y Z, degrees for A B C (machine coordinates).
    double max = 0.0;
    double maxRate = 0.0; ///< [mm/s]; 0 = unspecified.

    double travel() const noexcept { return max - min; }
    bool rotary() const noexcept { return name == "A" || name == "B" || name == "C"; }
    bool operator==(const Axis&) const = default;
};

/// A solid part of the machine for collision checks (phase 9), as a box or a cylinder in the
/// frame of what carries it:
///  - Head: origin at the spindle gauge point (tool tip + gauge length along the tool axis), +Z
///    along the tool axis towards the spindle, X and Y as the machine's when the head is not
///    tilted; moves and tilts with the spindle.
///  - Table: the table frame (the machine's axes at rotary 0), origin at the table centre (where
///    the rotary axes meet); turns with every table rotary axis, like the part.
///  - Cradle: the machine frame with its origin at the table centre; turns with the outer table
///    rotary axis only (A of an A/C trunnion, B of a B/C one), otherwise fixed.
struct Component {
    enum class Attach { Head, Table, Cradle };
    enum class Shape { Box, Cylinder };

    std::string name;
    Attach attach = Attach::Table;
    Shape shape = Shape::Box;
    Vec3 min = Vec3::Zero(); ///< Box corners [mm].
    Vec3 max = Vec3::Zero();
    Vec3 base = Vec3::Zero(); ///< Cylinder base centre, unit axis, radius and length [mm].
    Vec3 axis = Vec3::UnitZ();
    double radius = 0.0;
    double length = 0.0;

    /// The solid in its carrier's frame.
    geom3d::Convex convex() const;
    bool operator==(const Component&) const = default;
};

std::string_view toString(Component::Attach attach) noexcept;

// -- Machine models (DL-0037; functions in Model.hpp) --------------------------------------

/// Whether an axis of a kinematic tree turns: A, B and C, a turret's T, and the same letters
/// numbered for a second spindle or channel (C2, T2). W, X2, Z2 and the like slide.
inline bool isRotaryAxis(std::string_view name) noexcept {
    return !name.empty() && (name[0] == 'A' || name[0] == 'B' || name[0] == 'C' || name[0] == 'T');
}

/// An axis (joint) of the kinematic tree.
struct ModelAxis {
    /// The machine axis it is: X, Y, Z, A, B or C (in Machine::axes). A lathe's tree also has T
    /// (the turret's index), C (its spindle), W (its tailstock), Z2 and C2 (its sub-spindle).
    std::string name;
    std::string parent; ///< The axis that carries it; empty = the base.
    /// Unit vector in the model frame at machine zero. Linear axes: what the axis carries moves
    /// this way for positive values (an axis that moves the table points against the machine
    /// axis, so the tool moves along it relative to the table). Rotary axes: what it carries
    /// turns right-handed about it for positive values.
    Vec3 direction = Vec3::UnitX();
    Vec3 pivot = Vec3::Zero(); ///< Rotary axes: a point of the centre line at machine zero.

    bool rotary() const noexcept { return isRotaryAxis(name); }
    bool operator==(const ModelAxis&) const = default;
};

/// A part of the machine: its exact geometry as a STEP file, or (imported from STL) only a mesh.
struct ModelPart {
    std::string name;
    std::string carrier;             ///< The axis that carries it; empty = the base (fixed).
    Vec3 offset = Vec3::Zero();      ///< Placement of the mesh at machine zero: translation [mm]
    Vec3 rotation = Vec3::Zero();    ///< and rotation about X, then Y, then Z [degrees].
    std::uint32_t colour = 0xB4B8BE; ///< 0xRRGGBB
    bool collides = true;            ///< Checked for collisions.
    /// Drawn in the views by default (an enclosure may be hidden); hidden parts are still checked
    /// for collisions.
    bool shown = true;
    /// Of the machine's enclosure (sheet metal, way covers, walls): the views can hide the
    /// enclosure as one, keeping the rest of the machine. It is still checked for collisions.
    bool enclosure = false;
    /// SHA-256 of the part's STEP file, the name it is stored under: its exact geometry, the
    /// source of its mesh. Empty for parts that only have a mesh.
    std::string step;
    /// Parts without a STEP file (imported from STL): SHA-256 of the mesh's binary STL
    /// (writeStl), the name it is stored under. Empty for STEP parts.
    std::string mesh;
    /// The STEP file's bytes, once loaded (attachPartFiles).
    std::shared_ptr<const std::string> stepFile;
    /// The mesh in the part's own coordinates for display and collision checks: a mesh part's
    /// own (attachPartFiles), or for a STEP part tessellated from it at the model's
    /// meshTolerance (document::meshMachineParts); never saved for STEP parts.
    std::shared_ptr<const geom3d::Mesh> geometry;

    /// The part has exact geometry (a STEP file).
    bool exact() const noexcept { return !step.empty(); }
    /// Part coordinates -> model frame at machine zero.
    Transform3 placement() const;
    /// Compares everything but the loaded data (`step` and `mesh` identify it).
    bool operator==(const ModelPart& other) const;
};

struct MachineModel {
    std::vector<ModelAxis> axes;
    std::vector<ModelPart> parts;
    std::string spindleCarrier; ///< The axis that carries the spindle; empty = the base.
    /// The spindle gauge point at machine zero (model frame); the tool axis is +Z there.
    Vec3 spindlePoint = Vec3::Zero();
    /// Chord tolerance the STEP parts are tessellated at for display and checks [mm].
    double meshTolerance = 0.2;

    bool empty() const noexcept { return axes.empty() && parts.empty(); }
    const ModelAxis* axis(std::string_view name) const;
    bool operator==(const MachineModel&) const = default;
};

/// Stations of a turret that take other kinds of block than the rest (DL-0068).
struct StationMounts {
    std::vector<int> stations;
    std::vector<std::string> blockMounts;
    bool operator==(const StationMounts&) const = default;
};

/// A lathe's turret (DL-0067): a disc turning about an index line, with a tool in each station. The
/// machine's capacity (Machine::toolCapacity) is its number of stations, and a tool's number is its
/// station. The turret stands on the +X side of the spindle. Lengths in mm, in the model frame.
struct LatheTurret {
    Vec3 centre = Vec3::Zero(); ///< Where the index line meets the disc's middle, at machine zero.
    Vec3 axis = Vec3::UnitZ();  ///< The index line; the turret turns right-handed about it.
    double diameter = 0.0; ///< The disc as a solid; 0 = none (a part of the model is the disc).
    double width = 0.0;
    double radialReach = 0.0; ///< From the index line to the tip of a tool across the spindle.
    double axialPitch = 0.0;  ///< From the index line to the centre line of a tool along it.
    double axialReach = 0.0;  ///< From the disc's face to the tip of a tool along the spindle.
    /// Where a tooling block seats on a station, in the working position (DL-0068): from the index
    /// line towards the spindle, across (the working station's Y: a turret whose station faces are
    /// shifted sideways, its centre off the spindle's plane), and along the index line from the
    /// turret's centre. Every station has the same seat, turned about the index line. A tool
    /// without a block stands `blockAcross` sideways too.
    double blockRadius = 0.0;
    double blockAcross = 0.0;
    double blockAlong = 0.0;
    /// The kinds of tooling block it takes, by name ("VDI 40", "BMT 55"): a block fits when its
    /// mount type is one of them (mountsMatch). None named: the machine does not say.
    std::vector<std::string> blockMounts;
    /// Stations that take other kinds than `blockMounts` (milling stations on a turret with turning
    /// ones). A station in none of them takes `blockMounts`.
    std::vector<StationMounts> stationMounts;
    /// Where it indexes: at machine zero (the post goes there first), or else where the last
    /// operation ended.
    bool indexAtHome = false;

    /// The kinds of block `station` takes: its entry's in `stationMounts`, else `blockMounts`.
    const std::vector<std::string>& blockMountsAt(int station) const;
    bool operator==(const LatheTurret&) const = default;
};

/// A lathe's tailstock body, on the W axis. What holds the centre is the setup's (LatheTailstock).
struct LatheTailstockBody {
    double z = 0.0; ///< Its front at W = 0, in the model frame; the quill stands in front of it.
    double diameter = 0.0;
    double length = 0.0;
    /// What its quill takes, by name ("MT 4"); empty: the machine does not say.
    std::string quillInterface;
    /// How far its quill goes out of the body, its stroke [mm]; 0: the machine does not say. A
    /// setup's quill out further than this is said, never shortened.
    double quillTravel = 0.0;
    bool operator==(const LatheTailstockBody&) const = default;
};

/// A lathe's sub-spindle, facing the main one: Z2 slides it, C2 turns it.
struct LatheSubSpindle {
    double nose = 0.0;     ///< Where its nose is at Z2 = 0, in the model frame.
    double diameter = 0.0; ///< Its housing, behind the nose.
    double length = 0.0;
    double chuckDiameter = 0.0; ///< Its chuck, in front of the nose.
    double chuckLength = 0.0;
    double grip = 0.0;   ///< How far the work goes into the chuck.
    double workZ2 = 0.0; ///< Z2 while it cuts.
    /// Its speed range [rev/s] (machine file version 16, ADR-0007); 0: the machine does not say.
    /// A speed outside it is refused, never changed.
    double minSpindle = 0.0;
    double maxSpindle = 0.0;
    bool operator==(const LatheSubSpindle&) const = default;
};

/// What a lathe is like, for showing it and checking it in the simulation (DL-0067). The model
/// frame's Z is the spindle's centre line, +Z away from the headstock, with the spindle's nose at
/// Z 0 and +X towards the turret. Lengths in mm. Each solid is optional: a diameter or a size of 0
/// means there is none here, as when a part of the machine's model is that solid.
struct LatheLayout {
    LatheTurret turret;
    double clearance = 1.0; ///< What the turret and its tools must keep from anything [mm].
    double headstockDiameter = 0.0;
    double headstockLength = 0.0;
    /// Where a setup's workholding seats on the main spindle (DL-0068): its back on the spindle's
    /// axis this far along Z from the nose (a back plate's front), turning with C. The machine's
    /// own chuck stands there too.
    double workholdingMount = 0.0;
    /// The machine's own chuck, on the workholding mount: for setups with no workholding of their
    /// own.
    double chuckDiameter = 0.0;
    double chuckLength = 0.0;
    double chuckGrip = 0.0;
    /// The inside of the enclosure, walled on all sides (the door side and the roof start hidden).
    Vec3 enclosureMin = Vec3::Zero();
    Vec3 enclosureMax = Vec3::Zero();
    double wall = 10.0;
    std::optional<LatheTailstockBody> tailstock;
    std::optional<LatheSubSpindle> subSpindle;

    bool hasEnclosure() const noexcept {
        return (enclosureMax.array() > enclosureMin.array()).all();
    }
    bool operator==(const LatheLayout&) const = default;
};

/// A machine axis and a sense along it: +X, -X, +Y, -Y, +Z or -Z.
enum class AxisSense { PlusX, MinusX, PlusY, MinusY, PlusZ, MinusZ };

/// "+X", "-Z", ...
std::string_view toString(AxisSense a) noexcept;
/// Reads "+X", "-z", "x" (+), ...; nullopt for anything else.
std::optional<AxisSense> axisSenseFromString(std::string_view text);
/// The unit vector along it, in machine axes.
Vec3 direction(AxisSense a) noexcept;

/// How a machine stands in the world (DL-0077): the machine axis that points up, and the one that
/// points out of its front, towards where the operator stands. The views turn to it: Top looks
/// down on the machine, Front from where the operator stands.
struct MachineOrientation {
    AxisSense up = AxisSense::PlusZ;
    AxisSense front = AxisSense::MinusY;

    /// The world's axes in machine axes, as columns: right (as the operator sees it), away from the
    /// operator, up. Identity for a machine with Z up and its front at -Y.
    Mat3 worldAxes() const noexcept;
    bool operator==(const MachineOrientation&) const = default;
};

/// What a machine whose file does not say stands like, from its kinematics: mills with Z up and
/// their front at -Y, lathes with the spindle's Z across, Y up and the operator at -X (the turret,
/// at +X, behind the spindle).
MachineOrientation defaultOrientation(Kinematics k) noexcept;

struct Machine {
    std::string id;
    std::string name;
    /// How many times it was changed and saved in the machine library (MachineLibrary::save
    /// counts it): a project's copy of a lower revision is older than the library's. 0: never
    /// counted (files from before version 13, or written outside the library).
    int revision = 0;
    std::string vendor;
    Kinematics kinematics = Kinematics::Mill3;
    /// How it stands, when its file says (a vertical lathe: Z up); otherwise the default of its
    /// kinematics (standing()).
    std::optional<MachineOrientation> orientation;
    std::vector<Axis> axes;
    double minSpindle = 0.0; ///< [rev/s]
    double maxSpindle = 0.0; ///< [rev/s]
    double maxFeed = 0.0;    ///< [mm/s]
    double rapidRate = 0.0;  ///< [mm/s]
    int toolCapacity = 0;
    double toolChangeTime = 0.0; ///< [s]
    int workOffsets = 6;
    /// Head kinematics: distance from the head's rotation centre to the spindle gauge line along
    /// the tool axis [mm] (positions without tool centre point control depend on it).
    double pivotLength = 0.0;
    /// Lathes: driven tools in the turret, with the spindle as a C axis (DL-0062). `liveMaxSpindle`
    /// is the fastest they turn [rev/s]; a speed above it is refused, as above the spindle's.
    bool liveTooling = false;
    double liveMaxSpindle = 0.0;
    /// Lathes: its layout, which turns the machine simulation on (DL-0067). Without one a setup
    /// simulates as it always has: nothing but the tool and the work.
    std::optional<LatheLayout> lathe;
    /// The post processor the machine is posted with unless a setup chooses another: a built-in
    /// post's id or the id of a post the project carries; empty: the built-in post for its
    /// kinematics (document::postFor, DL-0039).
    std::string post;
    /// The machine's values of its default post's program options ({id: value}): what setups on
    /// the machine start from, under their own values (document::postSetup, DL-0039).
    Json postOptions = Json::object();
    std::vector<Component> components;
    /// The machine's parts and kinematic tree for the simulation (Model.hpp); empty = none.
    MachineModel model;

    const Axis* axis(std::string_view name) const;
    /// Table-table machines index the part for 3+2 setups.
    bool canIndex() const noexcept {
        return kinematics == Kinematics::Mill5TableAC || kinematics == Kinematics::Mill5TableBC;
    }
    bool isLathe() const noexcept { return kinematics == Kinematics::Lathe2; }
    /// How it stands: its own orientation, else its kinematics' default.
    MachineOrientation standing() const noexcept {
        return orientation ? *orientation : defaultOrientation(kinematics);
    }
    /// Lathes: a Y axis lifts the tool off the spindle's centre, for holes that do not cross it.
    bool hasYAxis() const { return axis("Y") != nullptr; }
    /// Machines with rotary axes that can orient the tool relative to the part (4 and 5 axes).
    bool hasRotary() const noexcept {
        return kinematics != Kinematics::Mill3 && kinematics != Kinematics::Lathe2;
    }
    /// Rotary axes that turn the part (as opposed to the spindle head).
    bool hasTableRotary() const noexcept {
        return hasRotary() && kinematics != Kinematics::Mill5HeadBC;
    }
    /// Rotary axis letters in output order: "A", "AC", "BC" (empty without rotary axes).
    std::string rotaryAxes() const;
    std::vector<std::string> validate() const;
    bool operator==(const Machine&) const = default;
};

/// Version 2 added the optional machine model (`model`, DL-0037).
/// Version 3 added the parts' `shown` flag.
/// Version 4 dropped the model's table carrier and origin (where the work sits is each setup's
/// placement); older values are ignored.
/// Version 5 stores parts as STEP (`step`, their exact geometry) with `mesh_tolerance`; parts
/// with only a mesh (STL imports, older files) keep `mesh`.
/// Version 6 added the optional default post (`post`).
/// Version 7 added lathes' optional live tooling (`live_tool`).
/// Version 8 added lathes' optional layout (`lathe`), and the model axes T, W, Z2 and C2 for it.
/// Version 9 added where a tooling block seats on a lathe's turret (`block_radius`, `block_along`).
/// Version 10 added the kinds of block a turret takes (`block_mounts`) and what its quill takes.
/// Version 11 added the kinds of block per station (`station_mounts`), the seat's sideways offset
/// (`block_across`) and where workholding seats on the spindle (`workholding_mount`).
/// Version 12 added how the machine stands (`orientation`: `up` and `front`).
/// Version 13 added the machine's `revision`, counted by the machine library.
/// Version 14 added which parts are the enclosure (a part's `enclosure`).
/// Version 15 added how far the tailstock's quill goes out (`quill_travel`).
/// Version 16 added the sub-spindle's speed range (`min_rpm`, `max_rpm`) and Z2 as an axis
/// (ADR-0007).
/// Whether two names of a mount or an interface are the same: ignoring case and spacing, so
/// "VDI 40", "vdi40" and "VDI  40" are one.
bool sameMountName(std::string_view a, std::string_view b);

inline constexpr int kMachineVersion = 16;
const FormatSpec& machineFormat();
const SchemaValidator& machineSchema();

Json toJson(const Machine& machine);
Machine machineFromJson(Json doc, std::string_view sourceName = "machine");
/// Reads a machine file and its model parts' files (meshFolder(), Model.hpp). STEP parts get
/// their file's bytes but no mesh: document::loadMachine also tessellates them.
Machine loadMachine(const std::filesystem::path& path);

/// The built-in generic 3-axis VMC (data/machines/generic-3axis-vmc.json, embedded).
const Machine& genericMill3();

/// Built-in machine by id, or nullptr.
const Machine* builtinMachine(std::string_view id);

// -- Solve --------------------------------------------------------------------------------

/// Rotary positions of an indexed (3+2) setup [rad]. Unused axes stay 0.
struct IndexAngles {
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    bool operator==(const IndexAngles&) const = default;
};

/// Rotary positions that turn `direction` (a unit vector in the table frame, i.e. the machine
/// frame with all rotary axes at 0) to the spindle axis +Z. Positive angles rotate the table
/// right-handed about the machine axis. Of the solutions within the axis limits, the one with
/// the smallest tilt, then the smallest table rotation, is chosen; nullopt when none is within
/// limits (or the machine cannot index).
std::optional<IndexAngles> indexAngles(const Machine& machine, const Vec3& direction);

/// Rotation of the table for given rotary positions (applied to table-frame vectors; the result
/// is in the machine frame, origin at the table centre).
Mat3 tableRotation(const Machine& machine, const IndexAngles& angles);
/// Rotation of the outer table axis only (the cradle of a trunnion).
Mat3 cradleRotation(const Machine& machine, const IndexAngles& angles);
/// Rotation of the spindle head (head frame -> machine frame); identity for table machines.
Mat3 headRotation(const Machine& machine, const IndexAngles& angles);

/// Rotary motion of a solved multi-axis toolpath.
struct RotaryMotion {
    std::string axes;                ///< Rotary axis letters in output order ("A", "AC", "BC").
    IndexAngles start;               ///< Before the first move [rad].
    std::vector<IndexAngles> angles; ///< At the end of each move of SolveResult::path [rad].
    std::vector<double> times;       ///< Duration of each feed move along the tool tip [s].
    bool tcp = true;                 ///< Positions are tool tips (tool centre point control).
};

struct SolveResult {
    toolpath::Toolpath path;          ///< In machine axes (3-axis: WCS), with limits applied.
    std::optional<IndexAngles> index; ///< Set for indexed setups on 5-axis machines.
    /// Set for multi-axis toolpaths (Kinematics.hpp): rotary positions per move. Each move's
    /// axis is then the tool axis in the table frame. Without tool centre point control, moves
    /// that turn a rotary axis carry the feed along the programmed positions (their duration is
    /// in RotaryMotion::times).
    std::optional<RotaryMotion> rotary;
    std::vector<std::string> warnings; ///< Rotary swings near singular positions, ...
    /// Unreachable motion, speeds or feeds beyond the machine limits; the path must not be posted.
    std::vector<std::string> errors;

    bool ok() const noexcept { return errors.empty(); }
};

/// Checks a toolpath against the machine and maps it to machine axes:
///  - every move's tool axis must be +Z (3-axis);
///  - the motion span must fit each axis's travel;
///  - spindle speed and feed rates must be within the machine limits; they are never changed
///    (ADR-0004): a value beyond a limit is an error naming it and where to change it.
SolveResult solve(const Machine& machine, const toolpath::Toolpath& path);

/// Solve for a setup whose WCS axes, expressed in the table frame, are the columns of
/// `wcsInTable`. On a machine that can index, the table is turned so the WCS Z axis points up
/// the spindle (3+2), and the program coordinates are the WCS coordinates rotated about Z so
/// that X and Y follow the machine axes; the work offset is measured at the WCS origin in the
/// indexed position. On a 3-axis machine a tilted WCS means a separate fixturing and the
/// coordinates are used as they are.
SolveResult solve(const Machine& machine, const toolpath::Toolpath& path, const Mat3& wcsInTable);

/// The built-in generic 5-axis trunnion machine (A/C), for 3+2 indexing.
const Machine& genericMill5TableAC();

/// The built-in generic 2-axis lathe.
const Machine& genericLathe2();
const Machine& genericLatheLive();
const Machine& genericLatheLiveY();
/// The built-in lathes with a layout (DL-0067): turret, chuck, tailstock and enclosure; and with a
/// sub-spindle in place of the tailstock.
const Machine& genericLatheTurret();
const Machine& genericLatheSub();

/// The built-in generic 4-axis VMC (rotary table A about X).
const Machine& genericMill4A();

/// The built-in generic 5-axis head-table machine (B head, C table).
const Machine& genericMill5HeadTableBC();

/// Every built-in machine.
std::vector<const Machine*> builtinMachines();

} // namespace ocam::machine
