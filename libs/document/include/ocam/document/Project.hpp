// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The project data model: plain values, cheap to copy (undo keeps snapshots), Qt-free and
// OpenCASCADE-free. Runtime state such as loaded B-rep models lives in Document.
//
// Project -> models (embedded STEP files: parts and fixtures)
//         -> tools (snapshot of the tools and holders the project uses)
//         -> machines (custom machine definitions the setups use; built-ins are not stored)
//         -> workholdings (copies of the workholding templates the setups use; their bodies are
//            embedded STEP files like the models)
//         -> setups -> operations
//
// All coordinates are in the project frame, in millimetres.

#pragma once

#include <ocam/cad/SubShape.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Math.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/foundation/Uuid.hpp>
#include <ocam/machine/Machine.hpp>
#include <ocam/tooling/ToolLibrary.hpp>

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ocam::document {

/// Right-handed coordinate frame: origin plus X and Z axes (Y = Z x X).
struct Frame {
    Vec3 origin = Vec3::Zero();
    Vec3 xAxis = Vec3::UnitX();
    Vec3 zAxis = Vec3::UnitZ();

    Vec3 yAxis() const { return zAxis.cross(xAxis); }
    /// Maps frame-local coordinates to the parent frame.
    Transform3 toParent() const;
    /// Re-orthonormalizes the axes (Gram-Schmidt on X against Z). Throws Error(InvalidArgument)
    /// for zero or parallel axes.
    Frame normalized() const;

    bool operator==(const Frame&) const = default;
};

/// A STEP model embedded in the project, identified by the SHA-256 of its file content.
struct ModelEntry {
    Uuid id = Uuid::generate();
    std::string name;
    std::string sourcePath; ///< Where it was imported from (informational).
    std::string sha256;     ///< Key of the embedded file: models/<sha256>.step
    Frame placement;        ///< Model frame within the project.

    bool operator==(const ModelEntry&) const = default;
};

/// The shape of a setup's stock (DL-0038).
enum class StockKind {
    Box,       ///< A block along the WCS axes.
    Cylinder,  ///< A round bar along a WCS axis.
    Tube,      ///< A round bar with a bore along a WCS axis.
    FromSetup, ///< The stock another, earlier setup left.
};

/// How a box, cylinder or tube stock gets its size (DL-0038).
enum class StockSize {
    Fixed,          ///< Typed dimensions.
    OffsetFromPart, ///< The part's extent grown by the offsets, rounded up.
    Part,           ///< The part's extent, nothing added ("No additional stock").
};

std::string_view toString(StockKind kind) noexcept;
std::optional<StockKind> parseStockKind(std::string_view text) noexcept;
std::string_view toString(StockSize size) noexcept;
std::optional<StockSize> parseStockSize(std::string_view text) noexcept;

/// The stock of a setup. Box, cylinder and tube stock is centred on the part's box along the
/// WCS axes (on the project origin without a model; an offset stock where the offsets put it)
/// and then moved by `shift`.
struct Stock {
    StockKind kind = StockKind::Box;
    StockSize size = StockSize::OffsetFromPart;
    /// Cylinder, tube: the WCS axis it runs along (0 = X, 1 = Y, 2 = Z).
    int axis = 2;
    /// Box, Fixed: its width, length and height along the WCS X, Y and Z axes [mm].
    Vec3 boxSize = Vec3::Zero();
    double diameter = 0.0;      ///< Cylinder, tube, Fixed: the outer diameter [mm].
    double length = 0.0;        ///< Cylinder, tube, Fixed: the length along the axis [mm].
    double innerDiameter = 0.0; ///< Tube, every size: the bore's diameter [mm].
    /// OffsetFromPart: added on every side across the stock (box: in WCS X and Y; cylinder and
    /// tube: to the radius).
    double offsetSide = 1.0;
    /// OffsetFromPart: added at the top (box, and along Z: WCS +Z) or at the + end (along X or
    /// Y) of the part.
    double offsetTop = 1.0;
    double offsetBottom = 0.0; ///< OffsetFromPart: added at the bottom or the - end.
    /// OffsetFromPart: sizes (a cylinder's diameter and length) are rounded up to a multiple of
    /// this [mm] (0 = exact). Across the stock the extra goes equally to both sides, along WCS Z
    /// to the top, along X or Y equally to both ends.
    double roundTo = 0.0;
    /// Box, cylinder, tube: moved from centred along the WCS X, Y and Z axes [mm] ("Base X").
    Vec3 shift = Vec3::Zero();
    /// Box, Fixed, from files of earlier versions: its corners min and max in the project frame,
    /// kept until the setup page edits the stock (which sizes it where it is).
    std::optional<std::array<Vec3, 2>> boxCorners;
    /// Cylinder, Fixed, from files of earlier versions: its base centre and axis in the project
    /// frame, kept until the setup page edits the stock (which puts it on the nearest WCS axis).
    std::optional<std::array<Vec3, 2>> cylinderPlacement;
    std::optional<Uuid> sourceSetup; ///< FromSetup.

    bool operator==(const Stock&) const = default;
};

/// The side of a selected outline a contour or chamfer runs on.
enum class GeometrySide {
    Default, ///< The operation's side.
    Outside,
    Inside,
};

/// Reference to a face or edge of a model, robust to re-import (see cad/SubShape.hpp).
struct ShapeRef {
    Uuid modelId;
    cad::SubShapeType type = cad::SubShapeType::Face;
    int index = 0; ///< 1-based hint.
    cad::ShapeSignature signature;
    /// Contours and chamfers: the side of this face's outline, or of the closed loop this edge
    /// belongs to (all edges of a loop carry the same side).
    GeometrySide side = GeometrySide::Default;

    bool operator==(const ShapeRef&) const = default;
    /// The same face, edge or vertex (whatever its side).
    bool sameShape(const ShapeRef& o) const noexcept {
        return modelId == o.modelId && type == o.type && index == o.index;
    }
};

/// A body of a workholding template (DL-0035): an embedded STEP file in the template's
/// coordinates (every part of a template shares them, as one assembly file places its bodies),
/// carried by the base or by a moving member.
struct WorkholdingPart {
    Uuid id = Uuid::generate();
    std::string name;
    std::string sha256;     ///< Key of the embedded file (models/<sha256>.step in a project).
    std::string sourcePath; ///< Where it was imported from (informational).
    std::string member;     ///< The member that moves it; empty = the fixed base.
    std::uint32_t colour = 0x6B809E; ///< 0xRRGGBB
    bool collides = true;            ///< Toolpaths keep clear of it; checked in the simulation.
    bool shown = true;               ///< Drawn in the views.
    /// A chuck's own top jaw: left out when a setup puts a jaw set on its master jaws (DL-0069).
    bool topJaw = false;

    bool operator==(const WorkholdingPart&) const = default;
};

/// A moving member of a workholding (a jaw, a clamp): its parts and datums move along a line.
struct WorkholdingMember {
    std::string name;
    Vec3 direction = Vec3::UnitX(); ///< Positive travel, template coordinates (unit).
    double minTravel = 0.0;         ///< From where its parts are modelled [mm].
    double maxTravel = 100.0;
    /// Moves with another member (a self-centring vise's second jaw): its travel is `ratio`
    /// times that member's, along its own direction. Empty = moved on its own.
    std::string linkedTo;
    double ratio = 1.0;

    bool operator==(const WorkholdingMember&) const = default;
};

enum class DatumKind {
    Picked,   ///< A face, edge or vertex of a part, or a point clicked on a face.
    Typed,    ///< A point and direction typed in template coordinates.
    Offset,   ///< Another datum moved along an axis or its own direction.
    Midpoint, ///< Halfway between two datums.
};
/// What a picked or typed datum is.
enum class DatumShape {
    Plane, ///< A flat face: its centre and normal.
    Axis,  ///< A round face or circular edge: a point of its axis and the axis.
    Line,  ///< A straight edge: its middle and direction.
    Point, ///< A vertex or a point on a face (the face's normal, when on one).
};
/// The direction an offset datum moves along.
enum class DatumAxis { X, Y, Z, Direction };
std::string_view toString(DatumKind kind) noexcept;
std::optional<DatumKind> parseDatumKind(std::string_view text) noexcept;
std::string_view toString(DatumShape shape) noexcept;
std::optional<DatumShape> parseDatumShape(std::string_view text) noexcept;
std::string_view toString(DatumAxis axis) noexcept;
std::optional<DatumAxis> parseDatumAxis(std::string_view text) noexcept;

/// A named point with a direction (DL-0035), defined on the geometry or relative to other datums.
/// Picked and typed datums move with a member: a picked one with its part's, a typed one with
/// `member`; offsets and midpoints follow the datums they come from.
struct WorkholdingDatum {
    std::string name;
    DatumKind kind = DatumKind::Typed;
    DatumShape shape = DatumShape::Point;
    /// Picked: the part and its face, edge or vertex (persistent reference, cad/SubShape.hpp).
    std::optional<Uuid> part;
    cad::SubShapeType type = cad::SubShapeType::Face;
    int index = 0;
    cad::ShapeSignature signature;
    std::string member; ///< Typed: the member it moves with; empty = the base.
    /// Picked and typed: template coordinates with the parts as modelled.
    Vec3 point = Vec3::Zero();
    Vec3 direction = Vec3::UnitZ(); ///< Unit, or zero for a point without one.
    std::string from;               ///< Offset and midpoint: the datum it starts from.
    std::string to;                 ///< Midpoint: the other datum.
    DatumAxis along = DatumAxis::Z; ///< Offset: template X, Y, Z or `from`'s direction.
    double distance = 0.0;          ///< Offset [mm].

    bool operator==(const WorkholdingDatum&) const = default;
};

/// How a stock holding holds the stock.
enum class HoldingKind {
    Jaws,     ///< Two faces clamp the stock's width.
    Dovetail, ///< Flanks hold a dovetail tenon machined on the stock's bottom.
    Seat,     ///< A frame the stock's chosen point goes to (custom fixtures).
    Chuck,    ///< Jaws (or a collet) gripping a bar on its diameter, centred on an axis.
};
std::string_view toString(HoldingKind kind) noexcept;
std::optional<HoldingKind> parseHoldingKind(std::string_view text) noexcept;

/// Where and how a workholding holds the stock (DL-0035); a template may have several (two vise
/// stations, step jaws).
struct StockHolding {
    std::string name;
    HoldingKind kind = HoldingKind::Jaws;
    /// Jaws: the clamping faces; the stock's side rests on `faceA` (and is centred when both
    /// move). Dovetail: the two flanks. Chuck: `faceA` is a jaw's gripping face (or a collet's
    /// bore), its distance from the axis the radius gripped.
    std::string faceA;
    std::string faceB;
    /// Jaws: the jaw tops (the grip depth is measured down from them). Dovetail: the face the
    /// stock's shoulder sits on. Seat: the frame's origin, its Z along the datum's direction.
    /// Chuck: the front of the jaws (or the step the bar's end sits against), facing out of the
    /// chuck; the grip length is measured back from its plane.
    std::string seat;
    std::string xDatum;     ///< Seat: the frame's X along this datum's direction (else X).
    std::string axis;       ///< Chuck: the spindle axis (a datum on it, along it).
    double gripDepth = 5.0; ///< Jaws: how far the stock reaches below the jaw tops; chuck: into
                            ///< the jaws behind the seat [mm].
    /// Chuck whose faces no member moves (a collet): how much smaller than its bore a bar may be
    /// and still be gripped [mm].
    double gripRange = 0.0;
    double depth = 5.0; ///< Dovetail: the tenon's height below the shoulder [mm].
    /// Dovetail: the tenon's width at the shoulder it is made for [mm]; 0 = the flanks as
    /// modelled.
    double tenonWidth = 0.0;
    /// Seat: the point of the stock's box at the frame's origin, per stock axis 0 = min,
    /// 1 = middle, 2 = max.
    std::array<int, 3> anchor{1, 1, 0};

    bool operator==(const StockHolding&) const = default;
};

/// How jaws go on a chuck's master jaws (DL-0069): `interface` names them, a label compared as
/// mount names are (a setup offers the jaw sets whose interface the chuck takes); `seat` is a datum
/// on master jaw 1 (member "Jaw 1"), its direction out of the chuck, where a jaw's seat goes. The
/// other master jaws have theirs turned about the chuck's axis.
struct JawMount {
    std::string interfaceName; ///< "interface" in the file.
    std::string seat;

    bool operator==(const JawMount&) const = default;
};

/// A step of a jaw (DL-0069): the face that grips (a datum: its normal facing the axis grips an
/// outside diameter, facing away an inside one) and the face the work's end stops against.
struct JawStep {
    std::string name;
    std::string grip;
    std::string stop;

    bool operator==(const JawStep&) const = default;
};

/// One way a jaw set goes on a master jaw (DL-0069), as a vise's holdings are its ways of holding
/// (DL-0035): a jaw that goes on either way round is two options ("OD", "Reversed, ID"), each with
/// its own seat and steps. `seat` is the datum that sits on the chuck's seat, its direction out of
/// the chuck; `inward` (the set's coordinates) is towards the chuck's axis.
struct JawOption {
    std::string name;
    std::string seat;
    Vec3 inward = Vec3::UnitX();
    std::vector<JawStep> steps;

    bool operator==(const JawOption&) const = default;
};

/// A set of top jaws for chucks with a jaw mount (DL-0069): one jaw's parts (a template with no
/// members or holdings), copied onto each master jaw, and the ways it goes on.
struct JawSet {
    enum class Kind {
        Hard, ///< Steps to grip with, as made.
        Soft, ///< A blank, bored to fit in the setup; its steps are what it clamps a ring by.
    };
    std::string interfaceName; ///< "interface" in the file.
    Kind kind = Kind::Hard;
    std::string front; ///< The face towards the work: soft jaws are bored back from it.
    std::vector<JawOption> options;

    /// The option by name; empty: the first. Null when there is none.
    const JawOption* findOption(std::string_view name) const;
    bool operator==(const JawSet&) const = default;
};

/// A reusable fixture (DL-0035): STEP parts on a fixed base or on moving members, datums on
/// them, and the stock holdings that say where the stock goes. `mount` is the datum the
/// workholding sits on the machine by (its base's bottom, a zero-point pin); empty = the
/// template origin.
struct WorkholdingTemplate {
    Uuid id = Uuid::generate();
    std::string name;
    std::string description;
    std::vector<WorkholdingPart> parts;
    std::vector<WorkholdingMember> members;
    std::vector<WorkholdingDatum> datums;
    std::vector<StockHolding> holdings;
    std::string mount;
    Vec3 up = Vec3::UnitZ();          ///< Up, out of the fixture, template coordinates.
    std::optional<JawMount> jawMount; ///< A chuck that takes jaw sets.
    std::optional<JawSet> jawSet;     ///< A jaw set, not a fixture of its own.

    const WorkholdingPart* findPart(const Uuid& id) const;
    const WorkholdingMember* findMember(std::string_view name) const;
    const WorkholdingDatum* findDatum(std::string_view name) const;
    const StockHolding* findHolding(std::string_view name) const;
    /// Problems as human-readable sentences (empty = usable).
    std::vector<std::string> validate() const;
    bool operator==(const WorkholdingTemplate&) const = default;
};

/// How a setup's jaws (or dovetail flanks) are positioned.
enum class JawSetting {
    OnStock, ///< Closed onto the stock's width (a dovetail: onto the holding's tenon width).
    Opening, ///< Opened to a typed value (a dovetail: the tenon width).
};

/// Which diameter a chuck's jaws grip (DL-0069).
enum class GripSide {
    Outside, ///< OD: the jaws close on it.
    Inside,  ///< ID: the jaws open into a bore.
};

/// Soft jaws bored to fit (DL-0069). The jaws clamp a ring or plug of `clampDiameter` on the jaw
/// set's step while they are bored to `diameter` (an outside grip) or turned to it (an inside one),
/// `depth` back from their front. One definition for the setup's jaws and the operation that bores
/// them (turn_soft_jaws).
struct SoftBore {
    /// Unset: the held diameter of the stock, plus `clearance` for an outside grip, minus it for an
    /// inside one.
    std::optional<double> diameter;
    double clearance = 0.0;
    double depth = 0.0;
    double clampDiameter = 0.0;

    bool operator==(const SoftBore&) const = default;
};

/// A workholding template holding a setup's stock (DL-0035). The stock's axes are the WCS's
/// turned by `rotation` about WCS Z onto the holding's (jaws: their closing direction); the
/// moving members close on it and the workholding is placed around it.
struct SetupWorkholding {
    Uuid templateId;     ///< In Project::workholdings.
    std::string holding; ///< A stock holding of the template; empty = its first.
    JawSetting jaws = JawSetting::OnStock;
    double opening = 0.0; ///< JawSetting::Opening [mm].
    /// How far the stock reaches below the jaw tops [mm]; unset = the holding's.
    std::optional<double> gripDepth;
    int rotation = 0;   ///< Degrees about WCS Z: 0, 90, 180 or 270.
    double shift = 0.0; ///< The stock moved along the jaws (Y of the holding) [mm].
    /// Travel of members the holding does not close [mm]; missing ones stay as modelled.
    std::map<std::string, double> travel;
    /// A chuck with a jaw mount (DL-0069): the jaw set on its master jaws (in
    /// Project::workholdings); unset: the chuck's own jaws.
    std::optional<Uuid> jawSet;
    std::string jawOption; ///< The set's option by name; empty: its first.
    GripSide grip = GripSide::Outside;
    std::string step; ///< The step that grips; empty: the first that grips the way `grip` says.
    /// A jaw set: how far the stock's held end stands out from the step's stop face (soft jaws:
    /// from the bore's floor) [mm]; 0: against it.
    double stopOffset = 0.0;
    std::optional<SoftBore> softBore; ///< A soft jaw set: bored to fit.

    bool operator==(const SetupWorkholding&) const = default;
};

/// Where a setup's work (its WCS with the stock, part, fixtures and workholding) sits on its
/// machine (DL-0037): carried by a machine axis, with a point of the work (the WCS origin unless
/// another was picked) at a point of the machine, both at machine zero. The work's axes are the
/// project frame's turned by `rotation` onto the machine's (0 = the part upright, as modelled).
/// Setups without one use defaultPlacement (PostSetup.hpp).
struct SetupPlacement {
    std::string carrier;        ///< The machine axis that carries the work; empty = the base.
    Vec3 target = Vec3::Zero(); ///< Machine point, model frame at machine zero [mm].
    std::string targetOn;       ///< What it was snapped to, as shown; empty = typed.
    /// The point of the work placed there, project frame; unset = the WCS origin.
    std::optional<Vec3> workPoint;
    std::string workPointOn; ///< What it was picked on, as shown.
    /// The workholding's mount datum (posed around the stock) is the point placed there, so the
    /// workholding snaps onto the machine and the stock, part and WCS follow (DL-0035).
    bool onWorkholding = false;
    Vec3 offset = Vec3::Zero();   ///< Moves the work from the target, model frame [mm].
    Vec3 rotation = Vec3::Zero(); ///< About machine X, then Y, then Z, about the target [deg].

    bool operator==(const SetupPlacement&) const = default;
};

/// Where an operation height is measured from (WCS Z).
enum class HeightRef {
    Default, ///< The operation type's default (defaultHeights); files of earlier versions.
    StockTop,
    StockBottom,
    PartTop,
    PartBottom,
    Geometry,  ///< The operation's selected faces: the highest (top) or lowest (bottom).
    Origin,    ///< WCS Z0.
    Clearance, ///< The operation's clearance height.
    Retract,   ///< The operation's retract height.
    TopHeight, ///< The operation's top height.
    Picked,    ///< A face or edge picked for this height: the Z of its centre.
    HoleTop,   ///< Each hole's own top (the feed height, R plane, of hole operations).
};
std::string_view toString(HeightRef ref) noexcept;
std::optional<HeightRef> parseHeightRef(std::string_view text) noexcept;

/// An operation height: a reference plus an offset along WCS Z [mm].
struct HeightSetting {
    HeightRef from = HeightRef::Default;
    double offset = 0.0;
    std::optional<ShapeRef> shape; ///< With HeightRef::Picked.

    bool isDefault() const noexcept { return from == HeightRef::Default; }
    bool operator==(const HeightSetting&) const = default;
};

/// The heights of an operation. New operations start with defaultHeights(); a Default setting
/// (files of earlier versions) means that value.
struct OperationHeights {
    HeightSetting clearance; ///< Rapid level between regions and operations.
    HeightSetting retract;   ///< Rapid level within the operation.
    HeightSetting feed;      ///< Where feed moves start (the R plane of hole cycles).
    HeightSetting top;       ///< Top of the material the operation cuts.
    HeightSetting bottom;    ///< Final depth.

    bool isDefault() const noexcept {
        return clearance.isDefault() && retract.isDefault() && feed.isDefault() &&
               top.isDefault() && bottom.isDefault();
    }
    bool operator==(const OperationHeights&) const = default;
};

/// Values of the operation's cutting preset changed for this operation only (unset = the
/// preset's value). The preset stays the source of everything else.
struct PresetOverrides {
    std::optional<double> surfaceSpeed;   ///< vc [mm/s]
    std::optional<double> feedPerTooth;   ///< fz [mm] (feed per revolution for inserts)
    std::optional<double> axialDepth;     ///< ap [mm]
    std::optional<double> radialDepth;    ///< ae [mm]
    std::optional<double> plungeFeed;     ///< Plunge feed rate [mm/s].
    std::optional<double> rampFeed;       ///< Ramp and helix feed rate [mm/s].
    std::optional<double> leadFeed;       ///< Lead-in and lead-out feed rate [mm/s].
    std::optional<double> partOffEndFeed; ///< Feed per revolution over a part-off's end [mm].
    std::optional<double> leadFeedPerRev; ///< Feed per revolution of turning's lead moves [mm].
    std::optional<double> pullFeed;       ///< A bar pull's feed rate [mm/s].

    bool empty() const noexcept {
        return !surfaceSpeed && !feedPerTooth && !axialDepth && !radialDepth && !plungeFeed &&
               !rampFeed && !leadFeed && !partOffEndFeed && !leadFeedPerRev && !pullFeed;
    }
    /// The preset with these values applied.
    tooling::CuttingPreset appliedTo(const tooling::CuttingPreset& preset) const;
    bool operator==(const PresetOverrides&) const = default;
};

/// A tool orientation of an operation's own (positional, 3+2 machining within a setup): the
/// operation is generated in `frame`, given in the setup's WCS, whose Z axis is the tool axis
/// and whose X and Y are the operation's X and Y; its heights are measured along it. When
/// posted, the machine's rotary axes index to it before the operation (docs/design-log/0036).
struct ToolOrientation {
    Frame frame;
    /// 0: positions are relative to the setup's work offset, measured at the setup's WCS origin
    /// with the table at the setup's index, through the table centre (where the setup's
    /// placement puts the work relative to it,
    /// where the rotary axes meet). 1..: a work offset of its own (as Setup::workOffset),
    /// measured at the frame's origin with the rotary axes at this orientation.
    int workOffset = 0;

    bool operator==(const ToolOrientation&) const = default;
};

struct Operation {
    Uuid id = Uuid::generate();
    std::string name;
    std::string type; ///< Strategy id, e.g. "facing", "pocket2d", "contour2d", "drill".
    bool enabled = true;
    std::optional<Uuid> toolId;   ///< Tool in Project::tools.
    std::optional<Uuid> presetId; ///< Cutting preset of that tool.
    std::vector<ShapeRef> geometry;
    Json parameters = Json::object(); ///< Strategy parameters; schema owned by the strategy.
    OperationHeights heights;
    PresetOverrides presetOverrides;
    /// Where the toolpath should start (2D contour: the lead-in goes in at the loop point
    /// nearest to this vertex or vertical edge, seen from above).
    std::optional<ShapeRef> startPoint;
    /// Unset: the setup's WCS is the operation's frame (its Z the tool axis), as before.
    std::optional<ToolOrientation> orientation;
    std::string comment;

    bool operator==(const Operation&) const = default;
};

/// The frame an operation is generated in, in the setup's WCS (identity without an
/// orientation of its own).
Transform3 operationFrame(const Operation& op);

/// The allowance of the default clearance and retract heights: 0.2" [mm].
inline constexpr double kDefaultHeightAllowance = 5.08;

/// Whether an operation type cuts between a top and a bottom height (the hole operations take
/// their depths from the holes, turning and multi-axis operations from the part).
bool usesTopAndBottom(std::string_view type) noexcept;
/// Whether an operation type drills, taps, threads or bores holes.
bool isHoleOperation(std::string_view type) noexcept;

/// The heights an operation of `op.type` starts with: clearance 0.2" above the retract height,
/// retract 0.2" above the stock top, feed at the retract height (hole operations: 2 mm above
/// each hole), top at the stock top for roughing and the part top for finishing, bottom at the
/// part bottom (the selected geometry where the operation cuts down to it, the part top for
/// facing).
OperationHeights defaultHeights(const Operation& op);
/// The operation's heights with every Default setting replaced by its defaultHeights() value.
OperationHeights effectiveHeights(const Operation& op);

/// A face or edge picked to give a WCS axis (WcsOrientation).
struct AxisPick {
    std::string label;              ///< What was picked, as shown.
    std::optional<ShapeRef> shape;  ///< On the part or a fixture model (shown selected).
    Vec3 direction = Vec3::UnitZ(); ///< The direction it gave, project frame (unit).
    bool flip = false;              ///< The axis runs the other way.

    bool operator==(const AxisPick&) const = default;
};

enum class OrientationMode {
    Default, ///< The part's own axes (the project frame's).
    XZ,      ///< Z from a picked face or edge, X from another, square to it.
    YZ,      ///< Z from a picked face or edge, Y from another, square to it.
};

/// How the setup's WCS axes were chosen (the setup page's Part orientation). The axes are in
/// Setup::wcs; picks are worked out when made (a direction on the workholding depends on how the
/// WCS turns it, so it is not followed).
struct WcsOrientation {
    OrientationMode mode = OrientationMode::Default;
    std::optional<AxisPick> first; ///< X (XZ) or Y (YZ).
    std::optional<AxisPick> z;

    bool operator==(const WcsOrientation&) const = default;
};

/// What the setup's WCS origin is measured from.
enum class OriginFrom {
    StockBox,
    PartBox,
    FixtureBox, ///< The setup's fixture models and its workholding, together.
    Point,      ///< A picked circle's centre, face's centre or vertex.
};

/// Where the setup's WCS origin is (the setup page's Origin from): a point of the stock's, the
/// part's or the fixtures' box along the WCS axes, or a picked point of the part, a fixture
/// model or the workholding, moved by an offset along the WCS axes. Document::modify keeps
/// Setup::wcs.origin there after every change, so it follows the stock, the part and the
/// workholding.
struct WcsOrigin {
    OriginFrom from = OriginFrom::StockBox;
    std::array<int, 3> box{1, 1, 2}; ///< Boxes: per WCS axis 0 = min, 1 = middle, 2 = max.
    std::optional<ShapeRef> point;   ///< Point: on the part or a fixture model.
    /// Point: picked on the workholding (moves with its member).
    std::optional<WorkholdingDatum> workholdingPoint;
    std::string label;          ///< Point: what was picked, as shown.
    Vec3 offset = Vec3::Zero(); ///< From the reference, along the WCS axes [mm].

    bool operator==(const WcsOrigin&) const = default;
};

/// The post processor a setup is posted with (DL-0039): a built-in post or one the project
/// carries (Project::posts), by its manifest id.
struct PostChoice {
    std::string id;
    bool inProject = false; ///< One of Project::posts rather than a built-in post.

    bool operator==(const PostChoice&) const = default;
};

/// A post processor the project carries: a copy of a user's post file, so the project posts the
/// same anywhere (as Project::machines do for machines).
struct ProjectPost {
    std::string file;   ///< The name of the file it was copied from ("my-vmc.js").
    std::string source; ///< The JavaScript module.

    bool operator==(const ProjectPost&) const = default;
};

/// A point of a fitting's model with a direction there, picked on its B-rep or typed.
struct FittingPoint {
    Vec3 point = Vec3::Zero();
    Vec3 direction = Vec3::UnitZ();
    std::string on; ///< What it was picked on, as shown; empty = typed.

    bool operator==(const FittingPoint&) const = default;
};

/// What is fitted to a lathe for a job, from the user's library (DL-0068; LatheFitting.hpp). A STEP
/// model and how it goes on.
struct LatheFitting {
    enum class Kind {
        Block,  ///< A tooling block on a turret's station.
        Centre, ///< A centre in the tailstock's quill.
    };
    Uuid id = Uuid::generate();
    std::string name;
    std::string description;
    Kind kind = Kind::Block;
    std::string sha256; ///< Its STEP file, its exact geometry.
    std::uint32_t colour = 0x7C8690;
    /// Block: where it seats on the station, the direction out of the turret. Centre: where it sits
    /// in the quill, the direction towards the tip.
    FittingPoint seat;
    double seatTurn = 0.0; ///< Block: turned about the seat, when the points do not say [deg].
    /// Block: on the tool's centre line at its mouth, the direction towards the tool's tip, at the
    /// height of the tip. Centre: its tip.
    FittingPoint mount;
    double overhang = 30.0; ///< Block: how far the tip stands beyond `mount`, unless a setup says.
    /// Block: what it holds, a square shank (a turning tool's), a round one (a boring bar's, a
    /// drill's) or both.
    enum class Takes { Square, Round, Both };
    Takes takes = Takes::Square;
    /// Block: the sizes it says it takes [mm]; one it does not say is not compared. A square slot
    /// takes its own size; a bore is the largest round shank it takes, a smaller one going in
    /// through a collet or sleeve (a milling unit's, a boring holder's).
    std::optional<double> squareShank;
    std::optional<double> roundBar;
    /// How it goes on, by name: a block's kind of mount ("VDI 40", "BMT 55"), a centre's interface
    /// with the quill ("MT 4"). Matched against the machine's (machine::sameMountName); empty: it
    /// does not say.
    std::string mountType;

    bool takesSquare() const noexcept { return takes != Takes::Round; }
    bool takesRound() const noexcept { return takes != Takes::Square; }

    /// Problems as sentences (empty = usable).
    std::vector<std::string> validate() const;
    bool operator==(const LatheFitting&) const = default;
};

/// A lathe's tailstock centre on the spindle axis, in the setup's WCS (DL-0052).
struct LatheTailstock {
    double tipZ = 0.0; ///< Where its point is: in the part's centre hole.
    double centreDiameter = 20.0;
    double centreAngle = 60.0 * kPi / 180.0; ///< Included angle of the centre [rad].
    double quillDiameter = 45.0;
    double quillLength = 80.0;
    /// The model of the centre fitted, in Project::fittings; unset: a plain cone of the sizes
    /// above. Its tip goes at tipZ, and the quill starts where it seats.
    std::optional<Uuid> centre;

    bool operator==(const LatheTailstock&) const = default;
};

/// A lathe's steady rest around the work, in the setup's WCS (DL-0052).
struct LatheSteadyRest {
    double z = 0.0;            ///< The middle of its jaws.
    double width = 25.0;       ///< Along Z.
    double gripDiameter = 0.0; ///< What its jaws close on; 0 = the stock there.
    double outerDiameter = 150.0;

    bool operator==(const LatheSteadyRest&) const = default;
};

/// A tool in a lathe's turret beyond what the operations say (DL-0067): one that rides along
/// without being used, or how one sits there. A tool's station is its number.
struct TurretEntry {
    Uuid tool;
    std::optional<bool> across; ///< Across the spindle or along it; unset: by the kind of tool.
    /// Its tip from the turret's centre: (towards the spindle, along Z) [mm]; unset: where the
    /// machine's turret has such a tool.
    std::optional<Vec2> tip;
    /// How far its tip stands beyond the block's tool mount; unset: as the block says.
    std::optional<double> overhang;

    bool operator==(const TurretEntry&) const = default;
};

/// The tooling block on a turret's station, in Project::fittings (DL-0068).
struct StationBlock {
    int station = 1;
    Uuid block;

    bool operator==(const StationBlock&) const = default;
};

struct Setup {
    Uuid id = Uuid::generate();
    std::string name;
    std::optional<Uuid> modelId;
    Frame wcs; ///< Work coordinate system in the project frame.
    /// How the WCS axes were chosen; unset: as set (files of earlier versions).
    std::optional<WcsOrientation> orientation;
    /// Where the WCS origin is kept; unset: wcs.origin as set (files of earlier versions).
    std::optional<WcsOrigin> originFrom;
    int workOffset = 1; ///< 1 = G54 ... 6 = G59; higher values are extended offsets.
    Stock stock;
    /// Kept for files of earlier versions; operations now set their own heights.
    double clearanceHeight = 10.0; ///< Above WCS Z0 [mm].
    double retractHeight = 5.0;    ///< Above WCS Z0 [mm].
    /// Models (in Project::models, placed like the part) that hold the part in this setup: vises,
    /// clamps, plates. Milling toolpaths keep the tool and its holder clear of them.
    std::vector<Uuid> fixtures;
    double fixtureClearance = 1.0; ///< Closest approach of the tool and holder to them [mm].
    /// A vise from Project::workholdings placed around the stock; kept clear of like a fixture.
    std::optional<SetupWorkholding> workholding;
    std::optional<LatheTailstock> tailstock;   ///< Lathes: kept clear of like a fixture.
    std::optional<LatheSteadyRest> steadyRest; ///< Lathes: kept clear of like a fixture.
    std::vector<TurretEntry> turret;           ///< Lathes with a layout: see TurretEntry.
    /// Lathes with a layout: the blocks on its stations. Others hold their tools as the machine's
    /// turret has it.
    std::vector<StationBlock> turretBlocks;
    std::string machine; ///< Machine id: a project machine or a built-in (empty = 3-axis VMC).
    /// Where the work sits on the machine; unset = defaultPlacement (PostSetup.hpp).
    std::optional<SetupPlacement> placement;
    /// The post it is posted with; unset: the machine's (document::postFor).
    std::optional<PostChoice> post;
    /// Values of the post's options: {"program": {id: value}, "operations": {operation id:
    /// {id: value}}}. Options left out take the post's defaults, which the editors show.
    Json postOptions = Json::object();
    std::vector<Operation> operations;

    bool operator==(const Setup&) const = default;
};

struct Project {
    Uuid id = Uuid::generate();
    std::string name = "Untitled";
    LengthUnit displayUnit = LengthUnit::Millimeter;
    std::string notes;
    std::vector<ModelEntry> models;
    tooling::ToolLibrary tools;
    /// Custom machines (copies from the user's machine library), so the project opens anywhere.
    std::vector<machine::Machine> machines;
    /// Workholding templates the setups use (copies from the user's library).
    std::vector<WorkholdingTemplate> workholdings;
    /// Tooling blocks and tailstock centres the setups use (copies from the user's library).
    std::vector<LatheFitting> fittings;
    /// Post processors the setups use (copies of the user's post files, DL-0039).
    std::vector<ProjectPost> posts;
    std::vector<Setup> setups;

    const ModelEntry* findModel(const Uuid& id) const;
    const Setup* findSetup(const Uuid& id) const;
    /// A project machine by id, or nullptr (built-ins are looked up with machine::builtinMachine).
    const machine::Machine* findMachine(std::string_view machineId) const;
    const WorkholdingTemplate* findWorkholding(const Uuid& id) const;
    const LatheFitting* findFitting(const Uuid& id) const;
    Setup* findSetup(const Uuid& id);
    /// Finds an operation in any setup; also reports its setup.
    const Operation* findOperation(const Uuid& id, const Setup** setup = nullptr) const;
    Operation* findOperation(const Uuid& id, Setup** setup = nullptr);
    /// Moves an operation in front of position `index` of a setup's operations (counted before
    /// the move; past the end appends), possibly to another setup. False when the operation
    /// or the setup does not exist, or the order would not change.
    bool moveOperation(const Uuid& operationId, const Uuid& setupId, std::size_t index);

    /// Consistency problems as human-readable sentences (empty = consistent): duplicate ids,
    /// references to missing models, setups, tools or presets, invalid stock, FromSetup chains
    /// that are cyclic or refer to a later setup, plus tool library issues.
    std::vector<std::string> validate() const;

    bool operator==(const Project&) const = default;
};

/// Whether a setup holds the part with fixtures or a workholding (toolpaths keep clear of them).
bool hasFixtures(const Setup& setup) noexcept;

/// Human-readable work offset, e.g. 1 -> "G54", 7 -> "G54.1 P1".
std::string workOffsetName(int workOffset);

/// The first of the bar pulls that end a setup's enabled operations (DL-0091), if it ends with
/// any: the part is as it comes off before it, and after it the bar is the next cycle's.
std::optional<Uuid> trailingBarPull(const Setup& setup);

} // namespace ocam::document
