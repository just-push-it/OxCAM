// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Cutting tools, holders and cutting-data presets. All values use internal units: mm, rad,
// seconds (so surface speed is mm/s and feed rates are mm/s). File formats convert at the
// boundary; see LibraryIo.hpp.

#pragma once

#include <ocam/foundation/Math.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/foundation/Uuid.hpp>
#include <ocam/geom2d/Polyline.hpp>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ocam::tooling {

enum class ToolType {
    // Milling and drilling (rotating tools)
    FlatEndMill,
    BallEndMill,
    BullNoseEndMill,
    Chamfer,
    Engraver,
    Drill,
    SpotDrill,
    CenterDrill,
    Reamer,
    BoringBar,
    Tap,
    ThreadMill,
    SlotCutter,
    Lollipop,
    Form,
    /// A spindle probe: a touch-trigger probe in the spindle, which measures and does not cut.
    Probe,
    // Turning (stationary tools)
    TurningInsert,
    GroovingInsert,
    ThreadingInsert,
    PartingInsert,
    // Bar work (DL-0091): no cutting
    BarPuller,
};

/// Stable identifier used in files, e.g. "flat_end_mill".
std::string_view toString(ToolType type) noexcept;
std::optional<ToolType> parseToolType(std::string_view text) noexcept;
/// Human-readable name, e.g. "Flat end mill".
std::string_view displayName(ToolType type) noexcept;
bool isTurningTool(ToolType type) noexcept;
const std::vector<ToolType>& allToolTypes();

/// Groups of tool types, for filtering tool lists: milling cutters, hole-making tools (drills,
/// reamers, taps, boring bars), turning inserts and probes.
enum class ToolFamily { Milling, HoleMaking, Turning, Probing };
ToolFamily toolFamily(ToolType type) noexcept;
/// The types of `family`, in allToolTypes() order.
std::vector<ToolType> toolTypesOf(ToolFamily family);

/// What a holder holds: rotating tools in the milling spindle (collet, shrink-fit, end mill and
/// drill chucks, tapping holders), or turning tools on the lathe.
enum class HolderKind { Milling, Turning };
/// Stable identifier used in files: "milling" or "turning".
std::string_view toString(HolderKind kind) noexcept;
std::optional<HolderKind> parseHolderKind(std::string_view text) noexcept;
/// The kind of holder a tool of `type` sits in.
HolderKind holderKindFor(ToolType type) noexcept;

enum class Coolant { Off, Flood, Mist, Air, ThroughTool };
std::string_view toString(Coolant c) noexcept;
std::optional<Coolant> parseCoolant(std::string_view text) noexcept;

/// Cutting data for one tool in one workpiece material.
struct CuttingPreset {
    Uuid id = Uuid::generate();
    std::string name;
    std::string materialGroup; ///< e.g. ISO 513 group "P", "M", "K", "N", "S", "H".
    double surfaceSpeed = 0.0; ///< vc [mm/s]
    double feedPerTooth = 0.0; ///< fz [mm]
    double axialDepth = 0.0;   ///< ap [mm]; 0 = unspecified
    double radialDepth = 0.0;  ///< ae [mm]; 0 = unspecified
    double rampAngle = degToRad(2.0);
    double plungeFeed = 0.0;     ///< Feed rate of plunges into material [mm/s].
    double rampFeed = 0.0;       ///< Feed rate of ramps and helical entries into material [mm/s].
    double leadFeed = 0.0;       ///< Feed rate of lead-in and lead-out moves [mm/s].
    double partOffEndFeed = 0.0; ///< Feed per revolution over the end of a part-off [mm].
    double leadFeedPerRev = 0.0; ///< Feed per revolution of turning's lead moves [mm].
    double pullFeed = 0.0;       ///< Bar pullers: feed rate of the pull, the chuck open [mm/s].
    Coolant coolant = Coolant::Flood;

    /// Spindle speed n = vc / (pi D) [rev/s]. 0 for a non-positive diameter.
    double spindleSpeed(double toolDiameter) const noexcept;
    /// Feed rate vf = fz * z * n [mm/s].
    double feedRate(double toolDiameter, int flutes) const noexcept;

    bool operator==(const CuttingPreset&) const = default;
};

/// Geometry of rotating tools. Field meaning per type:
///  - diameter: largest cutting diameter.
///  - cornerRadius: bull-nose radius (BullNoseEndMill).
///  - fluteLength: cutting length; for SlotCutter the cutting width.
///  - tipAngle: included point angle (Drill, SpotDrill, CenterDrill, Chamfer, Engraver). A
///    BoringBar with a neck: the included angle of the cone that narrows its head's top to the
///    neck (a back-chamfer cutter, DL-0083); 0 = a flat top (a back-boring bar or back-spotface
///    cutter).
///  - tipDiameter: flat at the tip (Chamfer, Engraver) or pilot diameter (CenterDrill).
///  - tipLength: pilot length (CenterDrill).
///  - threadPitch: Tap, ThreadMill.
///  - neckDiameter / neckLength: relieved neck above the head, measured from the tip
///    (SlotCutter, Lollipop; optional for end mills and boring bars). 0 = no neck.
///  - formProfile: Form tools only; (radius, height) points from the tip, starting on the axis.
///  - Probe: diameter is the stylus ball's, fluteLength the stylus length (from the ball's tip to
///    the probe body), neckDiameter the stylus stem's, shankDiameter the probe body's, and
///    triggerOffset the effective trigger offset.
///  - BarPuller (DL-0091): diameter is its body's, tipDiameter the largest bar it grips,
///    fluteLength how deep the bar goes into its jaws (its grip), from its front.
struct MillGeometry {
    double diameter = 0.0;
    double cornerRadius = 0.0;
    double fluteLength = 0.0;
    double overallLength = 0.0;
    double shankDiameter = 0.0; ///< 0 = same as diameter.
    int flutes = 2;
    double tipAngle = 0.0;
    double tipDiameter = 0.0;
    double tipLength = 0.0;
    double threadPitch = 0.0;
    bool leftHand = false; ///< Tap: cuts a left-hand thread, turning the other way.
    double neckDiameter = 0.0;
    double neckLength = 0.0;
    std::vector<Vec2> formProfile;
    /// Probe: how far the stylus ball travels past first contact before the probe triggers, as
    /// its calibration finds it [mm]. The ball's effective radius is its radius less this.
    double triggerOffset = 0.0;

    bool operator==(const MillGeometry&) const = default;
};

/// A probe's effective stylus radius: the ball's radius less the trigger offset. Measured surfaces
/// are this far from where the stylus centre stops.
inline double effectiveStylusRadius(const MillGeometry& probe) {
    return probe.diameter / 2.0 - probe.triggerOffset;
}

/// A tap's core, which has to find no material: what is left of it inside its tap drill's hole.
inline double tapCoreRadius(const MillGeometry& tap) {
    return tap.diameter / 2.0 - 0.75 * tap.threadPitch;
}

/// How far from a tap's tip a blind hole's drill point may still be in its way, for a core of
/// that radius: a 118° point's height. A tap is chamfered there, and a drill to the same depth
/// leaves it no less than this.
inline double tapPointAllowance(double coreRadius) {
    return 0.6 * coreRadius;
}

/// Turning insert description (ISO 1832-style fields). Profiles for turning arrive in phase 8.
/// How a turning insert is held, by the hole it has. NotSet (older libraries) fits any clamping.
enum class InsertLocking { NotSet, CylindricalHole, NoHole, CountersunkHole };
std::string_view toString(InsertLocking locking) noexcept;
std::optional<InsertLocking> parseInsertLocking(std::string_view text) noexcept;

struct TurningInsert {
    std::string shape = "C"; ///< ISO shape letter: C, D, R, S, T, V, W, ...
    double inscribedCircle = 0.0;
    double noseRadius = 0.0;
    double thickness = 0.0;
    double reliefAngle = 0.0;
    std::string hand = "N";   ///< "L", "R" or "N"
    int orientation = 3;      ///< Tip quadrant 1-9 (nose-radius compensation convention).
    double width = 0.0;       ///< Grooving and parting width.
    double threadPitch = 0.0; ///< Threading inserts.
    /// Turning inserts: the leading edge's angle to a feed towards -Z [rad], which places the
    /// insert (an ISO holder gives it, fitToHolder). 0 = as on a 93-95 degree holder.
    double cuttingEdgeAngle = 0.0;
    InsertLocking locking = InsertLocking::NotSet; ///< Turning inserts: the holders that take it.

    bool operator==(const TurningInsert&) const = default;
};

/// A solid turning tool: one piece of carbide with no insert or holder, such as a micro boring
/// bar, sold by these dimensions. Its tip is described by the tool's `insert` (fitSolidBar).
struct SolidBar {
    double minBore = 0.0;       ///< Smallest bore it machines (ØD).
    double shankDiameter = 0.0; ///< ØDMM.
    double headLength = 0.0;    ///< Reduced head, from the tip to the shank (LH).
    double overallLength = 0.0; ///< OAL.
    double maxOverhang = 0.0;   ///< Deepest it may reach out of its sleeve (OHX); 0 = OAL.
    double centreToTip = 0.0;   ///< From the shank's axis to the tip (S).
    double maxDepth = 0.0;      ///< Largest depth of cut (T-MAX); 0 = not stated.

    bool operator==(const SolidBar&) const = default;
};

struct Tool {
    Uuid id = Uuid::generate();
    int number = 1; ///< Pocket / T number.
    std::string name;
    std::string comment;
    ToolType type = ToolType::FlatEndMill;
    std::string material = "carbide";
    std::string coating;
    std::string vendor;
    std::string partNumber;
    MillGeometry mill;
    TurningInsert insert;
    std::optional<SolidBar> solid; ///< Turning tools made in one piece: no holder.
    std::optional<Uuid> holderId;
    /// Rotating tools: from the holder's face to the tool tip [mm]; 0 = the overall length.
    /// With the holder it gives the assembly's gauge length (gaugeLength()).
    double stickout = 0.0;
    /// Unit the tool is described in (display and entry only; values are stored in mm).
    LengthUnit unit = LengthUnit::Millimeter;
    std::vector<CuttingPreset> presets;

    /// Problems that make the tool unusable, as human-readable sentences (empty = valid).
    std::vector<std::string> validate() const;

    const CuttingPreset* findPreset(const Uuid& presetId) const;

    bool operator==(const Tool&) const = default;
};

/// The designation of an ISO turning holder for indexable inserts: a rectangular shank (ISO 5608)
/// or a boring bar (ISO 5609-1). See TurningHolder.hpp.
struct TurningHolderCode {
    bool boringBar = false;
    char barType = 'S';  ///< Boring bars: what the shank is made of.
    char clamping = 'P'; ///< How the insert is held.
    char insertShape = 'C';
    char style = 'L';     ///< Tool style: the cutting edge angle and the shank's offset.
    char clearance = 'N'; ///< Insert normal clearance.
    char hand = 'R';      ///< 'R', 'L' or 'N'.
    double height = 25.0; ///< Shank height, or the boring bar's diameter [mm].
    double width = 25.0;  ///< Shank width; boring bars: the diameter again [mm].
    char length = 'M';    ///< Tool length symbol.
    int insertSize = 12;  ///< Insert size symbol: a side's length in whole millimetres.
    double f = 32.0; ///< From the tip to the shank's back, or to the bar's axis [mm]. The holder's
                     ///< value: the designation does not give it (ISO 5610, ISO 6261).
    /// Values outside the standard's lists, with the letter that says so: style 'X' (the cutting
    /// edge's angle to a feed towards -Z [deg]), clearance 'O' [deg], length 'X' [mm].
    std::optional<double> customStyle;
    std::optional<double> customClearance;
    std::optional<double> customLength;

    bool operator==(const TurningHolderCode&) const = default;
};

/// A plain turning holder with no designation, for grooving, threading and parting tools (their
/// holders are the makers' own) or any insert: a rectangular shank, or a round bar.
struct TurningStick {
    bool bar = false;      ///< A bar along +Z, for inside cuts; else a shank along +X.
    double width = 20.0;   ///< Shank width, or the bar's diameter [mm].
    double f = 25.0;       ///< From the tip to the shank's back, or to the bar's axis [mm].
    double length = 125.0; ///< From the tip along the holder [mm].

    bool operator==(const TurningStick&) const = default;
};

/// Tool holder as a revolved outline in the (radius, height) half-plane. Height 0 is the face
/// that the tool protrudes from; heights increase towards the spindle, and the top of the outline
/// is the gauge line (the spindle face): the taper inside the spindle is not part of it.
struct Holder {
    Uuid id = Uuid::generate();
    HolderKind kind = HolderKind::Milling;
    std::string name;
    std::string comment;
    std::string vendor;
    std::string partNumber;
    geom2d::Polyline profile; ///< Open polyline from the axis at height 0 to the axis at the top.
    /// ISO turning holders: the designation, which the profile (the shank's outline) follows.
    std::optional<TurningHolderCode> iso;
    /// Plain turning holders: their shape, which the profile follows.
    std::optional<TurningStick> stick;

    std::vector<std::string> validate() const;
    /// Largest radius of the outline (0 when empty).
    double maxRadius() const;
    /// Height of the outline: from the tool face to the gauge line (0 when empty).
    double length() const;
    bool operator==(const Holder&) const = default;
};

/// True when a tool of `type` sits in holders of `holder`'s kind. ISO holders take turning inserts
/// only: their designation describes an indexable insert's seat.
bool holds(const Holder& holder, ToolType type) noexcept;

/// Length of a rotating tool out of its holder: the stickout, or the overall length when the
/// stickout is 0.
double visibleLength(const Tool& tool) noexcept;
/// From the spindle gauge line to the tool tip: the visible length plus the holder's height
/// (from its tool face to its gauge line). The machine's tool length (G43), the simulation's
/// tool assembly and the post's tool table all use it.
double gaugeLength(const Tool& tool, const Holder* holder) noexcept;
/// The stickout that gives the assembly gauge length `gauge` in `holder` (or without one). Not
/// positive when the holder alone is that long.
double stickoutForGauge(double gauge, const Holder* holder) noexcept;

/// Dimensions of a stepped holder: a nose (optionally tapered, like shrink-fit and collet
/// chucks), a body and the flange up to the gauge line. A step with a zero diameter or length is
/// left out.
struct HolderDimensions {
    double noseDiameter = 32.0;   ///< At the tool face.
    double noseTopDiameter = 0.0; ///< At the top of the nose; 0 = the same as noseDiameter.
    double noseLength = 40.0;
    double bodyDiameter = 44.0;
    double bodyLength = 20.0;
    double flangeDiameter = 63.0;
    double flangeLength = 16.0; ///< Up to the gauge line.
};

/// A holder with the outline of `dimensions`. Throws Error(InvalidArgument) when no step has a
/// positive diameter and length.
Holder makeHolder(std::string name, const HolderDimensions& dimensions);

/// Convenience constructors with sensible defaults (used by tests, samples and the GUI's
/// "new tool" action).
Tool makeFlatEndMill(int number, double diameter, double fluteLength, double overallLength);
Tool makeBallEndMill(int number, double diameter, double fluteLength, double overallLength);
Tool makeBullNoseEndMill(int number, double diameter, double cornerRadius, double fluteLength,
                         double overallLength);
Tool makeDrill(int number, double diameter, double tipAngle, double fluteLength,
               double overallLength);
Tool makeChamferMill(int number, double diameter, double tipDiameter, double includedAngle,
                     double overallLength);
/// A tool of `type` with typical proportions and one cutting preset, defaultPreset(); a probe
/// (which does not cut) has no preset, and `diameter` is its stylus ball's.
Tool makeDefaultTool(ToolType type, int number, double diameter = 10.0);

/// A generic starting preset for a new tool (named "Default"): typical carbide values for the
/// tool type and size, the plunge feed half the cutting feed (all of it for drills and other
/// hole tools), the ramp and lead feeds the cutting feed. Toolpaths only ever use presets, so a
/// new tool is
/// usable at once; check the values against the tool vendor's data.
CuttingPreset defaultPreset(const Tool& tool);

} // namespace ocam::tooling
