// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Workholding templates (DL-0035): fixtures made of STEP parts on a fixed base or on moving
// members (jaws, clamps), datums picked on them or defined from each other, and stock holdings
// that say where the stock sits (between jaws, on a dovetail, on a seat). A setup chooses a
// template and a holding; the members close on the stock and the template is placed around it.
//
// A template file (.ocamhold) is a zip archive:
//   workholding.json          the WorkholdingTemplate (format "oxcam.workholding")
//   bodies/<sha256>.step      the parts' STEP files, named by content hash
// Other entries are ignored when reading. Projects embed the templates their setups use
// (Project::workholdings) and the part files among their models.

#pragma once

#include "ocam/document/MachineImport.hpp"
#include "ocam/document/Project.hpp"

#include <ocam/cad/Model.hpp>
#include <ocam/geom3d/Bounds.hpp>

#include <array>
#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ocam::document {

class Document;

/// 2: parts on members, datums, stock holdings (version 1 had bodies with jaw roles and one
/// datum frame; it migrates to a template that holds the stock the same way). 3: chucks and
/// collets (HoldingKind::Chuck). 4: jaw mounts on chucks and jaw sets (DL-0069).
inline constexpr int kWorkholdingVersion = 4;
const FormatSpec& workholdingFormat();
const SchemaValidator& workholdingSchema();

Json toJson(const WorkholdingTemplate& workholding);
/// Migrates, validates against the schema and converts. Throws ocam::Error (Parse /
/// Unsupported).
WorkholdingTemplate workholdingFromJson(Json doc, std::string_view sourceName);

/// A template with its part files: what a template file holds.
struct WorkholdingPackage {
    WorkholdingTemplate workholding;
    std::map<std::string, std::string> files; ///< sha256 -> STEP file bytes
};

/// Adds a part made of `model` (written as a STEP file of its own, kept in `package.files`),
/// carried by the base, and returns it. Datums are picked on the model read back from that file
/// (Document::modelData or cad::Model::importStepData of its bytes): its sub-shape indices are the
/// ones stored. Throws Error(Io) when the model cannot be written.
WorkholdingPart addWorkholdingPart(WorkholdingPackage& package, std::string name,
                                   const cad::Model& model, std::string sourcePath = {});

/// A part kept as solids, from a CAD import (MachineImport.hpp) grouped by the user.
struct SolidPart {
    std::string name;
    cad::Model model;
    std::optional<std::uint32_t> colour;
};

/// Merges the STEP parts of each group into one solid part named after the group, in the order
/// groups first appear, with the colour of its first part that has one. Parts whose group is
/// empty are left out. Throws Error(InvalidArgument) for STL parts: workholding parts are STEP
/// solids, picked on their exact geometry.
std::vector<SolidPart> solidPartsFrom(const std::vector<ImportedPart>& parts,
                                      const std::vector<std::string>& groups);

/// A datum picked on a part's face, edge or vertex (`model`: the part's model, read from its
/// file): a flat face gives its centre and outward normal, a round face or circular edge its
/// axis, a straight edge its middle and direction, a vertex its point. With `onFace`, the point
/// of the face nearest it and the normal there. Throws Error(InvalidArgument) for a bad index.
WorkholdingDatum pickedDatum(std::string name, const WorkholdingPart& part, const cad::Model& model,
                             cad::SubShapeType type, int index,
                             const std::optional<Vec3>& onFace = std::nullopt);

/// The archive bytes of a template file. Throws Error(InvalidArgument) when a part's file is
/// missing.
std::string writeWorkholdingArchive(const WorkholdingPackage& package);
/// Reads a template file's bytes; checks each part file against its hash. Throws Error(Parse).
WorkholdingPackage readWorkholdingArchive(std::string_view bytes, std::string_view sourceName);
/// Writes a template file atomically. Throws like writeWorkholdingArchive and Error(Io).
void saveWorkholdingFile(const std::filesystem::path& path, const WorkholdingPackage& package);
WorkholdingPackage loadWorkholdingFile(const std::filesystem::path& path);

/// A datum where its members have moved: template coordinates.
struct DatumValue {
    Vec3 point = Vec3::Zero();
    Vec3 direction = Vec3::Zero(); ///< Unit or zero.
};

/// Every member's travel, from the travel of those that move on their own (missing: 0); linked
/// members follow theirs.
std::map<std::string, double> memberTravels(const WorkholdingTemplate& workholding,
                                            const std::map<std::string, double>& own);

/// Where datum `name` is with the members at `travel` (memberTravels). Throws
/// Error(InvalidArgument) for unknown datums, cycles or an offset along a missing direction.
DatumValue datumAt(const WorkholdingTemplate& workholding, std::string_view name,
                   const std::map<std::string, double>& travel);

/// A jaw holding's opening (between its faces) or a dovetail's (between its flanks at the
/// seat) with the members at `travel`. Throws Error(InvalidArgument) for seats.
double holdingOpening(const WorkholdingTemplate& workholding, const StockHolding& holding,
                      const std::map<std::string, double>& travel);

/// Every member's travel for the holding at `opening`: its driving member (the one its face B
/// moves with) and the members linked to it set, the others at `own`. Throws
/// Error(InvalidArgument) when no member moves the faces and the opening differs. Ranges are not
/// checked (poseWorkholding does).
std::map<std::string, double> travelsForOpening(const WorkholdingTemplate& workholding,
                                                const StockHolding& holding, double opening,
                                                const std::map<std::string, double>& own = {});

/// The smallest and largest opening the holding's driving member and those linked to it allow
/// (each within its travel range); nullopt when no member moves its faces.
std::optional<std::pair<double, double>>
openingRange(const WorkholdingTemplate& workholding, const StockHolding& holding,
             const std::map<std::string, double>& own = {});

/// Sets the travel ranges of the holding's driving member and those linked to it so the holding
/// opens from `smallest` to `largest`. Throws Error(InvalidArgument) when no member moves its
/// faces.
void setOpeningRange(WorkholdingTemplate& workholding, const StockHolding& holding, double smallest,
                     double largest);

/// What boring soft jaws to fit took off a jaw, in the jaw part's own coordinates (DL-0069):
/// about the line through `axisPoint` along `axisDirection` (the chuck's axis while it was
/// bored), from `from` along it onwards, everything nearer the line than `radius` (a bore, for an
/// outside grip) or farther from it (a step on their outside, for an inside grip).
struct JawCut {
    Vec3 axisPoint = Vec3::Zero();
    Vec3 axisDirection = Vec3::UnitZ();
    double radius = 0.0;
    double from = 0.0;
    bool bore = true;

    bool operator==(const JawCut&) const = default;
};

/// A part of a placed workholding.
struct PosedPart {
    Uuid id;
    std::string name;
    std::string sha256;
    std::string member;
    std::uint32_t colour = 0;
    bool collides = true;
    bool shown = true;
    Transform3 toWcs = Transform3::Identity(); ///< Part (template) coordinates -> WCS.
    std::optional<JawCut> cut;                 ///< A soft jaw, bored to fit.
    /// Its member stopped at the end of its travel, short of the stock
    /// (WorkholdingPose::outOfReach): drawn in an error colour.
    bool outOfReach = false;
};

/// Stock beyond a workholding's reach: what it would hold and what it can, in one measure (a
/// diameter gripped, clamped or held by a collet, or a width the jaws or flanks open to) [mm].
struct WorkholdingReach {
    enum class Kind {
        Grip,   ///< Jaws on a diameter (a chuck's, or a jaw set's step).
        Clamp,  ///< Soft jaws on the ring or plug they clamp while bored.
        Collet, ///< A collet's range of bars.
        Open,   ///< A vise's jaws on a width.
        Tenon,  ///< A dovetail's flanks on a tenon.
    };
    Kind kind = Kind::Grip;
    double wanted = 0.0;
    double from = 0.0;
    double to = 0.0;
    std::string step; ///< A jaw set's step; empty for the rest.
    bool operator==(const WorkholdingReach&) const = default;
};

/// What a setup page says of it, in `unit`: "the jaws can't reach Ø3.163 in: on step 'Step 1'
/// they grip Ø0.996–1.343 in". Names no members or travel.
std::string reachMessage(const WorkholdingReach& reach, LengthUnit unit);

/// Where a jaw set went on a chuck's master jaws, and why (DL-0069): what the setup page shows.
struct JawPlacement {
    std::string jawSet; ///< Its name.
    std::string option; ///< The option it goes on by.
    std::string step;   ///< The step that grips (soft jaws: the one that clamped the ring).
    bool outside = true;
    double travel = 0.0;      ///< Master jaw 1's [mm].
    double radiusPerMm = 1.0; ///< How far a jaw moves out per mm of master jaw travel.
    double stopZ = 0.0;       ///< The stop face (a soft jaw's bore floor), WCS Z.
    double minTravel = 0.0;
    double maxTravel = 0.0;
    double gripDiameter = 0.0; ///< What the jaws hold [mm].
    /// Soft jaws: what they are bored to and how deep, and the master jaws' travel then.
    std::optional<double> boreDiameter;
    double boreDepth = 0.0;
    double clampDiameter = 0.0;
    double boringTravel = 0.0;
};

struct WorkholdingPose {
    std::string holding;     ///< The stock holding used.
    double opening = 0.0;    ///< Jaws and dovetails: the opening the members closed to [mm].
    double stockWidth = 0.0; ///< The stock's size along the holding's X axis [mm].
    std::map<std::string, double> travel;              ///< Per member [mm].
    Transform3 templateToWcs = Transform3::Identity(); ///< Template coordinates -> WCS.
    std::vector<PosedPart> parts;
    Vec3 mount = Vec3::Zero(); ///< The mount datum (or the template origin), WCS.
    std::vector<std::string> warnings;
    std::optional<JawPlacement> jaws; ///< A jaw set on a chuck's master jaws.
    /// The stock beyond its reach: the members stand at the end of their travel nearest it, and
    /// the parts they carry are marked (poseWorkholding with `asFarAsItGoes`).
    std::optional<WorkholdingReach> outOfReach;
    /// Why the jaws cannot hold the stock at all this way (a jaw set with no step that grips the
    /// way asked): they are drawn where they stand, marked (with `asFarAsItGoes`; else thrown).
    std::string refusal;
};

/// Places a template around a stock with these bounds (WCS): the holding's members close on the
/// stock (or to the typed opening), the stock sits where the holding puts it, and the template
/// follows. Throws Error(InvalidArgument) with a user-facing message when a member would travel
/// outside its range, the holding's datums do not make sense (jaw faces with no gap between
/// them, flanks along the jaws) or there is no stock.
/// With a chuck taking a jaw set (`placement.jawSet`, `jawSet` its template, DL-0069): the jaws go
/// on the master jaws as the placement says, gripping the stock's width or, for an inside grip,
/// `insideDiameter`; the stock's end against the step's stop face. The jaw set is copied onto each
/// master jaw, and the chuck's own top jaws are left out.
/// Stock beyond the members' reach is refused (reachMessage, in mm), or with `asFarAsItGoes`
/// posed with the members at the end of their travel and said in `outOfReach`.
WorkholdingPose poseWorkholding(const WorkholdingTemplate& workholding,
                                const SetupWorkholding& placement, const geom3d::Aabb3& stockWcs,
                                const WorkholdingTemplate* jawSet = nullptr,
                                std::optional<double> insideDiameter = std::nullopt,
                                bool asFarAsItGoes = false);

/// The model of a placed part: the project's, with soft jaws' cut taken off (made once and kept).
/// Nullptr when the part's file is not loaded.
const cad::Model* posedPartModel(const Document& doc, const PosedPart& part);

/// A jaw set's interface fits the chuck's jaw mount (names compared as mount names are).
bool jawSetFits(const WorkholdingTemplate& chuck, const WorkholdingTemplate& jawSet);

/// The setup's workholding placed around its stock; nullopt without one. Throws
/// Error(InvalidArgument) when it cannot be placed (missing template, stock that cannot be
/// resolved, stock beyond its reach: reachMessage in the project's display unit).
std::optional<WorkholdingPose> setupWorkholdingPose(const Document& doc, const Setup& setup);
/// The same for showing it: stock beyond its reach is posed as far as it goes, and said in
/// `outOfReach`, so the views draw what does not reach. Other problems still throw.
std::optional<WorkholdingPose> shownWorkholdingPose(const Document& doc, const Setup& setup);

/// Where a datum of the setup's workholding is (not one of the template's: one picked on its
/// parts, or offset from one of its datums), project frame, with the workholding placed around
/// the stock. Throws Error(InvalidArgument) when the setup has no workholding, it cannot be
/// placed or the datum's part or datum is not in it.
DatumValue workholdingPoint(const Document& doc, const Setup& setup, const WorkholdingDatum& datum);

Json toJson(const WorkholdingDatum& datum);
/// Throws Error(Parse) for an unknown kind, shape or axis.
WorkholdingDatum datumFromJson(const Json& j);

/// Drops the templates no setup uses.
void removeUnusedWorkholdings(Project& project);

/// A vise template from its parts and their bounding boxes (template coordinates): the fixed jaw
/// on the base and a moving jaw travelling along `axis` (a self-centring vise: a moving jaw and
/// an opposite jaw linked to it), datums on the jaw faces and tops, one jaw holding, the mount
/// at the bottom centre of all the parts. Used for samples, tests and as a start in the editor;
/// nullopt (with the reason in `why`) when the jaws are missing or the moving jaw is not on the
/// `axis` side.
struct ViseParts {
    std::vector<std::pair<WorkholdingPart, geom3d::Aabb3>> parts;
    Uuid fixedJaw; ///< Self-centring: the opposite jaw.
    Uuid movingJaw;
    bool selfCentring = false;
};
std::optional<WorkholdingTemplate> viseTemplate(const std::string& name, const ViseParts& vise,
                                                const Vec3& axis, const Vec3& up, double maxOpening,
                                                std::string* why = nullptr);

// -- Setting up common workholding by its type (the editor's setup wizard) ---------------------

/// The common kinds of workholding the setup wizard knows. Others are set up by hand (members,
/// datums and holdings).
enum class FixtureType {
    Vise,             ///< A fixed jaw and a moving jaw.
    SelfCentringVise, ///< Two jaws moving apart symmetrically.
    Dovetail,         ///< Flanks holding a tenon; one may move.
    Seat,             ///< A face the stock sits on (a plate, a custom fixture).
    Chuck,            ///< Jaws (3, 4, 6 or any number) moving together on a bar's diameter.
    Collet,           ///< A bore gripping bars a little smaller than itself.
};

/// A face, edge or vertex clicked on a part (or, with `onFace`, a point clicked on a face).
struct WorkholdingPick {
    Uuid part;
    cad::SubShapeType type = cad::SubShapeType::Face;
    int index = 0;
    std::optional<Vec3> onFace;
};

/// What the setup wizard asks.
struct WorkholdingAnswers {
    FixtureType type = FixtureType::Vise;
    std::string name;
    std::string description;
    /// Vise: the moving jaw's parts. Self-centring: the second jaw's. Dovetail: the moving
    /// flank's (none: both flanks fixed). Chuck: every jaw's parts (parts at the same angle about
    /// the axis move as one jaw).
    std::vector<Uuid> moving;
    std::vector<Uuid> opposite; ///< Self-centring: the first jaw's parts.
    /// Vise: the fixed jaw's clamping face; self-centring: the first jaw's; dovetail: the fixed
    /// flank. The stock's side rests on it. Chuck: a jaw's gripping face; collet: its bore.
    std::optional<WorkholdingPick> faceA;
    /// The moving (or second) jaw's clamping face; dovetail: the other flank.
    std::optional<WorkholdingPick> faceB;
    /// Vises: the jaw tops; dovetail: the seat the stock's shoulder sits on; seat: the face the
    /// stock sits on. Chuck and collet: the front of the jaws or collet (or the step the bar's end
    /// sits against).
    std::optional<WorkholdingPick> top;
    std::optional<WorkholdingPick> axis;  ///< Chuck and collet: a round face on the spindle axis.
    std::optional<WorkholdingPick> xEdge; ///< Seat: the stock's X along this edge (else X).
    std::optional<WorkholdingPick> mount; ///< What it stands on; unset = its bottom centre.
    double minOpening = 0.0; ///< Vises: the jaws' range; dovetail: the tenon's; chuck: diameters.
    double maxOpening = 150.0;
    double gripDepth = 5.0;             ///< Vises; chuck and collet: the grip length.
    double gripRange = 0.5;             ///< Collet: how much smaller than its bore it grips.
    double tenonDepth = 5.0;            ///< Dovetail.
    std::array<int, 3> anchor{1, 1, 0}; ///< Seat: the stock's point on it (StockHolding::anchor).
};

/// What the answers still lack, as prompts for the user (empty = complete).
std::vector<std::string> missingAnswers(const WorkholdingAnswers& answers);

/// The parts' models (read from their files), for picking datums.
using PartModelFn = std::function<const cad::Model*(const WorkholdingPart&)>;

/// Removes a part (a body the fixture does not need: an alternative jaw set, a riser), its file
/// when no other part uses it, and the datums picked on it. Throws Error(InvalidArgument) for an
/// unknown part.
void removeWorkholdingPart(WorkholdingPackage& package, const Uuid& part);

/// Merges parts into one part named `name` with all their solids, in the first one's place, on
/// its member and with its colour, and returns it. The merged parts and the datums picked on them
/// are removed (their sub-shape numbers change). Throws Error(InvalidArgument) for fewer than two
/// parts, an unknown part or one whose model is not loaded, Error(Io) when the merged model
/// cannot be written.
WorkholdingPart mergeWorkholdingParts(WorkholdingPackage& package, const std::vector<Uuid>& parts,
                                      std::string name, const PartModelFn& modelOf);

/// A template from the wizard's answers: the clicked parts on the members the type has (a
/// self-centring vise's second jaw linked to the first, moving the other way), picked datums on
/// the clicked faces, the type's holding, the opening range on the members, the mount (the
/// clicked face, else the bottom centre of all the parts). Up is the jaw tops' (or seat's)
/// normal; a chuck's is out of the chuck, its jaws (grouped by their angle about the axis) all
/// moving radially with the one carrying the gripping face. Throws Error(InvalidArgument) with a
/// message for the user when an answer is missing or does not work (the faces have no gap, the
/// moving jaw's face is on a fixed part).
WorkholdingTemplate buildWorkholding(const WorkholdingAnswers& answers,
                                     std::vector<WorkholdingPart> parts,
                                     const PartModelFn& modelOf);

} // namespace ocam::document
