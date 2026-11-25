// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// What is fitted to a lathe for a job (DL-0068): tooling blocks on the turret's stations and the
// centre in the tailstock's quill. Each is a STEP model with the points that say how it goes on,
// kept in a library of the user's, and copied into the project that uses it (as workholding is).
// Which block sits where is the setup's; how many stations there are and where a block seats on
// one is the machine's.

#pragma once

#include "ocam/document/Project.hpp"
#include "ocam/document/StepBodies.hpp"

#include <ocam/foundation/Json.hpp>
#include <ocam/machine/Machine.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ocam::document {

inline constexpr int kFittingVersion = 2;
const FormatSpec& fittingFormat();
const SchemaValidator& fittingSchema();

Json toJson(const LatheFitting& fitting);
/// Migrates, validates against the schema and converts. Throws ocam::Error.
LatheFitting fittingFromJson(Json doc, std::string_view sourceName);

/// A fitting with its STEP file.
struct FittingPackage {
    LatheFitting fitting;
    StepFile file;
    bool builtin = false; ///< One of builtinFittings: read-only.
};

/// A new fitting of `kind` from a model: its file made, its points at the model's origin to be
/// picked or typed.
FittingPackage makeFitting(std::string name, LatheFitting::Kind kind, const cad::Model& model);

/// The generic examples (data/fittings): shapes of this project's own, with placeholder sizes, for
/// starting from or for trying out. Not any maker's.
const std::vector<FittingPackage>& builtinFittings();

/// Writes <folder>/<id>.json and <folder>/<sha256>.step. Throws Error for an invalid fitting or
/// when it cannot be written.
void saveFitting(const std::filesystem::path& folder, const FittingPackage& package);
/// Every valid fitting in the folder, by name. Unreadable ones are reported in `problems` (one
/// sentence each) and skipped.
std::vector<FittingPackage> loadFittings(const std::filesystem::path& folder,
                                         std::vector<std::string>* problems = nullptr);
/// Deletes a fitting's file, and its STEP file if nothing else in the folder uses it. False when
/// there is none.
bool removeFittingFile(const std::filesystem::path& folder, const Uuid& id);

/// The direction in a block's coordinates that goes towards the work once it is on a station: its
/// tool mount's direction if that lies across the seat (a tool along the spindle); else from the
/// seat's point towards the tool mount in the seat's plane (the side a shank lies on), else the
/// block's own X; then turned by `seatTurn` about the seat's direction.
Vec3 towardsWork(const LatheFitting& block);

/// The direction in a block's coordinates from a held tool's shank to its tip's height: the way
/// its rake face looks (the seat's direction across towardsWork). A square shank lies on the other
/// side of the tool mount; on a turret it is the station's Y, so a rear turret holds it face down.
Vec3 blockUp(const LatheFitting& block);

/// Whether a block holds a tool across the spindle (the tool points the way the seat does) rather
/// than along it.
bool blockAcross(const LatheFitting& block);

/// A block's coordinates -> the model frame at machine zero, in the working position of its
/// station: its seat on the turret's seat point (blockRadius, blockAcross, blockAlong), the seat's
/// direction towards the spindle, towards
/// the work as towardsWork has it. `towardsPlusZ`: the work is on +Z of the turret (a
/// sub-spindle's).
Transform3 blockOnStation(const LatheFitting& block, const machine::LatheLayout& layout,
                          bool towardsPlusZ);

/// A tailstock centre's coordinates -> the setup's WCS: its tip at `tipZ` on the spindle's axis,
/// pointing towards -Z.
Transform3 centreInWcs(const LatheFitting& centre, double tipZ);

/// How far a tailstock centre reaches from its tip back to its seat in the quill [mm].
double centreLength(const LatheFitting& centre);

/// The mount type of the generic examples, and the one the generic lathes take: placeholders.
inline constexpr const char* kGenericBlockMount = "Generic bolt-on (placeholder)";
inline constexpr const char* kGenericQuillInterface = "Generic taper (placeholder)";

/// Why a fitting does not go on a machine, as a sentence: a block whose mount type is not one the
/// station takes (`station` 0: none of the turret's stations), a centre that does not fit the
/// quill. None when it fits, when the machine does not say what it takes, or when it is not a lathe
/// with a layout.
std::optional<std::string> mountMismatch(const LatheFitting& fitting,
                                         const machine::Machine& machine, int station = 0);

/// Drops the fittings no setup uses.
void removeUnusedFittings(Project& project);

} // namespace ocam::document
