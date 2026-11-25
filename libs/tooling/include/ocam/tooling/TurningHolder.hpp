// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// ISO turning holders for indexable inserts: rectangular shanks (designation ISO 5608) and boring
// bars (designation ISO 5609-1). The designation fixes what the holder takes and how it places
// the insert in the lathe plane. See docs/design-log/0042.

#pragma once

#include "ocam/tooling/Profile.hpp"
#include "ocam/tooling/Tool.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ocam::tooling {

/// A letter of a designation and what it stands for.
struct IsoLetter {
    char letter = ' ';
    double value = 0.0;      ///< Style: cutting edge angle [deg]; clearance [deg]; length [mm].
    const char* name = "";   ///< In English, for the "Tooling" translations.
    bool endCutting = false; ///< Styles: the cutting edge angle is to the facing feed.
};

const std::vector<IsoLetter>& holderClampings();  ///< ISO 5608 table 1.
const std::vector<IsoLetter>& holderStyles();     ///< ISO 5608 table 3.
const std::vector<IsoLetter>& insertClearances(); ///< ISO 5608 table 4.
const std::vector<IsoLetter>& holderLengths();    ///< Tool length symbols, in mm.
const std::vector<IsoLetter>& boringBarTypes();   ///< What a boring bar's shank is made of.
const IsoLetter* findIsoLetter(const std::vector<IsoLetter>& letters, char letter);

/// The letters of a value the standard's lists do not have (TurningHolderCode).
inline constexpr char kCustomStyle = 'X';
inline constexpr char kCustomClearance = 'O';
inline constexpr char kCustomLength = 'X';

/// "PCLNR 2525M 12" or "S25T-PCLNR 12".
std::string designation(const TurningHolderCode& code);
/// A designation in either form, with or without spaces and dashes, in either case (the
/// standardized part: a manufacturer's addition after it is ignored). f is the usual one for the
/// shank (standardF). nullopt when it is not a designation or has an unknown letter. The letters of
/// a custom value say nothing of the value: they take it from `custom`, and without one are unknown.
std::optional<TurningHolderCode> parseDesignation(std::string_view text,
                                                  const TurningHolderCode* custom = nullptr);

/// The usual dimension f of a shank: a neutral holder has the tip on the shank's middle, others
/// follow the ISO 5610 series for rectangular shanks and typical ISO 6261 values for boring bars.
/// Check it against the holder's data sheet.
double standardF(const TurningHolderCode& code);
/// Overall length from the tool length symbol or the custom value [mm]; 0 for an unknown symbol.
double holderLength(const TurningHolderCode& code);
/// The style's cutting edge angle as its letter states it [deg]; nullopt for an unknown letter.
std::optional<double> holderStyleAngle(const TurningHolderCode& code);
/// The insert clearance [deg]; nullopt for an unknown letter.
std::optional<double> holderClearanceAngle(const TurningHolderCode& code);
/// Inscribed circle of the insert from its shape and size symbol: the nearest of the ISO 1832
/// series (the size of a round insert is its diameter).
double inscribedCircle(const TurningHolderCode& code);
/// The angle of the insert's leading edge to a feed towards -Z [rad], for
/// TurningInsert::cuttingEdgeAngle: the style's own for side cutting, and for end cutting what
/// the insert's corner leaves after the style's angle to the facing feed. 0 for round inserts.
double sideCuttingEdgeAngle(const TurningHolderCode& code);
/// Bores the bar with its insert fits: dimension f plus half the bar [mm]. 0 for other holders.
double smallestBore(const TurningHolderCode& code);
/// Problems of a designation as human-readable sentences (empty = valid). The insert's corner has
/// to leave its other edge clear of the work in the holder's style.
std::vector<std::string> validate(const TurningHolderCode& code);

/// A turning holder of `code`, named by its designation. Its `profile` is the shank's outline
/// (shank width by overall length), so the holder's width and length read as any holder's.
Holder makeTurningHolder(const TurningHolderCode& code);
/// Sets a holder's code, and the outline that goes with it.
void setTurningHolderCode(Holder& holder, const TurningHolderCode& code);

/// Which way a holder of `code` cuts: a boring bar inside, a shank outside.
TurningSide holderSide(const TurningHolderCode& code);
/// Why a holder does not fit a tool. A holder without a designation says nothing about its seat, so
/// only its kind can be wrong. The hand is not among them: a tool takes its hand from its holder.
enum class HolderMismatch { Kind, Shape, Size, Clearance, Locking };
/// What keeps `holder` from taking `tool`'s insert: its shape, inscribed circle, relief and, when
/// the insert has one set, its locking style. Empty = it fits.
std::vector<HolderMismatch> holderMismatches(const Tool& tool, const Holder& holder);
/// The insert with this locking style goes in a holder of this ISO clamping letter.
bool clampingTakes(char clamping, InsertLocking locking);

/// Takes on what a tool gets from its ISO holder: the insert's shape, inscribed circle, relief,
/// hand, tip orientation and cutting edge angle. Other tools and holders are left as they are.
void fitToHolder(Tool& tool, const Holder& holder);

/// A holder in the lathe plane relative to the programmed point. A rectangular shank stands along
/// +X, its back f behind the tip towards +Z; a boring bar runs along +Z, its axis f below the tip.
/// Left-hand holders are mirrored in Z, as their `insert` has to be.
struct TurningHolderOutline {
    geom2d::Polyline head;  ///< Closed, convex: from the insert to the start of the shank.
    geom2d::Polyline shank; ///< Closed: the rest of the holder up to its overall length.
};
/// `insert` is the outline turningShape gives on the holder's side (unmirrored).
TurningHolderOutline turningHolderOutline(const TurningHolderCode& code,
                                          const geom2d::Polyline& insert);

/// A plain turning holder, and setting its shape with the outline that goes with it.
Holder makeTurningStick(const TurningStick& stick);
void setTurningStick(Holder& holder, const TurningStick& stick);

/// The parts of a turning tool that must not touch material, in the lathe plane relative to the
/// programmed point, each convex: an ISO holder's head beyond its insert, and its shank; a plain
/// holder's shank or bar; a solid bar's head and shank. Empty for an outline of its own, or no
/// holder: nothing says what shape they have in this plane.
/// With `awayFromChuck` a neutral holder stands mirrored in Z, as a left-hand one always does.
std::vector<geom2d::Polyline> turningBody(const Tool& tool, const Holder* holder, TurningSide side,
                                          bool awayFromChuck = false);

/// A piece of a turning tool's holder as it is shown, in the lathe plane relative to the programmed
/// point. Across the plane it runs from `low` to `high`, the tip's height being 0.
struct TurningToolPart {
    geom2d::Polyline outline; ///< Closed and convex.
    double low = 0.0;
    double high = 0.0;
    bool round = false; ///< A bar: a cylinder along Z, as wide across the plane as in it.

    bool operator==(const TurningToolPart&) const = default;
};
/// All of a turning tool but its cutting part, for showing it (turningBody is what is checked, and
/// leaves out what stands beside the insert). A shank's top is at the tip's height, a bar's axis at
/// it. An outline of its own is drawn where the tool editor's preview does: it says nothing about
/// where the holder really is, and nothing checks it. Empty with no holder.
std::vector<TurningToolPart> turningToolParts(const Tool& tool, const Holder* holder,
                                              TurningSide side, bool awayFromChuck = false);

/// A solid boring bar, with the dimensions of a common 6 mm micro bar to start from.
Tool makeSolidBoringBar(int number);
/// Takes on what a solid bar's dimensions say about its tip, which is not sold by these: a tip
/// reaching no further than the shank's axis, on the inside, right hand unless left.
void fitSolidBar(Tool& tool);

} // namespace ocam::tooling
