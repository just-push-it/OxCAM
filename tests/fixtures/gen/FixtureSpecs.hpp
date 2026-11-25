// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Parameters of the generated STEP fixtures, shared by the generator and the tests so analytic
// expectations stay in sync with the geometry. All fixtures are original geometry built from
// OpenCASCADE primitives; no third-party models are used.

#pragma once

#include <cmath>
#include <numbers>

namespace ocam::fixtures {

/// block_pocket.step (and block_pocket_inch.step, the same part written in inches):
/// a block with a rectangular pocket whose vertical corners are filleted.
namespace block_pocket {
inline constexpr double kLength = 100, kWidth = 60, kHeight = 30;
inline constexpr double kPocketX = 30, kPocketY = 20; // pocket min corner
inline constexpr double kPocketLength = 40, kPocketWidth = 20, kPocketDepth = 10;
inline constexpr double kCornerRadius = 5;
inline constexpr double kPocketArea =
    kPocketLength * kPocketWidth - (4.0 - std::numbers::pi) * kCornerRadius * kCornerRadius;
inline constexpr double kVolume = kLength * kWidth * kHeight - kPocketArea * kPocketDepth;
} // namespace block_pocket

/// fillet_pocket.step: a block with a rectangular pocket whose vertical corners and floor edges
/// are filleted (pencil finishing: the floor fillet is tighter than the tools that finish it).
namespace fillet_pocket {
inline constexpr double kLength = 80, kWidth = 60, kHeight = 25;
inline constexpr double kPocketX = 15, kPocketY = 15; // pocket min corner
inline constexpr double kPocketLength = 50, kPocketWidth = 30, kPocketDepth = 12;
inline constexpr double kCornerRadius = 8; // vertical corners
inline constexpr double kFloorRadius = 3;  // floor edges
inline constexpr double kFloorZ = kHeight - kPocketDepth;
} // namespace fillet_pocket

/// hole_plate.step: a plate with one hole of each kind, drilled from the top (+Z face).
namespace hole_plate {
inline constexpr double kSize = 80, kThickness = 20;
// A: through hole
inline constexpr double kThroughX = 20, kThroughY = 20, kThroughDiameter = 10;
// B: blind, flat bottom
inline constexpr double kFlatX = 60, kFlatY = 20, kFlatDiameter = 8, kFlatDepth = 6;
// C: blind, drill point
inline constexpr double kDrillX = 20, kDrillY = 60, kDrillDiameter = 6, kDrillDepth = 8;
inline constexpr double kDrillTipAngle = 118.0 * std::numbers::pi / 180.0;
// D: counterbored through hole
inline constexpr double kCboreX = 60, kCboreY = 60, kCboreDiameter = 16, kCboreDepth = 4;
inline constexpr double kCboreHoleDiameter = 9;

inline double drillTipLength() {
    return 0.5 * kDrillDiameter / std::tan(0.5 * kDrillTipAngle);
}
} // namespace hole_plate

/// boss_plate.step: a plate with a cylindrical boss (must not be recognized as a hole) and one
/// small through hole.
namespace boss_plate {
inline constexpr double kSize = 60, kThickness = 10;
inline constexpr double kBossX = 30, kBossY = 30, kBossDiameter = 20, kBossHeight = 15;
inline constexpr double kHoleX = 10, kHoleY = 10, kHoleDiameter = 6;
} // namespace boss_plate

/// shaft.step: a stepped turned shaft along +Z.
namespace shaft {
inline constexpr double kMajorDiameter = 30, kMajorLength = 50;
inline constexpr double kMinorDiameter = 20, kMinorLength = 30;
inline constexpr double kVolume =
    std::numbers::pi * (kMajorDiameter * kMajorDiameter / 4 * kMajorLength +
                        kMinorDiameter * kMinorDiameter / 4 * kMinorLength);
} // namespace shaft

/// bored.step: a turned part along +Z with a shoulder and a blind bore from its front (z =
/// kLength).
namespace bored {
inline constexpr double kOuterDiameter = 40, kShoulderDiameter = 30, kShoulderLength = 20;
inline constexpr double kLength = 60, kBoreDiameter = 20, kBoreDepth = 35;
inline constexpr double kVolume = std::numbers::pi / 4 *
                                  (kOuterDiameter * kOuterDiameter * (kLength - kShoulderLength) +
                                   kShoulderDiameter * kShoulderDiameter * kShoulderLength -
                                   kBoreDiameter * kBoreDiameter * kBoreDepth);
} // namespace bored

/// spool.step: a part along +Z with a flange between two ends, turned from both sides.
namespace spool {
inline constexpr double kEndDiameter = 20, kEndLength = 25, kFlangeDiameter = 40,
                        kFlangeLength = 10;
inline constexpr double kLength = 2 * kEndLength + kFlangeLength;
inline constexpr double kVolume = std::numbers::pi / 4 *
                                  (kEndDiameter * kEndDiameter * 2 * kEndLength +
                                   kFlangeDiameter * kFlangeDiameter * kFlangeLength);
} // namespace spool

/// grooved.step: a bored part along +Z with a groove outside, in the bore and in its front face.
namespace grooved {
inline constexpr double kOuterDiameter = 40, kBoreDiameter = 20, kLength = 30;
inline constexpr double kOutsideLow = 10, kOutsideHigh = 14, kOutsideFloor = 34;
inline constexpr double kInsideLow = 12, kInsideHigh = 16, kInsideRoof = 24;
inline constexpr double kFaceInner = 28, kFaceOuter = 34, kFaceDepth = 3; // diameters
} // namespace grooved

/// tapered.step: a shaft along +Z with a cone at its front, from Ø20 at z = kLength.
namespace tapered {
inline constexpr double kFrontDiameter = 20, kDiameter = 30, kTaperLength = 10, kLength = 40;
} // namespace tapered

/// bushing.step: a plain bushing along +Z with an M10 tap drill's through bore.
namespace bushing {
inline constexpr double kOuterDiameter = 30, kLength = 40, kBoreDiameter = 8.5;
} // namespace bushing

/// live.step: a shaft along +Z with holes a live tool drills: a bolt circle in its front face, a
/// hole across the diameter through its axis, and one across it off its axis.
namespace livetool {
inline constexpr double kDiameter = 40, kLength = 60;
inline constexpr double kBoltCircleRadius = 12, kBoltDiameter = 6, kBoltDepth = 15;
inline constexpr double kCrossDiameter = 8, kCrossZ = 30;
inline constexpr double kOffsetDiameter = 5, kOffsetZ = 15, kOffsetY = 8;
} // namespace livetool

/// millturn.step: a bar turned, then milled by live tools: a pocket on the axis, a counterbore, a
/// threaded hole and a tapped one in the face, and a flat across the part (millturn_project.ocam).
namespace millturn {
inline constexpr double kDiameter = 40, kLength = 60;
inline constexpr double kPocketSide = 12, kPocketDepth = 3;
inline constexpr double kBoreDiameter = 10, kBoreDepth = 12, kBoreY = 13;
inline constexpr double kThreadDrill = 8.5, kThreadMajor = 10, kThreadPitch = 1.5;
inline constexpr double kThreadDepth = 12, kThreadX = -12;
inline constexpr double kTapDrill = 5, kTapMajor = 6, kTapPitch = 1, kTapDepth = 10, kTapY = -12;
inline constexpr double kFlatX = 17;
} // namespace millturn

/// stud_bar.step: a bar with a flat milled along its side and a stud standing out of the flat,
/// across the part, for an external thread a live tool mills with C held still.
namespace stud_bar {
inline constexpr double kDiameter = 40, kLength = 60;
inline constexpr double kFlatX = 15, kStudDiameter = 10, kStudLength = 8, kStudZ = 30;
} // namespace stud_bar

/// wrapped.step: a shaft on Z, with a neck turned down at its front, and a slot wrapped around the
/// big diameter over `kSlotAngle`, centred on +X, its front end open into the neck
/// (wrapped_project).
namespace wrapped {
inline constexpr double kDiameter = 40, kLength = 60, kNeckDiameter = 30, kNeckLength = 15;
inline constexpr double kSlotZ0 = 25, kSlotZ1 = 35, kSlotDepth = 3;
inline constexpr double kSlotAngle = 120.0 * std::numbers::pi / 180.0;
} // namespace wrapped

/// dome.step: a square base plate with a spherical cap on top (3D milling).
namespace dome {
inline constexpr double kBase = 80, kBaseHeight = 10;
inline constexpr double kSphereRadius = 35, kSphereCenterZ = -5; // centre above (40, 40)
inline constexpr double kTop = kSphereCenterZ + kSphereRadius;   // 30
inline constexpr double kCapHeight = kTop - kBaseHeight;         // 20
inline constexpr double kVolume =
    kBase * kBase * kBaseHeight +
    std::numbers::pi * kCapHeight * kCapHeight * (3 * kSphereRadius - kCapHeight) / 3;
} // namespace dome

/// wedge_hole.step: a block whose top slopes down along +X, with a vertical through hole in the
/// slope: edges that are not flat for 3D edge breaking (DL-0083).
namespace wedge_hole {
inline constexpr double kLength = 60, kWidth = 40;
inline constexpr double kHigh = 30, kLow = 15; // top height at x = 0 and at x = kLength
inline constexpr double kHoleX = 30, kHoleY = 20, kHoleDiameter = 10;
} // namespace wedge_hole

/// swarf.step: a base plate with two bosses whose walls are ruled (swarf milling, DL-0085): a
/// tapered round boss (a cone) and a twisted square boss, its top square turned about the boss's
/// axis against its bottom one, each side a ruled surface between the two edges.
namespace swarf {
inline constexpr double kBaseX = 120, kBaseY = 70, kBaseHeight = 10, kBossHeight = 20;
inline constexpr double kTop = kBaseHeight + kBossHeight; // 30
inline constexpr double kConeX = 30, kConeY = 35, kConeBottomRadius = 18, kConeTopRadius = 13;
inline constexpr double kTwistX = 88, kTwistY = 35, kTwistSide = 28;
inline constexpr double kTwistAngle = 10.0 * std::numbers::pi / 180.0;
} // namespace swarf

// A shaft for 4th-axis work (rotary_shaft.step): diameter 40 along X from x = 0 to 80, with a
// groove over 120 degrees (centred on +Z, radial walls, cylindrical floor) for x in [30, 50],
// 3 mm deep, and a ring groove for x in [8, 16], 2 mm deep.
namespace rotary {
inline constexpr double kDiameter = 40, kLength = 80;
inline constexpr double kGrooveX0 = 30, kGrooveX1 = 50, kGrooveDepth = 3;
inline constexpr double kGrooveAngle = 120.0 * std::numbers::pi / 180.0;
inline constexpr double kRingX0 = 8, kRingX1 = 16, kRingDepth = 2;
} // namespace rotary

/// vise_jaws.step: two vise jaws gripping block_pocket's stock (1 mm side offset) along its long
/// sides, shorter than the block so its ends stand free (fixture_project.ocam).
namespace vise_jaws {
inline constexpr double kX0 = 10, kX1 = 90; // along the block's length
inline constexpr double kThickness = 15;    // each jaw, outward from the stock
inline constexpr double kGap = 1;           // the stock's side offset
inline constexpr double kTop = 22;          // jaw tops; the block is 30 tall
} // namespace vise_jaws

/// vise_base.step, vise_fixed_jaw.step, vise_moving_jaw.step: a machine vise in three bodies
/// (the jaws open along +Y), its moving jaw modelled 50 mm from the fixed jaw; vise.ocamhold is
/// the workholding template made of them, and vise_project.ocam holds block_pocket's stock in
/// it, the jaws along the block's width.
namespace vise {
inline constexpr double kJawHalfWidth = 50;                     // the jaws span X -50..50
inline constexpr double kJawThickness = 20;                     // along Y
inline constexpr double kFixedFace = 20;                        // the fixed jaw spans Y 0..20
inline constexpr double kModelledOpening = 50;                  // the moving jaw spans Y 70..90
inline constexpr double kBaseTop = 40, kJawTop = 70;            // jaws from the base top up
inline constexpr double kBaseHalfWidth = 60, kBaseLength = 200; // base X -60..60, Y -10..190
inline constexpr double kMaxOpening = 150;
inline constexpr double kGripDepth = 8; // vise_project.ocam
} // namespace vise

/// trunnion_machine.step: a 5-axis trunnion machine (A tilts the cradle about X, C turns the
/// round table) as one STEP part of many bodies in inches, the way machine makers often ship a
/// machine: base, enclosure, column, saddle, x table, support left and right, cradle, rotary
/// table, spindle nose, spindle head (named) and kBolts bolts (unnamed): kBodies bodies placed at
/// machine zero, with the table centre (where A and C meet, on the table top) at the origin.
/// trunnion_assembly.step: the same machine (without the enclosure) as an assembly: below the top
/// assembly Frame (base, column, Bolts: kBolts bolts), Y saddle (saddle), X table (x table, two
/// instances of one support), Cradle, Rotary table, Head (spindle nose, spindle head):
/// kAssemblyParts parts. machines/trunnion-demo.json is the machine built from
/// trunnion_machine.step one part per body, and dome5_machine_project.ocam the 5-axis dome
/// project on it (DL-0037).
namespace trunnion_machine {
inline constexpr double kSpindleZ = 450;     // spindle gauge point at machine zero
inline constexpr double kTableRadius = 210;  // round table, top at z = 0
inline constexpr double kCradleInner = 262;  // cradle walls from |x| = 262 to 300
inline constexpr double kSupportInner = 330; // trunnion supports from |x| = 330 to 410
inline constexpr double kColumnFront = 620;  // fixed column behind the table
inline constexpr int kBolts = 8;
inline constexpr int kBodies = 11 + kBolts;                           // 19
inline constexpr int kAssemblyParts = 2 + kBolts + 1 + 3 + 1 + 1 + 2; // 18
inline constexpr int kSubassemblies = 6;
inline constexpr double kEnclosureX = 1000;    // side walls inside |x| = 990 .. 1000
inline constexpr double kEnclosureBack = 1100; // back wall y = 1090 .. 1100, open front and top
} // namespace trunnion_machine

/// vise_assembly.step: the same three vise bodies (base, fixed jaw, moving jaw) in one file.
/// selfcentring_assembly.step: a self-centring vise, its jaws modelled 50 mm apart about Y 0
/// (base, then the jaw on -Y, then the jaw on +Y), opening along +Y.
namespace selfcentring {
inline constexpr double kJawHalfWidth = 40; // the jaws span X -40..40
inline constexpr double kJawThickness = 20;
inline constexpr double kModelledOpening = 50;
inline constexpr double kBaseTop = 30, kJawTop = 60;
inline constexpr double kMaxOpening = 120;
} // namespace selfcentring

/// dovetail_assembly.step: a dovetail fixture across Y (base, fixed jaw on -Y, moving jaw on +Y),
/// 80 mm long in X. The jaws' tops (the seat) are at Z 26, their flanks lean in at 30 degrees
/// from vertical, 30 mm apart at the seat as modelled; the tenon is 5 mm deep.
namespace dovetail {
inline constexpr double kLength = 80;
inline constexpr double kBaseTop = 20, kSeat = 26;
inline constexpr double kModelledWidth = 30; // between the flanks at the seat
inline constexpr double kFlankAngle = 30.0;  // degrees from vertical
inline constexpr double kDepth = 5;
inline constexpr double kOuter = 60; // the jaws reach Y -60 and 60
} // namespace dovetail

/// plate_assembly.step: a fixture plate (X 0..200, Y 0..150, Z 0..15) with a locating pin
/// (Ø10 at X 20, Y 20, 10 tall); the stock's bottom centre goes to the plate's top centre.
namespace plate {
inline constexpr double kLength = 200, kWidth = 150, kThickness = 15;
inline constexpr double kPinX = 20, kPinY = 20, kPinDiameter = 10, kPinHeight = 10;
} // namespace plate

/// chuck_assembly.step: a 3-jaw lathe chuck on the Z axis, facing +Z: a body (Ø160, Z 0..60, bore
/// Ø40) and three jaws at 0, 120 and 240 degrees about Z (the first on +X), each 20 wide, from
/// radius 25 (the gripping face: Ø50 as modelled) out to 70, Z 60..100 (the jaw front).
/// chuck.ocamhold grips Ø4 to Ø130, the bar 15 mm into the jaws.
namespace chuck {
inline constexpr double kBodyDiameter = 160, kBodyLength = 60, kBore = 40;
inline constexpr double kGripRadius = 25, kJawOuter = 70, kJawWidth = 20, kJawFront = 100;
inline constexpr double kMinDiameter = 4, kMaxDiameter = 130;
inline constexpr double kGripLength = 15;
} // namespace chuck

/// collet_assembly.step: a collet chuck on the Z axis, facing +Z: its body (Ø60, Z 0..40, bore
/// Ø40) and a collet in it (Ø40, bore Ø20, Z 0..45: its front). collet.ocamhold grips bars from
/// Ø19.5 to Ø20, 20 mm deep.
namespace collet {
inline constexpr double kBodyDiameter = 60, kBodyLength = 40;
inline constexpr double kOuter = 40, kBore = 20, kFront = 45;
inline constexpr double kGripRange = 0.5, kGripLength = 20;
} // namespace collet

} // namespace ocam::fixtures
