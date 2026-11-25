// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Turning strategies (phase 8). Geometry lives in the lathe plane: u = axial Z, v = radius,
// both in the setup's WCS whose Z axis is the spindle axis; the part's front is at its largest
// Z and tools approach from there. Toolpaths use x = radius, y = 0, z = axial, and program the
// insert's reference point (tooling::turningShape): the imaginary tip of turning inserts, so
// nose-radius compensation is applied here ("in the computer"). See docs/design-log/0020.

#pragma once

#include "ocam/strategies/Common.hpp"

#include <ocam/geom2d/Region.hpp>
#include <ocam/tooling/Profile.hpp>

#include <limits>

namespace ocam::strategies {

struct TurnGeometry {
    geom2d::Regions part;  ///< Half profile of the finished part, (z, r >= 0).
    geom2d::Regions stock; ///< Half profile of the stock before the operation.
    /// Outside roughing and finishing: the programmed point stays at or in front of this z (the
    /// chuck's jaws in front of it, the insert's reach and the clearance off) [mm].
    double zSign = 1.0; ///< -1 while mirrored in Z (back turning): for the messages.
    double backStop = -std::numeric_limits<double>::infinity();
};

struct TurnCommon {
    double clearance = 1.0;    ///< Air gap for approaches and retracts [mm].
    double surfaceSpeed = 0.0; ///< Constant surface speed [mm/s]; 0 = constant spindle speed.
    double maxSpindle = 0.0;   ///< Spindle limit under constant surface speed [rev/s].
};

struct TurnFaceParams {
    TurnCommon common;
    double depthOfCut = 1.0;
    double stockToLeave = 0.0; ///< Left on the front face (axial).
    double overshoot = 0.5;    ///< Past the axis [mm].
};

/// The part of the profile a rough or finish operation works on, WCS Z [mm].
struct TurnZRange {
    double front = 0.0;
    double back = 0.0;
};

struct TurnRoughParams {
    TurnCommon common;
    tooling::TurningSide side = tooling::TurningSide::Outside;
    std::optional<TurnZRange> zRange; ///< Unset: the whole profile.
    bool across = false;              ///< Level passes across Z, as in facing, instead of along it.
    bool awayFromChuck = false;       ///< Passes run towards the front (back turning).
    double depthOfCut = 2.0;          ///< Radial, per pass.
    double stockToLeave = 0.3;        ///< Left on the profile (radial and axial).
    double retract = 0.5;             ///< 45 degree pull-off after each pass.
    double backLimit = 0.0;           ///< How far beyond the part's back end passes may run [mm].
    bool profilePass = true;          ///< Finish with a pass along the allowance (even steps out).
};

struct TurnFinishParams {
    TurnCommon common;
    tooling::TurningSide side = tooling::TurningSide::Outside;
    std::optional<TurnZRange> zRange; ///< Unset: the whole profile.
    bool awayFromChuck = false;       ///< Passes run towards the front (back turning).
    /// The program is the profile itself, and the control compensates the nose (G41/G42).
    bool inControl = false;
    double stockToLeave = 0.0;
    double backLimit = 0.0;
};

inline constexpr double kGrooveStepShare = 0.8; ///< Groove plunge step, of the insert width.

struct TurnGrooveParams {
    TurnCommon common;
    tooling::TurningSide side = tooling::TurningSide::Outside;
    double stockToLeave = 0.0;
    std::optional<double> step; ///< Sideways step between plunges [mm]; unset: kGrooveStepShare w.
    double peck = 0.0;          ///< Radial peck depth; 0 = single plunge.
};

struct TurnThreadParams {
    TurnCommon common;
    double pitch = 0.0;         ///< 0 = the insert's pitch.
    double majorDiameter = 0.0; ///< 0 = the part diameter at the thread start.
    double startOffset = 0.0;   ///< Thread start behind the part's front face [mm].
    double length = 10.0;
    double depth = 0.0; ///< Radial thread depth; 0 = 0.6134 x pitch (ISO external).
    int passes = 6;     ///< Degressive infeed: depth after pass k is depth * sqrt(k / passes).
    int springPasses = 1;
    double leadIn = 0.0; ///< Run-up before the thread for spindle synchronization; 0 = 3 pitches.
    tooling::TurningSide side = tooling::TurningSide::Outside;
    bool leftHand = false; ///< Cut from the back towards the front, the spindle still forward.
    int starts = 1;        ///< Lead = pitch x starts; each start shifted by a pitch.
    double taper = 0.0;    ///< Radius at the thread's end minus at its start [mm].
};

struct TurnPartOffParams {
    TurnCommon common;
    double zOffset = 0.0; ///< Cut position behind the part's back face [mm].
    double xEnd = -0.5;   ///< Final radius of the reference corner (below 0: past the axis).
    double peck = 0.0;
    double slowDistance = 2.0; ///< The end feed runs over this last radial distance.
};

toolpath::Toolpath turnFace(const TurnGeometry& geometry, const TurnFaceParams& params,
                            const Context& ctx);
toolpath::Toolpath turnRough(const TurnGeometry& geometry, const TurnRoughParams& params,
                             const Context& ctx);
toolpath::Toolpath turnFinish(const TurnGeometry& geometry, const TurnFinishParams& params,
                              const Context& ctx);
toolpath::Toolpath turnGroove(const TurnGeometry& geometry, const TurnGrooveParams& params,
                              const Context& ctx);
toolpath::Toolpath turnThread(const TurnGeometry& geometry, const TurnThreadParams& params,
                              const Context& ctx);
toolpath::Toolpath turnPartOff(const TurnGeometry& geometry, const TurnPartOffParams& params,
                               const Context& ctx);

/// The OD (or ID) nose-centre path that keeps a round nose of radius `rn` outside `part` grown
/// by `stockToLeave`, from the part's front to `zBack`, descending towards -Z no steeper than
/// the insert's trailing edge (`trailingAngle`) and, with a leading edge ahead of the nose
/// (`leadAngle` over 90 degrees), rising towards -Z no steeper than that edge allows: it reaches
/// a shoulder before the nose does. Points (z, r) from front to back.
std::vector<Vec2> noseCentrePath(const geom2d::Regions& part, double stockToLeave, double rn,
                                 double trailingAngle, double zBack, tooling::TurningSide side,
                                 double leadAngle = kPi / 2.0);

// -- Bar work (DL-0091) ------------------------------------------------------------------------

/// The bar a puller takes, in the lathe plane of the setup's WCS.
struct BarPullGeometry {
    double front = 0.0;  ///< The bar's front face, WCS Z [mm].
    double radius = 0.0; ///< Its largest radius where the puller's jaws go over it [mm].
};

struct BarPullParams {
    double pullLength = 0.0; ///< How far the bar comes out of the chuck [mm].
    double grip = 0.0;       ///< How far the jaws go over the bar, from its front [mm].
    double clearance = 0.0;  ///< In front of the bar, where the rapid stops [mm].
    double dwell = 0.0;      ///< After the chuck opens and after it closes [s].
    bool barFeeder = false;  ///< The bar feeder pushes while the chuck is open.
};

/// A puller on the spindle's axis (x = radius 0): in at rapid to the clearance, onto the bar at the
/// plunge feed, the chuck opens, the bar comes out by the pull length at the pull feed, the chuck
/// closes, and the puller lets go back off the bar at the plunge feed. The spindle stands still.
/// The machine functions are Dwell moves carrying them. Refuses (Error InvalidArgument) a tool that
/// is not a bar puller, a bar it cannot take, a pull or grip of 0, and a plunge or pull feed of 0,
/// naming the value.
toolpath::Toolpath barPull(const BarPullGeometry& geometry, const BarPullParams& params,
                           const Context& ctx);

// -- Parameter objects ------------------------------------------------------------------------

BarPullParams barPullParams(const Json& values, const tooling::Tool& tool);

TurnCommon turnCommon(const Json& values, const tooling::CuttingPreset* preset);
TurnFaceParams turnFaceParams(const Json& values, const tooling::CuttingPreset* preset);
/// Whether a turning operation's parameters limit it to a Z range: it then works from the stock
/// every earlier operation left, not only the setup's.
bool turnHasZRange(std::string_view operationType, const Json& values);
TurnRoughParams turnRoughParams(const Json& values, const tooling::CuttingPreset* preset);
/// Boring soft jaws (DL-0069): roughing passes in the jaws, `inside` for a bore (an outside grip),
/// else on their outside, then the profile pass if asked.
TurnRoughParams turnSoftJawsParams(const Json& values, const tooling::CuttingPreset* preset,
                                   bool inside);
TurnFinishParams turnFinishParams(const Json& values, const tooling::CuttingPreset* preset);
TurnGrooveParams turnGrooveParams(const Json& values, const tooling::CuttingPreset* preset);
TurnThreadParams turnThreadParams(const Json& values, const tooling::CuttingPreset* preset);
TurnPartOffParams turnPartOffParams(const Json& values, const tooling::CuttingPreset* preset);

/// Turning feeds for a reference diameter: spindle = vc / (pi d) (capped by "max_rpm"), cut
/// feed = feed per revolution x spindle. vc and f come only from the insert's cutting preset
/// (surface speed, and feed per tooth read as feed per revolution); there are no defaults.
/// Throws Error(InvalidArgument) without a preset, or when it lacks the surface speed or (with
/// `needsFeed`, everything but threading, whose feed is the pitch) the feed.
Feeds turningFeeds(const Json& values, const tooling::CuttingPreset* preset,
                   double referenceDiameter, bool needsFeed = true);

} // namespace ocam::strategies
