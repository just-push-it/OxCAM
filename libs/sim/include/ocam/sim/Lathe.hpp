// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Turning simulation on an axisymmetric stock: the stock is its half profile in the lathe plane
// (u = axial Z, v = radius), and each move removes the area its insert outline sweeps. The
// result converts one way to the tri-dexel stock for milling it afterwards.
// What is not axisymmetric, threads and holes off the axis, is held beside the profile as exact
// cuts. A job without any is the profile alone. See docs/design-log/0021 and 0061.

#pragma once

#include "ocam/sim/Compare.hpp"
#include "ocam/sim/Dexel.hpp"
#include "ocam/sim/FixtureCheck.hpp"
#include "ocam/sim/Simulate.hpp"

#include <ocam/foundation/Cancel.hpp>
#include <ocam/geom2d/Region.hpp>
#include <ocam/geom3d/Mesh.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace ocam::sim {

/// Volume of the solid swept by revolving (u, v >= 0) regions about the u axis [mm^3].
double revolvedVolume(const geom2d::Regions& profile);

/// A thread-cutting pass: the insert's outline going from `from` to `to` (lathe plane) while the
/// part turns `lead` per revolution. Exact at any pitch: it is the helix, not a sampling of it.
struct LatheThreadCut {
    Vec2 from;
    Vec2 to;
    double lead = 0.0;
    double turn = 0.0; ///< Z per radian of the part: positive is a right-hand helix.
    double done = 1.0; ///< The share of the pass cut so far.
    geom2d::Polyline insert;

    bool operator==(const LatheThreadCut&) const = default;
};

/// A hole off the spindle axis, or across it, by a tool standing on `axis`.
struct LatheHoleCut {
    Vec3 tip = Vec3::Zero();   ///< The deepest the tool's tip got (WCS).
    Vec3 axis = Vec3::UnitZ(); ///< Unit, from the tip towards the way the tool came in.
    double length = 0.0;       ///< From the tip back along the axis, as far as it cut.
    std::vector<Vec2> profile; ///< (distance from the tip, radius), not getting narrower.

    bool operator==(const LatheHoleCut&) const = default;
};

/// A hole's radius at `along` from its tip.
double holeRadiusAt(const LatheHoleCut& hole, double along);

class LatheStock {
public:
    LatheStock() = default;
    explicit LatheStock(geom2d::Regions profile) : profile_(std::move(profile)) {}
    /// A bar (or tube, rInner > 0) between zMin and zMax.
    static LatheStock cylinder(double zMin, double zMax, double rOuter, double rInner = 0.0);

    /// What turning leaves: the stock's envelope, with no thread or hole off the axis in it.
    const geom2d::Regions& profile() const noexcept { return profile_; }
    /// What is left, cuts taken off (those sampled: a thread's within a fraction of a percent).
    double volume() const;
    void subtract(const geom2d::Regions& area);

    const std::vector<LatheThreadCut>& threads() const noexcept { return threads_; }
    const std::vector<LatheHoleCut>& holes() const noexcept { return holes_; }
    bool hasCuts() const noexcept { return !threads_.empty() || !holes_.empty(); }
    /// Cuts share [t0, t1] of a pass from `from` to `to`; going on with a pass adds to it.
    void cutThread(const geom2d::Polyline& insert, const Vec2& from, const Vec2& to, double lead,
                   double t0 = 0.0, double t1 = 1.0);
    /// A thread cut as it was (reading one back).
    void keepThread(LatheThreadCut cut) { threads_.push_back(std::move(cut)); }
    /// Cuts a hole; the same hole again, deeper or not, adds to it.
    void cutHole(const LatheHoleCut& hole);
    /// Takes on the cuts of `other` (a stock carried from an earlier setup), put through `move`:
    /// along the spindle axis, or flipped end for end.
    void takeCuts(const LatheStock& other, const Transform3& move);

    /// The bar the chuck holds (DL-0091): the pieces of the profile that reach its back end, the
    /// lowest Z. Pieces in front of them are parts already parted off.
    geom2d::Regions heldBar() const;
    /// The stock with the held bar pulled `distance` along +Z out of the chuck (DL-0091): its
    /// pieces and the cuts in them move; behind them the bar goes on as it was at its back end;
    /// what was parted off is left out, having come off.
    LatheStock pulled(double distance) const;

private:
    geom2d::Regions profile_;
    std::vector<LatheThreadCut> threads_;
    std::vector<LatheHoleCut> holes_;
};

struct LatheSimResult {
    std::size_t movesSimulated = 0;
    std::size_t threadMoves = 0; ///< Held as thread cuts: they leave the profile as it is.
};

/// Applies moves [begin, end) of a live tool's toolpath (in the WCS, its axis on each move): each
/// feed into the material along the tool's axis cuts a hole of `profile` ((distance from the tip,
/// radius); tooling::holeToolSide).
void cutHoles(LatheStock& stock, const std::vector<Vec2>& profile, const toolpath::Toolpath& path,
              std::size_t begin = 0, std::size_t end = std::numeric_limits<std::size_t>::max());

/// Applies moves [begin, end) of a turning toolpath (x = radius, z = axial). The insert outline
/// (closed, convex, relative to the programmed point; tooling::turningShape) sweeps the convex
/// hull of its start and end placements along each straight move; arcs are sampled. A threading
/// toolpath's cutting passes become thread cuts, and its other moves remove nothing. With `rapids`,
/// a rapid whose insert goes through what the profile still holds at that move is reported there.
LatheSimResult simulateTurning(LatheStock& stock, const geom2d::Polyline& insert,
                               const toolpath::Toolpath& path, std::size_t begin = 0,
                               std::size_t end = std::numeric_limits<std::size_t>::max(),
                               CancelToken* cancel = nullptr,
                               std::vector<Collision>* rapids = nullptr);

/// A turned stock against the part's half profile, both in the lathe plane: the excess is the
/// stock outside the part grown by the tolerance, the gouge the part shrunk by it that the stock
/// no longer holds. Exact where the dexel comparison is sampled, so it has no per-ray fields
/// (nu = nv = 0, and excessRays and gougeRays count areas).
Comparison compareProfile(const geom2d::Regions& stock, const geom2d::Regions& part,
                          double tolerance);

/// The area a body sweeps turning about the Z axis (a chuck's jaws, a collet), in the lathe plane
/// (u = z, v = radius): per slice of `step` along Z, from the body's smallest radius in the slice
/// out to its largest, grown by `grow` all round. `mesh` is in the WCS, the spindle on its Z axis.
/// Throws Error(InvalidArgument) for a step that is not positive.
geom2d::Regions revolvedEnvelope(const geom3d::Mesh& mesh, double step, double grow = 0.0);

/// A body turning with the spindle, as the area it sweeps (revolvedEnvelope).
struct LatheBody {
    std::string name;
    geom2d::Regions area; ///< Lathe plane (u = z, v = radius).
};

/// Checks a turning toolpath (x = radius, z = axial; threading included) against the bodies: the
/// insert outline swept along each move, as simulateTurning sweeps it, overlapping a body's area.
/// Consecutive moves touching the same body form one collision: toolPart "insert", position
/// (radius, 0, z) of a point of the first contact, distance 0. A shape reaching across the axis
/// (`bothSides`, a boring bar's) meets the body on the other side of it too.
std::vector<FixtureCollision>
checkTurningFixtures(const toolpath::Toolpath& path, const geom2d::Polyline& insert,
                     const std::vector<LatheBody>& bodies, std::size_t maxCollisions = 200,
                     CancelToken* cancel = nullptr, bool bothSides = false);

/// Where the non-cutting parts of a turning tool (`body`, tooling::turningBody, convex and across
/// the axis as above) touch the stock while `insert` cuts it, from `stock` on: kind Holder,
/// position (radius, 0, z) of a point of the first contact.
std::vector<Collision> checkTurningBody(const toolpath::Toolpath& path,
                                        const geom2d::Polyline& insert,
                                        const std::vector<geom2d::Polyline>& body, LatheStock stock,
                                        std::size_t maxCollisions = 200,
                                        CancelToken* cancel = nullptr);

/// The stock as dexels, threads and holes included (from its mesh; without them, ray by ray from
/// the profile). A thread finer than the resolution does not resolve.
Stock latheStockDexels(const LatheStock& stock, double resolution, bool triDexel = true);

/// Takes off `stock` whatever is not in the turned stock: what turning does to milled dexels.
void clipToLathe(Stock& stock, const LatheStock& lathe);

/// True when a toolpath leaves the spindle's axis plane or its tool lies off the axis: it cuts
/// holes rather than sweeping an area in the lathe plane. How far a bar-pull toolpath has moved the
/// bar by move `end` (moves before it; DL-0091): the tool's travel along Z while the chuck is open,
/// with a share `t` of move `end` itself.
double barPulledBy(const toolpath::Toolpath& path, std::size_t end, double t = 0.0);

bool offTheAxis(const toolpath::Toolpath& path);

/// Convex hull of points (counter-clockwise, no repeated first point).
std::vector<Vec2> convexHull(std::vector<Vec2> points);

} // namespace ocam::sim
