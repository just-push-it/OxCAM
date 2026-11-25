// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Smooth stock surfaces for display. The three dexel grids of a stock share one lattice: the
// rays along X, Y and Z pass through the same points, so every lattice edge that the stock's
// boundary crosses has its exact crossing on a ray. Dual contouring places one vertex in each
// lattice cube the boundary passes through, from the crossings on the cube's edges and their
// normals, and joins the four cubes around each crossed edge with a quad: a closed mesh that
// follows sloped and curved surfaces and keeps sharp edges. A turned stock's exact half profile
// is revolved instead. See docs/design-log/0040.

#pragma once

#include "ocam/sim/Compare.hpp"

#include <ocam/geom2d/Region.hpp>

#include <functional>
#include <optional>
#include <vector>

namespace ocam::sim {

/// Every `stride`-th ray of a stock's grids as lattice points (i, j, k) at
/// origin + (i, j, k) * step. The lattice reaches one point past the grids on each axis; points
/// there hold no material, so the boundary closes.
struct SurfaceLattice {
    int stride = 1;
    int ni = 0; ///< Lattice points along X.
    int nj = 0; ///< Along Y.
    int nk = 0; ///< Along Z.
    Vec3 origin = Vec3::Zero();
    double step = 1.0; ///< Lattice spacing: the stride times the ray spacing.

    bool isEmpty() const noexcept { return ni < 2 || nj < 2 || nk < 2; }
};

/// The lattice with at most `maxCells` cubes along X and along Y (every ray when they fit).
SurfaceLattice surfaceLattice(const Stock& stock, int maxCells);

struct SurfaceOptions {
    /// Faces meeting at a larger angle are shaded apart (a sharp edge) [rad].
    double creaseAngle = 0.6;
    /// Place vertices on sharp edges and corners from the crossings' normals (dual contouring).
    /// False takes the mean of the crossings: smooth everywhere, edges rounded within a cube.
    bool sharpFeatures = true;
};

/// Colour of a surface point with its (unit, outward) normal.
using SurfaceColour = std::function<Rgb(const Vec3& position, const Vec3& normal)>;

/// Appends to `out` the faces of the stock's boundary across the lattice edges whose lower end
/// is a lattice point (i, j, k) with i in [i0, i1] and j in [j0, j1] (any k). Calls over ranges
/// that tile the lattice's (i, j) points give one closed mesh without seams; vertices on a
/// range's border are repeated in each piece.
void contourStock(const Stock& stock, const SurfaceLattice& lattice, int i0, int i1, int j0, int j1,
                  const SurfaceColour& colour, const SurfaceOptions& options, DisplayMesh& out);

/// The stock's whole boundary (contourStock over the lattice), coloured `colour` or in the plain
/// stock colour.
DisplayMesh buildSurfaceMesh(const Stock& stock, int maxCells = 400,
                             const SurfaceColour& colour = {}, const SurfaceOptions& options = {});

/// `source` moved by `sourceToTarget` and sampled again on the grids of `target`: the source's
/// boundary at every ray (buildSurfaceMesh with sharp features) filled into `target`'s lattice,
/// each ray kept within `target`'s material. Used to start a setup from the stock an earlier
/// setup left (its grid turned with the part). Sampling the surface again can move it by up to
/// about a cell.
Stock resampleStock(const Stock& source, const Transform3& sourceToTarget, const Stock& target);

/// Which operation cut each point of a stock's boundary, from the stocks before each operation
/// (`starts[k]` before operation k, all on the stock's grids): the boundary crossing on the ray
/// nearest the point along its normal's main axis is looked up in the earlier stocks, and the
/// last operation whose starting stock lacks it cut it. Walls get the colour of the cut that
/// made them, not of the neighbouring ray's top.
class CutColours {
public:
    CutColours(const Stock& stock, std::vector<const Stock*> starts, const Rgb& uncut);
    /// operationColour(k) for operation k, `uncut` for boundary the first stock already had,
    /// nullopt when no crossing is near the point.
    std::optional<Rgb> operator()(const Vec3& p, const Vec3& n) const;

private:
    const Stock* stock_ = nullptr;
    std::vector<const Stock*> starts_;
    Rgb uncut_{};
};

/// Colour of a point of a turning profile (u = Z, v = radius) with its outward normal there.
using ProfileColour = std::function<Rgb(const Vec2& point, const Vec2& normal)>;

/// The surface of revolution of a turning stock's half profile (u = Z, v = radius >= 0) about
/// the Z axis, within `chordTolerance` of the true circles. Profile corners sharper than
/// `creaseAngle` are shaded as edges. Each profile point keeps its colour all round; with a
/// colour, the profile is coloured in pieces of at most `colourStep` along it.
DisplayMesh revolvedMesh(const geom2d::Regions& profile, double chordTolerance,
                         const ProfileColour& colour, double creaseAngle = 0.6,
                         double colourStep = 0.25);
DisplayMesh revolvedMesh(const geom2d::Regions& profile, double chordTolerance,
                         const Rgb& colour = kStockColour, double creaseAngle = 0.6);

/// The same from the profile's loops as points (material on the left), leaving out the edges
/// `skip` says to.
DisplayMesh revolvedLoops(const std::vector<std::vector<Vec2>>& loops, double chordTolerance,
                          const ProfileColour& colour, double creaseAngle, double colourStep,
                          const std::function<bool(const Vec2&, const Vec2&)>& skip);

class LatheStock;
/// The colour of a cut's surface: a hole's or a thread's, by its index among them.
using LatheCutColour = std::function<Rgb(bool hole, int index)>;
/// A turning stock's whole surface: its profile revolved as in revolvedMesh (exactly that when it
/// has no cuts), threads swept along their helix over the profile's edges, holes punched through
/// with their walls added.
DisplayMesh latheStockMesh(const LatheStock& stock, double chordTolerance,
                           const ProfileColour& colour, const LatheCutColour& cutColour,
                           double creaseAngle = 0.6, double colourStep = 0.25);

/// Signed distance from points to the boundary of planar regions (negative inside), for
/// colouring a turning profile by its deviation from the part's or by the operation that cut it.
class ProfileDistance {
public:
    ProfileDistance() = default;
    explicit ProfileDistance(const geom2d::Regions& regions, double chordTolerance = 1e-3);
    /// A closed mesh turned about the Z axis (a turned part) cut by the plane through the axis,
    /// as (u = z, v = x): its exact half profile, bores included.
    static ProfileDistance section(const geom3d::Mesh& mesh);
    double operator()(const Vec2& p) const;
    bool isEmpty() const noexcept { return segments_.empty(); }

private:
    std::vector<std::pair<Vec2, Vec2>> segments_;
};

} // namespace ocam::sim
