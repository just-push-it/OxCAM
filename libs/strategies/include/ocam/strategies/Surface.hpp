// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// 3D milling (phase 7): drop-cutter positioning of a rotating tool on a triangle mesh, the
// cutter-location height field built from it, and the strategies that use them: parallel
// (raster) finishing, waterline (Z-level) finishing and Z-level roughing. All geometry is in WCS
// coordinates with the tool axis +Z. See docs/design-log/0016 to 0018.

#pragma once

#include "ocam/strategies/Strategies.hpp"

#include <ocam/geom3d/Mesh.hpp>

#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace ocam::strategies {

/// The lower surface of a rotating tool, as needed by drop-cutter: the height lo(d) of the tool
/// surface above its tip at radial distance d. Supported shapes are the torus family (flat,
/// bull-nose and ball end mills: corner radius 0..R) and cones with a flat tip (chamfer mills,
/// spot drills, drills).
class Cutter {
public:
    enum class Kind { Torus, Cone };

    /// Throws Error(Unsupported) for tool types that are not flat, ball, bull-nose or conical.
    explicit Cutter(const tooling::Tool& tool);
    static Cutter torus(double radius, double cornerRadius);
    static Cutter cone(double radius, double tipRadius, double halfAngle);

    Kind kind() const noexcept { return kind_; }
    double radius() const noexcept { return radius_; }
    double cornerRadius() const noexcept { return corner_; }
    /// Largest radius of the flat bottom (lo = 0).
    double flatRadius() const noexcept;
    /// Height of the lower surface above the tip at radial distance d in [0, radius].
    double lo(double d) const noexcept;
    /// Contact with a plane of slope s (|gradient|): the r in [0, radius] maximizing
    /// s * r - lo(r), and that maximum.
    std::pair<double, double> facetContact(double slope) const noexcept;
    /// The tool grown by `amount` (stock to leave): radius and corner radius grow, and the tip
    /// moves down by `amount`. Torus tools only.
    Cutter offset(double amount) const;

private:
    Cutter() = default;
    Kind kind_ = Kind::Torus;
    double radius_ = 0.0;
    double corner_ = 0.0;    ///< Torus: corner radius.
    double tipRadius_ = 0.0; ///< Cone.
    double tanHalf_ = 1.0;   ///< Cone: tan(half point angle).
};

/// Drop-cutter: the lowest tip height at an XY position where the tool touches no triangle of
/// the mesh. Exact for each triangle (vertices, the facet plane and edges); thread-safe.
class DropCutter {
public:
    DropCutter(const geom3d::Mesh& mesh, const Cutter& cutter);
    ~DropCutter();
    DropCutter(DropCutter&&) noexcept;
    DropCutter& operator=(DropCutter&&) noexcept;

    /// max(floor, highest contact) for the tool axis at `xy`.
    double drop(const Vec2& xy, double floor) const;
    const Cutter& cutter() const noexcept;
    const geom3d::Aabb3& bounds() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// Drop-cutter heights on a regular XY grid (the cutter-location surface), computed in
/// parallel. Point (i, j) is at origin + (i, j) * spacing.
struct HeightField {
    Vec2 origin = Vec2::Zero();
    double spacing = 1.0;
    int nx = 0;
    int ny = 0;
    std::vector<double> z;

    double at(int i, int j) const {
        return z[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) +
                 static_cast<std::size_t>(i)];
    }
    Vec2 point(int i, int j) const { return origin + Vec2(i, j) * spacing; }
};

HeightField computeHeightField(const DropCutter& dc, const geom2d::Aabb2& area, double spacing,
                               double floor, CancelToken& cancel);

/// Tool-centre positions where the tool tip at height z would cut into the part: the closed
/// region {drop > z}, traced on the height field with crossings refined on the exact
/// drop-cutter (to `tolerance`). Outer boundaries counter-clockwise, holes clockwise.
geom2d::Regions forbiddenRegions(const DropCutter& dc, const HeightField& field, double z,
                                 double floor, double tolerance);

// -- Parallel finishing -----------------------------------------------------------------------

struct ParallelParams {
    double stepover = 0.0;       ///< [mm]; 0 = from the scallop height.
    double scallopHeight = 0.01; ///< Target cusp height on flat areas [mm].
    double angle = 0.0;          ///< Raster direction from +X [rad].
    double sampleStep = 0.0;     ///< Spacing of drop-cutter samples along a line; 0 = automatic.
    double stockToLeave = 0.0;
    bool zigzag = true;
    /// Cut each area the lines cross once (between the boundary's turns) through all its lines
    /// before the next, the nearest first, instead of line by line across the whole boundary.
    bool byArea = false;
    /// Tool-centre area to cover; empty = the part's XY bounds.
    geom2d::Regions boundary;
};

/// Raster finishing: parallel lines whose points are dropped onto the part.
toolpath::Toolpath parallelFinish(const geom3d::Mesh& part, double floorZ,
                                  const ParallelParams& params, const Context& ctx);

// -- Waterline finishing ----------------------------------------------------------------------

struct WaterlineParams {
    double stepDown = 0.5;
    double stockToLeave = 0.0;
    double resolution = 0.0; ///< Height-field spacing; 0 = automatic.
    double topZ = 0.0;       ///< Levels from below topZ...
    double bottomZ = 0.0;    ///< ...down to bottomZ (inclusive).
    CutDirection direction = CutDirection::Climb;
    /// Tool-centre containment: the passes are clipped to it; empty = everywhere.
    geom2d::Regions boundary;
};

/// Constant-Z contours around the part at each level (steep-wall finishing).
toolpath::Toolpath waterline(const geom3d::Mesh& part, const WaterlineParams& params,
                             const Context& ctx);

struct ContourFinishParams {
    double scallopHeight = 0.01; ///< Cusp left between passes [mm].
    double maxStepDown = 0.0;    ///< Cap on the step down [mm]; 0 = as the cusp allows.
    /// Walls shallower than this [rad] do not set the step (flat and parallel finishing's).
    double minSlope = 5.0 * kPi / 180.0;
    double stockToLeave = 0.0;
    double resolution = 0.0; ///< Height-field spacing; 0 = automatic.
    double topZ = 0.0;
    double bottomZ = 0.0;
    CutDirection direction = CutDirection::Climb;
    /// Tool-centre containment (selected faces, rest stock): the passes are clipped to it;
    /// empty = everywhere.
    geom2d::Regions boundary;
};

/// Constant-Z contours around the part, the levels spaced by the wall's slope so the cusp
/// between passes stays within `scallopHeight` (curved blends get as many levels as they need,
/// steep walls few), linked at depth from one level down to the next (DL-0033).
toolpath::Toolpath contourFinish(const geom3d::Mesh& part, const ContourFinishParams& params,
                                 const Context& ctx);

// -- Z-level roughing -------------------------------------------------------------------------

struct ZLevelRoughParams {
    double stepDown = 1.0;
    double stepoverRatio = 0.4;
    double stockToLeaveXY = 0.3;
    double stockToLeaveZ = 0.3;
    double resolution = 0.0; ///< Height-field spacing; 0 = automatic.
    EntryKind entry = EntryKind::Helix;
    double rampAngle = 2.0 * kPi / 180.0;
    std::optional<double> helixRadius; ///< [mm]; unset: kHelixRadiusShare r.
    CutDirection direction = CutDirection::Climb;
    /// XY regions holding stock material between z and zAbove (+infinity for the first level).
    /// Unset: the stock outline at every level. A simulated in-process stock makes this 3D rest
    /// roughing (DL-0018).
    std::function<geom2d::Regions(double z, double zAbove)> material;
    /// With `material`: material within this distance outside the part and its allowance is
    /// ignored (the in-process stock's ray spacing: its cells reach past the true surface).
    double materialSlack = 0.0;
    /// Spaces the levels so one lands on each flat area (its height plus the Z allowance), so
    /// flats keep just the allowance.
    bool flatAreaLevels = true;
    /// Cut each area (the pieces of material standing over one another through the levels)
    /// through all its levels before the next, instead of level by level.
    bool byArea = false;
    /// Areas with air to start from (the outside of a part) on circles too, instead of traced
    /// passes from the air inward (DL-0031).
    bool medialOpenAreas = false;
    /// Fine levels this far apart (0: none) between the step-down levels, each taking only the
    /// band next to sloped and rounded walls that the level under it cannot reach, so the stairs
    /// left there are at most about this tall. (The operation's parameter defaults to 1.25 mm.)
    double fineStepDown = 0.0;
};

/// Roughing by levels, leaving stock-to-leave on the part, the cutter's engagement kept within
/// that of a cut one stepover wide (IP-0004): each closed area on the largest circles that fit
/// along its medial axis, joined by bi-tangents (MATHSM, DL-0031); open areas by traced passes
/// from the air inward (DL-0018) unless `medialOpenAreas`.
toolpath::Toolpath zLevelRough(const geom3d::Mesh& part, const geom2d::Regions& stockOutline,
                               double topZ, double bottomZ, const ZLevelRoughParams& params,
                               const Context& ctx);

// -- Flat area finishing ----------------------------------------------------------------------

/// The upward-facing flat areas of a part at one height.
struct FlatLevel {
    double z = 0.0;
    geom2d::Regions area; ///< XY outline of the flat faces at that height.
};

/// Flat areas of a mesh: triangles facing up and tilted at most `maxSlope` [rad] from
/// horizontal, grouped by height (within `zTolerance`) and united per height. Highest first.
std::vector<FlatLevel> flatLevels(const geom3d::Mesh& part, double maxSlope, double zTolerance,
                                  double chordTolerance);

struct FlatFinishParams {
    double stepover = 0.0;       ///< [mm], at most the flat bottom's diameter; 0 = 0.6 × that.
    double stockToLeaveXY = 0.0; ///< On the walls next to the flats.
    double stockToLeaveZ = 0.0;  ///< On the flats.
    double maxSlope = 0.5 * kPi / 180.0;
    double resolution = 0.0; ///< Height-field spacing; 0 = automatic.
    EntryKind entry = EntryKind::Ramp;
    double rampAngle = 2.0 * kPi / 180.0;
    CutDirection direction = CutDirection::Climb;
    /// The stock's XY outline: flats open to the side are entered by a plunge outside it and a
    /// feed in. Empty: every flat is entered from above (`entry`).
    geom2d::Regions stockOutline;
    /// Openings in a flat (holes, slots, pockets: where its floor drops away) at most this wide
    /// (the largest circle that fits in them) are machined over as if the flat went on across
    /// them; wider ones are gone around. 0: every opening is gone around.
    double bridgeOpenings = 0.0;
    /// Only these flats (the selected faces' plan view): the flat areas are cut to it; empty =
    /// every flat in the height range.
    geom2d::Regions only;
};

/// Finishes every flat area of the part between bottomZ and topZ: at the height of each one,
/// contour-parallel loops over the tool positions that reach it without touching the part.
/// Needs a tool with a flat bottom (flat or bull-nose end mill).
toolpath::Toolpath flatFinish(const geom3d::Mesh& part, double topZ, double bottomZ,
                              const FlatFinishParams& params, const Context& ctx);

// -- Pencil finishing -------------------------------------------------------------------------

struct PencilParams {
    /// Radius of the reference tool: the passes go where it touches the part at two points
    /// (corners and fillets tighter than it). 0 = the tool's own radius.
    double referenceRadius = 0.0;
    /// Least angle between the reference tool's two contact normals for a corner [rad].
    double cornerAngle = 20.0 * kPi / 180.0;
    int offsetPasses = 0;  ///< Extra passes each side of the pencil pass.
    double stepover = 0.0; ///< Between the offset passes, along the surface [mm].
    double stockToLeave = 0.0;
    double resolution = 0.0; ///< Height-field spacing; 0 = automatic.
    /// Tool-centre containment (selected faces, rest stock); empty = everywhere.
    geom2d::Regions boundary;
};

/// The concave corners of the part, found as creases in the reference tool's cutter-location
/// surface (the reference tool touches two faces at once): each crease as a chain of plan
/// points with the reference tool's two contact slopes across it.
struct PencilCorner {
    std::vector<Vec2> points; ///< Plan positions of the reference tool on the crease.
    bool closed = false;
};
std::vector<PencilCorner> pencilCorners(const DropCutter& reference, const geom2d::Aabb2& area,
                                        double spacing, double floor, double cornerAngle,
                                        CancelToken& cancel);

/// Single passes along the concave corners and fillets that the reference tool cannot reach
/// into, each placed where the tool's contact normal halves the reference tool's two contact
/// normals; optional offset passes either side along the surface, out to where the reference
/// tool touched (DL-0080).
toolpath::Toolpath pencilFinish(const geom3d::Mesh& part, const PencilParams& params,
                                const Context& ctx);

// -- Steep and shallow finishing --------------------------------------------------------------

struct SteepShallowParams {
    double thresholdAngle = 40.0 * kPi / 180.0; ///< Steeper than this: constant-Z passes [rad].
    double overlap = 0.5; ///< Width of the band both kinds of passes cover [mm].
    /// Steep areas: constant-Z levels `stepDown` apart, or spaced by the wall's slope to leave
    /// `steepScallop` when that is set (as contour finishing, DL-0033).
    double stepDown = 0.0;
    double steepScallop = 0.0;
    /// Shallow areas: parallel passes `stepover` apart (0: from `shallowScallop`).
    double stepover = 0.0;
    double shallowScallop = 0.0;
    double angle = 0.0; ///< Direction of the parallel passes from +X [rad].
    double sampleStep = 0.0;
    bool zigzag = true;
    double stockToLeave = 0.0;
    double resolution = 0.0;
    double topZ = 0.0;
    double bottomZ = 0.0;
    CutDirection direction = CutDirection::Climb;
    bool steepFirst = true;
    /// Tool-centre containment (selected faces, rest stock); empty = everywhere.
    geom2d::Regions boundary;
};

/// The tool-centre areas where the part under the tool is shallower than `thresholdAngle`
/// (the slope of the cutter-location surface), within `area`.
geom2d::Regions shallowAreas(const DropCutter& dc, const geom2d::Aabb2& area, double spacing,
                             double floor, double thresholdAngle, double chordTolerance,
                             CancelToken& cancel);

/// One operation over the whole surface: constant-Z passes (waterline, or contour finishing by
/// cusp) where it is steeper than the threshold, parallel passes where it is shallower, both
/// reaching half the overlap past the line between them (DL-0080).
toolpath::Toolpath steepShallowFinish(const geom3d::Mesh& part, const SteepShallowParams& params,
                                      const Context& ctx);

// -- Parameter objects ------------------------------------------------------------------------

/// Step downs and stepovers come from the preset (presetStepDown, presetStepover); the
/// finishing stepover from a scallop height when one is set.
ParallelParams parallelParams(const Json& values, const tooling::Tool& tool,
                              const tooling::CuttingPreset* preset);
WaterlineParams waterlineParams(const Json& values, const tooling::CuttingPreset* preset);
ContourFinishParams contourFinishParams(const Json& values);
ZLevelRoughParams zLevelRoughParams(const Json& values, const tooling::Tool& tool,
                                    const tooling::CuttingPreset* preset);
FlatFinishParams flatFinishParams(const Json& values, const tooling::Tool& tool,
                                  const tooling::CuttingPreset* preset);
/// The stepover (for offset passes only) from the scallop height, else the preset's ae.
PencilParams pencilParams(const Json& values, const tooling::Tool& tool,
                          const tooling::CuttingPreset* preset);
/// The step down from the preset's ap unless a steep cusp height is set; the stepover from the
/// preset's ae unless a shallow scallop height is set.
SteepShallowParams steepShallowParams(const Json& values, const tooling::Tool& tool,
                                      const tooling::CuttingPreset* preset);

} // namespace ocam::strategies
