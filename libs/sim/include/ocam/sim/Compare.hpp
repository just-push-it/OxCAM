// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Part comparison and display: material left over (excess) and part material cut away
// (gouge) per Z ray, and a coloured height-field mesh of the stock for the viewer.

#pragma once

#include "ocam/sim/Dexel.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace ocam::sim {

struct CompareOptions {
    /// Deviations up to this size count as "at size" [mm]. The horizontal tolerance is at
    /// least one ray spacing, so single-ray slivers along tessellated walls are not reported.
    double tolerance = 0.02;
};

/// Per Z ray of the stock grid (index j * nu + i).
struct Comparison {
    int nu = 0;
    int nv = 0;
    double tolerance = 0.0;
    std::vector<float> excess; ///< Stock outside the part, along the ray [mm] (0 = at size).
    std::vector<float> gouge;  ///< Part material missing beyond the tolerance [mm].
    /// Where along a gouged ray the part material is missing (lowest and highest Z of it; 0 on
    /// rays without a gouge): the display colours only the surface there, not the whole ray.
    std::vector<float> gougeLo;
    std::vector<float> gougeHi;
    double excessVolume = 0.0; ///< Stock beyond the tolerance outside the part [mm^3].
    double gougeVolume = 0.0;  ///< Part material removed beyond the tolerance [mm^3].
    double maxExcess = 0.0;
    double maxGouge = 0.0;
    std::size_t gougeRays = 0;
    std::size_t excessRays = 0;

    bool hasGouges() const noexcept { return gougeRays > 0; }
};

/// The part rasterised on a stock's Z grid with the tolerance applied: everything
/// compareToPart needs from the part, prepared once for repeated comparisons (playback).
struct PartReference {
    double u0 = 0.0;
    double v0 = 0.0;
    double spacing = 0.0;
    int nu = 0;
    int nv = 0;
    double tolerance = 0.0;
    /// Per Z ray (index j * nu + i): the part, the part eroded by the tolerance (material that
    /// must remain) and dilated by it (where stock may remain).
    std::vector<IntervalList> part;
    std::vector<IntervalList> eroded;
    std::vector<IntervalList> dilated;

    /// True when prepared on this grid (same origin, spacing and size).
    bool matches(const DexelGrid& grid) const noexcept;
};

/// Rasterises a closed part mesh (same coordinates as the stock) on the stock's Z grid.
PartReference preparePart(const Stock& stock, const geom3d::Mesh& part,
                          const CompareOptions& options = {});

/// Compares the stock's Z grid with a closed part mesh (same coordinates as the stock).
Comparison compareToPart(const Stock& stock, const geom3d::Mesh& part,
                         const CompareOptions& options = {});

/// Compares the stock with a prepared part. Throws Error(InvalidArgument) when `part` was
/// prepared on a different grid.
Comparison compareToPart(const Stock& stock, const PartReference& part);

using Rgb = std::array<std::uint8_t, 3>;

inline constexpr Rgb kStockColour{196, 178, 150};

/// Colour scale of the comparison map: gouge red, at size green, a little excess yellow through
/// orange, and excess of `excessScale` or more in the plain `stock` colour (uncut material reads
/// as stock rather than as a saturated colour).
Rgb comparisonColour(double excess, double gouge, double tolerance, double excessScale,
                     const Rgb& stock = kStockColour);

/// A distinct colour for the material operation `index` (0-based) cut; green and red are left
/// to "at size" and gouges. Repeats after eight operations.
Rgb operationColour(int index);

/// Per Z ray (j * nu + i): 1 where `after` holds different material than `before` (with
/// `topOnly`, where its top moved: the surface seen from above was cut), else 0. Empty when the
/// stocks do not share a grid.
std::vector<std::uint8_t> changedRays(const Stock& before, const Stock& after,
                                      bool topOnly = false);

/// Triangle mesh for display with flat normals and per-vertex colours.
struct DisplayMesh {
    std::vector<std::array<float, 3>> positions;
    std::vector<std::array<float, 3>> normals;
    std::vector<Rgb> colours;
    std::vector<std::uint32_t> indices;

    std::size_t triangleCount() const noexcept { return indices.size() / 3; }
};

struct DisplayOptions {
    int maxCells = 400;       ///< Rays are merged into at most this many cells per side.
    double excessScale = 2.0; ///< Excess shown fully blue [mm].
    /// Per Z ray: 0 = top not cut, k = top last cut by operation k - 1. A cell's top shows the
    /// operation that cut it, its walls the operation that cut the neighbour the wall faces
    /// (which exposed it); uncut surfaces show the stock colour. With a comparison, surfaces at
    /// size stay green, however much stock their ray has left elsewhere, and gouges red.
    const std::vector<std::uint8_t>* cutBy = nullptr;
    Rgb stockColour = kStockColour; ///< Uncut material.
};

/// The boundary of all three grids as a smooth surface (Surface.hpp, dual contouring on the lattice
/// of every maxCells-th ray), coloured by `comparison` when given.
DisplayMesh buildDisplayMesh(const Stock& stock, const Comparison* comparison = nullptr,
                             const DisplayOptions& options = {});

/// The display mesh of a stock that changes a little at a time (playback), split into chunks of
/// kChunk x kChunk cells. After the first update, an update looks only at the dexel tiles that
/// changed since the previous one (stocks copied from each other share untouched tiles) and
/// rebuilds only the chunks over them, so its cost follows the material cut in between.
class StockDisplay {
public:
    static constexpr int kChunk = 32;

    /// `options.cutBy` is ignored: the cut marks are kept here.
    explicit StockDisplay(const DisplayOptions& options = {});
    ~StockDisplay();
    StockDisplay(const StockDisplay&) = delete;
    StockDisplay& operator=(const StockDisplay&) = delete;

    /// Shows `stock`. With `part`, material at size is green and gouges red. Rays that changed
    /// since the previous update are marked as cut by operation `cutting` (0-based; -1 = none).
    /// `restart` (a new starting point: another operation, a jump back) rebuilds everything
    /// with `cutBy` as the cut marks (see DisplayOptions::cutBy); so does another grid or part.
    /// With `starts` (the stocks before operations 0 .. cutting) the display colours
    /// each surface point by the operation that cut it (CutColours) instead of by the cut marks.
    void update(const Stock& stock, const PartReference* part, int cutting, bool restart,
                const std::vector<std::uint8_t>* cutBy = nullptr,
                const std::vector<const Stock*>* starts = nullptr);

    /// The chunk meshes (null or empty where there is no material). Unchanged chunks keep
    /// their pointers between updates.
    const std::vector<std::shared_ptr<const DisplayMesh>>& chunks() const noexcept {
        return chunks_;
    }
    /// Totals of the comparison with the part of the last update (no per-ray arrays).
    std::optional<Comparison> comparison() const;

private:
    struct State;
    DisplayOptions options_;
    std::unique_ptr<State> state_;
    std::vector<std::shared_ptr<const DisplayMesh>> chunks_;
};

} // namespace ocam::sim
