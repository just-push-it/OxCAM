// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Dexel grids: parallel rays through the stock, each holding the sorted, disjoint intervals of
// material along it. A tri-dexel stock uses three grids (rays along X, Y and Z). Grids are split
// into tiles held by shared pointers so snapshots are cheap and copy-on-write.
// See docs/design-log/0012.

#pragma once

#include <ocam/foundation/Math.hpp>
#include <ocam/geom2d/Region.hpp>
#include <ocam/geom3d/Bounds.hpp>
#include <ocam/geom3d/Mesh.hpp>

#include <array>
#include <memory>
#include <vector>

namespace ocam::sim {

struct StockCodec;

struct Interval {
    float lo = 0.0f;
    float hi = 0.0f;
    bool operator==(const Interval&) const = default;
};

/// Sorted, disjoint material intervals along one ray.
using IntervalList = std::vector<Interval>;

/// Removes [lo, hi] from the list. Returns true if anything changed.
bool subtract(IntervalList& list, double lo, double hi);
/// Total length of the intervals.
double totalLength(const IntervalList& list);
/// Length of the part of `list` overlapping [lo, hi].
double overlapLength(const IntervalList& list, double lo, double hi);
/// Highest material coordinate, or -infinity when empty.
double topOf(const IntervalList& list);
/// Length of a minus b (both sorted and disjoint).
double differenceLength(const IntervalList& a, const IntervalList& b);

/// Intersection, union and offset of interval lists (results sorted and disjoint). `grow`
/// widens every interval by `amount` at both ends (shrinks for negative amounts).
IntervalList intersect(const IntervalList& a, const IntervalList& b);
IntervalList unite(const IntervalList& a, const IntervalList& b);
IntervalList grow(const IntervalList& list, double amount);

/// Rays parallel to `axis` (0 = X, 1 = Y, 2 = Z) on a regular grid over the other two axes
/// (u = axis+1, v = axis+2, cyclic). Ray (i, j) passes through u = u0 + (i + 0.5) h,
/// v = v0 + (j + 0.5) h.
class DexelGrid {
public:
    static constexpr int kTile = 16;

    DexelGrid() = default;
    DexelGrid(int axis, double u0, double v0, double spacing, int nu, int nv);

    int axis() const noexcept { return axis_; }
    int uAxis() const noexcept { return (axis_ + 1) % 3; }
    int vAxis() const noexcept { return (axis_ + 2) % 3; }
    double spacing() const noexcept { return h_; }
    int nu() const noexcept { return nu_; }
    int nv() const noexcept { return nv_; }
    double u(int i) const noexcept { return u0_ + (i + 0.5) * h_; }
    double v(int j) const noexcept { return v0_ + (j + 0.5) * h_; }
    double u0() const noexcept { return u0_; }
    double v0() const noexcept { return v0_; }

    /// Index range of rays whose coordinate lies in [a, b] along u (or v), clamped.
    std::pair<int, int> uRange(double a, double b) const noexcept;
    std::pair<int, int> vRange(double a, double b) const noexcept;

    const IntervalList& ray(int i, int j) const;
    /// Mutable access; clones the containing tile when it is shared with a snapshot.
    IntervalList& mutableRay(int i, int j);

    /// Makes the tiles covering rays [i0, i1] x [j0, j1] exclusive to this grid (clones shared
    /// tiles), so rays inside can then be edited concurrently from different threads.
    void detach(int i0, int i1, int j0, int j1);
    int tileRows() const noexcept { return tilesV_; }
    int tileColumns() const noexcept { return tilesU_; }
    /// Identity of a tile's storage: two grids (one copied from the other) give the same value
    /// exactly when neither has edited that tile since.
    const void* tileId(int ti, int tj) const noexcept {
        return tiles_[static_cast<std::size_t>(tj) * static_cast<std::size_t>(tilesU_) +
                      static_cast<std::size_t>(ti)]
            .get();
    }

    /// Sum over rays of material length times the cell area.
    double volume() const;

    bool isEmpty() const noexcept { return nu_ == 0 || nv_ == 0; }

    /// Fills the grid from a closed, outward-oriented triangle mesh (rasterization along the
    /// ray axis; top-left fill rule so shared edges are counted once).
    void fillFromMesh(const geom3d::Mesh& mesh);

private:
    friend struct StockCodec; // the local simulation cache (StockCodec.hpp)
    struct Tile {
        std::array<IntervalList, kTile * kTile> rays;
    };
    std::size_t tileIndex(int i, int j) const noexcept {
        return static_cast<std::size_t>(j / kTile) * static_cast<std::size_t>(tilesU_) +
               static_cast<std::size_t>(i / kTile);
    }
    static std::size_t local(int i, int j) noexcept {
        return static_cast<std::size_t>(j % kTile) * kTile + static_cast<std::size_t>(i % kTile);
    }

    int axis_ = 2;
    double u0_ = 0.0, v0_ = 0.0, h_ = 1.0;
    int nu_ = 0, nv_ = 0, tilesU_ = 0, tilesV_ = 0;
    std::vector<std::shared_ptr<Tile>> tiles_;
};

/// Stock as three dexel grids. The Z grid is authoritative for volume and display; the X and Y
/// grids capture vertical walls and undercuts (tri-dexel). Copying a stock is cheap (tiles are
/// shared) and later edits clone only the tiles they touch.
class Stock {
public:
    Stock() = default;

    /// Builds the grids over `bounds` (expanded by one cell) at `resolution` and fills them from
    /// a closed mesh. `triDexel` false keeps only the Z grid (faster, 2.5D only).
    static Stock fromMesh(const geom3d::Mesh& mesh, const geom3d::Aabb3& bounds, double resolution,
                          bool triDexel = true);
    static Stock box(const geom3d::Aabb3& box, double resolution, bool triDexel = true);
    /// The solid of revolution of (u = Z, v = radius >= 0) regions about the Z axis, filled
    /// exactly ray by ray (one-way conversion of an axisymmetric stock, see Lathe.hpp).
    static Stock fromRevolvedProfile(const geom2d::Regions& profile, double resolution,
                                     bool triDexel = true);

    const DexelGrid& grid(int axis) const { return grids_[static_cast<std::size_t>(axis)]; }
    DexelGrid& grid(int axis) { return grids_[static_cast<std::size_t>(axis)]; }
    bool triDexel() const noexcept { return triDexel_; }
    double resolution() const noexcept { return resolution_; }
    const geom3d::Aabb3& bounds() const noexcept { return bounds_; }

    /// Material volume [mm^3] from the Z grid.
    double volume() const { return grids_[2].volume(); }

private:
    friend struct StockCodec;
    std::array<DexelGrid, 3> grids_;
    geom3d::Aabb3 bounds_;
    double resolution_ = 1.0;
    bool triDexel_ = true;
};

/// Closed mesh of an axis-aligned box (outward triangles).
geom3d::Mesh boxMesh(const geom3d::Aabb3& box);

} // namespace ocam::sim
