// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Clearing one Z level of 3D roughing with the cutter's engagement kept within a limit
// (DL-0018, IP-0004). Passes follow the material left after the passes before them, one
// stepover in; wherever the material would wrap around the tool more than a cut one stepover
// wide does (inside corners), the pass pulls back toward the cleared side until the engagement
// angle, measured on a grid of the level's material, is within the limit.

#pragma once

#include "ocam/strategies/Strategies.hpp"

#include <ocam/geom2d/Bounds.hpp>
#include <ocam/geom2d/Region.hpp>
#include <ocam/toolpath/PathBuilder.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace ocam::strategies::detail {

/// A boolean grid over a rectangle (cell centres at origin + (i + 1/2, j + 1/2) * cell).
class CellGrid {
public:
    CellGrid(const geom2d::Aabb2& box, double cell);

    /// Sets every cell whose centre lies in the regions to `value`.
    void fill(const geom2d::Regions& regions, bool value, double chordTolerance);
    /// False outside the grid.
    bool at(const Vec2& p) const;
    double cell() const noexcept { return cell_; }

    /// The part of the circle of `radius` around `centre` ahead of the motion (the half facing
    /// `direction`, a unit vector) on set cells, as an angle [rad]: for remaining material, the
    /// cutter's engagement (contact) angle. Sampled half a cell outside the radius, so material
    /// the tool's own sweep cleared does not show at the sides.
    double contactAngle(const Vec2& centre, double radius, const Vec2& direction) const;
    /// True when the circle of `radius` around `centre` (sampled half a cell outside it), or
    /// the centre, is on a set cell: the tool there touches material.
    bool touches(const Vec2& centre, double radius) const;
    /// True when a set cell's centre lies within `radius` of `centre`: the disk holds material.
    bool anyWithin(const Vec2& centre, double radius) const;
    /// Clears the cells within `radius` of `centre`, appending their indices to `cleared` (an
    /// undo log) when given.
    void clearDisk(const Vec2& centre, double radius,
                   std::vector<std::uint32_t>* cleared = nullptr);
    /// Sets the cells listed in an undo log again.
    void restore(const std::vector<std::uint32_t>& cells);

private:
    Vec2 origin_;
    double cell_;
    int nx_ = 0;
    int ny_ = 0;
    std::vector<std::uint8_t> bits_;
};

/// The engagement angle of a straight cut `width` wide with a tool of `radius`.
double engagementAngle(double width, double radius);

/// The clearing grid's cell for a tool radius: the tool's centre keeps half a cell off the
/// area's edge and the passes along the edge (walls) run a cell and a half inside it, so a
/// wall keeps the allowance plus up to that (0.225 mm at most).
double clearingCell(double toolRadius);

/// One connected piece of a level's allowed tool-centre area and what it needs to be cleared.
struct LevelClearing {
    geom2d::Region area;       ///< Allowed tool centres (clear of the part and its allowance).
    geom2d::Regions material;  ///< The level's material, in stock space.
    geom2d::Regions forbidden; ///< Tool centres that would touch the part or its allowance.
    geom2d::Regions air;       ///< Tool centres clear of all material at and above the level.
    geom2d::Regions overhead;  ///< Tool centres under material above the level (hops avoid).
    /// Tool centres outside the area that touch neither the part and its allowance nor any
    /// material at or above the level (around the part's outside): links may pass there.
    geom2d::Regions travel;
    /// Tool centres that touch material at or above the level that is not the clearing's to cut
    /// (left to a deeper level that runs later, say): not on its grid, so links keep out.
    geom2d::Regions standing;
    double toolRadius = 0.0;
    double stepover = 0.0;
    double z = 0.0;
    double approachZ = 0.0; ///< Hops within the area and feeds down start here.
    double retract = 0.0;   ///< Hops that leave the area go over this height.
    bool climb = true;
    EntryKind entry = EntryKind::Helix; ///< Into a closed area (no air to start from).
    double rampAngle = 0.0;
    std::optional<double> helixRadius; ///< [mm]; unset: kHelixRadiusShare r.
    /// Medial-axis clearing: open areas (air to start from) by circles too, instead of traced.
    bool medialOpenAreas = false;
    /// Medial-axis clearing: the lines across the circles' backs (out of the material) rise
    /// this far above the level and come back down where they meet material again; 0: at depth.
    double linkLift = 0.0;
};

/// The ways a level's clearing moves the tool from where it is to a clear point of the area
/// (an entry, a circle's centre), the lowest that is safe first (DL-0018, links). A way is
/// clear when the tool on it stays in the area or `travel`, out of `overhead` and `standing`,
/// and cuts nothing
/// (no material within its radius less a cell); the shortest clear way is straight or, around
/// what stands in between, a lattice path straightened:
/// 1. at depth (`Link`, at the cutting feed), when the clear way is at most `maxLink` long;
/// 2. else the feed clearance above the level's floor (a rapid), when the straight way is clear
///    with the feed clearance to the side as well: no material within the tool's radius plus
///    that, and that far off the forbidden area, whose material (the stairs over a sloped part
///    left to finer levels) the tool may graze at depth but a rapid must not (a way around
///    walls is fed, never rapid);
/// 3. else the approach height (the feed clearance over the top of the level's material),
///    straight within the area and `travel`, out of `overhead`;
/// 4. else over the retract height.
/// From 2 to 4, a rapid down to the feed clearance and a plunge feed of the rest; where the tool
/// at the end point comes within the feed clearance of the level's material to the side (an
/// entry just off it), which stands from the top of its column, the plunge feed starts at the
/// approach height.
class LevelLinks {
public:
    /// `material`: the level's material left (updated by the caller as it cuts); `box`, `cell`:
    /// its grid's. The tool goes in `in.area` and `in.travel` grown by `edge` (the clearing's
    /// own margin at the area's edge; negative: inside it), less `in.overhead` and
    /// `in.standing`.
    LevelLinks(const LevelClearing& in, const CellGrid& material, const geom2d::Aabb2& box,
               double cell, double edge, double maxLink, const Context& ctx);

    /// Moves the tool to `e` as above. Returns the way at depth (empty when it lifted).
    std::vector<Vec2> moveTo(toolpath::PathBuilder& b, const Vec2& e) const;
    /// Whether the tool can feed straight from `a` to `c` at depth cutting nothing; with
    /// `rapid`, whether it keeps the feed clearance from everything to the side as well.
    bool clearLine(const Vec2& a, const Vec2& c, bool rapid = false) const;
    /// The shortest way at depth from `a` to `c` cutting nothing, at most `budget` long, on a
    /// lattice a step apart, straightened; empty when there is none.
    std::vector<Vec2> clearWay(const Vec2& a, const Vec2& c, double budget) const;

private:
    bool clearAt(const Vec2& p) const;

    const LevelClearing& in_;
    const CellGrid& material_;
    CellGrid linkable_;
    CellGrid rapidable_; ///< As linkable_, the feed clearance off the forbidden area.
    geom2d::Regions hopArea_;
    double cell_;
    double step_;
    double maxLink_;
    const Context& ctx_;
};

struct ClearingReport {
    bool cutAnything = false;
    /// A closed area left uncut: neither a helix nor a ramp fits in it (a point inside).
    std::optional<Vec2> noEntry;
    int stalled = 0;            ///< Passes that removed nothing (material left).
    std::vector<Vec2> leftover; ///< A point in each piece of material left that way.
    double leftoverArea = 0.0;  ///< Their area [mm²].
};

/// Clears an open area (one with air to start from) from the air inward, pass by pass, each
/// pass at the stepover from the material left by the ones before, pulled back where the
/// engagement would exceed that of a cut one stepover wide. Closed areas are
/// `clearLevelMedial`'s.
ClearingReport clearLevel(toolpath::PathBuilder& b, const LevelClearing& in, const Context& ctx);

/// Clears the area on circles centred on its medial axis, each the largest that fits there,
/// successive circles a crescent of at most the stepover apart and joined by their external
/// bi-tangent (MATHSM, DL-0031); a helix down the largest circle starts a closed area. Open
/// areas are traced (`clearLevel`) unless `medialOpenAreas` is set.
ClearingReport clearLevelMedial(toolpath::PathBuilder& b, const LevelClearing& in,
                                const Context& ctx);

} // namespace ocam::strategies::detail
