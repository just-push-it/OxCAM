// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "ocam/toolpath/Toolpath.hpp"

#include <ocam/geom2d/Polyline.hpp>

namespace ocam::toolpath {

/// Appends moves to a Toolpath while tracking the tool position. Zero-length moves are dropped.
class PathBuilder {
public:
    /// Starts at `path.start`.
    explicit PathBuilder(Toolpath& path);

    const Vec3& position() const noexcept { return position_; }

    void rapid(const Vec3& target);
    void feed(const Vec3& target, FeedClass feedClass, double feedRate);
    void arc(const Vec3& target, const Vec2& center, bool clockwise, FeedClass feedClass,
             double feedRate);
    void dwell(double seconds);
    /// Spindle-synchronized linear move (thread cutting): `pitch` is the lead per revolution.
    void thread(const Vec3& target, double pitch, double feedRate);

    /// Rapid straight up to `z` if below it.
    void retractTo(double z);

    /// Moves to (xy, z) by rapid: first up to `safeZ` (if below), across, then down to `z`.
    void rapidAbove(const Vec2& xy, double safeZ, double z);

    /// Follows a polyline at height z (lines and bulge arcs). If the tool is not at the first
    /// vertex, a feed move of `feedClass` goes there first. Closed polylines return to the start.
    void follow(const geom2d::Polyline& path, double z, FeedClass feedClass, double feedRate);

    /// Follows a closed polyline while descending linearly with path length from the current z
    /// to `zEnd` at `rampAngle`, lap after lap; once at depth the current lap is finished flat,
    /// so the tool ends at the first vertex at `zEnd`. Used for ramp entry along a pocket loop.
    void rampAlong(const geom2d::Polyline& path, double zEnd, double rampAngle, double feedRate);

    /// Helical descent around `center` with radius `radius`, starting at angle 0 (center +
    /// (radius, 0)) at the current z, down to zEnd with at most `pitch` per revolution, then one
    /// full flat circle at zEnd. The tool must already be at the start point.
    void helix(const Vec2& center, double radius, double zEnd, double pitch, bool clockwise,
               double feedRate);

    /// Canned cycle at xy (the tool should already be at or above the R plane).
    void cycle(const Vec2& xy, const DrillCycle& cycle);

    /// A spindle probe's move of probing cycle `probe` (MoveKind::Probe): a protected
    /// positioning move, or with `measure` a measuring move to where it touches nominally.
    void probe(const Vec3& target, double feedRate, int probe, bool measure);

private:
    void push(Move move);
    Toolpath& path_;
    Vec3 position_;
};

} // namespace ocam::toolpath
