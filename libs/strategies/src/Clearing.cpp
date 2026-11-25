// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Engagement-limited clearing of one Z level: see Clearing.hpp, DL-0018 and IP-0004.

#include "Clearing.hpp"

#include "Internal.hpp"

#include <ocam/geom2d/Clip.hpp>
#include <ocam/geom2d/PathOps.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <numbers>
#include <optional>
#include <queue>
#include <unordered_map>

namespace ocam::strategies::detail {
namespace {

using geom2d::Polyline;
using geom2d::Region;
using geom2d::Regions;
using toolpath::FeedClass;

constexpr int kCircleSamples = 360;
constexpr double kTwoPi = 2.0 * std::numbers::pi;

Vec2 leftOf(const Vec2& t) {
    return {-t.y(), t.x()};
}

/// Points along a polyline's segments at most `spacing` apart (its own vertices kept).
std::vector<Vec2> resample(const std::vector<Vec2>& pts, double spacing) {
    std::vector<Vec2> out;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        if (i > 0) {
            const Vec2 a = pts[i - 1];
            const Vec2 d = pts[i] - a;
            const int n = static_cast<int>(std::ceil(d.norm() / spacing));
            for (int k = 1; k < n; ++k) {
                out.push_back(a + d * (static_cast<double>(k) / n));
            }
        }
        out.push_back(pts[i]);
    }
    return out;
}

double pathLength(const std::vector<Vec2>& pts) {
    double length = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        length += (pts[i] - pts[i - 1]).norm();
    }
    return length;
}

} // namespace

// ---------------------------------------------------------------------------------- CellGrid

CellGrid::CellGrid(const geom2d::Aabb2& box, double cell) : origin_(box.min), cell_(cell) {
    nx_ = std::max(1, static_cast<int>(std::ceil((box.max.x() - box.min.x()) / cell)) + 1);
    ny_ = std::max(1, static_cast<int>(std::ceil((box.max.y() - box.min.y()) / cell)) + 1);
    bits_.assign(static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_), 0);
}

void CellGrid::fill(const geom2d::Regions& regions, bool value, double chordTolerance) {
    const auto loops = discretizedLoops(regions, chordTolerance, [](const Vec2& p) { return p; });
    for (int j = 0; j < ny_; ++j) {
        const double y = origin_.y() + (j + 0.5) * cell_;
        for (const auto& [x0, x1] : scanIntervals(loops, y)) {
            const int i0 =
                std::max(0, static_cast<int>(std::ceil((x0 - origin_.x()) / cell_ - 0.5)));
            const int i1 =
                std::min(nx_ - 1, static_cast<int>(std::floor((x1 - origin_.x()) / cell_ - 0.5)));
            for (int i = i0; i <= i1; ++i) {
                bits_[static_cast<std::size_t>(j) * nx_ + i] = value ? 1 : 0;
            }
        }
    }
}

bool CellGrid::at(const Vec2& p) const {
    const int i = static_cast<int>(std::floor((p.x() - origin_.x()) / cell_));
    const int j = static_cast<int>(std::floor((p.y() - origin_.y()) / cell_));
    if (i < 0 || j < 0 || i >= nx_ || j >= ny_) {
        return false;
    }
    return bits_[static_cast<std::size_t>(j) * nx_ + i] != 0;
}

double CellGrid::contactAngle(const Vec2& centre, double radius, const Vec2& direction) const {
    const double rr = radius + 0.5 * cell_;
    const double heading = std::atan2(direction.y(), direction.x());
    int on = 0;
    for (int k = 0; k <= kCircleSamples / 2; ++k) {
        const double a = heading - 0.5 * std::numbers::pi + kTwoPi * k / kCircleSamples;
        if (at(centre + rr * Vec2(std::cos(a), std::sin(a)))) {
            ++on;
        }
    }
    return kTwoPi * on / kCircleSamples;
}

bool CellGrid::touches(const Vec2& centre, double radius) const {
    if (at(centre)) {
        return true;
    }
    const double rr = radius + 0.5 * cell_;
    for (int k = 0; k < kCircleSamples; ++k) {
        const double a = kTwoPi * k / kCircleSamples;
        if (at(centre + rr * Vec2(std::cos(a), std::sin(a)))) {
            return true;
        }
    }
    return false;
}

bool CellGrid::anyWithin(const Vec2& centre, double radius) const {
    const auto index = [&](double v, double o) {
        return (v - o) / cell_;
    };
    const int i0 =
        std::max(0, static_cast<int>(std::floor(index(centre.x() - radius, origin_.x()))));
    const int i1 =
        std::min(nx_ - 1, static_cast<int>(std::ceil(index(centre.x() + radius, origin_.x()))));
    const int j0 =
        std::max(0, static_cast<int>(std::floor(index(centre.y() - radius, origin_.y()))));
    const int j1 =
        std::min(ny_ - 1, static_cast<int>(std::ceil(index(centre.y() + radius, origin_.y()))));
    const double r2 = radius * radius;
    for (int j = j0; j <= j1; ++j) {
        const double dy = origin_.y() + (j + 0.5) * cell_ - centre.y();
        for (int i = i0; i <= i1; ++i) {
            const double dx = origin_.x() + (i + 0.5) * cell_ - centre.x();
            if (bits_[static_cast<std::size_t>(j) * nx_ + i] && dx * dx + dy * dy <= r2) {
                return true;
            }
        }
    }
    return false;
}

void CellGrid::clearDisk(const Vec2& centre, double radius, std::vector<std::uint32_t>* cleared) {
    const auto index = [&](double v, double o) {
        return (v - o) / cell_;
    };
    const int i0 =
        std::max(0, static_cast<int>(std::floor(index(centre.x() - radius, origin_.x()))));
    const int i1 =
        std::min(nx_ - 1, static_cast<int>(std::ceil(index(centre.x() + radius, origin_.x()))));
    const int j0 =
        std::max(0, static_cast<int>(std::floor(index(centre.y() - radius, origin_.y()))));
    const int j1 =
        std::min(ny_ - 1, static_cast<int>(std::ceil(index(centre.y() + radius, origin_.y()))));
    const double r2 = radius * radius;
    for (int j = j0; j <= j1; ++j) {
        const double dy = origin_.y() + (j + 0.5) * cell_ - centre.y();
        for (int i = i0; i <= i1; ++i) {
            const double dx = origin_.x() + (i + 0.5) * cell_ - centre.x();
            const std::size_t k = static_cast<std::size_t>(j) * nx_ + i;
            if (bits_[k] && dx * dx + dy * dy <= r2) {
                bits_[k] = 0;
                if (cleared) {
                    cleared->push_back(static_cast<std::uint32_t>(k));
                }
            }
        }
    }
}

void CellGrid::restore(const std::vector<std::uint32_t>& cells) {
    for (const std::uint32_t k : cells) {
        bits_[k] = 1;
    }
}

double engagementAngle(double width, double radius) {
    return std::acos(std::clamp(1.0 - width / radius, -1.0, 1.0));
}

double clearingCell(double toolRadius) {
    // Capped: the tool's centre keeps half a cell off the area's edge and the wall passes a
    // cell and a half inside it, so a wall keeps the allowance plus up to that; a 1/2" tool
    // on a 0.2 mm grid left 0.3 mm more than asked.
    return std::clamp(toolRadius / 32.0, 0.03, 0.15);
}

// -------------------------------------------------------------------------------- LevelLinks

LevelLinks::LevelLinks(const LevelClearing& in, const CellGrid& material, const geom2d::Aabb2& box,
                       double cell, double edge, double maxLink, const Context& ctx)
    : in_(in), material_(material),
      // Whole cells wider than the material's grid, so the two grids' cells coincide.
      linkable_({box.min - Vec2::Constant(std::ceil(2.0 * in.toolRadius / cell) * cell),
                 box.max + Vec2::Constant(std::ceil(2.0 * in.toolRadius / cell) * cell)},
                cell),
      rapidable_({box.min - Vec2::Constant(std::ceil(2.0 * in.toolRadius / cell) * cell),
                  box.max + Vec2::Constant(std::ceil(2.0 * in.toolRadius / cell) * cell)},
                 cell),
      cell_(cell), step_(std::max(cell, std::min(0.5, 0.1 * in.toolRadius))), maxLink_(maxLink),
      ctx_(ctx) {
    const geom2d::ClipOptions clip{ctx.chordTolerance};
    const Regions whole{in.area};
    const Regions reach = geom2d::offset(geom2d::unite(whole, in.travel, clip), edge, clip);
    const Regions keepOut = geom2d::unite(in.overhead, in.standing, clip);
    linkable_.fill(geom2d::subtract(reach, keepOut, clip), true, ctx.chordTolerance);
    rapidable_.fill(geom2d::subtract(geom2d::subtract(reach, keepOut, clip),
                                     geom2d::offset(in.forbidden, ctx.heights.feedClearance, clip),
                                     clip),
                    true, ctx.chordTolerance);
    hopArea_ = geom2d::subtract(
        geom2d::unite(geom2d::offset(whole, 4.0 * ctx.chordTolerance, clip), in.travel, clip),
        in.overhead, clip);
}

bool LevelLinks::clearAt(const Vec2& p) const {
    // A cell short of the radius: the tool's own cut leaves the material's edge within a cell
    // of it (the grid's resolution), and next to that edge it cuts nothing.
    return linkable_.at(p) && !material_.anyWithin(p, in_.toolRadius - cell_);
}

bool LevelLinks::clearLine(const Vec2& a, const Vec2& c, bool rapid) const {
    const double len = (c - a).norm();
    const CellGrid& area = rapid ? rapidable_ : linkable_;
    const double reach =
        rapid ? in_.toolRadius + ctx_.heights.feedClearance : in_.toolRadius - cell_;
    // The area at every cell (a corner of the part is not cut across), the material at every
    // step (between two the tool's sweep reaches past their disks by step² / 8r, a few µm).
    const int n = std::max(1, static_cast<int>(std::ceil(len / cell_)));
    const int every = std::max(1, static_cast<int>(std::floor(step_ / cell_)));
    for (int k = 1; k <= n; ++k) { // the tool is at `a` already
        const Vec2 p = a + (c - a) * (static_cast<double>(k) / n);
        if (!area.at(p)) {
            return false;
        }
        if ((k % every == 0 || k == n) && material_.anyWithin(p, reach)) {
            return false;
        }
    }
    return true;
}

std::vector<Vec2> LevelLinks::clearWay(const Vec2& a, const Vec2& c, double budget) const {
    if ((c - a).norm() > budget) {
        return {};
    }
    // A* over a lattice a step apart, from the tool's position, within the length allowed.
    const double h = step_;
    const auto key = [](int i, int j) {
        return (static_cast<std::int64_t>(i) << 32) ^ static_cast<std::uint32_t>(j);
    };
    const auto point = [&](int i, int j) {
        return Vec2(a + h * Vec2(i, j));
    };
    const int gi = static_cast<int>(std::lround((c - a).x() / h));
    const int gj = static_cast<int>(std::lround((c - a).y() / h));
    std::unordered_map<std::int64_t, char> free;
    const auto isFree = [&](int i, int j) {
        const auto [it, inserted] = free.try_emplace(key(i, j), char{0});
        if (inserted) {
            it->second = clearAt(point(i, j)) ? 1 : 0;
        }
        return it->second != 0;
    };
    struct Node {
        double f;
        double g;
        int i;
        int j;
        bool operator<(const Node& o) const { return f > o.f; }
    };
    std::unordered_map<std::int64_t, double> best;
    std::unordered_map<std::int64_t, std::int64_t> parent;
    std::priority_queue<Node> open;
    const auto remainingTo = [&](int i, int j) {
        return (point(i, j) - c).norm();
    };
    open.push({remainingTo(0, 0), 0.0, 0, 0});
    best[key(0, 0)] = 0.0;
    bool found = false;
    for (int expanded = 0; !open.empty() && expanded < 200000; ++expanded) {
        const Node n = open.top();
        open.pop();
        if (n.g > best[key(n.i, n.j)] + 1e-12) {
            continue;
        }
        if (n.i == gi && n.j == gj) {
            found = true;
            break;
        }
        for (int di = -1; di <= 1; ++di) {
            for (int dj = -1; dj <= 1; ++dj) {
                if (di == 0 && dj == 0) {
                    continue;
                }
                const int i = n.i + di;
                const int j = n.j + dj;
                const double g = n.g + h * std::hypot(di, dj);
                if (g + remainingTo(i, j) > budget || ((i != gi || j != gj) && !isFree(i, j))) {
                    continue;
                }
                const auto it = best.find(key(i, j));
                if (it != best.end() && it->second <= g) {
                    continue;
                }
                // The step itself as the straightening below checks it.
                if (!clearLine(point(n.i, n.j), (i == gi && j == gj) ? c : point(i, j))) {
                    continue;
                }
                best[key(i, j)] = g;
                parent[key(i, j)] = key(n.i, n.j);
                open.push({g + remainingTo(i, j), g, i, j});
            }
        }
    }
    if (!found) {
        return {};
    }
    std::vector<Vec2> lattice{c};
    for (std::int64_t k = parent[key(gi, gj)];; k = parent[k]) {
        const int i = static_cast<int>(k >> 32);
        const int j = static_cast<int>(static_cast<std::int32_t>(k & 0xffffffff));
        lattice.push_back(point(i, j));
        if (i == 0 && j == 0) {
            break;
        }
    }
    std::reverse(lattice.begin(), lattice.end());
    lattice.front() = a;
    // Straightened: from each corner on to the furthest point it sees clear.
    std::vector<Vec2> way{a};
    for (std::size_t at = 0; at + 1 < lattice.size();) {
        std::size_t next = at + 1;
        for (std::size_t k = lattice.size() - 1; k > at + 1; --k) {
            if (clearLine(lattice[at], lattice[k])) {
                next = k;
                break;
            }
        }
        if (!clearLine(lattice[at], lattice[next])) {
            return {};
        }
        way.push_back(lattice[next]);
        at = next;
    }
    return pathLength(way) <= budget + 1e-9 ? way : std::vector<Vec2>{};
}

std::vector<Vec2> LevelLinks::moveTo(toolpath::PathBuilder& b, const Vec2& e) const {
    const Vec3 here3 = b.position();
    const Vec2 here = here3.head<2>();
    const double z = in_.z;
    const bool atDepth = std::abs(here3.z() - z) < 1e-9;
    const bool straight = atDepth && clearLine(here, e);
    std::vector<Vec2> way;
    if (straight) {
        way = {here, e};
    } else if (atDepth) {
        way = clearWay(here, e, maxLink_);
    }
    if (!way.empty() && pathLength(way) <= maxLink_ + 1e-9) {
        b.follow(Polyline::fromPoints(way, false), z, FeedClass::Link, ctx_.feeds.cut);
        return way;
    }
    const double feedFrom = z + ctx_.heights.feedClearance;
    if (straight && clearLine(here, e, true)) {
        // Nothing of the level stands in the way (and nothing over it: material above the
        // level stands over material of the level), just too far to feed: just off the floor,
        // straight (a rapid is not held to a way around things).
        b.retractTo(feedFrom);
        b.rapid(Vec3(e.x(), e.y(), feedFrom));
    } else if (here3.z() <= in_.approachZ + 1e-9 &&
               geom2d::segmentInside(hopArea_, here, e, ctx_.chordTolerance)) {
        b.retractTo(in_.approachZ);
        b.rapid(Vec3(e.x(), e.y(), in_.approachZ));
    } else {
        b.rapidAbove(e, in_.retract, in_.approachZ);
    }
    // The level's material stands from the top of its column down to the level, so the column
    // over a point the tool there keeps the feed clearance from holds none of it: rapid down to
    // the feed clearance, feed only the last bit. An entry just off the material (clear of the
    // grid by half a cell) is fed down from the approach height: a rapid straight down there
    // grazed the material's edge over the column's whole height (the stair a fine level takes
    // on the dome, 0.97 mm deep in the simulation).
    if (b.position().z() > feedFrom + 1e-9 &&
        !material_.anyWithin(e, in_.toolRadius + ctx_.heights.feedClearance)) {
        b.rapid(Vec3(e.x(), e.y(), feedFrom));
    }
    b.feed(Vec3(e.x(), e.y(), z), FeedClass::Plunge, ctx_.feeds.plunge);
    return {};
}

// -------------------------------------------------------------------------------- clearLevel

ClearingReport clearLevel(toolpath::PathBuilder& b, const LevelClearing& in, const Context& ctx) {
    const double tol = ctx.chordTolerance;
    const geom2d::ClipOptions clip{tol};
    const double r = in.toolRadius;
    const double ae = in.stepover;
    const double z = in.z;
    const double noise = std::max(10.0 * tol, 0.01);
    const Regions whole{in.area};
    ClearingReport report;

    // The level's material on a grid (the part and its allowance are never cut, so they are
    // not material), and the allowed centres on another.
    geom2d::Aabb2 box = in.area.bounds();
    box.min -= Vec2::Constant(r + 1.0);
    box.max += Vec2::Constant(r + 1.0);
    const double cell = clearingCell(r);
    const Regions partMaterial = geom2d::offset(in.forbidden, -(r - 2.0 * cell), clip);
    Regions remaining = geom2d::subtract(
        geom2d::intersect(in.material, geom2d::offset(whole, r, clip), clip), partMaterial, clip);
    std::erase_if(remaining, [](const Region& piece) { return piece.area() < 1e-3; });
    if (remaining.empty()) {
        return report;
    }
    if (const char* dumpDir = std::getenv("OCAM_CLEARING_DUMP")) {
        // Debugging aid: the level's material as polygons, for an outside check.
        char name[512];
        std::snprintf(name, sizeof name, "%s/material_z%.3f_%zu.txt", dumpDir, z,
                      static_cast<std::size_t>(std::llround(in.area.bounds().min.x() * 1000.0)));
        if (std::ofstream f{name}) {
            for (const Region& region : remaining) {
                std::vector<const Polyline*> loops{&region.outer};
                for (const Polyline& hole : region.holes) {
                    loops.push_back(&hole);
                }
                for (const Polyline* loop : loops) {
                    for (const Vec2& q : loop->discretize(tol)) {
                        f << q.x() << ' ' << q.y() << ' ';
                    }
                    f << '\n';
                }
            }
        }
    }
    CellGrid grid(box, cell);
    grid.fill(remaining, true, tol);
    // Allowed positions stay half a cell inside the area, so the tool never crosses its edge;
    // passes along the edge (walls) run a cell and a half inside it, where they are allowed
    // (a cell inside was tried: on the grid's edge, half the points failed).
    CellGrid allowed(box, cell);
    allowed.fill(geom2d::offset(whole, -0.5 * cell, clip), true, tol);
    const Regions outside = geom2d::offset(
        geom2d::subtract(Regions{{Polyline::rectangle(box.min, box.max), {}}}, whole, clip),
        1.5 * cell, clip);

    // The limit: the engagement of a cut one stepover and a cell wide. The passes themselves
    // run a cell short of the stepover, so a pass at its depth has two cells of margin to the
    // limit: the angle measured on the grid jitters by a degree or two, and without that
    // margin a straight pass at half the diameter (90 degrees) would be thrown off its line
    // again and again, each loop wavier than the one before.
    const double thetaMax = engagementAngle(ae + cell, r);
    const double ds = std::max(cell, std::min(0.5, 0.1 * r));
    const double lead = ae / std::tan(degToRad(20.0));
    const double clearOffset = ae + 2.0 * cell; // from a pass to where the tool cuts nothing
    const double maxLink = 4.0 * ae;

    const auto angleAt = [&](const Vec2& p, const Vec2& t) {
        return grid.contactAngle(p, r, t);
    };
    const auto lineAllowed = [&](const Vec2& a, const Vec2& c) {
        const int n = std::max(1, static_cast<int>(std::ceil((c - a).norm() / cell)));
        for (int k = 0; k <= n; ++k) {
            if (!allowed.at(a + (c - a) * (static_cast<double>(k) / n))) {
                return false;
            }
        }
        return true;
    };
    const auto touchesAlong = [&](const Vec2& a, const Vec2& c) {
        const int n = std::max(1, static_cast<int>(std::ceil((c - a).norm() / ds)));
        for (int k = 0; k <= n; ++k) {
            if (grid.touches(a + (c - a) * (static_cast<double>(k) / n), r)) {
                return true;
            }
        }
        return false;
    };
    // Whether the straight way from the tool to a point is clear of material but for the last
    // stretch (a clear offset) before it: the way the tool may flow on along at depth.
    const auto wayClear = [&](const Vec2& from, const Vec2& to) {
        const Vec2 d = to - from;
        const double len = d.norm();
        if (len <= clearOffset) {
            return true;
        }
        const Vec2 p = from + d * ((len - clearOffset) / len);
        return lineAllowed(from, p) && !touchesAlong(from, p);
    };
    // Where a pass starting at q0 (direction t, cleared side n) can be entered from: a point in
    // the clear behind it and to the cleared side, in a straight line of it. The way from there
    // to the pass is traced like the pass itself, so the engagement rises to the limit and no
    // further; a pass with no such point starts further along, where it has one.
    const auto findEntry = [&](const Vec2& q0, const Vec2& t,
                               const Vec2& n) -> std::optional<Vec2> {
        // The nearest of the candidates to where the tool is (a pass that starts next to the
        // end of the last one is entered from there, without going back first).
        const Vec2 here = b.position().head<2>();
        std::optional<Vec2> best;
        // Candidates a stepover and more out first; a narrow area (a slot two diameters wide,
        // the film an allowance leaves on a wall) has no room for those, so nearer ones
        // follow, down to just off the pass.
        for (const std::initializer_list<double> outs :
             {std::initializer_list<double>{clearOffset, clearOffset + 0.5 * r},
              std::initializer_list<double>{0.5 * clearOffset, 0.25 * clearOffset, 2.5 * cell}}) {
            for (const double along : {0.5 * lead, 0.0}) {
                for (const double out : outs) {
                    const Vec2 e = q0 - along * t + out * n;
                    if (allowed.at(e) && !grid.touches(e, r) && lineAllowed(e, q0) &&
                        (!best || (e - here).norm() < (*best - here).norm())) {
                        best = e;
                    }
                }
            }
            if (best) {
                break;
            }
        }
        return best;
    };
    // One step of the tracer from `a`: as directly toward `desired` as the limit allows, turned
    // toward the cleared side `n` in steps of 2.5 degrees until the engagement at the new
    // point is within it (and the point allowed); none when even a right-angle turn is not.
    // A move is within the limit when the engagement is at both its ends, facing its direction
    // (a turn back toward material shows at the start).
    const auto moveWithin = [&](const Vec2& a, const Vec2& p) {
        const Vec2 dir = (p - a).normalized();
        return angleAt(a, dir) <= thetaMax && angleAt(p, dir) <= thetaMax;
    };
    // Directions change by at most this much per step: the path turns, it does not zigzag
    // (held off by the limit, the tool would otherwise flip between into and away from the
    // material every step).
    const double maxTurn = degToRad(25.0);
    double turnLimit = maxTurn; // the first step from a carried heading may turn further
    const auto turnLimited = [&](const Vec2& dir, const std::optional<Vec2>& previous) {
        if (!previous) {
            return dir;
        }
        const double cross = previous->x() * dir.y() - previous->y() * dir.x();
        const double angle = std::atan2(cross, previous->dot(dir));
        if (std::abs(angle) <= turnLimit) {
            return dir;
        }
        return rotate(*previous, angle > 0.0 ? turnLimit : -turnLimit);
    };
    const auto step = [&](const Vec2& a, const Vec2& desired, const Vec2& n,
                          const std::optional<Vec2>& previous) -> std::optional<Vec2> {
        // Heading straight at the material, either way round is as direct: turn so that the
        // material ends up on the cutting direction's side.
        const double cross = desired.x() * n.y() - desired.y() * n.x();
        const double sign = std::abs(cross) > 1e-9 ? (cross > 0.0 ? 1.0 : -1.0)
                            : in.climb             ? 1.0
                                                   : -1.0;
        Vec2 last = Vec2::Zero();
        for (int k = 0; k <= 72; ++k) {
            const Vec2 dir = turnLimited(rotate(desired, sign * degToRad(2.5 * k)), previous);
            if (k > 0 && (dir - last).norm() < 1e-9) {
                continue; // the turn limit gives the same direction again
            }
            last = dir;
            const Vec2 p = a + ds * dir;
            if (allowed.at(p) && moveWithin(a, p)) {
                return p;
            }
        }
        return std::nullopt;
    };

    // The tool's direction at the end of its last cut at depth: a path flowing on from there
    // starts with it (a fresh start would turn back on itself).
    std::optional<Vec2> heading;
    // Material removed by a path (for the next pass's geometry).
    std::vector<std::vector<Vec2>> cutPaths;
    const auto cutPath = [&](const std::vector<Vec2>& pts) {
        if (pts.size() >= 2) {
            cutPaths.push_back(pts);
        } else if (pts.size() == 1) {
            cutPaths.push_back({pts.front(), pts.front()});
        }
    };
    // Moves the tool to an entry point: at depth through clear area when close, else by the
    // lowest hop that is safe and a feed down (LevelLinks).
    const LevelLinks links(in, grid, box, cell, -0.5 * cell, maxLink, ctx);
    const auto goTo = [&](const Vec2& e) {
        const std::vector<Vec2> way = links.moveTo(b, e);
        for (const Vec2& p : resample(way, ds)) {
            grid.clearDisk(p, r);
        }
        if (way.size() >= 2) {
            cutPath(way);
        }
        grid.clearDisk(e, r);
    };

    // Open areas only: the passes start from the air (closed areas are the medial clearing's,
    // DL-0031).
    if (std::getenv("OCAM_CLEARING_DEBUG") != nullptr) {
        std::fprintf(stderr, "DBG level z %.3f: remaining %.1f, air %.1f\n", z,
                     geom2d::totalArea(remaining), geom2d::totalArea(in.air));
    }

    // Pass by pass: the positions at the stepover's depth into the remaining material (or on
    // the area's edge where the material there is thinner than that), where the tool touches
    // material, in the cutting direction; each traced on the grid and pulled back where the
    // engagement would exceed the limit.
    const double depthOffset = std::max(r - ae + cell, 0.0);
    // Material the passes could not take, reported when a pass removes nothing.
    const auto reportLeftover = [&](Regions left) {
        // Only material some allowed position reaches is the passes' to take, and not a film
        // thinner than two cells (the in-process stock's resolution at the allowance, at the
        // edge of the tool's reach): nobody's.
        left = geom2d::intersect(left, geom2d::offset(whole, r - 2.0 * cell, clip), clip);
        left = geom2d::offset(geom2d::offset(left, -cell, clip), cell, clip);
        // Pieces too small to matter at roughing are not reported: the sliver an inside corner
        // keeps for the finishing pass (the tool's whole arc in the corner would be engaged at
        // once there): r² (1 − π/4) and the film along its walls, under half the tool radius
        // squared.
        for (const Region& piece : left) {
            if (piece.area() > 0.5 * r * r) {
                report.leftover.push_back(geom2d::interiorPoint(piece, tol));
                report.leftoverArea += piece.area();
            }
        }
        if (!report.leftover.empty()) {
            ++report.stalled;
        }
    };
    for (int pass = 0; pass < 5000; ++pass) {
        ctx.cancel.throwIfCancelled();
        if (!cutPaths.empty()) {
            remaining = geom2d::subtract(remaining, geom2d::sweep(cutPaths, r, clip), clip);
            remaining = geom2d::offset(geom2d::offset(remaining, -noise, clip), noise, clip);
            std::erase_if(remaining, [](const Region& piece) { return piece.area() < 1e-3; });
            cutPaths.clear();
        }
        if (remaining.empty()) {
            break;
        }
        // The passes: along the material at the stepover's depth into it, and along the area's
        // edge (the walls) where material stands within reach, each from its own loops. United,
        // the edge's pass vanished under material standing at a wall, which only a pass along
        // the wall can take (the tool's centre cannot go nearer): in a slot the corners at the
        // walls lagged further behind each pass until nothing could be entered.
        // (All pieces of the remaining material at once: one at a time, the nearest to the
        // tool, finished a slot's end before the other but split an open area's late pieces
        // into many passes with their own entries; the slices of a slot's two ends alternate.)
        const Regions& working = remaining;
        const Regions touch = geom2d::offset(working, r + 2.0 * cell, clip);
        std::vector<std::vector<Vec2>> loops;
        const auto loopsOf = [&](const Regions& source) {
            std::vector<std::vector<Vec2>> loops;
            for (const Region& region : source) {
                std::vector<const Polyline*> boundary{&region.outer};
                for (const Polyline& hole : region.holes) {
                    boundary.push_back(&hole);
                }
                for (const Polyline* loop : boundary) {
                    std::vector<Vec2> pts = loop->discretize(tol);
                    if (pts.size() < 3) {
                        continue;
                    }
                    pts.push_back(pts.front());
                    // Region loops keep the material (or the wall) on the left; climb wants it
                    // on the right.
                    if (in.climb) {
                        std::reverse(pts.begin(), pts.end());
                    }
                    loops.push_back(std::move(pts));
                }
            }
            return loops;
        };
        const Regions inside = geom2d::offset(whole, 2.0 * tol, clip);
        std::vector<std::vector<Vec2>> pieces =
            geom2d::clipPaths(geom2d::clipPaths(loopsOf(geom2d::offset(working, depthOffset, clip)),
                                                touch, true, clip),
                              inside, true, clip);
        // The wall passes only where the material's own passes do not reach (along a wall
        // they would run the same way twice), and out of the tool's reach of material thicker
        // than a stepover: the body of the material is the passes' to take, front by front,
        // and a wall pass run through it slotted along the wall, over the limit, pulling back
        // in loops (the whole of a slot after its helix); the thin bands and wedges a front
        // leaves at a wall are the wall pass's.
        const Regions thick =
            geom2d::offset(geom2d::offset(working, -0.5 * ae, clip), 0.5 * ae, clip);
        const Regions wallTouch =
            geom2d::subtract(geom2d::subtract(touch, geom2d::sweep(pieces, r - cell, clip), clip),
                             geom2d::offset(thick, r + 2.0 * cell, clip), clip);
        for (auto& piece : geom2d::clipPaths(
                 geom2d::clipPaths(loopsOf(outside), wallTouch, true, clip), inside, true, clip)) {
            pieces.push_back(std::move(piece));
        }
        std::erase_if(
            pieces, [&](const std::vector<Vec2>& piece) { return pathLength(piece) < 10.0 * tol; });
        if (pieces.empty()) {
            break;
        }
        // Diagnostics: OCAM_CLEARING_DEBUG prints each pass and piece to stderr;
        // OCAM_CLEARING_DUMP=<dir> writes each level's material as polygons (see above) and
        // each pass's remaining material ("R" lines) and pieces ("P" lines).
        const bool dbg = std::getenv("OCAM_CLEARING_DEBUG") != nullptr;
        std::string dumpName; // this pass's dump file: the traced paths are appended ("T")
        if (const char* dumpDir = std::getenv("OCAM_CLEARING_DUMP")) {
            char name[512];
            std::snprintf(name, sizeof name, "%s/pass_z%.3f_%zu_%d.txt", dumpDir, z,
                          static_cast<std::size_t>(std::llround(in.area.bounds().min.x() * 1000.0)),
                          pass);
            dumpName = name;
            if (std::ofstream f{name}) {
                for (const Region& region : remaining) {
                    std::vector<const Polyline*> boundary{&region.outer};
                    for (const Polyline& hole : region.holes) {
                        boundary.push_back(&hole);
                    }
                    for (const Polyline* loop : boundary) {
                        f << "R";
                        for (const Vec2& q : loop->discretize(tol)) {
                            f << ' ' << q.x() << ' ' << q.y();
                        }
                        f << '\n';
                    }
                }
                for (const auto& piece : pieces) {
                    f << "P";
                    for (const Vec2& q : piece) {
                        f << ' ' << q.x() << ' ' << q.y();
                    }
                    f << '\n';
                }
            }
        }
        double worstAccepted = 0.0; // the tracer's own largest engagement this pass
        if (dbg) {
            std::fprintf(stderr, "DBG pass %d z %.3f: %zu pieces, remaining %.2f mm2\n", pass, z,
                         pieces.size(), geom2d::totalArea(remaining));
        }
        const double areaBefore = geom2d::totalArea(remaining);

        while (!pieces.empty()) {
            ctx.cancel.throwIfCancelled();
            // The nearest piece; a closed one starts at its point nearest to the tool.
            const Vec2 here = b.position().head<2>();
            std::size_t best = 0;
            double bestDistance = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < pieces.size(); ++i) {
                const bool closed = (pieces[i].front() - pieces[i].back()).norm() < tol;
                double d = (pieces[i].front() - here).norm();
                if (closed) {
                    for (const Vec2& q : pieces[i]) {
                        d = std::min(d, (q - here).norm());
                    }
                }
                if (d < bestDistance) {
                    bestDistance = d;
                    best = i;
                }
            }
            std::vector<Vec2> nominal = resample(pieces[best], ds);
            pieces.erase(pieces.begin() + static_cast<std::ptrdiff_t>(best));
            const bool closed = (nominal.front() - nominal.back()).norm() < tol;
            if (closed) {
                nominal.pop_back();
            }
            if (nominal.size() < 2) {
                continue;
            }
            // Nothing left along it (taken by the pass before: a wall pass and the material's
            // own pass can run together): not worth an entry.
            if (std::none_of(nominal.begin(), nominal.end(),
                             [&](const Vec2& q) { return grid.touches(q, r); })) {
                continue;
            }
            // Directions along the piece, and the side the cleared material is on.
            const auto tangentAt = [&](std::size_t i) {
                const Vec2& p = nominal[i];
                Vec2 t = Vec2::Zero();
                for (std::size_t k = i + 1; k < nominal.size() && t.norm() < 0.5 * ds; ++k) {
                    t = nominal[k] - p;
                }
                if (t.norm() < 1e-9) {
                    for (std::size_t k = i; k-- > 0 && t.norm() < 0.5 * ds;) {
                        t = p - nominal[k];
                    }
                }
                return t.norm() > 1e-12 ? Vec2(t.normalized()) : Vec2(1.0, 0.0);
            };
            const auto normalAt = [&](std::size_t i) {
                const Vec2 t = tangentAt(i);
                return in.climb ? leftOf(t) : Vec2(-leftOf(t));
            };
            // A closed piece starts at the point nearest to the tool that can be entered (its
            // loop goes on from there).
            if (closed) {
                std::vector<std::size_t> order(nominal.size());
                for (std::size_t i = 0; i < order.size(); ++i) {
                    order[i] = i;
                }
                std::sort(order.begin(), order.end(), [&](std::size_t x, std::size_t y) {
                    return (nominal[x] - here).squaredNorm() < (nominal[y] - here).squaredNorm();
                });
                std::size_t at = order.front();
                if (std::abs(b.position().z() - z) > 1e-9) {
                    // Not at depth: the loop is entered, so start where it can be.
                    for (const std::size_t i : order) {
                        if (findEntry(nominal[i], tangentAt(i), normalAt(i))) {
                            at = i;
                            break;
                        }
                    }
                }
                std::rotate(nominal.begin(), nominal.begin() + static_cast<std::ptrdiff_t>(at),
                            nominal.end());
                nominal.push_back(nominal.front());
            }

            // Trace the piece: from an entry point, step by step toward a point of the nominal
            // pass a stepover ahead of where the tool is along it, each step as direct as the
            // limit allows. The traced path leads in, follows the pass at the stepover, and is
            // held back from inside corners (where it runs along the material at the limit),
            // all by that one rule. Where the tool has cut nothing for a while, or makes no way
            // along the pass, the path ends and a new one starts further along.
            std::vector<double> cum(nominal.size(), 0.0);
            for (std::size_t k = 1; k < nominal.size(); ++k) {
                cum[k] = cum[k - 1] + (nominal[k] - nominal[k - 1]).norm();
            }
            const double total = cum.back();
            const double fullClear = r + ae + 2.0 * cell;
            // Where `a` is along the pass: the nearest point of the segments from `seg` on, within
            // a window (never backwards, so a closed loop's end is not mistaken for its start).
            double lateral = 0.0; // distance from the tool to the pass, set by project()
            const auto project = [&](const Vec2& a, std::size_t& seg) {
                double best = std::numeric_limits<double>::infinity();
                double at = cum[seg];
                std::size_t bestSeg = seg;
                for (std::size_t k = seg; k + 1 < nominal.size() && cum[k] <= cum[seg] + 3.0 * ae;
                     ++k) {
                    const Vec2 ab = nominal[k + 1] - nominal[k];
                    const double len2 = ab.squaredNorm();
                    const double u =
                        len2 > 1e-18 ? std::clamp((a - nominal[k]).dot(ab) / len2, 0.0, 1.0) : 0.0;
                    const double d = (nominal[k] + u * ab - a).squaredNorm();
                    if (d < best) {
                        best = d;
                        at = cum[k] + u * std::sqrt(len2);
                        bestSeg = k;
                    }
                }
                seg = bestSeg;
                lateral = std::sqrt(best);
                return at;
            };
            // The point of the pass at arc length `at` (from segment `from` on).
            const auto pointAt = [&](double at, std::size_t from) {
                std::size_t k = from;
                while (k + 2 < nominal.size() && cum[k + 1] < at) {
                    ++k;
                }
                const double len = cum[k + 1] - cum[k];
                const double u = len > 1e-12 ? std::clamp((at - cum[k]) / len, 0.0, 1.0) : 0.0;
                return Vec2(nominal[k] + u * (nominal[k + 1] - nominal[k]));
            };
            struct Trace {
                std::vector<Vec2> actual;
                std::size_t seg = 0;
                bool arrived = false;
                bool cut = false;         ///< The tool met material somewhere along it.
                std::size_t firstCut = 0; ///< Index in `actual` of the first point touching.
            };
            // Traces the pass from index `i` on, the tool being at `start`; the grid is cut as
            // it goes (into `log` when given, so a trial can be taken back).
            const auto trace = [&](const Vec2& start, std::size_t i,
                                   std::vector<std::uint32_t>* log,
                                   const std::optional<Vec2>& from) {
                Trace out;
                out.actual.push_back(start);
                Vec2 a = start;
                std::size_t seg = i;
                {
                    std::size_t probe = seg;
                    project(a, probe); // for `lateral`
                }
                double progress = cum[i];
                double lastProgress = progress;
                double sinceAdvance = 0.0;
                double sinceTouch = 0.0;
                double travelled = 0.0;
                // The last step's direction. Flowing on from the tool's last cut, the path
                // starts with that heading. From the end of a path it may turn sharply at once
                // (into a corner piece) but not back on itself.
                std::optional<Vec2> previous = from;
                const char* why = "guard"; // why the path ended (diagnostics)
                for (int guard = 0; guard < 200000; ++guard) {
                    turnLimit = guard == 0 && from ? degToRad(90.0) : maxTurn;
                    // Leading in: off the pass with nothing cut yet. The tool then heads for
                    // the start of the pass itself (a short piece would otherwise be skipped
                    // for a point near its end), and its place along the pass stays the start.
                    const bool leadingIn =
                        !out.cut && lateral > 2.0 * ds && (nominal[i] - a).norm() > 2.0 * ds;
                    // At the end: along the pass, or next to its last point once past
                    // half of it (a closed loop ends where it starts: the lead-in must not
                    // count as arriving).
                    if (!leadingIn &&
                        (progress >= total - 0.75 * ds ||
                         (progress > 0.5 * total && (nominal.back() - a).norm() < 0.75 * ds))) {
                        out.arrived = true;
                        why = "arrived";
                        break;
                    }
                    // Next to the pass (within a step and a half), its next point is taken
                    // when the limit allows: the pass is followed exactly, corners included,
                    // with no wobble. Further off, the tool aims at the point of the pass as
                    // far ahead as it is off it, at least half a stepover, so it comes back
                    // at 45 degrees at most and ever more gently as it closes in; the limit
                    // shapes that into the lead-in where it meets material.
                    const Vec2 onPass = pointAt(progress + ds, seg);
                    const Vec2 aim = leadingIn
                                         ? nominal[i]
                                         : pointAt(progress + std::max(0.5 * ae, lateral), seg);
                    const std::size_t target =
                        std::min(nominal.size() - 1,
                                 seg + 1 + static_cast<std::size_t>(std::ceil(lateral / ds)));
                    const Vec2 toPass = onPass - a;
                    const Vec2 desired =
                        (aim - a).norm() > 1e-9 ? Vec2((aim - a).normalized()) : tangentAt(target);
                    std::optional<Vec2> next;
                    if (!leadingIn && toPass.norm() > 0.25 * ds && toPass.norm() <= 1.5 * ds &&
                        allowed.at(onPass) && moveWithin(a, onPass) &&
                        (turnLimited(toPass.normalized(), previous) - toPass.normalized()).norm() <
                            1e-9) {
                        next = onPass;
                    } else {
                        next = step(a, desired, normalAt(target), previous);
                    }
                    if (!next) {
                        why = "no step";
                        if (dbg) {
                            const Vec2 p = a + ds * desired;
                            std::fprintf(stderr,
                                         "DBG     stuck at (%.2f, %.2f) desired (%.2f, %.2f): "
                                         "allowed %d, angle here %.0f, there %.0f, lateral %.2f, "
                                         "leading %d\n",
                                         a.x(), a.y(), desired.x(), desired.y(), int(allowed.at(p)),
                                         radToDeg(angleAt(a, desired)),
                                         radToDeg(angleAt(p, desired)), lateral, int(leadingIn));
                        }
                        break;
                    }
                    previous = (*next - a).normalized();
                    if (dbg) {
                        worstAccepted = std::max(
                            {worstAccepted, angleAt(a, *previous), angleAt(*next, *previous)});
                    }
                    const bool touching = grid.touches(*next, r);
                    if (touching && !out.cut) {
                        out.firstCut = out.actual.size();
                    }
                    out.cut = out.cut || touching;
                    sinceTouch = touching ? 0.0 : sinceTouch + ds;
                    travelled += ds;
                    grid.clearDisk(*next, r, log);
                    out.actual.push_back(*next);
                    a = *next;
                    // Leading in, the tool's place along the pass stays the start: the nearest
                    // part of the pass may be a later one, across the area (a loop around the
                    // end of a slot), and measured against that the lead never ended.
                    std::size_t probe = seg;
                    const double at = project(a, probe);
                    if (!leadingIn) {
                        seg = probe;
                        progress = std::max(progress, at);
                    }
                    if (progress > lastProgress + 0.5 * ds) {
                        lastProgress = progress;
                        sinceAdvance = 0.0;
                    } else {
                        sinceAdvance += ds;
                    }
                    // Until the tool has met material it is leading in; after that a stretch
                    // cutting nothing, or making no way along the pass, ends the path. A lead
                    // that never meets material ends too.
                    const bool onThePass = progress > cum[i] + 0.5 * ds;
                    if ((out.cut && sinceTouch > 2.0 * ae) ||
                        (onThePass && sinceAdvance > 4.0 * ae) ||
                        (!out.cut && travelled > lead + fullClear + 2.0 * ae)) {
                        why = out.cut && sinceTouch > 2.0 * ae       ? "nothing cut for a while"
                              : onThePass && sinceAdvance > 4.0 * ae ? "no way along the pass"
                                                                     : "lead met nothing";
                        break;
                    }
                }
                turnLimit = maxTurn;
                out.seg = seg;
                if (dbg) {
                    std::fprintf(stderr,
                                 "DBG   piece len %.1f from %zu: start (%.2f, %.2f) -> %zu pts, "
                                 "arrived %d, cut %d, end (%.2f, %.2f): %s\n",
                                 total, i, start.x(), start.y(), out.actual.size(),
                                 int(out.arrived), int(out.cut), a.x(), a.y(), why);
                }
                return out;
            };
            // Smooths a traced path: each inner point moves to the mean of its neighbours
            // (two each side), when that keeps it allowed and within the limit on the
            // material as it was before the path was cut; the path is then cut for real.
            // Headings come in steps of 2.5 degrees, and a pass held off by the limit follows
            // the bumps of the material edge the pass before it left: smoothing here keeps
            // the bumps from growing pass to pass.
            const auto smoothAndCut = [&](std::vector<Vec2> pts) {
                if (pts.size() >= 5) {
                    std::vector<Vec2> smoothed = pts;
                    for (std::size_t k = 2; k + 2 < pts.size(); ++k) {
                        smoothed[k] =
                            (pts[k - 2] + pts[k - 1] + pts[k] + pts[k + 1] + pts[k + 2]) / 5.0;
                    }
                    // Checked and cut in order, so each point sees the cut before it.
                    for (std::size_t k = 1; k < pts.size(); ++k) {
                        const Vec2 q = k + 2 < pts.size() && k >= 2 ? smoothed[k] : pts[k];
                        const Vec2 dir = (q - pts[k - 1]).norm() > 1e-9
                                             ? Vec2((q - pts[k - 1]).normalized())
                                             : Vec2(1.0, 0.0);
                        if (allowed.at(q) && angleAt(q, dir) <= thetaMax) {
                            pts[k] = q;
                        }
                        grid.clearDisk(pts[k], r);
                    }
                } else {
                    for (std::size_t k = 1; k < pts.size(); ++k) {
                        grid.clearDisk(pts[k], r);
                    }
                }
                return pts;
            };
            const auto emit = [&](Trace t, std::vector<std::uint32_t>& log) {
                grid.restore(log);
                t.actual = smoothAndCut(std::move(t.actual));
                heading.reset();
                if (t.actual.size() >= 2) {
                    // The way to the material (the lead-in) is shown and posted as such.
                    if (t.firstCut >= 2 && t.firstCut + 1 < t.actual.size()) {
                        const std::vector<Vec2> lead(t.actual.begin(),
                                                     t.actual.begin() + t.firstCut + 1);
                        b.follow(geom2d::fitArcs(lead, false, 2.0 * tol), z, FeedClass::LeadIn,
                                 ctx.feeds.lead);
                        const std::vector<Vec2> rest(t.actual.begin() + t.firstCut, t.actual.end());
                        b.follow(geom2d::fitArcs(rest, false, 2.0 * tol), z, FeedClass::Cut,
                                 ctx.feeds.cut);
                    } else {
                        const Polyline path = geom2d::fitArcs(t.actual, false, 2.0 * tol);
                        b.follow(path, z, FeedClass::Cut, ctx.feeds.cut);
                    }
                    report.cutAnything = true;
                    const Vec2 last = t.actual.back() - t.actual[t.actual.size() - 2];
                    if (last.norm() > 1e-9) {
                        heading = last.normalized();
                    }
                    if (!dumpName.empty()) {
                        if (std::ofstream f{dumpName, std::ios::app}) {
                            f << 'T';
                            for (const Vec2& q : t.actual) {
                                f << ' ' << q.x() << ' ' << q.y();
                            }
                            f << '\n';
                        }
                    }
                }
                cutPath(t.actual);
            };

            std::size_t i = 0;
            while (i + 1 < nominal.size()) {
                // At depth already (the end of the last path): flow straight on into this
                // piece, the tracer's rule shaping the way over, when that cuts something. An
                // open piece only from near its start: flowing into its middle would leave its
                // first part for a later pass. And only from behind the start: at the far wall
                // of a slot the tool stands ahead of the next slice's start at the near wall,
                // and flowing on it ground its way back across the slice diagonally; it goes
                // back along the chord through the cleared side and enters the slice from
                // behind its start instead (link, lead-in, arc). Where the way over is not clear
                // (the pass along a wall, cut off by the part's outline bulging into its way,
                // starts again past the bulge, beyond the material the tool stands at), the
                // tool flows on only when the traced way reaches the pass within a short way:
                // else it would leave the material for an entry, step out and lead back in
                // where it just was.
                bool nearStart = false;
                bool clearOver = false;
                double joinBudget = 0.0;
                if (std::abs(b.position().z() - z) < 1e-9) {
                    std::size_t sg = i;
                    const Vec2 tool = b.position().head<2>();
                    const double at = project(tool, sg);
                    const bool behind = (nominal[i] - tool).dot(tangentAt(i)) >= -ds;
                    nearStart = lateral <= 1.5 * ae && total - at >= 2.0 * ds &&
                                (closed || (at - cum[i] <= 1.5 * ae && behind));
                    clearOver = nearStart && wayClear(tool, nominal[i]);
                    joinBudget = 2.0 * (nominal[i] - tool).norm() + ae;
                }
                // Whether a traced path from the tool comes within a step and a half of the pass
                // before it has run `joinBudget`.
                const auto joins = [&](const Trace& t) {
                    double run = 0.0;
                    for (std::size_t k = 1; k < t.actual.size() && run <= joinBudget; ++k) {
                        run += (t.actual[k] - t.actual[k - 1]).norm();
                        std::size_t sg = i;
                        project(t.actual[k], sg);
                        if (lateral <= 1.5 * ds) {
                            return true;
                        }
                    }
                    return false;
                };
                if (nearStart) {
                    std::vector<std::uint32_t> log;
                    const Trace t = trace(b.position().head<2>(), i, &log, heading);
                    if (t.cut && t.actual.size() >= 2 && (clearOver || joins(t))) {
                        emit(t, log);
                        if (t.arrived) {
                            break;
                        }
                        i = std::max(t.seg + 1, i + 1);
                        continue;
                    }
                    grid.restore(log);
                }
                const auto e = findEntry(nominal[i], tangentAt(i), normalAt(i));
                if (!e) {
                    if (dbg) {
                        std::fprintf(stderr, "DBG   no entry at %zu/%zu (%.2f, %.2f)\n", i,
                                     nominal.size(), nominal[i].x(), nominal[i].y());
                    }
                    ++i;
                    continue;
                }
                goTo(*e);
                std::vector<std::uint32_t> log;
                const Trace t = trace(*e, i, &log, std::nullopt);
                emit(t, log);
                if (t.arrived) {
                    break;
                }
                i = std::max(t.seg + 1, i + 1);
            }
        }
        if (dbg) {
            std::fprintf(stderr, "DBG pass %d worst accepted engagement %.1f deg (limit %.1f)\n",
                         pass, radToDeg(worstAccepted), radToDeg(thetaMax));
        }
        // No progress (material the passes cannot take): stop rather than loop forever.
        const Regions after = geom2d::subtract(remaining, geom2d::sweep(cutPaths, r, clip), clip);
        if (areaBefore - geom2d::totalArea(after) < 1e-3) {
            reportLeftover(after);
            break;
        }
    }
    return report;
}

} // namespace ocam::strategies::detail
