// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Obstacle avoidance (DL-0034). The safe tip height over an XY position is the drop-cutter
// height of each solid of the tool assembly on the obstacle mesh, less the height of that
// solid's bottom above the tip; the highest of them wins. A toolpath is walked move by move,
// sampled finely enough that no obstacle (widened by the tool's radius and the clearance) fits
// between two samples, and cut where a sample lies below the safe height.

#include "ocam/strategies/Avoidance.hpp"
#include "ocam/strategies/Surface.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/tooling/Profile.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace ocam::strategies {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

/// A slice of the solid above the cutting part: heights [h0, h1] above the tip, radius r.
struct Band {
    double h0 = 0.0;
    double h1 = 0.0;
    double r = 0.0;
};

/// Bands of a (radius, height) outline's segments within heights [lo, hi], shifted up by
/// `shift`; each at the larger radius of its ends (the segments are short: the outline is
/// discretized).
void addBands(const std::vector<Vec2>& pts, double shift, double lo, double hi,
              std::vector<Band>& out) {
    for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
        Vec2 a = pts[k] + Vec2(0.0, shift);
        Vec2 b = pts[k + 1] + Vec2(0.0, shift);
        if (a.y() > b.y()) {
            std::swap(a, b);
        }
        if (b.y() - a.y() < 1e-9 || b.y() <= lo || a.y() >= hi) {
            continue;
        }
        auto radiusAt = [&](double h) {
            return a.x() + (b.x() - a.x()) * (h - a.y()) / (b.y() - a.y());
        };
        const double h0 = std::max(a.y(), lo);
        const double h1 = std::min(b.y(), hi);
        out.push_back({h0, h1, std::max(radiusAt(h0), radiusAt(h1))});
    }
}

} // namespace

struct ObstacleField::Impl {
    std::vector<DropCutter> pieces;
    std::vector<double> bases; ///< Height of each piece's bottom above the tip.
    geom3d::Aabb3 bounds;
    double ceiling = -kInf;
    double reach = 0.0;
    double cutterReach = 0.0;
};

ObstacleField::ObstacleField(const geom3d::Mesh& obstacles, const tooling::Tool& tool,
                             const tooling::Holder* holder, double clearance)
    : impl_(std::make_unique<Impl>()) {
    if (tooling::isTurningTool(tool.type)) {
        throw Error(ErrorCode::InvalidArgument, "obstacle avoidance needs a rotating tool");
    }
    if (clearance < 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the clearance must not be negative");
    }
    const tooling::ToolProfile profile = tooling::toolProfile(tool);
    Impl& m = *impl_;
    m.bounds = obstacles.bounds();
    if (obstacles.triangles.empty()) {
        return;
    }
    const double c = clearance;

    // The cutting part: exact for torus tools, else a cylinder of the largest cutting radius.
    std::optional<Cutter> cutter;
    try {
        const Cutter exact(tool);
        if (exact.kind() == Cutter::Kind::Torus && exact.radius() >= profile.maxRadius - 1e-9) {
            cutter = exact.offset(c);
        }
    } catch (const Error&) {
        // other shapes: the cylinder below
    }
    if (!cutter) {
        cutter = Cutter::torus(profile.maxRadius + c, 0.0);
    }
    m.cutterReach = cutter->radius();
    m.pieces.emplace_back(obstacles, *cutter);
    m.bases.push_back(-c);

    // Above it: the tool body up to the holder face, then the holder. Every piece reaches up
    // without end, so a slice only matters where the assembly grows wider than everything
    // below it; slices close above a piece widen that piece instead (fewer drop-cutters, a
    // little more room).
    const double visible = tooling::visibleLength(tool);
    std::vector<Band> bands;
    addBands(profile.outline.discretize(0.01), 0.0, profile.cuttingLength,
             std::min(visible, profile.length), bands);
    double top = std::min(visible, profile.length);
    if (holder && holder->validate().empty()) {
        addBands(holder->profile.discretize(0.01), visible, -kInf, kInf, bands);
        top = visible + holder->length();
    }
    std::sort(bands.begin(), bands.end(), [](const Band& a, const Band& b) { return a.h0 < b.h0; });
    const double window = std::max(2.0, top / 16.0);
    std::vector<std::pair<double, double>> steps; // (bottom height, radius)
    double widest = profile.maxRadius;
    for (const Band& b : bands) {
        if (b.r <= widest + 1e-6) {
            continue;
        }
        if (!steps.empty() && b.h0 - steps.back().first <= window) {
            steps.back().second = b.r;
        } else {
            steps.emplace_back(b.h0, b.r);
        }
        widest = b.r;
    }
    m.reach = m.cutterReach;
    for (const auto& [h0, r] : steps) {
        m.pieces.emplace_back(obstacles, Cutter::torus(r + c, 0.0));
        m.bases.push_back(h0 - c);
        m.reach = std::max(m.reach, r + c);
    }
    // No piece reaches below its bottom, and the lowest bottom is the cutter's.
    m.ceiling = m.bounds.max.z() + c;
}

ObstacleField::~ObstacleField() = default;
ObstacleField::ObstacleField(ObstacleField&&) noexcept = default;
ObstacleField& ObstacleField::operator=(ObstacleField&&) noexcept = default;

double ObstacleField::safeHeight(const Vec2& xy) const {
    const Impl& m = *impl_;
    double best = -kInf;
    for (std::size_t i = 0; i < m.pieces.size(); ++i) {
        const double h = m.pieces[i].drop(xy, -kInf);
        if (h > -kInf) {
            best = std::max(best, h - m.bases[i]);
        }
    }
    return best;
}

double ObstacleField::ceiling() const noexcept {
    return impl_->ceiling;
}

double ObstacleField::reach() const noexcept {
    return impl_->reach;
}

const geom3d::Aabb3& ObstacleField::bounds() const noexcept {
    return impl_->bounds;
}

double ObstacleField::cutterReach() const noexcept {
    return impl_->cutterReach;
}

std::size_t ObstacleField::pieceCount() const noexcept {
    return impl_->pieces.size();
}

bool ObstacleField::isEmpty() const noexcept {
    return impl_->pieces.empty();
}

AvoidanceResult avoidObstacles(toolpath::Toolpath& path, const ObstacleField& field,
                               const AvoidanceParams& params, CancelToken& cancel) {
    using toolpath::FeedClass;
    using toolpath::Move;
    using toolpath::MoveKind;

    AvoidanceResult result;
    if (field.isEmpty() || path.moves.empty()) {
        return result;
    }
    if (toolpath::isMultiAxis(path)) {
        throw Error(ErrorCode::InvalidArgument, "obstacle avoidance needs a 3-axis toolpath");
    }
    // Obstacles are widened by at least the cutter's reach, so plateaus of the safe height are
    // at least twice that wide: samples half of it apart cannot step over one.
    const double step = std::clamp(0.5 * field.cutterReach(), 0.02, 1.0);
    const geom3d::Aabb3& ob = field.bounds();
    const double reach = field.reach();
    const double ceiling = field.ceiling();
    constexpr double kTol = 1e-4;

    auto safe = [&](const Vec2& xy) {
        return field.safeHeight(xy);
    };
    auto isFree = [&](const Vec3& p) {
        return p.z() >= safe(p.head<2>()) - kTol;
    };
    auto outOfReach = [&](const Vec2& a, const Vec2& b) {
        return std::max(a.x(), b.x()) < ob.min.x() - reach ||
               std::min(a.x(), b.x()) > ob.max.x() + reach ||
               std::max(a.y(), b.y()) < ob.min.y() - reach ||
               std::min(a.y(), b.y()) > ob.max.y() + reach;
    };
    auto farAway = [&](const Vec3& a, const Vec3& b) {
        return std::min(a.z(), b.z()) >= ceiling || outOfReach(a.head<2>(), b.head<2>());
    };
    auto samples = [&](double length) {
        return std::max(1, static_cast<int>(std::ceil(length / step)));
    };
    auto segmentFree = [&](const Vec3& a, const Vec3& b) {
        if (farAway(a, b)) {
            return true;
        }
        const int n = samples((b - a).norm());
        for (int i = 0; i <= n; ++i) {
            if (!isFree(a + (b - a) * (static_cast<double>(i) / n))) {
                return false;
            }
        }
        return true;
    };
    // The highest safe height along the straight XY line from a to b.
    auto neededAlong = [&](const Vec2& a, const Vec2& b) {
        if (outOfReach(a, b)) {
            return -kInf;
        }
        double need = -kInf;
        const int n = samples((b - a).norm());
        for (int i = 0; i <= n; ++i) {
            need = std::max(need, safe(a + (b - a) * (static_cast<double>(i) / n)));
        }
        return need;
    };

    toolpath::Toolpath out = path;
    out.moves.clear();
    Vec3 cur = path.start; // where the tool is
    if (!isFree(cur)) {
        cur.z() = safe(cur.head<2>()) + 1e-3;
        out.start = cur;
    }
    Vec3 pos = cur;      // where the original path is
    bool lifted = false; // the tool is not at `pos` (it stopped short of an obstacle)
    bool counted = true; // the current lift has been counted as a trimmed section

    auto push = [&](const Move& m) {
        out.moves.push_back(m);
        cur = m.end;
    };
    auto rapid = [&](const Vec3& to) {
        if ((to - cur).norm() > 1e-9) {
            Move m;
            m.kind = MoveKind::Rapid;
            m.feedClass = FeedClass::Rapid;
            m.end = to;
            push(m);
        }
    };
    // Over the obstacles from the tool's position to the free point `to`: straight up (free
    // above a free point), across, and down; to a cut, the last part down is a feed plunge
    // from the retract height.
    auto linkTo = [&](const Vec3& to, bool toCut) {
        double level =
            std::max({neededAlong(cur.head<2>(), to.head<2>()) + 1e-3, params.retract, cur.z()});
        if (!toCut) {
            level = std::max(level, to.z());
        }
        rapid({cur.x(), cur.y(), level});
        rapid({to.x(), to.y(), level});
        if (toCut && to.z() < params.retract) {
            rapid({to.x(), to.y(), params.retract});
            Move plunge;
            plunge.kind = MoveKind::Linear;
            plunge.feedClass = FeedClass::Plunge;
            plunge.feedRate = params.plungeFeed;
            plunge.end = to;
            push(plunge);
        } else {
            rapid(to);
        }
    };
    // Stops short of an obstacle. A lift in a feed move counts as a trimmed section at once;
    // one in a rapid counts when a feed move is left out under it.
    auto lift = [&](bool inFeed) {
        lifted = true;
        counted = inFeed;
        if (inFeed) {
            ++result.trimmedSections;
        }
    };

    // A straight feed move from `pos` to `b` with the feed class and rate of `proto`.
    auto feedSegment = [&](const Vec3& b, const Move& proto) {
        const Vec3 a = pos;
        Move line = proto;
        line.kind = MoveKind::Linear;
        if (!lifted && farAway(a, b)) {
            line.end = b;
            push(line);
            pos = b;
            return;
        }
        const double length = (b - a).norm();
        auto at = [&](double t) {
            return Vec3(a + (b - a) * t);
        };
        // The boundary between a free (tf) and a blocked (tb) parameter, on the free side.
        auto boundary = [&](double tf, double tb) {
            for (int it = 0; it < 50 && std::abs(tb - tf) * length > 1e-4; ++it) {
                const double tm = 0.5 * (tf + tb);
                (isFree(at(tm)) ? tf : tb) = tm;
            }
            return tf;
        };
        double blockedFrom = 0.0;
        const bool startFree = isFree(a);
        if (lifted && startFree) {
            linkTo(a, true);
            lifted = false;
        } else if (!lifted && !startFree) {
            lift(true); // cannot happen after a free end; stay on the safe side
        }
        if (lifted && !counted) {
            ++result.trimmedSections;
            counted = true;
        }
        const int n = samples(length);
        double prevT = 0.0;
        for (int i = 1; i <= n; ++i) {
            const double t = static_cast<double>(i) / n;
            const bool f = isFree(at(t));
            if (!lifted && !f) {
                const double tb = boundary(prevT, t);
                if ((at(tb) - cur).norm() > 1e-9) {
                    line.end = at(tb);
                    push(line);
                }
                lift(true);
                blockedFrom = tb;
            } else if (lifted && f) {
                const double tf = boundary(t, prevT);
                result.trimmedLength += (tf - blockedFrom) * length;
                linkTo(at(tf), true);
                lifted = false;
            }
            prevT = t;
        }
        if (lifted) {
            result.trimmedLength += (1.0 - blockedFrom) * length;
        } else if ((b - cur).norm() > 1e-9) {
            line.end = b;
            push(line);
        }
        pos = b;
    };

    for (const Move& m : path.moves) {
        cancel.throwIfCancelled();
        switch (m.kind) {
        case MoveKind::Rapid: {
            const Vec3 b = m.end;
            if (!lifted) {
                if (segmentFree(pos, b)) {
                    push(m);
                } else if (isFree(b)) {
                    linkTo(b, false);
                    ++result.raisedRapids;
                } else {
                    lift(false);
                }
            } else if (isFree(b)) {
                linkTo(b, false);
                lifted = false;
            }
            pos = b;
            break;
        }
        case MoveKind::Linear:
            feedSegment(m.end, m);
            break;
        case MoveKind::ArcCW:
        case MoveKind::ArcCCW: {
            const std::vector<Vec3> pts =
                toolpath::sampleArc(pos, m, std::max(params.chordTolerance, 1e-4));
            bool whole = !lifted;
            for (std::size_t k = 0; whole && k + 1 < pts.size(); ++k) {
                whole = segmentFree(pts[k], pts[k + 1]);
            }
            if (whole) {
                push(m);
                pos = m.end;
            } else {
                for (std::size_t k = 1; k < pts.size(); ++k) {
                    feedSegment(pts[k], m);
                }
            }
            break;
        }
        case MoveKind::Dwell:
            if (!lifted) {
                push(m);
            }
            break;
        case MoveKind::Probe:
            // Probing is not rerouted: a probe that meets the workholding stops (DL-0090).
            throw Error(ErrorCode::Unsupported, "probing moves are not rerouted around obstacles");
        case MoveKind::Cycle: {
            const toolpath::DrillCycle& cycle = path.cycles.at(static_cast<std::size_t>(m.cycle));
            if (cycle.bottomZ < safe(m.end.head<2>()) - kTol) {
                ++result.skippedHoles;
                if (!lifted) {
                    lift(false);
                }
                pos = m.end;
                break;
            }
            // The cycle starts with a move across at the current height.
            const Vec3 above(m.end.x(), m.end.y(), cur.z());
            if (lifted || !segmentFree(cur, above)) {
                linkTo(m.end, false);
                lifted = false;
            }
            push(m);
            pos = m.end;
            break;
        }
        }
    }
    if (lifted) {
        rapid({cur.x(), cur.y(), std::max(cur.z(), pos.z())});
    }

    if (result.trimmedSections > 0) {
        out.warnings.push_back(fmt::format(
            "kept {:g} mm clear of the {}: {} stretch{} of cutting ({:.1f} mm) left out",
            params.clearance, params.name, result.trimmedSections,
            result.trimmedSections == 1 ? "" : "es", result.trimmedLength));
    }
    if (result.raisedRapids > 0) {
        out.warnings.push_back(fmt::format("{} rapid move{} raised over the {}",
                                           result.raisedRapids, result.raisedRapids == 1 ? "" : "s",
                                           params.name));
    }
    if (result.skippedHoles > 0) {
        out.warnings.push_back(fmt::format("{} hole{} left out: the {} are in the way",
                                           result.skippedHoles, result.skippedHoles == 1 ? "" : "s",
                                           params.name));
    }
    path = std::move(out);
    return result;
}

} // namespace ocam::strategies
