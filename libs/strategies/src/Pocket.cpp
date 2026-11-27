// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// 2D pocket: contour-parallel offset loops, innermost first, with helix / ramp / plunge entry
// and residual clean-up. See docs/design-log/0007 (method after Held, 1991).

#include "Clearing.hpp"
#include "Internal.hpp"
#include "ocam/strategies/Strategies.hpp"
#include "ocam/strategies/Surface.hpp"

#include <ocam/geom2d/Clip.hpp>
#include <ocam/geom2d/PathOps.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <optional>

namespace ocam::strategies {
namespace {

using geom2d::Polyline;
using geom2d::Region;
using geom2d::Regions;
using toolpath::FeedClass;

struct Node {
    Region region; ///< Area enclosed by this node's loops (tool-centre space).
    int parent = -1;
    std::vector<int> children;
    std::vector<Polyline> loops; ///< Cutting loops: holes first, outer last.
};

/// Offset loops of a tool-centre area and the order to cut them: a node's children (its
/// inside) before the node itself, so every loop cuts at most one stepover of new material.
struct LoopTree {
    std::vector<Node> nodes;
    std::vector<int> order;
};

/// With `coverRadius` > 0 (the radius a pass covers to each side: a flat bottom's), bands are
/// closed: offsets a whole stepover apart leave the last loops from either side of a band up to
/// two stepovers apart, and where a node's region holds points farther than that from its own
/// loop and from its children's, a loop half a stepover in is added between them. (With the
/// stepover at most twice the cover radius that covers everything.)
LoopTree buildLoopTree(const Regions& centreArea, double stepover, bool climb, double tol,
                       CancelToken& cancel, double coverRadius = 0.0) {
    const geom2d::ClipOptions clip{tol};
    LoopTree tree;
    std::vector<int> previousLevel;
    for (int k = 0; k < 100000; ++k) {
        cancel.throwIfCancelled();
        const Regions level = k == 0 ? centreArea : geom2d::offset(centreArea, -k * stepover, clip);
        if (level.empty()) {
            break;
        }
        std::vector<int> current;
        for (const Region& region : level) {
            Node node;
            node.region = region;
            const Vec2 inside = geom2d::interiorPoint(region, tol);
            for (int parent : previousLevel) {
                if (tree.nodes[static_cast<std::size_t>(parent)].region.contains(inside, tol)) {
                    node.parent = parent;
                    break;
                }
            }
            auto prepare = [&](const Polyline& loop) {
                Polyline fitted = geom2d::fitArcs(loop.discretize(tol), true, 2.0 * tol);
                // Region orientation (outer CCW, holes CW) keeps the wall on the right: climb.
                if (!climb) {
                    fitted.reverse();
                }
                return fitted;
            };
            for (const Polyline& hole : region.holes) {
                node.loops.push_back(prepare(hole));
            }
            node.loops.push_back(prepare(region.outer));
            const int id = static_cast<int>(tree.nodes.size());
            if (node.parent >= 0) {
                tree.nodes[static_cast<std::size_t>(node.parent)].children.push_back(id);
            }
            tree.nodes.push_back(std::move(node));
            current.push_back(id);
        }
        previousLevel = std::move(current);
    }
    if (coverRadius > 0.0) {
        const auto prepare = [&](const Polyline& loop) {
            Polyline fitted = geom2d::fitArcs(loop.discretize(tol), true, 2.0 * tol);
            if (!climb) {
                fitted.reverse();
            }
            return fitted;
        };
        const std::size_t original = tree.nodes.size();
        for (std::size_t id = 0; id < original; ++id) {
            cancel.throwIfCancelled();
            const Region region = tree.nodes[id].region;
            const std::vector<int> children = tree.nodes[id].children;
            // Points of the region out of the node's own loop's reach and of its children's.
            Regions out = geom2d::offset(Regions{region}, -coverRadius, clip);
            if (out.empty()) {
                continue;
            }
            Regions inner;
            for (int child : children) {
                inner.push_back(tree.nodes[static_cast<std::size_t>(child)].region);
            }
            if (!inner.empty()) {
                out = geom2d::subtract(out, geom2d::offset(inner, coverRadius, clip), clip);
            }
            out = geom2d::offset(geom2d::offset(out, -tol, clip), tol, clip);
            if (geom2d::totalArea(out) < 1e-3) {
                continue;
            }
            const Regions half = geom2d::offset(Regions{region}, -0.5 * stepover, clip);
            if (half.empty()) {
                continue;
            }
            // The half-step pieces become the node's children, its children theirs (each child,
            // a whole stepover in, lies in one of them).
            std::vector<int> pieces;
            for (const Region& piece : half) {
                Node node;
                node.region = piece;
                node.parent = static_cast<int>(id);
                for (const Polyline& hole : piece.holes) {
                    node.loops.push_back(prepare(hole));
                }
                node.loops.push_back(prepare(piece.outer));
                pieces.push_back(static_cast<int>(tree.nodes.size()));
                tree.nodes.push_back(std::move(node));
            }
            for (int child : children) {
                Node& c = tree.nodes[static_cast<std::size_t>(child)];
                const Vec2 inside = geom2d::interiorPoint(c.region, tol);
                int into = pieces.front();
                for (int piece : pieces) {
                    if (tree.nodes[static_cast<std::size_t>(piece)].region.contains(inside, tol)) {
                        into = piece;
                        break;
                    }
                }
                c.parent = into;
                tree.nodes[static_cast<std::size_t>(into)].children.push_back(child);
            }
            tree.nodes[id].children = pieces;
        }
    }
    std::function<void(int)> visit = [&](int id) {
        for (int child : tree.nodes[static_cast<std::size_t>(id)].children) {
            visit(child);
        }
        tree.order.push_back(id);
    };
    for (int id = 0; id < static_cast<int>(tree.nodes.size()); ++id) {
        if (tree.nodes[static_cast<std::size_t>(id)].parent < 0) {
            visit(id);
        }
    }
    return tree;
}

class PocketCutter {
public:
    PocketCutter(toolpath::Toolpath& tp, toolpath::PathBuilder& b, const PocketParams& p,
                 const Context& ctx, const Regions& allowed, double retract)
        : tp_(tp), b_(b), p_(p), ctx_(ctx), allowed_(allowed), retract_(retract),
          r_(ctx.tool.mill.diameter / 2.0) {}

    /// Opt-in behaviour for flat area finishing; the defaults keep pocketing and roughing as
    /// they are.
    struct Options {
        /// Links at depth longer than this lift and enter again.
        double maxLink = std::numeric_limits<double>::infinity();
        /// Moves to a loop of the same connected area hop at the approach height instead of the
        /// retract height.
        bool lowHops = false;
        /// For a region's outer loop: a point outside the stock to plunge at and the point of
        /// the loop to feed to from there. Regions with one are cut from the outside in.
        std::function<std::optional<std::pair<Vec2, Vec2>>(const Polyline& loop)> sideEntry;
        /// Clean-up loops (`uncut` false) enter by a plunge: the floor around them is cut.
        bool plungeCleanup = false;
    };
    void setOptions(Options options) { opt_ = std::move(options); }

    /// Cuts all loops of `tree` at height z. With `uncut` the innermost loops enter fresh
    /// material (helix/ramp/plunge per parameters); otherwise the area is assumed mostly
    /// cleared (clean-up) and entries ramp along the loop.
    ///
    /// Z-level roughing and rest machining pass `plungeForbidden` (tool centres that would
    /// touch material at or above this level). A region whose outer loop reaches outside that
    /// zone (open to the side) is then cut from the outside in and entered by a plunge in air; a
    /// closed region (a pocket) is cut from the inside out and entered like a pocket: helix,
    /// ramp or plunge per parameters.
    ///
    /// With `material` (the material of this level), loops of open regions that do not reach it
    /// are not followed: the tool plunges at their air point and steps on to the next loop.
    void cut(const LoopTree& tree, double z, double approachZ, bool uncut,
             const Regions* plungeForbidden = nullptr, const Regions* material = nullptr) {
        const double tol = ctx_.chordTolerance;
        const double d = 2.0 * r_;
        const bool climb = p_.direction == CutDirection::Climb;
        const double rampTan = std::tan(std::clamp(p_.rampAngle, degToRad(0.1), degToRad(45.0)));
        int lastNode = -1;
        for (const Step& step : cuttingOrder(tree, plungeForbidden)) {
            ctx_.cancel.throwIfCancelled();
            const int id = step.node;
            const std::size_t loopIndex = step.loop;
            const bool outsideIn = step.outsideIn;
            const Node& node = tree.nodes[static_cast<std::size_t>(id)];
            const bool leaf = node.children.empty();
            // Above an entry point: within the same connected area a low hop just above this
            // level (with lowHops), otherwise over the retract height.
            const auto approach = [&](const Vec2& xy) {
                const Vec3 here = b_.position();
                if (opt_.lowHops && here.z() <= approachZ + 1e-9) {
                    int top = id;
                    while (tree.nodes[static_cast<std::size_t>(top)].parent >= 0) {
                        top = tree.nodes[static_cast<std::size_t>(top)].parent;
                    }
                    const Regions hopArea =
                        geom2d::offset(Regions{tree.nodes[static_cast<std::size_t>(top)].region},
                                       4.0 * tol, {tol});
                    if (geom2d::segmentInside(hopArea, here.head<2>(), xy, tol)) {
                        b_.retractTo(approachZ);
                        b_.rapid(Vec3(xy.x(), xy.y(), approachZ));
                        return;
                    }
                }
                b_.rapidAbove(xy, retract_, approachZ);
            };
            {
                const Polyline loop =
                    geom2d::restartAtNearest(node.loops[loopIndex], b_.position().head<2>());
                const Vec2 start = loop.vertices().front().p;
                const int lastParent =
                    lastNode >= 0 ? tree.nodes[static_cast<std::size_t>(lastNode)].parent : -1;
                // From the outside in, the parent's loop has cleared the band its children and
                // their siblings start from.
                const bool continuing =
                    lastNode >= 0 &&
                    (lastNode == id || lastParent == id ||
                     (outsideIn && (node.parent == lastNode ||
                                    (node.parent >= 0 && lastParent == node.parent))));
                // Links start and end on arc-fitted loops, which may lie a hair outside their
                // regions: test against the region grown by a few tolerances.
                const Regions linkArea =
                    outsideIn ? geom2d::offset(
                                    Regions{tree.nodes[static_cast<std::size_t>(
                                                           node.parent >= 0 ? node.parent : id)]
                                                .region},
                                    4.0 * tol, {tol})
                              : (uncut ? geom2d::offset(Regions{node.region}, 4.0 * tol, {tol})
                                       : allowed_);
                // From the outside in the inside is still uncut: only short links stay down
                // (about a stepover; more at corners), longer ones enter again.
                const double stepover = p_.stepoverRatio * d;
                const double linkLength = (b_.position().head<2>() - start).norm();
                const bool shortLink = (!outsideIn || linkLength <= 2.0 * stepover + tol) &&
                                       linkLength <= opt_.maxLink;
                if ((continuing || !uncut) && shortLink && std::abs(b_.position().z() - z) < 1e-9 &&
                    geom2d::segmentInside(linkArea, b_.position().head<2>(), start, tol)) {
                    // Step across to the next loop through at most one stepover of material.
                    b_.feed(Vec3(start.x(), start.y(), z), FeedClass::Cut, ctx_.feeds.cut);
                    b_.follow(loop, z, FeedClass::Cut, ctx_.feeds.cut);
                    lastNode = id;
                    continue;
                }
                if (outsideIn) {
                    // An open flat: plunge outside the stock and feed in to the outer loop.
                    const bool rootOuter = node.parent < 0 && loopIndex + 1 == node.loops.size();
                    if (const auto side = !plungeForbidden && opt_.sideEntry && rootOuter
                                              ? opt_.sideEntry(node.loops[loopIndex])
                                              : std::nullopt) {
                        approach(side->first);
                        b_.feed(Vec3(side->first.x(), side->first.y(), z), FeedClass::Plunge,
                                ctx_.feeds.plunge);
                        const Polyline fromSide =
                            geom2d::restartAtNearest(node.loops[loopIndex], side->second);
                        const Vec2 s = fromSide.vertices().front().p;
                        b_.feed(Vec3(s.x(), s.y(), z), FeedClass::LeadIn, ctx_.feeds.lead);
                        b_.follow(fromSide, z, FeedClass::Cut, ctx_.feeds.cut);
                        lastNode = id;
                        continue;
                    }
                    if (const auto entry = plungeForbidden
                                               ? airEntry(node.loops[loopIndex], *plungeForbidden)
                                               : std::nullopt) {
                        const Polyline fromAir =
                            geom2d::restartAtNearest(node.loops[loopIndex], *entry);
                        const Vec2 s = fromAir.vertices().front().p;
                        if (material && !reaches(node.loops[loopIndex], *material)) {
                            // All air: skip it. The next loop enters at its own air point, or
                            // by a plunge onto this loop (outside the stock) next to its start.
                            continue;
                        }
                        approach(s);
                        b_.feed(Vec3(s.x(), s.y(), z), FeedClass::Plunge, ctx_.feeds.plunge);
                        b_.follow(fromAir, z, FeedClass::Cut, ctx_.feeds.cut);
                        lastNode = id;
                        continue;
                    }
                    // Otherwise plunge into the slot the enclosing loop (the parent's outer loop,
                    // or this node's own for its island loops) has cut at this depth, and step
                    // over from there.
                    const bool hole = loopIndex + 1 < node.loops.size();
                    const int enclosing = hole ? id : node.parent;
                    if (enclosing >= 0) {
                        const Polyline& cutLoop =
                            tree.nodes[static_cast<std::size_t>(enclosing)].loops.back();
                        const Vec2 q =
                            geom2d::restartAtNearest(cutLoop, start).vertices().front().p;
                        if (geom2d::segmentInside(linkArea, q, start, tol)) {
                            approach(q);
                            b_.feed(Vec3(q.x(), q.y(), z), FeedClass::Plunge, ctx_.feeds.plunge);
                            b_.feed(Vec3(start.x(), start.y(), z), FeedClass::Cut, ctx_.feeds.cut);
                            b_.follow(loop, z, FeedClass::Cut, ctx_.feeds.cut);
                            lastNode = id;
                            continue;
                        }
                    }
                }

                if (!uncut && opt_.plungeCleanup) {
                    approach(start);
                    b_.feed(Vec3(start.x(), start.y(), z), FeedClass::Plunge, ctx_.feeds.plunge);
                    b_.follow(loop, z, FeedClass::Cut, ctx_.feeds.cut);
                    lastNode = id;
                    continue;
                }
                bool entered = false;
                const bool freshInside = uncut && leaf && loopIndex + 1 == node.loops.size();
                const double rh = p_.helixRadius.value_or(kHelixRadiusShare * r_);
                if (freshInside && p_.entry == EntryKind::Helix && rh > 0.05 * r_) {
                    if (const auto c = helixCentre(tree, id, rh)) {
                        approach(*c + Vec2(rh, 0.0));
                        b_.helix(*c, rh, z, 2.0 * kPi * rh * rampTan, !climb, ctx_.feeds.ramp);
                        const Polyline fromHelix = geom2d::restartAtNearest(
                            node.loops[loopIndex], b_.position().head<2>());
                        const Vec2 s = fromHelix.vertices().front().p;
                        b_.feed(Vec3(s.x(), s.y(), z), FeedClass::Cut, ctx_.feeds.cut);
                        b_.follow(fromHelix, z, FeedClass::Cut, ctx_.feeds.cut);
                        entered = true;
                    }
                }
                if (!entered && p_.entry != EntryKind::Plunge && loop.length() >= 0.5 * d) {
                    approach(start);
                    b_.rampAlong(loop, z, p_.rampAngle, ctx_.feeds.ramp);
                    b_.follow(loop, z, FeedClass::Cut, ctx_.feeds.cut);
                    entered = true;
                }
                if (!entered) {
                    if (p_.entry != EntryKind::Plunge && uncut && !warnedPlunge_) {
                        tp_.warnings.push_back(
                            "plunge entry used where a helix or ramp does not fit");
                        warnedPlunge_ = true;
                    }
                    approach(start);
                    b_.feed(Vec3(start.x(), start.y(), z), FeedClass::Plunge, ctx_.feeds.plunge);
                    b_.follow(loop, z, FeedClass::Cut, ctx_.feeds.cut);
                }
                lastNode = id;
            }
        }
    }

private:
    struct Step {
        int node = -1;
        std::size_t loop = 0;   ///< Index into the node's loops.
        bool outsideIn = false; ///< Part of an open region cut from the outside in.
    };

    /// The loops in cutting order. Without `plungeForbidden`, from the inside out (a node's
    /// children before the node). With it, each open region from the outside in: a node's outer
    /// loop, its children, then its island loops (which lie next to the islands); and each closed
    /// region from the inside out.
    std::vector<Step> cuttingOrder(const LoopTree& tree, const Regions* plungeForbidden) const {
        std::vector<Step> steps;
        const auto nodeAt = [&](int id) -> const Node& {
            return tree.nodes[static_cast<std::size_t>(id)];
        };
        std::function<void(int)> insideOut = [&](int id) {
            for (int child : nodeAt(id).children) {
                insideOut(child);
            }
            for (std::size_t k = 0; k < nodeAt(id).loops.size(); ++k) {
                steps.push_back({id, k, false});
            }
        };
        std::function<void(int)> outsideIn = [&](int id) {
            const std::size_t outer = nodeAt(id).loops.size() - 1;
            steps.push_back({id, outer, true});
            for (int child : nodeAt(id).children) {
                outsideIn(child);
            }
            for (std::size_t k = 0; k < outer; ++k) {
                steps.push_back({id, k, true});
            }
        };
        if (!plungeForbidden && !opt_.sideEntry) {
            for (int id : tree.order) {
                for (std::size_t k = 0; k < nodeAt(id).loops.size(); ++k) {
                    steps.push_back({id, k, false});
                }
            }
            return steps;
        }
        for (auto it = tree.order.rbegin(); it != tree.order.rend(); ++it) {
            if (nodeAt(*it).parent >= 0 || nodeAt(*it).loops.empty()) {
                continue;
            }
            const bool open = plungeForbidden
                                  ? airEntry(nodeAt(*it).loops.back(), *plungeForbidden).has_value()
                                  : opt_.sideEntry(nodeAt(*it).loops.back()).has_value();
            if (open) {
                outsideIn(*it);
            } else {
                insideOut(*it);
            }
        }
        return steps;
    }

    /// Where a circular helix of radius `rh` (tool centre path) enters for the innermost loop of
    /// node `leaf`: inside the leaf's own area when the circle fits there; otherwise the point
    /// nearest to it where the circle fits inside the whole connected area being cleared (the
    /// tree's root above the leaf), when the straight move from the helix to the loop stays in
    /// that area. None when the helix fits nowhere.
    std::optional<Vec2> helixCentre(const LoopTree& tree, int leaf, double rh) const {
        const double tol = ctx_.chordTolerance;
        const geom2d::ClipOptions clip{tol};
        const Node& node = tree.nodes[static_cast<std::size_t>(leaf)];
        const Regions own = geom2d::offset(Regions{node.region}, -rh, clip);
        if (!own.empty()) {
            return geom2d::interiorPoint(own.front(), tol);
        }
        int top = leaf;
        while (tree.nodes[static_cast<std::size_t>(top)].parent >= 0) {
            top = tree.nodes[static_cast<std::size_t>(top)].parent;
        }
        const Regions area{tree.nodes[static_cast<std::size_t>(top)].region};
        // A little more than rh, so the circle stays clear of the area's boundary.
        const Regions fits = geom2d::offset(area, -(rh + 2.0 * tol), clip);
        if (fits.empty()) {
            return std::nullopt;
        }
        const Vec2 target = geom2d::interiorPoint(node.region, tol);
        std::optional<Vec2> best;
        double bestDistance = std::numeric_limits<double>::infinity();
        for (const Region& region : fits) {
            if (region.contains(target, tol)) {
                best = target;
                break;
            }
            std::vector<const Polyline*> loops{&region.outer};
            for (const Polyline& hole : region.holes) {
                loops.push_back(&hole);
            }
            for (const Polyline* loop : loops) {
                const geom2d::ClosestPoint q = geom2d::closestPoint(*loop, target);
                if (q.distance < bestDistance) {
                    bestDistance = q.distance;
                    best = q.point;
                }
            }
        }
        if (!best) {
            return std::nullopt;
        }
        const Vec2 helixEnd = *best + Vec2(rh, 0.0);
        const Vec2 loopStart =
            geom2d::restartAtNearest(node.loops.back(), helixEnd).vertices().front().p;
        const Regions link = geom2d::offset(area, 4.0 * tol, clip);
        if (!geom2d::segmentInside(link, helixEnd, loopStart, tol)) {
            return std::nullopt;
        }
        return best;
    }

    /// True when the tool following `loop` touches `material` (more than a sliver).
    bool reaches(const Polyline& loop, const Regions& material) const {
        std::vector<Vec2> pts = loop.discretize(ctx_.chordTolerance);
        if (pts.empty()) {
            return false;
        }
        pts.push_back(pts.front());
        const Regions swept = geom2d::sweep({pts}, r_, {ctx_.chordTolerance});
        return geom2d::totalArea(geom2d::intersect(swept, material, {ctx_.chordTolerance})) >
               0.01 * r_ * r_;
    }

    /// The point of `loop` outside `forbidden` nearest to the tool, if any.
    std::optional<Vec2> airEntry(const Polyline& loop, const Regions& forbidden) const {
        const double tol = ctx_.chordTolerance;
        const std::vector<Vec2> pts = loop.discretize(tol);
        std::optional<Vec2> best;
        double bestDistance = std::numeric_limits<double>::infinity();
        const Vec2 here = b_.position().head<2>();
        for (std::size_t k = 0; k < pts.size(); ++k) {
            const Vec2& a = pts[k];
            const Vec2& b = pts[(k + 1) % pts.size()];
            const int steps = std::max(1, static_cast<int>(std::ceil((b - a).norm() / 0.5)));
            for (int t = 0; t < steps; ++t) {
                const Vec2 q = a + (b - a) * (static_cast<double>(t) / steps);
                const double dist = (q - here).norm();
                if (dist < bestDistance &&
                    std::none_of(forbidden.begin(), forbidden.end(),
                                 [&](const Region& f) { return f.contains(q, tol); })) {
                    best = q;
                    bestDistance = dist;
                }
            }
        }
        return best;
    }

    toolpath::Toolpath& tp_;
    toolpath::PathBuilder& b_;
    const PocketParams& p_;
    const Context& ctx_;
    const Regions& allowed_;
    double retract_;
    double r_;
    bool warnedPlunge_ = false;
    Options opt_;
};

} // namespace

toolpath::Toolpath pocket(const Regions& pocketArea, double topZ, double bottomZ,
                          const PocketParams& p, const Context& ctx) {
    using namespace detail;
    const double r = requireMillingTool(ctx.tool);
    const double d = 2.0 * r;
    if (pocketArea.empty()) {
        throw Error(ErrorCode::InvalidArgument, "pocket needs a boundary");
    }
    if (p.stepoverRatio <= 0.0 || p.stepoverRatio > 1.0) {
        throw Error(ErrorCode::InvalidArgument, "pocket stepover must be between 0 and 1 × D");
    }
    const double tol = ctx.chordTolerance;
    const geom2d::ClipOptions clip{tol};
    const double stepover = p.stepoverRatio * d;
    const bool climb = p.direction == CutDirection::Climb;

    const Regions allowed = geom2d::offset(pocketArea, -(r + p.stockToLeaveXY), clip);
    toolpath::Toolpath tp = beginToolpath(ctx, "pocket2d", pocketArea.front().bounds().center());
    if (allowed.empty()) {
        tp.warnings.push_back("the tool does not fit in the pocket");
        return tp;
    }
    const LoopTree tree = buildLoopTree(allowed, stepover, climb, tol, ctx.cancel);
    const Regions reachable = geom2d::offset(allowed, r, clip); // what a round tool can clear
    const auto levels = depthLevels(topZ, bottomZ + p.stockToLeaveZ, p.stepDown);
    if (levels.empty()) {
        tp.warnings.push_back("nothing to cut: the pocket top is at or below its floor");
        return tp;
    }
    tp.start.head<2>() =
        tree.nodes[static_cast<std::size_t>(tree.order.front())].loops.back().vertices().front().p;

    toolpath::PathBuilder b(tp);
    const double retract = std::max(ctx.heights.retract, topZ + ctx.heights.feedClearance);
    PocketCutter cutter(tp, b, p, ctx, allowed, retract);
    const double noise = std::max(10.0 * tol, 0.01);

    bool cutAnything = false;
    for (std::size_t li = 0; li < levels.size(); ++li) {
        const double z = levels[li];
        const double approachZ = (li == 0 ? topZ : levels[li - 1]) + ctx.heights.feedClearance;
        const std::size_t levelFirstMove = tp.moves.size();
        const Vec3 levelStart = b.position();

        // Rest machining: clear only tool-centre positions within reach of remaining material.
        // Every reachable remaining point x has a centre c in `allowed` with |x - c| <= R, and c
        // lies in allowed ∩ (remaining ⊕ R), so loops of that area plus the residual clean-up
        // below (targeting the remaining material) sweep all of it.
        const LoopTree* levelTree = &tree;
        Regions levelTarget = reachable;
        LoopTree restTree;
        Regions plungeForbidden;
        if (p.restMaterial) {
            const double zAbove =
                li == 0 ? std::numeric_limits<double>::infinity() : levels[li - 1];
            Regions remaining = geom2d::intersect(p.restMaterial(z, zAbove), reachable, clip);
            const double opening = std::max(noise, p.restOpening);
            remaining = geom2d::offset(geom2d::offset(remaining, -opening, clip), opening, clip);
            std::erase_if(remaining, [](const Region& piece) { return piece.area() < 1e-3; });
            if (remaining.empty()) {
                continue;
            }
            // Centres reach a little past the material so each region has a rim in air to
            // plunge into; plunges stay clear of all material above this level.
            const double rim = std::max(noise, 0.2 * r);
            const Regions centres =
                geom2d::intersect(allowed, geom2d::offset(remaining, r + rim, clip), clip);
            if (centres.empty()) {
                continue;
            }
            restTree = buildLoopTree(centres, stepover, climb, tol, ctx.cancel);
            if (restTree.order.empty()) {
                continue;
            }
            Regions above = p.restMaterial(z, std::numeric_limits<double>::infinity());
            above = geom2d::offset(geom2d::offset(above, -opening, clip), opening, clip);
            plungeForbidden = geom2d::offset(above, r + 0.5 * rim, clip);
            levelTree = &restTree;
            levelTarget = remaining;
        }
        cutAnything = true;
        cutter.cut(*levelTree, z, approachZ, true, p.restMaterial ? &plungeForbidden : nullptr);

        // Residual clean-up. Loops of S = allowed ∩ (residual ⊕ R) with a stepover of at most R
        // sweep every reachable point of the residual (see the design log), so a mini-pocket of S
        // removes what wide stepovers leave near corners and the medial axis.
        for (int pass = 0; pass < 2; ++pass) {
            const auto paths = pathsAtDepth(tp, levelFirstMove, levelStart, z, tol);
            Regions residual = geom2d::subtract(levelTarget, geom2d::sweep(paths, r, clip), clip);
            residual = geom2d::offset(geom2d::offset(residual, -noise, clip), noise, clip);
            std::erase_if(residual, [](const Region& piece) { return piece.area() < 1e-3; });
            if (residual.empty()) {
                break;
            }
            const Regions cleanupArea =
                geom2d::intersect(allowed, geom2d::offset(residual, r + noise, clip), clip);
            if (cleanupArea.empty()) {
                break;
            }
            const LoopTree cleanup =
                buildLoopTree(cleanupArea, std::min(stepover, 0.9 * r), climb, tol, ctx.cancel);
            cutter.cut(cleanup, z, approachZ, false);
        }
        b.retractTo(retract);
        ctx.progress.report(static_cast<double>(li + 1) / levels.size(), "Pocket");
    }
    if (!cutAnything) {
        tp.warnings.push_back("no remaining material to cut");
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

toolpath::Toolpath zLevelRough(const geom3d::Mesh& part, const Regions& stockOutline, double topZ,
                               double bottomZ, const ZLevelRoughParams& zp, const Context& ctx) {
    using namespace detail;
    const double r = requireMillingTool(ctx.tool);
    if (part.isEmpty()) {
        throw Error(ErrorCode::InvalidArgument, "Z-level roughing needs the part's surface");
    }
    if (stockOutline.empty()) {
        throw Error(ErrorCode::InvalidArgument, "Z-level roughing needs the stock outline");
    }
    if (zp.stepDown <= 0.0 || zp.stepoverRatio <= 0.0 || zp.stepoverRatio > 1.0) {
        throw Error(ErrorCode::InvalidArgument, "invalid step down or stepover");
    }
    const double tol = ctx.chordTolerance;
    const geom2d::ClipOptions clip{tol};
    const double stepover = zp.stepoverRatio * 2.0 * r;
    const bool climb = zp.direction == CutDirection::Climb;
    const double noise = std::max(10.0 * tol, 0.01);
    const double rim = std::max(noise, 0.2 * r);

    // Forbidden tool-centre areas come from the real tool's drop-cutter surface: the tip must
    // stay stock-to-leave (Z) above it, and the area grows by stock-to-leave (XY).
    const DropCutter dc(part, Cutter(ctx.tool));
    const geom3d::Aabb3 bounds = part.bounds();
    const double res = zp.resolution > 0.0 ? zp.resolution : std::clamp(r / 4.0, 0.1, 2.0);
    geom2d::Aabb2 area;
    for (const auto& region : stockOutline) {
        area.extend(region.bounds());
    }
    area.extend(bounds.min.head<2>());
    area.extend(bounds.max.head<2>());
    area.min -= Vec2::Constant(r + rim + 3.0 * res);
    area.max += Vec2::Constant(r + rim + 3.0 * res);
    const double floor = std::min(bounds.min.z(), bottomZ) - 10.0;
    const HeightField field = computeHeightField(dc, area, res, floor, ctx.cancel);

    toolpath::Toolpath tp = beginToolpath(ctx, "zlevel_rough", area.min);
    // Step-down levels. With flat area levels, every flat (its height plus the Z allowance) gets
    // a level of its own: the range is split at the flats and each part gets evenly spaced
    // levels ending on its flat, so no flat keeps up to a step of extra stock and every level
    // is still a whole level (open areas entered from outside the stock).
    struct Level {
        double z = 0.0;
        bool onFlat = false;
        /// A fine level (between the step-down levels): it takes only the stair the level below
        /// leaves on sloped and rounded walls.
        bool fine = false;
        std::size_t coarseBelow = 0; ///< For a fine level: the step-down level under it.
    };
    std::vector<double> flatHeights;
    if (zp.flatAreaLevels) {
        for (const FlatLevel& flat : flatLevels(part, degToRad(0.5), std::max(tol, 0.001), tol)) {
            const double z = flat.z + zp.stockToLeaveZ;
            // Flats a hair apart share the higher level (the lower one keeps a hair more).
            const double merge = std::max(0.01, 0.02 * zp.stepDown);
            if (z < topZ - 1e-6 && z > bottomZ + 1e-6 &&
                (flatHeights.empty() || flatHeights.back() - z > merge)) {
                flatHeights.push_back(z); // flatLevels() lists the highest first
            }
        }
    }
    std::vector<Level> coarseLevels;
    double segmentTop = topZ;
    for (std::size_t k = 0; k <= flatHeights.size(); ++k) {
        const bool onFlat = k < flatHeights.size();
        const double segmentBottom = onFlat ? flatHeights[k] : bottomZ;
        for (const double z : depthLevels(segmentTop, segmentBottom, zp.stepDown)) {
            coarseLevels.push_back({z, onFlat && std::abs(z - segmentBottom) < 1e-9});
        }
        segmentTop = segmentBottom;
    }
    if (coarseLevels.empty()) {
        tp.warnings.push_back("nothing to cut: the top is at or below the bottom");
        return tp;
    }
    // Fine levels: above each step-down level, evenly spaced at most the fine step down apart.
    std::vector<Level> levels;
    const bool useFine = zp.fineStepDown > 0.0 && zp.fineStepDown < zp.stepDown - 1e-9;
    segmentTop = topZ;
    for (const Level& coarse : coarseLevels) {
        if (useFine) {
            const std::vector<double> zs = depthLevels(segmentTop, coarse.z, zp.fineStepDown);
            for (std::size_t k = 0; k + 1 < zs.size(); ++k) {
                Level fine;
                fine.z = zs[k];
                fine.fine = true;
                fine.coarseBelow = levels.size() + (zs.size() - 1 - k); // the coarse one, below
                levels.push_back(fine);
            }
        }
        levels.push_back(coarse);
        segmentTop = coarse.z;
    }
    // On a flat's level the tip is exactly the allowance above the flat, which must not count
    // as forbidden.
    const double flatSlack = std::max(10.0 * tol, 0.002);

    toolpath::PathBuilder b(tp);
    const double retract = std::max(ctx.heights.retract, topZ + ctx.heights.feedClearance);
    bool cutAnything = false;
    int stalled = 0;
    double leftoverArea = 0.0;
    std::vector<Vec2> leftover;
    std::vector<Vec2> noEntry; // closed areas neither a helix nor a ramp fits in
    // The levels are planned before any is cut. Without in-process material, material is cut
    // at the deepest level the step down allows: a level's own material is what the tool can
    // reach there but not at the deepest level within a step down of the top of its column
    // (over the part's top faces, for instance); the rest is left to that deeper level, which
    // then takes a column from the same top in one go (the stock around a part lower than a
    // step down, in one pass instead of one per level). Levels run in the order of their
    // column tops, the deeper first among equals, so what a deeper column cleared is air to
    // the levels above it (the top of a boss is then entered from the cleared outside).
    // Fine levels take, after all the step-down levels, the band next to a sloped or rounded
    // wall that the step-down level under them cannot reach (the part is wider there). Like the
    // step-down levels, material is taken at the deepest fine level that reaches it, from the
    // top of the column (the step-down level above) in one go: the stock over a face between
    // the step-down levels by the fine level just above the face, the band along a sloped wall
    // ring by ring, each ring by the deepest level that reaches it. Deepest first, so each is
    // entered from the air cleared beside it.
    struct Plan {
        double z = 0.0;
        double zAbove = 0.0;    ///< The level above (for in-process material).
        double columnTop = 0.0; ///< The level's material stands from here.
        Regions forbidden;
        Regions partZone; ///< Material under the part and its allowance at this level.
        Regions material;
    };
    std::vector<std::size_t> coarseIndex; // positions of the step-down levels in `levels`
    for (std::size_t li = 0; li < levels.size(); ++li) {
        if (!levels[li].fine) {
            coarseIndex.push_back(li);
        }
    }
    std::vector<Plan> plans(levels.size());
    for (std::size_t li = 0; li < levels.size(); ++li) {
        Plan& plan = plans[li];
        plan.z = levels[li].z;
        if (levels[li].fine) {
            // The column stands from the step-down level above.
            const auto below =
                std::find(coarseIndex.begin(), coarseIndex.end(), levels[li].coarseBelow);
            const bool first = below == coarseIndex.begin();
            plan.zAbove = first ? std::numeric_limits<double>::infinity() : levels[*(below - 1)].z;
            plan.columnTop = first ? topZ : levels[*(below - 1)].z;
        } else {
            // The step-down levels see one another only (fine ones between change nothing).
            const auto it = std::find(coarseIndex.begin(), coarseIndex.end(), li);
            const bool first = it == coarseIndex.begin();
            plan.zAbove = first ? std::numeric_limits<double>::infinity() : levels[*(it - 1)].z;
            plan.columnTop = first ? topZ : levels[*(it - 1)].z;
        }
        const double slack = levels[li].onFlat ? flatSlack : 0.0;
        plan.forbidden = forbiddenRegions(dc, field, plan.z - zp.stockToLeaveZ + slack, floor,
                                          std::max(tol, 1e-3));
        if (zp.stockToLeaveXY > 0.0) {
            plan.forbidden = geom2d::offset(plan.forbidden, zp.stockToLeaveXY, clip);
        }
        // The in-process stock still holds the part: only material outside the part and its
        // allowance (the forbidden area shrunk back by the tool radius) is rest. Without this
        // every wall would be traced again.
        plan.partZone =
            geom2d::offset(plan.forbidden, -r + (zp.material ? zp.materialSlack : 0.0), clip);
    }
    auto tidy = [&](Regions regions) {
        regions = geom2d::offset(geom2d::offset(regions, -noise, clip), noise, clip);
        std::erase_if(regions, [](const Region& piece) { return piece.area() < 1e-3; });
        return regions;
    };
    std::vector<std::size_t> order = coarseIndex;
    if (zp.material) {
        for (const std::size_t li : coarseIndex) {
            Plan& plan = plans[li];
            plan.material =
                tidy(geom2d::subtract(zp.material(plan.z, plan.zAbove), plan.partZone, clip));
        }
    } else {
        std::vector<Regions> reach(levels.size()); // material the tool can reach at the level
        for (std::size_t li = 0; li < levels.size(); ++li) {
            reach[li] = tidy(geom2d::subtract(stockOutline, plans[li].partZone, clip));
        }
        for (std::size_t ci = 0; ci < coarseIndex.size(); ++ci) {
            const std::size_t li = coarseIndex[ci];
            // The column's top: the top of the shallowest slab within a step down above.
            for (std::size_t above = 0; above < ci; ++above) {
                const double top = above == 0 ? topZ : levels[coarseIndex[above - 1]].z;
                if (top - plans[li].z <= zp.stepDown + 1e-6) {
                    plans[li].columnTop = std::max(plans[li].columnTop, top);
                }
            }
        }
        for (std::size_t ci = 0; ci < coarseIndex.size(); ++ci) {
            const std::size_t li = coarseIndex[ci];
            // What the deepest level within a step down of this column's top can reach is
            // left to that level.
            std::size_t deepest = li;
            for (std::size_t m = ci + 1; m < coarseIndex.size(); ++m) {
                if (plans[li].columnTop - plans[coarseIndex[m]].z <= zp.stepDown + 1e-6) {
                    deepest = coarseIndex[m];
                }
            }
            plans[li].material =
                deepest == li ? reach[li] : tidy(geom2d::subtract(reach[li], reach[deepest], clip));
        }
        std::stable_sort(order.begin(), order.end(), [&](std::size_t x, std::size_t y) {
            if (std::abs(plans[x].columnTop - plans[y].columnTop) > 1e-9) {
                return plans[x].columnTop > plans[y].columnTop;
            }
            return plans[x].z < plans[y].z;
        });
    }
    std::vector<std::size_t> fineOrder;
    for (std::size_t li = 0; li < levels.size(); ++li) {
        if (!levels[li].fine) {
            continue;
        }
        // The band: material the tool reaches at this level (anywhere up the column) that the
        // level under it (the next fine one, or the step-down one) cannot reach, being in its
        // part zone: each point goes to the deepest level that reaches it.
        Plan& plan = plans[li];
        const Regions here =
            zp.material ? geom2d::subtract(zp.material(plan.z, plan.zAbove), plan.partZone, clip)
                        : geom2d::subtract(stockOutline, plan.partZone, clip);
        plan.material = tidy(geom2d::intersect(here, plans[li + 1].partZone, clip));
        fineOrder.push_back(li);
    }
    // Column by column from the top, the deepest first in each.
    std::stable_sort(fineOrder.begin(), fineOrder.end(), [&](std::size_t x, std::size_t y) {
        if (std::abs(plans[x].columnTop - plans[y].columnTop) > 1e-9) {
            return plans[x].columnTop > plans[y].columnTop;
        }
        return plans[x].z < plans[y].z;
    });
    order.insert(order.end(), fineOrder.begin(), fineOrder.end());

    // What is cut: at each level, the pieces of allowed tool centres (which may leave the
    // stock by r + rim at open sides, never enter the forbidden area, and only go where they
    // reach material of the level), in the levels' order.
    struct Item {
        std::size_t plan = 0;
        Region piece;
    };
    std::vector<Item> items;
    std::vector<int> itemsLeft(plans.size(), 0); // per plan, not cut yet
    for (const std::size_t pi : order) {
        const Plan& plan = plans[pi];
        if (plan.material.empty()) {
            continue;
        }
        Regions centres =
            geom2d::subtract(geom2d::offset(plan.material, r + rim, clip), plan.forbidden, clip);
        centres = geom2d::offset(geom2d::offset(centres, -noise, clip), noise, clip);
        std::erase_if(centres, [](const Region& piece) { return piece.area() < 1e-3; });
        for (Region& piece : centres) {
            items.push_back({pi, std::move(piece)});
            ++itemsLeft[pi];
        }
    }
    if (zp.byArea && items.size() > 1) {
        // Area by area: pieces standing over one another (overlapping in XY) across the levels
        // form an area; each area goes through all its levels, in the order above, before the
        // next (the areas in the order their first pieces come).
        std::vector<std::size_t> group(items.size());
        for (std::size_t i = 0; i < items.size(); ++i) {
            group[i] = i;
        }
        const auto find = [&](std::size_t i) {
            while (group[i] != i) {
                group[i] = group[group[i]];
                i = group[i];
            }
            return i;
        };
        for (std::size_t i = 0; i < items.size(); ++i) {
            const geom2d::Aabb2 bi = items[i].piece.bounds();
            for (std::size_t j = i + 1; j < items.size(); ++j) {
                const geom2d::Aabb2 bj = items[j].piece.bounds();
                if (find(i) == find(j) || bi.min.x() > bj.max.x() || bj.min.x() > bi.max.x() ||
                    bi.min.y() > bj.max.y() || bj.min.y() > bi.max.y()) {
                    continue;
                }
                if (!geom2d::intersect({items[i].piece}, {items[j].piece}, clip).empty()) {
                    group[find(j)] = find(i);
                }
            }
        }
        std::vector<std::size_t> rank(items.size(), items.size()); // a group's first item
        for (std::size_t i = 0; i < items.size(); ++i) {
            rank[find(i)] = std::min(rank[find(i)], i);
        }
        std::vector<std::size_t> byGroup(items.size());
        for (std::size_t i = 0; i < items.size(); ++i) {
            byGroup[i] = i;
        }
        std::stable_sort(byGroup.begin(), byGroup.end(), [&](std::size_t left, std::size_t right) {
            return rank[find(left)] < rank[find(right)];
        });
        std::vector<Item> ordered;
        ordered.reserve(items.size());
        for (const std::size_t i : byGroup) {
            ordered.push_back(std::move(items[i]));
        }
        items = std::move(ordered);
    }

    // Columns cut so far (stock space; without in-process material), with the level that cut
    // them: air to a level only when cut down to it or further (an upper level's cut leaves
    // the slab below it standing; counted as air, a closed pocket's lower levels found no
    // entry anywhere and were skipped without a word).
    std::vector<std::pair<double, Regions>> cleared;
    // In rest roughing, the material this operation has cut (the in-process stock is the one
    // before it): each piece's reach, with the slab of its level. Without it a later level (a
    // fine one over a face the step-down level had cleared around) saw no air anywhere and was
    // cut as a closed pocket, entered by a helix.
    struct RestCut {
        double lo = 0.0;
        double hi = 0.0;
        Regions regions;
    };
    std::vector<RestCut> restCuts;
    // In rest roughing, the material at or above z still standing: the in-process stock less
    // what the levels cut so far took, slab by slab (between every slab boundary above z, less
    // the regions of the cuts whose slab spans it).
    const auto restAbove = [&](double z, const Regions& partZone) {
        std::vector<double> slabs{z, std::numeric_limits<double>::infinity()};
        for (const RestCut& c : restCuts) {
            for (const double edge : {c.lo, c.hi}) {
                if (edge > z + 1e-9 && std::isfinite(edge)) {
                    slabs.push_back(edge);
                }
            }
        }
        std::sort(slabs.begin(), slabs.end());
        slabs.erase(
            std::unique(slabs.begin(), slabs.end(),
                        [](double left, double right) { return std::abs(left - right) < 1e-9; }),
            slabs.end());
        Regions standing;
        for (std::size_t k = 0; k + 1 < slabs.size(); ++k) {
            const double lo = slabs[k];
            const double hi = slabs[k + 1];
            Regions slab = geom2d::subtract(zp.material(lo, hi), partZone, clip);
            for (const RestCut& c : restCuts) {
                if (c.lo <= lo + 1e-9 && c.hi >= hi - 1e-9) {
                    slab = geom2d::subtract(slab, c.regions, clip);
                }
            }
            standing = geom2d::unite(standing, slab, clip);
        }
        return tidy(standing);
    };
    const auto clearedDownTo = [&](double z) {
        Regions all;
        for (const auto& [zc, regions] : cleared) {
            if (zc <= z + 1e-9) {
                all = geom2d::unite(all, regions, clip);
            }
        }
        return all;
    };
    for (std::size_t n = 0; n < items.size(); ++n) {
        ctx.cancel.throwIfCancelled();
        const Item& item = items[n];
        const Plan& plan = plans[item.plan];
        const double z = plan.z;
        const double approachZ = plan.columnTop + ctx.heights.feedClearance;
        const Regions& material = plan.material;
        // Where the tool is clear of every material at or above this level it may plunge and
        // start from; in rest roughing hops must also miss material above the level.
        const Regions above = zp.material ? restAbove(z, plan.partZone)
                                          : geom2d::subtract(stockOutline, clearedDownTo(z), clip);
        const Regions aboveReach = geom2d::offset(above, r + 0.5 * rim, clip);
        const Regions air = geom2d::subtract(Regions{item.piece}, aboveReach, clip);
        // Around the piece, where nothing at or above the level and not the part is within the
        // tool's reach: links may leave the piece there (around a part reaching the stock's
        // edge, in the air beside the stock).
        geom2d::Aabb2 around = item.piece.bounds();
        around.min -= Vec2::Constant(3.0 * r + 1.0);
        around.max += Vec2::Constant(3.0 * r + 1.0);
        // Material at or above the level that is not this level's (left to a deeper level that
        // runs later, standing over the slab): not on the clearing's grid, so links keep clear.
        const Regions standing = geom2d::offset(
            tidy(geom2d::subtract(geom2d::subtract(above, material, clip), plan.partZone, clip)),
            r + 0.5 * rim, clip);
        const Regions travel = geom2d::subtract(
            geom2d::subtract(Regions{{Polyline::rectangle(around.min, around.max), {}}},
                             plan.forbidden, clip),
            aboveReach, clip);
        // In rest roughing, the material above the level still standing: the in-process stock
        // less what this operation's levels above have cut. (The in-process stock alone put the
        // levels this operation had just cleared over every lower one, so no link at depth was
        // clear there and the circles went round through the air.)
        Regions overhead;
        if (zp.material && std::isfinite(plan.zAbove)) {
            overhead = geom2d::offset(restAbove(plan.zAbove, plan.partZone), r + 0.5 * rim, clip);
        }
        LevelClearing in;
        in.area = item.piece;
        in.material = material;
        in.forbidden = plan.forbidden;
        in.air = air;
        in.overhead = overhead;
        in.travel = travel;
        in.standing = standing;
        in.toolRadius = r;
        in.stepover = stepover;
        in.z = z;
        in.approachZ = approachZ;
        in.retract = retract;
        in.climb = climb;
        in.entry = zp.entry;
        in.rampAngle = zp.rampAngle;
        in.helixRadius = zp.helixRadius;
        in.medialOpenAreas = zp.medialOpenAreas;
        in.linkLift = zp.linkLift;
        if (std::getenv("OCAM_CLEARING_DEBUG") != nullptr) {
            std::fprintf(
                stderr, "DBG item z %.3f top %.3f material %.1f piece %.1f air %.1f cleared %.1f\n",
                z, plan.columnTop, geom2d::totalArea(material), item.piece.area(),
                geom2d::totalArea(air), geom2d::totalArea(above));
        }
        const ClearingReport report = clearLevelMedial(b, in, ctx);
        cutAnything = cutAnything || report.cutAnything;
        stalled += report.stalled;
        leftoverArea += report.leftoverArea;
        leftover.insert(leftover.end(), report.leftover.begin(), report.leftover.end());
        if (report.noEntry && std::none_of(noEntry.begin(), noEntry.end(), [&](const Vec2& q) {
                return (q - *report.noEntry).norm() < r; // the same area at another level
            })) {
            noEntry.push_back(*report.noEntry);
        }
        // What this piece reached is cleared; the level's whole material once all of its
        // pieces are done.
        const Regions reached =
            geom2d::intersect(material, geom2d::offset({item.piece}, r + rim, clip), clip);
        const bool levelDone = --itemsLeft[item.plan] == 0;
        if (zp.material) {
            restCuts.push_back({z, plan.zAbove, levelDone ? material : reached});
        } else {
            cleared.emplace_back(z, reached);
            if (levelDone) {
                cleared.emplace_back(z, material);
            }
        }
        b.retractTo(retract);
        ctx.progress.report(static_cast<double>(n + 1) / items.size(), "Z-level roughing");
    }
    if (!cutAnything && noEntry.empty()) {
        tp.warnings.push_back("no stock to remove around the part");
    }
    if (stalled > 0) {
        std::string where;
        for (std::size_t k = 0; k < std::min<std::size_t>(leftover.size(), 3); ++k) {
            where += fmt::format("{}({:.1f}, {:.1f})", k ? ", " : " near ", leftover[k].x(),
                                 leftover[k].y());
        }
        tp.warnings.push_back(
            fmt::format("{:.1f} mm² of material the passes could not take was left on {} "
                        "level(s){}",
                        leftoverArea, stalled, where));
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

std::vector<FlatLevel> flatLevels(const geom3d::Mesh& part, double maxSlope, double zTolerance,
                                  double chordTolerance) {
    const double minNz = std::cos(std::clamp(maxSlope, 0.0, kPi / 4.0)) - 1e-12;
    struct Facet {
        double z;
        std::size_t t;
    };
    std::vector<Facet> facets;
    for (std::size_t t = 0; t < part.triangles.size(); ++t) {
        const Vec3 n = part.triangleNormal(t);
        if (n.z() < minNz) {
            continue;
        }
        const auto& tri = part.triangles[t];
        const double z =
            (part.vertices[tri[0]].z() + part.vertices[tri[1]].z() + part.vertices[tri[2]].z()) /
            3.0;
        facets.push_back({z, t});
    }
    std::sort(facets.begin(), facets.end(), [](const Facet& a, const Facet& b) {
        return a.z > b.z || (a.z == b.z && a.t < b.t);
    });
    const geom2d::ClipOptions clip{chordTolerance};
    const double close = std::max(10.0 * chordTolerance, 0.005);
    std::vector<FlatLevel> levels;
    for (std::size_t i = 0; i < facets.size();) {
        // Facets within zTolerance below the group's first (highest) one share a level.
        std::size_t j = i;
        Regions pieces;
        double weighted = 0.0;
        double total = 0.0;
        for (; j < facets.size() && facets[i].z - facets[j].z <= zTolerance; ++j) {
            const auto& tri = part.triangles[facets[j].t];
            std::vector<Vec2> pts{part.vertices[tri[0]].head<2>(), part.vertices[tri[1]].head<2>(),
                                  part.vertices[tri[2]].head<2>()};
            Polyline outline = Polyline::fromPoints(pts, true);
            const double area = outline.signedArea();
            if (std::abs(area) < 1e-12) {
                continue;
            }
            if (area < 0.0) {
                outline.reverse(); // regions are counter-clockwise
            }
            pieces.push_back({std::move(outline), {}});
            weighted += facets[j].z * std::abs(area);
            total += std::abs(area);
        }
        if (total > 0.0) {
            Regions area = geom2d::unite(pieces, {}, clip);
            // Close the hairline gaps between facets that do not quite meet.
            area = geom2d::offset(geom2d::offset(area, close, clip), -close, clip);
            std::erase_if(area, [](const Region& piece) { return piece.area() < 1e-4; });
            if (!area.empty()) {
                levels.push_back({weighted / total, std::move(area)});
            }
        }
        i = j;
    }
    return levels;
}

namespace {

/// The regions with their holes no wider than `maxWidth` filled: a hole is that narrow when no
/// circle of that diameter fits in it (its inward offset by half the width is empty).
Regions fillNarrowHoles(const Regions& regions, double maxWidth, const geom2d::ClipOptions& clip) {
    Regions out;
    bool filled = false;
    for (Region piece : regions) {
        std::vector<Polyline> kept;
        for (const Polyline& hole : piece.holes) {
            Polyline outline = hole;
            if (outline.signedArea() < 0.0) {
                outline.reverse();
            }
            if (geom2d::offset(Regions{{std::move(outline), {}}}, -0.5 * maxWidth, clip).empty()) {
                filled = true;
            } else {
                kept.push_back(hole);
            }
        }
        piece.holes = std::move(kept);
        out.push_back(std::move(piece));
    }
    // A filled hole may hold another piece (a post's top at the same height): one region.
    return filled ? geom2d::unite(out, {}, clip) : out;
}

} // namespace

toolpath::Toolpath flatFinish(const geom3d::Mesh& part, double topZ, double bottomZ,
                              const FlatFinishParams& fp, const Context& ctx) {
    using namespace detail;
    const double r = requireMillingTool(ctx.tool);
    if (part.isEmpty()) {
        throw Error(ErrorCode::InvalidArgument, "flat area finishing needs the part's surface");
    }
    if (fp.stepover < 0.0) {
        throw Error(ErrorCode::InvalidArgument, "invalid stepover");
    }
    const double tol = ctx.chordTolerance;
    const geom2d::ClipOptions clip{tol};
    const geom3d::Aabb3 bounds = part.bounds();
    toolpath::Toolpath tp = beginToolpath(ctx, "flat3d", bounds.center().head<2>());
    const Cutter cutter(ctx.tool);
    // The flat bottom finishes the floor; a bull-nose corner only blends into it.
    const double flatR = cutter.flatRadius();
    if (flatR < 0.05 * r) {
        tp.warnings.push_back(
            "flat areas need a tool with a flat bottom (flat or bull-nose end mill)");
        return tp;
    }
    const double zTolerance = std::max(tol, 0.001);
    std::vector<FlatLevel> levels = flatLevels(part, fp.maxSlope, zTolerance, tol);
    // Every flat's height, those out of range included: the air beside a flat is only sure down
    // to the next one below it (what stands above that was roughed only where it is flat).
    std::vector<double> flatHeights;
    for (const FlatLevel& l : levels) {
        flatHeights.push_back(l.z);
    }
    std::erase_if(levels, [&](const FlatLevel& l) {
        return l.z > topZ + zTolerance || l.z < bottomZ - zTolerance;
    });
    if (!fp.only.empty()) {
        // The selected faces only.
        for (FlatLevel& l : levels) {
            l.area = geom2d::intersect(l.area, fp.only, geom2d::ClipOptions{tol});
            std::erase_if(l.area, [](const Region& piece) { return piece.area() < 1e-4; });
        }
        std::erase_if(levels, [](const FlatLevel& l) { return l.area.empty(); });
    }
    if (levels.empty()) {
        tp.warnings.push_back("no flat areas between the top and bottom heights");
        return tp;
    }
    const double stepover = fp.stepover > 0.0 ? std::min(fp.stepover, 2.0 * flatR) : 1.2 * flatR;
    const bool climb = fp.direction == CutDirection::Climb;
    const double noise = std::max(10.0 * tol, 0.01);
    // Tool centres may pass the edge of a flat by a quarter of the flat radius, so its outer
    // edge is finished cleanly where nothing stands next to it.
    const double rim = std::max(noise, 0.25 * flatR);

    const DropCutter dc(part, cutter);
    const double res = fp.resolution > 0.0 ? fp.resolution : std::clamp(r / 4.0, 0.1, 2.0);
    geom2d::Aabb2 area;
    area.extend(bounds.min.head<2>());
    area.extend(bounds.max.head<2>());
    area.min -= Vec2::Constant(r + rim + 3.0 * res);
    area.max += Vec2::Constant(r + rim + 3.0 * res);
    const double floor = std::min(bounds.min.z(), bottomZ) - 10.0;
    const HeightField field = computeHeightField(dc, area, res, floor, ctx.cancel);

    PocketParams pp;
    pp.stepoverRatio = stepover / (2.0 * r); // the cutter measures stepovers in tool diameters
    pp.entry = fp.entry;
    pp.rampAngle = fp.rampAngle;
    pp.direction = fp.direction;

    // Outside the stock: tool centres at least the radius plus a gap from it.
    const Regions clearOfStock =
        fp.stockOutline.empty()
            ? Regions{}
            : geom2d::offset(fp.stockOutline, r + std::max(noise, 0.1 * r), clip);
    // For an outer loop, the nearest pair (point outside the stock, point of the loop) whose
    // straight feed stays out of `forbidden` (the part at this level), if any.
    // `beside`: tool centres too near the flat (its allowance) or the part at its height; the
    // air around it (where the part drops away: the outside of a top face, a bore) is a way in
    // as well as the outside of the stock.
    const auto sideEntry = [&](const Polyline& loop, const Regions& forbidden,
                               const Regions& beside) -> std::optional<std::pair<Vec2, Vec2>> {
        struct Candidate {
            double distance;
            Vec2 outside;
            Vec2 onLoop;
        };
        std::vector<Candidate> candidates;
        const std::vector<Vec2> pts = loop.discretize(tol);
        for (std::size_t k = 0; k < pts.size(); ++k) {
            const Vec2& a = pts[k];
            const Vec2& c = pts[(k + 1) % pts.size()];
            const int steps = std::max(1, static_cast<int>(std::ceil((c - a).norm() / 1.0)));
            for (int t = 0; t < steps; ++t) {
                const Vec2 q = a + (c - a) * (static_cast<double>(t) / steps);
                for (const Region& region : clearOfStock) {
                    const geom2d::ClosestPoint out = geom2d::closestPoint(region.outer, q);
                    candidates.push_back({out.distance, out.point, q});
                }
                for (const Region& region : beside) {
                    if (!region.contains(q, tol)) {
                        continue;
                    }
                    geom2d::ClosestPoint out = geom2d::closestPoint(region.outer, q);
                    for (const Polyline& hole : region.holes) {
                        const geom2d::ClosestPoint inHole = geom2d::closestPoint(hole, q);
                        if (inHole.distance < out.distance) {
                            out = inHole;
                        }
                    }
                    candidates.push_back({out.distance, out.point, q});
                }
            }
        }
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& x, const Candidate& y) { return x.distance < y.distance; });
        const auto blocked = [&](const Vec2& q) {
            return std::any_of(forbidden.begin(), forbidden.end(),
                               [&](const Region& f) { return f.contains(q, tol); });
        };
        // A longer feed in (an island's top far from the stock's edge) takes longer than an
        // entry from above.
        const double longest = 4.0 * r;
        for (const Candidate& c : candidates) {
            if (c.distance > longest) {
                break;
            }
            const int samples = std::max(2, static_cast<int>(std::ceil(c.distance / 0.25)));
            bool clear = true;
            for (int t = 0; t <= samples && clear; ++t) {
                clear = !blocked(c.outside + (c.onLoop - c.outside) * (double(t) / samples));
            }
            if (clear) {
                return std::pair{c.outside, c.onLoop};
            }
        }
        return std::nullopt;
    };

    toolpath::PathBuilder b(tp);
    const double retract =
        std::max(ctx.heights.retract, std::max(topZ, bounds.max.z()) + ctx.heights.feedClearance);
    int unreachable = 0;
    bool cutAnything = false;
    for (std::size_t li = 0; li < levels.size(); ++li) {
        ctx.cancel.throwIfCancelled();
        const FlatLevel& level = levels[li];
        const double z = level.z + fp.stockToLeaveZ;
        // Positions where the tool at the flat's height would cut into the part (with a little
        // slack for the facets of the flat itself), grown by the wall allowance.
        Regions forbidden = forbiddenRegions(dc, field, level.z + std::max(10.0 * tol, 0.002),
                                             floor, std::max(tol, 1e-3));
        if (fp.stockToLeaveXY > 0.0) {
            forbidden = geom2d::offset(forbidden, fp.stockToLeaveXY, clip);
        }
        // Openings narrow enough are machined over: the passes are planned on the flat with
        // them filled (the tool over a hole stands in air, so nothing there is forbidden); the
        // flat itself (`target`) is what has to be covered.
        const Regions surface = fp.bridgeOpenings > 0.0
                                    ? fillNarrowHoles(level.area, fp.bridgeOpenings, clip)
                                    : level.area;
        Regions centres = geom2d::subtract(geom2d::offset(surface, rim, clip), forbidden, clip);
        centres = geom2d::offset(geom2d::offset(centres, -noise, clip), noise, clip);
        std::erase_if(centres, [](const Region& piece) { return piece.area() < 1e-3; });
        const Regions target =
            centres.empty()
                ? Regions{}
                : geom2d::intersect(level.area, geom2d::offset(centres, flatR, clip), clip);
        for (const Region& piece : level.area) {
            if (piece.area() > 0.01 &&
                geom2d::totalArea(geom2d::intersect({piece}, target, clip)) < 1e-3) {
                ++unreachable;
            }
        }
        if (centres.empty() || geom2d::totalArea(target) < 1e-3) {
            continue;
        }
        // Bands closed: the flat bottom covers the floor between the loops, with no strip left
        // for the clean-up where a band is not a whole number of stepovers wide.
        const LoopTree tree =
            buildLoopTree(centres, stepover, climb, tol, ctx.cancel, std::max(flatR - noise, tol));
        if (tree.order.empty()) {
            continue;
        }
        const double approachZ = z + ctx.heights.feedClearance;
        const std::size_t levelFirstMove = tp.moves.size();
        const Vec3 levelStart = b.position();
        // Long links lift; hops within a flat stay just above it.
        PocketCutter::Options options;
        options.maxLink = 2.0 * stepover + tol;
        options.lowHops = true;
        PocketCutter cleanupCutter(tp, b, pp, ctx, centres, retract);
        options.plungeCleanup = true;
        cleanupCutter.setOptions(options);
        // Open flats start outside the stock: a plunge in air, then a feed in to the outer
        // loop that crosses no part of the part (none of the forbidden area).
        // Flats with air beside them start there and are cut from the outside in, each loop
        // taking at most a stepover (from the inside out the first loop takes the full width):
        // outside the stock, or where the part drops away next to the flat (the material above
        // the lower surfaces is roughed, as the operation expects of every flat).
        // A plunge beside the flat clears all of the part standing above the next flat down
        // (or the part's bottom), not only what stands at the flat's own height: a rounded or
        // chamfered edge is not a flat, roughing leaves stock above it, and a plunge just past
        // the flat's edge landed on that. The gap clears the walls' roughing allowance.
        double below = floor;
        for (const double h : flatHeights) {
            if (h < level.z - zTolerance) {
                below = std::max(below, h);
            }
        }
        const Regions footprint = forbiddenRegions(dc, field, below + std::max(10.0 * tol, 0.002),
                                                   floor, std::max(tol, 1e-3));
        const double besideGap = std::max({noise, 0.25 * r, 1.0});
        const Regions beside = geom2d::unite(geom2d::offset(level.area, r + besideGap, clip),
                                             geom2d::offset(footprint, besideGap, clip), clip);
        options.sideEntry = [&](const Polyline& loop) {
            return sideEntry(loop, forbidden, beside);
        };
        PocketCutter pocketCutter(tp, b, pp, ctx, centres, retract);
        pocketCutter.setOptions(options);
        pocketCutter.cut(tree, z, approachZ, true);
        cutAnything = true;
        // Clean-up of what the loops' flat bottoms missed. A first pass hugs the missed spots
        // (usually small cusps), which the flat bottom covers from close by; whatever is left
        // (next to walls, where the tool centre cannot come close) gets the pass of pocket()
        // that is sure to reach every reachable point, with the flat radius.
        for (int pass = 0; pass < 2; ++pass) {
            const auto paths = pathsAtDepth(tp, levelFirstMove, levelStart, z, tol);
            Regions residual = geom2d::subtract(target, geom2d::sweep(paths, flatR, clip), clip);
            residual = geom2d::offset(geom2d::offset(residual, -noise, clip), noise, clip);
            std::erase_if(residual, [](const Region& piece) { return piece.area() < 1e-3; });
            if (residual.empty()) {
                break;
            }
            const double reach = pass == 0 ? std::max(noise, 0.1 * flatR) : flatR + noise;
            const Regions cleanupArea =
                geom2d::intersect(centres, geom2d::offset(residual, reach, clip), clip);
            if (cleanupArea.empty()) {
                continue;
            }
            const LoopTree cleanup =
                buildLoopTree(cleanupArea, std::min(stepover, 0.9 * flatR), climb, tol, ctx.cancel);
            cleanupCutter.cut(cleanup, z, approachZ, false);
        }
        b.retractTo(retract);
        ctx.progress.report(static_cast<double>(li + 1) / levels.size(), "Flat areas");
    }
    if (unreachable > 0) {
        tp.warnings.push_back(std::to_string(unreachable) +
                              " flat area(s) too small for the tool were left");
    }
    if (!cutAnything) {
        tp.warnings.push_back("no flat area the tool can reach");
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

} // namespace ocam::strategies
