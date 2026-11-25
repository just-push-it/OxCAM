// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// 2D trace: the tool centre along chains of edges, open or closed, on them or offset to a side,
// level by level to a depth below them or below the surface under the tool. Engraving, scribing,
// part marking and slots along a curve. See docs/design-log/0088.

#include "Internal.hpp"
#include "ocam/strategies/Strategies.hpp"

#include <ocam/geom2d/Clip.hpp>
#include <ocam/geom2d/PathOps.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace ocam::strategies {

namespace {

using geom2d::Polyline;
using toolpath::FeedClass;

Vec2 leftOf(const Vec2& t) {
    return {-t.y(), t.x()};
}

double cross(const Vec2& a, const Vec2& b) {
    return a.x() * b.y() - a.y() * b.x();
}

std::vector<Vec2> withoutRepeats(const std::vector<Vec2>& pts) {
    std::vector<Vec2> out;
    for (const Vec2& p : pts) {
        if (out.empty() || (p - out.back()).norm() > 1e-9) {
            out.push_back(p);
        }
    }
    return out;
}

/// Each segment of the open point path `pts` moved by `d` to its left (right when negative),
/// joined by an arc about the vertex where the segments part (the outside of a corner) and at
/// the crossing of the two lines where they overlap. A corner sharper than its segments allow
/// keeps both ends; the loop that leaves is trimmed afterwards.
std::vector<Vec2> rawOffset(const std::vector<Vec2>& pts, double d, double tol) {
    const std::size_t m = pts.size() - 1; // segments
    std::vector<Vec2> t(m);
    std::vector<double> len(m);
    for (std::size_t i = 0; i < m; ++i) {
        len[i] = (pts[i + 1] - pts[i]).norm();
        t[i] = (pts[i + 1] - pts[i]) / len[i];
    }
    std::vector<Vec2> out{pts[0] + d * leftOf(t[0])};
    const double radius = std::abs(d);
    const double step = 2.0 * std::acos(std::clamp(1.0 - tol / radius, -1.0, 1.0));
    for (std::size_t j = 1; j < m; ++j) {
        const Vec2& c = pts[j];
        const Vec2 a = c + d * leftOf(t[j - 1]);
        const Vec2 b = c + d * leftOf(t[j]);
        const double turn = cross(t[j - 1], t[j]); // > 0: the path turns left
        if (d * turn < 0.0) {
            // The offset is on the outside of the corner: round it.
            out.push_back(a);
            const double sweep = std::atan2(turn, t[j - 1].dot(t[j])); // signed turn
            const int pieces =
                step > 1e-12 ? static_cast<int>(std::ceil(std::abs(sweep) / step)) : 1;
            const Vec2 from = a - c;
            for (int k = 1; k < pieces; ++k) {
                out.push_back(c + detail::rotate(from, sweep * k / pieces));
            }
            out.push_back(b);
            continue;
        }
        if (std::abs(turn) < 1e-12) {
            out.push_back(a); // straight on
            continue;
        }
        // Inside of the corner: where the two offset lines cross, when that is on both segments.
        const Vec2 p0 = pts[j - 1] + d * leftOf(t[j - 1]);
        const Vec2 q0 = b;
        const double s = cross(q0 - p0, t[j]) / cross(t[j - 1], t[j]);
        const double u = cross(q0 - p0, t[j - 1]) / cross(t[j - 1], t[j]);
        if (s >= 0.0 && s <= len[j - 1] && u >= 0.0 && u <= len[j]) {
            out.push_back(p0 + s * t[j - 1]);
        } else {
            out.push_back(a);
            out.push_back(b);
        }
    }
    out.push_back(pts[m] + d * leftOf(t[m - 1]));
    return withoutRepeats(out);
}

/// Where `p` lies along the open point path `pts`, as segment index plus fraction.
double paramOn(const std::vector<Vec2>& pts, const Vec2& p) {
    double best = std::numeric_limits<double>::infinity();
    double at = 0.0;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const Vec2 e = pts[i + 1] - pts[i];
        const double l2 = e.squaredNorm();
        const double f = l2 > 0.0 ? std::clamp((p - pts[i]).dot(e) / l2, 0.0, 1.0) : 0.0;
        const double dist = (pts[i] + f * e - p).norm();
        if (dist < best) {
            best = dist;
            at = static_cast<double>(i) + f;
        }
    }
    return at;
}

double lengthOf(const std::vector<Vec2>& pts) {
    double l = 0.0;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        l += (pts[i + 1] - pts[i]).norm();
    }
    return l;
}

/// A path sampled for ramps and for following a surface: points with their distance along the
/// path and the height its depth is measured from. A closed path repeats its first point.
struct Sampled {
    std::vector<Vec2> pts;
    std::vector<double> s;
    std::vector<double> top;
    bool closed = false;

    double length() const { return s.empty() ? 0.0 : s.back(); }
    /// Point and top at distance `at` along the path.
    std::pair<Vec2, double> at(double at) const {
        const auto it = std::upper_bound(s.begin(), s.end(), at);
        if (it == s.begin()) {
            return {pts.front(), top.front()};
        }
        if (it == s.end()) {
            return {pts.back(), top.back()};
        }
        const std::size_t i = static_cast<std::size_t>(it - s.begin());
        const double span = s[i] - s[i - 1];
        const double f = span > 0.0 ? (at - s[i - 1]) / span : 0.0;
        return {pts[i - 1] + f * (pts[i] - pts[i - 1]), top[i - 1] + f * (top[i] - top[i - 1])};
    }
};

/// `path` as points no farther apart than `step` (0: as discretized), each with its top.
Sampled sample(const Polyline& path, double tol, double step,
               const std::function<double(const Vec2&)>& topAt) {
    std::vector<Vec2> raw = path.discretize(tol);
    if (path.isClosed() && !raw.empty()) {
        raw.push_back(raw.front());
    }
    raw = withoutRepeats(raw);
    Sampled out;
    out.closed = path.isClosed();
    double s = 0.0;
    for (std::size_t i = 0; i < raw.size(); ++i) {
        if (i > 0) {
            const Vec2 e = raw[i] - raw[i - 1];
            const double l = e.norm();
            const int pieces = step > 0.0 ? std::max(1, static_cast<int>(std::ceil(l / step))) : 1;
            for (int k = 1; k <= pieces; ++k) {
                out.pts.push_back(raw[i - 1] + e * (static_cast<double>(k) / pieces));
                out.s.push_back(s + l * k / pieces);
            }
            s += l;
        } else {
            out.pts.push_back(raw[0]);
            out.s.push_back(0.0);
        }
    }
    for (const Vec2& p : out.pts) {
        out.top.push_back(topAt(p));
    }
    return out;
}

/// Feed moves along `path` from distance s0 to s1 (either way), the depth below the tops going
/// from d0 to d1 with the distance travelled.
void along(toolpath::PathBuilder& b, const Sampled& path, double s0, double s1, double d0,
           double d1, FeedClass feedClass, double feedRate) {
    const double span = std::abs(s1 - s0);
    if (span <= 1e-12) {
        return;
    }
    const auto emit = [&](const Vec2& p, double top, double s) {
        const double f = std::abs(s - s0) / span;
        b.feed(Vec3(p.x(), p.y(), top - (d0 + (d1 - d0) * f)), feedClass, feedRate);
    };
    const double lo = std::min(s0, s1);
    const double hi = std::max(s0, s1);
    if (s1 > s0) {
        for (std::size_t i = 0; i < path.pts.size(); ++i) {
            if (path.s[i] > lo + 1e-9 && path.s[i] < hi - 1e-9) {
                emit(path.pts[i], path.top[i], path.s[i]);
            }
        }
    } else {
        for (std::size_t i = path.pts.size(); i-- > 0;) {
            if (path.s[i] > lo + 1e-9 && path.s[i] < hi - 1e-9) {
                emit(path.pts[i], path.top[i], path.s[i]);
            }
        }
    }
    const auto [end, top] = path.at(s1);
    emit(end, top, s1);
}

/// Ramps from depth `from` (above the top when negative) to `to` below the tops at `angle`,
/// starting and ending at the path's start: a closed path forward round it, lap after lap,
/// finishing the lap at depth; an open one back and forth along its start, an even number of
/// legs. The tool is at the path's start.
void ramp(toolpath::PathBuilder& b, const Sampled& path, double from, double to, double angle,
          const Feeds& feeds) {
    const double run = (to - from) / std::tan(std::clamp(angle, degToRad(0.1), degToRad(89.0)));
    const double length = path.length();
    if (run <= 1e-9 || length <= 1e-9) {
        const auto [p, top] = path.at(0.0);
        b.feed(Vec3(p.x(), p.y(), top - to), FeedClass::Plunge, feeds.plunge);
        return;
    }
    if (path.closed) {
        double travelled = 0.0;
        double s = 0.0;
        while (travelled < run - 1e-9) {
            const double next = std::min(length, s + (run - travelled));
            const double dFrom = from + (to - from) * travelled / run;
            travelled += next - s;
            const double dTo = from + (to - from) * std::min(1.0, travelled / run);
            along(b, path, s, next, dFrom, dTo, FeedClass::Ramp, feeds.ramp);
            s = next >= length - 1e-9 ? 0.0 : next;
        }
        if (s > 1e-9) {
            along(b, path, s, length, to, to, FeedClass::Cut, feeds.cut); // the lap at depth
        }
        return;
    }
    const int legs = 2 * std::max(1, static_cast<int>(std::ceil(run / (2.0 * length) - 1e-9)));
    const double leg = run / legs;
    for (int i = 0; i < legs; ++i) {
        const double d0 = from + (to - from) * i / legs;
        const double d1 = from + (to - from) * (i + 1) / legs;
        if (i % 2 == 0) {
            along(b, path, 0.0, leg, d0, d1, FeedClass::Ramp, feeds.ramp);
        } else {
            along(b, path, leg, 0.0, d0, d1, FeedClass::Ramp, feeds.ramp);
        }
    }
}

} // namespace

std::vector<Polyline> traceOffset(const Polyline& chain, double distance, double tolerance) {
    const double tol = std::max(tolerance, 1e-6);
    if (chain.segmentCount() == 0) {
        return {};
    }
    if (std::abs(distance) < 1e-12) {
        return {chain};
    }
    const geom2d::ClipOptions clip{tol};
    if (chain.isClosed()) {
        // Counter-clockwise, its left is inside: the region shrinks.
        const bool ccw = chain.isCounterClockwise();
        Polyline loop = chain;
        if (!ccw) {
            loop.reverse();
        }
        std::vector<Polyline> out;
        for (const geom2d::Region& r :
             geom2d::offset({{loop, {}}}, ccw ? -distance : distance, clip)) {
            Polyline path = geom2d::fitArcs(r.outer.discretize(tol), true, 2.0 * tol);
            if (!ccw) {
                path.reverse();
            }
            out.push_back(geom2d::restartAtNearest(path, chain.vertices().front().p));
        }
        return out;
    }
    const std::vector<Vec2> pts = withoutRepeats(chain.discretize(tol));
    if (pts.size() < 2) {
        return {};
    }
    const std::vector<Vec2> raw = rawOffset(pts, distance, tol);
    // What comes nearer the chain than the distance (less a margin for the chords) is a loop of
    // the raw offset at a corner tighter than the tool: trimmed.
    const double reach = std::abs(distance);
    const double margin = 3.0 * tol;
    std::vector<std::vector<Vec2>> pieces{raw};
    if (reach > 2.0 * margin) {
        pieces = geom2d::clipPaths({raw}, geom2d::sweep({pts}, reach - margin, clip), false, clip);
    }
    std::sort(pieces.begin(), pieces.end(), [&](const auto& a, const auto& b) {
        return paramOn(raw, a.front()) < paramOn(raw, b.front());
    });
    // Pieces a trimmed loop parted join again where the line between them keeps its distance.
    const auto joins = [&](const Vec2& a, const Vec2& b) {
        for (int k = 0; k <= 4; ++k) {
            const Vec2 q = a + (b - a) * (k / 4.0);
            if (geom2d::closestPoint(chain, q).distance < reach - 2.0 * margin) {
                return false;
            }
        }
        return true;
    };
    std::vector<std::vector<Vec2>> runs;
    for (auto& piece : pieces) {
        piece = withoutRepeats(piece);
        if (piece.size() < 2) {
            continue;
        }
        if (!runs.empty() && joins(runs.back().back(), piece.front())) {
            runs.back().insert(runs.back().end(), piece.begin(), piece.end());
            runs.back() = withoutRepeats(runs.back());
        } else {
            runs.push_back(std::move(piece));
        }
    }
    std::vector<Polyline> out;
    for (const auto& run : runs) {
        if (lengthOf(run) > std::max(10.0 * tol, 0.01)) {
            out.push_back(geom2d::fitArcs(run, false, 2.0 * tol));
        }
    }
    return out;
}

toolpath::Toolpath trace(const std::vector<TraceChain>& chains, const TraceParams& p,
                         const Context& ctx, const TraceSurface& surface) {
    using namespace detail;
    const double r = requireMillingTool(ctx.tool);
    if (chains.empty()) {
        throw Error(ErrorCode::InvalidArgument, "a trace needs edges to follow");
    }
    if (p.entry == EntryKind::Helix) {
        throw Error(ErrorCode::InvalidArgument, "a trace enters by plunging or ramping");
    }
    if (p.passes < 1) {
        throw Error(ErrorCode::InvalidArgument, "a trace needs at least one pass");
    }
    if (p.depth < 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the trace depth cannot be negative");
    }
    if (surface && p.sampleStep <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "following a surface needs a sample spacing");
    }
    const double tol = ctx.chordTolerance;
    const double fc = ctx.heights.feedClearance;

    // Depths below the top, one per level.
    std::vector<double> depths;
    {
        const int count =
            p.depth > 1e-9 && p.stepDown > 1e-9
                ? std::max(1, static_cast<int>(std::ceil(p.depth / p.stepDown - 1e-9)))
                : 1;
        for (int k = 1; k <= count; ++k) {
            depths.push_back(p.depth * k / count);
        }
    }
    // Tool-centre distances from the line, farthest first.
    std::vector<double> distances;
    if (p.side == TraceSide::On) {
        distances.push_back(0.0);
    } else {
        const double sign = p.side == TraceSide::Left ? 1.0 : -1.0;
        for (int k = p.passes - 1; k >= 0; --k) {
            distances.push_back(sign * (r + p.offset + k * p.passSpacing));
        }
    }

    struct Pass {
        Sampled sampled; // with the tops
        Polyline path;
        double z = 0.0; // the chain's height (flat)
    };
    struct Run {
        std::vector<Pass> passes;
    };
    std::vector<Run> runs;
    bool narrow = false;
    double highest = -std::numeric_limits<double>::infinity();
    for (const TraceChain& chain : chains) {
        ctx.cancel.throwIfCancelled();
        if (chain.path.segmentCount() == 0) {
            continue;
        }
        Polyline c = chain.path;
        Vec2 start;
        if (c.isClosed()) {
            if (!c.isCounterClockwise()) {
                c.reverse();
            }
            c = geom2d::restartAtNearest(c, p.start ? *p.start : preferredStart(c));
            start = c.vertices().front().p;
            if (p.reverse) {
                c.reverse();
                c = geom2d::restartAtNearest(c, start);
            }
        } else {
            const Vec2 a = c.vertices().front().p;
            const Vec2 e = c.vertices().back().p;
            bool flip = false;
            if (p.start) {
                flip = (e - *p.start).norm() < (a - *p.start).norm() - 1e-9;
            } else {
                flip = e.x() < a.x() - 1e-6 || (std::abs(e.x() - a.x()) <= 1e-6 && e.y() < a.y());
            }
            if (flip != p.reverse) {
                c.reverse();
            }
            start = c.vertices().front().p;
        }
        Run run;
        for (const double d : distances) {
            const std::size_t before = run.passes.size();
            for (Polyline& path : traceOffset(c, d, tol)) {
                if (path.isClosed()) {
                    path = geom2d::restartAtNearest(path, start);
                }
                const double z = chain.z;
                Pass pass;
                pass.z = z;
                pass.sampled = surface ? sample(path, tol, p.sampleStep, surface)
                                       : sample(path, tol, 0.0, [z](const Vec2&) { return z; });
                pass.path = std::move(path);
                for (const double top : pass.sampled.top) {
                    highest = std::max(highest, top);
                }
                run.passes.push_back(std::move(pass));
            }
            narrow |= d != 0.0 && run.passes.size() == before;
        }
        if (!run.passes.empty()) {
            runs.push_back(std::move(run));
        }
    }
    const Vec2 startXY = runs.empty() ? chains.front().path.vertices().front().p
                                      : runs.front().passes.front().sampled.pts.front();
    toolpath::Toolpath tp = beginToolpath(ctx, "trace2d", startXY);
    if (narrow) {
        tp.warnings.push_back("the tool does not fit beside some of the edges at that distance");
    }
    if (runs.empty()) {
        tp.warnings.push_back("nothing to trace: the tool does not fit along these edges");
        return tp;
    }
    const double retract = std::max(ctx.heights.retract, highest + fc);
    toolpath::PathBuilder b(tp);
    for (std::size_t ri = 0; ri < runs.size(); ++ri) {
        const Run& run = runs[ri];
        // A single closed path goes on down where it ends, at its start.
        const bool stays = run.passes.size() == 1 && run.passes.front().path.isClosed();
        for (std::size_t li = 0; li < depths.size(); ++li) {
            ctx.cancel.throwIfCancelled();
            const double depth = depths[li];
            const double above = li == 0 ? 0.0 : depths[li - 1]; // the level above
            for (const Pass& pass : run.passes) {
                const Sampled& s = pass.sampled;
                const Vec2 s0 = s.pts.front();
                const bool inPlace = stays && li > 0;
                if (!inPlace) {
                    b.rapidAbove(s0, retract, s.top.front() - above + fc);
                }
                const double from = inPlace ? above : above - fc;
                if (p.entry == EntryKind::Ramp) {
                    ramp(b, s, from, depth, p.rampAngle, ctx.feeds);
                } else {
                    b.feed(Vec3(s0.x(), s0.y(), s.top.front() - depth), FeedClass::Plunge,
                           ctx.feeds.plunge);
                }
                if (surface) {
                    along(b, s, 0.0, s.length(), depth, depth, FeedClass::Cut, ctx.feeds.cut);
                } else {
                    b.follow(pass.path, pass.z - depth, FeedClass::Cut, ctx.feeds.cut);
                }
                if (!stays || li + 1 == depths.size()) {
                    b.retractTo(retract);
                }
            }
        }
        ctx.progress.report(static_cast<double>(ri + 1) / runs.size(), "Trace");
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

} // namespace ocam::strategies
