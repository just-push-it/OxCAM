// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// 2D contour with tangent arc lead-in/out (docs/design-log/0008) and holding tabs
// (docs/design-log/0080).

#include "Internal.hpp"
#include "ocam/strategies/Strategies.hpp"

#include <ocam/geom2d/Clip.hpp>
#include <ocam/geom2d/PathOps.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ocam::strategies {
namespace {

using geom2d::Polyline;

/// Turns sharper than this at a vertex make it a corner for tab placement.
constexpr double kCornerTurn = 10.0 * kPi / 180.0;

/// Arc lengths, segment lengths and corners of a closed loop.
struct LoopMeasure {
    std::vector<double> at;   ///< Arc length at each vertex, and the loop's length last.
    std::vector<bool> corner; ///< At vertex i the path turns by more than kCornerTurn.
    double length = 0.0;
};

/// Unit tangents at the start and end of the segment from a to b.
std::pair<Vec2, Vec2> segmentTangents(const Vec2& a, const Vec2& b, double bulge) {
    if (const auto g = geom2d::arcFromBulge(a, b, bulge)) {
        const auto along = [&](const Vec2& q) {
            const Vec2 radial = q - g->center;
            const Vec2 t(-radial.y(), radial.x());
            return Vec2((g->sweep < 0.0 ? -t : t).normalized());
        };
        return {along(a), along(b)};
    }
    const Vec2 d = (b - a).normalized();
    return {d, d};
}

double segmentLength(const Vec2& a, const Vec2& b, double bulge) {
    if (const auto g = geom2d::arcFromBulge(a, b, bulge)) {
        return g->radius * std::abs(g->sweep);
    }
    return (b - a).norm();
}

LoopMeasure measure(const Polyline& loop) {
    LoopMeasure m;
    const auto& v = loop.vertices();
    const std::size_t n = v.size();
    const std::size_t segments = loop.segmentCount();
    m.at.assign(1, 0.0);
    std::vector<std::pair<Vec2, Vec2>> tangents;
    for (std::size_t i = 0; i < segments; ++i) {
        const Vec2& b = v[(i + 1) % n].p;
        m.at.push_back(m.at.back() + segmentLength(v[i].p, b, v[i].bulge));
        tangents.push_back(segmentTangents(v[i].p, b, v[i].bulge));
    }
    m.length = m.at.back();
    m.corner.assign(n, false);
    for (std::size_t i = 0; i < segments; ++i) {
        const Vec2& in = tangents[(i + segments - 1) % segments].second;
        const Vec2& out = tangents[i].first;
        m.corner[i] = std::acos(std::clamp(in.dot(out), -1.0, 1.0)) > kCornerTurn;
    }
    return m;
}

/// The arc length along `loop` of its point closest to `p`.
double arcLengthAt(const Polyline& loop, const LoopMeasure& m, const Vec2& p) {
    const geom2d::ClosestPoint c = geom2d::closestPoint(loop, p);
    return m.at[c.segment] + c.t * (m.at[c.segment + 1] - m.at[c.segment]);
}

/// The point at arc length s (within [0, length]) along `loop`.
Vec2 pointAt(const Polyline& loop, const LoopMeasure& m, double s) {
    const auto& v = loop.vertices();
    const std::size_t segments = loop.segmentCount();
    std::size_t i =
        static_cast<std::size_t>(std::upper_bound(m.at.begin(), m.at.end(), s) - m.at.begin());
    i = std::min(i == 0 ? 0 : i - 1, segments - 1);
    const double len = m.at[i + 1] - m.at[i];
    const double t = len > 1e-12 ? std::clamp((s - m.at[i]) / len, 0.0, 1.0) : 0.0;
    const Vec2& a = v[i].p;
    const Vec2& b = v[(i + 1) % v.size()].p;
    if (const auto g = geom2d::arcFromBulge(a, b, v[i].bulge)) {
        return g->pointAt(t);
    }
    return a + t * (b - a);
}

/// Follows `loop` from arc length s0 to s1 (s0 <= s1, within [0, length]), z going linearly
/// from z0 to z1; arcs stay arcs (helical where z changes). The tool is at s0 already.
void followPiece(toolpath::PathBuilder& b, const Polyline& loop, const LoopMeasure& m, double s0,
                 double s1, double z0, double z1, toolpath::FeedClass fc, double feedRate) {
    const auto& v = loop.vertices();
    const std::size_t segments = loop.segmentCount();
    const auto zAt = [&](double s) {
        return s1 - s0 > 1e-12 ? z0 + (z1 - z0) * (s - s0) / (s1 - s0) : z1;
    };
    for (std::size_t i = 0; i < segments; ++i) {
        const double a = m.at[i];
        const double e = m.at[i + 1];
        if (e <= s0 + 1e-12 || a >= s1 - 1e-12) {
            continue;
        }
        const double to = std::min(e, s1);
        const Vec2 q = pointAt(loop, m, to);
        const Vec3 target(q.x(), q.y(), zAt(to));
        const Vec2& pa = v[i].p;
        const Vec2& pb = v[(i + 1) % v.size()].p;
        if (const auto g = geom2d::arcFromBulge(pa, pb, v[i].bulge)) {
            b.arc(target, g->center, g->sweep < 0.0, fc, feedRate);
        } else {
            b.feed(target, fc, feedRate);
        }
    }
}

/// Follows a closed loop at height z, going over the tabs in `spans` (sorted, apart, within the
/// loop) at `tabTop`: straight up and down, or on ramps `ramp` long that start on the floor
/// `floorZ` (the tool meets them where they reach z). Every move at the cut feed (ADR-0004).
void followOverTabs(toolpath::PathBuilder& b, const Polyline& loop, const LoopMeasure& m,
                    const std::vector<std::pair<double, double>>& spans, double z, double floorZ,
                    double tabTop, double ramp, double feedRate) {
    using toolpath::FeedClass;
    const Vec2 first = loop.vertices().front().p;
    b.feed(Vec3(first.x(), first.y(), z), FeedClass::Cut, feedRate);
    const double rise = tabTop - floorZ > 1e-12 ? ramp * (z - floorZ) / (tabTop - floorZ) : 0.0;
    double s = 0.0;
    for (const auto& [a, e] : spans) {
        const double up0 = a + rise;
        const double up1 = a + ramp;
        const double down0 = e - ramp;
        const double down1 = e - rise;
        followPiece(b, loop, m, s, up0, z, z, FeedClass::Cut, feedRate);
        if (ramp > 0.0) {
            followPiece(b, loop, m, up0, up1, z, tabTop, FeedClass::Cut, feedRate);
        } else {
            const Vec2 q = pointAt(loop, m, up0);
            b.feed(Vec3(q.x(), q.y(), tabTop), FeedClass::Cut, feedRate);
        }
        followPiece(b, loop, m, up1, down0, tabTop, tabTop, FeedClass::Cut, feedRate);
        if (ramp > 0.0) {
            followPiece(b, loop, m, down0, down1, tabTop, z, FeedClass::Cut, feedRate);
        } else {
            const Vec2 q = pointAt(loop, m, down1);
            b.feed(Vec3(q.x(), q.y(), z), FeedClass::Cut, feedRate);
        }
        s = down1;
    }
    followPiece(b, loop, m, s, m.length, z, z, FeedClass::Cut, feedRate);
}

} // namespace

std::vector<double> tabCentres(const geom2d::Polyline& loop, const ContourTabs& tabs,
                               double halfSpan, double seamMargin) {
    std::vector<double> centres;
    if (!tabs.enabled || !loop.isClosed() || loop.segmentCount() < 1) {
        return centres;
    }
    const LoopMeasure m = measure(loop);
    const double length = m.length;
    const int n = tabs.spacing > 0.0
                      ? std::max(1, static_cast<int>(std::lround(length / tabs.spacing)))
                      : tabs.count;
    if (n < 1 || length <= 1e-9) {
        return centres;
    }
    const std::size_t segments = loop.segmentCount();
    const double pitch = length / n;
    const double window = 0.4 * pitch;
    // What a tab costs where it is: over the loop's start (the leads) most, then over a corner,
    // then over each segment too short to hold it; ties go to the even position.
    const auto cost = [&](double s) {
        const double a = s - halfSpan;
        const double e = s + halfSpan;
        double c = 0.0;
        if (a < seamMargin || e > length - seamMargin) {
            c += 1e6;
        }
        for (std::size_t i = 1; i < segments; ++i) {
            if (m.corner[i] && m.at[i] > a && m.at[i] < e) {
                c += 1e3;
            }
        }
        for (std::size_t i = 0; i < segments; ++i) {
            if (m.at[i + 1] > a && m.at[i] < e && m.at[i + 1] - m.at[i] < 2.0 * halfSpan) {
                c += 10.0;
            }
        }
        return c;
    };
    constexpr int kSteps = 40; // candidates either side of the even position
    for (int k = 0; k < n; ++k) {
        const double even = (k + 0.5) * pitch;
        double best = even;
        double bestCost = cost(even);
        for (int j = 1; j <= kSteps && bestCost > 0.0; ++j) {
            for (const double sign : {-1.0, 1.0}) {
                const double offset = sign * window * j / kSteps;
                const double c = cost(even + offset) + std::abs(offset) / window;
                if (c < bestCost) {
                    best = even + offset;
                    bestCost = c;
                }
            }
        }
        centres.push_back(best);
    }
    return centres;
}

toolpath::Toolpath contour(const geom2d::Regions& shape, double topZ, double bottomZ,
                           const ContourParams& p, const Context& ctx) {
    using namespace detail;
    using geom2d::Polyline;
    using geom2d::Regions;
    using toolpath::FeedClass;
    const double r = requireMillingTool(ctx.tool);
    const double d = 2.0 * r;
    if (shape.empty()) {
        throw Error(ErrorCode::InvalidArgument, "contour needs a boundary");
    }
    if (p.passes < 1) {
        throw Error(ErrorCode::InvalidArgument, "contour needs at least one pass");
    }
    const double tol = ctx.chordTolerance;
    const geom2d::ClipOptions clip{tol};
    const bool outside = p.side == ContourSide::Outside;
    const bool climb = p.direction == CutDirection::Climb;
    const bool inControl = p.compensation == CompensationMode::Control;
    const double sign = outside ? 1.0 : -1.0;

    struct Loop {
        Polyline path;
        Regions centreArea;  ///< Where the tool centre may be (for lead checks), as "inside" test.
        bool allowedInside;  ///< true: centre must be inside centreArea; false: outside it.
        bool spring = false; ///< The spring pass: cut at the final level only.
        int pass = 0;        ///< The radial pass, farthest first.
        double extra = 0.0;  ///< How much farther from the wall than the final pass [mm].
        std::vector<Vec2> tabPoints;                     ///< Tab centres on the path.
        std::vector<std::pair<double, double>> tabSpans; ///< Lifted stretches (arc lengths).
        bool allTab = false; ///< The tabs cover the whole loop: nothing to cut below their top.
    };
    std::vector<Loop> loops;
    std::size_t finalStart = 0; // the final pass's loops start here
    for (int pass = 0; pass < p.passes; ++pass) {
        finalStart = loops.size();
        const double extra = (p.passes - 1 - pass) * p.passSpacing; // farthest pass first
        const double centreDistance = r + p.stockToLeaveXY + extra;
        const double pathDistance = (inControl ? 0.0 : r) + p.stockToLeaveXY + extra;
        const Regions pathArea = geom2d::offset(shape, sign * pathDistance, clip);
        const Regions centreArea = geom2d::offset(shape, sign * centreDistance, clip);
        for (const auto& region : pathArea) {
            std::vector<const Polyline*> selected{&region.outer};
            if (!outside) {
                for (const auto& h : region.holes) {
                    selected.push_back(&h);
                }
            }
            for (const Polyline* raw : selected) {
                Polyline path = geom2d::fitArcs(raw->discretize(tol), true, 2.0 * tol);
                // Region orientation keeps the region's inside on the left. Climb milling (M03)
                // needs the material on the right: outside profiles reverse, inside ones do not.
                if (outside == climb) {
                    path.reverse();
                }
                // With compensation in control the programmed path is the nominal contour, so leads
                // are checked against the nominal area rather than the tool-centre area.
                loops.push_back(
                    {geom2d::restartAtNearest(path, p.start ? *p.start : preferredStart(path)),
                     inControl ? pathArea : centreArea, !outside, false, pass, extra});
            }
        }
    }
    bool warnTabsEverywhere = false;
    if (p.tabs.enabled) {
        // Holding tabs (DL-0080): placed along the final pass's loops, and at the same places on
        // the passes farther out, which cut the same slot. The tool centre stays up over the tab
        // width plus the tool diameter, so the tab is the full width where the tool's edge passes.
        const double halfSpan = 0.5 * p.tabs.width + r;
        const double seamMargin = std::max(0.0, p.leadRadius.value_or(kContourLeadShare * d)) + tol;
        for (std::size_t i = finalStart; i < loops.size(); ++i) {
            const LoopMeasure m = measure(loops[i].path);
            for (const double s : tabCentres(loops[i].path, p.tabs, halfSpan, seamMargin)) {
                loops[i].tabPoints.push_back(
                    pointAt(loops[i].path, m, std::fmod(s + m.length, m.length)));
            }
        }
        const double slack = 0.1 * r + 10.0 * tol;
        for (std::size_t f = finalStart; f < loops.size(); ++f) {
            for (const Vec2& q : loops[f].tabPoints) {
                for (int pass = 0; pass + 1 < p.passes; ++pass) {
                    Loop* nearest = nullptr;
                    double best = std::numeric_limits<double>::infinity();
                    for (std::size_t i = 0; i < finalStart; ++i) {
                        if (loops[i].pass != pass) {
                            continue;
                        }
                        const double dist =
                            (geom2d::closestPoint(loops[i].path, q).point - q).norm();
                        if (dist < best) {
                            best = dist;
                            nearest = &loops[i];
                        }
                    }
                    if (nearest && best <= nearest->extra + slack) {
                        nearest->tabPoints.push_back(geom2d::closestPoint(nearest->path, q).point);
                    }
                }
            }
        }
        for (Loop& loop : loops) {
            if (loop.tabPoints.empty()) {
                continue;
            }
            auto spansOf = [&](const LoopMeasure& m) {
                std::vector<std::pair<double, double>> spans;
                for (const Vec2& q : loop.tabPoints) {
                    const double s = arcLengthAt(loop.path, m, q);
                    spans.emplace_back(s - halfSpan, s + halfSpan);
                }
                std::sort(spans.begin(), spans.end());
                return spans;
            };
            LoopMeasure m = measure(loop.path);
            auto spans = spansOf(m);
            if (spans.front().first < seamMargin || spans.back().second > m.length - seamMargin) {
                // A tab over the start, where the leads are: start in the widest gap instead.
                double gapStart = spans.back().second - m.length;
                double widest = spans.front().first - gapStart;
                double restart = 0.5 * (gapStart + spans.front().first);
                for (std::size_t k = 0; k + 1 < spans.size(); ++k) {
                    const double gap = spans[k + 1].first - spans[k].second;
                    if (gap > widest) {
                        widest = gap;
                        restart = 0.5 * (spans[k].second + spans[k + 1].first);
                    }
                }
                if (widest <= 0.0) {
                    loop.allTab = true;
                    warnTabsEverywhere = true;
                    continue;
                }
                loop.path = geom2d::restartAtNearest(
                    loop.path, pointAt(loop.path, m, std::fmod(restart + m.length, m.length)));
                m = measure(loop.path);
                spans = spansOf(m);
            }
            // Tabs that overlap are one longer tab.
            for (const auto& span : spans) {
                if (!loop.tabSpans.empty() && span.first <= loop.tabSpans.back().second) {
                    loop.tabSpans.back().second =
                        std::max(loop.tabSpans.back().second, span.second);
                } else {
                    loop.tabSpans.push_back(span);
                }
            }
            // A span still over the start (no gap was wide enough) stays within the loop.
            loop.tabSpans.front().first = std::max(loop.tabSpans.front().first, 0.0);
            loop.tabSpans.back().second = std::min(loop.tabSpans.back().second, m.length);
        }
    }
    if (p.springPass) {
        // The final pass once more, at full depth.
        const std::vector<Loop> last(loops.begin() + static_cast<std::ptrdiff_t>(finalStart),
                                     loops.end());
        for (Loop loop : last) {
            loop.spring = true;
            loops.push_back(std::move(loop));
        }
    }

    const auto levels = depthLevels(topZ, bottomZ - p.bottomOffset + p.stockToLeaveZ, p.stepDown);
    const Vec2 startXY =
        loops.empty() ? shape.front().bounds().center() : loops.front().path.vertices().front().p;
    toolpath::Toolpath tp = beginToolpath(ctx, "contour2d", startXY);
    if (inControl) {
        // Material on the right of travel (climb) means the tool is to the left: G41.
        tp.compensation = climb ? toolpath::Compensation::Left : toolpath::Compensation::Right;
    }
    if (loops.empty()) {
        tp.warnings.push_back("the tool does not fit along this contour");
        return tp;
    }
    if (levels.empty()) {
        tp.warnings.push_back("nothing to cut: the contour top is at or below its floor");
        return tp;
    }

    toolpath::PathBuilder b(tp);
    const double retract = std::max(ctx.heights.retract, topZ + ctx.heights.feedClearance);
    bool warnedLead = false;
    // The tabs stand on the contour's lowest level; the levels below their top go over them.
    const double floorZ = levels.back();
    const double tabTop = floorZ + p.tabs.height;
    const double tabRamp = p.tabs.shape == TabShape::Triangle ? 0.5 * p.tabs.width : 0.0;
    if (warnTabsEverywhere) {
        tp.warnings.push_back("the tabs cover a whole outline: it is not cut below their top");
    }
    if (p.tabs.enabled && tabTop >= topZ - 1e-9) {
        tp.warnings.push_back("the tabs reach the contour's top: nothing is cut where they are");
    }

    for (std::size_t li = 0; li < levels.size(); ++li) {
        ctx.cancel.throwIfCancelled();
        const double z = levels[li];
        const double approachZ = (li == 0 ? topZ : levels[li - 1]) + ctx.heights.feedClearance;
        for (const Loop& loop : loops) {
            if (loop.spring && li + 1 != levels.size()) {
                continue; // the spring pass runs at the final depth only
            }
            const bool overTabs = !loop.tabSpans.empty() && z < tabTop - 1e-9;
            if (loop.allTab && z < tabTop - 1e-9) {
                continue; // all of it is tab here
            }
            const Vec2 s = loop.path.vertices().front().p;
            const Vec2 t = startTangent(loop.path);
            // Air side: material is on the right for climb, on the left for conventional.
            const Vec2 left(-t.y(), t.x());
            const Vec2 n = climb ? left : Vec2(-left);
            const bool ccw = climb; // lead arcs turn towards the material side

            // Largest lead radius (halving) whose arcs and lines stay in the allowed area.
            auto allowedAt = [&](const Vec2& q) {
                return anyContains(loop.centreArea, q, tol) == loop.allowedInside;
            };
            auto leadFits = [&](double rl) {
                const Vec2 c = s + rl * n;
                // Sample both arcs, skipping s itself (it lies exactly on the allowed boundary).
                for (int k = 0; k < 12; ++k) {
                    const double a = (kPi / 2.0) * k / 12.0;
                    // Lead-in arc from c - rl t (a = 0) towards s; lead-out from c + rl t back
                    // towards s.
                    const Vec2 in = c + rl * (std::cos(a) * -t + std::sin(a) * -n);
                    const Vec2 out = c + rl * (std::cos(a) * t + std::sin(a) * -n);
                    if (!allowedAt(in) || !allowedAt(out)) {
                        return false;
                    }
                }
                for (int k = 1; k <= 4 && p.leadLength > 0.0; ++k) {
                    const double l = p.leadLength * k / 4.0;
                    if (!allowedAt(c - rl * t + l * n) || !allowedAt(c + rl * t + l * n)) {
                        return false;
                    }
                }
                return true;
            };
            // Compensated arcs need a radius larger than the tool radius on the control.
            const double minLead = inControl ? 1.1 * r : 1e-6;
            double rl =
                std::max(p.leadRadius.value_or(kContourLeadShare * d), inControl ? minLead : 0.0);
            int attempts = 0;
            while (rl > minLead && !leadFits(rl) && attempts++ < 4) {
                rl = std::max(minLead, rl * 0.5);
            }
            if (rl > 1e-6 && !leadFits(rl)) {
                rl = 0.0;
                if (!warnedLead) {
                    tp.warnings.push_back("lead-in/out removed where it would gouge the part");
                    warnedLead = true;
                }
            }

            const Vec2 c = s + rl * n;
            const Vec2 arcStart = rl > 0.0 ? Vec2(c - rl * t) : s;
            const Vec2 entry =
                rl > 0.0 && p.leadLength > 0.0 ? Vec2(arcStart + p.leadLength * n) : arcStart;
            b.rapidAbove(entry, retract, approachZ);
            b.feed(Vec3(entry.x(), entry.y(), z), FeedClass::Plunge, ctx.feeds.plunge);
            if (rl > 0.0) {
                b.feed(Vec3(arcStart.x(), arcStart.y(), z), FeedClass::LeadIn, ctx.feeds.lead);
                b.arc(Vec3(s.x(), s.y(), z), c, !ccw, FeedClass::LeadIn, ctx.feeds.lead);
            }
            if (overTabs) {
                followOverTabs(b, loop.path, measure(loop.path), loop.tabSpans, z, floorZ, tabTop,
                               tabRamp, ctx.feeds.cut);
            } else {
                b.follow(loop.path, z, FeedClass::Cut, ctx.feeds.cut);
            }
            if (rl > 0.0) {
                const Vec2 arcEnd = c + rl * t;
                b.arc(Vec3(arcEnd.x(), arcEnd.y(), z), c, !ccw, FeedClass::LeadOut, ctx.feeds.lead);
                if (p.leadLength > 0.0) {
                    const Vec2 exit = arcEnd + p.leadLength * n;
                    b.feed(Vec3(exit.x(), exit.y(), z), FeedClass::LeadOut, ctx.feeds.lead);
                }
            }
            b.retractTo(retract);
        }
        ctx.progress.report(static_cast<double>(li + 1) / levels.size(), "Contour");
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

} // namespace ocam::strategies
