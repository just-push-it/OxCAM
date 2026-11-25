// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Medial-axis clearing of one Z level (MATHSM): the tool runs on circles centred on the medial
// axis of the area, each the largest that fits there, successive circles a crescent of at most
// the stepover apart and joined by their external bi-tangent, so the path is C¹ and the load
// bounded; a helix down the largest circle starts a closed area. Where the tool would cut only
// air it crosses straight to the material. See Clearing.hpp, DL-0031 and IP-0004.

#include "Clearing.hpp"
#include "Internal.hpp"
#include "MedialAxis.hpp"

#include <ocam/geom2d/Clip.hpp>
#include <ocam/geom2d/PathOps.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>

namespace ocam::strategies::detail {
namespace {

using geom2d::Polyline;
using geom2d::Region;
using geom2d::Regions;
using toolpath::FeedClass;

constexpr double kPi = std::numbers::pi;
constexpr double kTwoPi = 2.0 * std::numbers::pi;

Vec2 leftOf(const Vec2& t) {
    return {-t.y(), t.x()};
}

Vec2 rightOf(const Vec2& t) {
    return {t.y(), -t.x()};
}

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
    double s = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        s += (pts[i] - pts[i - 1]).norm();
    }
    return s;
}

/// The loops of regions as point paths, closed by repeating the first point, in the cutting
/// direction (material on the right when climbing).
std::vector<std::vector<Vec2>> loopsOf(const Regions& regions, bool climb, double tol) {
    std::vector<std::vector<Vec2>> loops;
    for (const Region& region : regions) {
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
            if (climb) {
                std::reverse(pts.begin(), pts.end());
            }
            loops.push_back(std::move(pts));
        }
    }
    return loops;
}

std::vector<Vec2> simplify(const std::vector<Vec2>& pts, double eps) {
    if (pts.size() < 3) {
        return pts;
    }
    std::vector<bool> keep(pts.size(), false);
    keep.front() = keep.back() = true;
    std::vector<std::pair<std::size_t, std::size_t>> stack{{0, pts.size() - 1}};
    while (!stack.empty()) {
        const auto [a, c] = stack.back();
        stack.pop_back();
        if (c <= a + 1) {
            continue;
        }
        const Vec2 d = pts[c] - pts[a];
        const double len = d.norm();
        double best = 0.0;
        std::size_t at = a;
        for (std::size_t i = a + 1; i < c; ++i) {
            const Vec2 v = pts[i] - pts[a];
            const double dist =
                len > 1e-12 ? std::abs(d.x() * v.y() - d.y() * v.x()) / len : v.norm();
            if (dist > best) {
                best = dist;
                at = i;
            }
        }
        if (best > eps) {
            keep[at] = true;
            stack.emplace_back(a, at);
            stack.emplace_back(at, c);
        }
    }
    std::vector<Vec2> out;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        if (keep[i]) {
            out.push_back(pts[i]);
        }
    }
    return out;
}

Vec2 tangentAt(const std::vector<Vec2>& pts, std::size_t i) {
    const Vec2 a = pts[i > 0 ? i - 1 : 0];
    const Vec2 b = pts[std::min(i + 1, pts.size() - 1)];
    const Vec2 t = b - a;
    return t.norm() > 1e-12 ? Vec2(t.normalized()) : Vec2(1.0, 0.0);
}

/// A circle the tool's centre runs on: the maximal inscribed circle of the pocket less the
/// tool radius.
struct Circle {
    Vec2 c = Vec2::Zero();
    double rho = 0.0;
};

Vec2 onCircle(const Circle& k, double a) {
    return k.c + k.rho * Vec2(std::cos(a), std::sin(a));
}

/// Points along a circle from angle a0 over a signed sweep, at most `spacing` apart.
std::vector<Vec2> arcPoints(const Circle& k, double a0, double sweep, double spacing) {
    // A small circle is sampled by angle (a fifth of its radius per step): the direction the
    // engagement is read in comes from neighbouring samples.
    spacing = std::max(std::min(spacing, 0.2 * k.rho), 0.25 * spacing);
    const int n = std::max(2, static_cast<int>(std::ceil(std::abs(sweep) * k.rho / spacing)));
    std::vector<Vec2> pts;
    pts.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
        pts.push_back(onCircle(k, a0 + sweep * i / n));
    }
    return pts;
}

double angleOf(const Vec2& v) {
    return std::atan2(v.y(), v.x());
}

/// The sweep from angle a to angle b going round in the given sense, in [0, 2π).
double sweepTo(double a, double b, bool ccw) {
    double s = std::fmod(ccw ? b - a : a - b, kTwoPi);
    if (s < 0.0) {
        s += kTwoPi;
    }
    return s;
}

/// The external bi-tangent of two circles on the side the tool leaves by going round them in
/// the given sense: the circles' tangent points and the angle of their common normal.
struct Bitangent {
    Vec2 p1 = Vec2::Zero();
    Vec2 p2 = Vec2::Zero();
    double angle = 0.0;
};

std::optional<Bitangent> externalTangent(const Circle& k1, const Circle& k2, bool ccw) {
    const Vec2 d = k2.c - k1.c;
    const double len = d.norm();
    if (len < 1e-9) {
        return std::nullopt;
    }
    const Vec2 dn = d / len;
    const double sigma = (k1.rho - k2.rho) / len;
    if (std::abs(sigma) > 1.0) {
        return std::nullopt; // one circle inside the other
    }
    const Vec2 n = ccw ? rightOf(dn) : leftOf(dn);
    const Vec2 m = sigma * dn + std::sqrt(1.0 - sigma * sigma) * n;
    return Bitangent{k1.c + k1.rho * m, k2.c + k2.rho * m, angleOf(m)};
}

} // namespace

ClearingReport clearLevelMedial(toolpath::PathBuilder& b, const LevelClearing& in,
                                const Context& ctx) {
    const double tol = ctx.chordTolerance;
    const geom2d::ClipOptions clip{std::max(tol, 0.02)};
    const double r = in.toolRadius;
    const double ae = in.stepover;
    const double z = in.z;
    const double noise = std::max(10.0 * tol, 0.01);
    const Regions whole{in.area};
    ClearingReport report;

    // Open areas (air to start from) are traced unless asked for: the circles are at their
    // best in closed, elongated regions and spend half their length in air elsewhere, as the
    // method's authors say.
    Regions airCentres = geom2d::intersect(in.air, whole, clip);
    std::erase_if(airCentres, [](const Region& piece) { return piece.area() < 1e-3; });
    if (!airCentres.empty() && !in.medialOpenAreas) {
        return clearLevel(b, in, ctx);
    }

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
    const bool dbg = std::getenv("OCAM_CLEARING_DEBUG") != nullptr;
    const char* dumpDir = std::getenv("OCAM_CLEARING_DUMP");
    const std::size_t dumpKey = static_cast<std::size_t>(std::llround(box.min.x() * 1000.0));
    const auto dumpRegions = [&](std::ofstream& f, const Regions& regions, const char* tag) {
        for (const Region& region : regions) {
            std::vector<const Polyline*> loops{&region.outer};
            for (const Polyline& hole : region.holes) {
                loops.push_back(&hole);
            }
            for (const Polyline* loop : loops) {
                if (tag) {
                    f << tag;
                }
                for (const Vec2& q : loop->discretize(tol)) {
                    f << (tag ? " " : "") << q.x() << ' ' << q.y() << (tag ? "" : " ");
                }
                f << '\n';
            }
        }
    };
    if (dumpDir) {
        char name[512];
        std::snprintf(name, sizeof name, "%s/material_z%.3f_%zu.txt", dumpDir, z, dumpKey);
        if (std::ofstream f{name}) {
            dumpRegions(f, remaining, nullptr);
        }
    }
    int dumpIndex = 0;
    const auto dumpPath = [&](char tag, const std::vector<Vec2>& pts) {
        if (!dumpDir) {
            return;
        }
        char name[512];
        std::snprintf(name, sizeof name, "%s/pass_z%.3f_%zu_%d.txt", dumpDir, z, dumpKey,
                      dumpIndex);
        if (std::ofstream f{name, std::ios::app}) {
            f << tag;
            for (const Vec2& q : pts) {
                f << ' ' << q.x() << ' ' << q.y();
            }
            f << '\n';
        }
    };

    // The material on a grid, to measure the engagement of a circle before it is cut and to
    // tell cleared ground; the geometry itself is exact.
    CellGrid grid(box, cell);
    grid.fill(remaining, true, tol);
    const double thetaMax = engagementAngle(ae + cell, r);
    const double slack = degToRad(3.0); // the grid's own jitter
    const double ds = std::max(cell, std::min(0.5, 0.1 * r));
    // The engagement is read at every cell along a circle: a spike at a corner is narrower
    // than the clearing step.
    const double dsm = cell;
    const double clearOffset = ae + 2.0 * cell;
    const double maxLink = 4.0 * ae;
    const Regions inside = geom2d::offset(whole, 2.0 * tol, clip);
    CellGrid allowed(box, cell);
    allowed.fill(geom2d::offset(whole, cell, clip), true, tol);
    const auto allowedAt = [&](const Vec2& p) {
        return allowed.at(p);
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
            if (grid.anyWithin(a + (c - a) * (static_cast<double>(k) / n), r - cell)) {
                return true;
            }
        }
        return false;
    };
    // The engagement along a path before it is cut (the half circle facing the motion, on
    // material, the tool's own wake cleared as it goes, the direction from a tangent over two
    // steps each way), taken back after.
    const auto engagementProfile = [&](const std::vector<Vec2>& pts) {
        std::vector<double> profile(pts.size(), 0.0);
        std::vector<std::uint32_t> log;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            if (i > 0) {
                // The direction from two steps each way where the path runs straight (one
                // segment of an offset polygon swings by its facet angle), from one step
                // each way where it turns (a small circle turns through most of a quarter
                // in four steps).
                const Vec2 a = pts[i >= 2 ? i - 2 : 0];
                const Vec2 c = pts[std::min(i + 2, pts.size() - 1)];
                const Vec2 wide = c - a;
                const Vec2 narrow = tangentAt(pts, i);
                Vec2 dir = narrow;
                if (wide.norm() > 1e-9) {
                    const Vec2 w = wide.normalized();
                    dir = w.dot(narrow) > std::cos(degToRad(10.0)) ? w : narrow;
                }
                profile[i] = grid.contactAngle(pts[i], r, dir);
            }
            grid.clearDisk(pts[i], r, &log);
        }
        grid.restore(log);
        return profile;
    };
    const auto engagementAlong = [&](const std::vector<Vec2>& pts) {
        double worst = 0.0;
        for (const double e : engagementProfile(pts)) {
            worst = std::max(worst, e);
        }
        return worst;
    };
    // What a cut removed, on the grid at once and from the exact material when settled (one
    // subtraction of all the cuts at a time: the exact material is for the wall pass and the
    // report, the grid for every decision on the way).
    std::vector<std::vector<Vec2>> cutPaths;
    const auto applyGrid = [&](const std::vector<Vec2>& pts) {
        if (pts.empty()) {
            return;
        }
        for (const Vec2& p : resample(pts, ds)) {
            grid.clearDisk(p, r);
        }
        cutPaths.push_back(pts.size() >= 2 ? pts : std::vector<Vec2>{pts.front(), pts.front()});
        report.cutAnything = true;
        dumpPath('T', pts);
    };
    const auto settle = [&] {
        if (cutPaths.empty()) {
            return;
        }
        remaining = geom2d::subtract(remaining, geom2d::sweep(cutPaths, r, clip), clip);
        cutPaths.clear();
        remaining = geom2d::offset(geom2d::offset(remaining, -noise, clip), noise, clip);
        std::erase_if(remaining, [](const Region& piece) { return piece.area() < 1e-3; });
        const double eps = std::min(0.01, 0.25 * cell);
        for (Region& region : remaining) {
            const auto simplified = [&](const Polyline& loop) {
                std::vector<Vec2> pts = loop.discretize(tol);
                pts.push_back(pts.front());
                pts = simplify(pts, eps);
                pts.pop_back();
                return Polyline::fromPoints(pts, true);
            };
            region.outer = simplified(region.outer);
            for (Polyline& hole : region.holes) {
                hole = simplified(hole);
            }
        }
        std::erase_if(remaining,
                      [](const Region& piece) { return piece.outer.vertices().size() < 3; });
    };
    const auto findEntry = [&](const Vec2& q0, const Vec2& t, const Vec2& n,
                               double alongMax) -> std::optional<Vec2> {
        const Vec2 here = b.position().head<2>();
        std::optional<Vec2> best;
        for (const double along : {alongMax, 0.5 * alongMax, 0.0}) {
            for (const std::initializer_list<double> outs :
                 {std::initializer_list<double>{clearOffset, clearOffset + 0.5 * r},
                  std::initializer_list<double>{0.5 * clearOffset, 0.25 * clearOffset,
                                                2.5 * cell}}) {
                for (const double out : outs) {
                    const Vec2 e = q0 - along * t + out * n;
                    if (allowedAt(e) && !grid.touches(e, r) && lineAllowed(e, q0) &&
                        (!best || (e - here).norm() < (*best - here).norm())) {
                        best = e;
                    }
                }
                if (best) {
                    return best;
                }
            }
        }
        return best;
    };
    const LevelLinks links(in, grid, box, cell, cell, maxLink, ctx);

    const bool ccw = in.climb; // climb: the material on the right, the circles run anticlockwise
    const double sgn = ccw ? 1.0 : -1.0;
    const auto emitArc = [&](const Circle& k, double a0, double sweep, FeedClass fc) {
        const int n = std::max(1, static_cast<int>(std::ceil(std::abs(sweep) / (0.5 * kPi))));
        for (int i = 1; i <= n; ++i) {
            const Vec2 q = onCircle(k, a0 + sweep * i / n);
            b.arc(Vec3(q.x(), q.y(), z), k.c, sweep < 0.0, fc, ctx.feeds.cut);
        }
    };
    // A stretch of the circles' path: an arc on a circle, or a line.
    struct Span {
        std::optional<Circle> k; // the arc's circle; none: a line
        double a0 = 0.0;
        double sweep = 0.0; // signed
        Vec2 p = Vec2::Zero();
        Vec2 q = Vec2::Zero();
        FeedClass fc = FeedClass::Cut;
        Vec2 at(double u) const { return k ? onCircle(*k, a0 + sweep * u) : Vec2(p + (q - p) * u); }
        double length() const { return k ? std::abs(sweep) * k->rho : (q - p).norm(); }
    };
    const auto arcSpan = [](const Circle& k, double a0, double sweep) {
        return Span{k, a0, sweep, Vec2::Zero(), Vec2::Zero(), FeedClass::Cut};
    };
    const auto lineSpan = [](const Vec2& p, const Vec2& q, FeedClass fc) {
        return Span{std::nullopt, 0.0, 0.0, p, q, fc};
    };
    // Cuts a run of spans, a stretch where the tool would cut nothing (no material within its
    // radius, its own wake cleared as it goes) replaced by a line at depth to where it meets
    // material again, at the cutting feed, when that line is clear (LevelLinks) and shorter
    // by a fifth of a stepover or more: the tool does not go round a circle through air. The
    // grid is cleared along the path taken. Held, a stretch in air at the run's end waits for
    // the next run (the tangent off one circle and the start of the next), so that one line
    // can cross both; flushTrack cuts what waits before the tool goes anywhere else.
    std::vector<Span> pending;
    const auto emitTrack = [&](std::vector<Span> spans, bool hold) {
        spans.insert(spans.begin(), pending.begin(), pending.end());
        pending.clear();
        if (spans.empty()) {
            return;
        }
        struct Sample {
            Vec2 q;
            std::size_t span = 0;
            double u = 0.0;
            double s = 0.0; // along the spans
        };
        std::vector<Sample> samples;
        double along = 0.0;
        for (std::size_t k = 0; k < spans.size(); ++k) {
            const double len = spans[k].length();
            const int n = std::max(1, static_cast<int>(std::ceil(len / dsm)));
            for (int i = k == 0 ? 0 : 1; i <= n; ++i) {
                const double u = static_cast<double>(i) / n;
                samples.push_back({spans[k].at(u), k, u, along + len * u});
            }
            along += len;
        }
        struct Chord {
            std::size_t from = 0;
            std::size_t to = 0;
        };
        std::vector<Chord> chords;
        std::vector<std::uint32_t> log;
        std::optional<std::size_t> airFrom;
        const auto closeAir = [&](std::size_t last) {
            if (airFrom && last > *airFrom) {
                // The line across the whole stretch, else the one that saves most of those
                // from and to the joins between spans in it (the path's corners).
                std::vector<std::size_t> starts{*airFrom};
                std::vector<std::size_t> ends{last};
                for (std::size_t i = *airFrom + 1; i < last; ++i) {
                    if (samples[i].u >= 1.0) {
                        starts.push_back(i);
                        ends.push_back(i);
                    }
                }
                std::optional<Chord> best;
                double most = 0.2 * ae;
                for (const std::size_t from : starts) {
                    for (const std::size_t to : ends) {
                        const Sample& a = samples[from];
                        const Sample& c = samples[to];
                        const double saves = c.s - a.s - (c.q - a.q).norm();
                        if (to > from && saves >= most && links.clearLine(a.q, c.q)) {
                            best = Chord{from, to};
                            most = saves;
                        }
                    }
                }
                if (best) {
                    chords.push_back(*best);
                }
            }
            airFrom.reset();
        };
        for (std::size_t i = 0; i < samples.size(); ++i) {
            if (grid.anyWithin(samples[i].q, r)) {
                if (airFrom) {
                    closeAir(i - 1);
                }
            } else if (!airFrom) {
                airFrom = i;
            }
            grid.clearDisk(samples[i].q, r, &log);
        }
        std::optional<std::size_t> tail;
        if (airFrom && hold) {
            tail = airFrom;
        } else if (airFrom) {
            closeAir(samples.size() - 1);
        }
        grid.restore(log);

        std::vector<Vec2> taken{samples.front().q};
        std::size_t span = 0;
        double u = 0.0;
        const auto cutTo = [&](std::size_t toSpan, double toU) {
            for (;;) {
                const Span& sp = spans[span];
                const double u1 = span == toSpan ? toU : 1.0;
                if (u1 - u > 1e-9) {
                    if (sp.k) {
                        const double a = sp.a0 + sp.sweep * u;
                        const double sweep = sp.sweep * (u1 - u);
                        emitArc(*sp.k, a, sweep, sp.fc);
                        const std::vector<Vec2> pts = arcPoints(*sp.k, a, sweep, dsm);
                        taken.insert(taken.end(), pts.begin() + 1, pts.end());
                    } else {
                        const Vec2 e = sp.at(u1);
                        b.feed(Vec3(e.x(), e.y(), z), sp.fc, ctx.feeds.cut);
                        taken.push_back(e);
                    }
                }
                if (span == toSpan) {
                    u = u1;
                    return;
                }
                ++span;
                u = 0.0;
            }
        };
        for (const Chord& c : chords) {
            cutTo(samples[c.from].span, samples[c.from].u);
            const Vec2 e = samples[c.to].q;
            if ((e - samples[c.from].q).norm() > 1e-9) {
                b.feed(Vec3(e.x(), e.y(), z), FeedClass::Link, ctx.feeds.cut);
                taken.push_back(e);
            }
            span = samples[c.to].span;
            u = samples[c.to].u;
        }
        if (tail) {
            const Sample& t = samples[*tail];
            cutTo(t.span, t.u);
            Span rest = spans[t.span];
            if (rest.k) {
                rest.a0 += rest.sweep * t.u;
                rest.sweep *= 1.0 - t.u;
            } else {
                rest.p = rest.at(t.u);
            }
            if (rest.length() > 1e-9) {
                pending.push_back(rest);
            }
            pending.insert(pending.end(), spans.begin() + static_cast<std::ptrdiff_t>(t.span) + 1,
                           spans.end());
        } else {
            cutTo(spans.size() - 1, 1.0);
        }
        if (taken.size() >= 2) {
            applyGrid(taken);
        }
    };
    const auto flushTrack = [&] {
        emitTrack({}, false);
    };
    // Moves the tool to a clear point: at depth through clear ground when close, else by the
    // lowest hop that is safe and a feed down (LevelLinks).
    const auto goTo = [&](const Vec2& e) {
        flushTrack();
        for (const Vec2& p : resample(links.moveTo(b, e), ds)) {
            grid.clearDisk(p, r);
        }
    };

    // The walk's state: the last circle cut, where the tool is on it (none: at its centre,
    // after a link), whether it has been gone round (its front cut), and the circles cut so
    // far, in order, for the links back through them.
    std::optional<Circle> prev;
    std::optional<double> at;
    bool prevComplete = false;
    std::vector<Circle> trail;

    // A step from the last circle to the next: the arc on the last to the tangent point (at
    // least the half turn through its front, unless gone round already; from the centre a
    // link to it), then the bi-tangent line.
    struct Step {
        Bitangent t;
        double sweep = 0.0; // signed, on the last circle
        std::vector<Vec2> pts;
    };
    const auto planStep = [&](const Circle& next) -> std::optional<Step> {
        const auto t = externalTangent(*prev, next, ccw);
        if (!t) {
            return std::nullopt;
        }
        Step s;
        s.t = *t;
        if (at) {
            double sweep = sweepTo(*at, t->angle, ccw);
            if (prevComplete) {
                if (sweep > kTwoPi - 0.5) {
                    sweep -= kTwoPi; // a little back, in air, rather than once more round
                }
            } else if (sweep < kPi) {
                sweep += kTwoPi;
            }
            s.sweep = sgn * sweep;
            s.pts = arcPoints(*prev, *at, s.sweep, dsm);
        } else {
            s.pts = {prev->c, t->p1};
        }
        s.pts.push_back(t->p2);
        return s;
    };
    const auto measureStep = [&](const Step& s, const Circle& next) {
        std::vector<Vec2> pts = s.pts;
        const std::vector<Vec2> round = arcPoints(next, s.t.angle, sgn * kTwoPi, dsm);
        pts.insert(pts.end(), round.begin() + 1, round.end());
        return engagementAlong(pts);
    };
    const auto commitStep = [&](const Step& s, const Circle& next) {
        emitTrack({at ? arcSpan(*prev, *at, s.sweep) : lineSpan(prev->c, s.t.p1, FeedClass::Link),
                   lineSpan(s.t.p1, s.t.p2, FeedClass::Cut)},
                  true);
        prev = next;
        at = s.t.angle;
        prevComplete = false;
        trail.push_back(next);
    };
    // Round the rest of the last circle (at a leaf of the axis, or where the walk stops).
    const auto finishCircle = [&] {
        if (prev && at && !prevComplete) {
            emitTrack({arcSpan(*prev, *at, sgn * kTwoPi)}, true);
            prevComplete = true;
        }
    };
    // Back to a circle cut earlier, at depth through the circles cut since (their centres),
    // else a hop; the tool ends at its centre.
    const auto linkTo = [&](const Circle& k) {
        flushTrack();
        std::vector<Vec2> path{b.position().head<2>()};
        bool found = false;
        for (auto it = trail.rbegin(); it != trail.rend(); ++it) {
            path.push_back(it->c);
            if ((it->c - k.c).norm() < 1e-9) {
                found = true;
                break;
            }
        }
        bool clear = found && std::abs(b.position().z() - z) < 1e-9;
        for (std::size_t i = 1; clear && i < path.size(); ++i) {
            clear = lineAllowed(path[i - 1], path[i]) && !touchesAlong(path[i - 1], path[i]);
        }
        if (clear) {
            b.follow(geom2d::fitArcs(path, false, 2.0 * tol), z, FeedClass::Link, ctx.feeds.cut);
        } else {
            goTo(k.c);
        }
        prev = k;
        at.reset();
        prevComplete = true;
    };
    // Whether a circle's tool, a cell short, meets material anywhere along it.
    const auto removesSomething = [&](const Circle& k) {
        for (const Vec2& q : arcPoints(k, 0.0, kTwoPi, ds)) {
            if (grid.anyWithin(q, r - cell)) {
                return true;
            }
        }
        return false;
    };

    // Circles along a branch of the axis from the last circle: each a crescent of at most the
    // stepover beyond the last candidate (|Δc| + Δρ ≤ ae), nearer where the measured
    // engagement asks; a circle that stands out of the last cut one by less than a fifth of a
    // stepover, or whose tool meets no material, is passed over (MATHSM's purge of circles
    // that remove little: a branch narrowing as fast as it advances would otherwise stand
    // still); the branch's end gets a circle. True when the walk stops short: over the limit
    // at the closest spacing (the branch is thinner than the tool can take within it).
    const auto walk = [&](const std::vector<Vec2>& pts, const std::vector<double>& radii) {
        std::vector<Vec2> P{prev->c};
        P.insert(P.end(), pts.begin(), pts.end());
        std::vector<double> R{prev->rho};
        R.insert(R.end(), radii.begin(), radii.end());
        std::vector<double> S(P.size(), 0.0);
        for (std::size_t i = 1; i < P.size(); ++i) {
            S[i] = S[i - 1] + (P[i] - P[i - 1]).norm();
        }
        const double total = S.back();
        const auto sample = [&](double s) {
            s = std::clamp(s, 0.0, total);
            std::size_t i = 1;
            while (i + 1 < S.size() && S[i] < s) {
                ++i;
            }
            const double seg = S[i] - S[i - 1];
            const double u = seg > 1e-12 ? (s - S[i - 1]) / seg : 0.0;
            return Circle{P[i - 1] + (P[i] - P[i - 1]) * u, R[i - 1] + (R[i] - R[i - 1]) * u};
        };
        const auto standsOut = [&](const Circle& k) {
            return (k.c - prev->c).norm() + k.rho - prev->rho;
        };
        double sFrom = 0.0;
        for (int guard = 0; guard < 100000 && sFrom < total - 1e-9; ++guard) {
            ctx.cancel.throwIfCancelled();
            // The next circle: the first along the branch whose crescent beyond the last cut
            // circle is a stepover wide at its widest (|Δc| + Δρ: nearer where the circles
            // grow, further where they shrink), or the branch's end.
            const double march = std::max(ds, 0.05 * ae);
            double s = sFrom;
            Circle next;
            for (;;) {
                s = std::min(s + march, total);
                next = sample(s);
                if (standsOut(next) >= ae || s >= total - 1e-9) {
                    break;
                }
            }
            bool last = s >= total - 1e-9;
            // A circle under 0.3 r (the helix's own minimum) is passed over too: the tool on
            // one that small in a corner meets both walls at once, a spike the reading along
            // the circle does not catch; the wall pass takes the corner within the limit.
            if (next.rho < 0.3 * r || standsOut(next) < 0.2 * ae || !removesSomething(next)) {
                sFrom = s;
                continue;
            }
            std::optional<Step> st;
            for (int round = 0; round < 8; ++round) {
                st = planStep(next);
                if (!st) {
                    break;
                }
                if (measureStep(*st, next) <= thetaMax + slack) {
                    break;
                }
                st.reset();
                const double shorter = 0.7 * (s - sFrom);
                if (shorter < 0.2 * ae) {
                    break;
                }
                s = sFrom + shorter;
                next = sample(s);
                last = false;
                if (!removesSomething(next)) {
                    break;
                }
            }
            if (!st) {
                if (dbg) {
                    std::fprintf(stderr,
                                 "DBG   branch stops at (%.2f, %.2f) rho %.2f: over the limit\n",
                                 next.c.x(), next.c.y(), next.rho);
                }
                return true;
            }
            commitStep(*st, next);
            sFrom = s;
            if (last) {
                break;
            }
        }
        return false;
    };

    // The entry at the root circle of a piece: a helix (or a ramp on a circle) at its centre,
    // a spiral out to the circle at the stepover per turn or less, ending where the walk will
    // leave, and once round. The tool already clear of material there starts its spiral from
    // the centre. False when nothing fits: the piece is left.
    const auto enter = [&](const Circle& root, const Vec2& firstDir, double firstRho) {
        flushTrack();
        const Vec2 c = root.c;
        const double rampTan = std::tan(std::clamp(in.rampAngle, degToRad(0.1), degToRad(45.0)));
        double rh = 0.0;
        if (grid.touches(c, r) || grid.touches(c + Vec2(root.rho, 0.0), r)) {
            rh = std::min(in.helixRadius.value_or(kHelixRadiusShare * r), root.rho - cell);
            if (in.entry == EntryKind::Plunge) {
                b.rapidAbove(c, in.retract, in.approachZ);
                b.feed(Vec3(c.x(), c.y(), z), FeedClass::Plunge, ctx.feeds.plunge);
                applyGrid({c});
                rh = 0.0;
            } else if (rh >= 0.3 * r) {
                Polyline circle = Polyline::circle(c, rh); // anticlockwise: climb
                if (!ccw) {
                    circle.reverse();
                }
                const Vec2 start = circle.vertices().front().p;
                b.rapidAbove(start, in.retract, in.approachZ);
                if (in.entry == EntryKind::Helix) {
                    b.helix(c, rh, z, kTwoPi * rh * rampTan, !ccw, ctx.feeds.ramp);
                } else {
                    b.rampAlong(circle, z, in.rampAngle, ctx.feeds.ramp);
                }
                std::vector<Vec2> swept = circle.discretize(tol);
                swept.push_back(swept.front());
                applyGrid(swept);
            } else {
                return false;
            }
        } else {
            goTo(c);
        }
        // The spiral: pitch at most the stepover, the turns rounded up so it ends at the
        // departure angle; more turns where the measured engagement asks.
        const Vec2 dn = firstDir.norm() > 1e-9 ? Vec2(firstDir.normalized()) : Vec2(1.0, 0.0);
        const Vec2 n = ccw ? rightOf(dn) : leftOf(dn);
        const double sigma = std::clamp((root.rho - firstRho) / std::max(ae, 1e-9), -1.0, 1.0);
        const double depart = angleOf(sigma * dn + std::sqrt(1.0 - sigma * sigma) * n);
        const double startAngle = angleOf(b.position().head<2>() - c);
        const double partial = sweepTo(startAngle, depart, ccw);
        const double grow = std::max(root.rho - rh, 0.0);
        std::vector<Vec2> spiral;
        const auto spiralAt = [&](int turns) {
            const double theta = kTwoPi * turns + partial;
            const int steps = std::max(8, static_cast<int>(std::ceil(theta * root.rho / dsm)));
            std::vector<Vec2> pts;
            for (int i = 0; i <= steps; ++i) {
                const double th = theta * i / steps;
                const double rr = rh + grow * (theta > 1e-9 ? th / theta : 1.0);
                const double a = startAngle + sgn * th;
                pts.push_back(c + rr * Vec2(std::cos(a), std::sin(a)));
            }
            return pts;
        };
        int turns = static_cast<int>(std::ceil((kTwoPi * grow / ae - partial) / kTwoPi));
        turns = std::max(turns, 0);
        for (int round = 0; round < 8; ++round) {
            spiral = spiralAt(turns);
            if (grow < 1e-9 || engagementAlong(spiral) <= thetaMax + slack) {
                break;
            }
            ++turns;
        }
        if (grow > 1e-9 || partial > 1e-9) {
            b.follow(geom2d::fitArcs(spiral, false, 2.0 * tol), z, FeedClass::Cut, ctx.feeds.cut);
            applyGrid(spiral);
        }
        emitTrack({arcSpan(root, depart, sgn * kTwoPi)}, true);
        prev = root;
        at = depart;
        prevComplete = true;
        trail.assign(1, root);
        return true;
    };

    // The pieces of the area within the tool's reach of material, the nearest first.
    Regions reach = geom2d::intersect(whole, geom2d::offset(remaining, r - 2.0 * cell, clip), clip);
    std::erase_if(reach, [](const Region& piece) { return piece.area() < 1e-3; });
    {
        const Vec2 here = b.position().head<2>();
        std::sort(reach.begin(), reach.end(), [&](const Region& p, const Region& q) {
            const auto centre = [](const Region& region) {
                const geom2d::Aabb2 bb = region.bounds();
                return Vec2(0.5 * (bb.min + bb.max));
            };
            return (centre(p) - here).norm() < (centre(q) - here).norm();
        });
    }
    Regions unentered; // pieces no helix fits in: left, and not reported as leftover
    for (const Region& piece : reach) {
        ctx.cancel.throwIfCancelled();
        flushTrack(); // the tool where its last cut ended, for the choice of the root
        ++dumpIndex;
        if (dumpDir) {
            char name[512];
            std::snprintf(name, sizeof name, "%s/pass_z%.3f_%zu_%d.txt", dumpDir, z, dumpKey,
                          dumpIndex);
            if (std::ofstream f{name}) {
                dumpRegions(f, remaining, "R");
                dumpRegions(f, Regions{piece}, "W");
            }
        }
        const MedialGraph graph = medialAxis({piece}, cell, 3.0 * cell, 1.5 * cell, tol);
        for (const MedialGraph::Edge& edge : graph.edges) {
            dumpPath('P', edge.points);
        }
        // The root: the largest circle, on a branch or at a node; a piece without an axis
        // (a disc, its ridge a point) takes its widest point by inward offsets.
        Circle root;
        int rootEdge = -1;
        std::size_t rootIndex = 0;
        int rootNode = -1;
        for (std::size_t e = 0; e < graph.edges.size(); ++e) {
            const MedialGraph::Edge& edge = graph.edges[e];
            for (std::size_t i = 0; i < edge.points.size(); ++i) {
                if (edge.radii[i] > root.rho) {
                    root = {edge.points[i], edge.radii[i]};
                    rootEdge = static_cast<int>(e);
                    rootIndex = i;
                }
            }
        }
        for (std::size_t n = 0; n < graph.nodes.size(); ++n) {
            if (graph.nodes[n].radius > root.rho + 1e-9) {
                root = {graph.nodes[n].p, graph.nodes[n].radius};
                rootEdge = -1;
                rootNode = static_cast<int>(n);
            }
        }
        // Among circles as large as the largest (to a cell: a slot of even width), one at an
        // end of the axis nearest the tool: the walk then runs one way, with no link back.
        {
            const Vec2 here = b.position().head<2>();
            int bestEnd = -1;
            for (std::size_t n = 0; n < graph.nodes.size(); ++n) {
                const MedialGraph::Node& node = graph.nodes[n];
                if (node.edges.size() == 1 && node.radius >= root.rho - cell &&
                    (bestEnd < 0 ||
                     (node.p - here).norm() <
                         (graph.nodes[static_cast<std::size_t>(bestEnd)].p - here).norm())) {
                    bestEnd = static_cast<int>(n);
                }
            }
            if (bestEnd >= 0) {
                const MedialGraph::Node& node = graph.nodes[static_cast<std::size_t>(bestEnd)];
                root = {node.p, node.radius};
                rootEdge = -1;
                rootNode = bestEnd;
            }
        }
        if (rootEdge >= 0) {
            const MedialGraph::Edge& edge = graph.edges[static_cast<std::size_t>(rootEdge)];
            if (rootIndex == 0) {
                rootNode = edge.a;
                rootEdge = -1;
            } else if (rootIndex + 1 == edge.points.size()) {
                rootNode = edge.b;
                rootEdge = -1;
            }
        }
        if (rootEdge < 0 && rootNode < 0) {
            double lo = 0.0;
            double hi = 2.0 * r + 2.0 * ae;
            Regions core{piece};
            for (int k = 0; k < 16; ++k) {
                const double mid = 0.5 * (lo + hi);
                const Regions shrunk = geom2d::offset(Regions{piece}, -mid, clip);
                if (shrunk.empty()) {
                    hi = mid;
                } else {
                    lo = mid;
                    core = shrunk;
                }
            }
            root = {geom2d::interiorPoint(core.front(), tol), lo};
        }
        if (dbg) {
            std::fprintf(stderr,
                         "DBG medial z %.3f: piece %.1f mm2, %zu branches, %zu nodes, root "
                         "(%.2f, %.2f) rho %.2f\n",
                         z, piece.area(), graph.edges.size(), graph.nodes.size(), root.c.x(),
                         root.c.y(), root.rho);
        }

        // The first direction the walk leaves in (the spiral ends there).
        std::vector<bool> visited(graph.edges.size(), false);
        const auto branchFrom = [&](int e, bool fromA, std::size_t from) {
            const MedialGraph::Edge& edge = graph.edges[static_cast<std::size_t>(e)];
            std::vector<Vec2> pts;
            std::vector<double> radii;
            if (fromA) {
                pts.assign(edge.points.begin() + static_cast<std::ptrdiff_t>(from),
                           edge.points.end());
                radii.assign(edge.radii.begin() + static_cast<std::ptrdiff_t>(from),
                             edge.radii.end());
            } else {
                pts.assign(edge.points.begin(),
                           edge.points.begin() + static_cast<std::ptrdiff_t>(from + 1));
                radii.assign(edge.radii.begin(),
                             edge.radii.begin() + static_cast<std::ptrdiff_t>(from + 1));
                std::reverse(pts.begin(), pts.end());
                std::reverse(radii.begin(), radii.end());
            }
            return std::pair{pts, radii};
        };
        Vec2 firstDir(1.0, 0.0);
        double firstRho = root.rho;
        {
            std::vector<Vec2> pts;
            std::vector<double> radii;
            if (rootEdge >= 0) {
                std::tie(pts, radii) = branchFrom(rootEdge, true, rootIndex);
            } else if (rootNode >= 0 &&
                       !graph.nodes[static_cast<std::size_t>(rootNode)].edges.empty()) {
                const int e = graph.nodes[static_cast<std::size_t>(rootNode)].edges.front();
                const MedialGraph::Edge& edge = graph.edges[static_cast<std::size_t>(e)];
                std::tie(pts, radii) = branchFrom(e, edge.a == rootNode,
                                                  edge.a == rootNode ? 0 : edge.points.size() - 1);
            }
            double s = 0.0;
            for (std::size_t i = 1; i < pts.size(); ++i) {
                s += (pts[i] - pts[i - 1]).norm();
                if (s >= ae || i + 1 == pts.size()) {
                    firstDir = pts[i] - root.c;
                    firstRho = radii[i];
                    break;
                }
            }
        }
        prev.reset();
        at.reset();
        trail.clear();
        if (!enter(root, firstDir, firstRho)) {
            if (!report.noEntry) {
                report.noEntry = root.c;
            }
            unentered.push_back(piece);
            if (dbg) {
                std::fprintf(stderr, "DBG   no helix fits at (%.2f, %.2f) rho %.2f\n", root.c.x(),
                             root.c.y(), root.rho);
            }
            continue;
        }

        // Depth first over the axis: each branch walked from the circle the tool arrived on,
        // the circle finished at a leaf, a link back through the cleared circles to the
        // junction before the next branch.
        std::function<void(int, const Circle&)> visitNode = [&](int node, const Circle& arrival) {
            bool first = true;
            for (const int e : graph.nodes[static_cast<std::size_t>(node)].edges) {
                if (visited[static_cast<std::size_t>(e)]) {
                    continue;
                }
                visited[static_cast<std::size_t>(e)] = true;
                const MedialGraph::Edge& edge = graph.edges[static_cast<std::size_t>(e)];
                const bool fromA = edge.a == node;
                if (!first) {
                    linkTo(arrival);
                }
                first = false;
                const auto [pts, radii] = branchFrom(e, fromA, fromA ? 0 : edge.points.size() - 1);
                if (walk(pts, radii)) {
                    finishCircle();
                    continue;
                }
                visitNode(fromA ? edge.b : edge.a, *prev);
            }
            if (first) {
                finishCircle();
            }
        };
        if (rootEdge >= 0) {
            visited[static_cast<std::size_t>(rootEdge)] = true;
            const MedialGraph::Edge& edge = graph.edges[static_cast<std::size_t>(rootEdge)];
            const Circle rootCircle = root;
            {
                const auto [pts, radii] = branchFrom(rootEdge, true, rootIndex);
                if (walk(pts, radii)) {
                    finishCircle();
                } else {
                    visitNode(edge.b, *prev);
                }
            }
            linkTo(rootCircle);
            {
                const auto [pts, radii] = branchFrom(rootEdge, false, rootIndex);
                if (walk(pts, radii)) {
                    finishCircle();
                } else {
                    visitNode(edge.a, *prev);
                }
            }
        } else if (rootNode >= 0) {
            visitNode(rootNode, root);
        }
        finishCircle();
    }
    flushTrack();
    settle();

    // The walls: the cusps the circles leave between them along the area's edge, taken by one
    // pass round the edge, a cell and a half inside it, away from any body thicker than a
    // stepover within reach (a neck the circles could not take is not slotted along), within
    // the limit (split where over it). A loop starts at its point in air nearest the tool, so
    // no lead-in is needed there.
    {
        const Regions outside =
            geom2d::subtract(Regions{{Polyline::rectangle(box.min, box.max), {}}},
                             geom2d::offset(whole, -2.0 * tol, clip), clip);
        const Regions thick =
            geom2d::offset(geom2d::offset(remaining, -0.5 * ae, clip), 0.5 * ae, clip);
        const Regions wallArea =
            geom2d::subtract(inside, geom2d::offset(thick, r + 2.0 * cell, clip), clip);
        std::vector<std::vector<Vec2>> walls =
            geom2d::clipPaths(loopsOf(outside, in.climb, tol), wallArea, true, clip);
        struct Candidate {
            std::vector<Vec2> pts;
            double distance = 0.0;
        };
        std::vector<Candidate> candidates;
        const Vec2 here = b.position().head<2>();
        for (std::vector<Vec2>& pts : walls) {
            if (pathLength(pts) < 10.0 * tol ||
                std::none_of(pts.begin(), pts.end(),
                             [&](const Vec2& q) { return grid.touches(q, r); })) {
                continue;
            }
            const bool closed = (pts.front() - pts.back()).norm() < tol;
            double distance = (pts.front() - here).norm();
            if (closed) {
                pts.pop_back();
                pts = resample(pts, ds);
                std::size_t where = 0;
                bool air = false;
                distance = std::numeric_limits<double>::infinity();
                for (std::size_t i = 0; i < pts.size(); ++i) {
                    const bool clear = !grid.touches(pts[i], r);
                    const double d = (pts[i] - here).norm();
                    if ((clear && !air) || (clear == air && d < distance)) {
                        distance = d;
                        where = i;
                        air = clear;
                    }
                }
                std::rotate(pts.begin(), pts.begin() + static_cast<std::ptrdiff_t>(where),
                            pts.end());
                pts.push_back(pts.front());
            }
            candidates.push_back({std::move(pts), distance});
        }
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& p, const Candidate& q) { return p.distance < q.distance; });
        for (const Candidate& candidate : candidates) {
            const std::vector<Vec2> pts = resample(candidate.pts, ds);
            const std::vector<double> profile = engagementProfile(pts);
            std::vector<std::vector<Vec2>> runs;
            std::vector<Vec2> run;
            const auto flush = [&] {
                if (pathLength(run) >= std::max(ds, 10.0 * tol)) {
                    runs.push_back(run);
                }
                run.clear();
            };
            for (std::size_t i = 0; i < pts.size(); ++i) {
                if (profile[i] <= thetaMax + slack) {
                    run.push_back(pts[i]);
                } else {
                    flush();
                }
            }
            flush();
            for (std::vector<Vec2>& piece : runs) {
                if (std::none_of(piece.begin(), piece.end(),
                                 [&](const Vec2& q) { return grid.anyWithin(q, r - cell); })) {
                    continue;
                }
                // A run starting in air is simply gone to; one starting on material (after a
                // split) is entered from a short way behind and beside its start.
                std::optional<Vec2> entry;
                std::size_t from = 0;
                if (!grid.touches(piece.front(), r)) {
                    entry = piece.front();
                } else {
                    for (; from + 1 < piece.size(); ++from) {
                        const Vec2 t0 = tangentAt(piece, from);
                        const Vec2 n0 = in.climb ? leftOf(t0) : rightOf(t0);
                        entry = findEntry(piece[from], t0, n0, 0.5 * ae);
                        if (entry) {
                            break;
                        }
                    }
                }
                if (entry) {
                    piece.erase(piece.begin(), piece.begin() + static_cast<std::ptrdiff_t>(from));
                }
                if (!entry || pathLength(piece) < std::max(ds, 10.0 * tol)) {
                    continue;
                }
                goTo(*entry);
                std::vector<Vec2> path{*entry};
                path.insert(path.end(), piece.begin(), piece.end());
                if ((*entry - piece.front()).norm() > 1e-9) {
                    b.feed(Vec3(piece.front().x(), piece.front().y(), z), FeedClass::LeadIn,
                           ctx.feeds.lead);
                }
                b.follow(geom2d::fitArcs(piece, false, 2.0 * tol), z, FeedClass::Cut,
                         ctx.feeds.cut);
                applyGrid(path);
            }
        }
    }
    settle();

    // Material left within reach, thicker than two cells and more than r²/2 in one piece.
    Regions left = geom2d::intersect(remaining, geom2d::offset(whole, r - 2.0 * cell, clip), clip);
    if (!unentered.empty()) {
        left = geom2d::subtract(left, geom2d::offset(unentered, r, clip), clip);
    }
    left = geom2d::offset(geom2d::offset(left, -cell, clip), cell, clip);
    for (const Region& piece : left) {
        if (piece.area() > 0.5 * r * r) {
            report.leftover.push_back(geom2d::interiorPoint(piece, tol));
            report.leftoverArea += piece.area();
        }
    }
    if (!report.leftover.empty()) {
        ++report.stalled;
    }
    return report;
}

} // namespace ocam::strategies::detail
