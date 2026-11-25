// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Wrapped contour: curves and slots on a cylinder about the WCS X axis. Each selection is
// unrolled at the radius its floor is cut at (the arc-length wrap of DL-0024), followed in that
// plane on the line, beside it, or inside a slot's walls, and wrapped back. A curve or slot that
// goes all the way around is periodic in the plane: it is offset over three periods and one is
// kept. See docs/design-log/0082.

#include "Internal.hpp"
#include "ocam/strategies/MultiAxis.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/geom2d/Clip.hpp>
#include <ocam/geom2d/PathOps.hpp>
#include <ocam/toolpath/PathBuilder.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

namespace ocam::strategies {
namespace {

using geom2d::Polyline;
using geom2d::Region;
using geom2d::Regions;
using toolpath::FeedClass;

constexpr double kTwoPi = 2.0 * kPi;
/// A slot drawn the tool's width may come out a hair narrower: its centreline is found within
/// this, and cut (DL-0084).
constexpr double kSlack = 0.01;

Vec2 leftOf(const Vec2& t) {
    return {-t.y(), t.x()};
}

/// Lengths along the points to each of them (closed: and back to the first, last).
std::vector<double> lengthsAlong(const std::vector<Vec2>& pts, bool closed) {
    std::vector<double> cum(pts.size() + (closed ? 1 : 0), 0.0);
    for (std::size_t k = 1; k < cum.size(); ++k) {
        cum[k] = cum[k - 1] + (pts[k % pts.size()] - pts[k - 1]).norm();
    }
    return cum;
}

/// The nearest point of an open run of points to q: how far, how far along, and on which side
/// (positive on the left).
struct Foot {
    double distance = std::numeric_limits<double>::infinity();
    double along = 0.0;
    double side = 0.0;
    Vec2 point = Vec2::Zero();
};
Foot footOn(const std::vector<Vec2>& pts, const std::vector<double>& cum, const Vec2& q) {
    Foot best;
    for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
        const Vec2 a = pts[k], ab = pts[k + 1] - a;
        const double len2 = ab.squaredNorm();
        const double t = len2 > 0.0 ? std::clamp((q - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
        const double d = (q - (a + t * ab)).norm();
        if (d < best.distance) {
            best.distance = d;
            best.along = cum[k] + t * std::sqrt(len2);
            best.side = ab.x() * (q - a).y() - ab.y() * (q - a).x();
            best.point = a + t * ab;
        }
    }
    return best;
}

/// Without points closer together than `eps`.
std::vector<Vec2> deduped(const std::vector<Vec2>& pts, double eps) {
    std::vector<Vec2> out;
    for (const Vec2& p : pts) {
        if (out.empty() || (p - out.back()).norm() > eps) {
            out.push_back(p);
        }
    }
    return out;
}

/// The first and last directions of a run of points.
Vec2 firstDirection(const std::vector<Vec2>& pts) {
    for (std::size_t k = 1; k < pts.size(); ++k) {
        if ((pts[k] - pts[0]).norm() > 1e-9) {
            return (pts[k] - pts[0]).normalized();
        }
    }
    return Vec2::UnitX();
}
Vec2 lastDirection(const std::vector<Vec2>& pts) {
    for (std::size_t k = pts.size() - 1; k-- > 0;) {
        if ((pts.back() - pts[k]).norm() > 1e-9) {
            return (pts.back() - pts[k]).normalized();
        }
    }
    return Vec2::UnitX();
}

/// A curve unrolled at `radius`: (x, arc length). `turns` counts how often a closed curve goes
/// around the axis; then its points do not close in the plane but a period on.
struct Unrolled {
    std::vector<Vec2> pts;
    bool closed = false;
    int turns = 0;
};

Unrolled unroll(const WrappedCurve& c, double radius, double tol) {
    const std::size_t n = c.points.size();
    if (n < 2) {
        throw Error(ErrorCode::InvalidArgument, "a selected curve has no length");
    }
    const auto angleOf = [](const Vec3& p) {
        return std::atan2(p.y(), p.z()); // from +Z towards +Y, as wrapPoint
    };
    for (const Vec3& p : c.points) {
        if (std::hypot(p.y(), p.z()) < 1e-6) {
            throw Error(ErrorCode::InvalidArgument,
                        "a selected curve reaches the rotary axis: it is not around it");
        }
    }
    Unrolled u;
    double prev = angleOf(c.points[0]);
    double t = prev;
    const double t0 = t;
    u.pts.emplace_back(c.points[0].x(), radius * t);
    const std::size_t segments = c.closed ? n : n - 1;
    for (std::size_t k = 0; k < segments; ++k) {
        const Vec3& a = c.points[k];
        const Vec3& b = c.points[(k + 1) % n];
        // Split so the unrolled curve stays within the tolerance of the chords' images.
        const double rho = std::max(std::hypot(a.y(), a.z()), std::hypot(b.y(), b.z()));
        const double step = tol < rho ? 2.0 * std::acos(1.0 - tol / rho) : kPi / 4.0;
        const double sweep = std::abs(std::remainder(angleOf(b) - angleOf(a), kTwoPi));
        const int m = std::max(1, static_cast<int>(std::ceil(sweep / std::max(step, 1e-6))));
        for (int j = 1; j <= m; ++j) {
            const Vec3 q = a + (b - a) * (static_cast<double>(j) / m);
            const double tq = angleOf(q);
            t += std::remainder(tq - prev, kTwoPi);
            prev = tq;
            u.pts.emplace_back(q.x(), radius * t);
        }
    }
    if (c.closed) {
        u.turns = static_cast<int>(std::lround((t - t0) / kTwoPi));
        u.pts.pop_back(); // the first point again, or a period on
        u.closed = u.turns == 0;
    }
    u.pts = deduped(u.pts, 1e-9);
    return u;
}

/// The run of points a distance d to one side of an open run, in its direction, from abreast of
/// its start to abreast of its end: the side of the area a disk sweeps along it. Empty when
/// nothing of that side is left (it curls up within d).
std::vector<Vec2> sideOffset(const std::vector<Vec2>& pts, double d, bool left, double tol) {
    const std::vector<double> cum = lengthsAlong(pts, false);
    const double length = cum.back();
    const double sign = left ? 1.0 : -1.0;
    const Vec2 t0 = firstDirection(pts), t1 = lastDirection(pts);
    const double abreast = std::max(10.0 * tol, 1e-4);
    std::vector<Vec2> best;
    for (const Region& region : geom2d::sweep({pts}, d, {tol})) {
        const std::vector<Vec2> boundary = region.outer.discretize(tol);
        const std::size_t n = boundary.size();
        std::vector<char> mark(n, 0);
        for (std::size_t k = 0; k < n; ++k) {
            const Vec2& q = boundary[k];
            const Foot f = footOn(pts, cum, q);
            // Beside the run, or abreast of an end, where the round end begins.
            const bool beside = (f.along > 1e-7 && f.along < length - 1e-7) ||
                                std::abs((q - pts.front()).dot(t0)) <= abreast ||
                                std::abs((q - pts.back()).dot(t1)) <= abreast;
            mark[k] = beside && sign * f.side > 0.0;
        }
        // The longest run of marked points, around the loop.
        std::size_t start = n;
        for (std::size_t k = 0; k < n; ++k) {
            if (!mark[k]) {
                start = k;
                break;
            }
        }
        if (start == n) {
            continue; // all one side: not an offset of an open run
        }
        std::vector<Vec2> run;
        for (std::size_t j = 1; j <= n; ++j) {
            const std::size_t k = (start + j) % n;
            if (mark[k]) {
                run.push_back(boundary[k]);
            }
            if ((!mark[k] || j == n) && !run.empty()) {
                if (run.size() > best.size()) {
                    best = run;
                }
                run.clear();
            }
        }
    }
    if (best.size() < 2) {
        return {};
    }
    if (footOn(pts, cum, best.front()).along > footOn(pts, cum, best.back()).along) {
        std::reverse(best.begin(), best.end());
    }
    const Vec2 a = pts.front() + sign * d * leftOf(t0);
    const Vec2 b = pts.back() + sign * d * leftOf(t1);
    if ((a - best.front()).norm() > tol) {
        best.insert(best.begin(), a);
    }
    if ((b - best.back()).norm() > tol) {
        best.push_back(b);
    }
    return best;
}

/// The middle line of a sliver: a loop whose two sides lie close together, from one tip to the
/// other, each tip taken at the middle of where the loop turns back there; `shorten` comes off
/// each end. nullopt when it has not two tips.
std::optional<std::vector<Vec2>> middleOf(const std::vector<Vec2>& corners, double tol,
                                          double shorten) {
    const double reach = std::max(10.0 * kSlack, 10.0 * tol);
    // Points no further apart than half the reach: a sliver's long sides may have none.
    std::vector<Vec2> loop;
    for (std::size_t k = 0; k < corners.size(); ++k) {
        const Vec2& a = corners[k];
        const Vec2& b = corners[(k + 1) % corners.size()];
        const int m = std::max(1, static_cast<int>(std::ceil((b - a).norm() / (0.5 * reach))));
        for (int j = 0; j < m; ++j) {
            loop.push_back(a + (b - a) * (static_cast<double>(j) / m));
        }
    }
    const std::size_t n = loop.size();
    if (n < 4) {
        return std::nullopt;
    }
    const std::vector<double> cum = lengthsAlong(loop, true);
    const double length = cum.back();
    if (length < 4.0 * reach) {
        return std::nullopt;
    }
    const auto wrapS = [&](double s) {
        return std::fmod(std::fmod(s, length) + length, length);
    };
    const auto at = [&](double s) {
        s = wrapS(s);
        const auto it = std::upper_bound(cum.begin(), cum.end(), s);
        const std::size_t k = std::min<std::size_t>(
            static_cast<std::size_t>(std::max<std::ptrdiff_t>(it - cum.begin() - 1, 0)), n - 1);
        const double seg = cum[k + 1] - cum[k];
        const double t = seg > 0.0 ? (s - cum[k]) / seg : 0.0;
        return Vec2(loop[k] + t * (loop[(k + 1) % n] - loop[k]));
    };
    // A tip turns the way back within a short way.
    std::vector<char> turns(n, 0);
    for (std::size_t k = 0; k < n; ++k) {
        const Vec2 in = loop[k] - at(cum[k] - reach), out = at(cum[k] + reach) - loop[k];
        turns[k] =
            in.norm() > 1e-12 && out.norm() > 1e-12 && in.normalized().dot(out.normalized()) < -0.5;
    }
    std::size_t first = n;
    for (std::size_t k = 0; k < n; ++k) {
        if (!turns[k]) {
            first = k;
            break;
        }
    }
    if (first == n) {
        return std::nullopt;
    }
    // Each tip: the middle (along the loop) of a run of points that turn back.
    std::vector<double> tips;
    std::optional<std::size_t> runStart;
    std::size_t runEnd = 0;
    for (std::size_t j = 1; j <= n; ++j) {
        const std::size_t k = (first + j) % n;
        if (turns[k]) {
            if (!runStart) {
                runStart = k;
            }
            runEnd = k;
        } else if (runStart) {
            const double span = wrapS(cum[runEnd] - cum[*runStart]);
            tips.push_back(wrapS(cum[*runStart] + 0.5 * span));
            runStart.reset();
        }
    }
    if (tips.size() != 2) {
        return std::nullopt;
    }
    // The two sides, each from one tip to the other.
    const auto side = [&](double from, double to) {
        std::vector<Vec2> pts{at(from)};
        const double span = wrapS(to - from);
        std::vector<std::pair<double, Vec2>> ordered;
        for (std::size_t k = 0; k < n; ++k) {
            const double s = wrapS(cum[k] - from);
            if (s > 1e-12 && s < span - 1e-12) {
                ordered.emplace_back(s, loop[k]);
            }
        }
        std::sort(ordered.begin(), ordered.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        for (const auto& [s, q] : ordered) {
            pts.push_back(q);
        }
        pts.push_back(at(to));
        return pts;
    };
    const std::vector<Vec2> one = side(tips[0], tips[1]);
    const std::vector<Vec2> other = side(tips[1], tips[0]);
    const std::vector<double> otherCum = lengthsAlong(other, false);
    const std::vector<double> oneCum = lengthsAlong(one, false);
    // Between the sides, away from the tips, where the sides are each other's across; then on
    // straight to as far as each tip reaches.
    std::vector<Vec2> middle;
    for (std::size_t k = 0; k < one.size(); ++k) {
        if (oneCum[k] > reach && oneCum[k] < oneCum.back() - reach) {
            middle.push_back(0.5 * (one[k] + footOn(other, otherCum, one[k]).point));
        }
    }
    middle = deduped(middle, 0.1 * tol);
    if (middle.size() < 2) {
        return std::nullopt;
    }
    {
        const Vec2 u0 = firstDirection(middle), u1 = lastDirection(middle);
        const Vec2 start = middle.front() + u0 * (one.front() - middle.front()).dot(u0);
        const Vec2 end = middle.back() + u1 * (one.back() - middle.back()).dot(u1);
        middle.insert(middle.begin(), start);
        middle.push_back(end);
        middle = deduped(middle, 0.1 * tol);
    }
    // Shortened at each end.
    if (shorten > 0.0 && middle.size() >= 2) {
        std::vector<double> c = lengthsAlong(middle, false);
        if (c.back() <= 2.0 * shorten) {
            return std::nullopt;
        }
        const auto point = [&](double s) {
            const auto it = std::upper_bound(c.begin(), c.end(), s);
            const std::size_t k = std::clamp<std::size_t>(static_cast<std::size_t>(it - c.begin()),
                                                          1, middle.size() - 1);
            const double seg = c[k] - c[k - 1];
            const double t = seg > 0.0 ? (s - c[k - 1]) / seg : 0.0;
            return Vec2(middle[k - 1] + t * (middle[k] - middle[k - 1]));
        };
        std::vector<Vec2> cut{point(shorten)};
        for (std::size_t k = 0; k < middle.size(); ++k) {
            if (c[k] > shorten && c[k] < c.back() - shorten) {
                cut.push_back(middle[k]);
            }
        }
        cut.push_back(point(c.back() - shorten));
        middle = deduped(cut, 0.1 * tol);
    }
    return middle;
}

/// Copies of a run of points a period on (`shift` periods along the arc).
std::vector<Vec2> shifted(const std::vector<Vec2>& pts, double by) {
    std::vector<Vec2> out = pts;
    for (Vec2& p : out) {
        p.y() += by;
    }
    return out;
}

/// The run three periods long: a period before, the curve, and a period after.
std::vector<Vec2> tiled(const std::vector<Vec2>& pts, double period) {
    std::vector<Vec2> out;
    for (const double k : {-1.0, 0.0, 1.0}) {
        const std::vector<Vec2> copy = shifted(pts, k * period);
        out.insert(out.end(), copy.begin(), copy.end());
    }
    out.push_back(pts.front() + Vec2(0.0, 2.0 * period));
    return deduped(out, 1e-9);
}

/// The part of a run inside the window of one period, [from, from + period] along the arc; the
/// longest piece, which must cross all of it.
std::vector<Vec2> onePeriod(const std::vector<Vec2>& run, double from, double period, double tol) {
    double x0 = std::numeric_limits<double>::infinity(), x1 = -x0;
    for (const Vec2& p : run) {
        x0 = std::min(x0, p.x());
        x1 = std::max(x1, p.x());
    }
    const Regions window{{Polyline::rectangle({x0 - 1.0, from}, {x1 + 1.0, from + period}), {}}};
    std::vector<Vec2> best;
    for (const std::vector<Vec2>& piece : geom2d::clipPaths({run}, window, true, {tol})) {
        if (piece.size() > best.size()) {
            best = piece;
        }
    }
    if (best.size() < 2 || std::abs(std::abs(best.back().y() - best.front().y()) - period) >
                               std::max(10.0 * tol, 1e-3 * period)) {
        return {};
    }
    return best;
}

/// A closed pass that starts in the middle of its longest segment, where leads go on straight.
std::vector<Vec2> startOnLongest(const std::vector<Vec2>& loop) {
    const std::size_t n = loop.size();
    std::size_t longest = 0;
    for (std::size_t k = 0; k < n; ++k) {
        if ((loop[(k + 1) % n] - loop[k]).norm() >
            (loop[(longest + 1) % n] - loop[longest]).norm()) {
            longest = k;
        }
    }
    std::vector<Vec2> out{0.5 * (loop[longest] + loop[(longest + 1) % n])};
    for (std::size_t j = 1; j <= n; ++j) {
        out.push_back(loop[(longest + j) % n]);
    }
    return deduped(out, 1e-9);
}

struct Pass {
    std::vector<Vec2> pts; ///< The tool centre, in its own way.
    bool closed = false;   ///< Back to its start in the plane.
    /// Beside a wall: whether its own way is climb milling (the wall on the right). Without a
    /// wall beside it (on the line, a slot's middle) it goes its own way and has no leads.
    std::optional<bool> climbs;
    std::function<bool(const Vec2&)> allowed; ///< Where the tool centre may go (for leads).
};

struct Group {
    std::string name;
    double radius = 0.0; ///< Unrolled and cut at.
    double xMin = std::numeric_limits<double>::infinity();
    double xMax = -std::numeric_limits<double>::infinity();
    double offset = 0.0; ///< Of passes beside walls, 0 = none.
    std::vector<Pass> passes;
};

void extendX(Group& g, const std::vector<Vec2>& pts) {
    for (const Vec2& p : pts) {
        g.xMin = std::min(g.xMin, p.x());
        g.xMax = std::max(g.xMax, p.x());
    }
}

double leastRadius(const std::vector<Vec3>& pts, double* most = nullptr) {
    double least = std::numeric_limits<double>::infinity(), largest = 0.0;
    for (const Vec3& p : pts) {
        least = std::min(least, std::hypot(p.y(), p.z()));
        largest = std::max(largest, std::hypot(p.y(), p.z()));
    }
    if (most) {
        *most = largest;
    }
    return least;
}

/// The cutting radius of a floor at `floor`: the depth below it, the stock to leave above.
double cutRadius(const std::string& name, double floor, const RotaryContourParams& p, double tol) {
    const double radius = floor - p.depth + p.stockToLeaveZ;
    if (radius <= 10.0 * tol) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("{}: {:.3f} mm below its floor, {:.3f} mm from the axis, is the "
                                "axis or past it",
                                name, p.depth - p.stockToLeaveZ, floor));
    }
    return radius;
}

Group curveGroup(const WrappedCurve& c, const std::string& name, const RotaryContourParams& p,
                 double d, double tol, std::vector<std::string>& warnings) {
    Group g;
    g.name = name;
    double most = 0.0;
    const double floor = leastRadius(c.points, &most);
    if (floor < 1e-6) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("{} reaches the rotary axis: it is not around it", name));
    }
    if (most - floor > 0.05) {
        warnings.push_back(fmt::format("{} lies from {:.3f} to {:.3f} mm from the axis: it is cut "
                                       "at its deepest",
                                       name, floor, most));
    }
    g.radius = cutRadius(name, floor, p, tol);
    Unrolled u = unroll(c, g.radius, tol);
    if (u.pts.size() < 2) {
        throw Error(ErrorCode::InvalidArgument, fmt::format("{} has no length", name));
    }
    const bool beside = p.side != WrapSide::On;
    const bool left = p.side == WrapSide::Left;
    g.offset = beside ? d : 0.0;
    const double base = kTwoPi * g.radius;
    if (u.turns != 0) {
        // Around the part: with the angle, and periodic.
        if (u.turns < 0) {
            std::reverse(u.pts.begin(), u.pts.end());
        }
        const double period = std::abs(u.turns) * base;
        Pass pass;
        if (!beside) {
            pass.pts = u.pts;
            pass.pts.push_back(u.pts.front() + Vec2(0.0, period));
        } else {
            if (period < 2.0 * d + 10.0 * tol) {
                throw Error(ErrorCode::InvalidArgument,
                            fmt::format("{}: the way around, {:.3f} mm, is too short for the tool "
                                        "beside it",
                                        name, period));
            }
            const std::vector<Vec2> three = tiled(u.pts, period);
            pass.pts = onePeriod(sideOffset(three, d, left, tol), u.pts.front().y(), period, tol);
            if (pass.pts.empty()) {
                warnings.push_back(
                    fmt::format("{}: there is no room beside it for the tool", name));
                return g;
            }
            const auto cum = std::make_shared<std::vector<double>>(lengthsAlong(three, false));
            const auto line = std::make_shared<std::vector<Vec2>>(three);
            pass.allowed = [line, cum, d, left, tol](const Vec2& q) {
                const Foot f = footOn(*line, *cum, q);
                return f.distance >= d - 2.0 * tol && (left ? f.side > 0.0 : f.side < 0.0);
            };
            pass.climbs = left; // left of it, the wall is on the right
        }
        extendX(g, pass.pts);
        g.passes.push_back(std::move(pass));
        return g;
    }
    if (u.closed) {
        Polyline loop = Polyline::fromPoints(u.pts, true);
        if (loop.signedArea() < 0.0) {
            loop.reverse();
        }
        if (!beside) {
            Pass pass;
            pass.pts = startOnLongest(loop.discretize(tol));
            pass.closed = true;
            extendX(g, pass.pts);
            g.passes.push_back(std::move(pass));
            return g;
        }
        const Regions shape = geom2d::buildRegions({loop}, tol);
        const auto area = std::make_shared<Regions>(geom2d::offset(shape, left ? -d : d, {tol}));
        for (const Region& r : *area) {
            std::vector<const Polyline*> loops{&r.outer};
            if (left) {
                for (const Polyline& h : r.holes) {
                    loops.push_back(&h);
                }
            }
            for (const Polyline* l : loops) {
                Pass pass;
                pass.pts = startOnLongest(l->discretize(tol));
                pass.closed = true;
                pass.allowed = [area, left, tol](const Vec2& q) {
                    return detail::anyContains(*area, q, tol) == left;
                };
                // Counter-clockwise: inside, the wall is on the right; outside, on the left.
                pass.climbs = left;
                extendX(g, pass.pts);
                g.passes.push_back(std::move(pass));
            }
        }
        if (g.passes.empty()) {
            warnings.push_back(fmt::format("{}: the tool does not fit inside it", name));
        }
        return g;
    }
    // An open curve: from its end at the lower X (the same X: the lower angle).
    const Vec2 span = u.pts.back() - u.pts.front();
    if (std::abs(span.x()) > tol ? span.x() < 0.0 : span.y() < 0.0) {
        std::reverse(u.pts.begin(), u.pts.end());
    }
    Pass pass;
    if (!beside) {
        pass.pts = u.pts;
    } else {
        pass.pts = sideOffset(u.pts, d, left, tol);
        if (pass.pts.empty()) {
            warnings.push_back(fmt::format("{}: there is no room beside it for the tool", name));
            return g;
        }
        const auto line = std::make_shared<std::vector<Vec2>>(u.pts);
        const auto cum = std::make_shared<std::vector<double>>(lengthsAlong(u.pts, false));
        pass.allowed = [line, cum, d, left, tol](const Vec2& q) {
            const Foot f = footOn(*line, *cum, q);
            const bool beyond = f.along <= 1e-9 || f.along >= cum->back() - 1e-9;
            return f.distance >= d - 2.0 * tol && (beyond || (left ? f.side > 0.0 : f.side < 0.0));
        };
        pass.climbs = left;
    }
    extendX(g, pass.pts);
    g.passes.push_back(std::move(pass));
    return g;
}

/// A slot's floor unrolled: each part of it either goes all the way around (periodic: three
/// periods of it, of which one is cut) or does not (one copy of it).
struct Floor {
    Regions area;
    bool periodic = false;
};

/// The floor's faces unrolled at `radius` and joined, also where they meet across the angle
/// where the unrolling starts: each face three times, a period apart, and of what does not go
/// around one copy, the one in the period from 0. Ends at the part's ends are open: the slot
/// runs out past them by `runOut`.
std::vector<Floor> unrolledFloor(const WrappedSlot& s, double radius, double partXMin,
                                 double partXMax, double runOut, double tol,
                                 const std::string& name) {
    const double period = kTwoPi * radius;
    const geom2d::ClipOptions clip{tol};
    const auto open = [&](std::vector<Vec2> pts) {
        for (Vec2& q : pts) {
            if (q.x() >= partXMax - 1e-3) {
                q.x() = partXMax + runOut;
            } else if (q.x() <= partXMin + 1e-3) {
                q.x() = partXMin - runOut;
            }
        }
        return pts;
    };
    Regions all;
    for (const std::vector<WrappedCurve>& face : s.faces) {
        std::vector<Unrolled> around, plain;
        for (const WrappedCurve& c : face) {
            Unrolled u = unroll(c, radius, tol);
            if (u.pts.size() < 3) {
                continue;
            }
            if (u.turns < 0) {
                std::reverse(u.pts.begin(), u.pts.end());
                u.turns = -u.turns;
            }
            (u.turns != 0 ? around : plain).push_back(std::move(u));
        }
        std::vector<Polyline> loops;
        if (around.size() == 2 && around[0].turns == 1 && around[1].turns == 1) {
            // A band around the part: its two rims, three periods long, make a strip.
            std::vector<Vec2> strip = open(tiled(around[0].pts, period));
            const std::vector<Vec2> other = open(tiled(around[1].pts, period));
            strip.insert(strip.end(), other.rbegin(), other.rend());
            loops.push_back(Polyline::fromPoints(strip, true));
        } else if (!around.empty()) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("{}: its floor is not a slot's: select a slot's floor, or "
                                    "the edges to follow",
                                    name));
        }
        for (const Unrolled& u : plain) {
            for (const double k : {-1.0, 0.0, 1.0}) {
                loops.push_back(Polyline::fromPoints(open(shifted(u.pts, k * period)), true));
            }
        }
        const Regions area = geom2d::buildRegions(std::move(loops), tol);
        all = all.empty() ? area : geom2d::unite(all, area, clip);
    }
    // Hairline gaps where faces meet are closed.
    constexpr double kGap = 1e-4;
    all = geom2d::offset(geom2d::offset(all, kGap, clip), -kGap, clip);
    std::vector<Floor> out;
    for (Region& region : all) {
        const Vec2 q = geom2d::interiorPoint(region, tol);
        const bool periodic = region.contains(q + Vec2(0.0, period), tol) ||
                              region.contains(q - Vec2(0.0, period), tol);
        const double middle = 0.5 * (region.bounds().min.y() + region.bounds().max.y());
        if (periodic || (middle >= 0.0 && middle < period)) {
            out.push_back({{std::move(region)}, periodic});
        }
    }
    if (out.empty()) {
        throw Error(ErrorCode::InvalidArgument, fmt::format("{}: its floor has no area", name));
    }
    return out;
}

Group slotGroup(const WrappedSlot& s, const std::string& name, const RotaryContourParams& p,
                double d, double r, double partXMin, double partXMax, double tol,
                std::vector<std::string>& warnings) {
    Group g;
    g.name = name;
    if (s.faces.empty()) {
        throw Error(ErrorCode::InvalidArgument, fmt::format("{} has no face", name));
    }
    g.radius = cutRadius(name, s.floorRadius, p, tol);
    const double period = kTwoPi * g.radius;
    const geom2d::ClipOptions clip{tol};
    for (const Floor& floor : unrolledFloor(s, g.radius, partXMin, partXMax, d + r, tol, name)) {
        const Regions& slot = floor.area;
        if (floor.periodic && period < 2.0 * (d + r) + 10.0 * tol) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("{}: the way around, {:.3f} mm, is too short for the tool",
                                    name, period));
        }
        // The passes: inside the walls, stepping in until nothing is left; a slot the tool's
        // width has only its middle line.
        auto centre = std::make_shared<Regions>(geom2d::offset(slot, -d, clip));
        double last = d;
        bool narrow = false; // narrower than the tool by up to twice the slack
        if (centre->empty()) {
            *centre = geom2d::offset(slot, -(d - kSlack), clip);
            last = d - kSlack;
            narrow = true;
            if (centre->empty()) {
                throw Error(ErrorCode::InvalidArgument,
                            fmt::format("{} is narrower than the tool{}", name,
                                        p.stockToLeaveXY > 0.0 ? " and the stock to leave" : ""));
            }
        }
        std::vector<Regions> rings{*centre};
        for (int k = 1; k < 1000 && !narrow; ++k) {
            if (p.stepover <= 0.0) {
                if (geom2d::totalArea(geom2d::offset(slot, -(d + r), clip)) > tol * tol) {
                    throw Error(ErrorCode::InvalidArgument,
                                fmt::format("{} is wider than the tool cuts in one pass: the "
                                            "passes across it step by the cutting preset's "
                                            "radial depth (ae), which is not set",
                                            name));
                }
                break;
            }
            Regions next = geom2d::offset(slot, -(d + k * p.stepover), clip);
            if (next.empty()) {
                break;
            }
            last = d + k * p.stepover;
            rings.push_back(std::move(next));
        }
        if (geom2d::totalArea(geom2d::offset(slot, -(last + r), clip)) > tol * tol) {
            warnings.push_back(fmt::format("{}: the passes do not reach its middle: the stepover "
                                           "(the preset's ae) is more than the tool's radius",
                                           name));
        }
        // Innermost first: the walls last, each pass taking a share of the width.
        std::reverse(rings.begin(), rings.end());
        for (const Regions& ring : rings) {
            for (const Region& region : ring) {
                std::vector<const Polyline*> ls{&region.outer};
                for (const Polyline& h : region.holes) {
                    ls.push_back(&h);
                }
                for (const Polyline* l : ls) {
                    std::vector<Vec2> pts = l->discretize(tol);
                    bool closed = true;
                    bool middle = false;
                    // A sliver: the slot is the tool's width there. Its middle line is cut once.
                    const double width =
                        2.0 * std::abs(l->signedArea()) / std::max(l->length(), 1e-9);
                    if (width < 3.0 * kSlack) {
                        // Found a slack in from the walls: its ends come back by as much.
                        if (const auto line = middleOf(pts, tol, narrow ? kSlack : 0.0)) {
                            pts = *line;
                            closed = false;
                            middle = true;
                            // Into each wall by as much as the slot is narrower than the tool.
                            const double into = narrow ? kSlack - 0.5 * width : 0.0;
                            if (into > 1e-3) {
                                warnings.push_back(fmt::format(
                                    "{} is {:.3f} mm narrower than the tool{}: its middle is "
                                    "cut, {:.3f} mm into each wall",
                                    name, 2.0 * into,
                                    p.stockToLeaveXY > 0.0 ? " and the stock to leave" : "", into));
                            }
                        }
                    }
                    std::vector<std::vector<Vec2>> pieces;
                    double v0 = std::numeric_limits<double>::infinity(), v1 = -v0, mean = 0.0;
                    for (const Vec2& q : pts) {
                        v0 = std::min(v0, q.y());
                        v1 = std::max(v1, q.y());
                        mean += q.y() / static_cast<double>(pts.size());
                    }
                    if (!floor.periodic) {
                        pieces.push_back(pts);
                    } else if (v1 - v0 >= period) {
                        // Along the band: one period of it.
                        std::vector<Vec2> run = pts;
                        if (closed) {
                            run.push_back(pts.front());
                        }
                        const Regions window{{Polyline::rectangle({-1e9, 0.0}, {1e9, period}), {}}};
                        for (const std::vector<Vec2>& piece :
                             geom2d::clipPaths({run}, window, true, clip)) {
                            if (std::abs(std::abs(piece.back().y() - piece.front().y()) - period) <=
                                std::max(10.0 * tol, 1e-3 * period)) {
                                pieces.push_back(piece);
                            }
                        }
                        closed = false;
                    } else if (mean >= 0.0 && mean < period) {
                        pieces.push_back(pts); // an island's copy in this period
                    }
                    for (std::vector<Vec2>& piece : pieces) {
                        Pass pass;
                        pass.pts = closed ? startOnLongest(piece) : std::move(piece);
                        pass.closed = closed;
                        if (!middle) {
                            // Outer loops counter-clockwise, holes clockwise: the walls on the
                            // right.
                            pass.climbs = true;
                            pass.allowed = [centre, tol](const Vec2& q) {
                                return detail::anyContains(*centre, q, tol);
                            };
                        }
                        extendX(g, pass.pts);
                        g.passes.push_back(std::move(pass));
                    }
                }
            }
        }
    }
    g.offset = d; // the walls are beside every pass
    return g;
}

/// Ramps down along the start of a pass, forth and back, from the tool's height to z, ending
/// where it began.
void rampDown(toolpath::PathBuilder& b, const std::vector<Vec2>& pts, bool closed, double z,
              double angle, double feed, const std::string& name) {
    std::vector<Vec2> path = pts;
    if (closed) {
        path.push_back(pts.front());
    }
    const std::vector<double> cum = lengthsAlong(path, false);
    const double drop = b.position().z() - z;
    if (drop <= 1e-9) {
        return;
    }
    const double slope = std::tan(std::clamp(angle, 1e-3, kPi / 2.0 - 1e-3));
    const double run = std::min(cum.back(), drop / slope);
    const int legs =
        2 * std::max(1, static_cast<int>(std::ceil(drop / (2.0 * std::max(run, 1e-9) * slope))));
    if (run <= 1e-6 || legs > 1000) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("{} is too short to ramp into at {:.1f}°: plunge, or ramp steeper",
                                name, angle * 180.0 / kPi));
    }
    // The way forth: the points up to `run` and the point there.
    std::vector<std::pair<Vec2, double>> forth;
    for (std::size_t k = 1; k < path.size() && cum[k] < run; ++k) {
        forth.emplace_back(path[k], cum[k]);
    }
    {
        const auto it = std::upper_bound(cum.begin(), cum.end(), run);
        const std::size_t k =
            std::min<std::size_t>(static_cast<std::size_t>(it - cum.begin()), path.size() - 1);
        const double seg = cum[k] - cum[k - 1];
        const double t = seg > 0.0 ? (run - cum[k - 1]) / seg : 1.0;
        forth.emplace_back(path[k - 1] + t * (path[k] - path[k - 1]), run);
    }
    const double perLeg = drop / legs;
    double top = b.position().z();
    for (int leg = 0; leg < legs; ++leg) {
        const bool out = leg % 2 == 0;
        const std::size_t count = forth.size();
        for (std::size_t j = 0; j < count; ++j) {
            // Back: the points in reverse, then the start.
            const auto& [q, s] = out ? forth[j]
                                     : (j + 1 < count ? forth[count - 2 - j]
                                                      : std::pair<Vec2, double>{path.front(), 0.0});
            const double gone = out ? s : run - s;
            b.feed(Vec3(q.x(), q.y(), top - perLeg * gone / run), FeedClass::Ramp, feed);
        }
        top -= perLeg;
    }
}

/// The flat toolpath of a group: levels down to 0 (its cutting radius), each pass at each.
toolpath::Toolpath cutGroup(const Group& g, const std::vector<double>& levels, double topZ,
                            double clearZ, double retractZ, double r, const RotaryContourParams& p,
                            const Context& ctx, std::vector<std::string>& warnings) {
    const double period = kTwoPi * g.radius;
    const bool climb = p.direction == CutDirection::Climb;
    toolpath::Toolpath flat;
    flat.start = Vec3(g.passes.front().pts.front().x(), g.passes.front().pts.front().y(), clearZ);
    toolpath::PathBuilder b(flat);
    double vNow = flat.start.y();
    bool warnedLead = false;
    const double wanted = p.leadRadius.value_or(kContourLeadShare * 2.0 * r);

    // Largest lead radius (halving) whose arc and line keep the tool centre where it may go.
    const auto leadRadius = [&](const Pass& pass, const Vec2& at, const Vec2& t, bool in) {
        if (!pass.climbs || wanted <= 1e-6) {
            return 0.0;
        }
        const Vec2 n = climb ? leftOf(t) : Vec2(-leftOf(t));
        const auto fits = [&](double rl) {
            const Vec2 c = at + rl * n;
            for (int k = 0; k < 12; ++k) {
                const double a = (kPi / 2.0) * k / 12.0;
                const Vec2 q = c + rl * (std::cos(a) * (in ? -t : t) + std::sin(a) * -n);
                if (pass.allowed && !pass.allowed(q)) {
                    return false;
                }
            }
            for (int k = 1; k <= 4 && p.leadLength > 0.0; ++k) {
                const Vec2 q = c + rl * (in ? -t : t) + (p.leadLength * k / 4.0) * n;
                if (pass.allowed && !pass.allowed(q)) {
                    return false;
                }
            }
            return true;
        };
        double rl = wanted;
        for (int k = 0; k < 4 && !fits(rl); ++k) {
            rl *= 0.5;
        }
        if (!fits(rl)) {
            if (!warnedLead) {
                warnings.push_back(
                    fmt::format("{}: leads left out where they would cut the wall", g.name));
                warnedLead = true;
            }
            return 0.0;
        }
        return rl;
    };

    for (std::size_t li = 0; li < levels.size(); ++li) {
        ctx.cancel.throwIfCancelled();
        const double z = levels[li];
        const double approachZ = (li == 0 ? topZ : levels[li - 1]) + ctx.heights.feedClearance;
        for (const Pass& pass : g.passes) {
            // A whole turn on is the same place: start the nearest way round.
            std::vector<Vec2> pts = pass.pts;
            if (pass.climbs && *pass.climbs != climb) {
                std::reverse(pts.begin(), pts.end());
            }
            const double turns = std::round((vNow - pts.front().y()) / period);
            pts = shifted(pts, turns * period);
            Pass placed = pass;
            if (pass.allowed && turns != 0.0) {
                const double by = turns * period;
                placed.allowed = [inner = pass.allowed, by](const Vec2& q) {
                    return inner(q - Vec2(0.0, by));
                };
            }
            const Vec2 s = pts.front();
            const Vec2 t = firstDirection(pts);
            const Vec2 n = climb ? leftOf(t) : Vec2(-leftOf(t));
            const double rlIn = p.entry == EntryKind::Plunge ? leadRadius(placed, s, t, true) : 0.0;
            if (p.entry == EntryKind::Ramp) {
                b.rapidAbove(s, retractZ, approachZ);
                rampDown(b, pts, pass.closed, z, p.rampAngle, ctx.feeds.ramp, g.name);
            } else {
                const Vec2 c = s + rlIn * n;
                const Vec2 arcStart = rlIn > 0.0 ? Vec2(c - rlIn * t) : s;
                const Vec2 entry =
                    rlIn > 0.0 && p.leadLength > 0.0 ? Vec2(arcStart + p.leadLength * n) : arcStart;
                b.rapidAbove(entry, retractZ, approachZ);
                b.feed(Vec3(entry.x(), entry.y(), z), FeedClass::Plunge, ctx.feeds.plunge);
                if (rlIn > 0.0) {
                    b.feed(Vec3(arcStart.x(), arcStart.y(), z), FeedClass::LeadIn, ctx.feeds.lead);
                    b.arc(Vec3(s.x(), s.y(), z), c, !climb, FeedClass::LeadIn, ctx.feeds.lead);
                }
            }
            b.follow(Polyline::fromPoints(pts, pass.closed), z, FeedClass::Cut, ctx.feeds.cut);
            const Vec2 e = pass.closed ? s : pts.back();
            const Vec2 te = pass.closed ? Vec2((s - pts.back()).normalized()) : lastDirection(pts);
            const double rlOut = leadRadius(placed, e, te, false);
            if (rlOut > 0.0) {
                const Vec2 ne = climb ? leftOf(te) : Vec2(-leftOf(te));
                const Vec2 c = e + rlOut * ne;
                const Vec2 arcEnd = c + rlOut * te;
                b.arc(Vec3(arcEnd.x(), arcEnd.y(), z), c, !climb, FeedClass::LeadOut,
                      ctx.feeds.lead);
                if (p.leadLength > 0.0) {
                    const Vec2 exit = arcEnd + p.leadLength * ne;
                    b.feed(Vec3(exit.x(), exit.y(), z), FeedClass::LeadOut, ctx.feeds.lead);
                }
            }
            b.retractTo(retractZ);
            vNow = b.position().y();
        }
    }
    b.retractTo(clearZ);
    return flat;
}

} // namespace

toolpath::Toolpath rotaryContour(const std::vector<WrappedCurve>& curves,
                                 const std::vector<WrappedSlot>& slots, double stockRadius,
                                 double partXMin, double partXMax, const RotaryContourParams& p,
                                 const Context& ctx) {
    const double r = detail::requireMillingTool(ctx.tool);
    if (curves.empty() && slots.empty()) {
        throw Error(ErrorCode::InvalidArgument,
                    "a wrapped contour needs edges to follow or a slot's floor");
    }
    if (stockRadius <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "a wrapped contour needs a stock about WCS X");
    }
    const double d = r + p.stockToLeaveXY; // from the wall to the tool's centre
    if (d <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    "the stock to leave on the walls is more negative than the tool's radius");
    }
    const double tol = std::max(ctx.chordTolerance, 1e-3);
    toolpath::Toolpath tp = beginToolpath(ctx, "rotary_contour", Vec2::Zero());
    const double clearR = std::max(ctx.heights.clearance, stockRadius + 2.0);
    const double retractR = std::max(ctx.heights.retract, stockRadius + 1.0);

    std::vector<Group> groups;
    for (std::size_t k = 0; k < curves.size(); ++k) {
        groups.push_back(
            curveGroup(curves[k], fmt::format("curve {}", k + 1), p, d, tol, tp.warnings));
    }
    for (std::size_t k = 0; k < slots.size(); ++k) {
        groups.push_back(slotGroup(slots[k], fmt::format("slot {}", k + 1), p, d, r, partXMin,
                                   partXMax, tol, tp.warnings));
    }

    bool started = false;
    for (std::size_t gi = 0; gi < groups.size(); ++gi) {
        ctx.cancel.throwIfCancelled();
        const Group& g = groups[gi];
        if (g.passes.empty()) {
            continue;
        }
        // The stock over the group (and the tool either side of it).
        double top = stockRadius;
        if (p.stockRadiusAt) {
            top = 0.0;
            const double x0 = g.xMin - r, x1 = g.xMax + r;
            const int samples = std::max(2, static_cast<int>(std::ceil((x1 - x0) / 0.25)));
            for (int k = 0; k <= samples; ++k) {
                top = std::max(top, p.stockRadiusAt(x0 + (x1 - x0) * k / samples));
            }
        }
        const std::vector<double> levels =
            depthLevels(top - g.radius, 0.0, std::max(p.stepDown, 0.0));
        if (levels.empty()) {
            tp.warnings.push_back(fmt::format("{}: the stock, at radius {:.3f} mm, is not above "
                                              "its floor at {:.3f} mm: nothing to cut",
                                              g.name, top, g.radius));
            continue;
        }
        // Offsets are along the arc at the floor: a wall along the axis comes nearer to the
        // tool's side there by d - rho sin(d / rho).
        const double nearer = g.offset - g.radius * std::sin(g.offset / g.radius);
        if (g.offset > 0.0 && nearer > 0.01) {
            tp.warnings.push_back(fmt::format(
                "{}: offsets are along the arc at the floor, so a wall along the axis comes "
                "{:.3f} mm nearer the tool than the arc says at its foot",
                g.name, nearer));
        }
        const toolpath::Toolpath flat = cutGroup(g, levels, top - g.radius, clearR - g.radius,
                                                 retractR - g.radius, r, p, ctx, tp.warnings);
        appendWrapped(tp, wrapToolpath(flat, g.radius, tol), clearR, tol, started);
        ctx.progress.report(static_cast<double>(gi + 1) / static_cast<double>(groups.size()),
                            "Wrapped contour");
    }
    if (!started) {
        tp.warnings.push_back("nothing to cut");
        tp.start =
            wrapPoint(Vec3(0.5 * (partXMin + partXMax), 0.0, clearR - stockRadius), stockRadius)
                .first;
    }
    tp.strategy = "rotary_contour";
    return tp;
}

RotaryContourParams rotaryContourParams(const Json& values, const tooling::Tool& /*tool*/,
                                        const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("rotary_contour"), values);
    RotaryContourParams p;
    const std::string side = r.choice("side");
    p.side = side == "left" ? WrapSide::Left : side == "right" ? WrapSide::Right : WrapSide::On;
    p.depth = r.number("depth");
    p.stepDown = presetStepDown(preset);
    // Only a slot wider than the tool needs it, and refuses without it.
    p.stepover = preset && preset->radialDepth > 0.0 ? preset->radialDepth : 0.0;
    p.stockToLeaveXY = r.number("stock_to_leave_xy");
    p.stockToLeaveZ = r.number("stock_to_leave_z");
    if (r.isSet("lead_radius")) {
        p.leadRadius = r.number("lead_radius");
    }
    p.leadLength = r.number("lead_length");
    p.entry = r.choice("entry") == "ramp" ? EntryKind::Ramp : EntryKind::Plunge;
    p.rampAngle = r.number("ramp_angle");
    if (!r.isSet("ramp_angle") && preset && preset->rampAngle > 0.0) {
        p.rampAngle = preset->rampAngle;
    }
    p.direction =
        r.choice("direction") == "conventional" ? CutDirection::Conventional : CutDirection::Climb;
    return p;
}

} // namespace ocam::strategies
