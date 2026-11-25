// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Turning strategies in the lathe plane (u = axial Z, v = radius). See docs/design-log/0020.
//
// Nose-radius compensation in the computer: a round nose of radius r_n must stay outside the
// part grown by the stock allowance, so its centre stays outside C = part offset by
// (allowance + r_n). OD inserts are programmed at the imaginary tip, which is the nose centre
// minus (r_n, r_n); straight axial and radial cuts then land exactly on their targets and
// tapers and radii are compensated by the offset.

#include "ocam/strategies/Turning.hpp"

#include "Internal.hpp"
#include "ocam/strategies/Strategies.hpp"

#include <ocam/geom2d/Clip.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace ocam::strategies {
namespace {

using geom2d::Regions;
using tooling::TurningSide;
using toolpath::FeedClass;
using LoopSet = std::vector<std::vector<Vec2>>;

constexpr double kInf = std::numeric_limits<double>::infinity();

LoopSet loopsOf(const Regions& regions, double tol = 1e-4) {
    LoopSet loops;
    for (const auto& r : regions) {
        loops.push_back(r.outer.discretize(tol));
        for (const auto& h : r.holes) {
            loops.push_back(h.discretize(tol));
        }
    }
    return loops;
}

/// Even-odd intervals of the other coordinate where the line {coordinate axis = value} is inside.
std::vector<std::pair<double, double>> scanLine(const LoopSet& loops, int axis, double value) {
    const int other = 1 - axis;
    std::vector<double> xs;
    for (const auto& pts : loops) {
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const Vec2& p = pts[i];
            const Vec2& q = pts[(i + 1) % pts.size()];
            if ((p[axis] <= value) != (q[axis] <= value)) {
                xs.push_back(p[other] +
                             (value - p[axis]) * (q[other] - p[other]) / (q[axis] - p[axis]));
            }
        }
    }
    std::sort(xs.begin(), xs.end());
    std::vector<std::pair<double, double>> out;
    for (std::size_t i = 0; i + 1 < xs.size(); i += 2) {
        if (xs[i + 1] > xs[i]) {
            out.emplace_back(xs[i], xs[i + 1]);
        }
    }
    return out;
}

double outerAt(const LoopSet& loops, double z) {
    const auto iv = scanLine(loops, 0, z);
    return iv.empty() ? -kInf : iv.back().second;
}

double innerAt(const LoopSet& loops, double z) {
    const auto iv = scanLine(loops, 0, z);
    return iv.empty() ? kInf : iv.front().first;
}

geom2d::Aabb2 boundsOf(const Regions& regions) {
    geom2d::Aabb2 b;
    for (const auto& r : regions) {
        b.extend(r.bounds());
    }
    return b;
}

Vec3 at(double z, double r) {
    return Vec3(r, 0.0, z);
}

/// Vertex z values of the loops within [zBack, zFront], plus both ends, descending.
std::vector<double> breakpoints(const LoopSet& loops, double zBack, double zFront) {
    std::vector<double> zs{zFront, zBack};
    for (const auto& loop : loops) {
        for (const Vec2& p : loop) {
            if (p.x() > zBack && p.x() < zFront) {
                zs.push_back(p.x());
            }
        }
    }
    std::sort(zs.begin(), zs.end(), std::greater<>());
    zs.erase(std::unique(zs.begin(), zs.end(),
                         [](double a, double b) { return std::abs(a - b) < 1e-9; }),
             zs.end());
    return zs;
}

/// A polyline (from high z to low z, walls vertical) raised where it descends towards -Z faster
/// than slope t.
std::vector<Vec2> limitDescent(const std::vector<Vec2>& env, double t) {
    std::vector<Vec2> out{env.front()};
    Vec2 cur = env.front();
    for (std::size_t i = 1; i < env.size(); ++i) {
        const Vec2 a = env[i - 1];
        const Vec2 b = env[i];
        if (std::abs(b.x() - a.x()) < 1e-12) {
            const double r = std::max(b.y(), cur.y());
            if (std::abs(r - cur.y()) > 1e-12) {
                cur = Vec2(b.x(), r);
                out.push_back(cur);
            }
            continue;
        }
        auto limiter = [&](double z) {
            return cur.y() - t * (cur.x() - z);
        };
        const double dA = a.y() - limiter(a.x());
        const double dB = b.y() - limiter(b.x());
        if (dA >= -1e-12 && dB >= -1e-12) {
            out.push_back(b);
        } else if (dB <= 1e-12) {
            out.emplace_back(b.x(), limiter(b.x()));
        } else {
            const double s = dA / (dA - dB);
            const double zc = a.x() + s * (b.x() - a.x());
            out.emplace_back(zc, limiter(zc));
            out.push_back(b);
        }
        cur = out.back();
    }
    return out;
}

/// The envelope g (piecewise linear between breakpoints, possibly jumping at them) followed
/// from front to back, never descending faster than slope t: path(z) = max(g(z), limiter).
std::vector<Vec2> limitedEnvelope(const std::function<double(double)>& g,
                                  const std::vector<double>& breaks, double t) {
    constexpr double eps = 1e-7;
    // Envelope polyline with explicit vertical jumps.
    std::vector<Vec2> env;
    for (std::size_t i = 0; i < breaks.size(); ++i) {
        const double z = breaks[i];
        const double before = i == 0 ? g(z - eps) : g(z + eps);
        const double after = i + 1 == breaks.size() ? g(z + eps) : g(z - eps);
        env.emplace_back(z, std::max(before, -1e9));
        if (std::abs(after - before) > 1e-9) {
            env.emplace_back(z, std::max(after, -1e9));
        }
    }
    return limitDescent(env, t);
}

/// `path` (from high z to low z, walls vertical) lifted towards -Z as `t` allows.
std::vector<Vec2> limitAscent(std::vector<Vec2> path, double t) {
    for (Vec2& p : path) {
        p.x() = -p.x();
    }
    std::reverse(path.begin(), path.end());
    std::vector<Vec2> out = limitDescent(path, t);
    for (Vec2& p : out) {
        p.x() = -p.x();
    }
    std::reverse(out.begin(), out.end());
    return out;
}

/// Where a pass of the nose centre at radius `centre` meets `path` (from front to back) coming
/// from the front: `dir` is the side of the path the nose stays on. nullopt when it does not.
std::optional<double> firstReach(const std::vector<Vec2>& path, double centre, double dir) {
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const Vec2& a = path[i];
        const Vec2& b = path[i + 1];
        const double da = dir * (a.y() - centre);
        const double db = dir * (b.y() - centre);
        if (da > 0.0) {
            return a.x();
        }
        if (db > 0.0) {
            return a.x() + (b.x() - a.x()) * (-da) / (db - da);
        }
    }
    return std::nullopt;
}
/// Removes collinear points (within tol) of a 2D polyline.
std::vector<Vec2> simplify2(const std::vector<Vec2>& pts, double tol) {
    if (pts.size() < 3) {
        return pts;
    }
    std::vector<Vec2> out{pts.front()};
    for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
        const Vec2 a = out.back();
        const Vec2 b = pts[i + 1];
        const Vec2 d = b - a;
        const double len = d.norm();
        const double dist =
            len > 1e-12 ? std::abs((pts[i] - a).x() * d.y() - (pts[i] - a).y() * d.x()) / len
                        : (pts[i] - a).norm();
        if (dist > tol) {
            out.push_back(pts[i]);
        }
    }
    out.push_back(pts.back());
    return out;
}

void requireInsert(const tooling::Tool& tool, std::initializer_list<tooling::ToolType> types,
                   const char* what) {
    if (std::find(types.begin(), types.end(), tool.type) == types.end()) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("{} needs {}", what,
                                tooling::isTurningTool(tool.type) ? "another kind of insert"
                                                                  : "a turning insert"));
    }
}

/// Turning along Z needs the end edge off the turned surface: an end-cutting holder's can dip
/// below the nose, which only suits facing.
void requireEndClearance(const tooling::TurningShape& shape) {
    if (shape.trailingAngle < -1e-9) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("this holder faces: its end edge lies {:.1f}° below the feed "
                                "along Z and would cut into the turned surface",
                                radToDeg(-shape.trailingAngle)));
    }
}

toolpath::Toolpath beginTurning(const Context& ctx, std::string_view name, const TurnCommon& c,
                                const geom2d::Aabb2& stock, int quadrant) {
    toolpath::Toolpath tp = beginToolpath(ctx, name, Vec2::Zero());
    tp.start =
        at(stock.max.x() + 5.0 * c.clearance, std::max(stock.max.y(), 0.0) + 5.0 * c.clearance);
    tp.surfaceSpeed = c.surfaceSpeed;
    tp.maxSpindle = c.maxSpindle;
    tp.tipQuadrant = quadrant;
    return tp;
}

} // namespace

namespace {

/// Where passes end towards -Z: `backLimit` beyond the part's back end, outside passes no further
/// than the geometry's back stop (the nose centre `rn` in front of it), with a warning when the
/// part reaches behind it.
double backOf(const TurnGeometry& g, const geom2d::Aabb2& pb, double backLimit, TurningSide side,
              double rn, toolpath::Toolpath& tp) {
    double zBack = pb.min.x() - backLimit;
    if (side == TurningSide::Outside && g.backStop + rn > zBack) {
        zBack = g.backStop + rn;
        if (zBack > pb.min.x() + 1e-6) {
            tp.warnings.push_back(fmt::format("the passes stop at Z{:.3f}, clear of the chuck: the "
                                              "part behind it is not turned",
                                              zBack - rn));
        }
    }
    return zBack;
}

} // namespace

std::vector<Vec2> noseCentrePath(const Regions& part, double stockToLeave, double rn,
                                 double trailingAngle, double zBack, TurningSide side,
                                 double leadAngle) {
    const double grow = stockToLeave + rn;
    const Regions c = grow > 0.0 ? geom2d::offset(part, grow, {1e-3}) : part;
    if (c.empty()) {
        return {};
    }
    const LoopSet loops = loopsOf(c);
    const geom2d::Aabb2 cb = boundsOf(c);
    const auto breaks = breakpoints(loops, std::min(zBack, cb.max.x()), cb.max.x());
    const double t = std::tan(std::clamp(trailingAngle, 0.0, degToRad(89.0)));
    // Above 90 degrees the leading edge runs ahead of the nose, up at this slope.
    const double rise = leadAngle > kPi / 2.0 + 1e-9 ? std::tan(kPi - leadAngle) : 0.0;
    auto limited = [&](std::vector<Vec2> envelope) {
        return rise > 0.0 ? limitAscent(std::move(envelope), rise) : envelope;
    };
    if (side == TurningSide::Outside) {
        return limited(limitedEnvelope([&](double z) { return outerAt(loops, z); }, breaks, t));
    }
    auto path = limited(limitedEnvelope([&](double z) { return -innerAt(loops, z); }, breaks, t));
    for (auto& p : path) {
        p.y() = -p.y();
    }
    // A blind bore's bottom: the nose ends on the axis. What lies behind it is the part.
    for (std::size_t k = 0; k < path.size(); ++k) {
        if (std::isfinite(path[k].y()) && path[k].y() <= 0.0) {
            path[k].y() = 0.0;
            path.resize(k + 1);
            break;
        }
    }
    return path;
}

namespace {

/// What a tool on `side` cannot reach past `part`: outside, all below the part's outer envelope
/// (a bore's material is not an outside tool's); inside, all above its inner envelope. Between
/// lo and hi along Z, down to `rMin` or up to `rMax`.
Regions beyondEnvelope(const Regions& part, TurningSide side, double zBack, double zFront,
                       double rMin, double rMax) {
    constexpr double eps = 1e-7;
    const LoopSet loops = loopsOf(part);
    const bool od = side == TurningSide::Outside;
    const double base = od ? rMin : rMax;
    auto envelope = [&](double z) {
        const double v = od ? outerAt(loops, z) : innerAt(loops, z);
        return std::isfinite(v) ? v : base;
    };
    std::vector<Vec2> points{{zFront, base}};
    for (const double z : breakpoints(loops, zBack, zFront)) {
        points.emplace_back(z, envelope(z + eps));
        points.emplace_back(z, envelope(z - eps));
    }
    points.emplace_back(zBack, base);
    geom2d::Polyline outline = geom2d::Polyline::fromPoints(points, true);
    if (!outline.isCounterClockwise()) {
        std::reverse(points.begin(), points.end());
        outline = geom2d::Polyline::fromPoints(points, true);
    }
    return {{std::move(outline), {}}};
}

} // namespace

namespace {

/// A nose centre path (from the front back) up to where it last meets the stock: past that lies
/// only what an earlier setup finished. Stock that holds nothing to cut leaves the path whole.
std::vector<Vec2> trimmedToStock(std::vector<Vec2> centre, const TurnGeometry& g,
                                 double stockToLeave, double rn) {
    // Thinner than this is the clipping's noise along a finished surface.
    constexpr double kSliver = 0.02;
    const geom2d::ClipOptions clip{1e-3};
    const Regions left = geom2d::offset(
        geom2d::subtract(g.stock, geom2d::offset(g.part, stockToLeave + kSliver, clip), clip),
        -kSliver, clip);
    if (left.empty() || centre.size() < 2) {
        return centre;
    }
    const Regions reach = geom2d::offset(left, rn + 2.0 * kSliver, clip);
    for (std::size_t k = centre.size() - 1; k > 0; --k) {
        const Vec2& a = centre[k - 1];
        const Vec2& b = centre[k];
        // Lines and steps along an axis have no area of their own.
        const Vec2 lo = a.cwiseMin(b) - Vec2::Constant(1e-3);
        const Vec2 hi = a.cwiseMax(b) + Vec2::Constant(1e-3);
        if (!geom2d::intersect(reach, {{geom2d::Polyline::rectangle(lo, hi), {}}}, clip).empty()) {
            centre.resize(k + 1);
            return centre;
        }
    }
    return centre;
}

} // namespace

namespace {

/// Refuses a range whose tool would come through stock: what lies just in front of the range, at or
/// beyond the radius it starts at on its side, is in the way of a tool coming along Z.
void requireClearInFront(const TurnGeometry& g, const std::optional<TurnZRange>& range,
                         double rangeFront, double radius, double clear, bool od) {
    if (!range || g.stock.empty()) {
        return;
    }
    const geom2d::Aabb2 sb = boundsOf(g.stock);
    const Vec2 lo(rangeFront + 0.02, od ? radius : -1.0); // past the clipping's noise
    const Vec2 hi(rangeFront + clear, od ? sb.max.y() + 1.0 : radius);
    if (hi.y() <= lo.y()) {
        return;
    }
    if (!geom2d::intersect(g.stock, {{geom2d::Polyline::rectangle(lo, hi), {}}}, {1e-3}).empty()) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("stock stands where the tool comes from, next to the Z range at "
                                "Z{:.3f}: machine it first, or start the range further along",
                                g.zSign * rangeFront));
    }
}

/// `centre` (from the front back) from `front` on, cut exactly there.
std::vector<Vec2> fromFront(std::vector<Vec2> centre, double front) {
    std::vector<Vec2> out;
    for (std::size_t k = 0; k < centre.size(); ++k) {
        if (centre[k].x() > front + 1e-9) {
            continue;
        }
        if (out.empty() && k > 0 && centre[k - 1].x() > front) {
            const double t = (centre[k - 1].x() - front) / (centre[k - 1].x() - centre[k].x());
            out.push_back(centre[k - 1] + t * (centre[k] - centre[k - 1]));
        }
        out.push_back(centre[k]);
    }
    return out;
}

} // namespace

namespace {

/// Back turning: the same passes on the geometry mirrored in Z, and their moves mirrored back. What
/// stops a tool towards the chuck does not stop one running away from it.
Regions mirrored(const Regions& regions) {
    const auto flip = [](const geom2d::Polyline& line) {
        std::vector<Vec2> points = line.discretize(1e-4);
        for (Vec2& q : points) {
            q.x() = -q.x();
        }
        std::reverse(points.begin(), points.end());
        return geom2d::Polyline::fromPoints(points, true);
    };
    Regions out;
    for (const auto& r : regions) {
        geom2d::Region m{flip(r.outer), {}};
        for (const auto& h : r.holes) {
            m.holes.push_back(flip(h));
        }
        out.push_back(std::move(m));
    }
    return out;
}

TurnGeometry mirrored(const TurnGeometry& g) {
    TurnGeometry m{mirrored(g.part), mirrored(g.stock)};
    m.zSign = -g.zSign;
    return m;
}

std::optional<TurnZRange> mirrored(const std::optional<TurnZRange>& range) {
    if (!range) {
        return std::nullopt;
    }
    return TurnZRange{-range->back, -range->front};
}

toolpath::Toolpath mirrored(toolpath::Toolpath tp) {
    tp.start.z() = -tp.start.z();
    for (toolpath::Move& m : tp.moves) {
        m.end.z() = -m.end.z();
    }
    tp.tipQuadrant = tp.tipQuadrant == 2 ? 1 : 4;
    // Mirrored, the material is on the other side of the direction of travel.
    tp.compensation =
        tp.compensation == toolpath::Compensation::Left    ? toolpath::Compensation::Right
        : tp.compensation == toolpath::Compensation::Right ? toolpath::Compensation::Left
                                                           : tp.compensation;
    return tp;
}

} // namespace

toolpath::Toolpath turnFace(const TurnGeometry& g, const TurnFaceParams& p, const Context& ctx) {
    requireInsert(ctx.tool, {tooling::ToolType::TurningInsert}, "facing");
    if (g.stock.empty() || g.part.empty()) {
        throw Error(ErrorCode::InvalidArgument, "facing needs the stock and the part");
    }
    if (const tooling::TurningShape shape = tooling::turningShape(ctx.tool);
        shape.leadAngle > kPi / 2.0 + 1e-9) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("facing needs a cutting edge angle of at least 90°: at the tool's "
                                "{:.1f}° its side edge would cut into the face",
                                radToDeg(kPi - shape.leadAngle)));
    }
    const geom2d::Aabb2 sb = boundsOf(g.stock), pb = boundsOf(g.part);
    toolpath::Toolpath tp = beginTurning(ctx, "turn_face", p.common, sb, 3);
    const double target = pb.max.x() + p.stockToLeave;
    const auto levels = depthLevels(sb.max.x(), target, p.depthOfCut);
    if (levels.empty()) {
        tp.warnings.push_back("nothing to face: the stock front is at or behind the part's front");
        return tp;
    }
    const double clear = p.common.clearance;
    const double rSafe = sb.max.y() + clear;
    const double bore = innerAt(loopsOf(g.stock), sb.max.x() - 1e-6);
    // The nose centre must cross the axis, or its arc leaves a small nub at the centre.
    const double rn = tooling::turningShape(ctx.tool).noseRadius;
    const double xEnd = bore > 1e-6 && bore < kInf ? bore - p.overshoot : -(p.overshoot + rn);
    toolpath::PathBuilder b(tp);
    b.rapid(at(sb.max.x() + clear, rSafe));
    for (const double z : levels) {
        ctx.cancel.throwIfCancelled();
        b.rapid(at(z, rSafe));
        b.feed(at(z, xEnd), FeedClass::Cut, ctx.feeds.cut);
        b.feed(at(z + clear, xEnd), FeedClass::LeadOut, ctx.feeds.lead);
        b.rapid(at(z + clear, rSafe));
    }
    b.rapid(tp.start);
    return tp;
}

toolpath::Toolpath turnRough(const TurnGeometry& g, const TurnRoughParams& p, const Context& ctx) {
    if (p.awayFromChuck) {
        TurnRoughParams q = p;
        q.awayFromChuck = false;
        q.zRange = mirrored(p.zRange);
        return mirrored(turnRough(mirrored(g), q, ctx));
    }
    requireInsert(ctx.tool, {tooling::ToolType::TurningInsert}, "rough turning");
    if (g.stock.empty() || g.part.empty()) {
        throw Error(ErrorCode::InvalidArgument, "rough turning needs the stock and the part");
    }
    if (p.depthOfCut <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the depth of cut must be positive");
    }
    const bool od = p.side == TurningSide::Outside;
    const tooling::TurningShape shape = tooling::turningShape(ctx.tool, p.side);
    requireEndClearance(shape);
    const double rn = shape.noseRadius;
    const double clear = p.common.clearance;
    const geom2d::ClipOptions clip{1e-3};
    const geom2d::Aabb2 sb = boundsOf(g.stock), pb = boundsOf(g.part);
    toolpath::Toolpath tp = beginTurning(ctx, "turn_rough", p.common, sb, od ? 3 : 2);

    const Regions partOffset =
        p.stockToLeave > 0.0 ? geom2d::offset(g.part, p.stockToLeave, clip) : g.part;
    const Regions keepOut = geom2d::offset(g.part, p.stockToLeave + rn, clip);
    const double fullBack = backOf(g, pb, p.backLimit, p.side, 0.0, tp);
    const double rangeFront = p.zRange ? std::min(pb.max.x(), p.zRange->front) : pb.max.x();
    const double zBack = p.zRange ? std::max(fullBack, p.zRange->back) : fullBack;
    if (zBack >= rangeFront) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("the Z range from Z{:.3f} to Z{:.3f} holds none of the part",
                                g.zSign * rangeFront, g.zSign * zBack));
    }
    // Roughing does not face: material in front of the part's front face (plus allowance) is
    // left to facing.
    const Regions window{
        {geom2d::Polyline::rectangle(Vec2(zBack, -1.0), Vec2(rangeFront, sb.max.y() + 1.0)), {}}};
    const Regions material = geom2d::intersect(
        geom2d::subtract(
            geom2d::subtract(g.stock, partOffset, clip),
            beyondEnvelope(partOffset, p.side, zBack, pb.max.x(), -1.0, sb.max.y() + 1.0), clip),
        window, clip);
    if (material.empty()) {
        tp.warnings.push_back("no stock to remove outside the allowance");
        return tp;
    }
    const geom2d::Aabb2 mb = boundsOf(material);
    const LoopSet keepLoops = loopsOf(keepOut);
    const LoopSet matLoops = loopsOf(material);
    const double zFront = p.zRange ? rangeFront + clear : sb.max.x() + clear;

    std::vector<double> levels;
    double stockRadius = 0.0;
    if (od) {
        stockRadius = sb.max.y();
        levels = depthLevels(stockRadius, mb.min.y(), p.depthOfCut);
    } else {
        stockRadius = innerAt(loopsOf(g.stock), sb.max.x() - 1e-6);
        if (!(stockRadius > clear) || stockRadius == kInf) {
            throw Error(ErrorCode::InvalidArgument,
                        "inside roughing needs a bore in the stock (tube stock or drilling first)");
        }
        // Increasing radii from the bore out to the part (depthLevels works on descending values).
        for (const double v : depthLevels(-stockRadius, -mb.max.y(), p.depthOfCut)) {
            levels.push_back(-v);
        }
    }
    const double dir = od ? 1.0 : -1.0; // retract direction in radius
    // A leading edge ahead of the nose reaches a shoulder first: the passes end where it does.
    const std::vector<Vec2> edgeLimit =
        shape.leadAngle > kPi / 2.0 + 1e-9
            ? noseCentrePath(g.part, p.stockToLeave, rn, degToRad(89.0), zBack, p.side,
                             shape.leadAngle)
            : std::vector<Vec2>{};
    if (!p.across) {
        requireClearInFront(g, p.zRange, rangeFront, od ? mb.min.y() : mb.max.y(), clear, od);
    }
    toolpath::PathBuilder b(tp);
    if (p.zRange) {
        b.rapid(at(sb.max.x() + clear, od ? stockRadius + clear : stockRadius - clear));
    }
    b.rapid(at(zFront, od ? stockRadius + clear : stockRadius - clear));
    double previous = stockRadius;
    bool any = false;
    if (p.across) {
        if (shape.leadAngle > kPi / 2.0 + 1e-9) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("passes across Z need a cutting edge angle of at least 90°: "
                                    "at the tool's {:.1f}° its side edge would cut into the face",
                                    radToDeg(kPi - shape.leadAngle)));
        }
        // From the front back. Behind a part that stands higher further forward the tool cannot
        // come down: the passes stop where they stopped in front of it.
        double reached = od ? -kInf : kInf;
        double from = mb.max.x();
        for (const double z : depthLevels(mb.max.x(), mb.min.x(), p.depthOfCut)) {
            ctx.cancel.throwIfCancelled();
            const double centre = z + rn + 1e-7;
            for (const auto& [lo, hi] : scanLine(keepLoops, 0, centre)) {
                reached = od ? std::max(reached, hi - rn) : std::min(reached, lo + rn);
            }
            const Regions band = geom2d::intersect(
                material,
                {{geom2d::Polyline::rectangle(Vec2(z, -1.0), Vec2(from, sb.max.y() + 1.0)), {}}},
                clip);
            from = z;
            if (band.empty()) {
                continue;
            }
            const geom2d::Aabb2 bb = boundsOf(band);
            const double start = (od ? bb.max.y() : bb.min.y()) + dir * clear;
            const double end =
                od ? std::max(bb.min.y(), std::isfinite(reached) ? reached : bb.min.y())
                   : std::min(bb.max.y(), std::isfinite(reached) ? reached : bb.max.y());
            if (dir * (start - end) <= 0.0) {
                continue;
            }
            any = true;
            b.rapid(at(z, od ? stockRadius + clear : stockRadius - clear));
            b.rapid(at(z, start));
            b.feed(at(z, end), FeedClass::Cut, ctx.feeds.cut);
            b.feed(at(z + p.retract, end + dir * p.retract), FeedClass::LeadOut, ctx.feeds.lead);
            b.rapid(at(z + p.retract, od ? stockRadius + clear : stockRadius - clear));
        }
        levels.clear();
    }
    for (const double x : levels) {
        ctx.cancel.throwIfCancelled();
        // Just off the allowance boundary, so a final pass exactly on it is not blocked.
        const double centre = x + dir * (rn + 1e-7);
        double zEnd = zBack;
        for (const auto& [lo, hi] : scanLine(keepLoops, 1, centre)) {
            if (hi - rn > zEnd) {
                zEnd = hi - rn; // the nose touches the allowance here
            }
        }
        if (const auto reach = firstReach(edgeLimit, centre, dir)) {
            zEnd = std::max(zEnd, *reach - rn);
        }
        zEnd = std::max(zEnd, zBack);
        if (zEnd >= sb.max.x() - 1e-6) {
            continue;
        }
        // Material between this pass and the previous one, ahead of the end point?
        bool cuts = false;
        for (const auto& [lo, hi] : scanLine(matLoops, 1, x + dir * 1e-6)) {
            cuts = cuts || (hi > zEnd + 1e-6);
        }
        for (const auto& [lo, hi] : scanLine(matLoops, 1, 0.5 * (x + previous))) {
            cuts = cuts || (hi > zEnd + 1e-6);
        }
        if (!cuts) {
            previous = x;
            continue;
        }
        // Stock an earlier setup cut may end sooner than the part stops the pass: no air cutting.
        const Regions band = geom2d::intersect(
            material,
            {{geom2d::Polyline::rectangle(Vec2(zBack - 1.0, std::min(x, previous)),
                                          Vec2(sb.max.x() + 1.0, std::max(x, previous))),
              {}}},
            clip);
        if (!band.empty()) {
            zEnd = std::max(zEnd, boundsOf(band).min.x() - clear);
        }
        previous = x;
        any = true;
        b.rapid(at(zFront, x));
        b.feed(at(zEnd, x), FeedClass::Cut, ctx.feeds.cut);
        b.feed(at(zEnd + p.retract, x + dir * p.retract), FeedClass::LeadOut, ctx.feeds.lead);
        b.rapid(at(zFront, x + dir * p.retract));
    }
    if (!any) {
        tp.warnings.push_back("no rough pass reaches the stock");
    }
    // Profile pass along the allowance: removes the steps the level passes leave (at most one
    // depth of cut thick) so the finish pass meets a uniform allowance.
    if (p.profilePass) {
        std::vector<Vec2> centre =
            noseCentrePath(g.part, p.stockToLeave, rn, shape.trailingAngle,
                           od ? std::max(zBack, g.backStop + rn) : zBack, p.side, shape.leadAngle);
        std::erase_if(centre, [](const Vec2& q) { return !std::isfinite(q.y()) || q.y() < -1e6; });
        centre = trimmedToStock(std::move(centre), g, p.stockToLeave, rn);
        if (p.zRange) {
            centre = fromFront(std::move(centre), rangeFront + rn);
        }
        if (centre.size() >= 2) {
            std::vector<Vec2> prog;
            for (const Vec2& c : centre) {
                prog.emplace_back(c.x() - rn, od ? c.y() - rn : c.y() + rn);
            }
            prog = simplify2(prog, 1e-4);
            const Vec2 s = prog.front();
            b.rapid(at(zFront, s.y() + dir * clear));
            b.rapid(at(s.x() + clear, s.y() + dir * clear));
            b.feed(at(s.x(), s.y()), FeedClass::LeadIn, ctx.feeds.lead);
            for (std::size_t k = 1; k < prog.size(); ++k) {
                b.feed(at(prog[k].x(), prog[k].y()), FeedClass::Cut, ctx.feeds.cut);
            }
            const Vec2 e = prog.back();
            b.feed(at(e.x() + p.retract, std::max(e.y() + dir * p.retract, od ? e.y() : 0.0)),
                   FeedClass::LeadOut, ctx.feeds.lead);
        }
    }
    b.rapid(at(b.position().z(), od ? stockRadius + clear : stockRadius - clear));
    b.rapid(at(zFront, od ? stockRadius + clear : stockRadius - clear));
    if (p.zRange) {
        b.rapid(at(sb.max.x() + clear, od ? stockRadius + clear : stockRadius - clear));
    }
    b.rapid(tp.start);
    return tp;
}

toolpath::Toolpath turnFinish(const TurnGeometry& g, const TurnFinishParams& p,
                              const Context& ctx) {
    if (p.awayFromChuck) {
        TurnFinishParams q = p;
        q.awayFromChuck = false;
        q.zRange = mirrored(p.zRange);
        return mirrored(turnFinish(mirrored(g), q, ctx));
    }
    requireInsert(ctx.tool, {tooling::ToolType::TurningInsert}, "finish turning");
    if (g.part.empty()) {
        throw Error(ErrorCode::InvalidArgument, "finish turning needs the part");
    }
    const bool od = p.side == TurningSide::Outside;
    const tooling::TurningShape shape = tooling::turningShape(ctx.tool, p.side);
    requireEndClearance(shape);
    const double rn = shape.noseRadius;
    const double clear = p.common.clearance;
    const geom2d::Aabb2 sb = boundsOf(g.stock.empty() ? g.part : g.stock), pb = boundsOf(g.part);
    toolpath::Toolpath tp = beginTurning(ctx, "turn_finish", p.common, sb, od ? 3 : 2);
    const double fullBack = backOf(g, pb, p.backLimit, p.side, rn, tp);
    // In control the program is the profile: the control offsets the nose.
    const double offset = p.inControl ? 0.0 : rn;
    const double rangeFront = p.zRange ? std::min(pb.max.x(), p.zRange->front) : pb.max.x();
    const double zBack = p.zRange ? std::max(fullBack, p.zRange->back + offset) : fullBack;
    if (p.zRange && zBack >= rangeFront + offset) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("the Z range from Z{:.3f} to Z{:.3f} holds none of the part",
                                g.zSign * rangeFront, g.zSign * p.zRange->back));
    }
    std::vector<Vec2> centre = noseCentrePath(g.part, p.stockToLeave, offset, shape.trailingAngle,
                                              zBack, p.side, shape.leadAngle);
    // Inside: only where there is a bore to run in.
    std::erase_if(centre, [&](const Vec2& q) { return !std::isfinite(q.y()) || q.y() < -1e6; });
    centre = trimmedToStock(std::move(centre), g, p.stockToLeave, rn);
    if (p.zRange) {
        centre = fromFront(std::move(centre), rangeFront + offset);
    }
    if (centre.size() < 2) {
        tp.warnings.push_back("no profile to finish");
        return tp;
    }
    std::vector<Vec2> prog;
    for (const Vec2& c : centre) {
        prog.emplace_back(c.x() - offset, od ? c.y() - offset : c.y() + offset);
    }
    prog = simplify2(prog, 1e-4);
    const double dir = od ? 1.0 : -1.0;
    const double rSafe = od ? sb.max.y() + clear : std::max(0.0, prog.front().y() - clear);
    if (p.inControl) {
        // Towards the chuck the material is on the left of an outside pass in the lathe plane as
        // controls see it: the tool keeps right of it, and left of a bore's.
        tp.compensation = od ? toolpath::Compensation::Right : toolpath::Compensation::Left;
    }
    toolpath::PathBuilder b(tp);
    const Vec2 s = prog.front();
    if (p.zRange) {
        // Past whatever stands in front of the range, and down on to it from off the surface.
        requireClearInFront(g, p.zRange, s.x(), s.y(), clear, od);
        b.rapid(at(sb.max.x() + clear, rSafe));
    }
    b.rapid(at(s.x() + clear, rSafe));
    // Compensation comes on over a move across the radius too.
    b.rapid(at(s.x() + clear, s.y() + (p.zRange || p.inControl ? dir * clear : 0.0)));
    b.feed(at(s.x(), s.y()), FeedClass::LeadIn, ctx.feeds.lead);
    for (std::size_t k = 1; k < prog.size(); ++k) {
        b.feed(at(prog[k].x(), prog[k].y()), FeedClass::Cut, ctx.feeds.cut);
    }
    const Vec2 e = prog.back();
    // Off the wall; off a blind bore's bottom, whose axis it ends on, along the axis instead.
    const Vec2 off =
        !od && e.y() < clear ? Vec2(e.x() + clear, e.y()) : Vec2(e.x(), e.y() + dir * clear);
    b.feed(at(off.x(), off.y()), FeedClass::LeadOut, ctx.feeds.lead);
    b.rapid(at(off.x(), rSafe));
    b.rapid(at(sb.max.x() + clear, rSafe));
    b.rapid(tp.start);
    return tp;
}

namespace {

/// Grooves in a front face: where the part's front, seen along Z, dips between higher faces on
/// both sides of the radius. The insert plunges along -Z, its programmed point on its inner corner.
toolpath::Toolpath faceGrooves(const TurnGeometry& g, const TurnGrooveParams& p,
                               const Context& ctx) {
    const tooling::TurningShape shape = tooling::turningShape(ctx.tool, TurningSide::Face);
    const double w = shape.width;
    const double clear = p.common.clearance;
    const geom2d::Aabb2 sb = boundsOf(g.stock.empty() ? g.part : g.stock), pb = boundsOf(g.part);
    toolpath::Toolpath tp = beginTurning(ctx, "turn_groove", p.common, sb, 8);
    const LoopSet loops = loopsOf(g.part);
    // The part's front at a radius (-infinity off the part).
    const auto front = [&](double r) {
        const auto iv = scanLine(loops, 1, r);
        return iv.empty() ? -kInf : iv.back().second;
    };
    std::vector<double> radii{pb.min.y(), pb.max.y()};
    for (const auto& loop : loops) {
        for (const Vec2& q : loop) {
            radii.push_back(q.y());
        }
    }
    std::sort(radii.begin(), radii.end());
    radii.erase(std::unique(radii.begin(), radii.end(),
                            [](double a, double b) { return std::abs(a - b) < 1e-9; }),
                radii.end());
    std::vector<double> mids; // the middle of each span between radii
    for (std::size_t i = 0; i + 1 < radii.size(); ++i) {
        mids.push_back(0.5 * (radii[i] + radii[i + 1]));
    }
    std::vector<std::pair<double, double>> grooves; // (low r, high r)
    for (std::size_t i = 0; i < mids.size(); ++i) {
        double inward = -kInf, outward = -kInf;
        for (std::size_t k = 0; k < i; ++k) {
            inward = std::max(inward, front(mids[k]));
        }
        for (std::size_t k = i + 1; k < mids.size(); ++k) {
            outward = std::max(outward, front(mids[k]));
        }
        if (front(mids[i]) > -kInf && std::min(inward, outward) - front(mids[i]) > 0.01) {
            if (!grooves.empty() && std::abs(grooves.back().second - radii[i]) < 1e-9) {
                grooves.back().second = radii[i + 1];
            } else {
                grooves.emplace_back(radii[i], radii[i + 1]);
            }
        }
    }
    if (grooves.empty()) {
        tp.warnings.push_back("the part has no face grooves (dips in its front between higher "
                              "faces on both sides)");
        return tp;
    }
    const double zSafe = sb.max.x() + clear;
    const double step = std::clamp(p.step.value_or(kGrooveStepShare * w), 0.1 * w, w);
    toolpath::PathBuilder b(tp);
    b.rapid(at(zSafe, tp.start.x()));
    for (const auto& [rl, rh] : grooves) {
        const double lo = rl + p.stockToLeave;
        const double hi = rh - p.stockToLeave - w;
        if (hi < lo - 1e-9) {
            tp.warnings.push_back(fmt::format(
                "the face groove at radius {:.3f} is narrower than the insert ({:.3f} mm)", rl, w));
            continue;
        }
        std::vector<double> positions;
        for (double r = hi; r > lo + 1e-9; r -= step) {
            positions.push_back(r);
        }
        positions.push_back(lo);
        for (const double r : positions) {
            ctx.cancel.throwIfCancelled();
            double floor = std::max(front(r + 1e-7), front(r + w - 1e-7));
            for (const double m : mids) {
                if (m > r && m < r + w) {
                    floor = std::max(floor, front(m));
                }
            }
            floor += p.stockToLeave;
            b.rapid(at(zSafe, r));
            if (p.peck > 0.0) {
                for (double z = zSafe - clear - p.peck; z > floor; z -= p.peck) {
                    b.feed(at(z, r), FeedClass::Cut, ctx.feeds.cut);
                    b.rapid(at(z + 0.5, r));
                    b.rapid(at(z + 0.05, r));
                }
            }
            b.feed(at(floor, r), FeedClass::Cut, ctx.feeds.cut);
            b.rapid(at(zSafe, r));
        }
    }
    b.rapid(tp.start);
    return tp;
}

} // namespace

toolpath::Toolpath turnGroove(const TurnGeometry& g, const TurnGrooveParams& p,
                              const Context& ctx) {
    requireInsert(ctx.tool, {tooling::ToolType::GroovingInsert, tooling::ToolType::PartingInsert},
                  "grooving");
    if (g.part.empty()) {
        throw Error(ErrorCode::InvalidArgument, "grooving needs the part");
    }
    if (p.side == TurningSide::Face) {
        return faceGrooves(g, p, ctx);
    }
    // Radii signed so that "out of the cut" is up: the bore's as negatives.
    const bool od = p.side == TurningSide::Outside;
    const double dir = od ? 1.0 : -1.0;
    const tooling::TurningShape shape = tooling::turningShape(ctx.tool, p.side);
    const double w = shape.width;
    const double clear = p.common.clearance;
    const geom2d::Aabb2 sb = boundsOf(g.stock.empty() ? g.part : g.stock), pb = boundsOf(g.part);
    toolpath::Toolpath tp = beginTurning(ctx, "turn_groove", p.common, sb, od ? 3 : 2);
    const LoopSet loops = loopsOf(g.part);
    const auto breaks = breakpoints(loops, pb.min.x(), pb.max.x());
    auto envelope = [&](double z) {
        return od ? outerAt(loops, z) : -innerAt(loops, z);
    };
    // Recesses: where the part lies below what is visible from the front (running maximum).
    const auto visible = limitedEnvelope(envelope, breaks, 0.0);
    auto visibleAt = [&](double z) {
        double best = -kInf;
        for (const Vec2& q : visible) {
            if (q.x() >= z - 1e-9) {
                best = std::max(best, q.y());
            }
        }
        return best;
    };
    std::vector<std::pair<double, double>> grooves; // (low z, high z)
    for (std::size_t i = 0; i + 1 < breaks.size(); ++i) {
        const double mid = 0.5 * (breaks[i] + breaks[i + 1]);
        if (visibleAt(mid) - envelope(mid) > 0.01) {
            if (!grooves.empty() && std::abs(grooves.back().first - breaks[i]) < 1e-9) {
                grooves.back().first = breaks[i + 1];
            } else {
                grooves.emplace_back(breaks[i + 1], breaks[i]);
            }
        }
    }
    if (grooves.empty()) {
        tp.warnings.push_back(
            "the part has no grooves (recesses below the profile seen from the front)");
        return tp;
    }
    double rSafe = sb.max.y() + clear; // signed
    if (!od) {
        // Clear of the narrowest of the stock and the part on the way in.
        const LoopSet stockLoops = loopsOf(g.stock.empty() ? g.part : g.stock);
        double bore = kInf;
        for (const double z : breakpoints(stockLoops, grooves.front().first, sb.max.x())) {
            bore = std::min(bore, innerAt(stockLoops, std::min(z, sb.max.x() - 1e-6)));
        }
        for (const double z : breaks) {
            if (z > grooves.front().first) {
                bore = std::min(bore, innerAt(loops, z - 1e-7));
            }
        }
        if (!(bore > clear) || bore == kInf) {
            throw Error(ErrorCode::InvalidArgument,
                        "inside grooving needs a bore in the stock (tube stock or drilling first)");
        }
        rSafe = -(bore - clear);
    }
    const double step = std::clamp(p.step.value_or(kGrooveStepShare * w), 0.1 * w, w);
    toolpath::PathBuilder b(tp);
    b.rapid(at(pb.max.x() + clear, dir * rSafe));
    for (const auto& [zl, zh] : grooves) {
        const double lo = zl + p.stockToLeave;
        const double hi = zh - p.stockToLeave - w;
        if (hi < lo - 1e-9) {
            tp.warnings.push_back(fmt::format(
                "the groove at Z{:.3f} is narrower than the insert ({:.3f} mm)", zh, w));
            continue;
        }
        std::vector<double> positions;
        for (double z = hi; z > lo + 1e-9; z -= step) {
            positions.push_back(z);
        }
        positions.push_back(lo);
        for (const double z : positions) {
            ctx.cancel.throwIfCancelled();
            double floor = std::max(envelope(z + 1e-7), envelope(z + w - 1e-7));
            for (const double q : breaks) {
                if (q > z && q < z + w) {
                    floor = std::max({floor, envelope(q + 1e-7), envelope(q - 1e-7)});
                }
            }
            floor += p.stockToLeave;
            b.rapid(at(z, dir * rSafe));
            if (p.peck > 0.0) {
                for (double r = rSafe - clear - p.peck; r > floor; r -= p.peck) {
                    b.feed(at(z, dir * r), FeedClass::Cut, ctx.feeds.cut);
                    b.rapid(at(z, dir * (r + 0.5)));
                    b.rapid(at(z, dir * (r + 0.05)));
                }
            }
            b.feed(at(z, dir * floor), FeedClass::Cut, ctx.feeds.cut);
            b.rapid(at(z, dir * rSafe));
        }
    }
    if (!od) {
        b.rapid(at(pb.max.x() + clear, dir * rSafe)); // out of the bore along Z first
    }
    b.rapid(tp.start);
    return tp;
}

toolpath::Toolpath turnThread(const TurnGeometry& g, const TurnThreadParams& p,
                              const Context& ctx) {
    requireInsert(ctx.tool, {tooling::ToolType::ThreadingInsert}, "threading");
    if (g.part.empty()) {
        throw Error(ErrorCode::InvalidArgument, "threading needs the part");
    }
    const double pitch = p.pitch > 0.0 ? p.pitch : ctx.tool.insert.threadPitch;
    if (pitch <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "set the thread pitch (operation or insert)");
    }
    if (p.length <= 0.0 || p.passes < 1) {
        throw Error(ErrorCode::InvalidArgument, "the thread needs a length and at least one pass");
    }
    if (p.starts < 1) {
        throw Error(ErrorCode::InvalidArgument, "a thread needs at least one start");
    }
    const bool od = p.side == TurningSide::Outside;
    const double dir = od ? 1.0 : -1.0; // radial direction out of the cut
    const geom2d::Aabb2 pb = boundsOf(g.part);
    const LoopSet loops = loopsOf(g.part);
    const double zFront = pb.max.x() - p.startOffset; // the thread's end nearest the front
    const double zBack = zFront - p.length;
    // Crest: where the cut starts, the major outside and the minor inside. Depth: towards the part.
    const double depth = p.depth > 0.0 ? p.depth : (od ? 0.6134 : 0.5413) * pitch;
    const double crest = [&] {
        const double z = zFront - 1e-6;
        if (od) {
            return p.majorDiameter > 0.0 ? p.majorDiameter / 2.0 : outerAt(loops, z);
        }
        return p.majorDiameter > 0.0 ? p.majorDiameter / 2.0 - depth : innerAt(loops, z);
    }();
    if (!(crest > 0.0) || !std::isfinite(crest)) {
        throw Error(ErrorCode::InvalidArgument,
                    od ? "no part diameter at the thread start; set the major diameter"
                       : "no bore at the thread start; set the major diameter");
    }
    if (!od && !g.stock.empty()) {
        const double bore = innerAt(loopsOf(g.stock), boundsOf(g.stock).max.x() - 1e-6);
        if (!(bore > p.common.clearance) || bore == kInf) {
            throw Error(
                ErrorCode::InvalidArgument,
                "inside threading needs a bore in the stock (tube stock or drilling first)");
        }
    }
    const double leadIn = p.leadIn > 0.0 ? p.leadIn : 3.0 * pitch * p.starts;
    const double clear = p.common.clearance;
    TurnCommon common = p.common;
    common.surfaceSpeed = 0.0; // threads need a constant spindle speed
    toolpath::Toolpath tp = beginTurning(ctx, "turn_thread", common, pb, od ? 3 : 2);
    // The cut runs from `from` to `to`, with its run-up beyond `from`: right hand towards the
    // chuck, left hand away from it.
    const double from = p.leftHand ? zBack : zFront;
    const double to = p.leftHand ? zFront : zBack;
    const double along = p.leftHand ? -1.0 : 1.0; // from the cut towards its run-up
    const double slope = p.taper / p.length;      // radius per mm from the front to the back
    const auto taperAt = [&](double z) {
        return slope * (zFront - z);
    };
    // The run-up must be in air, which is above the cut's floor outside and below it inside.
    for (const double z :
         {from + along * 0.25 * leadIn, from + along * 0.5 * leadIn, from + along * leadIn}) {
        const double there = od ? outerAt(loops, z) : innerAt(loops, z);
        const double floor = crest - dir * depth + taperAt(z);
        if (std::isfinite(there) && dir * (there - floor) > 1e-3) {
            tp.warnings.push_back(p.leftHand
                                      ? "the thread run-up crosses material: a left-hand thread "
                                        "needs a relief groove behind it"
                                      : "the thread run-up crosses material: start the thread at "
                                        "the front or after a relief groove");
            break;
        }
    }
    // Clear of the crest all along, tapered or not, and of the stock outside.
    const double zRun = from + along * leadIn;
    const double stockRadius = g.stock.empty() ? -kInf : boundsOf(g.stock).max.y();
    const double rSafe =
        od ? std::max({crest + taperAt(zRun), crest + taperAt(to), stockRadius}) + clear
           : std::min(crest + taperAt(zRun), crest + taperAt(to)) - clear;
    const double lead = pitch * p.starts;
    const double feed = lead * ctx.feeds.spindle;
    toolpath::PathBuilder b(tp);
    b.rapid(at(zRun, rSafe));
    for (int start = 0; start < p.starts; ++start) {
        // Each start begins a pitch further along, so the helices lie a pitch apart.
        const double shift = along * pitch * start;
        for (int k = 1; k <= p.passes + std::max(0, p.springPasses); ++k) {
            ctx.cancel.throwIfCancelled();
            const double d = depth * std::sqrt(std::min(1.0, static_cast<double>(k) / p.passes));
            const double z0 = zRun + shift;
            b.rapid(at(z0, rSafe));
            b.rapid(at(z0, crest - dir * d + taperAt(z0)));
            b.thread(at(to, crest - dir * d + taperAt(to)), lead, feed);
            b.rapid(at(to, rSafe));
            b.rapid(at(z0, rSafe));
        }
    }
    b.rapid(tp.start);
    return tp;
}

toolpath::Toolpath turnPartOff(const TurnGeometry& g, const TurnPartOffParams& p,
                               const Context& ctx) {
    requireInsert(ctx.tool, {tooling::ToolType::PartingInsert, tooling::ToolType::GroovingInsert},
                  "parting off");
    if (g.part.empty()) {
        throw Error(ErrorCode::InvalidArgument, "parting off needs the part");
    }
    const tooling::TurningShape shape = tooling::turningShape(ctx.tool);
    const geom2d::Aabb2 sb = boundsOf(g.stock.empty() ? g.part : g.stock), pb = boundsOf(g.part);
    toolpath::Toolpath tp = beginTurning(ctx, "turn_partoff", p.common, sb, 3);
    const double zRef =
        pb.min.x() - p.zOffset - shape.width; // left corner; the right face cuts the part
    const double clear = p.common.clearance;
    const double rSafe = sb.max.y() + clear;
    const double slowFrom = p.xEnd + std::max(0.0, p.slowDistance);
    if (slowFrom > p.xEnd && ctx.feeds.partOffEnd <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    "the part-off end feed of T" + std::to_string(ctx.tool.number) +
                        " is 0: set it in the cutting preset or in the operation's speeds and "
                        "feeds, or set \"End feed over the last\" to 0");
    }
    toolpath::PathBuilder b(tp);
    b.rapid(at(pb.max.x() + clear, rSafe));
    b.rapid(at(zRef, rSafe));
    double r = rSafe;
    if (p.peck > 0.0) {
        for (double next = r - clear - p.peck; next > slowFrom; next -= p.peck) {
            b.feed(at(zRef, next), FeedClass::Cut, ctx.feeds.cut);
            b.rapid(at(zRef, next + 0.5));
            b.rapid(at(zRef, next + 0.05));
            r = next;
        }
    }
    if (slowFrom < r) {
        b.feed(at(zRef, slowFrom), FeedClass::Cut, ctx.feeds.cut);
    }
    if (slowFrom > p.xEnd) {
        b.feed(at(zRef, p.xEnd), FeedClass::Cut, ctx.feeds.partOffEnd);
    }
    b.rapid(at(zRef, rSafe));
    b.rapid(tp.start);
    return tp;
}

toolpath::Toolpath barPull(const BarPullGeometry& g, const BarPullParams& p, const Context& ctx) {
    const tooling::Tool& tool = ctx.tool;
    if (tool.type != tooling::ToolType::BarPuller) {
        throw Error(ErrorCode::InvalidArgument, "a bar pull needs a bar puller");
    }
    if (p.pullLength <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    "the pull length is 0: set how far the bar comes out of the chuck in the "
                    "operation's Pull length");
    }
    if (p.grip <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    "the grip is 0: set how far the puller goes over the bar in the operation's "
                    "Grip");
    }
    if (p.grip > tool.mill.fluteLength + 1e-9) {
        throw Error(
            ErrorCode::InvalidArgument,
            fmt::format("the grip is {:.3f} mm, and the jaws of T{} are {:.3f} mm deep: set "
                        "a smaller Grip",
                        p.grip, tool.number, tool.mill.fluteLength));
    }
    if (g.radius <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "there is no bar in front of the chuck to pull");
    }
    if (g.radius > tool.mill.tipDiameter / 2.0 + 1e-9) {
        throw Error(
            ErrorCode::InvalidArgument,
            fmt::format("the bar is Ø{:.3f} where the puller grips it, and T{} takes a bar of "
                        "at most Ø{:.3f}",
                        2.0 * g.radius, tool.number, tool.mill.tipDiameter));
    }
    const std::string presetName = "the cutting preset of T" + std::to_string(tool.number);
    if (ctx.feeds.plunge <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    "the plunge feed of " + presetName +
                        " is 0: it is the feed onto and off the bar; set it in the preset or in "
                        "the operation's speeds and feeds");
    }
    if (ctx.feeds.pull <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    "the pull feed of " + presetName +
                        " is 0: set it in the preset or in the operation's speeds and feeds");
    }

    toolpath::Toolpath tp = beginToolpath(ctx, "bar_pull", Vec2::Zero());
    tp.spindleSpeed = 0.0; // the chuck opens: the spindle stands still
    tp.start = at(g.front + p.clearance, 0.0);
    toolpath::PathBuilder b(tp);
    const auto function = [&](toolpath::MachineFunction f, double seconds) {
        toolpath::Move m;
        m.kind = toolpath::MoveKind::Dwell;
        m.feedClass = FeedClass::Plunge;
        m.end = b.position();
        m.dwell = seconds;
        m.function = f;
        tp.moves.push_back(m); // standing still: the builder's position stays
    };
    const double gripZ = g.front - p.grip;
    b.feed(at(gripZ, 0.0), FeedClass::Plunge, ctx.feeds.plunge);
    if (p.barFeeder) {
        function(toolpath::MachineFunction::BarFeederOn, 0.0);
    }
    function(toolpath::MachineFunction::ChuckOpen, p.dwell);
    b.feed(at(gripZ + p.pullLength, 0.0), FeedClass::Cut, ctx.feeds.pull);
    function(toolpath::MachineFunction::ChuckClose, p.dwell);
    if (p.barFeeder) {
        function(toolpath::MachineFunction::BarFeederOff, 0.0);
    }
    b.feed(at(g.front + p.pullLength + p.clearance, 0.0), FeedClass::Plunge, ctx.feeds.plunge);
    return tp;
}

BarPullParams barPullParams(const Json& values, const tooling::Tool& tool) {
    const ParamReader r(*findStrategy("bar_pull"), values);
    BarPullParams p;
    p.pullLength = r.number("pull_length");
    // While not set, the grip is the puller's: its ParamDef says how (toolDefault).
    p.grip = r.isSet("grip") ? r.number("grip") : r.info().find("grip")->toolDefault(tool);
    p.clearance = r.number("clearance");
    p.dwell = r.number("dwell");
    p.barFeeder = r.flag("bar_feeder");
    return p;
}

// -- Parameters -------------------------------------------------------------------------------

Feeds turningFeeds(const Json& values, const tooling::CuttingPreset* preset,
                   double referenceDiameter, bool needsFeed) {
    // Speeds and feeds are never guessed: they come from the insert's cutting preset.
    if (!preset) {
        throw Error(ErrorCode::InvalidArgument, "choose a cutting preset for the insert");
    }
    if (preset->surfaceSpeed <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    "the cutting preset '" + preset->name + "' has no surface speed");
    }
    if (needsFeed && preset->feedPerTooth <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    "the cutting preset '" + preset->name + "' has no feed per revolution");
    }
    const double vc = preset->surfaceSpeed;
    const double f = preset->feedPerTooth;
    // The spindle limit every turning operation shows (its default included): read through the
    // parameter definition, never a second copy of the default.
    const double maxRpm =
        ParamReader(*findStrategy("turn_face"), values.is_object() ? values : Json::object())
            .integer("max_rpm");
    if (referenceDiameter <= 1e-6) {
        throw Error(ErrorCode::InvalidArgument, "the stock has no diameter to set the speed at");
    }
    const double d = referenceDiameter;
    Feeds feeds;
    feeds.spindle = std::min(vc / (kPi * d), maxRpm / 60.0);
    feeds.cut = f * feeds.spindle;
    feeds.lead = preset->leadFeedPerRev * feeds.spindle;
    feeds.partOffEnd = preset->partOffEndFeed * feeds.spindle;
    feeds.ramp = preset->rampFeed;
    feeds.plunge = preset->plungeFeed;
    feeds.coolant = preset ? preset->coolant : tooling::Coolant::Flood;
    return feeds;
}

namespace {

TurnCommon commonFrom(const ParamReader& r, const tooling::CuttingPreset* preset) {
    TurnCommon c;
    c.clearance = r.number("clearance");
    if (r.flag("css")) {
        if (!preset || preset->surfaceSpeed <= 0.0) {
            throw Error(ErrorCode::InvalidArgument,
                        "constant surface speed needs a cutting preset with a surface speed");
        }
        c.surfaceSpeed = preset->surfaceSpeed;
    }
    c.maxSpindle = r.integer("max_rpm") / 60.0;
    return c;
}

TurningSide sideFrom(const ParamReader& r) {
    return r.choice("side") == "inside" ? TurningSide::Inside : TurningSide::Outside;
}

} // namespace

TurnCommon turnCommon(const Json& values, const tooling::CuttingPreset* preset) {
    return commonFrom(ParamReader(*findStrategy("turn_rough"), values), preset);
}

TurnFaceParams turnFaceParams(const Json& values, const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("turn_face"), values);
    TurnFaceParams p;
    p.common = commonFrom(r, preset);
    p.depthOfCut = presetStepDown(preset);
    p.stockToLeave = r.number("stock_to_leave");
    p.overshoot = r.number("overshoot");
    return p;
}

bool turnHasZRange(std::string_view operationType, const Json& values) {
    const StrategyInfo* info = findStrategy(std::string(operationType));
    if (!info || (operationType != "turn_rough" && operationType != "turn_finish")) {
        return false;
    }
    return ParamReader(*info, values).choice("z_range") == "limited";
}

namespace {

std::optional<TurnZRange> zRangeFrom(const ParamReader& r) {
    if (r.choice("z_range") != "limited") {
        return std::nullopt;
    }
    const TurnZRange range{r.number("z_front"), r.number("z_back")};
    if (range.back >= range.front) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("the Z range's front (Z{:.3f}) must be in front of its back "
                                "(Z{:.3f})",
                                range.front, range.back));
    }
    return range;
}

} // namespace

TurnRoughParams turnRoughParams(const Json& values, const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("turn_rough"), values);
    TurnRoughParams p;
    p.common = commonFrom(r, preset);
    p.side = sideFrom(r);
    p.zRange = zRangeFrom(r);
    p.across = r.choice("pattern") == "across";
    p.awayFromChuck = r.choice("direction") == "away_from_chuck";
    p.depthOfCut = presetStepDown(preset);
    p.stockToLeave = r.number("stock_to_leave");
    p.retract = r.number("retract");
    p.backLimit = r.number("back_limit");
    p.profilePass = r.flag("profile_pass");
    return p;
}

TurnRoughParams turnSoftJawsParams(const Json& values, const tooling::CuttingPreset* preset,
                                   bool inside) {
    const ParamReader r(*findStrategy("turn_soft_jaws"), values);
    TurnRoughParams p;
    p.common = commonFrom(r, preset);
    p.side = inside ? tooling::TurningSide::Inside : tooling::TurningSide::Outside;
    p.depthOfCut = presetStepDown(preset);
    p.stockToLeave = r.number("stock_to_leave");
    p.retract = r.number("retract");
    p.profilePass = r.flag("profile_pass");
    return p;
}

TurnFinishParams turnFinishParams(const Json& values, const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("turn_finish"), values);
    TurnFinishParams p;
    p.common = commonFrom(r, preset);
    p.side = sideFrom(r);
    p.zRange = zRangeFrom(r);
    p.awayFromChuck = r.choice("direction") == "away_from_chuck";
    p.inControl = r.choice("compensation") == "control";
    p.stockToLeave = r.number("stock_to_leave");
    p.backLimit = r.number("back_limit");
    return p;
}

TurnGrooveParams turnGrooveParams(const Json& values, const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("turn_groove"), values);
    TurnGrooveParams p;
    p.common = commonFrom(r, preset);
    p.side = r.choice("side") == "face"     ? TurningSide::Face
             : r.choice("side") == "inside" ? TurningSide::Inside
                                            : TurningSide::Outside;
    p.stockToLeave = r.number("stock_to_leave");
    if (r.isSet("plunge_step")) {
        p.step = r.number("plunge_step");
    }
    p.peck = r.number("peck");
    return p;
}

TurnThreadParams turnThreadParams(const Json& values, const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("turn_thread"), values);
    TurnThreadParams p;
    p.common = commonFrom(r, preset);
    p.pitch = r.number("pitch");
    p.majorDiameter = r.number("major_diameter");
    p.startOffset = r.number("start_offset");
    p.length = r.number("length");
    p.depth = r.number("depth");
    p.passes = r.integer("passes");
    p.springPasses = r.integer("spring_passes");
    p.leadIn = r.number("lead_in");
    p.side = sideFrom(r);
    p.leftHand = r.choice("hand") == "left";
    p.starts = r.integer("starts");
    p.taper = r.number("taper");
    return p;
}

TurnPartOffParams turnPartOffParams(const Json& values, const tooling::CuttingPreset* preset) {
    const ParamReader r(*findStrategy("turn_partoff"), values);
    TurnPartOffParams p;
    p.common = commonFrom(r, preset);
    p.zOffset = r.number("z_offset");
    p.xEnd = r.number("x_end");
    p.peck = r.number("peck");
    p.slowDistance = r.number("slow_distance");
    return p;
}

} // namespace ocam::strategies
