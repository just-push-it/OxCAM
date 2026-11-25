// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/sim/Lathe.hpp"

#include "ocam/sim/Simulate.hpp"
#include "ocam/sim/Surface.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/geom2d/Clip.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <tuple>

namespace ocam::sim {
namespace {

std::vector<std::vector<Vec2>> loopsOf(const geom2d::Regions& regions, double tol) {
    std::vector<std::vector<Vec2>> loops;
    for (const auto& r : regions) {
        loops.push_back(r.outer.discretize(tol));
        for (const auto& h : r.holes) {
            loops.push_back(h.discretize(tol));
        }
    }
    return loops;
}

using Edges = std::vector<std::pair<Vec2, Vec2>>;

/// The edges of closed loops whose coordinate `axis` reaches into [lo, hi].
Edges edgesOf(const std::vector<std::vector<Vec2>>& loops, int axis, double lo, double hi) {
    Edges edges;
    for (const auto& pts : loops) {
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const Vec2& p = pts[i];
            const Vec2& q = pts[(i + 1) % pts.size()];
            if (std::max(p[axis], q[axis]) >= lo && std::min(p[axis], q[axis]) <= hi) {
                edges.emplace_back(p, q);
            }
        }
    }
    return edges;
}

/// Intervals of coordinate `other` where the line {coordinate `axis` = value} lies inside the
/// loops the edges are of (even-odd); edges that cannot reach `value` may be left out.
std::vector<std::pair<double, double>> scan(const Edges& edges, int axis, double value) {
    const int other = 1 - axis;
    std::vector<double> xs;
    for (const auto& [p, q] : edges) {
        if ((p[axis] <= value) != (q[axis] <= value)) {
            xs.push_back(p[other] +
                         (value - p[axis]) * (q[other] - p[other]) / (q[axis] - p[axis]));
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

double loopMoment(const std::vector<Vec2>& pts) {
    // Integral of v over the enclosed area (signed by orientation).
    double sum = 0.0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const Vec2& a = pts[i];
        const Vec2& b = pts[(i + 1) % pts.size()];
        sum += (a.x() * b.y() - b.x() * a.y()) * (a.y() + b.y());
    }
    return sum / 6.0;
}

} // namespace

std::vector<Vec2> convexHull(std::vector<Vec2> pts) {
    std::sort(pts.begin(), pts.end(), [](const Vec2& a, const Vec2& b) {
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    pts.erase(std::unique(pts.begin(), pts.end(),
                          [](const Vec2& a, const Vec2& b) { return (a - b).norm() < 1e-12; }),
              pts.end());
    if (pts.size() < 3) {
        return pts;
    }
    auto cross = [](const Vec2& o, const Vec2& a, const Vec2& b) {
        return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    };
    std::vector<Vec2> hull(2 * pts.size());
    std::size_t k = 0;
    for (const Vec2& p : pts) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], p) <= 0) {
            --k;
        }
        hull[k++] = p;
    }
    for (std::size_t i = pts.size() - 1, lower = k + 1; i-- > 0;) {
        while (k >= lower && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0) {
            --k;
        }
        hull[k++] = pts[i];
    }
    hull.resize(k - 1);
    return hull;
}

double revolvedVolume(const geom2d::Regions& profile) {
    double moment = 0.0;
    for (const auto& r : profile) {
        moment += std::abs(loopMoment(r.outer.discretize(1e-4)));
        for (const auto& h : r.holes) {
            moment -= std::abs(loopMoment(h.discretize(1e-4)));
        }
    }
    return 2.0 * kPi * moment;
}

LatheStock LatheStock::cylinder(double zMin, double zMax, double rOuter, double rInner) {
    if (!(zMax > zMin) || !(rOuter > rInner) || rInner < 0.0) {
        throw Error(ErrorCode::InvalidArgument, "invalid bar stock");
    }
    return LatheStock({{geom2d::Polyline::rectangle(Vec2(zMin, rInner), Vec2(zMax, rOuter)), {}}});
}

void LatheStock::subtract(const geom2d::Regions& area) {
    if (!area.empty() && !profile_.empty()) {
        profile_ = geom2d::subtract(profile_, area, {1e-4});
    }
}

void cutHoles(LatheStock& stock, const std::vector<Vec2>& profile, const toolpath::Toolpath& path,
              std::size_t begin, std::size_t end) {
    const std::size_t n = std::min(end, path.moves.size());
    Vec3 pos = toolPositionAt(path, begin);
    for (std::size_t k = begin; k < n; ++k) {
        const toolpath::Move& m = path.moves[k];
        if (m.kind == toolpath::MoveKind::Dwell) {
            continue;
        }
        if (m.kind == toolpath::MoveKind::Linear) {
            const Vec3 back = pos - m.end;
            const double along = back.dot(m.axis);
            if (along > 1e-6 && (back - along * m.axis).norm() < 1e-6) {
                stock.cutHole({m.end, m.axis, along, profile});
            }
        }
        pos = m.end;
    }
}

// -- Bar work (DL-0091) --------------------------------------------------------------------------

geom2d::Regions LatheStock::heldBar() const {
    double back = std::numeric_limits<double>::infinity();
    for (const geom2d::Region& r : profile_) {
        back = std::min(back, r.bounds().min.x());
    }
    geom2d::Regions held;
    for (const geom2d::Region& r : profile_) {
        if (r.bounds().min.x() <= back + 1e-6) {
            held.push_back(r);
        }
    }
    return held;
}

LatheStock LatheStock::pulled(double distance) const {
    const geom2d::Regions held = heldBar();
    if (held.empty() || distance == 0.0) {
        return *this;
    }
    geom2d::Aabb2 box;
    for (const geom2d::Region& r : held) {
        box.extend(r.bounds());
    }
    const auto moved = [&](const geom2d::Polyline& line) {
        std::vector<Vec2> points = line.discretize(1e-4);
        for (Vec2& q : points) {
            q.x() += distance;
        }
        return geom2d::Polyline::fromPoints(points, true);
    };
    geom2d::Regions out;
    for (const geom2d::Region& r : held) {
        geom2d::Region piece{moved(r.outer), {}};
        for (const geom2d::Polyline& hole : r.holes) {
            piece.holes.push_back(moved(hole));
        }
        out.push_back(std::move(piece));
    }
    // Behind it, the bar as it was at its back end: what a slice there holds, radially.
    const double slice = std::min(1e-2, 0.5 * (box.max.x() - box.min.x()));
    const geom2d::Regions back = geom2d::intersect(
        held, {{geom2d::Polyline::rectangle(Vec2(box.min.x(), -1.0),
                                            Vec2(box.min.x() + slice, box.max.y() + 1.0)),
                {}}});
    geom2d::Regions behind;
    for (const geom2d::Region& r : back) {
        const geom2d::Aabb2 b = r.bounds();
        behind.push_back({geom2d::Polyline::rectangle(Vec2(box.min.x(), b.min.y()),
                                                      Vec2(box.min.x() + distance, b.max.y())),
                          {}});
    }
    LatheStock result(behind.empty() ? out : geom2d::unite(out, behind));
    // The cuts in the bar go with it; those in what came off go with that.
    LatheStock cuts;
    const auto inBar = [&](double z) {
        return z >= box.min.x() - 1e-6 && z <= box.max.x() + 1e-6;
    };
    for (const LatheThreadCut& c : threads_) {
        if (inBar(c.from.x()) || inBar(c.to.x())) {
            cuts.threads_.push_back(c);
        }
    }
    for (const LatheHoleCut& h : holes_) {
        if (inBar(h.tip.z())) {
            cuts.holes_.push_back(h);
        }
    }
    Transform3 along = Transform3::Identity();
    along.translation() = Vec3(0.0, 0.0, distance);
    result.takeCuts(cuts, along);
    return result;
}

double barPulledBy(const toolpath::Toolpath& path, std::size_t end, double t) {
    double pulled = 0.0;
    bool open = false;
    Vec3 pos = path.start;
    const std::size_t n = std::min(end + 1, path.moves.size());
    for (std::size_t k = 0; k < n; ++k) {
        const toolpath::Move& m = path.moves[k];
        const double share = k == end ? std::clamp(t, 0.0, 1.0) : 1.0;
        if (m.kind == toolpath::MoveKind::Dwell) {
            if (k < end) {
                if (m.function == toolpath::MachineFunction::ChuckOpen) {
                    open = true;
                } else if (m.function == toolpath::MachineFunction::ChuckClose) {
                    open = false;
                }
            }
            continue;
        }
        if (open) {
            pulled += share * (m.end.z() - pos.z());
        }
        pos = m.end;
    }
    return pulled;
}

bool offTheAxis(const toolpath::Toolpath& path) {
    return std::any_of(path.moves.begin(), path.moves.end(), [](const toolpath::Move& m) {
        return (m.axis - Vec3::UnitZ()).norm() > 1e-9 ||
               (m.kind != toolpath::MoveKind::Dwell && m.end.head<2>().norm() > 1e-9);
    });
}

namespace {

/// A swept hull (counter-clockwise) mirrored over the axis, still counter-clockwise.
geom2d::Region acrossAxis(std::vector<Vec2> hull) {
    for (Vec2& p : hull) {
        p.y() = -p.y();
    }
    std::reverse(hull.begin(), hull.end());
    return {geom2d::Polyline::fromPoints(hull, true), {}};
}

} // namespace

LatheSimResult simulateTurning(LatheStock& stock, const geom2d::Polyline& insert,
                               const toolpath::Toolpath& path, std::size_t begin, std::size_t end,
                               CancelToken* cancel, std::vector<Collision>* rapids) {
    using toolpath::MoveKind;
    LatheSimResult result;
    const std::vector<Vec2> shape = insert.discretize(2e-3);
    if (shape.size() < 3) {
        throw Error(ErrorCode::InvalidArgument, "the insert outline is empty");
    }
    auto lathePoint = [](const Vec3& p) {
        return Vec2(p.z(), p.x());
    };
    geom2d::Regions pending;
    auto flush = [&] {
        if (!pending.empty()) {
            stock.subtract(geom2d::unite(pending, {}, {1e-4}));
            pending.clear();
        }
    };
    auto sweep = [&](const Vec2& a, const Vec2& b) {
        std::vector<Vec2> pts;
        pts.reserve(2 * shape.size());
        for (const Vec2& s : shape) {
            pts.push_back(a + s);
            pts.push_back(b + s);
        }
        const auto hull = convexHull(std::move(pts));
        if (hull.size() >= 3) {
            pending.push_back({geom2d::Polyline::fromPoints(hull, true), {}});
        }
        if (pending.size() >= 256) {
            flush();
        }
    };
    // A rapid through material: more than an insert resting on a wall it has just cut.
    constexpr double kThroughArea = 1e-3;
    auto checkRapid = [&](std::size_t k, const Vec3& from) {
        if (!rapids || path.moves[k].kind != MoveKind::Rapid) {
            return;
        }
        flush();
        std::vector<Vec2> pts;
        pts.reserve(2 * shape.size());
        for (const Vec2& s : shape) {
            pts.push_back(lathePoint(from) + s);
            pts.push_back(lathePoint(path.moves[k].end) + s);
        }
        const auto hull = convexHull(std::move(pts));
        if (hull.size() < 3) {
            return;
        }
        geom2d::Regions swept{{geom2d::Polyline::fromPoints(hull, true), {}}};
        if (std::any_of(hull.begin(), hull.end(), [](const Vec2& p) { return p.y() < 0.0; })) {
            swept.push_back(acrossAxis(hull));
        }
        for (const geom2d::Region& s : swept) {
            const geom2d::Regions overlap = geom2d::intersect({s}, stock.profile(), {1e-4});
            double area = 0.0;
            for (const auto& r : overlap) {
                area += r.area();
            }
            if (area > kThroughArea) {
                const Vec2 at = overlap.front().outer.vertices().front().p;
                if (!rapids->empty() && rapids->back().lastMove + 1 >= k) {
                    rapids->back().lastMove = k;
                } else {
                    rapids->push_back({CollisionKind::Rapid, k, k, Vec3(at.y(), 0.0, at.x()), 0.0});
                }
                return;
            }
        }
    };
    const std::size_t n = std::min(end, path.moves.size());
    Vec3 pos = toolPositionAt(path, begin);
    // Thread cutting leaves a helical form the profile cannot hold: its passes are held as cuts,
    // and the rapids around them take nothing off.
    if (std::any_of(path.moves.begin(), path.moves.end(),
                    [](const toolpath::Move& m) { return m.pitch > 0.0; })) {
        for (std::size_t k = begin; k < n; ++k) {
            const toolpath::Move& m = path.moves[k];
            // A tap cuts its thread into a bore its silhouette does not show: it adds nothing.
            if (m.pitch > 0.0 && path.isTurning()) {
                stock.cutThread(insert, lathePoint(pos), lathePoint(m.end), m.pitch);
                ++result.threadMoves;
            }
            checkRapid(k, pos);
            pos = m.end;
        }
        result.movesSimulated = n > begin ? n - begin : 0;
        return result;
    }
    for (std::size_t k = begin; k < n; ++k) {
        if (cancel && (k & 255u) == 0) {
            cancel->throwIfCancelled();
        }
        const toolpath::Move& m = path.moves[k];
        if (m.kind == MoveKind::Dwell || m.kind == MoveKind::Cycle) {
            continue;
        }
        if (m.isArc()) {
            const auto pts = toolpath::sampleArc(pos, m, 2e-3);
            for (std::size_t q = 0; q + 1 < pts.size(); ++q) {
                sweep(lathePoint(pts[q]), lathePoint(pts[q + 1]));
            }
        } else {
            checkRapid(k, pos);
            sweep(lathePoint(pos), lathePoint(m.end));
        }
        pos = m.end;
    }
    flush();
    result.movesSimulated = n > begin ? n - begin : 0;
    return result;
}

std::vector<Collision> checkTurningBody(const toolpath::Toolpath& path,
                                        const geom2d::Polyline& insert,
                                        const std::vector<geom2d::Polyline>& body, LatheStock stock,
                                        std::size_t maxCollisions, CancelToken* cancel) {
    using toolpath::MoveKind;
    std::vector<Collision> out;
    std::vector<std::vector<Vec2>> shapes;
    for (const geom2d::Polyline& piece : body) {
        shapes.push_back(piece.discretize(1e-2));
    }
    auto lathePoint = [](const Vec3& p) {
        return Vec2(p.z(), p.x());
    };
    Vec3 pos = toolPositionAt(path, 0);
    for (std::size_t k = 0; k < path.moves.size(); ++k) {
        if (cancel && (k & 255u) == 0) {
            cancel->throwIfCancelled();
        }
        const toolpath::Move& m = path.moves[k];
        if (m.kind == MoveKind::Dwell || m.kind == MoveKind::Cycle) {
            continue;
        }
        // Behind the insert, what it cuts on this move is gone by the time the body gets there.
        simulateTurning(stock, insert, path, k, k + 1, cancel);
        const geom2d::Regions& now = stock.profile();
        geom2d::Aabb2 box;
        for (const auto& r : now) {
            box.extend(r.bounds());
        }
        bool touches = false;
        Vec2 contact = Vec2::Zero();
        // Arcs by their chord: holders stand well off the cut.
        const Vec2 a = lathePoint(pos);
        const Vec2 b = lathePoint(m.end);
        for (const std::vector<Vec2>& shape : shapes) {
            std::vector<Vec2> pts;
            pts.reserve(2 * shape.size());
            for (const Vec2& s : shape) {
                pts.push_back(a + s);
                pts.push_back(b + s);
            }
            const auto hull = convexHull(std::move(pts));
            if (hull.size() < 3) {
                continue;
            }
            geom2d::Regions swept{{geom2d::Polyline::fromPoints(hull, true), {}}};
            geom2d::Aabb2 hb;
            for (const Vec2& p : hull) {
                hb.extend(p);
            }
            if (hb.min.y() < 0.0) {
                swept.push_back(acrossAxis(hull));
                hb.extend(Vec2(hb.min.x(), -hb.min.y()));
            }
            if (box.isEmpty() || hb.max.x() < box.min.x() || hb.min.x() > box.max.x() ||
                hb.max.y() < box.min.y() || hb.min.y() > box.max.y()) {
                continue;
            }
            for (const geom2d::Region& s : swept) {
                double area = 0.0;
                const geom2d::Regions overlap = geom2d::intersect({s}, now, {1e-4});
                for (const auto& r : overlap) {
                    area += r.area();
                }
                if (area > 1e-6) {
                    touches = true;
                    contact = overlap.front().outer.vertices().front().p;
                }
            }
        }
        if (touches) {
            if (!out.empty() && out.back().lastMove + 1 >= k) {
                out.back().lastMove = k;
            } else if (out.size() < maxCollisions) {
                out.push_back(
                    {CollisionKind::Holder, k, k, Vec3(contact.y(), 0.0, contact.x()), 0.0});
            }
        }
        pos = m.end;
    }
    return out;
}

Comparison compareProfile(const geom2d::Regions& stock, const geom2d::Regions& part,
                          double tolerance) {
    Comparison c;
    c.tolerance = tolerance;
    const geom2d::ClipOptions clip{1e-4};
    const ProfileDistance distance(part, clip.chordTolerance);
    // What is left of `area` outside `by`: its volume, and how far it reaches from the part's
    // outline (the farthest point of a polygon from a line is one of its vertices).
    auto measure = [&](const geom2d::Regions& area, const geom2d::Regions& by, double& volume,
                       double& deepest, std::size_t& count) {
        const geom2d::Regions left = geom2d::subtract(area, by, clip);
        geom2d::Regions kept;
        for (const geom2d::Region& r : left) {
            if (r.area() < 1e-6) {
                continue; // a sliver of the booleans
            }
            ++count;
            for (const geom2d::Vertex& v : r.outer.vertices()) {
                deepest = std::max(deepest, std::abs(distance(v.p)));
            }
            kept.push_back(r);
        }
        volume = revolvedVolume(kept);
    };
    measure(stock, geom2d::offset(part, tolerance, clip), c.excessVolume, c.maxExcess,
            c.excessRays);
    measure(geom2d::offset(part, -tolerance, clip), stock, c.gougeVolume, c.maxGouge, c.gougeRays);
    return c;
}

// -- Bodies turning with the spindle ----------------------------------------------------------

namespace {

/// The part of a polygon on the side `sign * (z - level) >= 0`.
std::vector<Vec3> clipZ(const std::vector<Vec3>& poly, double level, double sign) {
    std::vector<Vec3> out;
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const Vec3& p = poly[i];
        const Vec3& q = poly[(i + 1) % poly.size()];
        const double dp = sign * (p.z() - level);
        const double dq = sign * (q.z() - level);
        if (dp >= 0.0) {
            out.push_back(p);
        }
        if ((dp >= 0.0) != (dq >= 0.0)) {
            out.push_back(p + (q - p) * (dp / (dp - dq)));
        }
    }
    return out;
}

/// The smallest distance from the Z axis to a convex polygon seen along Z.
double nearestRadius(const std::vector<Vec3>& poly) {
    double best = std::numeric_limits<double>::infinity();
    double turn = 0.0;
    bool positive = false, negative = false;
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const Vec2 a = poly[i].head<2>();
        const Vec2 b = poly[(i + 1) % poly.size()].head<2>();
        const Vec2 ab = b - a;
        const double len2 = ab.squaredNorm();
        const double t = len2 > 0.0 ? std::clamp(-a.dot(ab) / len2, 0.0, 1.0) : 0.0;
        best = std::min(best, (a + t * ab).norm());
        const double c = a.x() * b.y() - a.y() * b.x(); // the origin's side of the edge
        turn += c;
        positive = positive || c > 1e-12;
        negative = negative || c < -1e-12;
    }
    // The axis passes through the polygon: every edge has it on the same side.
    if (poly.size() >= 3 && std::abs(turn) > 1e-12 && !(positive && negative)) {
        return 0.0;
    }
    return best;
}

} // namespace

geom2d::Regions revolvedEnvelope(const geom3d::Mesh& mesh, double step, double grow) {
    if (!(step > 0.0)) {
        throw Error(ErrorCode::InvalidArgument, "the envelope's slice step must be positive");
    }
    if (mesh.isEmpty()) {
        return {};
    }
    const geom3d::Aabb3 b = mesh.bounds();
    const double z0 = b.min.z();
    const auto slices =
        static_cast<std::size_t>(std::max(1.0, std::ceil((b.max.z() - z0) / step - 1e-9)));
    std::vector<double> lo(slices, std::numeric_limits<double>::infinity());
    std::vector<double> hi(slices, -std::numeric_limits<double>::infinity());
    auto sliceOf = [&](double z) {
        return std::min(slices - 1,
                        static_cast<std::size_t>(std::max(0.0, std::floor((z - z0) / step))));
    };
    for (const auto& t : mesh.triangles) {
        const std::vector<Vec3> tri{mesh.vertices[t[0]], mesh.vertices[t[1]], mesh.vertices[t[2]]};
        const double tz0 = std::min({tri[0].z(), tri[1].z(), tri[2].z()});
        const double tz1 = std::max({tri[0].z(), tri[1].z(), tri[2].z()});
        for (std::size_t i = sliceOf(tz0), last = sliceOf(tz1); i <= last; ++i) {
            const double s0 = z0 + static_cast<double>(i) * step;
            const std::vector<Vec3> piece = clipZ(clipZ(tri, s0, 1.0), s0 + step, -1.0);
            if (piece.empty()) {
                continue;
            }
            for (const Vec3& p : piece) {
                hi[i] = std::max(hi[i], p.head<2>().norm());
            }
            lo[i] = std::min(lo[i], nearestRadius(piece));
        }
    }
    geom2d::Regions rings;
    for (std::size_t i = 0; i < slices; ++i) {
        if (hi[i] >= lo[i]) {
            const double s0 = z0 + static_cast<double>(i) * step;
            const double s1 = std::min(s0 + step, b.max.z());
            rings.push_back(
                {geom2d::Polyline::rectangle(Vec2(s0 - grow, std::max(0.0, lo[i] - grow)),
                                             Vec2(std::max(s1, s0 + 1e-6) + grow, hi[i] + grow)),
                 {}});
        }
    }
    return geom2d::unite(rings, {}, {1e-4});
}

std::vector<FixtureCollision> checkTurningFixtures(const toolpath::Toolpath& path,
                                                   const geom2d::Polyline& insert,
                                                   const std::vector<LatheBody>& bodies,
                                                   std::size_t maxCollisions, CancelToken* cancel,
                                                   bool bothSides) {
    using toolpath::MoveKind;
    std::vector<FixtureCollision> out;
    if (bodies.empty() || path.moves.empty()) {
        return out;
    }
    const std::vector<Vec2> shape = insert.discretize(1e-2);
    if (shape.size() < 3) {
        throw Error(ErrorCode::InvalidArgument, "the insert outline is empty");
    }
    std::vector<geom2d::Aabb2> boxes;
    for (const LatheBody& body : bodies) {
        geom2d::Aabb2 box;
        for (const auto& r : body.area) {
            box.extend(r.bounds());
        }
        boxes.push_back(box);
    }
    // The collision still growing per body (index into `out`).
    std::vector<std::optional<std::size_t>> open(bodies.size());
    auto lathePoint = [](const Vec3& p) {
        return Vec2(p.z(), p.x());
    };
    auto sweep = [&](std::size_t k, const Vec2& a, const Vec2& c) {
        std::vector<Vec2> pts;
        pts.reserve(2 * shape.size());
        for (const Vec2& s : shape) {
            pts.push_back(a + s);
            pts.push_back(c + s);
        }
        const auto hull = convexHull(std::move(pts));
        if (hull.size() < 3) {
            return;
        }
        geom2d::Aabb2 hb;
        for (const Vec2& p : hull) {
            hb.extend(p);
        }
        // The area swept, and with `bothSides` what of it lies across the axis, in the half plane.
        geom2d::Regions swept{{geom2d::Polyline::fromPoints(hull, true), {}}};
        if (bothSides && hb.min.y() < 0.0) {
            swept.push_back(acrossAxis(hull));
            hb.extend(Vec2(hb.min.x(), -hb.min.y()));
        }
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            if (boxes[i].isEmpty() || hb.max.x() < boxes[i].min.x() ||
                hb.min.x() > boxes[i].max.x() || hb.max.y() < boxes[i].min.y() ||
                hb.min.y() > boxes[i].max.y()) {
                continue;
            }
            geom2d::Regions overlap;
            double area = 0.0;
            for (const geom2d::Region& s : swept) {
                for (auto& r : geom2d::intersect({s}, bodies[i].area, {1e-4})) {
                    area += r.area();
                    overlap.push_back(std::move(r));
                }
            }
            if (area < 1e-6) {
                continue;
            }
            if (open[i] && out[*open[i]].lastMove + 1 >= k) {
                out[*open[i]].lastMove = k;
                continue;
            }
            if (out.size() >= maxCollisions) {
                continue;
            }
            const Vec2 q = overlap.front().outer.vertices().front().p;
            out.push_back({k, k, bodies[i].name, "insert", Vec3(q.y(), 0.0, q.x()), 0.0});
            open[i] = out.size() - 1;
        }
    };
    Vec3 pos = toolPositionAt(path, 0);
    for (std::size_t k = 0; k < path.moves.size(); ++k) {
        if (cancel && (k & 255u) == 0) {
            cancel->throwIfCancelled();
        }
        const toolpath::Move& m = path.moves[k];
        if (m.kind == MoveKind::Dwell || m.kind == MoveKind::Cycle) {
            continue;
        }
        if (m.isArc()) {
            const auto pts = toolpath::sampleArc(pos, m, 1e-2);
            for (std::size_t q = 0; q + 1 < pts.size(); ++q) {
                sweep(k, lathePoint(pts[q]), lathePoint(pts[q + 1]));
            }
        } else {
            sweep(k, lathePoint(pos), lathePoint(m.end));
        }
        pos = m.end;
    }
    return out;
}

// -- One-way conversion to the tri-dexel stock --------------------------------------------------

namespace {

/// Fills the rays of `g` with the solid of revolution of `loops` (lathe plane u = Z, v = radius).
/// With `window` (lathe-plane bounds of what changed since the grid was filled), only the rays
/// crossing it are filled again, and only those whose material changed are written (so the
/// others keep the tiles they share with a snapshot).
void fillRevolved(DexelGrid& g, const std::vector<std::vector<Vec2>>& loops,
                  const geom2d::Aabb2* window) {
    const int axis = g.axis();
    const int nu = g.nu();
    const int nv = g.nv();
    const double h = g.spacing();
    auto store = [&](int i, int j, IntervalList&& ray) {
        if (window && g.ray(i, j) == ray) {
            return;
        }
        g.mutableRay(i, j) = std::move(ray);
    };
    constexpr double kAll = std::numeric_limits<double>::infinity();
    if (axis == 2) {
        // Rays along the spindle axis: z-intervals of the profile at radius hypot(x, y).
        const Edges edges = window ? edgesOf(loops, 1, window->min.y() - h, window->max.y() + h)
                                   : edgesOf(loops, 1, -kAll, kAll);
        for (int j = 0; j < nv; ++j) {
            for (int i = 0; i < nu; ++i) {
                const double rho = std::hypot(g.u(i), g.v(j));
                if (window && (rho < window->min.y() - h || rho > window->max.y() + h)) {
                    continue;
                }
                IntervalList ray;
                for (const auto& [a, b] : scan(edges, 1, rho)) {
                    ray.push_back({static_cast<float>(a), static_cast<float>(b)});
                }
                store(i, j, std::move(ray));
            }
        }
        return;
    }
    // Rays along X (u = Y, v = Z) or Y (u = Z, v = X): at height z the solid is a set of
    // annuli; the ray at offset c from the axis crosses each where c^2 + t^2 is inside.
    const bool zIsV = axis == 0;
    const int nz = zIsV ? nv : nu;
    int k0 = 0;
    int k1 = nz - 1;
    if (window) {
        std::tie(k0, k1) = zIsV ? g.vRange(window->min.x() - h, window->max.x() + h)
                                : g.uRange(window->min.x() - h, window->max.x() + h);
    }
    const Edges edges = window ? edgesOf(loops, 0, window->min.x() - h, window->max.x() + h)
                               : edgesOf(loops, 0, -kAll, kAll);
    std::vector<std::vector<std::pair<double, double>>> annuli(static_cast<std::size_t>(nz));
    for (int k = k0; k <= k1; ++k) {
        annuli[static_cast<std::size_t>(k)] = scan(edges, 0, zIsV ? g.v(k) : g.u(k));
    }
    for (int j = zIsV ? k0 : 0; j <= (zIsV ? k1 : nv - 1); ++j) {
        for (int i = zIsV ? 0 : k0; i <= (zIsV ? nu - 1 : k1); ++i) {
            const double c = zIsV ? g.u(i) : g.v(j);
            if (window && std::abs(c) > window->max.y() + h) {
                continue;
            }
            const auto& rings = annuli[static_cast<std::size_t>(zIsV ? j : i)];
            std::vector<std::pair<double, double>> hits;
            for (const auto& [r1, r2] : rings) {
                if (std::abs(c) >= r2) {
                    continue;
                }
                const double tb = std::sqrt(r2 * r2 - c * c);
                const double ta = std::abs(c) < r1 ? std::sqrt(r1 * r1 - c * c) : 0.0;
                if (ta > 0.0) {
                    hits.emplace_back(-tb, -ta);
                    hits.emplace_back(ta, tb);
                } else {
                    hits.emplace_back(-tb, tb);
                }
            }
            std::sort(hits.begin(), hits.end());
            IntervalList ray;
            for (const auto& [a, b] : hits) {
                if (!ray.empty() && a <= ray.back().hi) {
                    ray.back().hi = std::max(ray.back().hi, static_cast<float>(b));
                } else {
                    ray.push_back({static_cast<float>(a), static_cast<float>(b)});
                }
            }
            store(i, j, std::move(ray));
        }
    }
}

} // namespace

Stock Stock::fromRevolvedProfile(const geom2d::Regions& profile, double resolution, bool triDexel) {
    if (resolution <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "simulation resolution must be positive");
    }
    geom2d::Aabb2 pb;
    for (const auto& r : profile) {
        pb.extend(r.bounds());
    }
    Stock s;
    s.resolution_ = resolution;
    s.triDexel_ = triDexel;
    if (pb.isEmpty()) {
        return s;
    }
    const double R = std::max(pb.max.y(), 0.0);
    s.bounds_.extend(Vec3(-R, -R, pb.min.x()));
    s.bounds_.extend(Vec3(R, R, pb.max.x()));
    s.bounds_.min -= Vec3::Constant(resolution);
    s.bounds_.max += Vec3::Constant(resolution);
    const Vec3 size = s.bounds_.size();
    const auto loops = loopsOf(profile, 1e-3);

    for (int axis = 0; axis < 3; ++axis) {
        if (!triDexel && axis != 2) {
            continue;
        }
        const int ua = (axis + 1) % 3;
        const int va = (axis + 2) % 3;
        const int nu = std::max(1, static_cast<int>(std::ceil(size[ua] / resolution)));
        const int nv = std::max(1, static_cast<int>(std::ceil(size[va] / resolution)));
        DexelGrid g(axis, s.bounds_.min[ua], s.bounds_.min[va], resolution, nu, nv);
        fillRevolved(g, loops, nullptr);
        s.grids_[static_cast<std::size_t>(axis)] = std::move(g);
    }
    return s;
}

namespace {

/// A hole off a grid's rays, frustum by frustum along its side, exactly: a mesh's seams lose rays.
void cutHole(DexelGrid& g, const LatheHoleCut& hole) {
    double maxRadius = 0.0;
    for (const Vec2& q : hole.profile) {
        maxRadius = std::max(maxRadius, q.y());
    }
    if (maxRadius <= 0.0 || hole.length <= 0.0) {
        return;
    }
    // (distance from the tip, radius) up to where it cut.
    std::vector<Vec2> side = hole.profile;
    if (side.back().x() < hole.length) {
        side.emplace_back(hole.length, side.back().y());
    }
    const int ua = g.uAxis();
    const int va = g.vAxis();
    const Vec3 end = hole.tip + hole.length * hole.axis;
    const auto [i0, i1] = g.uRange(std::min(hole.tip[ua], end[ua]) - maxRadius,
                                   std::max(hole.tip[ua], end[ua]) + maxRadius);
    const auto [j0, j1] = g.vRange(std::min(hole.tip[va], end[va]) - maxRadius,
                                   std::max(hole.tip[va], end[va]) + maxRadius);
    Vec3 along = Vec3::Zero();
    along[g.axis()] = 1.0;
    const double da = along.dot(hole.axis);
    const Vec3 q1 = along - da * hole.axis;
    for (int j = j0; j <= j1; ++j) {
        for (int i = i0; i <= i1; ++i) {
            if (g.ray(i, j).empty()) {
                continue;
            }
            Vec3 p = Vec3::Zero();
            p[ua] = g.u(i);
            p[va] = g.v(j);
            const Vec3 w = p - hole.tip;
            const double wa = w.dot(hole.axis);
            const Vec3 q0 = w - wa * hole.axis;
            for (std::size_t k = 0; k + 1 < side.size(); ++k) {
                const double a0 = side[k].x();
                const double a1 = std::min(side[k + 1].x(), hole.length);
                if (a1 <= a0) {
                    continue;
                }
                double tA = -1e7;
                double tB = 1e7;
                if (std::abs(da) > 1e-12) {
                    tA = (a0 - wa) / da;
                    tB = (a1 - wa) / da;
                    if (tA > tB) {
                        std::swap(tA, tB);
                    }
                } else if (wa < a0 || wa > a1) {
                    continue;
                }
                // Inside where |q0 + t q1|^2 <= (e + f t)^2, the radius at t being e + f t.
                const double slope =
                    (side[k + 1].y() - side[k].y()) / (side[k + 1].x() - side[k].x());
                const double e = side[k].y() + slope * (wa - a0);
                const double f = slope * da;
                const double A = q1.dot(q1) - f * f;
                const double B = 2.0 * (q0.dot(q1) - e * f);
                const double C = q0.dot(q0) - e * e;
                std::vector<double> cuts{tA};
                if (std::abs(A) > 1e-14) {
                    const double disc = B * B - 4.0 * A * C;
                    if (disc > 0.0) {
                        const double root = std::sqrt(disc);
                        for (const double t : {(-B - root) / (2.0 * A), (-B + root) / (2.0 * A)}) {
                            if (t > tA && t < tB) {
                                cuts.push_back(t);
                            }
                        }
                    }
                } else if (std::abs(B) > 1e-14) {
                    const double t = -C / B;
                    if (t > tA && t < tB) {
                        cuts.push_back(t);
                    }
                }
                std::sort(cuts.begin(), cuts.end());
                cuts.push_back(tB);
                for (std::size_t c = 0; c + 1 < cuts.size(); ++c) {
                    const double mid = 0.5 * (cuts[c] + cuts[c + 1]);
                    if ((q0 + mid * q1).squaredNorm() <= (e + f * mid) * (e + f * mid) &&
                        e + f * mid >= 0.0) {
                        IntervalList& ray = g.mutableRay(i, j);
                        subtract(ray, cuts[c], cuts[c + 1]);
                    }
                }
            }
        }
    }
}

/// What a stock's cuts take off dexels. A thread by its mesh, which has no seam to lose rays on.
void applyLatheCuts(Stock& stock, const LatheStock& lathe) {
    for (int axis = 0; axis < 3; ++axis) {
        if (stock.triDexel() || axis == 2) {
            for (const LatheHoleCut& hole : lathe.holes()) {
                cutHole(stock.grid(axis), hole);
            }
        }
    }
    if (lathe.threads().empty()) {
        return;
    }
    LatheStock threads(lathe.profile());
    for (const LatheThreadCut& thread : lathe.threads()) {
        threads.keepThread(thread);
    }
    const double resolution = stock.resolution();
    const DisplayMesh shown = latheStockMesh(
        threads, std::min(0.005, 0.1 * resolution), [](const Vec2&, const Vec2&) { return Rgb{}; },
        [](bool, int) { return Rgb{}; });
    geom3d::Mesh mesh;
    mesh.vertices.reserve(shown.positions.size());
    for (const auto& p : shown.positions) {
        mesh.vertices.emplace_back(p[0], p[1], p[2]);
    }
    mesh.triangles.reserve(shown.triangleCount());
    for (std::size_t k = 0; k + 2 < shown.indices.size(); k += 3) {
        mesh.triangles.push_back({shown.indices[k], shown.indices[k + 1], shown.indices[k + 2]});
    }
    const Stock aligned =
        resampleStock(Stock::fromMesh(mesh, mesh.bounds(), resolution, stock.triDexel()),
                      Transform3::Identity(), stock);
    for (int axis = 0; axis < 3; ++axis) {
        if (!stock.triDexel() && axis != 2) {
            continue;
        }
        DexelGrid& grid = stock.grid(axis);
        for (int j = 0; j < grid.nv(); ++j) {
            for (int i = 0; i < grid.nu(); ++i) {
                if (!grid.ray(i, j).empty() && grid.ray(i, j) != aligned.grid(axis).ray(i, j)) {
                    grid.mutableRay(i, j) = intersect(grid.ray(i, j), aligned.grid(axis).ray(i, j));
                }
            }
        }
    }
}

} // namespace

Stock latheStockDexels(const LatheStock& stock, double resolution, bool triDexel) {
    Stock out = Stock::fromRevolvedProfile(stock.profile(), resolution, triDexel);
    if (stock.hasCuts()) {
        applyLatheCuts(out, stock);
    }
    return out;
}

void clipToLathe(Stock& stock, const LatheStock& lathe) {
    const auto loops = loopsOf(lathe.profile(), 1e-3);
    for (int axis = 0; axis < 3; ++axis) {
        if (!stock.triDexel() && axis != 2) {
            continue;
        }
        DexelGrid& grid = stock.grid(axis);
        DexelGrid shape(axis, grid.u0(), grid.v0(), grid.spacing(), grid.nu(), grid.nv());
        fillRevolved(shape, loops, nullptr);
        for (int j = 0; j < grid.nv(); ++j) {
            for (int i = 0; i < grid.nu(); ++i) {
                const IntervalList& ray = grid.ray(i, j);
                if (ray.empty() || ray == shape.ray(i, j)) {
                    continue;
                }
                IntervalList left = intersect(ray, shape.ray(i, j));
                if (left != ray) {
                    grid.mutableRay(i, j) = std::move(left);
                }
            }
        }
    }
    if (lathe.hasCuts()) {
        applyLatheCuts(stock, lathe);
    }
}

} // namespace ocam::sim
