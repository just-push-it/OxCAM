// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/toolpath/PathBuilder.hpp"

#include <algorithm>
#include <cmath>

namespace ocam::toolpath {
namespace {

constexpr double kEps = 1e-9;

} // namespace

PathBuilder::PathBuilder(Toolpath& path) : path_(path), position_(path.start) {}

void PathBuilder::push(Move move) {
    if (move.kind != MoveKind::Dwell && move.kind != MoveKind::Cycle &&
        (move.end - position_).norm() < kEps) {
        return;
    }
    if (move.kind != MoveKind::Dwell) {
        position_ = move.end;
    }
    path_.moves.push_back(move);
}

void PathBuilder::rapid(const Vec3& target) {
    Move m;
    m.kind = MoveKind::Rapid;
    m.feedClass = FeedClass::Rapid;
    m.end = target;
    push(m);
}

void PathBuilder::feed(const Vec3& target, FeedClass feedClass, double feedRate) {
    Move m;
    m.kind = MoveKind::Linear;
    m.feedClass = feedClass;
    m.end = target;
    m.feedRate = feedRate;
    push(m);
}

void PathBuilder::thread(const Vec3& target, double pitch, double feedRate) {
    Move m;
    m.kind = MoveKind::Linear;
    m.feedClass = FeedClass::Cut;
    m.end = target;
    m.feedRate = feedRate;
    m.pitch = pitch;
    push(m);
}

void PathBuilder::arc(const Vec3& target, const Vec2& center, bool clockwise, FeedClass feedClass,
                      double feedRate) {
    // Degenerate arcs (radius ~ 0) become lines.
    if ((position_.head<2>() - center).norm() < kEps) {
        feed(target, feedClass, feedRate);
        return;
    }
    Move m;
    m.kind = clockwise ? MoveKind::ArcCW : MoveKind::ArcCCW;
    m.feedClass = feedClass;
    m.end = target;
    m.center = Vec3(center.x(), center.y(), position_.z());
    m.feedRate = feedRate;
    path_.moves.push_back(m); // full circles are legal: never dropped as zero-length
    position_ = target;
}

void PathBuilder::dwell(double seconds) {
    if (seconds <= 0.0) {
        return;
    }
    Move m;
    m.kind = MoveKind::Dwell;
    m.feedClass = FeedClass::Plunge;
    m.end = position_;
    m.dwell = seconds;
    push(m);
}

void PathBuilder::retractTo(double z) {
    if (position_.z() < z - kEps) {
        rapid(Vec3(position_.x(), position_.y(), z));
    }
}

void PathBuilder::rapidAbove(const Vec2& xy, double safeZ, double z) {
    if ((position_.head<2>() - xy).norm() > kEps) {
        retractTo(safeZ);
        rapid(Vec3(xy.x(), xy.y(), std::max(position_.z(), safeZ)));
    }
    rapid(Vec3(xy.x(), xy.y(), z));
}

void PathBuilder::follow(const geom2d::Polyline& path, double z, FeedClass feedClass,
                         double feedRate) {
    const auto& v = path.vertices();
    if (v.empty()) {
        return;
    }
    feed(Vec3(v[0].p.x(), v[0].p.y(), z), feedClass, feedRate);
    const std::size_t n = v.size();
    for (std::size_t i = 0; i < path.segmentCount(); ++i) {
        const geom2d::Vertex& a = v[i];
        const Vec2& b = v[(i + 1) % n].p;
        const Vec3 target(b.x(), b.y(), z);
        if (auto arcGeom = geom2d::arcFromBulge(a.p, b, a.bulge)) {
            arc(target, arcGeom->center, arcGeom->sweep < 0.0, feedClass, feedRate);
        } else {
            feed(target, feedClass, feedRate);
        }
    }
}

void PathBuilder::rampAlong(const geom2d::Polyline& path, double zEnd, double rampAngle,
                            double feedRate) {
    const auto& v = path.vertices();
    if (v.empty()) {
        return;
    }
    const double zStart = position_.z();
    const double drop = zStart - zEnd;
    const double run = drop / std::tan(std::clamp(rampAngle, degToRad(0.1), degToRad(89.0)));
    const double lap = path.length();
    if (drop <= kEps || lap <= kEps) {
        follow(path, zEnd, FeedClass::Ramp, feedRate);
        return;
    }
    feed(Vec3(v[0].p.x(), v[0].p.y(), zStart), FeedClass::Ramp, feedRate);

    // Walk whole laps, lowering z in proportion to distance travelled. Once at depth, the rest of
    // the lap is cut flat, so the builder always finishes at the first vertex.
    const std::size_t n = v.size();
    double travelled = 0.0;
    bool atDepth = false;
    int guard = 0;
    while (!atDepth && guard++ < 10000) {
        for (std::size_t i = 0; i < path.segmentCount(); ++i) {
            const geom2d::Vertex& a = v[i];
            const Vec2& b = v[(i + 1) % n].p;
            const auto g = geom2d::arcFromBulge(a.p, b, a.bulge);
            const double segLen = g ? g->radius * std::abs(g->sweep) : (b - a.p).norm();
            auto emit = [&](const Vec2& p, double z, FeedClass fc) {
                if (g) {
                    arc(Vec3(p.x(), p.y(), z), g->center, g->sweep < 0.0, fc, feedRate);
                } else {
                    feed(Vec3(p.x(), p.y(), z), fc, feedRate);
                }
            };
            if (atDepth || segLen <= kEps) {
                emit(b, atDepth ? zEnd : position_.z(), atDepth ? FeedClass::Cut : FeedClass::Ramp);
                continue;
            }
            if (travelled + segLen >= run - kEps) {
                // The ramp reaches depth inside this segment: split it there.
                const double t = std::clamp((run - travelled) / segLen, 0.0, 1.0);
                const Vec2 mid = g ? g->pointAt(t) : Vec2(a.p + (b - a.p) * t);
                emit(mid, zEnd, FeedClass::Ramp);
                emit(b, zEnd, FeedClass::Cut);
                travelled = run;
                atDepth = true;
            } else {
                travelled += segLen;
                emit(b, zStart - drop * travelled / run, FeedClass::Ramp);
            }
        }
        if (!path.isClosed()) {
            break; // open paths are ramped once, end to end
        }
    }
}

void PathBuilder::helix(const Vec2& center, double radius, double zEnd, double pitch,
                        bool clockwise, double feedRate) {
    const double zStart = position_.z();
    const double drop = zStart - zEnd;
    const int turns =
        drop > kEps ? std::max(1, static_cast<int>(std::ceil(drop / std::max(pitch, 1e-6)))) : 0;
    const Vec2 p0 = center + Vec2(radius, 0.0);
    const Vec2 p1 = center - Vec2(radius, 0.0);
    // Each turn as two half circles (keeps every arc below 360 degrees for any controller).
    for (int t = 0; t < turns; ++t) {
        const double zHalf = zStart - drop * (t + 0.5) / turns;
        const double zFull = zStart - drop * (t + 1.0) / turns;
        arc(Vec3(p1.x(), p1.y(), zHalf), center, clockwise, FeedClass::Ramp, feedRate);
        arc(Vec3(p0.x(), p0.y(), zFull), center, clockwise, FeedClass::Ramp, feedRate);
    }
    // Flat clean-up circle at depth.
    arc(Vec3(p1.x(), p1.y(), zEnd), center, clockwise, FeedClass::Cut, feedRate);
    arc(Vec3(p0.x(), p0.y(), zEnd), center, clockwise, FeedClass::Cut, feedRate);
}

void PathBuilder::cycle(const Vec2& xy, const DrillCycle& cycle) {
    auto it = std::find(path_.cycles.begin(), path_.cycles.end(), cycle);
    int index = static_cast<int>(it - path_.cycles.begin());
    if (it == path_.cycles.end()) {
        path_.cycles.push_back(cycle);
    }
    Move m;
    m.kind = MoveKind::Cycle;
    m.feedClass = FeedClass::Plunge;
    m.end = Vec3(xy.x(), xy.y(), cycle.retractZ);
    m.feedRate = cycle.feedRate;
    m.cycle = index;
    push(m);
}

void PathBuilder::probe(const Vec3& target, double feedRate, int probe, bool measure) {
    Move m;
    m.kind = MoveKind::Probe;
    m.feedClass = FeedClass::Probe;
    m.end = target;
    m.feedRate = feedRate;
    m.probe = probe;
    m.measure = measure;
    push(m);
}

} // namespace ocam::toolpath
