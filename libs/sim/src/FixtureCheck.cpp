// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/sim/FixtureCheck.hpp"

#include <ocam/geom3d/Bvh.hpp>
#include <ocam/geom3d/Convex.hpp>

#include <algorithm>
#include <cmath>

namespace ocam::sim {
namespace {

struct ToolSolid {
    std::string part;
    geom3d::Convex shape; ///< Tool frame: tip at the origin, axis +Z.
};

/// Points on the circle of radius r at height h (inscribed polygon: never outside the solid, so
/// a tool kept exactly clear is not reported).
void ring(std::vector<Vec3>& out, double r, double h) {
    constexpr int kSides = 32;
    if (r <= 1e-12) {
        out.emplace_back(0.0, 0.0, h);
        return;
    }
    for (int k = 0; k < kSides; ++k) {
        const double a = 2.0 * kPi * k / kSides;
        out.emplace_back(r * std::cos(a), r * std::sin(a), h);
    }
}

/// A convex slice of the tool as a point hull.
geom3d::Convex pieceShape(const SolidPiece& p) {
    std::vector<Vec3> pts;
    if (p.sphere > 0.0) {
        // A spherical zone: rings at heights through it, and its widest height when inside.
        constexpr int kRings = 8;
        std::vector<double> heights;
        for (int i = 0; i <= kRings; ++i) {
            heights.push_back(p.h0 + (p.h1 - p.h0) * i / kRings);
        }
        if (p.centre > p.h0 && p.centre < p.h1) {
            heights.push_back(p.centre);
        }
        for (const double h : heights) {
            const double d = h - p.centre;
            ring(pts, std::sqrt(std::max(0.0, p.sphere * p.sphere - d * d)), h);
        }
    } else {
        ring(pts, p.r0, p.h0);
        ring(pts, p.r1, p.h1);
    }
    return geom3d::Convex::points(std::move(pts));
}

/// Holder bands of the same radius one above the other, as one band (fewer solids to test).
std::vector<SolidPiece> mergeBands(const std::vector<SolidPiece>& pieces) {
    std::vector<SolidPiece> out;
    for (const SolidPiece& p : pieces) {
        if (!out.empty() && out.back().sphere <= 0.0 && p.sphere <= 0.0 &&
            std::abs(out.back().r1 - p.r0) < 1e-9 &&
            std::abs(out.back().r0 - out.back().r1) < 1e-9 && std::abs(p.r0 - p.r1) < 1e-9 &&
            std::abs(out.back().h1 - p.h0) < 1e-9) {
            out.back().h1 = p.h1;
        } else {
            out.push_back(p);
        }
    }
    return out;
}

} // namespace

std::vector<FixtureCollision>
checkFixtures(const toolpath::Toolpath& path, const std::vector<FixtureBody>& bodies,
              const ToolShape& tool, const FixtureCheckOptions& options, CancelToken* cancel) {
    std::vector<FixtureCollision> out;
    if (path.moves.empty()) {
        return out;
    }
    struct Body {
        const std::string* name;
        geom3d::TriangleBvh bvh;
    };
    std::vector<Body> obstacles;
    geom3d::Aabb3 all;
    for (const FixtureBody& b : bodies) {
        if (!b.mesh.isEmpty()) {
            obstacles.push_back({&b.name, geom3d::TriangleBvh(b.mesh)});
            all.extend(obstacles.back().bvh.bounds());
        }
    }
    if (obstacles.empty()) {
        return out;
    }

    std::vector<ToolSolid> solids;
    for (const SolidPiece& p : tool.cutterPieces()) {
        solids.push_back({"tool", pieceShape(p)});
    }
    for (const SolidPiece& p : tool.shankPieces()) {
        solids.push_back({"tool shank", pieceShape(p)});
    }
    for (const SolidPiece& p : mergeBands(tool.holderPieces())) {
        solids.push_back({"tool holder", pieceShape(p)});
    }
    // The whole assembly's reach from the tip, for a quick rejection of far poses.
    double reach = 0.0;
    for (const ToolSolid& s : solids) {
        const geom3d::Aabb3 b = s.shape.bounds();
        reach = std::max({reach, b.min.norm(), b.max.norm()});
    }
    const double clearance = options.clearance;
    all.min -= Vec3::Constant(reach + clearance);
    all.max += Vec3::Constant(reach + clearance);

    auto record = [&](std::size_t move, const std::string& body, const std::string& part,
                      const geom3d::ConvexDistance& d) {
        for (auto it = out.rbegin(); it != out.rend() && it->lastMove + 1 >= move; ++it) {
            if (it->body == body && it->toolPart == part) {
                it->lastMove = std::max(it->lastMove, move);
                it->distance = std::min(it->distance, d.distance);
                return;
            }
        }
        if (out.size() < options.maxCollisions) {
            out.push_back({move, move, body, part, d.pointA, d.distance});
        }
    };

    auto checkPose = [&](std::size_t move, const Vec3& tip, const Vec3& axis) {
        if (!all.contains(tip)) {
            return;
        }
        Transform3 xf = Transform3::Identity();
        xf.linear() =
            Eigen::Quaterniond::FromTwoVectors(Vec3::UnitZ(), axis.normalized()).toRotationMatrix();
        xf.translation() = tip;
        for (const ToolSolid& s : solids) {
            const geom3d::Convex shape = s.shape.transformed(xf);
            geom3d::Aabb3 box = shape.bounds();
            box.min -= Vec3::Constant(clearance);
            box.max += Vec3::Constant(clearance);
            for (const Body& b : obstacles) {
                b.bvh.query(box, [&](std::size_t t) {
                    const auto [p, q, r] = b.bvh.triangle(t);
                    const geom3d::ConvexDistance d =
                        geom3d::distance(geom3d::Convex::triangle(p, q, r), shape);
                    if (d.intersecting || d.distance < clearance) {
                        record(move, *b.name, s.part, d);
                        return false;
                    }
                    return true;
                });
            }
        }
    };

    // Poses every `step` of tip travel and at both ends of every move.
    Vec3 p = path.start;
    Vec3 u = toolpath::axisBefore(path, 0);
    checkPose(0, p, u);
    for (std::size_t k = 0; k < path.moves.size(); ++k) {
        if (cancel && (k & 255u) == 0) {
            cancel->throwIfCancelled();
        }
        const toolpath::Move& m = path.moves[k];
        if (m.kind == toolpath::MoveKind::Dwell) {
            continue;
        }
        double travel = (m.end - p).norm();
        if (m.isArc()) {
            const std::vector<Vec3> pts = toolpath::sampleArc(p, m, 0.01);
            travel = 0.0;
            for (std::size_t i = 1; i < pts.size(); ++i) {
                travel += (pts[i] - pts[i - 1]).norm();
            }
        }
        const int steps =
            std::max(1, static_cast<int>(std::min(1e5, std::ceil(travel / options.step))));
        for (int s = 1; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            if (const auto part = toolpath::partialMove(p, u, m, t)) {
                checkPose(k, part->end, part->axis);
            }
        }
        p = m.end;
        u = m.axis;
    }
    return out;
}

} // namespace ocam::sim
