// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// 3D edge breaking with a chamfer mill or a ball end mill, the tool axis held at +Z. See
// docs/design-log/0080.

#include "ocam/strategies/EdgeBreak.hpp"

#include "Internal.hpp"

#include <ocam/foundation/Units.hpp>
#include <ocam/geom3d/Bvh.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ocam::strategies {
namespace {

using toolpath::FeedClass;

enum class Shape { Cone, Ball };

struct ToolShape {
    Shape shape = Shape::Cone;
    double radius = 0.0;    ///< Largest radius.
    double tipRadius = 0.0; ///< Cone: flat tip.
    double half = 0.0;      ///< Cone: half the point angle (from the axis) [rad].
};

ToolShape shapeOf(const tooling::Tool& tool) {
    using tooling::ToolType;
    const double r = detail::requireMillingTool(tool);
    const ToolType t = tool.type;
    if (t == ToolType::BallEndMill) {
        return {Shape::Ball, r, 0.0, 0.0};
    }
    if ((t == ToolType::Chamfer || t == ToolType::Engraver || t == ToolType::SpotDrill) &&
        tool.mill.tipAngle > 0.0 && tool.mill.tipAngle < kPi) {
        const double rt =
            t == ToolType::SpotDrill ? 0.0 : std::max(0.0, tool.mill.tipDiameter / 2.0);
        return {Shape::Cone, r, rt, tool.mill.tipAngle / 2.0};
    }
    throw Error(ErrorCode::InvalidArgument,
                "3D edge breaking needs a chamfer mill, engraver or spot drill (with a point "
                "angle), or a ball end mill");
}

std::string where(const Vec3& p) {
    return fmt::format("({:.3f}, {:.3f}, {:.3f})", p.x(), p.y(), p.z());
}

std::optional<BreakPose> fail(std::string* why, std::string reason) {
    if (why) {
        *why = std::move(reason);
    }
    return std::nullopt;
}

std::optional<BreakPose> poseFor(const ToolShape& s, const EdgeSample& e, double w,
                                 double tipOffset, std::string* why) {
    const Vec3 t = e.tangent.normalized();
    const auto across = [&](const Vec3& v) {
        return (v - v.dot(t) * t).normalized();
    };
    const Vec3 na = across(e.normal[0]);
    const Vec3 nb = across(e.normal[1]);
    const Vec3 ua = across(e.inFace[0]);
    const Vec3 ub = across(e.inFace[1]);
    // Each face lies behind the other's plane on a convex edge; an inside corner has nothing
    // to break.
    if (ua.dot(nb) > -1e-6 || ub.dot(na) > -1e-6) {
        return fail(why, "an inside corner");
    }
    Vec3 b = na + nb;
    if (b.norm() < 1e-9) {
        return fail(why, "the faces fold back on each other");
    }
    b.normalize();
    const Vec3 up = Vec3::UnitZ();
    const Vec3 P = e.point;
    BreakPose pose;
    if (s.shape == Shape::Ball) {
        // The ball's centre on the bisector, its circle across the edge meeting both faces the
        // same distance from the edge, the width apart.
        const double sw = w / (ua - ub).norm();
        const double kappa = 0.5 * (ua.dot(b) + ub.dot(b));
        const double disc = s.radius * s.radius - sw * sw * (1.0 - kappa * kappa);
        if (disc < 0.0) {
            return fail(why, "the break is wider than the ball can make");
        }
        const Vec3 C = P + (sw * kappa + std::sqrt(disc)) * b;
        const Vec3 ea = P + sw * ua;
        const Vec3 eb = P + sw * ub;
        if (std::max(ea.z(), eb.z()) > C.z() + 1e-9) {
            return fail(why, "a face turns up past the ball's equator (an undercut)");
        }
        pose.tip = C - s.radius * up;
        pose.faceWidth = {sw, sw};
        pose.planeNormal = b;
        pose.middle = 0.5 * (ea + eb);
        const Vec3 bh(b.x(), b.y(), 0.0);
        pose.out = bh.norm() > 1e-6 ? Vec3(bh.normalized()) : Vec3(Vec3::Zero());
        return pose;
    }
    // A cone with its axis vertical touches planes whose normal rises at its half angle: the
    // break's plane is the one along the edge nearest the faces' bisector.
    const Vec3 c = t.cross(b).normalized();
    const double A = b.z();
    const double B = c.z();
    const double rho = std::hypot(A, B);
    const double sinHalf = std::sin(s.half);
    if (rho < sinHalf - 1e-12) {
        return fail(why, fmt::format("the edge is too steep for the tool's cone ({:.1f}° from "
                                     "horizontal)",
                                     radToDeg(std::asin(std::min(1.0, std::abs(t.z()))))));
    }
    const double psi = std::atan2(B, A);
    const double dphi = std::acos(std::clamp(sinHalf / rho, -1.0, 1.0));
    const auto wrap = [](double a) {
        return std::remainder(a, 2.0 * kPi);
    };
    const double p1 = wrap(psi + dphi);
    const double p2 = wrap(psi - dphi);
    const double phi = std::abs(p1) <= std::abs(p2) ? p1 : p2;
    const Vec3 m = (std::cos(phi) * b + std::sin(phi) * c).normalized();
    const double ma = m.dot(ua);
    const double mb = m.dot(ub);
    if (ma > -1e-6 || mb > -1e-6) {
        return fail(why, "the cone cannot reach both faces with its axis vertical");
    }
    const double delta = w / (ua / ma - ub / mb).norm();
    const double sa = -delta / ma;
    const double sb = -delta / mb;
    const Vec3 ea = P + sa * ua;
    const Vec3 eb = P + sb * ub;
    const Vec3 M = 0.5 * (ea + eb);
    const Vec3 q = (eb - ea).normalized();
    const Vec3 mh(m.x(), m.y(), 0.0);
    const Vec3 a = -mh.normalized();                             // from the axis to the contact
    const Vec3 d = std::sin(s.half) * a + std::cos(s.half) * up; // the touching generator
    const double dq = std::abs(d.dot(q));
    if (dq < 0.05) {
        return fail(why, "the edge runs along the cone's flank");
    }
    const double half = w / (2.0 * dq);
    const double hM = tipOffset + half * std::cos(s.half);
    const double hUp = hM + half * std::cos(s.half);
    const double tanHalf = std::tan(s.half);
    if (s.tipRadius + hUp * tanHalf > s.radius + 1e-9) {
        return fail(why, "the break is too wide for the tool's cone");
    }
    const double rM = s.tipRadius + hM * tanHalf;
    pose.tip = Vec3(M.x() - rM * a.x(), M.y() - rM * a.y(), M.z() - hM);
    pose.faceWidth = {sa, sb};
    pose.planeNormal = m;
    pose.middle = M;
    pose.out = -a;
    return pose;
}

/// The tool at a pose against the part, everything but the break (DL-0083). The edge's own two
/// faces lose a band along the edge as wide as the break meets them, curved with the edge: their
/// points outside it must stay out of the tool (the drop-cutter's point contact). Other faces
/// lose what lies in the break's prism (between the two faces' planes and the break's plane, as
/// at a corner where the edge ends) and are checked by drop-cutter against the rest.
class GougeCheck {
public:
    GougeCheck(const std::vector<geom3d::Mesh>& faces, const tooling::Tool& tool, double radius)
        : cutter_(tool), radius_(radius) {
        geom3d::Mesh all;
        for (std::size_t f = 0; f < faces.size(); ++f) {
            all.append(faces[f]);
            faceOf_.insert(faceOf_.end(), faces[f].triangles.size(), static_cast<int>(f));
        }
        part_ = all;
        bvh_ = geom3d::TriangleBvh(std::move(all));
    }

    const geom3d::Mesh& part() const { return part_; }
    const Cutter& cutter() const { return cutter_; }

    /// True when the tool at `tip` reaches nothing beyond the break of `edge` at its sample
    /// `index` by more than `tol`.
    bool clear(const Vec3& tip, const BreakEdge& edge, std::size_t index, const BreakPose& pose,
               double tol) const {
        if (bvh_.isEmpty()) {
            return true;
        }
        const EdgeSample& sample = edge.samples[index];
        const double reach = radius_ + 0.05;
        geom3d::Aabb3 box;
        box.extend(Vec3(tip.x() - reach, tip.y() - reach, tip.z() - 1e-3));
        box.extend(Vec3(tip.x() + reach, tip.y() + reach, bvh_.bounds().max.z() + 1.0));
        // The edge near the tool, as segments.
        std::vector<std::pair<Vec3, Vec3>> near;
        const std::size_t n = edge.samples.size();
        const std::size_t segments = edge.closed ? n : n - 1;
        const double span = reach + std::max(pose.faceWidth[0], pose.faceWidth[1]) + 0.1;
        for (std::size_t k = 0; k < segments; ++k) {
            const Vec3& a = edge.samples[k].point;
            const Vec3& b = edge.samples[(k + 1) % n].point;
            const Vec2 t = tip.head<2>();
            const double d = std::min((a.head<2>() - t).norm(), (b.head<2>() - t).norm());
            if (d <= span + (b - a).norm()) {
                near.emplace_back(a, b);
            }
        }
        const auto distanceToEdge = [&](const Vec3& x) {
            double best = std::numeric_limits<double>::infinity();
            for (const auto& [a, b] : near) {
                const Vec3 ab = b - a;
                const double len2 = ab.squaredNorm();
                const double s = len2 > 0.0 ? std::clamp((x - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
                best = std::min(best, (a + s * ab - x).norm());
            }
            return best;
        };
        // A point of the part inside the tool beyond the tolerance.
        const auto inside = [&](const Vec3& x) {
            const double d = (x.head<2>() - tip.head<2>()).norm();
            return d < cutter_.radius() && x.z() > tip.z() + cutter_.lo(d) + tol;
        };
        const double step =
            std::clamp(0.25 * std::min(pose.faceWidth[0], pose.faceWidth[1]), 0.02, 0.1);

        const Vec3 t = sample.tangent.normalized();
        const auto across = [&](const Vec3& v) {
            return (v - v.dot(t) * t).normalized();
        };
        struct Half {
            Vec3 g;
            double o; // keep g.X >= o
        };
        constexpr double eps = 1e-3;
        const Vec3 na = across(sample.normal[0]);
        const Vec3 nb = across(sample.normal[1]);
        const std::array<Half, 3> keep{Half{na, na.dot(sample.point) + eps},
                                       Half{nb, nb.dot(sample.point) + eps},
                                       Half{-pose.planeNormal, -pose.planeNormal.dot(pose.middle)}};
        const auto clipTo = [](std::vector<Vec3>& poly, const Vec3& g, double o) {
            std::vector<Vec3> out;
            for (std::size_t i = 0; i < poly.size(); ++i) {
                const Vec3& p = poly[i];
                const Vec3& q = poly[(i + 1) % poly.size()];
                const double dp = g.dot(p) - o;
                const double dq = g.dot(q) - o;
                if (dp >= 0.0) {
                    out.push_back(p);
                }
                if ((dp >= 0.0) != (dq >= 0.0)) {
                    out.push_back(p + (q - p) * (dp / (dp - dq)));
                }
            }
            poly = std::move(out);
        };

        bool gouge = false;
        geom3d::Mesh local;
        bvh_.query(box, [&](std::size_t tri) {
            const auto corners = bvh_.triangle(tri);
            const int face = faceOf_[tri];
            if (face == edge.faces[0] || face == edge.faces[1]) {
                // Points over the part of the triangle under the tool, outside the band.
                std::vector<Vec3> poly(corners.begin(), corners.end());
                clipTo(poly, Vec3::UnitX(), tip.x() - reach);
                clipTo(poly, -Vec3::UnitX(), -(tip.x() + reach));
                clipTo(poly, Vec3::UnitY(), tip.y() - reach);
                clipTo(poly, -Vec3::UnitY(), -(tip.y() + reach));
                const double band = pose.faceWidth[face == edge.faces[0] ? 0 : 1] + eps;
                for (std::size_t i = 1; i + 1 < poly.size() && !gouge; ++i) {
                    const Vec3& p0 = poly[0];
                    const Vec3& p1 = poly[i];
                    const Vec3& p2 = poly[i + 1];
                    const double longest =
                        std::max({(p1 - p0).norm(), (p2 - p1).norm(), (p0 - p2).norm()});
                    const int m = std::max(1, static_cast<int>(std::ceil(longest / step)));
                    for (int u = 0; u <= m && !gouge; ++u) {
                        for (int v = 0; u + v <= m && !gouge; ++v) {
                            const Vec3 x = p0 + (p1 - p0) * (static_cast<double>(u) / m) +
                                           (p2 - p0) * (static_cast<double>(v) / m);
                            gouge = inside(x) && distanceToEdge(x) > band;
                        }
                    }
                }
                return !gouge;
            }
            // Disjoint pieces outside the prism: in the first half-space, or out of the
            // earlier ones and in the next.
            for (std::size_t k = 0; k < keep.size(); ++k) {
                std::vector<Vec3> poly(corners.begin(), corners.end());
                for (std::size_t j = 0; j < k && !poly.empty(); ++j) {
                    clipTo(poly, -keep[j].g, -keep[j].o);
                }
                if (!poly.empty()) {
                    clipTo(poly, keep[k].g, keep[k].o);
                }
                for (std::size_t i = 1; i + 1 < poly.size(); ++i) {
                    const auto base = static_cast<std::uint32_t>(local.vertices.size());
                    local.vertices.push_back(poly[0]);
                    local.vertices.push_back(poly[i]);
                    local.vertices.push_back(poly[i + 1]);
                    local.triangles.push_back({base, base + 1, base + 2});
                }
            }
            return true;
        });
        if (gouge) {
            return false;
        }
        if (local.isEmpty()) {
            return true;
        }
        const DropCutter dc(local, cutter_);
        return dc.drop(tip.head<2>(), -1e9) <= tip.z() + tol;
    }

private:
    Cutter cutter_;
    double radius_;
    geom3d::Mesh part_;
    std::vector<int> faceOf_;
    geom3d::TriangleBvh bvh_;
};

} // namespace

std::optional<BreakPose> breakPose(const tooling::Tool& tool, const EdgeSample& sample,
                                   double width, double tipOffset, std::string* why) {
    if (width <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the break's width must be positive");
    }
    return poseFor(shapeOf(tool), sample, width, tipOffset, why);
}

toolpath::Toolpath edgeBreak(const std::vector<BreakEdge>& edges,
                             const std::vector<geom3d::Mesh>& faces, const EdgeBreakParams& p,
                             const Context& ctx) {
    const ToolShape shape = shapeOf(ctx.tool);
    if (p.width <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the break's width must be positive");
    }
    const double lead = p.leadLength.value_or(kChamferLeadShare * 2.0 * shape.radius);
    Vec2 startXY = Vec2::Zero();
    for (const BreakEdge& e : edges) {
        if (!e.samples.empty()) {
            startXY = e.samples.front().point.head<2>();
            break;
        }
    }
    toolpath::Toolpath tp = beginToolpath(ctx, "edgebreak3d", startXY);
    if (edges.empty()) {
        tp.warnings.push_back("no edges to break");
        return tp;
    }
    const GougeCheck check(faces, ctx.tool, shape.radius);
    const DropCutter whole(check.part(), check.cutter());
    toolpath::PathBuilder b(tp);
    std::vector<std::string> skipped;
    bool first = true;
    for (std::size_t k = 0; k < edges.size(); ++k) {
        const BreakEdge& e = edges[k];
        ctx.cancel.throwIfCancelled();
        ctx.progress.report(static_cast<double>(k) / static_cast<double>(edges.size()),
                            "Breaking edges");
        if (!e.unusable.empty()) {
            skipped.push_back(fmt::format("{} ({})", e.name, e.unusable));
            continue;
        }
        if (e.samples.size() < 2) {
            skipped.push_back(fmt::format("{} (too short)", e.name));
            continue;
        }
        std::vector<BreakPose> poses;
        std::string why;
        for (std::size_t i = 0; i < e.samples.size(); ++i) {
            const EdgeSample& s = e.samples[i];
            const auto pose = poseFor(shape, s, p.width, p.tipOffset, &why);
            if (!pose) {
                why += " at " + where(s.point);
                break;
            }
            if (!check.clear(pose->tip, e, i, *pose, p.gougeTolerance)) {
                why = "the tool would cut past the break at " + where(s.point);
                break;
            }
            poses.push_back(*pose);
        }
        if (poses.size() != e.samples.size()) {
            skipped.push_back(fmt::format("{} ({})", e.name, why));
            continue;
        }
        // Short horizontal leads away from the part, where the tool stays clear along them.
        const auto leadPoint = [&](std::size_t i) {
            const BreakPose& pose = poses[i];
            if (lead <= 0.0 || pose.out.norm() < 0.5) {
                return pose.tip;
            }
            for (int j = 1; j <= 4; ++j) {
                const Vec3 at = pose.tip + pose.out * (lead * j / 4.0);
                if (!check.clear(at, e, i, pose, p.gougeTolerance)) {
                    return pose.tip;
                }
            }
            return Vec3(pose.tip + pose.out * lead);
        };
        const Vec3 in = leadPoint(0);
        const Vec3 out = e.closed ? poses.front().tip : leadPoint(poses.size() - 1);
        const Vec3 off = e.closed ? leadPoint(0) : out;
        // To the lead's start: over at the retract height where nothing stands higher on the
        // way, else at the clearance height.
        const double above =
            std::max(whole.drop(in.head<2>(), in.z()), in.z()) + ctx.heights.feedClearance;
        if (first) {
            b.rapidAbove(in.head<2>(), ctx.heights.clearance, std::max(above, in.z()));
            first = false;
        } else {
            bool low = true;
            const Vec2 from = b.position().head<2>();
            const double span = (in.head<2>() - from).norm();
            const int steps = std::max(1, static_cast<int>(std::ceil(span / (0.5 * shape.radius))));
            for (int j = 0; j <= steps && low; ++j) {
                const Vec2 xy = from + (in.head<2>() - from) * (static_cast<double>(j) / steps);
                low = whole.drop(xy, -1e9) <= ctx.heights.retract - ctx.heights.feedClearance;
            }
            b.retractTo(low ? ctx.heights.retract : ctx.heights.clearance);
            b.rapid(Vec3(in.x(), in.y(), b.position().z()));
            b.rapid(Vec3(in.x(), in.y(), std::max(above, in.z())));
        }
        b.feed(in, FeedClass::Plunge, ctx.feeds.plunge);
        b.feed(poses.front().tip, FeedClass::LeadIn, ctx.feeds.lead);
        for (std::size_t i = 1; i < poses.size(); ++i) {
            b.feed(poses[i].tip, FeedClass::Cut, ctx.feeds.cut);
        }
        if (e.closed) {
            b.feed(poses.front().tip, FeedClass::Cut, ctx.feeds.cut);
        }
        b.feed(off, FeedClass::LeadOut, ctx.feeds.lead);
        b.retractTo(ctx.heights.retract);
    }
    b.retractTo(ctx.heights.clearance);
    if (!skipped.empty()) {
        std::string list;
        for (const std::string& s : skipped) {
            list += (list.empty() ? "" : "; ") + s;
        }
        tp.warnings.push_back(
            fmt::format("{} of {} edges skipped, the tool cannot break them with its axis "
                        "vertical: {}",
                        skipped.size(), edges.size(), list));
    }
    return tp;
}

namespace detail {

StrategyInfo edgeBreakStrategy() {
    const auto length = [](std::string key, std::string label, double def, double lo, double hi,
                           std::string help) {
        ParamDef d;
        d.key = std::move(key);
        d.label = std::move(label);
        d.kind = ParamKind::Length;
        d.defaultValue = def;
        d.min = lo;
        d.max = hi;
        d.help = std::move(help);
        return d;
    };
    StrategyInfo s;
    s.id = "edgebreak3d";
    s.label = "3D edge break";
    s.geometry = GeometryUse::AnyEdges;
    s.description = "Breaks the selected edges, flat or not (on sloped and curved faces too), "
                    "with a chamfer mill or ball end mill, its axis vertical: the break's width "
                    "held along each edge, edges the tool cannot reach skipped and listed.";
    s.params.push_back(length("width", "Break width", 0.3, 0.01, 20.0,
                              "Width of the break from one face to the other, held along the "
                              "edge"));
    s.params.push_back(length("tip_offset", "Tip below the break", 0.3, 0.0, 20.0,
                              "Chamfer mills: how far the tip goes below the break's lower "
                              "edge, so the flank cuts it rather than the tip"));
    ParamDef lead = length("lead_length", "Lead length", 0.0, 0.0, 100.0,
                           "Horizontal lead onto and off each edge, away from the part, where "
                           "the tool is clear along it (else it comes straight down); while not "
                           "set, a quarter of the tool diameter");
    lead.toolDefault = [](const tooling::Tool& t) {
        return kChamferLeadShare * t.mill.diameter;
    };
    s.params.push_back(lead);
    s.params.push_back(length("gouge_tolerance", "Gouge tolerance", 0.01, 0.0, 1.0,
                              "How far the tool may reach into the part beyond the break before "
                              "an edge is skipped"));
    s.params.push_back(length("point_spacing", "Point spacing", 0.5, 0.01, 10.0,
                              "Largest distance between the points along each edge"));
    return s;
}

} // namespace detail

EdgeBreakParams edgeBreakParams(const Json& values) {
    const ParamReader r(*findStrategy("edgebreak3d"), values);
    EdgeBreakParams p;
    p.width = r.number("width");
    p.tipOffset = r.number("tip_offset");
    if (r.isSet("lead_length")) {
        p.leadLength = r.number("lead_length");
    }
    p.gougeTolerance = r.number("gouge_tolerance");
    return p;
}

} // namespace ocam::strategies
