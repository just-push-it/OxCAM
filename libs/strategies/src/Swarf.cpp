// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Swarf (flank) milling of ruled walls. Each tool position lays the cylindrical flank of the
// tool along one ruling: the axis is the line through the ruling's end points offset by the tool
// radius along their own surface normals (the double point offset of Liu 1995). On developable
// walls the flank touches the whole ruling; on twisted ones it strays between the ends, by an
// amount measured here and limited by the user's tolerance (Bedi, Mann and Menzel 2003).
// Nothing here chooses a tilt: the rulings set it, and walls beyond the user's limits are
// refused. See docs/design-log/0080.

#include "Internal.hpp"
#include "ocam/strategies/MultiAxis.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/geom3d/Bvh.hpp>
#include <ocam/geom3d/Convex.hpp>

#include <fmt/format.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>

namespace ocam::strategies {

using toolpath::FeedClass;
using toolpath::Move;
using toolpath::MoveKind;

namespace {

constexpr double kDeg = kPi / 180.0;

Vec3 slerp(const Vec3& a, const Vec3& b, double t) {
    const double c = std::clamp(a.dot(b), -1.0, 1.0);
    const double th = std::acos(c);
    if (th < 1e-9) {
        return a;
    }
    const double s = std::sin(th);
    return ((std::sin((1.0 - t) * th) * a + std::sin(t * th) * b) / s).normalized();
}

/// A ruling in the order cut, with the wall it belongs to (null on a corner).
struct Station {
    WallRuling ruling;
    const RuledWall* wall = nullptr;
};

/// A tool position on a station.
struct Pose {
    Vec3 tip;
    Vec3 axis;
    Vec3 normal;         ///< Where leads go: off the wall, parallel to the floor (or level).
    double height = 0.0; ///< From the tip to the ruling's top, along the axis.
};

std::string at(const Vec3& p) {
    return fmt::format("({:.3f}, {:.3f}, {:.3f})", p.x(), p.y(), p.z());
}

} // namespace

toolpath::Toolpath swarf(const geom3d::Mesh& part, const std::vector<RuledWall>& walls,
                         const std::optional<SwarfFloor>& floor, const SwarfParams& p,
                         const Context& ctx) {
    const double r = detail::requireMillingTool(ctx.tool);
    const tooling::Tool& tool = ctx.tool;
    double rc = 0.0; // corner radius
    switch (tool.type) {
    case tooling::ToolType::FlatEndMill:
        rc = 0.0;
        break;
    case tooling::ToolType::BullNoseEndMill:
        rc = std::clamp(tool.mill.cornerRadius, 0.0, r);
        break;
    case tooling::ToolType::BallEndMill:
        rc = r;
        break;
    default:
        throw Error(ErrorCode::Unsupported,
                    "swarf milling cuts with the side of the tool: it needs a flat, bull-nose or "
                    "ball end mill");
    }
    if (walls.empty()) {
        throw Error(ErrorCode::InvalidArgument,
                    "select the wall faces to cut with the side of the tool");
    }
    for (const RuledWall& w : walls) {
        if (w.rulings.size() < 2) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("the wall {} is too small to follow", w.name));
        }
    }
    const int passes = std::max(1, p.depthPasses);
    const double d = r + p.stockToLeave; // tool axis from the wall
    if (d <= 0.0) {
        throw Error(ErrorCode::InvalidArgument,
                    "the stock to leave is more negative than the tool radius");
    }
    const double joinTol = std::max(0.01, 5.0 * ctx.chordTolerance);

    // -- Walls that share a ruling, in chains ---------------------------------------------------
    struct Piece {
        const RuledWall* wall;
        bool reversed;
        const WallRuling& first() const {
            return reversed ? wall->rulings.back() : wall->rulings.front();
        }
        const WallRuling& last() const {
            return reversed ? wall->rulings.front() : wall->rulings.back();
        }
    };
    const auto same = [&](const WallRuling& a, const WallRuling& b) {
        return (a.points.front() - b.points.front()).norm() < joinTol &&
               (a.points.back() - b.points.back()).norm() < joinTol;
    };
    std::vector<std::vector<Piece>> chains;
    {
        std::vector<char> used(walls.size(), 0);
        for (std::size_t i = 0; i < walls.size(); ++i) {
            if (used[i]) {
                continue;
            }
            used[i] = 1;
            std::vector<Piece> chain{{&walls[i], false}};
            const bool loop = same(walls[i].rulings.front(), walls[i].rulings.back());
            for (bool grew = !loop; grew;) {
                grew = false;
                for (std::size_t j = 0; j < walls.size() && !grew; ++j) {
                    if (used[j]) {
                        continue;
                    }
                    if (same(chain.back().last(), walls[j].rulings.front())) {
                        chain.push_back({&walls[j], false});
                    } else if (same(chain.back().last(), walls[j].rulings.back())) {
                        chain.push_back({&walls[j], true});
                    } else if (same(chain.front().first(), walls[j].rulings.back())) {
                        chain.insert(chain.begin(), {&walls[j], false});
                    } else if (same(chain.front().first(), walls[j].rulings.front())) {
                        chain.insert(chain.begin(), {&walls[j], true});
                    } else {
                        continue;
                    }
                    used[j] = 1;
                    grew = !same(chain.back().last(), chain.front().first());
                }
            }
            chains.push_back(std::move(chain));
        }
    }

    // -- Stations along each chain, turning about the shared ruling at convex corners -----------
    const double turnStep = 2.0 * std::acos(std::clamp(1.0 - ctx.chordTolerance / d, -1.0, 1.0));
    // From station `a` (arrived along `travel`) to the ruling `b` of the next wall.
    const auto corner = [&](std::vector<Station>& out, const Vec3& travel, const WallRuling b) {
        const WallRuling a = out.back().ruling;
        double turn = 0.0;
        for (std::size_t k = 0; k < a.normals.size(); ++k) {
            turn = std::max(turn, std::acos(std::clamp(a.normals[k].dot(b.normals[k]), -1.0, 1.0)));
        }
        if (turn < 1.0 * kDeg) {
            return; // tangent: the same ruling
        }
        if (b.normals.front().dot(travel) <= 0.0) {
            throw Error(
                ErrorCode::InvalidArgument,
                fmt::format("the walls meet in an inside corner at {}: the side of the tool "
                            "cannot reach into it without cutting the other wall; give the "
                            "corner a radius of at least the tool radius ({:.3f} mm)",
                            at(a.points.front()), r));
        }
        if (turn > 175.0 * kDeg) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("the walls meet in a knife edge at {}", at(a.points.front())));
        }
        const int steps = std::max(1, static_cast<int>(std::ceil(turn / std::max(turnStep, 1e-3))));
        for (int s = 1; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            Station st;
            st.ruling.points = a.points;
            st.ruling.direction = slerp(a.direction, b.direction, t);
            for (std::size_t k = 0; k < a.normals.size(); ++k) {
                st.ruling.normals.push_back(slerp(a.normals[k], b.normals[k], t));
            }
            out.push_back(std::move(st));
        }
    };
    const auto travelInto = [](const std::vector<Station>& out) {
        if (out.size() < 2) {
            return Vec3(Vec3::Zero());
        }
        const Vec3 t = out.back().ruling.points.front() - out[out.size() - 2].ruling.points.front();
        return t.norm() > 1e-12 ? Vec3(t.normalized()) : Vec3(Vec3::Zero());
    };
    std::vector<std::vector<Station>> sequences;
    for (const std::vector<Piece>& chain : chains) {
        std::vector<Station> seq;
        for (const Piece& piece : chain) {
            std::vector<WallRuling> rulings = piece.wall->rulings;
            if (piece.reversed) {
                std::reverse(rulings.begin(), rulings.end());
            }
            std::size_t from = 0;
            if (!seq.empty()) {
                corner(seq, travelInto(seq), rulings.front());
                from = 1; // the shared ruling is already there
            }
            for (std::size_t k = from; k < rulings.size(); ++k) {
                seq.push_back({rulings[k], piece.wall});
            }
        }
        const bool closed = chain.size() > 1 ? same(chain.back().last(), chain.front().first())
                                             : same(chain.front().first(), chain.front().last());
        if (closed) {
            corner(seq, travelInto(seq), seq.front().ruling);
        }
        // Climb (M3): the wall on the right of the feed, seen with the tool axis up.
        double side = 0.0;
        for (std::size_t k = 0; k + 1 < seq.size(); ++k) {
            const Vec3 f = seq[k + 1].ruling.points.front() - seq[k].ruling.points.front();
            side += seq[k].ruling.normals.front().dot(f.cross(seq[k].ruling.direction));
        }
        const bool climb = p.direction == CutDirection::Climb;
        if ((climb && side > 0.0) || (!climb && side < 0.0)) {
            std::reverse(seq.begin(), seq.end());
        }
        sequences.push_back(std::move(seq));
    }
    ctx.cancel.throwIfCancelled();

    // -- Tool positions -----------------------------------------------------------------------
    double leaves = 0.0, gouges = 0.0; // flank against twisted walls
    const auto pose = [&](const Station& st) {
        const WallRuling& w = st.ruling;
        const Vec3 a = w.points.front() + d * w.normals.front();
        const Vec3 b = w.points.back() + d * w.normals.back();
        Vec3 u = (b - a).norm() > 0.5 ? Vec3((b - a).normalized()) : w.direction;
        if (u.dot(w.direction) <= 0.0) {
            u = w.direction;
        }
        const double tilt = std::acos(std::clamp(u.z(), -1.0, 1.0));
        if (tilt > p.maxTilt + 1e-9) {
            throw Error(
                ErrorCode::InvalidArgument,
                fmt::format("the wall {} leans {:.1f}° from the Z axis at {}, more than the "
                            "largest tilt ({:.1f}°)",
                            st.wall ? st.wall->name : std::string("corner"), tilt / kDeg,
                            at(w.points.front()), p.maxTilt / kDeg));
        }
        if (st.wall) {
            for (std::size_t k = 1; k + 1 < w.points.size(); ++k) {
                const Vec3 q = w.points[k] - a;
                const double dev = (q - q.dot(u) * u).norm() - d;
                leaves = std::max(leaves, dev);
                gouges = std::max(gouges, -dev);
            }
        }
        Pose ps;
        ps.axis = u;
        // Leads parallel to the floor (level without one): a twisted wall's normal may point
        // down at its foot, and a lead along it would take the tool's end into the floor.
        const Vec3 level = floor ? Vec3(floor->normal.normalized()) : Vec3(Vec3::UnitZ());
        const Vec3 off = w.normals.front() - w.normals.front().dot(level) * level;
        ps.normal = off.norm() > 1e-6 ? Vec3(off.normalized()) : w.normals.front();
        ps.tip = a + p.axisOffset * u;
        if (floor) {
            const Vec3 m = floor->normal.normalized();
            const double mu = m.dot(u);
            if (mu < 0.05) {
                throw Error(ErrorCode::InvalidArgument,
                            fmt::format("at {} the tool axis lies along the floor: the tool cannot "
                                        "be kept above it",
                                        at(w.points.front())));
            }
            const double across = std::sqrt(std::max(0.0, 1.0 - mu * mu));
            const double lowest = m.dot(a + rc * u) - (r - rc) * across - rc;
            const double target = m.dot(floor->point) + p.axisOffset;
            ps.tip = a + ((target - lowest) / mu) * u;
        }
        ps.height = (b - ps.tip).dot(u);
        return ps;
    };
    std::vector<std::vector<Pose>> poses;
    for (const std::vector<Station>& seq : sequences) {
        std::vector<Pose> chainPoses;
        chainPoses.reserve(seq.size());
        for (const Station& st : seq) {
            chainPoses.push_back(pose(st));
        }
        poses.push_back(std::move(chainPoses));
    }
    if (std::max(leaves, gouges) > p.tolerance) {
        throw Error(
            ErrorCode::InvalidArgument,
            fmt::format("the walls twist too much for the tool: its side would {} {:.3f} mm "
                        "(tolerance {:.3f} mm); use a smaller tool or a larger tolerance",
                        gouges >= leaves ? "cut into them by" : "stand off them by",
                        std::max(leaves, gouges), p.tolerance));
    }

    // -- Moves --------------------------------------------------------------------------------
    const Vec3 firstAbove = [&] {
        const Pose& q = poses.front().front();
        return Vec3(q.tip + p.leadDistance * q.normal + (q.height + p.leadDistance) * q.axis);
    }();
    toolpath::Toolpath tp = beginToolpath(ctx, "swarf5x", firstAbove.head<2>());
    Vec3 now = tp.start;
    const auto add = [&](MoveKind kind, FeedClass fc, const Vec3& end, const Vec3& axis,
                         double feed, bool always = false) {
        if (!always && (end - now).norm() < 1e-9 &&
            (kind != MoveKind::Rapid ||
             (!tp.moves.empty() && (tp.moves.back().axis - axis).norm() < 1e-12))) {
            return;
        }
        Move m;
        m.kind = kind;
        m.feedClass = fc;
        m.end = end;
        m.axis = axis;
        m.feedRate = kind == MoveKind::Rapid ? 0.0 : feed;
        tp.moves.push_back(m);
        now = end;
    };
    const double clearance = ctx.heights.clearance;
    const double retract = ctx.heights.retract;
    std::vector<std::pair<Vec3, Vec3>> cutPoses; // (tip, axis), for the checks
    int tallerThanFlutes = 0;
    for (const std::vector<Pose>& chain : poses) {
        for (int pass = 1; pass <= passes; ++pass) {
            const double share = static_cast<double>(passes - pass) / passes;
            std::vector<Vec3> tips;
            tips.reserve(chain.size());
            for (const Pose& q : chain) {
                tips.push_back(q.tip + share * std::max(0.0, q.height) * q.axis);
                cutPoses.emplace_back(tips.back(), q.axis);
                if (pass == passes && tool.mill.fluteLength > 0.0 &&
                    q.height > tool.mill.fluteLength + 1e-6) {
                    ++tallerThanFlutes;
                }
            }
            // Approach: down the axis, off the wall, then onto it.
            const Pose& q0 = chain.front();
            const Vec3 off = tips.front() + p.leadDistance * q0.normal;
            const double rise = std::max(0.0, (1.0 - share) * q0.height) + p.leadDistance;
            const Vec3 above = off + rise * q0.axis;
            if (now.z() < clearance - 1e-9) {
                add(MoveKind::Rapid, FeedClass::Rapid, Vec3(now.x(), now.y(), clearance),
                    tp.moves.empty() ? q0.axis : tp.moves.back().axis, 0.0);
            }
            // Across at the clearance height, even where it goes nowhere (between the passes of
            // a loop): a rotary table may unwind a turn here, clear of the part (planRotary).
            add(MoveKind::Rapid, FeedClass::Rapid,
                Vec3(above.x(), above.y(), std::max(clearance, above.z())), q0.axis, 0.0,
                !tp.moves.empty());
            if (above.z() < retract) {
                add(MoveKind::Rapid, FeedClass::Rapid, Vec3(above.x(), above.y(), retract), q0.axis,
                    0.0);
                add(MoveKind::Linear, FeedClass::Plunge, above, q0.axis, ctx.feeds.plunge);
            } else {
                add(MoveKind::Rapid, FeedClass::Rapid, above, q0.axis, 0.0);
            }
            add(MoveKind::Linear, FeedClass::Plunge, off, q0.axis, ctx.feeds.plunge);
            add(MoveKind::Linear, FeedClass::LeadIn, tips.front(), q0.axis, ctx.feeds.lead);
            for (std::size_t k = 1; k < chain.size(); ++k) {
                add(MoveKind::Linear, FeedClass::Cut, tips[k], chain[k].axis, ctx.feeds.cut);
            }
            // Leave: off the wall, up the axis past its top, then up to the clearance height.
            const Pose& qn = chain.back();
            const Vec3 out = tips.back() + p.leadDistance * qn.normal;
            add(MoveKind::Linear, FeedClass::LeadOut, out, qn.axis, ctx.feeds.lead);
            const double up = std::max(0.0, (1.0 - share) * qn.height) + p.leadDistance;
            add(MoveKind::Rapid, FeedClass::Rapid, out + up * qn.axis, qn.axis, 0.0);
            if (now.z() < clearance) {
                add(MoveKind::Rapid, FeedClass::Rapid, Vec3(now.x(), now.y(), clearance), qn.axis,
                    0.0);
            }
        }
    }

    // -- Clearance of the flutes and shank to the part (reported, nothing moved) ---------------
    if (!part.triangles.empty()) {
        const geom3d::TriangleBvh bvh(part);
        // A hair inside the tool: the wall it rides on, a floor it touches, mesh facets.
        const double g = p.tolerance + 0.006;
        const double flute = tool.mill.fluteLength > 0.0 ? tool.mill.fluteLength : 2.0 * r;
        const double visible = tooling::visibleLength(tool);
        const double shankR =
            (tool.mill.shankDiameter > 0.0 ? tool.mill.shankDiameter : 2.0 * r) / 2.0;
        const auto touches = [&](const geom3d::Convex& body, double clearanceGap) {
            bool hit = false;
            geom3d::Aabb3 box = body.bounds();
            box.min -= Vec3::Constant(clearanceGap);
            box.max += Vec3::Constant(clearanceGap);
            bvh.query(box, [&](std::size_t t) {
                const auto [a, b, c] = bvh.triangle(t);
                if (geom3d::intersects(body, geom3d::Convex::triangle(a, b, c), clearanceGap)) {
                    hit = true;
                    return false;
                }
                return true;
            });
            return hit;
        };
        std::atomic<int> fluteHits{0}, shankHits{0};
        std::mutex firstMutex;
        std::optional<std::pair<std::size_t, Vec3>> firstFlute, firstShank;
        const auto note = [&](std::optional<std::pair<std::size_t, Vec3>>& first, std::size_t k,
                              const Vec3& where) {
            std::lock_guard lock(firstMutex);
            if (!first || k < first->first) {
                first = {k, where};
            }
        };
        tbb::parallel_for(
            tbb::blocked_range<std::size_t>(0, cutPoses.size()),
            [&](const tbb::blocked_range<std::size_t>& range) {
                for (std::size_t k = range.begin(); k != range.end(); ++k) {
                    const auto& [tip, u] = cutPoses[k];
                    const double h0 = std::max(g, rc);
                    bool hit =
                        flute > h0 &&
                        touches(geom3d::Convex::cylinder(tip + h0 * u, u, r - g, flute - h0), 0.0);
                    if (!hit && r - rc - g > 1e-6 && rc > g) {
                        hit = touches(geom3d::Convex::cylinder(tip + g * u, u, r - rc - g, rc - g),
                                      0.0);
                    }
                    if (hit) {
                        ++fluteHits;
                        note(firstFlute, k, tip);
                    }
                    if (visible > flute + 1e-6 &&
                        touches(
                            geom3d::Convex::cylinder(tip + flute * u, u, shankR, visible - flute),
                            p.clearance)) {
                        ++shankHits;
                        note(firstShank, k, tip);
                    }
                }
            });
        if (fluteHits > 0) {
            tp.warnings.push_back(fmt::format(
                "the tool cuts into the part at {} positions (first with its end at {}): check the "
                "floor, the axis offset and the corners in the simulation",
                fluteHits.load(), at(firstFlute->second)));
        }
        if (shankHits > 0) {
            tp.warnings.push_back(
                fmt::format("the tool shank comes within {:.2f} mm of the part at {} positions "
                            "(first with the tool's end at {})",
                            p.clearance, shankHits.load(), at(firstShank->second)));
        }
    }
    if (tallerThanFlutes > 0) {
        tp.warnings.push_back(fmt::format(
            "the walls stand higher than the flute length ({:.2f} mm) at {} positions: the last "
            "pass cuts with the whole wall height",
            tool.mill.fluteLength, tallerThanFlutes));
    }
    if (std::max(leaves, gouges) > 1e-4) {
        tp.warnings.push_back(fmt::format(
            "the walls are twisted: the side of the tool stands off them by up to {:.4f} mm and "
            "cuts into them by up to {:.4f} mm between the ends of the rulings (tolerance {:.3f} "
            "mm)",
            leaves, gouges, p.tolerance));
    }
    ctx.progress.report(1.0, "Swarf");
    return tp;
}

SwarfParams swarfParams(const Json& values) {
    const ParamReader r(*findStrategy("swarf5x"), values);
    SwarfParams p;
    p.depthPasses = r.integer("depth_passes");
    p.axisOffset = r.number("axis_offset");
    p.stockToLeave = r.number("stock_to_leave");
    p.tolerance = r.number("tolerance");
    p.maxTilt = r.number("max_tilt");
    p.leadDistance = r.number("lead_distance");
    p.clearance = r.number("clearance");
    p.direction =
        r.choice("direction") == "conventional" ? CutDirection::Conventional : CutDirection::Climb;
    return p;
}

} // namespace ocam::strategies
