// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Back boring and back chamfering through a hole from its near side. See
// docs/design-log/0080.

#include "ocam/strategies/BackBore.hpp"

#include "Internal.hpp"

#include <ocam/foundation/Units.hpp>
#include <ocam/geom2d/Ordering.hpp>
#include <ocam/tooling/Profile.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>

namespace ocam::strategies {
namespace {

using toolpath::FeedClass;

constexpr double kStep = 0.01; ///< Height step of the profile scans [mm].
constexpr double kTol = 1e-3;  ///< Contact tolerance of the fit checks [mm].

/// The radius of a tool's revolved profile at each height above its tip.
class Profile {
public:
    explicit Profile(const tooling::Tool& tool) : profile_(tooling::toolProfile(tool)) {
        points_ = profile_.outline.discretize(1e-4);
        for (const Vec2& p : points_) {
            if (p.y() <= profile_.cuttingLength + 1e-9) {
                rMax_ = std::max(rMax_, p.x());
            }
        }
        for (const Vec2& p : points_) {
            if (p.y() <= profile_.cuttingLength + 1e-9 && p.x() >= rMax_ - 1e-9) {
                hMax_ = std::max(hMax_, p.y());
            }
        }
    }

    double length() const { return profile_.length; }
    double rMax() const { return rMax_; } ///< Largest radius of the cutting part.
    double hMax() const { return hMax_; } ///< Highest height at that radius.

    /// Largest radius at height h; 0 outside the tool.
    double at(double h) const {
        if (h < 0.0 || h > profile_.length) {
            return 0.0;
        }
        double best = 0.0;
        for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
            const Vec2& a = points_[i];
            const Vec2& c = points_[i + 1];
            const double lo = std::min(a.y(), c.y());
            const double hi = std::max(a.y(), c.y());
            if (h < lo || h > hi) {
                continue;
            }
            if (hi - lo < 1e-12) {
                best = std::max({best, a.x(), c.x()});
            } else {
                best = std::max(best, a.x() + (h - a.y()) / (c.y() - a.y()) * (c.x() - a.x()));
            }
        }
        return best;
    }

    /// The lowest height above `from` where the radius is below `radius`, refined to 1e-6 mm.
    std::optional<double> firstBelow(double radius, double from) const {
        double prev = from;
        for (double h = from; h <= profile_.length + 1e-9; h += kStep) {
            if (at(h) < radius) {
                double lo = prev;
                double hi = h;
                for (int k = 0; k < 40 && hi - lo > 1e-7; ++k) {
                    const double mid = 0.5 * (lo + hi);
                    (at(mid) < radius ? hi : lo) = mid;
                }
                return h == from ? from : hi;
            }
            prev = h;
        }
        return std::nullopt;
    }

private:
    tooling::ToolProfile profile_;
    std::vector<Vec2> points_;
    double rMax_ = 0.0;
    double hMax_ = 0.0;
};

std::string at(const HoleTarget& h) {
    return fmt::format("({:.3f}, {:.3f})", h.position.x(), h.position.y());
}

/// Moves to above `xy` at the R plane of a hole (Holes.cpp does the same for its operations).
void approachHole(toolpath::PathBuilder& b, const Context& ctx, const Vec2& xy, double rPlane,
                  double& previousTop, double top) {
    if (std::abs(top - previousTop) > 1e-9) {
        b.retractTo(ctx.heights.clearance);
    }
    b.retractTo(rPlane);
    if ((b.position().head<2>() - xy).norm() > 1e-9) {
        b.rapid(Vec3(xy.x(), xy.y(), b.position().z()));
    }
    if (b.position().z() > rPlane + 1e-9) {
        b.rapid(Vec3(xy.x(), xy.y(), rPlane));
    }
    previousTop = top;
}

/// The tool turning on the hole's axis with its tip from zLo up to zHi: at every height of the
/// hole above `freeTop` it must stay within the hole's radius (a neck long enough).
void checkReach(const Profile& tool, const tooling::Tool& t, const HoleTarget& h, double orbit,
                double zLo, double zHi, double freeTop) {
    const double rh = h.diameter / 2.0;
    const int steps = std::max(1, static_cast<int>(std::ceil((zHi - zLo) / 0.1)));
    for (int k = 0; k <= steps; ++k) {
        const double zt = zLo + (zHi - zLo) * k / steps;
        for (double z = freeTop + 1e-4; z <= h.topZ + 1e-9; z += 0.05) {
            const double r = tool.at(z - zt);
            if (orbit + r > rh + kTol) {
                throw Error(ErrorCode::InvalidArgument,
                            fmt::format("T{} cannot reach through the hole at {} (Ø{:.3f}): {:.3f} "
                                        "mm above its tip it is Ø{:.3f}, wider than the hole{}; it "
                                        "needs a longer or narrower neck",
                                        t.number, at(h), h.diameter, z - zt, 2.0 * r,
                                        orbit > 0.0 ? " on its circle" : ""));
            }
        }
    }
}

/// The oriented tool, its edge pointing away from the shift, moving with its tip between zLo
/// and zHi while shifted by `shift` from the hole's axis: its head (below `hNeck`) reaches
/// R(h) - shift on its edge's side and the neck's radius + shift on the other; above the head
/// it is round, R(h) + shift. At each height it must stay within `room` (the hole, or what the
/// cut has opened).
void checkPassage(const Profile& tool, const tooling::Tool& t, const HoleTarget& h, double shift,
                  double neck, double hNeck, double zLo, double zHi,
                  const std::function<double(double)>& room) {
    const double rh = h.diameter / 2.0;
    const double zFar = h.topZ - h.depth;
    const int steps = std::max(1, static_cast<int>(std::ceil((zHi - zLo) / 0.1)));
    for (int k = 0; k <= steps; ++k) {
        const double zt = zLo + (zHi - zLo) * k / steps;
        for (double z = zFar; z <= h.topZ + 1e-9; z += 0.05) {
            const double hh = z - zt;
            if (hh < 0.0 || hh > tool.length()) {
                continue;
            }
            const double r = tool.at(hh);
            const double edgeSide = hh < hNeck ? r - shift : r + shift;
            const double backSide = hh < hNeck ? neck + shift : r + shift;
            if (std::max(edgeSide, backSide) > room(z) - kTol) {
                throw Error(
                    ErrorCode::InvalidArgument,
                    fmt::format("T{} cannot pass through the hole at {} (Ø{:.3f}) shifted "
                                "{:.3f} mm: {:.3f} mm above its tip it reaches {:.3f} mm from "
                                "the hole's axis{}",
                                t.number, at(h), h.diameter, shift, hh,
                                std::max(edgeSide, backSide),
                                hh < hNeck ? fmt::format("; the shift must be between {:.3f} and "
                                                         "{:.3f} mm",
                                                         tool.rMax() - rh, rh - neck)
                                           : std::string("; it needs a longer neck")));
            }
        }
    }
}

/// Height above the tip where an interpolated chamfer of width w and vertical size v starts:
/// the head's radius falls by w over v from there.
std::optional<double> chamferHeight(const Profile& tool, double from, double to, double w,
                                    double v) {
    const auto f = [&](double h0) {
        return tool.at(h0) - tool.at(h0 + v) - w;
    };
    double prev = from;
    double fPrev = f(from);
    if (std::abs(fPrev) < 1e-6) {
        return from;
    }
    for (double h0 = from + kStep; h0 <= to + 1e-9; h0 += kStep) {
        const double fh = f(h0);
        if ((fh > 0.0) != (fPrev > 0.0) || std::abs(fh) < 1e-6) {
            double lo = prev;
            double hi = h0;
            for (int k = 0; k < 50; ++k) {
                const double mid = 0.5 * (lo + hi);
                ((f(mid) > 0.0) == (fPrev > 0.0) ? lo : hi) = mid;
            }
            return 0.5 * (lo + hi);
        }
        prev = h0;
        fPrev = fh;
    }
    return std::nullopt;
}

} // namespace

double backBoreNeckRadius(const tooling::Tool& tool) {
    const tooling::MillGeometry& g = tool.mill;
    if (g.neckDiameter > 0.0) {
        return g.neckDiameter / 2.0;
    }
    return (g.shankDiameter > 0.0 ? g.shankDiameter : g.diameter) / 2.0;
}

toolpath::Toolpath backBore(const std::vector<HoleTarget>& holes, const BackBoreParams& p,
                            const Context& ctx) {
    const tooling::Tool& tool = ctx.tool;
    detail::requireMillingTool(tool);
    const bool oriented = p.method == BackBoreMethod::Oriented;
    const bool chamfer = p.feature == BackFeature::Chamfer;
    if (oriented && tool.type != tooling::ToolType::BoringBar) {
        throw Error(ErrorCode::InvalidArgument,
                    "back boring with the spindle oriented needs a back-boring bar (a boring bar "
                    "with a neck narrower than its head); other tools back-bore by circular "
                    "interpolation");
    }
    const Profile profile(tool);
    const double neck = backBoreNeckRadius(tool);
    const double rMax = profile.rMax();
    const double hMax = profile.hMax();
    if (neck >= rMax - 1e-9) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("T{} has no head wider than its neck to cut the far side with: "
                                "set its neck diameter (a back-boring bar, slot cutter, lollipop "
                                "or end mill with a neck)",
                                tool.number));
    }
    // Where the head ends: the neck starts.
    const double hNeck = profile.firstBelow(neck + 1e-6, hMax).value_or(profile.length());
    if (chamfer && (p.chamferWidth <= 0.0)) {
        throw Error(ErrorCode::InvalidArgument, "the chamfer width must be positive");
    }
    if (!chamfer && p.depth <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the counterbore depth must be positive");
    }
    if (!oriented && !chamfer && p.helixPitch <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "the helix pitch must be positive");
    }

    std::vector<Vec2> points;
    for (const auto& h : holes) {
        points.push_back(h.position);
    }
    const auto order = geom2d::orderPoints(points, Vec2::Zero());
    toolpath::Toolpath tp =
        beginToolpath(ctx, "backbore", holes.empty() ? Vec2::Zero() : points[order.front()]);
    if (holes.empty()) {
        tp.warnings.push_back("no holes to back-bore");
        return tp;
    }
    toolpath::PathBuilder b(tp);
    const bool clockwise = p.direction == CutDirection::Conventional;
    double previousTop = holes[order.front()].topZ;
    for (std::size_t idx : order) {
        ctx.cancel.throwIfCancelled();
        const HoleTarget& h = holes[idx];
        if (!h.through || h.depth <= 0.0) {
            tp.warnings.push_back(
                fmt::format("hole at {} is not a through hole: nothing to reach; skipped", at(h)));
            continue;
        }
        if (h.diameter <= 0.0) {
            tp.warnings.push_back(fmt::format("hole at {} has no diameter; skipped", at(h)));
            continue;
        }
        const double rh = h.diameter / 2.0;
        const double zFar = h.topZ - h.depth;
        const double rPlane =
            p.rPlaneZ ? std::max(*p.rPlaneZ, h.topZ) : h.topZ + p.retractClearance;
        const Vec2 c = h.position;

        if (oriented) {
            if (!chamfer && rMax <= rh + 1e-6) {
                tp.warnings.push_back(fmt::format(
                    "T{} (Ø{:.3f}) is not larger than the hole at {} (Ø{:.3f}): nothing to "
                    "back-bore; skipped",
                    tool.number, 2.0 * rMax, at(h), h.diameter));
                continue;
            }
            // The cut ends with the head's top at the counterbore's depth, or with the cone
            // at the chamfer's width on the far side.
            double tipTop = zFar + p.depth - hMax;
            if (chamfer) {
                if (rh + p.chamferWidth > rMax + 1e-9) {
                    throw Error(ErrorCode::InvalidArgument,
                                fmt::format("a {:.3f} mm chamfer on the hole at {} (Ø{:.3f}) "
                                            "reaches Ø{:.3f}, more than T{}'s Ø{:.3f}",
                                            p.chamferWidth, at(h), h.diameter,
                                            h.diameter + 2.0 * p.chamferWidth, tool.number,
                                            2.0 * rMax));
                }
                const auto h0 = profile.firstBelow(rh + p.chamferWidth - 1e-9, hMax);
                if (!h0 || std::abs(profile.at(*h0) - (rh + p.chamferWidth)) > 0.01 ||
                    profile.at(*h0 + kStep) >= rh + p.chamferWidth) {
                    throw Error(ErrorCode::InvalidArgument,
                                fmt::format("T{}'s head has a flat top, which cuts counterbores: "
                                            "a back chamfer needs a back-chamfer cutter (a point "
                                            "angle on the bar)",
                                            tool.number));
                }
                tipTop = zFar - *h0;
            }
            // Below the hole the head (wider than the hole) is clear of the far side.
            const auto hIn = profile.firstBelow(rh, hMax);
            const double hClear = hIn.value_or(hNeck);
            const double bottomZ = zFar - p.clearanceBelow - hClear;
            if (bottomZ >= tipTop - 1e-6) {
                throw Error(ErrorCode::InvalidArgument,
                            "the clearance below the far side leaves nothing to cut");
            }
            // Down through the hole as it is; back up through what the cut opened.
            checkPassage(profile, tool, h, p.shift, neck, hNeck, bottomZ, rPlane,
                         [rh](double) { return rh; });
            checkPassage(profile, tool, h, p.shift, neck, hNeck, tipTop, rPlane,
                         [&](double z) { return std::max(rh, profile.at(z - tipTop)); });
            checkReach(profile, tool, h, 0.0, bottomZ, tipTop, tipTop + hClear);
            approachHole(b, ctx, c, rPlane, previousTop, h.topZ);
            toolpath::DrillCycle cy;
            cy.type = toolpath::CycleType::BackBore;
            cy.topZ = tipTop;
            cy.bottomZ = bottomZ;
            cy.retractZ = rPlane;
            cy.feedRate = ctx.feeds.cut;
            cy.dwell = p.dwell;
            cy.shift = p.shift * Vec2(std::cos(p.shiftAngle), std::sin(p.shiftAngle));
            cy.orient = p.orient;
            b.cycle(c, cy);
            ctx.progress.report(0.5, "Back boring");
            continue;
        }

        // Interpolated: the tool goes down the hole's axis, so all of it must pass.
        if (rMax >= rh - kTol) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("T{} (Ø{:.3f}) does not pass through the hole at {} "
                                    "(Ø{:.3f})",
                                    tool.number, 2.0 * rMax, at(h), h.diameter));
        }
        const double startZ = zFar - p.clearanceBelow - hNeck;
        approachHole(b, ctx, c, rPlane, previousTop, h.topZ);
        if (chamfer) {
            const double v = p.chamferWidth / std::tan(p.chamferAngle);
            const auto h0 = chamferHeight(profile, hMax, hNeck - v, p.chamferWidth, v);
            const double rho = h0 ? rh + p.chamferWidth - profile.at(*h0) : -1.0;
            if (!h0 || rho < 0.0) {
                throw Error(ErrorCode::InvalidArgument,
                            fmt::format("T{}'s head cannot cut a {:.3f} mm chamfer at {:.1f}° on "
                                        "the far side of the hole at {}: choose a tool whose head "
                                        "narrows by the chamfer's width over its height (a "
                                        "lollipop, or a back-chamfer cutter of that angle)",
                                        tool.number, p.chamferWidth, radToDeg(p.chamferAngle),
                                        at(h)));
            }
            const double tip = zFar - *h0;
            // The break's two edges are held: the far side is cut out to the width and no
            // further, the wall up to the break's height and no higher (between them the head's
            // own shape makes the break: a cone a chamfer, a ball a rounded one).
            for (double z = zFar; z <= zFar + v + 1e-9; z += 0.01) {
                if (rho + profile.at(z - tip) > rh + p.chamferWidth + kTol + ctx.chordTolerance) {
                    throw Error(ErrorCode::InvalidArgument,
                                fmt::format("T{} would cut past the chamfer on the hole at {} "
                                            "({:.3f} mm above the far side)",
                                            tool.number, at(h), z - zFar));
                }
            }
            checkReach(profile, tool, h, 0.0, startZ, tip, zFar);
            checkReach(profile, tool, h, rho, tip, tip, zFar + v);
            b.feed(Vec3(c.x(), c.y(), tip), FeedClass::Plunge, ctx.feeds.plunge);
            if (rho > 1e-6) {
                const Vec2 lead = c + Vec2(rho / 2.0, 0.0);
                const Vec2 start = c + Vec2(rho, 0.0);
                b.arc(Vec3(start.x(), start.y(), tip), lead, clockwise, FeedClass::LeadIn,
                      ctx.feeds.lead);
                const Vec2 opposite = c - Vec2(rho, 0.0);
                b.arc(Vec3(opposite.x(), opposite.y(), tip), c, clockwise, FeedClass::Cut,
                      ctx.feeds.cut);
                b.arc(Vec3(start.x(), start.y(), tip), c, clockwise, FeedClass::Cut, ctx.feeds.cut);
                b.arc(Vec3(c.x(), c.y(), tip), lead, clockwise, FeedClass::LeadOut, ctx.feeds.lead);
            }
            b.retractTo(rPlane);
            ctx.progress.report(0.5, "Back chamfering");
            continue;
        }

        // Interpolated counterbore: a helix up from below the far side to the counterbore's
        // depth, then a circle there. The neck stays in the hole, so the head reaches at most
        // rh - neck + R from the axis, and on that circle it also reaches back to the hole: one
        // circle always covers the ring.
        if (p.diameter <= h.diameter + 1e-6) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("set the counterbore diameter (larger than the hole at {}, "
                                    "Ø{:.3f})",
                                    at(h), h.diameter));
        }
        const double largest = 2.0 * (rh - neck + rMax);
        if (p.diameter > largest + 1e-9) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("a Ø{:.3f} counterbore on the hole at {} puts T{}'s neck "
                                    "outside the hole: with it, the largest is Ø{:.3f}",
                                    p.diameter, at(h), tool.number, largest));
        }
        const std::vector<double> rings{p.diameter / 2.0 - rMax};
        const double tip = zFar + p.depth - hMax;
        checkReach(profile, tool, h, 0.0, startZ, tip, zFar);
        for (const double rho : rings) {
            for (double z = zFar; z <= zFar + p.depth; z += 0.05) {
                if (rho + profile.at(z - tip) > p.diameter / 2.0 + kTol + ctx.chordTolerance) {
                    throw Error(ErrorCode::InvalidArgument,
                                fmt::format("T{} would cut past the Ø{:.3f} counterbore on the "
                                            "hole at {}",
                                            tool.number, p.diameter, at(h)));
                }
            }
            checkReach(profile, tool, h, rho, startZ, tip, zFar + p.depth);
        }
        b.feed(Vec3(c.x(), c.y(), startZ), FeedClass::Plunge, ctx.feeds.plunge);
        for (std::size_t k = 0; k < rings.size(); ++k) {
            const double rho = rings[k];
            if (k > 0) {
                b.feed(Vec3(c.x(), c.y(), startZ), FeedClass::Link, ctx.feeds.cut);
            }
            const Vec2 start = c + Vec2(rho, 0.0);
            const Vec2 opposite = c - Vec2(rho, 0.0);
            b.arc(Vec3(start.x(), start.y(), startZ), c + Vec2(rho / 2.0, 0.0), clockwise,
                  FeedClass::LeadIn, ctx.feeds.lead);
            // Up by at most the pitch a turn, in half circles, then a full circle at the top.
            const double rise = tip - startZ;
            const int turns = std::max(1, static_cast<int>(std::ceil(rise / p.helixPitch - 1e-9)));
            for (int t = 0; t < turns; ++t) {
                b.arc(Vec3(opposite.x(), opposite.y(), startZ + rise * (t + 0.5) / turns), c,
                      clockwise, FeedClass::Cut, ctx.feeds.cut);
                b.arc(Vec3(start.x(), start.y(), startZ + rise * (t + 1.0) / turns), c, clockwise,
                      FeedClass::Cut, ctx.feeds.cut);
            }
            b.arc(Vec3(opposite.x(), opposite.y(), tip), c, clockwise, FeedClass::Cut,
                  ctx.feeds.cut);
            b.arc(Vec3(start.x(), start.y(), tip), c, clockwise, FeedClass::Cut, ctx.feeds.cut);
            b.arc(Vec3(c.x(), c.y(), tip), c + Vec2(rho / 2.0, 0.0), clockwise, FeedClass::LeadOut,
                  ctx.feeds.lead);
        }
        b.retractTo(rPlane);
        ctx.progress.report(0.5, "Back boring");
    }
    b.retractTo(ctx.heights.clearance);
    return tp;
}

namespace detail {

StrategyInfo backBoreStrategy() {
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
    const auto when = [](ParamDef d, std::string key, std::vector<std::string> values) {
        d.onlyWhen = {std::move(key), std::move(values)};
        return d;
    };
    StrategyInfo s;
    s.id = "backbore";
    s.label = "Back boring";
    s.geometry = GeometryUse::Holes;
    s.description = "Machines the far side of through holes from the near side: a back "
                    "counterbore, spot face or chamfer, with a back-boring bar passed through "
                    "off centre or a tool small enough to pass that circles the far edge.";

    ParamDef method;
    method.key = "method";
    method.label = "Method";
    method.kind = ParamKind::Choice;
    method.choices = {{"oriented", "Oriented spindle: back-boring bar"},
                      {"interpolated", "Circular interpolation"}};
    method.defaultChoice = "oriented";
    method.help = "Oriented spindle: the spindle stops at its angle, the bar shifts off centre, "
                  "passes the hole, shifts back, starts and cuts upward (a back-boring cycle "
                  "where the post has one). Circular interpolation: a tool that passes the hole "
                  "circles the far edge with its head";
    s.params.push_back(method);

    ParamDef feature;
    feature.key = "feature";
    feature.label = "Feature";
    feature.kind = ParamKind::Choice;
    feature.choices = {{"counterbore", "Counterbore or spot face"}, {"chamfer", "Chamfer"}};
    feature.defaultChoice = "counterbore";
    feature.help = "What is cut on the far side of the hole";
    s.params.push_back(feature);

    s.params.push_back(when(length("depth", "Counterbore depth", 2.0, 0.01, 500.0,
                                   "Height of the counterbore or spot face above the far side; "
                                   "the head's top (its widest part) stops there"),
                            "feature", {"counterbore"}));
    s.params.push_back(when(length("diameter", "Counterbore diameter", 0.0, 0.0, 1000.0,
                                   "Circular interpolation of a counterbore: its diameter. The "
                                   "oriented method cuts the bar's own diameter"),
                            "method", {"interpolated"}));
    s.params.push_back(when(length("chamfer_width", "Chamfer width", 0.5, 0.01, 50.0,
                                   "Across the far side, from the hole's edge"),
                            "feature", {"chamfer"}));
    ParamDef angle;
    angle.key = "chamfer_angle";
    angle.label = "Chamfer angle";
    angle.kind = ParamKind::Angle;
    angle.defaultValue = 45.0;
    angle.min = 1.0;
    angle.max = 89.0;
    angle.help = "Circular interpolation: the chamfer's angle from the hole's axis. The oriented "
                 "method cuts the cutter's own angle";
    s.params.push_back(when(angle, "method", {"interpolated"}));

    ParamDef shift = when(length("shift", "Shift", 0.0, 0.0, 100.0,
                                 "How far the oriented bar moves off the hole's axis, away from "
                                 "its cutting edge, to pass the hole; while not set, half the "
                                 "difference between the head and the neck"),
                          "method", {"oriented"});
    shift.toolDefault = [](const tooling::Tool& t) {
        return std::max(0.0, (t.mill.diameter / 2.0 - backBoreNeckRadius(t)) / 2.0);
    };
    s.params.push_back(shift);

    ParamDef direction;
    direction.key = "shift_direction";
    direction.label = "Shift direction";
    direction.kind = ParamKind::Choice;
    direction.choices = {{"x+", "+X"}, {"x-", "-X"}, {"y+", "+Y"}, {"y-", "-Y"}};
    direction.defaultChoice = "x-";
    direction.help = "The direction of the shift, away from the bar's cutting edge as it stands "
                     "at the spindle orientation angle";
    s.params.push_back(when(direction, "method", {"oriented"}));

    ParamDef orient;
    orient.key = "orient_angle";
    orient.label = "Spindle orientation";
    orient.kind = ParamKind::Angle;
    orient.defaultValue = 0.0;
    orient.min = -360.0;
    orient.max = 360.0;
    orient.help = "The angle the spindle stops at (M19) before the bar shifts: where its cutting "
                  "edge points against the shift direction";
    s.params.push_back(when(orient, "method", {"oriented"}));

    s.params.push_back(length("clearance_below", "Clearance below the far side", 2.0, 0.0, 100.0,
                              "How far below the far side the head comes before it moves out "
                              "or starts turning"));
    ParamDef dwell;
    dwell.key = "dwell";
    dwell.label = "Dwell";
    dwell.kind = ParamKind::Time;
    dwell.max = 60.0;
    dwell.help = "Pause at the top of the cut (a clean spot face)";
    s.params.push_back(when(dwell, "method", {"oriented"}));
    s.params.push_back(when(length("helix_pitch", "Helix pitch", 1.0, 0.01, 50.0,
                                   "Circular interpolation of a counterbore: rise per turn"),
                            "method", {"interpolated"}));
    ParamDef cut;
    cut.key = "direction";
    cut.label = "Direction";
    cut.kind = ParamKind::Choice;
    cut.choices = {{"climb", "Climb"}, {"conventional", "Conventional"}};
    cut.defaultChoice = "climb";
    cut.help = "Circular interpolation: the direction the tool goes around";
    s.params.push_back(when(cut, "method", {"interpolated"}));
    return s;
}

} // namespace detail

BackBoreParams backBoreParams(const Json& values, const tooling::Tool& tool,
                              const tooling::CuttingPreset* preset) {
    const StrategyInfo& info = *findStrategy("backbore");
    const ParamReader r(info, values);
    BackBoreParams p;
    p.method = r.choice("method") == "interpolated" ? BackBoreMethod::Interpolated
                                                    : BackBoreMethod::Oriented;
    p.feature = r.choice("feature") == "chamfer" ? BackFeature::Chamfer : BackFeature::Counterbore;
    p.depth = r.number("depth");
    p.diameter = r.number("diameter");
    p.chamferWidth = r.number("chamfer_width");
    p.chamferAngle = r.number("chamfer_angle");
    p.shift = r.isSet("shift") ? r.number("shift") : info.find("shift")->toolDefault(tool);
    const std::string dir = r.choice("shift_direction");
    p.shiftAngle = dir == "x+" ? 0.0 : dir == "y+" ? kPi / 2.0 : dir == "y-" ? -kPi / 2.0 : kPi;
    p.orient = r.number("orient_angle");
    p.clearanceBelow = r.number("clearance_below");
    p.dwell = r.number("dwell");
    p.helixPitch = r.number("helix_pitch");
    (void)preset;
    p.direction =
        r.choice("direction") == "conventional" ? CutDirection::Conventional : CutDirection::Climb;
    return p;
}

} // namespace ocam::strategies
