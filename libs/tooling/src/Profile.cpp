// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/tooling/Profile.hpp"

#include <ocam/foundation/Error.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>

namespace ocam::tooling {
namespace {

// Accumulates outline vertices, skipping consecutive duplicates. The bulge belongs to the
// segment that starts at the vertex.
class OutlineBuilder {
public:
    void add(double r, double z, double bulge = 0.0) {
        const Vec2 p(r, z);
        if (!vertices_.empty() && (vertices_.back().p - p).norm() < 1e-12) {
            vertices_.back().bulge = bulge;
            return;
        }
        vertices_.push_back({p, bulge});
    }
    Vec2 last() const { return vertices_.back().p; }
    geom2d::Polyline build() { return {std::move(vertices_), false}; }

private:
    std::vector<geom2d::Vertex> vertices_;
};

constexpr double kQuarterBulge = 0.41421356237309503; // tan(pi/8): 90 degree CCW arc

} // namespace

ToolProfile toolProfile(const Tool& tool) {
    if (isTurningTool(tool.type)) {
        throw Error(ErrorCode::Unsupported, "turning tools have no revolved profile");
    }
    if (const auto issues = tool.validate(); !issues.empty()) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("tool T{} is invalid: {}", tool.number, issues.front()));
    }

    const MillGeometry& g = tool.mill;
    const double r = g.diameter / 2.0;
    const double shankR = g.shankDiameter > 0.0 ? g.shankDiameter / 2.0 : r;
    const double fl = g.fluteLength;

    OutlineBuilder b;
    ToolProfile profile;
    profile.maxRadius = r;
    profile.length = g.overallLength;
    profile.cuttingLength = fl;

    // Head: everything up to the top of the cutting part.
    switch (tool.type) {
    case ToolType::BallEndMill:
        b.add(0, 0, kQuarterBulge);
        b.add(r, r);
        b.add(r, fl);
        break;
    case ToolType::BullNoseEndMill:
        b.add(0, 0);
        b.add(r - g.cornerRadius, 0, kQuarterBulge);
        b.add(r, g.cornerRadius);
        b.add(r, fl);
        break;
    case ToolType::Drill:
    case ToolType::SpotDrill:
        b.add(0, 0);
        b.add(r, r / std::tan(g.tipAngle / 2.0));
        b.add(r, fl);
        break;
    case ToolType::Chamfer:
    case ToolType::Engraver: {
        const double rt = g.tipDiameter / 2.0;
        const double coneTop = (r - rt) / std::tan(g.tipAngle / 2.0);
        b.add(0, 0);
        b.add(rt, 0);
        b.add(r, coneTop);
        b.add(r, std::max(fl, coneTop));
        profile.cuttingLength = std::max(fl, coneTop);
        break;
    }
    case ToolType::CenterDrill: {
        const double rp = g.tipDiameter / 2.0;
        constexpr double kCountersinkHalfAngle = 30.0 * kPi / 180.0; // 60 degree body chamfer
        const double sinkTop = g.tipLength + (r - rp) / std::tan(kCountersinkHalfAngle);
        b.add(0, 0);
        b.add(rp, rp / std::tan(g.tipAngle / 2.0));
        b.add(rp, g.tipLength);
        b.add(r, sinkTop);
        b.add(r, std::max(fl, sinkTop));
        profile.cuttingLength = std::max(fl, sinkTop);
        break;
    }
    case ToolType::Lollipop: {
        // Sphere of radius r centred on the axis at height r, cut off where the neck starts.
        const double rn = g.neckDiameter / 2.0;
        const double top = r + std::sqrt(r * r - rn * rn);
        const double sweep = kPi / 2.0 + std::atan2(top - r, rn);
        b.add(0, 0, geom2d::bulgeFromSweep(sweep));
        b.add(rn, top);
        profile.cuttingLength = top;
        break;
    }
    case ToolType::Probe: {
        // The stylus: a ball on the axis at height r, then its stem up to the probe body. The
        // simulation takes the stylus for the "cutting" part (it never removes material: probe
        // moves only check for contact) and the body for the shank.
        const double rn = g.neckDiameter / 2.0;
        const double top = r + std::sqrt(std::max(0.0, r * r - rn * rn));
        const double sweep = kPi / 2.0 + std::atan2(top - r, rn);
        b.add(0, 0, geom2d::bulgeFromSweep(sweep));
        b.add(rn, top);
        b.add(rn, std::max(top, fl));
        profile.cuttingLength = std::max(top, fl);
        break;
    }
    case ToolType::BarPuller: {
        // A body around the bar's bore, as deep as its grip; the bar goes into it.
        const double rb = g.tipDiameter / 2.0;
        b.add(0, fl);
        b.add(rb, fl);
        b.add(rb, 0);
        b.add(r, 0);
        b.add(r, fl);
        break;
    }
    case ToolType::Form:
        for (const Vec2& p : g.formProfile) {
            b.add(p.x(), p.y());
        }
        profile.maxRadius = 0.0;
        for (const Vec2& p : g.formProfile) {
            profile.maxRadius = std::max(profile.maxRadius, p.x());
        }
        profile.cuttingLength = std::max(fl, g.formProfile.back().y());
        break;
    case ToolType::BoringBar:
        b.add(0, 0);
        b.add(r, 0);
        b.add(r, fl);
        // A back-chamfer cutter: its head narrows to the neck in a cone facing up (DL-0083).
        if (g.neckDiameter > 0.0 && g.tipAngle > 0.0 && g.tipAngle < kPi) {
            const double rn = g.neckDiameter / 2.0;
            b.add(rn, fl + (r - rn) / std::tan(g.tipAngle / 2.0));
            profile.cuttingLength = b.last().y();
        }
        break;
    default: // flat end mill, reamer, tap, thread mill, slot cutter
        b.add(0, 0);
        b.add(r, 0);
        b.add(r, fl);
        break;
    }

    // Neck, then shank.
    double z = b.last().y();
    if (g.neckDiameter > 0.0) {
        const double rn = g.neckDiameter / 2.0;
        b.add(rn, z);
        z = std::max(z, g.neckLength);
        b.add(rn, z);
    }
    b.add(shankR, z);
    b.add(shankR, g.overallLength);
    b.add(0, g.overallLength);

    profile.outline = b.build();
    return profile;
}

double profileRadiusAt(const ToolProfile& profile, double z) {
    if (z < 0.0 || z > profile.length) {
        return 0.0;
    }
    const auto pts = profile.outline.discretize(1e-4);
    double best = 0.0;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const Vec2& a = pts[i];
        const Vec2& c = pts[i + 1];
        const double lo = std::min(a.y(), c.y());
        const double hi = std::max(a.y(), c.y());
        if (z < lo || z > hi) {
            continue;
        }
        if (hi - lo < 1e-12) {
            best = std::max({best, a.x(), c.x()});
        } else {
            const double t = (z - a.y()) / (c.y() - a.y());
            best = std::max(best, a.x() + t * (c.x() - a.x()));
        }
    }
    return best;
}

} // namespace ocam::tooling
