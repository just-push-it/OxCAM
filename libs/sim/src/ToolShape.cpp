// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/sim/ToolShape.hpp"

#include <algorithm>
#include <cmath>

namespace ocam::sim {
namespace {

constexpr int kSpanSamples = 1025;
constexpr int kHeightCells = 512;

/// Closed outline of the tool solid in (r, h): the profile plus the axis.
std::vector<Vec2> solidOutline(const tooling::ToolProfile& profile) {
    std::vector<Vec2> pts = profile.outline.discretize(1e-3);
    if (!pts.empty() && pts.front().x() > 1e-12) {
        pts.insert(pts.begin(), Vec2(0.0, pts.front().y()));
    }
    if (!pts.empty() && pts.back().x() > 1e-12) {
        pts.emplace_back(0.0, pts.back().y());
    }
    return pts;
}

/// Heights where the vertical line r = d crosses the closed outline (sorted).
std::vector<double> crossingsAtRadius(const std::vector<Vec2>& poly, double d) {
    std::vector<double> hs;
    const std::size_t n = poly.size();
    for (std::size_t k = 0; k < n; ++k) {
        const Vec2& p = poly[k];
        const Vec2& q = poly[(k + 1) % n];
        if ((p.x() <= d) != (q.x() <= d)) {
            hs.push_back(p.y() + (d - p.x()) * (q.y() - p.y()) / (q.x() - p.x()));
        }
    }
    std::sort(hs.begin(), hs.end());
    return hs;
}

/// Largest radius where the horizontal line at height h crosses the closed outline.
double radiusAtHeight(const std::vector<Vec2>& poly, double h) {
    double r = 0.0;
    const std::size_t n = poly.size();
    for (std::size_t k = 0; k < n; ++k) {
        const Vec2& p = poly[k];
        const Vec2& q = poly[(k + 1) % n];
        if ((p.y() <= h) != (q.y() <= h)) {
            r = std::max(r, p.x() + (h - p.y()) * (q.x() - p.x()) / (q.y() - p.y()));
        }
    }
    return r;
}

/// Convex slices of the solid of revolution bounded by the profile outline: straight segments
/// give frustums, arcs centred on the axis spherical zones, other arcs chords.
std::vector<SolidPiece> solidPieces(const tooling::ToolProfile& profile) {
    std::vector<SolidPiece> out;
    const auto& vs = profile.outline.vertices();
    auto frustum = [&](Vec2 a, Vec2 b) {
        if (std::abs(b.y() - a.y()) < 1e-12 || (a.x() < 1e-12 && b.x() < 1e-12)) {
            return;
        }
        if (b.y() < a.y()) {
            std::swap(a, b);
        }
        out.push_back({a.y(), b.y(), std::max(0.0, a.x()), std::max(0.0, b.x()), 0.0, 0.0});
    };
    for (std::size_t k = 0; k + 1 < vs.size(); ++k) {
        const Vec2 a = vs[k].p, b = vs[k + 1].p;
        if (const auto arc = geom2d::arcFromBulge(a, b, vs[k].bulge)) {
            if (std::abs(arc->center.x()) < 1e-9) {
                SolidPiece s;
                s.h0 = std::min(a.y(), b.y());
                s.h1 = std::max(a.y(), b.y());
                s.sphere = arc->radius;
                s.centre = arc->center.y();
                if (s.h1 - s.h0 > 1e-12) {
                    out.push_back(s);
                }
                continue;
            }
            const geom2d::Polyline seg({vs[k], {b, 0.0}}, false);
            const std::vector<Vec2> pts = seg.discretize(2e-3);
            for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
                frustum(pts[i], pts[i + 1]);
            }
        } else {
            frustum(a, b);
        }
    }
    return out;
}

/// The part of the pieces between heights lo and hi.
std::vector<SolidPiece> clipPieces(const std::vector<SolidPiece>& pieces, double lo, double hi) {
    std::vector<SolidPiece> out;
    for (SolidPiece p : pieces) {
        const double a = std::max(p.h0, lo), b = std::min(p.h1, hi);
        if (b - a <= 1e-9) {
            continue;
        }
        if (p.sphere <= 0.0) {
            auto r = [&](double h) {
                return p.r0 + (p.r1 - p.r0) * (h - p.h0) / (p.h1 - p.h0);
            };
            const double ra = r(a), rb = r(b);
            p.r0 = ra;
            p.r1 = rb;
        }
        p.h0 = a;
        p.h1 = b;
        out.push_back(p);
    }
    return out;
}

} // namespace

ToolShape ToolShape::fromTool(const tooling::Tool& tool, const tooling::Holder* holder) {
    return fromProfile(tooling::toolProfile(tool), tooling::visibleLength(tool), holder);
}

ToolShape ToolShape::fromProfile(const tooling::ToolProfile& profile, double stickout,
                                 const tooling::Holder* holder) {
    ToolShape s;
    s.profile_ = profile;
    const std::vector<Vec2> poly = solidOutline(profile);
    const double L = std::max(profile.cuttingLength, 1e-6);
    s.cuttingLength_ = L;
    s.visibleLength_ = stickout > 0.0 ? stickout : profile.length;

    // Radius at the boundaries of equal height cells over [0, L].
    s.radiusTable_.resize(kHeightCells + 1);
    double rmax = 0.0;
    for (int k = 0; k <= kHeightCells; ++k) {
        const double h = std::min(L * k / kHeightCells, L * (1.0 - 1e-9));
        const double r = radiusAtHeight(poly, h);
        s.radiusTable_[static_cast<std::size_t>(k)] = static_cast<float>(r);
        rmax = std::max(rmax, r);
    }
    s.cutRadius_ = std::max(rmax, std::min(profile.maxRadius, rmax * 1.001 + 1e-9));

    // Height spans of the cutting part at sampled radii.
    s.spanTable_.resize(kSpanSamples);
    for (int k = 0; k < kSpanSamples; ++k) {
        const double d = std::clamp(s.cutRadius_ * k / (kSpanSamples - 1), s.cutRadius_ * 1e-7,
                                    s.cutRadius_ * (1.0 - 1e-7));
        const std::vector<double> hs = crossingsAtRadius(poly, d);
        HeightSpans spans;
        for (std::size_t c = 0; c + 1 < hs.size(); c += 2) {
            const double lo = std::max(0.0, hs[c]);
            const double hi = std::min(L, hs[c + 1]);
            if (hi <= lo) {
                continue;
            }
            if (spans.count < static_cast<int>(spans.spans.size())) {
                spans.spans[static_cast<std::size_t>(spans.count++)] = {static_cast<float>(lo),
                                                                        static_cast<float>(hi)};
            } else {
                spans.spans.back().hi = static_cast<float>(hi); // merge overflow (conservative)
            }
        }
        s.spanTable_[static_cast<std::size_t>(k)] = spans;
        s.singleSpan_ = s.singleSpan_ && spans.count <= 1;
    }

    // Non-cutting body between the cutting length and the holder face.
    const double shankTop = std::min(s.visibleLength_, profile.length);
    if (shankTop > L) {
        for (int k = 0; k <= 64; ++k) {
            const double h = std::min(L + (shankTop - L) * k / 64.0, shankTop * (1.0 - 1e-9));
            s.shankRadius_ = std::max(s.shankRadius_, radiusAtHeight(poly, h));
        }
        // Bands between the outline's corners: within one, the radius runs straight, so its
        // largest is at an end.
        std::vector<double> cuts{L, shankTop};
        for (const Vec2& p : poly) {
            if (p.y() > L + 1e-9 && p.y() < shankTop - 1e-9) {
                cuts.push_back(p.y());
            }
        }
        std::sort(cuts.begin(), cuts.end());
        for (std::size_t k = 0; k + 1 < cuts.size(); ++k) {
            const double h0 = cuts[k];
            const double h1 = cuts[k + 1];
            if (h1 - h0 < 1e-9) {
                continue;
            }
            const double r = std::max({radiusAtHeight(poly, h0 + (h1 - h0) * 1e-6),
                                       radiusAtHeight(poly, 0.5 * (h0 + h1)),
                                       radiusAtHeight(poly, h1 - (h1 - h0) * 1e-6)});
            if (!s.shankBands_.empty() && std::abs(s.shankBands_.back().radius - r) < 1e-12) {
                s.shankBands_.back().h1 = h1;
            } else {
                s.shankBands_.push_back({h0, h1, r});
            }
        }
    }

    const std::vector<SolidPiece> body = solidPieces(profile);
    s.cutterPieces_ = clipPieces(body, 0.0, L);
    if (shankTop > L) {
        s.shankPieces_ = clipPieces(body, L, shankTop);
    }

    if (holder) {
        const std::vector<Vec2> hp = holder->profile.discretize(0.01);
        for (std::size_t k = 0; k + 1 < hp.size(); ++k) {
            const double h0 = std::min(hp[k].y(), hp[k + 1].y());
            const double h1 = std::max(hp[k].y(), hp[k + 1].y());
            if (h1 - h0 < 1e-9) {
                continue;
            }
            const double r = std::max(hp[k].x(), hp[k + 1].x());
            s.holder_.push_back({s.visibleLength_ + h0, s.visibleLength_ + h1, r});
            s.holderPieces_.push_back(
                {s.visibleLength_ + h0, s.visibleLength_ + h1, r, r, 0.0, 0.0});
            s.holderRadius_ = std::max(s.holderRadius_, r);
        }
    }
    return s;
}

HeightSpans ToolShape::spansAt(double d) const noexcept {
    if (d > cutRadius_ || spanTable_.empty()) {
        return {};
    }
    const double t = std::max(0.0, d) / cutRadius_ * (kSpanSamples - 1);
    const auto k = static_cast<std::size_t>(t);
    if (k >= static_cast<std::size_t>(kSpanSamples - 1)) {
        return spanTable_.back();
    }
    const HeightSpans& a = spanTable_[k];
    const HeightSpans& b = spanTable_[k + 1];
    const double f = t - static_cast<double>(k);
    if (a.count != b.count) {
        return f < 0.5 ? a : b;
    }
    HeightSpans out = a;
    for (std::size_t c = 0; c < static_cast<std::size_t>(a.count); ++c) {
        out.spans[c].lo = static_cast<float>(a.spans[c].lo + f * (b.spans[c].lo - a.spans[c].lo));
        out.spans[c].hi = static_cast<float>(a.spans[c].hi + f * (b.spans[c].hi - a.spans[c].hi));
    }
    return out;
}

double ToolShape::maxRadiusIn(double h0, double h1) const noexcept {
    const double L = cuttingLength_;
    h0 = std::max(h0, 0.0);
    h1 = std::min(h1, L);
    if (h1 < h0 || radiusTable_.empty()) {
        return 0.0;
    }
    auto at = [&](double h) {
        const double t = h / L * kHeightCells;
        const auto k =
            std::min(static_cast<std::size_t>(t), static_cast<std::size_t>(kHeightCells - 1));
        const double f = t - static_cast<double>(k);
        return radiusTable_[k] + f * (radiusTable_[k + 1] - radiusTable_[k]);
    };
    double r = std::max(at(h0), at(h1));
    const auto k0 = static_cast<std::size_t>(std::ceil(h0 / L * kHeightCells));
    const auto k1 = static_cast<std::size_t>(std::floor(h1 / L * kHeightCells));
    for (std::size_t k = k0; k <= k1 && k <= static_cast<std::size_t>(kHeightCells); ++k) {
        r = std::max(r, static_cast<double>(radiusTable_[k]));
    }
    return r;
}

} // namespace ocam::sim
