// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Rulings of a face for swarf milling (docs/design-log/0080). A family of straight lines in the
// face's parameter plane (the parameter lines of one direction, or for a plane the lines along
// its steepest direction) is clipped to the face with the 2D face classifier, checked for
// straightness in 3D, and sampled across the face densely enough that the tool positions offset
// from the rulings stay within the chord tolerance (Pottmann and Wallner 2001 on ruled surfaces).

#include "ocam/cad/Ruled.hpp"

#include "ModelImpl.hpp"

#include <fmt/format.h>

#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <algorithm>
#include <cmath>
#include <gp_Pln.hxx>
#include <gp_Pnt2d.hxx>
#include <limits>
#include <optional>

namespace ocam::cad {
namespace {

/// Lines uv(s, t) = s * across + t * along in the face's parameter plane.
struct Family {
    Vec2 along = Vec2::UnitY();
    Vec2 across = Vec2::UnitX();
    double s0 = 0, s1 = 0, t0 = 0, t1 = 0;
};

struct Line {
    double s = 0;
    double ta = 0, tb = 0;
    FaceRuling ruling;
};

class Sampler {
public:
    Sampler(const TopoDS_Face& face, const Transform3& toFrame)
        : face_(face), surf_(face, true), classifier_(face, Precision::PConfusion()),
          toFrame_(toFrame), reversed_(face.Orientation() == TopAbs_REVERSED) {}

    const BRepAdaptor_Surface& surface() const { return surf_; }

    bool inside(const Vec2& uv) const {
        const TopAbs_State st = classifier_.Perform(gp_Pnt2d(uv.x(), uv.y()));
        return st == TopAbs_IN || st == TopAbs_ON;
    }

    /// Point, outward normal and the derivative along `along`, in the target frame.
    struct Eval {
        Vec3 point;
        Vec3 normal;
        Vec3 tangent;
    };
    Eval eval(const Vec2& uv, const Vec2& along) const {
        gp_Pnt p;
        gp_Vec du, dv;
        surf_.D1(uv.x(), uv.y(), p, du, dv);
        gp_Vec n = du.Crossed(dv);
        if (n.Magnitude() < 1e-12) {
            // A degenerate point (a cone's apex, a pole): the normal a hair inside.
            const Vec2 c((surf_.FirstUParameter() + surf_.LastUParameter()) / 2,
                         (surf_.FirstVParameter() + surf_.LastVParameter()) / 2);
            const Vec2 q = uv + 1e-6 * (c - uv);
            gp_Pnt p2;
            surf_.D1(q.x(), q.y(), p2, du, dv);
            n = du.Crossed(dv);
        }
        Vec3 normal =
            n.Magnitude() > 1e-15 ? Vec3(n.X(), n.Y(), n.Z()).normalized() : Vec3::UnitZ();
        if (reversed_) {
            normal = -normal;
        }
        const gp_Vec d = du * along.x() + dv * along.y();
        return {toFrame_ * Vec3(p.X(), p.Y(), p.Z()), toFrame_.linear() * normal,
                toFrame_.linear() * Vec3(d.X(), d.Y(), d.Z())};
    }

    /// The part of line s inside the face: nullopt when it misses; `split` when it crosses the
    /// face more than once (a hole or a notch across the rulings).
    std::optional<std::pair<double, double>> clip(const Family& f, double s, bool& split) const {
        constexpr int kSteps = 96;
        const auto uv = [&](double t) {
            return Vec2(s * f.across + t * f.along);
        };
        const auto tAt = [&](int i) {
            return f.t0 + (f.t1 - f.t0) * i / kSteps;
        };
        int first = -1, last = -1, runs = 0;
        bool previous = false;
        for (int i = 0; i <= kSteps; ++i) {
            const bool in = inside(uv(tAt(i)));
            if (in && !previous) {
                ++runs;
                if (first < 0) {
                    first = i;
                }
            }
            if (in) {
                last = i;
            }
            previous = in;
        }
        if (first < 0) {
            return std::nullopt;
        }
        split = split || runs > 1;
        // Bisect the ends onto the boundary.
        const auto edge = [&](double inT, double outT) {
            for (int k = 0; k < 40; ++k) {
                const double m = 0.5 * (inT + outT);
                (inside(uv(m)) ? inT : outT) = m;
            }
            return inT;
        };
        const double ta = first == 0 ? f.t0 : edge(tAt(first), tAt(first - 1));
        const double tb = last == kSteps ? f.t1 : edge(tAt(last), tAt(last + 1));
        if (tb - ta < 1e-12 * std::max(1.0, f.t1 - f.t0)) {
            return std::nullopt;
        }
        return std::make_pair(ta, tb);
    }

    /// The largest distance of the face from the chord of line s.
    double bow(const Family& f, double s, std::pair<double, double> range) const {
        const auto at = [&](double t) {
            return eval(Vec2(s * f.across + t * f.along), f.along);
        };
        const Vec3 a = at(range.first).point;
        const Vec3 b = at(range.second).point;
        const Vec3 ab = b - a;
        const double len = ab.norm();
        double worst = 0.0;
        constexpr int kSteps = 8;
        for (int i = 1; i < kSteps; ++i) {
            const Vec3 p = at(range.first + (range.second - range.first) * i / kSteps).point;
            const double d = len > 1e-12 ? (p - a).cross(ab).norm() / len : (p - a).norm();
            worst = std::max(worst, d);
        }
        return worst;
    }

private:
    const TopoDS_Face& face_;
    BRepAdaptor_Surface surf_;
    BRepTopAdaptor_FClass2d classifier_;
    Transform3 toFrame_;
    bool reversed_;
};

} // namespace

FaceRulings faceRulings(const Model& model, int faceIndex, const Transform3& toFrame,
                        const RulingOptions& o) {
    FaceRulings out;
    ShapeIndexMap faces;
    TopExp::MapShapes(model.impl().shape, TopAbs_FACE, faces);
    if (faceIndex < 1 || faceIndex > faces.Extent()) {
        out.problem = "the face does not exist";
        return out;
    }
    const TopoDS_Face face = TopoDS::Face(faces(faceIndex));
    const Sampler sampler(face, toFrame);
    const BRepAdaptor_Surface& surf = sampler.surface();
    const Vec3 up = o.up.normalized();
    const int samples = std::max(2, o.samples);

    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
    BRepTools::UVBounds(face, u0, u1, v0, v1);

    // Candidate families.
    std::vector<Family> families;
    out.planar = surf.GetType() == GeomAbs_Plane;
    if (out.planar) {
        const gp_Pln pln = surf.Plane();
        const gp_Ax3 pos = pln.Position();
        const Vec3 x = toFrame.linear() *
                       Vec3(pos.XDirection().X(), pos.XDirection().Y(), pos.XDirection().Z());
        const Vec3 y = toFrame.linear() *
                       Vec3(pos.YDirection().X(), pos.YDirection().Y(), pos.YDirection().Z());
        const Vec2 steep(up.dot(x), up.dot(y)); // up projected on the plane, in (u, v)
        if (steep.norm() < 1e-6) {
            out.problem = "the face is square to the tool axis: a floor, not a wall";
            return out;
        }
        Family f;
        f.along = steep.normalized();
        f.across = Vec2(-f.along.y(), f.along.x());
        f.s0 = f.t0 = std::numeric_limits<double>::infinity();
        f.s1 = f.t1 = -std::numeric_limits<double>::infinity();
        for (const Vec2 c : {Vec2(u0, v0), Vec2(u1, v0), Vec2(u0, v1), Vec2(u1, v1)}) {
            f.s0 = std::min(f.s0, c.dot(f.across));
            f.s1 = std::max(f.s1, c.dot(f.across));
            f.t0 = std::min(f.t0, c.dot(f.along));
            f.t1 = std::max(f.t1, c.dot(f.along));
        }
        families.push_back(f);
    } else {
        Family alongV; // u fixed, v varies
        alongV.along = Vec2::UnitY();
        alongV.across = Vec2::UnitX();
        alongV.s0 = u0, alongV.s1 = u1, alongV.t0 = v0, alongV.t1 = v1;
        Family alongU;
        alongU.along = Vec2::UnitX();
        alongU.across = Vec2::UnitY();
        alongU.s0 = v0, alongU.s1 = v1, alongU.t0 = u0, alongU.t1 = u1;
        families = {alongV, alongU};
    }

    // Each family's bow and slope, from lines across it.
    struct Score {
        double bow = std::numeric_limits<double>::infinity();
        double slope = 0.0; ///< Mean |cos| of the lines with `up`.
        bool split = false;
    };
    std::vector<Score> scores(families.size());
    for (std::size_t i = 0; i < families.size(); ++i) {
        const Family& f = families[i];
        Score& sc = scores[i];
        constexpr int kLines = 9;
        int counted = 0;
        double bow = 0.0, slope = 0.0;
        for (int k = 0; k < kLines; ++k) {
            const double s = f.s0 + (f.s1 - f.s0) * (k + 0.5) / kLines;
            const auto range = sampler.clip(f, s, sc.split);
            if (!range) {
                continue;
            }
            ++counted;
            bow = std::max(bow, sampler.bow(f, s, *range));
            const auto at = [&](double t) {
                return sampler.eval(Vec2(s * f.across + t * f.along), f.along).point;
            };
            const Vec3 d = at(range->second) - at(range->first);
            if (d.norm() > 1e-12) {
                slope += std::abs(d.normalized().dot(up));
            }
        }
        if (counted > 0) {
            sc.bow = bow;
            sc.slope = slope / counted;
        }
    }
    std::size_t pick = 0;
    for (std::size_t i = 1; i < families.size(); ++i) {
        const bool okI = scores[i].bow <= o.tolerance;
        const bool okPick = scores[pick].bow <= o.tolerance;
        if ((okI && !okPick) || (okI && okPick && scores[i].slope > scores[pick].slope + 1e-9) ||
            (!okI && !okPick && scores[i].bow < scores[pick].bow)) {
            pick = i;
        }
    }
    const Family& f = families[pick];
    out.straightness = scores[pick].bow;
    if (!std::isfinite(out.straightness)) {
        out.problem = "the face could not be sampled";
        return out;
    }
    if (out.straightness > o.tolerance) {
        out.problem = fmt::format("the face is not ruled: it strays {:.3f} mm from straight lines "
                                  "(tolerance {:.3f} mm)",
                                  out.straightness, o.tolerance);
        return out;
    }
    if (scores[pick].split) {
        out.problem = "a hole or a notch crosses the face's rulings";
        return out;
    }

    // Where the face starts and ends across the lines (a plane's box is larger than the face).
    bool split = false;
    constexpr int kScan = 200;
    const double span = f.s1 - f.s0;
    const double inset = 1e-9 * std::max(1.0, span);
    const auto sAt = [&](int i) {
        return f.s0 + inset + (span - 2 * inset) * i / kScan;
    };
    int firstIn = -1, lastIn = -1;
    for (int i = 0; i <= kScan; ++i) {
        if (sampler.clip(f, sAt(i), split)) {
            if (firstIn < 0) {
                firstIn = i;
            }
            lastIn = i;
        }
    }
    if (firstIn < 0) {
        out.problem = "the face could not be sampled";
        return out;
    }
    const auto edgeS = [&](double inS, double outS) {
        for (int k = 0; k < 40; ++k) {
            const double m = 0.5 * (inS + outS);
            bool ignored = false;
            (sampler.clip(f, m, ignored) ? inS : outS) = m;
        }
        return inS;
    };
    const double sA = firstIn == 0 ? sAt(0) : edgeS(sAt(firstIn), sAt(firstIn - 1));
    const double sB = lastIn == kScan ? sAt(kScan) : edgeS(sAt(lastIn), sAt(lastIn + 1));

    // Which way is up along the lines, once for the family.
    bool flip = false;
    {
        const double sm = 0.5 * (sA + sB);
        bool ignored = false;
        if (const auto r = sampler.clip(f, sm, ignored)) {
            const auto at = [&](double t) {
                return sampler.eval(Vec2(sm * f.across + t * f.along), f.along);
            };
            const Vec3 d = at(r->second).point - at(r->first).point;
            flip = (d.norm() > 1e-9 ? d : at(0.5 * (r->first + r->second)).tangent).dot(up) < 0;
        }
    }

    const auto make = [&](double s) -> std::optional<Line> {
        bool ignored = false;
        const auto range = sampler.clip(f, s, ignored);
        if (!range) {
            return std::nullopt;
        }
        Line line;
        line.s = s;
        line.ta = range->first;
        line.tb = range->second;
        for (int k = 0; k < samples; ++k) {
            const int j = flip ? samples - 1 - k : k;
            const double t = line.ta + (line.tb - line.ta) * j / (samples - 1);
            const Sampler::Eval e = sampler.eval(Vec2(s * f.across + t * f.along), f.along);
            line.ruling.points.push_back(e.point);
            line.ruling.normals.push_back(e.normal);
        }
        const Vec3 d = line.ruling.points.back() - line.ruling.points.front();
        if (d.norm() > 1e-6) {
            line.ruling.direction = d.normalized();
        } else {
            const Vec3 t =
                sampler.eval(Vec2(s * f.across + 0.5 * (line.ta + line.tb) * f.along), f.along)
                    .tangent;
            line.ruling.direction = (flip ? -t : t).normalized();
        }
        return line;
    };
    const auto needsMiddle = [&](const Line& a, const Line& b, const Line& m) {
        const auto& pa = a.ruling.points;
        const auto& pb = b.ruling.points;
        if ((pa.front() - pb.front()).norm() > o.maxSpacing ||
            (pa.back() - pb.back()).norm() > o.maxSpacing) {
            return true;
        }
        for (std::size_t k = 0; k < pa.size(); ++k) {
            const Vec3 qa = pa[k] + o.offset * a.ruling.normals[k];
            const Vec3 qb = pb[k] + o.offset * b.ruling.normals[k];
            const Vec3 qm = m.ruling.points[k] + o.offset * m.ruling.normals[k];
            if ((0.5 * (qa + qb) - qm).norm() > o.chordTolerance ||
                (0.5 * (pa[k] + pb[k]) - m.ruling.points[k]).norm() > o.chordTolerance) {
                return true;
            }
        }
        return false;
    };
    std::vector<Line> lines;
    const auto refine = [&](auto&& self, const Line& a, const Line& b, int depth) -> void {
        const double sm = 0.5 * (a.s + b.s);
        const std::optional<Line> m = make(sm);
        if (!m || depth > 16 || !needsMiddle(a, b, *m)) {
            lines.push_back(b);
            return;
        }
        self(self, a, *m, depth + 1);
        self(self, *m, b, depth + 1);
    };
    constexpr int kStart = 8;
    std::optional<Line> previous;
    for (int i = 0; i <= kStart; ++i) {
        const std::optional<Line> line = make(sA + (sB - sA) * i / kStart);
        if (!line) {
            continue;
        }
        if (!previous) {
            lines.push_back(*line);
        } else {
            refine(refine, *previous, *line, 0);
        }
        previous = line;
    }
    for (Line& line : lines) {
        out.rulings.push_back(std::move(line.ruling));
    }
    return out;
}

} // namespace ocam::cad
