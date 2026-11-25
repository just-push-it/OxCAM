// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Drop-cutter for rotating tools with a convex lower envelope lo(d). The tool, placed with its
// tip at height h above (x, y), is the solid {z >= h + lo(d)} within radius R. The lowest
// non-gouging tip height is the largest value of p.z - lo(d(p)) over all points p of the mesh
// within radius R. Per triangle the maximum lies at a vertex, inside the facet (where the
// tool's surface slope equals the facet slope: Cutter::facetContact) or on an edge, where
// z(t) - lo(d(t)) is concave in t (lo convex and non-decreasing, d convex) and a golden-section
// search finds it. See docs/design-log/0016.

#include "Internal.hpp"
#include "ocam/strategies/Surface.hpp"

#include <ocam/geom2d/Region.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>

namespace ocam::strategies {

// -- Cutter -----------------------------------------------------------------------------------

Cutter Cutter::torus(double radius, double cornerRadius) {
    if (radius <= 0.0 || cornerRadius < 0.0 || cornerRadius > radius + 1e-12) {
        throw Error(ErrorCode::InvalidArgument, "invalid torus cutter");
    }
    Cutter c;
    c.kind_ = Kind::Torus;
    c.radius_ = radius;
    c.corner_ = std::min(cornerRadius, radius);
    return c;
}

Cutter Cutter::cone(double radius, double tipRadius, double halfAngle) {
    if (radius <= 0.0 || tipRadius < 0.0 || tipRadius > radius || halfAngle <= 0.0 ||
        halfAngle >= kPi / 2.0) {
        throw Error(ErrorCode::InvalidArgument, "invalid conical cutter");
    }
    Cutter c;
    c.kind_ = Kind::Cone;
    c.radius_ = radius;
    c.tipRadius_ = tipRadius;
    c.tanHalf_ = std::tan(halfAngle);
    return c;
}

Cutter::Cutter(const tooling::Tool& tool) {
    using tooling::ToolType;
    const double r = detail::requireMillingTool(tool);
    const auto& g = tool.mill;
    switch (tool.type) {
    case ToolType::FlatEndMill:
        *this = torus(r, 0.0);
        break;
    case ToolType::BallEndMill:
        *this = torus(r, r);
        break;
    case ToolType::BullNoseEndMill:
        *this = torus(r, g.cornerRadius);
        break;
    case ToolType::Chamfer:
    case ToolType::Engraver:
        *this = cone(r, g.tipDiameter / 2.0, g.tipAngle / 2.0);
        break;
    case ToolType::Drill:
    case ToolType::SpotDrill:
    case ToolType::CenterDrill:
        *this = cone(r, 0.0, g.tipAngle / 2.0);
        break;
    default:
        throw Error(ErrorCode::Unsupported,
                    "3D strategies support flat, ball, bull-nose and conical tools");
    }
}

double Cutter::flatRadius() const noexcept {
    return kind_ == Kind::Torus ? radius_ - corner_ : tipRadius_;
}

double Cutter::lo(double d) const noexcept {
    d = std::clamp(d, 0.0, radius_);
    if (kind_ == Kind::Torus) {
        const double a = radius_ - corner_;
        if (d <= a || corner_ <= 0.0) {
            return 0.0;
        }
        const double t = std::min(d - a, corner_);
        return corner_ - std::sqrt(std::max(0.0, corner_ * corner_ - t * t));
    }
    return d <= tipRadius_ ? 0.0 : (d - tipRadius_) / tanHalf_;
}

std::pair<double, double> Cutter::facetContact(double s) const noexcept {
    if (kind_ == Kind::Torus) {
        if (s <= 1e-15) {
            return {0.0, 0.0};
        }
        if (corner_ <= 0.0) {
            return {radius_, s * radius_};
        }
        const double r = (radius_ - corner_) + corner_ * s / std::sqrt(1.0 + s * s);
        return {r, s * r - lo(r)};
    }
    if (s * tanHalf_ > 1.0) {
        return {radius_, s * radius_ - lo(radius_)};
    }
    return {tipRadius_, s * tipRadius_};
}

Cutter Cutter::offset(double amount) const {
    if (amount == 0.0) {
        return *this;
    }
    if (kind_ != Kind::Torus) {
        throw Error(ErrorCode::Unsupported, "stock to leave needs a flat, ball or bull-nose tool");
    }
    if (amount < 0.0 && -amount >= corner_) {
        throw Error(ErrorCode::InvalidArgument,
                    "negative stock to leave exceeds the corner radius");
    }
    return torus(radius_ + amount, corner_ + amount);
}

// -- DropCutter -------------------------------------------------------------------------------

namespace {

struct Tri {
    std::array<Vec3, 3> p;
    double maxZ = 0.0;
    geom2d::Aabb2 box;
};

double distToSegment2(const Vec2& c, const Vec2& a, const Vec2& b) {
    const Vec2 e = b - a;
    const double l2 = e.squaredNorm();
    const double t = l2 > 0.0 ? std::clamp((c - a).dot(e) / l2, 0.0, 1.0) : 0.0;
    return (a + t * e - c).norm();
}

double cross2(const Vec2& a, const Vec2& b) {
    return a.x() * b.y() - a.y() * b.x();
}

bool insideTriangle(const Vec2& q, const Vec2& a, const Vec2& b, const Vec2& c) {
    const double d1 = cross2(b - a, q - a);
    const double d2 = cross2(c - b, q - b);
    const double d3 = cross2(a - c, q - c);
    const double eps = 1e-12 * std::max(1.0, std::abs(cross2(b - a, c - a)));
    const bool neg = d1 < -eps || d2 < -eps || d3 < -eps;
    const bool pos = d1 > eps || d2 > eps || d3 > eps;
    return !(neg && pos);
}

} // namespace

struct DropCutter::Impl {
    Cutter cutter;
    std::vector<Tri> tris;
    geom3d::Aabb3 bounds;
    Vec2 origin = Vec2::Zero();
    double cell = 1.0;
    int nx = 0;
    int ny = 0;
    std::vector<std::vector<std::uint32_t>> cells;

    explicit Impl(const Cutter& c) : cutter(c) {}

    double contact(const Tri& t, const Vec2& c, double best) const {
        const double R = cutter.radius();
        // Vertices.
        for (const Vec3& v : t.p) {
            const double d = (v.head<2>() - c).norm();
            if (d <= R) {
                best = std::max(best, v.z() - cutter.lo(d));
            }
        }
        // Facet interior.
        const Vec3 n = (t.p[1] - t.p[0]).cross(t.p[2] - t.p[0]);
        if (std::abs(n.z()) > 1e-12 * n.norm()) {
            const Vec2 g(-n.x() / n.z(), -n.y() / n.z());
            const double s = g.norm();
            const auto [r, value] = cutter.facetContact(s);
            const Vec2 q = s > 1e-15 ? Vec2(c + g * (r / s)) : c;
            if (insideTriangle(q, t.p[0].head<2>(), t.p[1].head<2>(), t.p[2].head<2>())) {
                const double zc = t.p[0].z() + g.dot(c - t.p[0].head<2>());
                best = std::max(best, zc + value);
            }
        }
        // Edges.
        for (int k = 0; k < 3; ++k) {
            const Vec3& a = t.p[static_cast<std::size_t>(k)];
            const Vec3& b = t.p[static_cast<std::size_t>((k + 1) % 3)];
            const Vec2 e = b.head<2>() - a.head<2>();
            const double l2 = e.squaredNorm();
            if (l2 < 1e-18) {
                continue; // vertical edge: its vertices decide
            }
            const double dmin = distToSegment2(c, a.head<2>(), b.head<2>());
            if (dmin > R || std::max(a.z(), b.z()) - cutter.lo(dmin) <= best) {
                continue;
            }
            const Vec2 w = a.head<2>() - c;
            const double B = 2.0 * w.dot(e);
            const double C = w.squaredNorm() - R * R;
            const double disc = B * B - 4.0 * l2 * C;
            if (disc < 0.0) {
                continue;
            }
            const double sq = std::sqrt(disc);
            double t0 = std::max(0.0, (-B - sq) / (2.0 * l2));
            double t1 = std::min(1.0, (-B + sq) / (2.0 * l2));
            if (t1 < t0) {
                continue;
            }
            auto f = [&](double t) {
                return a.z() + t * (b.z() - a.z()) - cutter.lo((w + t * e).norm());
            };
            best = std::max({best, f(t0), f(t1)});
            constexpr double phi = 0.6180339887498949;
            double x1 = t1 - phi * (t1 - t0);
            double x2 = t0 + phi * (t1 - t0);
            double f1 = f(x1), f2 = f(x2);
            for (int it = 0; it < 48 && t1 - t0 > 1e-12; ++it) {
                if (f1 < f2) {
                    t0 = x1;
                    x1 = x2;
                    f1 = f2;
                    x2 = t0 + phi * (t1 - t0);
                    f2 = f(x2);
                } else {
                    t1 = x2;
                    x2 = x1;
                    f2 = f1;
                    x1 = t1 - phi * (t1 - t0);
                    f1 = f(x1);
                }
            }
            best = std::max({best, f1, f2});
        }
        return best;
    }
};

DropCutter::DropCutter(const geom3d::Mesh& mesh, const Cutter& cutter)
    : impl_(std::make_unique<Impl>(cutter)) {
    Impl& m = *impl_;
    m.bounds = mesh.bounds();
    m.tris.reserve(mesh.triangles.size());
    for (const auto& t : mesh.triangles) {
        Tri tri;
        for (int k = 0; k < 3; ++k) {
            tri.p[static_cast<std::size_t>(k)] = mesh.vertices[t[static_cast<std::size_t>(k)]];
            tri.box.extend(tri.p[static_cast<std::size_t>(k)].head<2>());
        }
        tri.maxZ = std::max({tri.p[0].z(), tri.p[1].z(), tri.p[2].z()});
        m.tris.push_back(tri);
    }
    if (m.tris.empty()) {
        return;
    }
    const Vec2 size = m.bounds.size().head<2>();
    m.cell = std::max({cutter.radius(), size.x() / 512.0, size.y() / 512.0, 1e-3});
    m.origin = m.bounds.min.head<2>();
    m.nx = static_cast<int>(size.x() / m.cell) + 1;
    m.ny = static_cast<int>(size.y() / m.cell) + 1;
    m.cells.resize(static_cast<std::size_t>(m.nx) * static_cast<std::size_t>(m.ny));
    for (std::size_t k = 0; k < m.tris.size(); ++k) {
        const auto& b = m.tris[k].box;
        const int i0 =
            std::clamp(static_cast<int>((b.min.x() - m.origin.x()) / m.cell), 0, m.nx - 1);
        const int i1 =
            std::clamp(static_cast<int>((b.max.x() - m.origin.x()) / m.cell), 0, m.nx - 1);
        const int j0 =
            std::clamp(static_cast<int>((b.min.y() - m.origin.y()) / m.cell), 0, m.ny - 1);
        const int j1 =
            std::clamp(static_cast<int>((b.max.y() - m.origin.y()) / m.cell), 0, m.ny - 1);
        for (int j = j0; j <= j1; ++j) {
            for (int i = i0; i <= i1; ++i) {
                m.cells[static_cast<std::size_t>(j) * static_cast<std::size_t>(m.nx) +
                        static_cast<std::size_t>(i)]
                    .push_back(static_cast<std::uint32_t>(k));
            }
        }
    }
}

DropCutter::~DropCutter() = default;
DropCutter::DropCutter(DropCutter&&) noexcept = default;
DropCutter& DropCutter::operator=(DropCutter&&) noexcept = default;

const Cutter& DropCutter::cutter() const noexcept {
    return impl_->cutter;
}

const geom3d::Aabb3& DropCutter::bounds() const noexcept {
    return impl_->bounds;
}

double DropCutter::drop(const Vec2& c, double floor) const {
    const Impl& m = *impl_;
    if (m.tris.empty()) {
        return floor;
    }
    const double R = m.cutter.radius();
    const int i0 = static_cast<int>(std::floor((c.x() - R - m.origin.x()) / m.cell));
    const int i1 = static_cast<int>(std::floor((c.x() + R - m.origin.x()) / m.cell));
    const int j0 = static_cast<int>(std::floor((c.y() - R - m.origin.y()) / m.cell));
    const int j1 = static_cast<int>(std::floor((c.y() + R - m.origin.y()) / m.cell));
    if (i1 < 0 || j1 < 0 || i0 >= m.nx || j0 >= m.ny) {
        return floor;
    }
    thread_local std::vector<std::uint32_t> candidates;
    candidates.clear();
    for (int j = std::max(0, j0); j <= std::min(m.ny - 1, j1); ++j) {
        for (int i = std::max(0, i0); i <= std::min(m.nx - 1, i1); ++i) {
            const auto& cell =
                m.cells[static_cast<std::size_t>(j) * static_cast<std::size_t>(m.nx) +
                        static_cast<std::size_t>(i)];
            candidates.insert(candidates.end(), cell.begin(), cell.end());
        }
    }
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    // Highest triangles first: a triangle cannot raise the tool above its highest vertex.
    std::sort(candidates.begin(), candidates.end(),
              [&](std::uint32_t a, std::uint32_t b) { return m.tris[a].maxZ > m.tris[b].maxZ; });
    double best = floor;
    for (const std::uint32_t id : candidates) {
        const Tri& t = m.tris[id];
        if (t.maxZ <= best) {
            break;
        }
        const geom2d::Aabb2& b = t.box;
        const double dx = std::max({b.min.x() - c.x(), 0.0, c.x() - b.max.x()});
        const double dy = std::max({b.min.y() - c.y(), 0.0, c.y() - b.max.y()});
        if (dx * dx + dy * dy > R * R) {
            continue;
        }
        best = m.contact(t, c, best);
    }
    return best;
}

// -- Height field and forbidden regions ------------------------------------------------------

HeightField computeHeightField(const DropCutter& dc, const geom2d::Aabb2& area, double spacing,
                               double floor, CancelToken& cancel) {
    if (spacing <= 0.0 || area.isEmpty()) {
        throw Error(ErrorCode::InvalidArgument, "invalid height-field area or spacing");
    }
    HeightField f;
    f.origin = area.min;
    f.spacing = spacing;
    f.nx = static_cast<int>(std::ceil(area.size().x() / spacing)) + 1;
    f.ny = static_cast<int>(std::ceil(area.size().y() / spacing)) + 1;
    if (static_cast<double>(f.nx) * f.ny > 2.5e7) {
        throw Error(ErrorCode::InvalidArgument,
                    "height-field resolution is too fine for the part size");
    }
    f.z.resize(static_cast<std::size_t>(f.nx) * static_cast<std::size_t>(f.ny));
    tbb::parallel_for(tbb::blocked_range<int>(0, f.ny), [&](const tbb::blocked_range<int>& rows) {
        for (int j = rows.begin(); j != rows.end(); ++j) {
            if (cancel.isCancelled()) {
                return;
            }
            for (int i = 0; i < f.nx; ++i) {
                f.z[static_cast<std::size_t>(j) * static_cast<std::size_t>(f.nx) +
                    static_cast<std::size_t>(i)] = dc.drop(f.point(i, j), floor);
            }
        }
    });
    cancel.throwIfCancelled();
    return f;
}

geom2d::Regions forbiddenRegions(const DropCutter& dc, const HeightField& field, double z,
                                 double floor, double tolerance) {
    const int nx = field.nx;
    const int ny = field.ny;
    const double h = field.spacing;
    auto inside = [&](int i, int j) {
        return i >= 0 && j >= 0 && i < nx && j < ny && field.at(i, j) > z;
    };
    auto heightAt = [&](const Vec2& p) {
        return dc.drop(p, floor);
    };
    auto exactInside = [&](const Vec2& p) {
        return heightAt(p) > z;
    };
    // Edge keys over the padded lattice (-1..n).
    const std::int64_t stride = nx + 2;
    auto hKey = [&](int i, int j) {
        return ((static_cast<std::int64_t>(j) + 1) * stride + (i + 1)) * 2;
    };
    auto vKey = [&](int i, int j) {
        return ((static_cast<std::int64_t>(j) + 1) * stride + (i + 1)) * 2 + 1;
    };
    std::unordered_map<std::int64_t, Vec2> crossings;
    auto crossing = [&](std::int64_t key, int ia, int ja, int ib, int jb) {
        auto it = crossings.find(key);
        if (it != crossings.end()) {
            return it->second;
        }
        Vec2 pin = field.point(ia, ja);
        Vec2 pout = field.point(ib, jb);
        if (!inside(ia, ja)) {
            std::swap(pin, pout);
            std::swap(ia, ib);
            std::swap(ja, jb);
        }
        const bool padded = ib < 0 || jb < 0 || ib >= nx || jb >= ny;
        Vec2 p = 0.5 * (pin + pout);
        if (!padded) {
            // False position (Illinois) on the drop height, from the grid's own heights: a
            // smooth wall converges in two or three drops where halving took eight. Every
            // third step halves, so a wall the height jumps at (a vertical one) still closes in.
            double fin = field.at(ia, ja) - z;  // > 0
            double fout = field.at(ib, jb) - z; // <= 0
            int side = 0;
            for (int it2 = 0; it2 < 60 && (pin - pout).norm() > tolerance; ++it2) {
                double t = 0.5;
                if (it2 % 3 != 2 && fin - fout > 1e-15) {
                    t = std::clamp(fin / (fin - fout), 0.02, 0.98);
                }
                const Vec2 m = pin + (pout - pin) * t;
                const double fm = heightAt(m) - z;
                if (fm > 0.0) {
                    pin = m;
                    fin = fm;
                    if (side == 1) {
                        fout *= 0.5;
                    }
                    side = 1;
                } else {
                    pout = m;
                    fout = fm;
                    if (side == -1) {
                        fin *= 0.5;
                    }
                    side = -1;
                }
            }
            p = pout; // on the safe side
        }
        crossings.emplace(key, p);
        return p;
    };

    struct Segment {
        std::int64_t a, b;
    };
    std::vector<Segment> segments;
    for (int j = -1; j < ny; ++j) {
        for (int i = -1; i < nx; ++i) {
            const int code = (inside(i, j) ? 1 : 0) | (inside(i + 1, j) ? 2 : 0) |
                             (inside(i + 1, j + 1) ? 4 : 0) | (inside(i, j + 1) ? 8 : 0);
            if (code == 0 || code == 15) {
                continue;
            }
            const std::int64_t B = hKey(i, j), T = hKey(i, j + 1), L = vKey(i, j),
                               Rk = vKey(i + 1, j);
            auto edgePoint = [&](std::int64_t key) {
                if (key == B) {
                    return crossing(key, i, j, i + 1, j);
                }
                if (key == T) {
                    return crossing(key, i, j + 1, i + 1, j + 1);
                }
                if (key == L) {
                    return crossing(key, i, j, i, j + 1);
                }
                return crossing(key, i + 1, j, i + 1, j + 1);
            };
            auto add = [&](std::int64_t a, std::int64_t b) {
                edgePoint(a);
                edgePoint(b);
                segments.push_back({a, b});
            };
            const bool centre = (code == 5 || code == 10) &&
                                exactInside(field.point(i, j) + Vec2(0.5 * h, 0.5 * h));
            switch (code) {
            case 1:
                add(L, B);
                break;
            case 2:
                add(B, Rk);
                break;
            case 3:
                add(L, Rk);
                break;
            case 4:
                add(Rk, T);
                break;
            case 5:
                if (centre) {
                    add(B, Rk);
                    add(T, L);
                } else {
                    add(L, B);
                    add(Rk, T);
                }
                break;
            case 6:
                add(B, T);
                break;
            case 7:
                add(L, T);
                break;
            case 8:
                add(T, L);
                break;
            case 9:
                add(B, T);
                break;
            case 10:
                if (centre) {
                    add(L, B);
                    add(Rk, T);
                } else {
                    add(B, Rk);
                    add(T, L);
                }
                break;
            case 11:
                add(Rk, T);
                break;
            case 12:
                add(L, Rk);
                break;
            case 13:
                add(B, Rk);
                break;
            case 14:
                add(L, B);
                break;
            default:
                break;
            }
        }
    }

    // Chain segments into loops through their shared edge keys.
    std::unordered_map<std::int64_t, std::vector<std::size_t>> byKey;
    for (std::size_t s = 0; s < segments.size(); ++s) {
        byKey[segments[s].a].push_back(s);
        byKey[segments[s].b].push_back(s);
    }
    std::vector<char> used(segments.size(), 0);
    std::vector<geom2d::Polyline> loops;
    for (std::size_t s0 = 0; s0 < segments.size(); ++s0) {
        if (used[s0]) {
            continue;
        }
        std::vector<Vec2> pts;
        std::size_t s = s0;
        std::int64_t key = segments[s0].a;
        for (std::size_t guard = 0; guard <= segments.size(); ++guard) {
            used[s] = 1;
            pts.push_back(crossings.at(key));
            const std::int64_t next = segments[s].a == key ? segments[s].b : segments[s].a;
            key = next;
            std::size_t found = segments.size();
            for (const std::size_t cand : byKey[key]) {
                if (!used[cand]) {
                    found = cand;
                    break;
                }
            }
            if (found == segments.size()) {
                break;
            }
            s = found;
        }
        if (pts.size() < 3) {
            continue;
        }
        // Where the straight chord between two points cuts into the forbidden region, insert
        // the boundary point found across it.
        std::vector<Vec2> refined;
        std::function<void(const Vec2&, const Vec2&, int)> refine = [&](const Vec2& a,
                                                                        const Vec2& b, int depth) {
            const Vec2 m = 0.5 * (a + b);
            if (depth >= 4 || (b - a).norm() < 4.0 * tolerance || !exactInside(m)) {
                refined.push_back(b);
                return;
            }
            const Vec2 d = b - a;
            const Vec2 nrm = Vec2(-d.y(), d.x()).normalized();
            for (int k = 1; k <= 8; ++k) {
                for (const double side : {1.0, -1.0}) {
                    const Vec2 q = m + side * nrm * (k * h / 8.0);
                    if (!exactInside(q)) {
                        Vec2 pin = m, pout = q;
                        for (int it = 0; it < 30 && (pin - pout).norm() > tolerance; ++it) {
                            const Vec2 mm = 0.5 * (pin + pout);
                            if (exactInside(mm)) {
                                pin = mm;
                            } else {
                                pout = mm;
                            }
                        }
                        refine(a, pout, depth + 1);
                        refine(pout, b, depth + 1);
                        return;
                    }
                }
            }
            refined.push_back(b);
        };
        refined.push_back(pts.front());
        for (std::size_t k = 0; k < pts.size(); ++k) {
            refine(pts[k], pts[(k + 1) % pts.size()], 0);
        }
        refined.pop_back(); // the closing point repeats the first
        loops.push_back(geom2d::Polyline::fromPoints(refined, true));
    }
    return geom2d::buildRegions(std::move(loops), tolerance);
}

} // namespace ocam::strategies
