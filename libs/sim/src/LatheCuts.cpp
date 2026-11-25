// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// What a turning stock holds beside its revolved profile: thread passes and holes, as exact cuts.
// Their volume, and the surface of the whole. See docs/design-log/0061.

#include "ocam/sim/Lathe.hpp"
#include "ocam/sim/Surface.hpp"

#include <ocam/foundation/Math.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace ocam::sim {
namespace {

template <class P> Vec3 storedVec3(const P& p) {
    return Vec3(static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2]));
}

// -- Thread passes ----------------------------------------------------------------------------

/// A convex outline's lowest and highest v at each u.
struct Envelope {
    std::vector<double> us, lo, hi;

    explicit Envelope(const geom2d::Polyline& outline) {
        std::vector<Vec2> pts = outline.discretize(0.005);
        if (pts.size() > 1 && (pts.front() - pts.back()).norm() < 1e-9) {
            pts.pop_back();
        }
        for (const Vec2& p : pts) {
            us.push_back(p.x());
        }
        std::sort(us.begin(), us.end());
        us.erase(std::unique(us.begin(), us.end(),
                             [](double a, double b) { return std::abs(a - b) < 1e-7; }),
                 us.end());
        for (const double u : us) {
            double low = std::numeric_limits<double>::infinity();
            double high = -low;
            for (std::size_t k = 0; k < pts.size(); ++k) {
                const Vec2& a = pts[k];
                const Vec2& b = pts[(k + 1) % pts.size()];
                if ((a.x() - u) * (b.x() - u) > 1e-14) {
                    continue;
                }
                const double t =
                    std::abs(b.x() - a.x()) > 1e-12 ? (u - a.x()) / (b.x() - a.x()) : 0.0;
                const double v0 = a.y() + std::clamp(t, 0.0, 1.0) * (b.y() - a.y());
                low = std::min(low, v0);
                high = std::max(high, v0);
                if (std::abs(b.x() - a.x()) <= 1e-12) {
                    low = std::min({low, a.y(), b.y()});
                    high = std::max({high, a.y(), b.y()});
                }
            }
            lo.push_back(low);
            hi.push_back(high);
        }
    }
    bool empty() const noexcept { return us.size() < 2; }
    double first() const noexcept { return us.front(); }
    double last() const noexcept { return us.back(); }
    /// `values` (lo or hi) at u, within [first, last].
    double at(const std::vector<double>& values, double u) const {
        const auto it = std::upper_bound(us.begin(), us.end(), u);
        if (it == us.begin()) {
            return values.front();
        }
        if (it == us.end()) {
            return values.back();
        }
        const auto k = static_cast<std::size_t>(it - us.begin());
        const double t = (u - us[k - 1]) / (us[k] - us[k - 1]);
        return values[k - 1] + t * (values[k] - values[k - 1]);
    }
};

/// The passes of one thread: the same helix and insert, on one side of the stock.
struct ThreadGroup {
    std::vector<const LatheThreadCut*> passes;
    Envelope shape;
    double turn = 0.0; ///< Z per radian of the part (right hand: positive).
    double lead = 0.0;
    bool outside = true; ///< Material lies below the groove.
    int firstCut = 0;    ///< The index of its first pass among the stock's thread cuts.
    double zMin = 0.0;   ///< Where the groove can be at all.
    double zMax = 0.0;

    /// The radius of the surface at z and part angle phi, from `edge`, the radius of the uncut one.
    double surface(double z, double phi, double edge) const {
        double r = edge;
        for (const LatheThreadCut* p : passes) {
            const double base = z - p->from.x() - turn * phi;
            const double k = std::round(base / lead);
            const double length = p->to.x() - p->from.x();
            for (double n = k - 1.0; n <= k + 1.0; n += 1.0) {
                const double du = base - n * lead;
                if (du < shape.first() || du > shape.last()) {
                    continue;
                }
                const double t = (z - du - p->from.x()) / length;
                if (t < -1e-9 || t > p->done + 1e-9) {
                    continue;
                }
                const double rt = p->from.y() + t * (p->to.y() - p->from.y());
                r = outside ? std::min(r, rt + shape.at(shape.lo, du))
                            : std::max(r, rt + shape.at(shape.hi, du));
            }
        }
        return r;
    }
};

std::vector<ThreadGroup> threadGroups(const std::vector<LatheThreadCut>& cuts) {
    std::vector<ThreadGroup> groups;
    for (std::size_t k = 0; k < cuts.size(); ++k) {
        const LatheThreadCut& c = cuts[k];
        const geom2d::Aabb2 box = c.insert.bounds();
        const bool outside = box.min.y() + box.max.y() > 0.0;
        auto it = std::find_if(groups.begin(), groups.end(), [&](const ThreadGroup& g) {
            return std::abs(g.turn - c.turn) < 1e-9 && g.outside == outside &&
                   g.passes.front()->insert == c.insert;
        });
        if (it == groups.end()) {
            ThreadGroup g{{}, Envelope(c.insert), c.turn, c.lead, outside, static_cast<int>(k)};
            if (g.shape.empty()) {
                continue;
            }
            g.zMin = std::numeric_limits<double>::infinity();
            g.zMax = -g.zMin;
            groups.push_back(std::move(g));
            it = std::prev(groups.end());
        }
        it->passes.push_back(&c);
        const double reached = c.from.x() + c.done * (c.to.x() - c.from.x());
        it->zMin =
            std::min({it->zMin, c.from.x() + it->shape.first(), reached + it->shape.first()});
        it->zMax = std::max({it->zMax, c.from.x() + it->shape.last(), reached + it->shape.last()});
    }
    // Deepest first would stop early, but passes are few: the order makes no difference.
    return groups;
}

/// The profile's loops, cut at `us` so an edge lies wholly on one side of each.
std::vector<std::vector<Vec2>> loopsCutAt(const geom2d::Regions& profile, double chordTolerance,
                                          const std::vector<double>& us) {
    std::vector<std::vector<Vec2>> loops;
    auto add = [&](const geom2d::Polyline& line) {
        std::vector<Vec2> pts = line.discretize(chordTolerance);
        if (pts.size() > 1 && (pts.front() - pts.back()).norm() < 1e-9) {
            pts.pop_back();
        }
        std::vector<Vec2> out;
        for (std::size_t k = 0; k < pts.size(); ++k) {
            const Vec2& a = pts[k];
            const Vec2& b = pts[(k + 1) % pts.size()];
            out.push_back(a);
            std::vector<double> cuts;
            for (const double u : us) {
                if ((a.x() - u) * (b.x() - u) < -1e-12) {
                    cuts.push_back((u - a.x()) / (b.x() - a.x()));
                }
            }
            std::sort(cuts.begin(), cuts.end());
            for (const double t : cuts) {
                out.push_back(a + t * (b - a));
            }
        }
        loops.push_back(std::move(out));
    };
    for (const auto& r : profile) {
        add(r.outer);
        for (const auto& hole : r.holes) {
            add(hole);
        }
    }
    return loops;
}

/// An edge of the profile on the side a thread is on, inside its zone: it runs along Z (`along`),
/// or is a face the groove opens through.
struct EdgeUse {
    bool used = false;
    bool along = false;
};
EdgeUse edgeUse(const ThreadGroup& g, const Vec2& p, const Vec2& q) {
    const Vec2 d = q - p;
    const double len = d.norm();
    if (len < 1e-9) {
        return {};
    }
    const double mid = 0.5 * (p.x() + q.x());
    if (mid < g.zMin || mid > g.zMax) {
        return {};
    }
    const double normalV = -d.x() / len; // outward normal's radial part: material on the left
    if (std::abs(normalV) < 0.2) {
        return {true, false};
    }
    return {(normalV > 0.0) == g.outside, true};
}

// -- Holes ------------------------------------------------------------------------------------

/// Negative inside the hole; never more than a little off the true distance near its surface.
double holeDistance(const LatheHoleCut& h, const Vec3& p) {
    const Vec3 d = p - h.tip;
    const double a = d.dot(h.axis);
    const double rho = (d - a * h.axis).norm();
    const double radius = holeRadiusAt(h, std::clamp(a, 1e-9, h.length));
    return std::max({rho - radius, -a, a - h.length});
}

double holeRadius(const LatheHoleCut& h) {
    double r = 0.0;
    for (const Vec2& p : h.profile) {
        r = std::max(r, p.y());
    }
    return r;
}

/// Is (z, radius) inside the profile (even-odd over its loops)?
struct ProfileInside {
    std::vector<std::pair<Vec2, Vec2>> segments;
    explicit ProfileInside(const geom2d::Regions& profile) {
        auto add = [&](const geom2d::Polyline& line) {
            const std::vector<Vec2> pts = line.discretize(1e-2);
            for (std::size_t k = 0; k < pts.size(); ++k) {
                segments.emplace_back(pts[k], pts[(k + 1) % pts.size()]);
            }
        };
        for (const auto& r : profile) {
            add(r.outer);
            for (const auto& hole : r.holes) {
                add(hole);
            }
        }
    }
    bool operator()(const Vec2& p) const {
        bool in = false;
        for (const auto& [a, b] : segments) {
            if ((a.y() > p.y()) != (b.y() > p.y()) &&
                p.x() < a.x() + (p.y() - a.y()) * (b.x() - a.x()) / (b.y() - a.y())) {
                in = !in;
            }
        }
        return in;
    }
};

// -- Clipping a mesh against a solid ----------------------------------------------------------

struct MeshVertex {
    Vec3 position;
    Vec3 normal;
    Rgb colour;
};

using Field = std::function<double(const Vec3&)>;

MeshVertex between(const MeshVertex& a, const MeshVertex& b, double t) {
    MeshVertex v;
    v.position = a.position + t * (b.position - a.position);
    const Vec3 n = a.normal + t * (b.normal - a.normal);
    v.normal = n.norm() > 1e-9 ? Vec3(n.normalized()) : a.normal;
    v.colour = t < 0.5 ? a.colour : b.colour;
    return v;
}

void emit(DisplayMesh& mesh, const MeshVertex& a, const MeshVertex& b, const MeshVertex& c) {
    const auto first = static_cast<std::uint32_t>(mesh.positions.size());
    for (const MeshVertex* v : {&a, &b, &c}) {
        mesh.positions.push_back({static_cast<float>(v->position.x()),
                                  static_cast<float>(v->position.y()),
                                  static_cast<float>(v->position.z())});
        mesh.normals.push_back({static_cast<float>(v->normal.x()),
                                static_cast<float>(v->normal.y()),
                                static_cast<float>(v->normal.z())});
        mesh.colours.push_back(v->colour);
    }
    mesh.indices.push_back(first);
    mesh.indices.push_back(first + 1);
    mesh.indices.push_back(first + 2);
}

/// What of a triangle has `keep` >= 0, in triangles at most `step` long near the field's zero.
void clipTriangle(const MeshVertex& a, const MeshVertex& b, const MeshVertex& c, const Field& keep,
                  double step, int depth, DisplayMesh& out) {
    const double ab = (b.position - a.position).norm();
    const double bc = (c.position - b.position).norm();
    const double ca = (a.position - c.position).norm();
    const double longest = std::max({ab, bc, ca});
    const Vec3 centre = (a.position + b.position + c.position) / 3.0;
    // The field changes by about its distance: this far from its zero the triangle is on one side.
    const double there = keep(centre);
    if (std::abs(there) > 1.2 * longest) {
        if (there > 0.0) {
            emit(out, a, b, c);
        }
        return;
    }
    if (longest > step && depth < 14) {
        if (ab >= bc && ab >= ca) {
            const MeshVertex m = between(a, b, 0.5);
            clipTriangle(a, m, c, keep, step, depth + 1, out);
            clipTriangle(m, b, c, keep, step, depth + 1, out);
        } else if (bc >= ca) {
            const MeshVertex m = between(b, c, 0.5);
            clipTriangle(a, b, m, keep, step, depth + 1, out);
            clipTriangle(a, m, c, keep, step, depth + 1, out);
        } else {
            const MeshVertex m = between(c, a, 0.5);
            clipTriangle(a, b, m, keep, step, depth + 1, out);
            clipTriangle(m, b, c, keep, step, depth + 1, out);
        }
        return;
    }
    const std::array<const MeshVertex*, 3> v{&a, &b, &c};
    const std::array<double, 3> f{keep(a.position), keep(b.position), keep(c.position)};
    const int kept = (f[0] >= 0.0) + (f[1] >= 0.0) + (f[2] >= 0.0);
    if (kept == 3) {
        emit(out, a, b, c);
        return;
    }
    if (kept == 0) {
        return;
    }
    // Where an edge crosses the field's zero, by bisection: the field is not linear.
    auto crossing = [&](int i, int j) {
        double t0 = 0.0, t1 = 1.0;
        const bool iKept = f[static_cast<std::size_t>(i)] >= 0.0;
        for (int k = 0; k < 12; ++k) {
            const double t = 0.5 * (t0 + t1);
            const MeshVertex m =
                between(*v[static_cast<std::size_t>(i)], *v[static_cast<std::size_t>(j)], t);
            if ((keep(m.position) >= 0.0) == iKept) {
                t0 = t;
            } else {
                t1 = t;
            }
        }
        return between(*v[static_cast<std::size_t>(i)], *v[static_cast<std::size_t>(j)],
                       0.5 * (t0 + t1));
    };
    for (int i = 0; i < 3; ++i) {
        const int j = (i + 1) % 3, l = (i + 2) % 3;
        const bool in = f[static_cast<std::size_t>(i)] >= 0.0;
        const bool jn = f[static_cast<std::size_t>(j)] >= 0.0;
        const bool ln = f[static_cast<std::size_t>(l)] >= 0.0;
        if (kept == 1 && in) {
            emit(out, *v[static_cast<std::size_t>(i)], crossing(i, j), crossing(i, l));
            return;
        }
        if (kept == 2 && !in && jn && ln) {
            const MeshVertex pj = crossing(j, i);
            const MeshVertex pl = crossing(l, i);
            emit(out, pj, *v[static_cast<std::size_t>(j)], *v[static_cast<std::size_t>(l)]);
            emit(out, pj, *v[static_cast<std::size_t>(l)], pl);
            return;
        }
    }
}

/// The mesh with what `keep` < 0 clipped off, looking only at triangles touching `near`.
void clipMesh(DisplayMesh& mesh, const Field& keep, const Vec3& nearMin, const Vec3& nearMax,
              double step) {
    DisplayMesh out;
    out.positions.reserve(mesh.positions.size());
    out.normals.reserve(mesh.normals.size());
    out.colours.reserve(mesh.colours.size());
    out.indices.reserve(mesh.indices.size());
    auto vertex = [&](std::uint32_t i) {
        const auto& p = mesh.positions[i];
        const auto& n = mesh.normals[i];
        return MeshVertex{
            Vec3(static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2])),
            Vec3(static_cast<double>(n[0]), static_cast<double>(n[1]), static_cast<double>(n[2])),
            mesh.colours[i]};
    };
    // Untouched triangles keep their vertices, shared as they were.
    std::vector<std::uint32_t> mapped(mesh.positions.size(),
                                      std::numeric_limits<std::uint32_t>::max());
    auto keepVertex = [&](std::uint32_t i) {
        if (mapped[i] == std::numeric_limits<std::uint32_t>::max()) {
            mapped[i] = static_cast<std::uint32_t>(out.positions.size());
            out.positions.push_back(mesh.positions[i]);
            out.normals.push_back(mesh.normals[i]);
            out.colours.push_back(mesh.colours[i]);
        }
        out.indices.push_back(mapped[i]);
    };
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const std::uint32_t i0 = mesh.indices[t], i1 = mesh.indices[t + 1],
                            i2 = mesh.indices[t + 2];
        bool touches = true;
        for (int axis = 0; axis < 3 && touches; ++axis) {
            const auto ua = static_cast<std::size_t>(axis);
            const float lo =
                std::min({mesh.positions[i0][ua], mesh.positions[i1][ua], mesh.positions[i2][ua]});
            const float hi =
                std::max({mesh.positions[i0][ua], mesh.positions[i1][ua], mesh.positions[i2][ua]});
            touches = static_cast<double>(hi) >= nearMin[axis] &&
                      static_cast<double>(lo) <= nearMax[axis];
        }
        if (!touches) {
            keepVertex(i0);
            keepVertex(i1);
            keepVertex(i2);
            continue;
        }
        clipTriangle(vertex(i0), vertex(i1), vertex(i2), keep, step, 0, out);
    }
    mesh = std::move(out);
}

void append(DisplayMesh& mesh, const DisplayMesh& more) {
    const auto base = static_cast<std::uint32_t>(mesh.positions.size());
    mesh.positions.insert(mesh.positions.end(), more.positions.begin(), more.positions.end());
    mesh.normals.insert(mesh.normals.end(), more.normals.begin(), more.normals.end());
    mesh.colours.insert(mesh.colours.end(), more.colours.begin(), more.colours.end());
    for (const std::uint32_t i : more.indices) {
        mesh.indices.push_back(base + i);
    }
}

/// The surface over one edge of the profile. Along Z, the rows follow the first thread on the edge
/// and the others on it (threads crossing) cut the same surface; a face takes each thread's end at
/// its own rim.
void threadStrip(const std::vector<const ThreadGroup*>& users, const Vec2& p, const Vec2& q,
                 bool along, double chord, const ProfileColour& colour, const Rgb& cutRgb,
                 DisplayMesh& mesh) {
    const ThreadGroup& g = *users.front();
    const double rmax = std::max({p.y(), q.y(), 1e-3});
    int segments = std::clamp(
        static_cast<int>(std::ceil(kPi / std::acos(1.0 - std::min(chord, 0.5 * rmax) / rmax))), 36,
        96);
    const Vec2 d = q - p;
    const double len = d.norm();
    const Vec2 outward(d.y() / len, -d.x() / len);

    // Rows run along the helix, so every corner of the groove's form is on one.
    std::vector<double> rows;
    if (along) {
        const double zLo = std::min(p.x(), q.x());
        const double zHi = std::max(p.x(), q.x());
        const double sweep = 2.0 * kPi * g.turn;
        const double sLo = zLo - std::max(0.0, sweep);
        const double sHi = zHi - std::min(0.0, sweep);
        std::vector<double> breaks;
        const double edgeR = 0.5 * (p.y() + q.y());
        for (const LatheThreadCut* c : g.passes) {
            for (const double u : g.shape.us) {
                breaks.push_back(c->from.x() + u);
            }
            // Where the groove's flanks leave the uncut surface.
            const double rt = c->from.y();
            const auto& side = g.outside ? g.shape.lo : g.shape.hi;
            for (std::size_t k = 0; k + 1 < side.size(); ++k) {
                const double a = rt + side[k] - edgeR;
                const double b = rt + side[k + 1] - edgeR;
                if (a * b < 0.0) {
                    breaks.push_back(c->from.x() + g.shape.us[k] +
                                     (g.shape.us[k + 1] - g.shape.us[k]) * a / (a - b));
                }
            }
        }
        for (int k = 0; k < 6; ++k) {
            breaks.push_back(g.lead * k / 6.0);
        }
        for (double& b : breaks) {
            b -= g.lead * std::floor(b / g.lead);
        }
        std::sort(breaks.begin(), breaks.end());
        breaks.erase(std::unique(breaks.begin(), breaks.end(),
                                 [](double a, double b) { return std::abs(a - b) < 1e-6; }),
                     breaks.end());
        for (double n = std::floor(sLo / g.lead); n * g.lead <= sHi; n += 1.0) {
            for (const double b : breaks) {
                const double s = n * g.lead + b;
                if (s > sLo && s < sHi) {
                    rows.push_back(s);
                }
            }
        }
        rows.insert(rows.begin(), sLo);
        rows.push_back(sHi);
        // Long threads: fewer segments round rather than more triangles than a part deserves.
        const double budget = 400000.0;
        segments = std::clamp(static_cast<int>(budget / (2.0 * static_cast<double>(rows.size()))),
                              36, segments);
    } else {
        rows = {0.0, 1.0}; // a face: only its end at the thread changes
    }
    const std::size_t nr = rows.size();
    const auto ns = static_cast<std::size_t>(segments);

    // The base surface's colour along the edge, once per step.
    const int steps = std::clamp(static_cast<int>(std::ceil(len / 0.25)), 1, 4000);
    std::vector<Rgb> base(static_cast<std::size_t>(steps));
    for (int k = 0; k < steps; ++k) {
        base[static_cast<std::size_t>(k)] =
            colour ? colour(p + d * ((k + 0.5) / steps), outward) : kStockColour;
    }
    auto baseAt = [&](double t) -> const Rgb& {
        return base[static_cast<std::size_t>(
            std::clamp(static_cast<int>(t * steps), 0, steps - 1))];
    };

    struct Point {
        Vec3 position;
        Rgb colour;
    };
    std::vector<Point> grid(nr * (ns + 1));
    for (std::size_t j = 0; j <= ns; ++j) {
        const double phi = 2.0 * kPi * static_cast<double>(j % ns) / segments;
        // Rows are one turn apart along the helix, so the seam closes on the same surface.
        const double rowPhi = 2.0 * kPi * static_cast<double>(j) / segments;
        const double c = std::cos(phi);
        const double sn = std::sin(phi);
        for (std::size_t i = 0; i < nr; ++i) {
            double z = 0.0, edge = 0.0, t = 0.0;
            if (along) {
                z = rows[i] + g.turn * rowPhi;
                t = std::clamp((z - p.x()) / d.x(), 0.0, 1.0);
                z = p.x() + t * d.x();
                edge = p.y() + t * d.y();
            } else {
                t = rows[i];
                z = p.x() + t * d.x();
                edge = p.y() + t * d.y();
            }
            double r = edge;
            for (const ThreadGroup* user : users) {
                r = user->surface(z, phi, r);
            }
            r = std::max(r, 0.0);
            // A face stays in its own colour: only its rim moves.
            const bool cut = along && std::abs(r - edge) > 1e-4;
            grid[i * (ns + 1) + j] = {Vec3(r * c, r * sn, z), cut ? cutRgb : baseAt(t)};
        }
    }
    // Facets across the rows are flat, so the form's corners stay sharp.
    for (std::size_t i = 0; i + 1 < nr; ++i) {
        const auto first = static_cast<std::uint32_t>(mesh.positions.size());
        for (std::size_t j = 0; j <= ns; ++j) {
            const Point& a = grid[i * (ns + 1) + j];
            const Point& b = grid[(i + 1) * (ns + 1) + j];
            const Point& c = grid[i * (ns + 1) + (j + 1 <= ns ? j + 1 : j - 1)];
            Vec3 n = (b.position - a.position).cross(c.position - a.position);
            const Vec3 radial(a.position.x(), a.position.y(), 0.0);
            const Vec3 expected = outward.y() * radial.normalized() + Vec3(0, 0, outward.x());
            if (n.norm() < 1e-14) {
                n = expected;
            } else if (n.dot(expected) < 0.0) {
                n = -n;
            }
            n.normalize();
            for (const Point* v : {&a, &b}) {
                mesh.positions.push_back({static_cast<float>(v->position.x()),
                                          static_cast<float>(v->position.y()),
                                          static_cast<float>(v->position.z())});
                mesh.normals.push_back({static_cast<float>(n.x()), static_cast<float>(n.y()),
                                        static_cast<float>(n.z())});
                mesh.colours.push_back(v->colour);
            }
        }
        for (std::size_t j = 0; j < ns; ++j) {
            const std::uint32_t v = first + 2u * static_cast<std::uint32_t>(j);
            // Facing out, whichever way the edge and the rows run.
            const Vec3 a(static_cast<double>(mesh.positions[v][0]),
                         static_cast<double>(mesh.positions[v][1]),
                         static_cast<double>(mesh.positions[v][2]));
            const Vec3 b(static_cast<double>(mesh.positions[v + 1][0]),
                         static_cast<double>(mesh.positions[v + 1][1]),
                         static_cast<double>(mesh.positions[v + 1][2]));
            const Vec3 c(static_cast<double>(mesh.positions[v + 3][0]),
                         static_cast<double>(mesh.positions[v + 3][1]),
                         static_cast<double>(mesh.positions[v + 3][2]));
            const Vec3 e(static_cast<double>(mesh.positions[v + 2][0]),
                         static_cast<double>(mesh.positions[v + 2][1]),
                         static_cast<double>(mesh.positions[v + 2][2]));
            const Vec3 n(static_cast<double>(mesh.normals[v][0]),
                         static_cast<double>(mesh.normals[v][1]),
                         static_cast<double>(mesh.normals[v][2]));
            const bool flip = (b - a).cross(c - a).dot(n) + (c - a).cross(e - a).dot(n) < 0.0;
            if (flip) {
                for (const std::uint32_t t : {v, v + 3, v + 1, v, v + 2, v + 3}) {
                    mesh.indices.push_back(t);
                }
            } else {
                for (const std::uint32_t t : {v, v + 1, v + 3, v, v + 3, v + 2}) {
                    mesh.indices.push_back(t);
                }
            }
        }
    }
}

/// The wall and bottom of a hole, as a surface around its axis facing into the hole.
DisplayMesh holeWall(const LatheHoleCut& h, const Rgb& rgb) {
    DisplayMesh mesh;
    constexpr int kSegments = 48;
    const Vec3 ref = std::abs(h.axis.z()) < 0.9 ? Vec3(0, 0, 1) : Vec3(1, 0, 0);
    const Vec3 ex = h.axis.cross(ref).normalized();
    const Vec3 ey = h.axis.cross(ex).normalized();
    std::vector<Vec2> rows;
    for (const Vec2& p : h.profile) {
        if (p.x() < h.length) {
            rows.push_back(p);
        }
    }
    rows.emplace_back(h.length, holeRadiusAt(h, h.length));
    for (std::size_t i = 0; i + 1 < rows.size(); ++i) {
        const Vec2 a = rows[i], b = rows[i + 1];
        if (b.x() - a.x() < 1e-9) {
            continue;
        }
        const Vec2 d = (b - a).normalized();
        const auto first = static_cast<std::uint32_t>(mesh.positions.size());
        for (int k = 0; k <= kSegments; ++k) {
            const double angle = 2.0 * kPi * (k % kSegments) / kSegments;
            const Vec3 radial = std::cos(angle) * ex + std::sin(angle) * ey;
            // Into the hole: towards the axis, and up the axis under a tip's cone.
            const Vec3 n = (-d.x() * radial + d.y() * h.axis).normalized();
            for (const Vec2& p : {a, b}) {
                const Vec3 x = h.tip + p.x() * h.axis + p.y() * radial;
                mesh.positions.push_back({static_cast<float>(x.x()), static_cast<float>(x.y()),
                                          static_cast<float>(x.z())});
                mesh.normals.push_back({static_cast<float>(n.x()), static_cast<float>(n.y()),
                                        static_cast<float>(n.z())});
                mesh.colours.push_back(rgb);
            }
        }
        for (int k = 0; k < kSegments; ++k) {
            const std::uint32_t v = first + 2u * static_cast<std::uint32_t>(k);
            const Vec3 pa = storedVec3(mesh.positions[v]);
            const Vec3 pb = storedVec3(mesh.positions[v + 3]);
            const Vec3 pc = storedVec3(mesh.positions[v + 2]);
            const Vec3 n = storedVec3(mesh.normals[v]);
            if ((pb - pa).cross(pc - pa).dot(n) >= 0.0) {
                for (const std::uint32_t t : {v, v + 1, v + 3, v, v + 3, v + 2}) {
                    mesh.indices.push_back(t);
                }
            } else {
                for (const std::uint32_t t : {v, v + 3, v + 1, v, v + 2, v + 3}) {
                    mesh.indices.push_back(t);
                }
            }
        }
    }
    return mesh;
}

} // namespace

double holeRadiusAt(const LatheHoleCut& h, double a) {
    const auto& pts = h.profile;
    if (a <= 0.0 || pts.empty() || a > h.length) {
        return 0.0;
    }
    if (a >= pts.back().x()) {
        return pts.back().y();
    }
    for (std::size_t k = 1; k < pts.size(); ++k) {
        if (a <= pts[k].x()) {
            const double span = pts[k].x() - pts[k - 1].x();
            const double t = span > 1e-12 ? (a - pts[k - 1].x()) / span : 1.0;
            return pts[k - 1].y() + t * (pts[k].y() - pts[k - 1].y());
        }
    }
    return pts.back().y();
}

void LatheStock::cutThread(const geom2d::Polyline& insert, const Vec2& from, const Vec2& to,
                           double lead, double t0, double t1) {
    if (lead <= 0.0 || std::abs(to.x() - from.x()) < 1e-9 || t1 <= t0) {
        return;
    }
    // The spindle turns the part the same way whichever way the tool goes (M03): towards the chuck
    // that cuts a right-hand helix, away from it a left-hand one.
    const double turn = (to.x() < from.x() ? lead : -lead) / (2.0 * kPi);
    for (LatheThreadCut& c : threads_) {
        if (c.from == from && c.to == to && c.lead == lead && c.insert == insert) {
            // The rest of a pass already begun: playback cuts a move part by part.
            if (t0 <= c.done + 1e-9) {
                c.done = std::max(c.done, t1);
            }
            return;
        }
    }
    threads_.push_back({from, to, lead, turn, t1, insert});
}

void LatheStock::cutHole(const LatheHoleCut& hole) {
    for (LatheHoleCut& h : holes_) {
        // The same hole again, deeper or not: pecks and repeated passes.
        const Vec3 along = hole.tip - h.tip;
        if ((h.axis - hole.axis).norm() < 1e-9 &&
            (along - along.dot(h.axis) * h.axis).norm() < 1e-6 && h.profile == hole.profile) {
            const double offset = along.dot(h.axis); // hole's tip above h's: positive
            if (offset < 0.0) {
                h.tip = hole.tip;
                h.length = std::max(h.length - offset, hole.length);
            } else {
                h.length = std::max(h.length, hole.length + offset);
            }
            return;
        }
    }
    holes_.push_back(hole);
}

void LatheStock::takeCuts(const LatheStock& other, const Transform3& move) {
    const bool flipped = move.linear()(2, 2) < 0.0;
    const double along = move.translation().z();
    for (LatheThreadCut c : other.threads_) {
        // Flipped end for end a helix keeps its hand: `turn` stays.
        c.from.x() = (flipped ? -c.from.x() : c.from.x()) + along;
        c.to.x() = (flipped ? -c.to.x() : c.to.x()) + along;
        if (flipped) {
            std::vector<Vec2> pts = c.insert.discretize(0.005);
            for (Vec2& p : pts) {
                p.x() = -p.x();
            }
            std::reverse(pts.begin(), pts.end());
            c.insert = geom2d::Polyline::fromPoints(pts, true);
        }
        threads_.push_back(std::move(c));
    }
    for (LatheHoleCut h : other.holes_) {
        h.tip = move * h.tip;
        h.axis = (move.linear() * h.axis).normalized();
        holes_.push_back(std::move(h));
    }
}

double LatheStock::volume() const {
    double v = revolvedVolume(profile_);
    if (!hasCuts()) {
        return v;
    }
    const ProfileInside inside(profile_);
    // A thread: what its surface lost against the profile's, over a grid along the helix.
    const std::vector<ThreadGroup> groups = threadGroups(threads_);
    if (!groups.empty()) {
        std::vector<double> us;
        for (const ThreadGroup& g : groups) {
            us.push_back(g.zMin);
            us.push_back(g.zMax);
        }
        const auto loops = loopsCutAt(profile_, 1e-2, us);
        for (const ThreadGroup& g : groups) {
            constexpr int kAround = 48;
            const double dz = g.lead / 24.0;
            for (const auto& loop : loops) {
                for (std::size_t e = 0; e < loop.size(); ++e) {
                    const Vec2& p = loop[e];
                    const Vec2& q = loop[(e + 1) % loop.size()];
                    if (!edgeUse(g, p, q).along) {
                        continue;
                    }
                    const double span = std::abs(q.x() - p.x());
                    const int nz = std::max(1, static_cast<int>(std::ceil(span / dz)));
                    for (int i = 0; i < nz; ++i) {
                        const double t = (i + 0.5) / nz;
                        const double z = p.x() + t * (q.x() - p.x());
                        const double edge = p.y() + t * (q.y() - p.y());
                        for (int j = 0; j < kAround; ++j) {
                            const double phi = 2.0 * kPi * (j + 0.5) / kAround;
                            const double r = std::max(g.surface(z, phi, edge), 0.0);
                            v -= 0.5 * std::abs(edge * edge - r * r) * (span / nz) *
                                 (2.0 * kPi / kAround);
                        }
                    }
                }
            }
        }
    }
    // A hole: the stock it passes through, sampled in rings along its axis.
    for (const LatheHoleCut& h : holes_) {
        constexpr int kAlong = 40, kRings = 6, kAround = 16;
        const Vec3 ref = std::abs(h.axis.z()) < 0.9 ? Vec3(0, 0, 1) : Vec3(1, 0, 0);
        const Vec3 ex = h.axis.cross(ref).normalized();
        const Vec3 ey = h.axis.cross(ex).normalized();
        const double da = h.length / kAlong;
        for (int i = 0; i < kAlong; ++i) {
            const double a = (i + 0.5) * da;
            const double radius = holeRadiusAt(h, a);
            for (int ring = 0; ring < kRings; ++ring) {
                const double rho = radius * std::sqrt((ring + 0.5) / kRings); // equal areas
                for (int j = 0; j < kAround; ++j) {
                    const double angle = 2.0 * kPi * (j + 0.5) / kAround;
                    const Vec3 x =
                        h.tip + a * h.axis + rho * (std::cos(angle) * ex + std::sin(angle) * ey);
                    if (inside(Vec2(x.z(), std::hypot(x.x(), x.y())))) {
                        v -= kPi * radius * radius * da / (kRings * kAround);
                    }
                }
            }
        }
    }
    return v;
}

DisplayMesh latheStockMesh(const LatheStock& stock, double chordTolerance,
                           const ProfileColour& colour, const LatheCutColour& cutColour,
                           double creaseAngle, double colourStep) {
    if (!stock.hasCuts()) {
        return revolvedMesh(stock.profile(), chordTolerance, colour, creaseAngle, colourStep);
    }
    const std::vector<ThreadGroup> groups = threadGroups(stock.threads());
    std::vector<double> us;
    for (const ThreadGroup& g : groups) {
        us.push_back(g.zMin);
        us.push_back(g.zMax);
    }
    const auto loops = loopsCutAt(stock.profile(), chordTolerance, us);
    // The profile's edges a thread does not touch revolve as ever.
    DisplayMesh mesh = revolvedLoops(
        loops, chordTolerance, colour, creaseAngle, colourStep, [&](const Vec2& p, const Vec2& q) {
            return std::any_of(groups.begin(), groups.end(),
                               [&](const ThreadGroup& g) { return edgeUse(g, p, q).used; });
        });
    // Each edge once, with every thread on it.
    for (const auto& loop : loops) {
        for (std::size_t e = 0; e < loop.size(); ++e) {
            const Vec2& p = loop[e];
            const Vec2& q = loop[(e + 1) % loop.size()];
            if (p.y() <= 1e-9 && q.y() <= 1e-9) {
                continue;
            }
            std::vector<const ThreadGroup*> users;
            bool along = false;
            for (const ThreadGroup& g : groups) {
                const EdgeUse use = edgeUse(g, p, q);
                if (use.used) {
                    users.push_back(&g);
                    along = along || use.along;
                }
            }
            if (!users.empty()) {
                threadStrip(users, p, q, along, chordTolerance, colour,
                            cutColour ? cutColour(false, users.front()->firstCut) : kStockColour,
                            mesh);
            }
        }
    }
    // Holes: punched out of what there is, and their walls where they pass through material.
    const auto& holes = stock.holes();
    const sim::ProfileDistance material(stock.profile(), 1e-2);
    for (std::size_t k = 0; k < holes.size(); ++k) {
        const LatheHoleCut& h = holes[k];
        const double radius = holeRadius(h);
        if (radius <= 0.0 || h.length <= 0.0) {
            continue;
        }
        const double step = std::max(radius / 8.0, 0.02);
        Vec3 lo = h.tip, hi = h.tip;
        const Vec3 top = h.tip + h.length * h.axis;
        for (int axis = 0; axis < 3; ++axis) {
            const double reach =
                radius * std::sqrt(std::max(0.0, 1.0 - h.axis[axis] * h.axis[axis]));
            lo[axis] = std::min(h.tip[axis], top[axis]) - reach - 2.0 * step;
            hi[axis] = std::max(h.tip[axis], top[axis]) + reach + 2.0 * step;
        }
        clipMesh(mesh, [&](const Vec3& x) { return holeDistance(h, x); }, lo, hi, step);
    }
    for (std::size_t k = 0; k < holes.size(); ++k) {
        const LatheHoleCut& h = holes[k];
        const double radius = holeRadius(h);
        if (radius <= 0.0 || h.length <= 0.0) {
            continue;
        }
        DisplayMesh wall =
            holeWall(h, cutColour ? cutColour(true, static_cast<int>(k)) : kStockColour);
        const double step = std::max(radius / 8.0, 0.02);
        const Vec3 far = Vec3::Constant(std::numeric_limits<double>::max() / 4.0);
        clipMesh(
            wall,
            [&](const Vec3& x) {
                // Inside the stock, and outside the other holes.
                double keep = -material(Vec2(x.z(), std::hypot(x.x(), x.y())));
                for (std::size_t o = 0; o < holes.size(); ++o) {
                    if (o != k) {
                        keep = std::min(keep, holeDistance(holes[o], x));
                    }
                }
                return keep;
            },
            -far, far, step);
        append(mesh, wall);
    }
    return mesh;
}

} // namespace ocam::sim
