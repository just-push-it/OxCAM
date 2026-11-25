// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Everything rides on a body of the machine's tree: the base or an axis. With a lathe's slides free
// of rotary axes (validateLatheModel), a body's motion is its motion with the slides at 0 (F, which
// only changes when the turret, tailstock or sub-spindle moves) moved by a fixed linear function of
// the slides (G), so a pose costs a matrix and a vector per body. Solids on the turret's chain are
// checked against everything off it; boxes first, per body and then per solid. See
// docs/design-log/0067.

#include "ocam/sim/LatheMachine.hpp"

#include "MachineContacts.hpp"

#include <ocam/foundation/Units.hpp>
#include <ocam/geom3d/Bvh.hpp>
#include <ocam/geom3d/Convex.hpp>
#include <ocam/sim/Lathe.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace ocam::sim {
namespace {

using detail::Contact;
using machine::AxisPosition;
using machine::LatheStep;

/// Andrew's monotone chain, counter-clockwise.
std::vector<Vec2> hullOf(std::vector<Vec2> points) {
    std::sort(points.begin(), points.end(), [](const Vec2& a, const Vec2& b) {
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    points.erase(std::unique(points.begin(), points.end(),
                             [](const Vec2& a, const Vec2& b) { return (a - b).norm() < 1e-9; }),
                 points.end());
    if (points.size() < 3) {
        return points;
    }
    std::vector<Vec2> hull(points.size() * 2);
    std::size_t k = 0;
    auto cross = [](const Vec2& o, const Vec2& a, const Vec2& b) {
        return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    };
    for (const Vec2& p : points) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], p) <= 0.0) {
            --k;
        }
        hull[k++] = p;
    }
    const std::size_t lower = k + 1;
    for (std::size_t i = points.size() - 1; i-- > 0;) {
        while (k >= lower && cross(hull[k - 2], hull[k - 1], points[i]) <= 0.0) {
            --k;
        }
        hull[k++] = points[i];
    }
    hull.resize(k - 1);
    return hull;
}

/// A swept area, by its edges, for what lies inside it along a line of constant z.
class Revolved {
public:
    explicit Revolved(const geom2d::Regions& area) {
        auto addLoop = [&](const geom2d::Polyline& loop) {
            const std::vector<Vec2> pts = loop.discretize(0.01);
            for (std::size_t i = 0; i < pts.size(); ++i) {
                const Vec2& a = pts[i];
                const Vec2& b = pts[(i + 1) % pts.size()];
                zMin_ = std::min(zMin_, a.x());
                zMax_ = std::max(zMax_, a.x());
                rMax_ = std::max(rMax_, a.y());
                if (a.x() != b.x()) {
                    edges_.push_back({a, b});
                }
            }
        };
        for (const geom2d::Region& r : area) {
            addLoop(r.outer);
            for (const geom2d::Polyline& h : r.holes) {
                addLoop(h);
            }
        }
        if (edges_.empty()) {
            return;
        }
        cell_ = std::max((zMax_ - zMin_) / 256.0, 0.5);
        buckets_.resize(static_cast<std::size_t>((zMax_ - zMin_) / cell_) + 2);
        for (std::size_t e = 0; e < edges_.size(); ++e) {
            const auto [lo, hi] = std::minmax(edges_[e].a.x(), edges_[e].b.x());
            for (std::size_t c = bucket(lo); c <= bucket(hi); ++c) {
                buckets_[c].push_back(static_cast<std::uint32_t>(e));
            }
        }
    }

    bool isEmpty() const noexcept { return edges_.empty(); }
    double zMin() const noexcept { return zMin_; }
    double zMax() const noexcept { return zMax_; }
    double rMax() const noexcept { return rMax_; }

    /// The stretches of radius inside the area at `z`, in `out`.
    void intervals(double z, std::vector<std::pair<double, double>>& out) const {
        out.clear();
        crossings_.clear();
        for (const std::uint32_t e : buckets_[bucket(z)]) {
            const Vec2& a = edges_[e].a;
            const Vec2& b = edges_[e].b;
            if ((a.x() <= z) == (b.x() <= z)) {
                continue; // half open, so a vertex counts once
            }
            crossings_.push_back(a.y() + (b.y() - a.y()) * (z - a.x()) / (b.x() - a.x()));
        }
        std::sort(crossings_.begin(), crossings_.end());
        for (std::size_t i = 0; i + 1 < crossings_.size(); i += 2) {
            out.emplace_back(crossings_[i], crossings_[i + 1]);
        }
    }

private:
    struct Edge {
        Vec2 a;
        Vec2 b;
    };
    std::size_t bucket(double z) const {
        return std::min(static_cast<std::size_t>(std::max(z - zMin_, 0.0) / cell_),
                        buckets_.size() - 1);
    }
    std::vector<Edge> edges_;
    std::vector<std::vector<std::uint32_t>> buckets_;
    double cell_ = 1.0;
    double zMin_ = std::numeric_limits<double>::infinity();
    double zMax_ = -std::numeric_limits<double>::infinity();
    double rMax_ = 0.0;
    mutable std::vector<double> crossings_;
};

/// A convex polytope's vertices, two caps of `m`, against a swept area: slice it across the axis,
/// and compare the slice's radii with what is inside the area there. All in the area's frame.
std::optional<Contact> polytopeToRevolved(const std::vector<Vec3>& pts, const Revolved& r,
                                          double clearance) {
    const std::size_t m = pts.size() / 2;
    double zLo = std::numeric_limits<double>::infinity();
    double zHi = -zLo;
    Vec2 boxLo(zLo, zLo), boxHi(zHi, zHi);
    for (const Vec3& p : pts) {
        zLo = std::min(zLo, p.z());
        zHi = std::max(zHi, p.z());
        boxLo = boxLo.cwiseMin(p.head<2>());
        boxHi = boxHi.cwiseMax(p.head<2>());
    }
    const double from = std::max(zLo - clearance, r.zMin() - clearance);
    const double to = std::min(zHi + clearance, r.zMax() + clearance);
    if (from > to) {
        return std::nullopt;
    }
    // The nearest the box comes to the axis.
    const Vec2 nearest = Vec2::Zero().cwiseMax(boxLo).cwiseMin(boxHi);
    if (nearest.norm() > r.rMax() + clearance) {
        return std::nullopt;
    }
    const int slices = std::clamp(static_cast<int>(std::ceil((to - from) / 1.0)) + 1, 2, 64);
    std::vector<Vec2> section;
    std::vector<std::pair<double, double>> inside;
    for (int s = 0; s < slices; ++s) {
        const double z = from + (to - from) * s / (slices - 1);
        const double zc = std::clamp(z, zLo, zHi);
        section.clear();
        auto edge = [&](const Vec3& a, const Vec3& b) {
            const double da = a.z() - zc;
            const double db = b.z() - zc;
            if (da == 0.0 && db == 0.0) {
                section.push_back(a.head<2>());
                section.push_back(b.head<2>());
            } else if (da * db <= 0.0) {
                const double t = da / (da - db);
                section.push_back(a.head<2>() + (b.head<2>() - a.head<2>()) * t);
            }
        };
        for (std::size_t i = 0; i < m; ++i) {
            const std::size_t j = (i + 1) % m;
            edge(pts[i], pts[j]);
            edge(pts[m + i], pts[m + j]);
            edge(pts[i], pts[m + i]);
        }
        if (section.empty()) {
            continue;
        }
        const std::vector<Vec2> hull = hullOf(section);
        double rho0 = std::numeric_limits<double>::infinity();
        double rho1 = 0.0;
        Vec2 near = hull.front();
        Vec2 far = hull.front();
        bool around = hull.size() >= 3;
        for (std::size_t i = 0; i < hull.size(); ++i) {
            const Vec2& a = hull[i];
            const Vec2& b = hull[(i + 1) % hull.size()];
            if (a.norm() > rho1) {
                rho1 = a.norm();
                far = a;
            }
            const Vec2 e = b - a;
            const double len2 = e.squaredNorm();
            const double t = len2 > 0.0 ? std::clamp(-a.dot(e) / len2, 0.0, 1.0) : 0.0;
            const Vec2 q = a + e * t;
            if (q.norm() < rho0) {
                rho0 = q.norm();
                near = q;
            }
            around = around && e.x() * (-a.y()) - e.y() * (-a.x()) >= 0.0;
        }
        if (around) {
            rho0 = 0.0;
            near = Vec2::Zero();
        }
        r.intervals(std::clamp(z, r.zMin() + 1e-9, r.zMax() - 1e-9), inside);
        for (const auto& [lo, hi] : inside) {
            if (rho1 + clearance >= lo && rho0 - clearance <= hi) {
                const double gap = std::max({lo - rho1, rho0 - hi, 0.0});
                const Vec2& p = rho1 < lo ? far : near;
                return Contact{gap, Vec3(p.x(), p.y(), zc)};
            }
        }
    }
    return std::nullopt;
}

Vec3 linearOf(const Eigen::Matrix3d& gain, const Vec3& slides) {
    return gain * slides;
}

void grow(geom3d::Aabb3& box, double by) {
    box.min -= Vec3::Constant(by);
    box.max += Vec3::Constant(by);
}

geom3d::Aabb3 shifted(geom3d::Aabb3 box, const Vec3& by) {
    box.min += by;
    box.max += by;
    return box;
}

std::vector<Vec3> prismPoints(const ConvexPrism& prism) {
    std::vector<Vec3> pts = prism.lower;
    pts.insert(pts.end(), prism.upper.begin(), prism.upper.end());
    return pts;
}

ConvexPrism componentPrism(const machine::Component& c) {
    if (c.shape == machine::Component::Shape::Cylinder) {
        return roundPrism(c.base, c.axis, c.radius, c.length, 48);
    }
    ConvexPrism p;
    for (const double z : {c.min.z(), c.max.z()}) {
        std::vector<Vec3>& cap = z == c.min.z() ? p.lower : p.upper;
        cap = {Vec3(c.min.x(), c.min.y(), z), Vec3(c.max.x(), c.min.y(), z),
               Vec3(c.max.x(), c.max.y(), z), Vec3(c.min.x(), c.max.y(), z)};
    }
    return p;
}

} // namespace

ConvexPrism platePrism(const std::vector<Vec2>& outline, double low, double high) {
    ConvexPrism p;
    for (const Vec2& q : hullOf(outline)) {
        p.lower.emplace_back(q.y(), low, q.x());
        p.upper.emplace_back(q.y(), high, q.x());
    }
    return p;
}

ConvexPrism roundPrism(const Vec3& base, const Vec3& axisIn, double radius, double length,
                       int sides) {
    const Vec3 axis = axisIn.normalized();
    const Vec3 u =
        (std::abs(axis.x()) < 0.9 ? Vec3::UnitX() : Vec3::UnitY()).cross(axis).normalized();
    const Vec3 v = axis.cross(u);
    const double r = radius / std::cos(kPi / sides);
    ConvexPrism p;
    for (int k = 0; k < sides; ++k) {
        const double a = 2.0 * kPi * k / sides;
        const Vec3 q = base + r * (std::cos(a) * u + std::sin(a) * v);
        p.lower.push_back(q);
        p.upper.push_back(q + axis * length);
    }
    return p;
}

geom3d::Mesh prismMesh(const ConvexPrism& prism) {
    geom3d::Mesh mesh;
    const auto m = static_cast<std::uint32_t>(prism.lower.size());
    if (m < 3) {
        return mesh;
    }
    mesh.vertices = prism.lower;
    mesh.vertices.insert(mesh.vertices.end(), prism.upper.begin(), prism.upper.end());
    // Which way round the outline runs: the triangles below face out when it turns right-handed
    // about the extrusion.
    const Vec3 n = (prism.lower[1] - prism.lower[0]).cross(prism.lower[2] - prism.lower[0]);
    const bool flip = n.dot(prism.upper[0] - prism.lower[0]) < 0.0;
    auto add = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
        if (flip) {
            mesh.triangles.push_back({a, c, b});
        } else {
            mesh.triangles.push_back({a, b, c});
        }
    };
    for (std::uint32_t i = 0; i < m; ++i) {
        const std::uint32_t j = (i + 1) % m;
        add(i, j, m + j);
        add(i, m + j, m + i);
        if (i >= 1 && i + 1 < m) {
            add(0, j, i);
            add(m, m + i, m + j);
        }
    }
    return mesh;
}

struct LatheMachineCheck::Impl {
    /// What rides on the turret's chain, or else slides along the spindle's axis.
    struct Moving {
        std::string name;
        std::size_t body = 0;
        int station = 0; ///< 0 for the turret itself.
        bool slider = false;
        std::vector<Vec3> points; ///< A polytope (two caps) at machine zero; empty with a mesh.
        std::shared_ptr<const geom3d::TriangleBvh> mesh;
        Transform3 placement = Transform3::Identity();
        bool wallByBox =
            false;         ///< A mesh held to the enclosure's inside by its box, as a solid is.
        geom3d::Aabb3 box; ///< With the slides at 0.
    };
    /// What the moving things can meet.
    struct Other {
        std::string name;
        std::size_t body = 0;
        std::optional<geom3d::Convex> convex;
        std::shared_ptr<const geom3d::TriangleBvh> mesh;
        Transform3 placement = Transform3::Identity();
        std::shared_ptr<const Revolved> revolved; ///< Sweeps this area, in `frame`.
        Transform3 frame = Transform3::Identity();
        bool work = false; ///< The setup's: the cutting tool is left to the turning checks.
        /// A wall of the layout's enclosure: solids with vertices are held to the inside of it
        /// instead, so one that has gone right through still counts.
        bool wall = false;
        geom3d::Aabb3 box;
    };

    machine::MachineModel model;
    machine::LatheLayout layout;
    machine::WorkMount mount;
    double rapidRadius = 1.0;
    std::size_t turretBody = 0;
    std::size_t workBody = 0;
    std::vector<std::size_t> parent;
    std::vector<Eigen::Matrix3d> gain; ///< Per body: its translation per unit of X, Y, Z.
    Eigen::Matrix3d toSlides = Eigen::Matrix3d::Zero();
    std::vector<Moving> moving;
    std::vector<Other> others;
    std::shared_ptr<const Revolved> part;
    std::size_t turretSolids = 0;

    std::size_t bodyOf(std::string_view carrier) const {
        for (std::size_t i = 0; i < model.axes.size(); ++i) {
            if (model.axes[i].name == carrier) {
                return i + 1;
            }
        }
        return 0;
    }

    std::vector<std::size_t> chainOf(std::size_t body) const {
        std::vector<std::size_t> chain;
        for (std::size_t b = body; b != 0; b = parent[b]) {
            chain.push_back(b);
        }
        return chain;
    }

    bool sameChain(std::size_t a, std::size_t b) const {
        const std::vector<std::size_t> ca = chainOf(a);
        const std::vector<std::size_t> cb = chainOf(b);
        return a == b || std::find(ca.begin(), ca.end(), b) != ca.end() ||
               std::find(cb.begin(), cb.end(), a) != cb.end();
    }

    /// The bodies' motions with the slides at 0, for T, W, Z2 and C of `at`.
    std::vector<Transform3> motions(const AxisPosition& at) const {
        AxisPosition still = at;
        still.linear = Vec3::Zero();
        const machine::ModelPose pose = machine::poseAt(model, still);
        std::vector<Transform3> out(parent.size(), Transform3::Identity());
        for (std::size_t i = 0; i < model.axes.size(); ++i) {
            out[i + 1] = pose.axes[i];
        }
        return out;
    }

    void addSolid(const std::string& name, std::size_t body, std::optional<ConvexPrism> prism,
                  const geom3d::Mesh* mesh, const Transform3& placement, bool spun,
                  bool onTurretChain, bool slider) {
        std::shared_ptr<const geom3d::TriangleBvh> bvh;
        if (mesh) {
            bvh = std::make_shared<const geom3d::TriangleBvh>(*mesh);
        }
        std::optional<geom3d::Convex> convex;
        if (prism) {
            convex = geom3d::Convex::points(prismPoints(*prism));
        }
        if (onTurretChain || slider) {
            Moving m;
            m.name = name;
            m.body = body;
            m.slider = slider;
            if (prism) {
                m.points = prismPoints(*prism);
            }
            m.mesh = bvh;
            m.placement = placement;
            moving.push_back(std::move(m));
            ++turretSolids;
        }
        if (onTurretChain) {
            return;
        }
        Other o;
        o.name = name;
        o.body = body;
        o.convex = std::move(convex);
        o.mesh = bvh;
        o.placement = placement;
        if (spun) {
            geom3d::Mesh swept;
            if (mesh) {
                swept = *mesh;
                swept.transform(placement);
            } else if (prism) {
                swept = prismMesh(*prism);
            }
            o.revolved = std::make_shared<const Revolved>(revolvedEnvelope(swept, 0.5));
        }
        others.push_back(std::move(o));
    }
};

LatheMachineCheck::LatheMachineCheck(const machine::Machine& machine,
                                     const machine::WorkMount& mount, std::vector<TurretTool> tools,
                                     std::vector<LatheWorkBody> bodies, geom2d::Regions partArea,
                                     bool withMachineChuck, const std::vector<TurretBlock>& blocks)
    : impl_(std::make_unique<Impl>()) {
    Impl& s = *impl_;
    s.model = machine::effectiveModel(machine);
    s.layout = *machine.lathe;
    s.mount = mount;
    const std::size_t n = s.model.axes.size();
    s.parent.assign(n + 1, 0);
    for (std::size_t i = 0; i < n; ++i) {
        s.parent[i + 1] = s.bodyOf(s.model.axes[i].parent);
    }
    s.turretBody = s.bodyOf(s.model.spindleCarrier);
    s.workBody = s.bodyOf(mount.carrier);
    if (!partArea.empty()) {
        s.part = std::make_shared<const Revolved>(partArea);
    }
    // What each slide does to each body.
    s.gain.assign(n + 1, Eigen::Matrix3d::Zero());
    const machine::ModelPose zero = machine::poseAt(s.model, {});
    for (int k = 0; k < 3; ++k) {
        AxisPosition e;
        e.linear[k] = 1.0;
        const machine::ModelPose moved = machine::poseAt(s.model, e);
        for (std::size_t i = 0; i < n; ++i) {
            s.gain[i + 1].col(k) = moved.axes[i].translation() - zero.axes[i].translation();
        }
    }
    s.toSlides = Eigen::Matrix3d(s.gain[s.turretBody] - s.gain[s.workBody])
                     .completeOrthogonalDecomposition()
                     .pseudoInverse();

    // Sorted onto the tree: the turret's chain, what slides along the spindle's axis and what
    // turns with a spindle (C, C2).
    const std::vector<std::size_t> turretChain = s.chainOf(s.turretBody);
    const std::size_t cBody = s.bodyOf("C");
    const std::size_t c2Body = s.bodyOf("C2");
    auto place = [&](const std::string& name, std::size_t body, std::optional<ConvexPrism> prism,
                     const geom3d::Mesh* mesh, const Transform3& placement) {
        const bool onTurret =
            std::find(turretChain.begin(), turretChain.end(), body) != turretChain.end();
        const std::vector<std::size_t> chain = s.chainOf(body);
        const auto carries = [&](std::size_t b) {
            return b != 0 && std::find(chain.begin(), chain.end(), b) != chain.end();
        };
        const bool spun = carries(cBody) || carries(c2Body);
        const bool slider = !onTurret && (carries(s.bodyOf("W")) || carries(s.bodyOf("Z2")));
        s.addSolid(name, body, std::move(prism), mesh, placement, spun, onTurret, slider);
    };
    for (const machine::LatheSolid& solid : machine::latheSolids(machine)) {
        if (solid.shape.name == machine::kLatheChuckPart && !withMachineChuck) {
            continue;
        }
        place(solid.shape.name, s.bodyOf(solid.carrier), componentPrism(solid.shape), nullptr,
              Transform3::Identity());
        if (solid.shape.name.rfind("Enclosure", 0) == 0) {
            s.others.back().wall = true;
        }
    }
    for (const machine::ModelPart& p : machine.model.parts) {
        if (p.collides && p.geometry && !p.geometry->isEmpty()) {
            place(p.name, s.bodyOf(p.carrier), std::nullopt, p.geometry.get(), p.placement());
        }
    }
    // The tools: at T 0, a station stands turned back from where it works.
    double reach = 0.0;
    for (const TurretTool& tool : tools) {
        const Transform3 atZero =
            machine::turretTurn(s.layout.turret, -machine::stationAngle(machine, tool.station)) *
            tool.placement;
        for (const TurretTool::Solid& solid : tool.solids) {
            Impl::Moving m;
            m.name = solid.part.empty() ? tool.name : tool.name + ", " + solid.part;
            m.body = s.turretBody;
            m.station = tool.station;
            for (const Vec3& q : prismPoints(solid.prism)) {
                m.points.push_back(atZero * q);
                reach = std::max(reach, (m.points.back() - s.layout.turret.centre).norm());
            }
            s.moving.push_back(std::move(m));
            ++s.turretSolids;
        }
    }
    for (const TurretBlock& block : blocks) {
        if (!block.mesh || block.mesh->isEmpty()) {
            continue;
        }
        Impl::Moving m;
        m.name = block.name;
        m.body = s.turretBody;
        m.mesh = std::make_shared<const geom3d::TriangleBvh>(*block.mesh);
        m.placement =
            machine::turretTurn(s.layout.turret, -machine::stationAngle(machine, block.station)) *
            block.placement;
        m.wallByBox = true;
        const geom3d::Aabb3 box = geom3d::transformed(block.mesh->bounds(), m.placement);
        for (const Vec3& corner : {box.min, box.max}) {
            reach = std::max(reach, (corner - s.layout.turret.centre).norm());
        }
        s.moving.push_back(std::move(m));
        ++s.turretSolids;
    }
    s.rapidRadius = std::max({reach, s.layout.turret.diameter / 2.0, 1.0});
    for (LatheWorkBody& b : bodies) {
        auto revolved = std::make_shared<const Revolved>(b.area);
        if (revolved->isEmpty()) {
            continue;
        }
        Impl::Other o;
        o.name = std::move(b.name);
        o.body = s.bodyOf(b.carrier);
        o.revolved = std::move(revolved);
        o.frame = b.frame;
        o.work = true;
        s.others.push_back(std::move(o));
    }
}

LatheMachineCheck::~LatheMachineCheck() = default;

std::size_t LatheMachineCheck::turretSolids() const noexcept {
    return impl_->turretSolids;
}

std::vector<MachineCollision> LatheMachineCheck::check(const machine::LatheMotion& motion,
                                                       const toolpath::Toolpath& path,
                                                       const LatheStockTrack& stock,
                                                       CancelToken* cancel) const {
    const Impl& s = *impl_;
    std::vector<MachineCollision> out;
    const double clearance = s.layout.clearance;
    constexpr std::size_t kMaxCollisions = 200;
    constexpr double kStep = 2.0;

    // Moved boxes with the slides at 0: only the turret, tailstock and sub-spindle change them.
    std::vector<Transform3> motions;
    std::vector<geom3d::Aabb3> movingBoxes(s.moving.size());
    std::vector<geom3d::Aabb3> otherBoxes(s.others.size());
    // A body's solids in one box, for the common case of nothing being near.
    std::vector<geom3d::Aabb3> groups(s.parent.size());
    Transform3 workAt = Transform3::Identity();
    geom3d::Aabb3 stockBox;
    auto setHeld = [&](const AxisPosition& at, const AxisPosition& work) {
        motions = s.motions(at);
        std::fill(groups.begin(), groups.end(), geom3d::Aabb3{});
        for (std::size_t i = 0; i < s.moving.size(); ++i) {
            const Impl::Moving& m = s.moving[i];
            geom3d::Aabb3 box;
            if (m.mesh) {
                box = geom3d::transformed(m.mesh->bounds(), motions[m.body] * m.placement);
            } else {
                for (const Vec3& p : m.points) {
                    box.extend(motions[m.body] * p);
                }
            }
            movingBoxes[i] = box;
            groups[m.body].extend(box);
        }
        for (std::size_t i = 0; i < s.others.size(); ++i) {
            const Impl::Other& o = s.others[i];
            geom3d::Aabb3 box;
            if (o.revolved) {
                const double r = o.revolved->rMax();
                box = geom3d::transformed(
                    geom3d::Aabb3{Vec3(-r, -r, o.revolved->zMin()), Vec3(r, r, o.revolved->zMax())},
                    motions[o.body] * o.frame);
            } else if (o.convex) {
                box = o.convex->transformed(motions[o.body]).bounds();
            } else {
                box = geom3d::transformed(o.mesh->bounds(), motions[o.body] * o.placement);
            }
            grow(box, clearance);
            otherBoxes[i] = box;
        }
        AxisPosition still = work;
        still.linear = Vec3::Zero();
        workAt = machine::poseAt(s.model, still).carried(s.model, s.mount.carrier) * s.mount.wcs;
    };

    const Revolved startStock(stock.start);
    if (!startStock.isEmpty()) {
        stockBox = geom3d::Aabb3{
            Vec3(-startStock.rMax(), -startStock.rMax(), startStock.zMin() - clearance),
            Vec3(startStock.rMax(), startStock.rMax(), startStock.zMax() + clearance)};
    }
    std::optional<Revolved> laterStock;
    std::size_t laterMove = std::numeric_limits<std::size_t>::max();

    auto record = [&](std::size_t move, const std::string& a, const std::string& b,
                      const Contact& c, const std::string& during, const Transform3& wcsToWorld) {
        for (auto it = out.rbegin(); it != out.rend() && it->lastMove + 1 >= move; ++it) {
            if (it->first == a && it->second == b && it->during == during) {
                it->lastMove = std::max(it->lastMove, move);
                it->distance = std::min(it->distance, c.distance);
                return;
            }
        }
        if (out.size() < kMaxCollisions) {
            out.push_back({move, move, a, b, wcsToWorld.inverse() * c.point, c.distance, during});
        }
    };

    const geom3d::Aabb3 inside{s.layout.enclosureMin + Vec3::Constant(clearance),
                               s.layout.enclosureMax - Vec3::Constant(clearance)};
    auto insideEnclosure = [&](const geom3d::Aabb3& box) {
        return (box.min.array() >= inside.min.array()).all() &&
               (box.max.array() <= inside.max.array()).all();
    };
    /// The wall a box reaches furthest past, and where.
    auto wallReached =
        [&](const geom3d::Aabb3& box) -> std::optional<std::pair<std::string, Contact>> {
        static const std::array<std::array<const char*, 2>, 3> walls{
            {{"Enclosure, door", "Enclosure, back"},
             {"Enclosure, floor", "Enclosure, roof"},
             {"Enclosure, headstock end", "Enclosure, far end"}}};
        double worst = 0.0;
        std::optional<std::pair<std::string, Contact>> hit;
        for (int k = 0; k < 3; ++k) {
            const double low = inside.min[k] - box.min[k];
            const double high = box.max[k] - inside.max[k];
            if (low > worst) {
                worst = low;
                Vec3 p = 0.5 * (box.min + box.max);
                p[k] = box.min[k];
                hit = std::make_pair(std::string(walls[static_cast<std::size_t>(k)][0]),
                                     Contact{std::max(clearance - low, 0.0), p});
            }
            if (high > worst) {
                worst = high;
                Vec3 p = 0.5 * (box.min + box.max);
                p[k] = box.max[k];
                hit = std::make_pair(std::string(walls[static_cast<std::size_t>(k)][1]),
                                     Contact{std::max(clearance - high, 0.0), p});
            }
        }
        return hit;
    };

    std::vector<Vec3> frame;
    /// A moving solid against an area swept in `frameToWorld`.
    auto solidToArea = [&](const Impl::Moving& m, const Transform3& toWorld, const Revolved& area,
                           const Transform3& frameToWorld) -> std::optional<Contact> {
        const Transform3 toFrame = frameToWorld.inverse() * toWorld;
        std::optional<Contact> hit;
        if (m.mesh) {
            const Transform3 meshToFrame = toFrame * m.placement;
            // The part of the mesh near the area: the area's box in the mesh's frame.
            const double r = area.rMax() + clearance;
            const geom3d::Aabb3 near =
                geom3d::transformed(geom3d::Aabb3{Vec3(-r, -r, area.zMin() - clearance),
                                                  Vec3(r, r, area.zMax() + clearance)},
                                    meshToFrame.inverse());
            m.mesh->query(near, [&](std::size_t t) {
                const std::array<Vec3, 3> tri = m.mesh->triangle(t);
                frame.assign({meshToFrame * tri[0], meshToFrame * tri[1], meshToFrame * tri[2],
                              meshToFrame * tri[0], meshToFrame * tri[1], meshToFrame * tri[2]});
                hit = polytopeToRevolved(frame, area, clearance);
                return !hit;
            });
        } else {
            frame.clear();
            for (const Vec3& p : m.points) {
                frame.push_back(toFrame * p);
            }
            hit = polytopeToRevolved(frame, area, clearance);
        }
        if (hit) {
            hit->point = frameToWorld * hit->point;
        }
        return hit;
    };

    auto checkPose = [&](std::size_t move, const Vec3& slides, int cutting,
                         const std::string& during) {
        const std::size_t count = s.parent.size();
        for (std::size_t b = 0; b < count; ++b) {
            if (groups[b].isEmpty()) {
                continue;
            }
            const Vec3 shift = linearOf(s.gain[b], slides);
            const geom3d::Aabb3 group = shifted(groups[b], shift);
            const Transform3 bodyToWorld = Eigen::Translation3d(shift) * motions[b];
            // The enclosure's inside: a solid's box is exactly how far its vertices reach.
            if (s.layout.hasEnclosure() && !insideEnclosure(group)) {
                for (std::size_t i = 0; i < s.moving.size(); ++i) {
                    const Impl::Moving& m = s.moving[i];
                    if (m.body != b || (m.mesh && !m.wallByBox) || m.slider) {
                        continue;
                    }
                    const geom3d::Aabb3 box = shifted(movingBoxes[i], shift);
                    if (const auto wall = wallReached(box)) {
                        const Transform3 stockFrame =
                            Eigen::Translation3d(linearOf(s.gain[s.workBody], slides)) * workAt;
                        record(move, m.name, wall->first, wall->second, during, stockFrame);
                    }
                }
            }
            for (std::size_t o = 0; o <= s.others.size(); ++o) {
                const bool isStock = o == s.others.size();
                if (isStock ? startStock.isEmpty() : false) {
                    continue;
                }
                const Transform3 stockFrame =
                    Eigen::Translation3d(linearOf(s.gain[s.workBody], slides)) * workAt;
                geom3d::Aabb3 otherBox;
                if (isStock) {
                    otherBox = geom3d::transformed(stockBox, stockFrame);
                } else {
                    otherBox = shifted(otherBoxes[o], linearOf(s.gain[s.others[o].body], slides));
                }
                if (!geom3d::overlaps(group, otherBox)) {
                    continue;
                }
                for (std::size_t i = 0; i < s.moving.size(); ++i) {
                    const Impl::Moving& m = s.moving[i];
                    if (m.body != b ||
                        !geom3d::overlaps(shifted(movingBoxes[i], shift), otherBox)) {
                        continue;
                    }
                    if (m.station != 0 && m.station == cutting && (isStock || s.others[o].work)) {
                        continue; // the tool cutting: the turning checks have these
                    }
                    if (isStock) {
                        if (m.slider && s.sameChain(m.body, s.workBody)) {
                            continue; // the sub-spindle holding the work
                        }
                        std::optional<Contact> hit =
                            solidToArea(m, bodyToWorld, startStock, stockFrame);
                        if (hit && stock.at && !during.size() && move > 0) {
                            // Material goes: is it still there at this move?
                            if (laterMove != move) {
                                laterStock.emplace(stock.at(move));
                                laterMove = move;
                            }
                            hit = laterStock->isEmpty()
                                      ? std::nullopt
                                      : solidToArea(m, bodyToWorld, *laterStock, stockFrame);
                        }
                        if (hit) {
                            const bool inPart =
                                s.part && solidToArea(m, bodyToWorld, *s.part, stockFrame);
                            record(move, m.name, inPart ? "part" : "stock", *hit, during,
                                   stockFrame);
                        }
                        continue;
                    }
                    const Impl::Other& other = s.others[o];
                    if (other.wall && (!m.mesh || m.wallByBox)) {
                        continue; // held to the inside, above
                    }
                    if (m.body == other.body || s.parent[m.body] == other.body ||
                        s.parent[other.body] == m.body) {
                        continue; // they move together or sit on each other
                    }
                    if (m.slider) {
                        if (!other.revolved || s.sameChain(m.body, other.body)) {
                            continue; // only what turns: the rest stands clear of its travel
                        }
                    }
                    const Transform3 otherToWorld =
                        Eigen::Translation3d(linearOf(s.gain[other.body], slides)) *
                        motions[other.body];
                    std::optional<Contact> hit;
                    if (other.revolved) {
                        hit = solidToArea(m, bodyToWorld, *other.revolved,
                                          otherToWorld * other.frame);
                    } else if (m.mesh && other.mesh) {
                        hit = detail::meshToMesh(*m.mesh, bodyToWorld * m.placement, *other.mesh,
                                                 otherToWorld * other.placement, clearance);
                    } else if (m.mesh) {
                        hit = detail::convexToMesh(other.convex->transformed(otherToWorld), *m.mesh,
                                                   bodyToWorld * m.placement, clearance);
                    } else {
                        frame.clear();
                        for (const Vec3& p : m.points) {
                            frame.push_back(bodyToWorld * p);
                        }
                        const geom3d::Convex solid = geom3d::Convex::points(frame);
                        if (other.mesh) {
                            hit = detail::convexToMesh(solid, *other.mesh,
                                                       otherToWorld * other.placement, clearance);
                        } else {
                            const geom3d::ConvexDistance d =
                                geom3d::distance(solid, other.convex->transformed(otherToWorld));
                            if (d.intersecting || d.distance < clearance) {
                                hit = Contact{d.distance, d.pointA};
                            }
                        }
                    }
                    if (hit) {
                        record(move, m.name, other.name, *hit, during, stockFrame);
                    }
                }
            }
        }
    };

    // Between operations: every tool is as any other, and the stock as the operation finds it.
    AxisPosition at = motion.start;
    setHeld(at, at);
    for (std::size_t n = 0; n < motion.steps.size(); ++n) {
        const LatheStep& step = motion.steps[n];
        if (cancel) {
            cancel->throwIfCancelled();
        }
        const double travel = (step.to.linear - at.linear).norm() + std::abs(step.to.w - at.w) +
                              std::abs(step.to.z2 - at.z2) +
                              std::abs(step.to.t - at.t) * s.rapidRadius;
        const int steps = std::clamp(static_cast<int>(std::ceil(travel / kStep)), 1, 4000);
        const std::string during(machine::toString(step.kind));
        const bool heldMoves = step.kind != LatheStep::Kind::Rapid;
        for (int k = 1; k <= steps; ++k) {
            const AxisPosition q = machine::lerpAxes(at, step.to, static_cast<double>(k) / steps);
            if (heldMoves) {
                setHeld(q, step.workStays ? step.to : q);
            }
            // Arriving, the tool is at the toolpath's first position, which may stand closer to
            // the work than the clearance: a feed height. From there it is the tool cutting.
            const bool arrived = n + 1 == motion.steps.size() && k == steps;
            checkPose(0, q.linear, arrived ? motion.station : 0, during);
        }
        at = step.to;
    }

    // The toolpath's moves, each tip put on the program's point by the tool working.
    setHeld(motion.held, motion.held);
    const bool turns = path.live || path.liveAxes != toolpath::LiveAxes::None;
    const Transform3 carrierAt = motions[s.workBody];
    auto slidesFor = [&](const Vec3& tip) -> Vec3 {
        Vec3 target = s.mount.wcs * tip;
        if (turns) {
            AxisPosition q = motion.held;
            q.rotary.c = machine::latheSpindleAngle(path, tip);
            target = machine::poseAt(s.model, q).carried(s.model, s.mount.carrier) * target;
        } else {
            target = carrierAt * target;
        }
        return s.toSlides * (target - motion.tip);
    };
    Vec3 p = path.start;
    checkPose(0, slidesFor(p), motion.station, {});
    double since = 0.0;
    for (std::size_t k = 0; k < path.moves.size(); ++k) {
        if (cancel && (k & 255u) == 0) {
            cancel->throwIfCancelled();
        }
        const toolpath::Move& m = path.moves[k];
        if (m.kind == toolpath::MoveKind::Dwell) {
            continue;
        }
        const double travel = (m.end - p).norm();
        const int steps = std::clamp(static_cast<int>(std::ceil(travel / kStep)), 1, 100000);
        for (int i = 1; i <= steps; ++i) {
            since += travel / steps;
            const bool last = k + 1 == path.moves.size() && i == steps;
            if (since >= kStep - 1e-9 || last ||
                (m.kind == toolpath::MoveKind::Rapid && i == steps)) {
                checkPose(k, slidesFor(p + (m.end - p) * (static_cast<double>(i) / steps)),
                          motion.station, {});
                since = 0.0;
            }
        }
        p = m.end;
    }
    return out;
}

} // namespace ocam::sim
