// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Everything is placed in the model frame (machine::Model.hpp) at each checked pose. Bodies are
// what moves rigidly: the base, each axis, and two attached to an axis: the tool (on the
// spindle's carrier) and the workpiece (part, stock and fixtures, on the table's carrier). Each
// collider is stored in the model frame at machine zero and moved by its body's motion.

#include "ocam/sim/MachineCheck.hpp"

#include "MachineContacts.hpp"

#include <ocam/geom3d/Bvh.hpp>
#include <ocam/geom3d/Convex.hpp>
#include <ocam/machine/Model.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>

namespace ocam::sim {
namespace {

using detail::Contact;
using detail::convexToMesh;
using detail::meshToMesh;
using machine::Component;
using machine::IndexAngles;

IndexAngles lerp(const IndexAngles& a, const IndexAngles& b, double t) {
    return {a.a + (b.a - a.a) * t, a.b + (b.b - a.b) * t, a.c + (b.c - a.c) * t};
}

double angleChange(const IndexAngles& a, const IndexAngles& b) {
    return std::max({std::abs(a.a - b.a), std::abs(a.b - b.b), std::abs(a.c - b.c)});
}

enum class Role {
    Head,      ///< A machine part on the spindle's chain.
    Machine,   ///< Any other machine part.
    Tool,      ///< The cutting part, shank and holder.
    Workpiece, ///< The part and the stock.
    Fixture,
};

struct Collider {
    std::string name;
    std::size_t body = 0;
    Role role = Role::Machine;
    std::optional<geom3d::Convex> convex; ///< Model frame at machine zero.
    std::shared_ptr<const geom3d::TriangleBvh> mesh;
    Transform3 placement = Transform3::Identity(); ///< Mesh -> model frame at machine zero.
};

Transform3 translation(const Vec3& v) {
    Transform3 xf = Transform3::Identity();
    xf.translation() = v;
    return xf;
}

} // namespace

double gaugeLength(const ToolShape& tool) {
    double gauge = tool.visibleLength();
    for (const HolderBand& b : tool.holder()) {
        gauge = std::max(gauge, b.h1);
    }
    return gauge;
}

std::vector<MachineCollision>
checkMachine(const machine::Machine& machine, const toolpath::Toolpath& path,
             const Transform3& wcsInTable, const geom3d::Mesh& partWcs,
             const geom3d::Aabb3& stockWcs, const ToolShape& tool,
             const MachineCheckOptions& options, CancelToken* cancel) {
    return checkMachine(machine, path,
                        machine::tableMount(machine::effectiveModel(machine), wcsInTable), partWcs,
                        stockWcs, {}, tool, options, cancel);
}

std::vector<MachineCollision>
checkMachine(const machine::Machine& machine, const toolpath::Toolpath& path,
             const machine::WorkMount& mount, const geom3d::Mesh& partWcs,
             const geom3d::Aabb3& stockWcs, const std::vector<FixtureBody>& fixturesWcs,
             const ToolShape& tool, const MachineCheckOptions& options, CancelToken* cancel) {
    std::vector<MachineCollision> out;
    const bool parts = std::any_of(machine.model.parts.begin(), machine.model.parts.end(),
                                   [](const machine::ModelPart& p) { return p.collides; });
    if ((machine.components.empty() && !parts) || path.moves.empty() || machine.isLathe()) {
        return out;
    }
    const machine::MachineModel model = machine::effectiveModel(machine);
    const std::size_t n = path.moves.size();
    const Transform3 wcsInTable = machine::wcsInTable(model, mount);
    const machine::RotaryTrack track =
        machine::rotaryTrack(machine, path, wcsInTable, options.index ? &*options.index : nullptr);
    if (!track.errors.empty()) {
        return out; // unreachable motion is reported by solve
    }
    const Transform3 mounted = machine::mountedWcs(machine, path, wcsInTable);
    const double gauge = gaugeLength(tool);

    // Bodies: 0 the base, 1 + i axis i, then the tool and the workpiece.
    auto bodyOf = [&](std::string_view carrier) -> std::size_t {
        for (std::size_t i = 0; i < model.axes.size(); ++i) {
            if (model.axes[i].name == carrier) {
                return i + 1;
            }
        }
        return 0;
    };
    const std::size_t toolBody = model.axes.size() + 1;
    const std::size_t workBody = model.axes.size() + 2;
    std::vector<std::size_t> parent(model.axes.size() + 3, 0);
    for (std::size_t i = 0; i < model.axes.size(); ++i) {
        parent[i + 1] = bodyOf(model.axes[i].parent);
    }
    const std::size_t spindleBody = bodyOf(model.spindleCarrier);
    const std::size_t tableBody = bodyOf(machine::tableEnd(model));
    const std::size_t carrierBody = bodyOf(mount.carrier);
    parent[toolBody] = spindleBody;
    parent[workBody] = carrierBody;
    std::vector<bool> onSpindleChain(parent.size(), false);
    for (std::size_t b = spindleBody; b != 0; b = parent[b]) {
        onSpindleChain[b] = true;
    }

    std::vector<Collider> colliders;
    auto machineRole = [&](std::size_t body) {
        return onSpindleChain[body] ? Role::Head : Role::Machine;
    };
    const Transform3 spindleZero = translation(model.spindlePoint);
    // Components sit at the table centre; the work at the centre for its WCS origin.
    const Transform3 tableZero = translation(machine::tableCentre(model, Vec3::Zero()));
    const std::size_t cradleBody = bodyOf(machine::cradleCarrier(model));
    for (const Component& c : machine.components) {
        Collider col;
        col.name = c.name;
        switch (c.attach) {
        case Component::Attach::Head:
            col.body = spindleBody;
            col.convex = c.convex().transformed(spindleZero);
            break;
        case Component::Attach::Table:
            col.body = tableBody;
            col.convex = c.convex().transformed(tableZero);
            break;
        case Component::Attach::Cradle:
            col.body = cradleBody;
            col.convex = c.convex().transformed(tableZero);
            break;
        }
        col.role = machineRole(col.body);
        colliders.push_back(std::move(col));
    }
    for (const machine::ModelPart& p : machine.model.parts) {
        if (!p.collides || !p.geometry || p.geometry->isEmpty()) {
            continue;
        }
        Collider col;
        col.name = p.name;
        col.body = bodyOf(p.carrier);
        col.role = machineRole(col.body);
        col.mesh = std::make_shared<const geom3d::TriangleBvh>(*p.geometry);
        col.placement = p.placement();
        colliders.push_back(std::move(col));
    }
    // The tool assembly, tip at the origin and axis +Z, then at machine zero.
    const Transform3 toolZero = spindleZero * translation(Vec3(0, 0, -gauge));
    auto addTool = [&](std::string name, const geom3d::Convex& shape) {
        Collider col;
        col.name = std::move(name);
        col.body = toolBody;
        col.role = Role::Tool;
        col.convex = shape.transformed(toolZero);
        colliders.push_back(std::move(col));
    };
    addTool("tool", geom3d::Convex::cylinder(Vec3::Zero(), Vec3::UnitZ(), tool.cutRadius(),
                                             tool.cuttingLength()));
    if (tool.shankRadius() > 0.0 && tool.visibleLength() > tool.cuttingLength()) {
        addTool("tool shank", geom3d::Convex::cylinder(
                                  Vec3(0, 0, tool.cuttingLength()), Vec3::UnitZ(),
                                  tool.shankRadius(), tool.visibleLength() - tool.cuttingLength()));
    }
    for (const HolderBand& b : tool.holder()) {
        addTool("tool holder",
                geom3d::Convex::cylinder(Vec3(0, 0, b.h0), Vec3::UnitZ(), b.radius, b.h1 - b.h0));
    }
    // The workpiece: WCS -> model frame at machine zero.
    const Transform3 wcsZero =
        translation(machine::tableCentre(model, mount.wcs.translation())) * mounted;
    if (!partWcs.isEmpty()) {
        Collider col;
        col.name = "part";
        col.body = workBody;
        col.role = Role::Workpiece;
        col.mesh = std::make_shared<const geom3d::TriangleBvh>(partWcs);
        col.placement = wcsZero;
        colliders.push_back(std::move(col));
    }
    if (!stockWcs.isEmpty()) {
        std::vector<Vec3> corners;
        for (int i = 0; i < 8; ++i) {
            corners.push_back(wcsZero * Vec3((i & 1) ? stockWcs.max.x() : stockWcs.min.x(),
                                             (i & 2) ? stockWcs.max.y() : stockWcs.min.y(),
                                             (i & 4) ? stockWcs.max.z() : stockWcs.min.z()));
        }
        Collider col;
        col.name = "stock";
        col.body = workBody;
        col.role = Role::Workpiece;
        col.convex = geom3d::Convex::points(std::move(corners));
        colliders.push_back(std::move(col));
    }
    for (const FixtureBody& f : fixturesWcs) {
        if (f.mesh.isEmpty()) {
            continue;
        }
        Collider col;
        col.name = f.name;
        col.body = workBody;
        col.role = Role::Fixture;
        col.mesh = std::make_shared<const geom3d::TriangleBvh>(f.mesh);
        col.placement = wcsZero;
        colliders.push_back(std::move(col));
    }

    // The pairs to check, the head-side or machine part first.
    struct Pair {
        std::size_t a = 0, b = 0;
        double clearance = 0.0;
    };
    std::vector<Pair> pairs;
    auto rank = [](Role r) {
        return static_cast<int>(r);
    };
    for (std::size_t i = 0; i < colliders.size(); ++i) {
        for (std::size_t j = i + 1; j < colliders.size(); ++j) {
            const Collider& x = colliders[i];
            const Collider& y = colliders[j];
            if (x.body == y.body || parent[x.body] == y.body || parent[y.body] == x.body) {
                continue; // they move together or sit on each other
            }
            const bool toolPair = x.role == Role::Tool || y.role == Role::Tool;
            const bool onTable = x.role == Role::Workpiece || y.role == Role::Workpiece ||
                                 x.role == Role::Fixture || y.role == Role::Fixture;
            if (toolPair && onTable) {
                // The stock simulation checks the tool against the part and stock, and
                // checkFixtures against the fixtures.
                continue;
            }
            pairs.push_back(rank(x.role) <= rank(y.role) ? Pair{i, j, options.clearance}
                                                         : Pair{j, i, options.clearance});
        }
    }
    if (pairs.empty()) {
        return out;
    }

    auto record = [&](std::size_t move, const std::string& a, const std::string& b,
                      const Contact& c, const Transform3& wcsToWorld) {
        for (auto it = out.rbegin(); it != out.rend() && it->lastMove + 1 >= move; ++it) {
            if (it->first == a && it->second == b) {
                it->lastMove = std::max(it->lastMove, move);
                it->distance = std::min(it->distance, c.distance);
                return;
            }
        }
        if (out.size() < options.maxCollisions) {
            out.push_back({move, move, a, b, wcsToWorld.inverse() * c.point, c.distance});
        }
    };

    std::vector<Transform3> bodies(parent.size(), Transform3::Identity());
    std::vector<std::optional<geom3d::Convex>> moved(colliders.size());
    std::vector<geom3d::Aabb3> boxes(colliders.size());
    auto checkPose = [&](std::size_t move, const Vec3& tipWcs, const IndexAngles& q) {
        const auto position =
            machine::positionFor(model, q, mount.carrier, wcsZero * tipWcs, gauge);
        if (!position) {
            return;
        }
        const machine::ModelPose pose = machine::poseAt(model, *position);
        for (std::size_t i = 0; i < model.axes.size(); ++i) {
            bodies[i + 1] = pose.axes[i];
        }
        bodies[toolBody] = bodies[spindleBody];
        bodies[workBody] = bodies[carrierBody];
        for (std::size_t i = 0; i < colliders.size(); ++i) {
            const Collider& c = colliders[i];
            if (c.convex) {
                moved[i] = c.convex->transformed(bodies[c.body]);
                boxes[i] = moved[i]->bounds();
            } else {
                boxes[i] = geom3d::transformed(c.mesh->bounds(), bodies[c.body] * c.placement);
            }
        }
        const Transform3 wcsToWorld = bodies[workBody] * wcsZero;
        for (const Pair& p : pairs) {
            const Collider& x = colliders[p.a];
            const Collider& y = colliders[p.b];
            geom3d::Aabb3 grown = boxes[p.a];
            grown.min -= Vec3::Constant(p.clearance);
            grown.max += Vec3::Constant(p.clearance);
            if (!geom3d::overlaps(grown, boxes[p.b])) {
                continue;
            }
            std::optional<Contact> hit;
            if (x.convex && y.convex) {
                const geom3d::ConvexDistance d = geom3d::distance(*moved[p.a], *moved[p.b]);
                if (d.intersecting || d.distance < p.clearance) {
                    hit = Contact{d.distance, d.pointA};
                }
            } else if (x.convex) {
                hit = convexToMesh(*moved[p.a], *y.mesh, bodies[y.body] * y.placement, p.clearance);
            } else if (y.convex) {
                hit = convexToMesh(*moved[p.b], *x.mesh, bodies[x.body] * x.placement, p.clearance);
            } else {
                hit = meshToMesh(*x.mesh, bodies[x.body] * x.placement, *y.mesh,
                                 bodies[y.body] * y.placement, p.clearance);
            }
            if (hit) {
                record(move, x.name, y.name, *hit, wcsToWorld);
            }
        }
    };

    // Poses every `step` of tip travel or `angleStep` of rotary motion, at the ends of rapids
    // and at the end of the toolpath.
    Vec3 p = path.start;
    IndexAngles q = track.start;
    checkPose(0, p, q);
    double since = 0.0;
    IndexAngles checked = q;
    for (std::size_t k = 0; k < n; ++k) {
        if (cancel && (k & 255u) == 0) {
            cancel->throwIfCancelled();
        }
        const auto& m = path.moves[k];
        if (m.kind == toolpath::MoveKind::Dwell) {
            continue;
        }
        const double travel = (m.end - p).norm();
        const int steps = std::max(
            1, static_cast<int>(std::min(
                   1e5, std::max(std::ceil(travel / options.step),
                                 std::ceil(angleChange(q, track.angles[k]) / options.angleStep)))));
        for (int s = 1; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            const IndexAngles qs = lerp(q, track.angles[k], t);
            since += travel / steps;
            const bool last = k + 1 == n && s == steps;
            if (since >= options.step - 1e-9 || angleChange(checked, qs) >= options.angleStep ||
                last || m.kind == toolpath::MoveKind::Rapid) {
                checkPose(k, p + (m.end - p) * t, qs);
                since = 0.0;
                checked = qs;
            }
        }
        p = m.end;
        q = track.angles[k];
    }
    return out;
}

} // namespace ocam::sim
