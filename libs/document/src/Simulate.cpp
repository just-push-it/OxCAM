// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/document/Simulate.hpp"

#include "ocam/document/Generate.hpp"
#include "ocam/document/LatheMachine.hpp"
#include "ocam/document/PostSetup.hpp"
#include "ocam/document/ProjectIo.hpp"

#include <ocam/cad/Tessellation.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Sha256.hpp>
#include <ocam/foundation/Version.hpp>
#include <ocam/geom2d/Clip.hpp>
#include <ocam/sim/Surface.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ocam::document {
namespace {

/// The operation's toolpath: the caller's cached one, or generated.
toolpath::Toolpath toolpathOf(const Document& doc, const Setup& setup, const Operation& op,
                              const SimulationOptions& options, CancelToken& cancel,
                              ProgressSink& progress) {
    if (options.toolpathFor) {
        if (std::optional<toolpath::Toolpath> cached = options.toolpathFor(op)) {
            return std::move(*cached);
        }
    }
    return *cachedToolpath(doc, setup, op, cancel, progress);
}

/// Whether an operation runs at a fixed tool orientation of its own (3+2): every move of its
/// toolpath (WCS) along the orientation's Z axis.
bool positional(const Operation& op, const toolpath::Toolpath& wcsPath) {
    if (!op.orientation) {
        return false;
    }
    const Vec3 z = operationFrame(op).linear().col(2);
    return std::all_of(wcsPath.moves.begin(), wcsPath.moves.end(),
                       [&](const toolpath::Move& m) { return (m.axis - z).norm() < 1e-9; });
}

/// Sorts per-ray intervals and merges those that overlap.
void tidy(sim::IntervalList& list) {
    std::sort(list.begin(), list.end(),
              [](const sim::Interval& a, const sim::Interval& b) { return a.lo < b.lo; });
    list = sim::unite(list, {});
}

/// The material chamfered in an earlier setup (`source`, on its grid) on the grid of `target`:
/// points along its rays a third of a spacing apart, moved by `sourceToTarget`, mark the target
/// rays within a spacing of them, a spacing above and below.
std::vector<sim::IntervalList> moveChamfered(const SetupSimulation& source,
                                             const Transform3& sourceToTarget,
                                             const sim::DexelGrid& target) {
    std::vector<sim::IntervalList> out;
    const sim::DexelGrid& from = source.finalStock().grid(2);
    const std::size_t count =
        static_cast<std::size_t>(target.nu()) * static_cast<std::size_t>(target.nv());
    if (source.chamfered.size() !=
        static_cast<std::size_t>(from.nu()) * static_cast<std::size_t>(from.nv())) {
        return out;
    }
    const double hs = from.spacing();
    const double h = target.spacing();
    const double step = std::min(hs, h) / 3.0;
    for (std::size_t idx = 0; idx < source.chamfered.size(); ++idx) {
        const sim::IntervalList& list = source.chamfered[idx];
        if (list.empty()) {
            continue;
        }
        if (out.empty()) {
            out.resize(count);
        }
        const int i = static_cast<int>(idx % static_cast<std::size_t>(from.nu()));
        const int j = static_cast<int>(idx / static_cast<std::size_t>(from.nu()));
        const double x = from.u0() + (i + 0.5) * hs;
        const double y = from.v0() + (j + 0.5) * hs;
        for (const sim::Interval& in : list) {
            const int n =
                std::max(1, static_cast<int>(std::ceil(
                                (static_cast<double>(in.hi) - static_cast<double>(in.lo)) / step)));
            for (int k = 0; k <= n; ++k) {
                const Vec3 p =
                    sourceToTarget *
                    Vec3(x, y,
                         static_cast<double>(in.lo) +
                             (static_cast<double>(in.hi) - static_cast<double>(in.lo)) * k / n);
                const double fu = (p.x() - target.u0()) / h - 0.5;
                const double fv = (p.y() - target.v0()) / h - 0.5;
                for (int tj = static_cast<int>(std::ceil(fv - 1.0));
                     tj <= static_cast<int>(std::floor(fv + 1.0)); ++tj) {
                    for (int ti = static_cast<int>(std::ceil(fu - 1.0));
                         ti <= static_cast<int>(std::floor(fu + 1.0)); ++ti) {
                        if (ti < 0 || tj < 0 || ti >= target.nu() || tj >= target.nv()) {
                            continue;
                        }
                        out[static_cast<std::size_t>(tj) * static_cast<std::size_t>(target.nu()) +
                            static_cast<std::size_t>(ti)]
                            .push_back(
                                {static_cast<float>(p.z() - h), static_cast<float>(p.z() + h)});
                    }
                }
            }
        }
    }
    for (sim::IntervalList& list : out) {
        tidy(list);
    }
    return out;
}

/// The setup's chamfered material (SetupSimulation::chamfered): the stock's from earlier setups
/// and what each chamfer operation removed near its edges (within its width, a ray spacing
/// and the tolerance, down to its depth). What a chamfer cuts anywhere else is not in it.
std::vector<sim::IntervalList> chamferedMaterial(const Document& doc, const Setup& setup,
                                                 const SimulationOptions& options,
                                                 const SetupSimulation& sim) {
    std::vector<sim::IntervalList> out = sim.initialChamfered;
    const sim::DexelGrid& grid = sim.finalStock().grid(2);
    const std::size_t count =
        static_cast<std::size_t>(grid.nu()) * static_cast<std::size_t>(grid.nv());
    const double margin = grid.spacing() + options.tolerance;
    for (std::size_t k = 0; k < sim.operations.size(); ++k) {
        const OperationSimulation& os = sim.operations[k];
        const auto op = std::find_if(setup.operations.begin(), setup.operations.end(),
                                     [&](const Operation& o) { return o.id == os.operationId; });
        if (op == setup.operations.end() || !os.error.empty()) {
            continue;
        }
        const std::vector<ChamferZone> zones = chamferZones(doc, setup, *op);
        // A thread mill takes the thread out of a hole modelled at its tap drill: as a chamfer's
        // cut, all of it is meant, and not required of the stock.
        const bool thread = op->type == "threadmill";
        if (zones.empty() && !thread) {
            continue;
        }
        const sim::DexelGrid& before = sim.stockBefore(k).grid(2);
        const sim::DexelGrid& after = os.stockAfter.grid(2);
        if (before.nu() != grid.nu() || before.nv() != grid.nv() || after.nu() != grid.nu() ||
            after.nv() != grid.nv()) {
            continue;
        }
        struct Band {
            geom2d::Regions area;
            double lo, hi;
        };
        std::vector<Band> bands;
        for (const ChamferZone& z : zones) {
            if (!z.band.empty()) {
                bands.push_back(
                    {geom2d::offset(z.band, margin), z.z - z.depth - margin, z.z + margin});
                continue;
            }
            const double reach = z.width + margin;
            bands.push_back({geom2d::subtract(geom2d::offset(z.outline, reach),
                                              geom2d::offset(z.outline, -reach)),
                             z.z - z.depth - margin, z.z + margin});
        }
        if (out.size() != count) {
            out.assign(count, {});
        }
        constexpr int T = sim::DexelGrid::kTile;
        // Only the tiles the chamfer changed: few.
        for (int tj = 0; tj < after.tileRows(); ++tj) {
            for (int ti = 0; ti < after.tileColumns(); ++ti) {
                if (after.tileId(ti, tj) == before.tileId(ti, tj)) {
                    continue;
                }
                for (int j = tj * T; j < std::min((tj + 1) * T, grid.nv()); ++j) {
                    for (int i = ti * T; i < std::min((ti + 1) * T, grid.nu()); ++i) {
                        const sim::IntervalList& now = after.ray(i, j);
                        sim::IntervalList removed = before.ray(i, j);
                        if (removed == now) {
                            continue;
                        }
                        for (const sim::Interval& in : now) {
                            sim::subtract(removed, static_cast<double>(in.lo),
                                          static_cast<double>(in.hi));
                        }
                        const Vec2 p(grid.u0() + (i + 0.5) * grid.spacing(),
                                     grid.v0() + (j + 0.5) * grid.spacing());
                        sim::IntervalList& list =
                            out[static_cast<std::size_t>(j) * static_cast<std::size_t>(grid.nu()) +
                                static_cast<std::size_t>(i)];
                        if (thread) {
                            list = sim::unite(list, removed);
                        }
                        for (const Band& b : bands) {
                            if (std::none_of(
                                    b.area.begin(), b.area.end(),
                                    [&](const geom2d::Region& r) { return r.contains(p); })) {
                                continue;
                            }
                            const sim::IntervalList within = sim::intersect(
                                removed, {{static_cast<float>(b.lo), static_cast<float>(b.hi)}});
                            list = sim::unite(list, within);
                        }
                    }
                }
            }
        }
    }
    return out;
}

/// The final comparison, with the part prepared once for the playback frames as well. The
/// chamfered material is not required of the stock: chamfers are seldom modelled.
void compareFinal(const SimulationOptions& options, SetupSimulation& out) {
    auto prepared = std::make_shared<sim::PartReference>(
        sim::preparePart(out.finalStock(), out.part, {options.tolerance}));
    if (out.chamfered.size() == prepared->eroded.size()) {
        for (std::size_t idx = 0; idx < out.chamfered.size(); ++idx) {
            for (const sim::Interval& in : out.chamfered[idx]) {
                sim::subtract(prepared->eroded[idx], static_cast<double>(in.lo),
                              static_cast<double>(in.hi));
            }
        }
    }
    std::shared_ptr<const sim::PartReference> reference = std::move(prepared);
    out.comparison = sim::compareToPart(out.finalStock(), *reference);
    out.partReference = std::move(reference);
}

/// Closed cylinder mesh (outward triangles); with an inner diameter, a tube (the bore's
/// triangles face the axis).
geom3d::Mesh cylinderMesh(const Vec3& base, const Vec3& axisIn, double diameter, double length,
                          double innerDiameter = 0.0) {
    const Vec3 axis = axisIn.normalized();
    const Vec3 u =
        (std::abs(axis.x()) < 0.9 ? Vec3::UnitX() : Vec3::UnitY()).cross(axis).normalized();
    const Vec3 v = axis.cross(u);
    const double r = diameter / 2.0;
    const double ri = innerDiameter / 2.0;
    const double tol = std::min(0.01, r * 0.01);
    const int n = std::clamp(static_cast<int>(std::ceil(kPi / std::acos(1.0 - tol / r))), 64, 720);
    geom3d::Mesh m;
    for (int k = 0; k < n; ++k) {
        const double a = 2.0 * kPi * k / n;
        const Vec3 p = base + r * (std::cos(a) * u + std::sin(a) * v);
        m.vertices.push_back(p);
        m.vertices.push_back(p + axis * length);
    }
    if (ri > 0.0) {
        // Bore vertices 2n.., at the same angles, so the end rings pair up.
        for (int k = 0; k < n; ++k) {
            const double a = 2.0 * kPi * k / n;
            const Vec3 p = base + ri * (std::cos(a) * u + std::sin(a) * v);
            m.vertices.push_back(p);
            m.vertices.push_back(p + axis * length);
        }
        const auto o = static_cast<std::uint32_t>(2 * n);
        for (int k = 0; k < n; ++k) {
            const auto b0 = static_cast<std::uint32_t>(2 * k);
            const auto b1 = static_cast<std::uint32_t>(2 * ((k + 1) % n));
            m.triangles.push_back({b0, b1, b1 + 1});
            m.triangles.push_back({b0, b1 + 1, b0 + 1});
            m.triangles.push_back({o + b0, o + b1 + 1, o + b1}); // bore, facing the axis
            m.triangles.push_back({o + b0, o + b0 + 1, o + b1 + 1});
            m.triangles.push_back({b0, o + b1, b1}); // bottom ring, facing -axis
            m.triangles.push_back({b0, o + b0, o + b1});
            m.triangles.push_back({b0 + 1, b1 + 1, o + b1 + 1}); // top ring, facing +axis
            m.triangles.push_back({b0 + 1, o + b1 + 1, o + b0 + 1});
        }
        return m;
    }
    const auto c = static_cast<std::uint32_t>(m.vertices.size());
    m.vertices.push_back(base);
    m.vertices.push_back(base + axis * length);
    for (int k = 0; k < n; ++k) {
        const auto b0 = static_cast<std::uint32_t>(2 * k);
        const auto b1 = static_cast<std::uint32_t>(2 * ((k + 1) % n));
        m.triangles.push_back({b0, b1, b1 + 1});
        m.triangles.push_back({b0, b1 + 1, b0 + 1});
        m.triangles.push_back({c, b1, b0});
        m.triangles.push_back({c + 1, b0 + 1, b1 + 1});
    }
    return m;
}

} // namespace

namespace {

/// What of the part a hole took: its volume, and how far in from the hole's wall it goes.
struct HoleInPart {
    double volume = 0.0;
    double deepest = 0.0;
};

/// Rays along +X through the part: inside it when they cross it an odd number of times.
bool insideMesh(const geom3d::TriangleBvh& mesh, const Vec3& p) {
    Vec3 origin = p;
    int crossings = 0;
    for (int k = 0; k < 64; ++k) {
        const auto hit = mesh.raycast(origin, Vec3::UnitX());
        if (!hit) {
            break;
        }
        ++crossings;
        origin = hit->point + Vec3(1e-5, 0.0, 0.0);
    }
    return crossings % 2 == 1;
}

/// Each of the stock's holes against the part, sampled a tolerance inside its wall.
std::vector<HoleInPart> holesInPart(const SetupSimulation& sim, const sim::LatheStock& stock,
                                    double tolerance) {
    std::vector<HoleInPart> out(stock.holes().size());
    if (!sim.partBvh || sim.partBvh->isEmpty()) {
        return out;
    }
    const geom3d::TriangleBvh& part = *sim.partBvh;
    for (std::size_t k = 0; k < stock.holes().size(); ++k) {
        const sim::LatheHoleCut& h = stock.holes()[k];
        constexpr int kAlong = 14, kRings = 4, kAround = 10;
        const Vec3 ref = std::abs(h.axis.z()) < 0.9 ? Vec3::UnitZ() : Vec3::UnitX();
        const Vec3 ex = h.axis.cross(ref).normalized();
        const Vec3 ey = h.axis.cross(ex).normalized();
        double radius = 0.0;
        for (const Vec2& q : h.profile) {
            radius = std::max(radius, q.y());
        }
        const double inner = radius - 2.0 * tolerance;
        if (inner <= 0.0) {
            continue;
        }
        // A hole's tip end: the cone is narrower than the sample rings, so start past it.
        double from = 0.0;
        for (const Vec2& q : h.profile) {
            if (q.y() >= radius - 1e-9) {
                from = q.x();
                break;
            }
        }
        const double span = std::max(h.length - from, 0.0);
        for (int i = 0; i < kAlong; ++i) {
            const double a = from + (i + 0.5) * span / kAlong;
            for (int ring = 0; ring < kRings; ++ring) {
                const double rho = inner * (ring + 1.0) / kRings;
                for (int j = 0; j < kAround; ++j) {
                    const double angle = 2.0 * kPi * (j + 0.5) / kAround;
                    const Vec3 x =
                        h.tip + a * h.axis + rho * (std::cos(angle) * ex + std::sin(angle) * ey);
                    if (insideMesh(part, x)) {
                        out[k].volume +=
                            kPi * radius * radius * (span / kAlong) / (kRings * kAround);
                        out[k].deepest = std::max(out[k].deepest, radius - rho);
                    }
                }
            }
        }
    }
    return out;
}

/// The comparison of the profile, with what the cuts add. A hole through material the part has is
/// a gouge. A part hollower than its revolved profile (a modelled hole, or a modelled thread)
/// that the cuts have not taken yet is excess, in total only: nothing says where it lies.
void addCutsToComparison(const SetupSimulation& sim, const sim::LatheStock& stock,
                         sim::Comparison& c) {
    if (!stock.holes().empty()) {
        for (const HoleInPart& h : holesInPart(sim, stock, c.tolerance)) {
            if (h.volume > 0.0) {
                ++c.gougeRays;
                c.gougeVolume += h.volume;
                c.maxGouge = std::max(c.maxGouge, h.deepest);
            }
        }
    }
    if (sim.partBvh && !sim.part.isEmpty()) {
        const double round = sim::revolvedVolume(sim.partRegions);
        const double hollow = round - sim.part.signedVolume();
        // Tessellation leaves about this much between the mesh and the profile.
        if (hollow > 1e-3 * round) {
            const double cut = sim::revolvedVolume(stock.profile()) - stock.volume();
            if (hollow - cut > 0.1 * hollow) {
                ++c.excessRays;
                c.excessVolume += hollow - cut;
            }
        }
    }
}

} // namespace

const sim::Stock& SetupSimulation::finalStock() const noexcept {
    return operations.empty() ? initialStock : operations.back().stockAfter;
}

sim::Stock SetupSimulation::lastStock(bool triDexel) const {
    if (!latheInitial || milledFrom) {
        return finalStock();
    }
    const auto last = latheBefore(operations.size());
    return sim::latheStockDexels(last ? *last : *latheInitial, resolution, triDexel);
}

const sim::Stock& SetupSimulation::stockBefore(std::size_t index) const noexcept {
    return index == 0 || operations.empty()
               ? initialStock
               : operations[std::min(index, operations.size()) - 1].stockAfter;
}

std::shared_ptr<const sim::LatheStock>
SetupSimulation::latheBefore(std::size_t index) const noexcept {
    return index == 0 || operations.empty()
               ? latheInitial
               : operations[std::min(index, operations.size()) - 1].latheAfter;
}

std::size_t SetupSimulation::collisionCount() const noexcept {
    std::size_t n = 0;
    for (const auto& op : operations) {
        n += op.collisions.size();
    }
    return n;
}

std::size_t SetupSimulation::fixtureCollisionCount() const noexcept {
    std::size_t n = 0;
    for (const auto& op : operations) {
        n += op.fixtureCollisions.size();
    }
    return n;
}

std::size_t SetupSimulation::machineCollisionCount() const noexcept {
    std::size_t n = 0;
    for (const auto& op : operations) {
        n += op.machineCollisions.size();
    }
    return n;
}

bool SetupSimulation::hasErrors() const noexcept {
    return std::any_of(operations.begin(), operations.end(),
                       [](const auto& op) { return !op.error.empty(); });
}

double automaticResolution(const geom3d::Aabb3& stockBounds) {
    const Vec3 size = stockBounds.size();
    const double side = std::max({size.x(), size.y(), 1.0});
    const double raw = side / 800.0;
    const double decade = std::pow(10.0, std::floor(std::log10(raw)));
    double nice = decade;
    for (const double m : {1.0, 2.0, 2.5, 5.0}) {
        if (m * decade <= raw * (1.0 + 1e-9)) {
            nice = m * decade;
        }
    }
    return std::clamp(nice, 0.05, 2.0);
}

std::optional<geom3d::Mesh> stockMeshInWcs(const Document& doc, const Setup& setup) {
    const std::optional<StockShape> shape = doc.stockShape(setup.id);
    if (!shape) {
        return std::nullopt;
    }
    geom3d::Mesh mesh;
    if (shape->kind == StockShape::Kind::Box) {
        mesh = sim::boxMesh(shape->box);
        mesh.transform(shape->boxToProject());
    } else {
        mesh = cylinderMesh(shape->base, shape->axis, shape->diameter, shape->length,
                            shape->innerDiameter);
    }
    mesh.transform(setup.wcs.toParent().inverse());
    return mesh;
}

namespace {

/// Part mesh in the WCS and the comparison against the final stock.
void comparePart(const Document& doc, const Setup& setup, const SimulationOptions& options,
                 SetupSimulation& out, CancelToken& cancel, ProgressSink& progress, double share) {
    const Project& project = doc.project();
    if (setup.modelId) {
        const ModelEntry* entry = project.findModel(*setup.modelId);
        const cad::Model* model = doc.model(*setup.modelId);
        if (entry && model) {
            try {
                out.part = cad::tessellate(*model, {options.tessellationDeflection, 0.35});
                out.part.transform(modelToWcs(*entry, setup));
            } catch (const Error& e) {
                out.warnings.push_back(
                    fmt::format("the part could not be meshed for comparison: {}", e.what()));
                out.part = {};
            }
        }
    }
    if (options.compare && !out.part.isEmpty()) {
        cancel.throwIfCancelled();
        progress.report(share, "Comparing with the part");
        compareFinal(options, out);
    }
    progress.report(1.0, "Simulated");
}

/// Lathe setups: turning on the axisymmetric stock.
SetupSimulation simulateLatheSetup(const Document& doc, const Setup& setup,
                                   const SimulationOptions& options, CancelToken& cancel,
                                   ProgressSink& progress) {
    const Project& project = doc.project();
    SetupSimulation out;
    out.setupId = setup.id;
    const LatheGeometry geometry = latheGeometry(doc, setup, cancel);
    sim::LatheStock current(geometry.stock);
    current.takeCuts(geometry.carriedCuts, Transform3::Identity());
    geom2d::Aabb2 sb;
    for (const auto& r : geometry.stock) {
        sb.extend(r.bounds());
    }
    geom3d::Aabb3 bounds;
    bounds.extend(Vec3(-sb.max.y(), -sb.max.y(), sb.min.x()));
    bounds.extend(Vec3(sb.max.y(), sb.max.y(), sb.max.x()));
    out.resolution = options.resolution > 0.0 ? options.resolution : automaticResolution(bounds);
    out.latheInitial = std::make_shared<const sim::LatheStock>(current);
    out.warnings.insert(out.warnings.end(), geometry.warnings.begin(), geometry.warnings.end());
    // The chuck (fixtures and workholding), as the areas they sweep turning with the spindle.
    std::vector<sim::LatheBody> bodies;
    if (hasFixtures(setup)) {
        try {
            bodies = latheBodies(doc, setup, 0.0);
        } catch (const Error& e) {
            out.warnings.push_back(
                fmt::format("the chuck is not checked for collisions: {}", e.what()));
        }
    }

    std::vector<const Operation*> ops;
    for (const Operation& op : setup.operations) {
        if (op.enabled) {
            ops.push_back(&op);
        }
        if (options.lastOperation && op.id == *options.lastOperation) {
            break;
        }
    }
    const double opShare = options.compare ? 0.9 : 1.0;
    // A stock an earlier lathe setup milled: that setup's simulation has what it left. Without
    // milling, the profile and its cuts carried over are all there is to know.
    std::shared_ptr<const SetupSimulation> source;
    const std::vector<const Setup*> sources = stockSources(project, setup);
    if (stockIsMilled(project, setup)) {
        ProgressRange sourceProgress(progress, 0.0, 0.0);
        source =
            options.sourceSimulation
                ? options.sourceSimulation(*sources.back(), cancel, sourceProgress)
                : std::make_shared<const SetupSimulation>(simulateSetup(
                      doc, sources.back()->id, sourceOptions(options), cancel, sourceProgress));
    }
    // A live tool milling: from then on the stock is dexels, as nothing else holds what it leaves.
    const bool mills = source || std::any_of(ops.begin(), ops.end(), [&](const Operation* op) {
                           return liveMills(project, *op);
                       });
    if (mills) {
        out.initialStock = sim::latheStockDexels(current, out.resolution, options.triDexel);
    }
    std::optional<sim::Stock> milled;
    if (source) {
        const Setup& from = *sources.back();
        const Transform3 fromToHere = setup.wcs.toParent().inverse() * from.wcs.toParent();
        out.initialStock =
            sim::resampleStock(source->lastStock(options.triDexel), fromToHere, out.initialStock);
        // What is left of the milled stock inside the profile: not what was parted off.
        sim::clipToLathe(out.initialStock, current);
        out.initialChamfered = moveChamfered(*source, fromToHere, out.initialStock.grid(2));
        milled = out.initialStock;
        out.milledFrom = 0;
        for (const OperationSimulation& op : source->operations) {
            if (!op.error.empty()) {
                out.warnings.push_back(
                    fmt::format("the stock from setup '{}' leaves out '{}', which failed: {}",
                                from.name, op.name, op.error));
            }
        }
    }
    const auto dexelsNow = [&] {
        sim::Stock now = out.initialStock;
        sim::clipToLathe(now, current);
        return now;
    };
    const auto finished = [&](OperationSimulation&& os) {
        if (mills) {
            os.stockAfter = milled ? *milled : dexelsNow();
        }
        out.operations.push_back(std::move(os));
    };
    // The part as it comes off: before the bar pulls that end the setup (DL-0091).
    const std::optional<Uuid> lastPull = trailingBarPull(setup);
    std::optional<sim::LatheStock> offBefore;
    for (std::size_t i = 0; i < ops.size(); ++i) {
        const Operation& op = *ops[i];
        cancel.throwIfCancelled();
        if (lastPull && op.id == *lastPull) {
            offBefore = current;
        }
        const double f0 = opShare * static_cast<double>(i) / static_cast<double>(ops.size());
        const double f1 = opShare * static_cast<double>(i + 1) / static_cast<double>(ops.size());
        ProgressRange genRange(progress, f0, f0 + 0.5 * (f1 - f0));
        OperationSimulation os;
        os.operationId = op.id;
        os.name = op.name;
        os.latheAfter = std::make_shared<const sim::LatheStock>(current);
        try {
            os.toolpath = toolpathOf(doc, setup, op, options, cancel, genRange);
            if (op.type == "spindle_transfer") {
                // The sub-spindle takes hold (ADR-0007): nothing is cut. Z2's moves are checked
                // in the sub-spindle setup's simulation, where they lead.
                finished(std::move(os));
                continue;
            }
            if (op.type == "bar_pull") {
                // The bar comes out of the chuck (DL-0091): it moves, and the puller cuts nothing.
                if (mills) {
                    throw Error(ErrorCode::Unsupported,
                                "a bar pull is not simulated in a setup that mills with a live "
                                "tool: the milled stock does not move with the bar");
                }
                current = current.pulled(sim::barPulledBy(os.toolpath, os.toolpath.moves.size()));
                os.latheAfter = std::make_shared<const sim::LatheStock>(current);
                finished(std::move(os));
                continue;
            }
            const tooling::Tool* tool = op.toolId ? project.tools.findTool(*op.toolId) : nullptr;
            const bool turning = tool && tooling::isTurningTool(tool->type);
            const bool around = tool && liveGoesAround(op, *tool);
            if (!tool || (!turning && !isHoleOperation(op.type) && !around)) {
                throw Error(ErrorCode::Unsupported,
                            "only turning operations, holes and live tools can be simulated on a "
                            "lathe");
            }
            if (op.type == "turn_soft_jaws") {
                // It bores the jaws before the work goes in (DL-0069): it cuts nothing of the work,
                // and its contact with the jaws is the cut.
                os.toolpath = latheMoves(os.toolpath, *tool);
                out.warnings.push_back(fmt::format(
                    "'{}' bores the soft jaws: it is not simulated against the work or the jaws",
                    op.name));
                finished(std::move(os));
                continue;
            }
            if (around) {
                const tooling::Holder* holder =
                    tool->holderId ? project.tools.findHolder(*tool->holderId) : nullptr;
                os.toolpath = toolpath::expandCycles(os.toolpath);
                os.tool =
                    std::make_shared<const sim::ToolShape>(sim::ToolShape::fromTool(*tool, holder));
                if (hasFixtures(setup)) {
                    os.fixtureCollisions = sim::checkFixtures(
                        os.toolpath, fixtureBodiesInWcs(doc, setup), *os.tool, {}, &cancel);
                }
                if (!milled) {
                    milled = dexelsNow();
                    out.milledFrom = out.operations.size();
                }
                const double before = milled->volume();
                os.collisions =
                    sim::simulateToolpath(*milled, *os.tool, os.toolpath, {}, &cancel).collisions;
                os.removedVolume = before - milled->volume();
                finished(std::move(os));
                continue;
            }
            // A live tool off the axis stands where the holes are, the part still.
            if (os.toolpath.live && sim::offTheAxis(os.toolpath)) {
                const tooling::Holder* holder =
                    tool->holderId ? project.tools.findHolder(*tool->holderId) : nullptr;
                os.toolpath = latheMoves(os.toolpath, *tool);
                os.tool =
                    std::make_shared<const sim::ToolShape>(sim::ToolShape::fromTool(*tool, holder));
                // A tap cuts its thread into a bore already there: it adds nothing to the stock.
                if (tool->type != tooling::ToolType::Tap) {
                    os.holeProfile = tooling::holeToolSide(*tool);
                }
                if (hasFixtures(setup)) {
                    os.fixtureCollisions = sim::checkFixtures(
                        os.toolpath, fixtureBodiesInWcs(doc, setup), *os.tool, {}, &cancel);
                }
                // Its rapids, shank and holder against the stock: a copy as dexels, threads and
                // holes included, as a tool that does not sweep the lathe plane reaches any of it.
                sim::Stock dexels =
                    milled ? *milled
                           : sim::latheStockDexels(current, out.resolution, options.triDexel);
                // A tap has to find a hole, however it came: generating looks only where it can
                // tell from the profile (DL-0072).
                std::vector<sim::Collision> noHole;
                if (tool->type == tooling::ToolType::Tap) {
                    noHole = sim::tapsWithoutAHole(dexels, os.toolpath,
                                                   tooling::tapCoreRadius(tool->mill), &cancel);
                }
                os.collisions =
                    sim::simulateToolpath(dexels, *os.tool, os.toolpath, {}, &cancel).collisions;
                os.collisions.insert(os.collisions.end(), noHole.begin(), noHole.end());
                if (milled && !os.holeProfile.empty()) {
                    milled = std::move(dexels);
                }
                sim::LatheStock next = current;
                sim::cutHoles(next, os.holeProfile, os.toolpath);
                os.removedVolume = current.volume() - next.volume();
                current = std::move(next);
                os.latheAfter = std::make_shared<const sim::LatheStock>(current);
                finished(std::move(os));
                continue;
            }
            os.toolpath =
                latheMoves(latheCutPath(doc, setup, op, os.toolpath, cancel, genRange), *tool);
            os.insert = latheCutter(*tool, os.toolpath);
            os.insertThickness = turning ? tool->insert.thickness : tool->mill.diameter;
            os.fixtureCollisions =
                sim::checkTurningFixtures(os.toolpath, os.insert, bodies, 200, &cancel);
            if (turning) {
                const tooling::Holder* holder =
                    tool->holderId ? project.tools.findHolder(*tool->holderId) : nullptr;
                const auto side = tooling::turningSideOf(os.toolpath.tipQuadrant);
                const bool away = tooling::turningAwayFromChuck(os.toolpath.tipQuadrant);
                os.toolParts = tooling::turningToolParts(*tool, holder, side, away);
                const auto body = tooling::turningBody(*tool, holder, side, away);
                if (!body.empty()) {
                    os.collisions =
                        sim::checkTurningBody(os.toolpath, os.insert, body, current, 200, &cancel);
                    for (const auto& piece : body) {
                        const auto hits = sim::checkTurningFixtures(os.toolpath, piece, bodies, 200,
                                                                    &cancel, true);
                        os.fixtureCollisions.insert(os.fixtureCollisions.end(), hits.begin(),
                                                    hits.end());
                    }
                }
            }
            sim::LatheStock next = current;
            sim::simulateTurning(next, os.insert, os.toolpath, 0,
                                 std::numeric_limits<std::size_t>::max(), &cancel, &os.collisions);
            os.removedVolume = current.volume() - next.volume();
            current = std::move(next);
            os.latheAfter = std::make_shared<const sim::LatheStock>(current);
            if (milled) {
                sim::clipToLathe(*milled, current);
            }
        } catch (const OperationCancelled&) {
            throw;
        } catch (const Error& e) {
            os.error = e.what();
        }
        finished(std::move(os));
    }
    // The next cycle: the bar as the pull leaves it, against where this setup's stock starts.
    if (offBefore && !current.heldBar().empty() && !out.latheInitial->heldBar().empty()) {
        const auto front = [](const geom2d::Regions& bar) {
            double z = -std::numeric_limits<double>::infinity();
            for (const geom2d::Region& r : bar) {
                z = std::max(z, r.bounds().max.x());
            }
            return z;
        };
        const double pulledTo = front(current.heldBar());
        const double startsAt = front(out.latheInitial->heldBar());
        if (std::abs(pulledTo - startsAt) > 1e-3) {
            out.warnings.push_back(fmt::format(
                "after the bar pull the bar's front is at Z {:.3f}, and this setup's stock starts "
                "at Z {:.3f}: the next part starts with the bar {:.3f} mm {} than this one did. "
                "Check the pull length against the setup's stock",
                pulledTo, startsAt, std::abs(pulledTo - startsAt),
                pulledTo > startsAt ? "further out" : "shorter"));
        }
    }
    const sim::LatheStock& comesOff = offBefore ? *offBefore : current;
    SimulationOptions partOptions;
    partOptions.compare = false; // in the lathe plane, below
    partOptions.tolerance = options.tolerance;
    partOptions.tessellationDeflection = options.tessellationDeflection;
    comparePart(doc, setup, partOptions, out, cancel, progress, opShare);
    if (!out.part.isEmpty()) {
        // A part with cross holes or flats: its section through the axis says nothing of the rest.
        out.partProfile = geometry.partIsRound ? sim::ProfileDistance::section(out.part)
                                               : sim::ProfileDistance(geometry.part);
        out.partRegions = geometry.part;
        if (comesOff.hasCuts() || !geometry.partIsRound) {
            out.partBvh = std::make_shared<const geom3d::TriangleBvh>(out.part);
        }
        if (options.compare && milled) {
            out.chamfered = chamferedMaterial(doc, setup, options, out);
            compareFinal(options, out);
        } else if (options.compare) {
            out.comparison =
                sim::compareProfile(comesOff.profile(), out.partRegions, options.tolerance);
            addCutsToComparison(out, comesOff, *out.comparison);
        }
    }
    if (const machine::Machine& onMachine = machineFor(project, setup); onMachine.lathe) {
        try {
            simulateLatheMachine(doc, setup, onMachine, out, cancel);
        } catch (const OperationCancelled&) {
            throw;
        } catch (const Error& e) {
            out.warnings.push_back(fmt::format("the machine is not simulated: {}", e.what()));
        }
    }
    return out;
}

} // namespace

std::vector<const Setup*> stockSources(const Project& project, const Setup& setup) {
    std::vector<const Setup*> sources;
    const Setup* current = &setup;
    while (current->stock.kind == StockKind::FromSetup && current->stock.sourceSetup) {
        // Only earlier setups are sources (as resolveStock), so the chain ends.
        const Setup* source = nullptr;
        for (const Setup& other : project.setups) {
            if (other.id == current->id) {
                break;
            }
            if (other.id == *current->stock.sourceSetup) {
                source = &other;
            }
        }
        if (!source) {
            break;
        }
        sources.push_back(source);
        current = source;
    }
    std::reverse(sources.begin(), sources.end());
    return sources;
}

void carryEarlierSetups(const Document& doc, const Setup& setup, sim::Stock& stock,
                        CancelToken& cancel, std::vector<std::string>& warnings,
                        const Transform3& frame) {
    const Project& project = doc.project();
    const std::vector<const Setup*> sources = stockSources(project, setup);
    if (sources.empty()) {
        return;
    }
    sim::SimulateOptions options;
    options.checkCollisions = false;
    // Arcs out of an arc plane after the move become lines well within a cell.
    const double chord = std::max(1e-3, 0.1 * stock.resolution());
    const Transform3 toFrame = frame.inverse() * setup.wcs.toParent().inverse();
    NullProgress quiet;
    for (const Setup* source : sources) {
        const Transform3 move = toFrame * source->wcs.toParent();
        for (const Operation& op : source->operations) {
            if (!op.enabled) {
                continue;
            }
            cancel.throwIfCancelled();
            try {
                const tooling::Tool* tool =
                    op.toolId ? project.tools.findTool(*op.toolId) : nullptr;
                if (!tool) {
                    throw Error(ErrorCode::InvalidArgument, "the operation has no tool");
                }
                const tooling::Holder* holder =
                    tool->holderId ? project.tools.findHolder(*tool->holderId) : nullptr;
                const auto tp = cachedToolpath(doc, *source, op, cancel, quiet);
                sim::simulateToolpath(stock, sim::ToolShape::fromTool(*tool, holder),
                                      toolpath::transformed(*tp, move, chord), options, &cancel);
            } catch (const OperationCancelled&) {
                throw;
            } catch (const Error& e) {
                warnings.push_back(fmt::format("the stock from setup '{}' leaves out '{}', which "
                                               "failed: {}",
                                               source->name, op.name, e.what()));
            }
        }
    }
}

namespace {

/// The text every key of a setup's simulation starts with: the program version, the setup
/// without its operations (its stock, WCS and placement) with the models and workholding it
/// uses, the simulation of the setup its stock comes from, the machine, where the work sits on
/// it and the options (all but lastOperation).
std::string keyBase(const Document& doc, const Setup& setup, const SimulationOptions& options) {
    const Project& p = doc.project();
    Project subset;
    subset.id = p.id;
    subset.tools.id = p.tools.id; // value types generate fresh ids by default: keep the key stable
    subset.models = p.models;
    subset.setups = {setup};
    subset.setups.front().operations.clear();
    if (setup.workholding) {
        if (const WorkholdingTemplate* w = p.findWorkholding(setup.workholding->templateId)) {
            subset.workholdings = {*w};
        }
        if (const auto& jaws = setup.workholding->jawSet) {
            if (const WorkholdingTemplate* j = p.findWorkholding(*jaws)) {
                subset.workholdings.push_back(*j);
            }
        }
    }
    subset.fittings = p.fittings;
    std::string text = fmt::format("opencam {} simulation|", kVersionString);
    text += dumpJson(toJson(subset));
    // Stock from an earlier setup: that setup's simulation (and so on up the chain).
    const std::vector<const Setup*> sources = stockSources(p, setup);
    if (!sources.empty()) {
        text += "|from " + simulationInputKey(doc, *sources.back(), sourceOptions(options));
    }
    text += '|';
    // Asked for what is shown as well: without a machine, simulating is what refuses.
    const machine::Machine* machine = findMachineFor(p, setup);
    if (!machine) {
        return text + "no machine " + setup.machine;
    }
    text += machine::toJson(*machine).dump();
    if (machine->lathe) {
        text += turretKey(doc, setup);
    }
    // Where the work sits on the machine (toolpath keys leave the placement out).
    machine::WorkMount mount;
    try {
        mount = workMount(withPlacedWorkholding(doc, setup), *machine);
    } catch (const Error& e) {
        text += e.what();
    }
    text += "|on " + mount.carrier;
    for (int k = 0; k < 12; ++k) {
        text += fmt::format(" {:.9g}", mount.wcs.matrix().data()[k]);
    }
    text += fmt::format("|{}|{}|{}|{}|{}", options.resolution, options.triDexel, options.compare,
                        options.tolerance, options.tessellationDeflection);
    return text;
}

std::string hashed(const std::string& text) {
    return Sha256::toHex(Sha256::hash(text));
}

} // namespace

std::string simulationInputKey(const Document& doc, const Setup& setup,
                               const SimulationOptions& options) {
    std::string text = keyBase(doc, setup, options);
    for (const Operation& op : setup.operations) {
        if (op.enabled) {
            text += '|';
            text += toolpathInputKey(doc, setup, op);
        }
        if (options.lastOperation && op.id == *options.lastOperation) {
            text += "|up to " + op.id.toString();
            break;
        }
    }
    return hashed(text);
}

std::vector<std::string> stockInputKeys(const Document& doc, const Setup& setup,
                                        const SimulationOptions& options) {
    std::string text = keyBase(doc, setup, options);
    std::vector<std::string> keys{hashed(text + "|initial")};
    for (const Operation& op : setup.operations) {
        if (op.enabled) {
            text += '|';
            text += toolpathInputKey(doc, setup, op);
            keys.push_back(hashed(text));
        }
        if (options.lastOperation && op.id == *options.lastOperation) {
            break;
        }
    }
    return keys;
}

SimulationOptions sourceOptions(const SimulationOptions& options) {
    SimulationOptions out = options;
    out.lastOperation.reset();
    out.resumeFrom.reset();
    return out;
}

SetupSimulation simulateSetup(const Document& doc, const Uuid& setupId,
                              const SimulationOptions& options, CancelToken& cancel,
                              ProgressSink& allProgress) {
    const Project& project = doc.project();
    const Setup* setup = project.findSetup(setupId);
    if (!setup) {
        throw Error(ErrorCode::InvalidArgument, "unknown setup");
    }
    if (machineFor(project, *setup).isLathe()) {
        return simulateLatheSetup(doc, *setup, options, cancel, allProgress);
    }
    // An earlier simulation of the setup with the same initial stock: it goes on from the last
    // of its leading operations whose stock is still the same.
    const std::vector<std::string> keys = stockInputKeys(doc, *setup, options);
    const SetupSimulation* resume = options.resumeFrom && options.resumeFrom->setupId == setupId &&
                                            options.resumeFrom->initialKey == keys.front()
                                        ? options.resumeFrom.get()
                                        : nullptr;
    // A stock from an earlier setup: that setup's simulation first, its share of the progress
    // by operation count along the chain.
    const std::vector<const Setup*> sources =
        resume ? std::vector<const Setup*>{} : stockSources(project, *setup);
    std::size_t sourceOps = 0;
    for (const Setup* s : sources) {
        sourceOps +=
            static_cast<std::size_t>(std::count_if(s->operations.begin(), s->operations.end(),
                                                   [](const Operation& op) { return op.enabled; }));
    }
    const std::size_t ownOps =
        static_cast<std::size_t>(std::count_if(setup->operations.begin(), setup->operations.end(),
                                               [](const Operation& op) { return op.enabled; }));
    const double sourceShare =
        sources.empty() ? 0.0
                        : static_cast<double>(sourceOps) /
                              static_cast<double>(std::max<std::size_t>(1, sourceOps + ownOps));
    std::shared_ptr<const SetupSimulation> source;
    if (!sources.empty()) {
        ProgressRange sourceProgress(allProgress, 0.0, sourceShare);
        source =
            options.sourceSimulation
                ? options.sourceSimulation(*sources.back(), cancel, sourceProgress)
                : std::make_shared<const SetupSimulation>(simulateSetup(
                      doc, sources.back()->id, sourceOptions(options), cancel, sourceProgress));
    }
    ProgressRange progress(allProgress, sourceShare, 1.0);
    const std::optional<geom3d::Mesh> stockMesh = stockMeshInWcs(doc, *setup);
    if (!stockMesh) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("the stock of setup '{}' cannot be resolved", setup->name));
    }

    SetupSimulation out;
    out.setupId = setupId;
    out.initialKey = keys.front();
    const geom3d::Aabb3 bounds = stockMesh->bounds();
    if (resume) {
        out.resolution = resume->resolution;
        out.initialStock = resume->initialStock;
        out.initialChamfered = resume->initialChamfered;
        out.stockWarnings = resume->stockWarnings;
    } else {
        out.resolution =
            options.resolution > 0.0 ? options.resolution : automaticResolution(bounds);
        out.initialStock =
            sim::Stock::fromMesh(*stockMesh, bounds, out.resolution, options.triDexel);
    }
    // Stock from an earlier setup starts as that setup left it: its final stock, moved into
    // this setup's WCS and sampled again on this grid.
    if (source) {
        cancel.throwIfCancelled();
        const Setup& from = *sources.back();
        const Transform3 fromToHere = setup->wcs.toParent().inverse() * from.wcs.toParent();
        out.initialStock =
            sim::resampleStock(source->lastStock(options.triDexel), fromToHere, out.initialStock);
        if (source->latheInitial) {
            const auto last = source->latheBefore(source->operations.size());
            if (last && last->hasCuts()) {
                out.stockWarnings.push_back(
                    fmt::format("the stock from setup '{}' has threads or holes off the axis, "
                                "which this setup's stock does not show",
                                from.name));
            }
        }
        out.initialChamfered = moveChamfered(*source, fromToHere, out.initialStock.grid(2));
        for (const OperationSimulation& op : source->operations) {
            if (!op.error.empty()) {
                out.stockWarnings.push_back(
                    fmt::format("the stock from setup '{}' leaves out '{}', which failed: {}",
                                from.name, op.name, op.error));
            }
        }
    }
    out.warnings = out.stockWarnings;
    sim::Stock current = out.initialStock;

    if (setup->modelId) {
        const ModelEntry* entry = project.findModel(*setup->modelId);
        const cad::Model* model = doc.model(*setup->modelId);
        if (entry && model) {
            try {
                out.part = cad::tessellate(*model, {options.tessellationDeflection, 0.35});
                out.part.transform(modelToWcs(*entry, *setup));
            } catch (const Error& e) {
                out.warnings.push_back(
                    fmt::format("the part could not be meshed for comparison: {}", e.what()));
                out.part = {};
            }
        }
    }
    const machine::Machine& machine = machineFor(project, *setup);
    out.machine = std::make_shared<const machine::Machine>(machine);
    // Where the work sits on the machine; a placement that cannot be used stops the machine.
    const machine::MachineModel model = machine::effectiveModel(machine);
    const std::string placementIssue = placementProblem(*setup, machine);
    if (!placementIssue.empty()) {
        out.warnings.push_back(placementIssue + "; the machine is not simulated");
    }
    // A placement on the workholding: its mount datum where the stock puts the workholding.
    Setup placed = *setup;
    try {
        placed = withPlacedWorkholding(doc, *setup);
    } catch (const Error& e) {
        out.warnings.push_back(std::string(e.what()) + "; it is simulated at its work point");
    }
    const machine::WorkMount mount = workMount(placed, machine);
    const Transform3 wcsInTable = machine::wcsInTable(model, mount);
    const std::vector<std::string> still =
        machine::rotariesNotCarrying(model, placementOf(*setup, machine).carrier);
    // The fixtures and the workholding, checked against the tool assembly and the machine.
    std::vector<sim::FixtureBody> fixtures;
    if (hasFixtures(*setup)) {
        try {
            fixtures = fixtureBodiesInWcs(doc, *setup);
        } catch (const Error& e) {
            out.warnings.push_back(
                fmt::format("the fixtures are not checked for collisions: {}", e.what()));
        }
    }

    std::vector<const Operation*> ops;
    for (const Operation& op : setup->operations) {
        if (op.enabled) {
            ops.push_back(&op);
        }
        if (options.lastOperation && op.id == *options.lastOperation) {
            break;
        }
    }
    // Table rotary axes that do not carry the work must stay at 0: the first one the operation
    // turns, or empty.
    auto turnedAxis = [&](const machine::RotaryTrack& rotary) {
        for (const std::string& axis : still) {
            auto value = [&](const machine::IndexAngles& q) {
                return axis == "A" ? q.a : axis == "B" ? q.b : q.c;
            };
            bool moves = std::abs(value(rotary.start)) > 1e-9;
            for (const auto& q : rotary.angles) {
                moves = moves || std::abs(value(q)) > 1e-9;
            }
            if (moves) {
                return axis;
            }
        }
        return std::string();
    };
    auto turnedWarning = [](const Operation& op, const std::string& axis) {
        return fmt::format("{}: it turns the {} axis, which does not carry the setup's work; the "
                           "machine is not simulated for it",
                           op.name, axis);
    };
    const double opShare = options.compare ? 0.9 : 1.0;
    for (std::size_t i = 0; i < ops.size(); ++i) {
        const Operation& op = *ops[i];
        cancel.throwIfCancelled();
        const double f0 = opShare * static_cast<double>(i) / static_cast<double>(ops.size());
        const double f1 = opShare * static_cast<double>(i + 1) / static_cast<double>(ops.size());
        // Unchanged since the earlier simulation (and so is everything before it): taken as it is.
        if (resume && i == out.operations.size() && i < resume->operations.size() &&
            resume->operations[i].inputKey == keys[i + 1]) {
            OperationSimulation os = resume->operations[i];
            os.name = op.name;
            if (const std::string axis = turnedAxis(os.rotary); !axis.empty()) {
                out.warnings.push_back(turnedWarning(op, axis));
            }
            current = os.stockAfter;
            out.operations.push_back(std::move(os));
            progress.report(f1, "Simulating");
            continue;
        }
        ProgressRange genRange(progress, f0, f0 + 0.3 * (f1 - f0));
        ProgressRange simRange(progress, f0 + 0.3 * (f1 - f0), f1);

        OperationSimulation os;
        os.operationId = op.id;
        os.name = op.name;
        os.inputKey = keys[i + 1];
        os.stockAfter = current;
        try {
            os.toolpath =
                toolpath::expandCycles(toolpathOf(doc, *setup, op, options, cancel, genRange));
            const tooling::Tool* tool = op.toolId ? project.tools.findTool(*op.toolId) : nullptr;
            if (!tool) {
                throw Error(ErrorCode::InvalidArgument, "the operation has no tool");
            }
            const tooling::Holder* holder =
                tool->holderId ? project.tools.findHolder(*tool->holderId) : nullptr;
            os.tool =
                std::make_shared<const sim::ToolShape>(sim::ToolShape::fromTool(*tool, holder));
            sim::Stock stock = current;
            sim::SimulateOptions simOptions;
            if (tool->type == tooling::ToolType::Probe) {
                // A touch runs the stylus into the surface by its trigger travel (DL-0090).
                simOptions.probeAllowance += tool->mill.triggerOffset;
            }
            sim::SimulateResult result =
                sim::simulateToolpath(stock, *os.tool, os.toolpath, simOptions, &cancel, &simRange);
            os.collisions = std::move(result.collisions);
            os.removedVolume = current.volume() - stock.volume();
            os.stockAfter = stock;
            current = std::move(stock);
            sim::MachineCheckOptions checkOptions;
            if (positional(op, os.toolpath)) {
                // Positional (3+2): the rotary axes stay at the operation's index.
                checkOptions.index = operationIndex(placed, op, machine).angles;
            }
            os.rotary = machine::rotaryTrack(machine, os.toolpath, wcsInTable,
                                             checkOptions.index ? &*checkOptions.index : nullptr);
            if (const std::string turned = turnedAxis(os.rotary); !turned.empty()) {
                out.warnings.push_back(turnedWarning(op, turned));
            } else if (placementIssue.empty()) {
                os.mount = machine::WorkMount{
                    mount.carrier,
                    Eigen::Translation3d(machine::tableCentre(model, mount.wcs.translation())) *
                        machine::mountedWcs(machine, os.toolpath, wcsInTable)};
                os.machineCollisions =
                    sim::checkMachine(machine, os.toolpath, mount, out.part, bounds, fixtures,
                                      *os.tool, checkOptions, &cancel);
            }
            if (!fixtures.empty()) {
                os.fixtureCollisions =
                    sim::checkFixtures(os.toolpath, fixtures, *os.tool, {}, &cancel);
            }
        } catch (const OperationCancelled&) {
            throw;
        } catch (const Error& e) {
            os.error = e.what();
        }
        out.operations.push_back(std::move(os));
    }

    out.chamfered = chamferedMaterial(doc, *setup, options, out);
    if (options.compare && !out.part.isEmpty()) {
        cancel.throwIfCancelled();
        progress.report(opShare, "Comparing with the part");
        compareFinal(options, out);
    }
    progress.report(1.0, "Simulated");
    return out;
}

StockBefore stockBeforeOperation(const Document& doc, const Setup& setup, const Uuid& operationId,
                                 double resolution, CancelToken& cancel, ProgressSink& progress,
                                 const Transform3& frame) {
    std::optional<geom3d::Mesh> stockMesh = stockMeshInWcs(doc, setup);
    if (!stockMesh) {
        throw Error(ErrorCode::InvalidArgument, "the setup's stock cannot be resolved");
    }
    const Transform3 toFrame = frame.inverse();
    const bool inWcs = frame.isApprox(Transform3::Identity(), 1e-12);
    if (!inWcs) {
        stockMesh->transform(toFrame);
    }
    const geom3d::Aabb3 bounds = stockMesh->bounds();
    const double h = resolution > 0.0 ? resolution : automaticResolution(bounds);
    StockBefore out{sim::Stock::fromMesh(*stockMesh, bounds, h, false), {}};
    // Stock from an earlier setup: what that setup cut first.
    carryEarlierSetups(doc, setup, out.stock, cancel, out.warnings, frame);
    std::vector<const Operation*> earlier;
    for (const Operation& op : setup.operations) {
        if (op.id == operationId) {
            break;
        }
        if (op.enabled) {
            earlier.push_back(&op);
        }
    }
    sim::SimulateOptions options;
    options.checkCollisions = false;
    const Project& project = doc.project();
    for (std::size_t i = 0; i < earlier.size(); ++i) {
        const Operation& op = *earlier[i];
        ProgressRange range(progress, static_cast<double>(i) / static_cast<double>(earlier.size()),
                            static_cast<double>(i + 1) / static_cast<double>(earlier.size()));
        try {
            auto tp = cachedToolpath(doc, setup, op, cancel, range);
            if (!inWcs) {
                tp =
                    std::make_shared<const toolpath::Toolpath>(toolpath::transformed(*tp, toFrame));
            }
            const tooling::Tool* tool = project.tools.findTool(*op.toolId);
            const tooling::Holder* holder =
                tool->holderId ? project.tools.findHolder(*tool->holderId) : nullptr;
            sim::simulateToolpath(out.stock, sim::ToolShape::fromTool(*tool, holder), *tp, options,
                                  &cancel);
        } catch (const OperationCancelled&) {
            throw;
        } catch (const Error& e) {
            out.warnings.push_back(
                fmt::format("rest stock ignores '{}', which failed: {}", op.name, e.what()));
        }
    }
    return out;
}

sim::Stock simulatePartial(const SetupSimulation& simulation, std::size_t operationIndex,
                           std::size_t moveIndex, CancelToken* cancel) {
    sim::Stock stock = simulation.stockBefore(operationIndex);
    if (operationIndex >= simulation.operations.size()) {
        return simulation.finalStock();
    }
    const OperationSimulation& os = simulation.operations[operationIndex];
    if (!os.tool || os.toolpath.moves.empty()) {
        return os.stockAfter;
    }
    sim::SimulateOptions options;
    options.checkCollisions = false;
    sim::simulateToolpath(stock, *os.tool, os.toolpath, options, cancel, nullptr, moveIndex);
    return stock;
}

std::optional<sim::LatheStock> simulatePartialLathe(const SetupSimulation& simulation,
                                                    std::size_t operationIndex,
                                                    std::size_t moveIndex, CancelToken* cancel) {
    if (!simulation.latheInitial) {
        return std::nullopt;
    }
    if (operationIndex >= simulation.operations.size()) {
        const auto last = simulation.latheBefore(simulation.operations.size());
        return last ? *last : *simulation.latheInitial;
    }
    const OperationSimulation& os = simulation.operations[operationIndex];
    if (os.toolpath.strategy == "bar_pull" && os.latheAfter &&
        moveIndex < os.toolpath.moves.size()) {
        // The bar as far as it has come (DL-0091).
        const auto before = simulation.latheBefore(operationIndex);
        const sim::LatheStock& start = before ? *before : *simulation.latheInitial;
        return start.pulled(sim::barPulledBy(os.toolpath, moveIndex));
    }
    if (os.latheAfter && (moveIndex >= os.toolpath.moves.size() ||
                          (os.insert.isEmpty() && os.holeProfile.empty()))) {
        return *os.latheAfter;
    }
    const auto before = simulation.latheBefore(operationIndex);
    sim::LatheStock lathe = before ? *before : *simulation.latheInitial;
    if (!os.holeProfile.empty()) {
        sim::cutHoles(lathe, os.holeProfile, os.toolpath, 0, moveIndex);
    } else if (!os.insert.isEmpty()) {
        sim::simulateTurning(lathe, os.insert, os.toolpath, 0, moveIndex, cancel);
    }
    return lathe;
}

std::optional<sim::Comparison> latheComparison(const SetupSimulation& simulation,
                                               const sim::LatheStock& stock) {
    if (!simulation.latheInitial || simulation.partRegions.empty()) {
        return std::nullopt;
    }
    sim::Comparison c =
        sim::compareProfile(stock.profile(), simulation.partRegions,
                            simulation.comparison ? simulation.comparison->tolerance : 0.02);
    addCutsToComparison(simulation, stock, c);
    return c;
}

sim::DisplayMesh latheDisplayMesh(const SetupSimulation& simulation, std::size_t operationIndex,
                                  const sim::LatheStock& stock, bool compare,
                                  const sim::Rgb& stockColour) {
    if (!simulation.latheInitial) {
        return {};
    }
    // The profiles each operation started from, up to the one shown.
    std::vector<sim::ProfileDistance> starts;
    for (std::size_t j = 0; j <= operationIndex && j < simulation.operations.size(); ++j) {
        const auto before = simulation.latheBefore(j);
        starts.emplace_back((before ? *before : *simulation.latheInitial).profile());
    }
    const sim::ProfileDistance* part =
        compare && !simulation.partProfile.isEmpty() ? &simulation.partProfile : nullptr;
    const double tol = simulation.comparison ? simulation.comparison->tolerance : 0.02;
    auto colour = [&](const Vec2& p, const Vec2&) -> sim::Rgb {
        if (part) {
            const double d = (*part)(p);
            if (d < -tol) {
                return sim::comparisonColour(0.0, -d, tol, 2.0, stockColour);
            }
            if (d <= tol) {
                return sim::comparisonColour(0.0, 0.0, tol, 2.0, stockColour);
            }
        }
        for (std::size_t j = starts.size(); j-- > 0;) {
            if (std::abs(starts[j](p)) > 1e-3) {
                return sim::operationColour(static_cast<int>(j));
            }
        }
        return stockColour;
    };
    // A cut is in the colour of the operation that made it: the last whose stock lacks it. A hole
    // through material the part has is a gouge.
    const std::vector<HoleInPart> inPart =
        part ? holesInPart(simulation, stock, tol) : std::vector<HoleInPart>{};
    auto cutColour = [&](bool hole, int index) -> sim::Rgb {
        if (hole && static_cast<std::size_t>(index) < inPart.size() &&
            inPart[static_cast<std::size_t>(index)].deepest > 0.0) {
            return sim::comparisonColour(0.0, inPart[static_cast<std::size_t>(index)].deepest, tol,
                                         2.0, stockColour);
        }
        for (std::size_t j = starts.size(); j-- > 0;) {
            const auto before = simulation.latheBefore(j);
            const sim::LatheStock& start = before ? *before : *simulation.latheInitial;
            const auto had = static_cast<int>(hole ? start.holes().size() : start.threads().size());
            if (index >= had) {
                return sim::operationColour(static_cast<int>(j));
            }
        }
        return stockColour; // came with the stock
    };
    // A chord tolerance far below the ray spacing: the profile is exact, the circles nearly so.
    return sim::latheStockMesh(stock, std::min(0.005, 0.1 * simulation.resolution), colour,
                               cutColour);
}

} // namespace ocam::document
