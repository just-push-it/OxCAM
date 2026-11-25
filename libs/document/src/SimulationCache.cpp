// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/document/SimulationCache.hpp"

#include "ocam/document/LatheMachine.hpp"
#include "ocam/document/PostSetup.hpp"

#include <ocam/foundation/Bytes.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/sim/StockCodec.hpp>

#include <miniz.h>

namespace ocam::document {
namespace {

constexpr std::uint32_t kMagic = 0x4d495358; // "XSIM"

void writePolyline(ByteWriter& w, const geom2d::Polyline& p) {
    w.boolean(p.isClosed());
    w.u64(p.vertices().size());
    for (const geom2d::Vertex& v : p.vertices()) {
        w.vec2(v.p);
        w.f64(v.bulge);
    }
}

geom2d::Polyline readPolyline(ByteReader& r) {
    const bool closed = r.boolean();
    std::vector<geom2d::Vertex> vertices(r.count(24));
    for (geom2d::Vertex& v : vertices) {
        v.p = r.vec2();
        v.bulge = r.f64();
    }
    return geom2d::Polyline(std::move(vertices), closed);
}

void writeRegions(ByteWriter& w, const geom2d::Regions& regions) {
    w.u64(regions.size());
    for (const geom2d::Region& region : regions) {
        writePolyline(w, region.outer);
        w.u64(region.holes.size());
        for (const geom2d::Polyline& hole : region.holes) {
            writePolyline(w, hole);
        }
    }
}

void writeAxes(ByteWriter& w, const machine::AxisPosition& a) {
    w.vec3(a.linear);
    w.pod(a.rotary);
    w.f64(a.t);
    w.f64(a.w);
    w.f64(a.z2);
    w.f64(a.c2);
}

machine::AxisPosition readAxes(ByteReader& r) {
    machine::AxisPosition a;
    a.linear = r.vec3();
    a.rotary = r.pod<machine::IndexAngles>();
    a.t = r.f64();
    a.w = r.f64();
    a.z2 = r.f64();
    a.c2 = r.f64();
    return a;
}

void writeLatheMotion(ByteWriter& w, const machine::LatheMotion& m) {
    w.u32(static_cast<std::uint32_t>(m.station));
    w.vec3(m.tip);
    writeAxes(w, m.held);
    writeAxes(w, m.start);
    w.u64(m.steps.size());
    for (const machine::LatheStep& s : m.steps) {
        w.u8(static_cast<std::uint8_t>(s.kind));
        writeAxes(w, s.to);
        w.f64(s.duration);
        w.boolean(s.workStays);
    }
}

machine::LatheMotion readLatheMotion(ByteReader& r) {
    machine::LatheMotion m;
    m.station = static_cast<int>(r.u32());
    m.tip = r.vec3();
    m.held = readAxes(r);
    m.start = readAxes(r);
    m.steps.resize(r.count(1));
    for (machine::LatheStep& s : m.steps) {
        s.kind = static_cast<machine::LatheStep::Kind>(r.u8());
        s.to = readAxes(r);
        s.duration = r.f64();
        s.workStays = r.boolean();
    }
    return m;
}

void writeLathe(ByteWriter& w, const sim::LatheStock& stock) {
    writeRegions(w, stock.profile());
    w.u64(stock.threads().size());
    for (const sim::LatheThreadCut& c : stock.threads()) {
        w.vec2(c.from);
        w.vec2(c.to);
        w.f64(c.lead);
        w.f64(c.turn);
        w.f64(c.done);
        writePolyline(w, c.insert);
    }
    w.u64(stock.holes().size());
    for (const sim::LatheHoleCut& h : stock.holes()) {
        w.vec3(h.tip);
        w.vec3(h.axis);
        w.f64(h.length);
        w.u64(h.profile.size());
        for (const Vec2& p : h.profile) {
            w.vec2(p);
        }
    }
}

geom2d::Regions readRegions(ByteReader& r);

std::shared_ptr<const sim::LatheStock> readLathe(ByteReader& r) {
    sim::LatheStock stock(readRegions(r));
    sim::LatheStock cuts;
    const std::size_t threads = r.count(1);
    for (std::size_t k = 0; k < threads; ++k) {
        sim::LatheThreadCut c;
        c.from = r.vec2();
        c.to = r.vec2();
        c.lead = r.f64();
        c.turn = r.f64();
        c.done = r.f64();
        c.insert = readPolyline(r);
        cuts.keepThread(std::move(c));
    }
    const std::size_t holes = r.count(1);
    for (std::size_t k = 0; k < holes; ++k) {
        sim::LatheHoleCut h;
        h.tip = r.vec3();
        h.axis = r.vec3();
        h.length = r.f64();
        h.profile.resize(r.count(16));
        for (Vec2& p : h.profile) {
            p = r.vec2();
        }
        cuts.cutHole(h);
    }
    stock.takeCuts(cuts, Transform3::Identity());
    return std::make_shared<const sim::LatheStock>(std::move(stock));
}

geom2d::Regions readRegions(ByteReader& r) {
    geom2d::Regions regions(r.count(1));
    for (geom2d::Region& region : regions) {
        region.outer = readPolyline(r);
        region.holes.resize(r.count(1));
        for (geom2d::Polyline& hole : region.holes) {
            hole = readPolyline(r);
        }
    }
    return regions;
}

void writeLists(ByteWriter& w, const std::vector<sim::IntervalList>& lists) {
    w.u64(lists.size());
    for (const sim::IntervalList& list : lists) {
        w.array(list);
    }
}

std::vector<sim::IntervalList> readLists(ByteReader& r) {
    std::vector<sim::IntervalList> lists(r.count(8));
    for (sim::IntervalList& list : lists) {
        list = r.array<sim::Interval>();
    }
    return lists;
}

void writeStrings(ByteWriter& w, const std::vector<std::string>& strings) {
    w.u64(strings.size());
    for (const std::string& s : strings) {
        w.str(s);
    }
}

std::vector<std::string> readStrings(ByteReader& r) {
    std::vector<std::string> strings(r.count(8));
    for (std::string& s : strings) {
        s = r.str();
    }
    return strings;
}

std::string body(const SetupSimulation& s) {
    ByteWriter w;
    w.str(s.setupId.toString());
    w.f64(s.resolution);
    w.str(s.initialKey);
    writeStrings(w, s.stockWarnings);
    writeLists(w, s.initialChamfered);
    writeLists(w, s.chamfered);
    std::vector<const sim::Stock*> stocks{&s.initialStock};
    for (const OperationSimulation& op : s.operations) {
        stocks.push_back(&op.stockAfter);
    }
    sim::writeStocks(w, stocks);

    w.boolean(s.machineChuck);
    w.u64(s.turretTools.size());
    for (const sim::TurretTool& tool : s.turretTools) {
        w.u32(static_cast<std::uint32_t>(tool.station));
        w.str(tool.name);
        w.transform(tool.placement);
        w.u64(tool.solids.size());
        for (const sim::TurretTool::Solid& solid : tool.solids) {
            w.str(solid.part);
            w.u64(solid.prism.lower.size());
            for (const Vec3& p : solid.prism.lower) {
                w.vec3(p);
            }
            for (const Vec3& p : solid.prism.upper) {
                w.vec3(p);
            }
        }
    }
    w.u64(s.operations.size());
    for (const OperationSimulation& op : s.operations) {
        w.str(op.operationId.toString());
        w.str(op.name);
        w.str(op.inputKey);
        w.str(toolpath::toJson(op.toolpath).dump());
        w.boolean(op.tool != nullptr);
        w.f64(op.removedVolume);
        w.u64(op.collisions.size());
        for (const sim::Collision& c : op.collisions) {
            w.u8(static_cast<std::uint8_t>(c.kind));
            w.u64(c.firstMove);
            w.u64(c.lastMove);
            w.vec3(c.position);
            w.f64(c.depth);
        }
        w.u64(op.machineCollisions.size());
        for (const sim::MachineCollision& c : op.machineCollisions) {
            w.u64(c.firstMove);
            w.u64(c.lastMove);
            w.str(c.first);
            w.str(c.second);
            w.vec3(c.position);
            w.f64(c.distance);
            w.str(c.during);
        }
        w.u64(op.fixtureCollisions.size());
        for (const sim::FixtureCollision& c : op.fixtureCollisions) {
            w.u64(c.firstMove);
            w.u64(c.lastMove);
            w.str(c.body);
            w.str(c.toolPart);
            w.vec3(c.position);
            w.f64(c.distance);
        }
        w.pod(op.rotary.start);
        w.array(op.rotary.angles);
        writeStrings(w, op.rotary.errors);
        w.boolean(op.mount.has_value());
        if (op.mount) {
            w.str(op.mount->carrier);
            w.transform(op.mount->wcs);
        }
        w.boolean(op.latheMotion.has_value());
        if (op.latheMotion) {
            writeLatheMotion(w, *op.latheMotion);
        }
        w.str(op.error);
        w.boolean(op.latheAfter != nullptr);
        if (op.latheAfter) {
            writeLathe(w, *op.latheAfter);
        }
        writePolyline(w, op.insert);
        w.f64(op.insertThickness);
        w.u64(op.holeProfile.size());
        for (const Vec2& p : op.holeProfile) {
            w.vec2(p);
        }
        w.u64(op.toolParts.size());
        for (const tooling::TurningToolPart& part : op.toolParts) {
            writePolyline(w, part.outline);
            w.f64(part.low);
            w.f64(part.high);
            w.boolean(part.round);
        }
    }

    w.u64(s.part.vertices.size());
    for (const Vec3& v : s.part.vertices) {
        w.vec3(v);
    }
    w.array(s.part.triangles);

    w.boolean(s.comparison.has_value());
    if (s.comparison) {
        const sim::Comparison& c = *s.comparison;
        w.u32(static_cast<std::uint32_t>(c.nu));
        w.u32(static_cast<std::uint32_t>(c.nv));
        w.f64(c.tolerance);
        w.array(c.excess);
        w.array(c.gouge);
        w.array(c.gougeLo);
        w.array(c.gougeHi);
        w.f64(c.excessVolume);
        w.f64(c.gougeVolume);
        w.f64(c.maxExcess);
        w.f64(c.maxGouge);
        w.u64(c.gougeRays);
        w.u64(c.excessRays);
    }
    w.boolean(s.partReference != nullptr);
    if (s.partReference) {
        const sim::PartReference& p = *s.partReference;
        w.f64(p.u0);
        w.f64(p.v0);
        w.f64(p.spacing);
        w.u32(static_cast<std::uint32_t>(p.nu));
        w.u32(static_cast<std::uint32_t>(p.nv));
        w.f64(p.tolerance);
        writeLists(w, p.part);
        writeLists(w, p.eroded);
        writeLists(w, p.dilated);
    }
    writeStrings(w, s.warnings);
    w.boolean(s.latheInitial != nullptr);
    if (s.latheInitial) {
        writeLathe(w, *s.latheInitial);
        writeRegions(w, s.partRegions);
        w.boolean(s.milledFrom.has_value());
        w.u64(s.milledFrom.value_or(0));
    }
    return w.take();
}

SetupSimulation readBody(ByteReader& r, const Document& doc, const Setup& setup) {
    const Project& project = doc.project();
    SetupSimulation s;
    const auto setupId = Uuid::parse(r.str());
    if (!setupId || *setupId != setup.id) {
        throw Error(ErrorCode::Parse, "a simulation of another setup");
    }
    s.setupId = *setupId;
    s.resolution = r.f64();
    s.initialKey = r.str();
    s.stockWarnings = readStrings(r);
    s.initialChamfered = readLists(r);
    s.chamfered = readLists(r);
    std::vector<sim::Stock> stocks = sim::readStocks(r);
    if (stocks.empty()) {
        throw Error(ErrorCode::Parse, "a simulation without stock");
    }
    s.initialStock = std::move(stocks.front());
    s.machine = std::make_shared<const machine::Machine>(machineFor(project, setup));
    // Meshes of what the setup fits: from the document, like the machine.
    s.turretBlocks = turretBlocks(doc, setup, *s.machine);

    s.machineChuck = r.boolean();
    s.turretTools.resize(r.count(1));
    for (sim::TurretTool& tool : s.turretTools) {
        tool.station = static_cast<int>(r.u32());
        tool.name = r.str();
        tool.placement = r.transform();
        tool.solids.resize(r.count(1));
        for (sim::TurretTool::Solid& solid : tool.solids) {
            solid.part = r.str();
            const std::size_t points = r.count(24);
            solid.prism.lower.resize(points);
            solid.prism.upper.resize(points);
            for (Vec3& p : solid.prism.lower) {
                p = r.vec3();
            }
            for (Vec3& p : solid.prism.upper) {
                p = r.vec3();
            }
        }
    }
    const std::size_t count = r.count(1);
    if (count + 1 != stocks.size()) {
        throw Error(ErrorCode::Parse, "operations and stocks do not match");
    }
    s.operations.resize(count);
    for (std::size_t k = 0; k < count; ++k) {
        OperationSimulation& op = s.operations[k];
        const auto id = Uuid::parse(r.str());
        if (!id) {
            throw Error(ErrorCode::Parse, "damaged operation id");
        }
        op.operationId = *id;
        op.name = r.str();
        op.inputKey = r.str();
        op.toolpath = toolpath::toolpathFromJson(parseJson(r.str(), "cached toolpath"));
        if (r.boolean()) {
            // The tool's shape, from the project as the simulation built it.
            const Operation* source = project.findOperation(op.operationId);
            const tooling::Tool* tool =
                source && source->toolId ? project.tools.findTool(*source->toolId) : nullptr;
            if (!tool) {
                throw Error(ErrorCode::Parse, "the cached operation's tool is gone");
            }
            const tooling::Holder* holder =
                tool->holderId ? project.tools.findHolder(*tool->holderId) : nullptr;
            op.tool =
                std::make_shared<const sim::ToolShape>(sim::ToolShape::fromTool(*tool, holder));
        }
        op.stockAfter = std::move(stocks[k + 1]);
        op.removedVolume = r.f64();
        op.collisions.resize(r.count(1));
        for (sim::Collision& c : op.collisions) {
            c.kind = static_cast<sim::CollisionKind>(r.u8());
            c.firstMove = r.u64();
            c.lastMove = r.u64();
            c.position = r.vec3();
            c.depth = r.f64();
        }
        op.machineCollisions.resize(r.count(1));
        for (sim::MachineCollision& c : op.machineCollisions) {
            c.firstMove = r.u64();
            c.lastMove = r.u64();
            c.first = r.str();
            c.second = r.str();
            c.position = r.vec3();
            c.distance = r.f64();
            c.during = r.str();
        }
        op.fixtureCollisions.resize(r.count(1));
        for (sim::FixtureCollision& c : op.fixtureCollisions) {
            c.firstMove = r.u64();
            c.lastMove = r.u64();
            c.body = r.str();
            c.toolPart = r.str();
            c.position = r.vec3();
            c.distance = r.f64();
        }
        op.rotary.start = r.pod<machine::IndexAngles>();
        op.rotary.angles = r.array<machine::IndexAngles>();
        op.rotary.errors = readStrings(r);
        if (r.boolean()) {
            machine::WorkMount mount;
            mount.carrier = r.str();
            mount.wcs = r.transform();
            op.mount = mount;
        }
        if (r.boolean()) {
            op.latheMotion = readLatheMotion(r);
        }
        op.error = r.str();
        if (r.boolean()) {
            op.latheAfter = readLathe(r);
        }
        op.insert = readPolyline(r);
        op.insertThickness = r.f64();
        op.holeProfile.resize(r.count(16));
        for (Vec2& p : op.holeProfile) {
            p = r.vec2();
        }
        op.toolParts.resize(r.count(25));
        for (tooling::TurningToolPart& part : op.toolParts) {
            part.outline = readPolyline(r);
            part.low = r.f64();
            part.high = r.f64();
            part.round = r.boolean();
        }
    }

    s.part.vertices.resize(r.count(24));
    for (Vec3& v : s.part.vertices) {
        v = r.vec3();
    }
    s.part.triangles = r.array<geom3d::Mesh::Triangle>();

    if (r.boolean()) {
        sim::Comparison c;
        c.nu = static_cast<int>(r.u32());
        c.nv = static_cast<int>(r.u32());
        c.tolerance = r.f64();
        c.excess = r.array<float>();
        c.gouge = r.array<float>();
        c.gougeLo = r.array<float>();
        c.gougeHi = r.array<float>();
        c.excessVolume = r.f64();
        c.gougeVolume = r.f64();
        c.maxExcess = r.f64();
        c.maxGouge = r.f64();
        c.gougeRays = r.u64();
        c.excessRays = r.u64();
        s.comparison = std::move(c);
    }
    if (r.boolean()) {
        auto p = std::make_shared<sim::PartReference>();
        p->u0 = r.f64();
        p->v0 = r.f64();
        p->spacing = r.f64();
        p->nu = static_cast<int>(r.u32());
        p->nv = static_cast<int>(r.u32());
        p->tolerance = r.f64();
        p->part = readLists(r);
        p->eroded = readLists(r);
        p->dilated = readLists(r);
        s.partReference = std::move(p);
    }
    s.warnings = readStrings(r);
    if (r.boolean()) {
        s.latheInitial = readLathe(r);
        s.partRegions = readRegions(r);
        const bool milled = r.boolean();
        const std::uint64_t milledFrom = r.u64();
        if (milled) {
            s.milledFrom = static_cast<std::size_t>(milledFrom);
        }
        const bool holes = std::any_of(s.operations.begin(), s.operations.end(),
                                       [](const OperationSimulation& op) {
                                           return op.latheAfter && op.latheAfter->hasCuts();
                                       });
        if (holes && !s.part.isEmpty()) {
            s.partBvh = std::make_shared<const geom3d::TriangleBvh>(s.part);
        }
        if (!s.part.isEmpty()) {
            s.partProfile = sim::ProfileDistance::section(s.part);
        }
    }
    if (!r.atEnd()) {
        throw Error(ErrorCode::Parse, "trailing data in a cached simulation");
    }
    return s;
}

} // namespace

std::string encodeSimulation(const SetupSimulation& simulation, const std::string& key) {
    const std::string raw = body(simulation);
    mz_ulong packedSize = mz_compressBound(static_cast<mz_ulong>(raw.size()));
    std::string packed(packedSize, '\0');
    // Fast compression: the cache is written after every simulation.
    if (mz_compress2(reinterpret_cast<unsigned char*>(packed.data()), &packedSize,
                     reinterpret_cast<const unsigned char*>(raw.data()),
                     static_cast<mz_ulong>(raw.size()), 1) != MZ_OK) {
        throw Error(ErrorCode::Internal, "cannot compress the simulation");
    }
    packed.resize(packedSize);
    ByteWriter w;
    w.u32(kMagic);
    w.u32(kSimulationCacheVersion);
    w.str(key);
    w.u64(raw.size());
    w.str(packed);
    return w.take();
}

std::optional<SetupSimulation> decodeSimulation(std::string_view bytes, const std::string& key,
                                                const Document& doc, const Setup& setup) {
    try {
        ByteReader head(bytes);
        if (head.u32() != kMagic || head.u32() != kSimulationCacheVersion || head.str() != key) {
            return std::nullopt;
        }
        const std::uint64_t rawSize = head.u64();
        const std::string packed = head.str();
        std::string raw(static_cast<std::size_t>(rawSize), '\0');
        mz_ulong size = static_cast<mz_ulong>(rawSize);
        if (mz_uncompress(reinterpret_cast<unsigned char*>(raw.data()), &size,
                          reinterpret_cast<const unsigned char*>(packed.data()),
                          static_cast<mz_ulong>(packed.size())) != MZ_OK ||
            size != rawSize) {
            return std::nullopt;
        }
        ByteReader r(raw);
        return readBody(r, doc, setup);
    } catch (const Error&) {
        return std::nullopt; // damaged: simulated again
    } catch (const Json::exception&) {
        return std::nullopt;
    }
}

} // namespace ocam::document
