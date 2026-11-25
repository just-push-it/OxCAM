// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/document/Generate.hpp"

#include "BreakEdges.hpp"
#include "ocam/document/LatheFitting.hpp"
#include "ocam/document/LatheMachine.hpp"
#include "ocam/document/PostSetup.hpp"
#include "ocam/document/Probing.hpp"
#include "ocam/document/ProjectIo.hpp"
#include "ocam/document/Simulate.hpp"
#include "ocam/document/ToolpathCache.hpp"
#include "ocam/document/Workholding.hpp"

#include <ocam/cad/Chain.hpp>
#include <ocam/cad/Features.hpp>
#include <ocam/cad/Revolve.hpp>
#include <ocam/cad/Ruled.hpp>
#include <ocam/cad/Section.hpp>
#include <ocam/cad/Tessellation.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Sha256.hpp>
#include <ocam/foundation/Version.hpp>
#include <ocam/geom2d/Clip.hpp>
#include <ocam/sim/Compare.hpp>
#include <ocam/sim/Rest.hpp>
#include <ocam/strategies/Avoidance.hpp>
#include <ocam/strategies/BackBore.hpp>
#include <ocam/strategies/MultiAxis.hpp>
#include <ocam/strategies/Probing.hpp>
#include <ocam/strategies/Strategies.hpp>
#include <ocam/strategies/Surface.hpp>
#include <ocam/strategies/Turning.hpp>
#include <ocam/tooling/TurningHolder.hpp>
#include <ocam/toolpath/Smoothing.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <set>
#include <tuple>

namespace ocam::document {
namespace {

using strategies::Context;

Error userError(const std::string& message) {
    return Error(ErrorCode::InvalidArgument, message);
}

// Convex hull (monotone chain) of points, counter-clockwise.
std::vector<Vec2> convexHull(std::vector<Vec2> pts) {
    std::sort(pts.begin(), pts.end(), [](const Vec2& a, const Vec2& b) {
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    pts.erase(std::unique(pts.begin(), pts.end(),
                          [](const Vec2& a, const Vec2& b) { return (a - b).norm() < 1e-12; }),
              pts.end());
    if (pts.size() < 3) {
        return pts;
    }
    auto cross = [](const Vec2& o, const Vec2& a, const Vec2& b) {
        return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    };
    std::vector<Vec2> hull(2 * pts.size());
    std::size_t k = 0;
    for (const Vec2& p : pts) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], p) <= 0) {
            --k;
        }
        hull[k++] = p;
    }
    for (std::size_t i = pts.size() - 1, lower = k + 1; i-- > 0;) {
        while (k >= lower && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0) {
            --k;
        }
        hull[k++] = pts[i];
    }
    hull.resize(k - 1);
    return hull;
}

struct StockInWcs {
    geom2d::Regions outline; ///< XY footprint.
    double top = 0.0;
    double bottom = 0.0;
};

StockInWcs stockInWcs(const Document& doc, const Setup& setup) {
    const auto stock = doc.stockShape(setup.id);
    if (!stock) {
        throw userError("the setup's stock cannot be resolved");
    }
    const Transform3 toWcs = setup.wcs.toParent().inverse();
    StockInWcs out;
    std::vector<Vec2> footprint;
    out.top = -std::numeric_limits<double>::infinity();
    out.bottom = std::numeric_limits<double>::infinity();
    auto add = [&](const Vec3& projectPoint) {
        const Vec3 p = toWcs * projectPoint;
        footprint.push_back(p.head<2>());
        out.top = std::max(out.top, p.z());
        out.bottom = std::min(out.bottom, p.z());
    };
    if (stock->kind == StockShape::Kind::Box) {
        for (const Vec3& corner : stock->corners()) {
            add(corner);
        }
    } else {
        // Sample both end circles of the cylinder.
        const Vec3 a = stock->axis.normalized();
        const Vec3 u = a.unitOrthogonal();
        const Vec3 v = a.cross(u);
        for (int i = 0; i < 72; ++i) {
            const double t = 2.0 * kPi * i / 72.0;
            const Vec3 radial = (std::cos(t) * u + std::sin(t) * v) * (stock->diameter / 2.0);
            add(stock->base + radial);
            add(stock->base + a * stock->length + radial);
        }
    }
    out.outline = {geom2d::Region{geom2d::Polyline::fromPoints(convexHull(footprint), true), {}}};
    return out;
}

const cad::Model& requireModel(const Document& doc, const Uuid& modelId) {
    const cad::Model* m = doc.model(modelId);
    if (!m) {
        throw userError("a referenced model is not loaded");
    }
    return *m;
}

/// Planar faces referenced by the operation, as one region set at one height in the WCS.
cad::FaceOutline selectedFaces(const Document& doc, const Setup& setup, const Operation& op,
                               double tol) {
    if (op.geometry.empty()) {
        throw userError("select the face that defines this operation (a horizontal face)");
    }
    cad::FaceOutline combined;
    bool first = true;
    for (const ShapeRef& ref : op.geometry) {
        const ModelEntry* entry = doc.project().findModel(ref.modelId);
        if (!entry) {
            throw userError("the operation refers to a model that is no longer in the project");
        }
        const cad::Model& model = requireModel(doc, ref.modelId);
        if (ref.type != cad::SubShapeType::Face) {
            throw userError("this operation uses faces only; remove the selected edges");
        }
        const auto index = cad::matchSubShape(model, ref.type, ref.signature, ref.index);
        if (!index) {
            throw userError("a selected face no longer exists in the model; select it again");
        }
        const cad::FaceOutline face =
            cad::faceOutline(model, *index, modelToWcs(*entry, setup), tol);
        if (first) {
            combined = face;
            first = false;
        } else {
            if (std::abs(face.z - combined.z) > 1e-6) {
                throw userError("selected faces must lie at the same height");
            }
            combined.regions = geom2d::unite(combined.regions, face.regions, {tol});
        }
    }
    return combined;
}

/// The selected faces of a 3D finishing operation, in the WCS: their triangles and the plan
/// view they cover. Empty when nothing is selected.
struct SelectedSurface {
    geom3d::Mesh mesh;
    geom2d::Regions plan;
};
SelectedSurface selectedSurface(const Document& doc, const Setup& setup, const Operation& op,
                                double tol) {
    SelectedSurface out;
    for (const ShapeRef& ref : op.geometry) {
        if (ref.type != cad::SubShapeType::Face) {
            throw userError("select faces to contain this operation; remove the selected edges");
        }
        const ModelEntry* entry = doc.project().findModel(ref.modelId);
        if (!entry) {
            throw userError("the operation refers to a model that is no longer in the project");
        }
        const cad::Model& model = requireModel(doc, ref.modelId);
        const auto index = cad::matchSubShape(model, ref.type, ref.signature, ref.index);
        if (!index) {
            throw userError("a selected face no longer exists in the model; select it again");
        }
        geom3d::Mesh mesh = cad::tessellateFaces(model, {*index}, {0.005, 0.2});
        mesh.transform(modelToWcs(*entry, setup));
        out.mesh.append(mesh);
    }
    // The plan view: the union of the triangles' projections, hairline gaps closed.
    geom2d::Regions pieces;
    for (std::size_t t = 0; t < out.mesh.triangles.size(); ++t) {
        const auto& tri = out.mesh.triangles[t];
        std::vector<Vec2> pts{out.mesh.vertices[tri[0]].head<2>(),
                              out.mesh.vertices[tri[1]].head<2>(),
                              out.mesh.vertices[tri[2]].head<2>()};
        geom2d::Polyline outline = geom2d::Polyline::fromPoints(pts, true);
        const double area = outline.signedArea();
        if (std::abs(area) < 1e-10) {
            continue;
        }
        if (area < 0.0) {
            outline.reverse();
        }
        pieces.push_back({std::move(outline), {}});
    }
    const geom2d::ClipOptions clip{tol};
    const double close = std::max(10.0 * tol, 0.01);
    out.plan =
        geom2d::offset(geom2d::offset(geom2d::unite(pieces, {}, clip), close, clip), -close, clip);
    return out;
}

/// The feed height: where the tool rapids straight down from above it into the operation's top
/// (to enter its first level), the rapid stops at the feed height and the rest is fed. Entries
/// to lower levels keep the strategy's own clearance above the material they enter.
void feedFromFeedHeight(toolpath::Toolpath& tp, double feedHeight, double topZ, double plungeFeed) {
    constexpr double eps = 1e-6;
    std::vector<toolpath::Move> moves;
    moves.reserve(tp.moves.size());
    Vec3 at = tp.start;
    for (const toolpath::Move& m : tp.moves) {
        const bool straightDown = m.kind == toolpath::MoveKind::Rapid &&
                                  (m.end.head<2>() - at.head<2>()).norm() < eps &&
                                  m.end.z() < at.z() - eps;
        if (straightDown && at.z() > feedHeight + eps && m.end.z() < feedHeight - eps &&
            m.end.z() >= topZ - eps) {
            toolpath::Move rapid = m;
            rapid.end.z() = feedHeight;
            moves.push_back(rapid);
            toolpath::Move feed = m;
            feed.kind = toolpath::MoveKind::Linear;
            feed.feedClass = toolpath::FeedClass::Plunge;
            feed.feedRate = plungeFeed;
            moves.push_back(feed);
        } else {
            moves.push_back(m);
        }
        at = m.end;
    }
    tp.moves = std::move(moves);
}

/// Selected outlines of a contour or chamfer that run on one side (and, for chamfers, at one
/// height): the faces' outlines and the closed loops of the edges, united.
struct ContourOutline {
    cad::FaceOutline outline; ///< z: the lowest of them.
    GeometrySide side = GeometrySide::Default;
};

/// The selection of a contour or chamfer grouped by the side each face or loop of edges runs on
/// and, with `byHeight` (chamfers: each at its own height), by height; highest first. Outlines
/// without a side of their own take the side they open to (automaticSide); outside when that
/// cannot be told.
std::vector<ContourOutline> contourOutlines(const Document& doc, const Setup& setup,
                                            const Operation& op, double tol, bool byHeight) {
    if (op.geometry.empty()) {
        throw userError("select the faces or the edges (closed loops) to follow");
    }
    // Outlines without a side of their own (not flipped): the side each opens to.
    const auto sideOf = [&](const ShapeRef& ref) {
        return ref.side == GeometrySide::Default ? automaticSide(doc, setup, ref) : ref.side;
    };
    std::vector<std::pair<long long, ContourOutline>> groups; // (height in um or 0, outline)
    const auto add = [&](const cad::FaceOutline& outline, GeometrySide side) {
        const long long level = byHeight ? std::llround(outline.z * 1000.0) : 0;
        for (auto& [l, g] : groups) {
            if (l == level && g.side == side) {
                g.outline.regions = geom2d::unite(g.outline.regions, outline.regions, {tol});
                g.outline.z = std::min(g.outline.z, outline.z);
                return;
            }
        }
        groups.push_back({level, ContourOutline{outline, side}});
    };
    // Edges by model, side and (chamfers) height: each group chains into closed loops.
    std::map<std::tuple<Uuid, GeometrySide, long long>, std::vector<int>> edges;
    for (const ShapeRef& ref : op.geometry) {
        if (ref.type == cad::SubShapeType::Face) {
            Operation one = op;
            one.geometry = {ref};
            add(selectedFaces(doc, setup, one, tol), sideOf(ref));
            continue;
        }
        const ModelEntry* entry = doc.project().findModel(ref.modelId);
        if (!entry) {
            throw userError("the operation refers to a model that is no longer in the project");
        }
        const cad::Model& model = requireModel(doc, ref.modelId);
        const auto index = cad::matchSubShape(model, ref.type, ref.signature, ref.index);
        if (!index) {
            throw userError("a selected edge no longer exists in the model; select it again");
        }
        const double z = (modelToWcs(*entry, setup) * ref.signature.centroid).z();
        edges[{ref.modelId, sideOf(ref), byHeight ? std::llround(z * 1000.0) : 0}].push_back(
            *index);
    }
    for (const auto& [group, indices] : edges) {
        const Uuid& modelId = std::get<0>(group);
        const ModelEntry* entry = doc.project().findModel(modelId);
        try {
            add(cad::edgeLoops(requireModel(doc, modelId), indices, modelToWcs(*entry, setup), tol),
                std::get<1>(group));
        } catch (const Error& e) {
            throw userError(
                byHeight ? fmt::format("{} (the edges at each height must close a loop)", e.what())
                         : std::string(e.what()));
        }
    }
    std::vector<ContourOutline> out;
    for (auto& [level, g] : groups) {
        out.push_back(std::move(g));
    }
    std::stable_sort(out.begin(), out.end(), [](const ContourOutline& a, const ContourOutline& b) {
        return a.outline.z > b.outline.z;
    });
    return out;
}

/// The selected edges of a trace, chained open or closed per model, in the setup's WCS.
std::vector<strategies::TraceChain> traceChains(const Document& doc, const Setup& setup,
                                                const Operation& op, double tol) {
    std::map<Uuid, std::vector<int>> edges;
    for (const ShapeRef& ref : op.geometry) {
        if (ref.type != cad::SubShapeType::Edge) {
            throw userError("a trace follows edges: select edges, not faces");
        }
        if (!doc.project().findModel(ref.modelId)) {
            throw userError("the operation refers to a model that is no longer in the project");
        }
        const auto index =
            cad::matchSubShape(requireModel(doc, ref.modelId), ref.type, ref.signature, ref.index);
        if (!index) {
            throw userError("a selected edge no longer exists in the model; select it again");
        }
        edges[ref.modelId].push_back(*index);
    }
    if (edges.empty()) {
        throw userError("select the edges to trace");
    }
    std::vector<strategies::TraceChain> chains;
    for (const auto& [modelId, indices] : edges) {
        const ModelEntry* entry = doc.project().findModel(modelId);
        for (cad::EdgeChain& c :
             cad::edgeChains(requireModel(doc, modelId), indices, modelToWcs(*entry, setup), tol)) {
            chains.push_back({std::move(c.path), c.z});
        }
    }
    if (chains.empty()) {
        throw userError("the selected edges run along the tool axis: there is nothing to trace");
    }
    return chains;
}

} // namespace

GeometrySide automaticSide(const Document& doc, const Setup& setup, const ShapeRef& ref) {
    const ModelEntry* entry = doc.project().findModel(ref.modelId);
    const cad::Model* model = doc.model(ref.modelId);
    if (!entry || !model || ref.type == cad::SubShapeType::Vertex) {
        return GeometrySide::Default;
    }
    const auto index = cad::matchSubShape(*model, ref.type, ref.signature, ref.index);
    if (!index) {
        return GeometrySide::Default;
    }
    const Transform3 toWcs = modelToWcs(*entry, setup);
    constexpr double tol = 0.01;
    cad::FaceOutline outline;
    try {
        if (ref.type == cad::SubShapeType::Face) {
            outline = cad::faceOutline(*model, *index, toWcs, tol);
        } else {
            // The loop on the face across the tool axis (the WCS Z in model coordinates).
            const Vec3 axis = toWcs.linear().transpose() * Vec3::UnitZ();
            outline = cad::edgeLoops(*model, cad::edgeLoopOf(*model, *index, axis), toWcs, tol);
        }
    } catch (const Error&) {
        return GeometrySide::Default; // not horizontal, or no closed loop
    }
    if (outline.regions.empty()) {
        return GeometrySide::Default;
    }
    // A point on the outline (the middle of its longest segment) and the way into it.
    geom2d::Polyline loop = outline.regions.front().outer;
    if (loop.signedArea() < 0.0) {
        loop.reverse();
    }
    const std::vector<Vec2> pts = loop.discretize(tol);
    if (pts.size() < 3) {
        return GeometrySide::Default;
    }
    std::size_t longest = 0;
    for (std::size_t k = 0; k < pts.size(); ++k) {
        if ((pts[(k + 1) % pts.size()] - pts[k]).norm() >
            (pts[(longest + 1) % pts.size()] - pts[longest]).norm()) {
            longest = k;
        }
    }
    const Vec2 a = pts[longest];
    const Vec2 b = pts[(longest + 1) % pts.size()];
    const Vec2 mid = 0.5 * (a + b);
    const Vec2 t = (b - a).normalized();
    const Vec2 in(-t.y(), t.x()); // counter-clockwise: the inside on the left
    const Transform3 toModel = toWcs.inverse();
    constexpr double step = 0.05; // across the outline and in height
    const auto solid = [&](const Vec2& xy, double z) {
        return cad::containsPoint(*model, toModel * Vec3(xy.x(), xy.y(), z));
    };
    // Just below its height first (a top edge: the part under one side), then just above (a
    // floor's edge: a wall rising on one side).
    for (const double dz : {-step, step}) {
        const bool inside = solid(mid + step * in, outline.z + dz);
        const bool outside = solid(mid - step * in, outline.z + dz);
        if (inside != outside) {
            return inside ? GeometrySide::Outside : GeometrySide::Inside;
        }
    }
    return GeometrySide::Default;
}

namespace {

/// The operation's side, or the outline's own.
strategies::ContourSide sideOf(GeometrySide side, strategies::ContourSide operationSide) {
    return side == GeometrySide::Outside  ? strategies::ContourSide::Outside
           : side == GeometrySide::Inside ? strategies::ContourSide::Inside
                                          : operationSide;
}

/// Appends a toolpath that starts and ends at the clearance height (the next pass of a contour
/// or chamfer on another side or height).
void appendPass(toolpath::Toolpath& tp, const toolpath::Toolpath& pass, bool first) {
    if (first) {
        tp = pass;
        return;
    }
    tp.moves.insert(tp.moves.end(), pass.moves.begin(), pass.moves.end());
    tp.warnings.insert(tp.warnings.end(), pass.warnings.begin(), pass.warnings.end());
}

/// Holes drillable along the tool axis: the ones whose faces the operation selects, or, without
/// a selection, every hole of the setup's model that `matches` accepts (by diameter). With
/// `bosses`, the bosses a tool can go around from along its axis instead (DL-0081).
std::vector<strategies::HoleTarget>
selectedHoles(const Document& doc, const Setup& setup, const Operation& op,
              const std::function<bool(double)>& matches, const std::string& noMatchMessage,
              std::vector<std::string>& warnings, bool bosses = false) {
    std::vector<strategies::HoleTarget> targets;
    const Project& project = doc.project();
    std::vector<Uuid> modelIds;
    if (!op.geometry.empty()) {
        for (const auto& ref : op.geometry) {
            if (std::find(modelIds.begin(), modelIds.end(), ref.modelId) == modelIds.end()) {
                modelIds.push_back(ref.modelId);
            }
        }
    } else if (setup.modelId) {
        modelIds.push_back(*setup.modelId);
    }
    for (const Uuid& modelId : modelIds) {
        const ModelEntry* entry = project.findModel(modelId);
        if (!entry) {
            continue;
        }
        const cad::Model& model = requireModel(doc, modelId);
        const Transform3 toWcs = modelToWcs(*entry, setup);
        // Tool axis (WCS +Z) expressed in model coordinates.
        const Vec3 axisInModel = toWcs.linear().transpose() * Vec3::UnitZ();
        std::set<int> selectedFaces;
        for (const auto& ref : op.geometry) {
            if (ref.modelId == modelId) {
                if (auto idx = cad::matchSubShape(model, ref.type, ref.signature, ref.index)) {
                    selectedFaces.insert(*idx);
                }
            }
        }
        // Bosses (external threads) stand where holes go in: from their free end, its length.
        std::vector<cad::Hole> found;
        if (bosses) {
            for (const cad::Boss& boss : cad::findBosses(model, axisInModel)) {
                found.push_back({boss.entry, boss.direction, boss.diameter, boss.length,
                                 cad::HoleBottom::Flat, 0.0, boss.faces});
            }
        } else {
            found = cad::findHoles(model, axisInModel);
        }
        for (const cad::Hole& hole : found) {
            if (hole.direction.dot(axisInModel) > -0.999) {
                continue; // not drillable from the top of this setup
            }
            const bool picked = std::any_of(hole.faces.begin(), hole.faces.end(),
                                            [&](int f) { return selectedFaces.contains(f); });
            if (!op.geometry.empty() ? !picked : !matches(hole.diameter)) {
                continue;
            }
            const Vec3 entryWcs = toWcs * hole.entry;
            targets.push_back({entryWcs.head<2>(), entryWcs.z(), hole.depth,
                               hole.bottom == cad::HoleBottom::Through, hole.diameter});
        }
    }
    if (targets.empty()) {
        warnings.push_back(op.geometry.empty() ? noMatchMessage
                           : bosses
                               ? "none of the selected faces belongs to a boss the tool can go "
                                 "around from along its axis"
                               : "none of the selected faces belongs to a hole drillable along "
                                 "the tool axis");
    }
    return targets;
}

/// The setup's model as a triangle mesh in the WCS (3D strategies).
geom3d::Mesh partMeshInWcs(const Document& doc, const Setup& setup, double deflection) {
    if (!setup.modelId || !doc.project().findModel(*setup.modelId)) {
        throw userError("3D operations need the setup's model");
    }
    const ModelEntry& entry = *doc.project().findModel(*setup.modelId);
    geom3d::Mesh mesh = cad::tessellate(requireModel(doc, entry.id), {deflection, 0.2});
    mesh.transform(modelToWcs(entry, setup));
    return mesh;
}

/// The setup's fixtures and workholding as one triangle mesh in the WCS (empty without them),
/// or in `frame` (given in the WCS: an operation's own frame). They are placed on the setup
/// itself (the workholding closes on the stock as the setup's WCS has it), then moved.
geom3d::Mesh fixtureMeshInWcs(const Document& doc, const Setup& setup,
                              const Transform3& frame = Transform3::Identity()) {
    geom3d::Mesh out;
    for (const sim::FixtureBody& body : fixtureBodiesInWcs(doc, setup)) {
        out.append(body.mesh);
    }
    if (!frame.isApprox(Transform3::Identity(), 1e-12)) {
        out.transform(frame.inverse());
    }
    return out;
}

/// Stock radius about the WCS X axis (4th-axis setups): a cylinder or tube stock of fixed size
/// on that axis, or the part's largest radius grown by the side offset.
double rotaryStockRadius(const Document& doc, const Setup& setup, const geom3d::Mesh& partWcs) {
    const Stock& st = setup.stock;
    if ((st.kind == StockKind::Cylinder || st.kind == StockKind::Tube) &&
        st.size == StockSize::Fixed) {
        const auto shape = doc.stockShape(setup.id);
        if (!shape) {
            throw userError("the setup's stock cannot be resolved");
        }
        const Transform3 toWcs = setup.wcs.toParent().inverse();
        const Vec3 base = toWcs * shape->base;
        const Vec3 axis = (toWcs.linear() * shape->axis).normalized();
        if (std::abs(std::abs(axis.x()) - 1.0) > 1e-6 || base.tail<2>().norm() > 1e-6) {
            throw userError("the cylinder stock must lie on the WCS X axis (the rotary axis)");
        }
        return shape->diameter / 2.0;
    }
    double r = 0.0;
    for (const Vec3& v : partWcs.vertices) {
        r = std::max(r, std::hypot(v.y(), v.z()));
    }
    return r + (st.size == StockSize::Part ? 0.0 : st.offsetSide);
}

bool restRequested(const Operation& op) {
    const strategies::StrategyInfo* info = strategies::findStrategy(op.type);
    if (!info || std::none_of(info->params.begin(), info->params.end(),
                              [](const auto& d) { return d.key == "rest_machining"; })) {
        return false;
    }
    try {
        return strategies::ParamReader(*info, op.parameters).flag("rest_machining");
    } catch (const Error&) {
        return false; // invalid parameters fail at generation; keys just have to be stable
    }
}

} // namespace

std::vector<LatheSupport> latheSupports(const Document& doc, const Setup& setup) {
    std::vector<LatheSupport> out;
    // Shown with the setup: without a machine there is none to show them on.
    const machine::Machine* machine = findMachineFor(doc.project(), setup);
    if (!machine || !machine->isLathe()) {
        return out;
    }
    if (const auto& t = setup.tailstock) {
        const LatheFitting* centre = t->centre ? doc.project().findFitting(*t->centre) : nullptr;
        const cad::Model* model = centre ? doc.modelData(centre->sha256) : nullptr;
        if (t->centre && !model) {
            throw userError("the tailstock's centre is not in the project: choose another");
        }
        if (centre && model) {
            // Its own model, tip at the point; the quill from where it seats.
            const double seat = t->tipZ + centreLength(*centre);
            const double q = t->quillDiameter / 2.0;
            out.push_back({"Tailstock", *Uuid::parse("0a5e1e0d-7a11-4570-8c6b-000000000001"),
                           cad::revolvedAboutZ({{seat, 0.0},
                                                {seat, q},
                                                {seat + t->quillLength, q},
                                                {seat + t->quillLength, 0.0}})});
            out.push_back({"Tailstock centre", tailstockCentreSupportId(),
                           cad::transformed(*model, centreInWcs(*centre, t->tipZ))});
        } else {
            const double r = t->centreDiameter / 2.0;
            const double point = r / std::tan(t->centreAngle / 2.0);
            const double back = t->tipZ + point + t->quillLength;
            out.push_back({"Tailstock", *Uuid::parse("0a5e1e0d-7a11-4570-8c6b-000000000001"),
                           cad::revolvedAboutZ({{t->tipZ, 0.0},
                                                {t->tipZ + point, r},
                                                {t->tipZ + point, t->quillDiameter / 2.0},
                                                {back, t->quillDiameter / 2.0},
                                                {back, 0.0}})});
        }
    }
    if (const auto& s = setup.steadyRest) {
        double grip = s->gripDiameter / 2.0;
        if (grip <= 0.0) {
            CancelToken never;
            const geom2d::Regions there =
                geom2d::intersect(latheGeometry(doc, setup, never).stock,
                                  {{geom2d::Polyline::rectangle(Vec2(s->z - 0.5 * s->width, -1.0),
                                                                Vec2(s->z + 0.5 * s->width, 1e6)),
                                    {}}},
                                  {1e-3});
            for (const auto& r : there) {
                grip = std::max(grip, r.bounds().max.y());
            }
            if (grip <= 0.0) {
                throw userError(fmt::format(
                    "the steady rest at Z{:.3f} has no stock there to close on: set what it grips",
                    s->z));
            }
        }
        const double z0 = s->z - 0.5 * s->width;
        const double z1 = s->z + 0.5 * s->width;
        const double outer = std::max(s->outerDiameter / 2.0, grip + 1.0);
        out.push_back({"Steady rest", *Uuid::parse("0a5e1e0d-7a11-4570-8c6b-000000000002"),
                       cad::revolvedAboutZ({{z0, grip}, {z1, grip}, {z1, outer}, {z0, outer}})});
    }
    return out;
}

std::vector<sim::FixtureBody> fixtureBodiesInWcs(const Document& doc, const Setup& setup) {
    std::vector<sim::FixtureBody> out;
    // Coarse facets are enough: the clearance is far larger than the chord error.
    const cad::TessellationOptions coarse{0.05, 0.35};
    for (const LatheSupport& support : latheSupports(doc, setup)) {
        out.push_back({support.name, cad::tessellate(support.model, coarse)});
    }
    for (const Uuid& id : setup.fixtures) {
        const ModelEntry* entry = doc.project().findModel(id);
        if (!entry) {
            throw userError("a fixture of the setup is no longer in the project");
        }
        geom3d::Mesh mesh = cad::tessellate(requireModel(doc, id), coarse);
        mesh.transform(modelToWcs(*entry, setup));
        out.push_back({entry->name, std::move(mesh)});
    }
    if (const auto pose = setupWorkholdingPose(doc, setup)) {
        for (const PosedPart& body : pose->parts) {
            if (!body.collides) {
                continue; // kept out of the clearance and the checks on purpose
            }
            const cad::Model* model = posedPartModel(doc, body);
            if (!model) {
                throw userError("a part of the setup's workholding is not loaded");
            }
            geom3d::Mesh mesh = cad::tessellate(*model, coarse);
            mesh.transform(body.toWcs);
            out.push_back({body.name, std::move(mesh)});
        }
    }
    return out;
}

SoftJawBoring softJawBoring(const Document& doc, const Setup& setup) {
    const auto pose = setupWorkholdingPose(doc, setup);
    if (!pose || !pose->jaws || !pose->jaws->boreDiameter) {
        throw userError("the setup's jaws are not soft jaws bored to fit: choose a soft jaw set "
                        "and its bore in the setup's workholding");
    }
    const JawPlacement& j = *pose->jaws;
    SoftJawBoring out;
    out.bore = j.outside;
    out.diameter = *j.boreDiameter;
    out.depth = j.boreDepth;
    out.clampDiameter = j.clampDiameter;
    out.floor = j.stopZ;
    // Each jaw as it stands while bored: back from where it holds the work, radially.
    const double back = (j.boringTravel - j.travel) * j.radiusPerMm;
    const cad::TessellationOptions coarse{0.05, 0.35};
    for (const PosedPart& part : pose->parts) {
        if (!part.cut) {
            continue;
        }
        const cad::Model* model = doc.modelData(part.sha256);
        if (!model) {
            throw userError("a soft jaw of the setup is not loaded");
        }
        geom3d::Mesh mesh = cad::tessellate(*model, coarse);
        mesh.transform(part.toWcs);
        const geom3d::Aabb3 box = mesh.bounds();
        Vec3 out3 = 0.5 * (box.min + box.max);
        out3.z() = 0.0;
        if (out3.norm() > 1e-9) {
            Transform3 move = Transform3::Identity();
            move.translation() = out3.normalized() * back;
            mesh.transform(move);
        }
        out.before = geom2d::unite(out.before, sim::revolvedEnvelope(mesh, 0.5, 0.0));
    }
    if (out.before.empty()) {
        throw userError("the setup's soft jaws have no parts to bore");
    }
    geom2d::Aabb2 all;
    for (const auto& r : out.before) {
        all.extend(r.bounds());
    }
    const double r = 0.5 * out.diameter;
    const double far = all.max.y() + 10.0;
    const geom2d::Regions removed{
        {out.bore ? geom2d::Polyline::rectangle(Vec2(j.stopZ, -1.0), Vec2(all.max.x() + 10.0, r))
                  : geom2d::Polyline::rectangle(Vec2(j.stopZ, r), Vec2(all.max.x() + 10.0, far)),
         {}}};
    out.after = geom2d::subtract(out.before, removed, {1e-3});
    return out;
}

std::vector<sim::LatheBody> latheBodies(const Document& doc, const Setup& setup, double grow) {
    std::vector<sim::LatheBody> out;
    for (const sim::FixtureBody& body : fixtureBodiesInWcs(doc, setup)) {
        geom2d::Regions area = sim::revolvedEnvelope(body.mesh, 0.5, grow);
        if (!area.empty()) {
            out.push_back({body.name, std::move(area)});
        }
    }
    return out;
}

Transform3 modelToWcs(const ModelEntry& model, const Setup& setup) {
    return setup.wcs.toParent().inverse() * model.placement.toParent();
}

namespace {

geom2d::Regions carriedLatheStock(const Document& doc, const Setup& setup,
                                  const geom2d::Regions& part, std::vector<std::string>& warnings,
                                  sim::LatheStock& cuts, CancelToken& cancel);

} // namespace

LatheGeometry latheGeometry(const Document& doc, const Setup& setup, CancelToken& cancel) {
    if (!setup.modelId || !doc.project().findModel(*setup.modelId)) {
        throw userError("turning needs the setup's model");
    }
    const ModelEntry& entry = *doc.project().findModel(*setup.modelId);
    LatheGeometry g;
    const cad::Model& model = requireModel(doc, entry.id);
    const Transform3 frame = modelToWcs(entry, setup);
    g.partIsRound = cad::turnsAboutZ(model, frame);
    g.part = g.partIsRound ? cad::axialProfile(model, frame, 1e-3)
                           : cad::axialEnvelope(model, frame, 1e-3);
    if (g.part.empty()) {
        throw userError("the model does not cross the spindle axis plane: align the setup's WCS Z "
                        "with the part's axis");
    }
    geom2d::Aabb2 pb;
    for (const auto& r : g.part) {
        pb.extend(r.bounds());
    }
    const Stock& st = setup.stock;
    if (st.kind == StockKind::FromSetup) {
        g.stock = carriedLatheStock(doc, setup, g.part, g.warnings, g.carriedCuts, cancel);
        return g;
    }
    const bool round = st.kind == StockKind::Cylinder || st.kind == StockKind::Tube;
    double z0 = 0.0, z1 = 0.0, rIn = 0.0, rOut = 0.0;
    if (round && st.size == StockSize::Fixed) {
        const auto shape = doc.stockShape(setup.id);
        if (!shape) {
            throw userError("the setup's stock cannot be resolved");
        }
        const Transform3 toWcs = setup.wcs.toParent().inverse();
        const Vec3 base = toWcs * shape->base;
        const Vec3 axis = (toWcs.linear() * shape->axis).normalized();
        if (std::abs(std::abs(axis.z()) - 1.0) > 1e-6 || base.head<2>().norm() > 1e-6) {
            throw userError("the cylinder stock must lie on the WCS Z axis (the spindle axis)");
        }
        z0 = std::min(base.z(), base.z() + axis.z() * shape->length);
        z1 = std::max(base.z(), base.z() + axis.z() * shape->length);
        rIn = shape->innerDiameter / 2.0;
        rOut = shape->diameter / 2.0;
    } else {
        // A bar around the part: radius grown by the side offset, front by the top offset.
        const bool part = st.size == StockSize::Part;
        z0 = pb.min.x() - (part ? 0.0 : st.offsetBottom);
        z1 = pb.max.x() + (part ? 0.0 : st.offsetTop);
        rOut = pb.max.y() + (part ? 0.0 : st.offsetSide);
        if (round && st.size == StockSize::OffsetFromPart && st.roundTo > 0.0) {
            // As resolveStock: the diameter and the length rounded up, the extra at the front.
            auto up = [&](double v) {
                return std::max(v, std::ceil(v / st.roundTo - 1e-9) * st.roundTo);
            };
            rOut = up(2.0 * rOut) / 2.0;
            z1 = z0 + up(z1 - z0);
        }
        if (st.kind == StockKind::Tube) {
            rIn = st.innerDiameter / 2.0;
            if (!(rIn > 0.0 && rIn < rOut)) {
                throw userError("the tube stock's inner diameter must be between 0 and its outer "
                                "diameter");
            }
        }
    }
    g.stock = {{geom2d::Polyline::rectangle(Vec2(z0, rIn), Vec2(z1, rOut)), {}}};
    return g;
}

bool liveDrive(const Operation& op) {
    if (op.type != "drill" && op.type != "tap" && op.type != "bore") {
        return !op.type.starts_with("turn_") && op.type != "bar_pull" &&
               op.type != "spindle_transfer";
    }
    return strategies::ParamReader(*strategies::findStrategy(op.type), op.parameters)
               .choice("drive") == "live";
}

bool liveGoesAround(const Operation& op, const tooling::Tool& tool) {
    if (!liveDrive(op) || op.type == "drill" || op.type == "tap") {
        return false;
    }
    return op.type != "bore" ||
           (tool.type != tooling::ToolType::Reamer && tool.type != tooling::ToolType::BoringBar);
}

bool liveMills(const Project& project, const Operation& op) {
    const tooling::Tool* tool = op.toolId ? project.tools.findTool(*op.toolId) : nullptr;
    return op.enabled && tool && liveGoesAround(op, *tool);
}

bool stockIsMilled(const Project& project, const Setup& setup) {
    for (const Setup* source : stockSources(project, setup)) {
        if (machineFor(project, *source).isLathe() &&
            std::any_of(source->operations.begin(), source->operations.end(),
                        [&](const Operation& op) { return liveMills(project, op); })) {
            return true;
        }
    }
    return false;
}

geom2d::Polyline latheCutter(const tooling::Tool& tool, const toolpath::Toolpath& path) {
    if (!tooling::isTurningTool(tool.type)) {
        return tooling::holeToolOutline(tool);
    }
    const geom2d::Polyline outline =
        tooling::turningShape(tool, tooling::turningSideOf(path.tipQuadrant)).outline;
    return tooling::turningAwayFromChuck(path.tipQuadrant) ? tooling::mirroredInZ(outline)
                                                           : outline;
}

toolpath::Toolpath latheMoves(const toolpath::Toolpath& path, const tooling::Tool& tool) {
    toolpath::Toolpath out = toolpath::expandCycles(path);
    if (tool.type == tooling::ToolType::Tap) {
        for (toolpath::Move& m : out.moves) {
            if (m.kind == toolpath::MoveKind::Linear) {
                m.pitch = tool.mill.threadPitch;
            }
        }
    }
    return out;
}

namespace {

OperationHeightValues heightsIn(const Document& doc, const Setup& setup, const Operation& op);

/// Bounds of lathe-plane regions.
geom2d::Aabb2 latheBounds(const geom2d::Regions& regions) {
    geom2d::Aabb2 box;
    for (const auto& r : regions) {
        box.extend(r.bounds());
    }
    return box;
}

/// The stock an operation of `setup` finds on a lathe (DL-0043): `stock` (the setup's) less what
/// its earlier enabled operations cut, or only their holes; all of them without an `operationId`.
/// An operation that fails is left out: it shows its own error in its setup, and for another
/// setup `warnings` says so.
sim::LatheStock latheStockBefore(const Document& doc, const Setup& setup,
                                 const geom2d::Regions& stock,
                                 const std::optional<Uuid>& operationId, bool holesOnly,
                                 CancelToken& cancel,
                                 std::vector<std::string>* warnings = nullptr) {
    sim::LatheStock current(stock);
    NullProgress quiet;
    for (const Operation& other : setup.operations) {
        if (operationId && other.id == *operationId) {
            break;
        }
        if (!other.enabled ||
            (holesOnly && !isHoleOperation(other.type) && other.type != "bar_pull")) {
            continue;
        }
        if (other.type == "spindle_transfer") {
            continue; // the sub-spindle takes hold (ADR-0007): nothing is cut
        }
        if (other.type == "bar_pull") {
            // The bar comes out of the chuck (DL-0091): whatever is held moves, the rest came off.
            current = current.pulled(
                strategies::ParamReader(*strategies::findStrategy("bar_pull"), other.parameters)
                    .number("pull_length"));
            continue;
        }
        try {
            const tooling::Tool* tool =
                other.toolId ? doc.project().tools.findTool(*other.toolId) : nullptr;
            if (!tool) {
                throw userError("the operation has no tool");
            }
            // In the lathe plane a mill's cut leaves the envelope as it was: the setup going on
            // from here plans around that, and its simulation has the milling (DL-0073).
            if (liveGoesAround(other, *tool)) {
                continue;
            }
            const auto path = cachedToolpath(doc, setup, other, cancel, quiet);
            if (path->live && sim::offTheAxis(*path)) {
                // A tap cuts its thread into a bore already there: it adds nothing to the stock.
                if (tool->type != tooling::ToolType::Tap) {
                    sim::cutHoles(current, tooling::holeToolSide(*tool),
                                  toolpath::expandCycles(*path));
                }
                continue;
            }
            sim::simulateTurning(
                current, latheCutter(*tool, *path),
                latheMoves(latheCutPath(doc, setup, other, *path, cancel, quiet), *tool), 0,
                std::numeric_limits<std::size_t>::max(), &cancel);
        } catch (const OperationCancelled&) {
            throw;
        } catch (const Error& e) {
            if (warnings) {
                warnings->push_back(fmt::format("the stock from setup '{}' leaves out '{}', which "
                                                "failed: {}",
                                                setup.name, other.name, e.what()));
            }
        }
    }
    return current;
}

/// What the earlier setup `setup` takes its stock from left, in `setup`'s lathe plane (DL-0044):
/// the pieces holding some of `part`, since one parted off stays behind in the other chuck.
geom2d::Regions carriedLatheStock(const Document& doc, const Setup& setup,
                                  const geom2d::Regions& part, std::vector<std::string>& warnings,
                                  sim::LatheStock& cuts, CancelToken& cancel) {
    const Project& project = doc.project();
    const std::vector<const Setup*> sources = stockSources(project, setup);
    if (sources.empty()) {
        throw userError("the stock comes from an earlier setup: choose one before this setup");
    }
    // What a mill cut is not axisymmetric, and a bar in its place would send rapids through it.
    for (const Setup* s : sources) {
        if (!machineFor(project, *s).isLathe()) {
            throw userError(fmt::format(
                "the stock comes from setup '{}', which is not on a lathe: what it cut cannot be "
                "held as a turned profile. Turn this setup from a bar or tube",
                s->name));
        }
    }
    const Setup& source = *sources.back();
    // The same spindle axis, the same way round or flipped: lathe plane to lathe plane.
    const Transform3 move = setup.wcs.toParent().inverse() * source.wcs.toParent();
    const Vec3 axis = move.linear() * Vec3::UnitZ();
    const Vec3 origin = move * Vec3::Zero();
    if (std::abs(std::abs(axis.z()) - 1.0) > 1e-6 || origin.head<2>().norm() > 1e-6) {
        throw userError(fmt::format("the WCS Z axis must lie along the WCS Z axis of setup '{}', "
                                    "whose stock this setup continues: both are the spindle axis",
                                    source.name));
    }
    const bool flipped = axis.z() < 0.0;
    const auto moved = [&](const geom2d::Polyline& line) {
        std::vector<Vec2> points = line.discretize(1e-3);
        for (Vec2& q : points) {
            q.x() = (flipped ? -q.x() : q.x()) + origin.z();
        }
        if (flipped) {
            std::reverse(points.begin(), points.end()); // mirrored: keeps the orientation
        }
        return geom2d::Polyline::fromPoints(points, true);
    };
    LatheGeometry earlier = latheGeometry(doc, source, cancel);
    warnings = std::move(earlier.warnings);
    const sim::LatheStock left = latheStockBefore(
        doc, source, earlier.stock, trailingBarPull(source), false, cancel, &warnings);
    geom2d::Regions stock;
    for (const geom2d::Region& r : left.profile()) {
        geom2d::Region piece{moved(r.outer), {}};
        for (const geom2d::Polyline& hole : r.holes) {
            piece.holes.push_back(moved(hole));
        }
        double shared = 0.0;
        for (const auto& s : geom2d::intersect({piece}, part, {1e-3})) {
            shared += s.area();
        }
        if (shared > 1e-6) {
            stock.push_back(std::move(piece));
        }
    }
    if (stock.empty()) {
        throw userError(
            fmt::format("setup '{}' leaves no stock around this setup's part", source.name));
    }
    cuts.takeCuts(earlier.carriedCuts, move);
    cuts.takeCuts(left, move);
    if (setup.workholding) {
        warnings.push_back(fmt::format(
            "the workholding closes on the first stock of setup '{}', not on what it left: check "
            "the jaws against the stock by eye",
            source.name));
    }
    return stock;
}

/// What a turret can reach (DL-0063), the toolpath in its operation's frame: a tool along Z, or
/// across the part. Across it, a hole must cross the spindle's axis or have a Y axis to go by.
/// Where a turret's tool can point: along the spindle or square across it.
void checkLiveReach(const Operation& op) {
    const Transform3 frame =
        op.orientation ? operationFrame(op) : Transform3(Transform3::Identity());
    const Vec3 axis = frame.linear().col(2);
    if (axis.z() < -1.0 + 1e-6) {
        throw userError("a turret's tool comes from the front of the part: turn the part round in "
                        "another setup to drill its back");
    }
    if (axis.z() < 1.0 - 1e-6 && std::abs(axis.z()) > 1e-6) {
        throw userError(fmt::format("the tool's axis is {:.3f}° off the spindle's: a turret's tool "
                                    "goes along it or square across it, without a B axis",
                                    radToDeg(std::acos(std::clamp(axis.z(), -1.0, 1.0)))));
    }
}

/// A live tool going around what it cuts, by the operation's Live tool axes.
void checkLiveMilling(const toolpath::Toolpath& tp, const machine::Machine& machine,
                      const Operation& op) {
    checkLiveReach(op);
    if (tp.compensation != toolpath::Compensation::None) {
        throw userError("a live tool on a lathe does not use the control's cutter compensation: "
                        "set the operation's Compensation to the computer's");
    }
    const Transform3 frame =
        op.orientation ? operationFrame(op) : Transform3(Transform3::Identity());
    const Vec3 axis = frame.linear().col(2);
    const bool across = std::abs(axis.z()) < 1e-6;
    if (tp.liveAxes == toolpath::LiveAxes::Polar) {
        if (across) {
            throw userError("polar interpolation works on the part's face: for a tool across the "
                            "part, set the operation's Live tool axes to the Y axis");
        }
        return;
    }
    if (machine.hasYAxis()) {
        return;
    }
    const double c = across ? std::atan2(axis.y(), axis.x()) : tp.liveC;
    const Vec3 side(-std::sin(c), std::cos(c), 0.0);
    double furthest = 0.0;
    for (const toolpath::Move& m : tp.moves) {
        if (m.kind != toolpath::MoveKind::Dwell) {
            furthest = std::max(furthest, std::abs((frame * m.end).dot(side)));
        }
    }
    if (furthest > 1e-6) {
        throw userError(fmt::format(
            "this operation goes {:.3f} mm off the spindle's centre in Y with C held still, and "
            "machine '{}' has no Y axis: choose a machine with one in the setup{}",
            furthest, machine.name,
            across ? "" : ", or set the operation's Live tool axes to polar interpolation"));
    }
}

void checkLiveHoles(const toolpath::Toolpath& tp, const machine::Machine& machine,
                    const Operation& op) {
    checkLiveReach(op);
    const Transform3 frame =
        op.orientation ? operationFrame(op) : Transform3(Transform3::Identity());
    const Vec3 axis = frame.linear().col(2);
    if (std::abs(axis.z()) > 1e-6 || machine.hasYAxis()) {
        return;
    }
    const double c = std::atan2(axis.y(), axis.x());
    const Vec3 across(-std::sin(c), std::cos(c), 0.0);
    for (const toolpath::Move& m : tp.moves) {
        if (m.kind != toolpath::MoveKind::Cycle) {
            continue;
        }
        const double off = (frame * m.end).dot(across);
        if (std::abs(off) > 1e-6) {
            throw userError(fmt::format(
                "this hole lies {:.3f} mm off the spindle's centre, across the tool, and machine "
                "'{}' has no Y axis: choose a machine with one in the setup, or drill a hole "
                "that crosses the part's axis",
                std::abs(off), machine.name));
        }
    }
}

/// Whether what `setup` starts from, or its operations before `op`, has been milled: a hole may
/// have come that way, which only the simulation's dexels hold.
bool milledBefore(const Project& project, const Setup& setup, const Operation& op) {
    if (stockIsMilled(project, setup)) {
        return true;
    }
    for (const Operation& other : setup.operations) {
        if (other.id == op.id) {
            break;
        }
        if (liveMills(project, other)) {
            return true;
        }
    }
    return false;
}

/// A tap off the spindle's axis needs a hole, as one on it does (DL-0072): each of its strokes in
/// the stock before it, around the core at stations along it, finds nothing solid. Refused when
/// something does. After milling the profile and the holes do not tell: the simulation looks.
void checkTapHoles(const Document& doc, const Setup& setup, const Operation& op,
                   const tooling::Tool& tool, const toolpath::Toolpath& tp, CancelToken& cancel) {
    if (tp.cycles.empty() || milledBefore(doc.project(), setup, op)) {
        return;
    }
    const LatheGeometry lg = latheGeometry(doc, setup, cancel);
    sim::LatheStock before = latheStockBefore(doc, setup, lg.stock, op.id, false, cancel);
    before.takeCuts(lg.carriedCuts, Transform3::Identity());
    const double core = tooling::tapCoreRadius(tool.mill);
    const Transform3 frame =
        op.orientation ? operationFrame(op) : Transform3(Transform3::Identity());
    const Vec3 axis = frame.linear().col(2);
    const Vec3 u = frame.linear().col(0);
    const Vec3 v = frame.linear().col(1);
    const auto inHole = [&](const Vec3& p) {
        return std::any_of(
            before.holes().begin(), before.holes().end(), [&](const sim::LatheHoleCut& h) {
                const Vec3 from = p - h.tip;
                const double along = from.dot(h.axis);
                if (along < -1e-9 || along > h.length + 1e-9) {
                    return false;
                }
                return (from - along * h.axis).norm() <= sim::holeRadiusAt(h, along) + 1e-6;
            });
    };
    const auto solid = [&](const Vec3& p) {
        const Vec2 q(p.z(), p.head<2>().norm());
        return std::any_of(before.profile().begin(), before.profile().end(),
                           [&](const geom2d::Region& r) { return r.contains(q); }) &&
               !inHole(p);
    };
    for (const toolpath::Move& m : tp.moves) {
        if (m.kind != toolpath::MoveKind::Cycle) {
            continue;
        }
        const toolpath::DrillCycle& c = tp.cycles.at(static_cast<std::size_t>(m.cycle));
        const Vec3 top = frame * Vec3(m.end.x(), m.end.y(), c.retractZ);
        const Vec3 tip = frame * Vec3(m.end.x(), m.end.y(), c.bottomZ);
        const double allowance = tooling::tapPointAllowance(core);
        const double length = (top - tip).norm() - allowance;
        const int stations = std::max(2, static_cast<int>(std::ceil(length / 0.25)) + 1);
        for (int k = 0; k < stations; ++k) {
            const Vec3 centre = tip + axis * (allowance + length * k / (stations - 1));
            for (int a = 0; a < 9; ++a) {
                const double angle = 2.0 * kPi * a / 8.0;
                const Vec3 p =
                    a == 8 ? centre
                           : Vec3(centre + core * (std::cos(angle) * u + std::sin(angle) * v));
                if (solid(p)) {
                    throw userError(fmt::format(
                        "T{} would cut solid stock at X{:.3f} Y{:.3f} Z{:.3f}: drill the hole "
                        "first, in an earlier operation",
                        tool.number, p.x(), p.y(), p.z()));
                }
            }
        }
    }
}

/// A hole on a lathe's spindle axis (DL-0043): where it goes, and what is checked of its toolpath.
class LatheHole {
public:
    LatheHole(const Document& doc, const Setup& setup, const Operation& op,
              const tooling::Tool& tool, CancelToken& cancel)
        : doc_(doc), setup_(setup), tool_(tool), cancel_(cancel) {
        using tooling::ToolType;
        const ToolType t = tool.type;
        if (!op.geometry.empty()) {
            throw userError("the spindle drills on its axis only: to drill the selected faces, set "
                            "the operation's Tool drive to Live tool, or clear them");
        }
        if (op.type == "tap" && t != ToolType::Tap) {
            throw userError("tapping needs a tap");
        }
        const bool sized = t == ToolType::Reamer || t == ToolType::BoringBar;
        if (op.type == "bore" && !sized) {
            throw userError("boring on a lathe needs a reamer or a boring bar: other tools bore by "
                            "turning around the hole, which needs live tooling");
        }
        if (op.type == "drill" && t != ToolType::Drill && t != ToolType::SpotDrill &&
            t != ToolType::CenterDrill) {
            throw userError("drilling on a lathe needs a drill, spot drill or centre drill: use "
                            "the Tap operation for a tap, the Bore operation for a reamer or a "
                            "boring bar");
        }
        const double r = tool.mill.diameter / 2.0;
        const double pitch = tool.mill.threadPitch;
        if (t == ToolType::Tap && pitch <= 0.0) {
            throw userError("tapping needs the tap's thread pitch");
        }
        needsBore_ = t == ToolType::Tap || sized;
        // A tap cuts into the wall by design: what has to fit is its tap drill.
        fit_ = t == ToolType::Tap ? r - 0.75 * pitch : r - kFitAllowance;
        core_ = t == ToolType::Tap ? tooling::tapCoreRadius(tool.mill) : r / 2.0;

        const LatheGeometry lg = latheGeometry(doc, setup, cancel);
        part_ = lg.part;
        warnings_ = lg.warnings;
        before_ = latheStockBefore(doc, setup, lg.stock, op.id, false, cancel);
        pb_ = latheBounds(part_);
        const geom2d::Aabb2 sb = latheBounds(lg.stock);
        // Starts at the stock the tool finds in front of it; a tool following a drill, at the
        // wall of the hole it runs in.
        const double band = needsBore_ ? r + 0.5 : r;
        const geom2d::Regions ahead =
            geom2d::intersect(before_.profile(),
                              {{geom2d::Polyline::rectangle(Vec2(sb.min.x() - 1.0, -1.0),
                                                            Vec2(sb.max.x() + 1.0, band)),
                                {}}},
                              {1e-3});
        top_ = ahead.empty() ? pb_.max.x() : latheBounds(ahead).max.x();
        if (std::abs(top_ - pb_.max.x()) < kSnap) {
            top_ = pb_.max.x(); // faced to the part's front
        }
    }

    /// The hole: `depth` from the part's front, or (0) as deep as the part lets the tool go.
    strategies::HoleTarget target(double depth) {
        strategies::HoleTarget hole{Vec2::Zero(), top_, 0.0, false, tool_.mill.diameter};
        double bottom = pb_.max.x() - depth;
        if (depth <= 0.0) {
            const std::vector<Vec2> shape = silhouette(fit_);
            // How far back the tool reaches, and where from its tip it has its full radius.
            double reach = 0.0;
            double point = std::numeric_limits<double>::infinity();
            for (const Vec2& q : shape) {
                reach = std::max(reach, q.x());
                if (std::abs(q.y()) >= fit_ - 1e-9) {
                    point = std::min(point, q.x());
                }
            }
            double lo = pb_.min.x() - reach;
            if (!meets(swept(shape, lo, pb_.max.x()), part_)) {
                bottom = pb_.min.x();
                hole.through = true;
            } else {
                double hi = pb_.max.x();
                while (hi - lo > 1e-4) {
                    const double mid = 0.5 * (lo + hi);
                    (meets(swept(shape, mid, pb_.max.x()), part_) ? lo : hi) = mid;
                }
                // Stopped before its full diameter is in: a hole it does not fit.
                if (pb_.max.x() - hi < std::max(point, 0.1)) {
                    throw userError(fmt::format(
                        "the part has no hole on the spindle axis that T{} fits: set the "
                        "operation's depth to drill one anyway",
                        tool_.number));
                }
                // Exactly on the bottom it found, not the search's last step.
                bottom = snapped(hi);
                tipAtBottom_ = true;
            }
        }
        hole.depth = top_ - bottom;
        return hole;
    }

    /// True when the tip is what reaches target()'s bottom, whatever the depth mode.
    bool tipAtBottom() const noexcept { return tipAtBottom_; }

    /// Refuses what must not run; warns of a hole that cuts the part, which may be meant.
    void check(toolpath::Toolpath& tp) const {
        tp.warnings.insert(tp.warnings.begin(), warnings_.begin(), warnings_.end());
        if (tp.cycles.empty()) {
            return;
        }
        const double bottom = tp.cycles.front().bottomZ;
        if (needsBore_ && meets(swept(silhouette(core_), bottom, top_), before_.profile())) {
            throw userError(fmt::format("T{} would cut solid stock on its way to Z{:.3f}: drill "
                                        "the hole first, in an earlier operation",
                                        tool_.number, bottom));
        }
        if (meets(swept(silhouette(fit_), bottom, pb_.max.x()), part_)) {
            tp.warnings.push_back(fmt::format(
                "T{} cuts into the part: the hole's depth goes further than the part's hole, down "
                "to Z{:.3f}",
                tool_.number, bottom));
        }
        if (!hasFixtures(setup_)) {
            return;
        }
        const auto hits = sim::checkTurningFixtures(
            latheMoves(tp, tool_), tooling::holeToolOutline(tool_),
            latheBodies(doc_, setup_, setup_.fixtureClearance), 1, &cancel_);
        if (!hits.empty()) {
            throw userError(fmt::format(
                "T{} comes within {:.3f} mm of '{}' at Z{:.3f}: set the operation's depth to stop "
                "short of the chuck, grip the bar shorter or lower the setup's fixture clearance",
                tool_.number, setup_.fixtureClearance, hits.front().body,
                hits.front().position.z()));
        }
    }

private:
    /// Half the ±0.05 mm on a hole's diameter that hole matching allows.
    static constexpr double kFitAllowance = 0.025;
    /// Clipping and the search land this close to the height they are after.
    static constexpr double kSnap = 5e-3;

    /// `z`, or the height of the part's vertex it is next to.
    double snapped(double z) const {
        for (const auto& r : part_) {
            for (const Vec2& q : r.outer.discretize(1e-3)) {
                if (std::abs(q.x() - z) < kSnap) {
                    return q.x();
                }
            }
        }
        return z;
    }

    std::vector<Vec2> silhouette(double radius) const {
        std::vector<Vec2> points = tooling::holeToolOutline(tool_).discretize(1e-3);
        for (Vec2& q : points) {
            q.y() = std::clamp(q.y(), -radius, radius);
        }
        return points;
    }
    /// The area `shape` covers with its tip going from `zTop` down to `zTip`.
    static geom2d::Regions swept(const std::vector<Vec2>& shape, double zTip, double zTop) {
        std::vector<Vec2> points;
        for (const Vec2& q : shape) {
            points.emplace_back(q.x() + zTip, q.y());
            points.emplace_back(q.x() + zTop, q.y());
        }
        return {{geom2d::Polyline::fromPoints(sim::convexHull(std::move(points)), true), {}}};
    }
    static bool meets(const geom2d::Regions& a, const geom2d::Regions& b) {
        double area = 0.0;
        for (const auto& r : geom2d::intersect(a, b, {1e-3})) {
            area += r.area();
        }
        return area > 1e-6;
    }

    const Document& doc_;
    const Setup& setup_;
    const tooling::Tool& tool_;
    CancelToken& cancel_;
    geom2d::Regions part_;
    std::vector<std::string> warnings_;
    sim::LatheStock before_;
    geom2d::Aabb2 pb_;
    double top_ = 0.0;
    double fit_ = 0.0;  ///< The radius that has to fit the part's hole.
    double core_ = 0.0; ///< The radius that has to find no stock, when it needs a bore.
    bool needsBore_ = false;
    bool tipAtBottom_ = false;
};

/// About a lathe's spindle, a wrapped operation works about the X axis (DL-0074): the part goes
/// there by a half turn that swaps the WCS's X and Z, and the toolpath comes back by it.
Transform3 swappedXZ() {
    Transform3 t = Transform3::Identity();
    t.linear() << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0;
    return t;
}

/// What a wrapped operation on a lathe works on: the stock as the earlier operations left it, by
/// its radius along the spindle, and heights that are radial, from the largest radius of that
/// stock by the offsets the operation's heights have from the stock's top.
struct LatheWrap {
    std::function<double(double)> radiusAt;
    strategies::Heights heights;
    std::vector<std::string> warnings;
};
LatheWrap latheWrap(const Document& doc, const Setup& setup, const Operation& op,
                    const StockInWcs& stock, const OperationHeightValues& hv, CancelToken& cancel) {
    const LatheGeometry lg = latheGeometry(doc, setup, cancel);
    const sim::LatheStock before = latheStockBefore(doc, setup, lg.stock, op.id, false, cancel);
    std::vector<std::vector<Vec2>> loops;
    for (const geom2d::Region& r : before.profile()) {
        loops.push_back(r.outer.discretize(1e-3));
    }
    LatheWrap out;
    out.radiusAt = [loops](double z) {
        double radius = 0.0;
        for (const std::vector<Vec2>& loop : loops) {
            for (std::size_t k = 0; k < loop.size(); ++k) {
                const Vec2& a = loop[k];
                const Vec2& b = loop[(k + 1) % loop.size()];
                if (z < std::min(a.x(), b.x()) || z > std::max(a.x(), b.x())) {
                    continue;
                }
                const double span = b.x() - a.x();
                radius = std::max(radius, std::abs(span) < 1e-12
                                              ? std::max(a.y(), b.y())
                                              : a.y() + (b.y() - a.y()) * (z - a.x()) / span);
            }
        }
        return radius;
    };
    const double outer = latheBounds(before.profile()).max.y();
    out.heights.clearance = outer + std::max(hv.clearance - stock.top, 0.0);
    out.heights.retract = outer + std::max(hv.retract - stock.top, 0.0);
    out.warnings = lg.warnings;
    return out;
}

/// The largest radius of the stock over the part's length.
double largestRadius(const std::function<double(double)>& radiusAt, const geom3d::Aabb3& part) {
    double radius = 0.0;
    const int samples = 200;
    for (int k = 0; k <= samples; ++k) {
        radius = std::max(radius, radiusAt(part.min.x() + part.size().x() * k / samples));
    }
    return radius;
}

/// A wrapped pocket about a lathe's spindle (DL-0074). The strategy works about the X axis: the
/// part goes there and the toolpath comes back by a half turn that swaps the WCS's X and Z. It
/// finds the stock as the earlier operations left it, so only what is above the part there gets
/// cut. Heights are radial here: from the largest radius of that stock.
toolpath::Toolpath wrappedOnLathe(const Document& doc, const Setup& setup, const Operation& op,
                                  const StockInWcs& stock, const OperationHeightValues& hv,
                                  strategies::RotaryPocketParams params,
                                  const strategies::Context& ctx, CancelToken& cancel) {
    const LatheWrap lw = latheWrap(doc, setup, op, stock, hv, cancel);
    const Transform3 swapped = swappedXZ();
    geom3d::Mesh part = partMeshInWcs(doc, setup, 0.005);
    part.transform(swapped);
    const geom3d::Aabb3 pb = part.bounds();
    const double stockRadius = largestRadius(lw.radiusAt, pb);
    if (stockRadius <= 0.0) {
        throw userError("the stock does not reach the part: nothing to wrap a pocket into");
    }
    params.stockRadiusAt = lw.radiusAt;
    const strategies::Context radial{ctx.tool,           ctx.feeds,  lw.heights,
                                     ctx.chordTolerance, ctx.cancel, ctx.progress};
    toolpath::Toolpath tp =
        strategies::rotaryPocket(part, stockRadius, pb.min.x(), pb.max.x(), params, radial);
    tp = toolpath::transformed(tp, swapped);
    tp.warnings.insert(tp.warnings.begin(), lw.warnings.begin(), lw.warnings.end());
    return tp;
}

/// The edges and slot floors a wrapped contour follows (DL-0084), in the frame `wcsToFrame` maps
/// the WCS to, about its X axis. Edges of one model chain together.
void wrappedSelection(const Document& doc, const Setup& setup, const Operation& op,
                      const Transform3& wcsToFrame, double tol,
                      std::vector<strategies::WrappedCurve>& curves,
                      std::vector<strategies::WrappedSlot>& slots) {
    if (op.geometry.empty()) {
        throw userError("select the edges to follow, or a slot's floor, around the part");
    }
    std::vector<std::pair<Uuid, std::vector<int>>> edges;
    for (const ShapeRef& ref : op.geometry) {
        const ModelEntry* entry = doc.project().findModel(ref.modelId);
        if (!entry) {
            throw userError("the operation refers to a model that is no longer in the project");
        }
        const cad::Model& model = requireModel(doc, ref.modelId);
        if (ref.type == cad::SubShapeType::Vertex) {
            throw userError("a wrapped contour follows edges or a slot's floor: remove the "
                            "selected points");
        }
        const auto index = cad::matchSubShape(model, ref.type, ref.signature, ref.index);
        if (!index) {
            throw userError("a selected face or edge no longer exists in the model; select it "
                            "again");
        }
        if (ref.type == cad::SubShapeType::Face) {
            // Faces of one floor radius are one floor (a cylinder's face may come split).
            const Transform3 toFrame = wcsToFrame * modelToWcs(*entry, setup);
            std::vector<strategies::WrappedCurve> face;
            for (const cad::Chain3& c : cad::faceChains(model, *index, toFrame, tol)) {
                face.push_back({c.points, c.closed});
            }
            const double radius = cad::faceDistanceFromXAxis(model, *index, toFrame);
            auto slot = std::find_if(slots.begin(), slots.end(), [&](const auto& s) {
                return std::abs(s.floorRadius - radius) < 1e-6;
            });
            if (slot == slots.end()) {
                slots.push_back({{}, radius});
                slot = std::prev(slots.end());
            }
            slot->faces.push_back(std::move(face));
            continue;
        }
        auto it = std::find_if(edges.begin(), edges.end(),
                               [&](const auto& e) { return e.first == ref.modelId; });
        if (it == edges.end()) {
            edges.push_back({ref.modelId, {}});
            it = std::prev(edges.end());
        }
        it->second.push_back(*index);
    }
    for (const auto& [modelId, indices] : edges) {
        const Transform3 toFrame =
            wcsToFrame * modelToWcs(*doc.project().findModel(modelId), setup);
        for (const cad::Chain3& c :
             cad::edgeChains3d(requireModel(doc, modelId), indices, toFrame, tol)) {
            curves.push_back({c.points, c.closed});
        }
    }
}

/// A wrapped contour (DL-0084) on a 4th axis about the WCS X axis, or about a lathe's spindle as
/// a wrapped pocket is there.
toolpath::Toolpath wrappedContour(const Document& doc, const Setup& setup, const Operation& op,
                                  const StockInWcs& stock, const OperationHeightValues& hv,
                                  bool onLathe, strategies::RotaryContourParams params,
                                  const strategies::Context& ctx, CancelToken& cancel) {
    const Transform3 toFrame = onLathe ? swappedXZ() : Transform3::Identity();
    std::vector<strategies::WrappedCurve> curves;
    std::vector<strategies::WrappedSlot> slots;
    wrappedSelection(doc, setup, op, toFrame, ctx.chordTolerance / 2.0, curves, slots);
    geom3d::Mesh part = partMeshInWcs(doc, setup, 0.005);
    part.transform(toFrame);
    const geom3d::Aabb3 pb = part.bounds();
    double stockRadius = 0.0;
    strategies::Heights heights = ctx.heights;
    std::vector<std::string> warnings;
    if (onLathe) {
        LatheWrap lw = latheWrap(doc, setup, op, stock, hv, cancel);
        stockRadius = largestRadius(lw.radiusAt, pb);
        if (stockRadius <= 0.0) {
            throw userError("the stock does not reach the part: nothing to cut around it");
        }
        params.stockRadiusAt = lw.radiusAt;
        heights = lw.heights;
        warnings = std::move(lw.warnings);
    } else {
        stockRadius = rotaryStockRadius(doc, setup, part);
    }
    const strategies::Context radial{ctx.tool,           ctx.feeds,  heights,
                                     ctx.chordTolerance, ctx.cancel, ctx.progress};
    toolpath::Toolpath tp = strategies::rotaryContour(curves, slots, stockRadius, pb.min.x(),
                                                      pb.max.x(), params, radial);
    if (onLathe) {
        tp = toolpath::transformed(tp, toFrame); // its own inverse
    }
    tp.warnings.insert(tp.warnings.begin(), warnings.begin(), warnings.end());
    return tp;
}

/// Wrapped around the part (DL-0024, DL-0074, DL-0084).
bool isWrapped(const std::string& type) {
    return type == "rotary_pocket" || type == "rotary_contour";
}

/// Swarf (DL-0085): the selected faces as ruled walls in the WCS, and the floor among them: a
/// flat face facing up, within 45 degrees of the tool axis.
std::pair<std::vector<strategies::RuledWall>, std::optional<strategies::SwarfFloor>>
swarfGeometry(const Document& doc, const Setup& setup, const Operation& op,
              const strategies::SwarfParams& sp, const tooling::Tool& tool, double chordTol) {
    if (op.geometry.empty()) {
        throw userError("select the wall faces to cut with the side of the tool (and a floor, "
                        "if the tool should stay above one)");
    }
    std::vector<strategies::RuledWall> walls;
    std::optional<strategies::SwarfFloor> floor;
    const double r = tool.mill.diameter / 2.0;
    for (const ShapeRef& ref : op.geometry) {
        if (ref.type != cad::SubShapeType::Face) {
            throw userError("swarf milling uses faces only; remove the selected edges");
        }
        const ModelEntry* entry = doc.project().findModel(ref.modelId);
        if (!entry) {
            throw userError("the operation refers to a model that is no longer in the project");
        }
        const cad::Model& model = requireModel(doc, ref.modelId);
        const auto index = cad::matchSubShape(model, ref.type, ref.signature, ref.index);
        if (!index) {
            throw userError("a selected face no longer exists in the model; select it again");
        }
        const Transform3 toWcs = modelToWcs(*entry, setup);
        if (const auto plane = cad::facePlane(model, *index)) {
            const Vec3 n = (toWcs.linear() * plane->normal).normalized();
            if (n.z() >= std::cos(kPi / 4.0)) {
                if (floor) {
                    throw userError("select one floor only: more than one selected flat face "
                                    "faces up");
                }
                floor = strategies::SwarfFloor{toWcs * plane->origin, n};
                continue;
            }
        }
        cad::RulingOptions ro;
        ro.tolerance = sp.tolerance;
        ro.offset = r + sp.stockToLeave;
        ro.chordTolerance = chordTol;
        ro.maxSpacing = std::max(0.5, r);
        cad::FaceRulings rulings = cad::faceRulings(model, *index, toWcs, ro);
        const std::string name = fmt::format("face {}", *index);
        if (!rulings.problem.empty()) {
            throw userError(fmt::format("{} cannot be cut with the side of the tool: {}", name,
                                        rulings.problem));
        }
        strategies::RuledWall wall;
        wall.name = name;
        for (cad::FaceRuling& fr : rulings.rulings) {
            wall.rulings.push_back({std::move(fr.points), std::move(fr.normals), fr.direction});
        }
        walls.push_back(std::move(wall));
    }
    if (walls.empty()) {
        throw userError("select the wall faces to cut with the side of the tool: the selected "
                        "face faces up, so it is taken as the floor");
    }
    return {std::move(walls), floor};
}

/// A bar pull (DL-0091): the puller on the spindle's axis, over the bar the chuck holds as the
/// earlier operations left it.
toolpath::Toolpath barPullToolpath(const Document& doc, const Setup& owner, const Setup& setup,
                                   const Operation& op, const tooling::Tool& tool,
                                   const tooling::CuttingPreset& preset, CancelToken& cancel,
                                   ProgressSink& progress) {
    if (!machineFor(doc.project(), setup).isLathe()) {
        throw userError("a bar pull is a lathe's: choose a lathe in the setup");
    }
    const LatheGeometry lg = latheGeometry(doc, setup, cancel);
    const sim::LatheStock before = latheStockBefore(doc, owner, lg.stock, op.id, false, cancel);
    const strategies::BarPullParams params = strategies::barPullParams(op.parameters, tool);
    const geom2d::Regions bar = before.heldBar();
    strategies::BarPullGeometry g;
    geom2d::Aabb2 box;
    for (const geom2d::Region& r : bar) {
        box.extend(r.bounds());
    }
    if (!bar.empty()) {
        g.front = box.max.x();
        // The bar where the jaws go over it.
        const geom2d::Regions gripped = geom2d::intersect(
            bar, {{geom2d::Polyline::rectangle(Vec2(g.front - params.grip, -1.0),
                                               Vec2(g.front + 1.0, box.max.y() + 1.0)),
                   {}}});
        for (const geom2d::Region& r : gripped) {
            g.radius = std::max(g.radius, r.bounds().max.y());
        }
    }
    const Context ctx{tool, strategies::feedsFor(tool, &preset), {}, 0.002, cancel, progress};
    toolpath::Toolpath tp = strategies::barPull(g, params, ctx);
    tp.warnings.insert(tp.warnings.begin(), lg.warnings.begin(), lg.warnings.end());
    tp.operationId = op.id;
    return tp;
}

/// A transfer to the sub-spindle (ADR-0007, DL-0092): Z2's moves and what the spindles do, every
/// value the user's and checked against the machine; no tool.
toolpath::Toolpath transferToolpath(const Document& doc, const Setup& owner, const Operation& op,
                                    CancelToken& cancel) {
    const Project& project = doc.project();
    const machine::Machine& machine = machineFor(project, owner);
    if (!machine.lathe || !machine.lathe->subSpindle) {
        throw userError(fmt::format("machine '{}' has no sub-spindle: choose a machine with one "
                                    "in the setup",
                                    machine.name));
    }
    if (onSubSpindle(owner)) {
        throw userError("the part is handed over from the main spindle: this setup is on the "
                        "sub-spindle");
    }
    const machine::LatheSubSpindle& sub = *machine.lathe->subSpindle;
    const Setup* target = transferTarget(project, owner);
    if (!target) {
        throw userError(fmt::format(
            "no setup on the sub-spindle takes its stock from this one: add a setup with its "
            "Spindle the sub-spindle and its stock From earlier setup '{}'",
            owner.name));
    }
    const strategies::ParamReader r(*strategies::findStrategy("spindle_transfer"), op.parameters);
    toolpath::SpindleTransfer t;
    t.feed = r.number("z2_feed");
    if (t.feed <= 0.0) {
        throw userError("the Z2 feed is 0: set the feed onto the part in the operation's Z2 feed");
    }
    const std::string spindles = r.choice("spindles");
    t.sync = spindles == "stopped"              ? toolpath::SpindleTransfer::Sync::Stopped
             : spindles == "synchronised_angle" ? toolpath::SpindleTransfer::Sync::Angle
                                                : toolpath::SpindleTransfer::Sync::Speed;
    if (t.sync != toolpath::SpindleTransfer::Sync::Stopped) {
        t.spindle = r.integer("rpm") / 60.0;
        if (t.spindle <= 0.0) {
            throw userError("the speed is 0: set the spindles' speed in the operation's Speed, or "
                            "choose both stopped");
        }
        const auto outside = [&](double lo, double hi) {
            return (hi > 0.0 && t.spindle > hi + 1e-9) || t.spindle < lo - 1e-9;
        };
        if (outside(machine.minSpindle, machine.maxSpindle)) {
            throw userError(fmt::format("the speed, {:.0f} rpm, is outside the main spindle's "
                                        "{:.0f} to {:.0f} rpm on machine '{}'",
                                        t.spindle * 60.0, machine.minSpindle * 60.0,
                                        machine.maxSpindle * 60.0, machine.name));
        }
        if (outside(sub.minSpindle, sub.maxSpindle)) {
            throw userError(fmt::format("the speed, {:.0f} rpm, is outside the sub-spindle's "
                                        "{:.0f} to {:.0f} rpm on machine '{}'",
                                        t.spindle * 60.0, sub.minSpindle * 60.0,
                                        sub.maxSpindle * 60.0, machine.name));
        }
    }
    t.angle = t.sync == toolpath::SpindleTransfer::Sync::Angle ? r.number("angle") : 0.0;
    t.partOff = r.choice("then") == "part_off";
    t.dwell = r.number("dwell");
    if (t.partOff) {
        bool after = false;
        bool partOff = false;
        for (const Operation& o : owner.operations) {
            if (o.id == op.id) {
                after = true;
            } else if (after && o.enabled && o.type == "turn_partoff") {
                partOff = true;
            }
        }
        if (!partOff) {
            throw userError("a part-off must follow the transfer in this setup: add one, or set "
                            "Then to open the main chuck");
        }
    }
    const double grip = transferGrip(op, machine);
    if (grip <= 0.0) {
        throw userError(fmt::format("the grip is 0, and machine '{}' does not say its "
                                    "sub-spindle's: set the operation's Grip",
                                    machine.name));
    }
    const auto pick = pickOffZ2(doc, owner, *target, machine, cancel);
    if (!pick) {
        throw userError(fmt::format(
            "the part on the sub-spindle does not line up with where this setup leaves it: set "
            "the WCS of setup '{}' on this one's Z axis, turned round",
            target->name));
    }
    t.pickZ2 = *pick;
    // Z2 grows away from the main spindle: the jaws stand off the part by the grip and clearance.
    t.approachZ2 = t.pickZ2 + grip + r.number("clearance");
    t.workZ2 = sub.workZ2;
    if (const machine::Axis* z2 = machine.axis("Z2")) {
        for (const auto& [what, value] :
             {std::pair{"takes the part", t.pickZ2}, std::pair{"comes up to it", t.approachZ2},
              std::pair{"cuts", t.workZ2}}) {
            if (value < z2->min - 1e-9 || value > z2->max + 1e-9) {
                throw userError(fmt::format("Z2 {:.3f}, where the sub-spindle {}, is outside "
                                            "its travel, {:.3f} to {:.3f}, on machine '{}'",
                                            value, what, z2->min, z2->max, machine.name));
            }
        }
    }
    toolpath::Toolpath tp;
    tp.strategy = "spindle_transfer";
    tp.operationId = op.id;
    tp.coolant = tooling::Coolant::Off;
    tp.spindleSpeed = t.spindle;
    tp.transfer = t;
    return tp;
}

/// The toolpath of `op` in its own frame. `setup` is the setup as the operation sees it
/// (operationSetup); `owner` the setup itself, whose earlier operations rest machining
/// simulates.
toolpath::Toolpath generateIn(const Document& doc, const Setup& owner, const Setup& setup,
                              const Operation& op, CancelToken& cancel, ProgressSink& progress) {
    const Project& project = doc.project();
    if (op.type == "spindle_transfer") {
        return transferToolpath(doc, owner, op, cancel);
    }
    if (!op.toolId) {
        throw userError("choose a tool for this operation");
    }
    const tooling::Tool* tool = project.tools.findTool(*op.toolId);
    if (!tool) {
        throw userError("the operation's tool is not in the project's tools");
    }
    if (const auto issues = tool->validate(); !issues.empty()) {
        throw userError(fmt::format("tool T{} is invalid: {}", tool->number, issues.front()));
    }
    if (strategies::isProbingStrategy(op.type)) {
        return generateProbing(doc, setup, op, *tool, cancel, progress); // no preset (DL-0090)
    }
    const tooling::CuttingPreset* preset = op.presetId ? tool->findPreset(*op.presetId) : nullptr;
    // Speeds and feeds come only from the operation's cutting preset: nothing is guessed.
    if (!preset) {
        throw userError(op.presetId
                            ? fmt::format("the operation's cutting preset is not on tool T{}; "
                                          "choose one of its presets",
                                          tool->number)
                            : fmt::format("choose a cutting preset for T{}: speeds and feeds come "
                                          "only from the tool's presets",
                                          tool->number));
    }
    // The operation may change some of the preset's values for itself.
    const tooling::CuttingPreset effectivePreset = op.presetOverrides.appliedTo(*preset);
    preset = &effectivePreset;
    if (op.type == "bar_pull") {
        return barPullToolpath(doc, owner, setup, op, *tool, *preset, cancel, progress);
    }
    if (preset->surfaceSpeed <= 0.0) {
        throw userError(fmt::format("the cutting preset '{}' of T{} has no cutting speed",
                                    preset->name, tool->number));
    }
    // Taps and threading inserts feed by the pitch; everything else needs a feed.
    const bool pitchFeed = tool->type == tooling::ToolType::Tap || op.type == "turn_thread";
    if (preset->feedPerTooth <= 0.0 && !pitchFeed) {
        throw userError(fmt::format("the cutting preset '{}' of T{} has no feed per {}",
                                    preset->name, tool->number,
                                    tooling::isTurningTool(tool->type) ? "revolution" : "tooth"));
    }

    const StockInWcs stock = stockInWcs(doc, setup);
    const OperationHeightValues hv = heightsIn(doc, setup, op);
    strategies::Heights heights;
    heights.retract = hv.retract;
    heights.clearance = hv.clearance;
    // The strategies enter each level just above the material (feedClearance); the feed height
    // applies where the tool comes down into the operation's top (feedFromFeedHeight below).
    const Context ctx{*tool, strategies::feedsFor(*tool, preset), heights, 0.002, cancel, progress};
    double entryTop = hv.top.value_or(stock.top); // where entries come down into the material
    const bool onLathe = machineFor(project, setup).isLathe();
    const bool liveAround = onLathe && liveGoesAround(op, *tool);
    if (liveAround && !machineFor(project, setup).liveTooling) {
        throw userError(fmt::format("machine '{}' has no live tooling, which milling on a lathe "
                                    "needs: choose a machine with it in the setup",
                                    machineFor(project, setup).name));
    }
    if (onLathe && (op.type == "multiaxis_finish" || op.type == "swarf5x")) {
        throw userError("multi-axis milling needs a milling machine");
    }
    if (isWrapped(op.type) && !onLathe && !machineFor(project, setup).hasRotary()) {
        throw userError(fmt::format("machine '{}' has no rotary axis, which a wrapped {} needs: "
                                    "choose a machine with one in the setup",
                                    machineFor(project, setup).name,
                                    op.type == "rotary_pocket" ? "pocket" : "contour"));
    }

    toolpath::Toolpath tp;
    if (op.type == "facing") {
        if (!setup.modelId || !project.findModel(*setup.modelId)) {
            throw userError("facing needs the setup's model to know the finished top height");
        }
        const ModelEntry& entry = *project.findModel(*setup.modelId);
        const cad::Model& model = requireModel(doc, entry.id);
        const geom3d::Aabb3 local = model.info().bounds;
        const Transform3 toWcs = modelToWcs(entry, setup);
        double modelTop = -std::numeric_limits<double>::infinity();
        for (int i = 0; i < 8; ++i) {
            const Vec3 corner((i & 1) ? local.max.x() : local.min.x(),
                              (i & 2) ? local.max.y() : local.min.y(),
                              (i & 4) ? local.max.z() : local.min.z());
            modelTop = std::max(modelTop, (toWcs * corner).z());
        }
        tp = strategies::facing(stock.outline, hv.top.value_or(stock.top),
                                hv.bottom.value_or(modelTop),
                                strategies::facingParams(op.parameters, *tool, preset), ctx);
    } else if (op.type == "pocket2d") {
        const cad::FaceOutline floor = selectedFaces(doc, setup, op, ctx.chordTolerance);
        strategies::PocketParams params = strategies::pocketParams(op.parameters, *tool, preset);
        const strategies::ParamReader reader(*strategies::findStrategy("pocket2d"), op.parameters);
        std::optional<StockBefore> restStock;
        if (reader.flag("rest_machining")) {
            ProgressRange simRange(progress, 0.0, 0.5);
            restStock = stockBeforeOperation(doc, owner, op.id, reader.number("rest_resolution"),
                                             cancel, simRange, operationFrame(op));
            const double minThickness = reader.number("rest_min_thickness");
            const sim::Stock* s = &restStock->stock;
            params.restMaterial = [s, minThickness](double z, double zAbove) {
                return sim::materialRegions(*s, z + minThickness, zAbove, minThickness);
            };
            params.restOpening = 0.6 * s->resolution();
        }
        ProgressRange cutRange(progress, restStock ? 0.5 : 0.0, 1.0);
        const Context cutCtx{ctx.tool,           ctx.feeds, ctx.heights,
                             ctx.chordTolerance, cancel,    cutRange};
        tp = strategies::pocket(floor.regions, hv.top.value_or(stock.top),
                                hv.bottom.value_or(floor.z), params, cutCtx);
        if (restStock) {
            tp.warnings.insert(tp.warnings.begin(), restStock->warnings.begin(),
                               restStock->warnings.end());
        }
    } else if (op.type == "plunge_rough") {
        const cad::FaceOutline floor = selectedFaces(doc, setup, op, ctx.chordTolerance);
        tp = strategies::plungeRough(floor.regions, hv.top.value_or(stock.top),
                                     hv.bottom.value_or(floor.z),
                                     strategies::plungeParams(op.parameters, *tool, preset), ctx);
    } else if (op.type == "contour2d") {
        const auto outlines = contourOutlines(doc, setup, op, ctx.chordTolerance, false);
        strategies::ContourParams cp = strategies::contourParams(op.parameters, *tool, preset);
        if (op.startPoint) {
            // The lead-in goes in next to the picked vertex or vertical edge.
            const ModelEntry* entry = project.findModel(op.startPoint->modelId);
            if (!entry) {
                throw userError("the start point refers to a model that is no longer there");
            }
            cp.start = (modelToWcs(*entry, setup) * op.startPoint->signature.centroid).head<2>();
        }
        // One pass per side, each from the clearance height.
        const strategies::ContourSide operationSide = cp.side;
        for (std::size_t k = 0; k < outlines.size(); ++k) {
            cp.side = sideOf(outlines[k].side, operationSide);
            appendPass(tp,
                       strategies::contour(outlines[k].outline.regions, hv.top.value_or(stock.top),
                                           hv.bottom.value_or(outlines[k].outline.z), cp, ctx),
                       k == 0);
        }
    } else if (op.type.rfind("turn_", 0) == 0) {
        if (!tooling::isTurningTool(tool->type)) {
            throw userError("turning operations need a turning insert");
        }
        const LatheGeometry lg = latheGeometry(doc, setup, cancel);
        geom2d::Aabb2 sb;
        for (const auto& r : lg.stock) {
            sb.extend(r.bounds());
        }
        const Context tctx{*tool,
                           strategies::turningFeeds(op.parameters, preset, 2.0 * sb.max.y(),
                                                    op.type != "turn_thread"),
                           heights,
                           ctx.chordTolerance,
                           cancel,
                           progress};
        // The bores of earlier hole operations; otherwise the stock stays the setup's, which sets
        // the passes' start and approach. A Z range works on what the earlier operations left, or
        // it would see stock in its way that is long gone.
        strategies::TurnGeometry tg{
            lg.part, latheStockBefore(doc, owner, lg.stock, op.id,
                                      !strategies::turnHasZRange(op.type, op.parameters), cancel)
                         .profile()};
        // The chuck (fixtures and workholding turning with the spindle, grown by the clearance):
        // outside passes stop in front of what is behind the part's front, by the insert's
        // reach behind its programmed point; any move still reaching them refuses the toolpath.
        const std::vector<sim::LatheBody> bodies =
            hasFixtures(setup) ? latheBodies(doc, owner, setup.fixtureClearance)
                               : std::vector<sim::LatheBody>{};
        const geom2d::Aabb2 pb = [&] {
            geom2d::Aabb2 box;
            for (const auto& r : lg.part) {
                box.extend(r.bounds());
            }
            return box;
        }();
        if (!bodies.empty()) {
            double reach = 0.0;
            for (const Vec2& q : tooling::turningShape(*tool, tooling::TurningSide::Outside)
                                     .outline.discretize(1e-2)) {
                reach = std::max(reach, -q.x());
            }
            for (const sim::LatheBody& body : bodies) {
                geom2d::Aabb2 box;
                for (const auto& r : body.area) {
                    box.extend(r.bounds());
                }
                if (box.max.x() <= pb.max.x()) {
                    tg.backStop = std::max(tg.backStop, box.max.x() + reach);
                }
            }
        }
        if (op.type == "turn_soft_jaws") {
            // The jaws, not the work: where they stand while bored, and what the bore takes off.
            const SoftJawBoring b = softJawBoring(doc, owner);
            strategies::TurnGeometry jaws{b.after, b.before};
            strategies::TurnRoughParams params =
                strategies::turnSoftJawsParams(op.parameters, preset, b.bore);
            // Down to the bore's floor, not the jaws' back.
            geom2d::Aabb2 box;
            for (const auto& r : b.before) {
                box.extend(r.bounds());
            }
            params.zRange = strategies::TurnZRange{box.max.x(), b.floor};
            tp = strategies::turnRough(jaws, params, tctx);
            tp.warnings.insert(tp.warnings.begin(),
                               fmt::format("bores the soft jaws to {:.3f} mm, {:.3f} mm deep, "
                                           "clamping {:.3f} mm: load that first",
                                           b.diameter, b.depth, b.clampDiameter));
        } else if (op.type == "turn_face") {
            tp = strategies::turnFace(tg, strategies::turnFaceParams(op.parameters, preset), tctx);
        } else if (op.type == "turn_rough") {
            tp =
                strategies::turnRough(tg, strategies::turnRoughParams(op.parameters, preset), tctx);
        } else if (op.type == "turn_finish") {
            tp = strategies::turnFinish(tg, strategies::turnFinishParams(op.parameters, preset),
                                        tctx);
        } else if (op.type == "turn_groove") {
            tp = strategies::turnGroove(tg, strategies::turnGrooveParams(op.parameters, preset),
                                        tctx);
        } else if (op.type == "turn_thread") {
            tp = strategies::turnThread(tg, strategies::turnThreadParams(op.parameters, preset),
                                        tctx);
        } else if (op.type == "turn_partoff") {
            tp = strategies::turnPartOff(tg, strategies::turnPartOffParams(op.parameters, preset),
                                         tctx);
        } else {
            throw userError(fmt::format("unknown operation type '{}'", op.type));
        }
        tp.warnings.insert(tp.warnings.begin(), lg.warnings.begin(), lg.warnings.end());
        // In the control, the compensation goes by the tip orientation on its offset page.
        if (tp.compensation != toolpath::Compensation::None &&
            tool->insert.orientation != tp.tipQuadrant) {
            throw userError(fmt::format(
                "nose compensation in the control needs tip orientation {} for this cut, and T{} "
                "has {}: set it in the tool, and on the machine's offset page",
                tp.tipQuadrant, tool->number, tool->insert.orientation));
        }
        if (const tooling::Holder* holder =
                tool->holderId ? project.tools.findHolder(*tool->holderId) : nullptr;
            holder && holder->iso) {
            if (holder->iso->boringBar !=
                (tooling::turningSideOf(tp.tipQuadrant) == tooling::TurningSide::Inside)) {
                throw userError(fmt::format(
                    "T{} sits in {}, {}: use another holder, or set the operation's Side",
                    tool->number, holder->name,
                    holder->iso->boringBar ? "a boring bar, which cuts inside"
                                           : "a shank holder, which cuts outside"));
            }
        }
        if (const tooling::Holder* holder =
                tool->holderId ? project.tools.findHolder(*tool->holderId) : nullptr;
            holder && holder->stick &&
            holder->stick->bar !=
                (tooling::turningSideOf(tp.tipQuadrant) == tooling::TurningSide::Inside)) {
            throw userError(fmt::format(
                "T{} sits in {}, {}: use another holder, or set the operation's Side", tool->number,
                holder->name,
                holder->stick->bar ? "a bar, which cuts inside" : "a shank, which cuts outside"));
        }
        // The hand says which way a tool cuts (DL-0050): passes the other way drag its back edge
        // through the work.
        if (tool->type == tooling::ToolType::TurningInsert) {
            const bool away = tooling::turningAwayFromChuck(tp.tipQuadrant);
            const tooling::Holder* fitted =
                tool->holderId ? project.tools.findHolder(*tool->holderId) : nullptr;
            const std::string hand =
                fitted && fitted->iso ? std::string(1, fitted->iso->hand) : tool->insert.hand;
            if (hand == "L" && !away) {
                throw userError(fmt::format(
                    "T{} is left-hand, which cuts away from the chuck: set the operation's "
                    "Direction, or use a right-hand or neutral tool",
                    tool->number));
            }
            if (hand == "R" && away) {
                throw userError(fmt::format(
                    "T{} is right-hand, which cuts towards the chuck: set the operation's "
                    "Direction, or use a left-hand or neutral tool",
                    tool->number));
            }
        }
        if (!bodies.empty() && op.type != "turn_soft_jaws") { // that one cuts the jaws
            const auto hits =
                sim::checkTurningFixtures(tp, latheCutter(*tool, tp), bodies, 1, &cancel);
            if (!hits.empty()) {
                const sim::FixtureCollision& hit = hits.front();
                throw userError(fmt::format(
                    "the insert comes within {:.3f} mm of '{}' at Z{:.3f} X{:.3f} (diameter): "
                    "keep the operation clear of the chuck (its back limit or geometry), grip the "
                    "bar shorter or lower the setup's fixture clearance",
                    setup.fixtureClearance, hit.body, hit.position.z(), 2.0 * hit.position.x()));
            }
        }
        if (tool->solid) {
            // A solid bar's own limits (DL-0046): nothing is scaled down to fit them.
            if (tooling::turningSideOf(tp.tipQuadrant) != tooling::TurningSide::Inside) {
                throw userError(fmt::format("T{} is a solid boring bar, which cuts inside: set "
                                            "the operation's Side to Inside",
                                            tool->number));
            }
            if (op.type == "turn_rough" && tool->solid->maxDepth > 0.0) {
                const double depth = strategies::turnRoughParams(op.parameters, preset).depthOfCut;
                if (depth > tool->solid->maxDepth + 1e-9) {
                    throw userError(fmt::format(
                        "the depth of cut {:.3f} mm is more than T{}'s maximum {:.3f} mm: lower it "
                        "in the cutting preset or the operation's speeds and feeds",
                        depth, tool->number, tool->solid->maxDepth));
                }
            }
            const double reach = tool->solid->maxOverhang > 0.0 ? tool->solid->maxOverhang
                                                                : tool->solid->overallLength;
            double deepest = sb.max.x();
            for (const toolpath::Move& m : tp.moves) {
                deepest = std::min(deepest, m.end.z());
            }
            if (sb.max.x() - deepest > reach + 1e-9) {
                throw userError(fmt::format(
                    "T{} would reach {:.3f} mm in from the stock's front, more than its {:.3f} mm "
                    "overhang: use a longer bar, or machine from both sides",
                    tool->number, sb.max.x() - deepest, reach));
            }
        }
        // The holder beyond the insert keeps clear of the chuck and the part (DL-0045).
        const tooling::Holder* bodyHolder =
            tool->holderId ? project.tools.findHolder(*tool->holderId) : nullptr;
        const auto bodySide = tooling::turningSideOf(tp.tipQuadrant);
        std::vector<sim::LatheBody> keepClear = bodies;
        keepClear.push_back({"the part", lg.part});
        for (const geom2d::Polyline& piece : tooling::turningBody(
                 *tool, bodyHolder, bodySide, tooling::turningAwayFromChuck(tp.tipQuadrant))) {
            const auto hits = sim::checkTurningFixtures(tp, piece, keepClear, 1, &cancel, true);
            if (!hits.empty()) {
                const sim::FixtureCollision& hit = hits.front();
                throw userError(fmt::format(
                    "the holder of T{} ({}) reaches '{}' at Z{:.3f} X{:.3f} (diameter): use a "
                    "smaller holder or bar, or keep the operation clear of it",
                    tool->number, bodyHolder ? bodyHolder->name : tool->name, hit.body,
                    hit.position.z(), 2.0 * hit.position.x()));
            }
        }
    } else if (op.type == "zlevel_rough" || op.type == "parallel3d" || op.type == "waterline3d" ||
               op.type == "contour3d" || op.type == "flat3d" || op.type == "pencil3d" ||
               op.type == "steep_shallow3d") {
        const geom3d::Mesh part = partMeshInWcs(doc, setup, 0.005);
        const geom3d::Aabb3 pb = part.bounds();
        const strategies::StrategyInfo& info = *strategies::findStrategy(op.type);
        const strategies::ParamReader reader(info, op.parameters);
        std::optional<StockBefore> restStock;
        const bool rest = restRequested(op);
        if (rest) {
            ProgressRange simRange(progress, 0.0, 0.5);
            restStock = stockBeforeOperation(doc, owner, op.id, reader.number("rest_resolution"),
                                             cancel, simRange, operationFrame(op));
        }
        ProgressRange cutRange(progress, rest ? 0.5 : 0.0, 1.0);
        const Context cutCtx{ctx.tool,           ctx.feeds, ctx.heights,
                             ctx.chordTolerance, cancel,    cutRange};
        const double r = tool->mill.diameter / 2.0;
        if (op.type == "zlevel_rough") {
            strategies::ZLevelRoughParams zp =
                strategies::zLevelRoughParams(op.parameters, *tool, preset);
            if (restStock) {
                const double minThickness = reader.number("rest_min_thickness");
                const sim::Stock* st = &restStock->stock;
                zp.material = [st, minThickness](double z, double zAbove) {
                    return sim::materialRegions(*st, z + minThickness, zAbove, minThickness);
                };
                // Within the stock's resolution plus the clearing's largest wall inset (a
                // cell and a half, 0.225 mm) of the allowance the stock is the earlier tool's
                // allowance, not rest: traced, it had the small tool run every wall again.
                zp.materialSlack = st->grid(2).spacing() + 0.225;
            }
            const double bottom = hv.bottom.value_or(std::max(stock.bottom, pb.min.z()));
            tp = strategies::zLevelRough(part, stock.outline, hv.top.value_or(stock.top), bottom,
                                         zp, cutCtx);
        } else if (op.type == "parallel3d") {
            strategies::ParallelParams pp =
                strategies::parallelParams(op.parameters, *tool, preset);
            if (restStock) {
                // 3D rest: only where the in-process stock stands above the part.
                const double minThickness = reader.number("rest_min_thickness");
                const sim::Comparison cmp =
                    sim::compareToPart(restStock->stock, part, {minThickness});
                const geom2d::Regions remaining =
                    sim::excessRegions(restStock->stock, cmp, minThickness);
                pp.boundary =
                    geom2d::offset(remaining, r + std::max(0.1, 0.25 * r), {ctx.chordTolerance});
            }
            if (!op.geometry.empty()) {
                // Contained to the selected faces: tool centres whose tool reaches them.
                const SelectedSurface surface = selectedSurface(doc, setup, op, ctx.chordTolerance);
                const geom2d::Regions reach = geom2d::offset(
                    surface.plan, r + std::max(0.05, 0.05 * r), {ctx.chordTolerance});
                pp.boundary =
                    rest ? geom2d::intersect(pp.boundary, reach, {ctx.chordTolerance}) : reach;
            }
            if ((rest || !op.geometry.empty()) && pp.boundary.empty()) {
                tp = strategies::beginToolpath(cutCtx, "parallel3d", pb.center().head<2>());
                tp.warnings.push_back(rest ? "no remaining stock to finish"
                                           : "the selected faces leave nothing to finish");
            } else {
                entryTop = pb.max.z();
                tp = strategies::parallelFinish(part, pb.min.z(), pp, cutCtx);
            }
        } else if (op.type == "flat3d") {
            strategies::FlatFinishParams fp =
                strategies::flatFinishParams(op.parameters, *tool, preset);
            fp.stockOutline = stock.outline;
            if (!op.geometry.empty()) {
                fp.only = selectedSurface(doc, setup, op, ctx.chordTolerance).plan;
            }
            tp = strategies::flatFinish(part, hv.top.value_or(pb.max.z()),
                                        hv.bottom.value_or(pb.min.z()), fp, cutCtx);
        } else if (op.type == "pencil3d" || op.type == "steep_shallow3d") {
            // Contained to the selected faces (steep and shallow by default over their heights
            // only) and, with rest machining, to where the in-process stock stands above the part.
            double top = pb.max.z();
            double bottom = pb.min.z();
            std::optional<geom2d::Regions> boundary;
            const geom2d::ClipOptions clip{ctx.chordTolerance};
            if (!op.geometry.empty()) {
                const SelectedSurface surface = selectedSurface(doc, setup, op, ctx.chordTolerance);
                boundary = geom2d::offset(surface.plan, r + std::max(0.05, 0.05 * r), clip);
                if (!surface.mesh.isEmpty()) {
                    top = surface.mesh.bounds().max.z();
                    bottom = surface.mesh.bounds().min.z();
                }
            }
            if (restStock) {
                const double minThickness = reader.number("rest_min_thickness");
                const sim::Comparison cmp =
                    sim::compareToPart(restStock->stock, part, {minThickness});
                const geom2d::Regions remaining =
                    geom2d::offset(sim::excessRegions(restStock->stock, cmp, minThickness),
                                   r + std::max(0.1, 0.25 * r), clip);
                boundary = boundary ? geom2d::intersect(*boundary, remaining, clip) : remaining;
            }
            entryTop = pb.max.z();
            if (boundary && boundary->empty()) {
                tp = strategies::beginToolpath(cutCtx, op.type, pb.center().head<2>());
                tp.warnings.push_back(rest ? "no remaining stock to finish"
                                           : "the selected faces leave nothing to finish");
            } else if (op.type == "pencil3d") {
                strategies::PencilParams pp =
                    strategies::pencilParams(op.parameters, *tool, preset);
                if (boundary) {
                    pp.boundary = *boundary;
                }
                tp = strategies::pencilFinish(part, pp, cutCtx);
            } else {
                strategies::SteepShallowParams sp =
                    strategies::steepShallowParams(op.parameters, *tool, preset);
                sp.topZ = hv.top.value_or(top);
                sp.bottomZ = hv.bottom.value_or(bottom);
                if (boundary) {
                    sp.boundary = *boundary;
                }
                tp = strategies::steepShallowFinish(part, sp, cutCtx);
            }
        } else if (op.type == "contour3d") {
            strategies::ContourFinishParams cp = strategies::contourFinishParams(op.parameters);
            double top = pb.max.z();
            double bottom = pb.min.z();
            std::optional<geom2d::Regions> boundary;
            const geom2d::ClipOptions clip{ctx.chordTolerance};
            if (!op.geometry.empty()) {
                // Contained to the selected faces, and by default over their heights only.
                const SelectedSurface surface = selectedSurface(doc, setup, op, ctx.chordTolerance);
                boundary = geom2d::offset(surface.plan, r + std::max(0.05, 0.05 * r), clip);
                if (!surface.mesh.isEmpty()) {
                    top = surface.mesh.bounds().max.z();
                    bottom = surface.mesh.bounds().min.z();
                }
            }
            if (restStock) {
                // Rest: only where the in-process stock stands above the part.
                const double minThickness = reader.number("rest_min_thickness");
                const sim::Comparison cmp =
                    sim::compareToPart(restStock->stock, part, {minThickness});
                const geom2d::Regions remaining =
                    geom2d::offset(sim::excessRegions(restStock->stock, cmp, minThickness),
                                   r + std::max(0.1, 0.25 * r), clip);
                boundary = boundary ? geom2d::intersect(*boundary, remaining, clip) : remaining;
            }
            cp.topZ = hv.top.value_or(top);
            cp.bottomZ = hv.bottom.value_or(bottom);
            if (boundary && boundary->empty()) {
                tp = strategies::beginToolpath(cutCtx, "contour3d", pb.center().head<2>());
                tp.warnings.push_back(rest ? "no remaining stock to finish"
                                           : "the selected faces leave nothing to finish");
            } else {
                if (boundary) {
                    cp.boundary = *boundary;
                }
                tp = strategies::contourFinish(part, cp, cutCtx);
            }
        } else {
            strategies::WaterlineParams wp = strategies::waterlineParams(op.parameters, preset);
            wp.topZ = hv.top.value_or(pb.max.z());
            wp.bottomZ = hv.bottom.value_or(pb.min.z());
            if (!op.geometry.empty()) {
                const SelectedSurface surface = selectedSurface(doc, setup, op, ctx.chordTolerance);
                wp.boundary = geom2d::offset(surface.plan, r + std::max(0.05, 0.05 * r),
                                             {ctx.chordTolerance});
                if (!surface.mesh.isEmpty()) {
                    wp.topZ = hv.top.value_or(surface.mesh.bounds().max.z());
                    wp.bottomZ = hv.bottom.value_or(surface.mesh.bounds().min.z());
                }
            }
            tp = strategies::waterline(part, wp, cutCtx);
        }
        if (restStock) {
            tp.warnings.insert(tp.warnings.begin(), restStock->warnings.begin(),
                               restStock->warnings.end());
        }
    } else if (op.type == "chamfer2d") {
        // One pass per height (highest first) and side, in one toolpath.
        const auto outlines = contourOutlines(doc, setup, op, ctx.chordTolerance, true);
        strategies::ChamferParams cp = strategies::chamferParams(op.parameters);
        const strategies::ContourSide operationSide = cp.side;
        entryTop = outlines.front().outline.z;
        for (std::size_t k = 0; k < outlines.size(); ++k) {
            cp.side = sideOf(outlines[k].side, operationSide);
            appendPass(
                tp,
                strategies::chamfer(outlines[k].outline.regions, outlines[k].outline.z, cp, ctx),
                k == 0);
        }
    } else if (op.type == "trace2d") {
        const auto chains = traceChains(doc, setup, op, ctx.chordTolerance);
        strategies::TraceParams tp2 = strategies::traceParams(op.parameters, preset);
        if (op.startPoint) {
            const ModelEntry* entry = project.findModel(op.startPoint->modelId);
            if (!entry) {
                throw userError("the start point refers to a model that is no longer there");
            }
            tp2.start = (modelToWcs(*entry, setup) * op.startPoint->signature.centroid).head<2>();
        }
        // Every chain's way down to its first level is fed below the feed height: stock may
        // still stand above the edges (the lowest chain's entry sets where that applies).
        entryTop = std::numeric_limits<double>::infinity();
        for (const auto& c : chains) {
            entryTop = std::min(entryTop, c.z);
        }
        if (strategies::traceFollowsSurface(op.parameters)) {
            // The depth below the part under the tool, by drop-cutter on its mesh.
            const geom3d::Mesh part = partMeshInWcs(doc, setup, 0.005);
            if (part.isEmpty()) {
                throw userError("following the surface needs the setup's model");
            }
            const strategies::DropCutter dc(part, strategies::Cutter(*tool));
            const double floor = part.bounds().min.z() - 1.0;
            tp = strategies::trace(chains, tp2, ctx,
                                   [&dc, floor](const Vec2& p) { return dc.drop(p, floor); });
            entryTop = part.bounds().min.z();
        } else {
            tp = strategies::trace(chains, tp2, ctx);
        }
    } else if (op.type == "drill" || op.type == "tap" || op.type == "threadmill" ||
               op.type == "bore") {
        const double d = tool->mill.diameter;
        const double pitch = tool->mill.threadPitch;
        std::function<bool(double)> matches;
        std::string noMatch;
        strategies::ThreadMillParams threadParams;
        if (op.type == "drill") {
            matches = [d](double hole) {
                return std::abs(hole - d) <= 0.05;
            };
            noMatch =
                fmt::format("no holes of Ø{:g} found; select hole faces to drill other sizes", d);
        } else if (op.type == "tap") {
            // Tap drill holes: about one pitch smaller than the tap.
            matches = [d, pitch](double hole) {
                return pitch > 0.0 && hole >= d - 1.5 * pitch && hole <= d - 0.5 * pitch;
            };
            noMatch =
                fmt::format("no tap drill holes for this Ø{:g} tap found; select hole faces", d);
        } else if (op.type == "threadmill") {
            threadParams = strategies::threadMillParams(op.parameters);
            const double major = threadParams.majorDiameter;
            const double p = threadParams.pitch > 0.0 ? threadParams.pitch : pitch;
            if (threadParams.external) {
                // A stud turned to the major diameter, within what a turned size varies by.
                matches = [major](double boss) {
                    return major > 0.0 && std::abs(boss - major) <= 0.2;
                };
                noMatch = "select the boss faces to thread (or set the major diameter)";
            } else {
                matches = [major, p](double hole) {
                    return major > 0.0 && p > 0.0 && hole >= major - 1.5 * p &&
                           hole <= major - 0.5 * p;
                };
                noMatch = "select the hole faces to thread (or set the major diameter)";
            }
        } else {
            const bool sized = tool->type == tooling::ToolType::Reamer ||
                               tool->type == tooling::ToolType::BoringBar;
            matches = [d, sized](double hole) {
                return sized ? std::abs(hole - d) <= 0.05 : hole > d + 0.05;
            };
            noMatch = sized ? fmt::format("no holes of Ø{:g} found; select hole faces", d)
                            : fmt::format("no holes larger than Ø{:g} found; select hole faces", d);
        }
        std::vector<std::string> warnings;
        std::optional<LatheHole> lathe;
        const bool live = onLathe && liveDrive(op);
        std::vector<strategies::HoleTarget> holes;
        // On a lathe the depths are from the part's front, not from a hole's top.
        const auto holesAt = [&](double depth) {
            if (lathe) {
                holes = {lathe->target(depth)};
                return;
            }
            // A live tool with nothing to find and nothing selected: on the axis, as the spindle's
            // would be.
            const bool mayGoOnAxis = live && !liveAround && op.geometry.empty() && !op.orientation;
            std::vector<std::string> found;
            holes = selectedHoles(doc, setup, op, matches, noMatch, found,
                                  op.type == "threadmill" && threadParams.external);
            if (holes.empty() && mayGoOnAxis) {
                lathe.emplace(doc, owner, op, *tool, cancel);
                holes = {lathe->target(depth)};
                return;
            }
            warnings.insert(warnings.end(), found.begin(), found.end());
            for (strategies::HoleTarget& h : holes) {
                if (depth > 0.0) {
                    h.depth = depth;
                    h.through = false;
                }
            }
        };
        if (onLathe && !live) {
            if (op.orientation) {
                throw userError("the spindle drills along its own axis: for a tool orientation of "
                                "its own, set the operation's Tool drive to Live tool");
            }
            lathe.emplace(doc, owner, op, *tool, cancel);
        }
        if (live) {
            const machine::Machine& onMachine = machineFor(project, setup);
            if (!onMachine.liveTooling) {
                throw userError(fmt::format(
                    "machine '{}' has no live tooling: set the operation's Tool drive to Spindle, "
                    "or choose a machine with it in the setup",
                    onMachine.name));
            }
        }
        // A feed height replaces the R plane above each hole.
        if (op.type == "drill") {
            strategies::DrillParams dp = strategies::drillParams(op.parameters);
            if (lathe && dp.cycle == toolpath::CycleType::Tap) {
                throw userError("a tap cycle on a lathe needs the Tap operation and a tap");
            }
            holesAt(dp.holeDepth);
            if (lathe && lathe->tipAtBottom()) {
                dp.depthMode = strategies::DepthMode::Tip;
            }
            dp.rPlaneZ = hv.feed;
            dp.retractClearance = hv.holeFeed.value_or(dp.retractClearance);
            tp = strategies::drill(holes, dp, ctx);
        } else if (op.type == "tap") {
            strategies::TapParams tpp = strategies::tapParams(op.parameters);
            if (lathe) {
                holesAt(tpp.threadDepth);
                tpp.threadDepth = 0.0;
            } else {
                holesAt(0.0);
            }
            tpp.rPlaneZ = hv.feed;
            tpp.retractClearance = hv.holeFeed.value_or(tpp.retractClearance);
            tp = strategies::tap(holes, tpp, ctx);
        } else if (op.type == "threadmill") {
            holesAt(0.0);
            threadParams.rPlaneZ = hv.feed;
            threadParams.retractClearance = hv.holeFeed.value_or(threadParams.retractClearance);
            tp = strategies::threadMill(holes, threadParams, ctx);
        } else {
            strategies::BoreParams bp = strategies::boreParams(op.parameters);
            holesAt(bp.holeDepth);
            bp.rPlaneZ = hv.feed;
            bp.retractClearance = hv.holeFeed.value_or(bp.retractClearance);
            tp = strategies::bore(holes, bp, ctx);
        }
        tp.warnings.insert(tp.warnings.begin(), warnings.begin(), warnings.end());
        if (lathe) {
            lathe->check(tp);
        }
        if (live && !liveAround) {
            checkLiveHoles(tp, machineFor(project, setup), op);
            tp.live = true;
            if (op.type == "tap") {
                checkTapHoles(doc, setup, op, *tool, tp, cancel);
            }
        }
    } else if (op.type == "backbore" || op.type == "edgebreak3d") {
        // The far side of through holes, and edges of any shape (DL-0083).
        if (onLathe) {
            throw userError(fmt::format("{} needs a milling machine", op.type == "backbore"
                                                                          ? "back boring"
                                                                          : "3D edge breaking"));
        }
        if (op.type == "backbore") {
            strategies::BackBoreParams bp =
                strategies::backBoreParams(op.parameters, *tool, preset);
            const bool oriented = bp.method == strategies::BackBoreMethod::Oriented;
            const double head = tool->mill.diameter / 2.0;
            const double neck = strategies::backBoreNeckRadius(*tool);
            std::vector<std::string> found;
            const auto holes = selectedHoles(
                doc, setup, op,
                [&](double d) {
                    return oriented ? d / 2.0 > neck && d / 2.0 < head : d / 2.0 > head;
                },
                fmt::format("no holes that T{} passes found; select hole faces", tool->number),
                found);
            bp.rPlaneZ = hv.feed;
            bp.retractClearance = hv.holeFeed.value_or(bp.retractClearance);
            tp = strategies::backBore(holes, bp, ctx);
            tp.warnings.insert(tp.warnings.begin(), found.begin(), found.end());
        } else {
            const strategies::ParamReader reader(*strategies::findStrategy(op.type), op.parameters);
            const BreakEdgesInput in =
                breakEdgesInput(doc, setup, op, reader.number("point_spacing"), 0.005);
            tp = strategies::edgeBreak(in.edges, in.faces,
                                       strategies::edgeBreakParams(op.parameters), ctx);
        }
    } else if (op.type == "rotary_pocket" && onLathe) {
        tp = wrappedOnLathe(doc, setup, op, stock, hv,
                            strategies::rotaryPocketParams(op.parameters, *tool, preset), ctx,
                            cancel);
    } else if (op.type == "rotary_pocket") {
        const geom3d::Mesh part = partMeshInWcs(doc, setup, 0.005);
        const geom3d::Aabb3 pb = part.bounds();
        tp = strategies::rotaryPocket(
            part, rotaryStockRadius(doc, setup, part), pb.min.x(), pb.max.x(),
            strategies::rotaryPocketParams(op.parameters, *tool, preset), ctx);
    } else if (op.type == "rotary_contour") {
        tp = wrappedContour(doc, setup, op, stock, hv, onLathe,
                            strategies::rotaryContourParams(op.parameters, *tool, preset), ctx,
                            cancel);
    } else if (op.type == "multiaxis_finish") {
        const geom3d::Mesh part = partMeshInWcs(doc, setup, 0.005);
        tp = strategies::multiAxisFinish(
            part, part.bounds().min.z(),
            strategies::multiAxisFinishParams(op.parameters, *tool, preset), ctx);
    } else if (op.type == "swarf5x") {
        const strategies::SwarfParams sp = strategies::swarfParams(op.parameters);
        const auto [walls, floor] = swarfGeometry(doc, setup, op, sp, *tool, ctx.chordTolerance);
        tp = strategies::swarf(partMeshInWcs(doc, setup, 0.005), walls, floor, sp, ctx);
    } else {
        throw userError(fmt::format("unknown operation type '{}'", op.type));
    }
    if (liveAround && isWrapped(op.type)) {
        tp.live = true;
        tp.liveAxes = toolpath::LiveAxes::Wrap;
    } else if (liveAround) {
        const strategies::ParamReader reader(*strategies::findStrategy(op.type), op.parameters);
        const bool polar = reader.choice("live_axes") == "polar";
        tp.live = true;
        tp.liveAxes = polar ? toolpath::LiveAxes::Polar : toolpath::LiveAxes::HeldC;
        tp.liveC = polar ? 0.0 : reader.number("live_c");
        checkLiveMilling(tp, machineFor(project, setup), op);
    }
    if (hv.feed && !tp.isTurning() && !isWrapped(op.type) && op.type != "multiaxis_finish" &&
        op.type != "swarf5x") {
        feedFromFeedHeight(tp, *hv.feed, entryTop, ctx.feeds.plunge);
    }
    // Smoothing (DL-0032): on, runs of short moves become arcs and lines within the tolerance;
    // off, the strategy's own arcs become short lines.
    if (const strategies::StrategyInfo* info = strategies::findStrategy(op.type);
        info && info->find("smoothing")) {
        const strategies::ParamReader reader(*info, op.parameters);
        if (reader.flag("smoothing")) {
            toolpath::fitArcs(tp, reader.number("smoothing_tolerance"));
        } else {
            toolpath::linearizeArcs(tp, ctx.chordTolerance);
        }
    }
    // Fixtures (DL-0034) and the workholding (DL-0035): the tool, its shank and its holder keep
    // clear of them.
    if (hasFixtures(setup)) {
        const std::string what = setup.fixtures.empty() ? "workholding" : "fixtures";
        if (const auto pose = setupWorkholdingPose(doc, owner)) {
            tp.warnings.insert(tp.warnings.end(), pose->warnings.begin(), pose->warnings.end());
        }
        if (tp.isTurning() || (onLathe && isHoleOperation(op.type))) {
            // Kept clear above, as the areas they sweep turning with the spindle.
        } else if (toolpath::isMultiAxis(tp) || isWrapped(op.type)) {
            tp.warnings.push_back("multi-axis toolpaths do not avoid the " + what +
                                  ": check them in the simulation");
        } else {
            const tooling::Holder* holder =
                tool->holderId ? project.tools.findHolder(*tool->holderId) : nullptr;
            // With cutter compensation the path is the contour: the tool centre is a radius off.
            const double clearance =
                setup.fixtureClearance +
                (tp.compensation != toolpath::Compensation::None ? tool->mill.diameter / 2.0 : 0.0);
            // Placed on the setup, moved into the operation's frame.
            const strategies::ObstacleField field(fixtureMeshInWcs(doc, owner, operationFrame(op)),
                                                  *tool, holder, clearance);
            strategies::AvoidanceParams ap;
            ap.retract = hv.retract;
            ap.plungeFeed = ctx.feeds.plunge;
            ap.chordTolerance = ctx.chordTolerance;
            ap.clearance = setup.fixtureClearance;
            ap.name = what;
            strategies::avoidObstacles(tp, field, ap, cancel);
        }
    }
    // Feeds are output exactly as set: a feed move whose feed is zero would stop the machine
    // (or alarm), so the toolpath is refused and the value to set is named.
    for (const toolpath::Move& m : tp.moves) {
        const bool fed = m.kind == toolpath::MoveKind::Linear || m.isArc();
        const double rate = m.kind == toolpath::MoveKind::Cycle
                                ? tp.cycles.at(static_cast<std::size_t>(m.cycle)).feedRate
                                : m.feedRate;
        if ((fed || m.kind == toolpath::MoveKind::Cycle) && rate <= 0.0) {
            const bool cycle = m.kind == toolpath::MoveKind::Cycle;
            const bool plunge = m.feedClass == toolpath::FeedClass::Plunge && !cycle;
            const bool ramp = m.feedClass == toolpath::FeedClass::Ramp && !cycle;
            const bool lead = (m.feedClass == toolpath::FeedClass::LeadIn ||
                               m.feedClass == toolpath::FeedClass::LeadOut) &&
                              !cycle;
            throw userError(fmt::format(
                "the toolpath {} but the {} of the cutting preset '{}' of T{} is 0: set it in the "
                "preset or in the operation's speeds and feeds",
                plunge ? "plunges"
                : ramp ? "ramps in"
                : lead ? "leads on or off the wall"
                       : "cuts",
                plunge ? "plunge feed"
                : ramp ? "ramp feed"
                : lead ? "lead feed"
                       : "feed rate",
                preset->name, tool->number));
        }
    }
    if (tp.spindleSpeed <= 0.0 && !tp.isTurning()) {
        throw userError(fmt::format("the spindle speed of T{} is 0: set the cutting preset's "
                                    "cutting speed",
                                    tool->number));
    }
    tp.warnings.insert(tp.warnings.begin(), hv.warnings.begin(), hv.warnings.end());
    tp.operationId = op.id;
    return tp;
}

} // namespace

Setup operationSetup(const Setup& setup, const Operation& op) {
    if (!op.orientation) {
        return setup;
    }
    Setup local = setup;
    const Transform3 t = setup.wcs.toParent() * operationFrame(op);
    local.wcs.origin = t.translation();
    local.wcs.xAxis = t.linear().col(0);
    local.wcs.zAxis = t.linear().col(2);
    return local;
}

toolpath::Toolpath generateToolpathInFrame(const Document& doc, const Setup& setup,
                                           const Operation& op, CancelToken& cancel,
                                           ProgressSink& progress) {
    if (!op.orientation) {
        return generateIn(doc, setup, setup, op, cancel, progress);
    }
    if (op.type.rfind("turn_", 0) == 0 || op.type == "bar_pull" || op.type == "spindle_transfer") {
        throw userError("turning operations cannot have a tool orientation of their own");
    }
    try {
        (void)op.orientation->frame.normalized();
    } catch (const Error&) {
        throw userError("the operation's tool orientation is invalid: its X axis must cross "
                        "its Z axis");
    }
    if (op.type != "multiaxis_finish" && op.type != "swarf5x" && !isWrapped(op.type) &&
        !machineFor(doc.project(), setup).isLathe()) {
        // Refused here already when the machine cannot index to it (throws).
        (void)operationIndex(setup, op, machineFor(doc.project(), setup));
    }
    return generateIn(doc, setup, operationSetup(setup, op), op, cancel, progress);
}

toolpath::Toolpath toolpathInWcs(const toolpath::Toolpath& inFrame, const Operation& op) {
    if (!op.orientation) {
        return inFrame;
    }
    return toolpath::transformed(inFrame, operationFrame(op));
}

toolpath::Toolpath latheCutPath(const Document& doc, const Setup& setup, const Operation& op,
                                const toolpath::Toolpath& path, CancelToken& cancel,
                                ProgressSink& progress) {
    if (path.compensation == toolpath::Compensation::None) {
        return path;
    }
    Operation asCut = op;
    asCut.parameters["compensation"] = "computer";
    return generateToolpath(doc, setup, asCut, cancel, progress);
}

toolpath::Toolpath generateToolpath(const Document& doc, const Setup& setup, const Operation& op,
                                    CancelToken& cancel, ProgressSink& progress) {
    return toolpathInWcs(generateToolpathInFrame(doc, setup, op, cancel, progress), op);
}

OperationHeightValues operationHeights(const Document& doc, const Setup& setup,
                                       const Operation& op) {
    return heightsIn(doc, operationSetup(setup, op), op);
}

std::vector<ChamferZone> chamferZones(const Document& doc, const Setup& setup,
                                      const Operation& op) {
    if ((op.type != "chamfer2d" && op.type != "trace2d") || op.orientation || !op.toolId) {
        return {};
    }
    const tooling::Tool* tool = doc.project().tools.findTool(*op.toolId);
    if (!tool) {
        return {};
    }
    if (op.type == "trace2d") {
        try {
            if (strategies::traceFollowsSurface(op.parameters) ||
                tooling::isTurningTool(tool->type)) {
                return {};
            }
            const strategies::ParamReader r(*strategies::findStrategy("trace2d"), op.parameters);
            const double radius = tool->mill.diameter / 2.0;
            // How far from its edges the tool reaches, either side (what the trace removes
            // elsewhere is still counted).
            double reach = radius;
            if (r.choice("side") != "on") {
                const int passes = r.integer("passes");
                reach = 2.0 * radius + std::max(0.0, r.number("offset")) +
                        (passes > 1 ? (passes - 1) * r.number("pass_spacing") : 0.0);
            }
            const double depth = r.number("depth");
            std::vector<ChamferZone> zones;
            for (const strategies::TraceChain& c : traceChains(doc, setup, op, 1e-3)) {
                std::vector<Vec2> pts = c.path.discretize(1e-3);
                if (c.path.isClosed() && !pts.empty()) {
                    pts.push_back(pts.front());
                }
                ChamferZone zone;
                zone.z = c.z;
                zone.depth = depth;
                zone.band = geom2d::sweep({pts}, reach, {1e-3});
                zones.push_back(std::move(zone));
            }
            return zones;
        } catch (const Error&) {
            return {}; // not generated either: nothing it cut
        }
    }
    try {
        const strategies::ChamferParams cp = strategies::chamferParams(op.parameters);
        const double depth = strategies::chamferGeometry(*tool, cp.width, cp.tipOffset).depth;
        std::vector<ChamferZone> zones;
        for (const ContourOutline& o : contourOutlines(doc, setup, op, 1e-3, true)) {
            zones.push_back({o.outline.regions, o.outline.z, cp.width, depth});
        }
        return zones;
    } catch (const Error&) {
        return {}; // not generated either: nothing it cut
    }
}

namespace {

OperationHeightValues heightsIn(const Document& doc, const Setup& setup, const Operation& op) {
    const Project& project = doc.project();
    const StockInWcs stock = stockInWcs(doc, setup);
    // The part's extent along the WCS Z (from its bounding box).
    auto partRange = [&]() -> std::pair<double, double> {
        const ModelEntry* entry = setup.modelId ? project.findModel(*setup.modelId) : nullptr;
        const cad::Model* model = entry ? doc.model(entry->id) : nullptr;
        if (!model) {
            throw userError("the setup has no model for a height measured from the part");
        }
        const geom3d::Aabb3 local = model->info().bounds;
        const Transform3 toWcs = modelToWcs(*entry, setup);
        double lo = std::numeric_limits<double>::infinity();
        double hi = -lo;
        for (int i = 0; i < 8; ++i) {
            const Vec3 corner((i & 1) ? local.max.x() : local.min.x(),
                              (i & 2) ? local.max.y() : local.min.y(),
                              (i & 4) ? local.max.z() : local.min.z());
            const double z = (toWcs * corner).z();
            lo = std::min(lo, z);
            hi = std::max(hi, z);
        }
        return {lo, hi};
    };
    // The selected faces' centres along the WCS Z.
    auto geometryRange = [&]() -> std::pair<double, double> {
        double lo = std::numeric_limits<double>::infinity();
        double hi = -lo;
        for (const ShapeRef& ref : op.geometry) {
            if (const ModelEntry* entry = project.findModel(ref.modelId)) {
                const double z = (modelToWcs(*entry, setup) * ref.signature.centroid).z();
                lo = std::min(lo, z);
                hi = std::max(hi, z);
            }
        }
        if (lo > hi) {
            throw userError("select the faces or edges to machine");
        }
        return {lo, hi};
    };

    OperationHeightValues out;
    const OperationHeights h = effectiveHeights(op);
    auto value = [&](const HeightSetting& s, bool top, const char* name) -> double {
        double base = 0.0;
        switch (s.from) {
        case HeightRef::StockTop:
            base = stock.top;
            break;
        case HeightRef::StockBottom:
            base = stock.bottom;
            break;
        case HeightRef::PartTop:
            base = partRange().second;
            break;
        case HeightRef::PartBottom:
            base = partRange().first;
            break;
        case HeightRef::Geometry:
            base = top ? geometryRange().second : geometryRange().first;
            break;
        case HeightRef::Origin:
            base = 0.0;
            break;
        case HeightRef::Clearance:
            base = out.clearance;
            break;
        case HeightRef::Retract:
            base = out.retract;
            break;
        case HeightRef::TopHeight:
            base = out.top.value_or(stock.top);
            break;
        case HeightRef::Picked: {
            const ModelEntry* entry = s.shape ? project.findModel(s.shape->modelId) : nullptr;
            if (!entry) {
                throw userError(fmt::format("pick a face or edge for the {} height", name));
            }
            base = (modelToWcs(*entry, setup) * s.shape->signature.centroid).z();
            break;
        }
        case HeightRef::HoleTop:
        case HeightRef::Default:
            throw userError(fmt::format("internal: the {} height has no single value", name));
        }
        return base + s.offset;
    };
    auto refUsable = [](const HeightSetting& s, std::initializer_list<HeightRef> allowed,
                        const char* name) {
        if (std::find(allowed.begin(), allowed.end(), s.from) == allowed.end()) {
            throw userError(fmt::format("the {} height cannot be measured from the {}", name,
                                        toString(s.from)));
        }
    };
    using R = HeightRef;
    const bool holes = isHoleOperation(op.type);
    refUsable(h.clearance, {R::Retract, R::StockTop, R::PartTop, R::Origin, R::Picked},
              "clearance");
    refUsable(h.retract,
              {R::Clearance, R::StockTop, R::PartTop, R::TopHeight, R::Origin, R::Picked},
              "retract");
    if (holes) {
        refUsable(
            h.feed,
            {R::HoleTop, R::Retract, R::TopHeight, R::StockTop, R::PartTop, R::Origin, R::Picked},
            "feed");
    } else {
        refUsable(h.feed, {R::Retract, R::TopHeight, R::StockTop, R::PartTop, R::Origin, R::Picked},
                  "feed");
    }
    if (h.clearance.from == R::Retract && h.retract.from == R::Clearance) {
        throw userError("the clearance and retract heights are measured from each other");
    }

    // Top and bottom first (the retract height stays above the top), then retract and
    // clearance in the order they refer to each other, then feed.
    if (usesTopAndBottom(op.type)) {
        refUsable(h.top, {R::StockTop, R::PartTop, R::Geometry, R::Origin, R::Picked}, "top");
        refUsable(h.bottom,
                  {R::Geometry, R::PartBottom, R::StockBottom, R::TopHeight, R::PartTop, R::Origin,
                   R::Picked},
                  "bottom");
        out.top = value(h.top, true, "top");
        out.bottom = value(h.bottom, false, "bottom");
        if (*out.bottom > *out.top) {
            throw userError("the bottom height is above the top height");
        }
    }
    const double top = out.top.value_or(stock.top);
    auto resolveRetract = [&] {
        out.retract = value(h.retract, true, "retract");
        if (out.retract < top + 0.5) {
            out.warnings.push_back(
                fmt::format("retract height {:.3f} is below the top {:.3f}; raised to {:.3f}",
                            out.retract, top, top + 1.0));
            out.retract = top + 1.0;
        }
    };
    if (h.retract.from == R::Clearance) {
        out.clearance = value(h.clearance, true, "clearance");
        resolveRetract();
    } else {
        resolveRetract();
        out.clearance = value(h.clearance, true, "clearance");
    }
    if (out.clearance < out.retract) {
        out.warnings.push_back(
            fmt::format("clearance height {:.3f} is below the retract height; raised to {:.3f}",
                        out.clearance, out.retract));
        out.clearance = out.retract;
    }
    if (h.feed.from == R::HoleTop) {
        out.holeFeed = h.feed.offset;
    } else {
        out.feed = value(h.feed, true, "feed");
        if (*out.feed > out.retract) {
            out.warnings.push_back(
                fmt::format("feed height {:.3f} is above the retract height; lowered to {:.3f}",
                            *out.feed, out.retract));
            out.feed = out.retract;
        }
    }
    return out;
}

} // namespace

std::string toolpathInputKey(const Document& doc, const Setup& setup, const Operation& op) {
    Sha256 h;
    const Project& p = doc.project();
    Project subset;
    subset.id = p.id;
    subset.tools.id = p.tools.id; // value types generate fresh ids by default: keep the key stable
    subset.models = p.models;
    subset.setups = {setup};
    subset.setups.front().operations = {op};
    // The workholding's bodies (by their files' hashes), datum and range, and its jaw set.
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
    if (setup.tailstock && setup.tailstock->centre) {
        if (const LatheFitting* centre = p.findFitting(*setup.tailstock->centre)) {
            subset.fittings = {*centre};
        }
    }
    // Rest machining depends on every earlier operation of the setup (and their tools). So do
    // holes on a lathe; turning there depends on the earlier holes (latheStockBefore).
    const bool rest = restRequested(op);
    // Asked for what is shown as well: without a machine, generating is what refuses.
    const machine::Machine* onMachine = findMachineFor(p, setup);
    const bool lathe = onMachine && onMachine->isLathe();
    if (rest || lathe) {
        std::vector<Operation> ops;
        for (const Operation& other : setup.operations) {
            if (other.id == op.id) {
                break;
            }
            if (rest || isHoleOperation(op.type) || isHoleOperation(other.type) ||
                strategies::turnHasZRange(op.type, op.parameters) || op.type == "bar_pull" ||
                other.type == "bar_pull") {
                ops.push_back(other);
            }
        }
        ops.push_back(op);
        subset.setups.front().operations = std::move(ops);
    }
    for (const Operation& o : subset.setups.front().operations) {
        if (o.toolId) {
            if (const auto* tool = p.tools.findTool(*o.toolId)) {
                if (!subset.tools.findTool(tool->id)) {
                    subset.tools.tools.push_back(*tool);
                }
                // Holders matter to the simulation (rest) and to fixture avoidance.
                if ((rest || hasFixtures(setup)) && tool->holderId) {
                    if (const auto* holder = p.tools.findHolder(*tool->holderId)) {
                        if (!subset.tools.findHolder(holder->id)) {
                            subset.tools.holders.push_back(*holder);
                        }
                    }
                }
            }
        }
    }
    // Stock from earlier setups depends on them too (the whole chain); rest machining and lathes
    // also on what their operations cut (their tools and holders).
    const std::vector<const Setup*> sources = stockSources(p, setup);
    subset.setups.insert(subset.setups.begin(), sources.size(), Setup{});
    for (std::size_t k = 0; k < sources.size(); ++k) {
        subset.setups[k] = *sources[k];
        if (!rest && !lathe) {
            continue;
        }
        for (const Operation& o : sources[k]->operations) {
            const tooling::Tool* tool = o.toolId ? p.tools.findTool(*o.toolId) : nullptr;
            if (!tool) {
                continue;
            }
            if (!subset.tools.findTool(tool->id)) {
                subset.tools.tools.push_back(*tool);
            }
            if (tool->holderId) {
                if (const auto* holder = p.tools.findHolder(*tool->holderId);
                    holder && !subset.tools.findHolder(holder->id)) {
                    subset.tools.holders.push_back(*holder);
                }
            }
        }
    }
    // Tool orientations are checked against the machine's rotary axes (built-in machines change
    // only with the program version) with the work where it sits on the machine; other
    // toolpaths do not depend on the placement.
    const auto& ops = subset.setups.back().operations;
    if (std::any_of(ops.begin(), ops.end(), [](const Operation& o) { return o.orientation; })) {
        if (const machine::Machine* m = p.findMachine(setup.machine)) {
            subset.machines.push_back(*m);
        }
    } else {
        subset.setups.back().placement.reset();
    }
    if (op.type == "spindle_transfer") {
        // Where it takes the part depends on both setups, their stock and the machine (ADR-0007).
        Setup& own = subset.setups.back();
        own.placement = setup.placement;
        own.operations = setup.operations; // a part-off must follow it
        if (const Setup* target = transferTarget(p, setup)) {
            subset.setups.push_back(*target);
        }
        if (const machine::Machine* m = findMachineFor(p, setup)) {
            subset.machines.push_back(*m);
        }
    }
    h.update(fmt::format("opencam {} toolpaths r{}|", kVersionString, kToolpathRevision));
    h.update(dumpJson(toJson(subset)));
    return Sha256::toHex(h.finish());
}

std::shared_ptr<const toolpath::Toolpath> cachedToolpath(const Document& doc, const Setup& setup,
                                                         const Operation& op, CancelToken& cancel,
                                                         ProgressSink& progress) {
    auto inFrame = cachedToolpathInFrame(doc, setup, op, cancel, progress);
    if (!op.orientation) {
        return inFrame;
    }
    return std::make_shared<const toolpath::Toolpath>(toolpathInWcs(*inFrame, op));
}

std::shared_ptr<const toolpath::Toolpath>
cachedToolpathInFrame(const Document& doc, const Setup& setup, const Operation& op,
                      CancelToken& cancel, ProgressSink& progress) {
    (void)machineFor(doc.project(), setup); // a toolpath kept from before is refused as well
    const std::string key = toolpathInputKey(doc, setup, op);
    ToolpathCache& cache = doc.toolpathCache();
    // Generated once: another thread generating the same toolpath is waited for.
    if (auto path = cache.findOrBegin(op.id, key, cancel)) {
        progress.report(1.0, "Cached");
        return path;
    }
    std::shared_ptr<const toolpath::Toolpath> path;
    try {
        path = std::make_shared<const toolpath::Toolpath>(
            generateToolpathInFrame(doc, setup, op, cancel, progress));
    } catch (...) {
        cache.finish(op.id, key, nullptr);
        throw;
    }
    cache.finish(op.id, key, path);
    return path;
}

} // namespace ocam::document
