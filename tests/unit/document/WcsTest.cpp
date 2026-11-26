// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A setup's WCS from the setup page's choices (DL-0038): the part orientation, the origin from a
// box or a picked point with an offset, kept there by Document::modify; box stock by size.

#include "FixtureSpecs.hpp"

#include <ocam/cad/SubShape.hpp>
#include <ocam/document/Document.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/ProjectIo.hpp>
#include <ocam/document/Wcs.hpp>
#include <ocam/document/Workholding.hpp>
#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Progress.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <filesystem>

using namespace ocam;
using namespace ocam::document;
using Catch::Matchers::ContainsSubstring;

namespace {

const std::filesystem::path kFixtures(OCAM_FIXTURE_DIR);

bool near(const Vec3& a, const Vec3& b, double tol = 1e-6) {
    return (a - b).norm() < tol;
}

AxisPick pickAlong(const Vec3& direction, bool flip = false) {
    return AxisPick{"test", std::nullopt, direction.normalized(), flip};
}

/// A model's first sub-shape of `type` whose exact geometry satisfies `wanted`.
template <typename Fn>
ShapeRef shapeOn(const Document& doc, const Uuid& model, cad::SubShapeType type, Fn&& wanted) {
    const cad::Model& m = *doc.model(model);
    for (int i = 1; i <= cad::subShapeCount(m, type); ++i) {
        if (wanted(cad::subShapeGeometry(m, type, i))) {
            return {model, type, i, cad::signatureOf(m, type, i)};
        }
    }
    FAIL("no such sub-shape");
    return {};
}

} // namespace

TEST_CASE("part orientation: the part's own axes, or X (or Y) and Z from picks",
          "[document][wcs]") {
    Frame current;
    current.origin = Vec3(1, 2, 3);
    current.zAxis = Vec3::UnitY();
    current.xAxis = Vec3::UnitX();
    WcsOrientation o;
    Frame f = orientedAxes(o, current);
    CHECK(near(f.zAxis, Vec3::UnitZ()));
    CHECK(near(f.xAxis, Vec3::UnitX()));
    CHECK(near(f.origin, current.origin));

    // X and Z: Z as picked, X made square to it; flips reverse each.
    o.mode = OrientationMode::XZ;
    o.z = pickAlong(-Vec3::UnitZ());
    o.first = pickAlong(Vec3(1, 0, 0.3));
    f = orientedAxes(o, current);
    CHECK(near(f.zAxis, -Vec3::UnitZ()));
    CHECK(near(f.xAxis, Vec3::UnitX()));
    o.z->flip = true;
    o.first->flip = true;
    f = orientedAxes(o, current);
    CHECK(near(f.zAxis, Vec3::UnitZ()));
    CHECK(near(f.xAxis, -Vec3::UnitX()));

    // Y and Z: Y as picked (square to Z), X = Y x Z.
    o = {};
    o.mode = OrientationMode::YZ;
    o.z = pickAlong(Vec3::UnitX());
    o.first = pickAlong(Vec3::UnitZ());
    f = orientedAxes(o, current);
    CHECK(near(f.zAxis, Vec3::UnitX()));
    CHECK(near(f.yAxis(), Vec3::UnitZ()));
    CHECK(near(f.xAxis, Vec3::UnitY()));

    // Only Z picked so far: X stays as far as it can.
    o = {};
    o.mode = OrientationMode::XZ;
    o.z = pickAlong(Vec3::UnitZ());
    f = orientedAxes(o, current);
    CHECK(near(f.xAxis, Vec3::UnitX()));

    // Parallel picks are refused.
    o.first = pickAlong(Vec3::UnitZ());
    CHECK_THROWS_WITH(orientedAxes(o, current), ContainsSubstring("parallel"));
}

TEST_CASE("the WCS origin from the stock, the part or a picked point, with an offset",
          "[document][wcs]") {
    using namespace fixtures::block_pocket;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid model = doc.importModel(kFixtures / "block_pocket.step", cancel, progress);
    const Uuid id = doc.addSetup(model);
    auto setup = [&]() -> const Setup& {
        return *doc.project().findSetup(id);
    };

    // A new setup: the part's own axes, the origin at the stock box's top front left corner.
    REQUIRE(setup().originFrom);
    REQUIRE(setup().orientation);
    CHECK(near(setup().wcs.origin, Vec3(-1, -1, kHeight + 1)));
    // Kept there: a larger stock moves it.
    doc.modify("wider", [&](Project& p) { p.findSetup(id)->stock.offsetSide = 5.0; });
    CHECK(near(setup().wcs.origin, Vec3(-5, -5, kHeight + 1)));

    // The part box's top centre, then offset along the WCS axes.
    doc.modify("part box", [&](Project& p) {
        WcsOrigin o;
        o.from = OriginFrom::PartBox;
        o.box = {1, 1, 2};
        p.findSetup(id)->originFrom = o;
    });
    CHECK(near(setup().wcs.origin, Vec3(kLength / 2, kWidth / 2, kHeight)));
    doc.modify("offset", [&](Project& p) { p.findSetup(id)->originFrom->offset = Vec3(1, 2, 3); });
    CHECK(near(setup().wcs.origin, Vec3(kLength / 2 + 1, kWidth / 2 + 2, kHeight + 3)));

    // Turned over (Z down, X along the part's X): the box point and the offset along the new
    // axes: the top centre is now the part's bottom, the offset's Z goes down.
    doc.modify("turn over", [&](Project& p) {
        Setup& s = *p.findSetup(id);
        WcsOrientation o;
        o.mode = OrientationMode::XZ;
        o.z = pickAlong(-Vec3::UnitZ());
        o.first = pickAlong(Vec3::UnitX());
        const Frame f = orientedAxes(o, s.wcs);
        s.wcs.xAxis = f.xAxis;
        s.wcs.zAxis = f.zAxis;
        s.orientation = o;
    });
    CHECK(near(setup().wcs.origin, Vec3(kLength / 2 + 1, kWidth / 2 - 2, -3)));

    // A picked point: the pocket's floor (a face's centre), a corner (a vertex).
    doc.modify("upright", [&](Project& p) {
        Setup& s = *p.findSetup(id);
        s.orientation = WcsOrientation{};
        s.wcs.xAxis = Vec3::UnitX();
        s.wcs.zAxis = Vec3::UnitZ();
    });
    const ShapeRef floor = shapeOn(doc, model, cad::SubShapeType::Face, [&](const auto& g) {
        return g.kind == cad::SubShapeGeometry::Kind::Plane &&
               g.direction.isApprox(Vec3::UnitZ()) &&
               std::abs(g.point.z() - (kHeight - kPocketDepth)) < 1e-6;
    });
    doc.modify("floor", [&](Project& p) {
        WcsOrigin o;
        o.from = OriginFrom::Point;
        o.point = floor;
        p.findSetup(id)->originFrom = o;
    });
    CHECK(near(
        setup().wcs.origin,
        Vec3(kPocketX + kPocketLength / 2, kPocketY + kPocketWidth / 2, kHeight - kPocketDepth),
        1e-3));
    const ShapeRef corner = shapeOn(doc, model, cad::SubShapeType::Vertex, [&](const auto& g) {
        return near(g.point, Vec3(kLength, kWidth, kHeight));
    });
    doc.modify("corner", [&](Project& p) { p.findSetup(id)->originFrom->point = corner; });
    CHECK(near(setup().wcs.origin, Vec3(kLength, kWidth, kHeight)));

    // Saved and read back.
    const Project back = projectFromJson(toJson(doc.project()));
    CHECK(back.findSetup(id)->originFrom == setup().originFrom);
    CHECK(back.findSetup(id)->orientation == setup().orientation);

    // A point origin with no point is reported; no fixture to measure from leaves it in place.
    Project wrong = doc.project();
    wrong.findSetup(id)->originFrom->point.reset();
    const auto issues = wrong.validate();
    CHECK(std::any_of(issues.begin(), issues.end(), [](const std::string& s) {
        return s.find("pick the point") != std::string::npos;
    }));
    const Vec3 before = setup().wcs.origin;
    doc.modify("fixture box",
               [&](Project& p) { p.findSetup(id)->originFrom->from = OriginFrom::FixtureBox; });
    CHECK(near(setup().wcs.origin, before));
    CHECK_THROWS_WITH(resolveWcsOrigin(doc, setup()), ContainsSubstring("no fixtures"));
}

TEST_CASE("X along a picked straight edge, Z from a picked face", "[document][wcs]") {
    using namespace fixtures::block_pocket;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid model = doc.importModel(kFixtures / "block_pocket.step", cancel, progress);
    const Uuid id = doc.addSetup(model);
    // The block's top face, and its top edge along Y at X 0.
    const ShapeRef top = shapeOn(doc, model, cad::SubShapeType::Face, [&](const auto& g) {
        return g.kind == cad::SubShapeGeometry::Kind::Plane &&
               g.direction.isApprox(Vec3::UnitZ()) && std::abs(g.point.z() - kHeight) < 1e-6;
    });
    const ShapeRef edge = shapeOn(doc, model, cad::SubShapeType::Edge, [&](const auto& g) {
        return g.kind == cad::SubShapeGeometry::Kind::Line &&
               std::abs(std::abs(g.direction.y()) - 1.0) < 1e-9 && std::abs(g.point.x()) < 1e-6 &&
               std::abs(g.point.z() - kHeight) < 1e-6;
    });
    const Vec3 along = axisOfShape(doc, edge);
    CHECK(std::abs(along.y()) > 1.0 - 1e-9); // the edge's own direction
    doc.modify("X along the edge", [&](Project& p) {
        Setup& s = *p.findSetup(id);
        WcsOrientation o;
        o.mode = OrientationMode::XZ;
        o.z = AxisPick{"top", top, axisOfShape(doc, top), false};
        o.first = AxisPick{"edge", edge, along, false};
        const Frame f = orientedAxes(o, s.wcs);
        s.wcs.xAxis = f.xAxis;
        s.wcs.zAxis = f.zAxis;
        s.orientation = o;
    });
    const Frame wcs = doc.project().findSetup(id)->wcs.normalized();
    CHECK(near(wcs.zAxis, Vec3::UnitZ()));
    CHECK(near(wcs.xAxis, along)); // parallel to the edge
}

TEST_CASE("a circle's centre, and the WCS origin at the bottom centre of the vise",
          "[document][wcs]") {
    CancelToken cancel;
    NullProgress progress;
    SECTION("circle") {
        using namespace fixtures::hole_plate;
        Document doc;
        const Uuid model = doc.importModel(kFixtures / "hole_plate.step", cancel, progress);
        const Uuid id = doc.addSetup(model);
        const ShapeRef circle = shapeOn(doc, model, cad::SubShapeType::Edge, [&](const auto& g) {
            return g.kind == cad::SubShapeGeometry::Kind::Circle &&
                   std::abs(g.radius - kThroughDiameter / 2) < 1e-6 &&
                   std::abs(g.point.z() - kThickness) < 1e-6;
        });
        doc.modify("hole", [&](Project& p) {
            WcsOrigin o;
            o.from = OriginFrom::Point;
            o.point = circle;
            p.findSetup(id)->originFrom = o;
        });
        CHECK(
            near(doc.project().findSetup(id)->wcs.origin, Vec3(kThroughX, kThroughY, kThickness)));
    }
    SECTION("vise") {
        // vise_origin_project: the origin at the bottom centre of the vise's box.
        Document doc = Document::open(kFixtures / "vise_origin_project.ocam");
        const Uuid id = doc.project().setups.front().id;
        auto setup = [&]() -> const Setup& {
            return *doc.project().findSetup(id);
        };
        REQUIRE(setup().originFrom);
        CHECK(setup().originFrom->from == OriginFrom::FixtureBox);
        auto bottomCentre = [&] {
            const geom3d::Aabb3 box = originBox(doc, setup(), OriginFrom::FixtureBox);
            const Frame f = setup().wcs.normalized();
            return Vec3(box.center().x() * f.xAxis + box.center().y() * f.yAxis() +
                        box.min.z() * f.zAxis);
        };
        CHECK(near(setup().wcs.origin, bottomCentre()));
        // The vise moves around the part with a wider stock: the origin follows it, and the
        // vise's mount (its bottom centre) stays at WCS zero.
        const Vec3 before = setup().wcs.origin;
        doc.modify("wider stock", [&](Project& p) { p.findSetup(id)->stock.offsetSide += 5.0; });
        CHECK_FALSE(near(setup().wcs.origin, before, 1e-3));
        CHECK(near(setup().wcs.origin, bottomCentre()));
        CHECK(near(setupWorkholdingPose(doc, setup())->mount, Vec3::Zero(), 1e-6));

        // A point picked on the fixed jaw's top: the origin there.
        const WorkholdingTemplate w = doc.project().workholdings.front();
        const WorkholdingPart& jaw = w.parts[1];
        const cad::Model& jawModel = *doc.modelData(jaw.sha256);
        int top = 0;
        for (int f = 1; f <= cad::subShapeCount(jawModel, cad::SubShapeType::Face); ++f) {
            const auto g = cad::subShapeGeometry(jawModel, cad::SubShapeType::Face, f);
            if (g.kind == cad::SubShapeGeometry::Kind::Plane &&
                g.direction.isApprox(Vec3::UnitZ())) {
                top = f;
            }
        }
        REQUIRE(top > 0);
        const WorkholdingDatum picked =
            pickedDatum("WCS pick", jaw, jawModel, cad::SubShapeType::Face, top);
        doc.modify("on the jaw", [&](Project& p) {
            WcsOrigin o;
            o.from = OriginFrom::Point;
            o.workholdingPoint = picked;
            p.findSetup(id)->originFrom = o;
        });
        const WorkholdingPose pose = *setupWorkholdingPose(doc, setup());
        CHECK(near(pose.templateToWcs * picked.point, Vec3::Zero(), 1e-6));
        const Project back = projectFromJson(toJson(doc.project()));
        CHECK(back.findSetup(id)->originFrom == setup().originFrom);

        // X from an edge of the vise across the current X: the WCS turns a quarter, the vise
        // turns back (its rotation), so it stays where it is around the part.
        auto partsInProject = [&] {
            std::vector<Transform3> out;
            const auto held = setupWorkholdingPose(doc, setup());
            for (const PosedPart& part : held->parts) {
                out.push_back(setup().wcs.toParent() * part.toWcs);
            }
            return out;
        };
        const std::vector<Transform3> unturned = partsInProject();
        const Frame wcs = setup().wcs.normalized();
        std::optional<Vec3> across;
        for (int e = 1; e <= cad::subShapeCount(jawModel, cad::SubShapeType::Edge) && !across;
             ++e) {
            if (cad::subShapeGeometry(jawModel, cad::SubShapeType::Edge, e).kind !=
                cad::SubShapeGeometry::Kind::Line) {
                continue;
            }
            const Vec3 d =
                workholdingPoint(doc, setup(),
                                 pickedDatum("e", jaw, jawModel, cad::SubShapeType::Edge, e))
                    .direction;
            if (std::abs(d.dot(wcs.xAxis)) < 1e-9 && std::abs(d.dot(wcs.zAxis)) < 1e-9) {
                across = d;
            }
        }
        REQUIRE(across);
        const int rotation = setup().workholding->rotation;
        doc.modify("X along the vise", [&](Project& p) {
            WcsOrientation o;
            o.mode = OrientationMode::XZ;
            o.z = AxisPick{"top", std::nullopt, wcs.zAxis, false};
            o.first = AxisPick{"jaw edge", std::nullopt, *across, false};
            setOrientation(*p.findSetup(id), o, true);
        });
        CHECK(near(setup().wcs.normalized().xAxis, *across));
        CHECK(setup().workholding->rotation != rotation);
        const std::vector<Transform3> after = partsInProject();
        REQUIRE(after.size() == unturned.size());
        for (std::size_t i = 0; i < after.size(); ++i) {
            CHECK(after[i].isApprox(unturned[i], 1e-9));
        }
        // Z on the vise is refused: it stands on WCS Z.
        Setup tilted = setup();
        WcsOrientation o = *tilted.orientation;
        o.z = AxisPick{"jaw edge", std::nullopt, *across, false};
        o.first = AxisPick{"top", std::nullopt, wcs.zAxis, false};
        CHECK_THROWS_WITH(setOrientation(tilted, o, true), ContainsSubstring("stands on"));
    }
}

TEST_CASE("box stock by size, centred on the part, nudged", "[document][wcs][stock]") {
    using namespace fixtures::block_pocket;
    CancelToken cancel;
    NullProgress progress;
    Document doc;
    const Uuid model = doc.importModel(kFixtures / "block_pocket.step", cancel, progress);
    const Uuid id = doc.addSetup(model);
    doc.modify("box", [&](Project& p) {
        Stock& st = p.findSetup(id)->stock;
        st.kind = StockKind::Box;
        st.size = StockSize::Fixed;
        st.boxSize = Vec3(110, 70, 40);
        st.shift = Vec3::Zero();
    });
    geom3d::Aabb3 b = doc.stockShape(id)->bounds();
    CHECK(near(b.min, Vec3(-5, -5, -5)));
    CHECK(near(b.max, Vec3(105, 65, 35)));
    // Nudged: more stock on top, the part's bottom 1 mm up from the stock's.
    doc.modify("nudge", [&](Project& p) { p.findSetup(id)->stock.shift = Vec3(2, 0, 4); });
    b = doc.stockShape(id)->bounds();
    CHECK(near(b.min, Vec3(-3, -5, -1)));
    CHECK(near(b.max, Vec3(107, 65, 39)));
    // The WCS origin (the stock box's top front left) follows it.
    CHECK(near(doc.project().findSetup(id)->wcs.origin, Vec3(-3, -5, 39)));
    // Saved and read back; a box of an earlier version (corners) still resolves as it was.
    const Project back = projectFromJson(toJson(doc.project()));
    CHECK(back.findSetup(id)->stock == doc.project().findSetup(id)->stock);
    doc.modify("corners", [&](Project& p) {
        Stock& st = p.findSetup(id)->stock;
        st.boxCorners = {Vec3(0, 0, 0), Vec3(10, 20, 30)};
    });
    b = doc.stockShape(id)->bounds();
    CHECK(near(b.max, Vec3(10, 20, 30)));
    // A size that is zero on one axis is reported.
    Project wrong = doc.project();
    wrong.findSetup(id)->stock.boxCorners.reset();
    wrong.findSetup(id)->stock.boxSize = Vec3(10, 0, 10);
    CHECK_FALSE(wrong.validate().empty());
}

TEST_CASE("a WCS origin on the workholding written before origin_from opens",
          "[document][wcs][io]") {
    Document doc = Document::open(kFixtures / "vise_project.ocam");
    Json j = toJson(doc.project());
    Json& setup = j.at("setups").at(0);
    setup.erase("origin_from");
    WorkholdingDatum mount;
    mount.name = "WCS origin";
    mount.kind = DatumKind::Offset;
    mount.from = doc.project().workholdings.front().mount;
    setup["workholding"]["origin"] = toJson(mount);
    const Project p = projectFromJson(j);
    REQUIRE(p.setups.front().originFrom);
    CHECK(p.setups.front().originFrom->from == OriginFrom::Point);
    REQUIRE(p.setups.front().originFrom->workholdingPoint);
    CHECK(p.setups.front().originFrom->workholdingPoint->from == mount.from);
}
