// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Setup placements (DL-0037): the work snapped onto a machine's geometry, carried by the axis
// under it, and solve, the post and the simulation using where it sits.

#include "FixtureSpecs.hpp"
#include "TestProject.hpp"

#include <ocam/document/Generate.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/document/Workholding.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/machine/Model.hpp>
#include <ocam/post/Post.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <filesystem>

using namespace ocam;
using namespace ocam::document;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

bool near(const Vec3& a, const Vec3& b, double tolerance = 1e-6) {
    return (a - b).norm() < tolerance;
}

geom3d::Mesh solid(machine::Component::Shape shape, const Vec3& a, const Vec3& b, double radius = 0,
                   double length = 0) {
    machine::Component c;
    c.shape = shape;
    c.min = a;
    c.max = b;
    c.base = a;
    c.axis = b;
    c.radius = radius;
    c.length = length;
    return machine::componentMesh(c);
}

/// The generic A/C trunnion with a zero-point plate 50 mm tall on its round table (on C).
machine::Machine trunnionWithPlate() {
    machine::Machine m = machine::genericMill5TableAC();
    m.id = "trunnion-plate";
    m.name = "Trunnion with a zero-point plate";
    m.model = machine::defaultModel(m);
    machine::ModelPart plate =
        machine::makePart("zero-point plate", solid(machine::Component::Shape::Box,
                                                    Vec3(-60, -60, 0), Vec3(60, 60, 50)));
    plate.carrier = "C";
    m.model.parts.push_back(plate);
    return m;
}

/// The generic 4th-axis mill with a faceplate on A: its face 180 mm left of machine zero.
machine::Machine fourthAxisWithFaceplate() {
    machine::Machine m = machine::genericMill4A();
    m.id = "fourth-axis-faceplate";
    m.name = "4th axis with a faceplate";
    m.model = machine::defaultModel(m);
    machine::ModelPart faceplate =
        machine::makePart("faceplate", solid(machine::Component::Shape::Cylinder, Vec3(-200, 0, 0),
                                             Vec3::UnitX(), 80.0, 20.0));
    faceplate.carrier = "A";
    m.model.parts.push_back(faceplate);
    machine::ModelPart table =
        machine::makePart("x table", solid(machine::Component::Shape::Box, Vec3(-400, -150, -150),
                                           Vec3(300, 150, -120)));
    table.carrier = "X";
    m.model.parts.push_back(table);
    return m;
}

const machine::ModelPart& partNamed(const machine::Machine& m, const std::string& name) {
    const auto it = std::find_if(m.model.parts.begin(), m.model.parts.end(),
                                 [&](const machine::ModelPart& p) { return p.name == name; });
    REQUIRE(it != m.model.parts.end());
    return *it;
}

/// block_pocket in its default setup (WCS at the stock's top corner (-1, -1, 31), the stock
/// 102 x 62 x 31 mm from z = 0), facing its top and, at an orientation of its own, its front.
struct Block {
    Document doc;
    Uuid setupId;
    CancelToken cancel;
    NullProgress progress;

    explicit Block(const machine::Machine& machine) {
        const tooling::Tool tool =
            testing::withPreset(tooling::makeFlatEndMill(1, 6.0, 15.0, 50.0));
        const Uuid modelId = doc.importModel(
            std::filesystem::path(OCAM_FIXTURE_DIR) / "block_pocket.step", cancel, progress);
        setupId = doc.addSetup(modelId);
        doc.modify("setup", [&](Project& p) {
            p.tools.tools.push_back(tool);
            p.machines.push_back(machine);
            Setup& s = *p.findSetup(setupId);
            s.machine = machine.id;
            for (const bool front : {false, true}) {
                Operation op;
                op.name = front ? "front" : "top";
                op.type = "facing";
                op.toolId = tool.id;
                op.presetId = tool.presets.front().id;
                if (front) {
                    op.orientation =
                        ToolOrientation{Frame{Vec3::Zero(), Vec3::UnitX(), -Vec3::UnitY()}, 0};
                }
                s.operations.push_back(op);
            }
        });
    }

    const Setup& setup() const { return *doc.project().findSetup(setupId); }
    const machine::Machine& machine() const { return machineFor(doc.project(), setup()); }

    void place(const SetupPlacement& placement) {
        doc.modify("place", [&](Project& p) { p.findSetup(setupId)->placement = placement; });
    }

    /// The lowest Z the front facing is posted at (relative to the setup's work offset).
    double frontLowest() {
        const PostOutput out =
            postSetup(doc, setupId, machine(), defaultPostFor(machine()), cancel, progress);
        INFO((out.errors.empty() ? std::string() : out.errors.front()));
        REQUIRE(out.ok());
        REQUIRE(out.toolpaths.size() == 2);
        double lowest = 1e9;
        for (const auto& mv : out.toolpaths[1].moves) {
            lowest = std::min(lowest, mv.end.z());
        }
        return lowest;
    }

    SetupSimulation simulate() {
        SimulationOptions options;
        options.resolution = 1.0;
        options.compare = false;
        return simulateSetup(doc, setupId, options, cancel, progress);
    }
};

/// The machine puts the tool tip on the work where the simulation shows it, at a move end.
void checkTipOnWork(const SetupSimulation& sim, const OperationSimulation& op) {
    REQUIRE(op.mount);
    REQUIRE_FALSE(op.toolpath.moves.empty());
    const machine::MachineModel model = machine::effectiveModel(*sim.machine);
    const std::size_t k = op.toolpath.moves.size() / 2;
    const Vec3 tip = op.toolpath.moves[k].end;
    const double gauge = sim::gaugeLength(*op.tool);
    const auto position = machine::positionFor(model, op.rotary.angles[k], op.mount->carrier,
                                               op.mount->wcs * tip, gauge);
    REQUIRE(position);
    const machine::ModelPose pose = machine::poseAt(model, *position);
    const Vec3 toolTip = pose.spindle.translation() - gauge * pose.spindle.linear().col(2);
    CHECK(near(toolTip, pose.carried(model, op.mount->carrier) * op.mount->wcs * tip, 1e-6));
}

} // namespace

TEST_CASE("a setup placed on a zero-point plate on the trunnion's table", "[document][placement]") {
    const machine::Machine m = trunnionWithPlate();
    REQUIRE(machine::validateModel(m).empty());
    Block b(m);
    // Not placed: the project origin at the table centre (where A and C meet), as before.
    CHECK(near(wcsInTable(b.setup(), b.machine()).translation(), Vec3(-1, -1, 31)));
    CHECK_THAT(b.frontLowest(), WithinAbs(-31.0, 1e-6));

    // The plate's top, snapped at its centre: carried by C. The stock's bottom centre goes there.
    const machine::MachineModel model = machine::effectiveModel(b.machine());
    const machine::ModelPart plate = partNamed(b.machine(), "zero-point plate");
    SetupPlacement p;
    p.target = machine::snapOnPart(model, plate, Vec3(17, -23, 50), machine::Snap::FaceCentre);
    CHECK(near(p.target, Vec3(0, 0, 50)));
    p.carrier = plate.carrier;
    p.workPoint = Vec3(50, 30, 0);
    b.place(p);
    CHECK(placementProblem(b.setup(), b.machine()).empty());
    // The WCS origin 81 mm above the table centre, 51 mm left, 31 mm in front.
    const machine::WorkMount mount = workMount(b.setup(), b.machine());
    CHECK(mount.carrier == "C");
    CHECK(near(mount.wcs.translation(), Vec3(-51, -31, 81)));
    CHECK(near(wcsInTable(b.setup(), b.machine()).translation(), Vec3(-51, -31, 81)));

    // The front, tilted up by A -90: its face (30 mm in front of the table centre) ends up 30 mm
    // above the centre, 51 mm below the setup's work offset measured at the WCS origin.
    CHECK_THAT(b.frontLowest(), WithinAbs(30.0 - 81.0, 1e-6));

    // The simulation carries the work on C, where the placement puts it.
    const SetupSimulation sim = b.simulate();
    REQUIRE(sim.operations.size() == 2);
    for (const auto& op : sim.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
        REQUIRE(op.mount);
        CHECK(op.mount->carrier == "C");
        CHECK(near(op.mount->wcs.translation(), Vec3(-51, -31, 81)));
        checkTipOnWork(sim, op);
    }
    CHECK_THAT(radToDeg(sim.operations[1].rotary.angles.front().a), WithinAbs(-90.0, 1e-6));

    // Turned 90 degrees on the plate (about machine Z, about the plate's centre).
    p.rotation = Vec3(0, 0, 90);
    b.place(p);
    const machine::WorkMount turned = workMount(b.setup(), b.machine());
    CHECK(near(turned.wcs.translation(), Vec3(31, -51, 81)));
    CHECK(near(turned.wcs * Vec3(1, 0, 0) - turned.wcs.translation(), Vec3(0, 1, 0)));
}

TEST_CASE("a setup placed on the A axis faceplate of a 4th axis", "[document][placement]") {
    const machine::Machine m = fourthAxisWithFaceplate();
    REQUIRE(machine::validateModel(m).empty());
    Block b(m);
    CHECK_THAT(b.frontLowest(), WithinAbs(-31.0, 1e-6));

    // The faceplate's face, where the A axis meets it: carried by A.
    const machine::MachineModel model = machine::effectiveModel(b.machine());
    const machine::ModelPart faceplate = partNamed(b.machine(), "faceplate");
    SetupPlacement p;
    p.target =
        machine::snapOnPart(model, faceplate, Vec3(-180, 30, -40), machine::Snap::AxisOnFace);
    CHECK(near(p.target, Vec3(-180, 0, 0), 1e-6));
    p.carrier = faceplate.carrier;
    // The stock's left end, centred on the axis, against the faceplate.
    p.workPoint = Vec3(-1, 30, 15.5);
    b.place(p);
    CHECK(placementProblem(b.setup(), b.machine()).empty());
    const machine::WorkMount mount = workMount(b.setup(), b.machine());
    CHECK(mount.carrier == "A");
    CHECK(near(mount.wcs.translation(), Vec3(-180, -31, 15.5)));
    // From the table centre on the A axis (its point nearest the WCS origin): 31 mm in front of
    // the axis, 15.5 mm above it.
    CHECK(near(wcsInTable(b.setup(), b.machine()).translation(), Vec3(0, -31, 15.5)));

    // A -90 turns the front up: 30 mm in front of the axis becomes 30 mm above it, 14.5 mm above
    // the WCS origin at A 0.
    CHECK_THAT(b.frontLowest(), WithinAbs(30.0 - 15.5, 1e-6));

    const SetupSimulation sim = b.simulate();
    REQUIRE(sim.operations.size() == 2);
    for (const auto& op : sim.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
        REQUIRE(op.mount);
        CHECK(op.mount->carrier == "A");
        checkTipOnWork(sim, op);
    }

    // A face of a part no rotary axis carries has no axis to snap to.
    CHECK_THROWS_WITH(machine::snapOnPart(model, partNamed(b.machine(), "x table"),
                                          Vec3(0, 0, -120), machine::Snap::AxisOnFace),
                      ContainsSubstring("no rotary axis carries 'x table'"));
    // The face's centre and the point clicked.
    CHECK(
        near(machine::snapOnPart(model, faceplate, Vec3(-180, 30, -40), machine::Snap::FaceCentre),
             Vec3(-180, 0, 0), 0.5));
    CHECK(near(machine::snapOnPart(model, faceplate, Vec3(-180, 30, -40), machine::Snap::Point),
               Vec3(-180, 30, -40)));
}

TEST_CASE("placements the machine cannot carry are refused, naming why", "[document][placement]") {
    Block b(fourthAxisWithFaceplate());
    SetupPlacement p;
    p.target = Vec3(0, 0, -120);

    SECTION("on the spindle's side") {
        p.carrier = "Z";
        b.place(p);
        CHECK_THAT(placementProblem(b.setup(), b.machine()),
                   ContainsSubstring("the Z axis carries the spindle"));
        const PostOutput out = postSetup(b.doc, b.setupId, b.machine(), defaultPostFor(b.machine()),
                                         b.cancel, b.progress);
        REQUIRE_FALSE(out.ok());
        CHECK_THAT(out.errors.front(), ContainsSubstring("placement on the machine"));
        const SetupSimulation sim = b.simulate();
        CHECK(std::any_of(sim.warnings.begin(), sim.warnings.end(), [](const std::string& w) {
            return w.find("the machine is not simulated") != std::string::npos;
        }));
        for (const auto& op : sim.operations) {
            CHECK_FALSE(op.mount);
        }
    }
    SECTION("on the base while X and Y move the table") {
        p.carrier.clear();
        b.place(p);
        CHECK_THAT(placementProblem(b.setup(), b.machine()),
                   ContainsSubstring("moves the table, but not the machine base"));
    }
    SECTION("on the X table beside the 4th axis: A must stay at 0") {
        p.carrier = "X";
        b.place(p);
        CHECK(placementProblem(b.setup(), b.machine()).empty());
        const PostOutput out = postSetup(b.doc, b.setupId, b.machine(), defaultPostFor(b.machine()),
                                         b.cancel, b.progress);
        REQUIRE_FALSE(out.ok());
        CHECK_THAT(out.errors.front(), ContainsSubstring("front") &&
                                           ContainsSubstring("turns the A axis") &&
                                           ContainsSubstring("does not sit on it"));
        // Without the front, the top facing posts with A at 0.
        b.doc.modify("top only",
                     [&](Project& pr) { pr.findSetup(b.setupId)->operations[1].enabled = false; });
        const PostOutput top = postSetup(b.doc, b.setupId, b.machine(), defaultPostFor(b.machine()),
                                         b.cancel, b.progress);
        INFO((top.errors.empty() ? std::string() : top.errors.front()));
        CHECK(top.ok());
        const SetupSimulation sim = b.simulate();
        REQUIRE(sim.operations.size() == 1);
        REQUIRE(sim.operations[0].mount);
        CHECK(sim.operations[0].mount->carrier == "X");
    }
}

TEST_CASE("workholding snapped onto the machine", "[document][placement]") {
    // The vise of vise_project.ocam stood on the zero-point plate: its base's bottom centre (a
    // point of the work, picked on the workholding) on the plate's top centre.
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "vise_project.ocam");
    const machine::Machine m = trunnionWithPlate();
    const Uuid setupId = doc.project().setups.front().id;
    doc.modify("machine", [&](Project& p) {
        p.machines.push_back(m);
        p.findSetup(setupId)->machine = m.id;
    });
    const Setup& setup = *doc.project().findSetup(setupId);
    const std::vector<sim::FixtureBody> bodies = fixtureBodiesInWcs(doc, setup);
    REQUIRE_FALSE(bodies.empty());
    const auto base =
        std::min_element(bodies.begin(), bodies.end(), [](const auto& x, const auto& y) {
            return x.mesh.bounds().min.z() < y.mesh.bounds().min.z();
        });
    machine::ModelPart body;
    body.name = base->name;
    body.geometry = std::make_shared<const geom3d::Mesh>(base->mesh);
    const geom3d::Aabb3 box = base->mesh.bounds();
    const Vec3 onBottom(box.center().x() + 3.0, box.center().y() - 4.0, box.min.z());
    const machine::MachineModel model = machine::effectiveModel(m);
    const Vec3 bottomCentre = machine::snapOnPart(model, body, onBottom, machine::Snap::FaceCentre);
    CHECK_THAT(bottomCentre.z(), WithinAbs(box.min.z(), 1e-6));

    SetupPlacement p;
    p.carrier = "C";
    p.target = machine::snapOnPart(model, partNamed(m, "zero-point plate"), Vec3(0, 10, 50),
                                   machine::Snap::FaceCentre);
    p.workPoint = setup.wcs.toParent() * bottomCentre;
    doc.modify("place", [&](Project& pr) { pr.findSetup(setupId)->placement = p; });
    const machine::WorkMount mount = workMount(*doc.project().findSetup(setupId), m);
    CHECK(near(mount.wcs * bottomCentre, Vec3(0, 0, 50)));

    // The work, vise and all, sits on the plate in the simulation.
    CancelToken cancel;
    NullProgress progress;
    SimulationOptions options;
    options.resolution = 1.0;
    options.compare = false;
    const SetupSimulation sim = simulateSetup(doc, setupId, options, cancel, progress);
    REQUIRE_FALSE(sim.operations.empty());
    for (const auto& op : sim.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
        REQUIRE(op.mount);
        CHECK(near(op.mount->wcs * bottomCentre, Vec3(0, 0, 50)));
    }
}

TEST_CASE("the workholding's mount datum snapped onto the machine", "[document][placement]") {
    // The vise placed by its mount datum (its base's bottom centre) on the zero-point plate:
    // the vise stays there whatever the stock, the stock, part and WCS follow (DL-0035).
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "vise_project.ocam");
    const machine::Machine m = trunnionWithPlate();
    const Uuid setupId = doc.project().setups.front().id;
    const machine::MachineModel model = machine::effectiveModel(m);
    const Vec3 plateTop = machine::snapOnPart(model, partNamed(m, "zero-point plate"),
                                              Vec3(0, 10, 50), machine::Snap::FaceCentre);
    doc.modify("machine", [&](Project& p) {
        p.machines.push_back(m);
        Setup& s = *p.findSetup(setupId);
        s.machine = m.id;
        SetupPlacement placement;
        placement.carrier = "C";
        placement.target = plateTop;
        placement.onWorkholding = true;
        s.placement = placement;
    });
    auto mountOfVise = [&] {
        const Setup& s = *doc.project().findSetup(setupId);
        const auto pose = setupWorkholdingPose(doc, s);
        REQUIRE(pose);
        const machine::WorkMount mount = workMount(withPlacedWorkholding(doc, s), m);
        return std::pair{mount.wcs * pose->mount, mount};
    };
    const auto [vise, mount] = mountOfVise();
    CHECK(near(vise, plateTop));
    // A wider stock: the vise's mount stays on the plate, the WCS moves.
    doc.modify("wider", [&](Project& p) { p.findSetup(setupId)->stock.offsetSide = 6.0; });
    const auto [vise2, mount2] = mountOfVise();
    CHECK(near(vise2, plateTop));
    CHECK_FALSE(near(mount2.wcs.translation(), mount.wcs.translation()));

    // The simulation and the post use the same mount.
    CancelToken cancel;
    NullProgress progress;
    SimulationOptions options;
    options.resolution = 1.0;
    options.compare = false;
    const SetupSimulation sim = simulateSetup(doc, setupId, options, cancel, progress);
    REQUIRE_FALSE(sim.operations.empty());
    for (const auto& op : sim.operations) {
        INFO(op.name << ": " << op.error);
        CHECK(op.error.empty());
        REQUIRE(op.mount);
        CHECK(op.mount->wcs.isApprox(mount2.wcs, 1e-9));
    }
    // Without a workholding the placement cannot sit on it.
    doc.modify("no vise", [&](Project& p) { p.findSetup(setupId)->workholding.reset(); });
    CHECK_THROWS_WITH(withPlacedWorkholding(doc, *doc.project().findSetup(setupId)),
                      ContainsSubstring("workholding"));
}
