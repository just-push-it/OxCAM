// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Machine parts stored as STEP (DL-0037): the exact geometry is the data, meshes are derived
// (StepBodies.hpp), and picking uses the B-rep.

#include "FixtureSpecs.hpp"

#include <ocam/cad/SubShape.hpp>
#include <ocam/document/Document.hpp>
#include <ocam/document/MachineImport.hpp>
#include <ocam/document/ProjectIo.hpp>
#include <ocam/document/StepBodies.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/machine/Model.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>

using namespace ocam;
using namespace ocam::document;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using Kind = cad::SubShapeGeometry::Kind;

namespace {

std::filesystem::path demoFile() {
    return std::filesystem::path(OCAM_FIXTURE_DIR) / "machines" / "trunnion-demo.json";
}

const machine::ModelPart& partNamed(const machine::Machine& m, const std::string& name) {
    const auto it = std::find_if(m.model.parts.begin(), m.model.parts.end(),
                                 [&](const machine::ModelPart& p) { return p.name == name; });
    REQUIRE(it != m.model.parts.end());
    return *it;
}

bool near(const Vec3& a, const Vec3& b, double tolerance = 1e-6) {
    return (a - b).norm() < tolerance;
}

/// The first sub-shape of `part` whose geometry passes `test`.
std::optional<cad::SubShapeGeometry>
find(const machine::ModelPart& part, cad::SubShapeType type,
     const std::function<bool(const cad::SubShapeGeometry&)>& test) {
    const cad::Model model = partModel(part);
    for (int i = 1; i <= cad::subShapeCount(model, type); ++i) {
        const cad::SubShapeGeometry g = cad::subShapeGeometry(model, type, i);
        if (test(g)) {
            return g;
        }
    }
    return std::nullopt;
}

double seconds(std::chrono::steady_clock::time_point since) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
}

} // namespace

TEST_CASE("machine parts keep their exact geometry as STEP files", "[document][machine][step]") {
    using namespace fixtures::trunnion_machine;
    const machine::Machine m = loadMachineFile(demoFile());
    REQUIRE(m.model.parts.size() == static_cast<std::size_t>(kBodies));
    const Json doc = parseJson(readFileBytes(demoFile()), "trunnion-demo.json");
    CHECK(doc.at("version") == machine::kMachineVersion);
    for (const machine::ModelPart& p : m.model.parts) {
        CAPTURE(p.name);
        CHECK(p.exact());
        CHECK(p.mesh.empty());
        CHECK(
            std::filesystem::is_regular_file(machine::meshFolder(demoFile()) / (p.step + ".step")));
        // The mesh is the STEP file's, at the machine's tolerance.
        REQUIRE(p.geometry);
        CHECK(p.geometry->isClosed());
        const cad::Model model = partModel(p);
        REQUIRE_FALSE(model.isNull());
        CHECK_THAT(p.geometry->signedVolume(), WithinRel(model.info().volume, 0.02));
    }
    CHECK(m.model.meshTolerance == 0.2);
}

TEST_CASE("meshes are made from the STEP files once, in memory and on disk",
          "[document][machine][step]") {
    using namespace fixtures::trunnion_machine;
    const auto cacheDir = std::filesystem::temp_directory_path() / "oxcam_test_step_meshes";
    std::filesystem::remove_all(cacheDir);
    clearStepCache();
    setStepMeshCacheFolder({});

    auto start = std::chrono::steady_clock::now();
    machine::Machine m = loadMachineFile(demoFile());
    const double cold = seconds(start);
    const machine::Machine fresh = m;
    StepCacheCounts c = stepCacheCounts();
    CHECK(c.modelsRead == static_cast<std::size_t>(kBodies));
    CHECK(c.meshesMade == static_cast<std::size_t>(kBodies));
    // Loaded again (another machine dialog, the project): nothing is read or made.
    start = std::chrono::steady_clock::now();
    (void)loadMachineFile(demoFile());
    const double warm = seconds(start);
    CHECK(stepCacheCounts().meshesMade == c.meshesMade);
    CHECK(stepCacheCounts().modelsRead == c.modelsRead);
    // Another tolerance: meshes made again, coarser.
    const std::size_t fine = partNamed(m, "rotary table").geometry->triangles.size();
    m.model.meshTolerance = 1.0;
    meshMachineParts(m);
    CHECK(stepCacheCounts().meshesMade == 2 * c.meshesMade);
    CHECK(stepCacheCounts().modelsRead == c.modelsRead);
    CHECK(partNamed(m, "rotary table").geometry->triangles.size() < fine);

    // On disk: the next session reads the meshes, not the STEP files.
    clearStepCache();
    setStepMeshCacheFolder(cacheDir);
    (void)loadMachineFile(demoFile());
    CHECK(stepCacheCounts().meshesMade == static_cast<std::size_t>(kBodies));
    clearStepCache();
    start = std::chrono::steady_clock::now();
    const machine::Machine fromDisk = loadMachineFile(demoFile());
    const double disk = seconds(start);
    c = stepCacheCounts();
    CHECK(c.meshesFromDisk == static_cast<std::size_t>(kBodies));
    CHECK(c.meshesMade == 0);
    CHECK(c.modelsRead == 0);
    // The same meshes as made from STEP.
    for (std::size_t i = 0; i < fromDisk.model.parts.size(); ++i) {
        CHECK(fromDisk.model.parts[i].geometry->vertices ==
              fresh.model.parts[i].geometry->vertices);
    }
    // A damaged cache file is made again.
    std::size_t damaged = 0;
    for (const auto& item : std::filesystem::directory_iterator(cacheDir)) {
        writeFileAtomic(item.path(), "not an STL");
        if (++damaged == 3) {
            break;
        }
    }
    clearStepCache();
    (void)loadMachineFile(demoFile());
    CHECK(stepCacheCounts().meshesMade == 3);
    setStepMeshCacheFolder({});
    std::filesystem::remove_all(cacheDir);
    std::printf("demo machine (%d STEP parts) loads in %.3f s from STEP, %.4f s again, "
                "%.3f s with meshes from the disk cache\n",
                kBodies, cold, warm, disk);
}

TEST_CASE("project archives keep machine parts as STEP files", "[document][machine][step]") {
    const machine::Machine demo = loadMachineFile(demoFile());
    Document doc;
    doc.modify("machine", [&](Project& p) { p.machines.push_back(demo); });
    const auto path = std::filesystem::temp_directory_path() / "oxcam_test_step_machine.ocam";
    doc.save(path);
    const ArchiveContents archive = readArchive(readFileBytes(path), "test");
    CHECK(archive.machineFiles.size() == demo.model.parts.size());
    for (const auto& [name, bytes] : archive.machineFiles) {
        CHECK(name.ends_with(".step"));
    }
    clearStepCache(); // opened in a new session
    const Document reopened = Document::open(path);
    CHECK(reopened.project() == doc.project());
    for (const machine::ModelPart& p : reopened.project().machines.at(0).model.parts) {
        REQUIRE(p.geometry);
        CHECK(p.geometry->isClosed());
    }
    std::filesystem::remove(path);
}

TEST_CASE("picking on the demo machine's exact geometry", "[document][machine][step]") {
    using namespace fixtures::trunnion_machine;
    const machine::Machine m = loadMachineFile(demoFile());
    const machine::MachineModel& model = m.model;
    const machine::ModelPart& table = partNamed(m, "rotary table");

    // The round table's rim: its centre line is Z through the table centre, at the rim's middle.
    const auto rim = find(table, cad::SubShapeType::Face, [](const cad::SubShapeGeometry& g) {
        return g.kind == Kind::Cylinder && std::abs(g.radius - kTableRadius) < 1e-6;
    });
    REQUIRE(rim);
    const auto axis = axisOfShape(*rim, table.placement());
    REQUIRE(axis);
    CHECK(near(axis->point, Vec3(0, 0, -25)));
    CHECK(std::abs(std::abs(axis->direction.z()) - 1.0) < 1e-12);
    CHECK_THAT(axis->radius, WithinAbs(kTableRadius, 1e-9));
    // Its top edge is the same circle.
    const auto edge = find(table, cad::SubShapeType::Edge, [](const cad::SubShapeGeometry& g) {
        return g.kind == Kind::Circle && std::abs(g.radius - kTableRadius) < 1e-6 &&
               std::abs(g.point.z()) < 1e-9;
    });
    REQUIRE(edge);
    CHECK(near(axisOfShape(*edge, table.placement())->point, Vec3::Zero()));

    // The table's top: its centre, where C meets it, or a point clicked on it.
    const auto top = find(table, cad::SubShapeType::Face, [](const cad::SubShapeGeometry& g) {
        return g.kind == Kind::Plane && g.direction.z() > 0.9 && std::abs(g.point.z()) < 1e-9;
    });
    REQUIRE(top);
    CHECK(near(snapOnShape(model, table, *top, Vec3(50, 30, 0), machine::Snap::FaceCentre),
               Vec3::Zero()));
    CHECK(near(snapOnShape(model, table, *top, Vec3(50, 30, 0), machine::Snap::AxisOnFace),
               Vec3::Zero()));
    CHECK(near(snapOnShape(model, table, *top, Vec3(50, 30, 0), machine::Snap::Point),
               Vec3(50, 30, 0)));
    // A vertex snaps exactly; the rim (round) has no plane for an axis to meet.
    CHECK_THROWS_AS(snapOnShape(model, table, *rim, Vec3(210, 0, -25), machine::Snap::AxisOnFace),
                    Error);
    const machine::ModelPart& xTable = partNamed(m, "x table");
    const auto corner = find(xTable, cad::SubShapeType::Vertex,
                             [](const cad::SubShapeGeometry& g) { return g.point.z() > -330.0; });
    REQUIRE(corner);
    CHECK(near(snapOnShape(model, xTable, *corner, Vec3(1, 2, 3), machine::Snap::Point),
               corner->point));

    // The spindle: the nose's bottom rim gives the gauge point in one click.
    const machine::ModelPart& nose = partNamed(m, "spindle nose");
    const auto seat = find(nose, cad::SubShapeType::Edge, [](const cad::SubShapeGeometry& g) {
        return g.kind == Kind::Circle && std::abs(g.point.z() - kSpindleZ) < 1e-9;
    });
    REQUIRE(seat);
    const auto spindle = axisOfShape(*seat, nose.placement());
    REQUIRE(spindle);
    CHECK(near(machine::gaugePointOn(*spindle, spindle->point), Vec3(0, 0, kSpindleZ)));
}

TEST_CASE("merged STEP parts stay exact", "[document][machine][step]") {
    const machine::Machine m = loadMachineFile(demoFile());
    machine::ModelPart left = partNamed(m, "support left");
    machine::ModelPart right = partNamed(m, "support right");
    right.offset = Vec3(0, 0, 10); // moved: merged where it stands
    const machine::ModelPart merged = mergeParts({left, right}, "supports", m.model.meshTolerance);
    CHECK(merged.exact());
    CHECK(merged.carrier == left.carrier);
    const cad::Model model = partModel(merged);
    CHECK(model.info().solids == 2);
    CHECK_THAT(model.info().volume,
               WithinRel(partModel(left).info().volume + partModel(right).info().volume, 1e-9));
    CHECK_THAT(model.info().bounds.max.z(),
               WithinAbs(partModel(right).info().bounds.max.z() + 10.0, 1e-6));
    REQUIRE(merged.geometry);
    CHECK(merged.geometry->isClosed());
    // With a mesh part among them, the result is a mesh part.
    machine::ModelPart mesh = machine::makePart("block", *left.geometry);
    const machine::ModelPart mixed = mergeParts({left, mesh}, "mixed", m.model.meshTolerance);
    CHECK_FALSE(mixed.exact());
    CHECK_FALSE(mixed.mesh.empty());
}

// Load times of a real machine's STEP file: set OXCAM_BENCH_MACHINE_STEP to its path and run
// test_document "[.bench]".
TEST_CASE("load times of a machine's STEP file", "[.bench]") {
    const char* file = std::getenv("OXCAM_BENCH_MACHINE_STEP");
    if (!file) {
        SKIP("OXCAM_BENCH_MACHINE_STEP is not set");
    }
    CancelToken cancel;
    NullProgress progress;
    clearStepCache();
    auto start = std::chrono::steady_clock::now();
    const auto found = readMachinePartFile(file, 1.0, cancel, progress);
    const double read = seconds(start);
    start = std::chrono::steady_clock::now();
    machine::Machine m = machine::genericMill3();
    m.id = "bench";
    m.model = machine::defaultModel(m);
    m.model.parts = machinePartsFrom(found, groupParts(found, 0), 0.2, cancel, progress);
    const double split = seconds(start);
    std::size_t triangles = 0;
    for (const auto& p : m.model.parts) {
        triangles += p.geometry->triangles.size();
    }
    const auto dir = std::filesystem::temp_directory_path() / "oxcam_bench_machine";
    std::filesystem::remove_all(dir);
    machine::saveMachine(m, dir / "bench.json");
    clearStepCache();
    setStepMeshCacheFolder({});
    start = std::chrono::steady_clock::now();
    (void)loadMachineFile(dir / "bench.json");
    const double cold = seconds(start);
    start = std::chrono::steady_clock::now();
    (void)loadMachineFile(dir / "bench.json");
    const double warm = seconds(start);
    setStepMeshCacheFolder(dir / "cache");
    clearStepCache();
    (void)loadMachineFile(dir / "bench.json");
    clearStepCache();
    start = std::chrono::steady_clock::now();
    (void)loadMachineFile(dir / "bench.json");
    const double disk = seconds(start);
    setStepMeshCacheFolder({});
    std::filesystem::remove_all(dir);
    std::printf("%s: %zu bodies, %zu triangles at 0.2 mm; import: read %.2f s, split and mesh "
                "%.2f s; load: from STEP %.2f s, again %.3f s, meshes from disk %.2f s\n",
                file, m.model.parts.size(), triangles, read, split, cold, warm, disk);
}
