// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "FixtureSpecs.hpp"

#include <ocam/document/MachineImport.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/geom3d/Stl.hpp>
#include <ocam/machine/Model.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <map>
#include <set>

using namespace ocam;
using namespace ocam::document;
using namespace ocam::fixtures;

namespace {

std::filesystem::path fixture(const char* name) {
    return std::filesystem::path(OCAM_FIXTURE_DIR) / name;
}

bool near(const geom3d::Aabb3& b, const Vec3& lo, const Vec3& hi, double tol) {
    return (b.min - lo).cwiseAbs().maxCoeff() < tol && (b.max - hi).cwiseAbs().maxCoeff() < tol;
}

} // namespace

TEST_CASE("each body of a machine's STEP file is a part on its own axis",
          "[document][machine][import]") {
    using namespace trunnion_machine;
    CancelToken cancel;
    NullProgress progress;
    const auto found = readMachinePartFile(fixture("trunnion_machine.step"), 1.0, cancel, progress);
    REQUIRE(found.size() == static_cast<std::size_t>(kBodies));
    CHECK(assemblyDepth(found) == 0); // one product: nothing to merge by
    const auto groups = groupParts(found, 0);
    CHECK(std::set<std::string>(groups.begin(), groups.end()).size() == found.size());
    const auto parts = machinePartsFrom(found, groups, 0.2, cancel, progress);
    REQUIRE(parts.size() == static_cast<std::size_t>(kBodies));

    // Assigned one by one, as the editor does: the bodies on the table side move with their axes.
    machine::Machine m = machine::genericMill5TableAC();
    m.components.clear();
    m.model = machine::defaultModel(m);
    m.model.spindlePoint = Vec3(0, 0, kSpindleZ);
    m.model.parts = parts;
    const std::map<std::string, std::string> carriers = {
        {"saddle", "Y"}, {"x table", "X"},      {"support left", "X"}, {"support right", "X"},
        {"cradle", "A"}, {"rotary table", "C"}, {"spindle nose", "Z"}, {"spindle head", "Z"}};
    for (auto& p : m.model.parts) {
        const auto it = carriers.find(p.name);
        p.carrier = it == carriers.end() ? std::string() : it->second;
        CHECK(p.offset == Vec3::Zero());
    }
    REQUIRE(m.validate().empty());
    machine::AxisPosition at;
    at.linear = Vec3(100, 0, -50);
    at.rotary.a = degToRad(90.0);
    const machine::ModelPose pose = machine::poseAt(m.model, at);
    auto placed = [&](const std::string& name) {
        const auto& p = *std::find_if(m.model.parts.begin(), m.model.parts.end(),
                                      [&](const machine::ModelPart& q) { return q.name == name; });
        geom3d::Mesh mesh = *p.geometry;
        mesh.transform(pose.carried(m.model, p.carrier) * p.placement());
        return mesh.bounds();
    };
    // X moves the table side the other way; the column stays; Z lowers the head; A stands the
    // cradle up about X.
    CHECK(near(placed("support right"), Vec3(kSupportInner - 100, -130, -320), Vec3(310, 130, 60),
               1e-3));
    CHECK(near(placed("column"), Vec3(-320, kColumnFront, -420), Vec3(320, 900, 1300), 1e-3));
    CHECK(near(placed("spindle nose"), Vec3(-55, -55, kSpindleZ - 50), Vec3(55, 55, kSpindleZ + 60),
               0.25));
    const geom3d::Aabb3 cradle = placed("cradle");
    CHECK(std::abs(cradle.min.z() - -125) < 1e-3);
    CHECK(std::abs(cradle.max.z() - 125) < 1e-3);
}

TEST_CASE("a rotary axis picked from the machine's round table", "[document][machine][import]") {
    using namespace trunnion_machine;
    const machine::Machine m = document::loadMachineFile(fixture("machines/trunnion-demo.json"));
    auto part = [&](const std::string& name) -> const machine::ModelPart& {
        return *std::find_if(m.model.parts.begin(), m.model.parts.end(),
                             [&](const machine::ModelPart& p) { return p.name == name; });
    };
    // A point on the round table's rim (machine zero): its centre line is Z through the table
    // origin, halfway up the rim.
    const auto table = machine::axisAtPoint(part("rotary table"), Vec3(kTableRadius, 0, -25));
    REQUIRE(table);
    CHECK(std::abs(std::abs(table->direction.z()) - 1.0) < 1e-6);
    CHECK(table->point.head<2>().norm() < 0.05);
    CHECK(std::abs(table->point.z() - -25) < 0.05);
    CHECK(std::abs(table->radius - kTableRadius) < 0.3);
    const auto sense = machine::sameSense(table->direction, Vec3::UnitZ());
    REQUIRE(sense);
    CHECK((*sense - Vec3::UnitZ()).norm() < 1e-6);
    // The spindle nose: Z through the spindle.
    const auto nose = machine::axisAtPoint(part("spindle nose"), Vec3(0, 55, kSpindleZ + 30));
    REQUIRE(nose);
    CHECK(nose->point.head<2>().norm() < 0.05);
    // The spindle's gauge point: the nose's centre line, then the face the holder seats against
    // (the nose's bottom face).
    const auto seatNormal = machine::normalAtPoint(part("spindle nose"), Vec3(20, 10, kSpindleZ));
    REQUIRE(seatNormal);
    CHECK(std::abs(std::abs(seatNormal->dot(nose->direction)) - 1.0) < 1e-6);
    const Vec3 gauge = machine::gaugePointOn(*nose, Vec3(20, 10, kSpindleZ));
    CHECK((gauge - m.model.spindlePoint).norm() < 0.05);
    CHECK((gauge - Vec3(0, 0, kSpindleZ)).norm() < 0.05);
    // A setup's work snapped to the round table's top: on the C axis line, or the top's centre.
    const auto top = machine::faceAtPoint(part("rotary table"), Vec3(50, 30, 0));
    REQUIRE(top);
    CHECK((top->normal - Vec3::UnitZ()).norm() < 1e-6);
    CHECK(top->centre.norm() < 0.05);
    const geom3d::AxisLine cAxis{Vec3(0, 0, -100), Vec3::UnitZ(), 0.0, 0};
    const auto origin = machine::lineMeetsPlane(cAxis, Vec3(50, 30, 0), top->normal);
    REQUIRE(origin);
    CHECK(origin->norm() < 1e-9);
    CHECK_FALSE(machine::lineMeetsPlane(cAxis, Vec3::Zero(), Vec3::UnitX()));
    // A flat face has no axis; a point off the part has none either.
    CHECK_FALSE(machine::axisAtPoint(part("rotary table"), Vec3(50, 30, 0)));
    CHECK_FALSE(machine::axisAtPoint(part("rotary table"), Vec3(5000, 0, 0)));
    CHECK_FALSE(machine::sameSense(Vec3::UnitX(), Vec3::UnitZ()));
}

TEST_CASE("a machine assembly becomes machine parts", "[document][machine][import]") {
    using namespace trunnion_machine;
    CancelToken cancel;
    NullProgress progress;
    const auto found =
        readMachinePartFile(fixture("trunnion_assembly.step"), 1.0, cancel, progress);
    REQUIRE(found.size() == static_cast<std::size_t>(kAssemblyParts));
    // The top assembly (the machine) is shared by all and left out of the paths.
    const auto column = std::find_if(found.begin(), found.end(),
                                     [](const ImportedPart& p) { return p.name == "column"; });
    REQUIRE(column != found.end());
    CHECK(column->path == std::vector<std::string>{"Frame"});
    CHECK(column->file == "trunnion_assembly.step");
    CHECK(assemblyDepth(found) == 2);

    // Each part on its own: repeated names are numbered.
    const auto alone = groupParts(found, 0);
    CHECK(std::set<std::string>(alone.begin(), alone.end()).size() == found.size());
    CHECK(std::count(alone.begin(), alone.end(), "bolt (2)") == 1);
    // One level down: one group per subassembly; two levels: the bolts on their own.
    const auto level1 = groupParts(found, 1);
    CHECK(std::set<std::string>(level1.begin(), level1.end()).size() ==
          static_cast<std::size_t>(kSubassemblies));
    const auto level2 = groupParts(found, 2);
    CHECK(std::set<std::string>(level2.begin(), level2.end()).size() ==
          static_cast<std::size_t>(kSubassemblies) + 1);

    // Merged and placed as the file places them (in millimetres; the file is in inches).
    const auto parts = machinePartsFrom(found, level1, 0.2, cancel, progress);
    REQUIRE(parts.size() == static_cast<std::size_t>(kSubassemblies));
    const auto xTable = std::find_if(parts.begin(), parts.end(), [](const machine::ModelPart& p) {
        return p.name == "X table";
    });
    REQUIRE(xTable != parts.end());
    REQUIRE(xTable->geometry);
    CHECK(near(xTable->geometry->bounds(), Vec3(-520, -260, -370), Vec3(520, 260, 60), 1e-3));
    CHECK(xTable->offset == Vec3::Zero());
    const auto cradle = std::find_if(
        parts.begin(), parts.end(), [](const machine::ModelPart& p) { return p.name == "Cradle"; });
    REQUIRE(cradle != parts.end());
    CHECK(cradle->colour == 0xC28A3Au);
    // The round table within the chord error.
    const auto table = std::find_if(parts.begin(), parts.end(), [](const machine::ModelPart& p) {
        return p.name == "Rotary table";
    });
    REQUIRE(table != parts.end());
    CHECK(near(table->geometry->bounds(), Vec3(-kTableRadius, -kTableRadius, -170),
               Vec3(kTableRadius, kTableRadius, 0), 0.25));

    // Left out: an empty group.
    std::vector<std::string> some = level1;
    std::replace(some.begin(), some.end(), std::string("Frame"), std::string());
    CHECK(machinePartsFrom(found, some, 0.2, cancel, progress).size() ==
          static_cast<std::size_t>(kSubassemblies) - 1);
    CHECK_THROWS_AS(machinePartsFrom(found, level1, 0.0, cancel, progress), Error);
}

TEST_CASE("STL machine parts in other units", "[document][machine][import]") {
    machine::Component box;
    box.min = Vec3::Zero();
    box.max = Vec3(1, 2, 3);
    const auto file = std::filesystem::temp_directory_path() / "oxcam-import-test-box.stl";
    writeFileAtomic(file, geom3d::writeStl(machine::componentMesh(box)));
    CancelToken cancel;
    NullProgress progress;
    const auto inches = readMachinePartFile(file, 25.4, cancel, progress);
    REQUIRE(inches.size() == 1);
    CHECK(inches[0].name == "oxcam-import-test-box");
    REQUIRE(inches[0].mesh);
    CHECK(near(inches[0].mesh->bounds(), Vec3::Zero(), Vec3(25.4, 50.8, 76.2), 1e-4));
    const auto parts = machinePartsFrom(inches, groupParts(inches, 0), 0.2, cancel, progress);
    REQUIRE(parts.size() == 1);
    CHECK(parts[0].geometry->triangles.size() == 12);
    std::filesystem::remove(file);
    CHECK_THROWS_AS(readMachinePartFile(fixture("sample_project.ocam"), 1.0, cancel, progress),
                    Error);
}
