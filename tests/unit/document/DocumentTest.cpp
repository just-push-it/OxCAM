// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "FixtureSpecs.hpp"
#include "TestProject.hpp"

#include <ocam/cad/SubShape.hpp>
#include <ocam/document/Document.hpp>
#include <ocam/document/ProjectIo.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/machine/Model.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <filesystem>
#include <stdexcept>

using namespace ocam;
using namespace ocam::document;
using Catch::Matchers::WithinRel;

namespace {

std::filesystem::path fixture(const char* name) {
    return std::filesystem::path(OCAM_FIXTURE_DIR) / name;
}

std::filesystem::path tempFile(const char* name) {
    return std::filesystem::temp_directory_path() / name;
}

} // namespace

TEST_CASE("undo, redo and modified state", "[document][undo]") {
    Document doc;
    int notifications = 0;
    const int handle = doc.addListener([&] { ++notifications; });
    CHECK_FALSE(doc.isModified());
    CHECK_FALSE(doc.canUndo());

    doc.modify("Rename", [](Project& p) { p.name = "A"; });
    doc.modify("Rename again", [](Project& p) { p.name = "B"; });
    CHECK(doc.project().name == "B");
    CHECK(doc.isModified());
    CHECK(doc.undoLabel() == "Rename again");
    CHECK(notifications == 2);

    doc.modify("No-op", [](Project& p) { p.name = "B"; });
    CHECK(doc.undoLabel() == "Rename again"); // not recorded
    CHECK(notifications == 2);

    CHECK_THROWS_AS(doc.modify("Fails",
                               [](Project& p) {
                                   p.name = "C";
                                   throw std::runtime_error("abort");
                               }),
                    std::runtime_error);
    CHECK(doc.project().name == "B");

    doc.undo();
    CHECK(doc.project().name == "A");
    CHECK(doc.redoLabel() == "Rename again");
    doc.undo();
    CHECK(doc.project().name == "Untitled");
    CHECK_FALSE(doc.isModified()); // back at the initial state
    doc.undo();                    // nothing left: no-op
    doc.redo();
    CHECK(doc.project().name == "A");

    doc.modify("Branch", [](Project& p) { p.name = "D"; });
    CHECK_FALSE(doc.canRedo()); // redo branch discarded
    doc.removeListener(handle);
    doc.undo();
    CHECK(notifications == 6);
}

TEST_CASE("import a model, add a setup and resolve its stock", "[document]") {
    using namespace fixtures::block_pocket;
    Document doc;
    CancelToken cancel;
    NullProgress progress;
    const Uuid modelId = doc.importModel(fixture("block_pocket.step"), cancel, progress);
    CHECK(doc.project().name == "block_pocket");
    REQUIRE(doc.model(modelId) != nullptr);
    CHECK(doc.project().models.at(0).sha256.size() == 64);

    const auto bounds = doc.modelBounds(modelId);
    REQUIRE(bounds.has_value());
    CHECK_THAT(bounds->size().x(), WithinRel(kLength, 1e-6));

    const Uuid setupId = doc.addSetup(modelId);
    const Setup* setup = doc.project().findSetup(setupId);
    REQUIRE(setup != nullptr);
    CHECK(setup->name == "Setup 1");
    const auto stock = doc.stockShape(setupId);
    REQUIRE(stock.has_value());
    // Default stock: 1 mm on the sides and top, none below.
    CHECK((stock->box.min - Vec3(-1, -1, 0)).norm() < 1e-6);
    CHECK((stock->box.max - Vec3(kLength + 1, kWidth + 1, kHeight + 1)).norm() < 1e-6);
    CHECK((setup->wcs.origin - Vec3(-1, -1, kHeight + 1)).norm() < 1e-6);
    CHECK(doc.project().validate().empty());

    // Moving the model moves its bounds.
    doc.modify("Move model", [&](Project& p) { p.models[0].placement.origin = Vec3(10, 0, 0); });
    CHECK_THAT(doc.modelBounds(modelId)->min.x(), WithinRel(10.0, 1e-9));

    // Undoing the import removes the entry; redo restores it with its geometry.
    doc.undo();
    doc.undo();
    doc.undo();
    CHECK(doc.project().models.empty());
    CHECK(doc.model(modelId) == nullptr);
    doc.redo();
    CHECK(doc.model(modelId) != nullptr);
}

TEST_CASE("save and reopen a project archive", "[document][io]") {
    const auto path = tempFile("oxcam_test_project.ocam");
    Document doc;
    CancelToken cancel;
    NullProgress progress;
    const Uuid plate = doc.importModel(fixture("hole_plate.step"), cancel, progress);
    doc.importModel(fixture("hole_plate.step"), cancel, progress); // same file twice
    const Uuid setupId = doc.addSetup(plate);
    doc.modify("Add tool and operation", [&](Project& p) {
        tooling::Tool t =
            testing::withPreset(tooling::makeDrill(1, 10.0, degToRad(118.0), 50.0, 90.0));
        Operation op;
        op.name = "Drill holes";
        op.type = "drill";
        op.toolId = t.id;
        op.presetId = t.presets.front().id;
        p.tools.tools.push_back(t);
        p.findSetup(setupId)->operations.push_back(op);
    });
    doc.save(path);
    CHECK_FALSE(doc.isModified());
    CHECK(doc.path() == path);

    const Document reopened = Document::open(path);
    CHECK(reopened.project() == doc.project());
    CHECK_FALSE(reopened.isModified());
    REQUIRE(reopened.model(plate) != nullptr);
    CHECK_THAT(reopened.model(plate)->info().volume,
               WithinRel(doc.model(plate)->info().volume, 1e-9));
    CHECK(reopened.stockShape(setupId) == doc.stockShape(setupId));

    // Undo after save makes the document modified again.
    doc.undo();
    CHECK(doc.isModified());
    doc.redo();
    CHECK_FALSE(doc.isModified());
    std::filesystem::remove(path);
}

TEST_CASE("project machines keep their part meshes in the archive", "[document][io][machine]") {
    const auto path = tempFile("oxcam_test_machine_project.ocam");
    machine::Machine m = machine::genericMill5TableAC();
    m.id = "shop-trunnion";
    m.model = machine::defaultModel(m);
    machine::Component box;
    box.min = Vec3(-300, 300, -600);
    box.max = Vec3(300, 500, 800);
    machine::ModelPart column = machine::makePart("column", machine::componentMesh(box));
    machine::ModelPart table = machine::makePart("table", machine::componentMesh(m.components[2]));
    table.carrier = "C";
    m.model.parts = {column, table, column}; // the same mesh twice is stored once
    m.model.parts[2].name = "column copy";
    REQUIRE(m.validate().empty());

    Document doc;
    doc.modify("Add machine", [&](Project& p) { p.machines.push_back(m); });
    doc.save(path);
    const auto archive = readArchive(readFileBytes(path), "test");
    CHECK(archive.machineFiles.size() == 2);

    const Document reopened = Document::open(path);
    CHECK(reopened.project() == doc.project());
    const machine::Machine& back = reopened.project().machines.at(0);
    REQUIRE(back.model.parts.size() == 3);
    for (const auto& part : back.model.parts) {
        REQUIRE(part.geometry);
        CHECK(part.geometry->isClosed());
    }
    std::filesystem::remove(path);

    // A project whose machine mesh is missing does not open.
    ArchiveContents broken = archive;
    broken.machineFiles.erase(table.mesh + ".stl");
    writeFileAtomic(path, writeArchive(broken));
    CHECK_THROWS_AS(Document::open(path), Error);
    std::filesystem::remove(path);
}

TEST_CASE("opening damaged archives fails cleanly", "[document][io]") {
    const auto path = tempFile("oxcam_damaged.ocam");
    writeFileAtomic(path, "PK\x03\x04 truncated");
    CHECK_THROWS_AS(Document::open(path), Error);
    std::filesystem::remove(path);
    CHECK_THROWS_AS(Document::open(tempFile("oxcam_missing.ocam")), Error);
}

TEST_CASE("face references survive re-import", "[document][subshape]") {
    const cad::Model original = cad::Model::importStep(fixture("hole_plate.step"));
    const int faces = cad::subShapeCount(original, cad::SubShapeType::Face);
    REQUIRE(faces > 5);

    const auto path = tempFile("oxcam_reexport.step");
    original.exportStep(path);
    const cad::Model reimported = cad::Model::importStep(path);
    std::filesystem::remove(path);

    for (int i = 1; i <= faces; ++i) {
        const cad::ShapeSignature sig = cad::signatureOf(original, cad::SubShapeType::Face, i);
        // A wrong hint must still find a face with the same signature.
        const int wrongHint = i % faces + 1;
        const auto match = cad::matchSubShape(reimported, cad::SubShapeType::Face, sig, wrongHint);
        REQUIRE(match.has_value());
        const cad::ShapeSignature found =
            cad::signatureOf(reimported, cad::SubShapeType::Face, *match);
        CHECK((found.centroid - sig.centroid).norm() < 1e-6);
        CHECK_THAT(found.measure, WithinRel(sig.measure, 1e-6));
    }

    cad::ShapeSignature bogus;
    bogus.centroid = Vec3(1000, 1000, 1000);
    bogus.measure = 1.0;
    CHECK_FALSE(cad::matchSubShape(reimported, cad::SubShapeType::Face, bogus, 1).has_value());
    CHECK_THROWS_AS(cad::signatureOf(original, cad::SubShapeType::Edge, 0), Error);
}
