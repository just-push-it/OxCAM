// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "FixtureSpecs.hpp"

#include <ocam/cad/Model.hpp>
#include <ocam/cad/Tessellation.hpp>
#include <ocam/foundation/Error.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <filesystem>
#include <fstream>

using namespace ocam;
using namespace ocam::fixtures;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

std::filesystem::path fixture(const char* name) {
    return std::filesystem::path(OCAM_FIXTURE_DIR) / name;
}

} // namespace

TEST_CASE("STEP import: block with pocket", "[cad][import]") {
    using namespace block_pocket;
    const cad::Model model = cad::Model::importStep(fixture("block_pocket.step"));
    const cad::ModelInfo info = model.info();
    CHECK(info.solids == 1);
    CHECK_THAT(info.volume, WithinRel(kVolume, 1e-6));
    CHECK_THAT(info.bounds.size().x(), WithinRel(kLength, 1e-6));
    CHECK_THAT(info.bounds.size().y(), WithinRel(kWidth, 1e-6));
    CHECK_THAT(info.bounds.size().z(), WithinRel(kHeight, 1e-6));
    CHECK_THAT(info.bounds.min.z(), WithinAbs(0.0, 1e-6));
}

TEST_CASE("STEP import converts inch files to millimetres", "[cad][import]") {
    using namespace block_pocket;
    const cad::ModelInfo mm = cad::Model::importStep(fixture("block_pocket.step")).info();
    const cad::ModelInfo in = cad::Model::importStep(fixture("block_pocket_inch.step")).info();
    CHECK_THAT(in.volume, WithinRel(mm.volume, 1e-6));
    CHECK_THAT(in.bounds.max.x(), WithinRel(kLength, 1e-6));
    CHECK_THAT(in.bounds.max.z(), WithinRel(kHeight, 1e-6));
}

TEST_CASE("STEP import: stepped shaft", "[cad][import]") {
    const cad::ModelInfo info = cad::Model::importStep(fixture("shaft.step")).info();
    CHECK(info.solids == 1);
    CHECK_THAT(info.volume, WithinRel(shaft::kVolume, 1e-6));
    CHECK_THAT(info.bounds.size().z(), WithinRel(shaft::kMajorLength + shaft::kMinorLength, 1e-6));
}

TEST_CASE("STEP import errors", "[cad][import]") {
    try {
        (void)cad::Model::importStep(fixture("does_not_exist.step"));
        FAIL("expected an exception");
    } catch (const Error& e) {
        CHECK(e.code() == ErrorCode::Io);
    }

    const auto garbage = std::filesystem::temp_directory_path() / "oxcam_garbage.step";
    std::ofstream(garbage) << "this is not ISO-10303-21\n";
    try {
        (void)cad::Model::importStep(garbage);
        FAIL("expected an exception");
    } catch (const Error& e) {
        CHECK(e.code() == ErrorCode::Parse);
    }
    std::filesystem::remove(garbage);
}

TEST_CASE("STEP import honours cancellation", "[cad][import]") {
    CancelToken cancel;
    cancel.requestCancel();
    NullProgress progress;
    CHECK_THROWS_AS(cad::Model::importStep(fixture("block_pocket.step"), cancel, progress),
                    OperationCancelled);
}

TEST_CASE("STEP export round-trips", "[cad][import]") {
    const cad::Model model = cad::Model::importStep(fixture("hole_plate.step"));
    const auto path = std::filesystem::temp_directory_path() / "oxcam_roundtrip.step";
    model.exportStep(path);
    CHECK_THAT(cad::Model::importStep(path).info().volume, WithinRel(model.info().volume, 1e-9));
    std::filesystem::remove(path);
}

TEST_CASE("tessellation is closed and matches the B-rep volume", "[cad][mesh]") {
    for (const char* name :
         {"block_pocket.step", "hole_plate.step", "boss_plate.step", "shaft.step"}) {
        INFO(name);
        const cad::Model model = cad::Model::importStep(fixture(name));
        const geom3d::Mesh mesh = cad::tessellate(model, {0.005, 0.2});
        CHECK(mesh.isClosed());
        CHECK_THAT(mesh.signedVolume(), WithinRel(model.info().volume, 2e-3));
        // Facets are inscribed, so curved extents may shrink by up to the deflection per side.
        CHECK((mesh.bounds().size() - model.info().bounds.size()).cwiseAbs().maxCoeff() <=
              2 * 0.005);
    }
}
