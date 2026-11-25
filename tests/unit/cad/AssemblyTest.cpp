// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "FixtureSpecs.hpp"

#include <ocam/cad/Assembly.hpp>
#include <ocam/foundation/Error.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>

using namespace ocam;
using namespace ocam::fixtures;

namespace {

std::filesystem::path fixture(const char* name) {
    return std::filesystem::path(OCAM_FIXTURE_DIR) / name;
}

bool near(const geom3d::Aabb3& b, const Vec3& lo, const Vec3& hi) {
    // OCCT's bounds carry a small gap; the file is in inches.
    return (b.min - lo).cwiseAbs().maxCoeff() < 1e-3 && (b.max - hi).cwiseAbs().maxCoeff() < 1e-3;
}

} // namespace

TEST_CASE("one STEP file of a whole machine splits into its bodies", "[cad][import][assembly]") {
    using namespace trunnion_machine;
    CancelToken cancel;
    NullProgress progress;
    const auto parts = cad::importStepAssembly(fixture("trunnion_machine.step"), cancel, progress);
    REQUIRE(parts.size() == static_cast<std::size_t>(kBodies));
    auto only = [&](const std::string& name) -> const cad::AssemblyPart& {
        const auto n = std::count_if(parts.begin(), parts.end(),
                                     [&](const cad::AssemblyPart& p) { return p.name == name; });
        REQUIRE(n == 1);
        return *std::find_if(parts.begin(), parts.end(),
                             [&](const cad::AssemblyPart& p) { return p.name == name; });
    };
    // Each body by its own name from the file, where the file places it (in millimetres; the
    // file is in inches).
    for (const char* name :
         {"base", "enclosure", "column", "saddle", "x table", "support left", "support right",
          "cradle", "rotary table", "spindle nose", "spindle head"}) {
        CAPTURE(name);
        const auto& p = only(name);
        CHECK(p.solids == 1);
        CHECK(p.path == std::vector<std::string>{"Demo trunnion machine"});
    }
    CHECK(near(only("support right").model.info().bounds, Vec3(kSupportInner, -130, -320),
               Vec3(410, 130, 60)));
    CHECK(near(only("enclosure").model.info().bounds, Vec3(-kEnclosureX, -800, -800),
               Vec3(kEnclosureX, kEnclosureBack, 1500)));
    CHECK(near(only("spindle head").model.info().bounds, Vec3(-250, -150, kSpindleZ + 110),
               Vec3(250, kColumnFront, kSpindleZ + 800)));
    CHECK(only("cradle").colour == 0xC28A3Au);
    // Bodies without a name take the product's, numbered.
    const auto bolts = std::count_if(parts.begin(), parts.end(), [](const cad::AssemblyPart& p) {
        return p.name.rfind("Demo trunnion machine ", 0) == 0 && p.colour == 0x30343Au;
    });
    CHECK(bolts == kBolts);
}

TEST_CASE("STEP assemblies split into placed, named parts", "[cad][import][assembly]") {
    using namespace trunnion_machine;
    CancelToken cancel;
    NullProgress progress;
    const auto parts = cad::importStepAssembly(fixture("trunnion_assembly.step"), cancel, progress);
    REQUIRE(parts.size() == static_cast<std::size_t>(kAssemblyParts));
    auto find = [&](const std::string& name) {
        std::vector<const cad::AssemblyPart*> out;
        for (const auto& p : parts) {
            if (p.name == name) {
                out.push_back(&p);
            }
        }
        return out;
    };
    // Placed where the assembly puts them, in millimetres (the file is in inches).
    const auto column = find("column");
    REQUIRE(column.size() == 1);
    CHECK(
        near(column[0]->model.info().bounds, Vec3(-320, kColumnFront, -420), Vec3(320, 900, 1300)));
    CHECK(column[0]->path == std::vector<std::string>{"Demo trunnion machine", "Frame"});
    CHECK(column[0]->solids == 1);
    // One product, two instances.
    const auto supports = find("support");
    REQUIRE(supports.size() == 2);
    std::vector<double> xs{supports[0]->model.info().bounds.min.x(),
                           supports[1]->model.info().bounds.min.x()};
    std::sort(xs.begin(), xs.end());
    CHECK(std::abs(xs[0] + 410) < 1e-3);
    CHECK(std::abs(xs[1] - kSupportInner) < 1e-3);
    // Nested subassemblies.
    const auto bolts = find("bolt");
    REQUIRE(bolts.size() == static_cast<std::size_t>(kBolts));
    CHECK(bolts[0]->path == std::vector<std::string>{"Demo trunnion machine", "Frame", "Bolts"});
    // Colours from the file.
    const auto cradle = find("cradle");
    REQUIRE(cradle.size() == 1);
    REQUIRE(cradle[0]->colour);
    CHECK(*cradle[0]->colour == 0xC28A3Au);
}

TEST_CASE("STEP files without structure split into solids", "[cad][import][assembly]") {
    CancelToken cancel;
    NullProgress progress;
    // vise_jaws.step: two jaws, one shape.
    const auto parts = cad::importStepAssembly(fixture("vise_jaws.step"), cancel, progress);
    REQUIRE(parts.size() == 2);
    CHECK(parts[0].name == "vise_jaws 1");
    CHECK(parts[1].name == "vise_jaws 2");
    CHECK(parts[0].solids == 1);
    // A single solid stays one part.
    const auto block = cad::importStepAssembly(fixture("block_pocket.step"), cancel, progress);
    REQUIRE(block.size() == 1);
    CHECK(block[0].name == "block_pocket");
    CHECK_THROWS_AS(cad::importStepAssembly(fixture("missing.step"), cancel, progress), Error);
    CancelToken cancelled;
    cancelled.requestCancel();
    CHECK_THROWS_AS(cad::importStepAssembly(fixture("trunnion_machine.step"), cancelled, progress),
                    OperationCancelled);
}
