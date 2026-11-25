// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Files written under the program's former name, OpenCAM (ADR-0005), keep opening. The files in
// tests/fixtures/opencam-0.1/ were written by OpenCAM 0.1.0 (the vise files at main dcbf95e) and
// must never be regenerated.

#include <ocam/document/Document.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/ProjectIo.hpp>
#include <ocam/document/ToolpathCache.hpp>
#include <ocam/document/Workholding.hpp>
#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Progress.hpp>
#include <ocam/machine/Machine.hpp>
#include <ocam/post/Post.hpp>
#include <ocam/tooling/ToolLibrary.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <map>
#include <string>

using namespace ocam;
using namespace ocam::document;

namespace {

std::filesystem::path frozen(const char* name) {
    return std::filesystem::path(OCAM_SOURCE_DIR) / "tests/fixtures/opencam-0.1" / name;
}

} // namespace

TEST_CASE("an OpenCAM project opens and saves under the new name", "[document][io][legacy]") {
    const Document doc = Document::open(frozen("sample_project.ocam"));
    CHECK(doc.project().name == "block_pocket");
    CHECK_FALSE(doc.project().setups.empty());
    CHECK_FALSE(doc.project().tools.tools.empty());
    CHECK(doc.project().validate().empty());
    CHECK(toJson(doc.project()).at("format") == "oxcam.project");
    CHECK(toJson(doc.project()).at("tools").at("format") == "oxcam.toollib");
}

TEST_CASE("an OpenCAM vise and a project holding it open", "[document][io][legacy]") {
    const WorkholdingPackage vise = loadWorkholdingFile(frozen("vise.ocamhold"));
    CHECK(vise.workholding.parts.size() == 3);
    CHECK(vise.workholding.validate().empty());
    CHECK(toJson(vise.workholding).at("format") == "oxcam.workholding");
    CHECK(toJson(vise.workholding).at("version") == kWorkholdingVersion);

    const Document doc = Document::open(frozen("vise_project.ocam"));
    REQUIRE(doc.project().workholdings.size() == 1);
    REQUIRE(doc.project().setups.front().workholding.has_value());
    CHECK(doc.project().validate().empty());
    CHECK(toJson(doc.project()).at("workholdings").at(0).at("format") == "oxcam.workholding");
    // The version 1 vise (DL-0035) holds the stock as it did: the jaws closed on its 62 mm.
    const auto pose = setupWorkholdingPose(doc, doc.project().setups.front());
    REQUIRE(pose);
    CHECK(std::abs(pose->opening - 62.0) < 1e-6);
    CancelToken cancel;
    NullProgress progress;
    for (const Operation& op : doc.project().setups.front().operations) {
        CHECK_NOTHROW(generateToolpath(doc, doc.project().setups.front(), op, cancel, progress));
    }
}

TEST_CASE("version 2 workholding templates open", "[document][io][legacy]") {
    // Written by OxCAM at main bd2258b (before chucks); never regenerate them.
    const auto dir = std::filesystem::path(OCAM_SOURCE_DIR) / "tests/fixtures/workholding-v2";
    const WorkholdingPackage centring = loadWorkholdingFile(dir / "selfcentring.ocamhold");
    CHECK(centring.workholding.validate().empty());
    CHECK(centring.workholding.members.size() == 2);
    CHECK(toJson(centring.workholding).at("version") == kWorkholdingVersion);
    const WorkholdingPackage dovetail = loadWorkholdingFile(dir / "dovetail.ocamhold");
    CHECK(dovetail.workholding.validate().empty());
    REQUIRE(dovetail.workholding.holdings.size() == 1);
    CHECK(dovetail.workholding.holdings.front().kind == HoldingKind::Dovetail);
}

TEST_CASE("an OpenCAM tool library and machine load", "[document][io][legacy]") {
    const auto library = tooling::loadToolLibrary(frozen("sample-metric.ocamtools"));
    CHECK_FALSE(library.tools.empty());
    CHECK(tooling::toJson(library).at("format") == "oxcam.toollib");

    const auto machine = machine::loadMachine(frozen("generic-5axis-trunnion-ac.json"));
    CHECK(machine::toJson(machine).at("format") == "oxcam.machine");
    // OpenCAM 0.1 post configurations (JSON) are no longer read: posts are JavaScript
    // (ADR-0006).
}

TEST_CASE("an OpenCAM NC program reads", "[document][post][legacy]") {
    const Json doc = parseJson(readFileBytes(frozen("sample_program.json")), "program");
    REQUIRE(doc.at("format") == "opencam-ncprogram");
    const post::NcProgram program = post::ncProgramFromJson(doc);
    CHECK(program.instrs.size() == 263);
    CHECK(post::toJson(program).at("format") == "oxcam-ncprogram");
}

TEST_CASE("an OpenCAM toolpath cache loads", "[document][cache][legacy]") {
    const Document doc = Document::open(frozen("sample_project.ocam"));
    const Setup& setup = doc.project().setups.front();
    const Operation& op = setup.operations.front();
    CancelToken cancel;
    NullProgress progress;
    REQUIRE(cachedToolpath(doc, setup, op, cancel, progress) != nullptr);

    const std::map<Uuid, std::string> current{{op.id, toolpathInputKey(doc, setup, op)}};
    Json saved = doc.toolpathCache().toJson(current);
    CHECK(saved.at("format") == "oxcam.toolpath-cache");
    saved["format"] = "opencam.toolpath-cache";
    ToolpathCache other;
    CHECK(other.load(saved) == 1);
    saved["format"] = "something.else";
    CHECK(ToolpathCache().load(saved) == 0);
}
