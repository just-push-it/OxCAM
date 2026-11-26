// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ocam/foundation/Error.hpp>
#include <ocam/tooling/ToolLibrary.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <filesystem>
#include <vector>

using namespace ocam;
using namespace ocam::tooling;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

ToolLibrary sampleLibrary() {
    ToolLibrary lib;
    lib.name = "Test library";
    Holder holder;
    holder.name = "ER20 collet chuck";
    holder.vendor = "Generic";
    holder.partNumber = "ER20-70";
    holder.profile = geom2d::Polyline(
        {{{0, 0}, 0.0}, {{8, 0}, 0.0}, {{17, 30}, 0.2}, {{20, 45}, 0.0}, {{0, 45}, 0.0}}, false);
    lib.holders.push_back(holder);

    int number = 1;
    for (ToolType type : allToolTypes()) {
        const int toolNumber = number++;
        Tool t = makeDefaultTool(type, toolNumber, 6.0 + toolNumber);
        t.comment = "made by test";
        t.vendor = "Generic";
        t.partNumber = "P-" + std::to_string(number);
        t.holderId = holder.id;
        t.stickout = isTurningTool(type) ? 0.0 : t.mill.overallLength / 2.0;
        t.mill.leftHand = type == ToolType::Tap;
        if (type == ToolType::Probe) {
            t.mill.triggerOffset = 0.0125; // read back as written
        }
        CuttingPreset p;
        p.name = "Aluminium";
        p.materialGroup = "N";
        p.surfaceSpeed = 5000.0;
        p.feedPerTooth = 0.04;
        p.axialDepth = 3.0;
        p.radialDepth = 1.5;
        p.coolant = Coolant::Mist;
        t.presets.insert(t.presets.begin(), p);
        lib.tools.push_back(t);
    }
    return lib;
}

ErrorCode loadError(const Json& doc) {
    try {
        (void)toolLibraryFromJson(doc, "doc");
    } catch (const Error& e) {
        return e.code();
    }
    return ErrorCode::Internal;
}

} // namespace

TEST_CASE("ISO turning holders in library files", "[tooling][io]") {
    ToolLibrary lib;
    lib.holders.push_back(makeTurningHolder(*parseDesignation("S25T-PCLNR 12")));
    lib.holders.back().iso->f = 18.5;
    Tool t = makeDefaultTool(ToolType::TurningInsert, 1);
    t.holderId = lib.holders.back().id;
    fitToHolder(t, lib.holders.back());
    lib.tools.push_back(t);
    for (const LengthUnit unit : {LengthUnit::Millimeter, LengthUnit::Inch}) {
        const ToolLibrary back = toolLibraryFromJson(toJson(lib, unit));
        REQUIRE(back.holders.size() == 1);
        REQUIRE(back.holders[0].iso);
        CHECK(back.holders[0].iso->boringBar);
        CHECK(back.holders[0].iso->style == 'L');
        CHECK(std::abs(back.holders[0].iso->f - 18.5) < 1e-9);
        CHECK(std::abs(back.tools[0].insert.cuttingEdgeAngle - t.insert.cuttingEdgeAngle) < 1e-9);
    }
    // Absent in other holders and tools: they read as before.
    ToolLibrary plain;
    plain.holders.push_back(makeHolder("collet", {}));
    plain.tools.push_back(makeDefaultTool(ToolType::TurningInsert, 1));
    const Json j = toJson(plain, LengthUnit::Millimeter);
    CHECK_FALSE(j.at("holders").at(0).contains("iso"));
    CHECK_FALSE(j.at("tools").at(0).at("insert").contains("cutting_edge_angle_deg"));
}

TEST_CASE("library lookups and validation", "[tooling][library]") {
    ToolLibrary lib = sampleLibrary();
    CHECK(lib.validate().empty());
    CHECK(lib.findToolByNumber(3) != nullptr);
    CHECK(lib.findTool(lib.tools[2].id) == &lib.tools[2]);
    CHECK(lib.findHolder(lib.holders[0].id) != nullptr);
    CHECK(lib.nextFreeNumber() == static_cast<int>(lib.tools.size()) + 1);

    lib.tools[1].number = lib.tools[0].number;
    lib.tools[2].holderId = Uuid::generate();
    lib.tools.push_back(lib.tools[3]);
    const auto issues = lib.validate();
    CHECK(issues.size() >= 4);
}

TEST_CASE("millimetre round-trip is exact", "[tooling][library]") {
    const ToolLibrary lib = sampleLibrary();
    const Json doc = toJson(lib);
    CHECK(doc.at("format") == "oxcam.toollib");
    CHECK(doc.at("version") == kToolLibraryVersion);
    const ToolLibrary back = toolLibraryFromJson(doc);
    CHECK(back == lib);
    CHECK(dumpJson(toJson(back)) == dumpJson(doc)); // deterministic output
}

TEST_CASE("inch files convert lengths and surface speed", "[tooling][library]") {
    const ToolLibrary lib = sampleLibrary();
    const Json doc = toJson(lib, LengthUnit::Inch);
    CHECK(doc.at("units") == "in");
    const Json& t0 = doc.at("tools").at(0);
    CHECK_THAT(t0.at("geometry").at("diameter").get<double>(),
               WithinRel(lib.tools[0].mill.diameter / 25.4, 1e-12));
    // 5000 mm/s = 300 m/min = 984.25 ft/min
    CHECK_THAT(t0.at("presets").at(0).at("surface_speed").get<double>(),
               WithinRel(5000.0 * 60.0 / 304.8, 1e-12));

    const ToolLibrary back = toolLibraryFromJson(doc);
    REQUIRE(back.tools.size() == lib.tools.size());
    CHECK_THAT(back.tools[0].mill.diameter, WithinRel(lib.tools[0].mill.diameter, 1e-12));
    CHECK_THAT(back.tools[0].presets[0].surfaceSpeed, WithinRel(5000.0, 1e-12));
    CHECK_THAT(back.holders[0].profile.vertices()[2].p.x(), WithinRel(17.0, 1e-12));
}

TEST_CASE("hand-written minimal file uses defaults", "[tooling][library]") {
    const Json doc = Json::parse(R"({
        "format": "oxcam.toollib", "version": 1, "units": "in",
        "tools": [{
            "id": "00000000-0000-4000-8000-000000000001", "number": 4, "type": "flat_end_mill",
            "geometry": {"diameter": 0.5, "flute_length": 1.0, "overall_length": 3.0},
            "presets": [{"id": "00000000-0000-4000-8000-000000000002", "surface_speed": 600}]
        }],
        "holders": []
    })");
    const ToolLibrary lib = toolLibraryFromJson(doc);
    REQUIRE(lib.tools.size() == 1);
    const Tool& t = lib.tools[0];
    CHECK_THAT(t.mill.diameter, WithinRel(12.7, 1e-12));
    CHECK(t.mill.flutes == 2);
    CHECK_FALSE(t.holderId.has_value());
    CHECK_THAT(t.presets[0].surfaceSpeed, WithinRel(600.0 * 304.8 / 60.0, 1e-12));
    CHECK(t.presets[0].coolant == Coolant::Flood);
    CHECK(t.validate().empty());
}

TEST_CASE("plunge feeds: inch files, and shares of the feed in older files", "[tooling][library]") {
    const Json doc = Json::parse(R"({
        "format": "oxcam.toollib", "version": 1, "units": "in",
        "tools": [{
            "id": "00000000-0000-4000-8000-000000000001", "number": 4, "type": "flat_end_mill",
            "geometry": {"diameter": 0.5, "flute_length": 1.0, "overall_length": 3.0, "flutes": 3},
            "presets": [
                {"id": "00000000-0000-4000-8000-000000000002", "surface_speed": 600,
                 "feed_per_tooth": 0.002, "plunge_feed": 12, "ramp_feed": 20, "lead_feed": 25},
                {"id": "00000000-0000-4000-8000-000000000003", "surface_speed": 600,
                 "feed_per_tooth": 0.002, "plunge_feed_factor": 0.3},
                {"id": "00000000-0000-4000-8000-000000000004", "surface_speed": 600,
                 "feed_per_tooth": 0.002}
            ]
        }],
        "holders": []
    })");
    const ToolLibrary lib = toolLibraryFromJson(doc);
    const Tool& t = lib.tools.at(0);
    CHECK_THAT(t.presets[0].plungeFeed, WithinRel(12.0 * 25.4 / 60.0, 1e-12)); // in/min
    const double vf = t.presets[1].feedRate(t.mill.diameter, t.mill.flutes);
    CHECK_THAT(t.presets[1].plungeFeed, WithinRel(0.3 * vf, 1e-12));
    CHECK(t.presets[2].plungeFeed == 0.0); // not given: nothing is assumed (ADR-0004)
    // Ramp feeds: as given; files from before they had their own ramped at the cutting feed.
    CHECK_THAT(t.presets[0].rampFeed, WithinRel(20.0 * 25.4 / 60.0, 1e-12));
    CHECK_THAT(t.presets[1].rampFeed, WithinRel(vf, 1e-12));
    CHECK_THAT(t.presets[2].rampFeed, WithinRel(vf, 1e-12));
    // Lead feeds the same way.
    CHECK_THAT(t.presets[0].leadFeed, WithinRel(25.0 * 25.4 / 60.0, 1e-12));
    CHECK_THAT(t.presets[1].leadFeed, WithinRel(vf, 1e-12));
    // Written back as feed rates in the file's unit.
    const Json out = toJson(lib, LengthUnit::Inch);
    const Json& presets = out.at("tools").at(0).at("presets");
    CHECK_THAT(presets.at(0).at("plunge_feed").get<double>(), WithinRel(12.0, 1e-12));
    CHECK_THAT(presets.at(0).at("ramp_feed").get<double>(), WithinRel(20.0, 1e-12));
    CHECK_THAT(presets.at(0).at("lead_feed").get<double>(), WithinRel(25.0, 1e-12));
    CHECK_FALSE(presets.at(1).contains("plunge_feed_factor"));

    // New tools' default presets plunge at half the cutting feed (drills at all of it).
    const Tool mill = makeDefaultTool(ToolType::FlatEndMill, 1, 10.0);
    CHECK(mill.presets.at(0).name == "Default");
    CHECK_THAT(mill.presets[0].plungeFeed,
               WithinAbs(0.5 * mill.presets[0].feedRate(10.0, mill.mill.flutes), 0.5 / 60.0));
    const Tool drill = makeDefaultTool(ToolType::Drill, 2, 8.0);
    CHECK_THAT(drill.presets[0].plungeFeed,
               WithinAbs(drill.presets[0].feedRate(8.0, drill.mill.flutes), 0.5 / 60.0));
}

TEST_CASE("a part-off's end feed in tool library files", "[tooling][library]") {
    ToolLibrary lib;
    lib.tools = {makeDefaultTool(ToolType::PartingInsert, 1),
                 makeDefaultTool(ToolType::GroovingInsert, 2),
                 makeDefaultTool(ToolType::TurningInsert, 3)};
    lib.tools[0].presets[0].partOffEndFeed = 0.012;
    lib.tools[1].presets[0].partOffEndFeed = 0.0;
    Json j = toJson(lib);
    SECTION("the value and a 0 stay as they are") {
        const ToolLibrary back = toolLibraryFromJson(j);
        CHECK_THAT(back.tools[0].presets[0].partOffEndFeed, WithinRel(0.012, 1e-12));
        CHECK(back.tools[1].presets[0].partOffEndFeed == 0.0);
    }
    SECTION("older files ran the end at half the feed per revolution") {
        for (Json& t : j["tools"]) {
            t["presets"][0].erase("part_off_end_feed");
        }
        const ToolLibrary back = toolLibraryFromJson(j);
        CHECK_THAT(back.tools[0].presets[0].partOffEndFeed, WithinRel(0.025, 1e-12));
        CHECK_THAT(back.tools[1].presets[0].partOffEndFeed, WithinRel(0.025, 1e-12));
        CHECK(back.tools[2].presets[0].partOffEndFeed == 0.0);
    }
    SECTION("negative is refused") {
        j["tools"][0]["presets"][0]["part_off_end_feed"] = -1.0;
        CHECK(loadError(j) != ErrorCode::Internal);
    }
}

TEST_CASE("turning's lead feed in tool library files", "[tooling][library]") {
    ToolLibrary lib;
    lib.tools = {makeDefaultTool(ToolType::TurningInsert, 1),
                 makeDefaultTool(ToolType::TurningInsert, 2),
                 makeDefaultTool(ToolType::PartingInsert, 3)};
    lib.tools[0].presets[0].leadFeedPerRev = 0.12;
    lib.tools[1].presets[0].leadFeedPerRev = 0.0;
    Json j = toJson(lib);
    SECTION("the value and a 0 stay as they are") {
        const ToolLibrary back = toolLibraryFromJson(j);
        CHECK_THAT(back.tools[0].presets[0].leadFeedPerRev, WithinRel(0.12, 1e-12));
        CHECK(back.tools[1].presets[0].leadFeedPerRev == 0.0);
    }
    SECTION("older files led at the feed per revolution") {
        for (Json& t : j["tools"]) {
            t["presets"][0].erase("lead_feed_per_rev");
        }
        const ToolLibrary back = toolLibraryFromJson(j);
        CHECK_THAT(back.tools[0].presets[0].leadFeedPerRev, WithinRel(0.2, 1e-12));
        CHECK(back.tools[2].presets[0].leadFeedPerRev == 0.0);
    }
    SECTION("negative is refused") {
        j["tools"][0]["presets"][0]["lead_feed_per_rev"] = -1.0;
        CHECK(loadError(j) != ErrorCode::Internal);
    }
}

TEST_CASE("an ISO holder's custom values in tool library files", "[tooling][library]") {
    TurningHolderCode c = *parseDesignation("PCLNR 2525M 12");
    c.style = kCustomStyle;
    c.customStyle = 70.0;
    c.clearance = kCustomClearance;
    c.customClearance = 11.0;
    c.length = kCustomLength;
    c.customLength = 175.0;
    ToolLibrary lib;
    lib.holders = {makeTurningHolder(c), makeTurningHolder(*parseDesignation("PCLNR 2525M 12"))};
    const Json j = toJson(lib);
    CHECK_FALSE(j["holders"][1]["iso"].contains("custom_style_deg"));
    const ToolLibrary back = toolLibraryFromJson(j);
    CHECK(back.holders[0].iso == lib.holders[0].iso);
    CHECK(back.holders[0].profile == lib.holders[0].profile);
    CHECK(back.holders[1].iso == lib.holders[1].iso);
}

TEST_CASE("an insert's locking style in tool library files", "[tooling][library]") {
    ToolLibrary lib;
    lib.tools = {makeDefaultTool(ToolType::TurningInsert, 1),
                 makeDefaultTool(ToolType::TurningInsert, 2)};
    lib.tools[0].insert.locking = InsertLocking::CountersunkHole;
    Json j = toJson(lib);
    // An older file has none, and neither does an insert with none set.
    CHECK_FALSE(j["tools"][1]["insert"].contains("locking"));
    const ToolLibrary back = toolLibraryFromJson(j);
    CHECK(back.tools[0].insert.locking == InsertLocking::CountersunkHole);
    CHECK(back.tools[1].insert.locking == InsertLocking::NotSet);
    j["tools"][0]["insert"]["locking"] = "glue";
    CHECK(loadError(j) == ErrorCode::Parse);
}

TEST_CASE("invalid tool library documents are rejected", "[tooling][library]") {
    const Json good = toJson(sampleLibrary());
    CHECK(loadError(good) == ErrorCode::Internal); // i.e. no error

    Json badType = good;
    badType["tools"][0]["type"] = "spork";
    CHECK(loadError(badType) == ErrorCode::Parse);

    Json negative = good;
    negative["tools"][0]["geometry"]["diameter"] = -3;
    CHECK(loadError(negative) == ErrorCode::Parse);

    Json unknownKey = good;
    unknownKey["tools"][0]["colour"] = "blue";
    CHECK(loadError(unknownKey) == ErrorCode::Parse);

    Json missing = good;
    missing.erase("tools");
    CHECK(loadError(missing) == ErrorCode::Parse);

    Json badUuid = good;
    badUuid["tools"][0]["id"] = "not-a-uuid";
    CHECK(loadError(badUuid) == ErrorCode::Parse);

    Json future = good;
    future["version"] = kToolLibraryVersion + 1;
    CHECK(loadError(future) == ErrorCode::Unsupported);

    Json wrongFormat = good;
    wrongFormat["format"] = "oxcam.project";
    CHECK(loadError(wrongFormat) == ErrorCode::Parse);

    try {
        (void)toolLibraryFromJson(negative, "lib.ocamtools");
    } catch (const Error& e) {
        CHECK_THAT(std::string(e.what()), ContainsSubstring("lib.ocamtools"));
        CHECK_THAT(std::string(e.what()), ContainsSubstring("/tools/0/geometry/diameter"));
    }
}

TEST_CASE("holder kinds: written for turning holders, migrated in older files",
          "[tooling][library]") {
    ToolLibrary lib = sampleLibrary();
    Holder block = makeHolder("Turret block", {});
    block.kind = HolderKind::Turning;
    lib.holders.push_back(block);
    const Json doc = toJson(lib);
    CHECK_FALSE(doc.at("holders").at(0).contains("kind")); // milling: as before
    CHECK(doc.at("holders").at(1).at("kind") == "turning");
    CHECK(toolLibraryFromJson(doc) == lib);

    // Files written before holders had a kind: a holder only turning tools use is a turning
    // holder; one used by milling tools too, or by none, a milling holder.
    const Json old = Json::parse(R"({
        "format": "oxcam.toollib", "version": 1, "units": "mm",
        "tools": [
            {"id": "00000000-0000-4000-8000-000000000001", "number": 1, "type": "turning_insert",
             "holder": "00000000-0000-4000-8000-0000000000a1"},
            {"id": "00000000-0000-4000-8000-000000000002", "number": 2, "type": "parting_insert",
             "holder": "00000000-0000-4000-8000-0000000000a1"},
            {"id": "00000000-0000-4000-8000-000000000003", "number": 3, "type": "turning_insert",
             "holder": "00000000-0000-4000-8000-0000000000a2"},
            {"id": "00000000-0000-4000-8000-000000000004", "number": 4, "type": "drill",
             "holder": "00000000-0000-4000-8000-0000000000a2"}
        ],
        "holders": [
            {"id": "00000000-0000-4000-8000-0000000000a1", "profile": [[0,0],[20,0],[20,30],[0,30]]},
            {"id": "00000000-0000-4000-8000-0000000000a2", "profile": [[0,0],[20,0],[20,30],[0,30]]},
            {"id": "00000000-0000-4000-8000-0000000000a3", "profile": [[0,0],[20,0],[20,30],[0,30]]}
        ]
    })");
    const ToolLibrary migrated = toolLibraryFromJson(old);
    REQUIRE(migrated.holders.size() == 3);
    CHECK(migrated.holders[0].kind == HolderKind::Turning);
    CHECK(migrated.holders[1].kind == HolderKind::Milling);
    CHECK(migrated.holders[2].kind == HolderKind::Milling);
    // The tools keep their holders, fitting or not.
    CHECK(migrated.tools[3].holderId == migrated.holders[1].id);
    CHECK(toolLibraryFromJson(toJson(migrated)) == migrated);

    Json badKind = doc;
    badKind["holders"][1]["kind"] = "boring";
    CHECK(loadError(badKind) == ErrorCode::Parse);
}

TEST_CASE("tool list filters", "[tooling][library]") {
    const ToolLibrary lib = sampleLibrary();
    auto shown = [&](const ToolFilter& f) {
        std::vector<int> numbers;
        for (const Tool& t : lib.tools) {
            if (f.matches(t)) {
                numbers.push_back(t.number);
            }
        }
        return numbers;
    };
    CHECK(ToolFilter{}.empty());
    CHECK(shown({}).size() == lib.tools.size());

    ToolFilter turning;
    turning.family = ToolFamily::Turning;
    CHECK_FALSE(turning.empty());
    for (int n : shown(turning)) {
        const ToolType type = lib.findToolByNumber(n)->type;
        CHECK((isTurningTool(type) || type == ToolType::BarPuller)); // a lathe's (DL-0091)
    }
    CHECK(shown(turning).size() == toolTypesOf(ToolFamily::Turning).size());

    ToolFilter drills;
    drills.type = ToolType::Drill;
    REQUIRE(shown(drills).size() == 1);
    CHECK(lib.findToolByNumber(shown(drills)[0])->type == ToolType::Drill);

    // Diameter range: inclusive, and tools without a diameter drop out.
    const Tool& t3 = *lib.findToolByNumber(3);
    ToolFilter size;
    size.minDiameter = t3.mill.diameter;
    size.maxDiameter = t3.mill.diameter;
    CHECK(shown(size) == std::vector<int>{3});
    size.maxDiameter = 0.0; // no upper limit
    for (int n : shown(size)) {
        const Tool& t = *lib.findToolByNumber(n);
        CHECK_FALSE(isTurningTool(t.type));
        CHECK(t.mill.diameter >= t3.mill.diameter);
    }

    // Text: the name, vendor or part number (any case), or the tool number.
    ToolFilter text;
    text.text = "T4";
    CHECK(shown(text) == std::vector<int>{4});
    text.text = " 4 "; // also the names with a 4 in them
    CHECK(std::ranges::count(shown(text), 4) == 1);
    text.text = "p-5";
    CHECK(shown(text) == std::vector<int>{4}); // part number "P-5"
    text.text = "GENERIC";
    CHECK(shown(text).size() == lib.tools.size());
    text.text = "no such tool";
    CHECK(shown(text).empty());
    text.text = "generic";
    text.family = ToolFamily::HoleMaking;
    CHECK(shown(text).size() == toolTypesOf(ToolFamily::HoleMaking).size());
}

TEST_CASE("save and load a library file", "[tooling][library]") {
    const auto path = std::filesystem::temp_directory_path() / "oxcam_test.ocamtools";
    const ToolLibrary lib = sampleLibrary();
    saveToolLibrary(lib, path);
    CHECK(loadToolLibrary(path) == lib);
    std::filesystem::remove(path);
}

TEST_CASE("bundled sample library is valid", "[tooling][library]") {
    const ToolLibrary lib = loadToolLibrary(OCAM_SAMPLE_TOOL_LIBRARY);
    CHECK(lib.tools.size() >= 5);
    CHECK(lib.validate().empty());
}

TEST_CASE("a bar puller and its pull feed in tool library files", "[tooling][library]") {
    ToolLibrary lib;
    lib.tools = {makeDefaultTool(ToolType::BarPuller, 7, 40.0)};
    lib.tools[0].presets[0].pullFeed = 1500.0 / 60.0;
    Json j = toJson(lib);
    CHECK(j["tools"][0]["type"] == "bar_puller");
    SECTION("the type, the largest bar and the feed come back") {
        const ToolLibrary back = toolLibraryFromJson(j);
        CHECK(back.tools[0].type == ToolType::BarPuller);
        CHECK_THAT(back.tools[0].mill.tipDiameter, WithinRel(40.0, 1e-12));
        CHECK_THAT(back.tools[0].presets[0].pullFeed, WithinRel(25.0, 1e-12));
    }
    SECTION("older files have no pull feed: 0, never a guess") {
        j["tools"][0]["presets"][0].erase("pull_feed");
        CHECK(toolLibraryFromJson(j).tools[0].presets[0].pullFeed == 0.0);
    }
    SECTION("negative is refused") {
        j["tools"][0]["presets"][0]["pull_feed"] = -1.0;
        CHECK(loadError(j) != ErrorCode::Internal);
    }
}
