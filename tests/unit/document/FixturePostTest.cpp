// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Every fixture project, posted with the JavaScript built-in post for its machine: the program
// back-plots onto the toolpaths it came from, as `oxcam-cli project post --verify` checks.
// (Until DL-0039 step 5 removed it, the C++ formatter wrote the same programs byte for byte.)
// One test case per project, so they run in parallel.

#include <ocam/document/Document.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/post/Engine.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <set>
#include <sstream>

using namespace ocam;
using namespace ocam::document;

namespace {

const char* const kProjects[] = {
    "backturn_project.ocam",
    "boss_thread_project.ocam",
    "bored_project.ocam",
    "centres_project.ocam",
    "chuck_project.ocam",
    "dome5_machine_project.ocam",
    "dome5_project.ocam",
    "dome_project.ocam",
    "dome_steep_shallow_project.ocam",
    "fillet_pocket_project.ocam",
    "fixture_project.ocam",
    "grooved_project.ocam",
    "flip_project.ocam",
    "nose_project.ocam",
    "plate32_machine_project.ocam",
    "plate32_project.ocam",
    "ranges_project.ocam",
    "rotary_project.ocam",
    "sample_project.ocam",
    "shaft_project.ocam",
    "threads_project.ocam",
    "trace_project.ocam",
    "vise_origin_project.ocam",
    "vise_project.ocam",
    "holders_project.ocam",
    "live_project.ocam",
    "millturn_project.ocam",
    "wrapped_project.ocam",
    "turret_project.ocam",
    "turret_tailstock_project.ocam",
    "turret_collisions_project.ocam",
    "turret_full_project.ocam",
    "turret_live_project.ocam",
    "turret_sub_project.ocam",
    "turret_blocks_project.ocam",
    "swarf_project.ocam",
};

/// Posts every setup of a fixture project with its machine's built-in post (or `with`, for the
/// setups whose machine it is for) and back-plots the program onto the toolpaths; returns the
/// posts used. `programs` receives the programs.
std::set<std::string> comparePosts(const char* file, const post::PostScript* with = nullptr,
                                   std::string* programs = nullptr) {
    const Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / file);
    std::set<std::string> posts;
    for (const Setup& setup : doc.project().setups) {
        CAPTURE(file, setup.name);
        const machine::Machine& machine = machineFor(doc.project(), setup);
        if (with && std::ranges::find(with->manifest().machines,
                                      std::string(machine::toString(machine.kinematics))) ==
                        with->manifest().machines.end()) {
            continue;
        }
        const post::PostScript& script = with ? *with : defaultPostFor(machine);
        CancelToken cancel;
        NullProgress progress;
        const PostOutput out = postSetup(doc, setup.id, machine, script, cancel, progress);
        if (!out.ok()) {
            continue; // setups that cannot be posted
        }
        REQUIRE(script.manifest().verify);
        const bool lathe = *script.manifest().verify == "fanuc-lathe";
        post::BackplotOptions options;
        options.lathe = lathe;
        options.machine = &machine;
        const post::BackplotResult plotted = post::backplot(out.gcode, options);
        // 3 decimals: end points and arc centres within one and a half units.
        CHECK(post::compareBackplot(out.toolpaths, plotted.path, 1.5e-3, nullptr, !lathe)
                  .value_or("") == "");
        posts.insert(script.manifest().id);
        if (programs) {
            *programs += out.gcode;
        }
    }
    return posts;
}

} // namespace

TEST_CASE("the fixture list covers every fixture project", "[document][post][builtin]") {
    std::set<std::string> files;
    for (const auto& entry : std::filesystem::directory_iterator(OCAM_FIXTURE_DIR)) {
        if (entry.path().extension() == ".ocam") {
            files.insert(entry.path().filename().string());
        }
    }
    CHECK(files == std::set<std::string>(std::begin(kProjects), std::end(kProjects)));
}

TEST_CASE("fixture posts: on a lathe's turret", "[document][post][builtin]") {
    const std::set<std::string> lathe{"fanuc-generic-lathe"};
    CHECK(comparePosts("turret_project.ocam") == lathe);
    CHECK(comparePosts("turret_tailstock_project.ocam") == lathe);
    CHECK(comparePosts("turret_blocks_project.ocam") == lathe);
}
TEST_CASE("fixture posts: trace", "[document][post][builtin]") {
    CHECK(comparePosts("trace_project.ocam") == std::set<std::string>{"fanuc-generic-mill"});
}
TEST_CASE("fixture posts: chuck", "[document][post][builtin]") {
    CHECK(comparePosts("chuck_project.ocam") == std::set<std::string>{"fanuc-generic-lathe"});
}
TEST_CASE("fixture posts: dome5 machine", "[document][post][builtin]") {
    CHECK(comparePosts("dome5_machine_project.ocam") ==
          std::set<std::string>{"fanuc-generic-mill"});
}
TEST_CASE("fixture posts: dome5", "[document][post][builtin]") {
    CHECK(comparePosts("dome5_project.ocam") == std::set<std::string>{"fanuc-generic-mill"});
}
TEST_CASE("fixture posts: swarf", "[document][post][builtin]") {
    CHECK(comparePosts("swarf_project.ocam") == std::set<std::string>{"fanuc-generic-mill"});
}
TEST_CASE("fixture posts: dome", "[document][post][builtin]") {
    CHECK(comparePosts("dome_project.ocam") == std::set<std::string>{"fanuc-generic-mill"});
}
TEST_CASE("fixture posts: fixture", "[document][post][builtin]") {
    CHECK(comparePosts("fixture_project.ocam") == std::set<std::string>{"fanuc-generic-mill"});
}
TEST_CASE("fixture posts: plate32 machine", "[document][post][builtin]") {
    CHECK(comparePosts("plate32_machine_project.ocam") ==
          std::set<std::string>{"fanuc-generic-mill"});
}
TEST_CASE("fixture posts: plate32", "[document][post][builtin]") {
    CHECK(comparePosts("plate32_project.ocam") == std::set<std::string>{"fanuc-generic-mill"});
}
TEST_CASE("fixture posts: rotary", "[document][post][builtin]") {
    CHECK(comparePosts("rotary_project.ocam") == std::set<std::string>{"fanuc-generic-mill-4axis"});
}
TEST_CASE("fixture posts: an external thread milled around a boss", "[document][post][builtin]") {
    CHECK(comparePosts("boss_thread_project.ocam") == std::set<std::string>{"fanuc-generic-mill"});
}
TEST_CASE("fixture posts: sample", "[document][post][builtin]") {
    CHECK(comparePosts("sample_project.ocam") == std::set<std::string>{"fanuc-generic-mill"});
}
TEST_CASE("fixture posts: shaft", "[document][post][builtin]") {
    CHECK(comparePosts("shaft_project.ocam") == std::set<std::string>{"fanuc-generic-lathe"});
    CHECK(comparePosts("bored_project.ocam") == std::set<std::string>{"fanuc-generic-lathe"});
    CHECK(comparePosts("holders_project.ocam") == std::set<std::string>{"fanuc-generic-lathe"});
    // Live tooling posts with a post that has the machine's codes.
    CHECK(comparePosts("millturn_project.ocam", &post::builtinPost("haas-lathe")) ==
          std::set<std::string>{"haas-lathe"});
    CHECK(comparePosts("live_project.ocam", &post::builtinPost("haas-lathe")) ==
          std::set<std::string>{"haas-lathe"});
    CHECK(comparePosts("wrapped_project.ocam", &post::builtinPost("haas-lathe")) ==
          std::set<std::string>{"haas-lathe"});
    CHECK(comparePosts("flip_project.ocam") == std::set<std::string>{"fanuc-generic-lathe"});
    CHECK(comparePosts("centres_project.ocam") == std::set<std::string>{"fanuc-generic-lathe"});
    CHECK(comparePosts("nose_project.ocam") == std::set<std::string>{"fanuc-generic-lathe"});
    CHECK(comparePosts("backturn_project.ocam") == std::set<std::string>{"fanuc-generic-lathe"});
    CHECK(comparePosts("ranges_project.ocam") == std::set<std::string>{"fanuc-generic-lathe"});
    CHECK(comparePosts("grooved_project.ocam") == std::set<std::string>{"fanuc-generic-lathe"});
    CHECK(comparePosts("threads_project.ocam") == std::set<std::string>{"fanuc-generic-lathe"});
}
TEST_CASE("fixture posts: vise origin", "[document][post][builtin]") {
    CHECK(comparePosts("vise_origin_project.ocam") == std::set<std::string>{"fanuc-generic-mill"});
}
TEST_CASE("fixture posts: vise", "[document][post][builtin]") {
    CHECK(comparePosts("vise_project.ocam") == std::set<std::string>{"fanuc-generic-mill"});
}

// The Haas-style post (no tool centre point control) on the mill fixtures: 3-axis, 3+2, 4-axis
// and 5-axis machine positions with inverse-time feeds.

namespace {

std::string haasPrograms(const char* file) {
    std::string programs;
    CHECK(comparePosts(file, &post::builtinPost("haas-classic-mill"), &programs) ==
          std::set<std::string>{"haas-classic-mill"});
    return programs;
}

} // namespace

TEST_CASE("Haas-style post: sample", "[document][post][builtin][haas]") {
    const std::string text = haasPrograms("sample_project.ocam");
    CHECK(text.starts_with("%\nO0"));
    CHECK(text.find("G93") == std::string::npos);
}
TEST_CASE("Haas-style post: plate32 (3+2)", "[document][post][builtin][haas]") {
    // Indexing without a tool change: Z returns with G28, which cancels the tool length offset,
    // so the approach applies it again.
    const std::string text = haasPrograms("plate32_project.ocam");
    const auto home = text.find("G91 G28 Z0.\nG90\n");
    REQUIRE(home != std::string::npos);
    const auto again = text.find("G43 H", home);
    CHECK(again != std::string::npos);
}
TEST_CASE("Haas-style post: plate32 machine", "[document][post][builtin][haas]") {
    (void)haasPrograms("plate32_machine_project.ocam");
}
TEST_CASE("Haas-style post: rotary (4-axis)", "[document][post][builtin][haas]") {
    const std::string text = haasPrograms("rotary_project.ocam");
    CHECK(text.find("G43.4") == std::string::npos);
}
TEST_CASE("Haas-style post: dome5 needs tool centre point control",
          "[document][post][builtin][haas]") {
    // The finishing pass crosses the singular tool axis at the top of the dome: as machine
    // positions the table swings half a turn within a short move, far above the machine's feed.
    // Posting refuses rather than writing it.
    const Document doc =
        Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "dome5_project.ocam");
    const Setup& setup = doc.project().setups.front();
    CancelToken cancel;
    NullProgress progress;
    const PostOutput out = postSetup(doc, setup.id, machineFor(doc.project(), setup),
                                     post::builtinPost("haas-classic-mill"), cancel, progress);
    REQUIRE_FALSE(out.ok());
    CHECK(out.gcode.empty());
    CHECK(out.errors.front().find("above the machine's max feed") != std::string::npos);
}

TEST_CASE("a machine's default post and option values; the post's work offsets",
          "[document][post]") {
    Document doc = Document::open(std::filesystem::path(OCAM_FIXTURE_DIR) / "sample_project.ocam");
    const Uuid sid = doc.project().setups.front().id;
    machine::Machine haasMachine = machineFor(doc.project(), doc.project().setups.front());
    haasMachine.id = "my-haas";
    haasMachine.name = "My Haas";
    haasMachine.post = "haas-classic-mill";
    haasMachine.postOptions = {{"units", "inch"}, {"optionalStop", false}};
    haasMachine.workOffsets = 16; // G54-G59 and G154 P1-P10
    doc.modify("machine", [&](Project& p) {
        p.machines.push_back(haasMachine);
        p.findSetup(sid)->machine = haasMachine.id;
    });
    auto setup = [&]() -> const Setup& {
        return *doc.project().findSetup(sid);
    };
    const machine::Machine machine = machineFor(doc.project(), setup());
    const post::PostScript haas = postFor(doc.project(), setup(), machine);
    CHECK(haas.manifest().id == "haas-classic-mill");

    // The machine's values, under the setup's; another post takes neither.
    Json options = postOptionsFor(doc.project(), setup(), machine, haas);
    CHECK(options["program"]["units"] == "inch");
    CHECK(options["program"]["optionalStop"] == false);
    doc.modify("units", [&](Project& p) {
        p.findSetup(sid)->postOptions = {{"program", {{"units", "mm"}}}};
    });
    options = postOptionsFor(doc.project(), setup(), machine, haas);
    CHECK(options["program"]["units"] == "mm");
    CHECK(options["program"]["optionalStop"] == false);
    CHECK(postOptionsFor(doc.project(), setup(), machine, post::builtinPost("fanuc-generic-mill"))
              .at("program")
              .empty());
    CancelToken cancel;
    NullProgress progress;
    PostOutput out = postSetup(doc, sid, machine, cancel, progress);
    REQUIRE(out.ok());
    CHECK(out.gcode.find(" G21 ") != std::string::npos);
    CHECK(out.gcode.find("M01") == std::string::npos);

    // Work offsets as the post writes them, as many as the machine has.
    CHECK(workOffsetName(doc.project(), setup(), 8) == "G154 P2");
    CHECK(lastWorkOffset(haas.manifest(), machine) == 16);
    doc.modify("offset", [&](Project& p) { p.findSetup(sid)->workOffset = 17; });
    out = postSetup(doc, sid, machine, cancel, progress);
    REQUIRE_FALSE(out.ok());
    CHECK(out.errors.front().find("work offset 17: the machine has 16") != std::string::npos);
    doc.modify("offset", [&](Project& p) { p.findSetup(sid)->workOffset = 12; });
    out = postSetup(doc, sid, machine, cancel, progress);
    REQUIRE(out.ok());
    CHECK(out.gcode.find("G154 P6") != std::string::npos);
}
