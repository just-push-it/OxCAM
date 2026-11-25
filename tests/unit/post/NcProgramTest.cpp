// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// NC program version 2 (DL-0039): operations, the tool table, feed classes and the machine in
// lowering and in the JSON form, and what a JavaScript post receives (program fields, operation
// handlers, program.peek()).

#include "RunPost.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/post/Engine.hpp>
#include <ocam/post/Post.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <filesystem>
#include <map>
#include <set>
#include <utility>

using namespace ocam;
using namespace ocam::post;
using namespace ocam::post::testing;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

toolpath::Toolpath loadGoldenToolpath(const std::string& name) {
    const auto path = std::filesystem::path(OCAM_GOLDEN_DIR) / (name + ".json");
    return toolpath::toolpathFromJson(parseJson(readFileBytes(path), name));
}

toolpath::Move move(toolpath::MoveKind kind, const Vec3& end, double feed = 5.0) {
    toolpath::Move m;
    m.kind = kind;
    m.end = end;
    m.feedRate = feed;
    return m;
}

/// Face, drill, face again with the first tool.
struct ThreeJobs {
    toolpath::Toolpath face = loadGoldenToolpath("facing_rect_zigzag");
    toolpath::Toolpath drill = loadGoldenToolpath("drill_mixed");
    std::vector<PostJob> jobs;

    ThreeJobs() {
        drill.toolNumber = face.toolNumber + 1;
        PostJob a{"Face top", face, "face mill", 1};
        a.comment = "skim 0.5 mm";
        NcTool tool;
        tool.name = "D50 face mill";
        tool.type = "flat_end_mill";
        tool.diameter = 50.0;
        tool.flutes = 5;
        tool.gaugeLength = 120.0;
        a.tool = tool;
        PostJob b{"Drill holes", drill, "drill 6", 2};
        PostJob c{"Face again", face, "face mill", 1};
        jobs = {a, b, c};
    }
};

} // namespace

TEST_CASE("lowering records each operation, the tool table and feed classes", "[post][ncprogram]") {
    const ThreeJobs t;
    const NcProgram program = lower(t.jobs, "part", 1000);

    REQUIRE(program.operations.size() == 3);
    const NcOperation& face = program.operations[0];
    CHECK(face.name == "Face top");
    CHECK(face.comment == "skim 0.5 mm");
    CHECK(face.strategy == t.face.strategy);
    CHECK(face.tool == t.face.toolNumber);
    CHECK(face.coolant == t.face.coolant);
    CHECK(face.workOffset == 1);
    CHECK(face.orientation == NcOrientation::ThreeAxis);
    CHECK(program.operations[1].name == "Drill holes");
    CHECK(program.operations[1].workOffset == 2);
    CHECK(program.operations[1].tool == t.drill.toolNumber);

    // Tools in order of first use, each once; the job's tool data when it has some.
    REQUIRE(program.tools.size() == 2);
    CHECK(program.tools[0].number == t.face.toolNumber);
    CHECK(program.tools[0].name == "D50 face mill");
    CHECK(program.tools[0].type == "flat_end_mill");
    CHECK(program.tools[0].diameter == 50.0);
    CHECK(program.tools[0].gaugeLength == 120.0);
    CHECK(program.tools[1].number == t.drill.toolNumber);
    CHECK(program.tools[1].name == "drill 6");
    CHECK(program.tools[1].type.empty());

    // Begin and end bracket each operation, in order; no operation-name comments any more.
    std::vector<std::pair<NcOp, int>> marks;
    for (const NcInstr& i : program.instrs) {
        CHECK(i.op != NcOp::Comment);
        if (i.op == NcOp::OperationBegin || i.op == NcOp::OperationEnd) {
            marks.emplace_back(i.op, i.operation);
        }
    }
    CHECK(marks == std::vector<std::pair<NcOp, int>>{{NcOp::OperationBegin, 0},
                                                     {NcOp::OperationEnd, 0},
                                                     {NcOp::OperationBegin, 1},
                                                     {NcOp::OperationEnd, 1},
                                                     {NcOp::OperationBegin, 2},
                                                     {NcOp::OperationEnd, 2}});
    CHECK(program.instrs.front().op == NcOp::OperationBegin);

    // Feed moves keep their feed class.
    std::map<toolpath::FeedClass, int> expected;
    for (const toolpath::Move& m : t.face.moves) {
        if (m.kind == toolpath::MoveKind::Linear || m.isArc()) {
            ++expected[m.feedClass];
        }
    }
    std::map<toolpath::FeedClass, int> lowered;
    for (const NcInstr& i : program.instrs) {
        if (i.op == NcOp::OperationBegin && i.operation == 1) {
            break;
        }
        if (i.op == NcOp::Linear || i.op == NcOp::Arc) {
            ++lowered[i.feedClass];
        }
    }
    CHECK(lowered == expected);
}

TEST_CASE("operation bounds include arc extremes and hole bottoms", "[post][ncprogram]") {
    toolpath::Toolpath circle;
    circle.toolNumber = 1;
    circle.start = Vec3(10.0, 0.0, 5.0);
    circle.moves.push_back(move(toolpath::MoveKind::Linear, Vec3(10.0, 0.0, 0.0)));
    toolpath::Move arc = move(toolpath::MoveKind::ArcCCW, Vec3(10.0, 0.0, 0.0));
    arc.center = Vec3::Zero();
    circle.moves.push_back(arc); // a full circle
    const NcProgram program = lower({{"circle", circle, "end mill", 1}}, "bounds", 1);
    REQUIRE(program.operations.size() == 1);
    const NcOperation& op = program.operations[0];
    CHECK((op.boundsMin - Vec3(-10.0, -10.0, 0.0)).norm() < 1e-9);
    CHECK((op.boundsMax - Vec3(10.0, 10.0, 5.0)).norm() < 1e-9);

    const toolpath::Toolpath drill = loadGoldenToolpath("drill_mixed");
    REQUIRE_FALSE(drill.cycles.empty());
    double deepest = drill.cycles.front().bottomZ;
    for (const auto& c : drill.cycles) {
        deepest = std::min(deepest, c.bottomZ);
    }
    const NcProgram drilled = lower({{"drill", drill, "drill", 1}}, "holes", 2);
    CHECK_THAT(drilled.operations.at(0).boundsMin.z(), WithinAbs(deepest, 1e-9));
}

TEST_CASE("indexed and simultaneous operations say so", "[post][ncprogram]") {
    const toolpath::Toolpath face = loadGoldenToolpath("facing_rect_zigzag");
    PostJob side{"side", face, "face mill", 2, machine::IndexAngles{kPi / 2.0, 0.0, 0.0}, "AC"};
    const NcProgram program = lower({side}, "indexed", 3000);
    REQUIRE(program.operations.size() == 1);
    CHECK(program.operations[0].orientation == NcOrientation::Indexed);
    CHECK_THAT(program.operations[0].index.a, WithinAbs(kPi / 2.0, 1e-12));
    const Json doc = toJson(program);
    CHECK(doc.at("operations").at(0).at("orientation") == "indexed");
    CHECK_THAT(doc.at("operations").at(0).at("index").at("a").get<double>(), WithinAbs(90.0, 1e-9));
    const NcProgram back = ncProgramFromJson(doc);
    CHECK(back.operations.at(0).orientation == NcOrientation::Indexed);
    CHECK_THAT(back.operations.at(0).index.a, WithinAbs(kPi / 2.0, 1e-12));
}

TEST_CASE("NC program JSON version 2 round-trips; version 1 is still read", "[post][ncprogram]") {
    const ThreeJobs t;
    NcProgram program = lower(t.jobs, "part", 1000);
    program.project = "Bracket";
    program.setup = "Setup 1";
    program.time = "2026-10-25T12:00:00Z";
    program.machine = ncMachine(machine::genericMill3());

    const Json doc = toJson(program);
    CHECK(doc.at("version") == kNcProgramJsonVersion);
    CHECK(doc.at("units").at("angle") == "deg");
    CHECK(doc.at("project") == "Bracket");
    CHECK(doc.at("setup") == "Setup 1");
    CHECK(doc.at("time") == "2026-10-25T12:00:00Z");
    CHECK(doc.at("tools").at(0).at("diameter") == 50.0);
    CHECK(doc.at("tools").at(0).at("gauge_length") == 120.0);
    CHECK(doc.at("operations").at(0).at("comment") == "skim 0.5 mm");
    CHECK(doc.at("operations").at(1).at("work_offset") == 2);
    CHECK(doc.at("operations").at(0).at("orientation") == "3axis");
    CHECK(doc.at("machine").at("kinematics") == "mill3");
    CHECK_THAT(doc.at("machine").at("spindle").at("max").get<double>(),
               WithinAbs(machine::genericMill3().maxSpindle * 60.0, 1e-9));
    CHECK(doc.at("instructions").at(0) == Json{{"op", "operation_begin"}, {"operation", 0}});
    bool sawFeedClass = false;
    for (const Json& i : doc.at("instructions")) {
        if (i.at("op") == "linear" || i.at("op") == "arc") {
            CHECK(i.contains("feed_class"));
            sawFeedClass = true;
        }
    }
    CHECK(sawFeedClass);

    const NcProgram back = ncProgramFromJson(Json::parse(dumpJson(doc)));
    CHECK(back.project == "Bracket");
    CHECK(back.setup == "Setup 1");
    CHECK(back.time == program.time);
    REQUIRE(back.tools.size() == 2);
    CHECK(back.tools[0].name == "D50 face mill");
    CHECK(back.tools[0].flutes == 5);
    REQUIRE(back.operations.size() == 3);
    CHECK(back.operations[0].comment == "skim 0.5 mm");
    CHECK((back.operations[1].boundsMin - program.operations[1].boundsMin).norm() < 1e-9);
    REQUIRE(back.machine);
    CHECK(back.machine->kinematics == "mill3");
    CHECK(back.machine->axes.size() == machine::genericMill3().axes.size());
    REQUIRE(back.instrs.size() == program.instrs.size());
    for (std::size_t k = 0; k < back.instrs.size(); ++k) {
        CHECK(back.instrs[k].op == program.instrs[k].op);
        CHECK(back.instrs[k].operation == program.instrs[k].operation);
        CHECK(back.instrs[k].feedClass == program.instrs[k].feedClass);
    }

    // The C++ formatter writes each operation's name where the comment instruction was.
    const std::string gcode = runPost(mill(), program);
    CHECK(runPost(mill(), back) == gcode);
    CHECK_THAT(gcode, ContainsSubstring("(FACE TOP)"));
    CHECK_THAT(gcode, ContainsSubstring("(DRILL HOLES)"));

    // A version 1 document: operation names as comments, no feed classes, no program data.
    Json v1 = doc;
    v1["version"] = 1;
    for (const char* key : {"project", "setup", "time", "tools", "operations", "machine"}) {
        v1.erase(key);
    }
    Json instructions = Json::array();
    for (Json i : doc.at("instructions")) {
        if (i.at("op") == "operation_end") {
            continue;
        }
        if (i.at("op") == "operation_begin") {
            i = Json{{"op", "comment"},
                     {"text", program.operations.at(i.at("operation").get<std::size_t>()).name}};
        }
        i.erase("feed_class");
        instructions.push_back(std::move(i));
    }
    v1["instructions"] = std::move(instructions);
    const NcProgram old = ncProgramFromJson(v1);
    CHECK(old.operations.empty());
    CHECK(old.instrs.front().op == NcOp::Comment);
    CHECK(runPost(mill(), old) == gcode);

    Json bad = doc;
    bad["instructions"][0]["operation"] = 7;
    CHECK_THROWS_WITH(ncProgramFromJson(bad), ContainsSubstring("refers to no operation"));
    bad = doc;
    for (Json& i : bad["instructions"]) {
        if (i.contains("feed_class")) {
            i["feed_class"] = "warp";
            break;
        }
    }
    CHECK_THROWS_WITH(ncProgramFromJson(bad), ContainsSubstring("unknown feed class"));
    bad = doc;
    bad["version"] = kNcProgramJsonVersion + 1;
    CHECK_THROWS_WITH(ncProgramFromJson(bad), ContainsSubstring("unsupported version"));
}

TEST_CASE("a post receives the program, its operations and program.peek()", "[post][ncprogram]") {
    NcProgram program;
    program.name = "BRACKET SETUP 1";
    program.number = 1000;
    program.project = "Bracket";
    program.setup = "Setup 1";
    program.time = "2026-10-25T12:00:00Z";
    program.machine = ncMachine(machine::genericMill3());
    NcTool tool;
    tool.number = 1;
    tool.name = "D6 end mill";
    program.tools = {tool};
    NcOperation face;
    face.name = "Face";
    face.tool = 1;
    program.operations = {face};
    NcInstr begin;
    begin.op = NcOp::OperationBegin;
    begin.operation = 0;
    NcInstr plunge;
    plunge.op = NcOp::Linear;
    plunge.target = Vec3(0.0, 0.0, -1.0);
    plunge.feed = 5.0;
    plunge.feedClass = toolpath::FeedClass::Plunge;
    NcInstr cut = plunge;
    cut.target = Vec3(10.0, 0.0, -1.0);
    cut.feedClass = toolpath::FeedClass::Cut;
    NcInstr end;
    end.op = NcOp::OperationEnd;
    end.operation = 0;
    program.instrs = {begin, plunge, cut, end};

    const std::string source = R"js(
import { out } from "oxcam:post";
export const manifest = {
  id: "nc-test", name: "NC test", version: "1.0.0", engine: 1, machines: ["mill3"],
};
export const options = [{ id: "flag", label: "Flag", type: "bool", default: false }];
export default class {
  constructor(context) { this.program = context.program; this.options = context.options; }
  programBegin(p) {
    out.line([p.name, p.number, p.project, p.setup, p.time, p.post.id, p.post.version,
              p.machine.kinematics, p.tools.length, p.tools[0].name, p.operations.length,
              p.operations[0].name, p.options.flag, this.options === p.options,
              typeof p.instructions].join("|"));
    out.line("first " + p.peek(1).op + " " + p.peek(2).op + " " + p.peek().op);
  }
  operationBegin(op) { out.line("begin " + op.name + " " + (op === this.program.operations[0])); }
  linear(i) { out.line(`linear ${i.feed_class} ${i.feed} next ${this.program.peek(1).op}`); }
  operationEnd(op) {
    out.line("end " + op.name + " " + this.program.peek(1));
    try { this.program.peek(0); } catch (e) { out.line(e.message); }
    try { this.program.peek(1.5); } catch (e) { out.line(e.message); }
  }
  programEnd(p) { out.line("after " + p.peek(1)); }
}
)js";
    CancelToken cancel;
    const PostScript script = PostScript::load(source, "post.js", cancel);
    const PostResult result = script.run(program, script.optionValues({{"flag", true}}), cancel);
    CHECK(result.text ==
          "BRACKET SETUP 1|1000|Bracket|Setup 1|2026-10-25T12:00:00Z|nc-test|1.0.0|mill3|1|"
          "D6 end mill|1|Face|true|true|undefined\n"
          "first operation_begin linear operation_begin\n"
          "begin Face true\n"
          "linear plunge 300 next linear\n"
          "linear cut 300 next operation_end\n"
          "end Face null\n"
          "program.peek: expected a whole number of 1 or more\n"
          "program.peek: expected a whole number of 1 or more\n"
          "after null\n");
}

TEST_CASE("declarations reshape lowering: arc planes, helices, sweeps, cycle types",
          "[post][ncprogram][declarations]") {
    // A plunge, a ZX half circle, a helical XY full turn down 2 mm, a flat XY full circle.
    toolpath::Toolpath tp;
    tp.toolNumber = 1;
    tp.start = Vec3(-10.0, 0.0, 5.0);
    tp.moves.push_back(move(toolpath::MoveKind::Linear, Vec3(-10.0, 0.0, 0.0)));
    toolpath::Move zx = move(toolpath::MoveKind::ArcCCW, Vec3(10.0, 0.0, 0.0));
    zx.center = Vec3::Zero();
    zx.plane = toolpath::ArcPlane::ZX;
    tp.moves.push_back(zx);
    toolpath::Move helix = move(toolpath::MoveKind::ArcCCW, Vec3(10.0, 0.0, -2.0));
    helix.center = Vec3(0.0, 0.0, 0.0);
    tp.moves.push_back(helix);
    toolpath::Move circle = move(toolpath::MoveKind::ArcCW, Vec3(10.0, 0.0, -2.0));
    circle.center = Vec3(0.0, 0.0, -2.0);
    tp.moves.push_back(circle);

    auto count = [](const NcProgram& p, NcOp op) {
        return std::ranges::count_if(p.instrs, [&](const NcInstr& i) { return i.op == op; });
    };

    SECTION("by default every arc stays one arc") {
        CHECK(count(lower({{"arcs", tp, "", 1}}, "p", 1), NcOp::Arc) == 3);
    }
    SECTION("arcs in planes the post does not take become lines within the tolerance") {
        LowerOptions options;
        options.arcPlanes = {toolpath::ArcPlane::XY};
        options.tolerance = 0.01;
        const NcProgram p = lower({{"arcs", tp, "", 1}}, "p", 1, options);
        CHECK(count(p, NcOp::Arc) == 2); // the helix and the circle
        // The ZX half circle as lines: every point on the circle, every chord within 0.01 mm.
        Vec3 prev = Vec3(-10.0, 0.0, 0.0);
        int lines = 0;
        for (const NcInstr& i : p.instrs) {
            if (i.op == NcOp::Linear && std::abs(i.target.y()) < 1e-12 && i.target.z() >= 0.0 &&
                prev.z() >= 0.0 && (i.target - Vec3(-10.0, 0.0, 0.0)).norm() > 1e-9) {
                CHECK_THAT(i.target.norm(), WithinAbs(10.0, 1e-9));
                CHECK(10.0 - (0.5 * (prev + i.target)).norm() <= 0.01 + 1e-9);
                CHECK(i.feedClass == toolpath::FeedClass::Cut);
                ++lines;
            }
            if (i.op == NcOp::Linear || i.op == NcOp::Arc) {
                prev = i.target;
            }
        }
        CHECK(lines > 10);
    }
    SECTION("helical arcs become lines when the post takes none") {
        LowerOptions options;
        options.helicalArcs = false;
        const NcProgram p = lower({{"arcs", tp, "", 1}}, "p", 1, options);
        CHECK(count(p, NcOp::Arc) == 2); // the ZX half circle and the flat circle
    }
    SECTION("arcs longer than the post takes are split into equal parts") {
        LowerOptions options;
        options.maxArcSweep = kPi / 2.0;
        const NcProgram p = lower({{"arcs", tp, "", 1}}, "p", 1, options);
        // ZX half circle: 2 parts; helix and circle: 4 parts each.
        CHECK(count(p, NcOp::Arc) == 10);
        for (const NcInstr& i : p.instrs) {
            if (i.op == NcOp::Arc && i.plane == toolpath::ArcPlane::XY) {
                // Every part ends on the circle.
                CHECK_THAT((i.target - i.center).head<2>().norm(), WithinAbs(10.0, 1e-9));
            }
        }
    }
    SECTION("cycle types the post does not take are expanded, the others kept") {
        const toolpath::Toolpath drill = loadGoldenToolpath("drill_mixed");
        std::set<toolpath::CycleType> types;
        for (const auto& c : drill.cycles) {
            types.insert(c.type);
        }
        REQUIRE(types.size() >= 2);
        const toolpath::CycleType kept = *types.begin();
        LowerOptions options;
        options.cycleTypes = {kept};
        const NcProgram p = lower({{"holes", drill, "", 1}}, "p", 1, options);
        std::size_t keptHoles = 0;
        for (const auto& m : drill.moves) {
            if (m.kind == toolpath::MoveKind::Cycle &&
                drill.cycles[static_cast<std::size_t>(m.cycle)].type == kept) {
                ++keptHoles;
            }
        }
        std::size_t cycles = 0;
        for (const NcInstr& i : p.instrs) {
            if (i.op == NcOp::Cycle) {
                CHECK(i.cycle.type == kept);
                ++cycles;
            }
        }
        CHECK(cycles == keptHoles);
        CHECK(count(p, NcOp::Linear) > 0); // the other holes as plunges
    }
}

TEST_CASE("a manifest's declarations become lowering options", "[post][declarations]") {
    CancelToken cancel;
    const PostScript post = PostScript::load(R"js(
export const manifest = {
  id: "p", name: "P", version: "1", engine: 1, machines: ["mill3"],
  arcs: { maxSweep: 90, planes: ["xy"], helical: false }, tolerance: 0.005,
  cycles: ["drill", "peck"], multiAxis: "machine",
};
export const options = [];
export default class {}
)js",
                                             "p.js", cancel);
    const LowerOptions options = lowerOptionsFor(post.manifest());
    CHECK(options.cannedCycles);
    CHECK(options.cycleTypes ==
          std::vector<toolpath::CycleType>{toolpath::CycleType::Drill, toolpath::CycleType::Peck});
    CHECK_FALSE(options.tcp);
    CHECK(options.arcPlanes == std::vector<toolpath::ArcPlane>{toolpath::ArcPlane::XY});
    CHECK_FALSE(options.helicalArcs);
    CHECK_THAT(options.maxArcSweep, WithinAbs(kPi / 2.0, 1e-12));
    CHECK_THAT(options.tolerance, WithinAbs(0.005, 1e-12));
}
