// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Headless entry point.
//   oxcam-cli info
//   oxcam-cli toollib check <file.ocamtools>
//   oxcam-cli project info <file.ocam>
//   oxcam-cli project toolpaths <file.ocam> [--json <dir>]
//   oxcam-cli project post <file.ocam> [--setup S] [--post ID|P.js] [--option name=value]...
//                                        [--machine M.json] [-o out.nc] [--verify]
//                                        [--tolerance T] [--ir program.json]
//   oxcam-cli project simulate <file.ocam> [--setup S] [--resolution R] [--tolerance T]
//                                            [--max-excess E] [--json report.json]
//   oxcam-cli backplot <file.nc>

#include <ocam/document/Document.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/MachineImport.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/document/Workholding.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Log.hpp>
#include <ocam/foundation/Version.hpp>
#include <ocam/post/Engine.hpp>
#include <ocam/post/Post.hpp>
#include <ocam/tooling/ToolLibrary.hpp>

#include <CLI/CLI.hpp>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <optional>
#include <string>

namespace {

struct Vec3Print {
    ocam::Vec3 v;
};

void printInfo() {
    fmt::print("{} {}\n", ocam::kProductName, ocam::kVersionString);
    fmt::print("license: GPL-3.0-or-later\n");
    fmt::print("internal units: mm, rad, s\n");
#if defined(_MSC_VER)
    fmt::print("compiler: MSVC {}\n", _MSC_VER);
#elif defined(__clang__)
    fmt::print("compiler: Clang {}.{}.{}\n", __clang_major__, __clang_minor__,
               __clang_patchlevel__);
#elif defined(__GNUC__)
    fmt::print("compiler: GCC {}.{}.{}\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#endif
}

// Prints issues and returns the process exit code: 0 when there are none, 1 otherwise.
int reportIssues(const std::vector<std::string>& issues) {
    if (issues.empty()) {
        return 0;
    }
    fmt::print("{} problem(s):\n", issues.size());
    for (const auto& issue : issues) {
        fmt::print("  - {}\n", issue);
    }
    return 1;
}

int checkToolLibrary(const std::filesystem::path& path) {
    const ocam::tooling::ToolLibrary lib = ocam::tooling::loadToolLibrary(path);
    fmt::print("{}\n", lib.name.empty() ? "(unnamed library)" : lib.name);
    for (const auto& t : lib.tools) {
        const bool turning = ocam::tooling::isTurningTool(t.type);
        fmt::print("  T{:<4} {:<22} {:>9}  {}\n", t.number, ocam::tooling::displayName(t.type),
                   turning ? std::string() : fmt::format("Ø{:g}", t.mill.diameter), t.name);
    }
    const auto issues = lib.validate();
    if (issues.empty()) {
        fmt::print("{} tools, {} holders, no problems\n", lib.tools.size(), lib.holders.size());
    }
    return reportIssues(issues);
}

int projectInfo(const std::filesystem::path& path) {
    const ocam::document::Document doc = ocam::document::Document::open(path);
    const ocam::document::Project& p = doc.project();
    fmt::print("project: {}\n", p.name);
    fmt::print("models: {}\n", p.models.size());
    for (const auto& m : p.models) {
        const auto bounds = doc.modelBounds(m.id);
        const ocam::Vec3 size = bounds ? bounds->size() : ocam::Vec3::Zero();
        fmt::print("  {} [{}] {:.3f} x {:.3f} x {:.3f} mm\n", m.name, m.sha256.substr(0, 12),
                   size.x(), size.y(), size.z());
    }
    fmt::print("tools: {}\n", p.tools.tools.size());
    fmt::print("setups: {}\n", p.setups.size());
    for (const auto& s : p.setups) {
        const auto stock = doc.stockShape(s.id);
        const ocam::Vec3 size = !stock ? ocam::Vec3::Zero()
                                : stock->kind == ocam::document::StockShape::Kind::Box
                                    ? stock->box.size()
                                    : stock->bounds().size();
        fmt::print("  {} ({}), stock {:.3f} x {:.3f} x {:.3f} mm, {} operation(s)\n", s.name,
                   ocam::document::workOffsetName(p, s, s.workOffset), size.x(), size.y(), size.z(),
                   s.operations.size());
        if (s.workholding) {
            const auto* w = p.findWorkholding(s.workholding->templateId);
            try {
                const auto pose = ocam::document::setupWorkholdingPose(doc, s);
                fmt::print("    workholding: {}, jaws {:.3f} mm apart\n", w ? w->name : "?",
                           pose ? pose->opening : 0.0);
            } catch (const ocam::Error& e) {
                fmt::print("    workholding: {} cannot be placed: {}\n", w ? w->name : "?",
                           e.what());
            }
        }
        for (const auto& op : s.operations) {
            fmt::print("    - {} [{}]{}\n", op.name, op.type, op.enabled ? "" : " (disabled)");
        }
    }
    return reportIssues(p.validate());
}

int projectToolpaths(const std::filesystem::path& path, const std::string& jsonDir) {
    const ocam::document::Document doc = ocam::document::Document::open(path);
    int total = 0;
    int ok = 0;
    for (const auto& setup : doc.project().setups) {
        fmt::print("{} ({})\n", setup.name,
                   ocam::document::workOffsetName(doc.project(), setup, setup.workOffset));
        for (const auto& op : setup.operations) {
            if (!op.enabled) {
                continue;
            }
            ++total;
            try {
                ocam::CancelToken cancel;
                ocam::NullProgress progress;
                const ocam::toolpath::Toolpath tp =
                    *ocam::document::cachedToolpath(doc, setup, op, cancel, progress);
                const auto stats = ocam::toolpath::computeStats(tp);
                fmt::print("  {:<20} T{:<3} {:>6} moves  {:>9.1f} mm  {:>6.1f} s\n", op.name,
                           tp.toolNumber, stats.moveCount, stats.cutLength, stats.totalTime);
                for (const auto& w : tp.warnings) {
                    fmt::print("      warning: {}\n", w);
                }
                if (!jsonDir.empty()) {
                    std::filesystem::create_directories(jsonDir);
                    ocam::writeFileAtomic(std::filesystem::path(jsonDir) /
                                              (op.id.toString() + ".json"),
                                          ocam::dumpJson(ocam::toolpath::toJson(tp)));
                }
                ++ok;
            } catch (const ocam::Error& e) {
                fmt::print("  {:<20} FAILED: {}\n", op.name, e.what());
            }
        }
    }
    fmt::print("{} of {} operations generated\n", ok, total);
    return ok == total ? 0 : 1;
}

const ocam::document::Setup* findSetup(const ocam::document::Document& doc,
                                       const std::string& name) {
    for (const auto& s : doc.project().setups) {
        if (name.empty() || s.name == name) {
            return &s;
        }
    }
    return nullptr;
}

struct PostArgs {
    std::string file;
    std::string setup;
    std::string post;
    std::vector<std::string> options;
    std::string machine;
    std::string output;
    std::string ir;
    bool verify = false;
    double tolerance = 0.0015;
};

/// A --option value: true, false, a number, or text.
ocam::Json optionValue(const std::string& text) {
    if (text == "true" || text == "false") {
        return text == "true";
    }
    try {
        std::size_t used = 0;
        const double number = std::stod(text, &used);
        if (used == text.size()) {
            return number;
        }
    } catch (const std::exception&) {
    }
    return text;
}

int projectPost(const PostArgs& args) {
    const ocam::document::Document doc =
        ocam::document::Document::open(std::filesystem::path(args.file));
    const ocam::document::Setup* setup = findSetup(doc, args.setup);
    if (!setup) {
        ocam::logError("no setup named '{}'", args.setup);
        return 2;
    }
    const ocam::machine::Machine machine =
        args.machine.empty() ? ocam::document::machineFor(doc.project(), *setup)
                             : ocam::document::loadMachineFile(std::filesystem::path(args.machine));
    ocam::CancelToken cancel;
    ocam::NullProgress progress;
    // A built-in post by id, or a post file (*.js).
    std::optional<ocam::post::PostScript> loaded;
    if (!args.post.empty() &&
        (args.post.ends_with(".js") || std::filesystem::exists(std::filesystem::path(args.post)))) {
        const std::filesystem::path path(args.post);
        const std::u8string name = path.filename().u8string();
        loaded = ocam::post::PostScript::load(ocam::readFileBytes(path),
                                              std::string(name.begin(), name.end()), cancel);
    }
    if (!loaded && args.post.empty()) {
        loaded = ocam::document::postFor(doc.project(), *setup, machine); // the setup's post
    }
    const ocam::post::PostScript& post = loaded ? *loaded : ocam::post::builtinPost(args.post);
    // The setup's and the machine's option values, with --option values for the program over
    // them.
    ocam::Json options = ocam::document::postOptionsFor(doc.project(), *setup, machine, post);
    for (const std::string& option : args.options) {
        const auto equals = option.find('=');
        if (equals == std::string::npos || equals == 0) {
            ocam::logError("--option '{}': expected name=value", option);
            return 2;
        }
        options["program"][option.substr(0, equals)] = optionValue(option.substr(equals + 1));
    }
    if (!args.ir.empty()) {
        // What the post receives: the lowered program (NC program JSON).
        const auto lowered = ocam::document::lowerSetup(
            doc, setup->id, machine, ocam::post::lowerOptionsFor(post.manifest()), cancel,
            progress);
        if (!lowered.ok()) {
            for (const auto& e : lowered.errors) {
                ocam::logError("{}", e);
            }
            return 1;
        }
        ocam::writeFileAtomic(std::filesystem::path(args.ir),
                              ocam::dumpJson(ocam::post::toJson(lowered.program)));
        ocam::logInfo("wrote NC program JSON {} ({} instructions)", args.ir,
                      lowered.program.instrs.size());
        if (args.output.empty() && !args.verify) {
            return 0;
        }
    }
    const auto out =
        ocam::document::postSetup(doc, setup->id, machine, post, cancel, progress, options);
    for (const auto& w : out.warnings) {
        ocam::logWarn("{}", w);
    }
    if (!out.ok()) {
        for (const auto& e : out.errors) {
            ocam::logError("{}", e);
        }
        return 1;
    }
    if (args.output.empty()) {
        fmt::print("{}", out.gcode);
    } else {
        ocam::writeFileAtomic(std::filesystem::path(args.output), out.gcode);
        ocam::logInfo("wrote {} ({} lines) for {} with {}", args.output,
                      std::count(out.gcode.begin(), out.gcode.end(), '\n'), machine.name,
                      post.manifest().name);
    }
    if (args.verify) {
        // The post says which back-plot dialect reads its output, if any.
        const auto& dialect = post.manifest().verify;
        if (!dialect) {
            ocam::logError("{} declares no back-plot dialect: its output cannot be verified",
                           post.name());
            return 1;
        }
        const bool lathe = *dialect == "fanuc-lathe";
        ocam::post::BackplotOptions plotOptions;
        plotOptions.lathe = lathe;
        plotOptions.machine = &machine; // rotary words give the tool axis
        const auto plotted = ocam::post::backplot(out.gcode, plotOptions);
        double deviation = 0.0;
        // Lathe feeds per revolution at constant surface speed have no rpm to compare against.
        const auto diff = ocam::post::compareBackplot(out.toolpaths, plotted.path, args.tolerance,
                                                      &deviation, !lathe);
        if (diff) {
            ocam::logError("verification failed: {}", *diff);
            return 1;
        }
        ocam::logInfo(
            "verified: back-plotted G-code matches the toolpaths (max deviation {:.6f} mm)",
            deviation);
    }
    return 0;
}

struct SimulateArgs {
    std::string file;
    std::string setup;
    double resolution = 0.0;
    double tolerance = 0.02;
    double maxExcess = -1.0;
    std::string json;
};

int projectSimulate(const SimulateArgs& args) {
    const ocam::document::Document doc =
        ocam::document::Document::open(std::filesystem::path(args.file));
    const ocam::document::Setup* setup = findSetup(doc, args.setup);
    if (!setup) {
        ocam::logError("no setup named '{}'", args.setup);
        return 2;
    }
    ocam::document::SimulationOptions options;
    options.resolution = args.resolution;
    options.tolerance = args.tolerance;
    ocam::CancelToken cancel;
    ocam::NullProgress progress;
    const auto sim = ocam::document::simulateSetup(doc, setup->id, options, cancel, progress);
    for (const auto& w : sim.warnings) {
        ocam::logWarn("{}", w);
    }
    // A lathe setup is simulated on its profile, without rays.
    ocam::Vec3 size =
        sim.initialStock.bounds().size() - ocam::Vec3::Constant(2.0 * sim.resolution);
    double initialVolume = sim.initialStock.volume();
    double finalVolume = sim.finalStock().volume();
    if (sim.latheInitial) {
        ocam::geom2d::Aabb2 profile;
        for (const auto& r : sim.latheInitial->profile()) {
            profile.extend(r.bounds());
        }
        size = ocam::Vec3(2.0 * profile.max.y(), 2.0 * profile.max.y(),
                          profile.max.x() - profile.min.x());
        initialVolume = sim.latheInitial->volume();
        const auto last = sim.latheBefore(sim.operations.size());
        finalVolume = (last ? *last : *sim.latheInitial).volume();
        fmt::print("{} ({}): stock {:.3f} x {:.3f} x {:.3f} mm, lathe profile\n", setup->name,
                   ocam::document::workOffsetName(doc.project(), *setup, setup->workOffset),
                   size.x(), size.y(), size.z());
    } else {
        fmt::print("{} ({}): stock {:.3f} x {:.3f} x {:.3f} mm, {:.3f} mm rays{}\n", setup->name,
                   ocam::document::workOffsetName(doc.project(), *setup, setup->workOffset),
                   size.x(), size.y(), size.z(), sim.resolution,
                   sim.initialStock.triDexel() ? " (tri-dexel)" : "");
    }
    ocam::Json report{{"setup", setup->name},
                      {"resolution", sim.resolution},
                      {"initialVolume", initialVolume},
                      {"operations", ocam::Json::array()}};
    bool failed = false;
    for (const auto& op : sim.operations) {
        ocam::Json entry{{"name", op.name},
                         {"removedVolume", op.removedVolume},
                         {"collisions", ocam::Json::array()}};
        if (!op.error.empty()) {
            fmt::print("  {:<20} FAILED: {}\n", op.name, op.error);
            entry["error"] = op.error;
            failed = true;
        } else {
            fmt::print("  {:<20} removed {:>10.1f} mm3  {} collision(s)", op.name, op.removedVolume,
                       op.collisions.size());
            if (!op.machineCollisions.empty()) {
                fmt::print(", {} machine collision(s)", op.machineCollisions.size());
            }
            if (!op.fixtureCollisions.empty()) {
                fmt::print(", {} workholding collision(s)", op.fixtureCollisions.size());
            }
            fmt::print("\n");
        }
        entry["fixtureCollisions"] = ocam::Json::array();
        for (const auto& c : op.fixtureCollisions) {
            fmt::print(
                "      workholding: {} hits {}, moves {}..{}, near X{:.3f} Y{:.3f} Z{:.3f}\n",
                c.toolPart, c.body, c.firstMove, c.lastMove, c.position.x(), c.position.y(),
                c.position.z());
            entry["fixtureCollisions"].push_back({{"body", c.body},
                                                  {"toolPart", c.toolPart},
                                                  {"firstMove", c.firstMove},
                                                  {"lastMove", c.lastMove},
                                                  {"distance", c.distance}});
            failed = true;
        }
        entry["machineCollisions"] = ocam::Json::array();
        for (const auto& c : op.machineCollisions) {
            fmt::print("      machine: {} / {}, moves {}..{}, near X{:.3f} Y{:.3f} Z{:.3f} "
                       "(distance {:.3f} mm)\n",
                       c.first, c.second, c.firstMove, c.lastMove, c.position.x(), c.position.y(),
                       c.position.z(), c.distance);
            entry["machineCollisions"].push_back({{"first", c.first},
                                                  {"second", c.second},
                                                  {"firstMove", c.firstMove},
                                                  {"lastMove", c.lastMove},
                                                  {"distance", c.distance}});
            failed = true;
        }
        for (const auto& c : op.collisions) {
            fmt::print(
                "      {} collision, moves {}..{}, near X{:.3f} Y{:.3f} Z{:.3f} ({:.3f} mm)\n",
                ocam::sim::toString(c.kind), c.firstMove, c.lastMove, c.position.x(),
                c.position.y(), c.position.z(), c.depth);
            entry["collisions"].push_back({{"kind", ocam::sim::toString(c.kind)},
                                           {"firstMove", c.firstMove},
                                           {"lastMove", c.lastMove},
                                           {"depth", c.depth}});
            failed = true;
        }
        report["operations"].push_back(entry);
    }
    report["finalVolume"] = finalVolume;
    fmt::print("final stock {:.1f} mm3", finalVolume);
    if (!sim.part.isEmpty()) {
        fmt::print(", part {:.1f} mm3", sim.part.signedVolume());
        report["partVolume"] = sim.part.signedVolume();
    }
    fmt::print("\n");
    if (sim.comparison) {
        const auto& c = *sim.comparison;
        fmt::print("comparison (tolerance {:.3f} mm): excess {:.1f} mm3 (max {:.3f} mm), "
                   "gouge {:.1f} mm3 (max {:.3f} mm)\n",
                   c.tolerance, c.excessVolume, c.maxExcess, c.gougeVolume, c.maxGouge);
        report["comparison"] = {{"tolerance", c.tolerance},
                                {"excessVolume", c.excessVolume},
                                {"maxExcess", c.maxExcess},
                                {"gougeVolume", c.gougeVolume},
                                {"maxGouge", c.maxGouge}};
        if (c.hasGouges()) {
            failed = true;
        }
        if (args.maxExcess >= 0.0 && c.maxExcess > args.maxExcess) {
            ocam::logError("stock left on the part exceeds {:.3f} mm", args.maxExcess);
            failed = true;
        }
    }
    if (!args.json.empty()) {
        ocam::writeFileAtomic(std::filesystem::path(args.json), ocam::dumpJson(report));
    }
    fmt::print("result: {}\n", failed ? "PROBLEMS FOUND" : "ok");
    return failed ? 1 : 0;
}

int backplotFile(const std::string& file) {
    const auto result = ocam::post::backplot(ocam::readFileBytes(std::filesystem::path(file)));
    const auto stats = ocam::toolpath::computeStats(result.path);
    for (const auto& w : result.warnings) {
        ocam::logWarn("{}", w);
    }
    fmt::print("tools: {}\n", fmt::join(result.toolChanges, ", "));
    fmt::print("moves: {}, feed length {:.1f} mm, rapid length {:.1f} mm, estimated {:.1f} s\n",
               stats.moveCount, stats.cutLength, stats.rapidLength, stats.totalTime);
    const Vec3Print lo{stats.bounds.min};
    const Vec3Print hi{stats.bounds.max};
    fmt::print("extent: X {:.3f}..{:.3f}  Y {:.3f}..{:.3f}  Z {:.3f}..{:.3f}\n", lo.v.x(), hi.v.x(),
               lo.v.y(), hi.v.y(), lo.v.z(), hi.v.z());
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    CLI::App app{"OxCAM command-line interface", "oxcam-cli"};
    app.set_version_flag("-V,--version", fmt::format("oxcam-cli {}", ocam::kVersionString));
    app.require_subcommand(0, 1);

    bool verbose = false;
    app.add_flag("-v,--verbose", verbose, "Enable debug logging");

    auto* info = app.add_subcommand("info", "Print version and build information");

    auto* toollib = app.add_subcommand("toollib", "Tool library files");
    toollib->require_subcommand(1);
    std::string toolLibraryFile;
    auto* check = toollib->add_subcommand("check", "Validate a tool library and list its tools");
    check->add_option("file", toolLibraryFile, "Tool library (*.ocamtools)")->required();

    auto* project = app.add_subcommand("project", "Project files");
    project->require_subcommand(1);
    std::string projectFile;
    auto* projectInfoCmd = project->add_subcommand("info", "Summarize and validate a project");
    projectInfoCmd->add_option("file", projectFile, "Project (*.ocam)")->required();

    std::string toolpathFile;
    std::string toolpathJsonDir;
    auto* toolpathsCmd =
        project->add_subcommand("toolpaths", "Generate every operation's toolpath");
    toolpathsCmd->add_option("file", toolpathFile, "Project (*.ocam)")->required();
    toolpathsCmd->add_option("--json", toolpathJsonDir,
                             "Write each toolpath as JSON into this directory");

    PostArgs postArgs;
    auto* postCmd = project->add_subcommand("post", "Post process a setup to G-code");
    postCmd->add_option("file", postArgs.file, "Project (*.ocam)")->required();
    postCmd->add_option("--setup", postArgs.setup, "Setup name (default: the first setup)");
    postCmd->add_option(
        "--post", postArgs.post,
        "Built-in post id or post file (*.js; default: the machine's built-in post)");
    postCmd->add_option("--option", postArgs.options,
                        "A post option, name=value (repeat for more)");
    postCmd->add_option("--machine", postArgs.machine,
                        "Machine definition (*.json; default: the setup's machine)");
    postCmd->add_option("-o,--output", postArgs.output, "Output file (default: standard output)");
    postCmd->add_option("--ir", postArgs.ir,
                        "Also write the NC program (post API JSON) for external post processors");
    postCmd->add_flag("--verify", postArgs.verify,
                      "Back-plot the G-code and compare it with the toolpaths");
    postCmd->add_option("--tolerance", postArgs.tolerance,
                        "Largest deviation --verify accepts, in mm (default 0.0015: 3 decimals)");

    SimulateArgs simArgs;
    auto* simulateCmd =
        project->add_subcommand("simulate", "Simulate a setup's material removal and compare");
    simulateCmd->add_option("file", simArgs.file, "Project (*.ocam)")->required();
    simulateCmd->add_option("--setup", simArgs.setup, "Setup name (default: the first setup)");
    simulateCmd->add_option("--resolution", simArgs.resolution,
                            "Ray spacing in mm (default: from the stock size)");
    simulateCmd->add_option("--tolerance", simArgs.tolerance,
                            "Comparison tolerance in mm (default: 0.02)");
    simulateCmd->add_option("--max-excess", simArgs.maxExcess,
                            "Fail when more stock than this is left on the part [mm]");
    simulateCmd->add_option("--json", simArgs.json, "Write a JSON report");

    std::string backplotPath;
    auto* backplotCmd = app.add_subcommand("backplot", "Parse a G-code file and report its motion");
    backplotCmd->add_option("file", backplotPath, "G-code file")->required();

    CLI11_PARSE(app, argc, argv);

    ocam::setLogLevel(verbose ? ocam::LogLevel::Debug : ocam::LogLevel::Info);

    try {
        if (check->parsed()) {
            return checkToolLibrary(std::filesystem::path(toolLibraryFile));
        }
        if (postCmd->parsed()) {
            return projectPost(postArgs);
        }
        if (simulateCmd->parsed()) {
            return projectSimulate(simArgs);
        }
        if (backplotCmd->parsed()) {
            return backplotFile(backplotPath);
        }
        if (toolpathsCmd->parsed()) {
            return projectToolpaths(std::filesystem::path(toolpathFile), toolpathJsonDir);
        }
        if (projectInfoCmd->parsed()) {
            return projectInfo(std::filesystem::path(projectFile));
        }
        if (info->parsed() || app.get_subcommands().empty()) {
            printInfo();
        }
    } catch (const ocam::Error& e) {
        ocam::logError("{}: {}", ocam::toString(e.code()), e.what());
        return 2;
    } catch (const std::exception& e) {
        ocam::logError("unexpected error: {}", e.what());
        return 3;
    }
    return 0;
}
