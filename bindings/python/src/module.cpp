// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Python bindings (nanobind). The surface is deliberately small and data-oriented: documents
// are opened from files, and results come back as plain Python dicts and lists in the same
// shapes as OxCAM's JSON formats (toolpaths, NC programs, simulation reports). See
// docs/design-log/0015.

#include <ocam/document/Document.hpp>
#include <ocam/document/Generate.hpp>
#include <ocam/document/PostSetup.hpp>
#include <ocam/document/Simulate.hpp>
#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Json.hpp>
#include <ocam/foundation/Version.hpp>
#include <ocam/post/Engine.hpp>
#include <ocam/post/Post.hpp>
#include <ocam/tooling/ToolLibrary.hpp>

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <filesystem>
#include <optional>
#include <string>

namespace nb = nanobind;
using namespace nb::literals;

namespace {

using ocam::Json;

nb::object toPy(const Json& j) {
    switch (j.type()) {
    case Json::value_t::null:
        return nb::none();
    case Json::value_t::boolean:
        return nb::bool_(j.get<bool>());
    case Json::value_t::number_integer:
        return nb::int_(j.get<std::int64_t>());
    case Json::value_t::number_unsigned:
        return nb::int_(j.get<std::uint64_t>());
    case Json::value_t::number_float:
        return nb::float_(j.get<double>());
    case Json::value_t::string:
        return nb::str(j.get_ref<const std::string&>().c_str());
    case Json::value_t::array: {
        nb::list list;
        for (const Json& item : j) {
            list.append(toPy(item));
        }
        return std::move(list);
    }
    case Json::value_t::object: {
        nb::dict dict;
        for (const auto& [key, value] : j.items()) {
            dict[nb::str(key.c_str())] = toPy(value);
        }
        return std::move(dict);
    }
    default:
        return nb::none();
    }
}

Json fromPy(nb::handle h) {
    if (h.is_none()) {
        return nullptr;
    }
    if (nb::isinstance<nb::bool_>(h)) {
        return nb::cast<bool>(h);
    }
    if (nb::isinstance<nb::int_>(h)) {
        return nb::cast<std::int64_t>(h);
    }
    if (nb::isinstance<nb::float_>(h)) {
        return nb::cast<double>(h);
    }
    if (nb::isinstance<nb::str>(h)) {
        return nb::cast<std::string>(h);
    }
    if (nb::isinstance<nb::dict>(h)) {
        Json obj = Json::object();
        for (auto [key, value] : nb::borrow<nb::dict>(h)) {
            obj[nb::cast<std::string>(nb::str(key))] = fromPy(value);
        }
        return obj;
    }
    if (nb::isinstance<nb::list>(h) || nb::isinstance<nb::tuple>(h)) {
        Json arr = Json::array();
        for (nb::handle item : h) {
            arr.push_back(fromPy(item));
        }
        return arr;
    }
    throw nb::type_error("expected JSON-compatible data (dict, list, str, number, bool, None)");
}

Json toolJson(const ocam::tooling::Tool& t) {
    return Json{{"id", t.id.toString()},
                {"number", t.number},
                {"name", t.name},
                {"type", std::string(ocam::tooling::displayName(t.type))},
                {"diameter", t.mill.diameter},
                {"flute_length", t.mill.fluteLength},
                {"thread_pitch", t.mill.threadPitch}};
}

class PyDocument {
public:
    explicit PyDocument(const std::string& path)
        : doc_(ocam::document::Document::open(std::filesystem::u8path(path))) {}

    std::string name() const { return doc_.project().name; }

    nb::object setups() const {
        Json out = Json::array();
        for (const auto& s : doc_.project().setups) {
            Json ops = Json::array();
            for (const auto& op : s.operations) {
                const auto* tool = op.toolId ? doc_.project().tools.findTool(*op.toolId) : nullptr;
                ops.push_back({{"id", op.id.toString()},
                               {"name", op.name},
                               {"type", op.type},
                               {"enabled", op.enabled},
                               {"tool", tool ? Json(tool->number) : Json(nullptr)},
                               {"parameters", op.parameters}});
            }
            out.push_back({{"id", s.id.toString()},
                           {"name", s.name},
                           {"work_offset", ocam::document::workOffsetName(s.workOffset)},
                           {"machine", s.machine},
                           {"operations", std::move(ops)}});
        }
        return toPy(out);
    }

    nb::object tools() const {
        Json out = Json::array();
        for (const auto& t : doc_.project().tools.tools) {
            out.push_back(toolJson(t));
        }
        return toPy(out);
    }

    nb::object toolpaths(const std::optional<std::string>& setupName) const {
        const auto& setup = findSetup(setupName);
        Json out = Json::array();
        {
            nb::gil_scoped_release release;
            for (const auto& op : setup.operations) {
                if (!op.enabled) {
                    continue;
                }
                Json entry{{"operation", op.name}, {"type", op.type}};
                try {
                    ocam::CancelToken cancel;
                    ocam::NullProgress progress;
                    const auto tp =
                        ocam::document::generateToolpath(doc_, setup, op, cancel, progress);
                    const auto stats = ocam::toolpath::computeStats(tp);
                    entry["tool_number"] = tp.toolNumber;
                    entry["warnings"] = tp.warnings;
                    entry["stats"] = {{"moves", stats.moveCount},
                                      {"cut_length", stats.cutLength},
                                      {"rapid_length", stats.rapidLength},
                                      {"time", stats.totalTime}};
                    entry["toolpath"] = ocam::toolpath::toJson(tp);
                } catch (const ocam::OperationCancelled&) {
                    throw;
                } catch (const ocam::Error& e) {
                    entry["error"] = e.what();
                }
                out.push_back(std::move(entry));
            }
        }
        return toPy(out);
    }

    nb::object simulate(const std::optional<std::string>& setupName, double resolution,
                        double tolerance) const {
        const auto& setup = findSetup(setupName);
        Json report;
        {
            nb::gil_scoped_release release;
            ocam::document::SimulationOptions options;
            options.resolution = resolution;
            options.tolerance = tolerance;
            ocam::CancelToken cancel;
            ocam::NullProgress progress;
            const auto sim =
                ocam::document::simulateSetup(doc_, setup.id, options, cancel, progress);
            report = {{"setup", setup.name},
                      {"resolution", sim.resolution},
                      {"initial_volume", sim.initialStock.volume()},
                      {"final_volume", sim.finalStock().volume()},
                      {"warnings", sim.warnings},
                      {"operations", Json::array()}};
            for (const auto& op : sim.operations) {
                Json entry{{"name", op.name},
                           {"removed_volume", op.removedVolume},
                           {"collisions", Json::array()}};
                if (!op.error.empty()) {
                    entry["error"] = op.error;
                }
                for (const auto& c : op.collisions) {
                    entry["collisions"].push_back(
                        {{"kind", std::string(ocam::sim::toString(c.kind))},
                         {"first_move", c.firstMove},
                         {"last_move", c.lastMove},
                         {"position", {c.position.x(), c.position.y(), c.position.z()}},
                         {"depth", c.depth}});
                }
                report["operations"].push_back(std::move(entry));
            }
            if (!sim.part.isEmpty()) {
                report["part_volume"] = sim.part.signedVolume();
            }
            if (sim.comparison) {
                const auto& c = *sim.comparison;
                report["comparison"] = {{"tolerance", c.tolerance},
                                        {"excess_volume", c.excessVolume},
                                        {"max_excess", c.maxExcess},
                                        {"gouge_volume", c.gougeVolume},
                                        {"max_gouge", c.maxGouge}};
            }
        }
        return toPy(report);
    }

    std::string post(const std::optional<std::string>& setupName,
                     const std::optional<std::string>& postName,
                     const std::optional<std::string>& machinePath, nb::handle options) const {
        const auto& setup = findSetup(setupName);
        const ocam::machine::Machine machine =
            machinePath ? ocam::machine::loadMachine(std::filesystem::u8path(*machinePath))
                        : ocam::document::machineFor(doc_.project(), setup);
        const Json optionValues = options.is_none() ? Json::object() : fromPy(options);
        ocam::document::PostOutput out;
        {
            nb::gil_scoped_release release;
            ocam::CancelToken cancel;
            ocam::NullProgress progress;
            // A built-in post by id, or a post file (*.js).
            std::optional<ocam::post::PostScript> loaded;
            if (postName && (postName->ends_with(".js") ||
                             std::filesystem::exists(std::filesystem::u8path(*postName)))) {
                const auto path = std::filesystem::u8path(*postName);
                const std::u8string file = path.filename().u8string();
                loaded = ocam::post::PostScript::load(
                    ocam::readFileBytes(path), std::string(file.begin(), file.end()), cancel);
            }
            if (!loaded && !postName) {
                loaded = ocam::document::postFor(doc_.project(), setup, machine);
            }
            const ocam::post::PostScript& post =
                loaded ? *loaded : ocam::post::builtinPost(*postName);
            // The setup's and the machine's option values, with the given program options over
            // them.
            Json values = ocam::document::postOptionsFor(doc_.project(), setup, machine, post);
            for (const auto& [id, value] : optionValues.items()) {
                values["program"][id] = value;
            }
            out =
                ocam::document::postSetup(doc_, setup.id, machine, post, cancel, progress, values);
        }
        if (!out.ok()) {
            throw ocam::Error(ocam::ErrorCode::InvalidArgument, out.errors.front());
        }
        return out.gcode;
    }

    nb::object ncProgram(const std::optional<std::string>& setupName, bool cannedCycles,
                         const std::optional<std::string>& machinePath, bool tcp) const {
        const auto& setup = findSetup(setupName);
        const ocam::machine::Machine machine =
            machinePath ? ocam::machine::loadMachine(std::filesystem::u8path(*machinePath))
                        : ocam::document::machineFor(doc_.project(), setup);
        ocam::document::LoweredSetup lowered;
        {
            nb::gil_scoped_release release;
            ocam::CancelToken cancel;
            ocam::NullProgress progress;
            lowered = ocam::document::lowerSetup(doc_, setup.id, machine,
                                                 ocam::post::LowerOptions{cannedCycles, tcp},
                                                 cancel, progress);
        }
        if (!lowered.ok()) {
            throw ocam::Error(ocam::ErrorCode::InvalidArgument, lowered.errors.front());
        }
        Json program = ocam::post::toJson(lowered.program);
        for (const auto& w : lowered.warnings) {
            program["warnings"].push_back(w);
        }
        return toPy(program);
    }

private:
    const ocam::document::Setup& findSetup(const std::optional<std::string>& name) const {
        for (const auto& s : doc_.project().setups) {
            if (!name || s.name == *name) {
                return s;
            }
        }
        throw ocam::Error(ocam::ErrorCode::InvalidArgument,
                          name ? "no setup named '" + *name + "'"
                               : std::string("the project has no setups"));
    }

    ocam::document::Document doc_;
};

} // namespace

NB_MODULE(_oxcam, m) {
    m.doc() = "OxCAM: open-source CAM (GPL-3.0-or-later)";
    m.attr("__version__") = std::string(ocam::kVersionString);
    nb::exception<ocam::Error>(m, "Error");

    nb::class_<PyDocument>(m, "Document", "An OxCAM project (*.ocam), opened read-only.")
        .def_static(
            "open", [](const std::string& path) { return new PyDocument(path); }, "path"_a,
            nb::rv_policy::take_ownership, "Opens a project file.")
        .def_prop_ro("name", &PyDocument::name)
        .def_prop_ro("setups", &PyDocument::setups, "Setups with their operations.")
        .def_prop_ro("tools", &PyDocument::tools, "The project's tools.")
        .def("toolpaths", &PyDocument::toolpaths, "setup"_a = nb::none(),
             "Generates every enabled operation of a setup (default: the first).")
        .def("simulate", &PyDocument::simulate, "setup"_a = nb::none(), "resolution"_a = 0.0,
             "tolerance"_a = 0.02, "Simulates a setup and compares the result with the part.")
        .def("post", &PyDocument::post, "setup"_a = nb::none(), "post"_a = nb::none(),
             "machine"_a = nb::none(), "options"_a = nb::none(),
             "Posts a setup to G-code with the built-in or a JSON post configuration.")
        .def("nc_program", &PyDocument::ncProgram, "setup"_a = nb::none(), "canned_cycles"_a = true,
             "machine"_a = nb::none(), "tcp"_a = true,
             "The setup lowered to the NC program form (post API) for custom post processors. "
             "Multi-axis toolpaths give tool tips with tool centre point control (tcp=True) or "
             "machine positions with inverse-time feeds (tcp=False).");

    m.def(
        "builtin_posts", [] { return ocam::post::builtinPostIds(); },
        "The ids of the built-in posts (for Document.post(post=...)).");
    m.def(
        "load_tool_library",
        [](const std::string& path) {
            const auto lib = ocam::tooling::loadToolLibrary(std::filesystem::u8path(path));
            Json tools = Json::array();
            for (const auto& t : lib.tools) {
                tools.push_back(toolJson(t));
            }
            return toPy(Json{{"name", lib.name}, {"tools", tools}, {"problems", lib.validate()}});
        },
        "path"_a, "Reads a tool library (*.ocamtools).");
    m.def(
        "backplot",
        [](const std::string& gcode) {
            const auto result = ocam::post::backplot(gcode);
            const auto stats = ocam::toolpath::computeStats(result.path);
            return toPy(Json{{"tools", result.toolChanges},
                             {"warnings", result.warnings},
                             {"stats",
                              {{"moves", stats.moveCount},
                               {"cut_length", stats.cutLength},
                               {"rapid_length", stats.rapidLength},
                               {"time", stats.totalTime}}},
                             {"toolpath", ocam::toolpath::toJson(result.path)}});
        },
        "gcode"_a, "Parses G-code (Fanuc-style subset) into motion.");
}
