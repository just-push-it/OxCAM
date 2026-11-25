// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/machine/Machine.hpp"

#include "SolveDetail.hpp"
#include "ocam/machine/Model.hpp"

#include <ocam/foundation/Error.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>

namespace ocam::embedded {
extern const std::string_view machineSchemaV16;
extern const std::string_view genericMill3Machine;
extern const std::string_view genericMill5TableACMachine;
extern const std::string_view genericLathe2Machine;
extern const std::string_view genericLatheLiveMachine;
extern const std::string_view genericLatheLiveYMachine;
extern const std::string_view genericLatheTurretMachine;
extern const std::string_view genericLatheSubMachine;
extern const std::string_view genericMill4AMachine;
extern const std::string_view genericMill5HeadTableBCMachine;
} // namespace ocam::embedded

namespace ocam::machine {
namespace {

constexpr double kPerMinute = 60.0;

std::string_view feedName(toolpath::FeedClass feedClass) {
    switch (feedClass) {
    case toolpath::FeedClass::Plunge:
        return "plunge";
    case toolpath::FeedClass::Ramp:
        return "ramp";
    case toolpath::FeedClass::LeadIn:
        return "lead-in";
    case toolpath::FeedClass::LeadOut:
        return "lead-out";
    case toolpath::FeedClass::Link:
        return "link";
    case toolpath::FeedClass::Probe:
        return "probing";
    default:
        return "cutting";
    }
}

Json vec3Json(const Vec3& v) {
    return Json::array({v.x(), v.y(), v.z()});
}
Vec3 vec3FromJson(const Json& j) {
    return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()};
}

std::string colourText(std::uint32_t rgb) {
    return fmt::format("#{:06x}", rgb & 0xFFFFFFu);
}
std::uint32_t colourFrom(const std::string& text) {
    return static_cast<std::uint32_t>(std::stoul(text.substr(1), nullptr, 16));
}

Json modelJson(const MachineModel& model) {
    Json axes = Json::array();
    for (const ModelAxis& a : model.axes) {
        Json j{{"name", a.name}, {"direction", vec3Json(a.direction)}};
        if (!a.parent.empty()) {
            j["parent"] = a.parent;
        }
        if (a.rotary()) {
            j["pivot"] = vec3Json(a.pivot);
        }
        axes.push_back(std::move(j));
    }
    Json parts = Json::array();
    for (const ModelPart& p : model.parts) {
        Json j{{"name", p.name},
               {"offset", vec3Json(p.offset)},
               {"rotation", vec3Json(p.rotation)},
               {"colour", colourText(p.colour)}};
        if (p.exact()) {
            j["step"] = p.step;
        } else {
            j["mesh"] = p.mesh;
        }
        if (!p.carrier.empty()) {
            j["carrier"] = p.carrier;
        }
        if (p.enclosure) {
            j["enclosure"] = true;
        }
        if (!p.shown) {
            j["shown"] = false;
        }
        if (!p.collides) {
            j["collides"] = false;
        }
        parts.push_back(std::move(j));
    }
    Json spindle{{"point", vec3Json(model.spindlePoint)}};
    if (!model.spindleCarrier.empty()) {
        spindle["carrier"] = model.spindleCarrier;
    }
    return {{"axes", std::move(axes)},
            {"parts", std::move(parts)},
            {"spindle", std::move(spindle)},
            {"mesh_tolerance", model.meshTolerance}};
}

Json latheJson(const LatheLayout& l) {
    Json j{{"turret",
            {{"centre", vec3Json(l.turret.centre)},
             {"axis", vec3Json(l.turret.axis)},
             {"diameter", l.turret.diameter},
             {"width", l.turret.width},
             {"radial_reach", l.turret.radialReach},
             {"axial_pitch", l.turret.axialPitch},
             {"axial_reach", l.turret.axialReach},
             {"block_radius", l.turret.blockRadius},
             {"block_along", l.turret.blockAlong},
             {"index_at_home", l.turret.indexAtHome}}},
           {"clearance", l.clearance},
           {"headstock", {{"diameter", l.headstockDiameter}, {"length", l.headstockLength}}},
           {"chuck",
            {{"diameter", l.chuckDiameter}, {"length", l.chuckLength}, {"grip", l.chuckGrip}}}};
    if (l.hasEnclosure()) {
        j["enclosure"] = {
            {"min", vec3Json(l.enclosureMin)}, {"max", vec3Json(l.enclosureMax)}, {"wall", l.wall}};
    }
    if (!l.turret.blockMounts.empty()) {
        j["turret"]["block_mounts"] = l.turret.blockMounts;
    }
    if (!l.turret.stationMounts.empty()) {
        Json groups = Json::array();
        for (const StationMounts& g : l.turret.stationMounts) {
            groups.push_back({{"stations", g.stations}, {"block_mounts", g.blockMounts}});
        }
        j["turret"]["station_mounts"] = std::move(groups);
    }
    if (l.turret.blockAcross != 0.0) {
        j["turret"]["block_across"] = l.turret.blockAcross;
    }
    if (l.workholdingMount != 0.0) {
        j["workholding_mount"] = {{"z", l.workholdingMount}};
    }
    if (const auto& t = l.tailstock) {
        j["tailstock"] = {{"z", t->z}, {"diameter", t->diameter}, {"length", t->length}};
        if (!t->quillInterface.empty()) {
            j["tailstock"]["quill_interface"] = t->quillInterface;
        }
        if (t->quillTravel > 0.0) {
            j["tailstock"]["quill_travel"] = t->quillTravel;
        }
    }
    if (const auto& s = l.subSpindle) {
        j["sub_spindle"] = {{"nose", s->nose},
                            {"diameter", s->diameter},
                            {"length", s->length},
                            {"chuck_diameter", s->chuckDiameter},
                            {"chuck_length", s->chuckLength},
                            {"grip", s->grip},
                            {"work_z2", s->workZ2}};
        if (s->maxSpindle > 0.0) {
            j["sub_spindle"]["min_rpm"] = s->minSpindle * kPerMinute;
            j["sub_spindle"]["max_rpm"] = s->maxSpindle * kPerMinute;
        }
    }
    return j;
}

LatheLayout latheFromJson(const Json& j) {
    LatheLayout l;
    const Json& t = j.at("turret");
    l.turret.centre = vec3FromJson(t.at("centre"));
    l.turret.axis = vec3FromJson(t.at("axis")).normalized();
    l.turret.diameter = t.value("diameter", 0.0);
    l.turret.width = t.value("width", 0.0);
    l.turret.radialReach = t.at("radial_reach").get<double>();
    l.turret.axialPitch = t.at("axial_pitch").get<double>();
    l.turret.axialReach = t.at("axial_reach").get<double>();
    l.turret.blockRadius = t.value("block_radius", l.turret.diameter / 2.0);
    l.turret.blockAlong = t.value("block_along", 0.0);
    l.turret.indexAtHome = t.value("index_at_home", false);
    l.turret.blockMounts = t.value("block_mounts", std::vector<std::string>());
    for (const Json& g : t.value("station_mounts", Json::array())) {
        l.turret.stationMounts.push_back({g.at("stations").get<std::vector<int>>(),
                                          g.at("block_mounts").get<std::vector<std::string>>()});
    }
    l.turret.blockAcross = t.value("block_across", 0.0);
    if (const auto it = j.find("workholding_mount"); it != j.end()) {
        l.workholdingMount = it->value("z", 0.0);
    }
    l.clearance = j.value("clearance", 1.0);
    if (const auto it = j.find("headstock"); it != j.end()) {
        l.headstockDiameter = it->value("diameter", 0.0);
        l.headstockLength = it->value("length", 0.0);
    }
    if (const auto it = j.find("chuck"); it != j.end()) {
        l.chuckDiameter = it->value("diameter", 0.0);
        l.chuckLength = it->value("length", 0.0);
        l.chuckGrip = it->value("grip", 0.0);
    }
    if (const auto it = j.find("enclosure"); it != j.end()) {
        l.enclosureMin = vec3FromJson(it->at("min"));
        l.enclosureMax = vec3FromJson(it->at("max"));
        l.wall = it->value("wall", 10.0);
    }
    if (const auto it = j.find("tailstock"); it != j.end()) {
        l.tailstock = LatheTailstockBody{it->at("z").get<double>(), it->value("diameter", 0.0),
                                         it->value("length", 0.0)};
        l.tailstock->quillInterface = it->value("quill_interface", std::string());
        l.tailstock->quillTravel = it->value("quill_travel", 0.0);
    }
    if (const auto it = j.find("sub_spindle"); it != j.end()) {
        LatheSubSpindle s;
        s.nose = it->at("nose").get<double>();
        s.diameter = it->value("diameter", 0.0);
        s.length = it->value("length", 0.0);
        s.chuckDiameter = it->value("chuck_diameter", 0.0);
        s.chuckLength = it->value("chuck_length", 0.0);
        s.grip = it->value("grip", 0.0);
        s.workZ2 = it->value("work_z2", 0.0);
        s.minSpindle = it->value("min_rpm", 0.0) / kPerMinute;
        s.maxSpindle = it->value("max_rpm", 0.0) / kPerMinute;
        l.subSpindle = s;
    }
    return l;
}

MachineModel modelFromJson(const Json& j) {
    MachineModel m;
    for (const Json& a : j.at("axes")) {
        ModelAxis axis;
        axis.name = a.at("name").get<std::string>();
        axis.parent = a.value("parent", "");
        axis.direction = vec3FromJson(a.at("direction"));
        if (axis.direction.norm() > 1e-12) {
            axis.direction.normalize();
        }
        if (a.contains("pivot")) {
            axis.pivot = vec3FromJson(a.at("pivot"));
        }
        m.axes.push_back(std::move(axis));
    }
    for (const Json& p : j.value("parts", Json::array())) {
        ModelPart part;
        part.name = p.at("name").get<std::string>();
        part.carrier = p.value("carrier", "");
        part.step = p.value("step", "");
        part.mesh = p.value("mesh", "");
        if (p.contains("offset")) {
            part.offset = vec3FromJson(p.at("offset"));
        }
        if (p.contains("rotation")) {
            part.rotation = vec3FromJson(p.at("rotation"));
        }
        if (p.contains("colour")) {
            part.colour = colourFrom(p.at("colour").get<std::string>());
        }
        part.collides = p.value("collides", true);
        part.shown = p.value("shown", true);
        part.enclosure = p.value("enclosure", false);
        m.parts.push_back(std::move(part));
    }
    m.meshTolerance = j.value("mesh_tolerance", m.meshTolerance);
    const Json& spindle = j.at("spindle");
    m.spindleCarrier = spindle.value("carrier", "");
    m.spindlePoint = vec3FromJson(spindle.at("point"));
    return m;
}

/// Rotary axes a kinematics type needs: tilting axis first (if any), then the rotating one.
std::vector<const char*> rotaryAxisNames(Kinematics k) {
    switch (k) {
    case Kinematics::Mill4A:
        return {"A"};
    case Kinematics::Mill5TableAC:
        return {"A", "C"};
    case Kinematics::Mill5TableBC:
    case Kinematics::Mill5HeadTableBC:
    case Kinematics::Mill5HeadBC:
        return {"B", "C"};
    case Kinematics::Mill3:
    case Kinematics::Lathe2:
        break;
    }
    return {};
}

} // namespace

geom3d::Convex Component::convex() const {
    if (shape == Shape::Cylinder) {
        return geom3d::Convex::cylinder(base, axis, radius, length);
    }
    geom3d::Aabb3 box;
    box.extend(min);
    box.extend(max);
    return geom3d::Convex::box(box);
}

std::string_view toString(Component::Attach attach) noexcept {
    switch (attach) {
    case Component::Attach::Head:
        return "head";
    case Component::Attach::Table:
        return "table";
    case Component::Attach::Cradle:
        return "cradle";
    }
    return "table";
}

std::string Machine::rotaryAxes() const {
    std::string out;
    for (const char* axisName : rotaryAxisNames(kinematics)) {
        out += axisName;
    }
    return out;
}

const Axis* Machine::axis(std::string_view axisName) const {
    auto it =
        std::find_if(axes.begin(), axes.end(), [&](const Axis& a) { return a.name == axisName; });
    return it == axes.end() ? nullptr : &*it;
}

std::vector<std::string> Machine::validate() const {
    std::vector<std::string> issues;
    if (orientation &&
        std::abs(direction(orientation->up).dot(direction(orientation->front))) > 0.5) {
        issues.push_back(fmt::format("the machine cannot have {} up and its front at {}: they must "
                                     "be two different axes",
                                     toString(orientation->up), toString(orientation->front)));
    }
    const std::vector<const char*> linear = kinematics == Kinematics::Lathe2
                                                ? std::vector<const char*>{"X", "Z"}
                                                : std::vector<const char*>{"X", "Y", "Z"};
    for (const char* axisName : linear) {
        const Axis* a = axis(axisName);
        if (!a) {
            issues.push_back(fmt::format("machine has no {} axis", axisName));
        } else if (a->max <= a->min) {
            issues.push_back(fmt::format("{} axis travel must be positive", axisName));
        }
    }
    if (isLathe()) {
        if (const Axis* y = axis("Y"); y && y->max <= y->min) {
            issues.emplace_back("Y axis travel must be positive");
        }
    }
    if (lathe) {
        if (!isLathe()) {
            issues.emplace_back("only a lathe has a lathe layout");
        }
        const LatheTurret& t = lathe->turret;
        if (toolCapacity < 1) {
            issues.emplace_back("a turret needs stations: set the tool changer's capacity");
        }
        if (t.axis.norm() < 1e-9) {
            issues.emplace_back("the turret's index line needs a direction");
        }
        if (t.diameter < 0.0 || t.width < 0.0 || t.radialReach <= 0.0 || t.axialPitch < 0.0 ||
            t.axialReach <= 0.0) {
            issues.emplace_back("the turret's sizes must be positive");
        }
        if (lathe->clearance < 0.0) {
            issues.emplace_back("the machine's clearance must not be negative");
        }
        if (lathe->tailstock && lathe->tailstock->quillTravel < 0.0) {
            issues.emplace_back("the tailstock's quill travel must not be negative");
        }
        if (const auto& s = lathe->subSpindle;
            s && (s->minSpindle < 0.0 || s->maxSpindle < 0.0 ||
                  (s->maxSpindle > 0.0 && s->minSpindle > s->maxSpindle))) {
            issues.emplace_back("the sub-spindle's lowest speed must not be above its highest");
        }
        std::vector<int> listed;
        for (const StationMounts& g : t.stationMounts) {
            for (const int station : g.stations) {
                if (station < 1 || station > toolCapacity) {
                    issues.push_back(fmt::format("the turret's blocks are said for station {}, and "
                                                 "it has stations 1 to {}",
                                                 station, toolCapacity));
                } else if (std::find(listed.begin(), listed.end(), station) != listed.end()) {
                    issues.push_back(fmt::format(
                        "the blocks station {} takes are said more than once", station));
                }
                listed.push_back(station);
            }
            if (g.blockMounts.empty()) {
                issues.emplace_back("stations that take other blocks need the kinds they take");
            }
        }
    }
    if (liveTooling && !isLathe()) {
        issues.emplace_back("only a lathe has live tooling here: a mill's tool always turns");
    }
    if (liveTooling && liveMaxSpindle <= 0.0) {
        issues.emplace_back("live tooling needs its top speed");
    }
    if (hasRotary()) {
        for (const char* axisName : rotaryAxisNames(kinematics)) {
            const Axis* a = axis(axisName);
            if (!a) {
                issues.push_back(
                    fmt::format("{} kinematics needs a {} axis", toString(kinematics), axisName));
            } else if (a->max < a->min) {
                issues.push_back(fmt::format("{} axis limits are reversed", axisName));
            }
        }
    }
    if (maxSpindle <= 0.0 || minSpindle < 0.0 || minSpindle > maxSpindle) {
        issues.emplace_back("spindle speed range is invalid");
    }
    if (maxFeed <= 0.0 || rapidRate <= 0.0) {
        issues.emplace_back("feed and rapid rates must be positive");
    }
    if (pivotLength < 0.0) {
        issues.emplace_back("the pivot length must not be negative");
    }
    for (const std::string& issue : validateModel(*this)) {
        issues.push_back(issue);
    }
    for (const Component& c : components) {
        const bool empty = c.shape == Component::Shape::Cylinder
                               ? (c.radius <= 0.0 || c.length <= 0.0 || c.axis.norm() < 1e-9)
                               : (c.max.array() <= c.min.array()).any();
        if (empty) {
            issues.push_back(fmt::format("component '{}' has no volume", c.name));
        }
    }
    return issues;
}

const std::vector<std::string>& LatheTurret::blockMountsAt(int station) const {
    for (const StationMounts& g : stationMounts) {
        if (std::find(g.stations.begin(), g.stations.end(), station) != g.stations.end()) {
            return g.blockMounts;
        }
    }
    return blockMounts;
}

bool sameMountName(std::string_view a, std::string_view b) {
    const auto squeezed = [](std::string_view s) {
        std::string out;
        for (const char c : s) {
            if (!std::isspace(static_cast<unsigned char>(c))) {
                out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
            }
        }
        return out;
    };
    return squeezed(a) == squeezed(b);
}

const FormatSpec& machineFormat() {
    static const FormatSpec spec = [] {
        FormatSpec s{"oxcam.machine", kMachineVersion, {}, {"opencam.machine"}};
        // Version 2 only added the optional `model`, version 3 the parts' optional `shown`.
        s.migrations[1] = [](Json&) {
        };
        s.migrations[2] = [](Json&) {
        };
        // Version 4 dropped the model's table (its carrier and origin): where the work sits on
        // the machine is each setup's placement now (DL-0037).
        s.migrations[3] = [](Json& doc) {
            if (doc.contains("model")) {
                doc["model"].erase("table");
            }
        };
        // Version 5 added STEP parts (`step`) and `mesh_tolerance`: mesh parts stay as they are.
        s.migrations[4] = [](Json&) {
        };
        // Version 6 added the optional default post (`post`).
        s.migrations[5] = [](Json&) {
        };
        // Version 7 added lathes' optional live tooling: without it a machine has none.
        s.migrations[6] = [](Json&) {
        };
        // Version 8 added lathes' optional layout: without it nothing changes.
        s.migrations[7] = [](Json&) {
        };
        // Version 9 added where a block seats on a turret: without it, at the disc's middle.
        s.migrations[8] = [](Json&) {
        };
        // Version 10 added the kinds of block a turret takes and what a quill takes: without
        // them the machine does not say.
        s.migrations[9] = [](Json&) {
        };
        // Version 11 added the kinds of block per station, the seat's sideways offset and where
        // workholding seats: without them every station takes the turret's kinds, the seat is on
        // the index line's plane, and workholding seats on the nose.
        s.migrations[10] = [](Json&) {
        };
        // Version 12 added how the machine stands: without it, as its kinematics.
        s.migrations[11] = [](Json&) {
        };
        // Version 13 added the machine's revision: without it, never counted (0).
        s.migrations[12] = [](Json&) {
        };
        // Version 14 added which parts are the enclosure: without it, none.
        s.migrations[13] = [](Json&) {
        };
        // Version 15 added how far the tailstock's quill goes out: without it, not said.
        s.migrations[14] = [](Json&) {
        };
        // Version 16 added the sub-spindle's speed range and the Z2 axis's travel: without them,
        // not said (ADR-0007).
        s.migrations[15] = [](Json&) {
        };
        return s;
    }();
    return spec;
}

const SchemaValidator& machineSchema() {
    static const SchemaValidator validator(
        parseJson(embedded::machineSchemaV16, "machine.v16.schema.json"));
    return validator;
}

Json toJson(const Machine& m) {
    Json axes = Json::array();
    Json doc;
    for (const Axis& a : m.axes) {
        Json j{{"name", a.name}, {"min", a.min}, {"max", a.max}};
        if (a.maxRate > 0.0) {
            j["max_rate"] = a.maxRate * kPerMinute;
        }
        axes.push_back(std::move(j));
    }
    doc = {{"format", machineFormat().format},
           {"version", kMachineVersion},
           {"id", m.id},
           {"name", m.name},
           {"vendor", m.vendor},
           {"kinematics", std::string(toString(m.kinematics))},
           {"axes", std::move(axes)},
           {"spindle",
            {{"min_rpm", m.minSpindle * kPerMinute}, {"max_rpm", m.maxSpindle * kPerMinute}}},
           {"feed", {{"max", m.maxFeed * kPerMinute}, {"rapid", m.rapidRate * kPerMinute}}},
           {"tool_changer", {{"capacity", m.toolCapacity}, {"change_time", m.toolChangeTime}}},
           {"work_offsets", m.workOffsets}};
    if (m.revision > 0) {
        doc["revision"] = m.revision;
    }
    if (m.orientation) {
        doc["orientation"] = {{"up", std::string(toString(m.orientation->up))},
                              {"front", std::string(toString(m.orientation->front))}};
    }
    if (m.pivotLength > 0.0) {
        doc["pivot_length"] = m.pivotLength;
    }
    if (m.liveTooling) {
        doc["live_tool"] = {{"max_rpm", m.liveMaxSpindle * kPerMinute}};
    }
    if (m.lathe) {
        doc["lathe"] = latheJson(*m.lathe);
    }
    if (!m.post.empty()) {
        doc["post"] = m.post;
    }
    if (m.postOptions.is_object() && !m.postOptions.empty()) {
        doc["post_options"] = m.postOptions;
    }
    if (!m.components.empty()) {
        Json comps = Json::array();
        for (const Component& c : m.components) {
            Json j{{"name", c.name}, {"attach", std::string(toString(c.attach))}};
            if (c.shape == Component::Shape::Cylinder) {
                j["cylinder"] = {{"base", vec3Json(c.base)},
                                 {"axis", vec3Json(c.axis)},
                                 {"radius", c.radius},
                                 {"length", c.length}};
            } else {
                j["box"] = {{"min", vec3Json(c.min)}, {"max", vec3Json(c.max)}};
            }
            comps.push_back(std::move(j));
        }
        doc["components"] = std::move(comps);
    }
    if (!m.model.empty()) {
        doc["model"] = modelJson(m.model);
    }
    return doc;
}

Machine machineFromJson(Json doc, std::string_view sourceName) {
    machineFormat().upgrade(doc, sourceName);
    machineSchema().validate(doc, sourceName);
    try {
        Machine m;
        m.id = doc.at("id").get<std::string>();
        m.name = doc.at("name").get<std::string>();
        m.revision = doc.value("revision", 0);
        m.vendor = doc.value("vendor", "");
        const std::string kin = doc.at("kinematics").get<std::string>();
        m.kinematics = kin == "mill5_table_ac"        ? Kinematics::Mill5TableAC
                       : kin == "mill5_table_bc"      ? Kinematics::Mill5TableBC
                       : kin == "lathe2"              ? Kinematics::Lathe2
                       : kin == "mill4_a"             ? Kinematics::Mill4A
                       : kin == "mill5_head_table_bc" ? Kinematics::Mill5HeadTableBC
                       : kin == "mill5_head_bc"       ? Kinematics::Mill5HeadBC
                                                      : Kinematics::Mill3;
        if (const auto it = doc.find("orientation"); it != doc.end()) {
            // The schema allows only the six names.
            m.orientation = MachineOrientation{
                axisSenseFromString(it->at("up").get<std::string>()).value_or(AxisSense::PlusZ),
                axisSenseFromString(it->at("front").get<std::string>())
                    .value_or(AxisSense::MinusY)};
        }
        for (const Json& a : doc.at("axes")) {
            m.axes.push_back({a.at("name").get<std::string>(), a.at("min").get<double>(),
                              a.at("max").get<double>(), a.value("max_rate", 0.0) / kPerMinute});
        }
        const Json& spindle = doc.at("spindle");
        m.minSpindle = spindle.value("min_rpm", 0.0) / kPerMinute;
        m.maxSpindle = spindle.at("max_rpm").get<double>() / kPerMinute;
        m.maxFeed = doc.at("feed").at("max").get<double>() / kPerMinute;
        m.rapidRate = doc.at("feed").at("rapid").get<double>() / kPerMinute;
        if (doc.contains("tool_changer")) {
            m.toolCapacity = doc.at("tool_changer").value("capacity", 0);
            m.toolChangeTime = doc.at("tool_changer").value("change_time", 0.0);
        }
        m.workOffsets = doc.value("work_offsets", 6);
        m.pivotLength = doc.value("pivot_length", 0.0);
        if (const auto it = doc.find("live_tool"); it != doc.end()) {
            m.liveTooling = true;
            m.liveMaxSpindle = it->at("max_rpm").get<double>() / kPerMinute;
        }
        if (const auto it = doc.find("lathe"); it != doc.end()) {
            m.lathe = latheFromJson(*it);
        }
        m.post = doc.value("post", std::string());
        m.postOptions = doc.value("post_options", Json::object());
        for (const Json& j : doc.value("components", Json::array())) {
            Component c;
            c.name = j.at("name").get<std::string>();
            const std::string attach = j.at("attach").get<std::string>();
            c.attach = attach == "head"     ? Component::Attach::Head
                       : attach == "cradle" ? Component::Attach::Cradle
                                            : Component::Attach::Table;
            if (const auto cyl = j.find("cylinder"); cyl != j.end()) {
                c.shape = Component::Shape::Cylinder;
                c.base = vec3FromJson(cyl->at("base"));
                c.axis = vec3FromJson(cyl->at("axis")).normalized();
                c.radius = cyl->at("radius").get<double>();
                c.length = cyl->at("length").get<double>();
            } else {
                c.shape = Component::Shape::Box;
                c.min = vec3FromJson(j.at("box").at("min"));
                c.max = vec3FromJson(j.at("box").at("max"));
            }
            m.components.push_back(std::move(c));
        }
        if (const auto model = doc.find("model"); model != doc.end()) {
            m.model = modelFromJson(*model);
        }
        if (const auto issues = m.validate(); !issues.empty()) {
            throw Error(ErrorCode::Parse, fmt::format("{}: {}", sourceName, issues.front()));
        }
        return m;
    } catch (const Json::exception& e) {
        throw Error(ErrorCode::Parse, fmt::format("{}: {}", sourceName, e.what()));
    }
}

Machine loadMachine(const std::filesystem::path& path) {
    const std::u8string u8 = path.filename().u8string();
    const std::string name(u8.begin(), u8.end());
    Machine m = machineFromJson(parseJson(readFileBytes(path), name), name);
    const std::filesystem::path folder = meshFolder(path);
    attachPartFiles(
        m,
        [&](const std::string& fileName) -> std::optional<std::string> {
            const std::filesystem::path file = folder / fileName;
            std::error_code ec;
            if (!std::filesystem::is_regular_file(file, ec)) {
                return std::nullopt;
            }
            return readFileBytes(file);
        },
        name);
    return m;
}

const Machine& genericMill3() {
    static const Machine machine =
        machineFromJson(parseJson(embedded::genericMill3Machine, "generic-3axis-vmc.json"),
                        "generic-3axis-vmc.json");
    return machine;
}

const Machine& genericMill5TableAC() {
    static const Machine machine = machineFromJson(
        parseJson(embedded::genericMill5TableACMachine, "generic-5axis-trunnion-ac.json"),
        "generic-5axis-trunnion-ac.json");
    return machine;
}

const Machine& genericLathe2() {
    static const Machine machine =
        machineFromJson(parseJson(embedded::genericLathe2Machine, "generic-2axis-lathe.json"),
                        "generic-2axis-lathe.json");
    return machine;
}

const Machine& genericLatheLive() {
    static const Machine machine =
        machineFromJson(parseJson(embedded::genericLatheLiveMachine, "generic-lathe-live.json"),
                        "generic-lathe-live.json");
    return machine;
}

const Machine& genericLatheLiveY() {
    static const Machine machine =
        machineFromJson(parseJson(embedded::genericLatheLiveYMachine, "generic-lathe-live-y.json"),
                        "generic-lathe-live-y.json");
    return machine;
}

const Machine& genericLatheTurret() {
    static const Machine machine =
        machineFromJson(parseJson(embedded::genericLatheTurretMachine, "generic-lathe-turret.json"),
                        "generic-lathe-turret.json");
    return machine;
}

const Machine& genericLatheSub() {
    static const Machine machine =
        machineFromJson(parseJson(embedded::genericLatheSubMachine, "generic-lathe-sub.json"),
                        "generic-lathe-sub.json");
    return machine;
}

const Machine& genericMill4A() {
    static const Machine machine =
        machineFromJson(parseJson(embedded::genericMill4AMachine, "generic-4axis-vmc.json"),
                        "generic-4axis-vmc.json");
    return machine;
}

const Machine& genericMill5HeadTableBC() {
    static const Machine machine = machineFromJson(
        parseJson(embedded::genericMill5HeadTableBCMachine, "generic-5axis-headtable-bc.json"),
        "generic-5axis-headtable-bc.json");
    return machine;
}

std::vector<const Machine*> builtinMachines() {
    return {
        &genericMill3(),   &genericMill4A(),    &genericMill5TableAC(), &genericMill5HeadTableBC(),
        &genericLathe2(),  &genericLatheLive(), &genericLatheLiveY(),   &genericLatheTurret(),
        &genericLatheSub()};
}

const Machine* builtinMachine(std::string_view id) {
    if (id.empty() || id == "generic-3axis") {
        return &genericMill3();
    }
    for (const Machine* m : builtinMachines()) {
        if (m->id == id) {
            return m;
        }
    }
    return nullptr;
}

std::string_view toString(AxisSense a) noexcept {
    switch (a) {
    case AxisSense::PlusX:
        return "+X";
    case AxisSense::MinusX:
        return "-X";
    case AxisSense::PlusY:
        return "+Y";
    case AxisSense::MinusY:
        return "-Y";
    case AxisSense::PlusZ:
        return "+Z";
    case AxisSense::MinusZ:
        return "-Z";
    }
    return "+Z";
}

std::optional<AxisSense> axisSenseFromString(std::string_view text) {
    bool minus = false;
    if (!text.empty() && (text.front() == '+' || text.front() == '-')) {
        minus = text.front() == '-';
        text.remove_prefix(1);
    }
    if (text.size() != 1) {
        return std::nullopt;
    }
    switch (std::toupper(static_cast<unsigned char>(text.front()))) {
    case 'X':
        return minus ? AxisSense::MinusX : AxisSense::PlusX;
    case 'Y':
        return minus ? AxisSense::MinusY : AxisSense::PlusY;
    case 'Z':
        return minus ? AxisSense::MinusZ : AxisSense::PlusZ;
    default:
        return std::nullopt;
    }
}

Vec3 direction(AxisSense a) noexcept {
    switch (a) {
    case AxisSense::PlusX:
        return Vec3::UnitX();
    case AxisSense::MinusX:
        return -Vec3::UnitX();
    case AxisSense::PlusY:
        return Vec3::UnitY();
    case AxisSense::MinusY:
        return -Vec3::UnitY();
    case AxisSense::PlusZ:
        return Vec3::UnitZ();
    case AxisSense::MinusZ:
        return -Vec3::UnitZ();
    }
    return Vec3::UnitZ();
}

Mat3 MachineOrientation::worldAxes() const noexcept {
    const Vec3 upward = direction(up);
    // Two axes along one line (refused by Machine::validate) keep the front a right angle away.
    Vec3 away = -direction(front);
    if (std::abs(away.dot(upward)) > 0.5) {
        away = upward.unitOrthogonal();
    }
    Mat3 axes;
    axes.col(0) = away.cross(upward);
    axes.col(1) = away;
    axes.col(2) = upward;
    return axes;
}

MachineOrientation defaultOrientation(Kinematics k) noexcept {
    if (k == Kinematics::Lathe2) {
        return {AxisSense::PlusY, AxisSense::MinusX};
    }
    return {AxisSense::PlusZ, AxisSense::MinusY};
}

std::string_view toString(Kinematics k) noexcept {
    switch (k) {
    case Kinematics::Mill3:
        return "mill3";
    case Kinematics::Mill5TableAC:
        return "mill5_table_ac";
    case Kinematics::Mill5TableBC:
        return "mill5_table_bc";
    case Kinematics::Lathe2:
        return "lathe2";
    case Kinematics::Mill4A:
        return "mill4_a";
    case Kinematics::Mill5HeadTableBC:
        return "mill5_head_table_bc";
    case Kinematics::Mill5HeadBC:
        return "mill5_head_bc";
    }
    return "mill3";
}

namespace {

Mat3 rotX(double t) {
    return Eigen::AngleAxisd(t, Vec3::UnitX()).toRotationMatrix();
}
Mat3 rotY(double t) {
    return Eigen::AngleAxisd(t, Vec3::UnitY()).toRotationMatrix();
}
Mat3 rotZ(double t) {
    return Eigen::AngleAxisd(t, Vec3::UnitZ()).toRotationMatrix();
}

bool withinLimits(const Machine& m, const char* name, double angle) {
    const Axis* a = m.axis(name);
    if (!a) {
        return std::abs(angle) < 1e-9;
    }
    const double deg = angle * 180.0 / kPi;
    return deg >= a->min - 1e-9 && deg <= a->max + 1e-9;
}

} // namespace

Mat3 tableRotation(const Machine& machine, const IndexAngles& angles) {
    switch (machine.kinematics) {
    case Kinematics::Mill5TableAC:
        return rotX(angles.a) * rotZ(angles.c);
    case Kinematics::Mill5TableBC:
        return rotY(angles.b) * rotZ(angles.c);
    case Kinematics::Mill4A:
        return rotX(angles.a);
    case Kinematics::Mill5HeadTableBC:
        return rotZ(angles.c);
    case Kinematics::Mill3:
    case Kinematics::Lathe2:
    case Kinematics::Mill5HeadBC:
        break;
    }
    return Mat3::Identity();
}

Mat3 cradleRotation(const Machine& machine, const IndexAngles& angles) {
    switch (machine.kinematics) {
    case Kinematics::Mill5TableAC:
        return rotX(angles.a);
    case Kinematics::Mill5TableBC:
        return rotY(angles.b);
    default:
        break;
    }
    return Mat3::Identity();
}

Mat3 headRotation(const Machine& machine, const IndexAngles& angles) {
    switch (machine.kinematics) {
    case Kinematics::Mill5HeadTableBC:
        return rotY(angles.b);
    case Kinematics::Mill5HeadBC:
        return rotZ(angles.c) * rotY(angles.b);
    default:
        break;
    }
    return Mat3::Identity();
}

std::optional<IndexAngles> indexAngles(const Machine& machine, const Vec3& direction) {
    if (!machine.canIndex()) {
        return (direction.normalized() - Vec3::UnitZ()).norm() < 1e-9 ? std::optional(IndexAngles{})
                                                                      : std::nullopt;
    }
    const Vec3 v = direction.normalized();
    const double horizontal = std::hypot(v.x(), v.y());
    std::vector<IndexAngles> candidates;
    const bool ac = machine.kinematics == Kinematics::Mill5TableAC;
    // Turn the table about Z to bring v into the tilt plane, then tilt it upright. The second
    // solution turns half a revolution further and tilts the other way.
    double c0 = 0.0;
    if (horizontal > 1e-12) {
        c0 = ac ? std::atan2(v.x(), v.y()) : std::atan2(-v.y(), v.x());
    }
    for (const double c : {c0, c0 - kPi, c0 + kPi, c0 - 2.0 * kPi, c0 + 2.0 * kPi}) {
        const Vec3 w = rotZ(c) * v;
        IndexAngles s;
        s.c = c;
        if (ac) {
            s.a = std::atan2(w.y(), w.z());
        } else {
            s.b = std::atan2(-w.x(), w.z());
        }
        if ((tableRotation(machine, s) * v - Vec3::UnitZ()).norm() > 1e-9) {
            continue;
        }
        if (withinLimits(machine, ac ? "A" : "B", ac ? s.a : s.b) &&
            withinLimits(machine, "C", s.c)) {
            candidates.push_back(s);
        }
    }
    if (candidates.empty()) {
        return std::nullopt;
    }
    return *std::min_element(
        candidates.begin(), candidates.end(), [](const IndexAngles& x, const IndexAngles& y) {
            const double tx = std::abs(x.a) + std::abs(x.b), ty = std::abs(y.a) + std::abs(y.b);
            if (std::abs(tx - ty) > 1e-9) {
                return tx < ty;
            }
            return std::abs(x.c) < std::abs(y.c);
        });
}

SolveResult solve(const Machine& machine, const toolpath::Toolpath& path, const Mat3& wcsInTable) {
    if (!machine.canIndex()) {
        return solve(machine, path);
    }
    const auto angles = indexAngles(machine, wcsInTable.col(2));
    if (!angles) {
        SolveResult r;
        r.path = path;
        r.errors.push_back("the setup's tool axis cannot be reached within the rotary axis limits");
        return r;
    }
    // After indexing the WCS Z axis is vertical; X and Y are turned about it by psi.
    const Vec3 x = tableRotation(machine, *angles) * wcsInTable.col(0);
    const double psi = std::atan2(x.y(), x.x());
    const Mat3 turn = rotZ(psi);
    toolpath::Toolpath rotated = path;
    rotated.start = turn * path.start;
    if (std::abs(psi) > 1e-12) {
        // Arcs in a vertical plane (ZX, YZ) would not stay in one after the turn: short lines.
        std::vector<toolpath::Move> moves;
        Vec3 pos = path.start;
        for (const auto& m : path.moves) {
            if (m.isArc() && m.plane != toolpath::ArcPlane::XY) {
                const auto pts = toolpath::sampleArc(pos, m, 1e-3);
                for (std::size_t k = 1; k < pts.size(); ++k) {
                    toolpath::Move line = m;
                    line.kind = toolpath::MoveKind::Linear;
                    line.plane = toolpath::ArcPlane::XY;
                    line.end = pts[k];
                    moves.push_back(line);
                }
            } else {
                moves.push_back(m);
            }
            if (m.kind != toolpath::MoveKind::Dwell) {
                pos = m.end;
            }
        }
        rotated.moves = std::move(moves);
    }
    for (auto& m : rotated.moves) {
        m.end = turn * m.end;
        m.center = turn * m.center;
    }
    SolveResult r = solve(machine, rotated);
    r.index = angles;
    return r;
}

SolveResult solve(const Machine& machine, const toolpath::Toolpath& path) {
    SolveResult result;
    result.path = path;
    toolpath::Toolpath& out = result.path;

    if (machine.isLathe()) {
        // Turning moves lie in the XZ plane (x = radius).
        bool offPlane = std::abs(out.start.y()) > 1e-9;
        for (const auto& move : out.moves) {
            offPlane = offPlane || std::abs(move.end.y()) > 1e-9;
        }
        if (offPlane) {
            result.errors.push_back("lathe moves must lie in the XZ plane (Y = 0)");
        }
        // A tool standing still works on the spindle axis (DL-0043).
        const bool onAxis = std::abs(out.start.x()) <= 1e-9 &&
                            std::all_of(out.moves.begin(), out.moves.end(), [](const auto& move) {
                                return std::abs(move.end.x()) <= 1e-9;
                            });
        if (!path.isTurning() && !path.moves.empty() && !onAxis) {
            result.errors.push_back(
                "milling operations and off-axis holes need live tooling (mill-turn, phase 10)");
        }
    }
    // 3-axis: the tool axis must stay +Z.
    for (const auto& move : out.moves) {
        if ((move.axis - Vec3::UnitZ()).norm() > 1e-9) {
            result.errors.push_back(
                machine.hasRotary()
                    ? "the toolpath tilts the tool; multi-axis toolpaths need the setup's WCS"
                    : "the toolpath tilts the tool; this machine has no rotary axes");
            break;
        }
    }
    detail::applyLimits(machine, result);
    return result;
}

void detail::applyLimits(const Machine& machine, SolveResult& result) {
    const toolpath::Toolpath& out = result.path;
    // Motion span against axis travel (the work offset position is set at the machine).
    const toolpath::Stats stats = toolpath::computeStats(out);
    if (!stats.bounds.isEmpty()) {
        const Vec3 span = stats.bounds.size();
        const char* names[] = {"X", "Y", "Z"};
        for (int i = 0; i < 3; ++i) {
            const Axis* a = machine.axis(names[i]);
            if (a && span[i] > a->travel() + 1e-9) {
                result.errors.push_back(
                    fmt::format("motion in {} spans {:.3f} mm but the axis travel is {:.3f} mm",
                                names[i], span[i], a->travel()));
            }
        }
    }

    // ADR-0004: speeds and feeds leave the solve exactly as the operation set them. A value the
    // machine cannot run refuses the post, naming the value, the limit and where to change them.
    constexpr const char* where = "the cutting preset or the operation's speeds and feeds";
    if (out.live) {
        if (!machine.liveTooling) {
            result.errors.push_back(
                fmt::format("machine '{}' has no live tooling: set the operation's Tool drive to "
                            "Spindle, or choose a machine with it in the setup",
                            machine.name));
        } else if (out.spindleSpeed > machine.liveMaxSpindle) {
            result.errors.push_back(fmt::format(
                "live tool speed {:.6g} rpm is above the machine's live tool max {:.6g} rpm: "
                "lower it in {}, or raise the live tool max in the machine library",
                out.spindleSpeed * kPerMinute, machine.liveMaxSpindle * kPerMinute, where));
        }
    } else if (out.spindleSpeed > machine.maxSpindle) {
        result.errors.push_back(fmt::format(
            "spindle speed {:.6g} rpm is above the machine's spindle max {:.6g} rpm: lower it in "
            "{}, or raise the spindle max in the machine library",
            out.spindleSpeed * kPerMinute, machine.maxSpindle * kPerMinute, where));
    } else if (!out.live && out.spindleSpeed > 0.0 && out.spindleSpeed < machine.minSpindle) {
        result.errors.push_back(fmt::format(
            "spindle speed {:.6g} rpm is below the machine's spindle min {:.6g} rpm: raise it in "
            "{}, or lower the spindle min in the machine library",
            out.spindleSpeed * kPerMinute, machine.minSpindle * kPerMinute, where));
    }

    // The fastest feed of each kind above the limit (in FeedClass order), cycles last (a cycle
    // move carries its cycle's feed: reported once, as the cycle's).
    std::map<toolpath::FeedClass, double> fastest;
    for (const auto& move : out.moves) {
        if (move.kind != toolpath::MoveKind::Rapid && move.kind != toolpath::MoveKind::Cycle &&
            move.feedRate > machine.maxFeed) {
            double& f = fastest[move.feedClass];
            f = std::max(f, move.feedRate);
        }
    }
    double fastestCycle = 0.0;
    for (const auto& cycle : out.cycles) {
        if (cycle.feedRate > machine.maxFeed) {
            fastestCycle = std::max(fastestCycle, cycle.feedRate);
        }
    }
    const auto feedError = [&](std::string_view what, double feed) {
        result.errors.push_back(fmt::format(
            "{} feed {:.6g} mm/min is above the machine's max feed {:.6g} mm/min: lower it in {}, "
            "or raise the max feed in the machine library",
            what, feed * kPerMinute, machine.maxFeed * kPerMinute, where));
    };
    for (const auto& [feedClass, feed] : fastest) {
        feedError(feedName(feedClass), feed);
    }
    if (fastestCycle > 0.0) {
        feedError("cycle", fastestCycle);
    }
}

} // namespace ocam::machine
