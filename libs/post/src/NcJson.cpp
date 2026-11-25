// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// JSON form of the NC intermediate representation: the post API, exactly what a post receives.
// Units follow the file conventions: lengths in mm, feeds in mm/min, spindle speed in rev/min,
// dwell in seconds, angles in degrees. See docs/design-log/0015 and 0039.

#include "NcFields.hpp"
#include "ocam/post/Post.hpp"

#include <ocam/foundation/Error.hpp>

#include <cmath>
#include <limits>

namespace ocam::post {
namespace {

struct OpName {
    NcOp op;
    const char* name;
};
constexpr OpName kOps[] = {
    {NcOp::Comment, "comment"},
    {NcOp::ToolChange, "tool_change"},
    {NcOp::WorkOffset, "work_offset"},
    {NcOp::Spindle, "spindle"},
    {NcOp::SpindleOrient, "spindle_orient"},
    {NcOp::LiveTool, "live_tool"},
    {NcOp::LiveTap, "live_tap"},
    {NcOp::Polar, "polar"},
    {NcOp::Wrap, "wrap"},
    {NcOp::Coolant, "coolant"},
    {NcOp::ToolLength, "tool_length"},
    {NcOp::Rapid, "rapid"},
    {NcOp::Linear, "linear"},
    {NcOp::Arc, "arc"},
    {NcOp::Dwell, "dwell"},
    {NcOp::Cycle, "cycle"},
    {NcOp::CycleCancel, "cycle_cancel"},
    {NcOp::CompensationOn, "compensation_on"},
    {NcOp::CompensationOff, "compensation_off"},
    {NcOp::Index, "index"},
    {NcOp::HomeZ, "home_z"},
    {NcOp::TcpOff, "tcp_off"},
    {NcOp::Probe, "probe"},
    {NcOp::OperationBegin, "operation_begin"},
    {NcOp::OperationEnd, "operation_end"},
    {NcOp::Chuck, "chuck"},
    {NcOp::BarFeeder, "bar_feeder"},
    {NcOp::SubAxis, "sub_axis"},
    {NcOp::SpindleSync, "spindle_sync"},
};

constexpr double kDeg = 180.0 / kPi;

Json vec3(const Vec3& v) {
    return Json::array({v.x(), v.y(), v.z()});
}

Vec3 readVec3(const Json& j) {
    return Vec3(j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>());
}

std::string_view toString(NcOrientation o) noexcept {
    switch (o) {
    case NcOrientation::Indexed:
        return "indexed";
    case NcOrientation::Simultaneous:
        return "simultaneous";
    case NcOrientation::ThreeAxis:
        break;
    }
    return "3axis";
}

Json toolJson(const NcTool& t) {
    Json j{{"number", t.number},           {"name", t.name},
           {"comment", t.comment},         {"type", t.type},
           {"diameter", t.diameter},       {"corner_radius", t.cornerRadius},
           {"flutes", t.flutes},           {"thread_pitch", t.threadPitch},
           {"gauge_length", t.gaugeLength}};
    if (!t.insertShape.empty()) {
        j["insert"] = {{"shape", t.insertShape},
                       {"nose_radius", t.noseRadius},
                       {"orientation", t.insertOrientation}};
    }
    return j;
}

NcTool toolFromJson(const Json& j) {
    NcTool t;
    t.number = j.at("number").get<int>();
    t.name = j.value("name", std::string());
    t.comment = j.value("comment", std::string());
    t.type = j.value("type", std::string());
    t.diameter = j.value("diameter", 0.0);
    t.cornerRadius = j.value("corner_radius", 0.0);
    t.flutes = j.value("flutes", 0);
    t.threadPitch = j.value("thread_pitch", 0.0);
    t.gaugeLength = j.value("gauge_length", 0.0);
    if (const auto it = j.find("insert"); it != j.end()) {
        t.insertShape = it->value("shape", std::string());
        t.noseRadius = it->value("nose_radius", 0.0);
        t.insertOrientation = it->value("orientation", 0);
    }
    return t;
}

Json operationJson(const NcOperation& o) {
    Json j{{"id", o.id},
           {"name", o.name},
           {"comment", o.comment},
           {"strategy", o.strategy},
           {"tool", o.tool},
           {"coolant", std::string(tooling::toString(o.coolant))},
           {"work_offset", o.workOffset},
           {"orientation", std::string(toString(o.orientation))},
           {"spindle", o.subSpindle ? "sub" : "main"},
           {"bounds", {{"min", vec3(o.boundsMin)}, {"max", vec3(o.boundsMax)}}}};
    if (o.orientation == NcOrientation::Indexed) {
        j["index"] = {{"a", o.index.a * kDeg}, {"b", o.index.b * kDeg}, {"c", o.index.c * kDeg}};
    }
    j["options"] = o.options.is_object() ? o.options : Json::object();
    return j;
}

Error parseError(const std::string& what);

NcOperation operationFromJson(const Json& j) {
    NcOperation o;
    o.id = j.value("id", std::string());
    o.name = j.value("name", std::string());
    o.comment = j.value("comment", std::string());
    o.strategy = j.value("strategy", std::string());
    o.tool = j.value("tool", 0);
    const auto coolant = tooling::parseCoolant(j.value("coolant", std::string("off")));
    if (!coolant) {
        throw parseError("unknown coolant");
    }
    o.coolant = *coolant;
    o.workOffset = j.value("work_offset", 1);
    o.subSpindle = j.value("spindle", std::string("main")) == "sub";
    const std::string orientation = j.value("orientation", std::string("3axis"));
    o.orientation = orientation == "indexed"        ? NcOrientation::Indexed
                    : orientation == "simultaneous" ? NcOrientation::Simultaneous
                                                    : NcOrientation::ThreeAxis;
    if (const auto it = j.find("index"); it != j.end()) {
        o.index = machine::IndexAngles{it->value("a", 0.0) / kDeg, it->value("b", 0.0) / kDeg,
                                       it->value("c", 0.0) / kDeg};
    }
    if (const auto it = j.find("bounds"); it != j.end()) {
        o.boundsMin = readVec3(it->at("min"));
        o.boundsMax = readVec3(it->at("max"));
    }
    o.options = j.value("options", Json::object());
    return o;
}

/// Feeds and speeds in the file conventions: per minute.
Json machineJson(const NcMachine& m) {
    Json axes = Json::array();
    for (const machine::Axis& a : m.axes) {
        axes.push_back({{"name", a.name}, {"min", a.min}, {"max", a.max}});
    }
    return Json{{"id", m.id},
                {"name", m.name},
                {"kinematics", m.kinematics},
                {"axes", std::move(axes)},
                {"rotary_axes", m.rotaryAxes},
                {"spindle", {{"min", m.minSpindle * 60.0}, {"max", m.maxSpindle * 60.0}}},
                {"max_feed", m.maxFeed * 60.0},
                {"rapid", m.rapidRate * 60.0},
                {"tool_capacity", m.toolCapacity},
                {"work_offsets", m.workOffsets}};
}

NcMachine machineFromJson(const Json& j) {
    NcMachine m;
    m.id = j.value("id", std::string());
    m.name = j.value("name", std::string());
    m.kinematics = j.value("kinematics", std::string());
    for (const Json& a : j.value("axes", Json::array())) {
        machine::Axis axis;
        axis.name = a.at("name").get<std::string>();
        axis.min = a.at("min").get<double>();
        axis.max = a.at("max").get<double>();
        m.axes.push_back(std::move(axis));
    }
    m.rotaryAxes = j.value("rotary_axes", std::string());
    if (const auto it = j.find("spindle"); it != j.end()) {
        m.minSpindle = it->value("min", 0.0) / 60.0;
        m.maxSpindle = it->value("max", 0.0) / 60.0;
    }
    m.maxFeed = j.value("max_feed", 0.0) / 60.0;
    m.rapidRate = j.value("rapid", 0.0) / 60.0;
    m.toolCapacity = j.value("tool_capacity", 0);
    m.workOffsets = j.value("work_offsets", 0);
    return m;
}

double readCoordinate(const Json& j, const char* key) {
    if (!j.contains(key) || j.at(key).is_null()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return j.at(key).get<double>();
}

Error parseError(const std::string& what) {
    return Error(ErrorCode::Parse, "NC program JSON: " + what);
}

/// A move's feed class; version 1 documents have none: cutting.
toolpath::FeedClass readFeedClass(const Json& j) {
    if (!j.contains("feed_class")) {
        return toolpath::FeedClass::Cut;
    }
    const auto c = toolpath::parseFeedClass(j.at("feed_class").get<std::string>());
    if (!c) {
        throw parseError("unknown feed class");
    }
    return *c;
}

} // namespace

std::string_view toString(NcOp op) noexcept {
    for (const auto& o : kOps) {
        if (o.op == op) {
            return o.name;
        }
    }
    return "unknown";
}

std::optional<NcOp> parseNcOp(std::string_view text) noexcept {
    for (const auto& o : kOps) {
        if (text == o.name) {
            return o.op;
        }
    }
    return std::nullopt;
}

namespace detail {

void instructionFields(const NcInstr& i, FieldSink& sink) {
    sink.text("op", toString(i.op));
    auto coordinate = [&](const char* key, double v) {
        if (std::isfinite(v)) {
            sink.number(key, v);
        } else {
            sink.null(key); // the axis does not change
        }
    };
    auto rotary = [&](std::string_view axes) {
        sink.text("axes", axes);
        sink.number("a", i.rotary.x() * 180.0 / kPi);
        sink.number("b", i.rotary.y() * 180.0 / kPi);
        sink.number("c", i.rotary.z() * 180.0 / kPi);
    };
    auto xyz = [&](const Vec3& p) {
        coordinate("x", p.x());
        coordinate("y", p.y());
        coordinate("z", p.z());
        if (!i.axes.empty()) {
            rotary(i.axes);
        }
    };
    switch (i.op) {
    case NcOp::Comment:
        sink.text("text", i.text);
        break;
    case NcOp::ToolChange:
        sink.integer("tool", i.tool);
        sink.text("tool_name", i.toolName);
        break;
    case NcOp::WorkOffset:
        sink.integer("work_offset", i.workOffset);
        break;
    case NcOp::Spindle:
        sink.number("rpm", i.spindle * 60.0);
        if (i.reverse) {
            sink.boolean("reverse", true);
        }
        if (i.sub) {
            sink.text("spindle", "sub");
        }
        if (i.surfaceSpeed > 0.0) {
            sink.number("surface_speed", i.surfaceSpeed * 0.06); // m/min
            sink.number("max_rpm", i.maxSpindle * 60.0);
        }
        break;
    case NcOp::LiveTool:
        sink.number("rpm", i.spindle * 60.0);
        if (i.tapping) {
            sink.boolean("tapping", true);
        }
        if (i.milling) {
            sink.text("plane", i.plane == toolpath::ArcPlane::XY ? "XY" : "YZ");
        }
        break;
    case NcOp::Polar:
        sink.boolean("on", i.polarOn);
        if (i.polarOn) {
            xyz(i.target);
        }
        break;
    case NcOp::Wrap:
        sink.boolean("on", i.polarOn);
        if (i.polarOn) {
            sink.number("radius", i.wrapRadius);
        }
        break;
    case NcOp::Chuck:
        sink.text("spindle", i.sub ? "sub" : "main");
        sink.boolean("open", i.open);
        break;
    case NcOp::SubAxis:
        sink.number("z2", i.target.z());
        if (i.feed > 0.0) {
            sink.number("feed", i.feed * 60.0);
        } else {
            sink.null("feed"); // rapid
        }
        break;
    case NcOp::SpindleSync:
        sink.boolean("on", i.polarOn);
        if (i.syncAngle) {
            sink.number("angle", *i.syncAngle * 180.0 / kPi);
        } else {
            sink.null("angle");
        }
        break;
    case NcOp::BarFeeder:
        sink.boolean("on", i.polarOn);
        break;
    case NcOp::LiveTap:
        xyz(i.target);
        sink.number("rpm", i.spindle * 60.0);
        sink.number("pitch", i.pitch);
        sink.number("r_plane", i.rPlane);
        sink.boolean("radial", i.radial);
        if (i.reverse) {
            sink.boolean("left_hand", true);
        }
        break;
    case NcOp::Coolant:
        sink.text("coolant", tooling::toString(i.coolant));
        break;
    case NcOp::ToolLength:
        sink.integer("tool", i.tool);
        xyz(i.target);
        if (i.tcp) {
            sink.boolean("tcp", true);
        }
        break;
    case NcOp::Rapid:
        xyz(i.target);
        break;
    case NcOp::OperationBegin:
    case NcOp::OperationEnd:
        sink.integer("operation", i.operation);
        break;
    case NcOp::Linear:
        xyz(i.target);
        sink.number("feed", i.feed * 60.0);
        sink.text("feed_class", toolpath::toString(i.feedClass));
        if (i.pitch > 0.0) {
            sink.number("pitch", i.pitch);
        }
        if (i.inverseTime > 0.0) {
            sink.number("inverse_time", i.inverseTime);
        }
        break;
    case NcOp::Arc:
        xyz(i.target);
        sink.number("cx", i.center.x());
        sink.number("cy", i.center.y());
        sink.number("cz", i.center.z());
        sink.boolean("clockwise", i.clockwise);
        if (i.plane != toolpath::ArcPlane::XY) {
            sink.text("plane", i.plane == toolpath::ArcPlane::ZX ? "ZX" : "YZ");
        }
        sink.number("feed", i.feed * 60.0);
        sink.text("feed_class", toolpath::toString(i.feedClass));
        break;
    case NcOp::Dwell:
        sink.number("seconds", i.dwell);
        break;
    case NcOp::SpindleOrient:
        sink.number("angle", i.angle * kDeg);
        break;
    case NcOp::Cycle:
        xyz(i.target);
        sink.beginObject("cycle");
        sink.text("type", toolpath::toString(i.cycle.type));
        sink.number("top", i.cycle.topZ);
        sink.number("bottom", i.cycle.bottomZ);
        sink.number("retract", i.cycle.retractZ);
        sink.number("peck", i.cycle.peck);
        sink.number("chip_break_retract", i.cycle.chipBreakRetract);
        sink.number("dwell", i.cycle.dwell);
        sink.number("feed", i.cycle.feedRate * 60.0);
        sink.number("pitch", i.cycle.pitch);
        if (i.cycle.type == toolpath::CycleType::BackBore) {
            sink.number("shift_x", i.cycle.shift.x());
            sink.number("shift_y", i.cycle.shift.y());
            sink.number("orient", i.cycle.orient * kDeg);
        }
        sink.endObject();
        break;
    case NcOp::CompensationOn:
        sink.text("side", i.side == toolpath::Compensation::Left ? "left" : "right");
        sink.integer("tool", i.tool);
        break;
    case NcOp::Index:
        rotary(i.text);
        break;
    case NcOp::Probe: {
        const toolpath::ProbeCycle& c = i.probe;
        auto point = [&](const char* key, const Vec3& v) {
            sink.beginObject(key);
            sink.number("x", v.x());
            sink.number("y", v.y());
            sink.number("z", v.z());
            sink.endObject();
        };
        sink.integer("operation", i.operation);
        sink.text("feature", toolpath::toString(c.feature));
        sink.boolean("check", c.isCheck());
        sink.integer("work_offset", c.workOffset);
        sink.text("axes", c.axes);
        point("nominal", c.nominal);
        point("direction", c.direction);
        sink.number("size", c.size);
        sink.number("tolerance", c.tolerance);
        sink.number("approach", c.approach);
        sink.number("overtravel", c.overtravel);
        sink.number("fast_feed", c.fastFeed * 60.0);
        sink.number("measure_feed", c.measureFeed * 60.0);
        sink.number("stylus_diameter", c.stylusDiameter);
        sink.number("trigger_offset", c.triggerOffset);
        sink.beginArray("moves");
        for (const NcProbeMove& m : i.probeMoves) {
            sink.beginObject(nullptr);
            sink.text("kind", m.measure ? "measure" : "position");
            sink.number("x", m.target.x());
            sink.number("y", m.target.y());
            sink.number("z", m.target.z());
            sink.number("feed", m.feed * 60.0);
            if (m.measure) {
                point("expected", m.expected);
            }
            sink.endObject();
        }
        sink.endArray();
        break;
    }
    case NcOp::CycleCancel:
    case NcOp::CompensationOff:
    case NcOp::HomeZ:
    case NcOp::TcpOff:
        break;
    }
}

Json programHeaderJson(const NcProgram& program) {
    Json tools = Json::array();
    for (const NcTool& t : program.tools) {
        tools.push_back(toolJson(t));
    }
    Json operations = Json::array();
    for (const NcOperation& o : program.operations) {
        operations.push_back(operationJson(o));
    }
    return Json{{"format", "oxcam-ncprogram"},
                {"version", kNcProgramJsonVersion},
                {"units",
                 {{"length", "mm"},
                  {"feed", "mm/min"},
                  {"spindle", "rev/min"},
                  {"time", "s"},
                  {"angle", "deg"}}},
                {"name", program.name},
                {"number", program.number},
                {"project", program.project},
                {"setup", program.setup},
                {"time", program.time},
                {"tools", std::move(tools)},
                {"operations", std::move(operations)},
                {"machine", program.machine ? machineJson(*program.machine) : Json(nullptr)},
                {"warnings", program.warnings}};
}

} // namespace detail

namespace {

/// Builds the JSON object of an instruction.
class JsonSink final : public detail::FieldSink {
public:
    explicit JsonSink(Json& object) : stack_{&object} {}
    void number(const char* key, double value) override { (*stack_.back())[key] = value; }
    void integer(const char* key, int value) override { (*stack_.back())[key] = value; }
    void boolean(const char* key, bool value) override { (*stack_.back())[key] = value; }
    void text(const char* key, std::string_view value) override {
        (*stack_.back())[key] = std::string(value);
    }
    void null(const char* key) override { (*stack_.back())[key] = nullptr; }
    void beginObject(const char* key) override {
        Json& parent = *stack_.back();
        if (!key) { // the next element of an array
            parent.push_back(Json::object());
            stack_.push_back(&parent.back());
            return;
        }
        Json& object = parent[key] = Json::object();
        stack_.push_back(&object);
    }
    void endObject() override { stack_.pop_back(); }
    void beginArray(const char* key) override {
        Json& array = (*stack_.back())[key] = Json::array();
        stack_.push_back(&array);
    }
    void endArray() override { stack_.pop_back(); }

private:
    std::vector<Json*> stack_;
};

} // namespace

Json toJson(const NcProgram& program) {
    Json instrs = Json::array();
    for (const NcInstr& i : program.instrs) {
        Json j = Json::object();
        JsonSink sink(j);
        detail::instructionFields(i, sink);
        instrs.push_back(std::move(j));
    }
    Json doc = detail::programHeaderJson(program);
    doc["instructions"] = std::move(instrs);
    return doc;
}

NcProgram ncProgramFromJson(const Json& doc) {
    try {
        // The name programs written by OxCAM, formerly OpenCAM, used (ADR-0005).
        constexpr std::string_view kLegacyFormat = "opencam-ncprogram";
        const std::string format =
            doc.is_object() ? doc.value("format", std::string()) : std::string();
        if (format != "oxcam-ncprogram" && format != kLegacyFormat) {
            throw parseError("not an oxcam-ncprogram document");
        }
        const int version = doc.value("version", 0);
        if (version < 1 || version > kNcProgramJsonVersion) {
            throw parseError("unsupported version");
        }
        NcProgram program;
        program.name = doc.value("name", std::string());
        program.number = doc.value("number", 1000);
        program.project = doc.value("project", std::string());
        program.setup = doc.value("setup", std::string());
        program.time = doc.value("time", std::string());
        for (const Json& t : doc.value("tools", Json::array())) {
            program.tools.push_back(toolFromJson(t));
        }
        for (const Json& o : doc.value("operations", Json::array())) {
            program.operations.push_back(operationFromJson(o));
        }
        if (const auto it = doc.find("machine"); it != doc.end() && it->is_object()) {
            program.machine = machineFromJson(*it);
        }
        if (doc.contains("warnings")) {
            program.warnings = doc.at("warnings").get<std::vector<std::string>>();
        }
        for (const Json& j : doc.at("instructions")) {
            NcInstr i;
            const auto op = parseNcOp(j.at("op").get<std::string>());
            if (!op) {
                throw parseError("unknown instruction '" + j.at("op").get<std::string>() + "'");
            }
            i.op = *op;
            if (i.op == NcOp::ToolLength || i.op == NcOp::Rapid || i.op == NcOp::Linear ||
                i.op == NcOp::Arc || i.op == NcOp::Cycle) {
                i.target =
                    Vec3(readCoordinate(j, "x"), readCoordinate(j, "y"), readCoordinate(j, "z"));
                if (i.op != NcOp::Cycle && i.op != NcOp::Arc && j.contains("axes")) {
                    i.axes = j.at("axes").get<std::string>();
                    i.rotary = Vec3(j.value("a", 0.0), j.value("b", 0.0), j.value("c", 0.0)) *
                               (kPi / 180.0);
                }
            }
            switch (i.op) {
            case NcOp::Comment:
                i.text = j.value("text", std::string());
                break;
            case NcOp::ToolChange:
                i.tool = j.at("tool").get<int>();
                i.toolName = j.value("tool_name", std::string());
                break;
            case NcOp::WorkOffset:
                i.workOffset = j.at("work_offset").get<int>();
                break;
            case NcOp::Spindle:
                i.spindle = j.at("rpm").get<double>() / 60.0;
                i.reverse = j.value("reverse", false);
                i.surfaceSpeed = j.value("surface_speed", 0.0) / 0.06;
                i.maxSpindle = j.value("max_rpm", 0.0) / 60.0;
                i.sub = j.value("spindle", std::string("main")) == "sub";
                break;
            case NcOp::LiveTool:
                i.spindle = j.at("rpm").get<double>() / 60.0;
                i.tapping = j.value("tapping", false);
                if (j.contains("plane")) {
                    i.milling = true;
                    i.plane = j.at("plane").get<std::string>() == "XY" ? toolpath::ArcPlane::XY
                                                                       : toolpath::ArcPlane::YZ;
                }
                break;
            case NcOp::Wrap:
                i.polarOn = j.at("on").get<bool>();
                i.wrapRadius = j.value("radius", 0.0);
                break;
            case NcOp::Polar:
                i.polarOn = j.at("on").get<bool>();
                break;
            case NcOp::Chuck:
                i.open = j.at("open").get<bool>();
                i.sub = j.value("spindle", std::string("main")) == "sub";
                break;
            case NcOp::SubAxis:
                i.target.z() = j.at("z2").get<double>();
                i.feed = j.contains("feed") && j.at("feed").is_number()
                             ? j.at("feed").get<double>() / 60.0
                             : 0.0;
                break;
            case NcOp::SpindleSync:
                i.polarOn = j.at("on").get<bool>();
                if (j.contains("angle") && j.at("angle").is_number()) {
                    i.syncAngle = j.at("angle").get<double>() * kPi / 180.0;
                }
                break;
            case NcOp::BarFeeder:
                i.polarOn = j.at("on").get<bool>();
                break;
            case NcOp::LiveTap:
                i.spindle = j.at("rpm").get<double>() / 60.0;
                i.pitch = j.at("pitch").get<double>();
                i.reverse = j.value("left_hand", false);
                i.rPlane = j.at("r_plane").get<double>();
                i.radial = j.at("radial").get<bool>();
                break;
            case NcOp::Coolant: {
                const auto c = tooling::parseCoolant(j.at("coolant").get<std::string>());
                if (!c) {
                    throw parseError("unknown coolant");
                }
                i.coolant = *c;
                break;
            }
            case NcOp::ToolLength:
                i.tool = j.at("tool").get<int>();
                i.tcp = j.value("tcp", false);
                break;
            case NcOp::Linear:
                i.feed = j.at("feed").get<double>() / 60.0;
                i.feedClass = readFeedClass(j);
                i.pitch = j.value("pitch", 0.0);
                i.inverseTime = j.value("inverse_time", 0.0);
                break;
            case NcOp::OperationBegin:
            case NcOp::OperationEnd:
                i.operation = j.at("operation").get<int>();
                if (i.operation < 0 ||
                    static_cast<std::size_t>(i.operation) >= program.operations.size()) {
                    throw parseError("an operation instruction refers to no operation");
                }
                break;
            case NcOp::Arc:
                i.center =
                    Vec3(j.at("cx").get<double>(), j.at("cy").get<double>(), j.value("cz", 0.0));
                i.clockwise = j.at("clockwise").get<bool>();
                {
                    const std::string plane = j.value("plane", "XY");
                    i.plane = plane == "ZX"   ? toolpath::ArcPlane::ZX
                              : plane == "YZ" ? toolpath::ArcPlane::YZ
                                              : toolpath::ArcPlane::XY;
                }
                i.feed = j.at("feed").get<double>() / 60.0;
                i.feedClass = readFeedClass(j);
                break;
            case NcOp::Dwell:
                i.dwell = j.at("seconds").get<double>();
                break;
            case NcOp::SpindleOrient:
                i.angle = j.at("angle").get<double>() / kDeg;
                break;
            case NcOp::Cycle: {
                const Json& c = j.at("cycle");
                const auto type = toolpath::parseCycleType(c.at("type").get<std::string>());
                if (!type) {
                    throw parseError("unknown cycle type");
                }
                i.cycle.type = *type;
                i.cycle.topZ = c.at("top").get<double>();
                i.cycle.bottomZ = c.at("bottom").get<double>();
                i.cycle.retractZ = c.at("retract").get<double>();
                i.cycle.peck = c.value("peck", 0.0);
                i.cycle.chipBreakRetract = c.value("chip_break_retract", i.cycle.chipBreakRetract);
                i.cycle.dwell = c.value("dwell", 0.0);
                i.cycle.feedRate = c.at("feed").get<double>() / 60.0;
                i.cycle.pitch = c.value("pitch", 0.0);
                i.cycle.shift = Vec2(c.value("shift_x", 0.0), c.value("shift_y", 0.0));
                i.cycle.orient = c.value("orient", 0.0) / kDeg;
                break;
            }
            case NcOp::CompensationOn:
                i.side = j.at("side").get<std::string>() == "left" ? toolpath::Compensation::Left
                                                                   : toolpath::Compensation::Right;
                i.tool = j.value("tool", 0);
                break;
            case NcOp::Index:
                i.text = j.value("axes", std::string("AC"));
                i.rotary =
                    Vec3(j.value("a", 0.0), j.value("b", 0.0), j.value("c", 0.0)) * (kPi / 180.0);
                break;
            case NcOp::Probe: {
                auto point = [](const Json& p) {
                    return Vec3(p.at("x").get<double>(), p.at("y").get<double>(),
                                p.at("z").get<double>());
                };
                i.operation = j.value("operation", -1);
                toolpath::ProbeCycle& c = i.probe;
                const auto feature =
                    toolpath::parseProbeFeature(j.at("feature").get<std::string>());
                if (!feature) {
                    throw parseError("unknown probing feature");
                }
                c.feature = *feature;
                c.workOffset = j.value("work_offset", 0);
                c.axes = j.value("axes", std::string());
                c.nominal = point(j.at("nominal"));
                c.direction = point(j.at("direction"));
                c.size = j.value("size", 0.0);
                c.tolerance = j.value("tolerance", 0.0);
                c.approach = j.value("approach", 0.0);
                c.overtravel = j.value("overtravel", 0.0);
                c.fastFeed = j.at("fast_feed").get<double>() / 60.0;
                c.measureFeed = j.at("measure_feed").get<double>() / 60.0;
                c.stylusDiameter = j.value("stylus_diameter", 0.0);
                c.triggerOffset = j.value("trigger_offset", 0.0);
                for (const Json& m : j.at("moves")) {
                    NcProbeMove move;
                    move.measure = m.at("kind").get<std::string>() == "measure";
                    move.target = point(m);
                    move.feed = m.at("feed").get<double>() / 60.0;
                    if (move.measure) {
                        move.expected = point(m.at("expected"));
                    }
                    i.probeMoves.push_back(move);
                }
                break;
            }
            case NcOp::Rapid:
            case NcOp::CycleCancel:
            case NcOp::CompensationOff:
            case NcOp::HomeZ:
            case NcOp::TcpOff:
                break;
            }
            program.instrs.push_back(std::move(i));
        }
        return program;
    } catch (const Json::exception& e) {
        throw parseError(e.what());
    }
}

} // namespace ocam::post
