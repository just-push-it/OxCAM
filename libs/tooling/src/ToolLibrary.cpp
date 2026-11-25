// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/tooling/ToolLibrary.hpp"

#include "ocam/tooling/TurningHolder.hpp"

#include <ocam/foundation/Error.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cctype>
#include <set>

namespace ocam::embedded {
extern const std::string_view toolLibrarySchemaV1;
}

namespace ocam::tooling {
namespace {

// Surface speed: internal mm/s <-> m/min (mm files) or ft/min (inch files).
constexpr double kMmPerFoot = 304.8;
double speedToFile(double mmPerSec, LengthUnit u) {
    return mmPerSec * kSecondsPerMinute / (u == LengthUnit::Inch ? kMmPerFoot : 1000.0);
}
double speedFromFile(double v, LengthUnit u) {
    return v * (u == LengthUnit::Inch ? kMmPerFoot : 1000.0) / kSecondsPerMinute;
}

Uuid parseUuid(const Json& j) {
    const auto id = Uuid::parse(j.get<std::string>());
    if (!id) {
        throw Error(ErrorCode::Parse, "invalid UUID: " + j.get<std::string>());
    }
    return *id;
}

struct Writer {
    LengthUnit unit;
    double len(double mm) const { return fromMm(mm, unit); }

    Json preset(const CuttingPreset& p) const {
        return {{"id", p.id.toString()},
                {"name", p.name},
                {"material_group", p.materialGroup},
                {"surface_speed", speedToFile(p.surfaceSpeed, unit)},
                {"feed_per_tooth", len(p.feedPerTooth)},
                {"axial_depth", len(p.axialDepth)},
                {"radial_depth", len(p.radialDepth)},
                {"ramp_angle_deg", radToDeg(p.rampAngle)},
                {"plunge_feed", mmPerSecToFeedPerMin(p.plungeFeed, unit)},
                {"ramp_feed", mmPerSecToFeedPerMin(p.rampFeed, unit)},
                {"lead_feed", mmPerSecToFeedPerMin(p.leadFeed, unit)},
                {"part_off_end_feed", len(p.partOffEndFeed)},
                {"lead_feed_per_rev", len(p.leadFeedPerRev)},
                {"pull_feed", mmPerSecToFeedPerMin(p.pullFeed, unit)},
                {"coolant", std::string(toString(p.coolant))}};
    }

    Json tool(const Tool& t) const {
        Json j{{"id", t.id.toString()},
               {"number", t.number},
               {"name", t.name},
               {"comment", t.comment},
               {"type", std::string(toString(t.type))},
               {"material", t.material},
               {"coating", t.coating},
               {"vendor", t.vendor},
               {"part_number", t.partNumber},
               {"stickout", len(t.stickout)}};
        if (t.holderId) {
            j["holder"] = t.holderId->toString();
        }
        if (t.unit == LengthUnit::Inch) {
            j["units"] = "in";
        }
        if (isTurningTool(t.type)) {
            const TurningInsert& in = t.insert;
            j["insert"] = {{"shape", in.shape},
                           {"inscribed_circle", len(in.inscribedCircle)},
                           {"nose_radius", len(in.noseRadius)},
                           {"thickness", len(in.thickness)},
                           {"relief_angle_deg", radToDeg(in.reliefAngle)},
                           {"hand", in.hand},
                           {"orientation", in.orientation},
                           {"width", len(in.width)},
                           {"thread_pitch", len(in.threadPitch)}};
            if (in.cuttingEdgeAngle != 0.0) {
                j["insert"]["cutting_edge_angle_deg"] = radToDeg(in.cuttingEdgeAngle);
            }
            if (in.locking != InsertLocking::NotSet) {
                j["insert"]["locking"] = std::string(toString(in.locking));
            }
            if (t.solid) {
                j["solid"] = {{"min_bore", len(t.solid->minBore)},
                              {"shank_diameter", len(t.solid->shankDiameter)},
                              {"head_length", len(t.solid->headLength)},
                              {"overall_length", len(t.solid->overallLength)},
                              {"max_overhang", len(t.solid->maxOverhang)},
                              {"centre_to_tip", len(t.solid->centreToTip)},
                              {"max_depth", len(t.solid->maxDepth)}};
            }
        } else {
            const MillGeometry& g = t.mill;
            Json geometry{{"diameter", len(g.diameter)},
                          {"corner_radius", len(g.cornerRadius)},
                          {"flute_length", len(g.fluteLength)},
                          {"overall_length", len(g.overallLength)},
                          {"shank_diameter", len(g.shankDiameter)},
                          {"flutes", g.flutes},
                          {"tip_angle_deg", radToDeg(g.tipAngle)},
                          {"tip_diameter", len(g.tipDiameter)},
                          {"tip_length", len(g.tipLength)},
                          {"thread_pitch", len(g.threadPitch)},
                          {"neck_diameter", len(g.neckDiameter)},
                          {"neck_length", len(g.neckLength)}};
            if (!g.formProfile.empty()) {
                Json pts = Json::array();
                for (const Vec2& p : g.formProfile) {
                    pts.push_back({len(p.x()), len(p.y())});
                }
                geometry["form_profile"] = std::move(pts);
            }
            if (g.leftHand) {
                geometry["left_hand"] = true;
            }
            if (t.type == ToolType::Probe) {
                geometry["trigger_offset"] = len(g.triggerOffset);
            }
            j["geometry"] = std::move(geometry);
        }
        Json presets = Json::array();
        for (const auto& p : t.presets) {
            presets.push_back(preset(p));
        }
        j["presets"] = std::move(presets);
        return j;
    }

    Json holder(const Holder& h) const {
        Json pts = Json::array();
        for (const auto& v : h.profile.vertices()) {
            pts.push_back({len(v.p.x()), len(v.p.y()), v.bulge});
        }
        Json j{{"id", h.id.toString()}, {"name", h.name}, {"comment", h.comment}, {"profile", pts}};
        if (h.kind != HolderKind::Milling) {
            j["kind"] = std::string(toString(h.kind));
        }
        if (!h.vendor.empty()) {
            j["vendor"] = h.vendor;
        }
        if (!h.partNumber.empty()) {
            j["part_number"] = h.partNumber;
        }
        if (h.stick) {
            j["stick"] = {{"bar", h.stick->bar},
                          {"width", len(h.stick->width)},
                          {"f", len(h.stick->f)},
                          {"length", len(h.stick->length)}};
        }
        if (h.iso) {
            const TurningHolderCode& c = *h.iso;
            auto letter = [](char l) {
                return std::string(1, l);
            };
            j["iso"] = {{"boring_bar", c.boringBar},      {"bar_type", letter(c.barType)},
                        {"clamping", letter(c.clamping)}, {"insert_shape", letter(c.insertShape)},
                        {"style", letter(c.style)},       {"clearance", letter(c.clearance)},
                        {"hand", letter(c.hand)},         {"height", len(c.height)},
                        {"width", len(c.width)},          {"length", letter(c.length)},
                        {"insert_size", c.insertSize},    {"f", len(c.f)}};
            if (c.customStyle) {
                j["iso"]["custom_style_deg"] = *c.customStyle;
            }
            if (c.customClearance) {
                j["iso"]["custom_clearance_deg"] = *c.customClearance;
            }
            if (c.customLength) {
                j["iso"]["custom_length"] = len(*c.customLength);
            }
        }
        return j;
    }
};

struct Reader {
    LengthUnit unit;
    double len(const Json& obj, const char* key) const { return toMm(obj.value(key, 0.0), unit); }

    CuttingPreset preset(const Json& j) const {
        CuttingPreset p;
        p.id = parseUuid(j.at("id"));
        p.name = j.value("name", "");
        p.materialGroup = j.value("material_group", "");
        p.surfaceSpeed = speedFromFile(j.value("surface_speed", 0.0), unit);
        p.feedPerTooth = len(j, "feed_per_tooth");
        p.axialDepth = len(j, "axial_depth");
        p.radialDepth = len(j, "radial_depth");
        p.rampAngle = degToRad(j.value("ramp_angle_deg", radToDeg(p.rampAngle)));
        p.plungeFeed = feedPerMinToMmPerSec(j.value("plunge_feed", 0.0), unit);
        p.rampFeed = feedPerMinToMmPerSec(j.value("ramp_feed", 0.0), unit);
        p.leadFeed = feedPerMinToMmPerSec(j.value("lead_feed", 0.0), unit);
        p.partOffEndFeed = len(j, "part_off_end_feed");
        p.leadFeedPerRev = len(j, "lead_feed_per_rev");
        p.pullFeed = feedPerMinToMmPerSec(j.value("pull_feed", 0.0), unit);
        p.coolant = parseCoolant(j.value("coolant", "flood")).value_or(Coolant::Flood);
        return p;
    }

    Tool tool(const Json& j) const {
        Tool t;
        t.id = parseUuid(j.at("id"));
        t.number = j.at("number").get<int>();
        t.name = j.value("name", "");
        t.comment = j.value("comment", "");
        t.type = parseToolType(j.at("type").get<std::string>()).value();
        t.material = j.value("material", "");
        t.coating = j.value("coating", "");
        t.vendor = j.value("vendor", "");
        t.partNumber = j.value("part_number", "");
        t.stickout = len(j, "stickout");
        t.unit = parseLengthUnit(j.value("units", "mm")).value_or(LengthUnit::Millimeter);
        if (j.contains("holder")) {
            t.holderId = parseUuid(j.at("holder"));
        }
        if (const auto it = j.find("geometry"); it != j.end()) {
            const Json& g = *it;
            MillGeometry& m = t.mill;
            m.diameter = len(g, "diameter");
            m.cornerRadius = len(g, "corner_radius");
            m.fluteLength = len(g, "flute_length");
            m.overallLength = len(g, "overall_length");
            m.shankDiameter = len(g, "shank_diameter");
            m.flutes = g.value("flutes", 2);
            m.tipAngle = degToRad(g.value("tip_angle_deg", 0.0));
            m.tipDiameter = len(g, "tip_diameter");
            m.tipLength = len(g, "tip_length");
            m.threadPitch = len(g, "thread_pitch");
            m.leftHand = g.value("left_hand", false);
            m.neckDiameter = len(g, "neck_diameter");
            m.neckLength = len(g, "neck_length");
            m.triggerOffset = len(g, "trigger_offset");
            if (const auto fp = g.find("form_profile"); fp != g.end()) {
                for (const Json& p : *fp) {
                    m.formProfile.emplace_back(toMm(p.at(0).get<double>(), unit),
                                               toMm(p.at(1).get<double>(), unit));
                }
            }
        }
        if (const auto it = j.find("insert"); it != j.end()) {
            const Json& in = *it;
            TurningInsert& ti = t.insert;
            ti.shape = in.value("shape", ti.shape);
            ti.inscribedCircle = len(in, "inscribed_circle");
            ti.noseRadius = len(in, "nose_radius");
            ti.thickness = len(in, "thickness");
            ti.reliefAngle = degToRad(in.value("relief_angle_deg", 0.0));
            ti.hand = in.value("hand", ti.hand);
            ti.orientation = in.value("orientation", ti.orientation);
            ti.width = len(in, "width");
            ti.threadPitch = len(in, "thread_pitch");
            ti.cuttingEdgeAngle = degToRad(in.value("cutting_edge_angle_deg", 0.0));
            ti.locking = parseInsertLocking(in.value("locking", "not_set"))
                             .value_or(InsertLocking::NotSet);
        }
        if (const auto it = j.find("solid"); it != j.end()) {
            const Json& s = *it;
            t.solid = SolidBar{len(s, "min_bore"),     len(s, "shank_diameter"),
                               len(s, "head_length"),  len(s, "overall_length"),
                               len(s, "max_overhang"), len(s, "centre_to_tip"),
                               len(s, "max_depth")};
        }
        for (const Json& pj : j.value("presets", Json::array())) {
            CuttingPreset p = preset(pj);
            if (!pj.contains("plunge_feed") && pj.contains("plunge_feed_factor")) {
                // Older files gave the plunge feed as a share of the cutting feed.
                p.plungeFeed = pj.at("plunge_feed_factor").get<double>() *
                               p.feedRate(t.mill.diameter, t.mill.flutes);
            }
            if (!pj.contains("ramp_feed") && !isTurningTool(t.type)) {
                // Files written before ramps had their own feed: they ramped at the cutting feed.
                p.rampFeed = p.feedRate(t.mill.diameter, t.mill.flutes);
            }
            if (!pj.contains("lead_feed") && !isTurningTool(t.type)) {
                // Files written before leads had their own feed: they led at the cutting feed.
                p.leadFeed = p.feedRate(t.mill.diameter, t.mill.flutes);
            }
            if (!pj.contains("part_off_end_feed") &&
                (t.type == ToolType::PartingInsert || t.type == ToolType::GroovingInsert)) {
                // Files written before the end of a part-off had its own feed: it ran at half.
                p.partOffEndFeed = 0.5 * p.feedPerTooth;
            }
            if (!pj.contains("lead_feed_per_rev") && t.type == ToolType::TurningInsert) {
                // Files written before turning's leads had their own feed: they ran at the feed.
                p.leadFeedPerRev = p.feedPerTooth;
            }
            t.presets.push_back(p);
        }
        return t;
    }

    Holder holder(const Json& j) const {
        Holder h;
        h.id = parseUuid(j.at("id"));
        h.kind = parseHolderKind(j.value("kind", "milling")).value();
        h.name = j.value("name", "");
        h.comment = j.value("comment", "");
        h.vendor = j.value("vendor", "");
        h.partNumber = j.value("part_number", "");
        std::vector<geom2d::Vertex> v;
        for (const Json& p : j.at("profile")) {
            v.push_back({{toMm(p.at(0).get<double>(), unit), toMm(p.at(1).get<double>(), unit)},
                         p.size() > 2 ? p.at(2).get<double>() : 0.0});
        }
        h.profile = geom2d::Polyline(std::move(v), false);
        if (const auto it = j.find("stick"); it != j.end()) {
            h.stick = TurningStick{it->value("bar", false), len(*it, "width"), len(*it, "f"),
                                   len(*it, "length")};
        }
        if (const auto it = j.find("iso"); it != j.end()) {
            const Json& iso = *it;
            TurningHolderCode c;
            auto letter = [&](const char* key, char fallback) {
                const std::string text = iso.value(key, std::string());
                return text.empty() ? fallback : text.front();
            };
            c.boringBar = iso.value("boring_bar", false);
            c.barType = letter("bar_type", c.barType);
            c.clamping = letter("clamping", c.clamping);
            c.insertShape = letter("insert_shape", c.insertShape);
            c.style = letter("style", c.style);
            c.clearance = letter("clearance", c.clearance);
            c.hand = letter("hand", c.hand);
            c.height = len(iso, "height");
            c.width = len(iso, "width");
            c.length = letter("length", c.length);
            c.insertSize = iso.value("insert_size", c.insertSize);
            c.f = len(iso, "f");
            if (iso.contains("custom_style_deg")) {
                c.customStyle = iso.at("custom_style_deg").get<double>();
            }
            if (iso.contains("custom_clearance_deg")) {
                c.customClearance = iso.at("custom_clearance_deg").get<double>();
            }
            if (iso.contains("custom_length")) {
                c.customLength = len(iso, "custom_length");
            }
            h.iso = c;
        }
        return h;
    }
};

} // namespace

const Tool* ToolLibrary::findTool(const Uuid& toolId) const {
    auto it =
        std::find_if(tools.begin(), tools.end(), [&](const Tool& t) { return t.id == toolId; });
    return it == tools.end() ? nullptr : &*it;
}

Tool* ToolLibrary::findTool(const Uuid& toolId) {
    auto it =
        std::find_if(tools.begin(), tools.end(), [&](const Tool& t) { return t.id == toolId; });
    return it == tools.end() ? nullptr : &*it;
}

const Tool* ToolLibrary::findToolByNumber(int number) const {
    auto it =
        std::find_if(tools.begin(), tools.end(), [&](const Tool& t) { return t.number == number; });
    return it == tools.end() ? nullptr : &*it;
}

const Holder* ToolLibrary::findHolder(const Uuid& holderId) const {
    auto it = std::find_if(holders.begin(), holders.end(),
                           [&](const Holder& h) { return h.id == holderId; });
    return it == holders.end() ? nullptr : &*it;
}

int ToolLibrary::nextFreeNumber() const {
    std::set<int> used;
    for (const auto& t : tools) {
        used.insert(t.number);
    }
    int n = 1;
    while (used.contains(n)) {
        ++n;
    }
    return n;
}

std::vector<std::string> ToolLibrary::validate() const {
    std::vector<std::string> issues;
    std::set<Uuid> ids;
    std::set<int> numbers;
    for (const auto& t : tools) {
        const std::string label = fmt::format("T{} ({})", t.number, t.name);
        for (const auto& issue : t.validate()) {
            issues.push_back(label + ": " + issue);
        }
        if (!ids.insert(t.id).second) {
            issues.push_back(label + ": duplicate id " + t.id.toString());
        }
        if (!numbers.insert(t.number).second) {
            issues.push_back(fmt::format("tool number {} is used more than once", t.number));
        }
        if (t.holderId && !findHolder(*t.holderId)) {
            issues.push_back(label + ": references a missing holder");
        }
        if (const Holder* holder = t.holderId ? findHolder(*t.holderId) : nullptr;
            holder && holder->iso && t.type == ToolType::TurningInsert) {
            Tool fitted = t;
            fitToHolder(fitted, *holder);
            if (fitted.insert != t.insert) {
                issues.push_back(label + ": its insert is not the one holder " + holder->name +
                                 " takes");
            }
        }
        std::set<Uuid> presetIds;
        for (const auto& p : t.presets) {
            if (!presetIds.insert(p.id).second) {
                issues.push_back(label + ": duplicate preset id " + p.id.toString());
            }
        }
    }
    for (const auto& h : holders) {
        for (const auto& issue : h.validate()) {
            issues.push_back("holder '" + h.name + "': " + issue);
        }
        if (!ids.insert(h.id).second) {
            issues.push_back("holder '" + h.name + "': duplicate id " + h.id.toString());
        }
    }
    return issues;
}

bool ToolFilter::empty() const noexcept {
    return text.empty() && !family && !type && minDiameter <= 0.0 && maxDiameter <= 0.0;
}

bool ToolFilter::matches(const Tool& tool) const {
    if (family && toolFamily(tool.type) != *family) {
        return false;
    }
    if (type && tool.type != *type) {
        return false;
    }
    if (minDiameter > 0.0 || maxDiameter > 0.0) {
        if (isTurningTool(tool.type)) {
            return false;
        }
        const double d = tool.mill.diameter;
        if ((minDiameter > 0.0 && d < minDiameter - 1e-9) ||
            (maxDiameter > 0.0 && d > maxDiameter + 1e-9)) {
            return false;
        }
    }
    auto lower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return s;
    };
    std::string needle = lower(text);
    needle.erase(0, needle.find_first_not_of(' '));
    needle.erase(needle.find_last_not_of(' ') + 1);
    if (needle.empty()) {
        return true;
    }
    for (const std::string* field : {&tool.name, &tool.vendor, &tool.partNumber}) {
        if (lower(*field).find(needle) != std::string::npos) {
            return true;
        }
    }
    const std::string number = std::to_string(tool.number);
    return needle == number || needle == "t" + number;
}

const FormatSpec& toolLibraryFormat() {
    static const FormatSpec spec{"oxcam.toollib", kToolLibraryVersion, {}, {"opencam.toollib"}};
    return spec;
}

const SchemaValidator& toolLibrarySchema() {
    static const SchemaValidator validator(
        parseJson(embedded::toolLibrarySchemaV1, "toollib.v1.schema.json"));
    return validator;
}

Json toJson(const ToolLibrary& library, LengthUnit fileUnit) {
    const Writer w{fileUnit};
    Json tools = Json::array();
    for (const auto& t : library.tools) {
        tools.push_back(w.tool(t));
    }
    Json holders = Json::array();
    for (const auto& h : library.holders) {
        holders.push_back(w.holder(h));
    }
    return {{"format", toolLibraryFormat().format},
            {"version", kToolLibraryVersion},
            {"units", std::string(toString(fileUnit))},
            {"id", library.id.toString()},
            {"name", library.name},
            {"tools", std::move(tools)},
            {"holders", std::move(holders)}};
}

ToolLibrary toolLibraryFromJson(Json doc, std::string_view sourceName) {
    toolLibraryFormat().upgrade(doc, sourceName);
    toolLibrarySchema().validate(doc, sourceName);
    try {
        const Reader r{parseLengthUnit(doc.at("units").get<std::string>()).value()};
        ToolLibrary lib;
        if (doc.contains("id")) {
            lib.id = parseUuid(doc.at("id"));
        }
        lib.name = doc.value("name", "");
        for (const Json& t : doc.at("tools")) {
            lib.tools.push_back(r.tool(t));
        }
        for (const Json& h : doc.at("holders")) {
            lib.holders.push_back(r.holder(h));
            if (!h.contains("kind")) {
                // Files written before holders had a kind: a holder that only turning tools use
                // is a turning holder; any other is a milling holder.
                Holder& holder = lib.holders.back();
                const auto users = [&](auto pred) {
                    return std::any_of(lib.tools.begin(), lib.tools.end(), [&](const Tool& t) {
                        return t.holderId == holder.id && pred(t);
                    });
                };
                if (users([](const Tool& t) { return isTurningTool(t.type); }) &&
                    !users([](const Tool& t) { return !isTurningTool(t.type); })) {
                    holder.kind = HolderKind::Turning;
                }
            }
        }
        return lib;
    } catch (const Json::exception& e) {
        throw Error(ErrorCode::Parse, fmt::format("{}: {}", sourceName, e.what()));
    } catch (const std::bad_optional_access&) {
        throw Error(ErrorCode::Parse, fmt::format("{}: unknown enumeration value", sourceName));
    }
}

ToolLibrary loadToolLibrary(const std::filesystem::path& path) {
    const std::u8string u8 = path.filename().u8string();
    const std::string name(u8.begin(), u8.end());
    return toolLibraryFromJson(parseJson(readFileBytes(path), name), name);
}

void saveToolLibrary(const ToolLibrary& library, const std::filesystem::path& path,
                     LengthUnit fileUnit) {
    writeFileAtomic(path, dumpJson(toJson(library, fileUnit)));
}

} // namespace ocam::tooling
