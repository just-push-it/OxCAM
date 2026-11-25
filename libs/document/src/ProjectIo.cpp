// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/document/ProjectIo.hpp"

#include "ocam/document/LatheFitting.hpp"
#include "ocam/document/Workholding.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Sha256.hpp>

#include <fmt/format.h>
#include <miniz.h>

#include <algorithm>
#include <map>
#include <memory>
#include <regex>

namespace ocam::embedded {
extern const std::string_view projectSchemaV1;
}

namespace ocam::document {
namespace {

// Refuse archive entries above this size (guards against decompression bombs).
constexpr mz_uint64 kMaxEntryBytes = 4ULL * 1024 * 1024 * 1024;

Json vec(const Vec3& v) {
    return Json::array({v.x(), v.y(), v.z()});
}

Vec3 vecFrom(const Json& j) {
    return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()};
}

Json frameJson(const Frame& f) {
    return {{"origin", vec(f.origin)}, {"x_axis", vec(f.xAxis)}, {"z_axis", vec(f.zAxis)}};
}

Frame frameFrom(const Json& j) {
    return {vecFrom(j.at("origin")), vecFrom(j.at("x_axis")), vecFrom(j.at("z_axis"))};
}

Uuid uuidFrom(const Json& j) {
    const auto id = Uuid::parse(j.get<std::string>());
    if (!id) {
        throw Error(ErrorCode::Parse, "invalid UUID: " + j.get<std::string>());
    }
    return *id;
}

Json stockJson(const Stock& s) {
    Json j{{"shape", std::string(toString(s.kind))}};
    if (s.kind == StockKind::FromSetup) {
        if (s.sourceSetup) {
            j["source_setup"] = s.sourceSetup->toString();
        }
        return j;
    }
    j["size"] = std::string(toString(s.size));
    j["box_size"] = vec(s.boxSize);
    j["axis"] = std::string(1, static_cast<char>('x' + std::clamp(s.axis, 0, 2)));
    j["diameter"] = s.diameter;
    j["length"] = s.length;
    j["inner_diameter"] = s.innerDiameter;
    j["offset_side"] = s.offsetSide;
    j["offset_top"] = s.offsetTop;
    j["offset_bottom"] = s.offsetBottom;
    j["round_to"] = s.roundTo;
    j["shift"] = vec(s.shift);
    if (s.boxCorners) {
        j["box_min"] = vec((*s.boxCorners)[0]);
        j["box_max"] = vec((*s.boxCorners)[1]);
    }
    if (s.cylinderPlacement) {
        j["cylinder_base"] = vec((*s.cylinderPlacement)[0]);
        j["cylinder_axis"] = vec((*s.cylinderPlacement)[1]);
    }
    return j;
}

Stock stockFrom(const Json& j) {
    Stock s;
    auto v3 = [&](const char* key, const Vec3& def) {
        return j.contains(key) ? vecFrom(j.at(key)) : def;
    };
    s.offsetSide = j.value("offset_side", s.offsetSide);
    s.offsetTop = j.value("offset_top", s.offsetTop);
    s.offsetBottom = j.value("offset_bottom", s.offsetBottom);
    s.roundTo = j.value("round_to", s.roundTo);
    if (j.contains("source_setup")) {
        s.sourceSetup = uuidFrom(j.at("source_setup"));
    }
    if (j.contains("shape")) {
        s.kind = parseStockKind(j.at("shape").get<std::string>()).value();
        if (j.contains("size")) {
            s.size = parseStockSize(j.at("size").get<std::string>()).value();
        }
        s.axis = j.value("axis", std::string("z")).at(0) - 'x';
        s.boxSize = v3("box_size", s.boxSize);
        s.diameter = j.value("diameter", s.diameter);
        s.length = j.value("length", s.length);
        s.innerDiameter = j.value("inner_diameter", s.innerDiameter);
        s.shift = v3("shift", s.shift);
        if (j.contains("box_min") && j.contains("box_max")) {
            s.boxCorners = {vecFrom(j.at("box_min")), vecFrom(j.at("box_max"))};
        }
        if (j.contains("cylinder_base") && j.contains("cylinder_axis")) {
            s.cylinderPlacement = {vecFrom(j.at("cylinder_base")), vecFrom(j.at("cylinder_axis"))};
        }
        return s;
    }
    // Files of earlier versions: one 'kind' for the shape and the size (the same stock).
    const std::string kind = j.at("kind").get<std::string>();
    if (kind == "offset_from_model") {
        s.kind = StockKind::Box;
        s.size = StockSize::OffsetFromPart;
    } else if (kind == "box") {
        s.kind = StockKind::Box;
        s.size = StockSize::Fixed;
        s.boxSize = v3("box_size", s.boxSize);
        s.shift = v3("box_shift", s.shift);
        if (!(s.boxSize.array() > 0.0).any()) {
            s.boxCorners = {v3("box_min", Vec3::Zero()), v3("box_max", Vec3::Zero())};
        }
    } else if (kind == "cylinder") {
        s.kind = StockKind::Cylinder;
        s.size = StockSize::Fixed;
        s.diameter = j.value("cylinder_diameter", s.diameter);
        s.length = j.value("cylinder_length", s.length);
        s.cylinderPlacement = {v3("cylinder_base", Vec3::Zero()),
                               v3("cylinder_axis", Vec3::UnitZ())};
    } else if (kind == "from_setup") {
        s.kind = StockKind::FromSetup;
    } else {
        throw Error(ErrorCode::Parse, "unknown stock kind: " + kind);
    }
    return s;
}

Json shapeRefJson(const ShapeRef& r) {
    Json j{{"model", r.modelId.toString()},
           {"type", r.type == cad::SubShapeType::Face   ? "face"
                    : r.type == cad::SubShapeType::Edge ? "edge"
                                                        : "vertex"},
           {"index", r.index},
           {"signature",
            {{"centroid", vec(r.signature.centroid)},
             {"direction", vec(r.signature.direction)},
             {"measure", r.signature.measure}}}};
    if (r.side != GeometrySide::Default) {
        j["side"] = r.side == GeometrySide::Outside ? "outside" : "inside";
    }
    return j;
}

ShapeRef shapeRefFrom(const Json& j) {
    ShapeRef r;
    r.modelId = uuidFrom(j.at("model"));
    const std::string type = j.at("type").get<std::string>();
    r.type = type == "edge"     ? cad::SubShapeType::Edge
             : type == "vertex" ? cad::SubShapeType::Vertex
                                : cad::SubShapeType::Face;
    r.index = j.at("index").get<int>();
    const Json& sig = j.at("signature");
    r.signature.centroid = vecFrom(sig.at("centroid"));
    r.signature.direction = vecFrom(sig.at("direction"));
    r.signature.measure = sig.at("measure").get<double>();
    const std::string side = j.value("side", "");
    r.side = side == "outside"  ? GeometrySide::Outside
             : side == "inside" ? GeometrySide::Inside
                                : GeometrySide::Default;
    return r;
}

Json operationJson(const Operation& op) {
    Json geometry = Json::array();
    for (const auto& r : op.geometry) {
        geometry.push_back(shapeRefJson(r));
    }
    Json j{{"id", op.id.toString()},
           {"name", op.name},
           {"type", op.type},
           {"enabled", op.enabled},
           {"geometry", std::move(geometry)},
           {"parameters", op.parameters},
           {"comment", op.comment}};
    if (op.toolId) {
        j["tool"] = op.toolId->toString();
    }
    if (op.presetId) {
        j["preset"] = op.presetId->toString();
    }
    if (!op.heights.isDefault()) {
        Json heights = Json::object();
        const std::pair<const char*, const HeightSetting*> all[] = {
            {"clearance", &op.heights.clearance},
            {"retract", &op.heights.retract},
            {"feed", &op.heights.feed},
            {"top", &op.heights.top},
            {"bottom", &op.heights.bottom}};
        for (const auto& [key, h] : all) {
            if (!h->isDefault()) {
                heights[key] =
                    Json{{"from", std::string(toString(h->from))}, {"offset", h->offset}};
                if (h->shape) {
                    heights[key]["shape"] = shapeRefJson(*h->shape);
                }
            }
        }
        j["heights"] = std::move(heights);
    }
    if (op.startPoint) {
        j["start_point"] = shapeRefJson(*op.startPoint);
    }
    if (op.orientation) {
        j["tool_orientation"] = Json{{"frame", frameJson(op.orientation->frame)},
                                     {"work_offset", op.orientation->workOffset}};
    }
    if (!op.presetOverrides.empty()) {
        Json o = Json::object();
        const PresetOverrides& po = op.presetOverrides;
        const std::pair<const char*, const std::optional<double>*> all[] = {
            {"surface_speed", &po.surfaceSpeed},
            {"feed_per_tooth", &po.feedPerTooth},
            {"axial_depth", &po.axialDepth},
            {"radial_depth", &po.radialDepth},
            {"plunge_feed", &po.plungeFeed},
            {"ramp_feed", &po.rampFeed},
            {"lead_feed", &po.leadFeed},
            {"part_off_end_feed", &po.partOffEndFeed},
            {"lead_feed_per_rev", &po.leadFeedPerRev},
            {"pull_feed", &po.pullFeed}};
        for (const auto& [key, v] : all) {
            if (*v) {
                o[key] = **v;
            }
        }
        j["preset_overrides"] = std::move(o);
    }
    return j;
}

Operation operationFrom(const Json& j) {
    Operation op;
    op.id = uuidFrom(j.at("id"));
    op.name = j.value("name", "");
    op.type = j.at("type").get<std::string>();
    op.enabled = j.value("enabled", true);
    if (j.contains("tool")) {
        op.toolId = uuidFrom(j.at("tool"));
    }
    if (j.contains("preset")) {
        op.presetId = uuidFrom(j.at("preset"));
    }
    for (const Json& r : j.value("geometry", Json::array())) {
        op.geometry.push_back(shapeRefFrom(r));
    }
    op.parameters = j.value("parameters", Json::object());
    if (j.contains("heights")) {
        const Json& h = j.at("heights");
        const std::pair<const char*, HeightSetting*> all[] = {{"clearance", &op.heights.clearance},
                                                              {"retract", &op.heights.retract},
                                                              {"feed", &op.heights.feed},
                                                              {"top", &op.heights.top},
                                                              {"bottom", &op.heights.bottom}};
        for (const auto& [key, setting] : all) {
            if (h.contains(key)) {
                const Json& v = h.at(key);
                setting->from =
                    parseHeightRef(v.at("from").get<std::string>()).value_or(HeightRef::Default);
                setting->offset = v.value("offset", 0.0);
                if (v.contains("shape")) {
                    setting->shape = shapeRefFrom(v.at("shape"));
                }
            }
        }
    }
    if (j.contains("start_point")) {
        op.startPoint = shapeRefFrom(j.at("start_point"));
    }
    if (j.contains("tool_orientation")) {
        const Json& o = j.at("tool_orientation");
        op.orientation = ToolOrientation{frameFrom(o.at("frame")), o.value("work_offset", 0)};
    }
    if (j.contains("preset_overrides")) {
        const Json& o = j.at("preset_overrides");
        PresetOverrides& po = op.presetOverrides;
        const std::pair<const char*, std::optional<double>*> all[] = {
            {"surface_speed", &po.surfaceSpeed},
            {"feed_per_tooth", &po.feedPerTooth},
            {"axial_depth", &po.axialDepth},
            {"radial_depth", &po.radialDepth},
            {"plunge_feed", &po.plungeFeed},
            {"ramp_feed", &po.rampFeed},
            {"lead_feed", &po.leadFeed},
            {"part_off_end_feed", &po.partOffEndFeed},
            {"lead_feed_per_rev", &po.leadFeedPerRev},
            {"pull_feed", &po.pullFeed}};
        for (const auto& [key, v] : all) {
            if (o.contains(key)) {
                *v = o.at(key).get<double>();
            }
        }
    }
    op.comment = j.value("comment", "");
    return op;
}

Json axisPickJson(const AxisPick& a) {
    Json j{{"label", a.label}, {"direction", vec(a.direction)}, {"flip", a.flip}};
    if (a.shape) {
        j["shape"] = shapeRefJson(*a.shape);
    }
    return j;
}

AxisPick axisPickFrom(const Json& j) {
    AxisPick a;
    a.label = j.value("label", "");
    a.direction = vecFrom(j.at("direction"));
    a.flip = j.value("flip", false);
    if (j.contains("shape")) {
        a.shape = shapeRefFrom(j.at("shape"));
    }
    return a;
}

constexpr std::pair<OriginFrom, const char*> kOriginFrom[] = {
    {OriginFrom::StockBox, "stock_box"},
    {OriginFrom::PartBox, "part_box"},
    {OriginFrom::FixtureBox, "fixture_box"},
    {OriginFrom::Point, "point"},
};

Json setupJson(const Setup& s) {
    Json ops = Json::array();
    for (const auto& op : s.operations) {
        ops.push_back(operationJson(op));
    }
    Json j{{"id", s.id.toString()},
           {"name", s.name},
           {"wcs", frameJson(s.wcs)},
           {"work_offset", s.workOffset},
           {"stock", stockJson(s.stock)},
           {"clearance_height", s.clearanceHeight},
           {"retract_height", s.retractHeight},
           {"machine", s.machine},
           {"operations", std::move(ops)}};
    if (const auto& o = s.orientation) {
        Json oj{{"mode", o->mode == OrientationMode::XZ   ? "xz"
                         : o->mode == OrientationMode::YZ ? "yz"
                                                          : "part"}};
        if (o->first) {
            oj["first"] = axisPickJson(*o->first);
        }
        if (o->z) {
            oj["z"] = axisPickJson(*o->z);
        }
        j["orientation"] = std::move(oj);
    }
    if (const auto& o = s.originFrom) {
        const char* from = "stock_box";
        for (const auto& [value, name] : kOriginFrom) {
            if (value == o->from) {
                from = name;
            }
        }
        Json oj{{"from", from},
                {"box", Json::array({o->box[0], o->box[1], o->box[2]})},
                {"offset", vec(o->offset)}};
        if (o->point) {
            oj["point"] = shapeRefJson(*o->point);
        }
        if (o->workholdingPoint) {
            oj["workholding_point"] = toJson(*o->workholdingPoint);
        }
        if (!o->label.empty()) {
            oj["label"] = o->label;
        }
        j["origin_from"] = std::move(oj);
    }
    if (s.modelId) {
        j["model"] = s.modelId->toString();
    }
    if (!s.fixtures.empty()) {
        Json fixtures = Json::array();
        for (const Uuid& f : s.fixtures) {
            fixtures.push_back(f.toString());
        }
        j["fixtures"] = std::move(fixtures);
    }
    if (hasFixtures(s)) {
        j["fixture_clearance"] = s.fixtureClearance;
    }
    if (const auto& t = s.tailstock) {
        j["tailstock"] = {{"tip_z", t->tipZ},
                          {"centre_diameter", t->centreDiameter},
                          {"centre_angle_deg", radToDeg(t->centreAngle)},
                          {"quill_diameter", t->quillDiameter},
                          {"quill_length", t->quillLength}};
        if (t->centre) {
            j["tailstock"]["centre"] = t->centre->toString();
        }
    }
    if (const auto& r = s.steadyRest) {
        j["steady_rest"] = {{"z", r->z},
                            {"width", r->width},
                            {"grip_diameter", r->gripDiameter},
                            {"outer_diameter", r->outerDiameter}};
    }
    if (!s.turret.empty()) {
        Json turret = Json::array();
        for (const TurretEntry& e : s.turret) {
            Json entry{{"tool", e.tool.toString()}};
            if (e.across) {
                entry["across"] = *e.across;
            }
            if (e.tip) {
                entry["tip"] = {e.tip->x(), e.tip->y()};
            }
            if (e.overhang) {
                entry["overhang"] = *e.overhang;
            }
            turret.push_back(std::move(entry));
        }
        j["turret"] = std::move(turret);
    }
    if (!s.turretBlocks.empty()) {
        Json blocks = Json::array();
        for (const StationBlock& b : s.turretBlocks) {
            blocks.push_back({{"station", b.station}, {"block", b.block.toString()}});
        }
        j["turret_blocks"] = std::move(blocks);
    }
    if (const auto& w = s.workholding) {
        Json wj{{"template", w->templateId.toString()},
                {"jaws", w->jaws == JawSetting::OnStock ? "stock" : "opening"},
                {"opening", w->opening},
                {"rotation", w->rotation},
                {"shift", w->shift}};
        if (!w->holding.empty()) {
            wj["holding"] = w->holding;
        }
        if (w->gripDepth) {
            wj["grip_depth"] = *w->gripDepth;
        }
        if (!w->travel.empty()) {
            Json travel = Json::object();
            for (const auto& [member, t] : w->travel) {
                travel[member] = t;
            }
            wj["travel"] = std::move(travel);
        }
        if (w->jawSet) {
            wj["jaw_set"] = w->jawSet->toString();
            if (!w->jawOption.empty()) {
                wj["jaw_option"] = w->jawOption;
            }
            wj["grip"] = w->grip == GripSide::Outside ? "od" : "id";
            if (!w->step.empty()) {
                wj["step"] = w->step;
            }
            if (w->stopOffset != 0.0) {
                wj["stop_offset"] = w->stopOffset;
            }
            if (const auto& b = w->softBore) {
                Json bj{{"clearance", b->clearance},
                        {"depth", b->depth},
                        {"clamp_diameter", b->clampDiameter}};
                if (b->diameter) {
                    bj["diameter"] = *b->diameter;
                }
                wj["soft_bore"] = std::move(bj);
            }
        }
        j["workholding"] = std::move(wj);
    }
    if (const auto& pl = s.placement) {
        Json pj{{"target", vec(pl->target)}};
        if (!pl->carrier.empty()) {
            pj["carrier"] = pl->carrier;
        }
        if (!pl->targetOn.empty()) {
            pj["target_on"] = pl->targetOn;
        }
        if (pl->workPoint) {
            pj["work_point"] = vec(*pl->workPoint);
        }
        if (!pl->workPointOn.empty()) {
            pj["work_point_on"] = pl->workPointOn;
        }
        if (pl->onWorkholding) {
            pj["on_workholding"] = true;
        }
        if (pl->offset != Vec3::Zero()) {
            pj["offset"] = vec(pl->offset);
        }
        if (pl->rotation != Vec3::Zero()) {
            pj["rotation"] = vec(pl->rotation);
        }
        j["placement"] = std::move(pj);
    }
    if (s.post) {
        j["post"] = {{"id", s.post->id}, {"source", s.post->inProject ? "project" : "builtin"}};
    }
    if (!s.postOptions.empty()) {
        j["post_options"] = s.postOptions;
    }
    return j;
}

Setup setupFrom(const Json& j) {
    Setup s;
    s.id = uuidFrom(j.at("id"));
    s.name = j.value("name", "");
    if (j.contains("model")) {
        s.modelId = uuidFrom(j.at("model"));
    }
    s.wcs = frameFrom(j.at("wcs"));
    if (j.contains("orientation")) {
        const Json& oj = j.at("orientation");
        WcsOrientation o;
        const std::string mode = oj.value("mode", "part");
        o.mode = mode == "xz"   ? OrientationMode::XZ
                 : mode == "yz" ? OrientationMode::YZ
                                : OrientationMode::Default;
        if (oj.contains("first")) {
            o.first = axisPickFrom(oj.at("first"));
        }
        if (oj.contains("z")) {
            o.z = axisPickFrom(oj.at("z"));
        }
        s.orientation = o;
    }
    if (j.contains("origin_from")) {
        const Json& oj = j.at("origin_from");
        WcsOrigin o;
        const std::string from = oj.value("from", "stock_box");
        for (const auto& [value, name] : kOriginFrom) {
            if (from == name) {
                o.from = value;
            }
        }
        if (oj.contains("box")) {
            for (std::size_t k = 0; k < 3; ++k) {
                o.box[k] = oj.at("box").at(k).get<int>();
            }
        }
        if (oj.contains("offset")) {
            o.offset = vecFrom(oj.at("offset"));
        }
        if (oj.contains("point")) {
            o.point = shapeRefFrom(oj.at("point"));
        }
        if (oj.contains("workholding_point")) {
            o.workholdingPoint = datumFromJson(oj.at("workholding_point"));
        }
        o.label = oj.value("label", "");
        s.originFrom = o;
    }
    s.workOffset = j.value("work_offset", 1);
    s.stock = stockFrom(j.at("stock"));
    s.clearanceHeight = j.value("clearance_height", s.clearanceHeight);
    s.retractHeight = j.value("retract_height", s.retractHeight);
    s.machine = j.value("machine", "");
    for (const Json& f : j.value("fixtures", Json::array())) {
        s.fixtures.push_back(uuidFrom(f));
    }
    s.fixtureClearance = j.value("fixture_clearance", s.fixtureClearance);
    if (const auto it = j.find("tailstock"); it != j.end()) {
        LatheTailstock t;
        t.tipZ = it->value("tip_z", t.tipZ);
        t.centreDiameter = it->value("centre_diameter", t.centreDiameter);
        t.centreAngle = degToRad(it->value("centre_angle_deg", radToDeg(t.centreAngle)));
        t.quillDiameter = it->value("quill_diameter", t.quillDiameter);
        t.quillLength = it->value("quill_length", t.quillLength);
        if (const auto c = it->find("centre"); c != it->end()) {
            t.centre = uuidFrom(*c);
        }
        s.tailstock = t;
    }
    if (const auto it = j.find("steady_rest"); it != j.end()) {
        LatheSteadyRest r;
        r.z = it->value("z", r.z);
        r.width = it->value("width", r.width);
        r.gripDiameter = it->value("grip_diameter", r.gripDiameter);
        r.outerDiameter = it->value("outer_diameter", r.outerDiameter);
        s.steadyRest = r;
    }
    for (const Json& e : j.value("turret", Json::array())) {
        TurretEntry entry;
        entry.tool = uuidFrom(e.at("tool"));
        if (const auto it = e.find("across"); it != e.end()) {
            entry.across = it->get<bool>();
        }
        if (const auto it = e.find("tip"); it != e.end()) {
            entry.tip = Vec2(it->at(0).get<double>(), it->at(1).get<double>());
        }
        if (const auto it = e.find("overhang"); it != e.end()) {
            entry.overhang = it->get<double>();
        }
        s.turret.push_back(std::move(entry));
    }
    for (const Json& b : j.value("turret_blocks", Json::array())) {
        s.turretBlocks.push_back({b.at("station").get<int>(), uuidFrom(b.at("block"))});
    }
    if (j.contains("workholding")) {
        const Json& w = j.at("workholding");
        SetupWorkholding sw;
        sw.templateId = uuidFrom(w.at("template"));
        sw.jaws = w.value("jaws", "stock") == "opening" ? JawSetting::Opening : JawSetting::OnStock;
        sw.opening = w.value("opening", sw.opening);
        sw.holding = w.value("holding", "");
        if (w.contains("grip_depth")) {
            sw.gripDepth = w.at("grip_depth").get<double>();
        }
        sw.rotation = w.value("rotation", sw.rotation);
        sw.shift = w.value("shift", sw.shift);
        const Json travel = w.value("travel", Json::object());
        for (const auto& [member, t] : travel.items()) {
            sw.travel[member] = t.get<double>();
        }
        if (w.contains("jaw_set")) {
            sw.jawSet = uuidFrom(w.at("jaw_set"));
            // An earlier draft's "jaw_reversed" and "serration" are not read.
            sw.jawOption = w.value("jaw_option", "");
            sw.grip = w.value("grip", "od") == "id" ? GripSide::Inside : GripSide::Outside;
            sw.step = w.value("step", "");
            sw.stopOffset = w.value("stop_offset", sw.stopOffset);
            if (w.contains("soft_bore")) {
                const Json& b = w.at("soft_bore");
                SoftBore bore;
                if (b.contains("diameter")) {
                    bore.diameter = b.at("diameter").get<double>();
                }
                bore.clearance = b.value("clearance", 0.0);
                bore.depth = b.at("depth").get<double>();
                bore.clampDiameter = b.at("clamp_diameter").get<double>();
                sw.softBore = bore;
            }
        }
        // Written for a few days before origin_from: the WCS origin kept on a workholding point.
        if (w.contains("origin") && !j.contains("origin_from")) {
            WcsOrigin o;
            o.from = OriginFrom::Point;
            o.workholdingPoint = datumFromJson(w.at("origin"));
            o.label = "a point of the workholding";
            s.originFrom = o;
        }
        s.workholding = sw;
    }
    if (j.contains("placement")) {
        const Json& pj = j.at("placement");
        SetupPlacement pl;
        pl.carrier = pj.value("carrier", "");
        pl.target = vecFrom(pj.at("target"));
        pl.targetOn = pj.value("target_on", "");
        if (pj.contains("work_point")) {
            pl.workPoint = vecFrom(pj.at("work_point"));
        }
        pl.workPointOn = pj.value("work_point_on", "");
        pl.onWorkholding = pj.value("on_workholding", false);
        if (pj.contains("offset")) {
            pl.offset = vecFrom(pj.at("offset"));
        }
        if (pj.contains("rotation")) {
            pl.rotation = vecFrom(pj.at("rotation"));
        }
        s.placement = pl;
    }
    if (j.contains("post")) {
        const Json& pj = j.at("post");
        s.post = PostChoice{pj.at("id").get<std::string>(), pj.value("source", "") == "project"};
    }
    s.postOptions = j.value("post_options", Json::object());
    for (const Json& op : j.value("operations", Json::array())) {
        s.operations.push_back(operationFrom(op));
    }
    return s;
}

struct ZipReader {
    mz_zip_archive zip{};
    ~ZipReader() { mz_zip_reader_end(&zip); }
};

struct ZipWriter {
    mz_zip_archive zip{};
    ~ZipWriter() { mz_zip_writer_end(&zip); }
};

// Older files set the step down and stepover as strategy parameters ("step_down",
// "depth_of_cut", "stepover_ratio", "stepover"). They are the cutting preset's axial and radial
// depth now, changed per operation through its overrides: an operation's values move there
// (unless it has overrides already) and the parameters are dropped.
void adoptStepParameters(Project& p) {
    for (Setup& s : p.setups) {
        for (Operation& op : s.operations) {
            Json& params = op.parameters;
            if (!params.is_object()) {
                continue;
            }
            const tooling::Tool* tool = op.toolId ? p.tools.findTool(*op.toolId) : nullptr;
            const double diameter = tool ? tool->mill.diameter : 0.0;
            auto take = [&](const char* key, std::optional<double>& into, double scale) {
                if (!params.contains(key)) {
                    return;
                }
                const Json v = params.at(key);
                params.erase(key);
                if (!into && v.is_number() && v.get<double>() > 0.0 && scale > 0.0) {
                    into = v.get<double>() * scale;
                }
            };
            take("step_down", op.presetOverrides.axialDepth, 1.0);
            take("depth_of_cut", op.presetOverrides.axialDepth, 1.0);
            take("stepover", op.presetOverrides.radialDepth, 1.0);
            take("stepover_ratio", op.presetOverrides.radialDepth, diameter);
        }
    }
}

// Older files gave some sizes as multiples of the tool ("helix_radius_ratio" of its radius,
// "lead_radius_ratio", "peck_ratio" and "pitch_ratio" of its diameter, "step_ratio" of a
// grooving insert's width). They are lengths now: the operation's value times its tool's size
// (dropped without a tool, so the tool's default applies).
void adoptToolLengths(Project& p) {
    for (Setup& s : p.setups) {
        for (Operation& op : s.operations) {
            Json& params = op.parameters;
            if (!params.is_object()) {
                continue;
            }
            const tooling::Tool* tool = op.toolId ? p.tools.findTool(*op.toolId) : nullptr;
            const double diameter = tool ? tool->mill.diameter : 0.0;
            const double width = tool ? tool->insert.width : 0.0;
            auto convert = [&](const char* from, const char* to, double size) {
                if (!params.contains(from)) {
                    return;
                }
                const Json v = params.at(from);
                params.erase(from);
                if (v.is_number() && size > 0.0 && !params.contains(to)) {
                    params[to] = v.get<double>() * size;
                }
            };
            convert("helix_radius_ratio", "helix_radius", 0.5 * diameter);
            convert("lead_radius_ratio", "lead_radius", diameter);
            convert("peck_ratio", "peck_depth", diameter);
            convert("pitch_ratio", "plunge_spacing", diameter);
            convert("step_ratio", "plunge_step", width);
        }
    }
}

// Contours and chamfers had one side for all their outlines ("side"); each outline has its own
// now (the side it opens to while not flipped). An operation's side goes onto its outlines
// without one, so they cut as before.
void adoptOutlineSides(Project& p) {
    for (Setup& s : p.setups) {
        for (Operation& op : s.operations) {
            if ((op.type != "chamfer2d" && op.type != "contour2d") || !op.parameters.is_object() ||
                !op.parameters.contains("side")) {
                continue;
            }
            const Json side = op.parameters.at("side");
            op.parameters.erase("side");
            if (!side.is_string()) {
                continue;
            }
            const GeometrySide value =
                side.get<std::string>() == "inside" ? GeometrySide::Inside : GeometrySide::Outside;
            for (ShapeRef& ref : op.geometry) {
                if (ref.side == GeometrySide::Default) {
                    ref.side = value;
                }
            }
        }
    }
}

// The front clearing (`zlevel_fronts`) and medial-axis (`zlevel_medial`) roughing types were
// trials beside `zlevel_rough`; the medial-axis method is `zlevel_rough` now (DL-0031). Their
// operations become `zlevel_rough`, and the traced clearing's "clearing" choice (an
// engagement limit, or none) goes: the limit is always kept.
void mergeRoughingTypes(Project& p) {
    for (Setup& s : p.setups) {
        for (Operation& op : s.operations) {
            if (op.type == "zlevel_fronts" || op.type == "zlevel_medial") {
                op.type = "zlevel_rough";
            }
            if (op.type == "zlevel_rough" && op.parameters.is_object()) {
                op.parameters.erase("clearing");
            }
        }
    }
}

// Older files: plunge feeds were a share of the cutting feed, and ramps and leads ran at the
// cutting feed,
// both following an operation's own cutting speed and feed per tooth. Each becomes the feed rate
// it gave, so every operation keeps the feeds it had. The end of a part-off ran at half the
// operation's own feed per revolution, and turning's leads at it, too. (The presets themselves are
// converted when the tools are read.)
void adoptExplicitFeeds(Project& p, const Json& doc) {
    struct OldPreset {
        std::optional<double> plungeShare; ///< The preset's plunge feed was this share.
        bool rampAtFeed = false;           ///< The preset ramped at the cutting feed.
        bool leadAtFeed = false;           ///< The preset led in and out at the cutting feed.
        bool endAtHalf = false;            ///< A part-off with it ended at half its feed.
        bool leadRevAtFeed = false;        ///< Turning with it led at the feed per revolution.
    };
    std::map<Uuid, OldPreset> old;
    for (const Json& t : doc.at("tools").value("tools", Json::array())) {
        for (const Json& pj : t.value("presets", Json::array())) {
            OldPreset o;
            if (!pj.contains("plunge_feed") && pj.contains("plunge_feed_factor")) {
                o.plungeShare = pj.at("plunge_feed_factor").get<double>();
            }
            o.rampAtFeed = !pj.contains("ramp_feed");
            o.leadAtFeed = !pj.contains("lead_feed");
            o.endAtHalf = !pj.contains("part_off_end_feed");
            o.leadRevAtFeed = !pj.contains("lead_feed_per_rev");
            if (o.plungeShare || o.rampAtFeed || o.leadAtFeed || o.endAtHalf || o.leadRevAtFeed) {
                old[uuidFrom(pj.at("id"))] = o;
            }
        }
    }
    const Json& setups = doc.at("setups");
    for (std::size_t s = 0; s < p.setups.size() && s < setups.size(); ++s) {
        const Json ops = setups[s].value("operations", Json::array());
        for (std::size_t k = 0; k < p.setups[s].operations.size() && k < ops.size(); ++k) {
            Operation& op = p.setups[s].operations[k];
            const tooling::Tool* tool = op.toolId ? p.tools.findTool(*op.toolId) : nullptr;
            const tooling::CuttingPreset* preset =
                tool && op.presetId ? tool->findPreset(*op.presetId) : nullptr;
            if (!preset) {
                continue;
            }
            PresetOverrides& po = op.presetOverrides;
            const auto was = old.find(preset->id);
            if (tooling::isTurningTool(tool->type)) {
                if (op.type == "turn_partoff" && was != old.end() && was->second.endAtHalf &&
                    po.feedPerTooth && !po.partOffEndFeed) {
                    po.partOffEndFeed = 0.5 * *po.feedPerTooth;
                }
                if (tool->type == tooling::ToolType::TurningInsert && was != old.end() &&
                    was->second.leadRevAtFeed && po.feedPerTooth && !po.leadFeedPerRev) {
                    po.leadFeedPerRev = *po.feedPerTooth;
                }
                continue;
            }
            const double vf =
                po.appliedTo(*preset).feedRate(tool->mill.diameter, tool->mill.flutes);
            const bool ownFeed = po.surfaceSpeed || po.feedPerTooth;
            const auto it = ops[k].find("preset_overrides");
            if (it != ops[k].end() && it->contains("plunge_feed_factor") &&
                !it->contains("plunge_feed")) {
                po.plungeFeed = it->at("plunge_feed_factor").get<double>() * vf;
            } else if (was != old.end() && was->second.plungeShare && ownFeed && !po.plungeFeed) {
                po.plungeFeed = *was->second.plungeShare * vf;
            }
            if (was != old.end() && was->second.rampAtFeed && ownFeed && !po.rampFeed) {
                po.rampFeed = vf;
            }
            if (was != old.end() && was->second.leadAtFeed && ownFeed && !po.leadFeed) {
                po.leadFeed = vf;
            }
        }
    }
}

} // namespace

const FormatSpec& projectFormat() {
    static const FormatSpec spec{"oxcam.project", kProjectVersion, {}, {"opencam.project"}};
    return spec;
}

const SchemaValidator& projectSchema() {
    static const SchemaValidator validator(
        parseJson(embedded::projectSchemaV1, "project.v1.schema.json"));
    return validator;
}

Json toJson(const Project& project) {
    Json models = Json::array();
    for (const auto& m : project.models) {
        models.push_back({{"id", m.id.toString()},
                          {"name", m.name},
                          {"source_path", m.sourcePath},
                          {"sha256", m.sha256},
                          {"placement", frameJson(m.placement)}});
    }
    Json setups = Json::array();
    for (const auto& s : project.setups) {
        setups.push_back(setupJson(s));
    }
    Json doc = {{"format", projectFormat().format},
                {"version", kProjectVersion},
                {"units", "mm"},
                {"id", project.id.toString()},
                {"name", project.name},
                {"display_unit", std::string(toString(project.displayUnit))},
                {"notes", project.notes},
                {"models", std::move(models)},
                {"tools", tooling::toJson(project.tools)},
                {"setups", std::move(setups)}};
    if (!project.machines.empty()) {
        Json machines = Json::array();
        for (const auto& m : project.machines) {
            machines.push_back(machine::toJson(m));
        }
        doc["machines"] = std::move(machines);
    }
    if (!project.fittings.empty()) {
        Json fittings = Json::array();
        for (const LatheFitting& f : project.fittings) {
            fittings.push_back(toJson(f));
        }
        doc["fittings"] = std::move(fittings);
    }
    if (!project.workholdings.empty()) {
        Json workholdings = Json::array();
        for (const auto& w : project.workholdings) {
            workholdings.push_back(toJson(w));
        }
        doc["workholdings"] = std::move(workholdings);
    }
    if (!project.posts.empty()) {
        Json posts = Json::array();
        for (const auto& post : project.posts) {
            posts.push_back({{"file", post.file}, {"source", post.source}});
        }
        doc["posts"] = std::move(posts);
    }
    return doc;
}

Project projectFromJson(Json doc, std::string_view sourceName) {
    projectFormat().upgrade(doc, sourceName);
    projectSchema().validate(doc, sourceName);
    try {
        Project p;
        p.id = uuidFrom(doc.at("id"));
        p.name = doc.at("name").get<std::string>();
        p.displayUnit =
            parseLengthUnit(doc.value("display_unit", "mm")).value_or(LengthUnit::Millimeter);
        p.notes = doc.value("notes", "");
        for (const Json& m : doc.at("models")) {
            ModelEntry e;
            e.id = uuidFrom(m.at("id"));
            e.name = m.value("name", "");
            e.sourcePath = m.value("source_path", "");
            e.sha256 = m.at("sha256").get<std::string>();
            if (m.contains("placement")) {
                e.placement = frameFrom(m.at("placement"));
            }
            p.models.push_back(std::move(e));
        }
        p.tools =
            tooling::toolLibraryFromJson(doc.at("tools"), fmt::format("{} (tools)", sourceName));
        for (const Json& m : doc.value("machines", Json::array())) {
            p.machines.push_back(
                machine::machineFromJson(m, fmt::format("{} (machine)", sourceName)));
        }
        for (const Json& f : doc.value("fittings", Json::array())) {
            p.fittings.push_back(fittingFromJson(f, fmt::format("{} (fitting)", sourceName)));
        }
        for (const Json& w : doc.value("workholdings", Json::array())) {
            p.workholdings.push_back(
                workholdingFromJson(w, fmt::format("{} (workholding)", sourceName)));
        }
        for (const Json& post : doc.value("posts", Json::array())) {
            p.posts.push_back(
                {post.value("file", std::string()), post.at("source").get<std::string>()});
        }
        for (const Json& s : doc.at("setups")) {
            p.setups.push_back(setupFrom(s));
        }
        adoptStepParameters(p);
        mergeRoughingTypes(p);
        adoptExplicitFeeds(p, doc);
        adoptToolLengths(p);
        adoptOutlineSides(p);
        return p;
    } catch (const Json::exception& e) {
        throw Error(ErrorCode::Parse, fmt::format("{}: {}", sourceName, e.what()));
    } catch (const std::bad_optional_access&) {
        throw Error(ErrorCode::Parse, fmt::format("{}: unknown enumeration value", sourceName));
    }
}

std::string writeArchive(const ArchiveContents& contents) {
    ZipWriter w;
    if (!mz_zip_writer_init_heap(&w.zip, 0, 64 * 1024)) {
        throw Error(ErrorCode::Internal, "cannot initialize zip writer");
    }
    auto add = [&](const std::string& name, const std::string& data) {
        if (!mz_zip_writer_add_mem(&w.zip, name.c_str(), data.data(), data.size(),
                                   MZ_DEFAULT_LEVEL)) {
            throw Error(ErrorCode::Internal, "cannot add zip entry " + name);
        }
    };
    add("project.json", contents.projectJson);
    for (const auto& [sha, bytes] : contents.models) {
        add("models/" + sha + ".step", bytes);
    }
    for (const auto& [file, bytes] : contents.machineFiles) {
        add("machines/" + file, bytes);
    }
    if (!contents.toolpathCache.empty()) {
        add("cache/toolpaths.json", contents.toolpathCache);
    }
    void* buffer = nullptr;
    size_t size = 0;
    if (!mz_zip_writer_finalize_heap_archive(&w.zip, &buffer, &size)) {
        throw Error(ErrorCode::Internal, "cannot finalize zip archive");
    }
    std::unique_ptr<void, decltype(&mz_free)> owned(buffer, &mz_free);
    return {static_cast<const char*>(buffer), size};
}

ArchiveContents readArchive(std::string_view bytes, std::string_view sourceName) {
    ZipReader r;
    if (!mz_zip_reader_init_mem(&r.zip, bytes.data(), bytes.size(), 0)) {
        throw Error(ErrorCode::Parse, fmt::format("{}: not an OxCAM project archive", sourceName));
    }
    static const std::regex modelName("^models/([0-9a-f]{64})\\.step$");
    static const std::regex machineName("^machines/(([0-9a-f]{64})\\.(stl|step))$");

    ArchiveContents out;
    bool haveProject = false;
    const mz_uint count = mz_zip_reader_get_num_files(&r.zip);
    for (mz_uint i = 0; i < count; ++i) {
        mz_zip_archive_file_stat stat{};
        if (!mz_zip_reader_file_stat(&r.zip, i, &stat) || stat.m_is_directory) {
            continue;
        }
        const std::string name = stat.m_filename;
        std::smatch m;
        const bool isProject = name == "project.json";
        const bool isModel = std::regex_match(name, m, modelName);
        const bool isMesh = !isModel && std::regex_match(name, m, machineName);
        const bool isCache = name == "cache/toolpaths.json";
        if (!isProject && !isModel && !isMesh && !isCache) {
            continue;
        }
        if (stat.m_uncomp_size > kMaxEntryBytes) {
            throw Error(ErrorCode::Parse,
                        fmt::format("{}: entry {} is too large", sourceName, name));
        }
        size_t size = 0;
        void* data = mz_zip_reader_extract_to_heap(&r.zip, i, &size, 0);
        if (!data) {
            throw Error(ErrorCode::Parse, fmt::format("{}: cannot extract {}", sourceName, name));
        }
        std::unique_ptr<void, decltype(&mz_free)> owned(data, &mz_free);
        std::string content(static_cast<const char*>(data), size);
        if (isProject) {
            out.projectJson = std::move(content);
            haveProject = true;
        } else if (isCache) {
            out.toolpathCache = std::move(content);
        } else {
            const std::string sha = m[isMesh ? 2 : 1].str();
            if (sha256Hex(content) != sha) {
                throw Error(ErrorCode::Parse,
                            fmt::format("{}: embedded {} {} is corrupt (hash mismatch)", sourceName,
                                        isMesh ? "machine part file" : "model", name));
            }
            if (isMesh) {
                out.machineFiles.emplace(m[1].str(), std::move(content));
            } else {
                out.models.emplace(sha, std::move(content));
            }
        }
    }
    if (!haveProject) {
        throw Error(ErrorCode::Parse, fmt::format("{}: archive has no project.json", sourceName));
    }
    return out;
}

} // namespace ocam::document
