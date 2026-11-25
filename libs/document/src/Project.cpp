// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/document/Project.hpp"

#include <ocam/foundation/Error.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <set>

namespace ocam::document {

namespace {
constexpr std::pair<HeightRef, std::string_view> kHeightRefs[] = {
    {HeightRef::Default, "default"},
    {HeightRef::StockTop, "stock_top"},
    {HeightRef::StockBottom, "stock_bottom"},
    {HeightRef::PartTop, "part_top"},
    {HeightRef::PartBottom, "part_bottom"},
    {HeightRef::Geometry, "geometry"},
    {HeightRef::Origin, "origin"},
    {HeightRef::Clearance, "clearance"},
    {HeightRef::Retract, "retract"},
    {HeightRef::TopHeight, "top"},
    {HeightRef::Picked, "picked"},
    {HeightRef::HoleTop, "hole_top"},
};
} // namespace

bool usesTopAndBottom(std::string_view type) noexcept {
    for (const std::string_view t :
         {"facing", "pocket2d", "plunge_rough", "contour2d", "zlevel_rough", "flat3d", "contour3d",
          "waterline3d", "steep_shallow3d"}) {
        if (type == t) {
            return true;
        }
    }
    return false;
}

bool isHoleOperation(std::string_view type) noexcept {
    return type == "drill" || type == "tap" || type == "threadmill" || type == "bore" ||
           type == "backbore";
}

std::optional<Uuid> trailingBarPull(const Setup& setup) {
    std::optional<Uuid> first;
    for (auto it = setup.operations.rbegin(); it != setup.operations.rend(); ++it) {
        if (!it->enabled) {
            continue;
        }
        if (it->type != "bar_pull") {
            break;
        }
        first = it->id;
    }
    return first;
}

OperationHeights defaultHeights(const Operation& op) {
    using R = HeightRef;
    OperationHeights h;
    h.clearance = {R::Retract, kDefaultHeightAllowance, std::nullopt};
    h.retract = {R::StockTop, kDefaultHeightAllowance, std::nullopt};
    if (isHoleOperation(op.type)) {
        // The R plane above each hole; earlier versions kept it as a strategy parameter.
        double above = 2.0;
        if (op.parameters.is_object() && op.parameters.contains("retract_clearance") &&
            op.parameters.at("retract_clearance").is_number()) {
            above = op.parameters.at("retract_clearance").get<double>();
        }
        h.feed = {R::HoleTop, above, std::nullopt};
    } else {
        h.feed = {R::Retract, 0.0, std::nullopt};
    }
    // Roughing starts at the stock top; finishing at the part top.
    const bool roughing = op.type == "facing" || op.type == "pocket2d" ||
                          op.type == "plunge_rough" || op.type == "zlevel_rough";
    h.top = {roughing ? R::StockTop : R::PartTop, 0.0, std::nullopt};
    if (op.type == "facing") {
        h.bottom = {R::PartTop, 0.0, std::nullopt}; // faced down to the part's top
    } else if (op.type == "pocket2d" || op.type == "plunge_rough" || op.type == "contour2d") {
        h.bottom = {R::Geometry, 0.0, std::nullopt}; // down to the selected floor or outline
    } else {
        h.bottom = {R::PartBottom, 0.0, std::nullopt};
    }
    return h;
}

OperationHeights effectiveHeights(const Operation& op) {
    const OperationHeights defaults = defaultHeights(op);
    OperationHeights h = op.heights;
    for (const auto field :
         {&OperationHeights::clearance, &OperationHeights::retract, &OperationHeights::feed,
          &OperationHeights::top, &OperationHeights::bottom}) {
        if ((h.*field).isDefault()) {
            h.*field = defaults.*field;
        }
    }
    return h;
}

std::string_view toString(HeightRef ref) noexcept {
    for (const auto& [r, name] : kHeightRefs) {
        if (r == ref) {
            return name;
        }
    }
    return "default";
}

std::optional<HeightRef> parseHeightRef(std::string_view text) noexcept {
    for (const auto& [r, name] : kHeightRefs) {
        if (name == text) {
            return r;
        }
    }
    return std::nullopt;
}

tooling::CuttingPreset PresetOverrides::appliedTo(const tooling::CuttingPreset& preset) const {
    tooling::CuttingPreset p = preset;
    p.surfaceSpeed = surfaceSpeed.value_or(p.surfaceSpeed);
    p.feedPerTooth = feedPerTooth.value_or(p.feedPerTooth);
    p.axialDepth = axialDepth.value_or(p.axialDepth);
    p.radialDepth = radialDepth.value_or(p.radialDepth);
    p.plungeFeed = plungeFeed.value_or(p.plungeFeed);
    p.rampFeed = rampFeed.value_or(p.rampFeed);
    p.leadFeed = leadFeed.value_or(p.leadFeed);
    p.partOffEndFeed = partOffEndFeed.value_or(p.partOffEndFeed);
    p.leadFeedPerRev = leadFeedPerRev.value_or(p.leadFeedPerRev);
    p.pullFeed = pullFeed.value_or(p.pullFeed);
    return p;
}
namespace {

constexpr std::array<std::pair<StockKind, std::string_view>, 4> kStockKinds{{
    {StockKind::Box, "box"},
    {StockKind::Cylinder, "cylinder"},
    {StockKind::Tube, "tube"},
    {StockKind::FromSetup, "from_setup"},
}};

constexpr std::array<std::pair<StockSize, std::string_view>, 3> kStockSizes{{
    {StockSize::Fixed, "fixed"},
    {StockSize::OffsetFromPart, "offset_from_part"},
    {StockSize::Part, "part"},
}};

} // namespace

Transform3 Frame::toParent() const {
    const Frame f = normalized();
    Transform3 t = Transform3::Identity();
    t.linear().col(0) = f.xAxis;
    t.linear().col(1) = f.yAxis();
    t.linear().col(2) = f.zAxis;
    t.translation() = f.origin;
    return t;
}

Frame Frame::normalized() const {
    const double zn = zAxis.norm();
    if (zn < 1e-12) {
        throw Error(ErrorCode::InvalidArgument, "frame Z axis is zero");
    }
    const Vec3 z = zAxis / zn;
    const Vec3 x = xAxis - xAxis.dot(z) * z;
    const double xn = x.norm();
    if (xn < 1e-9) {
        throw Error(ErrorCode::InvalidArgument, "frame X axis is zero or parallel to Z");
    }
    return {origin, x / xn, z};
}

std::string_view toString(StockKind kind) noexcept {
    for (const auto& [k, name] : kStockKinds) {
        if (k == kind) {
            return name;
        }
    }
    return "box";
}

std::optional<StockKind> parseStockKind(std::string_view text) noexcept {
    for (const auto& [k, name] : kStockKinds) {
        if (name == text) {
            return k;
        }
    }
    return std::nullopt;
}

std::string_view toString(StockSize size) noexcept {
    for (const auto& [k, name] : kStockSizes) {
        if (k == size) {
            return name;
        }
    }
    return "fixed";
}

std::optional<StockSize> parseStockSize(std::string_view text) noexcept {
    for (const auto& [k, name] : kStockSizes) {
        if (name == text) {
            return k;
        }
    }
    return std::nullopt;
}

namespace {
constexpr std::pair<DatumKind, std::string_view> kDatumKinds[] = {
    {DatumKind::Picked, "picked"},
    {DatumKind::Typed, "typed"},
    {DatumKind::Offset, "offset"},
    {DatumKind::Midpoint, "midpoint"},
};
constexpr std::pair<DatumShape, std::string_view> kDatumShapes[] = {
    {DatumShape::Plane, "plane"},
    {DatumShape::Axis, "axis"},
    {DatumShape::Line, "line"},
    {DatumShape::Point, "point"},
};
constexpr std::pair<DatumAxis, std::string_view> kDatumAxes[] = {
    {DatumAxis::X, "x"},
    {DatumAxis::Y, "y"},
    {DatumAxis::Z, "z"},
    {DatumAxis::Direction, "direction"},
};
constexpr std::pair<HoldingKind, std::string_view> kHoldingKinds[] = {
    {HoldingKind::Jaws, "jaws"},
    {HoldingKind::Dovetail, "dovetail"},
    {HoldingKind::Seat, "seat"},
    {HoldingKind::Chuck, "chuck"},
};

template <typename E, std::size_t N>
std::string_view nameOf(const std::pair<E, std::string_view> (&table)[N], E value) noexcept {
    for (const auto& [v, name] : table) {
        if (v == value) {
            return name;
        }
    }
    return table[0].second;
}

template <typename E, std::size_t N>
std::optional<E> valueOf(const std::pair<E, std::string_view> (&table)[N],
                         std::string_view text) noexcept {
    for (const auto& [v, name] : table) {
        if (name == text) {
            return v;
        }
    }
    return std::nullopt;
}

template <typename T> const T* byName(const std::vector<T>& items, std::string_view name) {
    auto it = std::find_if(items.begin(), items.end(), [&](const T& t) { return t.name == name; });
    return it == items.end() ? nullptr : &*it;
}
} // namespace

std::string_view toString(DatumKind kind) noexcept {
    return nameOf(kDatumKinds, kind);
}
std::optional<DatumKind> parseDatumKind(std::string_view text) noexcept {
    return valueOf(kDatumKinds, text);
}
std::string_view toString(DatumShape shape) noexcept {
    return nameOf(kDatumShapes, shape);
}
std::optional<DatumShape> parseDatumShape(std::string_view text) noexcept {
    return valueOf(kDatumShapes, text);
}
std::string_view toString(DatumAxis axis) noexcept {
    return nameOf(kDatumAxes, axis);
}
std::optional<DatumAxis> parseDatumAxis(std::string_view text) noexcept {
    return valueOf(kDatumAxes, text);
}
std::string_view toString(HoldingKind kind) noexcept {
    return nameOf(kHoldingKinds, kind);
}
std::optional<HoldingKind> parseHoldingKind(std::string_view text) noexcept {
    return valueOf(kHoldingKinds, text);
}

const WorkholdingPart* WorkholdingTemplate::findPart(const Uuid& wanted) const {
    auto it = std::find_if(parts.begin(), parts.end(),
                           [&](const WorkholdingPart& p) { return p.id == wanted; });
    return it == parts.end() ? nullptr : &*it;
}

const WorkholdingMember* WorkholdingTemplate::findMember(std::string_view wanted) const {
    return byName(members, wanted);
}

const WorkholdingDatum* WorkholdingTemplate::findDatum(std::string_view wanted) const {
    return byName(datums, wanted);
}

const StockHolding* WorkholdingTemplate::findHolding(std::string_view wanted) const {
    return byName(holdings, wanted);
}

const JawOption* JawSet::findOption(std::string_view wanted) const {
    if (wanted.empty()) {
        return options.empty() ? nullptr : &options.front();
    }
    const auto it = std::find_if(options.begin(), options.end(),
                                 [&](const JawOption& o) { return o.name == wanted; });
    return it == options.end() ? nullptr : &*it;
}

std::vector<std::string> WorkholdingTemplate::validate() const {
    std::vector<std::string> issues;
    if (name.empty()) {
        issues.emplace_back("the workholding needs a name");
    }
    if (parts.empty()) {
        issues.emplace_back("the workholding has no parts");
    }
    auto memberKnown = [&](const std::string& m) {
        return m.empty() || findMember(m);
    };
    std::set<Uuid> ids;
    for (const WorkholdingPart& p : parts) {
        if (!ids.insert(p.id).second) {
            issues.push_back("part '" + p.name + "': duplicate id " + p.id.toString());
        }
        if (p.sha256.size() != 64) {
            issues.push_back("part '" + p.name + "': invalid file hash");
        }
        if (!memberKnown(p.member)) {
            issues.push_back("part '" + p.name + "': no member '" + p.member + "'");
        }
    }
    std::set<std::string> names;
    for (const WorkholdingMember& m : members) {
        if (m.name.empty() || !names.insert(m.name).second) {
            issues.push_back("members need different, non-empty names ('" + m.name + "')");
        }
        if (m.direction.norm() < 1e-9) {
            issues.push_back("member '" + m.name + "': no direction");
        }
        if (m.maxTravel < m.minTravel) {
            issues.push_back("member '" + m.name + "': the largest travel is below the smallest");
        }
        if (!m.linkedTo.empty()) {
            const WorkholdingMember* leader = findMember(m.linkedTo);
            if (!leader || leader == &m) {
                issues.push_back("member '" + m.name + "': linked to an unknown member");
            } else if (!leader->linkedTo.empty()) {
                issues.push_back("member '" + m.name + "': linked to '" + leader->name +
                                 "', which follows another member itself");
            }
        }
    }
    names.clear();
    for (const WorkholdingDatum& d : datums) {
        const std::string label = "datum '" + d.name + "'";
        if (d.name.empty() || !names.insert(d.name).second) {
            issues.push_back("datums need different, non-empty names ('" + d.name + "')");
        }
        switch (d.kind) {
        case DatumKind::Picked:
            if (!d.part || !findPart(*d.part)) {
                issues.push_back(label + ": picked on a part that is gone");
            }
            break;
        case DatumKind::Typed:
            if (!memberKnown(d.member)) {
                issues.push_back(label + ": no member '" + d.member + "'");
            }
            break;
        case DatumKind::Offset:
        case DatumKind::Midpoint:
            if (!findDatum(d.from) || (d.kind == DatumKind::Midpoint && !findDatum(d.to))) {
                issues.push_back(label + ": refers to a datum that does not exist");
            }
            break;
        }
    }
    // Offsets and midpoints must not refer back to themselves.
    for (const WorkholdingDatum& d : datums) {
        std::set<std::string> seen{d.name};
        std::vector<const WorkholdingDatum*> todo{&d};
        bool cyclic = false;
        while (!todo.empty() && !cyclic) {
            const WorkholdingDatum* x = todo.back();
            todo.pop_back();
            if (x->kind != DatumKind::Offset && x->kind != DatumKind::Midpoint) {
                continue;
            }
            for (const std::string* ref : {&x->from, &x->to}) {
                if (ref->empty() || (x->kind == DatumKind::Offset && ref == &x->to)) {
                    continue;
                }
                if (*ref == d.name) {
                    cyclic = true;
                } else if (const WorkholdingDatum* r = findDatum(*ref);
                           r && seen.insert(r->name).second) {
                    todo.push_back(r);
                }
            }
        }
        if (cyclic) {
            issues.push_back("datum '" + d.name + "' is defined from itself");
        }
    }
    if (!mount.empty() && !findDatum(mount)) {
        issues.push_back("the mount datum '" + mount + "' does not exist");
    }
    if (up.norm() < 1e-9) {
        issues.emplace_back("the up direction is zero");
    }
    if (holdings.empty() && !jawSet) {
        issues.emplace_back("the workholding has no stock holding");
    }
    if (const auto& m = jawMount) {
        if (m->interfaceName.empty()) {
            issues.emplace_back("the jaw mount needs its interface's name");
        }
        if (!findDatum(m->seat)) {
            issues.push_back("the jaw mount's seat '" + m->seat + "' is not a datum");
        }
        if (!findMember("Jaw 1")) {
            issues.emplace_back("a jaw mount needs the chuck's master jaws (members 'Jaw 1' on)");
        }
    }
    if (const auto& s = jawSet) {
        if (jawMount) {
            issues.emplace_back("a jaw set is not a chuck: it has no jaw mount");
        }
        if (!holdings.empty() || !members.empty()) {
            issues.emplace_back("a jaw set is one jaw: no members or stock holdings");
        }
        if (s->interfaceName.empty()) {
            issues.emplace_back("the jaw set needs its interface's name");
        }
        if (s->kind == JawSet::Kind::Soft && !findDatum(s->front)) {
            issues.emplace_back(
                "soft jaws need their front (a datum): they are bored back from it");
        }
        if (s->options.empty()) {
            issues.emplace_back(
                "the jaw set needs a way to go on: an option with its seat and steps");
        }
        std::set<std::string> optionNames;
        for (const JawOption& o : s->options) {
            if (o.name.empty() || !optionNames.insert(o.name).second) {
                issues.push_back("a jaw set's options need different, non-empty names ('" + o.name +
                                 "')");
            }
            if (!findDatum(o.seat)) {
                issues.push_back("the seat '" + o.seat + "' of option '" + o.name +
                                 "' is not a datum");
            }
            if (o.inward.norm() < 1e-9) {
                issues.push_back("option '" + o.name +
                                 "' needs the way in towards the chuck's axis");
            }
            if (o.steps.empty()) {
                issues.push_back("option '" + o.name +
                                 "' needs a step: the face that grips and the one the work stops "
                                 "against");
            }
            std::set<std::string> stepNames;
            for (const JawStep& st : o.steps) {
                if (st.name.empty() || !stepNames.insert(st.name).second) {
                    issues.push_back("steps need different, non-empty names ('" + st.name + "')");
                }
                if (!findDatum(st.grip) || !findDatum(st.stop)) {
                    issues.push_back("step '" + st.name + "': its faces must be datums");
                }
            }
        }
    }
    names.clear();
    for (const StockHolding& h : holdings) {
        const std::string label = "holding '" + h.name + "'";
        if (h.name.empty() || !names.insert(h.name).second) {
            issues.push_back("holdings need different, non-empty names ('" + h.name + "')");
        }
        auto need = [&](const std::string& datum, const char* what) {
            if (datum.empty()) {
                issues.push_back(label + ": choose " + what);
            } else if (!findDatum(datum)) {
                issues.push_back(label + ": no datum '" + datum + "'");
            }
        };
        switch (h.kind) {
        case HoldingKind::Jaws:
            need(h.faceA, "the first jaw face");
            need(h.faceB, "the second jaw face");
            need(h.seat, "the jaw tops");
            if (h.gripDepth < 0.0) {
                issues.push_back(label + ": the grip depth must not be negative");
            }
            break;
        case HoldingKind::Dovetail:
            need(h.faceA, "the first flank");
            need(h.faceB, "the second flank");
            need(h.seat, "the seat");
            if (h.depth <= 0.0 || h.tenonWidth < 0.0) {
                issues.push_back(label + ": the tenon needs a positive depth");
            }
            break;
        case HoldingKind::Chuck:
            need(h.axis, "the spindle axis");
            need(h.faceA, "a jaw's gripping face");
            need(h.seat, "the front of the jaws");
            if (h.gripDepth < 0.0 || h.gripRange < 0.0) {
                issues.push_back(label + ": the grip length and range must not be negative");
            }
            break;
        case HoldingKind::Seat:
            need(h.seat, "the seat");
            if (!h.xDatum.empty() && !findDatum(h.xDatum)) {
                issues.push_back(label + ": no datum '" + h.xDatum + "'");
            }
            for (const int a : h.anchor) {
                if (a < 0 || a > 2) {
                    issues.push_back(label + ": the stock point is min, middle or max per axis");
                    break;
                }
            }
            break;
        }
    }
    return issues;
}

bool hasFixtures(const Setup& setup) noexcept {
    return !setup.fixtures.empty() || setup.workholding.has_value() ||
           setup.tailstock.has_value() || setup.steadyRest.has_value();
}

const ModelEntry* Project::findModel(const Uuid& wanted) const {
    auto it = std::find_if(models.begin(), models.end(),
                           [&](const ModelEntry& m) { return m.id == wanted; });
    return it == models.end() ? nullptr : &*it;
}

const Setup* Project::findSetup(const Uuid& wanted) const {
    auto it =
        std::find_if(setups.begin(), setups.end(), [&](const Setup& s) { return s.id == wanted; });
    return it == setups.end() ? nullptr : &*it;
}

Setup* Project::findSetup(const Uuid& wanted) {
    auto it =
        std::find_if(setups.begin(), setups.end(), [&](const Setup& s) { return s.id == wanted; });
    return it == setups.end() ? nullptr : &*it;
}

const Operation* Project::findOperation(const Uuid& wanted, const Setup** setup) const {
    for (const auto& s : setups) {
        for (const auto& op : s.operations) {
            if (op.id == wanted) {
                if (setup) {
                    *setup = &s;
                }
                return &op;
            }
        }
    }
    return nullptr;
}

Operation* Project::findOperation(const Uuid& wanted, Setup** setup) {
    for (auto& s : setups) {
        for (auto& op : s.operations) {
            if (op.id == wanted) {
                if (setup) {
                    *setup = &s;
                }
                return &op;
            }
        }
    }
    return nullptr;
}

bool Project::moveOperation(const Uuid& operationId, const Uuid& setupId, std::size_t index) {
    Setup* from = nullptr;
    const Operation* op = findOperation(operationId, &from);
    Setup* to = findSetup(setupId);
    if (!op || !to) {
        return false;
    }
    const auto at = static_cast<std::size_t>(op - from->operations.data());
    index = std::min(index, to->operations.size());
    if (from == to) {
        if (index == at || index == at + 1) {
            return false; // in front of itself or of the next one: same place
        }
        if (index > at) {
            --index; // the operation leaves its place first
        }
    }
    Operation moved = std::move(from->operations[at]);
    from->operations.erase(from->operations.begin() + static_cast<std::ptrdiff_t>(at));
    to->operations.insert(to->operations.begin() + static_cast<std::ptrdiff_t>(index),
                          std::move(moved));
    return true;
}

const machine::Machine* Project::findMachine(std::string_view machineId) const {
    for (const auto& m : machines) {
        if (m.id == machineId) {
            return &m;
        }
    }
    return nullptr;
}

const WorkholdingTemplate* Project::findWorkholding(const Uuid& wanted) const {
    auto it = std::find_if(workholdings.begin(), workholdings.end(),
                           [&](const WorkholdingTemplate& w) { return w.id == wanted; });
    return it == workholdings.end() ? nullptr : &*it;
}

const LatheFitting* Project::findFitting(const Uuid& wanted) const {
    auto it = std::find_if(fittings.begin(), fittings.end(),
                           [&](const LatheFitting& f) { return f.id == wanted; });
    return it == fittings.end() ? nullptr : &*it;
}

std::vector<std::string> Project::validate() const {
    std::vector<std::string> issues;
    std::set<Uuid> ids;
    auto unique = [&](const Uuid& itemId, const std::string& what) {
        if (!ids.insert(itemId).second) {
            issues.push_back(what + ": duplicate id " + itemId.toString());
        }
    };

    for (const auto& m : models) {
        unique(m.id, "model '" + m.name + "'");
        if (m.sha256.size() != 64) {
            issues.push_back("model '" + m.name + "': missing content hash");
        }
        try {
            (void)m.placement.normalized();
        } catch (const Error&) {
            issues.push_back("model '" + m.name + "': invalid placement frame");
        }
    }

    for (const auto& issue : tools.validate()) {
        issues.push_back("tools: " + issue);
    }

    std::set<std::string> machineIds;
    for (const auto& m : machines) {
        if (!machineIds.insert(m.id).second) {
            issues.push_back("machine '" + m.id + "': duplicate id");
        }
        for (const auto& issue : m.validate()) {
            issues.push_back("machine '" + m.id + "': " + issue);
        }
    }
    for (const auto& w : workholdings) {
        unique(w.id, "workholding '" + w.name + "'");
        for (const auto& issue : w.validate()) {
            issues.push_back("workholding '" + w.name + "': " + issue);
        }
    }
    for (std::size_t i = 0; i < setups.size(); ++i) {
        const Setup& s = setups[i];
        const std::string label = "setup '" + s.name + "'";
        unique(s.id, label);
        if (s.modelId && !findModel(*s.modelId)) {
            issues.push_back(label + ": references a missing model");
        }
        try {
            (void)s.wcs.normalized();
        } catch (const Error&) {
            issues.push_back(label + ": invalid work coordinate system");
        }
        if (s.workOffset < 1) {
            issues.push_back(label + ": work offset must be at least 1");
        }
        if (!s.machine.empty() && !findMachine(s.machine) && !machine::builtinMachine(s.machine)) {
            issues.push_back(label + ": unknown machine '" + s.machine + "'");
        }
        std::set<Uuid> fixtureIds;
        for (const Uuid& f : s.fixtures) {
            if (!findModel(f)) {
                issues.push_back(label + ": a fixture references a missing model");
            } else if (s.modelId == f) {
                issues.push_back(label + ": the part cannot be a fixture of its own setup");
            }
            if (!fixtureIds.insert(f).second) {
                issues.push_back(label + ": a fixture is listed more than once");
            }
        }
        for (const StationBlock& b : s.turretBlocks) {
            const LatheFitting* block = findFitting(b.block);
            if (!block || block->kind != LatheFitting::Kind::Block) {
                issues.push_back(label + fmt::format(": station {} has a block that is not in the "
                                                     "project",
                                                     b.station));
            }
        }
        if (s.tailstock && s.tailstock->centre) {
            const LatheFitting* centre = findFitting(*s.tailstock->centre);
            if (!centre || centre->kind != LatheFitting::Kind::Centre) {
                issues.push_back(label + ": the tailstock's centre is not in the project");
            }
        }
        if (s.tailstock && (s.tailstock->centreDiameter <= 0.0 ||
                            s.tailstock->quillDiameter <= 0.0 || s.tailstock->quillLength <= 0.0 ||
                            s.tailstock->centreAngle <= 0.0 || s.tailstock->centreAngle >= kPi)) {
            issues.push_back(fmt::format("setup '{}': the tailstock's sizes must be positive, its "
                                         "centre's angle below 180 degrees",
                                         s.name));
        }
        if (s.steadyRest && (s.steadyRest->width <= 0.0 || s.steadyRest->gripDiameter < 0.0 ||
                             s.steadyRest->outerDiameter <= s.steadyRest->gripDiameter)) {
            issues.push_back(fmt::format("setup '{}': the steady rest needs a width, and an outer "
                                         "diameter above what it grips",
                                         s.name));
        }
        if (s.fixtureClearance < 0.0) {
            issues.push_back(label + ": fixture clearance must not be negative");
        }
        if (const auto& w = s.workholding) {
            const WorkholdingTemplate* t = findWorkholding(w->templateId);
            if (!t) {
                issues.push_back(label + ": the workholding references a missing template");
            } else {
                if (!w->holding.empty() && !t->findHolding(w->holding)) {
                    issues.push_back(label + ": the workholding has no holding '" + w->holding +
                                     "'");
                }
                if (w->jawSet) {
                    const WorkholdingTemplate* set = findWorkholding(*w->jawSet);
                    if (!set || !set->jawSet) {
                        issues.push_back(label + ": the chuck's jaw set is not in the project");
                    } else if (!t->jawMount) {
                        issues.push_back(label + ": jaws are set on '" + t->name +
                                         "', which has no jaw mount");
                    } else if (const JawOption* option = set->jawSet->findOption(w->jawOption);
                               !option) {
                        issues.push_back(label + ": the jaw set '" + set->name +
                                         "' has no option '" + w->jawOption + "'");
                    } else if (!w->step.empty() &&
                               std::none_of(
                                   option->steps.begin(), option->steps.end(),
                                   [&](const JawStep& st) { return st.name == w->step; })) {
                        issues.push_back(label + ": the jaw set '" + set->name + "' has no step '" +
                                         w->step + "' as '" + option->name + "'");
                    }
                    if (w->softBore && set && set->jawSet &&
                        set->jawSet->kind != JawSet::Kind::Soft) {
                        issues.push_back(label + ": only soft jaws are bored to fit");
                    }
                    if (const auto& b = w->softBore;
                        b && (!(b->depth > 0.0) || !(b->clampDiameter > 0.0) ||
                              (b->diameter && !(*b->diameter > 0.0)))) {
                        issues.push_back(label + ": the soft jaws' bore needs its depth and the "
                                                 "diameter they clamp while bored, more than 0");
                    }
                }
                for (const auto& [member, travel] : w->travel) {
                    if (!t->findMember(member)) {
                        issues.push_back(label + ": the workholding has no member '" + member +
                                         "'");
                    }
                }
                if (const auto& o = s.originFrom; o && o->workholdingPoint) {
                    const WorkholdingDatum& d = *o->workholdingPoint;
                    if (d.kind == DatumKind::Picked && (!d.part || !t->findPart(*d.part))) {
                        issues.push_back(label + ": the WCS origin was picked on a part the "
                                                 "workholding does not have");
                    } else if (d.kind == DatumKind::Offset && !t->findDatum(d.from)) {
                        issues.push_back(label + ": the WCS origin is on the datum '" + d.from +
                                         "', which the workholding does not have");
                    }
                }
            }
            if (w->rotation != 0 && w->rotation != 90 && w->rotation != 180 && w->rotation != 270) {
                issues.push_back(label + ": the workholding turns by 0, 90, 180 or 270 degrees");
            }
            if (w->gripDepth && *w->gripDepth < 0.0) {
                issues.push_back(label + ": the grip depth must not be negative");
            }
            if (!(w->stopOffset >= 0.0)) {
                issues.push_back(label + ": the stock cannot go in past the jaws' stop: its "
                                         "distance out from the stop must not be negative");
            }
            if (w->jaws == JawSetting::Opening && w->opening < 0.0) {
                issues.push_back(label + ": the jaw opening must not be negative");
            }
        }
        if (const auto& o = s.originFrom) {
            if (std::any_of(o->box.begin(), o->box.end(), [](int k) { return k < 0 || k > 2; })) {
                issues.push_back(label + ": the WCS origin's box point is min, middle or max");
            }
            if (o->from == OriginFrom::Point && !o->point && !o->workholdingPoint) {
                issues.push_back(label + ": pick the point the WCS origin is at");
            }
            if (o->workholdingPoint && !s.workholding) {
                issues.push_back(label + ": the WCS origin is on a workholding the setup does "
                                         "not have");
            }
        }

        const Stock& st = s.stock;
        if (st.kind != StockKind::FromSetup) {
            if (st.axis < 0 || st.axis > 2) {
                issues.push_back(label + ": stock axis must be X, Y or Z");
            }
            if (st.size != StockSize::Fixed && !s.modelId) {
                issues.push_back(label + ": stock sized from the part needs a model");
            }
            if (st.size == StockSize::OffsetFromPart &&
                (st.offsetSide < 0.0 || st.offsetTop < 0.0 || st.offsetBottom < 0.0 ||
                 st.roundTo < 0.0)) {
                issues.push_back(label + ": stock offsets must not be negative");
            }
        }
        switch (st.kind) {
        case StockKind::Box:
            if (st.size == StockSize::Fixed &&
                (st.boxCorners ? !((*st.boxCorners)[1].array() > (*st.boxCorners)[0].array()).all()
                               : !(st.boxSize.array() > 0.0).all())) {
                issues.push_back(label + ": stock box must have positive size");
            }
            break;
        case StockKind::Cylinder:
        case StockKind::Tube:
            if (st.size == StockSize::Fixed &&
                (st.diameter <= 0.0 || st.length <= 0.0 ||
                 (st.cylinderPlacement && (*st.cylinderPlacement)[1].isZero()))) {
                issues.push_back(label + ": stock cylinder needs a positive diameter and length");
            }
            if (st.kind == StockKind::Tube &&
                (st.innerDiameter <= 0.0 ||
                 (st.size == StockSize::Fixed && st.innerDiameter >= st.diameter))) {
                issues.push_back(label + ": stock tube needs an inner diameter between 0 and its "
                                         "outer diameter");
            }
            break;
        case StockKind::FromSetup: {
            // Must refer to an earlier setup; that rules out cycles too.
            const auto earlier = std::find_if(
                setups.begin(), setups.begin() + static_cast<std::ptrdiff_t>(i),
                [&](const Setup& o) { return st.sourceSetup && o.id == *st.sourceSetup; });
            if (earlier == setups.begin() + static_cast<std::ptrdiff_t>(i)) {
                issues.push_back(label + ": stock must come from an earlier setup");
            }
            break;
        }
        }

        for (const auto& op : s.operations) {
            const std::string opLabel = label + ", operation '" + op.name + "'";
            unique(op.id, opLabel);
            if (op.type.empty()) {
                issues.push_back(opLabel + ": has no type");
            }
            const tooling::Tool* tool = op.toolId ? tools.findTool(*op.toolId) : nullptr;
            if (op.toolId && !tool) {
                issues.push_back(opLabel + ": references a missing tool");
            }
            if (op.presetId && (!tool || !tool->findPreset(*op.presetId))) {
                issues.push_back(opLabel + ": references a missing cutting preset");
            }
            for (const auto& ref : op.geometry) {
                if (!findModel(ref.modelId)) {
                    issues.push_back(opLabel + ": geometry references a missing model");
                }
            }
            if (!op.parameters.is_object()) {
                issues.push_back(opLabel + ": parameters must be an object");
            }
            if (op.orientation) {
                try {
                    (void)op.orientation->frame.normalized();
                } catch (const Error&) {
                    issues.push_back(opLabel + ": invalid tool orientation frame");
                }
                if (op.orientation->workOffset < 0) {
                    issues.push_back(opLabel + ": work offset must not be negative");
                }
            }
        }
    }
    return issues;
}

Transform3 operationFrame(const Operation& op) {
    return op.orientation ? op.orientation->frame.toParent() : Transform3::Identity();
}

std::string workOffsetName(int workOffset) {
    if (workOffset >= 1 && workOffset <= 6) {
        return fmt::format("G{}", 53 + workOffset);
    }
    return fmt::format("G54.1 P{}", workOffset - 6);
}

} // namespace ocam::document
