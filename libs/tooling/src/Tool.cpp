// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ocam/tooling/Tool.hpp"

#include "ocam/tooling/TurningHolder.hpp"

#include <ocam/foundation/Error.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace ocam::tooling {
namespace {

struct TypeInfo {
    ToolType type;
    std::string_view id;
    std::string_view display;
};

constexpr std::array<TypeInfo, 21> kTypes{{
    {ToolType::FlatEndMill, "flat_end_mill", "Flat end mill"},
    {ToolType::BallEndMill, "ball_end_mill", "Ball end mill"},
    {ToolType::BullNoseEndMill, "bull_nose_end_mill", "Bull-nose end mill"},
    {ToolType::Chamfer, "chamfer_mill", "Chamfer mill"},
    {ToolType::Engraver, "engraver", "Engraver"},
    {ToolType::Drill, "drill", "Drill"},
    {ToolType::SpotDrill, "spot_drill", "Spot drill"},
    {ToolType::CenterDrill, "center_drill", "Center drill"},
    {ToolType::Reamer, "reamer", "Reamer"},
    {ToolType::BoringBar, "boring_bar", "Boring bar"},
    {ToolType::Tap, "tap", "Tap"},
    {ToolType::ThreadMill, "thread_mill", "Thread mill"},
    {ToolType::SlotCutter, "slot_cutter", "Slot cutter"},
    {ToolType::Lollipop, "lollipop", "Lollipop cutter"},
    {ToolType::Form, "form", "Form tool"},
    {ToolType::Probe, "probe", "Probe"},
    {ToolType::TurningInsert, "turning_insert", "Turning insert"},
    {ToolType::GroovingInsert, "grooving_insert", "Grooving insert"},
    {ToolType::ThreadingInsert, "threading_insert", "Threading insert"},
    {ToolType::PartingInsert, "parting_insert", "Parting insert"},
    {ToolType::BarPuller, "bar_puller", "Bar puller"},
}};

constexpr std::array<std::pair<Coolant, std::string_view>, 5> kCoolants{{
    {Coolant::Off, "off"},
    {Coolant::Flood, "flood"},
    {Coolant::Mist, "mist"},
    {Coolant::Air, "air"},
    {Coolant::ThroughTool, "through_tool"},
}};

bool hasPointAngle(ToolType t) {
    return t == ToolType::Drill || t == ToolType::SpotDrill || t == ToolType::CenterDrill ||
           t == ToolType::Chamfer || t == ToolType::Engraver;
}

} // namespace

std::string_view toString(ToolType type) noexcept {
    for (const auto& t : kTypes) {
        if (t.type == type) {
            return t.id;
        }
    }
    return "unknown";
}

std::optional<ToolType> parseToolType(std::string_view text) noexcept {
    for (const auto& t : kTypes) {
        if (t.id == text) {
            return t.type;
        }
    }
    return std::nullopt;
}

std::string_view displayName(ToolType type) noexcept {
    for (const auto& t : kTypes) {
        if (t.type == type) {
            return t.display;
        }
    }
    return "Unknown";
}

bool isTurningTool(ToolType type) noexcept {
    return type == ToolType::TurningInsert || type == ToolType::GroovingInsert ||
           type == ToolType::ThreadingInsert || type == ToolType::PartingInsert;
}

const std::vector<ToolType>& allToolTypes() {
    static const std::vector<ToolType> types = [] {
        std::vector<ToolType> v;
        for (const auto& t : kTypes) {
            v.push_back(t.type);
        }
        return v;
    }();
    return types;
}

ToolFamily toolFamily(ToolType type) noexcept {
    switch (type) {
    case ToolType::Drill:
    case ToolType::SpotDrill:
    case ToolType::CenterDrill:
    case ToolType::Reamer:
    case ToolType::BoringBar:
    case ToolType::Tap:
        return ToolFamily::HoleMaking;
    case ToolType::Probe:
        return ToolFamily::Probing;
    case ToolType::BarPuller: // a lathe's tool, listed with them
        return ToolFamily::Turning;
    default:
        return isTurningTool(type) ? ToolFamily::Turning : ToolFamily::Milling;
    }
}

std::vector<ToolType> toolTypesOf(ToolFamily family) {
    std::vector<ToolType> out;
    for (ToolType t : allToolTypes()) {
        if (toolFamily(t) == family) {
            out.push_back(t);
        }
    }
    return out;
}

std::string_view toString(HolderKind kind) noexcept {
    return kind == HolderKind::Turning ? "turning" : "milling";
}

std::optional<HolderKind> parseHolderKind(std::string_view text) noexcept {
    if (text == "milling") {
        return HolderKind::Milling;
    }
    if (text == "turning") {
        return HolderKind::Turning;
    }
    return std::nullopt;
}

HolderKind holderKindFor(ToolType type) noexcept {
    return isTurningTool(type) ? HolderKind::Turning : HolderKind::Milling;
}

std::string_view toString(InsertLocking locking) noexcept {
    switch (locking) {
    case InsertLocking::CylindricalHole:
        return "cylindrical_hole";
    case InsertLocking::NoHole:
        return "no_hole";
    case InsertLocking::CountersunkHole:
        return "countersunk_hole";
    case InsertLocking::NotSet:
        break;
    }
    return "not_set";
}

std::optional<InsertLocking> parseInsertLocking(std::string_view text) noexcept {
    for (const InsertLocking l : {InsertLocking::NotSet, InsertLocking::CylindricalHole,
                                  InsertLocking::NoHole, InsertLocking::CountersunkHole}) {
        if (toString(l) == text) {
            return l;
        }
    }
    return std::nullopt;
}

std::string_view toString(Coolant c) noexcept {
    for (const auto& [value, id] : kCoolants) {
        if (value == c) {
            return id;
        }
    }
    return "off";
}

std::optional<Coolant> parseCoolant(std::string_view text) noexcept {
    for (const auto& [value, id] : kCoolants) {
        if (id == text) {
            return value;
        }
    }
    return std::nullopt;
}

double CuttingPreset::spindleSpeed(double toolDiameter) const noexcept {
    return toolDiameter > 0.0 ? surfaceSpeed / (kPi * toolDiameter) : 0.0;
}

double CuttingPreset::feedRate(double toolDiameter, int flutes) const noexcept {
    return feedPerTooth * std::max(flutes, 0) * spindleSpeed(toolDiameter);
}

const CuttingPreset* Tool::findPreset(const Uuid& presetId) const {
    auto it = std::find_if(presets.begin(), presets.end(),
                           [&](const CuttingPreset& p) { return p.id == presetId; });
    return it == presets.end() ? nullptr : &*it;
}

std::vector<std::string> Tool::validate() const {
    std::vector<std::string> issues;
    auto require = [&](bool ok, std::string message) {
        if (!ok) {
            issues.push_back(std::move(message));
        }
    };

    require(number >= 1, fmt::format("tool number must be at least 1 (is {})", number));
    require(stickout >= 0.0, "stickout must not be negative");
    if (solid) {
        require(type == ToolType::TurningInsert || type == ToolType::GroovingInsert ||
                    type == ToolType::ThreadingInsert,
                "only turning, grooving and threading tools can be solid bars");
        require(!holderId, "a solid bar is one piece: it has no holder");
        require(solid->minBore > 0.0, "a solid bar needs its minimum bore");
        require(solid->shankDiameter > 0.0, "a solid bar needs its shank diameter");
        require(solid->headLength > 0.0, "a solid bar needs its head length");
        require(solid->overallLength >= solid->headLength,
                "a solid bar's overall length must not be below its head length");
        require(solid->maxOverhang >= 0.0 && solid->maxDepth >= 0.0,
                "a solid bar's overhang and depth must not be negative");
        require(solid->centreToTip > 0.0 && solid->centreToTip < solid->minBore,
                "a solid bar's centre to tip must be between 0 and its minimum bore");
    }

    for (const auto& p : presets) {
        const std::string label =
            p.name.empty() ? std::string("unnamed preset") : "preset '" + p.name + "'";
        require(p.surfaceSpeed >= 0.0 && p.feedPerTooth >= 0.0 && p.axialDepth >= 0.0 &&
                    p.radialDepth >= 0.0,
                label + ": speeds, feeds and depths must not be negative");
        require(p.plungeFeed >= 0.0 && p.rampFeed >= 0.0 && p.leadFeed >= 0.0,
                label + ": plunge, ramp and lead feeds must not be negative");
        require(p.partOffEndFeed >= 0.0, label + ": the part-off end feed must not be negative");
        require(p.leadFeedPerRev >= 0.0, label + ": the lead feed must not be negative");
        require(p.pullFeed >= 0.0, label + ": the pull feed must not be negative");
        require(p.rampAngle >= 0.0 && p.rampAngle <= kPi / 2.0,
                label + ": ramp angle must be between 0 and 90 degrees");
    }

    if (isTurningTool(type)) {
        const TurningInsert& in = insert;
        const bool widthBased = type == ToolType::GroovingInsert || type == ToolType::PartingInsert;
        if (widthBased) {
            require(in.width > 0.0, "insert width must be positive");
        } else {
            require(in.inscribedCircle > 0.0, "inscribed circle must be positive");
        }
        require(in.noseRadius >= 0.0, "nose radius must not be negative");
        require(in.orientation >= 1 && in.orientation <= 9, "orientation must be 1-9");
        require(in.hand == "L" || in.hand == "R" || in.hand == "N", "hand must be L, R or N");
        if (type == ToolType::TurningInsert) {
            try {
                (void)turningShape(*this);
            } catch (const Error& e) {
                issues.emplace_back(e.what());
            }
        }
        if (type == ToolType::ThreadingInsert) {
            require(in.threadPitch > 0.0, "thread pitch must be positive");
        }
        return issues;
    }

    const MillGeometry& g = mill;
    const double r = g.diameter / 2.0;
    require(g.diameter > 0.0, "diameter must be positive");
    require(g.overallLength > 0.0, "overall length must be positive");
    require(g.fluteLength > 0.0, "flute length must be positive");
    require(g.fluteLength <= g.overallLength, "flute length must not exceed overall length");
    require(g.flutes >= 1, "a tool needs at least one flute");
    require(g.shankDiameter >= 0.0, "shank diameter must not be negative");
    require(stickout <= g.overallLength || stickout == 0.0, "stickout exceeds overall length");
    if (!issues.empty()) {
        return issues; // later checks assume basic sizes are valid
    }

    if (hasPointAngle(type)) {
        require(g.tipAngle > 0.0 && g.tipAngle < kPi,
                "point angle must be between 0 and 180 degrees");
    }

    switch (type) {
    case ToolType::BallEndMill:
        require(g.fluteLength >= r, "flute length of a ball end mill must reach the full diameter");
        break;
    case ToolType::BullNoseEndMill:
        require(g.cornerRadius > 0.0 && g.cornerRadius < r,
                "corner radius must be positive and smaller than the radius");
        require(g.fluteLength >= g.cornerRadius, "flute length must cover the corner radius");
        break;
    case ToolType::Drill:
    case ToolType::SpotDrill:
        if (g.tipAngle > 0.0 && g.tipAngle < kPi) {
            require(g.fluteLength >= r / std::tan(g.tipAngle / 2.0),
                    "flute length must cover the drill point");
        }
        break;
    case ToolType::Chamfer:
    case ToolType::Engraver:
        require(g.tipDiameter >= 0.0 && g.tipDiameter < g.diameter,
                "tip diameter must be smaller than the diameter");
        break;
    case ToolType::CenterDrill:
        require(g.tipDiameter > 0.0 && g.tipDiameter < g.diameter,
                "pilot diameter must be positive and smaller than the body diameter");
        require(g.tipLength > 0.0, "pilot length must be positive");
        break;
    case ToolType::Tap:
    case ToolType::ThreadMill:
        require(g.threadPitch > 0.0, "thread pitch must be positive");
        break;
    case ToolType::SlotCutter:
        require(g.neckDiameter > 0.0 && g.neckDiameter < g.diameter,
                "neck diameter must be positive and smaller than the cutter diameter");
        require(g.neckLength > g.fluteLength, "neck must extend above the cutting width");
        break;
    case ToolType::Lollipop:
        require(g.neckDiameter > 0.0 && g.neckDiameter < g.diameter,
                "neck diameter must be positive and smaller than the ball diameter");
        require(g.neckLength > g.diameter, "neck must extend above the ball");
        break;
    case ToolType::Probe:
        require(g.neckDiameter > 0.0 && g.neckDiameter <= g.diameter,
                "stylus stem diameter must be positive and not larger than the ball");
        require(g.fluteLength >= g.diameter, "the stylus must be longer than its ball");
        require(g.triggerOffset >= 0.0 && g.triggerOffset < r / 2.0,
                "trigger offset must not be negative and less than a quarter of the ball "
                "diameter");
        break;
    case ToolType::BarPuller:
        require(g.tipDiameter > 0.0 && g.tipDiameter < g.diameter,
                "the largest bar a puller grips must be positive and smaller than its body");
        break;
    case ToolType::Form:
        require(g.formProfile.size() >= 2, "form profile needs at least two points");
        if (!g.formProfile.empty()) {
            require(g.formProfile.front().x() == 0.0, "form profile must start on the axis");
            require(std::all_of(g.formProfile.begin(), g.formProfile.end(),
                                [](const Vec2& p) { return p.x() >= 0.0 && p.y() >= 0.0; }),
                    "form profile points must have non-negative radius and height");
        }
        break;
    default:
        break;
    }

    if (g.neckDiameter > 0.0 && type != ToolType::SlotCutter && type != ToolType::Lollipop &&
        type != ToolType::Probe) {
        require(g.neckDiameter <= g.diameter, "neck diameter must not exceed the diameter");
        require(g.neckLength >= g.fluteLength, "neck must start above the flutes");
    }
    if (type == ToolType::BoringBar && g.tipAngle > 0.0) {
        // A back-chamfer cutter's cone runs from its head up to its neck.
        require(g.tipAngle < kPi, "point angle must be below 180 degrees");
        require(g.neckDiameter > 0.0 && g.neckDiameter < g.diameter,
                "a boring bar with a point angle (a back-chamfer cutter) needs a neck narrower "
                "than its head");
        if (g.neckDiameter > 0.0 && g.neckDiameter < g.diameter && g.tipAngle < kPi) {
            require(g.neckLength >= g.fluteLength + (g.diameter - g.neckDiameter) / 2.0 /
                                                        std::tan(g.tipAngle / 2.0),
                    "the neck must start above the back-chamfer cutter's cone");
        }
    }
    return issues;
}

std::vector<std::string> Holder::validate() const {
    std::vector<std::string> issues;
    const auto& v = profile.vertices();
    if (v.size() < 2) {
        issues.emplace_back("holder profile needs at least two points");
        return issues;
    }
    if (profile.isClosed()) {
        issues.emplace_back("holder profile must be an open polyline");
    }
    if (std::any_of(v.begin(), v.end(), [](const geom2d::Vertex& p) { return p.p.x() < 0.0; })) {
        issues.emplace_back("holder profile radius must not be negative");
    }
    if (v.front().p.x() != 0.0 || v.back().p.x() != 0.0) {
        issues.emplace_back("holder profile must start and end on the axis");
    }
    if (length() <= 0.0 || maxRadius() <= 0.0) {
        issues.emplace_back("holder profile needs a positive length and radius");
    }
    if (iso) {
        for (std::string& issue : tooling::validate(*iso)) {
            issues.push_back(std::move(issue));
        }
    }
    return issues;
}

bool holds(const Holder& holder, ToolType type) noexcept {
    return holder.kind == holderKindFor(type) && (!holder.iso || type == ToolType::TurningInsert);
}

namespace {

/// Height of the holder's gauge line above its tool face (height 0).
double holderTop(const Holder& holder) noexcept {
    double top = 0.0;
    for (const auto& v : holder.profile.vertices()) {
        top = std::max(top, v.p.y());
    }
    return top;
}

} // namespace

double visibleLength(const Tool& tool) noexcept {
    return tool.stickout > 0.0 ? tool.stickout : tool.mill.overallLength;
}

double gaugeLength(const Tool& tool, const Holder* holder) noexcept {
    return visibleLength(tool) + (holder ? holderTop(*holder) : 0.0);
}

double stickoutForGauge(double gauge, const Holder* holder) noexcept {
    return gauge - (holder ? holderTop(*holder) : 0.0);
}

double Holder::maxRadius() const {
    double r = 0.0;
    for (const Vec2& p : profile.discretize(0.01)) {
        r = std::max(r, p.x());
    }
    return r;
}

double Holder::length() const {
    const auto& v = profile.vertices();
    if (v.empty()) {
        return 0.0;
    }
    double lo = v.front().p.y();
    double hi = lo;
    for (const auto& p : v) {
        lo = std::min(lo, p.p.y());
        hi = std::max(hi, p.p.y());
    }
    return hi - lo;
}

Holder makeHolder(std::string name, const HolderDimensions& d) {
    std::vector<Vec2> pts{{0.0, 0.0}};
    double h = 0.0;
    auto step = [&](double bottomDiameter, double topDiameter, double length) {
        if (bottomDiameter <= 0.0 || topDiameter <= 0.0 || length <= 0.0) {
            return;
        }
        pts.emplace_back(bottomDiameter / 2.0, h);
        h += length;
        pts.emplace_back(topDiameter / 2.0, h);
    };
    step(d.noseDiameter, d.noseTopDiameter > 0.0 ? d.noseTopDiameter : d.noseDiameter,
         d.noseLength);
    step(d.bodyDiameter, d.bodyDiameter, d.bodyLength);
    step(d.flangeDiameter, d.flangeDiameter, d.flangeLength);
    if (h <= 0.0) {
        throw Error(ErrorCode::InvalidArgument, "a holder needs a step with a diameter and length");
    }
    pts.emplace_back(0.0, h);
    Holder holder;
    holder.name = std::move(name);
    holder.profile = geom2d::Polyline::fromPoints(pts, false);
    return holder;
}

Tool makeFlatEndMill(int number, double diameter, double fluteLength, double overallLength) {
    Tool t;
    t.number = number;
    t.type = ToolType::FlatEndMill;
    t.name = fmt::format("Ø{:g} flat end mill", diameter);
    t.mill.diameter = diameter;
    t.mill.fluteLength = fluteLength;
    t.mill.overallLength = overallLength;
    t.mill.flutes = 3;
    return t;
}

Tool makeBallEndMill(int number, double diameter, double fluteLength, double overallLength) {
    Tool t = makeFlatEndMill(number, diameter, fluteLength, overallLength);
    t.type = ToolType::BallEndMill;
    t.name = fmt::format("Ø{:g} ball end mill", diameter);
    t.mill.flutes = 2;
    return t;
}

Tool makeBullNoseEndMill(int number, double diameter, double cornerRadius, double fluteLength,
                         double overallLength) {
    Tool t = makeFlatEndMill(number, diameter, fluteLength, overallLength);
    t.type = ToolType::BullNoseEndMill;
    t.name = fmt::format("Ø{:g} R{:g} bull-nose end mill", diameter, cornerRadius);
    t.mill.cornerRadius = cornerRadius;
    t.mill.flutes = 4;
    return t;
}

Tool makeDrill(int number, double diameter, double tipAngle, double fluteLength,
               double overallLength) {
    Tool t;
    t.number = number;
    t.type = ToolType::Drill;
    t.name = fmt::format("Ø{:g} drill", diameter);
    t.mill.diameter = diameter;
    t.mill.tipAngle = tipAngle;
    t.mill.fluteLength = fluteLength;
    t.mill.overallLength = overallLength;
    t.mill.flutes = 2;
    return t;
}

Tool makeChamferMill(int number, double diameter, double tipDiameter, double includedAngle,
                     double overallLength) {
    Tool t;
    t.number = number;
    t.type = ToolType::Chamfer;
    t.name = fmt::format("Ø{:g} {:g}° chamfer mill", diameter, radToDeg(includedAngle));
    t.mill.diameter = diameter;
    t.mill.tipDiameter = tipDiameter;
    t.mill.tipAngle = includedAngle;
    t.mill.fluteLength = (diameter - tipDiameter) / 2.0 / std::tan(includedAngle / 2.0);
    t.mill.overallLength = overallLength;
    t.mill.flutes = 4;
    return t;
}

Tool makeDefaultTool(ToolType type, int number, double d) {
    const double oal = std::max(50.0, 6.0 * d);
    Tool t;
    switch (type) {
    case ToolType::BallEndMill:
        t = makeBallEndMill(number, d, 2.0 * d, oal);
        break;
    case ToolType::BullNoseEndMill:
        t = makeBullNoseEndMill(number, d, d / 10.0, 2.0 * d, oal);
        break;
    case ToolType::Drill:
        t = makeDrill(number, d, degToRad(118.0), 5.0 * d, oal + 3.0 * d);
        break;
    case ToolType::SpotDrill:
        t = makeDrill(number, d, degToRad(90.0), d, oal);
        break;
    case ToolType::Chamfer:
    case ToolType::Engraver:
        t = makeChamferMill(number, d, type == ToolType::Engraver ? 0.2 : 0.0,
                            degToRad(type == ToolType::Engraver ? 60.0 : 90.0), oal);
        break;
    case ToolType::CenterDrill:
        t = makeDrill(number, d, degToRad(118.0), d, oal);
        t.mill.tipDiameter = d / 2.5;
        t.mill.tipLength = d / 2.0;
        break;
    case ToolType::SlotCutter:
        t = makeFlatEndMill(number, 3.0 * d, d / 3.0, oal);
        t.mill.neckDiameter = d;
        t.mill.neckLength = 2.0 * d;
        t.mill.shankDiameter = d;
        break;
    case ToolType::Lollipop:
        t = makeFlatEndMill(number, d, d, oal);
        t.mill.neckDiameter = d / 2.0;
        t.mill.neckLength = 3.0 * d;
        break;
    case ToolType::Tap:
    case ToolType::ThreadMill:
        t = makeFlatEndMill(number, d, 2.0 * d, oal);
        t.mill.threadPitch = 1.5;
        break;
    case ToolType::Probe:
        // A stylus of this ball diameter on a probe body (typical proportions; set the probe's
        // own from its maker's data and its calibration).
        t = makeFlatEndMill(number, d, std::max(50.0, 8.0 * d), 0.0);
        t.mill.overallLength = t.mill.fluteLength + 60.0;
        t.mill.flutes = 1;
        t.mill.neckDiameter = d / 2.0;
        t.mill.shankDiameter = std::max(40.0, 2.0 * d);
        break;
    case ToolType::BarPuller:
        // Sized around the bar it grips: a body over it, the jaws as deep as the bar.
        t = makeFlatEndMill(number, d + 20.0, d, std::max(80.0, 3.0 * d));
        t.mill.tipDiameter = d;
        t.mill.shankDiameter = 25.0;
        t.mill.flutes = 1;
        break;
    case ToolType::Form:
        t = makeFlatEndMill(number, d, d, oal);
        t.mill.formProfile = {{0.0, 0.0}, {d / 2.0, d / 4.0}, {d / 2.0, d}};
        break;
    case ToolType::TurningInsert:
    case ToolType::GroovingInsert:
    case ToolType::ThreadingInsert:
    case ToolType::PartingInsert:
        t.number = number;
        t.insert.inscribedCircle = 12.7;
        t.insert.noseRadius = 0.8;
        t.insert.thickness = 4.76;
        t.insert.reliefAngle = type == ToolType::TurningInsert ? 0.0 : degToRad(7.0);
        t.insert.width = 3.0;
        t.insert.threadPitch = 1.5;
        break;
    default:
        t = makeFlatEndMill(number, d, 2.0 * d, oal);
        break;
    }
    t.type = type;
    t.name =
        fmt::format("{} {}", displayName(type),
                    isTurningTool(type) ? std::string() : fmt::format("Ø{:g}", t.mill.diameter));
    while (!t.name.empty() && t.name.back() == ' ') {
        t.name.pop_back();
    }
    if (type != ToolType::Probe) {
        t.presets.push_back(
            defaultPreset(t)); // a probe does not cut: its feeds are its operations'
    }
    return t;
}

CuttingPreset defaultPreset(const Tool& tool) {
    constexpr double kPerMin = 1000.0 / 60.0; // m/min -> mm/s
    CuttingPreset p;
    p.name = "Default";
    p.materialGroup = "general";
    const double d = tool.mill.diameter;
    double plungeShare = 0.5; // of the cutting feed
    auto vc = [&](double metresPerMinute) {
        p.surfaceSpeed = metresPerMinute * kPerMin;
    };
    switch (tool.type) {
    case ToolType::Drill:
        vc(70.0);
        p.feedPerTooth = std::clamp(0.01 * d, 0.02, 0.2);
        plungeShare = 1.0;
        break;
    case ToolType::SpotDrill:
    case ToolType::CenterDrill:
        vc(40.0);
        p.feedPerTooth = 0.03;
        plungeShare = 1.0;
        break;
    case ToolType::Chamfer:
    case ToolType::Engraver:
        vc(100.0);
        p.feedPerTooth = 0.02;
        break;
    case ToolType::Tap:
        vc(10.0); // the feed is the pitch
        plungeShare = 1.0;
        break;
    case ToolType::Reamer:
        vc(20.0);
        p.feedPerTooth = 0.05;
        plungeShare = 1.0;
        break;
    case ToolType::BoringBar:
        vc(100.0);
        p.feedPerTooth = 0.05;
        plungeShare = 1.0;
        break;
    case ToolType::TurningInsert:
        vc(200.0);
        p.feedPerTooth = 0.2;   // feed per revolution
        p.leadFeedPerRev = 0.2; // on and off the cut
        p.axialDepth = 2.0;     // depth of cut
        if (tool.solid) {       // a micro tool: light cuts, as far as it is rated
            vc(80.0);
            p.feedPerTooth = 0.03;
            p.leadFeedPerRev = 0.03;
            p.axialDepth = tool.solid->maxDepth > 0.0 ? tool.solid->maxDepth : 0.5;
        }
        break;
    case ToolType::GroovingInsert:
        vc(150.0);
        p.feedPerTooth = 0.05;
        p.partOffEndFeed = 0.025;
        break;
    case ToolType::ThreadingInsert:
        vc(120.0); // the feed is the pitch
        break;
    case ToolType::BarPuller:
        // No spindle and no cut: onto the bar at the plunge feed, the pull at its own. A start
        // to check against the machine, like every default.
        p.plungeFeed = 500.0 / 60.0;
        p.pullFeed = 1000.0 / 60.0;
        p.coolant = Coolant::Off;
        break;
    case ToolType::PartingInsert:
        vc(120.0);
        p.feedPerTooth = 0.05;
        p.partOffEndFeed = 0.025;
        break;
    default: // end mills and other milling cutters
        vc(150.0);
        p.feedPerTooth = std::clamp(0.008 * d, 0.01, 0.12);
        p.axialDepth = 0.5 * d;
        p.radialDepth = 0.4 * d;
        break;
    }
    if (!isTurningTool(tool.type) && tool.type != ToolType::BarPuller) {
        // Whole numbers of mm/min: exact values in files, and easy ones to read.
        const double vf = p.feedRate(d, tool.mill.flutes);
        p.plungeFeed = std::round(plungeShare * vf * 60.0) / 60.0;
        p.rampFeed = std::round(vf * 60.0) / 60.0;
        p.leadFeed = p.rampFeed;
    }
    return p;
}

} // namespace ocam::tooling
