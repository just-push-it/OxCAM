// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// ISO turning holders. See docs/design-log/0042.

#include "ocam/tooling/TurningHolder.hpp"

#include <ocam/geom2d/Clip.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>

namespace ocam::tooling {

const std::vector<IsoLetter>& holderClampings() {
    static const std::vector<IsoLetter> letters{
        {'P', 0.0, "Hole clamping (lever)"},
        {'C', 0.0, "Top clamping"},
        {'M', 0.0, "Top and hole clamping"},
        {'S', 0.0, "Screw through the hole"},
    };
    return letters;
}

const std::vector<IsoLetter>& holderStyles() {
    static const std::vector<IsoLetter> letters{
        {'A', 90.0, "straight shank, side cutting"},
        {'B', 75.0, "straight shank, side cutting"},
        {'C', 90.0, "straight shank, end cutting", true},
        {'D', 45.0, "straight shank, side cutting"},
        {'E', 60.0, "straight shank, side cutting"},
        {'F', 90.0, "offset shank, end cutting", true},
        {'G', 90.0, "offset shank, side cutting"},
        {'H', 107.5, "offset shank, side cutting"},
        {'J', 93.0, "offset shank, side cutting"},
        {'K', 75.0, "offset shank, end cutting", true},
        {'L', 95.0, "offset shank, side and end cutting"},
        {'M', 50.0, "straight shank, side cutting"},
        {'N', 63.0, "straight shank, side cutting"},
        {'P', 117.5, "offset shank, side cutting"},
        {'R', 75.0, "offset shank, side cutting"},
        {'S', 45.0, "offset shank, side cutting"},
        {'T', 60.0, "offset shank, side cutting"},
        {'U', 93.0, "offset shank, end cutting", true},
        {'V', 72.5, "straight shank, side cutting"},
        {'W', 60.0, "offset shank, end cutting", true},
        {'Y', 85.0, "offset shank, end cutting", true},
    };
    return letters;
}

const std::vector<IsoLetter>& insertClearances() {
    static const std::vector<IsoLetter> letters{
        {'N', 0.0, ""},  {'A', 3.0, ""},  {'B', 5.0, ""},  {'C', 7.0, ""},  {'P', 11.0, ""},
        {'D', 15.0, ""}, {'E', 20.0, ""}, {'F', 25.0, ""}, {'G', 30.0, ""},
    };
    return letters;
}

const std::vector<IsoLetter>& holderLengths() {
    static const std::vector<IsoLetter> letters{
        {'A', 32.0, ""},  {'B', 40.0, ""},  {'C', 50.0, ""},  {'D', 60.0, ""},  {'E', 70.0, ""},
        {'F', 80.0, ""},  {'G', 90.0, ""},  {'H', 100.0, ""}, {'J', 110.0, ""}, {'K', 125.0, ""},
        {'L', 140.0, ""}, {'M', 150.0, ""}, {'N', 160.0, ""}, {'P', 170.0, ""}, {'Q', 180.0, ""},
        {'R', 200.0, ""}, {'S', 250.0, ""}, {'T', 300.0, ""}, {'U', 350.0, ""}, {'V', 400.0, ""},
        {'W', 450.0, ""}, {'Y', 500.0, ""},
    };
    return letters;
}

const std::vector<IsoLetter>& boringBarTypes() {
    static const std::vector<IsoLetter> letters{
        {'S', 0.0, "Steel"},
        {'A', 0.0, "Steel, with coolant"},
        {'C', 0.0, "Carbide"},
        {'E', 0.0, "Carbide, with coolant"},
    };
    return letters;
}

const IsoLetter* findIsoLetter(const std::vector<IsoLetter>& letters, char letter) {
    const auto it = std::find_if(letters.begin(), letters.end(),
                                 [&](const IsoLetter& l) { return l.letter == letter; });
    return it == letters.end() ? nullptr : &*it;
}

std::string designation(const TurningHolderCode& c) {
    const std::string letters =
        fmt::format("{}{}{}{}{}", c.clamping, c.insertShape, c.style, c.clearance, c.hand);
    if (c.boringBar) {
        return fmt::format("{}{:02.0f}{}-{} {:02d}", c.barType, c.height, c.length, letters,
                           c.insertSize);
    }
    return fmt::format("{} {:02.0f}{:02.0f}{} {:02d}", letters, c.height, c.width, c.length,
                       c.insertSize);
}

std::optional<TurningHolderCode> parseDesignation(std::string_view text,
                                                  const TurningHolderCode* custom) {
    std::string s;
    for (const char c : text) {
        if (std::isspace(static_cast<unsigned char>(c)) == 0 && c != '-') {
            s += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        }
    }
    std::size_t at = 0;
    auto digit = [&](std::size_t k) {
        return k < s.size() && std::isdigit(static_cast<unsigned char>(s[k])) != 0;
    };
    auto number = [&](std::size_t maxDigits) {
        int value = 0;
        std::size_t n = 0;
        while (n < maxDigits && digit(at)) {
            value = 10 * value + (s[at++] - '0');
            ++n;
        }
        return n > 0 ? std::optional<int>(value) : std::nullopt;
    };
    auto letter = [&]() -> char {
        return at < s.size() ? s[at++] : '\0';
    };
    TurningHolderCode c;
    c.boringBar = digit(1);
    if (c.boringBar) {
        c.barType = letter();
        const auto diameter = number(3);
        if (!diameter) {
            return std::nullopt;
        }
        c.height = c.width = *diameter;
        c.length = letter();
    }
    c.clamping = letter();
    c.insertShape = letter();
    c.style = letter();
    c.clearance = letter();
    c.hand = letter();
    if (!c.boringBar) {
        const auto height = number(2);
        const auto width = number(2);
        if (!height || !width) {
            return std::nullopt;
        }
        c.height = *height;
        c.width = *width;
        c.length = letter();
    }
    const auto size = number(2);
    if (!size) {
        return std::nullopt;
    }
    c.insertSize = *size;
    if (custom && c.style == kCustomStyle) {
        c.customStyle = custom->customStyle;
    }
    if (custom && c.clearance == kCustomClearance) {
        c.customClearance = custom->customClearance;
    }
    if (custom && c.length == kCustomLength) {
        c.customLength = custom->customLength;
    }
    if ((c.boringBar && !findIsoLetter(boringBarTypes(), c.barType)) ||
        !findIsoLetter(holderClampings(), c.clamping) || !findInsertShape({&c.insertShape, 1}) ||
        !holderStyleAngle(c) || !holderClearanceAngle(c) ||
        !(c.hand == 'R' || c.hand == 'L' || c.hand == 'N') || holderLength(c) <= 0.0) {
        return std::nullopt;
    }
    c.f = standardF(c);
    return c;
}

double standardF(const TurningHolderCode& c) {
    if (c.boringBar) {
        static constexpr std::array<std::array<double, 2>, 10> kBars{{{10, 7},
                                                                      {12, 9},
                                                                      {16, 11},
                                                                      {20, 13},
                                                                      {25, 17},
                                                                      {32, 22},
                                                                      {40, 27},
                                                                      {50, 35},
                                                                      {60, 43},
                                                                      {80, 53}}};
        for (const auto& [d, f] : kBars) {
            if (d == c.height) {
                return f;
            }
        }
        return std::round(0.68 * c.height);
    }
    if (c.hand == 'N') {
        return 0.5 * c.width;
    }
    static constexpr std::array<std::array<double, 2>, 8> kShanks{
        {{10, 12}, {12, 16}, {16, 20}, {20, 25}, {25, 32}, {32, 40}, {40, 50}, {50, 63}}};
    for (const auto& [b, f] : kShanks) {
        if (b == c.width) {
            return f;
        }
    }
    return std::round(1.25 * c.width);
}

double holderLength(const TurningHolderCode& c) {
    if (c.length == kCustomLength) {
        return c.customLength.value_or(0.0);
    }
    const IsoLetter* l = findIsoLetter(holderLengths(), c.length);
    return l ? l->value : 0.0;
}

std::optional<double> holderStyleAngle(const TurningHolderCode& c) {
    if (c.style == kCustomStyle) {
        return c.customStyle;
    }
    const IsoLetter* l = findIsoLetter(holderStyles(), c.style);
    return l ? std::optional(l->value) : std::nullopt;
}

std::optional<double> holderClearanceAngle(const TurningHolderCode& c) {
    if (c.clearance == kCustomClearance) {
        return c.customClearance;
    }
    const IsoLetter* l = findIsoLetter(insertClearances(), c.clearance);
    return l ? std::optional(l->value) : std::nullopt;
}

double inscribedCircle(const TurningHolderCode& c) {
    const InsertShape* shape = findInsertShape({&c.insertShape, 1});
    const double size = c.insertSize;
    if (shape && shape->includedAngle <= 0.0) {
        return size; // round: the size is the diameter
    }
    // The size symbol is a side's length in whole millimetres, cut off.
    const double side = size + 0.5;
    double ic = side;
    switch (c.insertShape) {
    case 'T':
        ic = side / std::tan(kPi / 3.0);
        break;
    case 'P':
        ic = side / std::tan(kPi / 5.0);
        break;
    case 'H':
        ic = side / std::tan(kPi / 6.0);
        break;
    case 'O':
        ic = side / std::tan(kPi / 8.0);
        break;
    case 'W':
        ic = side / (0.5 * (std::tan(degToRad(50.0)) + std::tan(degToRad(10.0))));
        break;
    default:
        if (shape) {
            ic = side * std::sin(degToRad(shape->includedAngle));
        }
    }
    static constexpr std::array<double, 11> kSeries{3.97, 4.76,   5.56,  6.35, 7.94, 9.525,
                                                    12.7, 15.875, 19.05, 25.4, 31.75};
    double best = kSeries.front();
    for (const double v : kSeries) {
        if (std::abs(v - ic) < std::abs(best - ic)) {
            best = v;
        }
    }
    return best;
}

double sideCuttingEdgeAngle(const TurningHolderCode& c) {
    const std::optional<double> angle = holderStyleAngle(c);
    const IsoLetter* style = findIsoLetter(holderStyles(), c.style);
    const InsertShape* shape = findInsertShape({&c.insertShape, 1});
    if (!angle || !shape || shape->includedAngle <= 0.0) {
        return 0.0;
    }
    // A custom style states its angle to the feed towards -Z, whatever way the holder cuts.
    return degToRad(style && c.style != kCustomStyle && style->endCutting
                        ? 270.0 - *angle - shape->includedAngle
                        : *angle);
}

double smallestBore(const TurningHolderCode& c) {
    return c.boringBar ? c.f + 0.5 * c.height : 0.0;
}

std::vector<std::string> validate(const TurningHolderCode& c) {
    std::vector<std::string> issues;
    const std::optional<double> style = holderStyleAngle(c);
    const InsertShape* shape = findInsertShape({&c.insertShape, 1});
    if (c.style == kCustomStyle) {
        if (!style || *style <= 0.0 || *style >= 180.0) {
            issues.emplace_back("a custom style's cutting edge angle must be between 0 and 180°");
        }
    } else if (!style) {
        issues.push_back(fmt::format("'{}' is not a holder style", c.style));
    }
    if (!shape) {
        issues.push_back(fmt::format("'{}' is not an insert shape", c.insertShape));
    }
    if (!findIsoLetter(holderClampings(), c.clamping)) {
        issues.push_back(fmt::format("'{}' is not a clamping", c.clamping));
    }
    if (c.clearance == kCustomClearance) {
        if (!c.customClearance || *c.customClearance < 0.0 || *c.customClearance >= 90.0) {
            issues.emplace_back("a custom insert clearance must be from 0 to below 90°");
        }
    } else if (!holderClearanceAngle(c)) {
        issues.push_back(fmt::format("'{}' is not an insert clearance", c.clearance));
    }
    if (c.hand != 'R' && c.hand != 'L' && c.hand != 'N') {
        issues.push_back("the hand must be R, L or N");
    }
    if (c.length == kCustomLength) {
        if (holderLength(c) <= 0.0) {
            issues.emplace_back("a custom length must be positive");
        }
    } else if (holderLength(c) <= 0.0) {
        issues.push_back(fmt::format("'{}' is not a tool length", c.length));
    }
    if (c.boringBar && !findIsoLetter(boringBarTypes(), c.barType)) {
        issues.push_back(fmt::format("'{}' is not a boring bar type", c.barType));
    }
    if (c.height <= 0.0 || c.width <= 0.0) {
        issues.emplace_back(c.boringBar ? "the bar's diameter must be positive"
                                        : "the shank's height and width must be positive");
    }
    if (c.insertSize <= 0) {
        issues.emplace_back("the insert size must be positive");
    }
    if (c.f <= 0.0) {
        issues.emplace_back("dimension f must be positive");
    } else if (c.boringBar && c.f <= 0.5 * c.height) {
        issues.emplace_back("dimension f must put the tip outside the bar: more than its radius");
    }
    if (style && shape && shape->includedAngle > 0.0) {
        // Side cutting: the end edge must stay off the turned surface. End cutting: the side edge
        // must stay off the faced surface.
        const double other = 180.0 - *style - shape->includedAngle;
        if (other <= 1e-9) {
            issues.push_back(fmt::format(
                "a {:g}° corner ({}) leaves no clearance in style {} ({:g}° cutting edge)",
                shape->includedAngle, shape->letter, c.style, *style));
        }
    }
    return issues;
}

Holder makeTurningHolder(const TurningHolderCode& code) {
    Holder h;
    h.kind = HolderKind::Turning;
    h.name = designation(code);
    setTurningHolderCode(h, code);
    return h;
}

void setTurningHolderCode(Holder& holder, const TurningHolderCode& code) {
    holder.kind = HolderKind::Turning;
    holder.iso = code;
    const double r = 0.5 * (code.boringBar ? code.height : code.width);
    const double l = holderLength(code);
    holder.profile = geom2d::Polyline::fromPoints(
        {Vec2(0.0, 0.0), Vec2(r, 0.0), Vec2(r, l), Vec2(0.0, l)}, false);
}

Holder makeTurningStick(const TurningStick& stick) {
    Holder h;
    h.kind = HolderKind::Turning;
    h.name = stick.bar ? "Turning bar" : "Turning shank";
    setTurningStick(h, stick);
    return h;
}

void setTurningStick(Holder& holder, const TurningStick& stick) {
    holder.kind = HolderKind::Turning;
    holder.stick = stick;
    const double r = 0.5 * stick.width;
    holder.profile = geom2d::Polyline::fromPoints(
        {Vec2(0.0, 0.0), Vec2(r, 0.0), Vec2(r, stick.length), Vec2(0.0, stick.length)}, false);
}

TurningSide holderSide(const TurningHolderCode& code) {
    return code.boringBar ? TurningSide::Inside : TurningSide::Outside;
}

bool clampingTakes(char clamping, InsertLocking locking) {
    switch (locking) {
    case InsertLocking::CylindricalHole:
        return clamping == 'P' || clamping == 'M';
    case InsertLocking::NoHole:
        return clamping == 'C';
    case InsertLocking::CountersunkHole:
        return clamping == 'S';
    case InsertLocking::NotSet:
        break;
    }
    return true;
}

std::vector<HolderMismatch> holderMismatches(const Tool& tool, const Holder& holder) {
    if (!holds(holder, tool.type)) {
        return {HolderMismatch::Kind};
    }
    std::vector<HolderMismatch> out;
    if (!holder.iso || tool.type != ToolType::TurningInsert || tool.solid) {
        return out;
    }
    const TurningHolderCode& c = *holder.iso;
    const TurningInsert& in = tool.insert;
    if (in.shape.empty() || std::toupper(static_cast<unsigned char>(in.shape.front())) !=
                                static_cast<unsigned char>(c.insertShape)) {
        out.push_back(HolderMismatch::Shape);
    }
    if (std::abs(in.inscribedCircle - inscribedCircle(c)) > 0.01) {
        out.push_back(HolderMismatch::Size);
    }
    if (const std::optional<double> clearance = holderClearanceAngle(c);
        clearance && std::abs(radToDeg(in.reliefAngle) - *clearance) > 0.01) {
        out.push_back(HolderMismatch::Clearance);
    }
    if (!clampingTakes(c.clamping, in.locking)) {
        out.push_back(HolderMismatch::Locking);
    }
    return out;
}

void fitToHolder(Tool& tool, const Holder& holder) {
    if (!holder.iso || tool.type != ToolType::TurningInsert) {
        return;
    }
    const TurningHolderCode& c = *holder.iso;
    TurningInsert& in = tool.insert;
    in.shape = std::string(1, c.insertShape);
    in.inscribedCircle = inscribedCircle(c);
    in.hand = std::string(1, c.hand);
    if (const std::optional<double> clearance = holderClearanceAngle(c)) {
        in.reliefAngle = degToRad(*clearance);
    }
    in.cuttingEdgeAngle = sideCuttingEdgeAngle(c);
    // Tip quadrants of nose-radius compensation.
    in.orientation = c.boringBar ? (c.hand == 'L' ? 1 : 2) : (c.hand == 'L' ? 4 : 3);
}

namespace {

std::vector<Vec2> hullOf(std::vector<Vec2> pts) {
    std::sort(pts.begin(), pts.end(), [](const Vec2& a, const Vec2& b) {
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    auto cross = [](const Vec2& o, const Vec2& a, const Vec2& b) {
        return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    };
    std::vector<Vec2> hull(2 * pts.size());
    std::size_t k = 0;
    for (const Vec2& p : pts) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], p) <= 0.0) {
            --k;
        }
        hull[k++] = p;
    }
    for (std::size_t i = pts.size() - 1, lower = k + 1; i-- > 0;) {
        while (k >= lower && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0.0) {
            --k;
        }
        hull[k++] = pts[i];
    }
    hull.resize(k - 1);
    return hull;
}

} // namespace

TurningHolderOutline turningHolderOutline(const TurningHolderCode& c,
                                          const geom2d::Polyline& insert) {
    const std::vector<Vec2> outline = insert.discretize(0.01);
    double far = 0.0;
    for (const Vec2& p : outline) {
        far = std::max(far, c.boringBar ? p.x() : p.y());
    }
    const double breadth = c.boringBar ? c.height : c.width;
    const double start = std::max(far + 0.1 * breadth, 1.2 * breadth);
    const double end = std::max(holderLength(c), start + 1.0);
    std::vector<Vec2> head = outline;
    Vec2 lo, hi;
    if (c.boringBar) {
        const double r = 0.5 * c.height;
        lo = Vec2(start, -c.f - r);
        hi = Vec2(end, -c.f + r);
        head.emplace_back(start, -c.f - r);
        head.emplace_back(start, -c.f + r);
    } else {
        lo = Vec2(c.f - c.width, start);
        hi = Vec2(c.f, end);
        head.emplace_back(c.f - c.width, start);
        head.emplace_back(c.f, start);
    }
    head = hullOf(std::move(head));
    std::vector<Vec2> shank{lo, Vec2(hi.x(), lo.y()), hi, Vec2(lo.x(), hi.y())};
    if (c.hand == 'L') {
        for (std::vector<Vec2>* pts : {&head, &shank}) {
            for (Vec2& p : *pts) {
                p.x() = -p.x();
            }
            std::reverse(pts->begin(), pts->end());
        }
    }
    return {geom2d::Polyline::fromPoints(head, true), geom2d::Polyline::fromPoints(shank, true)};
}

Tool makeSolidBoringBar(int number) {
    Tool t = makeDefaultTool(ToolType::TurningInsert, number);
    t.name = "Solid boring bar";
    t.solid = SolidBar{6.2, 6.0, 25.0, 53.0, 30.5, 2.95, 0.5};
    t.insert.noseRadius = 0.2;
    fitSolidBar(t);
    t.presets = {defaultPreset(t)};
    return t;
}

void fitSolidBar(Tool& tool) {
    if (!tool.solid) {
        return;
    }
    tool.holderId.reset();
    TurningInsert& in = tool.insert;
    in.shape = "D";
    in.inscribedCircle = tool.solid->centreToTip;
    in.thickness = tool.solid->shankDiameter;
    if (in.hand != "L") {
        in.hand = "R";
    }
    in.orientation = in.hand == "L" ? 1 : 2;
}

std::vector<geom2d::Polyline> turningBody(const Tool& tool, const Holder* holder, TurningSide side,
                                          bool awayFromChuck) {
    if (tool.solid) {
        // The head stands from just under the tip across the smallest bore it fits; the shank's
        // axis is the centre to tip below the tip. Both start behind the tip's cutting part.
        const SolidBar& s = *tool.solid;
        double far = 0.0;
        for (const Vec2& p : turningShape(tool, side).outline.discretize(0.01)) {
            far = std::max(far, std::abs(p.x()));
        }
        constexpr double kUnderTip = 0.02;
        const double start = std::min(far, 0.5 * s.headLength);
        const double sign = tool.insert.hand == "L" ? -1.0 : 1.0;
        const auto box = [&](double x0, double x1, double y0, double y1) {
            return geom2d::Polyline::rectangle(Vec2(std::min(sign * x0, sign * x1), y0),
                                               Vec2(std::max(sign * x0, sign * x1), y1));
        };
        std::vector<geom2d::Polyline> body{
            box(start, s.headLength, -(s.minBore - kUnderTip), -kUnderTip)};
        if (s.overallLength > s.headLength) {
            body.push_back(box(s.headLength, s.overallLength,
                               -s.centreToTip - 0.5 * s.shankDiameter,
                               -s.centreToTip + 0.5 * s.shankDiameter));
        }
        return body;
    }
    if (holder && holder->stick) {
        const TurningStick& s = *holder->stick;
        double reachX = 0.0;
        double reachY = 0.0;
        for (const Vec2& p : turningShape(tool, side).outline.discretize(0.01)) {
            reachX = std::max(reachX, p.x());
            reachY = std::max(reachY, std::abs(p.y()));
        }
        // Past what the tool takes up, up to its length.
        geom2d::Polyline piece;
        if (side == TurningSide::Inside) {
            piece = geom2d::Polyline::rectangle(
                Vec2(reachX, -s.f - 0.5 * s.width),
                Vec2(std::max(s.length, reachX + 1.0), -s.f + 0.5 * s.width));
        } else if (side == TurningSide::Face) {
            piece = geom2d::Polyline::rectangle(Vec2(reachX, s.f - s.width),
                                                Vec2(std::max(s.length, reachX + 1.0), s.f));
        } else {
            piece = geom2d::Polyline::rectangle(Vec2(s.f - s.width, reachY),
                                                Vec2(s.f, std::max(s.length, reachY + 1.0)));
        }
        return {awayFromChuck ? mirroredInZ(piece) : piece};
    }
    if (!holder || !holder->iso) {
        return {};
    }
    const TurningHolderCode& c = *holder->iso;
    const geom2d::Polyline insert = turningShape(tool, side).outline;
    const TurningHolderOutline outline = turningHolderOutline(c, insert);
    // What is beside the insert is the insert's own relief: only what is beyond it can meet
    // material the insert has not cut.
    double far = 0.0;
    for (const Vec2& p : insert.discretize(0.01)) {
        far = std::max(far, c.boringBar ? std::abs(p.x()) : p.y());
    }
    constexpr double kFar = 1e4;
    const Vec2 lo =
        c.boringBar ? (c.hand == 'L' ? Vec2(-kFar, -kFar) : Vec2(far, -kFar)) : Vec2(-kFar, far);
    const Vec2 hi =
        c.boringBar ? (c.hand == 'L' ? Vec2(-far, kFar) : Vec2(kFar, kFar)) : Vec2(kFar, kFar);
    std::vector<geom2d::Polyline> body{outline.shank};
    for (const auto& r : geom2d::intersect({{outline.head, {}}},
                                           {{geom2d::Polyline::rectangle(lo, hi), {}}}, {1e-3})) {
        body.push_back(r.outer);
    }
    if (awayFromChuck && c.hand != 'L') {
        for (geom2d::Polyline& piece : body) {
            piece = mirroredInZ(piece);
        }
    }
    return body;
}

std::vector<TurningToolPart> turningToolParts(const Tool& tool, const Holder* holder,
                                              TurningSide side, bool awayFromChuck) {
    std::vector<TurningToolPart> parts;
    if (tool.solid) {
        const auto body = turningBody(tool, nullptr, side, awayFromChuck);
        const double half = 0.5 * std::min(tool.solid->shankDiameter, tool.solid->minBore);
        for (std::size_t k = 0; k < body.size(); ++k) {
            parts.push_back({body[k], -half, half, k > 0}); // the head, then the round shank
        }
        return parts;
    }
    if (!holder) {
        return parts;
    }
    if (holder->stick) {
        const TurningStick& s = *holder->stick;
        for (const geom2d::Polyline& piece : turningBody(tool, holder, side, awayFromChuck)) {
            parts.push_back(
                {piece, s.bar ? -0.5 * s.width : -s.width, s.bar ? 0.5 * s.width : 0.0, s.bar});
        }
        return parts;
    }
    if (holder->iso) {
        const TurningHolderCode& c = *holder->iso;
        const TurningHolderOutline outline =
            turningHolderOutline(c, turningShape(tool, side).outline);
        const bool mirror = awayFromChuck && c.hand != 'L'; // a left-hand one already is
        const double low = c.boringBar ? -0.5 * c.height : -c.height;
        const double high = c.boringBar ? 0.5 * c.height : 0.0;
        parts.push_back({mirror ? mirroredInZ(outline.head) : outline.head, low, high, false});
        parts.push_back(
            {mirror ? mirroredInZ(outline.shank) : outline.shank, low, high, c.boringBar});
        return parts;
    }
    if (holder->kind != HolderKind::Turning) {
        return parts;
    }
    // An outline of its own: away from the insert, on its middle, along the way a holder runs on
    // this side.
    const bool inside = side == TurningSide::Inside;
    geom2d::Aabb2 cutter;
    for (const Vec2& p : turningShape(tool, side).outline.discretize(0.01)) {
        cutter.extend(p);
    }
    const double base = inside ? cutter.max.x() : cutter.max.y();
    const double middle =
        inside ? 0.5 * (cutter.min.y() + cutter.max.y()) : 0.5 * (cutter.min.x() + cutter.max.x());
    const std::vector<Vec2> profile = holder->profile.discretize(0.01);
    for (std::size_t k = 0; k + 1 < profile.size(); ++k) {
        const Vec2& a = profile[k];
        const Vec2& b = profile[k + 1];
        const double half = std::max(a.x(), b.x());
        if (b.y() - a.y() <= 1e-9 || half <= 0.0) {
            continue;
        }
        const auto at = [&](double radius, double height) {
            return inside ? Vec2(base + height, middle + radius)
                          : Vec2(middle + radius, base + height);
        };
        const geom2d::Polyline band = geom2d::Polyline::fromPoints(
            {at(-a.x(), a.y()), at(a.x(), a.y()), at(b.x(), b.y()), at(-b.x(), b.y())}, true);
        parts.push_back({awayFromChuck ? mirroredInZ(band) : band, -half, half, false});
    }
    return parts;
}

} // namespace ocam::tooling
