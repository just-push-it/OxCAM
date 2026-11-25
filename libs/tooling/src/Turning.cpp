// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Turning insert outlines in the lathe plane. The insert is placed as on a 93-95 degree
// approach holder: the end (trailing) edge leaves the nose 5 degrees above the axial direction
// (or more for narrow inserts) and the side (leading) edge follows at the insert's included
// angle. See docs/design-log/0020.

#include "ocam/tooling/Profile.hpp"

#include <ocam/foundation/Error.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cctype>
#include <cmath>

namespace ocam::tooling {
namespace {

double includedAngle(const std::string& shape) {
    const InsertShape* known = findInsertShape(shape);
    return degToRad(known && known->includedAngle > 0.0 ? known->includedAngle : 80.0);
}

/// The sides of the outline from the nose, counter-clockwise from the trailing edge: turning
/// angle at each corner after the first and the side lengths, for an inscribed circle `ic`.
struct Sides {
    std::vector<double> turns;
    std::vector<double> lengths;
};

Sides sidesOf(char letter, double alpha, double ic) {
    Sides s;
    const auto regular = [&](int n) {
        const double side = ic * std::tan(kPi / n);
        s.lengths.assign(static_cast<std::size_t>(n), side);
        s.turns.assign(static_cast<std::size_t>(n) - 1, 2.0 * kPi / n);
    };
    switch (letter) {
    case 'T':
        regular(3);
        break;
    case 'P':
        regular(5);
        break;
    case 'H':
        regular(6);
        break;
    case 'O':
        regular(8);
        break;
    case 'W': {
        // Trigon: six equal sides around the circle, corners alternating 80 and 160 degrees.
        const double blunt = degToRad(20.0);
        const double sharp = kPi - alpha;
        const double side = 0.5 * ic * (std::tan(0.5 * sharp) + std::tan(0.5 * blunt));
        s.lengths.assign(6, side);
        s.turns = {blunt, sharp, blunt, sharp, blunt};
        break;
    }
    default:
        // Rhombic, square, and (as rhombi on the circle) parallelogram and rectangular shapes.
        const double side = ic / std::sin(alpha);
        s.lengths.assign(4, side);
        s.turns = {alpha, kPi - alpha, alpha};
    }
    return s;
}

char shapeLetter(const std::string& shape) {
    return shape.empty() ? 'C'
                         : static_cast<char>(std::toupper(static_cast<unsigned char>(shape[0])));
}

geom2d::Polyline mirrored(const geom2d::Polyline& p) {
    std::vector<geom2d::Vertex> v;
    for (const auto& vert : p.vertices()) {
        v.push_back({Vec2(vert.p.x(), -vert.p.y()), -vert.bulge});
    }
    geom2d::Polyline out(std::move(v), true);
    out.reverse(); // back to counter-clockwise
    return out;
}

} // namespace

const std::vector<InsertShape>& insertShapes() {
    static const std::vector<InsertShape> shapes{
        {'C', 80.0, "Rhombic 80°"},
        {'D', 55.0, "Rhombic 55°"},
        {'E', 75.0, "Rhombic 75°"},
        {'F', 50.0, "Rhombic 50°"},
        {'M', 86.0, "Rhombic 86°"},
        {'V', 35.0, "Rhombic 35°"},
        {'S', 90.0, "Square"},
        {'T', 60.0, "Triangular"},
        {'W', 80.0, "Trigon 80°"},
        {'R', 0.0, "Round"},
        {'L', 90.0, "Rectangular"},
        {'A', 85.0, "Parallelogram 85°"},
        {'B', 82.0, "Parallelogram 82°"},
        {'K', 55.0, "Parallelogram 55°"},
        {'P', 108.0, "Pentagonal"},
        {'H', 120.0, "Hexagonal"},
        {'O', 135.0, "Octagonal"},
    };
    return shapes;
}

const InsertShape* findInsertShape(std::string_view shape) {
    if (shape.empty()) {
        return nullptr;
    }
    const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(shape[0])));
    for (const InsertShape& s : insertShapes()) {
        if (s.letter == c) {
            return &s;
        }
    }
    return nullptr;
}

TurningSide turningSideOf(int tipQuadrant) noexcept {
    return tipQuadrant == 2 || tipQuadrant == 1 ? TurningSide::Inside
           : tipQuadrant == 8                   ? TurningSide::Face
                                                : TurningSide::Outside;
}

bool turningAwayFromChuck(int tipQuadrant) noexcept {
    return tipQuadrant == 1 || tipQuadrant == 4;
}

geom2d::Polyline mirroredInZ(const geom2d::Polyline& outline) {
    std::vector<Vec2> points = outline.discretize(1e-3);
    for (Vec2& q : points) {
        q.x() = -q.x();
    }
    std::reverse(points.begin(), points.end());
    return geom2d::Polyline::fromPoints(points, true);
}

std::vector<Vec2> holeToolSide(const Tool& tool) {
    const MillGeometry& g = tool.mill;
    const double r = g.diameter / 2.0;
    // The upper side, from the tip back: the lower side mirrors it.
    std::vector<Vec2> side;
    if (tool.type == ToolType::CenterDrill) {
        constexpr double kCountersinkHalfAngle = 30.0 * kPi / 180.0; // as toolProfile
        const double rp = g.tipDiameter / 2.0;
        const double sinkTop = g.tipLength + (r - rp) / std::tan(kCountersinkHalfAngle);
        side = {{0.0, 0.0},
                {rp / std::tan(g.tipAngle / 2.0), rp},
                {sinkTop, r},
                {std::max(g.fluteLength, sinkTop), r}};
    } else if ((tool.type == ToolType::Drill || tool.type == ToolType::SpotDrill) &&
               g.tipAngle > 0.0 && g.tipAngle < kPi) {
        const double tip = r / std::tan(g.tipAngle / 2.0);
        side = {{0.0, 0.0}, {tip, r}, {std::max(g.fluteLength, tip), r}};
    } else {
        side = {{0.0, r}, {g.fluteLength, r}};
    }
    return side;
}

geom2d::Polyline holeToolOutline(const Tool& tool) {
    const std::vector<Vec2> side = holeToolSide(tool);
    std::vector<Vec2> points;
    for (const Vec2& q : side) {
        if (q.y() > 0.0) {
            points.emplace_back(q.x(), -q.y());
        } else {
            points.push_back(q);
        }
    }
    for (auto it = side.rbegin(); it != side.rend(); ++it) {
        if (it->y() > 0.0) {
            points.push_back(*it);
        }
    }
    return geom2d::Polyline::fromPoints(points, true);
}

TurningShape turningShape(const Tool& tool, TurningSide side) {
    if (!isTurningTool(tool.type)) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("tool T{} is not a turning insert", tool.number));
    }
    const TurningInsert& in = tool.insert;
    TurningShape s;
    s.noseRadius = std::max(0.0, in.noseRadius);
    const double rn = s.noseRadius;
    const double kBulge90 = std::tan(kPi / 8.0);

    if (tool.type == ToolType::GroovingInsert || tool.type == ToolType::PartingInsert) {
        const double w = in.width;
        if (w <= 0.0) {
            throw Error(ErrorCode::InvalidArgument, "grooving and parting inserts need a width");
        }
        const double r = std::min(rn, 0.45 * w);
        const double h = std::max(15.0, 2.0 * w);
        s.width = w;
        s.noseRadius = r;
        s.leadAngle = kPi / 2.0;
        s.trailingAngle = 0.0;
        s.outline = geom2d::Polyline({{Vec2(r, 0.0), 0.0},
                                      {Vec2(w - r, 0.0), r > 0.0 ? kBulge90 : 0.0},
                                      {Vec2(w, r), 0.0},
                                      {Vec2(w, h), 0.0},
                                      {Vec2(0.0, h), 0.0},
                                      {Vec2(0.0, r), r > 0.0 ? kBulge90 : 0.0}},
                                     true);
    } else if (tool.type == ToolType::ThreadingInsert) {
        const double pitch = in.threadPitch > 0.0 ? in.threadPitch : 1.5;
        const double h = 3.0 * pitch;
        const double a = h * std::tan(degToRad(30.0));
        s.leadAngle = degToRad(120.0);
        s.trailingAngle = degToRad(60.0);
        s.outline = geom2d::Polyline::fromPoints({Vec2(0.0, 0.0), Vec2(a, h), Vec2(-a, h)}, true);
    } else if (!in.shape.empty() && std::toupper(static_cast<unsigned char>(in.shape[0])) == 'R') {
        const double r = std::max(rn, in.inscribedCircle / 2.0);
        s.noseRadius = r;
        s.leadAngle = kPi / 2.0;
        s.trailingAngle = kPi / 2.0;
        s.outline = geom2d::Polyline::circle(Vec2(r, r), r);
    } else {
        const double alpha = includedAngle(in.shape);
        double a2 = std::max(degToRad(5.0), degToRad(85.0) - alpha); // trailing edge
        double a1 = a2 + alpha;                                      // leading edge
        if (in.cuttingEdgeAngle > 0.0) {
            a1 = kPi - in.cuttingEdgeAngle;
            a2 = a1 - alpha;
            // One edge may lie ahead of the nose along a feed, not both.
            if (a1 <= 0.0 || (a2 < -1e-9 && a1 > kPi / 2.0 + 1e-9)) {
                throw Error(ErrorCode::InvalidArgument,
                            fmt::format("a cutting edge angle of {:g}° leaves the insert's {:g}° "
                                        "corner no clearance",
                                        radToDeg(in.cuttingEdgeAngle), radToDeg(alpha)));
            }
        }
        const Vec2 e1(std::cos(a1), std::sin(a1));
        const Vec2 e2(std::cos(a2), std::sin(a2));
        const double ic = in.inscribedCircle > 0.0 ? in.inscribedCircle : 12.7;
        const Sides sides = sidesOf(shapeLetter(in.shape), alpha, ic);
        const double half = 0.5 * alpha;
        const Vec2 bis(std::cos(0.5 * (a1 + a2)), std::sin(0.5 * (a1 + a2)));
        const Vec2 centre(rn, rn);
        const Vec2 apex = rn > 0.0 ? Vec2(centre - bis * (rn / std::sin(half))) : Vec2::Zero();
        const double t = rn > 0.0 ? rn / std::tan(half) : 0.0;
        const Vec2 t1 = apex + e1 * t, t2 = apex + e2 * t;
        // Round the insert from the nose along its sides, back to the leading edge's far end.
        std::vector<geom2d::Vertex> v{{t2, 0.0}};
        Vec2 corner = apex;
        double heading = a2;
        for (std::size_t k = 0; k + 1 < sides.lengths.size(); ++k) {
            corner += Vec2(std::cos(heading), std::sin(heading)) * sides.lengths[k];
            v.push_back({corner, 0.0});
            heading += sides.turns[k];
        }
        if (rn > 0.0) {
            v.push_back({t1, std::tan((kPi - alpha) / 4.0)});
        }
        s.leadAngle = a1;
        s.trailingAngle = a2;
        s.outline = geom2d::Polyline(std::move(v), true);
    }
    if (side == TurningSide::Inside) {
        s.outline = mirrored(s.outline);
    } else if (side == TurningSide::Face) {
        if (tool.type != ToolType::GroovingInsert) {
            throw Error(ErrorCode::InvalidArgument, "only a grooving insert cuts in a face");
        }
        // Standing along Z: its width across the radius.
        std::vector<Vec2> points = s.outline.discretize(1e-3);
        for (Vec2& q : points) {
            q = Vec2(q.y(), q.x());
        }
        std::reverse(points.begin(), points.end());
        s.outline = geom2d::Polyline::fromPoints(points, true);
    }
    return s;
}

} // namespace ocam::tooling
