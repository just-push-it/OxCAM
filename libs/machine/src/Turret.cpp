// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// See docs/design-log/0067.

#include "ocam/machine/Turret.hpp"

#include <ocam/foundation/Units.hpp>

#include <Eigen/Dense>
#include <cmath>

namespace ocam::machine {
namespace {

Component cylinder(std::string name, const Vec3& base, const Vec3& axis, double diameter,
                   double length) {
    Component c;
    c.name = std::move(name);
    c.shape = Component::Shape::Cylinder;
    c.base = base;
    c.axis = axis;
    c.radius = diameter / 2.0;
    c.length = length;
    return c;
}

Component box(std::string name, const Vec3& min, const Vec3& max) {
    Component c;
    c.name = std::move(name);
    c.min = min;
    c.max = max;
    return c;
}

constexpr std::uint32_t kCastColour = 0x8E959C;
constexpr std::uint32_t kTurretColour = 0x6F8FAF;
constexpr std::uint32_t kChuckColour = 0x5B6066;
constexpr std::uint32_t kEnclosureColour = 0xC8CDD2;

} // namespace

std::vector<LatheSolid> latheSolids(const Machine& machine) {
    std::vector<LatheSolid> out;
    if (!machine.lathe) {
        return out;
    }
    const LatheLayout& l = *machine.lathe;
    const Vec3 z = Vec3::UnitZ();
    if (l.headstockDiameter > 0.0 && l.headstockLength > 0.0) {
        out.push_back({cylinder("Headstock", -l.headstockLength * z, z, l.headstockDiameter,
                                l.headstockLength),
                       "", true, kCastColour});
    }
    if (l.chuckDiameter > 0.0 && l.chuckLength > 0.0) {
        out.push_back({cylinder(std::string(kLatheChuckPart), l.workholdingMount * z, z,
                                l.chuckDiameter, l.chuckLength),
                       "C", true, kChuckColour});
    }
    const LatheTurret& t = l.turret;
    if (t.diameter > 0.0 && t.width > 0.0) {
        const Vec3 d = t.axis.normalized();
        out.push_back({cylinder("Turret", t.centre - 0.5 * t.width * d, d, t.diameter, t.width),
                       "T", true, kTurretColour});
    }
    if (const auto& w = l.tailstock; w && w->diameter > 0.0 && w->length > 0.0) {
        out.push_back({cylinder("Tailstock body", w->z * z, z, w->diameter, w->length), "W", true,
                       kCastColour});
    }
    if (const auto& s = l.subSpindle) {
        if (s->diameter > 0.0 && s->length > 0.0) {
            out.push_back({cylinder("Sub-spindle", s->nose * z, z, s->diameter, s->length), "Z2",
                           true, kCastColour});
        }
        if (s->chuckDiameter > 0.0 && s->chuckLength > 0.0) {
            out.push_back({cylinder("Sub-spindle chuck", (s->nose - s->chuckLength) * z, z,
                                    s->chuckDiameter, s->chuckLength),
                           "C2", true, kChuckColour});
        }
    }
    if (l.hasEnclosure()) {
        const Vec3& a = l.enclosureMin;
        const Vec3& b = l.enclosureMax;
        const double w = l.wall;
        // The walls on the machine's front and top start hidden: the door and the roof.
        const MachineOrientation stands = machine.standing();
        auto wall = [&](const char* name, const Vec3& lo, const Vec3& hi, AxisSense side) {
            const bool shown = side != stands.front && side != stands.up;
            out.push_back({box(name, lo, hi), "", shown, kEnclosureColour, true});
        };
        // The inside of it stays clear; the walls overlap at the corners, which nothing can see.
        const Vec3 oa = a - Vec3::Constant(w);
        const Vec3 ob = b + Vec3::Constant(w);
        wall("Enclosure, headstock end", oa, Vec3(ob.x(), ob.y(), a.z()), AxisSense::MinusZ);
        wall("Enclosure, far end", Vec3(oa.x(), oa.y(), b.z()), ob, AxisSense::PlusZ);
        wall("Enclosure, door", oa, Vec3(a.x(), ob.y(), ob.z()), AxisSense::MinusX);
        wall("Enclosure, back", Vec3(b.x(), oa.y(), oa.z()), ob, AxisSense::PlusX);
        wall("Enclosure, floor", oa, Vec3(ob.x(), a.y(), ob.z()), AxisSense::MinusY);
        wall("Enclosure, roof", Vec3(oa.x(), b.y(), oa.z()), ob, AxisSense::PlusY);
    }
    return out;
}

double stationAngle(const Machine& machine, int station) {
    const int stations = std::max(machine.toolCapacity, 1);
    return 2.0 * kPi * static_cast<double>(station - 1) / stations;
}

double indexTo(const Machine& machine, double from, int station) {
    const double to = stationAngle(machine, station);
    double delta = std::remainder(to - from, 2.0 * kPi);
    if (std::abs(delta + kPi) < 1e-9) {
        delta = kPi;
    }
    return from + delta;
}

Transform3 turretTurn(const LatheTurret& turret, double t) {
    Transform3 xf = Transform3::Identity();
    const Mat3 r = Eigen::AngleAxisd(t, turret.axis.normalized()).toRotationMatrix();
    xf.linear() = r;
    xf.translation() = turret.centre - r * turret.centre;
    return xf;
}

Vec2 defaultTipFromCentre(const LatheLayout& layout, const StationMount& mount) {
    const LatheTurret& t = layout.turret;
    if (mount.across) {
        // Its shank's centre on the disc's face: the disc stays behind the tip, clear of a chuck.
        return {t.radialReach, mount.towardsPlusZ ? 0.5 * t.width + mount.centreLine
                                                  : -0.5 * t.width - mount.centreLine};
    }
    const double along = 0.5 * t.width + t.axialReach;
    return {t.axialPitch + mount.centreLine, mount.towardsPlusZ ? along : -along};
}

Vec3 tipPoint(const LatheLayout& layout, const Vec2& fromCentre) {
    return layout.turret.centre + Vec3(-fromCentre.x(), layout.turret.blockAcross, fromCentre.y());
}

std::string_view toString(LatheStep::Kind kind) noexcept {
    switch (kind) {
    case LatheStep::Kind::Rapid:
        return "the rapid to the start";
    case LatheStep::Kind::Index:
        return "the turret's index";
    case LatheStep::Kind::Tailstock:
        return "the tailstock coming in";
    case LatheStep::Kind::Transfer:
        return "the part transfer";
    }
    return "";
}

bool approachOutFirst(const Vec3& from, const Vec3& to) {
    constexpr double kSame = 1e-6;
    return std::hypot(to.x(), to.y()) > std::hypot(from.x(), from.y()) + kSame &&
           to.z() < from.z() - kSame;
}

AxisPosition lerpAxes(const AxisPosition& a, const AxisPosition& b, double f) {
    AxisPosition p;
    p.linear = a.linear + (b.linear - a.linear) * f;
    p.rotary.a = a.rotary.a + (b.rotary.a - a.rotary.a) * f;
    p.rotary.b = a.rotary.b + (b.rotary.b - a.rotary.b) * f;
    p.rotary.c = a.rotary.c + (b.rotary.c - a.rotary.c) * f;
    p.t = a.t + (b.t - a.t) * f;
    p.w = a.w + (b.w - a.w) * f;
    p.z2 = a.z2 + (b.z2 - a.z2) * f;
    p.c2 = a.c2 + (b.c2 - a.c2) * f;
    return p;
}

double latheSpindleAngle(const toolpath::Toolpath& path, const Vec3& tip) {
    if (!path.live && path.liveAxes == toolpath::LiveAxes::None) {
        return 0.0;
    }
    const Vec3 axis = path.moves.empty() ? Vec3::UnitZ() : path.moves.front().axis;
    const bool across = std::abs(axis.z()) <= 1.0 - 1e-9;
    if (across) {
        return std::atan2(axis.y(), axis.x());
    }
    if (path.liveAxes == toolpath::LiveAxes::HeldC) {
        return path.liveC;
    }
    return std::hypot(tip.x(), tip.y()) < 1e-9 ? 0.0 : std::atan2(tip.y(), tip.x());
}

std::optional<AxisPosition> latheAxes(const MachineModel& model, const LatheMotion& motion,
                                      const WorkMount& mount, const toolpath::Toolpath& path,
                                      const Vec3& tipWcs) {
    AxisPosition p = motion.held;
    p.rotary.c = latheSpindleAngle(path, tipWcs);
    p.linear = Vec3::Zero();
    const Vec3 target = mount.wcs * tipWcs;
    // Gap between the tool and the work: affine in the linear axes. The tool's point is where it is
    // with the turret working, moved by what the linear axes do to the turret.
    const Vec3 still =
        poseAt(model, AxisPosition{}).carried(model, model.spindleCarrier).translation();
    auto gap = [&](const AxisPosition& at) {
        AxisPosition slides;
        slides.linear = at.linear;
        const Vec3 moved = poseAt(model, slides).carried(model, model.spindleCarrier).translation();
        return Vec3(motion.tip + moved - still -
                    poseAt(model, at).carried(model, mount.carrier) * target);
    };
    const Vec3 f0 = gap(p);
    Eigen::Matrix<double, 3, Eigen::Dynamic> j(3, 0);
    std::vector<int> used;
    for (int k = 0; k < 3; ++k) {
        const char* names[] = {"X", "Y", "Z"};
        if (!model.axis(names[k])) {
            continue;
        }
        AxisPosition e = p;
        e.linear[k] = 1.0;
        j.conservativeResize(3, j.cols() + 1);
        j.col(j.cols() - 1) = gap(e) - f0;
        used.push_back(k);
    }
    if (used.empty()) {
        return std::nullopt;
    }
    // A lathe's turret need not reach every direction: without Y, the tool stands on the centre
    // line by how it is set up.
    const Eigen::VectorXd l = j.completeOrthogonalDecomposition().solve(-f0);
    for (std::size_t i = 0; i < used.size(); ++i) {
        p.linear[used[i]] = l(static_cast<Eigen::Index>(i));
    }
    return p;
}

} // namespace ocam::machine
