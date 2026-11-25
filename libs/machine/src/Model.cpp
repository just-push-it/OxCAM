// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Machine models: forward kinematics of the axis tree (each axis's motion composed from the
// base), the linear axis positions for a tool tip on the work (an affine 3x3 system), the table
// chain and centre, and the checks that tie a model to its kinematics type. See
// docs/design-log/0037.

#include "ocam/machine/Model.hpp"

#include "ocam/machine/Turret.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Sha256.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/geom3d/Stl.hpp>

#include <fmt/format.h>

#include <Eigen/LU>
#include <algorithm>
#include <cmath>

namespace ocam::machine {
namespace {

Mat3 rotAbout(const Vec3& axis, double angle) {
    return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

std::string vecText(const Vec3& v) {
    return fmt::format("({:g}, {:g}, {:g})", v.x(), v.y(), v.z());
}

/// Index of the axis named `name` in the model, or -1.
int indexOf(const MachineModel& model, std::string_view name) {
    for (std::size_t i = 0; i < model.axes.size(); ++i) {
        if (model.axes[i].name == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

/// The chain from `carrier` up to the base (carrier first).
std::vector<int> chainOf(const MachineModel& model, std::string_view carrier) {
    std::vector<int> chain;
    int i = indexOf(model, carrier);
    while (i >= 0 && chain.size() <= model.axes.size()) {
        chain.push_back(i);
        i = indexOf(model, model.axes[static_cast<std::size_t>(i)].parent);
    }
    return chain;
}

/// How a kinematics type's rotary axes turn: the head or the table, about which machine axis.
struct RotaryRole {
    const char* axis;
    bool head;
    Vec3 direction;
};

std::vector<RotaryRole> rotaryRoles(Kinematics k) {
    switch (k) {
    case Kinematics::Mill4A:
        return {{"A", false, Vec3::UnitX()}};
    case Kinematics::Mill5TableAC:
        return {{"A", false, Vec3::UnitX()}, {"C", false, Vec3::UnitZ()}};
    case Kinematics::Mill5TableBC:
        return {{"B", false, Vec3::UnitY()}, {"C", false, Vec3::UnitZ()}};
    case Kinematics::Mill5HeadTableBC:
        return {{"B", true, Vec3::UnitY()}, {"C", false, Vec3::UnitZ()}};
    case Kinematics::Mill5HeadBC:
        return {{"C", true, Vec3::UnitZ()}, {"B", true, Vec3::UnitY()}};
    case Kinematics::Mill3:
    case Kinematics::Lathe2:
        break;
    }
    return {};
}

const char* axisText(const Vec3& d) {
    if (d == Vec3::UnitX()) {
        return "+X";
    }
    return d == Vec3::UnitY() ? "+Y" : "+Z";
}

bool sameRotation(const Mat3& a, const Mat3& b) {
    return (a - b).cwiseAbs().maxCoeff() < 1e-9;
}

void addTriangle(geom3d::Mesh& mesh, std::uint32_t a, std::uint32_t b, std::uint32_t c) {
    mesh.triangles.push_back({a, b, c});
}

} // namespace

Transform3 ModelPart::placement() const {
    Transform3 xf = Transform3::Identity();
    xf.translation() = offset;
    xf.linear() = (Eigen::AngleAxisd(degToRad(rotation.z()), Vec3::UnitZ()) *
                   Eigen::AngleAxisd(degToRad(rotation.y()), Vec3::UnitY()) *
                   Eigen::AngleAxisd(degToRad(rotation.x()), Vec3::UnitX()))
                      .toRotationMatrix();
    return xf;
}

bool ModelPart::operator==(const ModelPart& o) const {
    return name == o.name && carrier == o.carrier && offset == o.offset && rotation == o.rotation &&
           colour == o.colour && collides == o.collides && shown == o.shown &&
           enclosure == o.enclosure && step == o.step &&
           mesh == o.mesh && (!mesh.empty() || !step.empty() || geometry == o.geometry);
}

const ModelAxis* MachineModel::axis(std::string_view axisName) const {
    const int i = indexOf(*this, axisName);
    return i < 0 ? nullptr : &axes[static_cast<std::size_t>(i)];
}

double AxisPosition::value(std::string_view axis) const noexcept {
    if (axis == "X") {
        return linear.x();
    }
    if (axis == "Y") {
        return linear.y();
    }
    if (axis == "Z") {
        return linear.z();
    }
    if (axis == "A") {
        return rotary.a;
    }
    if (axis == "B") {
        return rotary.b;
    }
    if (axis == "C") {
        return rotary.c;
    }
    if (axis == "T") {
        return t;
    }
    if (axis == "W") {
        return w;
    }
    if (axis == "Z2") {
        return z2;
    }
    return axis == "C2" ? c2 : 0.0;
}

Transform3 ModelPose::carried(const MachineModel& model, std::string_view carrier) const {
    const int i = indexOf(model, carrier);
    return i < 0 || static_cast<std::size_t>(i) >= axes.size() ? Transform3::Identity()
                                                               : axes[static_cast<std::size_t>(i)];
}

ModelPose poseAt(const MachineModel& model, const AxisPosition& position) {
    const std::size_t n = model.axes.size();
    ModelPose pose;
    pose.axes.assign(n, Transform3::Identity());
    // 0 = not done, 1 = in progress, 2 = done.
    std::vector<int> state(n, 0);
    std::function<void(std::size_t)> resolve = [&](std::size_t i) {
        if (state[i] == 2) {
            return;
        }
        if (state[i] == 1) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("the machine model's axes carry each other in a loop ({})",
                                    model.axes[i].name));
        }
        state[i] = 1;
        const ModelAxis& a = model.axes[i];
        Transform3 parent = Transform3::Identity();
        if (!a.parent.empty()) {
            const int p = indexOf(model, a.parent);
            if (p < 0) {
                throw Error(
                    ErrorCode::InvalidArgument,
                    fmt::format("the {} axis sits on an unknown axis '{}'", a.name, a.parent));
            }
            resolve(static_cast<std::size_t>(p));
            parent = pose.axes[static_cast<std::size_t>(p)];
        }
        const double q = position.value(a.name);
        Transform3 motion = Transform3::Identity();
        if (a.rotary()) {
            const Mat3 r = rotAbout(a.direction, q);
            motion.linear() = r;
            motion.translation() = a.pivot - r * a.pivot;
        } else {
            motion.translation() = q * a.direction.normalized();
        }
        pose.axes[i] = parent * motion;
        state[i] = 2;
    };
    for (std::size_t i = 0; i < n; ++i) {
        resolve(i);
    }
    pose.spindle =
        pose.carried(model, model.spindleCarrier) * Eigen::Translation3d(model.spindlePoint);
    pose.table = pose.carried(model, tableEnd(model)) *
                 Eigen::Translation3d(tableCentre(model, Vec3::Zero()));
    return pose;
}

std::string tableEnd(const MachineModel& model) {
    const std::vector<int> spindle = chainOf(model, model.spindleCarrier);
    std::size_t longest = 0;
    std::string end;
    for (std::size_t i = 0; i < model.axes.size(); ++i) {
        if (std::find(spindle.begin(), spindle.end(), static_cast<int>(i)) != spindle.end()) {
            continue;
        }
        const std::size_t length = chainOf(model, model.axes[i].name).size();
        if (length > longest) {
            longest = length;
            end = model.axes[i].name;
        }
    }
    return end;
}

std::vector<const ModelAxis*> tableRotaries(const MachineModel& model) {
    std::vector<const ModelAxis*> out;
    const std::vector<int> chain = chainOf(model, tableEnd(model));
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        const ModelAxis& a = model.axes[static_cast<std::size_t>(*it)];
        if (a.rotary()) {
            out.push_back(&a);
        }
    }
    return out;
}

Vec3 tableCentre(const MachineModel& model, const Vec3& near) {
    const std::vector<const ModelAxis*> rotaries = tableRotaries(model);
    if (rotaries.empty()) {
        return Vec3::Zero();
    }
    const ModelAxis& inner = *rotaries.back();
    const Vec3 d = inner.direction.normalized();
    if (rotaries.size() >= 2) {
        // The point of the inner line nearest the outer line.
        const ModelAxis& outer = *rotaries.front();
        const Vec3 e = outer.direction.normalized();
        const Vec3 w = inner.pivot - outer.pivot;
        const double b = d.dot(e);
        const double denom = 1.0 - b * b;
        if (denom > 1e-12) {
            const double t = (b * e.dot(w) - d.dot(w)) / denom;
            return inner.pivot + t * d;
        }
    }
    return inner.pivot + d.dot(near - inner.pivot) * d;
}

Transform3 wcsInTable(const MachineModel& model, const WorkMount& mount) {
    return Eigen::Translation3d(-tableCentre(model, mount.wcs.translation())) * mount.wcs;
}

WorkMount tableMount(const MachineModel& model, const Transform3& wcsInTable) {
    return {tableEnd(model), Eigen::Translation3d(tableCentre(model, Vec3::Zero())) * wcsInTable};
}

std::string mountProblem(const MachineModel& model, std::string_view carrier) {
    if (!carrier.empty() && indexOf(model, carrier) < 0) {
        return fmt::format("'{}' is not an axis of the machine", carrier);
    }
    const std::vector<int> spindle = chainOf(model, model.spindleCarrier);
    const std::vector<int> work = chainOf(model, carrier);
    if (!carrier.empty() &&
        std::find(spindle.begin(), spindle.end(), indexOf(model, carrier)) != spindle.end()) {
        return fmt::format("the {} axis carries the spindle: the work must sit on the table side",
                           carrier);
    }
    for (const int i : chainOf(model, tableEnd(model))) {
        const ModelAxis& a = model.axes[static_cast<std::size_t>(i)];
        if (!a.rotary() && std::find(work.begin(), work.end(), i) == work.end()) {
            return fmt::format(
                "the {} axis moves the table, but not {}: the work must move with the table",
                a.name,
                carrier.empty() ? std::string("the machine base")
                                : fmt::format("the {} axis", carrier));
        }
    }
    return {};
}

std::vector<std::string> rotariesNotCarrying(const MachineModel& model, std::string_view carrier) {
    std::vector<std::string> out;
    const std::vector<int> work = chainOf(model, carrier);
    for (const ModelAxis* a : tableRotaries(model)) {
        if (std::find(work.begin(), work.end(), indexOf(model, a->name)) == work.end()) {
            out.push_back(a->name);
        }
    }
    return out;
}

std::optional<geom3d::AxisLine> rotaryLineOf(const MachineModel& model, std::string_view carrier,
                                             std::string* axisName) {
    for (const int i : chainOf(model, carrier)) {
        const ModelAxis& a = model.axes[static_cast<std::size_t>(i)];
        if (a.rotary()) {
            if (axisName) {
                *axisName = a.name;
            }
            return geom3d::AxisLine{a.pivot, a.direction.normalized(), 0.0, 0};
        }
    }
    return std::nullopt;
}

MachineModel defaultModel(const Machine& machine) {
    MachineModel m;
    if (machine.isLathe()) {
        if (!machine.lathe) {
            return m;
        }
        const LatheLayout& l = *machine.lathe;
        m.axes.push_back({"Z", "", Vec3::UnitZ(), Vec3::Zero()});
        m.axes.push_back({"X", "Z", Vec3::UnitX(), Vec3::Zero()});
        std::string last = "X";
        if (machine.hasYAxis()) {
            m.axes.push_back({"Y", "X", Vec3::UnitY(), Vec3::Zero()});
            last = "Y";
        }
        m.axes.push_back({"T", last, l.turret.axis.normalized(), l.turret.centre});
        // C positive turns the part clockwise, seen from the turret side of the chuck (DL-0066).
        m.axes.push_back({"C", "", -Vec3::UnitZ(), Vec3::Zero()});
        if (l.tailstock) {
            m.axes.push_back({"W", "", Vec3::UnitZ(), Vec3::Zero()});
        }
        if (l.subSpindle) {
            m.axes.push_back({"Z2", "", Vec3::UnitZ(), Vec3::Zero()});
            m.axes.push_back({"C2", "Z2", Vec3::UnitZ(), Vec3::Zero()});
        }
        m.spindleCarrier = "T";
        m.spindlePoint = l.turret.centre;
        return m;
    }
    m.spindlePoint = Vec3(0, 0, kDefaultSpindleHeight);
    const Vec3 headPivot = m.spindlePoint + machine.pivotLength * Vec3::UnitZ();
    auto add = [&](const char* name, const char* parent, const Vec3& direction,
                   const Vec3& pivot = Vec3::Zero()) {
        m.axes.push_back({name, parent, direction, pivot});
    };
    if (machine.kinematics == Kinematics::Mill5HeadBC) {
        // Gantry-style head-head machine over a fixed table.
        add("X", "", Vec3::UnitX());
        add("Y", "X", Vec3::UnitY());
        add("Z", "Y", Vec3::UnitZ());
        add("C", "Z", Vec3::UnitZ(), headPivot);
        add("B", "C", Vec3::UnitY(), headPivot);
        m.spindleCarrier = "B";
        return m;
    }
    add("Y", "", -Vec3::UnitY());
    add("X", "Y", -Vec3::UnitX());
    add("Z", "", Vec3::UnitZ());
    m.spindleCarrier = "Z";
    switch (machine.kinematics) {
    case Kinematics::Mill4A:
        add("A", "X", Vec3::UnitX());
        break;
    case Kinematics::Mill5TableAC:
        add("A", "X", Vec3::UnitX());
        add("C", "A", Vec3::UnitZ());
        break;
    case Kinematics::Mill5TableBC:
        add("B", "X", Vec3::UnitY());
        add("C", "B", Vec3::UnitZ());
        break;
    case Kinematics::Mill5HeadTableBC:
        add("C", "X", Vec3::UnitZ());
        add("B", "Z", Vec3::UnitY(), headPivot);
        m.spindleCarrier = "B";
        break;
    default:
        break;
    }
    return m;
}

MachineModel effectiveModel(const Machine& machine) {
    if (machine.isLathe() && !machine.lathe) {
        return {};
    }
    return machine.model.empty() ? defaultModel(machine) : machine.model;
}

namespace {

/// A lathe's tree: the axes a lathe has, the tools on T, and the slides free to move the tool
/// relative to the work along their machine axes (DL-0067).
std::vector<std::string> validateLatheModel(const Machine& machine) {
    std::vector<std::string> issues;
    const MachineModel& model = machine.model;
    if (!machine.lathe) {
        issues.emplace_back("a lathe's machine model needs its layout: the turret's stations and "
                            "sizes say where the tools are");
        return issues;
    }
    for (const ModelAxis& a : model.axes) {
        static const std::vector<std::string> known{"X", "Y", "Z", "T", "C", "W", "Z2", "C2"};
        if (std::find(known.begin(), known.end(), a.name) == known.end()) {
            issues.push_back(fmt::format("a lathe's model has no {} axis yet", a.name));
        }
        if (!a.parent.empty() && indexOf(model, a.parent) < 0) {
            issues.push_back(fmt::format(
                "the {} axis sits on '{}', which is not an axis of the model", a.name, a.parent));
        }
        if (a.direction.norm() < 1e-9 || !a.direction.allFinite()) {
            issues.push_back(fmt::format("the {} axis needs a direction", a.name));
        }
    }
    for (const char* name : {"X", "Z", "T", "C"}) {
        if (indexOf(model, name) < 0) {
            issues.push_back(fmt::format("the model needs the {} axis", name));
        }
    }
    if (machine.hasYAxis() && indexOf(model, "Y") < 0) {
        issues.emplace_back("the model needs the Y axis");
    }
    if (model.spindleCarrier != "T") {
        issues.emplace_back("the tools ride on the turret: the T axis must carry them");
    }
    for (const ModelPart& p : model.parts) {
        if (!p.carrier.empty() && indexOf(model, p.carrier) < 0) {
            issues.push_back(
                fmt::format("part '{}' is carried by '{}', which is not an axis of the model",
                            p.name, p.carrier));
        }
        if (p.mesh.empty() && p.step.empty() && !p.geometry) {
            issues.push_back(fmt::format("part '{}' has no geometry", p.name));
        }
    }
    if (!issues.empty()) {
        return issues;
    }
    AxisPosition zeroPosition;
    ModelPose zero;
    try {
        zero = poseAt(model, zeroPosition);
    } catch (const Error& e) {
        issues.emplace_back(e.what());
        return issues;
    }
    // Slides under a rotary axis would make the positions depend on it.
    for (const char* name : {"X", "Y", "Z"}) {
        for (const int i : chainOf(model, name)) {
            if (model.axes[static_cast<std::size_t>(i)].rotary()) {
                issues.push_back(fmt::format("the {} axis cannot ride on the {} axis", name,
                                             model.axes[static_cast<std::size_t>(i)].name));
            }
        }
    }
    const char* linear[] = {"X", "Y", "Z"};
    const Vec3 rel0 =
        zero.carried(model, "T").translation() - zero.carried(model, "C").translation();
    for (int k = 0; k < 3; ++k) {
        if (indexOf(model, linear[k]) < 0) {
            continue;
        }
        AxisPosition p;
        p.linear[k] = 1.0;
        const ModelPose moved = poseAt(model, p);
        const Vec3 rel = moved.carried(model, "T").translation() -
                         moved.carried(model, "C").translation() - rel0;
        if ((rel - Vec3::Unit(k)).norm() > 1e-9) {
            issues.push_back(fmt::format("the {0} axis must move the turret along {0}+ relative to "
                                         "the spindle",
                                         linear[k]));
        }
    }
    return issues;
}

} // namespace

std::vector<std::string> validateModel(const Machine& machine) {
    std::vector<std::string> issues;
    const MachineModel& model = machine.model;
    if (model.empty()) {
        return issues;
    }
    if (machine.isLathe()) {
        return validateLatheModel(machine);
    }
    for (std::size_t i = 0; i < model.axes.size(); ++i) {
        const ModelAxis& a = model.axes[i];
        if (!machine.axis(a.name)) {
            issues.push_back(fmt::format("the model's {} axis is not one of the machine's axes",
                                         a.name.empty() ? "unnamed" : a.name));
        }
        if (indexOf(model, a.name) != static_cast<int>(i)) {
            issues.push_back(fmt::format("the model has the {} axis twice", a.name));
        }
        if (!a.parent.empty() && indexOf(model, a.parent) < 0) {
            issues.push_back(fmt::format(
                "the {} axis sits on '{}', which is not an axis of the model", a.name, a.parent));
        }
        if (a.parent == a.name) {
            issues.push_back(fmt::format("the {} axis cannot carry itself", a.name));
        }
        if (a.direction.norm() < 1e-9 || !a.direction.allFinite()) {
            issues.push_back(fmt::format("the {} axis needs a direction", a.name));
        }
    }
    std::vector<std::string> needed{"X", "Y", "Z"};
    for (const char c : machine.rotaryAxes()) {
        needed.emplace_back(1, c);
    }
    for (const std::string& name : needed) {
        if (indexOf(model, name) < 0) {
            issues.push_back(fmt::format("the model needs the {} axis", name));
        }
    }
    for (const ModelAxis& a : model.axes) {
        if (a.rotary() && std::find(needed.begin(), needed.end(), a.name) == needed.end()) {
            issues.push_back(
                fmt::format("{} machines have no {} axis", toString(machine.kinematics), a.name));
        }
    }
    auto checkCarrier = [&](const std::string& carrier, const std::string& what) {
        if (!carrier.empty() && indexOf(model, carrier) < 0) {
            issues.push_back(fmt::format("{} is carried by '{}', which is not an axis of the model",
                                         what, carrier));
        }
    };
    checkCarrier(model.spindleCarrier, "the spindle");
    for (const ModelPart& p : model.parts) {
        checkCarrier(p.carrier, fmt::format("part '{}'", p.name));
        if (p.mesh.empty() && p.step.empty() && !p.geometry) {
            issues.push_back(fmt::format("part '{}' has no geometry", p.name));
        }
    }
    if (!issues.empty()) {
        return issues;
    }
    ModelPose zero;
    try {
        zero = poseAt(model, {});
    } catch (const Error& e) {
        issues.emplace_back(e.what());
        return issues;
    }

    // The axes that do not carry the spindle move the table: one chain of them.
    const std::vector<int> spindleChain = chainOf(model, model.spindleCarrier);
    const std::vector<int> tableChain = chainOf(model, tableEnd(model));
    auto carries = [](const std::vector<int>& chain, int i) {
        return std::find(chain.begin(), chain.end(), i) != chain.end();
    };
    for (std::size_t i = 0; i < model.axes.size(); ++i) {
        const int k = static_cast<int>(i);
        if (!carries(spindleChain, k) && !carries(tableChain, k)) {
            issues.push_back(fmt::format(
                "the {} axis carries neither the spindle nor the {} axis: the axes that do not "
                "move the spindle must carry each other, one table",
                model.axes[i].name, model.axes[static_cast<std::size_t>(tableChain.front())].name));
        }
    }
    if (!issues.empty()) {
        return issues;
    }

    // Linear axes: a positive value moves the tool along the machine axis relative to the table.
    const Vec3 rel0 = zero.spindle.translation() - zero.table.translation();
    const char* linear[] = {"X", "Y", "Z"};
    for (int k = 0; k < 3; ++k) {
        const int i = indexOf(model, linear[k]);
        AxisPosition p;
        p.linear[k] = 1.0;
        const ModelPose moved = poseAt(model, p);
        const Vec3 rel = moved.spindle.translation() - moved.table.translation() - rel0;
        const Vec3 want = Vec3::Unit(k);
        if ((rel - want).norm() < 1e-9) {
            continue;
        }
        const bool spindle = carries(spindleChain, i);
        const bool table = carries(tableChain, i);
        if (spindle == table) {
            issues.push_back(
                fmt::format("the {} axis must move either the spindle or the table (it moves {})",
                            linear[k], spindle ? "both" : "neither"));
        } else {
            issues.push_back(fmt::format(
                "the {0} axis moves the {1}: its direction must be {2}{0} so that {0}+ moves the "
                "tool towards {0}+ of the table",
                linear[k], spindle ? "spindle" : "table", spindle ? "+" : "-"));
        }
    }
    if (!issues.empty()) {
        return issues;
    }

    // Rotary axes: each turns the head or the table as the kinematics type (and solve) has it.
    for (const RotaryRole& role : rotaryRoles(machine.kinematics)) {
        const int i = indexOf(model, role.axis);
        AxisPosition p;
        const double angle = 0.7;
        IndexAngles q;
        (role.axis[0] == 'A' ? q.a : role.axis[0] == 'B' ? q.b : q.c) = angle;
        p.rotary = q;
        const ModelPose moved = poseAt(model, p);
        const bool ok = sameRotation(moved.table.linear(), tableRotation(machine, q)) &&
                        sameRotation(moved.spindle.linear(), headRotation(machine, q));
        if (!ok) {
            const bool onSpindle = carries(spindleChain, i);
            const bool onTable = carries(tableChain, i);
            if (onSpindle == onTable) {
                issues.push_back(fmt::format("the {} axis must turn the {} (it turns {})",
                                             role.axis, role.head ? "spindle head" : "table",
                                             onSpindle ? "both" : "neither"));
            } else {
                issues.push_back(fmt::format(
                    "the {} axis must turn the {} right-handed about {}, as {} machines do",
                    role.axis, role.head ? "spindle head" : "table", axisText(role.direction),
                    toString(machine.kinematics)));
            }
        }
    }
    if (issues.empty()) {
        // All rotaries together: the order in the tree.
        AxisPosition p;
        p.rotary = {0.4, 0.5, 0.9};
        for (const char c : std::string("ABC")) {
            if (machine.rotaryAxes().find(c) == std::string::npos) {
                (c == 'A' ? p.rotary.a : c == 'B' ? p.rotary.b : p.rotary.c) = 0.0;
            }
        }
        const ModelPose moved = poseAt(model, p);
        if (!sameRotation(moved.table.linear(), tableRotation(machine, p.rotary)) ||
            !sameRotation(moved.spindle.linear(), headRotation(machine, p.rotary))) {
            const auto roles = rotaryRoles(machine.kinematics);
            issues.push_back(fmt::format("the {} axis must carry the {} axis, as on {} machines",
                                         roles.front().axis, roles.back().axis,
                                         toString(machine.kinematics)));
        }
    }
    return issues;
}

std::optional<AxisPosition> positionFor(const MachineModel& model, const IndexAngles& rotary,
                                        std::string_view carrier, const Vec3& tip,
                                        double gaugeLength) {
    auto gap = [&](const AxisPosition& p) {
        const ModelPose pose = poseAt(model, p);
        const Vec3 tool = pose.spindle.translation() - gaugeLength * pose.spindle.linear().col(2);
        return Vec3(tool - pose.carried(model, carrier) * tip);
    };
    AxisPosition p;
    p.rotary = rotary;
    const Vec3 f0 = gap(p);
    Mat3 j;
    for (int k = 0; k < 3; ++k) {
        AxisPosition e = p;
        e.linear[k] = 1.0;
        j.col(k) = gap(e) - f0;
    }
    const Eigen::FullPivLU<Mat3> lu(j);
    if (lu.rank() < 3) {
        return std::nullopt;
    }
    p.linear = lu.solve(-f0);
    return p;
}

std::string cradleCarrier(const MachineModel& model) {
    const std::vector<int> chain = chainOf(model, tableEnd(model));
    int rotaries = 0;
    for (const int i : chain) {
        if (model.axes[static_cast<std::size_t>(i)].rotary() && ++rotaries == 2) {
            return model.axes[static_cast<std::size_t>(i)].name;
        }
    }
    for (const int i : chain) {
        if (!model.axes[static_cast<std::size_t>(i)].rotary()) {
            return model.axes[static_cast<std::size_t>(i)].name;
        }
    }
    return {};
}

std::optional<geom3d::AxisLine> axisAtPoint(const ModelPart& part, const Vec3& point) {
    if (!part.geometry || part.geometry->isEmpty()) {
        return std::nullopt;
    }
    const Transform3 placement = part.placement();
    const Vec3 local = placement.inverse() * point;
    const double size = part.geometry->bounds().size().norm();
    const auto triangle = geom3d::nearestTriangle(*part.geometry, local, 1e-3 * size + 0.05);
    if (!triangle) {
        return std::nullopt;
    }
    auto line = geom3d::revolutionAxis(*part.geometry, *triangle);
    if (line) {
        line->point = placement * line->point;
        line->direction = placement.linear() * line->direction;
    }
    return line;
}

std::optional<Vec3> normalAtPoint(const ModelPart& part, const Vec3& point) {
    if (!part.geometry || part.geometry->isEmpty()) {
        return std::nullopt;
    }
    const Transform3 placement = part.placement();
    const double size = part.geometry->bounds().size().norm();
    const auto triangle =
        geom3d::nearestTriangle(*part.geometry, placement.inverse() * point, 1e-3 * size + 0.05);
    if (!triangle) {
        return std::nullopt;
    }
    const Vec3 n = part.geometry->triangleNormal(*triangle);
    if (n.squaredNorm() < 0.5) {
        return std::nullopt;
    }
    return Vec3(placement.linear() * n);
}

std::optional<geom3d::FlatFace> faceAtPoint(const ModelPart& part, const Vec3& point) {
    if (!part.geometry || part.geometry->isEmpty()) {
        return std::nullopt;
    }
    const Transform3 placement = part.placement();
    const double size = part.geometry->bounds().size().norm();
    const auto triangle =
        geom3d::nearestTriangle(*part.geometry, placement.inverse() * point, 1e-3 * size + 0.05);
    if (!triangle) {
        return std::nullopt;
    }
    auto face = geom3d::flatFace(*part.geometry, *triangle);
    if (face) {
        face->centre = placement * face->centre;
        face->normal = placement.linear() * face->normal;
    }
    return face;
}

std::optional<Vec3> lineMeetsPlane(const geom3d::AxisLine& line, const Vec3& point,
                                   const Vec3& normal) {
    const Vec3 d = line.direction.normalized();
    const Vec3 n = normal.normalized();
    const double across = n.dot(d);
    if (std::abs(across) < 0.1) {
        return std::nullopt;
    }
    return Vec3(line.point + (n.dot(point - line.point) / across) * d);
}

Vec3 snapOnPart(const MachineModel& model, const ModelPart& part, const Vec3& point, Snap snap) {
    if (snap == Snap::Point) {
        if (!normalAtPoint(part, point)) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("{} is not on '{}'", vecText(point), part.name));
        }
        return point;
    }
    const auto face = faceAtPoint(part, point);
    if (!face) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("no flat face of '{}' at {}", part.name, vecText(point)));
    }
    if (snap == Snap::FaceCentre) {
        return face->centre;
    }
    std::string axis;
    const auto line = rotaryLineOf(model, part.carrier, &axis);
    if (!line) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("no rotary axis carries '{}': snap to the face's centre instead",
                                part.name));
    }
    const auto meet = lineMeetsPlane(*line, point, face->normal);
    if (!meet) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("that face of '{}' runs along the {} axis: click a face square to "
                                "it",
                                part.name, axis));
    }
    return *meet;
}

Vec3 gaugePointOn(const geom3d::AxisLine& axis, const Vec3& seat) {
    const Vec3 d = axis.direction.normalized();
    return axis.point + (seat - axis.point).dot(d) * d;
}

std::optional<Vec3> sameSense(const Vec3& direction, const Vec3& previous) {
    const Vec3 d = direction.normalized();
    const double c = previous.norm() > 1e-12 ? d.dot(previous.normalized()) : 0.0;
    if (std::abs(c) < 0.2) {
        return std::nullopt;
    }
    return c < 0.0 ? Vec3(-d) : d;
}

geom3d::Mesh componentMesh(const Component& c) {
    geom3d::Mesh mesh;
    if (c.shape == Component::Shape::Box) {
        const Vec3 lo = c.min.cwiseMin(c.max), hi = c.min.cwiseMax(c.max);
        for (int i = 0; i < 8; ++i) {
            mesh.vertices.emplace_back((i & 1) ? hi.x() : lo.x(), (i & 2) ? hi.y() : lo.y(),
                                       (i & 4) ? hi.z() : lo.z());
        }
        // Outward-facing quads as two triangles each.
        const int faces[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4},
                                 {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
        for (const auto& f : faces) {
            addTriangle(mesh, f[0], f[1], f[2]);
            addTriangle(mesh, f[0], f[2], f[3]);
        }
        return mesh;
    }
    constexpr int kSides = 32;
    const Vec3 w = c.axis.normalized();
    const Vec3 u = w.unitOrthogonal();
    const Vec3 v = w.cross(u);
    for (int end = 0; end < 2; ++end) {
        const Vec3 centre = c.base + (end ? c.length : 0.0) * w;
        for (int k = 0; k < kSides; ++k) {
            const double t = 2.0 * kPi * k / kSides;
            mesh.vertices.push_back(centre + c.radius * (std::cos(t) * u + std::sin(t) * v));
        }
        mesh.vertices.push_back(centre);
    }
    const auto bottomCentre = static_cast<std::uint32_t>(kSides);
    const auto top = static_cast<std::uint32_t>(kSides + 1);
    const auto topCentre = top + kSides;
    for (std::uint32_t k = 0; k < kSides; ++k) {
        const std::uint32_t n = (k + 1) % kSides;
        addTriangle(mesh, bottomCentre, n, k);
        addTriangle(mesh, topCentre, top + k, top + n);
        addTriangle(mesh, k, n, top + n);
        addTriangle(mesh, k, top + n, top + k);
    }
    return mesh;
}

std::vector<ModelPart> componentParts(const Machine& machine, const MachineModel& model) {
    std::vector<ModelPart> parts;
    for (const LatheSolid& s : latheSolids(machine)) {
        ModelPart p;
        p.name = s.shape.name;
        p.carrier = s.carrier;
        p.colour = s.colour;
        p.shown = s.shown;
        p.enclosure = s.enclosure;
        p.geometry = std::make_shared<const geom3d::Mesh>(componentMesh(s.shape));
        parts.push_back(std::move(p));
    }
    const std::string cradle = cradleCarrier(model);
    const std::string table = tableEnd(model);
    const Vec3 centre = tableCentre(model, Vec3::Zero());
    for (const Component& c : machine.components) {
        ModelPart p;
        p.name = c.name;
        p.geometry = std::make_shared<const geom3d::Mesh>(componentMesh(c));
        switch (c.attach) {
        case Component::Attach::Head:
            p.carrier = model.spindleCarrier;
            p.offset = model.spindlePoint;
            p.colour = 0x9AA6B8;
            break;
        case Component::Attach::Table:
            p.carrier = table;
            p.offset = centre;
            p.colour = 0x8C949E;
            break;
        case Component::Attach::Cradle:
            p.carrier = cradle;
            p.offset = centre;
            p.colour = 0x737C88;
            break;
        }
        parts.push_back(std::move(p));
    }
    return parts;
}

ModelPart makePart(std::string name, const geom3d::Mesh& mesh) {
    ModelPart p;
    p.name = std::move(name);
    const std::string bytes = geom3d::writeStl(mesh);
    auto stored = std::make_shared<const geom3d::Mesh>(geom3d::readStl(bytes, p.name));
    p.mesh = sha256Hex(geom3d::writeStl(*stored));
    p.geometry = std::move(stored);
    return p;
}

ModelPart stepPart(std::string name, std::shared_ptr<const std::string> stepFile) {
    ModelPart p;
    p.name = std::move(name);
    p.step = sha256Hex(*stepFile);
    p.stepFile = std::move(stepFile);
    return p;
}

std::string partFileName(const ModelPart& part) {
    return part.exact() ? part.step + ".step" : part.mesh + ".stl";
}

std::string partFileBytes(const ModelPart& part) {
    if (part.exact()) {
        if (!part.stepFile) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("the STEP file of machine part '{}' is not loaded", part.name));
        }
        return *part.stepFile;
    }
    if (!part.geometry) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("the mesh of machine part '{}' is not loaded", part.name));
    }
    return geom3d::writeStl(*part.geometry);
}

void attachPartFiles(Machine& machine,
                     const std::function<std::optional<std::string>(const std::string&)>& read,
                     std::string_view sourceName) {
    for (ModelPart& p : machine.model.parts) {
        if (p.mesh.empty() && p.step.empty()) {
            continue;
        }
        const char* what = p.exact() ? "STEP file" : "mesh";
        std::optional<std::string> bytes = read(partFileName(p));
        if (!bytes) {
            throw Error(ErrorCode::Parse, fmt::format("{}: the {} of machine part '{}' is missing",
                                                      sourceName, what, p.name));
        }
        if (sha256Hex(*bytes) != (p.exact() ? p.step : p.mesh)) {
            throw Error(ErrorCode::Parse,
                        fmt::format("{}: the {} of machine part '{}' is corrupt (hash mismatch)",
                                    sourceName, what, p.name));
        }
        if (p.exact()) {
            p.stepFile = std::make_shared<const std::string>(std::move(*bytes));
            continue;
        }
        p.geometry = std::make_shared<const geom3d::Mesh>(
            geom3d::readStl(*bytes, fmt::format("{} (part '{}')", sourceName, p.name)));
    }
}

std::filesystem::path meshFolder(const std::filesystem::path& machineFile) {
    std::filesystem::path folder = machineFile;
    folder.replace_extension(".meshes");
    return folder;
}

void saveMachine(const Machine& machine, const std::filesystem::path& path) {
    const std::filesystem::path folder = meshFolder(path);
    std::vector<std::string> keep;
    for (const ModelPart& p : machine.model.parts) {
        if (p.mesh.empty() && p.step.empty()) {
            continue;
        }
        const std::string bytes = partFileBytes(p);
        if (keep.empty()) {
            std::error_code ec;
            std::filesystem::create_directories(folder, ec);
        }
        const std::string name = partFileName(p);
        const std::filesystem::path file = folder / name;
        std::error_code ec;
        if (!std::filesystem::exists(file, ec)) {
            writeFileAtomic(file, bytes);
        }
        keep.push_back(name);
    }
    writeFileAtomic(path, dumpJson(toJson(machine)));
    // Meshes no part uses any more.
    std::error_code ec;
    if (std::filesystem::is_directory(folder, ec)) {
        for (const auto& item : std::filesystem::directory_iterator(folder, ec)) {
            const std::u8string u8 = item.path().filename().u8string();
            const std::string name(u8.begin(), u8.end());
            const auto ext = item.path().extension();
            if ((ext == ".stl" || ext == ".step") &&
                std::find(keep.begin(), keep.end(), name) == keep.end()) {
                std::filesystem::remove(item.path(), ec);
            }
        }
        if (keep.empty()) {
            std::filesystem::remove(folder, ec);
        }
    }
}

} // namespace ocam::machine
