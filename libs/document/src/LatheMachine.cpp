// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// See docs/design-log/0067.

#include "ocam/document/LatheMachine.hpp"

#include "ocam/document/Generate.hpp"
#include "ocam/document/LatheFitting.hpp"
#include "ocam/document/PostSetup.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Units.hpp>
#include <ocam/geom2d/Clip.hpp>
#include <ocam/sim/LatheMachine.hpp>
#include <ocam/strategies/Common.hpp>
#include <ocam/tooling/Profile.hpp>
#include <ocam/tooling/ToolLibrary.hpp>
#include <ocam/tooling/TurningHolder.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>

namespace ocam::document {
namespace {

using machine::AxisPosition;
using machine::LatheStep;

/// A tool across the spindle has its shank along X; a tool along it has a bar along Z.
bool acrossByDefault(const tooling::Tool& tool, const Operation* op, int quadrant) {
    if (tooling::isTurningTool(tool.type)) {
        return tooling::turningSideOf(quadrant) != tooling::TurningSide::Inside;
    }
    return op && op->orientation && std::abs(operationFrame(*op).linear().col(2).z()) < 0.5;
}

/// How far a turning tool's body stands back from its outline [mm]: where it meets the insert
/// (the insert's top on the holder's, its sides on the pocket's), the insert's faces are in front,
/// so neither shows through the other in a view. The insert, what cuts, stays as it is.
constexpr double kBodyInset = 0.05;

/// A tool body's outline moved in by `inset` all round.
std::vector<Vec2> insetOutline(const geom2d::Polyline& outline, double inset) {
    geom2d::ClipOptions options;
    options.join = geom2d::JoinStyle::Miter;
    const geom2d::Regions in = geom2d::offset({geom2d::Region{outline, {}}}, -inset, options);
    if (in.empty()) {
        return outline.discretize(0.05); // thinner than the inset: as it is
    }
    const auto biggest = std::max_element(
        in.begin(), in.end(),
        [](const geom2d::Region& a, const geom2d::Region& b) { return a.area() < b.area(); });
    return biggest->outer.discretize(0.05);
}

/// The solids turned over across the tip's height (Y -> -Y): the tool face down, its outline in
/// the lathe plane as it was. Their ends swap so their faces still look out.
void faceDown(std::vector<sim::TurretTool::Solid>& solids) {
    for (sim::TurretTool::Solid& s : solids) {
        for (Vec3& p : s.prism.lower) {
            p.y() = -p.y();
        }
        for (Vec3& p : s.prism.upper) {
            p.y() = -p.y();
        }
        std::swap(s.prism.lower, s.prism.upper);
    }
}

/// The solids of a tool in its own frame (the tip at the origin): in the lathe plane for a turning
/// tool, up its axis (+Z) for one that turns or stands on the axis.
std::vector<sim::TurretTool::Solid> toolSolids(const tooling::Tool& tool,
                                               const tooling::Holder* holder, int q) {
    std::vector<sim::TurretTool::Solid> out;
    if (tooling::isTurningTool(tool.type)) {
        const tooling::TurningSide side = tooling::turningSideOf(q);
        const bool away = tooling::turningAwayFromChuck(q);
        geom2d::Polyline insert = tooling::turningShape(tool, side).outline;
        if (away) {
            insert = tooling::mirroredInZ(insert);
        }
        const double thick = tool.insert.thickness > 0.0 ? tool.insert.thickness : 4.0;
        out.push_back({"insert", sim::platePrism(insert.discretize(0.05), -thick, 0.0)});
        for (const tooling::TurningToolPart& part :
             tooling::turningToolParts(tool, holder, side, away)) {
            if (part.round) {
                geom2d::Aabb2 box;
                for (const Vec2& p : part.outline.discretize(0.05)) {
                    box.extend(p);
                }
                const Vec2 size = box.max - box.min;
                out.push_back({"holder", sim::roundPrism(Vec3(0.5 * (box.min.y() + box.max.y()),
                                                              0.0, box.min.x() + kBodyInset),
                                                         Vec3::UnitZ(), 0.5 * size.y() - kBodyInset,
                                                         size.x() - 2.0 * kBodyInset)});
            } else {
                out.push_back(
                    {"holder", sim::platePrism(insetOutline(part.outline, kBodyInset),
                                               part.low + kBodyInset, part.high - kBodyInset)});
            }
        }
        return out;
    }
    const sim::ToolShape shape = sim::ToolShape::fromTool(tool, holder);
    out.push_back({"tool", sim::roundPrism(Vec3::Zero(), Vec3::UnitZ(), shape.cutRadius(),
                                           shape.cuttingLength())});
    if (shape.shankRadius() > 0.0 && shape.visibleLength() > shape.cuttingLength()) {
        out.push_back(
            {"tool shank",
             sim::roundPrism(Vec3(0, 0, shape.cuttingLength()), Vec3::UnitZ(), shape.shankRadius(),
                             shape.visibleLength() - shape.cuttingLength())});
    }
    for (const sim::HolderBand& b : shape.holder()) {
        out.push_back({"tool holder",
                       sim::roundPrism(Vec3(0, 0, b.h0), Vec3::UnitZ(), b.radius, b.h1 - b.h0)});
    }
    return out;
}

/// How far the tool's own centre line lies from its tip: along Z for a tool across the spindle, in
/// X for one along it. By its largest solid: the shank, not the head.
double centreLineOf(const std::vector<sim::TurretTool::Solid>& solids, bool across) {
    double best = -1.0;
    double line = 0.0;
    for (const sim::TurretTool::Solid& s : solids) {
        geom3d::Aabb3 box;
        for (const Vec3& p : s.prism.lower) {
            box.extend(p);
        }
        for (const Vec3& p : s.prism.upper) {
            box.extend(p);
        }
        const Vec3 size = box.size();
        const double volume = size.x() * std::max(size.y(), 1.0) * size.z();
        if (volume > best) {
            best = volume;
            line = across ? 0.5 * (box.min.z() + box.max.z()) : 0.5 * (box.min.x() + box.max.x());
        }
    }
    return line;
}

/// Duration of a move in the machine's rapid rate [s].
double rapidTime(const machine::Machine& machine, const AxisPosition& a, const AxisPosition& b) {
    const double travel = std::max(
        {(b.linear - a.linear).cwiseAbs().maxCoeff(), std::abs(b.w - a.w), std::abs(b.z2 - a.z2)});
    return machine.rapidRate > 0.0 ? travel / machine.rapidRate : 0.0;
}

} // namespace

TurretLoad turretLoad(const Document& doc, const Setup& setup, const machine::Machine& machine,
                      const std::map<Uuid, int>* quadrants) {
    TurretLoad load;
    if (!machine.lathe) {
        return load;
    }
    const Project& project = doc.project();
    // A block or centre the machine does not take is said, and used as it is.
    for (const StationBlock& b : setup.turretBlocks) {
        if (const LatheFitting* block = project.findFitting(b.block)) {
            if (const auto why = mountMismatch(*block, machine, b.station)) {
                load.problems.push_back(fmt::format("station {}: {}", b.station, *why));
            }
        }
    }
    if (setup.tailstock && setup.tailstock->centre) {
        if (const LatheFitting* centre = project.findFitting(*setup.tailstock->centre)) {
            if (const auto why = mountMismatch(*centre, machine)) {
                load.problems.push_back(*why);
            }
        }
    }
    std::map<Uuid, TurretStation> byTool;
    auto quadrantOf = [&](const tooling::Tool& tool) {
        if (quadrants) {
            if (const auto it = quadrants->find(tool.id); it != quadrants->end()) {
                return it->second;
            }
        }
        return tool.insert.orientation;
    };
    auto entryFor = [&](const Uuid& id) -> const TurretEntry* {
        const auto it = std::find_if(setup.turret.begin(), setup.turret.end(),
                                     [&](const TurretEntry& e) { return e.tool == id; });
        return it == setup.turret.end() ? nullptr : &*it;
    };
    auto add = [&](const tooling::Tool& tool, const Operation* op) {
        auto [it, fresh] = byTool.try_emplace(tool.id);
        TurretStation& s = it->second;
        if (fresh) {
            s.station = tool.number;
            s.tool = tool.id;
            s.name = tool.name;
            s.across = acrossByDefault(tool, op, quadrantOf(tool));
        }
        s.used = s.used || op != nullptr;
    };
    for (const Operation& op : setup.operations) {
        if (!op.enabled || !op.toolId) {
            continue;
        }
        if (const tooling::Tool* tool = project.tools.findTool(*op.toolId)) {
            add(*tool, &op);
        }
    }
    for (const TurretEntry& e : setup.turret) {
        if (const tooling::Tool* tool = project.tools.findTool(e.tool)) {
            add(*tool, nullptr);
        } else {
            load.problems.push_back("a tool the turret lists is no longer in the project");
        }
    }
    const bool sub = onSubSpindle(setup);
    std::map<int, std::string> taken;
    for (auto& [id, s] : byTool) {
        const tooling::Tool& tool = *project.tools.findTool(id);
        const tooling::Holder* holder =
            tool.holderId ? project.tools.findHolder(*tool.holderId) : nullptr;
        const TurretEntry* entry = entryFor(id);
        if (entry && entry->across) {
            s.across = *entry->across;
            s.acrossSet = true;
        }
        const machine::LatheLayout& layout = *machine.lathe;
        const std::vector<sim::TurretTool::Solid> solids =
            toolSolids(tool, holder, quadrantOf(tool));
        const auto seated =
            std::find_if(setup.turretBlocks.begin(), setup.turretBlocks.end(),
                         [&](const StationBlock& b) { return b.station == s.station; });
        const LatheFitting* block =
            seated != setup.turretBlocks.end() ? project.findFitting(seated->block) : nullptr;
        if (block) {
            // Through the block: station to block to tool.
            s.block = block->id;
            const bool holdsAcross = blockAcross(*block);
            if (holdsAcross != s.across) {
                load.problems.push_back(fmt::format("'{}' lies {} the spindle, and the block on "
                                                    "station {}, '{}', holds tools {} it",
                                                    tool.name, s.across ? "across" : "along",
                                                    s.station, block->name,
                                                    holdsAcross ? "across" : "along"));
            }
            s.across = holdsAcross;
            s.acrossSet = false;
            s.overhang = entry && entry->overhang ? *entry->overhang : block->overhang;
            const Transform3 onStation = blockOnStation(*block, layout, sub);
            const double centre = centreLineOf(solids, s.across);
            const Vec3 line =
                s.across ? Vec3(0.0, 0.0, sub ? -centre : centre) : Vec3(centre, 0.0, 0.0);
            s.tipPoint = onStation * block->mount.point +
                         (onStation.linear() * block->mount.direction.normalized()) * s.overhang -
                         line;
            s.tip = Vec2(layout.turret.centre.x() - s.tipPoint.x(),
                         s.tipPoint.z() - layout.turret.centre.z());
            // Which way its rake face looks is the block's: the shank on the block's side of the
            // tip's height. The work's Y is the model's on the main spindle, turned over on the
            // sub-spindle's.
            const Vec3 up = onStation.linear() * blockUp(*block);
            s.faceDown = (sub ? -up.y() : up.y()) < 0.0;
            if (std::abs(s.tipPoint.y()) > 0.01 && !machine.hasYAxis()) {
                load.problems.push_back(
                    fmt::format("'{}' stands {:.3f} mm off the spindle's centre height in its "
                                "block, and machine "
                                "'{}' has no Y axis: check the block's tool mount",
                                tool.name, s.tipPoint.y(), machine.name));
            }
            // What the block takes, where the tool says what it is.
            // A square shank (an ISO turning holder's) or a round one (a boring bar's, a drill's or
            // a mill's); a plain stick says neither.
            std::optional<std::pair<bool, double>> shank; // round, size
            if (holder && holder->iso) {
                shank = std::pair{holder->iso->boringBar, holder->iso->height};
            } else if (!tooling::isTurningTool(tool.type)) {
                shank = std::pair{true, tool.mill.shankDiameter > 0.0 ? tool.mill.shankDiameter
                                                                      : tool.mill.diameter};
            }
            if (shank) {
                const auto [round, size] = *shank;
                const char* what = round ? "round shank" : "square shank";
                if (round ? !block->takesRound() : !block->takesSquare()) {
                    load.problems.push_back(fmt::format(
                        "'{}' has a {}, and the block on station {}, '{}', takes only {}",
                        tool.name, what, s.station, block->name,
                        round ? "square shanks" : "round shanks"));
                } else if (!round && block->squareShank &&
                           std::abs(*block->squareShank - size) > 0.01) {
                    // A square slot takes its own size.
                    load.problems.push_back(fmt::format("'{}' has a {:.1f} mm {}, and the block on "
                                                        "station {}, '{}', takes {:.1f} mm",
                                                        tool.name, size, what, s.station,
                                                        block->name, *block->squareShank));
                } else if (round && block->roundBar && size > *block->roundBar + 0.01) {
                    // A bore is the largest it takes (a milling unit's collet, a boring holder's
                    // sleeve): only a larger shank does not go in, and a smaller one is not said.
                    load.problems.push_back(fmt::format(
                        "'{}' has a {:.1f} mm {}, larger than the {:.1f} mm bore of the block on "
                        "station {}, '{}'",
                        tool.name, size, what, *block->roundBar, s.station, block->name));
                }
            }
        } else {
            machine::StationMount mount;
            mount.across = s.across;
            mount.centreLine = centreLineOf(solids, s.across);
            mount.towardsPlusZ = sub;
            s.tip = machine::defaultTipFromCentre(layout, mount);
            s.tipPoint = machine::tipPoint(layout, s.tip);
        }
        // A tip the setup typed, from a tool setter: that is where it is.
        if (entry && entry->tip) {
            s.tip = *entry->tip;
            s.tipSet = true;
            s.tipPoint = machine::tipPoint(layout, s.tip);
        }
        if (s.station < 1 || s.station > machine.toolCapacity) {
            load.problems.push_back(fmt::format(
                "tool {} '{}' is at station {}, and machine '{}' has stations 1 to {}: change the "
                "tool's number",
                tool.number, tool.name, s.station, machine.name, machine.toolCapacity));
        }
        if (const auto [t, inserted] = taken.try_emplace(s.station, tool.name); !inserted) {
            load.problems.push_back(fmt::format(
                "'{}' and '{}' are both at station {}: their numbers call one place in the turret",
                t->second, tool.name, s.station));
        }
        load.stations.push_back(s);
    }
    std::sort(load.stations.begin(), load.stations.end(),
              [](const TurretStation& a, const TurretStation& b) { return a.station < b.station; });
    for (const StationBlock& b : setup.turretBlocks) {
        if (b.station < 1 || b.station > machine.toolCapacity) {
            load.problems.push_back(
                fmt::format("a block is on station {}, and machine '{}' has stations 1 to {}",
                            b.station, machine.name, machine.toolCapacity));
        }
    }
    return load;
}

std::vector<sim::TurretBlock> turretBlocks(const Document& doc, const Setup& setup,
                                           const machine::Machine& machine,
                                           std::vector<std::string>* problems) {
    std::vector<sim::TurretBlock> out;
    if (!machine.lathe) {
        return out;
    }
    const bool sub = onSubSpindle(setup);
    for (const StationBlock& b : setup.turretBlocks) {
        const LatheFitting* block = doc.project().findFitting(b.block);
        const auto file = block ? doc.modelFile(block->sha256) : nullptr;
        if (!file) {
            if (problems) {
                problems->push_back(
                    fmt::format("the block on station {} is not in the project", b.station));
            }
            continue;
        }
        sim::TurretBlock t;
        t.station = b.station;
        t.name = fmt::format("Station {} block '{}'", b.station, block->name);
        t.colour = block->colour;
        t.mesh = stepMesh({block->sha256, file}, machine.model.meshTolerance, block->name);
        t.placement = blockOnStation(*block, *machine.lathe, sub);
        out.push_back(std::move(t));
    }
    return out;
}

bool onSubSpindle(const Setup& setup) noexcept {
    return setup.placement && setup.placement->carrier == "C2";
}

const Operation* transferOperation(const Setup& main) {
    for (const Operation& op : main.operations) {
        if (op.enabled && op.type == "spindle_transfer") {
            return &op;
        }
    }
    return nullptr;
}

const Setup* transferTarget(const Project& project, const Setup& main) {
    for (const Setup& s : project.setups) {
        if (onSubSpindle(s) && s.stock.kind == StockKind::FromSetup && s.stock.sourceSetup &&
            *s.stock.sourceSetup == main.id && s.machine == main.machine) {
            return &s;
        }
    }
    return nullptr;
}

const Operation* transferInto(const Project& project, const Setup& sub, const Setup** from) {
    if (!onSubSpindle(sub)) {
        return nullptr;
    }
    const std::vector<const Setup*> sources = stockSources(project, sub);
    if (sources.empty() || onSubSpindle(*sources.back()) ||
        sources.back()->machine != sub.machine ||
        transferTarget(project, *sources.back()) != &sub) {
        return nullptr;
    }
    if (from) {
        *from = sources.back();
    }
    return transferOperation(*sources.back());
}

double transferGrip(const Operation& transfer, const machine::Machine& machine) {
    // 0 is the machine's grip: the operation's parameter says so (its ParamDef).
    const double own =
        strategies::ParamReader(*strategies::findStrategy("spindle_transfer"), transfer.parameters)
            .number("grip");
    if (own > 0.0) {
        return own;
    }
    return machine.lathe && machine.lathe->subSpindle ? machine.lathe->subSpindle->grip : 0.0;
}

std::optional<double> pickOffZ2(const Document& doc, const Setup& from, const Setup& sub,
                                const machine::Machine& machine, CancelToken& cancel) {
    const LatheGeometry main = latheGeometry(doc, from, cancel);
    const machine::WorkMount fromMount = latheWorkMount(doc, from, machine, main.stock);
    const LatheGeometry taken = latheGeometry(doc, sub, cancel);
    const machine::WorkMount subMount = latheWorkMount(doc, sub, machine, taken.stock);
    const Transform3 mainFromProject = fromMount.wcs * from.wcs.toParent().inverse();
    const Transform3 subFromProject = subMount.wcs * sub.wcs.toParent().inverse();
    const Mat3 turned = mainFromProject.linear() - subFromProject.linear();
    const Vec3 gap = mainFromProject.translation() - subFromProject.translation();
    if (turned.cwiseAbs().maxCoeff() > 1e-6 || gap.head<2>().norm() > 1e-6) {
        return std::nullopt;
    }
    return gap.z();
}

bool usesMachineChuck(const Setup& setup) noexcept {
    return !setup.workholding && setup.fixtures.empty();
}

std::optional<std::string> quillBeyondTravel(const Setup& setup, const machine::Machine& machine) {
    if (!setup.tailstock || !machine.lathe || !machine.lathe->tailstock) {
        return std::nullopt;
    }
    const double travel = machine.lathe->tailstock->quillTravel;
    const double out = setup.tailstock->quillLength;
    if (travel <= 0.0 || out <= travel) {
        return std::nullopt;
    }
    return fmt::format("the tailstock's quill is out {:g} mm, and machine '{}''s quill goes out "
                       "{:g} mm at most: set Quill length under the setup's Tailstock to {:g} mm "
                       "or less",
                       out, machine.name, travel, travel);
}

machine::WorkMount latheWorkMount(const Document& doc, const Setup& setup,
                                  const machine::Machine& machine, const geom2d::Regions& stock) {
    const machine::LatheLayout& l = *machine.lathe;
    const bool sub = onSubSpindle(setup) && l.subSpindle;
    // The held end, WCS Z: the back of what the setup holds it in, else of the stock.
    double back = std::numeric_limits<double>::infinity();
    const bool ownChuck = !usesMachineChuck(setup);
    if (ownChuck) {
        std::set<std::string> supports;
        for (const LatheSupport& s : latheSupports(doc, setup)) {
            supports.insert(s.name);
        }
        for (const sim::LatheBody& b : latheBodies(doc, setup, 0.0)) {
            if (supports.count(b.name)) {
                continue;
            }
            for (const geom2d::Region& r : b.area) {
                back = std::min(back, r.bounds().min.x());
            }
        }
    }
    double seat = 0.0; // where the held end goes, model frame
    if (!std::isfinite(back)) {
        for (const geom2d::Region& r : stock) {
            back = std::min(back, r.bounds().min.x());
        }
        if (!std::isfinite(back)) {
            back = 0.0;
        }
        // The transfer's grip, when one hands the part over (ADR-0007).
        const Operation* transfer = sub ? transferInto(doc.project(), setup) : nullptr;
        const double grip =
            sub ? (transfer ? transferGrip(*transfer, machine) : l.subSpindle->grip) : 0.0;
        seat = sub ? l.subSpindle->nose - l.subSpindle->chuckLength + grip
                   : l.workholdingMount + l.chuckLength - l.chuckGrip;
    } else if (sub) {
        seat = l.subSpindle->nose;
    } else {
        seat = l.workholdingMount; // the setup's own chuck's back on the mount
    }
    const double along = setup.placement ? setup.placement->offset.z() : 0.0;
    machine::WorkMount mount;
    mount.carrier = sub ? "C2" : "C";
    mount.wcs = Transform3::Identity();
    if (sub) {
        // The WCS's Z towards the main spindle: the part as the main spindle's setup left it.
        mount.wcs.linear() = Eigen::AngleAxisd(kPi, Vec3::UnitX()).toRotationMatrix();
        mount.wcs.translation() = Vec3(0.0, 0.0, seat + back + along);
    } else {
        mount.wcs.translation() = Vec3(0.0, 0.0, seat - back + along);
    }
    return mount;
}

Uuid tailstockSupportId() {
    return *Uuid::parse("0a5e1e0d-7a11-4570-8c6b-000000000001");
}

ExampleTool exampleTool(const LatheFitting& block) {
    // Placeholders for a size the block does not state: only for a picture.
    constexpr double kSquare = 20.0;
    constexpr double kBar = 16.0;
    ExampleTool out;
    const Vec3 d = block.mount.direction.normalized(); // towards the tip
    const Vec3 s = block.seat.direction.normalized();  // out of the turret
    const Vec3 w = towardsWork(block);
    const Vec3 h = blockUp(block); // up, from the shank to the tip's height
    const Vec3 m = block.mount.point;
    const auto prism = [](const std::vector<Vec3>& outline, const Vec3& from, const Vec3& to) {
        sim::ConvexPrism p;
        for (const Vec3& v : outline) {
            p.lower.push_back(from + v);
            p.upper.push_back(to + v);
        }
        return p;
    };
    if (blockAcross(block)) {
        // A turning holder: its shank below the tip's height, the tip at its side towards the
        // work.
        const double a = block.squareShank.value_or(kSquare);
        out.size = a;
        out.sizeStated = block.squareShank.has_value();
        out.tip = m + d * block.overhang + w * (a / 2.0);
        const Vec3 front = m + d * std::max(0.0, block.overhang - 0.8 * a);
        // Back into the block, short of the turret's face.
        const double depth = (m - block.seat.point).dot(d);
        const double back = depth > a ? std::min(3.0 * a, depth - 1.0) : 3.0 * a;
        out.shank.push_back(
            prism({w * (-a / 2.0), w * (a / 2.0), w * (a / 2.0) - h * a, w * (-a / 2.0) - h * a},
                  m - d * back, front));
        const Vec3 t = out.tip;
        out.insert.push_back(prism({t - h * (0.25 * a), t - d * (0.9 * a) - h * (0.25 * a),
                                    t - d * (0.45 * a) - w * (0.7 * a) - h * (0.25 * a)},
                                   Vec3::Zero(), h * (0.25 * a)));
        return out;
    }
    // A boring bar: its axis through the tool mount, the tip out of its side.
    const double r = block.roundBar.value_or(kBar) / 2.0;
    out.size = 2.0 * r;
    out.round = true;
    out.sizeStated = block.roundBar.has_value();
    out.tip = m + d * block.overhang - s * (1.1 * r);
    const Vec3 back = m - d * std::max(60.0, 6.0 * r);
    const double length = (m - back).norm() + std::max(0.0, block.overhang - 1.2 * r);
    out.shank.push_back(sim::roundPrism(back, d, r, length));
    const Vec3 t = out.tip;
    out.insert.push_back(prism({t - h * (0.3 * r), t - d * (1.2 * r) - h * (0.3 * r),
                                t - d * (0.5 * r) + s * (0.9 * r) - h * (0.3 * r)},
                               Vec3::Zero(), h * (0.6 * r)));
    return out;
}

Uuid tailstockCentreSupportId() {
    return *Uuid::parse("0a5e1e0d-7a11-4570-8c6b-000000000003");
}

std::string turretKey(const Document& doc, const Setup& setup) {
    tooling::ToolLibrary tools;
    tools.id = doc.project().tools.id;
    for (const TurretEntry& e : setup.turret) {
        const tooling::Tool* tool = doc.project().tools.findTool(e.tool);
        if (!tool) {
            continue;
        }
        tools.tools.push_back(*tool);
        if (tool->holderId) {
            if (const tooling::Holder* h = doc.project().tools.findHolder(*tool->holderId)) {
                tools.holders.push_back(*h);
            }
        }
    }
    return tools.tools.empty() ? std::string() : tooling::toJson(tools).dump();
}

std::vector<sim::TurretTool> turretTools(const Document& doc, const TurretLoad& load,
                                         const machine::WorkMount& mount,
                                         const std::map<Uuid, int>* quadrants) {
    const Project& project = doc.project();
    std::vector<sim::TurretTool> out;
    for (const TurretStation& s : load.stations) {
        const tooling::Tool* found = project.tools.findTool(s.tool);
        if (!found) {
            continue;
        }
        const tooling::Tool& tool = *found;
        const tooling::Holder* holder =
            tool.holderId ? project.tools.findHolder(*tool.holderId) : nullptr;
        sim::TurretTool t;
        t.station = s.station;
        t.name = fmt::format("T{:02d} {}", s.station, tool.name);
        int quadrant = tool.insert.orientation;
        if (quadrants) {
            if (const auto q = quadrants->find(s.tool); q != quadrants->end()) {
                quadrant = q->second;
            }
        }
        t.solids = toolSolids(tool, holder, quadrant);
        if (s.faceDown && tooling::isTurningTool(tool.type)) {
            faceDown(t.solids);
        }
        Transform3 toModel = Transform3::Identity();
        toModel.translation() = s.tipPoint;
        toModel.linear() = mount.wcs.linear();
        if (s.across && !tooling::isTurningTool(tool.type)) {
            // Its axis, +Z of its frame, away from the spindle: +X.
            toModel.linear() =
                toModel.linear() * Eigen::AngleAxisd(kPi / 2.0, Vec3::UnitY()).toRotationMatrix();
        }
        t.placement = toModel;
        out.push_back(std::move(t));
    }
    return out;
}

std::vector<sim::LatheWorkBody> latheWorkBodies(const Document& doc, const Setup& setup,
                                                const machine::Machine& machine,
                                                const machine::WorkMount& mount, double* tailstockW,
                                                std::vector<std::string>* warnings) {
    const Project& project = doc.project();
    const machine::MachineModel model = machine::effectiveModel(machine);
    const bool hasW = model.axis("W") != nullptr;
    double wWork = 0.0;
    if (machine.lathe && setup.tailstock && hasW && machine.lathe->tailstock) {
        const auto& t = *setup.tailstock;
        const LatheFitting* centre = t.centre ? project.findFitting(*t.centre) : nullptr;
        const double point =
            centre ? centreLength(*centre) : t.centreDiameter / 2.0 / std::tan(t.centreAngle / 2.0);
        const Vec3 quillBack = mount.wcs * Vec3(0.0, 0.0, t.tipZ + point + t.quillLength);
        wWork = quillBack.z() - machine.lathe->tailstock->z;
    }
    if (tailstockW) {
        *tailstockW = wWork;
    }
    std::vector<sim::LatheWorkBody> bodies;
    if (hasFixtures(setup)) {
        try {
            for (sim::LatheBody& b : latheBodies(doc, setup, 0.0)) {
                sim::LatheWorkBody body;
                body.name = b.name;
                body.area = std::move(b.area);
                body.frame = mount.wcs;
                if (body.name == "Tailstock" || body.name == "Tailstock centre") {
                    if (hasW) {
                        body.carrier = "W";
                        body.frame = Eigen::Translation3d(0.0, 0.0, -wWork) * mount.wcs;
                    }
                } else if (body.name != "Steady rest") {
                    body.carrier = mount.carrier;
                }
                bodies.push_back(std::move(body));
            }
        } catch (const Error& e) {
            if (warnings) {
                warnings->push_back(
                    fmt::format("the chuck is not checked against the turret: {}", e.what()));
            }
        }
    }
    return bodies;
}

LatheMachineParts latheMachineParts(const machine::Machine& machine,
                                    const machine::MachineModel& model,
                                    const std::vector<sim::TurretBlock>& blocks,
                                    const std::vector<sim::TurretTool>& tools, bool machineChuck) {
    LatheMachineParts out;
    out.parts = machine.model.parts;
    for (machine::ModelPart& p : machine::componentParts(machine, model)) {
        if (machineChuck || p.name != machine::kLatheChuckPart) {
            out.parts.push_back(std::move(p));
        }
    }
    out.stations.assign(out.parts.size(), 0);
    out.blocksFrom = out.parts.size();
    if (!machine.lathe) {
        out.toolsFrom = out.parts.size();
        return out;
    }
    // The blocks and tools ride on the turret, each where its station stands at T 0.
    const auto atZero = [&](int station, const Transform3& placement) {
        return machine::turretTurn(machine.lathe->turret,
                                   -machine::stationAngle(machine, station)) *
               placement;
    };
    for (const sim::TurretBlock& block : blocks) {
        machine::ModelPart p;
        p.name = block.name;
        p.carrier = model.spindleCarrier;
        p.colour = block.colour;
        geom3d::Mesh mesh = *block.mesh;
        mesh.transform(atZero(block.station, block.placement));
        p.geometry = std::make_shared<const geom3d::Mesh>(std::move(mesh));
        out.parts.push_back(std::move(p));
        out.stations.push_back(block.station);
    }
    out.toolsFrom = out.parts.size();
    for (const sim::TurretTool& tool : tools) {
        const Transform3 place = atZero(tool.station, tool.placement);
        for (const sim::TurretTool::Solid& solid : tool.solids) {
            machine::ModelPart p;
            p.name = tool.name + ", " + solid.part;
            p.carrier = model.spindleCarrier;
            // What cuts in gold, the rest of the tool black: each stands out from its block.
            p.colour = solid.part == "insert" || solid.part == "tool" ? 0xD4A82A : 0x26282B;
            geom3d::Mesh mesh = sim::prismMesh(solid.prism);
            mesh.transform(place);
            p.geometry = std::make_shared<const geom3d::Mesh>(std::move(mesh));
            out.parts.push_back(std::move(p));
            out.stations.push_back(tool.station);
        }
    }
    return out;
}

void simulateLatheMachine(const Document& doc, const Setup& setup, const machine::Machine& machine,
                          SetupSimulation& out, CancelToken& cancel) {
    const Project& project = doc.project();
    const machine::MachineModel model = machine::effectiveModel(machine);
    out.machine = std::make_shared<const machine::Machine>(machine);
    // Which way each turning tool cuts, as its first toolpath says.
    std::map<Uuid, int> quadrants;
    for (const OperationSimulation& os : out.operations) {
        const auto op = std::find_if(setup.operations.begin(), setup.operations.end(),
                                     [&](const Operation& o) { return o.id == os.operationId; });
        if (op != setup.operations.end() && op->toolId && os.toolpath.tipQuadrant != 0) {
            quadrants.try_emplace(*op->toolId, os.toolpath.tipQuadrant);
        }
    }
    const TurretLoad load = turretLoad(doc, setup, machine, &quadrants);
    out.warnings.insert(out.warnings.end(), load.problems.begin(), load.problems.end());
    if (const auto why = quillBeyondTravel(setup, machine)) {
        out.warnings.push_back(*why);
    }
    out.turretBlocks = turretBlocks(doc, setup, machine, &out.warnings);
    if (onSubSpindle(setup) && !machine.lathe->subSpindle) {
        out.warnings.push_back(fmt::format(
            "the setup is on a sub-spindle, and machine '{}' has none: it is simulated on the main "
            "spindle",
            machine.name));
    }
    const machine::LatheLayout& layout = *machine.lathe;
    const machine::WorkMount mount = latheWorkMount(
        doc, setup, machine, out.latheInitial ? out.latheInitial->profile() : geom2d::Regions{});
    out.machineChuck = usesMachineChuck(setup) || onSubSpindle(setup);

    // The tools, on the turret in the working position.
    out.turretTools = turretTools(doc, load, mount, &quadrants);
    std::map<int, Vec3> tips;
    for (const TurretStation& s : load.stations) {
        tips[s.station] = s.tipPoint;
    }

    // The setup's bodies: the chuck on the spindle, the tailstock on W, the rest where they stand.
    const bool hasW = model.axis("W") != nullptr;
    double wWork = 0.0;
    std::vector<sim::LatheWorkBody> bodies =
        latheWorkBodies(doc, setup, machine, mount, &wWork, &out.warnings);
    const sim::LatheMachineCheck check(machine, mount, out.turretTools, std::move(bodies),
                                       out.partRegions, out.machineChuck, out.turretBlocks);

    // Z2 while it cuts, and how it gets there: to take the part, where the work on the main spindle
    // and on this one are the same in the project.
    const bool sub = onSubSpindle(setup) && layout.subSpindle;
    std::vector<LatheStep> transfer;
    AxisPosition home;
    if (sub) {
        const double workZ2 = layout.subSpindle->workZ2;
        // Taken over by a transfer (ADR-0007): in to where the part in the two setups is the same
        // place in the project, then back. Without one the work is loaded by hand.
        bool taken = false;
        const Setup* fromSetup = nullptr;
        if (transferInto(project, setup, &fromSetup) && fromSetup) {
            CancelToken never;
            if (const auto pick = pickOffZ2(doc, *fromSetup, setup, machine, never)) {
                AxisPosition take = home;
                take.z2 = *pick;
                AxisPosition back = home;
                back.z2 = workZ2;
                LatheStep in{LatheStep::Kind::Transfer, take, rapidTime(machine, home, take), true};
                LatheStep outOf{LatheStep::Kind::Transfer, back, rapidTime(machine, take, back),
                                false};
                transfer = {in, outOf};
                taken = true;
            } else {
                out.warnings.push_back(fmt::format(
                    "the part on the sub-spindle does not line up with where setup '{}' leaves it "
                    "on the main one, so the transfer is not simulated: set this setup's WCS with "
                    "Z "
                    "turned round",
                    fromSetup->name));
            }
        }
        if (!taken) {
            AxisPosition back = home;
            back.z2 = workZ2;
            transfer = {{LatheStep::Kind::Transfer, back, rapidTime(machine, home, back), false}};
        }
    }

    AxisPosition last = home;
    int station = 0;
    std::optional<Vec3> lastEnd; // where the last toolpath left the tool, as Lower has it
    for (std::size_t i = 0; i < out.operations.size(); ++i) {
        OperationSimulation& os = out.operations[i];
        cancel.throwIfCancelled();
        if (!os.error.empty() || os.toolpath.moves.empty()) {
            continue;
        }
        const auto op = std::find_if(setup.operations.begin(), setup.operations.end(),
                                     [&](const Operation& o) { return o.id == os.operationId; });
        const tooling::Tool* tool = op != setup.operations.end() && op->toolId
                                        ? project.tools.findTool(*op->toolId)
                                        : nullptr;
        if (!tool || op->type == "turn_soft_jaws") {
            continue; // boring soft jaws: they are what it cuts (DL-0069)
        }
        machine::LatheMotion motion;
        motion.station = tool->number;
        motion.tip = tips.count(tool->number) ? tips[tool->number] : Vec3(layout.turret.centre);
        motion.held = last;
        motion.held.linear = Vec3::Zero();
        motion.held.rotary = {};
        motion.start = last;
        motion.start.rotary = {};
        AxisPosition at = motion.start;
        auto step = [&](LatheStep::Kind kind, const AxisPosition& to, double duration,
                        bool workStays = false) {
            motion.steps.push_back({kind, to, duration, workStays});
            at = to;
        };
        if (station == 0) {
            // The program's start: the machine at home, this tool already working.
            at.t = machine::stationAngle(machine, tool->number);
            motion.start = at;
            for (const LatheStep& t : transfer) {
                motion.steps.push_back(t);
                motion.steps.back().to.t = at.t;
            }
            if (!motion.steps.empty()) {
                at = motion.steps.back().to;
            }
            if (setup.tailstock && hasW && layout.tailstock) {
                AxisPosition to = at;
                to.w = wWork;
                step(LatheStep::Kind::Tailstock, to, rapidTime(machine, at, to));
            }
        } else if (tool->number != station) {
            // What the program does at a tool change: to home if the machine says so, then the T
            // word. The linear axes stay where they are for the index.
            if (layout.turret.indexAtHome) {
                AxisPosition to = at;
                to.linear.x() = 0.0;
                to.linear.y() = 0.0;
                step(LatheStep::Kind::Rapid, to, rapidTime(machine, at, to));
                to.linear.z() = 0.0;
                step(LatheStep::Kind::Rapid, to, rapidTime(machine, at, to));
            }
            AxisPosition to = at;
            to.t = machine::indexTo(machine, at.t, tool->number);
            step(LatheStep::Kind::Index, to, machine.toolChangeTime);
        }
        motion.held = at;
        motion.held.linear = Vec3::Zero();
        // The first position, as the program's rapids go: along Z, then in on X (and Y). From home
        // nothing is in the way of Z; approachOutFirst says when it would be otherwise. A spindle's
        // C at the start comes with it.
        const bool wentHome = station != 0 && tool->number != station && layout.turret.indexAtHome;
        const bool outFirst =
            lastEnd && !wentHome && machine::approachOutFirst(*lastEnd, os.toolpath.start);
        if (const auto first =
                machine::latheAxes(model, motion, mount, os.toolpath, os.toolpath.start)) {
            AxisPosition to = at;
            if (outFirst) {
                to.linear.x() = first->linear.x();
                to.linear.y() = first->linear.y();
                step(LatheStep::Kind::Rapid, to, rapidTime(machine, at, to));
                to.linear.z() = first->linear.z();
                step(LatheStep::Kind::Rapid, to, rapidTime(machine, at, to));
            } else {
                to.linear.z() = first->linear.z();
                step(LatheStep::Kind::Rapid, to, rapidTime(machine, at, to));
                to.linear.x() = first->linear.x();
                to.linear.y() = first->linear.y();
                step(LatheStep::Kind::Rapid, to, rapidTime(machine, at, to));
            }
        }
        os.mount = mount;

        // The stock as this operation goes: only where it is needed.
        const auto before = out.latheBefore(i);
        sim::LatheStock moving = before ? *before : sim::LatheStock{};
        std::size_t done = 0;
        sim::LatheStockTrack track;
        track.start = moving.profile();
        const bool cuts = !os.insert.isEmpty();
        sim::LatheStock pulledNow;
        if (os.toolpath.strategy == "bar_pull" && os.latheAfter) {
            // The bar comes out as it goes (DL-0091): everywhere it will be, then where it is.
            track.start = geom2d::unite(moving.profile(), os.latheAfter->profile());
            track.at = [&](std::size_t move) -> const geom2d::Regions& {
                pulledNow = moving.pulled(sim::barPulledBy(os.toolpath, move));
                return pulledNow.profile();
            };
        } else if (cuts) {
            track.at = [&](std::size_t move) -> const geom2d::Regions& {
                if (move > done) {
                    sim::simulateTurning(moving, os.insert, os.toolpath, done, move, &cancel);
                    done = move;
                }
                return moving.profile();
            };
        }
        os.machineCollisions = check.check(motion, os.toolpath, track, &cancel);
        // Where this operation leaves the machine.
        Vec3 end = os.toolpath.start;
        for (const toolpath::Move& m : os.toolpath.moves) {
            if (m.kind != toolpath::MoveKind::Dwell) {
                end = m.end;
            }
        }
        const auto finish = machine::latheAxes(model, motion, mount, os.toolpath, end);
        last = motion.held;
        if (finish) {
            last.linear = finish->linear;
        }
        station = tool->number;
        lastEnd = end;
        os.latheMotion = std::move(motion);
    }
}

} // namespace ocam::document
