// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The lathe's machine check (DL-0067): the turret and every tool in it, moved by the machine's
// kinematic tree through a toolpath and the moves between operations, against the enclosure, the
// spindle's chuck, the work, the tailstock and whatever else is on the machine. What turns with a
// spindle sweeps a solid of revolution, so it is checked as one, in the lathe plane; the rest as on
// a mill (MachineCheck.hpp, DL-0037).

#pragma once

#include "ocam/sim/MachineCheck.hpp"

#include <ocam/foundation/Cancel.hpp>
#include <ocam/geom2d/Region.hpp>
#include <ocam/geom3d/Mesh.hpp>
#include <ocam/machine/Turret.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ocam::sim {

/// A convex polygon extruded: `lower[i]` to `upper[i]`. All a tool on a turret is made of.
struct ConvexPrism {
    std::vector<Vec3> lower;
    std::vector<Vec3> upper;
};

/// A convex outline in the lathe plane (u = Z, v = X), from Y `low` to `high`. A shape that is not
/// convex is taken by its hull.
ConvexPrism platePrism(const std::vector<Vec2>& outline, double low, double high);
/// A round bar about the line through `base` along `axis`: the prism around the circle, so it is
/// never smaller than the bar.
ConvexPrism roundPrism(const Vec3& base, const Vec3& axis, double radius, double length,
                       int sides = 24);
geom3d::Mesh prismMesh(const ConvexPrism& prism);

/// A tool in a turret's station. Its frame is the WCS turned to the tool's mount, its tip at the
/// origin; `placement` puts it on the turret in the working position, at machine zero.
struct TurretTool {
    int station = 0;
    std::string name; ///< As reported: "T03 Boring bar".
    Transform3 placement = Transform3::Identity();
    struct Solid {
        std::string part; ///< "insert", "holder", "tool", "tool shank", "tool holder".
        ConvexPrism prism;
    };
    std::vector<Solid> solids;
};

/// A tooling block on a turret's station (DL-0068): rides along and is checked like the turret.
struct TurretBlock {
    int station = 0;
    std::string name; ///< As reported.
    std::uint32_t colour = 0x7C8690;
    std::shared_ptr<const geom3d::Mesh> mesh;
    /// Its mesh -> the model frame at machine zero, in the working position of its station.
    Transform3 placement = Transform3::Identity();
};

/// Something turning with a spindle or standing on the spindle's axis, by the area it sweeps.
struct LatheWorkBody {
    std::string name;
    geom2d::Regions area; ///< Lathe plane (u = its frame's Z, v = radius about it).
    std::string carrier;  ///< What carries it.
    /// Its frame at machine zero, model frame, Z along the axis it turns about.
    Transform3 frame = Transform3::Identity();
};

/// The stock along an operation: as it starts, and as it stands at a move. Material only goes, so
/// the later stock is only looked at where the first one is met.
struct LatheStockTrack {
    geom2d::Regions start;
    std::function<const geom2d::Regions&(std::size_t move)> at;
};

/// A setup on a lathe with a layout: everything that does not change from operation to operation,
/// its hierarchies and swept areas made once.
class LatheMachineCheck {
public:
    /// `bodies`: the setup's chuck, tailstock and steady rest. `partArea` tells the part from the
    /// stock in what is reported. `withMachineChuck`: the machine's own chuck is the one holding
    /// the work (false: the setup brings its own).
    LatheMachineCheck(const machine::Machine& machine, const machine::WorkMount& mount,
                      std::vector<TurretTool> tools, std::vector<LatheWorkBody> bodies,
                      geom2d::Regions partArea, bool withMachineChuck,
                      const std::vector<TurretBlock>& blocks = {});
    ~LatheMachineCheck();
    LatheMachineCheck(const LatheMachineCheck&) = delete;
    LatheMachineCheck& operator=(const LatheMachineCheck&) = delete;

    /// The moves before an operation's toolpath (motion.steps), reported as `during`, then its
    /// moves. The tool at `motion.station` is the one cutting: its own contacts with the stock and
    /// the setup's bodies are the turning checks' and the milling simulation's; here it only
    /// meets the machine. Every other tool, and every tool before the first move, meets it all.
    std::vector<MachineCollision> check(const machine::LatheMotion& motion,
                                        const toolpath::Toolpath& path,
                                        const LatheStockTrack& stock,
                                        CancelToken* cancel = nullptr) const;

    /// How many solids ride on the turret, tools and turret together (for the design log's
    /// numbers).
    std::size_t turretSolids() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace ocam::sim
