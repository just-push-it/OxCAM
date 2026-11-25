// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A lathe setup on a machine with a layout (DL-0067): what is in the turret, where the work sits on
// the spindle, and how the machine moves through the setup's operations.

#pragma once

#include "ocam/document/Document.hpp"
#include "ocam/document/Simulate.hpp"

#include <ocam/sim/LatheMachine.hpp>

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ocam::document {

/// A tool in the turret, as a setup has it. A tool's number is its station: it is the T word the
/// program calls it by.
struct TurretStation {
    int station = 0;
    Uuid tool;
    std::string name;
    bool used = false;   ///< An operation of the setup uses it (else it just rides along).
    bool across = false; ///< Across the spindle, or along it (a bar, a drill).
    bool acrossSet = false;
    Vec2 tip = Vec2::Zero(); ///< From the turret's centre: (towards the spindle, along Z) [mm].
    bool tipSet = false;     ///< `tip` is the setup's rather than the turret's for such a tool.
    /// The tip in the working position at machine zero, model frame (off the centre height only
    /// if a block puts it there).
    Vec3 tipPoint = Vec3::Zero();
    std::optional<Uuid> block; ///< The block it is held in (DL-0068).
    double overhang = 0.0;     ///< With a block: how far the tip stands beyond its tool mount.
    /// Its block holds it with its rake face looking down the work's Y (blockUp), as a rear
    /// turret does: a turning tool's shank above its tip's height.
    bool faceDown = false;
};

struct TurretLoad {
    std::vector<TurretStation> stations; ///< By station.
    /// Two tools at one station, a station it does not have, a block or centre the machine does
    /// not take.
    std::vector<std::string> problems;
};

/// The turret of a lathe setup: the tools its operations use and those it says ride along
/// (Setup::turret). Empty on a machine without a layout. `quadrants`, where known from the
/// toolpaths, says which way a turning tool cuts; else its insert's tip orientation does.
TurretLoad turretLoad(const Document& doc, const Setup& setup, const machine::Machine& machine,
                      const std::map<Uuid, int>* quadrants = nullptr);

/// Whether the setup's work sits on the sub-spindle (its placement's carrier is C2).
bool onSubSpindle(const Setup& setup) noexcept;

/// A main-spindle setup's transfer to the sub-spindle (ADR-0007): its enabled
/// `spindle_transfer` operation, if it has one.
const Operation* transferOperation(const Setup& main);
/// The setup a transfer of `main` hands its part to: the first setup on the sub-spindle of the same
/// machine whose stock comes from `main`; nullptr if none.
const Setup* transferTarget(const Project& project, const Setup& main);
/// The transfer that hands sub-spindle setup `sub` its part: the one in the setup its stock comes
/// from, on the main spindle of the same machine (`from` gets that setup). Nullptr when its work is
/// loaded by hand.
const Operation* transferInto(const Project& project, const Setup& sub,
                              const Setup** from = nullptr);
/// How far the sub-spindle's jaws go over the part in a transfer: its own Grip, else the machine's.
double transferGrip(const Operation& transfer, const machine::Machine& machine);
/// Z2 with the sub-spindle holding the part where main-spindle setup `from` leaves it, for
/// sub-spindle setup `sub` (ADR-0007): the part at the same place in the project in both. Nullopt
/// when their WCSs do not line up (one axis, Z turned round).
std::optional<double> pickOffZ2(const Document& doc, const Setup& from, const Setup& sub,
                                const machine::Machine& machine, CancelToken& cancel);

/// Why the setup's tailstock quill cannot be where the setup has it, as a sentence: out further
/// than the machine's quill goes (LatheTailstockBody::quillTravel), naming both and where to change
/// it. It is said, never shortened. None when it is within the travel, when the setup has no
/// tailstock, or when the machine does not say how far its quill goes.
std::optional<std::string> quillBeyondTravel(const Setup& setup, const machine::Machine& machine);

/// Whether the machine's own chuck is what is on the main spindle: unless the setup brings one of
/// its own (workholding or fixtures) to it.
bool usesMachineChuck(const Setup& setup) noexcept;

/// Where the setup's work sits on a lathe with a layout, `stock` being its profile: on the
/// spindle's centre line with the WCS's Z along the spindle's. When the setup brings a chuck, its
/// back on the machine's workholding mount and the work as it holds it; else the work in the
/// machine's chuck by its grip. The placement's offset moves it along Z.
machine::WorkMount latheWorkMount(const Document& doc, const Setup& setup,
                                  const machine::Machine& machine, const geom2d::Regions& stock);

/// The ids latheSupports gives the setup's tailstock and the model of its centre (for moving them
/// with W in a view).
Uuid tailstockSupportId();
Uuid tailstockCentreSupportId();

/// The blocks on a setup's stations, placed and meshed, for checks and display. What cannot be
/// placed is left out, and said in `problems`.
std::vector<sim::TurretBlock> turretBlocks(const Document& doc, const Setup& setup,
                                           const machine::Machine& machine,
                                           std::vector<std::string>* problems = nullptr);

/// A representative tool in a block, in the block's coordinates, to show which way a tool sits in
/// it (the block editor): a square shank with an insert for a block across, a round bar with one
/// for a block along. The sizes the block states, else placeholders; not a tool of the project.
struct ExampleTool {
    std::vector<sim::ConvexPrism> shank;
    std::vector<sim::ConvexPrism> insert;
    Vec3 tip = Vec3::Zero();
    double size = 0.0;       ///< The shank's side or the bar's diameter [mm].
    bool round = false;      ///< A bar (a block along) rather than a square shank.
    bool sizeStated = false; ///< The block's own size; else a placeholder.
};
ExampleTool exampleTool(const LatheFitting& block);

/// The tools of `load` as the machine check holds them: each on its station, in the working
/// position at machine zero, turned as the work on `mount` is. `quadrants` as turretLoad's.
std::vector<sim::TurretTool> turretTools(const Document& doc, const TurretLoad& load,
                                         const machine::WorkMount& mount,
                                         const std::map<Uuid, int>* quadrants = nullptr);

/// The setup's chuck, tailstock and steady rest as the machine check takes them: the chuck turning
/// with the work's spindle, the tailstock on W where it holds the work, the rest where they stand.
/// `tailstockW` gets W where the tailstock holds the work (0 without one). What cannot be made is
/// said in `warnings`.
std::vector<sim::LatheWorkBody> latheWorkBodies(const Document& doc, const Setup& setup,
                                                const machine::Machine& machine,
                                                const machine::WorkMount& mount,
                                                double* tailstockW,
                                                std::vector<std::string>* warnings = nullptr);

/// What a view of a lathe setup's machine draws: the machine's own parts (its chuck only when it
/// holds the work), then the blocks and the tools in the turret, each where its station stands at
/// T 0, carried by the turret. The simulation and the turret preview both show it.
struct LatheMachineParts {
    std::vector<machine::ModelPart> parts;
    /// Per part: the station of a block or tool, 0 for the machine's own.
    std::vector<int> stations;
    std::size_t blocksFrom = 0; ///< Parts from here on are blocks, then tools.
    std::size_t toolsFrom = 0;  ///< Parts from here on are tools.
};
LatheMachineParts latheMachineParts(const machine::Machine& machine,
                                    const machine::MachineModel& model,
                                    const std::vector<sim::TurretBlock>& blocks,
                                    const std::vector<sim::TurretTool>& tools, bool machineChuck);

/// What a key of the simulation needs on top of the setup and machine: tools riding along.
std::string turretKey(const Document& doc, const Setup& setup);

/// For a machine with a layout: places every operation of a simulated lathe setup on the machine
/// (OperationSimulation::mount and latheMotion), fills SetupSimulation::turretTools and runs
/// the machine check on each (OperationSimulation::machineCollisions). What it cannot do is a
/// warning in `out`.
void simulateLatheMachine(const Document& doc, const Setup& setup, const machine::Machine& machine,
                          SetupSimulation& out, CancelToken& cancel);

} // namespace ocam::document
