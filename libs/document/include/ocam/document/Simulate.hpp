// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Setup -> stock simulation: builds the initial stock in the setup's WCS, removes material
// operation by operation (keeping the in-process stock after each one as a checkpoint), and
// compares the result with the part.

#pragma once

#include "ocam/document/Document.hpp"

#include <ocam/geom3d/Bvh.hpp>
#include <ocam/sim/Compare.hpp>
#include <ocam/sim/FixtureCheck.hpp>
#include <ocam/sim/Lathe.hpp>
#include <ocam/sim/LatheMachine.hpp>
#include <ocam/sim/MachineCheck.hpp>
#include <ocam/sim/Simulate.hpp>
#include <ocam/sim/Surface.hpp>
#include <ocam/tooling/TurningHolder.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace ocam::document {

struct SetupSimulation;

struct SimulationOptions {
    /// Ray spacing [mm]; 0 picks one from the stock size (about 800 rays along the longest
    /// horizontal side, between 0.05 and 2 mm).
    double resolution = 0.0;
    bool triDexel = true;
    bool compare = true;
    double tolerance = 0.02;              ///< "At size" band of the comparison [mm].
    double tessellationDeflection = 0.01; ///< Part mesh for the comparison [mm].
    /// An already generated toolpath for an operation (from a cache checked against
    /// toolpathInputKey), used instead of generating it again; nullopt generates.
    std::function<std::optional<toolpath::Toolpath>(const Operation&)> toolpathFor;
    /// Simulate the setup's operations up to and including this one only (stock previews).
    std::optional<Uuid> lastOperation;
    /// The whole simulation of the setup this setup's stock comes from (stock "from an earlier
    /// setup"), simulated with sourceOptions(): a cache's (checked against simulationInputKey),
    /// or simulated by the callback. Null simulates it.
    std::function<std::shared_ptr<const SetupSimulation>(const Setup& source, CancelToken& cancel,
                                                         ProgressSink& progress)>
        sourceSimulation;
    /// An earlier simulation of the setup (with other inputs): when its initial stock is the same
    /// (initialKey), its leading operations whose stock is still the same (inputKey) are taken
    /// as they are and the simulation goes on from the last of them.
    std::shared_ptr<const SetupSimulation> resumeFrom;
};

/// The options a setup's stock source is simulated with: these, for the whole setup.
SimulationOptions sourceOptions(const SimulationOptions& options);

struct OperationSimulation {
    Uuid operationId;
    std::string name;
    /// stockInputKeys for the stock after it: what it and the operations before it depend on.
    std::string inputKey;
    /// WCS, as generated but with canned cycles expanded into their moves, so playback shows
    /// every feed, peck and retract.
    toolpath::Toolpath toolpath;
    std::shared_ptr<const sim::ToolShape> tool;
    sim::Stock stockAfter; ///< In-process stock after this operation (shares unchanged tiles).
    double removedVolume = 0.0;
    std::vector<sim::Collision> collisions;
    /// Machine parts and components (head, table, trunnion) against each other, the part, stock,
    /// fixtures and tool (phase 9, DL-0037; machines with components or model parts, setups
    /// with fixtures).
    std::vector<sim::MachineCollision> machineCollisions;
    /// The tool, its shank and its holder against the setup's fixtures and workholding
    /// (DL-0035): every contact, whatever the fixture clearance.
    std::vector<sim::FixtureCollision> fixtureCollisions;
    /// For showing the machine move (DL-0037): the rotary positions along `toolpath`
    /// (machine::rotaryTrack) and where the setup's work sits on the machine for it (its
    /// placement, the WCS squared as machine::mountedWcs has it); unset when the machine does
    /// not move for it (the placement cannot be used, or the toolpath turns a rotary axis that
    /// does not carry the work: SetupSimulation::warnings say so).
    machine::RotaryTrack rotary;
    std::optional<machine::WorkMount> mount;
    /// A lathe with a layout (DL-0067): which station cuts and how the machine got there.
    std::optional<machine::LatheMotion> latheMotion;
    std::string error; ///< Set when the operation could not be generated or simulated.
    /// Turning: the axisymmetric stock after this operation and the insert outline used (lathe
    /// plane u = Z, v = radius, relative to the programmed point), with the insert's thickness
    /// for showing it (0 when the tool gives none).
    std::shared_ptr<const sim::LatheStock> latheAfter;
    geom2d::Polyline insert;
    double insertThickness = 0.0;
    /// Turning: the tool's holder for showing it, placed like the insert.
    std::vector<tooling::TurningToolPart> toolParts;
    /// A live tool off the spindle's axis: what its holes take off ((distance from the tip,
    /// radius); sim::cutHoles). `tool` is the tool drawn, and `insert` stays empty.
    std::vector<Vec2> holeProfile;
};

struct SetupSimulation {
    Uuid setupId;
    double resolution = 0.0;
    sim::Stock initialStock;
    std::string initialKey; ///< stockInputKeys for the initial stock.
    /// Warnings about the initial stock (also in `warnings`): operations of the setup it comes
    /// from that failed.
    std::vector<std::string> stockWarnings;
    /// Material chamfers removed where they are meant to cut (chamferZones), per Z ray of the
    /// stock grid (j * nu + i; empty when there is none): the comparison does not count the part
    /// material in it as a gouge. `initialChamfered` comes with the stock from earlier setups,
    /// `chamfered` adds this setup's chamfers.
    std::vector<sim::IntervalList> initialChamfered;
    std::vector<sim::IntervalList> chamfered;
    /// The setup's machine (copied; its model's meshes are shared).
    std::shared_ptr<const machine::Machine> machine;
    /// A lathe with a layout (DL-0067): every tool in the turret, on it, and whether the machine's
    /// chuck is on the spindle (a setup can bring its own).
    std::vector<sim::TurretTool> turretTools;
    std::vector<sim::TurretBlock> turretBlocks; ///< The tooling blocks on its stations (DL-0068).
    bool machineChuck = true;
    std::vector<OperationSimulation> operations; ///< Enabled operations, in program order.
    geom3d::Mesh part;                           ///< WCS; empty without a model.
    std::optional<sim::Comparison> comparison;   ///< Final stock against the part.
    /// The part prepared on the stock grid, for comparisons of playback frames (with
    /// `options.compare` and a part).
    std::shared_ptr<const sim::PartReference> partReference;
    std::vector<std::string> warnings;
    /// Lathe setups: the initial axisymmetric stock (stocks after each operation are in the
    /// operations). Unless a live tool mills (milledFrom), they have no dexel stocks:
    /// `initialStock` and `stockAfter` stay empty, the comparison is the exact one in the lathe
    /// plane (sim::compareProfile) and playback works on the profile. A milling setup going on from
    /// one converts its final stock (lastStock).
    std::shared_ptr<const sim::LatheStock> latheInitial;
    /// Lathe setups with a part: distances to the part's half profile (u = z, v = radius), for
    /// comparing the turned stock in the lathe plane (latheDisplayMesh).
    sim::ProfileDistance partProfile;
    /// Lathe setups with a part: its half profile, for comparing playback frames in the lathe
    /// plane (latheComparison).
    geom2d::Regions partRegions;
    /// Lathe setups with holes off the axis: the part, for what a hole takes of it.
    std::shared_ptr<const geom3d::TriangleBvh> partBvh;
    /// Lathe setups a live tool mills (DL-0066): the first operation that does. From there on the
    /// stock is the dexels, in `stockAfter`, as on a mill. Those setups have dexels before it too,
    /// for colouring and comparing, but the operations before it play on the profile.
    std::optional<std::size_t> milledFrom;

    /// Whether operation `index` plays on the axisymmetric stock rather than on dexels.
    bool playsOnProfile(std::size_t index) const noexcept {
        return latheInitial && (!milledFrom || index < *milledFrom);
    }

    const sim::Stock& finalStock() const noexcept;
    /// What a setup going on from this one starts from, as dexels: the final stock, a lathe
    /// setup's converted at this resolution.
    sim::Stock lastStock(bool triDexel) const;
    /// Stock before operation `index` (the initial stock for 0).
    const sim::Stock& stockBefore(std::size_t index) const noexcept;
    /// Lathe setups: the axisymmetric stock before operation `index` (null for other setups).
    std::shared_ptr<const sim::LatheStock> latheBefore(std::size_t index) const noexcept;
    std::size_t collisionCount() const noexcept;
    std::size_t machineCollisionCount() const noexcept;
    std::size_t fixtureCollisionCount() const noexcept;
    bool hasErrors() const noexcept;
};

/// A key that changes whenever anything affecting the simulation of the setup changes (its
/// enabled operations' toolpath inputs, the machine and the options). Used to reuse a
/// simulation instead of running it again.
std::string simulationInputKey(const Document& doc, const Setup& setup,
                               const SimulationOptions& options = {});

/// Keys of the stocks of a setup's simulation: [0] of the initial stock, [k] of the stock after
/// the k-th enabled operation (up to options.lastOperation). Each covers everything that stock
/// depends on (the setup, its stock's source, the machine, the options and the operations up to
/// there), so a simulation can go on from the last stock whose key is unchanged.
std::vector<std::string> stockInputKeys(const Document& doc, const Setup& setup,
                                        const SimulationOptions& options = {});

/// The automatic ray spacing for a stock of the given bounds.
double automaticResolution(const geom3d::Aabb3& stockBounds);

/// Initial stock of a setup as a closed mesh in the setup's WCS, or nullopt when the stock
/// cannot be resolved.
std::optional<geom3d::Mesh> stockMeshInWcs(const Document& doc, const Setup& setup);

/// The setups whose cuts a setup's stock carries: its stock's source setup (stock "from an
/// earlier setup"), that setup's source and so on, the earliest first. Empty for a stock of its
/// own.
std::vector<const Setup*> stockSources(const Project& project, const Setup& setup);

/// Cuts into `stock` (in `setup`'s WCS, or in `frame` within it) what the setups it comes from
/// (stockSources) removed: their enabled operations' toolpaths, moved into this setup's frame,
/// simulated on its grid. Material removal does not depend on the order of the cuts, so this
/// gives the stock those setups left, exactly on this grid. Operations that fail are skipped
/// and reported in `warnings`. Throws OperationCancelled when cancelled.
void carryEarlierSetups(const Document& doc, const Setup& setup, sim::Stock& stock,
                        CancelToken& cancel, std::vector<std::string>& warnings,
                        const Transform3& frame = Transform3::Identity());

/// Simulates every enabled operation of a setup. A stock from an earlier setup starts as the
/// final stock of that setup's simulation, sampled again on this setup's grid
/// (sim::resampleStock). Operations that fail to generate are reported in their `error` and
/// skipped. Throws Error(InvalidArgument) for an unknown setup or when the
/// stock cannot be resolved, and OperationCancelled when cancelled.
SetupSimulation simulateSetup(const Document& doc, const Uuid& setupId,
                              const SimulationOptions& options, CancelToken& cancel,
                              ProgressSink& progress);

/// In-process stock just before operation `operationId`: the setup's initial stock with every
/// earlier enabled operation generated and simulated (no collision checks). Earlier operations
/// that fail are skipped and reported in `warnings`. Used by rest machining. The stock is
/// built in `frame` (given in the setup's WCS: an operation's own frame, operationFrame), its
/// dexels along the frame's axes.
struct StockBefore {
    sim::Stock stock;
    std::vector<std::string> warnings;
};
StockBefore stockBeforeOperation(const Document& doc, const Setup& setup, const Uuid& operationId,
                                 double resolution, CancelToken& cancel, ProgressSink& progress,
                                 const Transform3& frame = Transform3::Identity());

/// Re-runs one operation from its checkpoint up to (not including) `moveIndex`, for playback.
sim::Stock simulatePartial(const SetupSimulation& simulation, std::size_t operationIndex,
                           std::size_t moveIndex, CancelToken* cancel = nullptr);

/// Lathe setups: the axisymmetric stock just before move `moveIndex` of operation
/// `operationIndex` (the turning counterpart of simulatePartial); nullopt for other setups.
std::optional<sim::LatheStock> simulatePartialLathe(const SetupSimulation& simulation,
                                                    std::size_t operationIndex,
                                                    std::size_t moveIndex,
                                                    CancelToken* cancel = nullptr);

/// Lathe setups: a turning stock (during operation `operationIndex`) as its exact profile
/// revolved (DL-0040), coloured like the dexel display. With `compare` and a part, profile points
/// deeper than the comparison tolerance inside the part's half profile are gouges and those
/// within it are at size (the distance in the lathe plane, so radial on diameters); the rest
/// shows the operation that cut it (the last whose starting profile does not pass through it),
/// or `stockColour` where nothing did. Empty for other setups.
/// `stock` (a playback frame of the lathe setup) against the part in the lathe plane; nullopt for
/// a setup without a lathe stock or a part.
std::optional<sim::Comparison> latheComparison(const SetupSimulation& simulation,
                                               const sim::LatheStock& stock);

sim::DisplayMesh latheDisplayMesh(const SetupSimulation& simulation, std::size_t operationIndex,
                                  const sim::LatheStock& stock, bool compare,
                                  const sim::Rgb& stockColour);

} // namespace ocam::document
