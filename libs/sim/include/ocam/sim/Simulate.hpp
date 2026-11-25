// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Material removal: subtracts the volume swept by the cutting part of the tool along each move
// from the dexel stock, and reports holder, shank and rapid collisions with remaining material.

#pragma once

#include "ocam/sim/Dexel.hpp"
#include "ocam/sim/ToolShape.hpp"

#include <ocam/foundation/Cancel.hpp>
#include <ocam/foundation/Progress.hpp>
#include <ocam/toolpath/Toolpath.hpp>

#include <cstddef>
#include <limits>
#include <string_view>
#include <vector>

namespace ocam::sim {

enum class CollisionKind {
    Holder, ///< The holder touched material.
    Shank,  ///< The non-cutting part of the tool touched material (flutes too short).
    Rapid,  ///< A rapid move cut material.
    NoHole, ///< A tap went into solid material: it has no hole to run in.
    /// A spindle probe's stylus met material where its move does not expect a touch (DL-0090):
    /// probe moves never remove material.
    Probe,
};

std::string_view toString(CollisionKind kind) noexcept;

/// A run of consecutive moves with the same kind of collision.
struct Collision {
    CollisionKind kind = CollisionKind::Holder;
    std::size_t firstMove = 0; ///< Index into Toolpath::moves.
    std::size_t lastMove = 0;
    Vec3 position = Vec3::Zero(); ///< A point of the first contact (WCS).
    double depth = 0.0;           ///< Largest material overlap along a ray [mm].
};

struct SimulateOptions {
    /// Chord tolerance for arcs that are not simulated exactly (helices; X/Y rays). 0 = an
    /// eighth of the stock resolution.
    double arcTolerance = 0.0;
    bool checkCollisions = true;
    /// Material overlaps shorter than this are ignored by collision checks [mm].
    double collisionTolerance = 1e-3;
    std::size_t maxCollisions = 1000;
    /// Probe moves: how far the stylus may run into material at a touch (its trigger travel
    /// and the stock's resolution) before it is reported [mm].
    double probeAllowance = 0.05;
};

struct SimulateResult {
    std::size_t movesSimulated = 0;
    std::vector<Collision> collisions;
};

/// Where a tap's strokes in `path` (expanded, the tool's axis on each move) find material inside
/// `coreRadius`, which a hole should have taken out. A stroke is a feed move going down its axis.
/// Reported when that is a longer solid core than half a layer of `stock`'s resolution and than a
/// drill's point leaves at the bottom of a blind hole; `depth` is how long. Rays are point samples:
/// a core too thin to hold one reads as clear.
std::vector<Collision> tapsWithoutAHole(const Stock& stock, const toolpath::Toolpath& path,
                                        double coreRadius, CancelToken* cancel = nullptr);

/// Applies moves [0, moveLimit) of `path` to `stock` (WCS coordinates, tool axis +Z). Throws
/// OperationCancelled when cancelled.
SimulateResult simulateToolpath(Stock& stock, const ToolShape& tool, const toolpath::Toolpath& path,
                                const SimulateOptions& options = {}, CancelToken* cancel = nullptr,
                                ProgressSink* progress = nullptr,
                                std::size_t moveLimit = std::numeric_limits<std::size_t>::max());

/// Applies moves [begin, end) of `path`, starting from the tool position after move begin - 1
/// (incremental playback). Collisions report indices into Toolpath::moves.
SimulateResult simulateMoves(Stock& stock, const ToolShape& tool, const toolpath::Toolpath& path,
                             std::size_t begin, std::size_t end,
                             const SimulateOptions& options = {}, CancelToken* cancel = nullptr,
                             ProgressSink* progress = nullptr);

/// Tool tip position after moves [0, moveIndex) (the path start for 0).
Vec3 toolPositionAt(const toolpath::Toolpath& path, std::size_t moveIndex);

} // namespace ocam::sim
