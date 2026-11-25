// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Stocks as bytes, for the local simulation cache (ADR-0003). Stocks copied from one another
// (a setup's checkpoint after each operation) share the dexel tiles none of them changed; the
// encoding stores each tile once and decoding shares it again, so a setup's checkpoints take
// about as much room on disk as in memory.

#pragma once

#include "ocam/sim/Dexel.hpp"

#include <ocam/foundation/Bytes.hpp>

#include <vector>

namespace ocam::sim {

/// Appends the stocks to `w` (any number; tiles shared between them are written once).
void writeStocks(ByteWriter& w, const std::vector<const Stock*>& stocks);

/// Reads stocks written by writeStocks, in the same order and sharing the same tiles. Throws
/// Error(Parse) for damaged data.
std::vector<Stock> readStocks(ByteReader& r);

} // namespace ocam::sim
