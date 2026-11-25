# DL-0060: The cell display is gone

- **Status:** implemented
- **Module:** libs/sim (`Compare`, `Lathe`), libs/document (`Simulate`), apps/opencam-gui (`SimulationPanel`,
  `PreferencesDialog`), apps/opencam-cli
- **Patent gate:** none. Removing code.
- **Author(s):** OxCAM contributors

## Problem

Since DL-0040 the smooth stock display was the default, with the old cells kept as a preference. The maintainer's
decision, 2026-11-02: it need not stay. It also kept a second way of running a lathe simulation alive.

## Approach

**Everywhere, milling too.** The preference, the `--stock-display` test hook, `DisplayOptions::smooth` and the code
that drew cells (`emitCells`) are gone. `buildDisplayMesh` and `StockDisplay` always draw the surface. The cell
*grid* stays: the surface's colours and the comparison's values are still worked out per cell.

**A saved preference for cells** is simply not read. Such a file loads without complaint and shows the surface
(`gui.simulation_old_cells_preference`).

**Lathe setups have no dexels now.** `SimulationOptions::turningDexels`, `Stock::fromRevolvedProfile`'s update form
and `revolvedUpdate` existed so the cells could be drawn. A lathe setup is simulated on its profile, always, and
compared exactly in the lathe plane (`compareProfile`). So the command line now reports the comparison the GUI shows,
and takes the time the GUI does.

**What still needs dexels keeps them.** Milling simulation, its comparison and rest machining are as they were. A
milling setup going on from a lathe setup converts the lathe's final stock when it starts
(`SetupSimulation::lastStock`), the only place a lathe stock becomes dexels.

## Correctness and tests

`SimulateTest` "turning": no dexels on a lathe setup, `lastStock` holds the same volume, frames still compare and the
cache keeps them. `SimTest`, `SurfaceTest`: the display's colours and incremental update, on the surface alone.
`gui.simulation_old_cells_preference` opens a project with the old preference and simulates.

## Known limitations

- The benches (`SmoothStockBench`) no longer measure cells against the surface. DL-0040 keeps those numbers.
