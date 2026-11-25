# DL-0056: A turning tool's holder shows in playback

- **Status:** implemented
- **Module:** libs/tooling (`turningToolParts`), libs/document (`OperationSimulation::toolParts`,
  `SimulationCache`), apps/opencam-gui (`InsertPrs`, `SimulationPanel`)
- **Patent gate:** low (citation only). Drawing known shapes.
- **Author(s):** OxCAM contributors

## Problem

A milling tool's playback draws its holder. A turning tool's drew only the insert. That was never
decided: DL-0020 drew the insert when there was nothing else to draw, DL-0042 and DL-0045 gave holders a
shape and checked them against the stock, and neither design log mentions the display. So a tool could
collide by its holder in the simulation with no holder to see.

## Approach

**What is drawn (`turningToolParts`).** `turningBody` is what the checks use, and it leaves out what
stands beside the insert on purpose (DL-0045), so it would draw a holder with a hole in it. Playback gets
its own list from the same outlines: the ISO head with the insert in it, and the shank.

| Holder | Drawn as | Across the plane |
|---|---|---|
| ISO or plain shank | A plate | Its top at the tip's height (a shank's usual cutting height), its height below |
| ISO or plain bar | A round bar | Its axis at the tip's height |
| Solid bar | Its head as a plate, its shank as a round bar | Around the tip's height |
| An outline of its own (imported) | Bands, away from the insert on its middle | As wide as the outline |
| None, or a milling holder | Nothing | |

A plain shank has no height of its own, so it is drawn as high as it is wide.

An imported outline is drawn where the tool editor's preview puts it. That says nothing about where such a
holder really is, and it is still not checked (DL-0045), only shown.

**Look.** The same grey as a milling tool's holder, less translucent, so the insert stays readable through
it. It goes with the insert: the Tool checkbox hides both, and back turning mirrors both.

**Both stock displays.** Smooth and cells share this path: the holder is part of the tool's presentation,
not of the stock's.

**The cache.** A saved simulation keeps the parts, so `kSimulationCacheVersion` is 8. Older caches are
recomputed.

## Correctness and tests

`ToolTest` "what shows of a turning tool's holder": each row of the table, the shank's back at f, the mirror
away from the chuck. `SimulateTest` "turning in an ISO holder": the operations carry the parts, and they
survive the cache. `gui.lathe_holder_outside_*` and `gui.lathe_holder_inside_*` run playback of the new
`holders_project` fixture in smooth and in cells, a shank outside and a bar in the bore.

## Known limitations

- **Hole tools on a lathe** (drills, taps) still show as their silhouette, with no holder: a revolved holder
  has no place in this drawing yet.
- **The bar in a bore** is hidden by the stock around it, as the real one is. Hide the stock to see it.
