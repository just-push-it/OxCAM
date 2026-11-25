# DL-0012: Tri-dexel stock simulation, part comparison and holder collisions

- **Status:** implemented (phase 5, v0.1)
- **Module:** libs/sim, libs/document (`simulateSetup`), apps (CLI `project simulate`, GUI
  simulation dock)
- **Patent gate:** low (citation only). The IP policy lists dexel simulation as low risk. The
  comparison and collision checks are direct geometric tests on the same representation. Nothing
  here feeds the simulated stock back into toolpath generation; that is phase 6 rest machining,
  which has its own review.
- **Author(s):** OxCAM contributors

## Problem

Given the initial stock, the tools and the toolpaths of a setup, compute the in-process stock
after every operation and at any move inside an operation. Then:

- Show the stock coloured by its deviation from the part.
- Report places where the holder, the non-cutting shank or a rapid move meets material.

All coordinates are in the setup's WCS. The tool axis is +Z (3-axis).

## Approach

### Representation

A **dexel grid** is a regular 2D array of parallel rays. Each ray stores the sorted, disjoint
intervals of material along it.

- **Tri-dexel stock:** three grids, with rays along X, Y and Z, all at spacing `h`. Ray (i, j)
  passes through the cell centre `u0 + (i + ½) h`. The bounds are the stock box grown by one
  cell.
- **Authority:** the Z grid is authoritative for volume, display and comparison. The X and Y
  grids record vertical walls and undercuts at full resolution, for future surface
  reconstruction and 3D rest machining.
- **Tiles:** rays are grouped into 16 × 16 tiles held by shared pointers. Copying a stock copies
  pointers only. The first edit to a shared tile clones it (copy-on-write).
- **Checkpoints:** the checkpoint after each operation is a plain copy. Checkpoints share every
  tile an operation did not touch.

### Initial stock from a closed mesh

Each triangle is projected onto the grid plane and rasterized against the ray centres with edge
functions.

- **Shared edges:** a top-left fill rule counts a ray through a shared edge exactly once. Edge
  functions are evaluated with the endpoints in a canonical order, so the two triangles sharing
  an edge get exactly opposite values.
- **Hit classification:** the sign of the projected area says whether the face is entering
  (normal against the ray) or leaving.
- **Building intervals:** the hits along each ray are sorted and paired with a depth counter.

Box stock and the cylinder stock are meshed and go through the same path.

### Tool solid

The canonical revolved profile (DL-0004) and the axis close into a polygon in (r, h). Two tables
are precomputed from it:

- **Height spans at 1025 sampled radii d:** the heights where the cutting part (h ≤ flute length)
  reaches radius d. These come from crossings of the line r = d with the polygon. Up to four spans
  are stored, so slot cutters and lollipops work. Lookups interpolate between samples.
- **Radius at 513 heights:** gives the largest cutting radius over any height range.

Above the flute length, the shank up to the holder face is a cylinder of the largest body radius.
The holder outline becomes a stack of cylindrical bands.

### Removal

Each move is split into pieces:

- **Straight segments:** a segment whose height change exceeds h/4 while it also moves
  horizontally is split into parts of at most h/4 height.
- **Horizontal arcs:** these stay exact.
- **Helices:** these are chord-sampled at h/8, then treated as sloped segments.
- **Drill cycles:** these are expanded into plunges and retracts. Collisions still report the
  index of the cycle move.

**Z rays.** Let d be the horizontal distance from the ray to the piece's XY path. For an arc,
this is |‖p − c‖ − r| inside the swept angle, otherwise the distance to the nearer end. The tool
covers the heights `[z_lo + span.lo(d), z_hi + span.hi(d)]` for each span at d. This is exact for
horizontal and vertical motion, because the spans shrink as d grows. For sloped pieces it is an
outer bound, at most one split height (h/4) too generous.

**X and Y rays.** At height z the swept cross-section is a capsule around the piece's XY segment.
Its radius is the largest tool radius over the heights the tip passes, `[z − z_hi, z − z_lo]`.
The ray's interval is where the ray's line crosses the capsule. That comes from the two end
circles and the two offset lines; because the capsule is convex, the extremes of these candidates
are the exact extent.

**Batching and parallelism.**

- Removal is set subtraction, so its order does not matter. Pieces are batched (up to 4096) and
  applied in parallel with oneTBB, one task per tile row.
- Each piece is binned into the tile rows it touches. Its tiles are detached (copy-on-write)
  before the parallel pass, so no two threads clone the same tile.
- Rays whose material is already below the piece are skipped. For straight pieces, each row
  visits only the columns inside the capsule.

### Collisions

A collision check always flushes the pending batch first, so it sees the stock in program order.

| Check | When it runs | What counts as a collision |
|---|---|---|
| **Rapid** | Before removing the rapid's material, if the rapid goes below the top of the stock | Material inside the cutter's sweep |
| **Shank** | After removal, if the flute top passes below the top of the stock | Material between the flute length and the holder face, within the shank radius |
| **Holder** | After removal, if the holder face passes below the top of the stock | Material inside any holder band |

Consecutive moves with the same kind of collision merge into one record (first move, last move,
contact point, deepest overlap). The scans skip:

- overlaps shorter than 1 µm;
- the outer tenth of a cell of the shank and holder radii, so the tool rubbing an existing wall
  it just cut does not count.

### Part comparison

The part mesh is rasterized on the stock's Z grid layout. For each ray:

- **Gouge:** the length of (part eroded) minus stock. The part is eroded by the tolerance
  vertically and by at least one ray spacing horizontally: the intersection of the neighbouring
  part rays, then a vertical shrink.
- **Excess:** the length of stock minus (part dilated), where dilation is the union of the
  neighbouring rays plus a vertical grow.

The horizontal margin keeps single-ray slivers along tessellated curved walls from showing up as
false gouges or excess. The raw excess (stock minus part) is reported for rays beyond the
tolerance.

Colours: gouge is red, at size is green, and excess runs from yellow through orange to blue.

*Amended 2026-11-01.* A gouged ray also keeps where along it the part material is missing (the
lowest and highest Z of the eroded part minus the stock). Only the surface there is red, within
a display cell of it (1.5 cells on the smooth surface, whose vertices lie within a lattice cube
of the material's boundary): the cut floor and walls of the gouge and the walls facing it.
Before, the whole ray counted as gouged, so a chamfer cut on top showed red on the stock's
underside too.

*Amended 2026-11-01.* Chamfers are seldom modelled, so the part material a chamfer operation
(`chamfer2d`, DL-0014) removes on purpose is not a gouge. Per Z ray, the simulation keeps the
material each chamfer removed (its stock before minus after) within its zone (`chamferZones`):
horizontally within the chamfer width, one ray spacing and the tolerance of the selected edges
(the band between the edges' outline grown and shrunk by that much), vertically from the edges'
height down by the chamfer depth, with the same margin. The comparison takes that material out of
what the stock must hold (the eroded part). Material the chamfer tool cuts anywhere else, or
another operation cuts in the zone, is still a gouge. The chamfered material goes on with the
stock to later setups (`initialChamfered`): points along its rays a third of a spacing apart,
moved into the next setup's WCS, mark the rays within a spacing of them, a spacing above
and below. Chamfers with a tool orientation of their own (3+2) are not exempted. `kSimulationCacheVersion` 4.

### Display

The Z grid becomes a height field:

- Rays are merged into at most 400 × 400 cells.
- A cell is solid when at least half its rays hold material.
- Each cell has a top, a bottom and walls wherever it rises above a neighbour, with per-vertex
  colours.

The GUI draws it as an OCCT triangle array.

Since DL-0040 the GUI draws a smooth surface by default: dual contouring of all three grids. This
height field remains as the "Cells" preference and for callers that do not ask for the smooth
surface.

### Playback

`document::simulateSetup` keeps each operation's toolpath, tool shape and stock afterwards. The
GUI picks a position (operation, move):

- It re-simulates from the nearest earlier frame of the same operation, or from the operation's
  checkpoint, without collision checks.
- It builds the display mesh on a worker thread.

Playing forward is therefore incremental.

## Sources

- T. Van Hook, *Real-time shaded NC milling display*, Computer Graphics (SIGGRAPH '86) 20(4),
  1986. Introduces dexels: per-pixel depth intervals subtracted by the tool's swept volume.
- H. Müller, T. Surmann, M. Stautner, F. Albersmann, K. Weinert, *Online sculpting and
  visualization of multi-dexel volumes*, Proc. ACM Symposium on Solid Modeling and Applications,
  2003. Covers dexel grids along three axes.
- J. Pineda, *A parallel algorithm for polygon rasterization*, Computer Graphics (SIGGRAPH '88)
  22(4), 1988. Covers edge functions; the top-left tie rule is the common convention for shared
  edges.
- Y. Huang, J. H. Oliver, *NC milling error assessment and tool path correction*, SIGGRAPH '94,
  1994. Covers comparing a simulated workpiece with the design surface.

## Deviations from sources

- **Ray direction:** rays run through the workpiece in world axes rather than view rays, so the
  result does not depend on the view.
- **Swept volumes:** the swept volume per ray comes from precomputed height spans of the revolved
  profile, rather than from rendering the tool's swept surface.
- **Copy-on-write tiles:** these are added so that per-operation checkpoints are cheap. The cited
  works keep a single state.
- **Comparison tolerance:** the tolerance has a horizontal component of at least one ray spacing,
  to suppress discretization slivers.

## Correctness and tests

All tests are in `tests/unit/sim` unless noted. "Within 0.5 %" means the removed volume is within
0.5 % of the analytic value at h = 0.1 mm.

| What | Test | Result |
|---|---|---|
| Interval operations | Unit tests | — |
| Box stock volume | Exact on all three grids; faces exactly on ray coordinates are counted once | Exact |
| Mesh rasterization | 200-sided prism | Within 0.5 % on all three grids |
| Flat plunge, flat slot, ball groove, arc slot | Analytic volumes | Within 0.5 % |
| Pocket from `strategies::pocket` (helix entry, R5 corners) | Analytic volume | Within 0.5 % |
| Tri-dexel agreement | X and Y grid volumes against the Z grid | Within 2 % |
| Convergence | Error at 0.4 → 0.2 → 0.1 mm | Does not grow; ends below 0.5 % |
| Collisions | Shank and holder in a slot deeper than the flutes; none within the flute length; a rapid through material reports its 1 mm overlap | Reported as expected |
| Drill cycles | Peck cycles remove two holes' volume | Within 0.5 % |
| Comparison | Excess before facing; at size after; a plunge below the part gives a gouge of the right depth and volume; colours in the display mesh | As expected |
| Copy-on-write snapshots | — | Unchanged by later edits |
| Throughput | `OCAM_SIM_BENCH=1`: 100 000 short moves at 0.1 mm, tri-dexel, on a 100 × 100 mm stock | 2.4 s (RelWithDebInfo, 24 hardware threads). Target: < 30 s on 8 cores |
| Block fixture (`tests/unit/document`) | Face, pocket and contour simulated through the document layer | Facing and pocket volumes match; final stock equals the part volume within 0.5 %; no gouges; no excess |
| CLI (`cli.project_simulate`) | `oxcam-cli project simulate` on the sample project | Reports no gouges and exits 0 |
| GUI (`gui.smoke_test_simulation`) | Simulation and playback in the GUI | Renders |

## Known limitations

- **Excess magnitude:** excess is measured along Z. A 1 mm wall of side stock shows its full
  height as excess, so it appears blue rather than yellow. The X and Y grids could supply the
  horizontal thickness for wall colouring later.
- **Grids not yet used:** the display and collision checks use only the Z grid. Undercuts recorded
  by the X and Y grids are not drawn yet.
- **Sloped moves:** these over-cut by at most h/4.
- **Tool axis:** only +Z, so 3-axis only. 3+2 and simultaneous multi-axis need rotated grids or a
  transformed sweep (phases 7 and 9).
- **Stock between setups:** a `FromSetup` stock starts as its source setup left it: the final
  stock of that setup's simulation, moved into this setup's WCS and sampled again on this grid
  (`sim::resampleStock`: the source's smooth boundary at every ray, DL-0040, filled into this
  setup's grids and kept within its root stock). The source's simulation is the one already
  worked out where there is one (`SimulationOptions::sourceSimulation`: the GUI's recent
  simulations, its local cache, or the one in flight), else it is simulated first; the source's
  own source is resolved the same way, so a chain is simulated once per setup.
  *Amended 2026-11-01.* This replaced simulating the source setups' toolpaths again on this
  setup's grid (exact on this grid, but a second simulation of every earlier setup each time a
  later one was simulated, and 0% progress meanwhile). Sampling the surface again can move it by
  up to about a cell; the X and Y grids keep vertical walls. Rest machining
  (`stockBeforeOperation`, at its own resolution and in the operation's frame) still cuts the
  source setups' toolpaths on its grid (`carryEarlierSetups`).
  Tests: `SimulateTest` "stock from an earlier setup carries what it cut" (a pocket cut in the
  first setup is found, turned over, in the second setup's stock and its rest stock; a given
  source simulation is used), `SurfaceTest` "a stock sampled again on a turned grid keeps its
  cuts".
- **Holder model:** the holder is approximated by cylindrical bands, using the larger radius of
  each profile segment, so it is conservative.
- **Debug builds:** these are about 100× slower than optimized builds, because of MSVC iterator
  debugging. Use `msvc-release` for real work.
