# DL-0034: Fixture avoidance and tool holders from solid models

- **Status:** implemented
- **Module:** libs/strategies (`ObstacleField`, `avoidObstacles` in `Avoidance.hpp`),
  libs/cad (`holderOutline` in `Revolve.hpp`), libs/tooling (`Holder`, `makeHolder`),
  libs/document (`Setup::fixtures`, `fixtureClearance`, `Document::addImportedFixture`,
  `removeModel`), GUI (setup page, holder tab of the tool library)
- **Patent gate:** low (citation only). Trimming a toolpath against check surfaces with a
  drop-cutter and lifting over them, and approximating a holder by its revolved envelope, are
  long-established CAM practice; nothing here controls engagement or optimizes links.
- **Author(s):** OxCAM contributors

## Problem

Parts sit in vises, on plates and under clamps. A toolpath that ignores them crashes the tool
or the holder into the fixture. Fixtures are STEP models like the part, held per setup (a
second setup holds the flipped part its own way). Every 3-axis milling toolpath must keep the
tool, its shank and its holder a clearance away from them, without each strategy knowing about
fixtures.

Holders existed as revolved outlines but could not be made or edited. Vendors publish holders
as STEP solids; the outline must be derived from them so it is never smaller than the solid.

## Approach

### Fixtures in the project

`Setup::fixtures` lists models of `Project::models` (placed like the part, in the project
frame: a fixture modelled around the part in CAD lands in place). `fixtureClearance` (1 mm by
default) is the closest approach. Importing a fixture adds the model and lists it in one undo
step; removing a model removes every reference to it. Fixtures show in the view with their own
setup only, in a darker steel blue.

### The safe height field

`ObstacleField` gives, for an XY tool position, the lowest tip height that keeps the clearance.
The tool assembly is a stack of solids of revolution that each reach upward without end:

1. the cutting part: for flat, ball and bull-nose end mills the exact torus grown by the
   clearance (`Cutter::offset`, its tip the clearance lower); for other shapes a cylinder of the
   largest cutting radius grown by the clearance, its bottom the clearance below the tip;
2. a cylinder wherever the tool body (above the cutting length, up to the holder face at the
   stickout) or the holder grows wider than everything below it. The outlines are discretized;
   each segment is a band at the larger radius of its ends. Bands that start within
   max(2 mm, height / 16) above the last cylinder widen it instead of adding one (conservative,
   and few drop-cutters).

Each solid's drop-cutter height on the obstacle mesh, less the height of its bottom above the
tip, is the height that solid needs; the safe height is the largest. Above the mesh top plus
the clearance nothing is in reach, and nothing beyond the largest radius in XY.

### Trimming a toolpath

`avoidObstacles` walks the moves, tracking where the original path is and where the tool is:

- **Feed moves** (lines; arcs that need trimming become lines within the chord tolerance) are
  sampled at most half the cutter's reach apart: the safe height's plateaus are at least twice
  that wide, so no obstacle fits between two samples. Where a sample is below the safe height
  the move is cut at the boundary (bisection to 0.1 µm on the free side) and the tool lifts.
- **Resuming** at the next free point: rapid straight up (free above a free point), across at
  max(retract, the highest safe height along the XY line), down to the retract height and a
  plunge at the plunge feed to the cut.
- **Rapids** that pass too close are routed up, across and down; a rapid into a blocked point
  leaves the tool where it is until the path comes free.
- **Drilling cycles** whose hole bottom is below the safe height are left out; the travel to the
  next hole goes over the obstacle.
- With cutter compensation the path is the contour, so the clearance grows by the tool radius.

Warnings report the stretches of cutting left out (and their length), raised rapids and skipped
holes. Generation runs the pass last (after smoothing) on every milling operation of a setup
with fixtures; turning, 4th-axis and 5-axis toolpaths get a warning instead. The toolpath key
includes the holder when the setup has fixtures, and the clearance through the setup.

### Holders

`Holder` gains a vendor and part number, `length()` and `maxRadius()`; `makeHolder` builds a
stepped outline (a nose that may taper, a body, the flange up to the gauge line). The tool
library dialog has a holders tab: new, from dimensions, point-by-point editing, duplicate,
delete (tools using it lose it), and STEP import.

`holderOutline` turns a solid into an outline:

1. Tessellate with half the tolerance. The axis is the model axis across which the bounding box
   is roundest (the longest such axis on ties), through the box centre; or the one chosen.
2. Cut the length into up to 1200 slabs (at least 0.1 mm). Clip every facet to each slab it
   spans; the largest distance from the axis in a slab is at a corner of the clipped facet
   (distance from a line is convex).
3. The flange is the widest slabs. The tool comes out of the end farther from it, and the top of
   the flange (towards the spindle) is the suggested gauge line; the outline stops there unless
   another length is given (the taper inside the spindle is not part of a holder).
4. Each slab boundary takes the larger radius of its two slabs, so straight lines between
   boundaries contain every slab; a Douglas-Peucker pass on r(h) with the tolerance as radial
   error thins the points, and the tolerance is added back to every radius.

The import dialog shows the outline and lets the axis, the tool end and the gauge length be
changed; a warning appears when the model is not round about the axis.

## Sources

- C. Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2005 (polygon clipping against
  planes, 8.3).
- D. Douglas, T. Peucker, "Algorithms for the reduction of the number of points required to
  represent a digitized line or its caricature", *Cartographica* 10(2), 1973.
- The drop-cutter of DL-0016.

## Deviations from sources

Douglas-Peucker measures the radial error of r(h) rather than the perpendicular distance, so the
result can be pushed out by exactly the tolerance and stay outside the solid.

## Correctness and tests

- **Safe heights** (`tests/unit/strategies/AvoidanceTest.cpp`): over a clamp, beside it in the
  rounded clearance, out of reach; with a holder, the nose and the flange set the height beside
  a clamp at their heights above the tip.
- **Facing round a clamp:** no tool position (rapids included, densely resampled) within the
  clearance, the face beyond the clamp's reach still cut, warnings.
- **Holder against a tall wall:** passes the cutter could make are left out where the holder's
  nose would come within its clearance; without the holder they stay.
- **Drilling:** the hole under a clamp is left out, the travel goes over it.
- **Unchanged** toolpaths far from the obstacles.
- **Document** (`GenerateTest.cpp`): a stepped pin standing through the block keeps facing
  clear; the clearance and the holder change the toolpath key; removing the model removes the
  fixture. Project round-trip and validation of fixtures (`ProjectTest.cpp`).
- **Holder outline** (`tests/unit/cad/RevolveTest.cpp`): the stepped shaft's outline is never
  inside it and within two tolerances of it; automatic axis and tool end; the gauge line at the
  top of the wide part; explicit lengths; asymmetry about the wrong axis.
- **CLI:** `fixture_project.ocam` (the block in vise jaws, the tool in a collet chuck) generates
  with the profile trimmed at the jaws.

## Known limitations

- **3-axis only.** Multi-axis, 4th-axis and turning toolpaths are not trimmed (a warning says
  so); check them in the simulation.
- **Conservative shapes.** Every solid reaches up without end, so a narrow neck above a wide
  head (slot cutters, lollipops) and room under an overhanging clamp are not used. Non-torus
  cutters are cylinders.
- **No re-routing of cuts.** Trimmed areas are left uncut (the warnings say how much); the
  strategies do not plan around the fixtures.
- **The holder against the part** is not avoided, only reported by the simulation.
- **Plunges** back into the cut are straight down at the plunge feed, even where the strategy
  would have ramped.
- **Fixtures are not in the simulation's collision checks** yet.
