# DL-0010: Greedy line/arc fitting of polygonal paths

- **Status:** implemented (phase 3)
- **Module:** libs/geom2d (`fitArcs`)
- **Patent gate:** low (citation only)
- **Author(s):** OxCAM contributors

## Problem

Clipper2 offsets return polygons. Round joins and offsets of true arcs should reach the control as
G2/G3 arcs, not thousands of short lines.

## Approach

Walk the points from index i:

1. Find the longest run i..j that fits a straight segment within the tolerance.
2. Find the longest run i..j that fits one circular arc. The circle passes through points i,
   ⌊(i+j)/2⌋ and j, and the arc must satisfy all of these:
   - every point lies within tolerance of the circle;
   - the angular steps all have the same sign;
   - each chord's sagitta stays within tolerance;
   - the sweep is at most 180°;
   - the radius is at most 10⁵ mm.
3. Take whichever run covers more points, preferring the line on a tie, and continue from j.

Closed paths fit the closing segment like any other. Arcs are stored as bulges.

## Sources

- Elementary geometry: the circle through three points, and sagitta bounds.

## Correctness and tests

`tests/unit/geom2d/PathOpsTest.cpp`:

- circles collapse to at most 4 arcs with area within 1e-5 relative;
- rectangles keep 4 lines;
- the rounded offset of a rectangle keeps round corners as arcs with the correct area.

Indirectly, the pocket property tests (DL-0007) find no gouge with fitted loops.

## Known limitations

- The greedy method is not globally optimal (it may use more segments than necessary).
- There is no G1 continuity enforcement (biarcs).
