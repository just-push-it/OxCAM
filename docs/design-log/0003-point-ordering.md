# DL-0003: Point ordering for rapid travel (nearest neighbour + 2-opt)

- **Status:** implemented (phase 1)
- **Module:** libs/geom2d (`orderPoints`)
- **Patent gate:** low (citation only); classical TSP heuristics
- **Author(s):** OxCAM contributors

## Problem

Order drill positions, and later the entry points of pocket and contour regions, to keep
rapid-travel length short. The result must be deterministic so golden tests stay stable.

## Approach

The path is open and starts at a given point (typically the previous tool position).

1. **Nearest neighbour.** Build an initial path by repeatedly moving to the closest unvisited
   point. Ties go to the lowest index.
2. **2-opt.** Repeatedly reverse a sub-sequence whenever doing so shortens the path. For an
   open path, the last point has no outgoing edge. Stop when a full pass finds no improvement,
   or after a pass limit.

Cost is O(n²) per pass, which is fine for the hundreds of points typical of drilling.

## Sources

- D. J. Rosenkrantz, R. E. Stearns, P. M. Lewis II, "An Analysis of Several Heuristics for the
  Traveling Salesman Problem", *SIAM Journal on Computing* 6(3), 1977, pp. 563–581.
- G. A. Croes, "A Method for Solving Traveling-Salesman Problems", *Operations Research* 6(6),
  1958, pp. 791–812.

## Deviations from sources

The path is open with a fixed start, rather than a closed tour.

## Correctness and tests

`tests/unit/geom2d/OrderingTest.cpp`:

- the output is a permutation;
- 2-opt never lengthens the nearest-neighbour path;
- output is deterministic;
- collinear points are visited in order.

## Known limitations

- There is no Or-opt or 3-opt step, and no clustering for very large point sets.
