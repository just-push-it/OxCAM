# DL-0025: Machine collision checks, convex distances and the triangle BVH

- **Status:** implemented (phase 9)
- **Module:** libs/geom3d (`Bvh.hpp`, `Convex.hpp`), libs/machine (`Component`, machine file
  `components`), libs/sim (`MachineCheck.hpp`), libs/document (`OperationSimulation::
  machineCollisions`), CLI and GUI simulation reports
- **Patent gate:** low (citation only). Checking machine parts moved by forward kinematics with
  convex distance queries is standard practice. Collisions are reported, never avoided.
- **Author(s):** OxCAM contributors

## Geometry

- **Triangle BVH (`geom3d::TriangleBvh`).** A binary tree of triangle bounding boxes built by
  median split on the longest axis of the centroids, with leaves of up to 4 triangles. It
  answers box queries and nearest ray hits (slab test, then Moeller-Trumbore). Used by the
  radial height map (DL-0024), the 5-axis clearance report and the machine checks.
- **Convex solids (`geom3d::Convex`).** Convex hulls of points (boxes, oriented boxes,
  triangles, segments) and finite cylinders, given by their support mappings, optionally
  rounded by a margin (spheres and capsules).
- **GJK distance.** Gilbert-Johnson-Keerthi on the Minkowski difference. The closest point of
  the simplex is found by Voronoi-region tests on points, segments, triangles and tetrahedra.
  Margins are subtracted at the end. Two lessons from testing:
  - The segment case must copy its first point before reducing the simplex; reading it through a
    reference after the reduction extrapolated past the segment and reported false overlaps.
  - Flat tetrahedra (a parallelogram of two segment ends and two triangle corners) have no
    reliable inside test, so their hull is treated as the union of their four triangles.

## Machine components

Machine files list `components`, each a `box` (min, max) or a `cylinder` (base, axis, radius,
length) in the frame of what carries it:

| `attach` | Frame | Moves with |
|---|---|---|
| `head` | origin at the spindle gauge point, +Z along the tool axis | the spindle, including head rotaries |
| `table` | the table frame (project frame), origin at the table centre | every table rotary, like the part |
| `cradle` | the machine frame at the table centre | the outer table rotary only (A of A/C, B of B/C); fixed otherwise |

The built-in 4-axis VMC has a spindle nose and housing, the rotary table housing, the chuck and
a tailstock. The A/C trunnion has the spindle, the round table, and the trunnion walls and
cradle. The head-table machine has the spindle nose, the B-axis head and the table.

## The check (`sim::checkMachine`)

Everything is checked in the table frame:

1. **Rotary positions.** For multi-axis toolpaths they come from `planRotary` (as solve). For
   others: the 3+2 index on table-table machines, else 0.
2. **Poses.** The tip and axis are interpolated along each move and the rotary positions
   linearly. A pose is checked every 2 mm of tip travel or 2° of rotary motion, at the end of
   rapids and at the end of the toolpath.
3. **Placement.**
   - Head solids sit at tip + gauge · u, rotated by Tᵀ H.
   - Cradle solids are rotated by Tᵀ C.
   - Table solids, the part (BVH) and the stock box stay in place.
   - The tool assembly (cutting part, shank, holder bands as cylinders) follows the head from
     the tip.
4. **Pairs.**
   - Head components against the table and cradle components, the stock box and the part's
     triangles.
   - The tool, its shank and its holder against the table and cradle components.
   - The tool assembly against the part and stock is left to the stock simulation, which knows
     the in-process stock.
5. **Report.** A contact closer than the clearance (1 mm) is recorded with both names, the
   moves (merged over consecutive moves), a contact point in the WCS and the smallest distance.

`simulateSetup` runs the check for every operation on machines with components. The CLI prints
the contacts (and fails the run), and the GUI lists them in orange in the simulation panel.

## Sources

- E. G. Gilbert, D. W. Johnson, S. S. Keerthi, "A fast procedure for computing the distance
  between complex objects in three-dimensional space", *IEEE Journal of Robotics and Automation*
  4(2), 1988.
- C. Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2005. Closest points on simplex
  features (5.1), GJK (9.5), bounding volume hierarchies (6).
- T. Moeller, B. Trumbore, "Fast, minimum storage ray-triangle intersection", *Journal of
  Graphics Tools* 2(1), 1997.

## Correctness and tests

- **GJK (`tests/unit/geom3d/ConvexTest.cpp`):**
  - exact distances between boxes, spheres, capsules, cylinders and triangles;
  - 200 random box pairs against exact gaps;
  - 3000 random capsule-triangle pairs against brute force;
  - a regression case for the segment aliasing.
- **BVH:** box queries and ray casts on a grid with a raised triangle.
- **Machine checks (`tests/unit/sim/MachineCheckTest.cpp`):**
  - a pocket clears the trunnion;
  - cutting deeper brings the spindle nose into the part and the stock;
  - a long tool plunging off the part reaches the table;
  - a 30° tilt over a tall part is clear;
  - a horizontal tool far out along X brings the spindle housing against a trunnion wall;
  - machines without components are skipped.
- **CLI:** the 4- and 5-axis samples simulate with no machine contacts. The 5-axis sample stands
  on a 20 mm riser; without it, roughing down to the table top is reported.

## Known limitations

- **Sampled poses** every 2 mm / 2°; thin parts of fast moves between samples can be missed.
- **Rapids** follow the straight tool-tip path, not the machine's own interpolation.
- **Simplified machines.** Components are convex boxes and cylinders; machine frames, linear
  axis carriages and fixtures are not modelled. The part is checked by its triangles, the stock
  by its box.
- **No display.** The machine is not shown in the viewer yet. (Machine models, DL-0037, show
  the machine and move its parts with every axis; the components became parts of its model.)
