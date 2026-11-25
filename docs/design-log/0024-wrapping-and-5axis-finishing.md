# DL-0024: 4th-axis wrapping and simultaneous 5-axis ball finishing

- **Status:** implemented (phase 9)
- **Module:** libs/strategies (`MultiAxis.hpp`: `wrapToolpath`, `radialMap`, `rotaryPocket`,
  `toolAxisFor`, `multiAxisFinish`), libs/geom2d (`isoRegions`), libs/document (`rotary_pocket`,
  `multiaxis_finish`)
- **Patent gate:** low (citation only), with a condition. Wrapping a flat toolpath onto a
  cylinder and fixed lead/tilt angles to the surface normal are long-published practice. The IP
  policy lists **automatic 5-axis tilting** as medium risk: here the tool axis always follows a
  rule with fixed, user-given values (lead and tilt, a direction, or a point), limited by a fixed
  largest tilt. Nothing tilts the tool to avoid collisions; clearance problems are reported
  only. Any automatic avoidance needs an IP review first.
- **Author(s):** OxCAM contributors

## 4th-axis wrapping

**Mapping.** A flat point (x, y, z) wraps onto the cylinder of radius R about the WCS X axis:
x stays, the angle is t = y / R (from +Z towards +Y), the radius is R + z, and the tool axis is
radial: p = (x, (R + z) sin t, (R + z) cos t), u = (0, sin t, cos t). On a 4-axis machine A = t
brings the point to the top. `wrapToolpath` splits straight moves (and sampled arcs) so the
chords stay within the tolerance at the outer radius of each piece. On a machine without tool
centre point control the posted X and A interpolate linearly along the helix, which is closer to
the intended curve than the chords.

**Radial height map.** `radialMap` casts rays towards the X axis (BVH, DL-0025) on a grid of x
and angle and records the largest radius of the part (0 where a ray misses).

**Wrapped pocketing (`rotary_pocket`).** For recesses in a part turning about X: slots, flats,
grooves, engraving.

1. **Levels.** Radial levels from the stock radius down to the deepest point of the part, at
   most the preset's axial depth (ap) apart, plus a level at every flat floor (radii shared by many map cells).
   Recesses shallower than 0.02 mm are the facets of the tessellated cylinder and are ignored.
2. **Region of a level ρ.** Where the part lies below ρ (less the floor allowance): beyond the
   part's ends everything is free, so features open at an end are cut through; inside, rays that
   miss are holes and are left alone.
3. **Unrolled plane per level.** The level is unrolled at its own radius (u = ρ t), so tool
   offsets are exact at the floor, where the flat end of the tool cuts. The seam goes where the
   level has the least free space; when even that angle is free (a ring groove), the plane is
   extended across the seam by the tool radius plus margin, and the overlap is cut twice.
4. **Contours.** Marching squares on the free-space field (`geom2d::isoRegions`). The field
   jumps at walls, so linear interpolation would misplace them by up to half a cell (a 0.25 mm
   gouge was seen in testing): each crossing is bisected on the exact predicate (a ray cast) to
   0.25 × the tolerance, on the blocked side, and chords that cut into blocked space get the
   boundary point across them.
5. **Toolpath.** Each region is pocketed with the 2.5D kernel (DL-0007) from the level above to
   the level, in the unrolled plane with heights relative to the level, then wrapped at the
   level's radius. Levels are joined by rapids around the clearance cylinder.

## Five-axis ball finishing (`multiaxis_finish`)

A ball end mill touches the part with its sphere. Wherever the ball centre is, the contact is
the same whatever the axis, so the 3-axis drop-cutter path (DL-0016, DL-0017), which is exact
for the sphere, stays valid when the shank is tilted.

1. The ball centres come from parallel finishing: c = tip₃ + r ez.
2. **Normals.** At points on the surface (the tip at the drop-cutter height) the normal is that
   of the ball-centre surface, from central differences of drop-cutter heights (step 0.05 r). In
   concave creases this averages the faces touched.
3. **Feed direction.** The cut arriving and the cut leaving, averaged and projected onto the
   tangent plane. At the start of a cut the one leaving, at its end the one arriving, so
   plunges and retracts do not count.
4. **Axis rules (`toolAxisFor`).**
   - Lead/tilt: u = cos τ (cos λ n + sin λ f) + sin τ (n × f). A positive lead leans the tool
     top forward, a positive tilt to the left of the feed.
   - Fixed: tilt from WCS Z towards an azimuth.
   - From a point: u = (c − P) / |c − P|.
   - To a point: u = (P − c) / |P − c|.
   - Axes steeper than `max_tilt` from WCS Z are brought back to it in their vertical plane.
5. **Tip.** tip₅ = c − r u. Approaches take the axis of the next surface point; retracts
   straight up keep the previous axis.
6. **Clearance report.** At every surface point a capsule a hair thinner than the ball, along
   the flutes above the centre, and the shank cylinder (with `clearance`) are tested against the
   part's triangles (BVH + GJK, DL-0025). Points where the flutes cut into the part or the shank
   comes too close are counted in warnings, with the first position. Nothing is changed.

Zig-zag passes with a lead flip the lean at every reversal, which swings the table half a turn.
The 5-axis sample uses one-way passes. Solve warns about remaining swings, for example where
the normal jumps across a crease or near the singular position where the axis is vertical.

## Sources

- W. E. Lorensen, H. E. Cline, "Marching cubes: a high resolution 3D surface construction
  algorithm", SIGGRAPH 1987. Marching squares is its 2D case.
- G. W. Vickers, K. W. Quan, "Ball-mills versus end-mills for curved surface machining",
  *J. Engineering for Industry* 111(1), 1989.
- C. G. Jensen, D. C. Anderson, "Accurate tool placement and orientation for finish surface
  machining", *J. Design and Manufacturing* 3, 1993. Lead/tilt tool orientation.
- B. K. Choi, R. B. Jerard, *Sculptured Surface Machining*, Kluwer, 1998. Wrapping, ball-centre
  offset surfaces, multi-axis tool orientation.

## Correctness and tests

- **Wrapping:** radial axes through every point, chords within the tolerance, half a turn ends
  at the bottom of the cylinder.
- **Radial map and wrapped pocket (`tests/unit/strategies/MultiAxisTest.cpp`):** on a shaft with
  a 3 mm deep slot over 1 rad with radial walls, every cutting point keeps the tool inside the
  slot (axial walls at ±r, radial walls at angle ±r/ρ, to 0.02 mm), the floor radius is reached
  to 1e-3, and lateral rapids below the stock stay over the slot.
- **Axis rules:** lead and tilt directions, the tilt limit, from-point and fixed axes.
- **5-axis finishing:** the ball centres equal the 3-axis ones to 1e-9; the axis leans 15° from
  the normal (2.5°, mesh facets) where not limited; no flute report on a convex dome, a report
  where the leaning tool reaches the dome's foot and against a wall tilted into; flat end mills
  are refused.
- **CLI:** `cli.project_simulate_4axis` (wrapped grooves on the rotary shaft) and
  `cli.project_simulate_5axis` (the dome on a trunnion, lead 10°) simulate with no gouge and no
  collisions.

## Known limitations

- **Wrapped pocketing below the surface** offsets in each level's own plane; walls that are not
  radial are followed level by level (steps of the axial depth ap). Planar floors (flats) are cut in
  radial steps; 3+2 indexing suits them better.
- **Flat-bottomed tools on cylindrical floors** leave cusps of about r² / (2ρ) between passes
  across the angle.
- **5-axis finishing uses 3-axis ball centres:** steep walls get the raster's sparse coverage;
  swarf (flank) milling of ruled walls is DL-0085; other multi-axis patterns do not exist yet,
  and only ball end mills are used here.
- **Normals in creases** jump; the table may swing there (reported).
