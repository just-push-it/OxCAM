# DL-0082: Pencil finishing and steep and shallow finishing

- **Status:** implemented
- **Module:** libs/strategies (`pencilCorners`, `pencilFinish` in `Pencil.cpp`; `shallowAreas`,
  `steepShallowFinish` in `SteepShallow.cpp`; `ParallelParams::byArea` in `Surface.cpp`),
  libs/document (`pencil3d`, `steep_shallow3d`)
- **Patent gate:** low (citation only), recorded below. The rest machining option of both
  operations is the existing in-process-stock containment and stays under
  [IP-0002](../ip/reviews/0002-rest-machining-from-in-process-stock.md) (medium, pending), as it
  does for contour finishing (DL-0033).
- **Author(s):** OxCAM contributors

## Problem

A review of the 3D toolpath coverage found two gaps:

1. **Corner clean-up.** A ball that finishes a part cannot reach into a concave corner or fillet
   tighter than itself. It touches the two faces either side of the corner at once and leaves a
   band of material between those two contact points. Nothing finished that band except running
   a whole finishing operation again with a smaller tool.
2. **Combined steep and shallow finishing.** Parallel finishing (DL-0017) leaves wide cusps on
   steep walls; waterline and contour finishing (DL-0017, DL-0033) leave wide flats on shallow
   areas. A part with both needed two operations, each contained by hand to its own faces, with
   nothing to make them meet.

**Pencil finishing** (`pencil3d`): inputs are the part mesh, the tool (ball, bull nose or flat
end mill), a reference tool diameter (the larger tool that cut before; unset, the operation's
own), a corner angle, optional offset passes each side with their stepover, stock to leave, and
the containment. It outputs single passes along each corner the reference tool could not reach
into, and optional offset passes beside them.

**Steep and shallow** (`steep_shallow3d`): inputs are the part mesh, a ball or bull nose, a
threshold angle, an overlap, the steep areas' step down (the preset's ap) or cusp height, the
shallow areas' stepover (the preset's ae) or scallop height, the parallel passes' angle and
pattern, the order, the heights and the containment. It outputs one toolpath: constant-Z passes
where the surface under the tool is steeper than the threshold, and parallel passes where it is
shallower. Both reach half the overlap past the line between them.

Invariants: every point is on or above the drop-cutter surface (no gouge), every feed is the
operation's cut, plunge or link feed exactly as set (ADR-0004), and generation refuses, naming
the value, where one is missing.

## Approach

### Pencil finishing

1. **Where the reference tool touches twice.** The reference tool is the operation's tool grown
   to the reference radius: `Cutter::offset(stock + R_ref - R)`, so a ball stays a ball and a
   flat end mill becomes a bull nose. Its cutter-location (CL) surface comes from the existing
   drop-cutter (DL-0016) as a height field with spacing h (default R/4, between 0.05 and 0.5 mm).
   Where the tool rests on one face the CL surface is smooth. Where it rests on two faces at once
   it has a concave crease: the slope jumps from one face's to the other's. A vertical wall gives
   a cliff in the height field, and its foot is such a crease.
2. **Finding the creases.** At each grid point, in X and in Y, the section's direction turns
   between the chords before and after the point. Where it turns up (concave) by at least half
   the corner angle, the bend is refined on the exact drop-cutter. Within a cell either side,
   33 drops are taken and the sharpest turn is read between the chords on either side of each
   chord, so a bend between two samples counts whole. A bend that holds its angle at that fine
   scale is a crease. A smooth concave blend loses its angle as the samples close in (about
   2δ/ρ for curvature radius ρ). Points where a drop lands on the floor, off the part, are left
   out. So is a crease below the part's bottom, where the tool rolls off the part's lower edge.
3. **Chains.** The crease points are merged within 0.35 h and joined into chains. Each step goes
   to the nearest unused point within 1.6 h that turns less than 100° from the chain's
   direction. Points close beside a chain are absorbed into it. A chain whose ends meet is
   closed, and a light 1-2-1 smoothing removes the zig-zag of points found from two grid
   directions.
4. **Across each chain point.** The section across the chain (normal to its local direction) is
   read again on the exact drop-cutter. This gives the crease's exact position and the reference
   tool's slope on either side, φ₁ and φ₂: the directions of the two faces at its two contact
   points. A point whose turn φ₂ − φ₁ is below the corner angle is dropped.
5. **The pencil pass.** On the tool's own CL section through the crease, the pass point is where
   its slope is tan((φ₁ + φ₂)/2). There the tool's contact normal halves the reference tool's
   two contact normals, which puts it in the middle of the band the reference left. The slope of
   a concave section grows across the corner, so the point is bracketed by stepping from the
   crease in quarter cells and then bisected (central differences of exact drops). When the tool
   is itself larger than the corner, its own section has a crease there too and the bisection
   lands on it: the pass is where the tool touches both faces. When it is smaller (a 4 mm ball in
   a 6 mm fillet), the pass rides the fillet halfway round.
6. **Offset passes.** From the pencil point, along the tool's own section either way, the 3D
   arc length of the CL section is accumulated and a pass point is set every stepover. Each side
   stops at the set number of passes, or where the tool's contact passes the reference tool's
   contact on that side (R_ref sin φ from the crease) by more than half a stepover, or where the
   section gets steeper than 80°. That last one leaves a vertical wall's band to constant-Z
   finishing. The stepover is from the scallop height (2√(2ρh − h²) for the tool's corner radius
   ρ) or the preset's ae; it is only needed with offset passes.
7. **Pieces and order.** Each pass's points form pieces, split where consecutive points are more
   than 3 h apart and simplified by Douglas–Peucker to the tolerance. A closed chain whose pass is
   complete stays a closed loop. The corners are cut nearest first. Each corner is swept across
   its passes from the outermost on the nearer side to the outermost on the other. A pass next to
   the last one is reached along the surface (a dropped link, at the cut feed, class Link), and
   anything else by a retract and a feed plunge from the retract height, since the reference may
   have left a lot in the corner.
8. **Containment and rest.** Pass points outside the containment are dropped. The containment
   is the selected faces' plan grown by the tool radius, and with rest machining it is limited to
   where the simulated in-process stock stands above the part, the same as contour finishing.

### Steep and shallow

1. **The split.** The tool's own CL surface comes from drop-cutter on a grid (the surface
   resolution, default R/6). Its slope at a point is the slope of the part where the tool touches
   it. The field tan(threshold) − |∇CL| (central differences) is positive where the surface is
   shallow. Points off the part, or next to where the tool falls off it, get −1. The shallow area
   is that field's zero contour, traced by marching squares with linear interpolation
   (`geom2d::isoRegions`). Specks and nicks one or two cells across are removed by offsetting in
   h, out 2h and in h.
2. **The overlap.** The shallow zone is the shallow area grown by half the overlap, within the
   part's plan bounds (parallel finishing's own area). The steep zone is everything except the
   shallow area shrunk by half the overlap. Both are intersected with the containment.
3. **The passes**, from the existing kernels with those zones as their containment:
   - Steep: waterline (DL-0017) with the preset's ap as the step down, or contour finishing
     (DL-0033) with the steep cusp height and its shallowest wall set to the threshold, when a
     cusp height is set. Their loops are clipped to the zone.
   - Shallow: parallel finishing with the shallow scallop height or the preset's ae as the
     stepover. Its rasters are clipped to the zone.
4. **Parallel passes area by area.** On a split surface a raster line crosses the shallow zone
   several times (each side of a dome, around a pocket), and line-by-line ordering lifts the
   tool between every piece: 153 entries on the test dome. `ParallelParams::byArea` (used here,
   off for `parallel3d`) splits the runs into areas that each line crosses once. A run continues
   the area of the run on the line before that it overlaps, when each overlaps only the other.
   Otherwise it starts a new area. Each area is then zig-zagged through all its lines before the
   next, the nearest first, entered at whichever end of its first or last line is nearest. On
   the dome that gives 17 entries.
5. **Entries.** Contour finishing comes down by rapid to the feed clearance above each level
   it enters, which is safe when it works from the top down. Here a level's pieces can start
   part way down, with the roughing's stairs still standing above them: the shallow areas
   above come later or not at all. The first simulation of the dome project found three rapid
   collisions of up to 0.63 mm. So the rapid before each of its plunges stops at the retract
   height, and the plunge runs from there at the plunge feed, as parallel and waterline
   finishing enter.
6. **One toolpath.** The two halves are joined in the chosen order (steep areas first by
   default), with a rapid between them at the clearance height or the retract height over the
   part, whichever is higher. Both use the same tool and the
   operation's cut, plunge and link feeds. There is no feed per region (ADR-0004); a different
   feed for one of them would need its own labelled parameter.

## Patent gate

- **Category.** Neither method is in the IP policy's high or medium categories. Neither has
  engagement control, feed optimization, automatic tilt, or in-process-stock rest machining
  (the optional rest machining excepted, which stays under IP-0002). Pencil finishing is
  corner clean-up from geometry alone: a reference tool radius that the user sets, read
  through drop-cutter, which the policy lists as low risk. Steep and shallow combines two
  low-risk methods, waterline and raster finishing, with a slope test that only decides their
  containment. **Outcome: low, citation only.**
- **Prior art.** Both are long published:
  - Pencil (bitangent) clean-up of polyhedral models and of sculptured surfaces:
    Ren, Yau and Lee (2004), and Kim, Jun and Park (2005), below.
  - Splitting finishing by surface slope into Z-level and raster regions: the review by
    Lasemi, Xue and Gu (2010), and the Z-level and cutter-location treatment in Choi and
    Jerard (1998).
- **Search.** No patent search was done. The IP policy assigns that to the review group, and
  low-risk methods need a design-log citation only. Commercial CAM documentation of similar
  features was not used as a design input.

## Sources

- B. K. Choi, R. B. Jerard, *Sculptured Surface Machining: Theory and Applications*, Kluwer,
  1998. Covers cutter-location surfaces, scallop height, and raster and Z-level finishing.
- Y. Ren, H. T. Yau, Y.-S. Lee, *Clean-up tool path generation by contraction tool method for
  machining complex polyhedral models*, Computers in Industry 54, 2004, 17–33.
  DOI 10.1016/j.compind.2003.09.003. Covers clean-up (pencil) regions found with tools of
  different sizes, and strip-parallel clean-up paths.
- Kim, Jun and Park, *Tool path generation for clean-up machining by a curve-based approach*,
  Computer-Aided Design 37(9), 2005. DOI 10.1016/j.cad.2004.09.023. Covers pencil-cut
  and fillet-cut paths for STL models with a ball-end mill.
- A. Lasemi, D. Xue, P. Gu, *Recent development in CNC machining of freeform surfaces: A
  state-of-the-art review*, Computer-Aided Design 42(7), 2010. Surveys finishing strategies,
  iso-planar (raster) and Z-level among them, and where each suits a surface's slope.
- C.-M. Chuang, H.-T. Yau, *A new approach to z-level contour machining of triangulated surface
  models using fillet endmills*, Computer-Aided Design 37(10), 2005. Covers the waterline kernel
  reused here.

## Deviations from sources

- **Corner detection.** The sources find bitangent (pencil) points by offset-surface
  intersection or by contracting tools. Here they are the creases of the reference tool's
  drop-cutter surface, refined on exact drops, because the drop-cutter is the one 3-axis query
  the project has (DL-0016) and it handles every torus tool and stock allowance alike.
- **Pass placement.** The pass runs where the tool's contact normal halves the reference tool's
  two contact normals: in the middle of the band left behind. The sources place it at the tool's
  own bitangent point. Both agree when the tool is larger than the corner.
- **Offset passes.** They are spaced along the tool's CL section across the corner and stop at
  the reference tool's contact. They are not 3D offsets of the pencil curve.
- **Steep and shallow.** It adds no kernel: the slope only decides the containment of the
  existing waterline, contour and parallel kernels. Parallel finishing gained area-by-area
  ordering for it.

## Correctness and tests

`tests/unit/strategies/SurfaceTest.cpp`:

- "pencil passes run along the fillet tangency": a channel with vertical walls and radius-3
  floor fillets.
  - A 4 mm ball after a 12 mm reference runs one pass along each fillet over the channel's
    length. Its centre is 1 mm (r_f − r) from the fillet's axis at 45° (within 0.01 to 0.02 mm),
    so the ball touches the fillet halfway between the wall's and the floor's tangency lines.
  - An 8 mm ball on its own runs where it touches the wall and the floor (x = wall + r, tip on
    the floor).
  - Offset passes, 0.4 mm apart, stay on the drop-cutter surface. They go no further than where
    the reference touched the floor plus a stepover, and none go up the wall.
  - A containment keeps the passes inside it.
  - A smooth bump gives no passes and a warning.
  - A reference smaller than the tool is refused.
- "steep and shallow finishing covers the dome within the scallop height": a spherical cap
  (70.5° at its foot) on a plate, a 6 mm ball, a 40° threshold, an overlap of 1 mm, and cusps of
  0.02. At every mesh vertex the ball can reach (not the plate's sides, not within r of the
  cap's foot), the distance from the surface to the nearest ball along the path is checked:
  - never under −0.003 (no gouge);
  - on shallow areas at most 1.25 × 0.02 / cos²(slope) + 0.003 (parallel passes' cusp on their
    slope);
  - on steep ones at most 1.25 × 0.02 + 0.003.
  
  Measured: 0.022 at worst on shallow areas and 0.015 on steep ones. Parallel passes alone at
  the same scallop leave more than the limit on the steep sides, and over twice the steep worst.
  The operation has both constant-Z and surface-following runs, and fewer than 20 entries from
  above. No rapid goes across below the retract height, between the two halves included.
- "steep and shallow areas split at the threshold": the shallow area of the dome ends at the
  radius where the ball's centre sphere is 40° steep, and the plate beyond the cap's foot is
  shallow again.

`tests/unit/document/GenerateTest.cpp`:

- "pencil finishing in the filleted pocket's corners" (new fixture `fillet_pocket.step`: a
  pocket with radius-8 corners and radius-3 floor fillets): one closed loop with one entry, on
  the fillet halfway round.
- "steep and shallow finishing on the dome": both kinds of passes, the dome's top reached, and
  generation refused without the preset's ap.

Simulation:

- `cli.project_simulate_pencil` (`fillet_pocket_project.ocam`: roughing, steep and shallow with
  a 10 mm ball, pencil with a 4 mm ball and offset passes) and
  `cli.project_simulate_steep_shallow` (`dome_steep_shallow_project.ocam`) run with no gouge
  and no collision ("result: ok"). On the pocket at 0.25 mm rays, the stock left over the
  0.02 mm tolerance falls from 621.8 mm³ to 220.4 mm³ with the pencil operation.
- `gui.simulation_pencil` and `gui.simulation_steep_shallow` show the simulated stock.
- `FixturePostTest` posts both projects.

## Known limitations

- **Walls above a corner.** Offset passes do not climb walls steeper than 80°. On a vertical
  wall above a fillet tighter than the reference, the band the reference left (up to its
  radius above the floor) needs constant-Z passes: waterline, contour, or steep and shallow
  with the small tool.
- **Corners and junctions.** Where creases meet (a vertical corner meeting the floor's fillet),
  chains may break at the junction and the pass is entered again. A plan corner in a chain is
  cut as a short chord.
- **Monotone areas only.** Area-by-area parallel ordering splits only where a line meets the
  area a different number of times. Inside one area, lines whose ends are more than 2.5
  stepovers apart (near the top and bottom of a round area) are still entered from above.
- **Slope at the zone edge.** The split follows the CL surface on a grid. The overlap hides the
  grid's stairs, but a threshold close to a long constant slope (a 40° cone at a 40° threshold)
  gives ragged zones; pick a threshold away from the part's main slopes.
- **Accuracy.** Bounded by the tessellation and the height-field spacing, as for the other 3D
  operations (DL-0016).
