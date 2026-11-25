# DL-0033: 3D contour finishing (cusp-spaced constant-Z passes)

- **Status:** implemented
- **Module:** libs/strategies (`contourFinish`, `ContourFinishParams` in `Surface.hpp`),
  libs/document (`contour3d`)
- **Patent gate:** low (citation only). Constant-Z finishing with the step down set from a
  scallop height and the wall slope is long-established (waterline/Z-level finishing in the
  literature below); no claim-like features (no constant-engagement control).
- **Author(s):** OxCAM contributors

## Problem

Waterline finishing (DL-0017) steps down evenly. On a curved blend (the large radius on the
maintainer's flange's top edge) an even step leaves cusps that grow as the surface flattens: a
step that is fine on the steep part leaves wide flats near the top, and a step fine enough for
the top wastes passes on the steep part. The maintainer asked for a contour finishing operation
that goes around and around, stepping down each time, with the number of steps set by a cusp
height, as the capability that blends curved surfaces.

## Approach

1. **Contours.** Each level's passes are the boundary of the tool-centre region where the tool
   tip at that height would cut the part (the drop-cutter forbidden region of DL-0016/DL-0018,
   with the stock to leave), in the cutting direction (climb by default): constant-Z loops
   around every wall the level meets.
2. **Cusp width.** Passes s apart along the surface, cut by a radius rho in contact (a ball's
   radius, a bull nose's corner radius), leave a cusp h = rho - sqrt(rho^2 - s^2/4); the width
   allowed is s = 2 sqrt(2 rho h - h^2). On a wall of slope alpha that is a step down of
   s sin(alpha). A flat end mill leaves stairs instead: a step dz leaves dz cos(alpha) normal to
   the surface, so the step allowed is h / cos(alpha).
3. **Levels from the top down.** At each level the slope is read at points along its loops
   (about every surface resolution), from the drop cutter a small step to either side across the
   loop. Points shallower than the "shallowest wall" (5° by default) are left out of the
   spacing: near-flat areas would need endless levels and are the flat or parallel finishing's.
   The step is the smallest allowed over the points, checked again at the trial level below
   (a concave fillet flattens on the way down) and its own loops reused when the step stands,
   clamped between a quarter of the cusp and the "max step down" (0: as the cusp allows). The
   first level is where the part begins below the top height (found by bisection after a level
   with no loops), the last at the bottom height.
4. **Linking.** Within a level the loops go nearest first. Down to the next level the tool
   stays at depth when the next loop starts close by (within the larger of two steps, four cusp
   widths and the tool radius): a short link along a straight line, kept at or above the tool's
   contact height on the part (dropped), so it never gouges; otherwise it lifts and plunges at
   the plunge feed from the feed clearance.
5. **Smoothing.** With "Smoothing (arcs)" on (the default) the loops are fitted with XY arcs
   within the smoothing tolerance (DL-0032).
6. **Containment.** Selected faces contain the operation: the tool centres stay within the
   faces' plan view (the union of their triangles' projections, hairline gaps closed) grown by
   the tool radius, so the tool reaches every point of the faces and goes no further, and the
   heights default to the faces' own (their mesh's lowest and highest points). The loops are
   clipped to the containment and the pieces inside are cut as they are, in the cutting
   direction, with their own entries; only points of the pieces set the step (a shallow blend
   elsewhere no longer slows the selected wall), and the surface is sampled only over the
   containment's bounds. Waterline finishing clips its loops the same way, and parallel
   finishing limits its rasters to the same area.
7. **Rest machining.** With "Rest machining" on, the containment is also limited to where the
   setup's simulated in-process stock (DL-0012, DL-0018) stands more than "ignore rest thinner than"
   above the part, grown by the tool radius and a margin; nothing left gives an empty toolpath
   with a warning.

The contour tracing now refines each crossing of the height grid by false position (Illinois)
from the grid's own heights instead of halving, with every third step halving so a wall the
height jumps at still closes in; smooth walls take two or three drops where halving took eight.
That speeds up waterline, roughing and flat finishing, which share the tracing.

## Sources

- B.K. Choi, R.B. Jerard, *Sculptured Surface Machining: Theory and Applications*, Kluwer, 1998
  (cutter-location surfaces, scallop height and path interval, Z-level/contour machining).
- A. Lasemi, D. Xue, P. Gu, *Recent development in CNC machining of freeform surfaces: A
  state-of-the-art review*, Computer-Aided Design 42(7), 2010 (Z-level finishing with
  slope-dependent step down).

## Correctness and tests

- `SurfaceTest` "contour finishing spaces its levels by the cusp height": a round boss with a
  radius of 6 on its top edge and a vertical wall, a 6 mm ball, a cusp of 0.01: consecutive
  passes are at most 1.15 cusp widths apart along the tool centre's surface (sqrt(dr^2 + dz^2)),
  the step on the vertical wall is about the cusp width and near the top of the round under
  half of it, the loops come out as arcs, and no feed point gouges beyond the smoothing
  tolerance (a ball 5 µm smaller, within that sideways of each point, stays at or below it).
- On the maintainer's flange (1/4" bull nose with a 0.254 mm corner, cusp 0.01 mm, the top
  7 mm): 75 levels, the steps 0.014 mm near the flat top growing to 0.14 mm (the corner's cusp
  width) on the steep part, the passes tracing the edge radius out from x = 144.9 to 151.4 at the
  right lobe, all loops as XY arcs; 77 s to generate.

- `SurfaceTest` "waterline and contour finishing stay inside their containment": on a round
  boss contained to x >= 2, every cut end of both lies inside, and the cut length is between 0.3
  and 0.6 of the whole (81° either side of the X axis).

## Known limitations

- Without selected faces every wall in the height range is contoured (the bore, slots and
  holes too).
- The containment is in plan view: a selected wall's plan view is a thin band, so walls of
  other faces that pass under that band (a lower wall right beneath an overhang) are cut too
  when they lie in the height range.
- A small corner radius needs many levels for a small cusp (0.14 mm steps for the 1/4" bull
  nose at 0.01 mm): a ball end mill is the tool for blending.
- The levels are set by the shallowest steep-enough point of all the loops at a level, so a
  shallow blend on one feature sets the step for all of them.
- Generation traces two contours per level and reads the slope with the drop cutter: about a
  second per level on the flange.
