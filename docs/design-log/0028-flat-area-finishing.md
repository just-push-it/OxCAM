# DL-0028: Flat area finishing

- **Status:** implemented
- **Module:** libs/strategies (`flatLevels`, `flatFinish`, `src/Pocket.cpp`), libs/document
  (`flat3d`)
- **Patent gate:** low (citation only). Finishing the horizontal faces of a part with offset
  passes at their own heights is a long-established CAM practice; the method combines the
  contour-parallel pocketing of DL-0007 with the drop-cutter forbidden regions of DL-0016/0018.
- **Author(s):** OxCAM contributors

## Problem

After roughing, a part's horizontal faces (the top, pocket floors, steps between walls) still
carry the roughing allowance and the roughing tool's steps. Parallel and waterline finishing
handle sloped and steep surfaces but are poor on flats: rasters waste time and waterline skips
them. The operation finds every flat area between a bottom and a top height on its own and
finishes each one at its exact height with a flat-bottomed tool, without touching the part.

Input: the part as a triangle mesh in WCS (tool axis +Z), the top and bottom heights, the
stepover (a fraction of the tool's flat bottom diameter), stock to leave on the walls and on the
flats, the largest slope that still counts as flat, the entry and the cut direction.

## Approach

1. **Flat levels** (`flatLevels`): every triangle whose unit normal has z >= cos(max slope)
   faces up and is flat. Sorted from the top down, triangles whose centroid heights lie within a
   small tolerance of the group's highest one form a level at their area-weighted mean height.
   Each level's area is the union of its triangles' XY projections, closed by a tiny
   offset/inverse offset so facets that do not quite meet leave no hairline gaps.
2. **Reachable tool centres:** a drop-cutter height field of the real tool (DL-0016) gives the
   forbidden region of each level: tool centres where the tip at the flat's height would cut
   into the part ({drop > z}, DL-0018), grown by the wall allowance. The centre area is the flat
   area grown by a rim of a quarter of the flat radius (so outer edges are finished cleanly
   where nothing stands next to them) minus the forbidden region.
3. **Passes:** contour-parallel loops of the centre area (DL-0007) at the flat's height plus
   the floor allowance. The stepover is measured on the flat bottom radius (the full radius for
   a flat end mill, R - r for a bull nose), since only the flat bottom finishes the floor.
   - **Open flats:** for the outer loop of each connected centre area, the nearest pair of a
     point in air and a point of the loop whose straight segment crosses no forbidden area. A
     point in air is outside the stock (a tool radius plus a gap from it, when the stock outline
     is given), or beside the flat where the part drops away (the outside of a top face, a
     bore through it, the step below a boss): a tool radius plus a gap from the flat's own
     area, and the gap (a quarter tool radius, at least 1 mm, for the walls' roughing
     allowance) clear of every tool position that touches the part above the next flat down
     (or the part's bottom). The material above that lower flat is roughed, as the operation
     expects of every flat; above a rounded or chamfered edge it is not (the edge is not a
     flat, and roughing leaves its corner), so the plunge lands beyond the edge and its wall.
     A first version cleared only the part at the flat's own height and plunged just past the
     flat's edge, onto that corner. If such a pair exists within
     two tool diameters, the area is cut from the outside in: a plunge in air, a feed in to the
     loop, then each inner loop one stepover further in, so every loop takes at most a
     stepover (from the inside out the first loop slots the floor allowance at the full width;
     island tops far from the stock's edge were entered from above that way before).
   - **Closed flats** (pocket floors, walls all round): from the inside out, entered by the
     chosen entry (ramp along the loop, circular helix or plunge) from the feed clearance above
     the flat.
   - **Bands closed:** offsets a whole stepover apart leave the last loops from either side of a
     band up to two stepovers apart, and with the stepover near the flat diameter a strip in
     the middle stayed for the clean-up (short loops, each plunged into). Where a node's region
     holds points farther than the flat radius from its own loop and from its children's, a
     loop half a stepover in is added between them; with the stepover at most the flat
     diameter, that covers the floor.
   - Links at depth longer than two stepovers lift instead of dragging the tool across the
     finished floor, and hops within one connected area stay at the feed clearance above the
     flat instead of going up to the retract height (the area is clear of the part at the
     flat's height, so it is clear above it too).
4. **Clean-up:** the target (the flat area within reach of the centre area's flat-bottom sweep)
   minus what the loops swept is the residual, mostly small cusps at corners. A first pass cuts
   loops of (residual ⊕ 0.1·flat radius) ∩ centre area: tight loops around each cusp, which the
   flat bottom covers from close by. Whatever is still left (next to walls, where the tool
   centre cannot come close) gets the pass of pocketing that reaches every reachable point:
   loops of (residual ⊕ (flat radius + noise)) ∩ centre area. Clean-up loops are entered by a
   short plunge, since the floor around them is already cut. (A first version used only the
   wide pass, which swept a ring of finished floor about a tool diameter across around every
   cusp and linked between them at depth.)
5. Levels follow from the top down with a retract between them. A flat area no centre can reach
   (narrower than the tool) is reported in a warning.
6. **Machining over openings** (optional, "Machine over openings" with a "Max empty travel"):
   the holes of a flat's area are where its floor drops away (through holes, slots, pockets) or
   where something stands up (a post). A hole no wider than the max empty travel (no circle of
   that diameter fits in it: its inward offset by half the distance is empty) is filled before
   the centre area is formed, so the loops are planned as if the flat went on across it. Over
   a filled opening the drop-cutter finds nothing above the flat, so the tool crosses it in
   air at the flat's height; a post's top stands above the flat and stays forbidden. The width
   is the largest circle that fits, so a long narrow slot is crossed while a bore wider than
   the distance is gone around. The coverage target and the clean-up keep to the real floor
   (the flat's own area): nothing is cleaned up over an opening.

## Sources

- M. Held, *On the Computational Geometry of Pocket Machining*, LNCS 500, Springer, 1991.
  ISBN 3-540-54103-9 (contour-parallel offset pocketing).
- B.K. Choi and R.B. Jerard, *Sculptured Surface Machining: Theory and Applications*, Kluwer,
  1998. ISBN 0-412-78020-8 (cutter-location surfaces and drop-cutter).

## Deviations from sources

Flat detection works on the tessellated part rather than on B-rep planar faces, so all 3D
strategies share one input and parts imported as meshes work too; exactly planar faces give
exactly horizontal facets, so the slope tolerance only matters for nearly flat faces.

## Correctness and tests

`tests/unit/strategies/SurfaceTest.cpp` ("flat area finishing finds and covers every flat"), on
a boss on a step on a base:

- exactly three levels are found, at the right heights, with the boss top's area;
- feed moves are only at the flat heights (plus the floor allowance) and within the top/bottom
  limits;
- with a stock outline, the open base and step start with a plunge outside the stock, the boss
  top (far from the stock's edge) is entered from above, and coverage and no-gouge still hold;
- no gouge: a tool 5 um smaller in radius dropped at every feed point stays at or below it;
- sample points of each flat that the flat bottom can reach are swept by it;
- the wall allowance keeps the tool centre at least R + allowance from the boss;
- a ball end mill is refused with a warning.

`SurfaceTest.cpp` "open flats start in the air beside them": base, step and boss top each start
with a plunge beside them, none from above. "flat area finishing enters beside a flat clear of
its edge's stock": a round boss whose top has a 45° chamfer is entered by a plunge beyond the
chamfer and the wall (the first version plunged onto the chamfer's stock). "flat area finishing closes bands without clean-up
loops": a ring 18 wide with the stepover at the flat diameter is swept whole with one entry and
no plunged clean-up loop (before, the clean-up took a strip in its middle).

`SurfaceTest.cpp` ("flat area finishing machines over narrow openings"), on a plate with a slot
6 wide and a bore 16 across: without the option the passes go around both; at 10 they run along
the slot's middle and stay out of the bore (no gouge, no warning); at 20 they cross both.

`tests/unit/document/GenerateTest.cpp`: on the block fixture the operation cuts the top and the
pocket floor and stays inside the pocket on the floor.

## Selected faces

With faces selected, only the flats inside their plan view are finished (the flat areas are
intersected with it): selecting a pocket floor finishes that floor and not the top face. Tests:
`SurfaceTest` "flat area finishing on the selected flats only" (the boss top alone of three
flats), `GenerateTest` "flat area finishing on selected faces" (the block's pocket floor alone).

## Known limitations

- The operation finishes; it expects the material above each flat to be roughed already: it
  rapids down to the feed clearance above each flat before entering.
- Corners next to walls keep the tool's radius, and a bull-nose corner leaves its own fillet.
- The loops are offsets: where a region narrows to a cusp (the middle of a lobe between two
  arms), the material between consecutive loops at the cusp is wider than the stepover, and
  the innermost loop there takes it at up to nearly the full width over a few millimetres (on
  the maintainer's flange about 30 mm of 2300). Closed flats still start with a full-width
  loop.
- Stock left above a rounded or chamfered edge next to a flat (roughing cannot remove it, and
  it is not a flat) is not in the operation's picture: the loop along that edge takes it with
  the flat's allowance, wider and deeper than the stepover. Finish such edges first (3D
  finishing) or expect that load on the outer loop.
- Selected geometry does not restrict the operation yet; it finishes every flat in its height
  range.
