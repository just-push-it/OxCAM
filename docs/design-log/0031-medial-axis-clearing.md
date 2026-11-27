# DL-0031: Medial-axis clearing (Z-level roughing on maximal inscribed circles)

- **Status:** implemented; the method of the 3D roughing operation (`zlevel_rough`, "3D
  roughing"). It was first a third operation type (`zlevel_medial`) beside the traced clearing
  of DL-0018 and the front clearing of DL-0030; compared on the maintainer's parts it was
  chosen, the front clearing removed and the traced clearing kept for open areas only. Projects
  with `zlevel_fronts` or `zlevel_medial` operations load them as `zlevel_rough`, without the
  traced clearing's "clearing" choice (the engagement limit is always kept).
- **Module:** libs/strategies (`detail::clearLevelMedial` in `Medial.cpp`, the axis in
  `MedialAxis.cpp`; the level planning, rest material, order and reporting of `zLevelRough`
  are shared)
- **Patent gate:** [IP-0004](../ip/reviews/0004-engagement-controlled-roughing.md) (engagement
  control; the maintainer's decision, counsel not consulted). The path construction is the
  published MATHSM scheme below (2003/2005); the medial axis and its maximal circles are
  classical geometry.
- **Author(s):** OxCAM contributors

## Problem

The traced clearing (DL-0018) and the front clearing (DL-0030) both derive their passes from
offsets of the material and decide where to cut by measuring; in a slot the passes degenerate
to short arcs sliced across it, entered at an angle, with the pieces chosen by distance. The
maintainer, after both, asked for the toolpath to be built from the shape of the area, and
pointed to Elber, Cohen and Drake's MATHSM as the reference: the tool runs on the largest
circles that fit the pocket, strung along its medial axis.

## Approach

MATHSM's construction, with the project's exact material and engagement grid around it.

1. **The axis.** The tool-centre region of a piece (the allowed area within the tool's reach of
   material) is rasterised at the clearing cell; the squared Euclidean distance transform
   (Felzenszwalb and Huttenlocher's lower envelope of parabolas, separably) gives the inscribed
   radius at every cell; the cells whose disc no neighbour's disc holds (a neighbour's
   distance would be the cell's plus the step between them, less a fifth of a cell of slack:
   a ridge test along the grid's directions missed crests at other angles and left gaps in
   the axis) are thinned (Zhang and Suen, then the staircase corners it keeps for
   4-connectivity, one at a time while the neighbours stay 8-connected without them) to
   one-cell chains, the deepest cell always kept, and the chains joined into a graph of
   branches between nodes, each branch point carrying its radius. The graph
   is then cleaned until nothing changes: a branch's tail into a rounded end (a ridge of the
   distance but not of maximal circles: each lies within the circle at the end arc's centre)
   is trimmed back to the last circle that holds all beyond it, once per end; a spur to a dead
   end shorter than three cells, or whose circles all lie within its junction's circle (a
   boundary corner's own ridge, the pixel noise of a plateau), is removed; a bridge shorter
   than two cells between junctions is contracted (a junction the raster split); a node left
   with two branches joins them; loops shorter than the prune length go; of the pieces left,
   the longest is the axis (a connected area has a connected axis; the rest is debris). An
   area whose axis is a point (a disc) keeps that point. Not an exact Voronoi diagram, which
   MATHSM takes from VRONI: the raster reads the radius to about half a cell, which is why
   every decision below is measured on the material rather than trusted to the axis.
2. **The circles.** A branch is walked from the last circle cut; the next circle is the first
   point along it whose crescent beyond the last (|Δc| + Δρ at its widest, MATHSM's load
   between successive circles) reaches the stepover, or the branch's end: nearer where the
   circles grow, further where they shrink. Its engagement is read on the grid before it is
   cut (the project's reading: the half circle facing the motion, the wake cleared); over the
   limit, the spacing is cut back by 0.7 until within it, to a fifth of a stepover, where the
   branch stops (it is thinner than the tool can take within the limit) and the material is
   reported. A circle that stands out of the last cut one by less than a fifth of a stepover,
   or whose tool meets no material, is passed over: MATHSM's purge of circles that remove
   little, without which a branch narrowing as fast as it advances stands still. A circle
   under 0.3 r (the helix's own minimum) is passed over too: the tool on one that small in a
   corner meets both walls at once, a spike the reading along the circle does not catch; the
   wall pass takes the corner within the limit. The reading along a circle is taken at every
   cell (the clearing step is three cells and stepped over a corner's spike), its direction
   from two steps each way where the path runs straight and one where it turns.
3. **The path.** Successive circles are joined by their external bi-tangent on the side the tool
   leaves by going round (the right of the advance when climbing, anticlockwise circles): the
   tool rounds the circle it is on to the tangent point (at least the half turn through its
   front, so the crescent is cut, unless the circle was rounded already), runs the tangent
   line (which takes the cusp between the two circles on that side) and goes on round the
   next; at a leaf of the axis the last circle is completed. The path is C¹ at the joins, as in
   the paper, where it runs on material (item 9 crosses the air straight). Arcs are emitted in
   quarters.
4. **The traversal.** Depth first over the axis from the root; before each further branch out of
   a junction the tool links back at depth along the centres of the circles cut since (each a
   cleared disc), a hop only when that way is not clear: MATHSM retracts at every leaf of the
   central scheme, or halves the radii in its one-sided scheme to avoid that; the links are
   our way out of both. The root is the largest circle; among circles as large as it to a
   cell (a slot of even width), one at an end of the axis nearest the tool, so the walk runs
   one way with no link back.
5. **The entry.** A closed area starts with a helix (or a ramp on a circle) at the root, of the
   helix radius or the root circle less a cell, whichever is smaller; none of at least 0.3 r
   fitting, the piece is left (no plunge, as in DL-0018, and not reported as leftover). From
   the helix an Archimedean spiral grows to the root circle at the stepover per turn or less,
   its turns rounded up so it ends at the angle the first bi-tangent leaves from, with more
   turns where the measured engagement asks; then once round the root circle. The plunge
   entry plunges and spirals out from the centre.
6. **Open areas.** Areas with air to start from (the outside of a part) are cleared by the
   traced clearing of DL-0018 unless the operation's "Open areas" parameter asks for circles
   there too: by the paper's own account the circles spend about half their length in air and
   are at their best in closed, elongated regions, and MATHSM's authors recommend a hybrid.
   With circles asked for, an open area starts by a helix like a closed one, or from the air
   when the tool at the root is already clear.
7. **The walls.** The circles leave cusps between them along the walls (s²/8R for spacing s and
   circle radius R: 0.4 mm on the maintainer's slots). One pass round the area's edge, a cell
   and a half inside it, takes them, away from any body thicker than a stepover within reach,
   within the limit (split where over it), started at its point in air nearest the tool so it
   needs no lead-in.
8. **Bookkeeping.** The grid is cleared as the tool goes (every decision reads it); the exact
   material is settled once after the circles and once after the walls (one subtraction of
   all the cuts), for the wall pass and the remaining report of DL-0018.
9. **Crossing the air.** Each step (the arc on a circle and the tangent off it, a circle
   finished at a leaf, the root circle) is read on the grid as it would be cut, the tool's
   wake cleared as it goes. A stretch where no material lies within the tool's radius is
   replaced by a straight line at depth from where the tool leaves the material to where it
   meets it again, when the line is clear by the links' own test (DL-0018: the allowed area,
   no material within the radius less a cell, no rest material above) and shorter than the
   stretch by a fifth of a stepover or more. The line is a link at the cutting feed, the value
   the user set (ADR-0004), as are the links of DL-0018. A stretch in air at the end of a step
   waits for the next step, so that one line crosses both the tangent off a circle and the
   start of the next; where that line is not clear, the line that saves most among those from
   and to the joins between arcs and tangents in the stretch is taken, else the arcs are kept.
   The stretch removed nothing, so what the tool meets after the line is what it would have
   met on the circles: the engagement and its limit (IP-0004) are unchanged. The maintainer
   asked for it after a rest roughing whose circles ran mostly through air ("once it's out of
   material, it could just cut straight across to where it's about to engage material
   again"). The air test counts any material cell within the radius. With a cell's tolerance,
   as the other tests here have, the skipped arcs left rings a cell thick that the next
   circles' reading took for engagement, and the branches stopped short (700 mm² left on the
   L-shaped boss of the tests).

   On the maintainer's flange the 1/4" rest operation's upper level, where the 1/2" operation
   left material on the slots' outer walls only, now cuts each circle's half on the material
   and crosses the other half straight (94 more links, 351 mm of them); its other levels and
   the other operations do not change:

   | Setup 1, 1/4" rest | Moves | Length | Estimate |
   |---|---|---|---|
   | Circles whole | 1596 | 5663.7 mm | 173.0 s |
   | Air crossed | 1459 | 5307.7 mm | 162.5 s |

   The simulation finds no collision and the same gouge as before (from other operations);
   1.6 mm³ less is removed.

   **Lifted.** The maintainer asked for the line to lift so the tool does not drag on the
   floor. 3D roughing's *Linking move lift height* (`link_lift`, 0.1 mm by default; 0 = at
   depth) raises it. The line rises at 45 degrees to that height above the level, crosses, and
   comes down the same way to where it meets material. On a line shorter than twice the lift,
   it rises and falls more steeply. The tool stays over the line's own ground: the line is clear
   at depth, and clear of the material above the level (`overhead`), so the column over it is
   clear too. The moves are links at the cutting feed, as before. Only these lines are lifted.
   The links at depth from one circle to the next (DL-0018) still run on the floor. On the
   titanium test part (DL-0018), 50 lines are lifted: 3231 moves become 3331, the estimate goes
   from 600.9 s to 601.1 s, the removed volume is unchanged, and the simulation finds no
   collision. `ClearingTest` "the lines across the circles' backs rise by the link lift".

## Sources

- G. Elber, E. Cohen, S. Drake, *MATHSM: medial axis transform toward high speed machining of
  pockets*, Computer-Aided Design 37(2), 2005 (preprint 2003): maximal inscribed circles along
  the medial axis, successive circles joined by external bi-tangent lines (or arcs) for a C¹
  path, the material between successive circles as the load, the helix down a circle, the
  central and one-sided traversals, the purging of circles that remove little, the hybrid
  recommendation for large rounded regions.
- P. F. Felzenszwalb, D. P. Huttenlocher, *Distance transforms of sampled functions*, Theory of
  Computing 8, 2012 (the separable squared-distance transform).
- T. Y. Zhang, C. Y. Suen, *A fast parallel algorithm for thinning digital patterns*, Comm. ACM
  27(3), 1984.
- H. Blum, *A transformation for extracting new descriptors of shape*, 1967 (the medial axis).

## Deviations from sources

MATHSM computes the medial axis exactly (VRONI) and bounds the load only through the spacing of
its circles; here the axis is a cleaned raster ridge and the spacing is measured against the
engagement limit on the material grid, so the limit of IP-0004 is kept explicitly and the raster's
inaccuracy does not reach the cut. The paper's one-sided scheme (half radii, no retractions) is
not used: retractions are replaced by links at depth through the circles already cut. MATHSM's
path is C¹ throughout and runs whole circles; here the stretches that cut nothing are crossed
straight (item 9). The
paper's bi-tangent arcs of the outline's curvature (its §4) are not used; the wall pass takes
the cusps instead. The hybrid the authors recommend is the default: open areas are traced.

## Correctness and tests

- `ClearingTest` "the medial axis of simple areas": a long rectangle gives one branch along the
  middle at half the width; a disc its centre at its radius.
- `ClearingTest` "Z-level roughing keeps the engagement within the limit": the square
  pocket entered by a helix, spiralled out and circled; the narrow closed slot at 0.8 D (circles
  centred on its axis, the replay within the limit, under 14 mm² left); the L-shaped boss traced
  by default and circled when asked, both within the limit. All with the replay of DL-0018,
  which also measures the cutting moves on which the tool meets nothing: in the slot (the
  circles' backs, already cut) 95 mm against 189 mm with whole circles, under 130; on the
  L-shaped boss circled (the air outside the stock) 394 mm against 1704 mm, under 600.
- `ClearingTest` "Z-level roughing: no room for a helix, the order of areas": the hole too
  small for a helix left without a plunge or a cut (plunged into when the plunge entry is
  chosen); two pockets cut level by level, or each through its levels before the other.
- `ProjectTest` "the trial roughing types load as 3D roughing": the migration above.
- On the maintainer's flange (1/2" bull nose, ap 19.05, ae 6.35; 1/4" rest at ae 5.08) the
  independent replay finds 0.0 mm of travel over the limit on all three levels of the 1/4"
  rest op (largest 130.5° against a limit of 139.5°), the slots each a helix at the end
  nearest the tool, twelve circles along the axis joined by tangents on one wall, the end
  circle at the end arc's centre, one pass round; 138 s against the traced 157 s and the
  fronts' 138 s. The 1/2" op (traced outside, the round pocket spiralled) is 0.2–0.8 mm over
  the limit, 112 s against 110 s. The project generates in 30 s. On the maintainer's own
  copy of the project (stock to leave 0.508, area by area) the rest op is 0.0 mm over on both
  levels (largest 89°), 135 s, nothing reported; a first version left 118 mm² at one slot's
  end because the ridge test's gaps cut its axis short.
- `ClearingTest` "the medial axis of simple areas" and the slot section above check the axis
  runs the slot's whole length and a disc keeps its centre.

## Known limitations

- The raster axis of a wide, nearly round region is a few short branches at its centre; the
  spiral and the circles there are MATHSM's picture of a large rounded pocket (its authors'
  own example of the scheme's inefficiency), with circles from the centre into each corner.
- The cusps along the walls wait for the wall pass; a region the wall pass cannot reach within
  the limit keeps them for the finishing operation.
- A branch thinner than the tool can take within the limit is left from where its circles
  degenerate, and reported.
