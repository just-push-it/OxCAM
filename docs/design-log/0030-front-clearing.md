# DL-0030: Front clearing (Z-level roughing by exact offset fronts)

- **Status:** withdrawn. It was an operation of its own (`zlevel_fronts`, "3D roughing
  (fronts)") beside the traced clearing, to be compared on real parts; the maintainer found
  its paths worse than the traced ones and chose the medial-axis clearing
  ([DL-0031](0031-medial-axis-clearing.md)), and the code was removed. Projects with
  `zlevel_fronts` operations load them as 3D roughing (`zlevel_rough`). Kept as the record of
  what was tried.
- **Module:** libs/strategies (`detail::clearLevelFronts` in `Fronts.cpp`; the level planning,
  rest material, entries, order and reporting of `zLevelRough` are shared)
- **Patent gate:** [IP-0004](../ip/reviews/0004-engagement-controlled-roughing.md) (engagement
  control; the maintainer's decision, counsel not consulted). The geometry is offset
  pocketing and the medial-axis line of the literature below.
- **Author(s):** OxCAM contributors

## Problem

The traced clearing of DL-0018 knows the geometry exactly (part, stock, in-process stock,
every cut subtracted) but decides the *motion* locally: a tracer steps along a pass, measures
the engagement on a grid, turns away when it is over the limit, and gives up and re-enters when
stuck. Every rule added to it (wall passes, flow-on, entry tiers, thick cores) tamed one symptom
of that. In a slot the offset family it follows degenerates to arcs of the tool's own sweep and
the tracer's reactions dominated: diagonal lead-ins, loops, no two slices alike. The maintainer
asked for a method that plans from the shape instead of finding its way by trial.

## Approach

The passes are the geometry, taken exactly; the grid is used only to *measure*.

1. **Fronts.** With `remaining` the exact material of the level (stock space) and `whole` the
   allowed tool centres, a pass at stepover d is the boundary of `remaining` grown by
   r − d + 2 cells (the tool cuts d deep, two cells short: the limit has a cell of margin and
   the grid reads a straight front at the stepover a degree or two over it), clipped to where
   the tool touches material and to the area, in the cutting direction (material on the right
   when climbing). Wall passes (the area's edge, a cell and a half inside it) exist only where
   the fronts' sweep does not reach and out of the tool's reach of material thicker than a
   stepover; the body of the material is the fronts' to take.
2. **One piece at a time, the nearest to the tool.** After each piece is cut the material is
   updated (sweep subtracted, simplified) and the passes are computed again, so a slot's end is
   sliced to its end before the other, and in an open area the next front follows on where the
   last one ended.
3. **Engagement-spaced stepover.** Each piece is resampled at the step (ds) and its engagement
   profile read point by point: the same reading as the tracer's and the tests' replays (the
   half circle facing the motion, on material, the tool's own wake cleared as it goes), the
   direction from a tangent over two steps each way (from one segment of an offset polygon it
   swung by the facet angles, ±20°). Where the reading is over the limit, the stepover is cut
   back to what a straight front would need (the engaged width goes as 1 − cos θ) and the point
   moved toward the cleared side by the difference, its neighbours within a stepover moving
   with it, tapered to nothing (a stretch that rejoined the front at a kink turned the tool
   into the band there); smoothed, read again, up to four rounds; what is still over the limit
   splits the piece and waits for the front to come round to it. A front concave toward the
   tool (the first ring around a helix, the end of a slot) thus comes out at the smaller
   stepover that keeps the limit, as a smooth curve.
4. **Entries, leads, links.** A piece is entered from a clear point behind its start and to the
   cleared side, from behind first (the lead then comes in along the pass, not head on into
   the slice), nearer out only when there is no room; a start wedged against a wall is passed
   over and the piece entered at the first point along it that has an entry. The way to the
   entry is a `Link` at depth through clear ground when within four stepovers (a line that
   merely brushes the next slice is a link all the same), else a hop; the way from the entry to
   the start is a `LeadIn`.
5. **Closed areas.** A helix (or a ramp on a circle) at the widest point of the area, the
   largest inward offset that is not empty, as in MATHSM's choice of a maximal circle to spiral
   down in; where no helix of at least 0.3 r fits the area is left (no plunge, as in DL-0018);
   the plunge entry, chosen, plunges and spirals out to make room.
6. **Bookkeeping.** The material is kept at a coarser tolerance than the paths (0.02 mm) and
   simplified (Douglas-Peucker, a quarter cell) after every cut: at the path tolerance every
   sweep added hundreds of arc vertices and each iteration was slower than the last. The
   remaining report is the one of DL-0018.

## Sources

- G. Elber, E. Cohen, S. Drake, *MATHSM: medial axis transform toward high speed machining of
  pockets*, Computer-Aided Design 37(2), 2005 (preprint 2003). Successive maximally inscribed
  circles along the medial axis, connected by bi-tangent lines or arcs, for a C¹ toolpath; the
  material removed between successive circles (their crescent) as the load; a helix down one
  circle to start; one-sided traversal to avoid retractions.
- M. Held, *On the Computational Geometry of Pocket Machining*, LNCS 500, Springer 1991
  (offset pocketing, Voronoi diagrams).

## Deviations from sources

MATHSM places the tool on circles of the medial axis; it is at its best in elongated regions
and, by its authors' account, inefficient in large round ones, where unrelated circles overlap.
Front clearing keeps the offset family for the body of the material (the outside of a part, a
wide pocket) and arrives at MATHSM's picture where the region is narrow: there each front is a
crescent between successive sweeps across the slot, advanced by the engagement-spaced stepover,
with the chord link back and a lead-in, and the regions are taken one at a time. The engagement
limit is explicit here (MATHSM bounds the load only through the spacing of its circles), the
paths are fitted arcs and lines but not C¹ at the joins, and the medial axis itself is not
computed: the widest point comes from inward offsets and the slot's axis emerges from the
growing cleared region.

## Correctness and tests

- `ClearingTest` "front clearing keeps the engagement within the limit": the L-shaped boss in
  square stock, the square pocket entered by a helix, the narrow closed slot at 0.8 D (sliced
  with at most two changes of end) and the hole too small for a helix, with the replay of
  DL-0018 (which skips lead-ins as it skips plunges and ramps).
- On the maintainer's flange (1/2" bull nose, ap 19.05, ae 6.35; 1/4" rest at ae 5.08) the
  independent replay finds 0.0 mm of travel over the limit on both levels of the 1/2" rough
  (the traced clearing: 0.6–0.8 mm), 2.6–4.9° of heading change per millimetre, climb
  throughout, 114 s against the traced 110 s; the 1/4" rest op slices the three slots as the
  maintainer's reference picture does, 138 s against 157 s, with nothing reported.

## Known limitations

- The lead-in into a slice still meets the slice at an angle; MATHSM's bi-tangent joins would
  make it tangent. The arcs of consecutive slices are concentric with the cleared region, not
  the constant-radius circles of trochoidal slotting.
- A slot's two ends are sliced one after the other, but a region's pieces are chosen by
  distance, not by a walk of the medial axis; a part with several separate regions at one level
  is taken in distance order.
- The stepover reduction is tuned to the grid's reading; the first ring around a helix at
  ae = 0.5 D comes out near the limit by the independent replay (94.9° against 95.7°).
