# DL-0007: Contour-parallel pocketing with residual clean-up

- **Status:** implemented (phase 3)
- **Module:** libs/strategies (`pocket`)
- **Patent gate:** offset pocketing low (citation only). The residual clean-up is recorded in
  [IP-0001](../ip/reviews/0001-pocket-residual-cleanup.md) (pending).
- **Author(s):** OxCAM contributors

## Problem

Clear a pocket region, with islands, from a top height to a floor, without gouging walls or
islands, leaving no reachable material.

## Approach

### Loops

- **Allowed centre area:** A = pocket ⊖ disk(R + stock to leave), computed with Clipper2 round
  offsets.
- **Loop levels:** level k is A ⊖ disk(k·w) for stepover w, computed from A directly so errors do
  not accumulate.
- **Containment tree:** each level-k region's parent is the level k−1 region that contains its
  interior point.
- **Order:** post-order, so a node's inside (its children) is cut before its own loops. Each loop
  then cuts at most one stepover of fresh material, and the last loop finishes the walls.
- **Arcs:** loops are arc-fitted (DL-0010) so round joins become G2/G3.
- **Direction:** region orientation (outer CCW, holes CW) keeps the material on the right, which
  is climb milling with M03. Conventional milling reverses every loop.

### Linking and entry

- **Linking.** A move from a child loop to its parent's loop runs at depth to the nearest point of
  the next loop. The segment from a point inside a region to its nearest boundary point lies
  inside the region, so it cannot gouge.
- **Fresh material, innermost loops:**
  1. Circular helix, if a circle of the helix radius fits: the centre is a point of
     leafRegion ⊖ disk(r_h). The innermost region is often thinner than that (it is the last
     offset, less than a stepover wide); then the centre is the point of
     rootRegion ⊖ disk(r_h + 2·tol) nearest to the leaf, where rootRegion is the connected
     tool-centre area the leaf belongs to, provided the straight move from the helix to the
     leaf's loop stays inside rootRegion. Both keep the helix inside the safe centre area. The
     pitch per turn is 2π·r_h·tan(ramp angle).
  2. Otherwise, ramp along the loop (also the "Ramp along path" entry).
  3. Plunge as a last resort, with a warning.
- **Other entries** ramp along the loop, which always stays on the path.

### Residual clean-up

Let reachable material M = A ⊕ disk(R), the area a round tool can clear. Compute the residual as
M minus the swept area of this level's moves, discarding slivers below the discretization noise.

**Coverage argument.** Let S = A ∩ (residual ⊕ disk(R)).

- For any residual point q, a reaching centre c with |c − q| ≤ R lies in S.
- For any region S, the offset loops of S at spacing w ≤ R sweep every point of S ⊕ disk(R).
  Proof: walking from such a point towards its nearest boundary point of S, the distance to the
  boundary falls continuously, so the walk crosses a loop level within distance R.
- Therefore a mini-pocket of S with stepover min(w, 0.9 R) removes the residual.

This runs at most twice per level. Large stepovers (w > R) leave residual near corners and along
the medial axis, and the clean-up removes it.

## Sources

- M. Held, *On the Computational Geometry of Pocket Machining*, Lecture Notes in Computer
  Science 500, Springer, 1991. Covers contour-parallel offsetting and the problem of uncut
  regions.
- Polygon offsets and booleans: the Clipper2 library (BSL-1.0).

## Correctness and tests

`tests/unit/strategies/PocketPropertyTest.cpp` checks random star-shaped regions with optional
islands, random tool sizes, stepovers from 0.2 to 0.9 × D, random stock to leave and all three
entry types:

- **No gouge:** every feed point at depth lies in A (within 0.01 mm).
- **Full coverage:** the uncut reachable area, after removing slivers below 0.02 mm, is under
  0.01 mm².
- **Ramps:** never steeper than the ramp angle.

The test runs 300 regions per CI run; 10,000 regions were run for phase 3 acceptance. Also:

- `StrategiesTest.cpp`: islands, entries, direction, cancellation;
- golden file `tests/golden/pocket_rect_island.json`.

## Known limitations

- Pockets open to one side (steps) are not handled.
- There is no engagement control; that area is high-risk under the IP policy and deliberately
  out of scope.
- Clean-up entries in nearly cleared areas may plunge on very short loops.
