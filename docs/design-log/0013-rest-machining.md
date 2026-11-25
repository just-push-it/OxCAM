# DL-0013: Rest machining from the simulated in-process stock

- **Status:** implemented (phase 6). Off by default.
- **Module:** libs/sim (`materialRegions`), libs/strategies (`PocketParams::restMaterial`),
  libs/document (`stockBeforeOperation`, `pocket2d` parameter `rest_machining`)
- **Patent gate:** [IP-0002](../ip/reviews/0002-rest-machining-from-in-process-stock.md) is
  **pending**: medium risk, and the prior-art search is not yet done. Must be cleared before any
  public release.
- **Author(s):** OxCAM contributors

## Problem

After a large tool clears a pocket, material is left where it could not reach: corners tighter
than its radius, and narrow necks. A pocket operation marked "rest machining" should cut only
that material with a smaller tool. It must not re-cut the whole pocket, and it must not gouge.

## Approach

1. **In-process stock.** `stockBeforeOperation` builds the setup's initial stock on a Z-only
   dexel grid (DL-0012). It then generates and simulates every earlier enabled operation of the
   setup. The ray spacing is the `rest_resolution` parameter, or automatic.
2. **Remaining material per level.** For pocket level z, with the level above at z_above
   (+∞ for the first level), `materialRegions` marks the rays holding more than
   `rest_min_thickness` of material in (z + t, z_above]. It then traces the boundary of those
   cells:
   - Each edge between a filled and an empty cell is directed with the filled cell on its left.
   - Edges are chained into loops. At a vertex shared by two diagonal cells the trace turns left,
     so diagonal cells stay separate.
   - Counter-clockwise loops are outer boundaries and clockwise loops are holes. A Clipper union
     normalizes the result.
3. **Restricting the pocket.** The regions are intersected with the area a round tool can reach
   in the pocket, R = (allowed centres) ⊕ r.
   - An opening of 0.6 × the ray spacing removes the half-cell slivers that the staircase
     outline produces along walls.
   - The tool-centre area is A = allowed ∩ (remaining ⊕ (r + rim)), with rim = 0.2 r. A is
     cleared with the contour-parallel loop tree of DL-0007.
   - The residual clean-up targets the remaining material rather than the whole reachable area.
4. **Coverage.** Every reachable remaining point x has a centre c in `allowed` with |x − c| ≤ r.
   That centre lies in allowed ∩ (remaining ⊕ r) ⊆ A. The loops of A plus the residual clean-up
   therefore sweep every reachable remaining point, by the same argument as DL-0007.
5. **Entry in air.**
   - Rest regions are cut from the outside in.
   - Each region is entered by a plunge at a loop point whose tool disk stays clear of all
     material above the level: outside (material above z) ⊕ (r + rim/2). Because A reaches rim
     beyond the material, such points exist on the outer loop of every region that opens onto
     cleared space.
   - Regions enclosed by material fall back to the ramp and helix entries of DL-0007.
6. **Invalidation.** The toolpath input key of a rest operation includes every earlier
   operation of the setup, plus their tools and holders. Changing an earlier operation
   regenerates the rest pass.

## Sources

- B. K. Choi, R. B. Jerard, *Sculptured Surface Machining: Theory and Applications*, Kluwer,
  1998. Covers Z-map (dexel) workpiece models and the detection of uncut regions.
- M. Held, *On the Computational Geometry of Pocket Machining*, LNCS 500, 1991. Covers
  contour-parallel pocketing and uncut regions.
- DL-0007 (pocketing) and DL-0012 (stock simulation).

## Deviations from sources

- **2.5D only:** the remaining material is evaluated per pocket level, as XY regions, rather than
  as a 3D surface. The 3D rest machining of phase 7 will compare height maps instead.

## Correctness and tests

- **sim `remaining material regions follow the stock`:** the whole block gives one region; a
  slot splits it in two; a plunge gives a region with one hole; areas match within cell
  quantization; an empty slab gives nothing.
- **document `rest pocketing clears what a larger tool left`:** on the block fixture, the setup
  is face, Ø16 rough pocket (R5 corners), Ø6 rest pocket, profile.
  - The rest pass is shorter than 40 % of a full Ø6 pocket.
  - Its key changes when the roughing changes.
  - With the rest pass, the simulated part has no gouges and less than 0.05 mm of stock left.
  - Without it, the corners stay (more than 10 mm³, thicker than 1 mm).

## Known limitations

- **Cost:** each rest operation re-simulates every earlier operation when its inputs change.
  Checkpoints are not shared with the simulation dock yet.
- **Resolution:** remaining material thinner than about one ray spacing horizontally is ignored.
  Choose `rest_resolution` below the smallest step worth cutting.
- **Scope:** only `pocket2d` has a rest mode. Contours and facing do not.
- **Between setups:** a setup whose stock comes from an earlier setup starts its rest stock as
  that setup left it (DL-0012), so a rest operation sees what every earlier setup in the chain
  cut as well as the earlier operations of its own setup. Its toolpath key includes those setups'
  operations, tools and holders.
