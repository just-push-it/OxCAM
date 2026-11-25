# DL-0017: Parallel and waterline finishing

- **Status:** implemented (phase 7). Both have a "Smoothing (arcs)" option (on by default):
  waterline loops become XY arcs, parallel lines at 0° or 90° arcs in the ZX or YZ plane
  ([DL-0032](0032-arc-smoothing.md)). Contour finishing with the step from a cusp height is
  DL-0033.
- **Module:** libs/strategies (`parallelFinish`, `waterline`), libs/document (`parallel3d`,
  `waterline3d`)
- **Patent gate:** low (citation only). These are standard 3-axis finishing patterns.
- **Author(s):** OxCAM contributors

## Parallel finishing (`parallel3d`)

Raster lines at a chosen angle cover a tool-centre boundary. The default boundary is the part's
XY bounds. For 3D rest finishing, it is the remaining-stock regions grown by the tool radius
(DL-0018).

1. **Clipping:** each line is clipped to the boundary.
2. **Sampling:** each piece is sampled every `sample_step` (default R/8, between 0.02 and 0.5
   mm), and every sample is dropped onto the part (DL-0016).
3. **Simplification:** the 3D polyline is simplified by Douglas–Peucker to the chordal
   tolerance.
4. **Linking:**
   - Zig-zag lines are linked along the surface: the connecting segment is itself dropped, so the
     link cannot gouge.
   - Other links retract, and every entry is a feed plunge from the retract height (roughing may
     have left steps above the surface).
5. **Lines are computed in parallel.**

**Stepover from the scallop height.** On a flat area, a tool with corner radius r_c and flat
radius a = R − r_c leaves cusps of height h at a stepover of 2a + 2√(2 r_c h − h²). This is
capped at 1.8 R. On slopes the cusps grow; that is what waterline finishing is for.

## Waterline finishing (`waterline3d`)

At each Z level from just below the part top down to its bottom, the boundaries of the forbidden
region F(z) (DL-0016) are the tool-centre contours where the tool touches the part.

- **Direction:** loops are reversed for climb milling, so the part is on the right with M03.
- **Entry:** each loop is entered by a feed plunge from the retract height at the loop point
  nearest the tool. On the boundary of F the drop height equals z, so the plunge cannot gouge.
- **Noise:** loops shorter than four grid cells are dropped.

## Sources

- Choi & Jerard (1998), see DL-0016. Covers raster and contour-parallel finishing, and scallop
  height.
- C.-M. Chuang, H.-T. Yau, *A new approach to z-level contour machining of triangulated surface
  models using fillet endmills*, Computer-Aided Design 37(10), 2005. Covers z-level (waterline)
  machining of meshes.
- D. H. Douglas, T. K. Peucker, *Algorithms for the reduction of the number of points required
  to represent a digitized line or its caricature*, Cartographica 10(2), 1973.

## Correctness and tests

- **Strategy tests:**
  - Every sampled feed point lies on the drop-cutter surface.
  - The line spacing equals the scallop formula.
  - Waterline loops around a boss lie at the offset radius.
- **Simulation (`tests/unit/sim`):** a ball finish of a flat plate at a scallop height of
  0.05 mm leaves cusps between 0.03 and 0.06 mm (dexel resolution 0.02 mm), with no gouge.
- **Document (dome fixture):** parallel finishing reaches the dome top; waterline and roughing
  generate.
- **Rough → rest rough → parallel finish:** simulated on the dome with no gouge (see DL-0018).

## Known limitations

- **No slope split:** parallel finishing has no steep/shallow split. Steep walls get wide spacing
  and need a waterline pass, or steep and shallow finishing (DL-0082), which splits by slope and
  uses both kernels.
- **Waterline:** no flat-area detection and no adaptive levels; levels are evenly spaced.
- **Pencil and corner clean-up:** pencil finishing, DL-0082.
