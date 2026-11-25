# DL-0016: Drop-cutter on triangle meshes and the cutter-location surface

- **Status:** implemented (phase 7)
- **Module:** libs/strategies (`Cutter`, `DropCutter`, `computeHeightField`,
  `forbiddenRegions`)
- **Patent gate:** low (citation only). The IP policy lists drop-cutter and waterline as low
  risk.
- **Author(s):** OxCAM contributors

## Problem

Given a part as a triangle mesh and a rotating tool with its axis along +Z, find the lowest tip
height at an XY position where the tool does not penetrate the part. This is the basic 3-axis
positioning query. Every 3D strategy is built on it.

## Approach

### The cutter's lower envelope

The tool is the solid {z ≥ h + lo(d)} within radius R, where h is the tip height and d is the
distance from the axis. lo(d) is the height of the tool's lower surface above the tip at radial
distance d. Two shape families are supported, both with lo convex and non-decreasing:

| Family | Tools | lo(d) |
|---|---|---|
| Torus | flat (r_c = 0), bull-nose and ball (r_c = R) end mills | 0 for d ≤ R − r_c, otherwise r_c − √(r_c² − (d − R + r_c)²) |
| Cone with a flat tip | chamfer mills, engravers, spot drills, drills | 0 for d ≤ r_t, otherwise (d − r_t) / tan θ |

Other tools (slot cutters, lollipops, form tools) are rejected for 3D work.

The envelope is analytic, not tabulated. A table interpolated linearly would overestimate a
convex lo between samples and so gouge, worst near the vertical tangent of a ball.

### Drop-cutter

The answer is

  h* = max over points p of the mesh within radius R of ( p.z − lo(|p_xy − c|) ).

Per triangle, the maximum lies in one of three places:

1. **A vertex:** evaluate it directly.
2. **The facet interior:** on a plane of slope s (the magnitude of the gradient), the function is
   z(c) + s·r − lo(r) in the uphill direction. The best r is closed-form: r = R for flat tools;
   r = R − r_c + r_c·s/√(1+s²) for torus tools; r_t or R for cones. The contact counts only if the
   contact point r·ĝ from the axis falls inside the triangle.
3. **An edge:** f(t) = z(t) − lo(d(t)) is concave on the part of the edge within radius R.
   d(t) is convex, and a convex non-decreasing function of a convex function is convex. A
   golden-section search (48 iterations) plus the two end values finds the maximum.

Pruning keeps it fast:

- Triangles are bucketed in an XY grid (cell ≥ R). A query visits the cells within R of the
  axis.
- Candidates are sorted by their highest vertex. The scan stops once that is below the best
  height so far, because lo ≥ 0.
- Edges whose upper bound max(z) − lo(d_min) cannot beat the current best are skipped.

Queries are thread-safe. Height fields are computed in parallel with oneTBB.

### Stock to leave

The tool is offset by the allowance a: radius R + a and corner radius r_c + a, so a flat tool
becomes a bull nose with corner radius a. The offset tool's tip is a below the real tip. Only
torus tools can be offset.

### Forbidden regions

The waterline and roughing strategies need, at a height z, the tool-centre positions where the
tip at z would cut the part: F(z) = {c : h*(c) > z}.

1. h* is sampled on a regular grid (the cutter-location height field).
2. F(z) is traced by marching squares. A saddle is resolved by an exact query at the cell
   centre.
3. Every crossing on a grid edge is refined by bisection on the exact drop-cutter, to the
   tolerance. A cliff in the height field (a vertical wall) is therefore located exactly,
   instead of interpolated across a whole cell.
4. Where the midpoint of a chord between two boundary points lies inside F, a boundary point is
   found across it (by bisection along the normal) and inserted, to a depth of 4. This bounds how
   far chords can cut into F in convex places.
5. Loops are nested into regions: outer boundaries counter-clockwise, holes clockwise.

## Sources

- B. K. Choi, R. B. Jerard, *Sculptured Surface Machining: Theory and Applications*, Kluwer,
  1998. Covers cutter-location surfaces, inverse offsets and the vertex/facet/edge treatment of
  polyhedral models.
- H.-T. Yau, C.-M. Chuang, Y.-S. Lee, *Numerical control machining of triangulated sculptured
  surfaces in a stereo lithography format with a generalized cutter*, Int. J. Production
  Research 42(13), 2004. Covers drop-cutter on STL meshes for APT-style cutters.
- W. E. Lorensen, H. E. Cline, *Marching cubes: a high resolution 3D surface construction
  algorithm*, SIGGRAPH '87, 1987. Marching squares is its 2D form.

## Deviations from sources

- **Edge contact:** instead of per-cutter closed forms for edge contact (sphere–line,
  torus–line), one numeric search serves every convex envelope. This is exact to about 1e-9 of
  the edge length, and adding a tool family needs only lo(d) and its facet contact.

## Correctness and tests

All in `tests/unit/strategies/SurfaceTest.cpp`:

- **Envelopes:** facet contacts equal a brute-force maximization.
- **Analytic drops:** ball, flat and bull-nose tools on a 30° plane give the closed forms to
  1e-9. A ball on a finely tessellated dome is within 0.03 of the offset sphere.
- **Oracle:** at 160 random positions (4 tools, a coarse dome mesh), no densely sampled mesh
  point lies inside the tool. The tool touches to within the sampling density.
- **Forbidden regions:** the waterline around a cylindrical boss runs at boss radius + tool
  radius, and climb milling gives clockwise loops.

## Known limitations

- **Tool axis:** +Z only (3-axis and 3+2). Simultaneous 5-axis is phase 9.
- **Accuracy:** bounded by the part's tessellation (0.005 mm chordal for 3D operations).
- **Memory:** height fields are capped at 25 million points.
