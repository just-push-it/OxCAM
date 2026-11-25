# DL-0001: Arc-aware polylines, planar sections and region nesting

- **Status:** implemented (phase 1)
- **Module:** libs/geom2d, libs/cad (`sectionAtZ`)
- **Patent gate:** low (citation only)
- **Author(s):** OxCAM contributors

## Problem

2.5D strategies need the material boundary at a given Z height as regions: an outer boundary
plus holes, with islands as separate regions. True arcs (fillets, bores) must stay exact. They
determine the final contour quality, and discretizing them breaks analytic checks (section area
within 1e-6 relative).

## Approach

**Representation.** A polyline is a list of vertices. Each vertex carries a *bulge* for the
segment that follows it:

- bulge = tan(θ/4), where θ is the signed included angle of a circular arc (positive = CCW);
- bulge = 0 is a straight line.

Arcs are split so that |θ| ≤ π, which keeps |bulge| ≤ 1.

Arc geometry from a chord of length c and bulge b:

- radius r = c(1 + b²) / (4|b|);
- the centre sits on the chord's left normal, at signed distance c(1 − b²) / (4b) from the
  chord midpoint.

**Area.** Shoelace sum over the chords, plus each arc's circular-segment area
r²/2 · (θ − sin θ), signed by θ. This is exact.

**Containment.** Winding-number test on the discretized boundary. Discretization picks the
largest chord angle φ whose sagitta r(1 − cos φ/2) stays within the tolerance.

**Section (cad).**

1. Intersect the shape with the plane Z = z using a B-rep boolean section.
2. Connect the resulting edges into wires.
3. Walk each wire in order, with edges converted as follows:
   - lines map to straight segments;
   - circles whose axis is parallel to Z map to bulge arcs, with the sweep sign taken from the
     circle-axis direction and the edge orientation;
   - all other curves are sampled with quasi-uniform deflection.

**Nesting.**

1. Sort closed loops by area, largest first.
2. For each loop, take a sample point (the midpoint of its first segment). The loop's parent is
   the smallest earlier loop that contains that point.
3. Even depth makes a loop an outer boundary (oriented CCW); odd depth makes it a hole of its
   parent (oriented CW).

## Sources

- Circular segment area and chord/sagitta relations: standard plane geometry (any analytic
  geometry text).
- K. Hormann, A. Agathos, "The point in polygon problem for arbitrary polygons",
  *Computational Geometry* 20(3), 2001, pp. 131–144.
- The bulge convention is a long-standing, openly documented way to encode arcs in polylines.
  We use only the mathematical definition; no format code was consulted.

## Deviations from sources

None. Nesting assumes the loops of a valid planar section do not touch or cross.

## Correctness and tests

`tests/unit/geom2d/PolylineTest.cpp`:

- circle and rounded-rectangle area, length and bounds are exact to 1e-12;
- reversal preserves geometry;
- the discretization sagitta stays within tolerance.

`tests/unit/geom2d/RegionClipTest.cpp`:

- nesting of outers, holes and islands.

`tests/unit/cad/SectionFeatureTest.cpp`:

- section areas of the generated fixtures match analytic values within 1e-6 relative;
- fillet arcs survive as bulges.

## Known limitations

- Touching loops (for example, two pockets sharing a vertex at the section height) are not
  nested reliably.
- Sectioning exactly at the height of a horizontal face is ill-defined; callers offset z
  slightly.
