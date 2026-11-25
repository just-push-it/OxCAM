# DL-0008: 2D contour with tangent arc leads

- **Status:** implemented (phase 3)
- **Module:** libs/strategies (`contour`)
- **Patent gate:** low (citation only); standard profiling practice
- **Author(s):** OxCAM contributors

## Problem

Profile around a shape (outside) or along the inside of it (pocket walls and islands), at one or
more depths and radial passes, with smooth entry and exit.

## Approach

### Paths

- **Path area:** the shape offset by ±(R + stock to leave + pass offset). With *compensation in
  control*, the offset leaves out R and the toolpath carries a G41/G42 hint.
- **Loops:** outside contours use the outer loops of the area; inside contours use every loop.
- **Arcs:** loops are arc-fitted (DL-0010).
- **Direction:** climb milling with M03 needs the material on the right of travel. Region
  orientation keeps the region's inside on the left, so outside loops are reversed and inside
  loops are not. In-control compensation is therefore G41 for climb and G42 for conventional.
- **Start point:** the midpoint of the longest straight segment, to avoid starting in a corner.
- **Selection and sides:** the shape is the selected horizontal faces' outlines and the closed
  loops chained from the selected edges. Each face, and each closed loop of edges, runs on the
  side it opens to (below) unless flipped in the geometry list: the outlines are grouped
  by side and each group is contoured as above, one after the other in one toolpath, each pass
  from the clearance height. A profile around a part and the walls of its pockets are one
  operation. Tests: `GenerateTest` "contour outlines each on their own side" (the block's outline
  outside and the pocket's rim inside: cuts outside the block and inside the pocket);
  `ProjectTest` round-trips the side, written only when set.
- **Automatic side:** an outline not flipped runs on the side it opens to (`automaticSide`): at the middle of its longest segment, points 0.05 mm to either side of it
  are tested against the part's solid (OCCT's solid classifier), 0.05 mm below its height and,
  when that does not decide, 0.05 mm above. The side without material is the tool's: a top
  edge's or a top face's outline opens to the air around it (outside), a hole's or pocket's rim
  into the hole (inside), a pocket floor's outline inward (its walls rise around it). Four point
  tests, a few milliseconds. Undecided (level on both sides): outside. Flipping an outline
  stores the other side on it. Older files' operation side ("side") goes onto their outlines.
  Test:
  `GenerateTest` "outlines open to the side away from the part".
- **Lead radius** is a length; while not set, half the tool diameter (the panel shows that
  value for the chosen tool). Lead moves feed at the cutting preset's lead feed (or the
  operation's own), a value of its own (ADR-0004).

### Leads

For start point s, unit tangent t, and air-side normal n (pointing away from the material):

- the lead-in quarter arc runs from c − r·t to s, around the centre c = s + r·n;
- the lead-out mirrors it, ending at c + r·t;
- optional straight lines extend both along n.

**Gouge check.** Sample the leads; every sample must lie in the allowed centre area. If not, halve
the radius up to four times. If that still fails, drop the leads and record a warning.

## Sources

- P. Smid, *CNC Programming Handbook*, Industrial Press. General reference for cutter radius
  compensation and lead-in/out practice.

## Correctness and tests

`tests/unit/strategies/StrategiesTest.cpp`:

- every move stays outside the shape grown by R − 0.01;
- cut moves stay on the offset contour;
- climb outside runs clockwise and climb inside runs counter-clockwise;
- multiple passes;
- control compensation gives the nominal path and G41;
- golden file `tests/golden/contour_rect_outside.json`.

## Known limitations

- Leads on open contours are not supported (closed loops only).
- Holding tabs: DL-0080.
- Plunges at the lead start assume the area outside the part is clear or plungeable.
