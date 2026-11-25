# DL-0032: Arc smoothing of toolpaths (arcs in the XY, ZX and YZ planes)

- **Status:** implemented
- **Module:** libs/toolpath (`ArcPlane`, `fitArcs` and `linearizeArcs` in `Smoothing.hpp`),
  libs/post (G17/G18/G19, I/J/K per plane, back-plot), libs/sim, libs/machine; applied to every
  operation with the "Smoothing (arcs)" parameter by libs/document (`generateToolpath`)
- **Patent gate:** low (citation only). Arc fitting of polylines and plane selection for circular
  interpolation are textbook practice and ISO 6983 controller features.
- **Author(s):** OxCAM contributors

## Problem

The finishing strategies that sample the part (parallel finishing drops the cutter along a
line; waterline traces a contour on a height grid) produced long runs of short straight moves:
21,000 moves for the dome fixture's waterline, a few tenths of a millimetre each. The points are
within the chord tolerance of the surface, but the machine sees a stream of tiny G1 moves; on
curved areas that leaves a faceted look (and on many controls slows the feed). The maintainer
asked for a smoothing setting that outputs arcs, and for a general mechanism other toolpaths can
use later.

## Approach

1. **Arc planes in the toolpath IR.** A move's arc has a plane: XY (G17, the default and the
   only plane before), ZX (G18) or YZ (G19). Its centre and end share the start's coordinate
   along the plane's normal (XY arcs alone may climb: helices). The plane's axes (u, v, normal)
   are right-handed: XY (x, y, z), ZX (z, x, y), YZ (y, z, x), and clockwise is seen from the
   normal's positive side, as G-code defines it (so in G18, seen from +Y, Z points right and X
   up: a counter-clockwise arc from x = -10 to x = 10 goes over the top). Every consumer reads
   arcs through plane coordinates: length, sampling, partial moves (playback), bounds, JSON
   (`"pl": "ZX" | "YZ"`, absent for XY), comparison.
2. **Consumers.** The simulator sweeps arcs out of the XY plane as short lines (its exact arc
   sweep is an XY one). The post's NC instruction carries the plane; the formatter keeps the
   plane modal, emits G18/G19 before an arc out of the current plane and G17 before the first
   motion, cycle or cutter compensation after such arcs (everything else assumes XY), and writes
   the centre offsets of the plane's two axes (I/J, I/K, J/K) or R. The back-plotter parses
   G17/G18/G19 and the plane's offset words (it used to warn that only G17 was supported) and
   compares arc centres in the plane. A 3+2 indexed setup turns the program about Z, which
   would take a vertical-plane arc out of its axis plane: those arcs become short lines there.
   Multi-axis wrapping and 5-axis solving already sample arcs into lines.
3. **The general pass** (`toolpath::fitArcs(path, tolerance)`). Runs of consecutive straight
   feed moves with the same feed class and rate (3-axis, not threading) are split into pieces
   lying in one axis plane (constant z, y or x; the longest from each point), and each piece is
   fitted with arcs (at most 180°) and lines whose points stay within the tolerance of the
   original points (geom2d::fitArcs, DL-0010). Rapids, plunges at another rate, cycles and runs
   in no axis plane (a raster at an oblique angle) are kept as they are.
4. **Applied centrally.** Strategies do not smooth. Every strategy that does not arc by itself
   has the "Smoothing (arcs)" parameter (on by default; the GUI's operation defaults set it for
   new operations of each type) and "Smoothing tolerance" (0.005 mm, shown only while smoothing is on):
   2D pocket, 2D contour, chamfer, 3D roughing, flat area, parallel, waterline and contour
   finishing. After the strategy runs, the document layer fits arcs to the toolpath with the
   tolerance when it is on, and when it is off replaces every arc, the strategy's own included
   (pocket corners, leads), by lines within the chord tolerance (`linearizeArcs`), for controls
   or users that want lines only. Boring and thread milling interpolate circles by nature and
   have no setting; drilling and tapping are cycles; turning and multi-axis keep their own
   output.
5. **What becomes arcs.** Waterline and contour loops (constant z) become XY arcs. Parallel
   finishing lines at 0° lie in ZX planes and at 90° in YZ planes, and become G18/G19 arcs;
   other raster angles stay lines (an oblique vertical plane has no G-code plane). 2D loops
   already are arcs and lines; long runs of short lines between them merge.

## Sources

- ISO 6983-1:2009, *Numerical control of machines: program format and definitions of address
  words* (plane selection G17/G18/G19, circular interpolation G2/G3 with I/J/K or R).
- DL-0010 (arc fitting of polylines: runs of points on a line or a circular arc within a
  tolerance).

## Correctness and tests

- `ToolpathTest` "arcs in the ZX and YZ planes": the sweep and sampled points of a ZX half
  circle (in its plane, on its circle, over the top), the JSON round trip with the plane, the
  length.
- `ToolpathTest` "smoothing fits arcs to runs of short lines in the XY, ZX and YZ planes": a
  quarter circle sampled every half degree in each plane becomes one arc (its centre within
  1 µm) and the line after it one line; a run in no axis plane is left alone.
- `PostTest` "arcs in the ZX and YZ planes post as G18 and G19 and back-plot": a ZX arc, a YZ arc
  and an XY arc post with G18, G19 and G17 again, and the back-plot matches the toolpath within
  0.1 µm; "back-plotter handles common G-code forms" parses a G18 arc's centre from I and K.
- `SurfaceTest` "waterline finishing outputs arcs when smoothing": a round boss's waterline has
  no arcs without smoothing, arcs with it, and under a quarter of the moves.
- `ToolpathTest` "arcs become short lines when smoothing is off": a half circle becomes lines
  whose ends lie on it, on the side of its direction, ending at its end.
- `GenerateTest` "smoothing is applied to every operation that has it": a pocket has arcs with
  smoothing and none without it; parallel finishing on the dome has vertical-plane arcs with it
  and none without it.
- `StrategiesTest` "parameters show only where they do something": smoothing on the strategies
  listed above and not on boring, thread milling, drilling or tapping.

## Known limitations

- Parallel finishing at an angle other than 0° or 90° stays lines.
- An arc may stand up to the tolerance from the points it replaces, on either side: the
  default 0.005 mm is within a finishing allowance, and the setting is per operation.
- Simulation of vertical-plane arcs is by short lines (as accurate, a little slower).
