# DL-0014: Chamfering, tapping, thread milling and boring

- **Status:** implemented (phase 6)
- **Module:** libs/strategies (`chamfer`, `tap`, `threadMill`, `bore`), libs/document (hole
  selection by diameter)
- **Patent gate:** low (citation only). These are long-established machining practices:
  canned cycles, helical interpolation and edge chamfering with a pointed tool.
- **Author(s):** OxCAM contributors

## Chamfer (`chamfer2d`)

A pointed tool cuts a chamfer with its cone while its axis follows an offset of the edge at a
fixed depth.

Let θ be half the tool's point angle (the chamfer angle from vertical), r_t the radius of the
flat tip, w the horizontal chamfer width and o the tip offset (how far the tip runs below the
chamfer's lower edge). Then:

| Quantity | Value |
|---|---|
| Chamfer depth | c = w / tan θ |
| Axis distance from the edge | s = r_t + o · tan θ |
| Tip height | z_edge − c − o |

The cone then passes through the lower edge (radius s, height o above the tip) and through the
upper edge (radius s + w, height c + o). The chamfer is rejected when s + w exceeds the tool
radius.

The path itself comes from the contour kernel (DL-0008), with a stock-to-leave of s − R, so the
leads, climb or conventional direction and lead gouge checks are shared with contours. Like a
contour it follows closed loops of edges, each on the side it opens to unless flipped
(DL-0008); only edges across the tool axis can be picked (no vertical edges; `cad::edgeAcross`
samples the edge and checks it stays at one height), and each
height is chamfered at its own: the outlines are grouped by height and side, chamfered highest
first in one toolpath (a hole's top edge inside and the part's top and bottom outlines outside
in one operation; `GenerateTest` "chamfer selected edges, each height at its own").

Supported tools are chamfer mills, engravers, spot drills, centre drills and drills: any tool
with a point angle.

## Tapping (`tap`)

Rigid tapping uses the G84 cycle. The feed is pitch × spindle speed (DL-0009). The thread depth
defaults to the hole depth. Without selected faces, the operation takes holes whose diameter is
between D − 1.5 P and D − 0.5 P, which covers the usual tap drill sizes.

## Thread milling (`threadmill`)

These are internal threads.

- **Major diameter:** taken from the parameter, or from hole diameter + 1.0825 P (the ISO 68-1
  basic minor-diameter relation).
- **Helix radius:** major / 2 − r.
- **Toolpath:**
  1. Feed down the hole axis to the thread bottom.
  2. A semicircular lead-in, tangent to the helix at angle 0.
  3. Helical half-turns rising P/2 each.
  4. A semicircular lead-out back to the axis, then retract.
- **Direction:** right-hand threads run counter-clockwise from the bottom (climb milling with
  M03). Left-hand threads run clockwise.
- **Number of turns:** a tool whose flute length covers the thread length plus one pitch is
  treated as multi-form and makes one turn. Otherwise it makes ⌈length / P⌉ turns.
- **Feed:** the preset's feed rate, as set, for the tool centre. (Until 2026-10-22 it was scaled
  by helix radius / (major / 2); ADR-0004 removed every hidden feed adjustment.)

## Boring (`bore`)

- **Reamers and boring bars:** the G85 cycle at the hole diameter.
- **Other milling tools:** helical interpolation around the wall (radius hole / 2 − stock −
  r) down to the bottom, then a full circle at the bottom. This uses `PathBuilder::helix`, the
  same as pocket helix entries, at the preset's feed rate as set (ADR-0004). Holes not larger than the
  tool are skipped with a warning.

## Sources

- ISO 68-1:1998, *ISO general purpose screw threads — Basic profile — Metric screw threads*.
  Used for the basic minor-diameter relation (paraphrased; no text copied).
- P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press, 2008. Covers canned cycles,
  helical interpolation and thread milling practice.
- DL-0008 (contours and leads) and DL-0009 (drilling cycles).

## Correctness and tests

- **Strategies (`tests/unit/strategies/HoleOpsTest.cpp`):**
  - Chamfer geometry for 90° and 60° tools; errors for a chamfer that is too wide and for a tool
    without a point.
  - The chamfer path lies exactly at s from the edges, at the right depth.
  - The tap cycle's pitch, depth and feed.
  - Thread milling: helix radius, P/2 per half-turn, ⌈L/P⌉ turns, z range, and the direction
    for each hand.
  - Boring: helix radius and depth for end mills; skipping holes that are too small; the G85
    cycle for boring bars.
- **Simulation (`tests/unit/sim`):**
  - A 45° chamfer around a 40 × 20 block removes 120 · w²/2 − 4 · w³/3 (prisms minus the corner
    pyramids where they overlap), within 2 % at 0.05 mm.
  - A helical bore of a Ø20 hole with a Ø10 end mill removes the full cylinder, within 0.5 %.
- **Document (`tests/unit/document`):** tapping, thread milling (which needs a selection or a
  major diameter), boring and chamfering generate on the hole-plate fixture.

## Known limitations

- **External threads:** DL-0081, which also adds the cut direction, radial passes and the tool form.
- **Thread geometry:** tapered and multi-start threads are not supported.
- **Chamfer angle:** the chamfer angle is always the tool's half point angle. There is no
  interpolation between depths.
- **Tap drill matching:** this is a heuristic. Select the hole faces for anything unusual.
