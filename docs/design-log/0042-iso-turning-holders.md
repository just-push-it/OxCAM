# DL-0042: ISO turning holders

- **Status:** implemented
- **Module:** libs/tooling (`TurningHolderCode`, `TurningHolder.hpp`, `TurningInsert::cuttingEdgeAngle`,
  `turningShape`), libs/strategies (`noseCentrePath`, `turnFace`, `turnRough`, `turnFinish`),
  libs/document (`generateToolpath`), apps/opencam-gui (`ToolLibraryDialog`, `ToolProfileView`)
- **Patent gate:** low (citation only). Published designation standards, and insert placement by
  plain geometry.
- **Author(s):** OxCAM contributors

## Problem

A turning holder could only be made like a milling holder: a revolved outline of a nose, body and
flange. Real turning holders are stick tools and boring bars, named by an ISO designation that
fixes the insert they take and the angle they present it at. The insert's placement was one rule
for every tool (a 93–95° holder), so a 45° holder cut like a 95° one in the toolpath and the
simulation.

## Approach

**The designation is the holder.** `Holder::iso` holds it, letter by letter, in the two forms:

- Rectangular shanks (ISO 5608), for example `PCLNR 2525M 12`: holding method, insert shape, tool
  style, insert clearance, hand, height, width, length symbol, insert size.
- Boring bars (ISO 5609-1), for example `S25T-PCLNR 12`: bar material, diameter, length symbol, then
  the same letters without height and width.

`parseDesignation` reads either form, with or without spaces and dashes. Dimension f is not in the
designation (ISO 5610 gives it for shanks, ISO 6261 for bars), so the holder keeps it as a value of
its own, starting from the usual one for the size and following the size until it is changed.

A holder's `profile` stays valid and follows the code: the shank's outline, width by overall
length. So the holder list, `validate` and files work as for any holder. Imported turning holders
(non-ISO) keep their outline, and grooving, threading and parting inserts use those: ISO 5608 only
describes seats for indexable inserts of ISO 1832.

**How the holder reaches the toolpath.** The tool keeps the result as a visible value of its own,
`TurningInsert::cuttingEdgeAngle`, shown in the tool editor. `fitToHolder` sets it with the shape,
inscribed circle, relief, hand and tip orientation when an ISO holder is chosen or changed; the
editor shows these read-only then. Because it is on the tool, every caller of `turningShape`, the
toolpath keys and the post see it without knowing about holders. 0 keeps the earlier 93–95° rule,
so existing tools generate the same toolpaths. A library reports a tool whose insert is not the one
its ISO holder takes.

**Placing the insert.** In the lathe plane (u = Z, v = radius), from the nose, the leading edge
leaves at angle a₁ and the trailing edge at a₂ = a₁ − ε, with ε the insert's corner angle.

- A **side-cutting** style gives the cutting edge angle κ to a feed along −Z, so a₁ = 180° − κ.
- An **end-cutting** style gives κ to a feed along −X (facing). Mirroring in the diagonal gives
  a₂ = κ − 90°, which `sideCuttingEdgeAngle` expresses as the κ of the leading edge:
  270° − κ − ε.
- ε must leave the other edge clear of the work. Both cases come to the same condition,
  180° − κ − ε > 0, and `validate` refuses other combinations (a square insert in a 95° style).

**Strategies.** The angles now vary, so the two assumptions of DL-0020 become checks:

- **Edge ahead of the nose (κ < 90°, a₁ > 90°).** It reaches a shoulder before the nose does. The
  contour path is limited as for the trailing edge, walking from back to front:
  path(z) = max(g(z), path(z₀) − tan κ · (z − z₀)). Rough passes end where they meet that path.
  This leaves the corner the edge cannot reach, which a later tool has to cut. Facing is refused:
  a radial feed would drive the edge into the face.
- **End edge below the nose (a₂ < 0).** Only end-cutting holders can have this. Turning along Z is
  refused: the edge would cut into the turned surface.

Generation also refuses a boring bar on an outside operation (and the reverse) and a left-hand
holder, since the passes all run towards the chuck.

**Sizes.** The size symbol is a side's length in whole millimetres. `inscribedCircle` takes a side
of size + 0.5 mm, converts by the shape (sin ε for rhombi, 1/tan(π/n) for regular polygons, the
trigon's factor) and snaps to the series 3.97 ... 31.75. A round insert's size is its diameter.

**Drawing.** `turningHolderOutline` gives the shank (a rectangle, its back f behind the tip, or a
bar with its axis f below the tip) and a head: the convex hull of the insert and the end of the
shank. The head is a sketch of the real one. It is for the preview, not for collision checks. The
preview cuts the shank short with a break line so the insert stays readable.

## Sources

- ISO 5608:2012, *Turning and copying tool holders and cartridges for indexable inserts:
  Designation*: the holding method, shape, style, clearance and hand letters and their angles.
- ISO 5609-1 (boring bar designation) and ISO 6261:2011 (boring bars with a standardized f): the
  form of the code. The bar material letters and the f values for bars are as in manufacturers'
  catalogues, not read from the standards, and f stays editable for that reason.
- ISO 5610 series: f for rectangular shanks. The values were checked against catalogue data for
  common shanks (`PCLNR 2525M 12`: f = 32), not read from all parts of the series.
- ISO 1832: insert shape and size symbols.

## Correctness and tests

- **`ToolTest` "ISO turning holders":** designations in both forms read and write; catalogue sizes
  give their inscribed circles (C12, D15, T16, V16, W08, ...); styles place the insert at the
  expected angles, end cutting included; invalid combinations are refused; the outline has its
  dimension f.
- **`LibraryIoTest`:** holders round-trip in millimetres and inches, and other holders' files do not
  change.
- **`LatheTest` "an ISO holder's angles reach the toolpath":** a shaft with a shoulder. A 95° holder
  reaches the shoulder; a 45° holder roughs and finishes with no gouge and leaves its corner, and
  cannot face; an end-cutting holder faces and cannot turn along Z.
- **`SimulateTest` "turning in an ISO holder":** generation, simulation, the refusals, and the holder
  surviving a project save.

## Known limitations

- **Hand:** only right and neutral holders generate, because every pass runs towards the chuck.
- **No holder collisions:** the chuck check (DL-0021) sweeps the insert, not the holder's shank or
  head; `turningHolderOutline` is not accurate enough for it.
- **Grooving, threading and parting holders** have no designation path: they need an imported
  outline.
- **Rectangles and parallelograms** (L, A, B, K) are drawn as rhombi on the inscribed circle
  (DL-0020), as the tool does not hold their length.
