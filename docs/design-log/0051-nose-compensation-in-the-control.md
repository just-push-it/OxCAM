# DL-0051: Nose-radius compensation in the control

- **Status:** implemented
- **Module:** libs/strategies (`turnFinish`), libs/document (`latheCutPath`, `generateToolpath`,
  `simulateSetup`), libs/post (`FanucLathe`)
- **Patent gate:** low (citation only). G41/G42 are the controls' own; syntax from controller
  documentation.
- **Author(s):** OxCAM contributors

## Problem

Nose-radius compensation happened only in the computer: the program held the offset path, so an
operator could not correct a worn insert or a different nose radius with a wear offset the usual way.
The tool's tip orientation reached the post and went nowhere.

## Approach

**Nose compensation** on `turn_finish`: In computer (as it was), or In control (G41/G42).

**The program.** In control, the passes lie on the profile itself (plus the stock to leave), at the
programmed sharp tip, as the control expects. Coming on takes a move across the radius, so the lead-in
gets one. Rapids turn it off (`Lower`, as in milling).

**Which side.** Passing along the chuck-ward direction, outside is G42 and inside G41. Mirrored for back
turning (DL-0050) they swap, since the material then lies on the other side of the direction of travel.

**ADR-0004.** No feed changes: the program's feeds are the preset's. The control's offset changes where
the tool runs, which is what the user set on the machine.

**Tip orientation.** The control offsets by the tool's tip orientation, so it matters now: a tool whose
orientation does not suit the cut (3 outside, 2 inside, 4 and 1 mirrored) is refused, naming both
values. The generic lathe post writes what the offset page must hold as a comment before coming on.
There is no G-code word for it on a Fanuc-style control: it sits on the geometry page.

**Simulation (`latheCutPath`).** What the machine does with a compensated program is the path computed
in the computer, so that is what the simulation cuts: the operation regenerated with compensation in
the computer. The program posts the profile, the simulation shows the nose's true path, and earlier
stock for a later operation uses the same (`latheStockBefore`). The checks at generation run against
the programmed path, within a nose radius of the real one.

Rough and profile passes stay in the computer: nothing is gained from wear offsets on them.

## Correctness and tests

- **`LatheTest`:** in the control the taper's program lies on the taper; G41/G42 by side and direction.
- **`LatheStockTest` "nose compensation in the control":** the program lies on the part's radius; the
  lead-in crosses the radius; it posts G42, G40 and the offset page's comment, and verifies against its
  back-plot; the simulation has no gouge; a wrong tip orientation is refused.

## Known limitations

- **Program zero of the wear offset:** the program assumes a wear offset of zero, as any program does.
- **Arcs:** the path stays straight lines, which controls compensate the same way.
