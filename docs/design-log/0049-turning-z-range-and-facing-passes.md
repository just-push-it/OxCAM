# DL-0049: Turning a Z range, and roughing across Z

- **Status:** implemented
- **Module:** libs/strategies (`turnRough`, `turnFinish`, `turnHasZRange`), libs/document
  (`generateToolpath`, `toolpathInputKey`)
- **Patent gate:** low (citation only). Plain geometry in the lathe plane.
- **Author(s):** OxCAM contributors

## Problem

A rough or finish operation always took on the whole profile, running on past a flange at its
trailing angle (DL-0044). Roughing only ran along Z, which is the wrong way round for a part wider
than it is long, or a deep shoulder.

## Approach

**Z range** on `turn_rough` and `turn_finish`: **Whole profile**, or **Between two Z**, in the setup's
WCS. Both heights are visible parameters.

- Material outside the range is left alone, and the moves stay in it: rough passes start a clearance
  in front of it, finish passes come down on to the profile from a clearance off it.
- A ranged operation works on what the earlier operations left (`latheStockBefore`), not on the setup's
  stock, and its toolpath key takes them in. Otherwise a range behind an already-roughed section
  would see stock in its way that is long gone.
- Stock that really does stand just in front of the range is refused. A tool coming along Z would
  come through it: it has to be machined first, or the range started further forward.

**Across Z** on `turn_rough`: **Passes**, **Along Z** or **Across Z**. Radial passes from the front
back, stepping by the depth of cut, each running from just off the material in its band to the
allowance. A band behind a part standing higher further forward stops where the one in front of it
stopped: the tool cannot get down there. The profile pass is the same in both. As with facing, it
needs a cutting edge angle of at least 90°, and is refused below it.

## Correctness and tests

`LatheStockTest` "roughing across Z and turning a Z range", on `ranges_project.ocam`: radial passes
step back within the depth of cut, to the part; moves stay in the range, and what is behind it stays
at the bar's radius; no gouge; stock in front is refused, as is a range the wrong way round; an earlier
change reaches a ranged operation. It posts and verifies. The toolpaths were plotted per operation.

## Known limitations

- **Undercuts** (a surface rising towards the chuck) are still not roughed: grooving reaches recesses,
  and back turning (DL-0050) takes them from behind.
- **Inside across Z:** works as outside, but has not been tried on a real bore's proportions.
