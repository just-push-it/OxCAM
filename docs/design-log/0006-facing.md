# DL-0006: Facing

- **Status:** implemented (phase 3)
- **Module:** libs/strategies (`facing`)
- **Patent gate:** low (citation only); direction-parallel rastering
- **Author(s):** OxCAM contributors

## Problem

Remove material from the top of the stock down to a facing height, covering the whole stock
outline at every depth.

## Approach

1. **Tool-centre area.** The centre travels over the stock outline grown by R + overrun. The tool
   touches material only while its centre is inside the outline grown by R.
2. **Raster lines.** Work in a frame rotated by the pass angle.
   - The first and last lines put the tool edge a small margin (min(0.1·D, 1 mm)) beyond the
     outline edges.
   - Intermediate lines are evenly spaced, at most stepover × D apart.
   - Each line is clipped against the centre area by scanline intersection.
3. **Depths.** Levels are evenly spaced with at most the step-down between them.
4. **Linking.**
   - *Zig-zag:* passes alternate direction. Consecutive passes are linked at depth only when
     samples of the link segment show the tool cannot touch material (the link runs in the air
     beyond the outline). Otherwise the tool retracts.
   - *One way:* every pass starts on the same side.

   Plunges always happen outside the stock.

## Sources

- M. Held, *On the Computational Geometry of Pocket Machining*, Lecture Notes in Computer
  Science 500, Springer, 1991. Covers direction-parallel (zig-zag) machining.

## Correctness and tests

`tests/unit/strategies/StrategiesTest.cpp`:

- the swept area covers the stock at every level;
- links and plunges never touch the stock;
- angled and one-way variants are covered;
- golden file `tests/golden/facing_rect_zigzag.json`.

## Known limitations

- There is no climb/conventional choice per pass (zig-zag alternates by nature).
- Fixtures and clamps are not considered.
