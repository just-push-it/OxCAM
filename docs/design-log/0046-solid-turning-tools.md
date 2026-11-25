# DL-0046: Solid turning tools

- **Status:** implemented
- **Module:** libs/tooling (`SolidBar`, `makeSolidBoringBar`, `fitSolidBar`, `turningBody`),
  libs/document (`generateToolpath`), apps/opencam-gui (`ToolLibraryDialog`, `ToolProfileView`)
- **Patent gate:** low (citation only). Catalogue dimensions, and plain geometry.
- **Author(s):** OxCAM contributors

## Problem

The maintainer's example is a solid carbide micro boring bar with no separate insert or holder. Every
turning tool here was an insert, so such a tool could only be described as an insert with no holder,
which says nothing about what fits in a 6 mm bore or how deep it reaches.

## Approach

**What it is sold by.** From the [catalogue page of the maintainer's example](https://www.haastooling.com/p/03-3010)
(03-3010, HMBCB-6R6.2-25-20), a tool's data are these:

| Field | Catalogue | Example |
|---|---|---|
| `minBore` | Min. machining diameter, ØD | 6.2 mm |
| `shankDiameter` | Shank diameter, ØDMM | 6.0 mm |
| `headLength` | Head length, LH | 25.0 mm |
| `overallLength` | Overall length, OAL | 53.0 mm |
| `maxOverhang` | Maximum overhang, OHX | 30.5 mm |
| `centreToTip` | Center to insert dimension, S | 2.95 mm |
| `maxDepth` | Maximum depth, T-MAX | 0.5 mm |
| `insert.noseRadius` | Corner radius, RE | 0.2 mm |

**In the tool model.** `Tool::solid`, on a turning, grooving or threading tool, not a new tool type.
Micro bars are also sold for grooving and threading, and every turning operation works on the tool's
tip as before. A solid tool has no holder (`validate`).

**The tip.** The page does not give the tip's shape or cutting edge angle. `fitSolidBar` takes a 55°
tip of the centre-to-tip's inscribed circle: its size only matters for how far behind the programmed
point the simulation removes material. The nose radius, hand and cutting edge angle stay the tool's
values.

**The body (`turningBody`, DL-0045).** The head stands from just under the tip across `minBore`; the
shank is `centreToTip` below the tip. So a bore smaller than the tool's minimum is refused by the
same check as a boring bar, across the axis.

**Its limits are refused, not worked around:**

- Outside operations (it cuts inside).
- A rough deeper than T-MAX. The depth of cut is the preset's, and changing it behind the user's back
  is what ADR-0004 rules out for speeds and feeds: the message names both values.
- A toolpath reaching further from the stock's front than OHX.

**Defaults.** A new solid bar starts with the example's dimensions and a light preset, all shown and
editable, its depth of cut at T-MAX.

## Correctness and tests

`LatheHoleTest` "a solid boring bar": it bores a through bore to size with no holder and no collision;
each refusal; a bore below the minimum; the JSON round trip. `gui.solid_boring_bar` shows it in the
tool editor.

## Known limitations

- **Solid outside tools:** only bars (inside) are modelled.
- **Sleeve:** what holds the bar is not modelled, so it is not checked against the part: OHX stands in
  for it.
- **Tip geometry:** a chosen shape (above), for lack of catalogue data, so the simulation near the tip
  is approximate.
