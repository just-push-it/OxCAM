# DL-0027: Plunge roughing

- **Status:** implemented
- **Module:** libs/strategies (`plungeRough`, `src/Plunge.cpp`), libs/document (`plunge_rough`)
- **Patent gate:** low (citation only). Plunge milling (roughing by axial plunges at overlapping
  positions) is a long-established machining practice described in tooling handbooks.
- **Author(s):** OxCAM contributors

## Problem

Deep pockets and long tool overhangs make side milling slow or unstable: the radial cutting
force bends the tool. Plunging along the tool axis puts the force mostly along the spindle,
which a long tool takes much better, so material can be removed by a series of overlapping
plunges, like drilling with an end mill or a plunge mill.

## Method

Input: the area to clear (the selected floor faces' outline, as for a 2D pocket), a top and a
bottom height, the row spacing and the plunge spacing (fractions of the tool diameter D), stock
to leave and two switches.

1. **Tool centres:** the area shrunk by the tool radius plus the side allowance.
2. **Interior plunges:** rows parallel to X across the centre area, spaced by the row spacing
   and centred in its height. Plunges along a row are a plunge spacing apart; with *stagger*
   every other row is shifted by half a spacing. Positions outside the centre area are dropped.
   Rows are cut in zigzag order.
3. **Wall plunges** (optional, last): positions every plunge spacing along each boundary loop of
   the centre area (outer loops and islands). Interior positions closer to the wall than half
   the smaller spacing are dropped, since the wall plunges cover them.
4. **Motion:** rapid to the first position at the retract height, down to the feed clearance
   above the top, then for each position a plunge at the plunge feed to the bottom (plus the
   bottom allowance), a rapid straight back up to the feed clearance and a rapid across to the
   next position. Sideways moves never happen below the top.

**Coverage.** Two plunges of diameter D overlap without leaving material between them when
their distance is below D. On the staggered grid the farthest point from all plunge centres is
in the middle of a cell of half a spacing by one row; the toolpath warns when
(spacing/2)^2 + rows^2 (or spacing^2 + rows^2 without stagger) exceeds D^2. Between plunges the
floor keeps small cusps whose height depends on the spacings, and the wall keeps cusps
r - sqrt(r^2 - (s/2)^2) deep between wall plunges a spacing s apart (0.67 mm for a 10 mm tool at
half its diameter), as with any plunge roughing; a finishing contour removes them.

**Speeds and feeds.** The plunge feed is the cutting preset's feed times its plunge factor,
which the operation can change with its customized cutting values. An unset row spacing takes
the preset's radial depth ae.

Non-centre-cutting plunge mills need each plunge to overlap material already removed by the
previous one, as the staggered rows and the wall plunges do; the strategy does not check the
tool's centre cutting capability.

## Sources

- Sandvik Coromant, *Metalcutting technical guide*, milling section on plunge milling (general
  application guidance: radial step, overlap and feed for plunge milling).
- P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press, 2008 (canned cycles and
  plunging motion).
