# DL-0009: Drilling with canned cycles

- **Status:** implemented (phase 3)
- **Module:** libs/strategies (`drill`), libs/toolpath (cycle IR and expansion)
- **Patent gate:** low (citation only); standard G81 to G85 cycle semantics
- **Author(s):** OxCAM contributors

## Problem

Drill, tap or bore recognized holes in a short order, keeping canned cycles in the
intermediate representation (IR) so posts can emit G81 to G85. Cycles are also expanded into
explicit moves, which simulation needs.

## Approach

**Order.** Nearest neighbour plus 2-opt from the WCS origin (DL-0003).

**Cycle selection** (unless the user picks one):

| Condition | Cycle |
|---|---|
| Tap | Tap (G84) |
| Reamer or boring bar | Bore (G85) |
| Dwell > 0 | Dwell (G82) |
| Depth > 3·D | Peck (G83) |
| Otherwise | Drill (G81) |

**Depth.**

- In full-diameter mode, add the point length R / tan(θ/2) for pointed tools.
- Through holes add the break-through distance.

**Cycle parameters.**

- R plane: the hole top plus the retract clearance.
- Peck depth: the operation's peck depth, a length (the tool diameter while not set).
- Tap feed: pitch × spindle speed.

**Travel between holes.** Holes at different top heights are reached at the clearance height.

**Expansion (G99 semantics).**

1. Rapid to the hole at the current height, then down to the R plane.
2. Feed to depth:
   - peck depths are measured from the R plane, as the control does (changed in phase 4 so
     simulation matches the machine);
   - pecks retract to the R plane (G83), or by 0.5 mm (G73);
   - re-entry rapids to 0.5 mm above the previous depth.
3. Return to the R plane: by rapid, except tap and bore, which feed out.

## Sources

- ISO 6983-1, *Numerical control of machines — Program format and definitions of address words*
  (cycle semantics, paraphrased).
- P. Smid, *CNC Programming Handbook*, Industrial Press (fixed cycles).

## Correctness and tests

- `tests/unit/strategies/StrategiesTest.cpp`: ordering, depth modes, automatic cycle choice, tap
  feed, mixed hole heights, golden file `tests/golden/drill_mixed.json`.
- `tests/unit/toolpath/ToolpathTest.cpp`: peck expansion and dwell timing.

## Known limitations

- Holes are drilled only along the setup's tool axis.
- Spot-drill chamfer sizing and thread-milling cycles are not implemented yet.
