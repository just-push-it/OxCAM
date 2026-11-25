# DL-0045: Turning holders against the chuck, the part and the stock

- **Status:** implemented
- **Module:** libs/tooling (`turningBody`), libs/sim (`checkTurningBody`, `checkTurningFixtures`),
  libs/document (`generateToolpath`, `simulateSetup`)
- **Patent gate:** low (citation only). Plain geometry in the lathe plane.
- **Author(s):** OxCAM contributors

## Problem

Only the insert was checked against the chuck, and nothing that does not cut was checked against the
stock or the part. A 25 mm boring bar in a 20 mm bore generated a program without a word.

## Approach

**The body (`turningBody`).** ISO holders have a known shape in the lathe plane (DL-0042): the head
beyond the insert and the shank, both convex. What lies beside the insert is left out: that is the
insert's own relief, and a sweep of it would "hit" what the insert is cutting. A holder without a
designation, or no holder, gives no body: its revolved outline says nothing about this plane, and
nothing is claimed about it.

**Across the axis.** A boring bar's axis lies below the tip, and near the axis its far side lies
beyond it, where it meets the wall on the other side. A body's sweep therefore also counts mirrored
over the axis. That is the geometry behind a bar's minimum bore, with no table needed.

**Where each is checked:**

| Against | When | Result |
|---|---|---|
| The chuck | Generating, simulating | Refused; collision in the simulation |
| The part | Generating | Refused: the part is there whatever came before |
| The stock | Simulating, move by move as the insert cuts | Holder collision in the simulation |

The stock is left to the simulation because it depends on every earlier operation: generating would
need them all, on every change. All moves count, rapids included.

Arcs are taken by their chord for the body: lathe toolpaths are mostly lines, and holders stand well
off the cut.

## Correctness and tests

- **`LatheHoleTest`:** an `S25T` bar in a 20 mm bore is refused, naming the holder and the part; an
  `S08H` bar runs with no collision. Facing a blind bore's bottom to the axis needs more than the
  bar's minimum bore: with the tip on the axis its far side is f + h/2 out, so a 10 mm bar
  (12 mm from the tip) cannot do it in a 20 mm bore, and is refused.
- **`LatheTest`:** a bar's far side meets the wall across the axis, and clears a wider bore; the same
  against a chuck body.

## Known limitations

- **Holders without a designation:** not checked. They have a design for milling, a revolved outline.
- **Out of the plane:** a square shank's corners out of the lathe plane are not considered, nor the
  turret.
