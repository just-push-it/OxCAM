# DL-0050: Left-hand tools and back turning

- **Status:** implemented
- **Module:** libs/strategies (`turnRough`, `turnFinish`), libs/tooling (`mirroredInZ`,
  `turningAwayFromChuck`, `turningBody`), libs/document (`generateToolpath`, `latheCutter`)
- **Patent gate:** low (citation only). A mirror in the lathe plane.
- **Author(s):** OxCAM contributors

## Problem

Every pass ran towards the chuck, and left-hand holders were refused for it. So a part behind a flange
could not be reached from behind, and a tool's hand said nothing to the toolpath.

## Approach

**Direction** on `turn_rough` and `turn_finish`: towards the chuck, or away from it.

Away is the same pass strategy on the geometry mirrored in Z, with the moves mirrored back. Everything
those passes know (allowances, edge limits, Z ranges, clearing) stays one code path. Tip quadrants 4
outside and 1 inside record it: the simulation cuts with the outline mirrored, a neutral holder's body
stands mirrored, a left-hand one always did.

Nothing changes for a chuck towards which a tool runs away: its stop does not apply, and the chuck
check on the moves refuses what actually reaches it.

**The hand has to agree with the direction**, because passes the other way drag the tool's back edge
through the work. On turning inserts, ISO holders included (they set the insert's hand):

- A left-hand tool towards the chuck is refused.
- A right-hand tool away from it is refused.
- A neutral tool runs either way.

Before, a left-hand ISO holder was refused for all turning, and a left-hand insert without a holder
was not looked at.

**Where it can start.** Without a Z range, away starts behind the stock's back end, which is usually
the chuck, and is refused for reaching it. The usual use is a range starting behind what it cuts, in a
relief or at a part's free end. Stock next to that start is refused as for any range (DL-0049).

## Correctness and tests

`LatheStockTest` "back turning with a left-hand tool", on `backturn_project.ocam`, a spool with both
ends turned towards its flange: its passes run forwards from behind up to the flange; the part comes
out with no gouge; the tool's outline is mirrored; the hand and the direction have to agree; an ISO
left-hand holder works; stock in the way is refused. It posts and verifies, and was plotted.

## Known limitations

- **Facing and parting** stay towards the axis from the outside, whatever the hand: they are not
  mirrored in Z, so a left-hand tool is refused for facing.
- **Undercuts between two walls:** a recess that opens neither way is a groove's.
