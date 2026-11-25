# DL-0044: Lathe setups continue the stock of an earlier setup

- **Status:** implemented
- **Module:** libs/document (`latheGeometry`, `carriedLatheStock`, `simulateSetup`,
  `toolpathInputKey`), libs/strategies (`turnRough`, `turnFinish`, `trimmedToStock`),
  apps/opencam-gui (`--sim-last-setup`)
- **Patent gate:** low (citation only). A mirror in the lathe plane and plain geometry.
- **Author(s):** OxCAM contributors

## Problem

A lathe setup with **From earlier setup** as its stock got a plain bar around the part instead. A
second side after a flip was generated and simulated from material that was no longer there, and
the first side's cuts were not in the simulation.

## Approach

**What a setup starts from (`carriedLatheStock`).** The earlier setup's stock less everything its
enabled operations cut, in the same way a setup's own operations are simulated (`latheStockBefore`,
DL-0043), down the whole chain of earlier setups. It is then moved into this setup's lathe plane:

- Both setups' Z axes must lie on one line, the spindle axis. The same way round, the profile only
  shifts along Z; flipped, it is mirrored. The X axes do not matter to a revolved profile.
- Only the pieces holding some of this setup's part go on. What parting off left in the chuck stays
  behind there.

**What cannot be carried is refused, in generating and in simulating alike:**

| Case | Why |
|---|---|
| A setup in the chain is not on a lathe | What a mill cut (a flat, a pocket, cross holes) is not axisymmetric, so a profile cannot hold it. |
| The Z axes are not one line | A stock off the spindle axis is not a revolved profile about it either. |
| No earlier setup is chosen, or nothing of the part is left | There is no stock to start from. |

The first was a choice between refusing and warning. A warning would leave a bar in place of the
real stock, and the toolpath's approach and rapids are worked out from that stock: material it does
not know about can be in their way. A flat milled on a bar could also be approximated by its
envelope, which is safe for the tool, but it needs a mill-turn stock model to do properly. The
message says to turn the setup from a bar or tube, so the user states the stock instead.

**What is only warned of**, on the operations and the simulation:

- An operation of the earlier setup that failed is left out of the stock, in the wording milling's
  carried stock uses.

**The chuck.** A setup's workholding closes on what is there: `setupWorkholdingPose` gives the
largest radius of the carried stock within the holding's grip length from its back end, so the jaws
of a flipped part close on the turned diameter they hold.

**Passes on stock that is already cut.** Until now a lathe setup always started from a whole bar, so
every pass found material all along. On carried stock it would cut air over the finished side:

- A rough pass ends a clearance past the last material in its depth band, if that comes before the
  part or the back limit stop it (`turnRough`).
- Profile and finish passes end where they last meet the stock outside the allowance
  (`trimmedToStock`). Stock with nothing to cut leaves the path whole, as before.

A setup from a whole bar comes out as it did: its material reaches the part everywhere.

**Dependencies.** A lathe setup's toolpath key takes in the earlier setups in the chain with their
operations' tools, so a change on the first side reaches the second.

## Correctness and tests

`LatheStockTest` on `flip_project.ocam`, a spool turned from its front and then flipped:

- The second setup starts from the first's final stock, mirrored, with the same volume.
- It finishes the part: no gouge, the corners' nose radius as the only excess, and the part's volume.
- Its rough passes stop at the flange and do not run along the first side.
- Each refusal in generating and in simulating, a mill further up the chain, and each warning.
- A change on the first side changes the second side's toolpath key.
- After parting off, only the part's piece goes on.
- `gui.lathe_simulation_flip` draws the carried stock, and the program posts and verifies.

## Known limitations

- **Chuck on a flipped part:** drawn and checked at the first bar's diameter (warned of, above).
  Closing on a turned diameter needs the pose to take the carried profile.
- **Before simulating:** the 3D view shows the first bar's outline for such a setup, since
  `Document::stockShape` resolves to the source's initial shape. So do the heights measured from
  the stock: they stay clear of what is left, but can be higher than they need to be.
- **Turning after milling:** refused here. **Milling after turning** is unchanged (DL-0012).
- **First side's reach:** an operation cannot be limited to part of the profile yet, so the first
  side's finish runs on past a flange at its trailing angle, into stock the second side removes.
