# DL-0066: Milling with a live tool

- **Status:** implemented
- **Module:** libs/strategies (`live_axes`, `live_c`), libs/document (`liveGoesAround`, `generateToolpath`, `simulateLatheSetup`),
  libs/sim (`latheStockDexels`, `clipToLathe`), libs/machine (`solveLiveHeld`, `solveLivePolar`), libs/post (`Polar`, `FanucLathe`,
  back-plot), apps/opencam-gui (`SimulationPanel`, `PropertyPanel`)
- **Patent gate:** low (citation only). Polar interpolation and milling at a held C are ordinary mill-turn practice; code
  syntax from a controller's training manual.
- **Author(s):** OxCAM contributors

## Problem

DL-0063 stopped at holes. The maintainer asked for live tool milling, thread milling and helical boring. Every 2.5D and 3D
milling strategy already makes a toolpath in an operation's frame, so what was missing is the machine (how a lathe moves a
tool around), the program, and a stock that can hold what a mill leaves.

## Approach

**What goes around.** Any operation that is not turning, on a lathe, is a live tool going around what it cuts, except a drill,
a tap, and a bore with a reamer or a boring bar, which go straight in (DL-0063, DL-0065). A machine without live tooling refuses
them. The multi-axis strategies are refused on a lathe.

**The choice is a setting.** Each of those operations has **Live tool axes**, on the operation's page, on lathes only. Nothing
picks it from the geometry:

| Live tool axes | On the face | Across the part |
|---|---|---|
| *C and X: polar interpolation* (the default) | Goes, on any machine with a C axis | **Refused**, naming the setting |
| *Y axis: C held still* | Goes at **C held at**, a value on the page; without a Y axis, refused if it leaves Y 0 | C from the tool's direction; without a Y axis, refused if it leaves Y 0 |

A keyway along the part at Y 0 therefore goes without a Y axis. What is left of a turret's reach is DL-0063's: along Z or square
across, one direction per operation.

**The machine (`solveLiveHeld`, `solveLivePolar`).** Held still, the toolpath turns by C and is X, Y and Z, with X a diameter in the
program as elsewhere on a lathe. In polar interpolation rapids go in X and C, and each run of feed moves goes between the control's
polar on and off (`polar`, NC program version 4), in the part's X and Y. Going in and out around rapids is what both controls allow:
Fanuc has no G0 inside G12.1. A helix becomes lines in polar interpolation, as controls differ on Z moving with an arc there. Arcs are
written with R on a live tool, as the manual's examples are. A tool across the part has lines where it had arcs, as the toolpath's
transform has always done off the Z axis.

**The feeds (ADR-0004).** As the operation shows them. Polar interpolation is what keeps the feed the programmed one: the manual
says C is calculated so the feed stays constant. Nothing scales it on the way, in the toolpath or the post.

**The codes are options** on the post:

| Option | `haas-lathe` | `fanuc-generic-lathe` | From |
|---|---|---|---|
| Polar interpolation on | `G17`, `G112` | `G1 G12.1` | Haas Factory Outlet, *Live Tool for Lathe* training manual, September 2014, which gives both |
| Polar interpolation off | `G113`, `G18` | `G13.1` | The same |
| Letter for Y | `Y` | `C` | The same: in Fanuc, Y is programmed as a C value |
| X is a diameter there | No | Yes | The same: diameters in Fanuc, radius in Haas |

Those are the control's own codes rather than a builder's, so `fanuc-generic-lathe` starts with Fanuc's. The live tool and C axis codes
stay the builder's (DL-0063), so it still refuses until those are set. The same manual has the G17 axial mill with Y, X in diameter,
which is what *Y axis* writes, and G18 back on the way out.

**The stock.** Nothing but dexels holds what a mill leaves, so a setup a live tool mills has them (`SetupSimulation::milledFrom`):

- Before the first milling operation the profile plays as it always did, exact, and dexels are kept beside it for colouring and for
  the comparison.
- From there on the stock is dexels, the milling simulation's, with its rapid, shank and holder checks, and its comparison with the part.
- A turning operation after the milling clips the dexels to the new profile (`clipToLathe`), so the flat stays when the bar is
  turned again. Playback does that frame by frame.
- A setup with no milling is untouched: same profile, same speed.

**Holes in dexels.** The conversion's first form (DL-0064) filled rays from the display mesh, and it lost isolated rays on the hole's seams:
a long gouge on one ray. Holes are now cut analytically, frustum by frustum along the hole's side. Threads still go by their mesh,
which has no such seams.

**Thread milling against the part.** A thread mill finds a hole modelled at its tap drill, and takes the thread out of the part's material,
which read as a gouge, on mills too. What a thread mill removes is now treated as a chamfer's cut is: meant, and not required of the stock.

## Sources

Syntax facts only: Haas Factory Outlet, a division of Productivity Inc., [*Live Tool for Lathe* training manual, September 2014](https://www.productivity.com/wp-content/uploads/2019/01/Haas-Live-Tooling-for-Haas-Lathes.pdf):
G112 (p. 24 on), G17 axial mill with Y (p. 21), M133, M135, M154, M155, G95 and G195.

## Correctness and tests

`LiveMillTest`: what goes how, the refusals (polar across the part, no Y, no live tooling); the whole fixture simulated with no
collision, volumes taken, dexels against the exact profile, no gouge, turning after milling, the cache; the Haas program's words,
feeds as the operation's, and the back-plot agreeing with the toolpaths through polar interpolation. `gui.lathe_mill_*`.

## Known limitations

- **Cylindrical interpolation (G107, G7.1):** DL-0074.
- **Simultaneous C** is not used for milling beyond polar interpolation.
- **The control's cutter compensation** is refused on a live tool, so a contour's offset is the computer's.
- **Helix in polar interpolation** relies on the control moving Z with X and C. Haas's examples move Z in blocks of its own, and a report on
  a Fanuc 0i-TB says it will not move them together. On such a control use the Y axis.
- **Rapids in X and C** are checked as the straight line in the part's frame, and the real move is an arc around the axis. They run at the
  clearance height, but a part standing proud of it would not be seen.
- **C speed** near the axis in polar interpolation is not checked: the manual notes C turns faster there, and a control may slow.
- **Dexels' resolution:** from the first milling operation on, a thread finer than the spacing does not resolve in the display.
- **Another lathe setup** going on from this one: DL-0073.
- **Not run on a machine.** Run the first one in the air: above all the C direction (the manual says positive is clockwise, facing the
  chuck, which is what this assumes) and the sign of Y.
