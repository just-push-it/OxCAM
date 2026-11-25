# DL-0067: A lathe's machine in the simulation

- **Status:** implemented
- **Module:** libs/machine (`LatheLayout`, `Turret.hpp`, machine file version 8, `generic-lathe-turret`, `generic-lathe-sub`,
  the model's lathe axes), libs/sim (`LatheMachine.hpp`), libs/document (`LatheMachine.hpp`, `Setup::turret`,
  `OperationSimulation::latheMotion`, `SetupSimulation::turretTools`), libs/post (`LowerOptions::indexAtHome`, `FanucLathe`),
  apps/opencam-gui (`SimulationPanel`, `PropertyPanel`, `MachineLibraryDialog`)
- **Patent gate:** low (citation only). Forward kinematics of an axis tree, and checking solids at sampled poses, are DL-0037's
  textbook material. A solid of revolution compared with another solid slice by slice is plain geometry. Collisions are reported, never
  avoided: no toolpath, speed or feed changes. The review group has not looked at this; it should have the usual
  `needs-patent-review` label on any pull request.
- **Author(s):** OxCAM contributors

## Problem

The maintainer asked for lathe machine simulation: turrets, chucks, enclosure clearance, tools staying on the turret and checked
against the enclosure and chuck as it works, tailstock and sub-spindle, and later mill-turn. DL-0037 said *lathes have no machine
model yet*. Until now only the insert and an ISO holder were checked, in the lathe plane, against the chuck and stock (DL-0044, DL-0045).

What makes a lathe different from a mill is that **every tool stays on the turret**. On a mill the tool in the spindle is the only one
near anything. On a lathe, eleven tools that are not cutting go wherever the one cutting goes, and a tool change swings them through
the space around the part.

## Findings

**How tool numbers map to stations.** There was no map. The post writes `T{tool2}{tool2}`, so a tool's number is both its station and
its offset: `T0303`. `Machine::toolCapacity` (12) said how many there are and nothing used it. So the mapping stays one fact: **a
tool's number is its station**. The turret table on the setup shows it and edits it. A second number that the program did not write would be
a simulation of some other program. Two tools at one station, or a station the turret does not have, are reported.

**What the program does at a tool change.** `T0505` where the last operation left the turret: no move first. Then along Z and in on X
(see *The approach*). The simulation does exactly that.

## Approach

### The layout

A lathe's machine is a `LatheLayout` on the machine file (version 8, `lathe`), and having one is what turns all of this on. A setup on a
machine without one simulates exactly as before: the only thing added is one `if`. So the built-in `generic-2axis-lathe` and both live
lathes are unchanged, and the built-ins `generic-lathe-turret` and `generic-lathe-sub` have it. Their numbers are placeholders like the
other generic machines'.

**A user with no CAD of their machine** ticks *Simulate the machine* in the machine library's Lathe turret box and types sizes:
the turret's centre at home, disc diameter and width, how far a tool reaches, the headstock, chuck, enclosure, tailstock and sub-spindle.
Those become boxes and cylinders on the axes (`latheSolids`), for display and checks. **A user with CAD** imports it as on a mill
(DL-0037): STEP parts with carriers, now including T, C, W, Z2 and C2, and sets the layout's size to 0 for what a part replaces. The
layout still says where tools sit, which CAD of a machine cannot.

### The tree

The model frame: Z along the spindle's centre line, +Z away from the headstock (ISO 841), the spindle's nose at Z 0, +X towards the
turret. Y = Z x X.

| Axis | On | Moves | Value |
|---|---|---|---|
| Z | base | the carriage | The program's, less the tool's reach |
| X | Z | the slide | |
| Y | X | where the machine has one | |
| T | Y or X | turns the turret about its index line | Station k working at (k - 1) 2 pi / stations |
| C | base | turns the main spindle, about -Z (DL-0066's sense) | Live tools only; 0 when the spindle turns the part |
| W | base | the tailstock | From the setup's centre |
| Z2, C2 | base, Z2 | the sub-spindle | |

`ModelAxis` and `poseAt` are DL-0037's. `isRotaryAxis` says what turns by its first letter, so a B-axis head (`B` on Y, carrying the
tools) and a second channel (`X2`, `Z2`, `T2`) add axes rather than a new system. What is deliberately **not** decided yet for them:
tool numbers per turret, and `Machine::kinematics` still has one lathe type.

### Stations and tools

A tool on the turret is a set of convex prisms (`ConvexPrism`) in its own frame, its programmed point at the origin: for a turning
tool, the insert and `turningToolParts` (DL-0056), and for a drill or a live tool, `ToolShape`'s cylinders.

How it sits (`StationMount`, `TurretStation`):

- **Across** the spindle: a shank along X, its centre on the disc's face, the tip `radialReach` from the index line.
- **Along** it: a bar's centre line `axialPitch` from the index line, the tip `axialReach` beyond the disc's face.

A turning tool lies along if it cuts inside; a hole or milling tool along unless its operation has an orientation across the part.
Both values and which way it lies are shown in the setup's turret table and can be typed: that is what a tool setter measures.
`Setup::turret` holds only what the user changed, and tools that **ride along** with no operation here.

`tip` is where the tool's programmed point is in the working position. At T 0 a station k stands turned back by its own angle:
`turretTurn(-angle(k)) * placement`. Which tool is working decides the slides: (G_T - G_C) l = F_C target - tip, where F and G are below.

### The moves between operations

A toolpath says nothing about these, so each operation has a `LatheMotion`: the station, the tip, what T, W and Z2 hold, and
`steps`, the moves before its first:

1. **The first operation:** the machine at home with that tool working, then the sub-spindle taking the part if any, then the tailstock.
2. **A tool change:** `Index` (`indexTo`, the short way round, half a turn forward), where the last operation ended. If the machine says
   *Index at home*, two rapids home first, and **the post writes the same** (`G28 U0.`, `G28 W0.` before the `T`; `LowerOptions::indexAtHome`).
   Simulation and program must not differ, so this is one setting.
3. **To the first position:** along Z, then in on X and Y, as the post's rapids go (below).

Each step is reported by what it was (`MachineCollision::during`), with the operation it leads to, at move 0.

### The approach

The post used to go in X, then along Z. With the machine checked that showed: from home with a tailstock in, the tool went in to its radius and
then along Z through the tailstock. The maintainer decided on **Z first, then in on X**, for every lathe program.

Going through every lathe fixture on the turret machine (`test_document "[approachsurvey]"`) found one thing Z first gets wrong: **a start further out and
deeper than the tool stands.** After a hole near the axis, a cross hole or a flat down the part starts at, say, radius 31 and Z -30 with the tool at radius 12 in front
of the part. Along Z at radius 12 goes straight through it. So the rule (`approachOutFirst`) is:

| The start, from where the last toolpath ended | Goes |
|---|---|
| Further out **and** deeper | Out on X, then along Z |
| Anything else | Along Z, then in on X |
| The first toolpath, or after going home | Along Z, then in on X |

Both the post (`Lower`, `LowerOptions::latheApproach`) and the simulation call the one function, with the same program positions, so what is checked is what is posted.
The radius is `hypot(x, y)`, so a C angle does not change it. Mills stay as they were.

**Bores.** Inside tools start in front of the part (radius 26, Z 6 in `bored_project`) and their own moves go in, so no approach travels Z into a bore at the
wrong X on any fixture. Nothing in the rule knows where the stock is: a toolpath that starts inside a bore from further out would go along Z, then through the wall, and
the machine check reports that. None of them does.

**From home.** X is large there, so Z first runs clear of chuck and work: no fixture reports anything in a first approach.

**The spindle** now starts between Z and X (`G96` is written after the Z block), so constant surface speed starts with the tool further out and speeds up as it comes in, under the
`G50` limit. Nothing is written differently for it, and no speed or feed changes.

Tool changes: the rule uses the program positions the post knows, the last tool's. The new tool's tip stands elsewhere after an index, by the difference in their offsets. The
machine check uses the real tips, so it sees what that leaves.

**The fixtures, on the turret machine, tip in the lathe plane and whole machine in the rapid:**

| | X first | Z first, out first when it must |
|---|---|---|
| Tip's way through the stock | None | None (2 with Z first alone: `live_project` Cross hole, `millturn_project` Flat) |
| Rapid reports against part, stock, tailstock | 19 | 0 |
| Rapid reports against the chuck | 34 | 28 |

The chuck's are the fixtures' short bars against this machine's 210 mm chuck, with no workholding of their own, and tools sticking out towards it: no order helps those.

**A program, before and after** (`shaft_project`, each tool change; 12 lines in 166 differ, the same words in another order):

```
N20 T0101 (CNMG 120404)          N20 T0101 (CNMG 120404)
N30 G0 G54 X42.                  N30 G0 G54 Z6.
N40 G50 S3000                    N40 G50 S3000
N50 G96 S200 M03                 N50 G96 S200 M03
N60 Z6.                          N60 X42.
N70 M08                          N70 M08
```

`live_project`'s cross hole stays as it was (`N750 X62.32`, `N760 Z-30.`). No golden program is a lathe's (`tests/golden/posts` are mills), so none changed, and every lathe
fixture still posts and verifies against its back-plot.

### Collisions

**What meets what:**

| | Enclosure | Machine's chuck, tailstock body, sub-spindle | Setup's chuck, tailstock, steady rest | Stock, part |
|---|---|---|---|---|
| Turret | Yes | Yes | Yes | Yes |
| A tool not cutting, or any tool before the first move | Yes | Yes | Yes | Yes |
| The tool cutting | Yes | Yes | The turning checks (DL-0045) | The turning checks and milling simulation |
| Tailstock body, sub-spindle (while they move) | | What turns on another spindle | Yes | Unless holding it |

**What turns sweeps an area.** The chuck, the stock and whatever a spindle carries (a STEP chuck too, through `revolvedEnvelope`) are
checked as a solid of revolution, whatever C says. A convex polytope against an area in the lathe plane: slice it across the axis, and
compare the radii of each section with the stretches of radius inside the area at that z (`polytopeToRevolved`). Sections 1 mm apart,
at most 64. The clearance widens both, and sections go past the solid's ends by it.

**The stock only loses material.** So each pose is compared with the stock as the operation starts. Only where that is met is the stock
worked out at this move (`LatheStockTrack`, `simulateTurning` on from where it got to), and compared again. A long job that stays clear
never looks. A flat milled on the bar (DL-0066) is not in the profile, so it counts as still there.

**The enclosure is a box.** A polytope's box is exactly how far its vertices reach, so holding it inside the enclosure's inner box is exact and
free. Walls as solids would miss a tool that has gone right through, which the first version of the test did. A mesh part still meets the
walls as solids, since its box is only a bound.

**Everything else** is DL-0037's: `detail::convexToMesh`, `meshToMesh`, GJK (`MachineContacts.hpp`, moved from `MachineCheck.cpp`), every
2 mm of travel, at rapids' ends, and every 2 mm at the turret's reach for an index. Bodies on the same chain are skipped.

**Arriving.** At the very end of the last move before a toolpath, the tool is at the toolpath's first position, and that is the one cutting from there: it may
stand closer to the work than the clearance, as a feed height does.

**Reported** as `Tool or part / what`, with the operation and the moves. `second` is `part` when the solid also reaches into the part.

**Cost.** Slides have no rotary above them (`validateLatheModel`), so a body's motion is F (with the slides at 0, changing only when T, W or Z2
do) moved by G times the slides. A pose is a matrix and a vector per body, and boxes are shifted, not recomputed. Boxes are compared per
body, then per solid.

### Where the work sits

`latheWorkMount`: on the spindle's centre line, the WCS's Z along +Z. Its held end goes to the nose when the setup brings a chuck, or into
the machine's chuck by its grip (the stock's back at chuck length less grip). A placement's Z offset moves it, shown as *Work along the
spindle*. A sub-spindle's work has Z turned towards the main spindle.

### The tailstock

`LatheTailstock` on the setup (DL-0052) stays what it is for generating, and is the centre and quill here. The machine has a body on W.
W is where the body meets the back of the quill, and it comes in as the first step of the setup, with the quill on W as well.

### The sub-spindle

**What is done:**

- The tree: Z2, C2, a housing and a chuck on them. A setup's placement carrier C2 puts its work there; the setup page has a *Spindle* choice on a
  machine with one. Its WCS has Z towards the main spindle, which is what a setup turned round already is (DL-0044), so generating is
  unchanged.
- The tools work from the disc's other face.
- **Transfer:** when a transfer operation of the setup its stock comes from hands it the part (ADR-0007, DL-0092; before that, whenever its stock came
  from a setup on the main spindle of the same machine), Z2 goes in to where the part in the two setups is the same
  place in the project (`take`), the part staying where it is, then back to **Z2 while it cuts** with the part. Both are checked. The sub chuck against the machine's
  chuck, and turret and tools against it all. A part that does not line up is a warning, and the transfer is left out.
- Simulated and playing back. **Posting** was refused until ADR-0007: the transfer and the sub-spindle's work now post as one program with the main
  setup's, the codes from the post's options (DL-0092).

**What is not:** no synchronised rotation, no parting off during the transfer as one move; the part's stub left on the main spindle is not there;
the two setups simulate separately, so the part does not visibly go across between them.

### Display

`SimulationPanel` draws every tool on the turret as parts of the machine scene, solids by their tool name, the insert or cutting part gold, the
rest as holders are. So in machine view nothing is drawn at the tip: the tool cutting is on the turret like the others, and its tip is on the
programmed point by construction. The Tool box hides them all. Without the machine shown it is as before. They are not in the machine's list of
parts.

Playback plays the steps before an operation's first move when it starts from move 0: the slider's move 0 is the machine after them, and time
before it is `Position::between`. The axis row says `X Y Z`, C when it is used, the station, W and Z2. A collision in a step reads "in the
turret's index before its first move".

A lathe's first view is from its door, spindle on the left: Z up as on a mill stands it on end. The door and the roof start hidden.
The door and the roof are the walls on the front and top as the machine stands (DL-0077).

**Hiding the enclosure** (2026-11-08). The maintainer: "next up I'd like to be able to hide the enclosure of a machine model". Which parts are
the enclosure is the machine's to say, never guessed from names: a model part has `enclosure` (machine file version 14), set in the machine
library's parts table (*Enclosure*), and the walls a layout's enclosure box gives are always enclosure. An *Enclosure* box beside the machine's
other toggles in the simulation, the turret preview (beside *Whole machine*, which still means the turret, the spindle and the work) and the
machine library's preview hides those parts together and keeps the rest: the bed, the headstock, the turret, the tailstock. Each view remembers
its choice (`simulationEnclosure`, `turretPreviewEnclosure`, `machineLibraryEnclosure`). It is display only: the enclosure is checked for
collisions as before. The Wasino LJ-63M's enclosure is its layout's box, so its walls are enclosure without a change to its file; enclosure
parts of its own will say so when they come.

**The quill's travel** (2026-11-08). The maintainer pointed out that the quill goes out a set distance under program control: the
LJ-63M's goes out 100 mm (its manual's specifications, "Quill travel"; M32 advances it, M33 retracts it). The tailstock in the machine file
has `quill_travel` (machine file version 15), set in the machine library as *Quill travel*; absent or 0, the machine does not say. A setup's
*Quill length* (how far its quill is out) further than that is said, naming both and where to change it, on the setup page under the field
and in the simulation's warnings (`quillBeyondTravel`). It is never shortened to fit: the setup stays as the user set it, and the
simulation shows the quill where the setup has it. A machine that does not say says nothing.

## Measurements

The shaft of DL-0061 from a longer bar (face, rough 1.5 mm, finish, part off), RelWithDebInfo with asserts, best of five,
`test_document "[lathemachinebench]"`:

| Machine | Whole simulation |
|---|---|
| Lathe without a layout | 5.0 to 5.3 ms |
| With a layout, the job's 2 tools on the turret | 12.0 to 12.5 ms |
| 3 tools | 12.1 to 12.9 ms |
| 6 tools | 13.0 to 13.6 ms |
| 13 tools | 16.1 to 17.3 ms |

About 7 ms goes once, making the check: the solids, the swept areas and the boxes. Each tool adds about 0.3 ms. A job with no layout runs
none of this: one `if` in `simulateLatheSetup` and two in the keys.

Playback, the same build, 3 s of `--sim-play` from mid-roughing at 0.05 mm rays (frames shown):

| Job | Frames in 3 s |
|---|---|
| `shaft_project`, no layout | 69 to 71 |
| `turret_project`, 5 tools and the machine drawn | 63 to 64 |
| `turret_full_project`, 12 tools in 12 stations | 64 |

So the machine costs a frame about 10%, and more tools nothing that shows: the machine's parts keep their presentations and only their places change.
The check runs once in the simulation, never per frame.

## Sources

- ISO 841:2001, *Industrial automation systems and integration: Numerical control of machines: Coordinate system and motion
  nomenclature*. X, Z and W on a turning machine (paraphrased).
- J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 3rd ed., Pearson, 2005. Forward kinematics, as DL-0037.
- C. Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2005. GJK, bounding boxes, as DL-0037.
- A. M. Andrew, *Another efficient algorithm for convex hulls in two dimensions*, Information Processing Letters 9(5), 1979. The monotone chain.

## Deviations from sources

None: nothing here follows a source further than the kinematics above.

## Correctness and tests

- `TurretTest`: the tree and solids, stations and the short way round, a tool's default place, the axes putting the working tool on the program's
  point (turning, a tool of another reach, polar, held C with Y), the file round trip, older files, the model's problems.
- `sim` `LatheMachineTest`, on made-up moves, at the move each happens: clean; **a tool not cutting against the chuck**; the tool cutting left
  to the turning checks; **the disc against the enclosure**, and a tool opposite going first; **an index** swinging through the work, told from
  the part, and clear when standing back; **the disc against the tailstock**, the tailstock coming in on a parked turret, and a quill on W; material
  already cut away; the prism's mesh; a mesh turret and a mesh wall.
- `document` `LatheMachineTest`, the shaft: clean with the machine moving; no layout as before; a tool riding along against the chuck at the operation and
  move; an index reported on the operation it leads to; the enclosure; the tailstock clear and a short quill not; the turret's load and its
  problems; the setup and the cache; the fixtures; the sub-spindle's transfer lined up to 1e-9, and posting refused; index at home in both the machine and
  the program; a setup's own chuck in place of the machine's; a steady rest; live tools, the work turned by C to the tool at every move.
- `FixturePostTest` posts and verifies `turret_project` and `turret_tailstock_project`.
- `gui.lathe_machine_*`: playback with the machine in view on each fixture, the setup page's turret, playing through an index.
  `gui.lathe_machine_turret_edit` works the setup page's own widgets: adds a tool that rides along, moves a station (the tool's number changes), sets a tip, resets it.
  `gui.machine_library_lathe_layout` opens the layout in the editor.

## Known limitations

- **Where a program starts.** The machine is taken to be at home with the first tool working. A program does not say. Started from elsewhere, its first Z move is at whatever X it was left at.
- **The approach does not know the stock:** see *The approach*. It is reported, never avoided.
- **Between setups:** each setup is simulated on its own, so the last setup's position and station are not carried to the next.
- **The tool cutting and the stock:** only the existing checks, so a holder without an ISO designation, or a drill's holder, can still reach the stock unseen.
- **A live tool's C:** the machine turns the work, but the rapid's C is the straight line of DL-0064.
- **Tailstock in and out:** it comes in once per setup (DL-0052's unit), never out within one.
- **Station by tool number** means a tool has one station in a project. Two setups wanting it elsewhere need a copy of the tool.
- **Offsets:** `T0303` only, as the post writes. A second offset on one station, or two tools in one block, is not modelled.
- **Bar feeders, part catchers, steady rest travel, spindle covers:** not modelled.
- **Mill-turn:** not built. The tree takes a B head and second channels (above). What it does not yet have: tool numbers per turret, a head that tilts between
  operations, or two channels' timing.
- **Not run on a machine.** The geometry is not from any real machine, and the built-ins' numbers are placeholders.
