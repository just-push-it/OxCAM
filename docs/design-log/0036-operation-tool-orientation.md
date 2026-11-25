# DL-0036: Tool orientation per operation (positional 3+2 within a setup)

- **Status:** implemented
- **Module:** libs/document (`Operation::orientation`, `operationSetup`, generation in the
  operation's frame, `operationIndex`, `lowerSetup`, rest stock in a frame), libs/machine
  (`indexSolution`, `solveIndexed`), libs/toolpath (`transformed`), libs/sim
  (`MachineCheckOptions::index`), apps (operation panel, view triad), schema
- **Patent gate:** low (citation only). The orientation is chosen by the user, per operation;
  nothing chooses or changes a tool axis. Indexing is the closed-form inverse kinematics of
  DL-0019 and DL-0023.
- **Author(s):** OxCAM contributors

## Problem

DL-0019 indexes a whole setup: the setup's WCS Z is the tool axis of all its operations. On a 4-
or 5-axis machine one fixturing usually machines several faces of a part, each at its own
index (front face, side holes, the back). Each of those should be an operation of the same
setup, posted with its own rotary indexing, simulated on the same in-process stock, and refused
with a clear message when the machine cannot reach it.

## Approach

### Data

- `Operation::orientation` (optional `ToolOrientation`): a frame **in the setup's WCS** whose Z
  axis is the tool axis and whose X and Y are the operation's X and Y, plus a work offset
  (0 = the setup's). Unset, the operation uses the setup's WCS as before; older files read
  unchanged. The project file writes `tool_orientation: {frame, work_offset}` on the operation.
- Changing the setup's WCS turns its operations' orientations with it.

### Generation in the operation's frame

- `operationSetup(setup, op)` is the setup with its WCS replaced by the operation's frame (WCS ·
  frame, in the project frame). The operation is generated against it unchanged: the part mesh,
  stock outline, selected faces, holes, heights (`operationHeights` measures along the
  operation's Z; "Origin" is the frame origin) and fixture avoidance are all in that frame. The
  strategies see an ordinary 3-axis problem.
- Fixtures and the workholding (DL-0034, DL-0035) are placed on the setup itself (a vise closes
  on the stock as the setup's WCS has it) and then moved into the operation's frame, so the tool
  keeps the setup's fixture clearance from them along any orientation.
- The generated toolpath (in its own frame, tool along +Z, arcs and canned cycles intact) is
  what the toolpath cache keeps (`cachedToolpathInFrame`). Everything that works in the setup's
  WCS (display, simulation, machine checks, the Python `generate`) gets it mapped back by the
  frame (`toolpathInWcs`): positions and arc centres move rigidly, every move's tool axis
  becomes the frame's Z, arcs whose plane is no longer an arc plane become lines within 1 µm
  and canned cycles are expanded (`toolpath::transformed`). Feeds are unchanged: a rigid motion
  keeps every length.
- The input key includes the orientation (it is part of the operation) and, for operations with
  one, the setup's machine definition (reachability is checked against it).
- Turning operations refuse an orientation.

### Rest machining

- The in-process stock before an operation is built in the operation's frame
  (`stockBeforeOperation(..., frame)`): the initial stock mesh is moved into the frame and every
  earlier operation's WCS toolpath is mapped into it, then swept as usual (tilted tools sweep
  through the tilted-tool path of DL-0026). The rest strategies slice that stock at heights
  along the operation's Z, as they do for the setup's.

### Indexing (libs/machine)

- **Index solution** (`indexSolution`): table-table machines use `indexAngles` (DL-0019: the
  smallest tilt, then the smallest table rotation). The other kinematics take the solutions of
  `axisSolutions` with the rotating axis unwrapped nearest 0 and choose the smallest head tilt,
  then the smallest rotation. Machines without rotary axes reach only +Z.
- **Frame in the table frame.** On machines with rotary axes it is setup WCS · operation frame
  (the project frame is the table frame, DL-0023). On a 3-axis machine the setup's WCS is the
  machine's (a separate fixturing, DL-0019): only frames that turn about the WCS Z are
  reachable.
- **Positional solve** (`solveIndexed`): with the rotary axes fixed at the index angles,
  program positions are the machine positions of each tip (`machinePosition`: T · p, plus
  (pivot + L)(d − Z) when the head tilts) less the machine position of the work offset's
  origin. A frame Z that the angles do not turn up the spindle is an error.
  - **The setup's work offset** (`work_offset` 0) is measured, as in DL-0019, at the setup's
    WCS origin o_s with the table at the setup's index s (0 on machines that cannot index a
    setup). For an operation at index k the program is T_k p − T_s o_s. This needs no machine
    calibration beyond the project frame being the table frame: the part placed in the project
    where it sits relative to the table centre (where the rotary axes meet), as multi-axis
    posting already requires. For an operation at the setup's own orientation it reduces to the
    DL-0019 positions (T_s(p − o_s) = Rz(ψ) p_wcs).
  - **A work offset of its own** (`work_offset` ≥ 1) is measured at the frame's origin o_k
    with the machine at index k: the program is T_k(p − o_k), the DL-0019 rule applied to the
    operation. It does not depend on where the table centre is, at the cost of probing that
    offset. It may not be the setup's work offset number, and two operations sharing a number
    must measure it at the same origin and index (refused otherwise).
  - With the tool along the machine Z (every table rotation) the frame's XY plane is the
    machine's XY plane, so XY arcs and canned cycles stay; ZX/YZ arcs become lines unless the
    frame is only shifted. With a tilted head arcs become lines and drilling cycles are
    expanded into moves; tapping is refused (it needs the spindle along Z).
  - Each solved move carries the tool axis in the table frame, so the back-plot through the
    machine's forward kinematics (`compareBackplot`) checks the index of every operation.
- Multi-axis operations (whose own toolpath tilts the tool) with an orientation are posted as
  multi-axis toolpaths in the setup's WCS (DL-0023).

### Posting

- `lowerSetup` posts each positional operation as a 3+2 job at its index. The existing
  lowering (DL-0019) emits `G0 [G5x] A.. C..` when the index changes and, when the tool is
  already in the spindle, a Z home (`G91 G28 Z0.`) first, so the tool is clear of the part
  before the table turns. Operations at the same index follow each other without re-indexing.
- Generation and posting refuse an orientation the setup's machine cannot reach, naming the
  machine and the direction; the operation panel shows the same message, or the index angles.

### Simulation and checks

- The stock simulation runs on the setup's tri-dexel stock with the mapped toolpaths (tilted
  tool sweeps). The machine model (DL-0037) moves to the operation's index for the whole
  toolpath (`machine::rotaryTrack` with an index, `MachineCheckOptions::index`), not to a plan
  from its axes, so its display and collision checks show the positions the post outputs. The
  fixture checks take the tilted toolpath against the fixtures in the WCS.

### GUI

- The operation panel has a tool orientation section: the setup's WCS, one of the six WCS
  directions, or a custom frame from "Z ⟂ face" (outward normal, origin at the face centre) and
  "X ∥ edge"; the origin in WCS coordinates; the work offset (the setup's or its own); and the
  index the setup's machine will use (or why it cannot).
- The view's triad shows the selected operation's frame. Toolpaths and the simulated tool are
  drawn from the WCS toolpath, so they show tilted.
- Picks for chamfer edges and edge loops, contour sides and heights use the operation's frame.

## Sources

- R.-S. Lee, C.-H. She, *Developing a postprocessor for three types of five-axis machine
  tools*, Int. J. Advanced Manufacturing Technology 13(9), 1997. Inverse kinematics and
  machine positions of table-table, head-table and head-head machines.
- J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 3rd ed., Pearson, 2005. Frame
  composition and rigid transforms.
- Controller manuals (Fanuc-style syntax facts only): rotary axis words, G28 reference return,
  work offsets G54–G59 and G54.1.

## Deviations from sources

None in the kinematics. The work offset rule for operations at another index than the setup's
(through the table centre) is the project's own convention, documented above.

## Correctness and tests

- **Toolpath (`ToolpathTest`):** a turn about Z keeps arcs and cycles (centres and depths
  moved); a tilt makes lines of arcs, expands cycles, sets the tool axis, keeps the cut length;
  mapping back returns the original points.
- **Machine (`KinematicsTest`):** index solutions reproduce the axis on every kinematic class;
  table-table ones equal `indexAngles`; unreachable directions (−Z on the trunnion, X on a 4th
  axis, anything but +Z on a 3-axis machine) give none. The positional solve gives T p − T_s o
  with the setup's offset and puts the frame origin at zero with its own; keeps arcs and cycles
  on a trunnion; on a tilted head gives `machinePosition`, expands cycles and refuses tapping;
  refuses angles that do not give the frame's axis.
- **Project (`ProjectTest`):** round trip, files without an orientation write and read none,
  validation of the frame and the work offset.
- **Document (`GenerateTest`):** a front-face facing on the block cuts at the part's front face
  with the tool along −Y, in its frame along +Z with its heights along it; the trunnion post has
  `A0. C0.`, `A-90. C0.` and the right face's index, a Z home before each turn, a back-plot that
  matches through the machine (positions and tool axes), and the front face 31 mm below the
  setup's offset; a work offset of its own puts the face at its origin and may not reuse the
  setup's; unreachable orientations (−Z on the trunnion; any tilt on a 3-axis machine) are
  refused with their reason; turns about Z post on a 3-axis machine without rotary words; the
  simulation removes the 1 mm front skin. Rest machining in a frame turned 90° about Z and
  shifted cuts the same corners as in the setup's WCS.
- **CLI:** `cli.project_post_3plus2` posts the hole plate on the trunnion with five faces and a
  front drilling cycle (`G81` at `A-90.`, back face on `G55`) and verifies the back-plot;
  `cli.project_simulate_3plus2` simulates it without gouges;
  `cli.project_simulate_3plus2_machine` simulates it on the demo trunnion built from STEP parts
  with no machine collisions.
- **Workholding (`WorkholdingTest`):** a profile with a frame turned about the tool axis and
  shifted keeps the fixture clearance above the vise jaws, as in the setup's own frame.

## Known limitations

- **No tilted work plane** (G68.2 / dynamic work offsets): positions are computed by OxCAM.
  With the setup's work offset they rely on the part's place relative to the table centre in the
  project; with an offset of its own, the offset must be probed at that index.
- **Rotary centre offsets** between the axes are not modelled (DL-0023).
- **Tapping on tilted heads** is refused; other cycles on tilted heads are posted as moves.
- **No choice of index solution:** the index follows DL-0019's rule (smallest tilt, then
  smallest rotation) even where the other solution would keep the machine clear. The machine
  simulation (DL-0037) reports such collisions: on the demo trunnion the plate's back face at
  A90 C0 brings the spindle head's column side over the table, where A−90 C180 would not
  (`plate32_machine_project.ocam` leaves that face out). Choosing automatically between
  solutions is the automatic-tilting item DL-0019 keeps out of scope; a per-operation choice by
  the user would not be.
- **Indexing moves** are not checked for collisions while the table turns (only the positions
  before and after, as for DL-0019 setups).
- **In-process stock** is carried between setups (DL-0012), its source setups' toolpaths moved
  into the setup's WCS, operations with their own orientation included.
