# DL-0019: Indexed (3+2) setups on table-table 5-axis machines

- **Status:** implemented (phase 7)
- **Module:** libs/machine (`Kinematics::Mill5TableAC/BC`, `indexAngles`, `solve` with a WCS),
  libs/post (`NcOp::Index`, `NcOp::HomeZ`), libs/document, apps (setup panel)
- **Patent gate:** low (citation only). The orientation is chosen by the user. Automatic
  tilting is a medium-risk item and is **not** implemented.
- **Author(s):** OxCAM contributors

## Problem

A setup can have a tilted WCS, for example machining the front face of a part. On a 5-axis
machine, OxCAM should turn the table so the WCS Z axis points up the spindle, then run the
ordinary 3-axis toolpath. On a 3-axis machine a tilted setup means re-fixturing the part, and
the program is unchanged.

## Approach

- **Machines.** The machine definition gains `mill5_table_ac` (trunnion: A tilts about X,
  C rotates the table about Z) and `mill5_table_bc`. Rotary limits are in degrees. A built-in
  generic A/C trunnion is provided (`generic-5axis-trunnion-ac`).
- **Table frame.** The project frame is the machine table frame with every rotary axis at 0. The
  setup's WCS axes in that frame are the columns of `wcs.toParent().linear()`.
- **Index angles.** For a direction v (the WCS Z), solve R(A, C) · v = +Z with R = Rx(A) · Rz(C):
  - C = atan2(v_x, v_y) turns v into the YZ plane;
  - A = atan2(w_y, w_z) then stands it upright.
  - The other branch (C + π, −A) and wrapped C values are also tried. The solutions within the
    limits are kept, preferring the smallest tilt, then the smallest C.
  - B/C machines use Ry(B) · Rz(C) in the same way.
- **Program coordinates.** After indexing, the WCS X axis lies in the machine XY plane at angle
  ψ = atan2((R·x_w)_y, (R·x_w)_x). Solve rotates the toolpath by ψ about Z, so X and Y follow the
  machine axes. The work offset (one per setup) is measured at the WCS origin in the indexed
  position, which is common 3+2 practice without tilted-plane cycles.
- **Posting.**
  - Lowering emits an `Index` instruction (`G0 A.. C..`) after the work offset whenever the index
    changes.
  - When the tool is already in the spindle (no tool change in between), a `HomeZ` comes first
    (`G91 G28 Z0.` then `G90`, configurable as `home_z`) to clear the part before the table turns.
  - Rotary words are formatted to `decimals.rotary`.
  - The NC program JSON carries `index` and `home_z` instructions (DL-0015).
  - The back-plotter ignores rotary words, so linear motion is still verified.
- **GUI.** The setup panel has a machine selector and an "Align to picked face" button: WCS Z
  along the picked planar face's outward normal (`cad::facePlane`), origin at its centroid, X as
  close as possible to the project X axis. The existing six tool-axis presets cover the
  face-aligned cases.

## Sources

- R.-S. Lee, C.-H. She, *Developing a postprocessor for three types of five-axis machine tools*,
  Int. J. Advanced Manufacturing Technology 13(9), 1997. Covers inverse kinematics of
  table-table, head-head and head-table machines.
- Controller manuals: G28 reference return and rotary axis words (syntax facts only).

## Correctness and tests

- **Machine:**
  - Up gives (0, 0).
  - Several directions are turned onto +Z.
  - The front face (−Y) needs |A| = 90°.
  - Straight down is beyond the A limits.
  - A 3-axis machine cannot index.
  - Solve preserves heights and distances and aligns program X with the indexed WCS X.
- **Post:**
  - Two jobs with the same tool and different indexes produce `G0 G54 A0. C0.`, then a Z home
    before `G0 G55 A90. C-90.`.
  - The back-plot still matches.
  - The NC JSON round-trips.
- **Document:** a front-face setup of the block on the trunnion machine posts `G55` and
  `A-90. C0.`, and back-plot verification passes.

## Known limitations

- **No dynamic work offset:** there is no tilted work-plane mode (TCPM / G68.2 / CYCLE800).
  Each indexed setup needs its own probed work offset.
- **Head machines:** head-head and head-table kinematics are phase 9.
- **Collision checking:** the machine's moving parts are not checked against each other during
  indexing (phase 9).
