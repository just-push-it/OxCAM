# DL-0023: Multi-axis kinematics, simultaneous solve and posting

- **Status:** implemented (phase 9)
- **Module:** libs/toolpath (tool axis per move), libs/machine (`Kinematics.hpp`: kinematic
  classes, forward and inverse kinematics, rotary planning, multi-axis `solve`), libs/post
  (rotary words, tool centre point control, inverse-time feeds, back-plot), libs/document
  (`lowerSetup`), Python post base class
- **Patent gate:** low (citation only). Closed-form inverse kinematics of serial rotary axes,
  tool centre point control and inverse-time feeds are textbook and controller-manual material.
  The tool axis is always given by the toolpath; nothing here chooses or changes it.
- **Author(s):** OxCAM contributors

## Toolpath IR

Every move already carried `Move::axis`, the tool axis at its end (a unit vector from the tip
towards the spindle, in the WCS). Phase 9 makes it real:

- toolpaths whose axis is not +Z everywhere are **multi-axis** (`toolpath::isMultiAxis`);
- along a move the axis turns from the previous move's axis to its own (`slerp`); the start
  takes the first move's axis (`axisBefore`);
- multi-axis toolpaths use straight moves (solve splits arcs);
- the toolpath JSON writes `"a": [x, y, z]` on moves whose axis is not +Z, and toolpath
  comparison checks it.

## Kinematic classes

| `kinematics` | Rotary axes | Table rotation T | Head rotation H |
|---|---|---|---|
| `mill4_a` | A (table about X) | Rx(A) | I |
| `mill5_table_ac` | A tilt, C table | Rx(A) Rz(C) | I |
| `mill5_table_bc` | B tilt, C table | Ry(B) Rz(C) | I |
| `mill5_head_table_bc` | B head, C table | Rz(C) | Ry(B) |
| `mill5_head_bc` | C head, B head (fork) | I | Rz(C) Ry(B) |

- **Frames.** The table frame is the project frame, with its origin at the table centre (where
  the rotary axes meet); the part is fixed in it. The machine frame has the same origin with the
  rotary axes at 0 and the spindle along +Z. Positive angles turn right-handed about the machine
  axis. (Since DL-0037 a setup's placement puts its WCS where the work sits relative to the table
  centre; setups without one keep the project frame there.)
- **Forward kinematics.** The tool axis in the table frame is u = Tᵀ H ez.
- **Machine positions without tool centre point control.** For a tip p (table frame):
  X = T p + (L_p + L)(H ez − ez), with L_p the machine's `pivot_length` (head rotation centre to
  spindle gauge line) and L the tool's gauge length (visible length plus holder). This is the
  position the control's linear axes take when the tool length offset applies along machine Z.
- **Built-in machines:** `generic-4axis-vmc` (A ±9999°) and `generic-5axis-headtable-bc`
  (B ±110°, C continuous, pivot 150 mm) join the A/C trunnion, which now also does
  simultaneous motion. Machine files may give `pivot_length` and `components` (DL-0025).

## Inverse kinematics

For each class the tool axis fixes the rotations in closed form, with two branches:

- **4th axis:** the axis must be perpendicular to X; A = atan2(u_y, u_z).
- **Table-table A/C:** C = atan2(u_x, u_y) brings u into the YZ plane, then A = atan2(w_y, w_z)
  with w = Rz(C) u (as DL-0019). B/C: C = atan2(−u_y, u_x), B = atan2(−w_x, w_z).
- **Head-table B/C:** the table turns u into the XZ plane with positive x (C = atan2(−u_y, u_x)),
  then the head tilts B = atan2(w_x, w_z).
- **Head-head:** C = atan2(u_y, u_x), B = atan2(√(u_x² + u_y²), u_z).
- The second branch is C + π with the tilt negated. Solutions outside the tilt limits are
  dropped. The rotating axis is unwrapped by whole turns to the value nearest the previous
  position within its limits (`axisSolutions`).
- **Singular position:** with the tool along the rotating axis (u = ±Z for the 5-axis classes)
  the rotating axis does not change the tool axis and keeps its previous value.

## Rotary planning

`planRotary` picks one solution per move end so the axes move continuously: starting from each
solution of the first axis, it takes at every move the solution nearest (largest single-axis
change) to the previous one. The start with the fewest unreachable moves, then the least total
rotary travel, wins. Unreachable axes are reported by move. When a continuous axis runs into its
limit, the nearest remaining solution is the other branch: the table flips (A changes sign, C
half a turn), which shows as a large step.

## Solve

`solve(machine, path, wcsInTable, options)`:

- Toolpaths along WCS +Z use the 3-axis / 3+2 solve unchanged.
- Multi-axis toolpaths need a machine with rotary axes; canned cycles are refused.
- **With tool centre point control** (`tcp`, the default): positions are the tool tip in the
  WCS with its axes turned to the table's (R · p), the work offset at the WCS origin. The control
  keeps the tip on the straight line while the rotary axes move.
- **Without it:** positions are machine X Y Z relative to the table centre (machines that turn
  the part) or the WCS origin (head-head). The control interpolates every axis linearly, so the
  tip leaves the straight line: each move is split in half (tip midpoint, axis slerp, rotary
  solution nearest both ends) until the linear interpolation of the machine axes keeps the tip
  within `tolerance` (0.01 mm) at the midpoint, at most 12 levels deep.
- `SolveResult::rotary` holds the rotary axis letters, the start position, one position per
  output move, and each feed move's duration along the tool tip (CL length / feed). Each output
  move's axis is the tool axis in the table frame.
- Feed moves whose rotary axes swing by more than 30° (near a singular position, or across a
  normal discontinuity) are counted and reported in one warning.
- Linear axis travel and feed / spindle limits are checked as in the 3-axis solve (errors, never
  a changed value).

## Posting

- **Lowering.** A multi-axis job starts with an `Index` instruction at its start position (after
  a `HomeZ` when the tool is already in the spindle and the rotaries move). Rapid and linear
  instructions carry `axes` and the rotary positions. With tool centre point control the tool
  length instruction carries `tcp` and a `TcpOff` instruction ends the job; the next job applies
  the tool length again. Without it, linear moves that turn a rotary axis carry an inverse-time
  feed 1 / t [1/min].
- **3-axis jobs on rotary machines** index the rotary axes to 0 (machines that cannot do 3+2)
  or to their 3+2 position, so a job after a multi-axis one turns the table back first.
- **Formatting.** Rotary words follow the linear axis words, modal, at `decimals.rotary`.
  Continuous axes are written unwrapped (e.g. A540.). `G93` starts inverse time and F is written
  in every block; `G94` returns to feed per minute. The tool length block uses `multi_axis.tcp_on`
  (`G43.4 H{tool}`) and `multi_axis.tcp_off` (`G49`).
- **Posts.** The generic Fanuc-style mill post has `multi_axis.tcp = true`. The new built-in
  `fanuc-generic-mill-4axis` has `tcp = false` (machine positions, inverse time) and is the
  default for 4-axis machines (`document::defaultPostFor`).
- **Back-plot and verification.** With `BackplotOptions::machine`, A/B/C words give each move's
  tool axis through the machine's forward kinematics; G93 feeds become feeds along the
  programmed positions (the solved path carries the same for moves that turn a rotary axis).
  `compareBackplot` compares tool axes (to 1e-4) for multi-axis toolpaths, and allows for
  position rounding in the feeds of short inverse-time moves. The CLI's `--verify` passes the
  machine.
- **Post API.** The NC program JSON gains `axes`, `a`, `b`, `c` on rapid/linear instructions,
  `inverse_time` on linear ones, `tcp` on `tool_length`, and the `tcp_off` instruction. The
  Python `Post` base class writes rotary words, G93/G94 and G43.4/G49;
  `Document.nc_program(tcp=False)` lowers for machine positions.

## Sources

- J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 3rd ed., Pearson, 2005.
  Forward and inverse kinematics of serial rotary joints.
- R.-S. Lee, C.-H. She, "Developing a postprocessor for three types of five-axis machine
  tools", *Int. J. Advanced Manufacturing Technology* 13, 1997. Table-table, head-table and
  head-head configurations.
- Controller manuals (Fanuc-style syntax facts only): G43.4 tool centre point control, G93
  inverse-time feed, G49.

## Correctness and tests

- **IK round trips (`tests/unit/machine/KinematicsTest.cpp`)** for every kinematic class: every
  solution reproduces the axis to 1e-9; random rotary positions are found again; machine
  positions invert exactly. Conventions: A = t for a radial axis at angle t; head B = 90° tilts
  towards +X and moves the tip by (pivot + length)(d − Z); singular positions keep C; tilt limits
  make −Z unreachable on the trunnion.
- **Continuity:** an axis circling at 30° for two turns gives 5° steps and C through 720°; with C
  limited to ±200° the plan flips instead of failing.
- **Solve:** machine positions of a wrapped groove stay at Y = 0 with Z the radius; moves are
  split to the tolerance; durations sum to the CL length / feed; TCP output equals the CL.
- **Post (`tests/unit/post/MultiAxisPostTest.cpp`):** a 4-axis program with A540., G93 and G43;
  a later 3-axis job homes Z and returns A to 0; a 5-axis TCP program with G43.4/G49 and A/C.
  Both back-plot and verify within 2 µm including tool axes; a tampered axis is caught; the NC
  JSON round-trips.
- **CLI:** `cli.project_post_4axis` and `cli.project_post_5axis` post the sample projects with
  `--verify`; the Python test posts both with Python posts.

## Known limitations

- **Rotary centre offsets:** the rotary axes are taken to meet at one point (the table centre);
  machines with offsets between the A and C axes need tool centre point control.
- **Rapids without TCP:** rapid moves are not split; the machine interpolates them linearly in
  its own axes.
- **Tilted-plane cycles** (G68.2) and 3+2 on head machines are not posted; head machines index
  to zero for 3-axis jobs.
- **Rotary rollover:** continuous axes are written unwrapped; controls that wrap at 360° need a
  post option (not yet).
