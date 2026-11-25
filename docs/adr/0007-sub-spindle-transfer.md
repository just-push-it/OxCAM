# ADR-0007: A sub-spindle, and handing the part to it

- **Status:** accepted (the maintainer, 2026-11-09: "Approve as written")
- **Date:** 2026-11-09

## Context

Part transfer between spindles is a phase 10 item (multi-channel, mill-turn). Some of it exists:

- **The machine** (DL-0067): a lathe's layout can have a `sub_spindle` (its nose, housing, chuck, grip,
  and `work_z2`, Z2 while it cuts) on the axes Z2 and C2 of the machine's tree. The built-in
  `generic-lathe-sub` has one. A setup's *Spindle* choice puts its work on C2, with its WCS's Z towards
  the main spindle.
- **The transfer** is simulated, but implicitly: a sub-spindle setup whose stock comes from a main-spindle
  setup on the same machine has Z2 come in to where the part lines up in both setups, then go back to
  `work_z2`. There is no synchronisation, no parting off while both spindles hold the part, and the stub
  left on the main spindle is not there. The two setups simulate separately.
- **Posting refuses it:** the codes that select the sub-spindle, synchronise the spindles and open its
  chuck are each machine builder's, and no post has them.
- **Bar pull** (DL-0091, on the same branch as this ADR) adds what a transfer needs too: a toolpath
  move that is a machine function rather than motion (chuck open and close, the bar feeder), an NC
  program instruction `chuck`, and post options for the builder's codes, empty on the generic post and
  refused while empty (as the live tool's codes are, DL-0063).

On a real single-turret lathe with a sub-spindle, a transfer goes: the sub-spindle's chuck opens; Z2
comes up to the part at rapid, then feeds the last stretch over it; its chuck closes; the spindles turn
together, synchronised in speed (or in speed and angle, for a part that is not round), or both stand
still; the parting tool cuts the part off with both spindles holding it, or, when the part is already
free, the main chuck opens; Z2 withdraws with the part, and the back of the part is worked on the
sub-spindle.

No machine in `data/machines` other than `generic-lathe-sub` has a sub-spindle. The maintainer's Wasino
LJ-63M does not, so the work is tested on the generic one.

## Decision

### 1. The machine

- A lathe has a sub-spindle when its layout has one: the kinematics stay `lathe2`. One turret serves
  both spindles; its tools work the sub-spindle's part from the disc's other face (DL-0067).
- The sub-spindle gains, in the machine file (version 16) and the machine library:
  - its **speed range** (`min_rpm`, `max_rpm`), used as the main spindle's is: a speed outside it is
    refused, never changed (ADR-0004);
  - **Z2's travel and rapid rate**, as an axis `Z2` in the machine's `axes`. A transfer position
    outside the travel is refused, naming it.
- **Multi-channel is not in this ADR.** A second turret has its own tools, its own program and wait
  codes between the two programs. The axis tree already takes X2, Z2 and T2 (DL-0067); tool numbers
  per turret, two programs and their synchronisation are a later ADR.

### 2. The transfer is an operation

`spindle_transfer`, *Sub-spindle transfer*, is an operation of the **main-spindle setup**,
after the operations that work the front. It has no tool. Its parameters:

| Parameter | What it is |
|---|---|
| Grip | How far the sub-spindle's jaws go over the part from its front face. From the machine's sub-spindle grip while not set. Z2 at the pick-off follows from it and is shown beside it. |
| Clearance | Where Z2's rapid stops, in front of the part. |
| Z2 feed | The feed over the last stretch onto the part. The user's own (ADR-0004): no default, and 0 refuses, naming it. |
| Spindles | *Synchronised*: both turn at the speed below, the sub-spindle following the main. *Synchronised with angle*: the same, the sub-spindle at a set angle to the main. *Stopped*: both stand still. |
| Speed | With either synchronised choice: the spindles' speed while the part is taken, in rpm. The user's own: 0 refuses. |
| Then | *Part off*: a part-off operation follows in this setup and runs with both spindles holding the part, synchronised as above; the main chuck stays closed. *Open the main chuck*: the part is already free (a blank, not a bar). |
| Dwell | After the sub-spindle's chuck closes [s]. |

What it does, in order: the sub-spindle's chuck opens; Z2 rapids to the clearance; feeds to the pick-off;
the chuck closes; the dwell; synchronisation on, or the spindles stop. With *Open the main chuck*, the
main chuck opens and Z2 withdraws to `work_z2`. With *Part off*, the part-off that follows cuts it free
and Z2 withdraws after it.

**Feeds and speeds** (ADR-0004): the Z2 feed and the speed are values the user sets in the operation and
sees there. The part-off keeps its own feeds; while synchronised, its spindle speed is its own and the
sub-spindle follows. Nothing is scaled or derived. Operation parameters gain a *feed* kind (mm/min in
the operation, mm/s in code) for the Z2 feed, since there is no tool, and so no cutting preset, to
hold it.

### 3. The second setup's stock comes from the transfer

- The sub-spindle setup's stock is **From earlier setup** (DL-0044), naming the main-spindle setup.
  When that setup has a transfer, the sub-spindle setup starts from **the piece the sub-spindle holds**:
  the main setup's stock after its last operation (the part-off, under grip), the piece with the part
  in it (as DL-0044 takes it now), held by the transfer's grip. The stub stays on the main spindle and
  is the main setup's final stock: the next cycle's bar, with a bar pull (DL-0091).
- **Where it sits** is where the transfer took it: the sub-spindle's work mount takes the transfer's
  grip in place of the machine's. The setups' WCSs must still line up in the project, Z turned round
  (DL-0067's check); a part that does not line up is refused for a transfer, where it is only a warning
  today.
- Without a transfer, a sub-spindle setup's work is loaded by hand, as today: it simulates on the
  sub-spindle and posts as a program of its own, with no transfer in it.
- **Simulation:** the transfer is simulated as an operation: Z2's moves, checked against the turret, the
  tools and the main chuck as today, and the part-off's separation, after which the part goes with Z2.
  The sub-spindle setup starts with the part in the sub chuck where the transfer left it, and Z2 goes to
  `work_z2` (DL-0067's step).

### 4. Posting

- **One program for the pair.** A machine runs the transfer between the two setups' work as one cycle,
  so posting the main-spindle setup with a transfer posts it with the sub-spindle setup that takes the
  part, in this order: the main setup's operations, the transfer, the part-off, the sub-spindle setup's
  operations. Posting the sub-spindle setup alone is refused, naming the main setup.
- **NC program version 7** adds:

  | Instruction | Fields | Meaning |
  |---|---|---|
  | `chuck` | `spindle` (`"main"`, `"sub"`), `open` | From bar pull; `"sub"` added here. |
  | `sub_axis` | `z2`, `feed` (`null`: rapid) | Z2 moves to `z2`, in the machine's Z2 (the layout's frame, Z2 0 at the sub-spindle's home). |
  | `spindle_sync` | `on`, `angle` (degrees, or `null`) | Synchronisation on (in speed, or speed and angle) or off. |
  | `spindle` | `spindle`: `"main"` or `"sub"` | Which spindle the speed is for: the sub-spindle setup's operations say `"sub"`. |

  A post declares `subSpindle: true` in its manifest to take these. Posting a transfer with a post that
  does not is refused, naming the post.
- **The codes are the builder's.** The `oxcam:fanuc` lathe base writes them from options, empty on
  `fanuc-generic-lathe` and refused while empty: the sub chuck's open and close, synchronisation on (with
  `{angle}`) and off, the sub-spindle's speed and direction (`{rpm}`), and the letter Z2 is written with.
  The work offset of the sub-spindle setup is its own, as for any setup.

### 5. Order of work

1. The machine: the sub-spindle's speed range and Z2's travel; file version 16; the machine library.
2. The operation, its parameters and the feed kind; generating its moves; its checks.
3. The simulation: the transfer's moves, the part-off under grip, the sub-spindle setup's stock and
   mount from the transfer.
4. Lowering the pair as one program; NC program version 7; the Fanuc base's options; a test post that
   gives codes; golden programs on `generic-lathe-sub`.
5. The GUI: the operation's page, the machine library's fields, the post options.

## Consequences

- A sub-spindle job posts and simulates as the machine runs it, with every code the machine's builder
  defines coming from the user's post options.
- **One part at a time:** the program does the front, the transfer and the back in turn. A single-turret
  machine that works the next part's front while the sub-spindle holds this one (interleaved operations
  of two setups) is not possible here; nor is a second turret working at the same time (multi-channel).
- Ejecting the finished part from the sub-spindle (its chuck opening over a part catcher) is the post's
  for now: a later operation can make it explicit, as the transfer is.
- Machine files and NC programs gain versions; older ones are read as before.
- The implicit transfer of DL-0067 goes: a sub-spindle setup is taken by a transfer operation or
  loaded by hand.
