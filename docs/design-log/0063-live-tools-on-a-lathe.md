# DL-0063: Holes with a live tool

- **Status:** implemented
- **Module:** libs/strategies (`drive`), libs/cad (`axialEnvelope`, `turnsAboutZ`), libs/document (`liveDrive`,
  `generateToolpath`, `checkLiveHoles`, `simulateSetup`, `latheComparison`), libs/machine (`solveLiveLathe`),
  libs/sim (`cutHoles`), libs/post (`LiveTool`, `FanucLathe`, data/posts/haas-lathe.js, back-plot),
  apps/opencam-gui (`PropertyPanel`, `SimulationPanel`)
- **Patent gate:** low (citation only). Drilling at a C angle is ordinary lathe practice; code syntax from controller
  documentation.
- **Author(s):** OxCAM contributors

## Problem

A lathe's tools stood still: holes went on the spindle's axis (DL-0043), and anything else was refused. The maintainer
asked for off-centre holes with live tools, and pointed out that it is a choice per hole. On the axis one usually uses
the spindle, but may want a live tool, if the drill already sits in a live holder.

## Approach

**The choice is a setting.** Drill, Tap and Bore on a lathe have **Tool drive**: *Spindle: the part turns* (the default,
as every older project has it) or *Live tool: the tool turns*. Nothing picks it from where a hole is:

| Hole | Spindle | Live tool |
|---|---|---|
| On the axis | Drills it (DL-0043) | Drills it, part still, stock the same |
| Off the axis, or a tool orientation | **Refused, naming Tool drive** | Drills it |

**The tool's speed (ADR-0004).** A live tool's speed is not the part spindle's: it comes from the same preset as any
rotating tool's, cutting speed over diameter. It is shown on the operation's page as **Live tool speed**, next to the
cutting speed that sets it, and can be typed there. Above the machine's **Live tool max** posting is refused, naming the
value and where to change it. Nothing clamps it. The feed is the one shown as a plunge feed rate, as for any drill, and
posted per minute, since the part is not turning.

**Where the holes go.** As on a mill (`selectedHoles`): the selected faces, or else the part's holes of this tool's
diameter. With nothing selected and nothing to find, a live tool goes on the axis as the spindle's would, so a centre
hole by depth works either way.

- **In the face:** along the spindle's axis, by C and X.
- **Across the part:** a **Tool orientation** on the operation, the existing setting, shown for live tools. One radial
  direction per operation.

**What a turret can reach (`checkLiveHoles`).** Along +Z or square across. A tool coming from behind is refused (turn the
part round in another setup), as is one at an angle (no B axis). Across the part, a hole off the spindle's centre needs
Y, as in the table below. Without it the hole is refused, naming the machine and the distance.

| Hole | C | X | Y | Without a Y axis |
|---|---|---|---|---|
| In the face at (r, φ) | φ | r | 0 | Goes |
| Across, through the axis | The tool's direction | Depth | 0 | Goes |
| Across, off the axis by y | The tool's direction | Depth | y | **Refused** |

A machine with Y still reaches face holes by C: there is no second way to choose.

**Machine positions (`solveLiveLathe`).** The toolpath stays in the WCS, Cartesian, so the simulation can draw the tool
where it is and with its real axis. Solving turns each point into X, Y, Z and C, and cycles into plain moves.

**The program.** `NcOp::LiveTool` (NC program version 3): on, the post writes feed per minute, the C axis on and the
tool on; off, the reverse. Holes are plain G0/G1 moves, as canned drilling cycles differ between controls (G83 and G87
on Fanuc, G81 and G241 on Haas), and the back-plot checks them, C words included.

**The M-codes are not generic.** They are each machine builder's, so `fanuc-generic-lathe` has them as options, **empty**,
and refuses live tooling until they are set, rather than print one builder's codes as if they were Fanuc's. `haas-lathe`
is that post with Haas's codes as its starting options:

| Code | Means | From | Applies to |
|---|---|---|---|
| `M133 P<rpm>`, `M135` | Live tool forward at a speed; stop | Haas lathe M-code reference | Haas lathes with live tooling. Others differ: set them in the post's options |
| `M154`, `M155` | C axis engage; disengage | Haas lathe M-code reference | The same |
| `G98`, `G99` | Feed per minute; per revolution | Haas documentation, and Fanuc lathe manuals in G-code system A | Check yours: system B uses G94 and G95. Change in `FanucLathe` |
| `C` word | Spindle angle, degrees, 0 to 360 | Haas live tooling program examples | Check the direction on your machine |

Not run on a machine: see below.

**The simulation.** A live tool's feed moves cut holes beside the profile (DL-0061), with the tool's silhouette, and playback
grows them with the tool. The tool is drawn as a milling tool is, with its holder, at its real axis, and is checked against
the fixtures as on a mill (with the part standing still).

**The comparison.**

- A hole through material the part has is a **gouge**: sampled inside its wall against the part's mesh. It counts in the
  sum and draws red. A hole the part has as well reads nothing, so no false gouge.
- A part hollower than its revolved profile that the cuts have not taken yet (a hole the model has, not drilled yet) is
  **excess, in the total only**: nothing says where it lies.

**The part.** A section through the axis would show a cross hole as a gap, and turning would leave one. So a part that is
not the same all round (`turnsAboutZ`, a look at its faces) turns to the union of nine sections around the axis
(`axialEnvelope`), which also serves the display's comparison. A round part is the one section it always was, and the
look at its faces costs it no measurable time (DL-0061, Measurements): it moves the shape by a location, with no copy.

## Sources

Syntax facts only: [Haas lathe M-codes](https://www.haascnc.com/service/codes-settings.type=mcode.machine=lathe.value=M133.html),
[Haas G241](https://www.haascnc.com/service/codes-settings.type=gcode.machine=lathe.value=G241.html),
[Haas lathe code card](https://web.mae.ufl.edu/designlab/TA/Manuals/Haas%20Lathe%20G&M%20Code%20Card.pdf). The Haas site blocks
automated reading, so the codes above were taken from search summaries of those pages and from
[an indexing example](https://www.mfgrobots.com/Article/equipment/cnc/46168.html) and
[a live tooling guide](https://www.mfgrobots.com/Article/equipment/cnc/46171.html), not from a manual in hand.

## Correctness and tests

`LatheHoleTest`: "holes with a live tool" (where they go, the refusals by Tool drive, machine, Y and angle, tapping); "a
live tool on the spindle's axis" (both drives, the same stock); "a live tool's speed and the program" (the limit, the
generic post's refusal, Haas's codes, the back-plot with C); "a part with holes turns to its envelope"; "a live tool's holes
in the stock and against the part" (volumes, no gouge, excess before them, a wrong hole in the sum and in red, the mesh's
volume, part way, the cache). `FixturePostTest`: posts and verifies. `gui.lathe_live_*`, `gui.lathe_setup_turret`.

## Known limitations

- **Milling on a lathe:** DL-0066.
- **Taps on a live tool:** DL-0065.
- **A bolt circle across the part** takes one operation per direction.
- **Not on a machine.** No program from this has been run on a lathe. Check the first one in the air: above all the C
  direction and the live tool's rotation.
- **Spindle brake.** Nothing clamps the part while a hole is drilled: C holds it. Haas has M14 and M15 for a brake, and
  other builders their own. Not written, as no option holds them yet: add them to the post if a job needs it.
- **Sub-spindle:** the back of a part needs another setup.
- **Rapids against the stock:** DL-0064.
