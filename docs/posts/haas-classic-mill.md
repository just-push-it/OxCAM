# The Haas-style mill post: `haas-classic-mill`

A built-in post for Haas-style mill controls. It is written for the classic control and suits
the Next Generation Control too. It handles 3-axis mills, a 4th axis (A) and 5-axis trunnion
tables (A/C or B/C), **without tool centre point control**.
Source: [data/posts/haas-classic-mill.js](../../data/posts/haas-classic-mill.js), on
[`FanucMill`](fanuc-base.md).

OxCAM is not affiliated with Haas Automation. "Haas" names the control the output is for.

## Sources

The post's syntax comes only from these controller manuals (syntax facts only, paraphrased;
[IP policy](../ip/IP-POLICY.md)). No other post processor was used.

- *Haas Mill Operator's Manual*, 96-8000 Rev Y, December 2009 (classic control): program
  format, the G- and M-code references, and the 4th/5th axis programming notes.
- *Haas Mill Programming*, Productivity Inc / Haas Factory Outlet, revised 05/25 (Next
  Generation Control): work offsets, program structure, G93.

## No tool centre point control

The control does not move the tool tip when the table turns. So for 4- and 5-axis toolpaths
OxCAM writes **machine positions** (the manifest's `multiAxis: "machine"`): X, Y, Z are where
the machine axes go once the table has turned. Each such program is valid only for:

- the tool lengths it was posted with (each tool's *gauge length* in the tool library), and
- the pivot of the machine it was posted for (its kinematics in the machine library).

The program says so in a comment before each simultaneous toolpath:

```
(NO TCP: VALID ONLY FOR T5 GAUGE LENGTH 50. MM)
(AND THE PIVOT OF THE MACHINE IT WAS POSTED FOR)
```

Move the tool in its holder or post for another machine, and you must post again. 3+2
(indexed) toolpaths are not affected: the table turns, then the tool moves along Z as usual.

A toolpath through the singular tool axis (vertical, on an A/C table) cannot be written as
machine positions: the table would turn half a revolution within a short move, far above the
machine's feed. Posting refuses with the move's feed. Tilt the tool axis away from vertical, or
use a post with tool centre point control.

## What the program looks like

```
%
O01000 (BRACKET SETUP 1)
G00 G17 G21 G40 G49 G80 G90 G94
(BRACKET - SETUP 1)
(POSTED WITH HAAS-CLASSIC-MILL 1.0.0 2026-10-25T14:03:07Z)
(FACE TOP)
T1 M06 (FACE MILL 50)
G00 G90 G40 G49
G0 G54 X-30. Y10.
S3000 M03
G43 H1 Z15.
M08
...
(POCKET)
M09
M05
M01
T2 M06 (END MILL 10)
G00 G90 G40 G49
...
G91 G28 Z0.
G28 Y0.
G90
M30
%
```

| What | How |
|---|---|
| Program number | `O` and five digits; O09000-O09999 (macro programs) warn |
| Comments | In parentheses, cut to the control's 80 characters |
| M codes | One per block |
| Work offsets | 1-6: G54-G59; 7-105: `G154 P1`-`P99` (P1-P20 are also G110-G129), declared in its manifest; the machine's *Work offsets* count limits them (16: G154 P1-P10); more: refused |
| Tool numbers | Above the machine's tool capacity: refused |
| Tool change | `T M06`, then the safe-start block `G00 G90 G40 G49` |
| Z return before the table turns (3+2) | `G91 G28 Z0.` / `G90` (or `G00 G53 G49 Z0.`); both cancel the tool length offset, so the approach applies `G43 H` again |
| Arcs | I/J/K, at most 180° per block, XY/ZX/YZ, helical |
| Dwell | `G4 P` in seconds with a decimal point (`P0.5`) |
| Canned cycles | G81, G82 (P seconds), G83 and G73 (Q, and P always stated: it carries over), G84, G85; G99 returns |
| Coolant | Flood M08/M09, air M83/M84, through-spindle M88/M89 (before the spindle speed); mist: see the options |
| Simultaneous 4/5-axis | G93 inverse time, F in every block; no rapids: every move of the toolpath is a G1 move, positioning moves taking the time set |
| Rotary brakes | Released (M11 4th axis, M13 5th) just before the first G93 move, engaged (M10, M12) after the toolpath |

## Options

Set on the setup page (*Post*) or with `--option name=value`.

| Option | Default | Effect |
|---|---|---|
| `units` | `mm` | `inch`: G20, 4 decimals for lengths, 2 for feeds |
| `lineNumbers` | off | N10, N20, ... |
| `optionalStop` | on | `M01` before each tool change after the first |
| `safeStartAfterToolChange` | on | `G00 G90 G40 G49` after each tool change |
| `homeWith` | `G28` | `G53`: Z returns with `G00 G53 G49 Z0.` (and Y with `G53 Y0.`) |
| `homeYAtEnd` | on | Return Y at the end of the program |
| `mistCoolant` | `refuse` | The control has no mist code: posting a mist operation is refused unless you choose `shower M07` or `flood M08` |
| `fifthAxisLetter` | `C` | `B`: an A/C trunnion's rotary platter is written as B (as some Haas-style trunnions name it). The back-plot check (`--verify`) reads C, so use it without `--verify` |
| `rotaryBrakes` | on | Release and engage the brakes around simultaneous toolpaths (off: the control releases and engages them each block, pausing) |
| `positioningTime` | 1 s | How long each positioning move of a simultaneous toolpath takes (F = 60 / time). The control limits G93 motion to its feed limits |

Per operation (*Post options* on the operation page):

| Option | Default | Effect |
|---|---|---|
| `smoothness` | control default | `rough`, `medium`, `finish`: `G187 P1`/`P2`/`P3` for the operation; back to the control's with `G187` |
| `cornerRounding` | 0 | `G187 E`: the largest corner rounding, in program units (0: the control's setting) |
