# The Fanuc-style base posts: `oxcam:fanuc`

```js
import { FanucMill, FanucLathe } from "oxcam:fanuc";
```

`FanucMill` and `FanucLathe` write generic Fanuc-style G-code. The three `fanuc-generic-*`
built-in posts are these classes with nothing changed but their manifests; `haas-classic-mill`
subclasses `FanucMill` ([its source](../../data/posts/haas-classic-mill.js) is a worked example). A post for a Fanuc-style control
subclasses one, changes settings in its constructor and overrides the handlers that differ.
Source: [libs/post/js/fanuc.js](../../libs/post/js/fanuc.js).

## Settings

Set them in the constructor, after `super(context)`; `programBegin` reads them.

| Setting | `FanucMill` | `FanucLathe` |
|---|---|---|
| `units` | `"mm"`; `"in"` divides lengths and feeds by 25.4 (put G20 in `safeStart`) | |
| `lineNumbers` | `{ start: 10, step: 10, max: 99999 }`; `null`: none | |
| `decimals` | `{ linear: 3, feed: 0, dwell: 3, rotary: 3, feedPerRev: 3 }` | |
| `trailingPoint` | `true`: `X10.`; `false`: `X10` | |
| `arcFormat` | `"ijk"` (I/J/K from the arc start) or `"r"` (negative R over 180°) | |
| `maxArcSweep` | `180` [deg]: longer arcs are split (lowering already splits at the manifest's `arcs.maxSweep`) | |
| `dwellInMilliseconds` | `true`: `G4 P500`; `false`: `G4 X0.5` | |
| `commentStyle`, `upperComments` | `"parentheses"`, `true` | |
| `header` | `["%", "O{program_number} ({program_name})"]`: lines without numbers | |
| `safeStart` | `"G17 G21 G40 G49 G80 G90 G94"` | `"G18 G21 G40 G80 G99"` |
| `toolChangeBlocks` | `["T{tool} M06 ({tool_name})"]` | `["T{tool2}{tool2} ({tool_name})"]` |
| `toolLengthOffset` | `"G43 H{tool}"` | `""` (the T word applies it) |
| `cutterCompOffset` | `"D{tool}"` | `""` |
| `homeZBlocks` | `["G91 G28 Z0.", "G90"]`: before the table turns without a tool change | |
| `footer` | `["G91 G28 Z0.", "G90", "M30", "%"]`: `"%"` and `""` without numbers | `["G28 U0.", "G28 W0.", "M30", "%"]` |
| `coolantCodes` | `{ flood: "M08", mist: "M07", air: "M07", through_tool: "M88", off: "M09" }` | |
| `spindleCw`, `spindleStop` | `"M03"`, `"M05"` | |
| `spindleOrientBlock` | `"M19 R{angle}"` | Spindle orientation for a back bore played out as moves; `{angle}` in degrees. Controls differ in the word that takes the angle. |
| `tcpOnOffset`, `tcpOffBlock` | `"G43.4 H{tool}"`, `"G49"` | |
| `lathe`, `feedPerRev` | `false`, `false` | `true`, `true` |
| `probeSkip` | `"G31"`: the skip function the generic probing moves use ([Probing](#probing)) | |

Templates take `{program_number}`, `{program_name}`, `{tool}`, `{tool2}` (the tool number in
two digits) and `{tool_name}`; names are cleaned like comments.

## What each instruction becomes

| Handler | Output |
|---|---|
| `programBegin` | `header` lines, `safeStart` |
| `operationBegin` | `(OPERATION NAME)` |
| `toolChange` | `toolChangeBlocks`; afterwards every axis is written again |
| `workOffset` | `G54`-`G59` (1-6) or `G54.1 P<n-6>`, merged into the next block that moves |
| `spindle` | `S<rpm> M03` (`M04` with `reverse`), `M05`; lathe: `G97 S.. M03`, or `G50 S<max>` then `G96 S<m/min> M03` |
| `live_tool` | Lathe: `G98`, the options' C axis on and live tool on; off: live tool off, C axis off, `G99`. **The M-codes are each machine builder's:** the `liveToolOn` (with `{rpm}`), `liveToolOff`, `cAxisOn` and `cAxisOff` options, empty on `fanuc-generic-lathe`, where a live tool is refused until they are set. `haas-lathe` starts from Haas's `M133 P{rpm}`, `M135`, `M154`, `M155`. |
| `polar` | From the `polarOn` and `polarOff` options (blocks separated by `\|`), with `polarY` the letter for Y and `polarXDiameter` whether X is a diameter while it is on; moves there have no C word, and arcs have R. `haas-lathe` starts from `G17`, `G112`, `G113`, `G18`, `Y`, radius; `fanuc-generic-lathe` from `G1 G12.1`, `G13.1`, `C`, diameter. |
| `wrap` | From the `wrapOn` and `wrapOff` options (blocks separated by `\|`, with `{r}` the radius and `{d}` the diameter), with `wrapLetter` the word for the way around, and `wrapDegrees` whether it is in degrees or a length on the cylinder. `haas-lathe` starts from `G19`, `G107 Y0 C0 R{r}`, `G107`, `G18`, `Y`, a length; `fanuc-generic-lathe` from `G7.1 C{r}`, `G7.1 C0`, `C`, degrees. |
| `live_tap` | From the `liveTapAxial` and `liveTapRadial` options (`liveTapAxialLeft` and `liveTapRadialLeft` for a left-hand thread): blocks separated by `\|`, with `{rpm}`, `{lead}`, `{x}`, `{y}`, `{z}`, `{c}` and `{r}`. Empty on `fanuc-generic-lathe`, where a live tap is refused until they are set. `haas-lathe` starts from `S{rpm}`, `G18 G95 C{c} Z{z} R{r} F{lead}` and `G19 G195 X{x} F{lead}`. |
| `coolant` | the `coolantCodes` code |
| `toolLength` | `G0 G43 H<n> Z..` (`tcpOnOffset` when tool centre point control starts) |
| `rapid` | `G0 X Y Z [A B C]` |
| `linear` | `[G93/G94] G1 X Y Z [A B C] F`; with a thread lead `G32 X Z F<lead>`. Lathe: with the spindle stopped (a bar pull), `G98` first and feeds per minute, then `G99` when it turns again |
| `arc` | `G2`/`G3 X Y [Z] I J F` (`I K` in G18, `J K` in G19, or `R`); a plane change writes `G17`/`G18`/`G19` |
| `dwell` | `G4 P<ms>` or `G4 X<s>` |
| `cycle` | first hole: `G99 G81..G85/G73 X Y Z<bottom> R<retract> [Q<peck>] [P<ms>] F`; next holes of the same cycle: `X Y`. A left-hand tap (`tap_left`) is `G74` on a mill; on a lathe it is the `tapLeft` option, and refused while that is empty |
| `cycleCancel` | `G80` |
| `compensationOn` / `compensationOff` | `G41`/`G42` + `cutterCompOffset`, or `G40`, merged into the next move |
| `index` | `G0 A.. C..` (all rotary axes) |
| `homeZ` | `homeZBlocks` |
| `tcpOff` | `tcpOffBlock` |
| `probe` | the hooks of [Probing](#probing); refused for a cycle that sets a work offset unless `probeEnd` is overridden; a check by default `(PROBE <FEATURE>: CHECK WITHIN ..)`, every move as `G31 X Y Z F`, then `(PROBE RESULT NOT USED: ...)` and one warning |
| `programEnd` | `footer`; on `FanucLathe`, by the `partLoop` option: `end` (the footer), `repeat` (the footer up to its M30, then `M99`) or `count` (that, with `partCounter` counting up to `partCount` in a custom macro, `M99` until then, and `N9999` resetting it before `M30`) |
| `chuck` | Lathe: the main spindle's from the `chuckOpen` or `chuckClose` option, the sub-spindle's from `subChuckOpen` or `subChuckClose` (blocks separated by `\|`), empty on `fanuc-generic-lathe` and refused while empty; `haas-lathe` starts from `M11` and `M10` |
| `barFeeder` | Lathe: the `barFeederOn` or `barFeederOff` option, empty and refused while empty |
| `operationBegin` | Lathe: when the work changes spindle (the operation's `spindle`), the `subSpindleSelect` or `mainSpindleSelect` option first; empty and refused while empty |
| `subAxis` | Lathe: `G0` or `G98 G1` with the `z2Letter` option and Z2, `F` per minute; refused while the letter is empty |
| `spindleSync` | Lathe: the `syncOn`, `syncAngleOn` (with `{angle}`) or `syncOff` option; empty and refused while empty |
| `rapid`, `linear`, `arc`, `toolLength` on the sub-spindle | With the `subZ` option `mirror` (`fanuc-generic-lathe`'s start): Z and the arc centre's Z negated, arcs turned the other way, since the work's Z runs towards the main spindle; `as_is` for a control that mirrors it itself. Canned cycles there are refused while mirrored |

Words are written only when their value changes (X, Y, Z, A, B, C, F, the motion code).

## Helpers for subclasses

| Member | Does |
|---|---|
| `this.formats` | The number formats (`linear`, `feed`, `feedPerRev`, `dwell`, `rotary`, `inverseTime`, `lead`, `whole`), made in `programBegin` |
| `this.axisWords`, `this.rotaryWords` | The X/Y/Z and A/B/C words |
| `this.position` | Where the tool is, in mm (program coordinates) |
| `this.motion` | The motion code in effect (`"G0"`, `"G1"`, ...; `null` when unknown) |
| `this.pending` | Words waiting for the next block that moves (work offset, plane, G41/G42/G40) |
| `toUnit(mm)`, `length(mm)` | A length in the output unit; formatted |
| `expand(template, tool, toolName)` | Fills a template |
| `motionWord(g)` | `g` when it is not the motion in effect, else `""` |
| `axes(target, forceXY, words)` | Pushes the X/Y/Z words of a target onto `words` |
| `rotary(i, force, words)` | Pushes the A/B/C words of a move onto `words` |
| `feedWord(feed)` | `"F.."` when the feed changes (per revolution when `feedPerRev` and the spindle turns) |
| `takePending()`, `flushPending()` | Takes the pending words; writes them as a block |
| `selectPlane(plane)` | Queues `G17`/`G18`/`G19` when the plane changes |

## Overriding

Call `super` for what stays the same:

```js
toolChange(i) {
  out.block("G91 G28 Z0.");      // retract before the change
  out.block("G90");
  super.toolChange(i);
  // Preload the next tool (tools are listed in order of first use).
  const k = this.program.tools.findIndex((t) => t.number === i.tool);
  const next = this.program.tools[k + 1];
  if (next) out.block(`T${next.number}`);
}
```

Override `operationBegin` to write more about an operation (its tool, comment, options), and
`programBegin` / `programEnd` for headers and footers that need logic. `super.programBegin`
makes the formats and the state and writes the header and safe start: write lines that must
come first before calling it, the rest after.

## Probing

A probing cycle (DL-0090, [ADR-0008](../adr/0008-probing-in-the-post-api.md)) is one `probe`
instruction ([NC program](nc-program.md#probing)). `FanucMill.probe` writes it through hooks:

| Hook | Called | Writes by default |
|---|---|---|
| `probeOn(cycle)` | before the first cycle after a tool change | nothing |
| `probeBegin(cycle)` | each cycle | `(PROBE BORE: SET G55 XY)`, `(PROBE BOSS: CHECK WITHIN 0.05)` |
| `probePosition(move, cycle)` | each protected positioning move | `G31 X Y Z F<fast_feed>` |
| `probeMeasure(move, cycle)` | each measuring move | `G31 X Y Z F<measure_feed>` to `move.x/y/z`, past the surface by the overtravel |
| `probeEnd(cycle)` | each cycle | checks: `(PROBE RESULT NOT USED: ...)`, and one warning per program |
| `probeOff()` | before the next tool change, and at the program's end | nothing |

Plain G-code cannot use what the probe measured. A cycle that sets a work offset is therefore
**refused** unless the post overrides `probeEnd` (the base compares `this.probeEnd` with its own):
a program that probes and then cuts on the old offset is worse than none. The error names the
operation. A feature check changes nothing the program uses, so the generic output moves the
probe and touches, and says in the program and as a warning that the check is not made. After a
cycle every axis, the motion code and the feed are written again (a skip move or a macro may stop
anywhere).

A post for a machine with a probe overrides the hooks to call the probe's macro programs. Their
program numbers and argument letters are in the probe maker's manual; OxCAM ships none, and the
numbers below are made up:

```js
import { FanucMill } from "oxcam:fanuc";
import { out } from "oxcam:post";

export default class extends FanucMill {
  probeOn(cycle) { out.block("G65 P8000"); }            // the probe maker's "probe on"
  probeOff() { out.block("G65 P8001"); }
  probePosition(move, cycle) {                          // a protected move
    out.block(`G65 P8010 X${this.length(move.x)} Y${this.length(move.y)} Z${this.length(move.z)}`);
  }
  probeMeasure(move, cycle) {}                          // the cycle's macro below measures
  probeEnd(cycle) {                                     // measure, then set the offset
    if (cycle.feature === "bore") {
      out.block(`G65 P8020 D${this.length(cycle.size)} S${cycle.work_offset}`);
    }
  }
}
```

`cycle.work_offset` is 0 and `cycle.check` true for a feature check, with `cycle.tolerance`.
`this.workOffsetCode(n)` gives the code of work offset `n` as the manifest declares it.
