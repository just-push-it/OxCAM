# The NC program (version 7)

What every post receives: a setup's toolpaths, solved against the machine and sequenced into
typed instructions. It contains no G-code. In a post it is `this.program` (without its
instructions, which come one by one to the handlers); as a file it is what
`oxcam-cli project post --ir program.json` writes, and in Python `Document.nc_program()`.

Source: `post::NcProgram` ([libs/post/include/ocam/post/Post.hpp](../../libs/post/include/ocam/post/Post.hpp)),
lowered by `post::lower` and `document::lowerSetup`; the JSON fields are written once in
`libs/post/src/NcFields.hpp`, so the file and the post see the same thing.

## Units and coordinates

- Lengths mm, feeds mm/min, spindle rev/min, time s, angles degrees, surface speed m/min,
  inverse-time feed 1/min. Convert in the post (the Fanuc base does for `units: "in"`).
- `x`, `y`, `z` are absolute, in the work offset. **`null` means "this axis does not change"**,
  not zero.
- 3-axis and 3+2: the tool tip in the work coordinate system.
- Multi-axis with tool centre point control (`multiAxis: "tcp"`): the tool tip; rotary positions
  in `a`/`b`/`c`.
- Multi-axis without it (`"machine"`): machine axis positions.
- Lathes: `x` is the **radius**.
- Values are not rounded (a spindle speed may be `4774.648...`): rounding is the post's job.

## The program

| Field | Content |
|---|---|
| `format`, `version` | `"oxcam-ncprogram"`, 5 (files of versions 1 to 4 are still read) |
| `units` | `{ length: "mm", feed: "mm/min", spindle: "rev/min", time: "s", angle: "deg" }` |
| `name`, `number` | "*project* *setup*"; 1000 + the setup's index |
| `project`, `setup` | Names |
| `time` | When it was made, ISO 8601 UTC ("2026-10-25T14:03:07Z") |
| `tools` | The tools, in order of first use (below) |
| `operations` | The operations, in program order (below) |
| `machine` | The machine (below) |
| `warnings` | Messages from lowering (show them) |
| `post` | In a post only: its manifest's `id`, `name`, `version`, and `work_offsets` (its `workOffsets` declaration, defaults filled in) |
| `options` | In a post only: the program option values |
| `peek(k)` | In a post only: the k-th instruction after the one being handled (`peek()` = the next; in `programBegin` the k-th of the program); `null` past the end and in `programEnd` |

### Tools

`number`, `name`, `comment`, `type` (`"flat_end_mill"`, `"drill"`, `"tap"`, `"probe"`, ...), `diameter`,
`corner_radius`, `flutes`, `thread_pitch`, `gauge_length` (tool and holder, tip to gauge line);
turning tools also `insert`: `{ shape, nose_radius, orientation }`.

### Operations

| Field | Content |
|---|---|
| `id` | The operation's id in the project |
| `name`, `comment` | As in the project |
| `strategy` | Operation type id: `"facing"`, `"pocket2d"`, `"drill"`, ... |
| `tool` | Tool number |
| `coolant` | `"off"`, `"flood"`, `"mist"`, `"air"`, `"through_tool"` |
| `work_offset` | 1 = G54 ... 6 = G59, 7 and up extended |
| `orientation` | `"3axis"`, `"indexed"` (with `index`: `{ a, b, c }` degrees) or `"simultaneous"` |
| `bounds` | `{ min: [x, y, z], max: [x, y, z] }` of its motion (arc extremes and hole bottoms included) |
| `spindle` | Version 7: `"main"`, or `"sub"` for work on the sub-spindle (ADR-0007); its positions are in its setup's WCS, whose Z runs towards the main spindle |
| `options` | The values of the post's per-operation options |

### Machine

`id`, `name`, `kinematics` (`"mill3"`, `"mill4_a"`, `"mill5_table_ac"`, ...), `axes`
(`[{ name, min, max }]`, mm and degrees), `rotary_axes` (`"A"`, `"AC"`, `"BC"` or `""`),
`spindle` (`{ min, max }` rev/min), `max_feed` and `rapid` (mm/min), `tool_capacity`,
`work_offsets`.

## Instructions

Each has `op`; the handler is its camelCase name.

| `op` (handler) | Fields | Meaning |
|---|---|---|
| `operation_begin` (`operationBegin`) | `operation` (index) | An operation starts. The handler receives the operation itself. |
| `operation_end` (`operationEnd`) | `operation` | It ends (after its cycle cancel, compensation off, TCP off). |
| `comment` | `text` | A comment (raw text). |
| `tool_change` (`toolChange`) | `tool`, `tool_name` | Load a tool. Coolant off and spindle stop come before it. |
| `work_offset` (`workOffset`) | `work_offset` | After each tool change and change of offset. |
| `spindle` | `rpm`; lathe constant surface speed: `surface_speed`, `max_rpm`; `reverse`; `spindle: "sub"` for the sub-spindle's (version 7) | Start, change, or stop (`rpm` 0). Clockwise, or with `reverse: true` counter-clockwise, for a left-hand tap (version 4). |
| `spindle_orient` (`spindleOrient`) | `angle` | Version 5: the spindle stops and holds at `angle` (degrees), for a back-boring bar passing its hole off centre (DL-0083). The next `spindle` starts it again. Only programs with a back bore the post does not take as a cycle have it. |
| `live_tool` (`liveTool`) | `rpm`, `tapping` | Version 3, lathes: a driven tool on at `rpm`, the spindle stopped and a C axis, or all that off (`rpm` 0). While on, moves give `axes` "C" and `c`, `x` is the radius or the depth, and `y` may be set. Version 4: with `tapping`, for a tap whose own cycle starts the tool; with `plane` ("XY" or "YZ"), for milling with C held still. |
| `polar` (`polar`) | `on`; on: `x`, `y`, `z` | Version 4, lathes: polar interpolation on or off, around each run of feed moves. While on, moves' `x` and `y` are in the part's frame, with the axis at 0, for the control to turn into X and C, and rapids do not happen. Off, moves are as with any `live_tool`. |
| `wrap` (`wrap`) | `on`; on: `radius` | Version 4, lathes: cylindrical interpolation on or off, around each run of feed moves that turns C, on the deepest radius of the run. While on, moves' `x` is the radius, `z` as ever, and `c` is degrees, going on past a turn. The toolpath's feed is the one on that cylinder. |
| `live_tap` (`liveTap`) | `x`, `y`, `z`, `c`, `rpm`, `pitch`, `r_plane`, `radial`, `left_hand` | Version 4, lathes: a rigid tap with a driven tool, from where it stands to here and back, along X if `radial`, else along Z. `r_plane` is where it starts along that direction. `cycle_cancel` follows. |
| `coolant` | `coolant` | Coolant mode. |
| `tool_length` (`toolLength`) | `tool`, `x`, `y`, `z`; `tcp: true` when TCP starts | Rapid to the start height with tool length compensation. |
| `rapid` | `x`, `y`, `z`; multi-axis `axes`, `a`, `b`, `c` | Rapid move. |
| `linear` | `x`, `y`, `z`, `feed`, `feed_class`; optional `pitch` (thread lead, mm/rev), `inverse_time`; multi-axis `axes`, `a`, `b`, `c` | Feed move. |
| `arc` | `x`, `y`, `z` (end), `cx`, `cy`, `cz` (absolute centre), `clockwise`, `feed`, `feed_class`; `plane`: `"ZX"`/`"YZ"` (absent: XY) | Circular or helical move from the previous position; within the post's `arcs` declarations. |
| `dwell` | `seconds` | Pause. |
| `cycle` | `x`, `y`, `z`, `cycle` | One hole of a canned cycle the post takes. |
| `cycle_cancel` (`cycleCancel`) | | After a run of holes. |
| `compensation_on` (`compensationOn`) | `side` (`"left"`/`"right"`), `tool` | Cutter compensation starts on the next linear move (which moves in XY). |
| `compensation_off` (`compensationOff`) | | It ends, before the next rapid. |
| `index` | `axes`, `a`, `b`, `c` | Turn the rotary axes (3+2 index, or the start of a multi-axis toolpath). |
| `home_z` (`homeZ`) | | Retract Z to the machine reference before the table turns without a tool change. |
| `tcp_off` (`tcpOff`) | | End tool centre point control. |
| `probe` | below | Version 6: one probing cycle with a spindle probe ([Probing](#probing)). |
| `chuck` | `spindle` (`"main"` or `"sub"`), `open` | Version 7, lathes: a chuck opens or closes: the main spindle's for a bar pull (DL-0091), or to let a transferred part go; the sub-spindle's to take it (ADR-0007). |
| `sub_axis` (`subAxis`) | `z2`, `feed` (`null`: rapid) | Version 7: Z2 to `z2`, in the machine's own Z2 (the lathe layout's: 0 with the sub-spindle home), in a transfer (ADR-0007). |
| `spindle_sync` (`spindleSync`) | `on`, `angle` (degrees, or `null`) | Version 7: the spindles synchronised, at an angle or not, or no longer. |
| `bar_feeder` (`barFeeder`) | `on` | Version 7, lathes: the bar feeder pushes the bar, or stops. |

`feed_class`: `"cut"`, `"plunge"`, `"ramp"`, `"lead_in"`, `"lead_out"`, `"link"`.

`cycle`: `type` (`"drill"`, `"dwell"`, `"peck"`, `"chip_break"`, `"tap"`, `"tap_left"`,
`"bore"`, `"back_bore"`), `top`, `bottom`, `retract` (the R plane; the tool returns there after
each hole), `peck`, `chip_break_retract`, `dwell` (s), `feed` (mm/min), `pitch` (taps, mm/rev).
A run of holes with the same `cycle` is one cycle.

A `back_bore` (version 5, DL-0083) starts at `retract` above the hole: the spindle orients at
`orient` (degrees) and the bar shifts by (`shift_x`, `shift_y`) (mm, away from its cutting
edge), goes down through the hole to `bottom`, shifts back, starts and feeds up to `top` (the end
of the cut, above `bottom`), dwells, orients, shifts and returns to `retract`, where it shifts
back. Posts take it only when their manifest's `cycles` lists `"back_bore"`; others get the
same as `spindle_orient`, `rapid`, `spindle` and `linear` instructions.

## Order

For each operation:

1. `operation_begin`;
2. on a tool change: `coolant` off, `spindle` 0, `tool_change`;
3. `home_z` if the rotary axes turn without a tool change;
4. `work_offset`, `index` when they change;
5. `rapid` to the start X/Y (`z` null), `spindle`, `tool_length` (or `rapid` down), `coolant`;
6. the motion, with `cycle_cancel`, `compensation_on`/`off`;
7. `tcp_off` after a multi-axis toolpath with TCP; `operation_end`.

A live tool's operation has `spindle` 0 and `live_tool` before its first `index`; the next operation that is not has
`live_tool` 0 first, as does a tool change. After the last operation: `coolant` off, `live_tool` 0, `spindle` 0. Header, safe start, return home and
program end are the post's. Spindle, coolant, tool, work offset and rotary position appear only
when they change.

## Probing

Version 6 (DL-0090, [ADR-0008](../adr/0008-probing-in-the-post-api.md)). A probing operation's
tool is a probe (`type: "probe"`); it does not turn, so no `spindle` instruction starts it. Its
cycle is one `probe` instruction, between the usual rapids to and from it:

| Field | Content |
|---|---|
| `operation` | Index of its operation in `operations` |
| `feature` | `"surface"`, `"web"`, `"pocket"`, `"bore"`, `"boss"`, `"outside_corner"`, `"inside_corner"` |
| `check` | `true` for a feature check: compare with the nominal values within `tolerance`; `false`: set a work offset |
| `work_offset`, `axes` | The work offset to set (1 = G54, ...; 0 for a check) and what of it: `"X"`, `"Y"`, `"Z"` or `"XY"`, so that the feature is at its nominal position |
| `nominal` | `{ x, y, z }`: a surface's touch point; a web's, pocket's, bore's or boss's centre; a corner. Its `z` is where the stylus ball's centre is while it measures, on a side face |
| `direction` | `{ x, y, z }`: a surface's direction of touch; the axis a web or pocket is measured across; a corner's two directions of touch as (+-1, +-1, 0); zero for a bore or boss |
| `size` | A web's or pocket's width, a bore's or boss's diameter |
| `tolerance` | A check's tolerance |
| `approach`, `overtravel` | From the surface where each measuring move starts; how far past it it looks |
| `fast_feed`, `measure_feed` | The operation's positioning and measuring feeds (mm/min) |
| `stylus_diameter`, `trigger_offset` | The probe's ball and its effective trigger offset |
| `moves` | The cycle's moves in order: `{ kind, x, y, z, feed }`, `kind` `"position"` (a protected move: stop if the stylus touches anything) or `"measure"`. A measuring move's `x`, `y`, `z` are past the surface by the overtravel, and `expected` `{ x, y, z }` is where it stops on the model's surface |

Coordinates are the tool tip, as everywhere: the ball's centre is half its diameter above. What the
control does with the measurement (the probe maker's macros) is the post's: the generic posts
refuse cycles that set a work offset and write checks as moves only
([oxcam:fanuc](fanuc-base.md#probing)).
