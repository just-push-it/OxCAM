# DL-0065: Tapping with a live tool

- **Status:** implemented
- **Module:** libs/post (`LiveTap`, `Lower`, `FanucLathe.liveTap`, back-plot), libs/toolpath (`expandCycles`),
  data/posts/haas-lathe.js, data/posts/fanuc-generic-lathe.js, libs/document (`generateToolpath`, `simulateLatheSetup`)
- **Patent gate:** low (citation only). Code syntax from a controller's training manual.
- **Author(s):** OxCAM contributors

## Problem

DL-0063 refused taps on a live tool, as rigid tapping has its own codes on each control. The maintainer asked for it. A tap
cannot be plain moves as holes are: the tool has to stay in step with the feed, which only the control's own cycle does.

## Approach

**The operation.** A Tap operation's Tool drive now takes *Live tool*, on the face or across the part, with a tool
orientation, as for a drill. The speed is the operation's Live tool speed and the lead is the tap's pitch, both shown.

**The program.** A new instruction, `live_tap` (NC program version 4), carries the bottom of the hole, the R plane, the
speed, the lead and C. `live_tool` says `tapping`, since a tap's cycle starts the tool itself: the post writes no live tool
on, and feeds per revolution. After it, `G80`, then the usual live tool and C axis off.

**The codes are options.** The post writes them from two templates, with blocks separated by `|` and {rpm}, {lead}, {x},
{y}, {z}, {c} and {r}. `fanuc-generic-lathe` leaves them empty, and a live tap is refused until they are set, naming the
option. `haas-lathe` starts from Haas's:

| Option | Starts as | From | Applies to |
|---|---|---|---|
| Live tap along Z | `S{rpm}` then `G18 G95 C{c} Z{z} R{r} F{lead}` | Haas Factory Outlet, *Live Tool for Lathe* training manual, September 2014 | Haas lathes with live tooling |
| Live tap across the part | `S{rpm}` then `G19 G195 X{x} F{lead}` | The same | The same. The manual says some radial tools (Heimatec) turn the other way and take G196 for a right-hand thread |

Both feed per revolution (G99), and the manual says M133 is left out: S starts the tool. A Fanuc lathe with rigid tapping
is usually `M29 S{rpm}` then `G84 Z{z} F{lead}` or `G88 X{x} F{lead}`, but builders differ (a blog on a Mori SL says M329 on
the live tool's path), so that is an example to check, not a default.

**The back-plot** reads G95 and G186 (a lathe's block with a Z), G84, and G88, G195 and G196, so `--verify` covers them.

**The stock.** As on the spindle's axis (DL-0043), a tap's thread does not show: the maintainer says that is fine. Its rapids,
shank and holder are checked as a drill's are (DL-0064).

## Correctness and tests

`LatheHoleTest` "a live tool's speed and the program": face and cross taps post with Haas's codes, no M133 at the tap's
speed, and the back-plot matches the toolpaths, C included.

## Known limitations

- **Left-hand taps:** DL-0071.
- **That a tap has a hole** off the axis: DL-0072.
- **Not run on a machine.** The Haas blocks follow the manual's examples; the X of a tap past the part's axis is what the
  operation asks for, and a real machine's travel may refuse it.
