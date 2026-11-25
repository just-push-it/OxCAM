# DL-0071: Left-hand taps

- **Status:** implemented
- **Module:** libs/tooling (`MillGeometry::leftHand`), libs/toolpath (`CycleType::TapLeft`, `spindleReverse`), libs/strategies
  (`tap`, `drill`), libs/post (`Lower`, `FanucLathe`, back-plot), data/posts, apps/opencam-gui (`ToolLibraryDialog`)
- **Patent gate:** low (citation only). Code syntax from controllers' documentation.
- **Author(s):** OxCAM contributors

## Problem

DL-0065 left a left-hand thread to the post's option, per program. The maintainer asked for it to come from the tool: a left-hand
tap is what the tool is.

## Approach

**The tool.** A tap has a **Thread hand** on its page in the tool library. Left hand is stored as `left_hand` in the tool's
geometry, written only when it is set, so libraries and projects without it read and write as they did.

**The toolpath.** A left-hand tap's cycle is `TapLeft`, and the toolpath has `spindleReverse`. The speed and the feed stay the
values the operation shows (ADR-0004): only the direction changes, and the program's words say it. `spindleSpeed` remains positive.

**A separate cycle type, deliberately.** A post lists the cycles it takes in its manifest. A post that has not been written for
this has no `tap_left` there, so it refuses, rather than tapping a left-hand thread with its right-hand cycle: that would run the
wrong way into the hole. The NC program's `spindle` has `reverse`, and `live_tap` has `left_hand`.

**The codes**, all on the post:

| What | Writes | From | Applies to |
|---|---|---|---|
| Mill, `fanuc-generic-mill`, `fanuc-generic-mill-4axis`, `haas-classic-mill` | `M04`, `G74` | Fanuc's and Haas's mill cycle lists | Mills. Not an option: `G74` is the left-hand cycle on both |
| Lathe, on the spindle's axis (`tapLeft`) | `haas-lathe`: `M04`, `G184`. `fanuc-generic-lathe`: empty, refused until set | Haas lathe G-code list | Haas lathes. `G74` on a Fanuc lathe is another cycle altogether, and whether a lathe's `G84` follows `M04` depends on the control, so it is not assumed |
| Live tool along Z (`liveTapAxialLeft`) | `haas-lathe`: `S{rpm}`, `G18 G186 C{c} Z{z} R{r} F{lead}`. `fanuc-generic-lathe`: empty, refused until set | Haas Factory Outlet, *Live Tool for Lathe* training manual, September 2014 | Haas lathes with live tooling |
| Live tool across (`liveTapRadialLeft`) | `haas-lathe`: `S{rpm}`, `G19 G196 X{x} F{lead}`. `fanuc-generic-lathe`: empty, refused until set | The same | The same. The manual says some radial tools (Heimatec) turn the other way, so there G195 and G196 change places |

The back-plot reads `G74` on a mill and `G184` on a lathe, so `--verify` covers them.

**A thread mill's hand stays on its operation.** One thread mill cuts either hand; a tap cuts one.

## Correctness and tests

`HoleOpsTest`: the tool sets the cycle and the direction, with speed and feed unchanged. `PostTest`: M04 and G74 on both mill posts, read
back by the back-plot, and a post not listing `tap_left` refuses. `LatheHoleTest`: G184 and M04 on Haas, refused on the generic lathe naming
the option; G186 and G196 on a live tool, matching the back-plot. `LibraryIoTest` keeps the hand through the file.

## Known limitations

- **Not run on a machine.** Run the first in the air, and check the direction.
- **A tap's name is not read.** Nothing sets the hand from "M6x1 LH": it is the setting.
