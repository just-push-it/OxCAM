# DL-0072: A tap off the axis needs a hole

- **Status:** implemented
- **Module:** libs/document (`checkTapHoles`, `milledBefore`, `simulateLatheSetup`), libs/sim (`tapsWithoutAHole`,
  `CollisionKind::NoHole`, `holeRadiusAt`)
- **Patent gate:** low (citation only). A plain clearance check.
- **Author(s):** OxCAM contributors

## Problem

A tap on the spindle's axis that finds solid stock is refused (DL-0043). A live tool's tap off the axis (DL-0065) was not checked, so a
program could send it into solid metal, and it would break. A tap's cut leaves nothing in the stock, so the simulation did not show it either.

## Approach

**What has to be clear.** A tap's core, the same radius as on the axis: `r - 0.75 × pitch`, what is left of it inside a tap drill's
hole. M6×1 has to find a Ø4.5 core clear, and a Ø5 hole gives it that. A blind hole drilled to the tap's depth has a drill's point
in the way of that core at the bottom, and that is ordinary, as a tap is chamfered there: both checks allow 0.6 × the core radius, a 118° point's
height.

**Refused when generating, where the stock can tell.** With the earlier operations' profile and holes in hand (`latheStockBefore`, and the
cuts carried from an earlier setup), the strokes are sampled around the core at 0.25 mm stations, and a point inside the profile that no hole
holds is solid. The message names the place: `T7 would cut solid stock at X.. Y.. Z..: drill the hole first, in an earlier operation`. It stops
the program going out, as on the axis.

**After milling, the simulation says.** A hole may have come from a helical bore, or a pocket may have taken the material away, and only dexels
hold those. So after any milling (`milledBefore`: in this setup, or in one its stock comes from) generating does not refuse, and the simulation
looks instead: a copy of the stock is cut by a cylinder of the core along each stroke (`tapsWithoutAHole`), and solid across more than half a
layer of the resolution is reported as a collision of its own, `no hole`, with how long a core of solid it found. That is a warning in the
simulation's list, and it does not stop posting: nothing else is known at generating time.

The simulation runs this for every live tap off the axis, so a stock without milling has both checks. The refusal comes first.

## Correctness and tests

`LatheHoleTest` "holes with a live tool": a tap with the drills disabled is refused. `LiveMillTest`: after milling a tap whose drill is disabled
generates, and the simulation reports `no hole`; with its drill it reports nothing.

## Known limitations

- **A hole's size** counts only as far as the core: a hole under the tap drill but over the core passes.
- **Rays are point samples:** in the simulation's check a core thinner than the resolution can hold no ray and reads as clear. At automatic
  resolution this means a tap under about M2 on a bar of 100 mm.
- **A tap on a mill** is not checked here: this entry is lathes.
