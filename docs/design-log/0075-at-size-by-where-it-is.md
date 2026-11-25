# DL-0075: A surface is at size by where it is

- **Status:** implemented
- **Module:** libs/sim (`StockDisplay`, `compareRay`, `surfaceColours`)
- **Patent gate:** low (citation only). Colouring a display from a comparison the simulation already makes; nothing chooses a toolpath.
- **Author(s):** OxCAM contributors

## Problem

Playback colours a surface green when it is at size, and otherwise by the operation that cut it (DL-0012). At size was decided for a whole Z ray. On a mill that does: what is left is on top of the ray.
A live tool on a lathe goes in from the side (DL-0066, DL-0074), so a ray can have stock left in its middle and be finished at both ends. A wrapped slot not yet cut showed a band of the turning operation's colour along the whole
diameter and shoulder above and below it, which read as a cut where nothing happened. It stayed once the slot was cut, as the corners a round tool leaves keep those rays in excess.

The same code had the same fault wherever a ray ends on finished surface with stock left along it. On a 4th axis (`rotary_project`) the underside opposite a groove showed the stock's colour. Milling after turning (`millturn_project`) showed it on the back face, under the pocket and holes.

## Approach

The comparison keeps where along each ray the stock beyond the part is, as it does for a gouge (`excessLo`, `excessHi`), and a cell gathers those of its rays. A surface is at size if the cell's rays mostly are, as before, or if it lies more than the
same margin a gouge has (a cell and a half) clear of that span. So the part of the diameter by the slot still shows its cut, until the slot is cut, and the rest stays green.

Only the colours change. The comparison's numbers are as they were, and so is the cache.

## Correctness and tests

`SimTest` "a surface is at size by where it is along its ray": a part with a groove all around and a stock still filling it: every wall below the groove green, none beside it. `LiveMillTest` "wrapping a pocket around a lathe's diameter" on `wrapped_project`, part way and finished: nothing but green away from the slot and the corner the insert's nose leaves at the shoulder. Both fail on the colouring before this. `gui.lathe_wrap_part_way` and `gui.lathe_wrap_all`, looked at.

Without a comparison nothing changes: on all three projects each colour covers the same number of vertices as before, and on `wrapped_project` the slot's colour stays by the slot.

## Known limitations

- **Spans are per cell, from lowest to highest:** a ray with stock left in two places treats what lies between them as not at size.
- **`buildDisplayMesh`** (tests and benchmarks only) has no spans, and counts excess for the whole ray as before.
