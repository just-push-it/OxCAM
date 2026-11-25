# DL-0074: Wrap milling on a lathe

- **Status:** implemented
- **Module:** libs/strategies (`rotaryPocket`, `stockRadiusAt`), libs/document (`wrappedOnLathe`), libs/machine (`solveLiveWrap`), libs/post
  (`Wrap`, `Lower`, `FanucLathe.wrap`, back-plot), data/posts
- **Patent gate:** low (citation only). Unrolling a cylinder for a rotary axis is long-published practice (DL-0024); code syntax from
  controllers' documentation. Nothing here chooses a tool axis: on a lathe it is radial.
- **Author(s):** OxCAM contributors

## Problem

DL-0066 milled a lathe's face by polar interpolation and across the part at a held C. A slot that goes around the diameter needs C and Z moving
together, with the feed kept on the surface: cylindrical interpolation, `G107` on Haas and `G7.1` on Fanuc. The maintainer asked for it.

## Approach

**The strategy is the one a 4th axis has.** `rotary_pocket` (DL-0024) already unrolls a recess in the part at each level and pockets it flat. On a lathe it
goes about the spindle instead of X: the part is turned so the strategy sees its axis as X, and the toolpath is turned back, each level unrolled at its own
radius as before. A *wrapped pocket* takes what the model has: slots, cam grooves, flats. A machine without live tooling refuses it, and so does a mill with no rotary axis: it cannot
cut one (DL-0076).

**It cuts what is there.** On a 4th axis everything under a single stock radius is a recess. On a turned part that would clear every neck in the air.
So on a lathe it gets the stock as the earlier operations left it (`stockRadiusAt`): levels start from it, only cells with stock above them count, and the
tool goes only as far from those as its radius and way out need. A slot opening into a neck still runs out through it. Heights are radial here, from the
largest radius of the stock, by the offsets the operation's heights already have from the stock's top.

**Two faults of the strategy this slot found**, which a 4th axis had too. A slot across the map's angle 0 put the seam just past its far wall, on a column of no free cells, and the grid
stopped a row short of it: that wall fell half a cell from where it is, either way, up to 0.25 mm. The grid now goes back to the seam. And a floor at exactly the level's radius, as at its mesh's edges,
read as solid, so a mesh edge on a map column (a slot centred on angle 0 has both) split the deepest level in two around a wall of no width, leaving a fin 0.2 mm wide. A floor at the level is free now, to 1e-6 mm.

**The machine (`solveLiveWrap`).** X is the radius, and C goes on past a turn, since the control has to take it that way. Lines from the strategy's chord
splitting go back into the one line they were in the unrolled plane: the control interpolates there, so the path stays exact and the program stays short.

**The program.** Rapids go in X, Z and C with the mode off, as neither control's example rapids inside it. Each run of feed moves that turns C goes between
the control's on and off (`wrap`, NC program version 4); X-only and Z-only runs need neither. Arcs are lines, which are exact in the plane as the control interpolates.

**The feeds (ADR-0004).** As the operation shows them. The mode's radius is the deepest radius of its run, the floor of the level. That is where the strategy
unrolled it, and where the flat end of the tool cuts, so the F in the program is the feed there. Nothing scales it on the way. Ramps and plunges inside a run have
their own feeds and X moving with them.

**The codes are options** on the post:

| Option | `haas-lathe` | `fanuc-generic-lathe` | From |
|---|---|---|---|
| Cylindrical interpolation on | `G19`, `G107 Y0 C0 R{r}` | `G7.1 C{r}` | Haas: its lathe G107 page (`Y`, `C`, `R` or `Q`, G19 only). Fanuc: a lathe manual extract quoted on Practical Machinist, and a program example on MfgRobots, not a Fanuc manual in hand |
| Off | `G107`, `G18` | `G7.1 C0` | The same. Fanuc has them in separate blocks, which this keeps |
| Letter for the way around | `Y` | `C` | The same |
| It is in degrees | No: a length on the cylinder | Yes | The same |

Those are the control's own, like polar interpolation, so both posts start with them. The live tool's and C axis's codes stay the builder's (DL-0063), so `fanuc-generic-lathe` still refuses until those are set.

**The back-plot** reads `G107` with R or Q (Y back into C) and `G7.1`, so `--verify` covers them.

## Sources

Syntax facts only. Haas: [G107 on the lathe](https://www.haascnc.com/service/codes-settings.type=gcode.machine=lathe.value=G107.html), read through a search index as the site refuses automated readers. Fanuc:
[a program example](https://www.mfgrobots.com/Article/equipment/cnc/46597.html), [a forum quoting a lathe manual](https://www.practicalmachinist.com/forum/threads/g07-1-fanuc-example.338917/).

## Correctness and tests

`LiveMillTest` "wrapping a pocket around a lathe's diameter", on `wrapped_project`, a shaft with a neck: the path stays by the slot, not the neck, reaches the floor, the tool axis radial; refused without live tooling; simulated with no
collision, about the slot's volume taken, no gouge; Haas's and Fanuc's words, the mode's radius the floor's; `--verify`'s back-plot agrees, C included; empty codes refused, naming the option.

## Known limitations

- **Not run on a machine.** Run the first in the air: above all the C direction and whether `Y0` is where C0 is.
- **Haas's Setting 102** (the C axis's diameter) governs feeds with `M154` on: check it against the mode's radius on your machine.
- **Only from the model:** wrapped pockets come from a recess the part has. There is no text or sketch wrapped onto a diameter, and other strategies are not wrapped, save the wrapped contour (DL-0084), which follows edges or a slot's floor.
- **Generating does not avoid the fixtures here**, as for a 4th axis: it warns, and the simulation shows them.
- **Arcs in the unrolled plane are lines:** within the tolerance, more blocks than a `G2`/`G3` would be.
- **A part not about X on a 4th axis** is refused when it lies across the axis, its surface coming to the axis without going around it (DL-0076). With the axis through it, it is not: it is slow, and cancels between levels.
- **A square-cornered slot keeps its corners:** a round tool leaves its radius there, and the simulation shows that as stock left.
