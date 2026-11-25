# DL-0084: Wrapped contour

- **Status:** implemented
- **Module:** libs/strategies (`rotaryContour`, `RotaryContour.cpp`; `appendWrapped` in `MultiAxis.cpp`), libs/cad
  (`edgeChains`, `faceChains`, `faceDistanceFromXAxis`), libs/document (`wrappedContour`, `latheWrap`), apps/opencam-gui
  (Multi-axis group)
- **Patent gate:** low (citation only). Following a curve unrolled from a cylinder with a radial tool, on the line or offset
  from it, is the classic rotary-axis wrap of DL-0024 applied to a 2D contour (DL-0008); offsetting a region inward until it is
  gone is textbook pocketing. Nothing here chooses a tool axis: it is radial, as in DL-0024 and DL-0074.
- **Author(s):** OxCAM contributors

## Problem

`rotary_pocket` (DL-0024, DL-0074) clears what the model has below the stock. Keyways, helical grooves, cam tracks and the walls of
wrapped profiles are better cut by following them: a slot the tool's width is one pass along its middle, not a pocket; a wall wants a
finishing pass beside it. The maintainer asked for a wrapped contour on a mill's 4th axis and on a lathe's C axis with a live tool,
from edges (open or closed) or a slot's floor, with depth, step down, an offset left, right or on the line, leads, plunge or ramp
entry and stock to leave.

## Approach

**The same wrap.** Everything about the cylinder is DL-0024's: a point at angle t (from +Z towards +Y about the WCS X axis) unrolls
to arc length v = ρ t at a chosen radius ρ, and `wrapToolpath` maps the flat toolpath back with the tool axis radial. On a lathe the
part is turned so its spindle is X and the toolpath turned back, as DL-0074 does, sharing its stock (`latheWrap`: the stock as the
earlier operations left it, heights radial from its largest radius). Selections that go to the strategy are 3D chains of points
(`cad::edgeChains`, `cad::faceChains`); the strategy unrolls them itself.

**The radius a selection is unrolled at** is the radius its floor is cut at: the selection's nearest point to the axis, less the
depth, plus the stock to leave on the floor. Offsets are then exact along the arc there, where the flat end of the tool cuts. A
face's nearest point comes from the exact B-rep (`faceDistanceFromXAxis`), so a flat keyway floor is cut at its middle, not at its
edges. Points between the chain's points are added where the angle turns more than the chord tolerance allows, so the unrolled
curve is the image of the chords, not a chord of it.

**Curves (edges).** Chained end to start in 3D. On the line, the tool centre follows the unrolled curve. Left or right, it follows
the curve's side at the tool radius plus the stock to leave: for a closed curve the inward or outward offset of the region it bounds,
for an open one the side of the area a disc of that radius sweeps along it, from abreast of its start to abreast of its end. Which
side is left: an open curve runs from its end at the lower X (on a lathe, Z; at the same X, the lower angle), a closed one
counter-clockwise seen from outside the part, so left is inside. Climb or conventional then decides which way it is cut.

**Around the part.** A curve closed in 3D whose angle grows by a turn going round it is periodic in the plane, not closed. It is
offset over three periods and one period is kept, from where the curve starts; at the next level it goes on round, a turn later,
rather than back. Every pass is moved by whole turns to start nearest where the tool is, which is the same place on the part.

**Slots (faces).** A slot's floor may come in several faces (OCCT splits a cylinder's face at its seam): faces of one floor radius
are one slot. Each face's boundary, without seam edges, is unrolled. A face whose boundary is two rims going once round (a ring or
endless cam groove) is a band: its rims, three periods long, make a strip. Every face is laid down three times a period apart and
all are united, so faces that meet only across the angle where unrolling starts join up. A part of the union that holds its own
copy a period on goes all the way round and is cut over one period; of the others the copy in the first period is kept. Boundary
points at the part's ends are moved out by the tool's diameter (plus stock): a keyway open at an end runs out past it.

The passes are inward offsets of the floor: at the tool radius plus the stock to leave from the walls, then each the cutting preset's
radial depth (ae) further in until nothing is left. They are cut innermost first, so the walls come last. A pass narrower than three
hundredths of a millimetre is a sliver: the slot is the tool's width there, and its middle line is cut once instead of the sliver's
two sides. Where the offset at the tool radius leaves nothing (a slot drawn at exactly the tool's width), the floor is offset 0.01 mm
less, the sliver's middle line found from it and its ends brought back by 0.01 mm, so they keep the tool's radius from the slot's
ends. The middle line is the midpoints of one side and its nearest points on the other, away from the tips, continued straight to
the tips (where the loop turns back within 0.1 mm). Should the slot be narrower than the tool by more than 2 µm, the toolpath warns
how far the tool cuts into each wall; narrower than the tool by more than 0.02 mm, it is refused.

**Levels and entries.** Levels go from the stock (on a lathe, its largest radius over the selection and a tool radius each side) to
the floor at most the preset's axial depth (ap) apart (`depthLevels`), as the other wrapped and 2D strategies take the step down.
Each pass is entered at each level by a plunge at the plunge feed, at the start of its lead arc where there is room for one, or by a
ramp: forth and back along the start of the pass, an even number of legs, each no steeper than the ramp angle, ending where it began.
A pass that is too short to ramp into at that angle is refused, naming the angle. Leads are tangent arcs (and lines) on the side away
from the wall, as in DL-0008; they are halved up to four times to stay where the tool centre may go (away from the curve by the
offset, inside the slot's first offset) and left out with a warning otherwise. On the line and on a slot's middle there are none.

**Joining.** Each selection's flat toolpath is wrapped at its own radius; they are joined around the clearance cylinder the short
way (`appendWrapped`, shared with `rotary_pocket`).

**The feeds (ADR-0004).** As the cutting preset gives them: cut, plunge, ramp and lead feeds, each its own value, in mm/min along the
tool tip. Nothing scales them. How they reach the machine is what `rotary_pocket` already does, and the operation's description says
so: on a 4th axis without tool centre point control the solve gives moves that turn A an inverse-time feed, 1 / the minutes the move
takes at the set feed along the tip (DL-0023, G93); on a lathe each run that turns C is in cylindrical interpolation on its own
radius (DL-0074), each level its own, so F there is the set feed on the surface the tool cuts. Posts may change that for their machine
(ADR-0004 amendment).

## Sources

- B. K. Choi, R. B. Jerard, *Sculptured Surface Machining*, Kluwer, 1998. Wrapping a flat toolpath onto a cylinder for a rotary
  axis.
- M. Held, *On the Computational Geometry of Pocket Machining*, LNCS 500, Springer, 1991. Contour-parallel offsets.
- DL-0008 (contour leads), DL-0023 (4-axis posting and inverse time), DL-0024 (the wrap), DL-0074 (on a lathe).

## Deviations from sources

None of substance; the sliver's middle line is a simple construction for strips two walls apart, not a medial axis.

## Correctness and tests

`tests/unit/strategies/RotaryContourTest.cpp`:

- **Keyway along the axis:** on a line with a depth, every level's points at the line's angle (the arc length wraps back to it) to
  1e-9 rad, the deepest point at the floor to 1e-6, no leads; its floor drawn at the tool's width, one pass a level along the middle
  to 1e-3 mm of arc, the ends the tool radius from the slot's; open at the part's end, run out past it; narrower than the tool,
  refused.
- **Beside a wall:** left and right of an open curve picked either way, at r/ρ along the arc, climb and conventional each way.
- **Helical groove:** 1.5 turns at 5 mm: every floor point on the helix (its angle the helix's at its x) to 1e-6, 1.5 turns of A,
  and the length the helix unrolls to.
- **Closed wrapped outline:** outside a pad, every floor point the tool radius from the outline unrolled at the floor (5 µm),
  clockwise when climbing, leads clear of it; inside, with stock on the walls and floor.
- **Around the part:** a ring groove the tool's width cut on its middle once round a level; wider, a pass by each wall; a circle on
  the line once round.
- **A floor in two faces** across the angle where unrolling starts: one slot, no wall between.
- **Entries and refusals:** ramps no steeper than their angle, at the ramp feed; every feed the preset's; a curve through the axis, a
  depth past it, a wide slot without ae refused.

`tests/unit/document/WrappedContourTest.cpp`, on the fixture projects:

- **Lathe (`wrapped_project`):** the slot's floor faces cut inside the walls to the floor, all four walls reached, the tool radial,
  live, wrapped; a wall's foot along Z cut on the line with C held; the simulation takes the slot's volume (5%), no collision, no
  gouge; Haas `G107 … R17.` and Fanuc `G7.1 C17.` with the feed as set, both back-plots agreeing; refused without live tooling and
  with nothing selected.
- **4th axis (`rotary_project`):** the ring groove and the 120° groove: a pass by each wall of the ring, the groove's floor reached;
  the 4-axis post writes A words and G93 and its back-plot (with the machine) agrees; refused on a machine with no rotary axis.

## Known limitations

- **Offsets are along the arc at the floor.** A wall that runs along the axis comes nearer the tool at its foot than the arc says,
  by d − ρ sin(d / ρ) (d the offset): 0.016 mm for a 6 mm mill at radius 17. The toolpath warns above 0.01 mm. `rotary_pocket` has
  the same.
- **Walls are what a radial tool leaves.** A wide keyway with parallel walls cannot be cut by a tool that stays radial; this is
  for walls normal to the surface, or a slot the tool's width.
- **Slot faces only on one floor radius each.** A floor that is not round (a flat) is cut at its nearest point's radius, its edges
  then above the tool by up to the flat's sagitta.
- **An open end is found only at the part's ends** (by X): a slot running out into a neck or a chamfer keeps that end as a wall.
- **No selection order between curves and slots:** curves are cut first, then slots, each in the order selected.
- **Fixtures are not avoided** (as for the other wrapped operation): it warns, and the simulation shows them.
- **Not run on a machine.**
