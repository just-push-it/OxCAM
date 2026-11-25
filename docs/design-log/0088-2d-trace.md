# DL-0088: 2D trace along edges

- **Status:** implemented
- **Module:** libs/strategies (`trace`, `traceOffset`, `traceParams`), libs/cad (`edgeChains`), libs/document
  (`traceChains`, `generateToolpath`, `chamferZones`), apps/opencam-gui (`PropertyPanel`, `MainWindow`)
- **Patent gate:** low (citation only). Running a tool along a line, beside it at a fixed distance, or a set depth below a
  surface is long-published practice; the open-curve offset follows a published pair-wise offset method.
- **Author(s):** OxCAM contributors

## Problem

No operation followed an open curve. A 2D contour takes faces or closed loops of edges, a chamfer closed loops at one
height. Engraving a line, scribing a layout mark, marking a part and cutting a slot along a curve all need the tool centre
on (or beside) selected edges, open chains included, to a depth.

Inputs: the selected edges of a model, the tool, and the operation's parameters. Output: a toolpath whose cutting moves keep
the tool centre on each chain or a set distance from it, at the depth below the chain (or below the surface under the
tool), reached in levels no deeper than the step down.

## Approach

**Chains (`cad::edgeChains`).** The selected edges are chained end to end in space, not in plan: an outline over another
(the top and bottom of a plate) stays two chains. Each chain goes as far as it can both ways from the edge it starts with;
where three edges meet, it takes one and the others start chains of their own. A chain is closed when it ends where it
started. It is flattened onto the plane across the tool axis: lines and arcs about the axis stay exact, other curves are
discretized within the tolerance, and edges along the axis vanish. Its height is its lowest point. Edges at any slope are
taken, as their plan view.

**Direction.** A closed chain runs counter-clockwise, starting at the start point if one is picked, else at the middle of
its longest straight segment (as contours do). An open chain starts from its end nearer the start point, else from its end
with the lower X (then Y). *Reverse* runs every chain the other way.

**Tool position.** *On the line*, the tool centre is the chain. *Left* and *Right* are seen looking the way the tool
travels; the tool's side runs the *Distance from line* away (negative takes it over the line), so the centre runs at
R + distance. With the spindle turning clockwise, the material is on the right of the tool when it climb mills, so Left is
climb milling against the line and Right conventional (DL-0008 states the same rule for contours). There is no separate
climb/conventional choice: with Reverse it would give the same four combinations twice over. On a closed chain run
counter-clockwise, Left is the inside. *Side-by-side passes* run farther from the line first, a pass spacing apart,
stepping in to it: a wider slot.

**Offsetting a closed chain** is a region offset of the loop (Clipper2, as for contours, DL-0008), its outer loops arc-fitted
(DL-0010) and turned back to the chain's direction. The tool not fitting leaves nothing, and a warning.

**Offsetting an open chain (`traceOffset`)** cannot use a region offset, which has two sides and closed ends. It follows the
pair-wise offset of a point sequence (Choi and Park, 1999):

1. The chain is discretized within the tolerance. Each segment is moved by the distance to its side.
2. At each vertex, the two moved segments are joined. Where they part (the outside of the corner) an arc about the vertex
   joins them. Where they overlap, the crossing of the two lines is taken when it lies on both moved segments, which covers
   the many shallow corners of a discretized curve. Otherwise both ends are kept, and the path forms a small loop there.
3. **Trimming.** Every point of a correct offset is the distance from the chain, and points of a local loop, or of a part of
   the chain that comes back near itself, are nearer. The raw path is clipped against the area swept by a disc of the
   distance, less a margin of three chord tolerances, along the chain (`geom2d::sweep`, `geom2d::clipPaths`). This is
   Choi and Park's removal of invalid loops, done by Boolean clipping rather than by pairwise intersection tests.
4. The pieces left are put back in order along the raw path. Two neighbours join again when the line between them keeps
   (within twice the margin) the distance from the chain, as across a trimmed corner loop. Otherwise they stay separate
   paths, each entered from above. Pieces shorter than ten tolerances are dropped, and the rest are arc-fitted.

**Depth and levels.** The depth is below the chain's height. The step down is the cutting preset's axial depth ap (an
operation may change it for itself), as for every other strategy. It is only asked for when there is a depth: a trace at
depth 0 runs on the line's height and needs no ap. The levels are equal and none is deeper than the step down
(`depthLevels`' rule). Each chain goes to its full depth before the next. A single closed path goes on down where it ends,
at its start. Open paths and several passes retract to the retract height and come down again just above the level before.

**Entry.** *Plunge*: straight down at the plunge feed. *Ramp*: along the path at the ramp feed and ramp angle (the preset's
when not set here), ending at the start at the level's depth. Round a closed path the ramp goes forward lap after lap and
finishes its lap at depth; along an open path it goes back and forth over the start of the path in an even number of equal
legs, each no longer than the path. Ramp moves never exceed the angle.

**Following the surface** (an option). The tool tip's height on the part under each tool-centre point comes from the
existing drop-cutter on the part's mesh with the tool's shape (DL-0016), at points no farther apart than the sample spacing.
The depth is then below that height. This covers engraving on curved and sloping faces. Retracts clear the highest point
the paths run over.

**Feeds (ADR-0004).** Cutting moves use the cutting feed, plunges the plunge feed and ramps the ramp feed, all from the
preset as the operation shows them. Nothing is derived or scaled. A zero feed is refused by generation, which names the
value. The way down into each chain's first level is fed below the operation's feed height, so stock still standing above
the edges is not entered at rapid.

**On a lathe.** A trace is milling, so a live tool goes round what it cuts through the existing live-tool path (DL-0066):
polar interpolation on the face by default, or the Y axis with C held. A wrapped trace around the diameter is not done (see
limitations).

**In the simulation.** An engraving is seldom modelled, so the part comparison would show its groove as a gouge. As for
chamfers (DL-0014), the material a trace removes within the tool's reach of its edges, from their height down to its depth,
is not counted as a gouge. The reach is R on the line; beside it, 2R plus the distance and the extra passes. What it removes
anywhere else is still counted. A trace following the surface has no such zone.

**Selecting.** In the interface a trace's geometry is edges, one click per edge (`GeometryUse::Chains`). Contours and
chamfers take a whole outline per click; that is no help for an open chain. Faces are not taken. A start point can be
picked, as for contours.

## Sources

- B. K. Choi, S. C. Park, "A pair-wise offset algorithm for 2D point-sequence curve", *Computer-Aided Design* 31(12),
  735–745, 1999. DOI 10.1016/S0010-4485(99)00060-9. Offsetting segments of a point sequence, joining them at corners, and
  removing the local and global invalid loops.
- M. Held, *On the Computational Geometry of Pocket Machining*, Lecture Notes in Computer Science 500, Springer, 1991.
  DOI 10.1007/3-540-54103-9. Offsets of curves made of lines and arcs, and the rounding of convex corners.
- P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press, 2008. ISBN 978-0-8311-3347-4. Climb and conventional
  milling, plunge and ramp entries (general practice, as in DL-0008).
- Drop-cutter: DL-0016 and its sources.

## Deviations from sources

- Choi and Park find invalid loops by intersecting the offset with itself. Here they are clipped away against the chain's
  swept disc instead, which is simpler with the Clipper2 Booleans already in use. It also removes parts of the offset that
  come near another part of the same chain (their global invalid loops) in the same step.
- A margin of three chord tolerances keeps chord-approximated arcs of the offset from being clipped. A loop shallower than
  that margin (up to 0.006 mm at the default tolerance of 0.002 mm) is not clipped: the tool goes that much nearer the line
  at such a corner.

## Correctness and tests

`TraceTest` (strategies):

- On an open chain the tool centre is on the line, at the depth below its height, from its lower-X end. A start point and
  Reverse pick the end.
- The plunge is at the plunge feed and the cut at the cutting feed.
- The depth goes in equal levels, none deeper than the step down, each starting at the chain's start. A closed chain goes
  down in place with no rapid in between.
- Left and right of a line and of an L keep the distance (±0.01 mm), with the outside corner rounded. Inside corners of a
  U and of a step shorter than the distance give one path at the distance, trimmed. A U narrower than the tool gives
  nothing and a warning. Side-by-side passes go farthest first.
- A closed chain given clockwise runs counter-clockwise, and reversed clockwise. Left is inside and right outside, at the
  distance. A loop too small for the tool gives nothing and a warning.
- Ramps run on the path at the ramp feed, never steeper than the angle, ending at depth where the cut starts: once forward
  and back on a long open chain, more legs on a short one, laps on a closed one.
- Following a sloping surface keeps the depth below it.
- The parameters: what does not apply is not read, the step down is the preset's only with a depth, and the ramp angle is
  the preset's unless set.

`GenerateTest` (document):

- On the hole plate, two top edges meeting at a corner are one open chain, cut in levels at the depth below them.
- The through hole's top edge, with the tool beside it on the left, is a circle inside it, the tool's side on the edge.
- No rapid goes below the edges' height.
- A zero plunge feed is refused, naming it. Faces and an empty selection are refused, naming edges.
- The simulation removes the groove with no collision and no gouge. The fixture project `trace_project` (an open and a
  closed trace after facing) simulates cleanly.

`LiveMillTest`: on the mill-turn part's face, an open chain of the pocket's mouth is traced by a live tool under polar
interpolation, on its edges at the depth. It is refused without live tooling, simulates with no collision and no gouge,
and Haas's program has it under G112 with the back-plot agreeing.

GUI tests (on the VM):

- `gui.trace_pick_open_chain` adds a trace, presses Select and clicks two of the block's top edges in the top view: one
  open chain.
- `gui.trace_simulation` and `gui.trace_panel` screenshot the fixture project's simulation and a trace's page.

## Known limitations

- **No wrapped trace** around a turned diameter or a 4th axis: the stretch goal of the request, left for the maintainer to
  decide (it would unroll the chain as `rotary_pocket` does, DL-0024 and DL-0074).
- **No text:** traces follow model edges, not fonts.
- **Plan view only:** a chain is flattened, and its depth is from its lowest point. A sloping 3D edge is cut at one height
  unless the trace follows the surface, and then the depth is below the part's surface, not below the edge.
- **No leads:** a trace enters by plunge or ramp and leaves straight up. A side trace is not compensated in the control
  (no G41/G42).
- **Where three edges meet,** which way a chain goes on is the order the edges come in, not a choice.
- **Not run on a machine.** Run the first in the air.
