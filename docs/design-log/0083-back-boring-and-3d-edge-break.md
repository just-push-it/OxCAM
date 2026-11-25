# DL-0083: Back boring and back chamfering, and 3D edge breaking

- **Status:** implemented
- **Module:** libs/strategies (`backBore`, `edgeBreak`, `breakPose`), libs/toolpath
  (`CycleType::BackBore`, `Move::orient`), libs/post (`NcOp::SpindleOrient`, the back-bore
  cycle in the NC program and `oxcam:fanuc`), libs/sim (oriented moves, shank bands), libs/cad
  (`edgeFaces`, `tessellateEachFace`), libs/document (`backbore`, `edgebreak3d`), libs/tooling
  (a boring bar's neck and head angle)
- **Patent gate:** low (citation only). Back boring with an oriented, shifted bar is a canned
  cycle (G87 style) documented in controller programming manuals and textbooks for decades;
  interpolating a circle with a tool's head and breaking an edge with a pointed tool or a ball
  are long-established practice; the gouge check reuses drop-cutter (DL-0016), which the IP
  policy lists as low risk. Neither feature automates tool tilt or collision avoidance: the
  tool axis stays +Z and edges the tool cannot reach are skipped and listed, not worked around.
  New strategies get the `needs-patent-review` label (CONTRIBUTING §3); no review record
  exists yet.
- **Author(s):** OxCAM contributors

## Problem

1. **Back boring and back chamfering.** Machine the far side of a through hole (a counterbore,
   spot face or chamfer facing away from the spindle) by reaching through the hole from the
   near side, either with a back-boring bar (a single cutting edge on a head wider than the
   hole, on a neck narrower than it) or with a tool small enough to pass the hole.
2. **3D edge breaking.** `chamfer2d` breaks closed loops of edges at one height
   (`cad::edgeAcross`). Break edges that are not flat as well: edges on sloped or curved faces,
   in 3 axes with the tool axis +Z, with a chamfer mill or a ball, the break's width and its
   contact held along the edge, every face checked against gouging, and edges the tool cannot
   reach skipped and listed.

Both follow ADR-0004: every feed is the preset's (cut, plunge, lead) as set; nothing is
derived or slowed down. The shift and the spindle orientation angle are parameters the user
sees and sets.

## Where they live

**Back boring is its own Holes operation (`backbore`), not a mode of `bore` or `chamfer2d`.**
It shares hole selection, the R plane above each hole and the hole heights with `bore`
(`isHoleOperation`), but almost none of its parameters: a method (oriented bar or
interpolation), a feature (counterbore or chamfer), the shift, its direction, the spindle
angle, the clearance below the far side, a dwell. As a `bore` mode, most of `bore`'s
parameters would have been hidden for it and most of its own hidden for `bore`, with two-key
conditions the parameter model does not have (`ParamCondition` names one key). As a
`chamfer2d` mode it would have needed hole selection, which `chamfer2d` does not take, and it
also cuts counterbores. Its tool checks (the passage through the hole) are its own too.

**3D edge breaking is its own operation (`edgebreak3d`, in the 3D group)**, not a mode of
`chamfer2d`. `chamfer2d` is a contour around closed outlines at one height (DL-0014, built on
the contour kernel with leads, sides and climb or conventional direction per outline); the 3D
break follows single edges of any shape point by point and needs the part's faces for its gouge
check. It takes edges one by one (`GeometryUse::AnyEdges`: any edge, no outline grouping, no
side to flip).

## Approach: back boring

### Tool model

A **back-boring bar** is a `BoringBar` with a neck narrower than its head (`neckDiameter`,
`neckLength`, measured from the tip like other necked tools). A flat-topped head cuts back
counterbores and spot faces; with a **point angle** (`tipAngle`, the included angle of a cone
that narrows the head's top up to the neck) it is a **back-chamfer cutter**. The revolved
profile (`toolProfile`) carries both, so the simulation and the checks see the real shape; the
tool library dialog shows the point angle and the neck for boring bars.

For interpolation, any tool whose revolved profile has a head wider than what is above it:
slot cutters, lollipops, end mills with a neck, form tools.

The strategy reads all sizes from the revolved profile R(h) (radius at height h above the tip):
the head's largest radius R_max and the highest height h_max at it, the neck radius r_n, and
where the head ends (the profile falls to the neck).

### Oriented method (a back-boring cycle)

With the hole of radius r_h, its far side at z_f and its top at z_t:

| Quantity | Value |
|---|---|
| Counterbore: end of the cut (tip) | z_f + depth − h_max |
| Chamfer: end of the cut (tip) | z_f − h₀, with R(h₀) = r_h + w on the head's cone |
| Start of the cut (tip) | z_f − clearance − h_in, h_in the lowest height above h_max with R < r_h |
| Initial plane | the hole's R plane (feed height above the hole) |

The cycle (`CycleType::BackBore`): from the initial plane the spindle orients (M19 at the
operation's angle) and the bar shifts by the operation's shift away from its edge; it rapids
down through the hole, shifts back on centre below the far side, starts the spindle in its
cutting direction, feeds up to the end of the cut at the preset's cutting feed, dwells,
orients, shifts, rapids up out of the hole and shifts back. This is the sequence of the G87
back-boring cycle in Fanuc-style controls (Smid, ch. on boring cycles); `DrillCycle` carries it
with topZ = the end of the cut, bottomZ = its start, retractZ = the initial plane, and the
shift as a vector in the program's X and Y with the orientation angle.

**Passage check (refusal).** Standing still, the bar's cross-section is modelled as its head
on the edge's side only (out to R(h) along the edge's direction) and its neck all round (r_n);
above the head it is round. Shifted by s away from the edge, it fits a hole of radius r_h when,
at every height of the hole the tool passes:

  R(h) − s < r_h and r_n + s < r_h (head), R(h) + s < r_h (above the head),

so s must lie between R_max − r_h and r_h − r_n; the error names that range. Coming back up,
the room is what the cut opened. Turning on centre from the start to the end of the cut, every
part of the tool inside the hole above the feature must stay within r_h (a neck long enough).
The shift's default (shown in the panel as the value) is (R_max − r_n) / 2, the middle of the
range whatever the hole.

**Canned cycle or moves.** Posts that list `back_bore` in their manifest's cycles get the cycle
(`NcOp::Cycle` with `shift_x`, `shift_y`, `orient`; NC program version 5); `oxcam:fanuc` writes
`G98 G87 X Y Z(end of cut) R(start) Q(|shift|) P(dwell) F`. On those controls the shift's
direction and the orientation angle are the control's own settings, so the built-in generic
posts do not list `back_bore`: a machine's post lists it when its settings match. Everywhere
else `post::lower` plays the cycle out (`expandCycles`): the moves while the spindle stands
oriented carry `Move::orient`, and lowering writes `NcOp::SpindleOrient` (M19 with the angle,
`spindleOrientBlock` in `oxcam:fanuc`) before the first of them and starts the spindle again
(the toolpath's speed, as set) before the next move that is not.

### Interpolated method

- **Counterbore** (diameter D, depth d): one circle of radius ρ = D/2 − R_max. The neck stays
  in the hole (ρ + r_n ≤ r_h), so D is at most 2(r_h − r_n + R_max), and an error names that
  largest; on such a circle the head also reaches back past the hole's wall (ρ − R_max <
  r_h − r_n − R_max < r_h), so one circle always covers the ring. It starts below the far side
  (the head clear of it by the clearance), leads out on a half circle, climbs a helix of at most
  the helix pitch per turn to z_f + d − h_max, makes a full circle there and leads back to the
  axis.
- **Chamfer** (width w at angle α from the hole's axis, vertical size v = w / tan α): the head's
  upper flank must fall by w over v: h₀ solves R(h₀) − R(h₀ + v) = w on the head above h_max
  (scan and bisection), the circle's radius is ρ = r_h + w − R(h₀) and the tip is at z_f − h₀.
  The envelope of the turning tool on its circle is ρ + R(z − tip), so it passes through both
  edges of the chamfer, which are held: the far side is cut out to r_h + w and no further, the
  wall up to v and no higher. Between them the head's own shape makes the break: a cone of the
  chamfer's angle a straight chamfer, a lollipop a rounded one (a cove through the two edges).

The tool goes down the hole's axis (it must pass: R_max < r_h) at the plunge feed, leads at the
lead feed, cuts at the cutting feed.

## Approach: 3D edge breaking

### Input

`cad::edgeFaces` samples a B-rep edge evenly by length (at most the operation's point spacing
apart) and, at each point, gives the tangent, the outward normal of each of its two faces
(from the surface's derivatives and the face's orientation), and each face's direction away
from the edge across it. Which way a face lies is found once, by stepping off the edge in the
face's parameters and classifying the point on the face, and kept continuous along the edge.
Edges with one face (seams, free edges) or none come with the reason and are skipped.

### The break at one point

Work in the plane across the edge (normal t): faces a, b with outward normals n_a, n_b and
in-face directions u_a, u_b. An edge is convex when each face lies behind the other's plane
(u_a·n_b < 0 and u_b·n_a < 0); an inside corner has nothing to break.

**Cone** (half angle θ from its axis, flat tip r_t, radius R). With its axis +Z a cone touches
only planes whose normal rises at θ above horizontal, along one generator. The break's plane
must contain the edge (so that sweeping the tool along a straight edge leaves that plane), so
its normal m is the unit vector in the cross plane with m_z = sin θ nearest the faces' bisector
b = (n_a + n_b)/|n_a + n_b|: with c = t × b, m = cos φ b + sin φ c and φ solving
b_z cos φ + c_z sin φ = sin θ (the root of smallest |φ|). None exists when the edge is steeper
than 90° − θ. The plane cuts each face at s_i = −δ / (m·u_i) from the edge, δ chosen so the two
cuts are the width w apart: δ = w / |u_a/(m·u_a) − u_b/(m·u_b)|. Both m·u_i must be negative
(the plane reaches both faces).

The generator that touches the plane runs along d = (sin θ a, cos θ), a the horizontal direction
from the axis to the contact (a = −m_h / |m_h|). It crosses the break's band over a length
w / |d·q| (q across the band); its lower end sits `tipOffset` above the tip, as for
`chamfer2d` (DL-0014), so the flank cuts, not the tip. The middle of the band M lies on it:
the tip is at height M_z − h_M with h_M = tipOffset + (w / 2|d·q|) cos θ, and the axis
r_t + h_M tan θ from M along −a. The upper end must stay on the cone (radius ≤ R).

**Ball** (radius r). Its centre lies on the bisector, C = P + d_c b, with the circle across the
edge meeting both faces at the same distance s = w / |u_a − u_b| from it:
d_c = s κ + √(r² − s²(1 − κ²)), κ = u·b. Both meeting points must be below the ball's equator
(its lower half cuts); else the faces turn under it.

For a straight edge between planar faces both give exactly the requested width; along curved
edges the pose is the local one at each point, and the points are at most the point spacing
apart.

### Gouge check

The finished shape near the point is the part less the break. The tool at each point (and at
the lead points) must reach nothing else by more than the gouge tolerance:

- **The edge's own two faces** lose a band along the edge as wide as the break meets them (s_a,
  s_b), curved with the edge (distance to the edge's polyline). Their triangles under the tool
  are sampled (at most a quarter of the width apart) and every point outside the band must stay
  below the tool's lower surface lo(d) (the drop-cutter's vertex contact, DL-0016).
- **Every other face** loses only what lies inside the break's prism: between the two faces'
  tangent planes and the break's plane (for a ball, the chord between its two meeting points).
  That is what a corner where the edge ends gives up to the break. Their triangles are clipped
  to the outside of the prism (three disjoint convex pieces) and dropped against with
  drop-cutter (`DropCutter`, exact per triangle).

The cone lies entirely on the air side of its tangent plane, so faces beyond the band on the
material side are never cut by it; the check matters for nearby walls, curved faces, the far
side of a narrow hole, and the ball.

Edges where a pose cannot be made or a check fails are skipped, and one warning lists each with
its reason and the point (the edge's index as the model numbers it, as in the selection list).

### Path

Each edge is one pass from its first point to its last (a closed edge back to its first): over
at the retract height where the part stays below it along the way (drop-cutter along the line),
else at the clearance height; down to just above the part (rapid), plunge to the lead's start,
a horizontal lead of the lead length away from the part where the tool stays clear along it
(else straight down onto the edge), the points at the cutting feed, a lead out, retract. The
tool axis is +Z throughout.

## Simulation

The dexel simulation plays a back bore's cycle as its moves. Moves with `Move::orient` (the
spindle standing still) cut nothing and are not checked: a revolved shape cannot show a head
off to one side, and the strategy checked the passage itself. The shank check now follows the
tool's body band by band (`ToolShape::shankBands`): a neck narrower than the shank passes where
the shank cannot (before, the whole body above the flutes was taken at its largest radius, so a
back-boring bar's neck reported a collision in its own hole). For plain shanks the result is
unchanged.

## Sources

- P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press, 2008, ISBN 978-0-8311-3347-4:
  the back-boring cycle (G87: oriented spindle stop, shift, rapid to the R level below the hole,
  shift back, spindle on, feed up to Z, orient, shift, return to the initial level), spindle
  orientation (M19) and helical interpolation.
- Fanuc-style controller programming manuals, for the G87 and M19 block syntax only.
- B. K. Choi, R. B. Jerard, *Sculptured Surface Machining: Theory and Applications*, Kluwer,
  1998, ISBN 978-0-412-78020-2: cutter contact and cutter location for a fixed tool axis, and
  gouge checking against the part's surfaces.
- DL-0014 (chamfer geometry, tip below the chamfer), DL-0016 (drop-cutter), DL-0012 (dexel
  simulation), DL-0009 (canned cycles), DL-0039 (post manifests and cycle lists).

## Deviations from sources

- The G87 cycle takes its shift direction and spindle angle from the control's settings; the
  NC program carries both from the operation, and the built-in posts play the cycle out so the
  output is exactly what the operation shows (ADR-0004). A post may write G87 when its machine's
  settings match.
- The passage model of a standing bar (head on one side, neck all round) is a simplification of
  the real insert and holder; it is conservative for bars whose head is no wider across the edge
  than the neck.

## Correctness and tests

- **Strategies (`tests/unit/strategies/BackEdgeTest.cpp`):** the back-bore cycle's heights,
  shift, angle, dwell and feed (as set); its expansion: oriented, shifted moves down and up, the
  cut up turning, no turning rapid inside the part; JSON round trips; back chamfer height from
  the cutter's cone, and refusal with a flat-topped bar; blind holes skipped; refusals for a
  shift putting the head or the neck on the wall (naming the range), a neck too short, a head
  too wide for any shift, a non-bar oriented, a tool that does not pass; the lollipop chamfer's
  envelope through both chamfer edges; the slot cutter's counterbore circle, climbing from below
  the far side at the cutting feed, and the refusal past the largest the neck allows;
  parameters and the shift's tool default. Edge break poses: a horizontal
  square edge matches `chamfer2d`'s geometry; a sloped edge keeps the width with the plane along
  the edge at the cone's angle; a ball meets both faces the width apart; inside corners, too
  steep edges and undercuts are refused with a reason. The edge break toolpath follows the poses
  with the axis +Z at the preset's feeds; a wall next to the edge skips it with a warning naming
  it; unreachable and unusable edges are listed while the others are cut.
- **Simulation (`tests/unit/sim/BackEdgeSimTest.cpp`):** the oriented bar's counterbore (within
  2 %), the slot cutter's interpolated counterbore (2 %), the lollipop's back break (against the
  sphere's envelope, 5 %), all without collisions (the neck passing its hole); a sloped edge's
  break removes its prism (5 %).
- **Post (`tests/unit/post/BackBorePostTest.cpp`):** lowering without the cycle writes
  spindle_orient before the oriented moves and starts the spindle again (at its speed, as set)
  before the cut and after; the NC program JSON round-trips the op and the cycle's fields; the
  generic mill post's G-code has M19 R90., the shifts, S600 M03 before the cut and back-plots to
  the toolpath; a post listing `back_bore` writes `G98 G87 … Q2. P500 F1200`; the orientation
  block is a post setting.
- **Document (`tests/unit/document/BackEdgeDocTest.cpp`):** back boring the hole plate's
  through hole (heights in the WCS, the R plane), its simulation after drilling the stock (the
  ring's volume, no collisions), a refused shift; edge breaking the wedge fixture
  (`wedge_hole.step`): the rim of a hole in a slope is not flat and its faces' normals are
  right, the vertical corner is skipped and named, the slope's edge and the rim are cut.

## Known limitations

- Back boring with the oriented method needs a `BoringBar` with a neck; the standing bar's
  shape is the model above. Only the passage is checked while the spindle stands still: the
  simulation shows no cut for those moves.
- The back-boring G87 output is not back-plotted (its shift's direction is the control's
  setting), so posts that write it cannot be verified by back-plotting.
- Edge breaking keeps the tool axis +Z; edges steeper than 90° − θ for a cone, undercut for a
  ball, or with a wall in the way are skipped, not tilted around. The break along a curved edge
  is made point by point; between points the tool moves straight.
- An edge that ends where one of its faces goes on (a step's edge running into a wall) is
  skipped when the tool, at the end, would cut that face beyond the band.
