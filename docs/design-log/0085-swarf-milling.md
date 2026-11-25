# DL-0085: Swarf (flank) milling of ruled walls, and roughing in a tilted orientation

- **Status:** implemented
- **Module:** libs/cad (`Ruled.hpp`: `faceRulings`), libs/strategies (`MultiAxis.hpp`: `swarf`,
  `SwarfParams`, `RuledWall`; `src/Swarf.cpp`), libs/document (`swarf5x` in `Generate.cpp`),
  libs/machine (`planRotary`'s unwinding on clear rapids), apps (operation menu, walls list)
- **Patent gate:** [IP-0005](../ip/reviews/0005-swarf-and-tilted-roughing.md): low risk by the
  policy's categories, with conditions. The tool axis follows the walls' rulings with fixed,
  user-given values; nothing tilts the tool to avoid collisions. Tilted roughing's rest option
  stays under [IP-0002](../ip/reviews/0002-rest-machining-from-in-process-stock.md).
- **Author(s):** OxCAM contributors

## Problem

5-axis ball finishing (DL-0024) cuts with the tip of the tool. Drafted, tapered and twisted
walls (turbine-style blades, drafted bosses, chamfered outlines) are cut far better with the side
of a cylindrical tool laid along the wall: one pass instead of many, no scallops. This needs the
wall to be *ruled* (made of straight lines) and the tool axis to follow those lines.

Roughing also benefits from a tilt: material under an overhang or on an inclined face is reached
by Z-level roughing in a tilted frame (3+2), resting on what earlier operations left.

## Approach

### Rulings of a face (`cad::faceRulings`)

From the exact B-rep surface (STEP is the geometry source):

1. **Candidate families** of straight lines in the face's (u, v) parameter plane:
   - a plane: the lines along its steepest direction with respect to the tool axis (the axis
     projected on the plane), since a plane is ruled in every direction;
   - any other surface: the two families of parameter lines (u fixed, v fixed).
2. **Clipping.** Each line is sampled across the parameter box and classified against the face
   (2D face classifier), the ends bisected onto the boundary. A line that crosses the face more
   than once (a hole or a notch across the rulings) refuses the face.
3. **Straightness.** On nine lines across the face, the largest distance of the surface from the
   chord between the line's ends. The family within the user's tolerance is used (the one nearer
   the tool axis when both are, as on a hyperbolic paraboloid); a face straight in neither is
   refused with the distance found ("not ruled: it strays 0.8 mm from straight lines").
4. **Spacing across the face.** Lines start eight per face and are halved until, between two
   neighbours, the middle line's points and its points offset by the tool radius along the
   normal both stay within the chord tolerance of the straight move between the neighbours, and
   neighbours are at most max(0.5 mm, r) apart. The offset term matters on twisted walls, which
   are straight across (a bilinear patch is linear in each direction) while their normals turn.
5. Each ruling gives `samples` points (bottom first along the tool axis) with the face's outward
   normals, and its direction.

The selected faces are walls, except one flat face whose outward normal is within 45° of the tool
axis: that is the floor (`swarfGeometry` in Generate.cpp).

### Tool positions (`strategies::swarf`)

For a flat, bull-nose or ball end mill of radius r (corner radius rc; a ball's is r) and stock to
leave s, with d = r + s:

1. **Axis (double point offset).** A = p₀ + d n₀ and B = p₁ + d n₁ for the ruling's ends p₀, p₁
   and normals n₀, n₁; the axis is u = (B − A)/|B − A| through A. On a developable wall (plane,
   cylinder, cone) n is constant along a ruling and the flank touches the whole ruling. On a
   twisted wall the flank strays from the ruling between its ends. For every interior sample q,
   dist(q, axis) − d is measured; the largest stand-off and the deepest cut are reported, and a
   wall beyond the tolerance is refused ("the walls twist too much for the tool").
2. **Tilt limit.** A ruling further from Z than `max_tilt` refuses the operation, naming the wall
   and the position. Nothing is tilted back: tilting away from the ruling would gouge the wall.
3. **Tool end.** Without a floor the tip is A moved along the axis by the axis offset (a flat end
   square to the axis then has its lowest point on the wall's foot). With a floor (point f,
   upward normal m) the tip is moved along the axis until the tool's lowest point,
   m·(tip + rc u) − (r − rc)√(1 − (m·u)²) − rc, is the axis offset above m·f. An axis nearly
   parallel to the floor is refused.
4. **Chains.** Walls whose end rulings coincide (both ends within max(0.01, 5 × chord
   tolerance)) are cut in one pass, in any order or direction given. At a joint:
   - tangent (normals within 1°): the shared ruling once;
   - convex (the next wall's normal points along the travel): the tool turns about the shared
     ruling, the normals at each sample slerped in steps of 2 acos(1 − tol/d), so the tool end
     moves on a circle of radius d within the chord tolerance;
   - concave: refused ("an inside corner ... give the corner a radius of at least the tool
     radius"), since the flank on one wall would cut the other.
   A chain whose last ruling meets its first is a closed loop and ends where it began.
5. **Direction.** Climb (spindle M3): the wall on the right of the feed, seen along the axis;
   the chain is reversed when the sum of n · (f × u) says otherwise. Conventional the opposite.
6. **Depth passes.** Pass j of n has every tip moved up the axis by (n − j)/n of the height from
   the tip to the ruling's top: equal shares of the wall, the deepest last.
7. **Moves.** Per pass: rapid at the clearance height above the start; down to the retract
   height; plunge feed down to a point the lead distance off the wall and above the wall's top
   along the axis; plunge feed down the axis to the lead point; lead feed onto the wall; cut feed along the stations (straight moves, each with its tool axis); lead feed off
   the wall; rapid up the axis past the wall's top, then up to the clearance height. Every feed is
   the preset's own value for its class (ADR-0004): no inverse-time or rotary feed is computed by
   the strategy. (A post may; posts are exempt.) Leads go along the wall's normal at the foot
   made parallel to the floor (level without one): a twisted wall's normal can point down at its
   foot (−0.11 at the corners of the fixture's twisted boss), and a lead along it took the tool's
   end 0.2 mm into the base plate, which the simulation found.
8. **Checks (reported, nothing moved).** At every cutting position, against the part's mesh
   (BVH + GJK, DL-0025): the flute cylinder r − g from max(g, rc) above the tip to the flute
   length, plus for bull noses the flat underside, with g = tolerance + 6 µm (the wall ridden on,
   a floor touched, mesh facets); the shank with the shank clearance. Walls taller than the flute
   length are reported. The simulation's tilted sweeps (DL-0026) and the machine checks (DL-0025,
   DL-0037) apply as to any multi-axis toolpath.

### Unwinding the rotary table on clear rapids (`machine::planRotary`)

A loop round a boss turns a table's C axis a whole turn. With depth passes the second loop would
take C past its limit (±360° on the generic trunnion), and the planner, which keeps each step
nearest the last, then switched to the other solution (A negated, C + 180°) inside a cut: a swing
the solve warned about, and a gouge on a real machine. `planRotary` now takes the moves where the
rotating axis may unwind: rapids that start and end at or above the toolpath's start height (its
clearance height, where the tool is clear above the part). There it goes back by whole turns
towards zero when it stands more than half a turn away; the tool axis is unchanged. The post's
solve and the machine simulation's `rotaryTrack` use the same rule, so the simulation shows the
positions the program has.

### Z-level roughing in a tilted orientation

No new code: Z-level roughing (DL-0018) is generated in the operation's tool orientation
(DL-0036), whose frame the user picks (one of the WCS directions, "Z ⟂ face", or a custom frame).
With rest machining on, the in-process stock is simulated in that frame and sliced at levels along
its Z. The swarf fixture checks it with the tool tilted 20° about X after a 3-axis roughing.

## Sources

- X.-W. Liu, "Five-axis NC cylindrical milling of sculptured surfaces", *Computer-Aided Design*
  27(12), 1995. Flank milling with the tool offset from the ruling's points along the surface
  normals (single and double point offset).
- S. Bedi, S. Mann, C. Menzel, "Flank milling with flat end milling cutters", *Computer-Aided
  Design* 35(3), 2003. Tool positions along the rulings of a ruled surface and the error of the
  flank on non-developable surfaces.
- H. Pottmann, J. Wallner, *Computational Line Geometry*, Springer, 2001. Ruled and developable
  surfaces; the normal turning along a ruling of a non-developable surface.
- B. K. Choi, R. B. Jerard, *Sculptured Surface Machining*, Kluwer, 1998. Multi-axis tool
  positioning and orientation.
- DL-0024 (5-axis finishing and its clearance report), DL-0025 (collision checks), DL-0026
  (tilted sweeps), DL-0036 (operation tool orientation), DL-0018 (Z-level roughing and rest).

## Deviations from sources

- No optimization of the position on twisted walls (Bedi et al. and later work move the tool to
  reduce the error): the double point offset is used as it is, its error measured and limited.
  Any such optimization needs a patent review first (IP-0005).
- Rulings are parameter lines (or a plane's steepest lines), not the surface's own rulings when
  those run across the parameter lines (a hyperboloid modelled as a surface of revolution, for
  example). Such faces are refused as not ruled.

## Correctness and tests

- **Strategy (`tests/unit/strategies/SwarfTest.cpp`)** on analytic walls:
  - a cone (r 18 to 13 over 20): every cut axis is the generator's direction (to 1e-12), a tool
    radius from it, the flat end square on its foot; climb puts the wall on the right of the
    feed, conventional on the left; the loop closes; lead, plunge and cut feeds are the preset's;
  - three depth passes and a floor 2 mm below the foot with an axis offset of 0.5: three levels in
    equal shares, the last with the tilted end's lowest point 0.5 above the floor;
  - a tilt limit below the cone's 14° and a drill are refused;
  - a bilinear patch whose top edge is turned 10°: every axis runs through both offset ends of a
    ruling; the flank strays 0.002 to 0.02 mm between them (warned), and a 0.002 mm tolerance
    refuses it; the leads stay level where the patch's normal points down;
  - four upright walls of a square boss, given out of order and two reversed: one closed pass,
    one lead in, a quarter turn about each corner's edge at a radius from it; two walls meeting
    in an inside corner are refused.
- **Document (`tests/unit/document/SwarfTest.cpp`)** on `swarf.step` (tests/fixtures/gen: a base
  plate with a tapered round boss, a cone, and a square boss twisted 10° over 20 mm, its sides
  ruled lofts):
  - the cone: each cut axis parallel to the generator there (1e-3 rad) and 3 mm from it (2 µm),
    leaning in by the half-angle, climb, two depth passes, the last with the tool's lowest point
    0.1 above the base plate; one feed per feed class, all the preset's;
  - the twisted boss against its exact bilinear sides: the flank never cuts in by more than the
    0.02 tolerance and touches them; the twist (about 0.012 mm) is reported; 0.005 refuses it;
  - a tilt limit of 10°, a floor without walls, and an axis offset of −1 mm without the floor
    (the end in the base plate: reported, not avoided);
  - the tilted rest roughing: the toolpath is 3-axis in its frame and along the frame's Z (20°
    from the WCS Z) in the WCS; with rest machining it cuts a third or less of what it cuts
    without.
- **Post and simulation:** `FixturePostTest` posts `swarf_project.ocam` on the generic AC
  trunnion and back-plots it; `cli.project_post_swarf` (verify) and `cli.project_simulate_swarf`
  (no gouge, result ok).
- **Machine:** the second loop round the cone used to swing A and C within a cut; with the
  unwinding on clear rapids the post reports no swing.

## Known limitations

- **Planes are ruled along their steepest direction.** Drafted planar walls that meet at a
  slanted corner edge do not share a ruling, so each is cut on its own pass (with its own leads),
  and inside corners between such walls are only reported by the clearance check. Walls with
  corner fillets of at least the tool radius join into one pass.
- **No automatic tilt.** Rulings steeper than the limit, walls too twisted for the tool, and
  inside corners are refused rather than adjusted; the flute and shank checks report and do not
  avoid. Choosing tilts automatically is the medium-risk item of the IP policy.
- **One floor, flat.** A curved or second floor is not supported.
- **Leads** go along the wall's normal at the start and end of each pass; on a closed loop the
  lead in and out are at the same place.
- **Rest roughing with a coarse rest resolution.** On the swarf fixture a rest detection
  resolution of 0.5 mm left rapids of the rest roughing running up to 1.1 mm into rest material
  next to the bosses (12 in the tilted frame, 2 upright); the automatic resolution gives none.
  This is rest roughing's own behaviour (DL-0018), not the tilt's; the fixture keeps the automatic
  resolution.
- **The C table unwinds** only on rapids at the clearance height. A single toolpath that winds
  more than the axis range between such rapids still swings, and the solve says so.
