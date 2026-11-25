# DL-0018: Z-level roughing and 3D rest machining

- **Status:** implemented (phase 7). Rest variants are off by default. Closed areas are
  cleared on the circles of the medial axis now ([DL-0031](0031-medial-axis-clearing.md)); the
  traced clearing below clears open areas only (its helix entry into closed areas was
  removed), and the "clearing" choice went: the engagement limit is always kept.
- **Module:** libs/strategies (`zLevelRough`), libs/sim (`excessRegions`, exact sloped sweeps),
  libs/document (`zlevel_rough`, `parallel3d` rest mode)
- **Patent gate:** Z-level roughing is low risk (offset pocketing per level). The rest variants
  fall under [IP-0002](../ip/reviews/0002-rest-machining-from-in-process-stock.md), which is
  **pending** and must be cleared before release.
- **Author(s):** OxCAM contributors

## Z-level roughing (`zlevel_rough`)

Levels are evenly spaced from below the stock top down to the part bottom (or the stock bottom,
whichever is higher). At each level z:

1. **Forbidden area:** F = (tool centres where the real tool's drop height exceeds z −
   stock-to-leave Z), grown by stock-to-leave XY. See DL-0016.
2. **Material:** M = the stock outline, or the in-process material of the slab (see below).
3. **Centre area:** A = (M ⊕ (r + rim)) ∖ F, with rim = 0.2 r. Tool centres may leave the stock
   (open sides), never enter F, and only go where they reach material.
4. **Clearing:** A is cleared with the contour-parallel loop tree of DL-0007. Each connected
   piece of A is open or closed:
   - **Open** (its outer loop reaches a point clear of all material above the level by
     r + rim/2, which on open stock is outside the stock): cut outside in. The first entry is a
     plunge at that point, in air. Inner loops follow at depth: a node's outer loop, its
     children, then its island loops, each one stepover from a loop already cut. A loop that
     cannot be reached at depth (a sibling behind an island) is entered by a plunge into the
     slot its enclosing loop has just cut.
   - **Closed** (a pocket): cut inside out like a 2D pocket, entered by a helix at the innermost
     loop, or a ramp or plunge when the helix does not fit or the parameters ask for one.
   - A residual clean-up targets M ∩ (A ⊕ r).

Because A excludes F, every centre position is clear of the part at the level height and at any
height above it. So links, entries and loops cannot gouge.

**Open pieces by passes from the cleared side** (replaces the outside-in loop tree for open
pieces). Offsetting A itself moves its outer edge in and its holes (the part) out at the same
time; where the band between the stock's edge and the part is narrower than two stepovers there
is no loop between the outer loop (in air) and the loop along the part, which then takes the
whole band, up to a full slot. Now:

1. The air is the plane minus the level's material M; C₀ = A ∩ (air ⊖ r), the positions where
   the tool touches no material. A piece of A without such positions is closed (a pocket) and
   keeps the inside-out loops and helix entry of DL-0007.
2. Cₖ = (Cₖ₋₁ ⊕ stepover) ∩ A until it stops growing. Pass k is ∂Cₖ minus its parts on ∂A,
   clipped as open paths (geom2d::clipPaths, which keeps their direction: region boundaries keep
   the cleared side on the left, so climb). Each pass lies one stepover beyond the area already
   cleared, so the tool cuts about a stepover wide.
3. A last pass follows the parts of ∂A on F (the walls). (Following each stretch of wall as
   passes reach it lowers the width there a little but splits passes into many more pieces,
   each with its own entry, which cost more time than it saved.)
4. Pieces are cut nearest first. A pass is entered at a shallow angle (20°): from a point in air
   or on a path already cut, about a stepover / tan 20° behind its start, so the width of cut
   grows from zero to a stepover. Where there is no room for that (passes starting at a wall),
   from the nearest such point; a plunge at the start itself only if there is none. Links at
   depth go only through area cut before the pass, or close along paths just cut.

Measured on a 152 mm flange with three lobes (Ø12.7 bull nose, 5.08 mm stepover, a material
grid replay of each level): cutting wider than 1.1 stepovers outside the bore went from about
340 mm per level, mostly full slots (12.9 mm) along the lobes, to about 105 mm per level in
short stretches of 2 to 5 mm where passes turn into a wall or start next to one, some still
briefly near full width. Offset passes cannot avoid those inside corners and starts; that is
what engagement-controlled clearing addresses (IP-POLICY.md, high risk). Concentric loops in a
round pocket cut about stepover · (1 + (r − stepover/2) / ρ) wide at radius ρ (1.3 stepovers
near the bore's centre), as offset pocketing always does. See IP-0003.

**Material at the deepest level the step down allows.** Without in-process material the
levels are planned first: what the tool can reach at each level is the stock less the part's
footprint there (the forbidden area shrunk by the tool radius). A level's own material is what
it can reach but the deepest level within a step down of the top of its column cannot (over
the part's top faces, for instance); the rest is left to that deeper level, which takes a
column from the same top in one go. Levels run in the order of their column tops, the deeper
first among equals, and what a deeper column cleared counts as air for the levels after it. On
the flange with a 19 mm step down the stock around the part is cut at the lower level in one
pass instead of once per level, and the top face's level, run after it, is entered from the
cleared outside. (Evenly spaced levels one step down apart are unchanged: each cuts its own
slab, top down.) Rest roughing keeps the slab-by-slab order, since its material comes from the
in-process stock.

**Fine levels** (`fine_step_down`, 1.25 mm by default, 0 = none). With levels a step down apart, a sloped or
rounded wall keeps a stair up to a step down tall between them: on the maintainer's flange
(19 mm step down, levels at the top and the floor 13 mm apart) the radius on the top edge was
left whole to the finishing operations, and neither the 1/2" nor the 1/4" took it. Fine levels
go in above every step-down level, evenly spaced at most the fine step down apart, and take
only the band of the stair: material in the part zone of the step-down level under them (where
that level cannot go, because the part is wider there). A vertical wall gives no band; open
areas are not cut again. As for the step-down levels, each point of the band is cut at the
deepest fine level that reaches it, from the top of its column (the step-down level above) in
one go: a fine level takes the material up the column that it reaches and the level under it
(the next fine one, or the step-down one) does not, a ring along a sloped wall each, and the
stock over a face between two step-down levels all at the fine level just above the face. They
run after all the step-down levels, column by column from the top and the deepest first, so
each ring is entered from the air cleared beside it. In rest roughing the band is the
in-process stock of the column within that part zone. On the flange (fine step 1.27 mm) four
fine levels take the top edge's radius down to stairs of 1.18 mm (3400 mm³ more for the
1/2", which the contour and flat operations no longer have to take; 112 s to 147 s of
cutting); the 1/4" rest at the same fine step finds nothing left to take.

The first version gave every fine level the whole band, one fine step deep each, top down. On
the maintainer's second setup (the flange's back, rest roughing, 19 mm step down, extra levels
at flat areas off, the fine step at its default) the 9 mm of stock over the back face lay in
the part zone of the one step-down level, so eight fine levels 1.14 mm apart each took the
whole face. Each also saw it as a closed pocket: in rest roughing the in-process stock is the
one before the operation, which still held the stock around the face that the operation's own
step-down level had just cleared, so no level found air, and each was cut on the medial
circles from a helix, the circles as large as the area of allowed centres (the face and its
surround), mostly in air. Rest roughing now counts what the operation's earlier levels took
(each piece's reach, with its level's slab: the stock standing at or above a level is the
in-process stock less, slab by slab, the cuts whose slabs span it). The face is now taken
once, at the fine level above it, by traced passes from the cleared outside:

| Setup 2 rough | Moves | Levels over the face | Lifts | Estimate | Removed |
|---|---|---|---|---|---|
| Before (with the links above) | 5278 | 8 (circles) | 29 | 1539.0 s | 222149 mm³ |
| After | 1936 | 1 (traced) | 26 | 230.6 s | 222036 mm³ |

The simulation finds no collision and the gouge as before (from other operations); 113 mm³
more is left (0.05 %), the largest excess unchanged.

**Engagement-limited clearing** (`clearing`: "Engagement-limited", the default; "Offset
passes" turns the limit off). High-risk IP category, proceeding on the maintainer's decision
without counsel sign-off: IP-0004. This replaces both the loop tree for open pieces and the
offset passes of the paragraph above (src/Clearing.cpp):

1. **Material and limit.** The level's material (less the part and its allowance, which are
   never cut) is kept on a grid of cells r/32 wide, together with the allowed tool centres. The
   engagement angle at a position is the part of the circle half a cell outside the tool, on the
   half facing the motion, that lies on material not yet removed (360 samples around). The
   limit is the angle of a straight cut one stepover and a cell wide, acos(1 − (ae + cell)/r);
   the passes themselves run a cell short of the stepover, so a pass at its depth has two cells
   of margin to the limit. The measured angle jitters by a degree or two, and without that
   margin a pass at half the diameter (90°) was thrown off its line again and again, each loop
   wavier than the one before. The stepover is the preset's radial depth and the step down its
   axial depth, as for every strategy (DL-0029).
2. **Passes.** Each pass is the boundary of the remaining material grown by r − ae (where the
   tool cuts ae deep), together with the area's edge where the material there is thinner than
   that (wall passes, a cell and a half inside the edge), restricted to where the tool touches
   material and in the cutting direction (material on the right when climbing). The wall
   passes come from the edge's own loops, kept only where the material's passes do not reach:
   taken from the union of the two, the wall pass vanished under material standing at a wall,
   which only a pass along the wall can take (the tool's centre cannot go nearer), and in a
   slot two diameters wide the corners at the walls lagged further behind each pass until
   nothing could be entered. A wall pass runs only out of the tool's reach of material
   thicker than a stepover (the material less its opening by half a stepover): the body of
   the material is the fronts' to take, and a wall pass run through it slotted along the wall
   over the limit, pulling back in loops (the whole of a slot after its helix was done that
   way); the thin bands and wedges a front leaves at a wall are the wall pass's. That rule
   costs the flange's 1/2" rough about a tenth of its time (110 s against 98 s), which the
   wall slotting had been saving. A pass the tool no longer touches anywhere (taken by the
   pass before) is not entered. After each pass
   the remaining material is what the traced paths' sweeps leave, so the next pass starts from
   what was really cut. Open pieces start from the air; closed ones from a helix (or a ramp on a
   circle, or a plunge).
3. **Tracing.** A pass is not followed blindly: from an entry point in the clear behind its
   start, the tool advances in steps of r/10, each one aimed at the point of the pass as far
   ahead as the tool is off it (its next point when on it). A step is taken only when the
   engagement is within the limit at both its ends, facing its direction; otherwise the heading
   turns toward the cleared side, 5° at a time, until it is. Headings change by at most 25° a
   step, so the path turns rather than zigzags. The same rule gives the lead-in (the engagement
   rises to the limit and no further), the exact following of the pass where that is within
   the limit, and the pulling away from inside corners, where the tool runs along the material
   at the limit and the corner is taken in thinner layers over the following passes.
4. **Entries and links.** A pass is entered from a clear point behind its start and to the
   cleared side, a stepover and more out when there is room, nearer (down to just off the
   pass) when there is not: a slot two diameters wide, or the film an allowance leaves on a
   wall, has no room for the far candidates and found no entry at all. The lead-in's place
   along the pass stays the pass's start until the tool is there (the nearest part of a loop
   around the end of a slot is the far side, and measured against that the lead never ended);
   a closed loop ends where it starts, so being next to its last point counts as arriving only
   past half of it. At the end of a path the tool flows straight on into the next
   pass when it is at depth there already and close to the pass (within a stepover and a half,
   not at its end, and for an open pass near its start): the tracer's rule shapes the way over,
   and a trial that cuts nothing is taken back. (Flowing on from further away dragged the tool
   across the material and, aimed at a point behind it, back along a loop the wrong way:
   conventional cutting.) Leading in, off the pass with nothing cut yet, the tool heads for
   the start of the pass itself and its place along the pass stays the start; aimed as far
   ahead as it was off the pass, it skipped short pieces (the inside corners) for a point near
   their end. A path flowing on from the tool's last cut at depth starts with that heading:
   from the end of the path before, its first step may turn up to 90° (a corner piece needs
   the sharp turn; held to the usual 25° the L-boss test left 2.6 mm² in its corner) but not
   back on itself; out of the helix, in the air next to the material, the first step turns no
   more than any other. Started afresh there, the tool stepped straight at the material and,
   over the limit, was turned the wrong way round (the aim dead ahead of the cleared side makes
   either turn as direct; the tie now goes to the cutting direction's side), curling 2 mm
   backwards out of the helix before heading for the ring. The carried heading also took the
   kinks out of the flow from path to path: 97 s and no reversals on the flange.
   (A spiral is then one continuous path; the first version re-entered every loop from a point
   behind its start, which showed as a hook at each one.) Otherwise a pass is entered only
   where a clear point behind its start exists (a pass is never entered straight through its
   own front, which would engage both sides of the tool at once); the nearest such point to
   the tool is used, by the lowest way that is safe (see "Links between passes" below), then
   a rapid down to the feed clearance (the column over a clear point holds nothing of the
   level) and a plunge of the last millimetre. A clear point is clear of the grid by half a
   cell, and the level's material stands from the top of its column down to the level, so a
   point within the feed clearance of it is fed down from the approach height instead: the
   rapid straight down there grazed the material's edge over the column's height (0.97 mm on
   the dome's fine-level stair at 0.25 mm rays, a rapid collision). Where the
   tool has cut nothing for two stepovers, or makes no way along the pass, the path ends and
   starts again further along.
5. **Smoothing.** Each traced path is cut into an undo log first; then every inner point is
   moved to the mean of its two neighbours each side where that keeps it allowed and within the
   limit on the material as it was, and the path is cut for real. Headings come in steps of
   2.5°, and a pass held off by the limit follows the bumps of the material edge the pass
   before it left; smoothing keeps those bumps from growing pass to pass (heading change per
   millimetre on the flange's upper level halved).
6. **End.** Levels end when no pass touches material, or when a pass removes nothing; material
   left then is reported with its position when some allowed position reaches it, it is thicker
   than two cells (the in-process stock's resolution leaves films at the allowance, at the edge
   of the tool's reach) and more than r²/2 in one piece. An inside corner always keeps a sliver
   for the finishing pass: the tool's whole arc in the corner would be engaged at once; a 90°
   corner keeps r² (1 − π/4) and the film along its walls.
7. **Closed areas without room.** Where neither a helix nor a ramp fits (a hole little bigger
   than the tool), the area is left uncut, never plunged into: a plunged tool is engaged over
   its whole front at the first move, which the limit forbids, so the plunge would only mark
   the floor. The plunge entry, chosen, plunges. No warning (the maintainer's call: the tool
   was the user's choice); the area shows in the simulation.
8. **Wall inset.** The tool's centre keeps half a grid cell off the area's edge and the wall
   passes run a cell and a half inside it (a cell inside was tried: on the grid's edge half
   the points failed and the walls stalled), so a wall keeps the allowance plus up to that.
   The cell is r/32, now capped at 0.15 mm: a 1/2" tool on a 0.2 mm grid left 0.3 mm more than
   asked, and a rest pass with the same allowance found that as a film on every wall and
   traced the whole part again to take it. Growing the forbidden area by the allowance less
   the inset instead was tried and backed out: each tool's effective allowance became its
   own, and the rest pass found more, not less. The rest pass's side: stock within the
   resolution plus 0.225 mm (the largest inset) of its allowance is the earlier tool's
   allowance, not rest (`materialSlack`).
9. **Links and lead-ins.** A link at depth through clear area is a `Link` move and the way
   from an entry to the material a `LeadIn` move (at the lead feed), so the view and the post
   tell them from the cuts. The tool flows on from the end of a path into the next pass only
   from behind that pass's start and along a clear way; at the far wall of a slot it stands
   ahead of the next slice's start at the near wall, and flowing on it ground its way back
   across the slice diagonally. It goes back along the chord through the cleared side and
   enters the slice from behind its start instead: link, lead-in, arc, the trochoidal picture
   of slotting.

   Where the way over is not clear, the tool still flows on when the path traced from it
   comes within a step and a half of the pass before it has run twice the distance to the
   pass's start plus a stepover; else the trial is taken back. On the maintainer's flange the
   lobes come within a millimetre or two of the stock's sides: the pass along a side is cut off
   where a lobe's outline bulges into its way, a wall pass runs round the lobe's tip, and the
   side's pass starts again past it, about 8 mm on, across material. Refused for want of a
   clear way, the tool left the material at the end of the wall pass for an entry a stepover
   out, three millimetres off the part, and led back in where it had been (the maintainer:
   "it keeps wanting to do these little segments instead of just keep the tool engaged ... it
   pulls the cutter away from the part"). The traced way over keeps the engagement within the
   limit as every traced move does (IP-0004 unchanged); the slot's case stays excluded by the
   tool being ahead of the start. Entries left are fronts that end at the part between the
   lobes and new fronts past a lobe whose tip a pass before took: those are reached at depth
   along the cut and led into along the pass's tangent.

   | Flange rough (estimate) | Entries | Moves | Time |
   |---|---|---|---|
   | Setup 2, clear way only | 45 | 1501 | 117.3 s |
   | Setup 2, flowing on | 21 | 1474 | 113.5 s |
   | Setup 1 1/2", clear way only | 40 | 1374 | 105.6 s |
   | Setup 1 1/2", flowing on | 19 | 1381 | 102.7 s |

**Links between passes** (`LevelLinks`, src/Clearing.cpp; the traced passes and the medial
circles share it). On the maintainer's flange the 1/2" rough lifted 50 times, 46 of them
16 to 21 mm, mostly between neighbouring passes at the open side, each lift followed by a
plunge and a lead back in. The link at depth was refused because its test asked whether the
tool *touches* material anywhere along the way, sampled half a cell outside its radius: at
the end of a pass the tool always touches the band it has just left standing, so every link
failed at its first point. A link now asks only whether the tool would *cut*: whether any
material lies within its radius less a cell (the grid's resolution; the medial circles' test
already read it that way). From the tool to the next clear point, the lowest of:

1. **At depth** (`Link`, at the cutting feed, the preset's feed rate shown in its editor):
   along the straight way, or around what stands in it (an A* search over a lattice a step
   apart, the steps checked like the straight way, then straightened from each corner to the
   furthest point it sees), when the way is at most four stepovers long (as before) and the
   tool on it stays in the area or in `travel`, out of the material over the level and of
   material at or above it that is not on the grid (left to a deeper level that runs later),
   and cuts nothing. `travel` is the air around the piece: tool centres clear of the part and
   its allowance and of every material at or above the level (beside the stock where the part
   reaches the stock's edge).
2. **The feed clearance above the floor** (a rapid), when the straight way is clear like that
   but longer than a link, with the feed clearance to the side as well: no material within
   the tool's radius plus the feed clearance, and the feed clearance off the forbidden area.
   The forbidden area's edge holds material too: the stairs over a sloped part that the fine
   levels take after this one (not on the grid). The tool grazes them at depth by up to the
   grid's cell, harmless in a feed move; the first version's rapids grazed them in the 5-axis
   fixture's rough, which the simulation flagged as a rapid collision. A way around walls is
   never taken at rapid: on machines that move their axes independently a rapid cuts its
   corners.
3. **The approach height** (the feed clearance over the top of the level's material), straight
   within the area and `travel`: material of the level stands in the way.
4. **Over the retract height**: the straight way crosses the part.

No feed is invented: links at depth run at the cutting feed, as before, the rest is rapid,
and the last millimetre down is the plunge feed (from the approach height where the tool at
the end comes within the feed clearance of the level's material). Longer links at the feed would often be
slower than a lift (with the estimate's 10 m/min rapids a full lift breaks even with a link
of about 27 mm at 3.7 m/min), so the four stepovers stay.

Measured on the maintainer's flange (1/2" bull nose; the first rough at a 19 mm step down
from the top, the second setup's rough from the bottom; estimates at the preset's feeds and
10 m/min rapids):

| | Lifts | Over 6 mm | Plunges | Links at depth | Estimate |
|---|---|---|---|---|---|
| Setup 1 rough, before | 50 | 46 | 50 | 9 (67 mm) | 110.0 s |
| Setup 1 rough, after | 25 | 20 | 25 | 34 (200 mm) | 105.6 s |
| Setup 2 rough, before | 55 | 55 | 48 | 27 (623 mm) | 1542.2 s |
| Setup 2 rough, after | 29 | 29 | 22 | 53 (753 mm) | 1539.0 s |

Every lift left between points less than 22 mm apart is gone but two in the first rough: a
3 mm hop over material of the level that still stands in the way, and the retract
between two pieces of a level (each piece ends at the retract height). The rest join passes
22 to 121 mm apart across the part (full lifts) or across standing material; making those
shorter is the passes' order, not the links'. Simulated (0.1 mm rays) the setups remove the
same volume before and after, with no collision, and the same gouge as before the change
(52.5 and 54.2 mm³, from other operations: the first rough alone shows none).

**Levels and areas.** A column cut at a level is air only to the levels at or above it: an
upper level's cut leaves the slab below it standing. Counted as air, a closed pocket's lower
levels found no entry anywhere and were skipped without a word (the passes broke off when no
piece was left, before the report), so a pocket was only ever roughed one step down. With
`order` = "areas", the pieces of allowed tool centres standing over one another through the
levels form an area, and each area goes through all its levels before the next (a pocket at a
time, for a part with several); the default keeps level by level.

A first version instead kept the offset passes and patched them afterwards with trochoidal
loops wherever the engagement exceeded the limit. It could never guarantee the limit (where no
loop fit it gave up), and it put loops where none were needed; it is gone.

Measured on the three-lobe flange (152 mm stock, Ø12.7 bull nose, 5.08 mm stepover) with an
independent replay of the engagement angle on the level's material (limit 83°, a cut 1.1
stepovers wide), per level:

| Version | Travel over the limit | Largest angle |
|---|---|---|
| Loop tree (before) | 789 mm | 180° (slots along the lobes) |
| Offset passes, no limit (`clearing: offset`) | 475 mm | 139° |
| Engagement-limited | 0 to 1 mm (single samples) | 83° to 91° |

With the maintainer's cutting data for that tool (ap 19.05, ae 6.35, 7400 rpm, 3.7 m/min) the
flange roughs in 97 s against 224 s for the first version of this method: flowing from pass
to pass took it to 166 s (29 entries instead of 116), cutting the outside at the lower level in
one full-depth pass to 125 s (those two with the 0.4 D default stepover), the preset's 0.5 D
stepover to 110 s, the entry fixes above to 98 s with 20 entries, and the carried heading
to 97 s. A replay on the level's
material finds no travel over the limit beyond single samples of grid noise, and no cutting
with the material on the left (4 mm at the helix's exit into the first ring before the heading
was carried over, 177 mm before the entry fixes). Feeds are not adjusted by the strategy.

## Simulation accuracy for sloped moves

3D toolpaths are mostly short sloped moves. DL-0012 swept them in steps of a quarter cell, which
can cut up to h/4 too deep in the simulation (0.06 mm at 0.25 mm rays) and showed up as false
gouges on the dome.

The vertical rays now sweep a sloped straight move exactly for single-span tools. The lowest
point of the swept tool over a ray at p is min over t of z(t) + lo(d(t)), which is convex in t.
A golden-section search over the part of the move within reach finds it. Rays already below the
lower bound z_min + lo(d_min) are skipped. X/Y rays and multi-span tools keep the stepped sweep.

## Sources

- M. Held (1991), Choi & Jerard (1998), and DL-0007, DL-0012, DL-0013, DL-0016.

## Correctness and tests

- **Links (`[links]`):** on a grid of material, `LevelLinks` links at depth away from the
  material the tool stands next to (the end of a pass), around material in the way when the
  way is short, through `travel` round a part reaching out of the area; hops just over the
  material when the way round is too long, just off the floor when the straight way is clear
  but long, over the retract height across the part; to an entry just off the material
  (clear of the grid, within the feed clearance of it), fed down from the approach height with
  no rapid below it. Links run at the cutting feed, the plunges at the plunge feed. A round boss in a wide stock is roughed with no lift above the
  feed clearance between points within a link's length of each other, and every position at
  the level keeps the allowance.

- **Clearing tests (`[clearing]`):** the grid's engagement angle matches acos(1 − ae/r) for a
  straight cut; an L-shaped boss in square stock (an open area with a 90° inside corner) and a
  square pocket (a closed area, entered by a helix) are roughed with the engagement, replayed on
  a finer grid, within the limit everywhere away from entries, and with only the corner slivers
  left; a slot two diameters wide is cleared at a stepover of 0.8 D with nothing reported; a
  hole too small for a helix is left, with no plunge, no cut and no warning, and plunged into
  when the plunge entry is chosen; two pockets are cut level by level, or each through both
  its levels with `order` = "areas". A round boss half a millimetre from the stock's side
  (`ClearingTest`, "a round boss next to the stock's side"): the passes along the side flow on
  round it, 13 entries near the boss against 23 when only a clear way was flowed along, under
  18, within the limit and with nothing left.
- **Strategy test `Z-level roughing cuts open areas at most a stepover wide`:** a boss in a
  square stock, each level replayed on a 0.1 mm material grid. Away from entries the width of
  cut stays within 1.6 stepovers (a slot would be 2.5 here; short stretches where passes meet
  the round wall reach about 1.4).

- **Strategy test:** on a cylindrical boss, roughing centres stay at least boss radius + tool
  radius + allowance from the axis. Levels are correct.
- **Document test `3D roughing, rest roughing and finishing of the dome`:** Ø12 rough, Ø6 rest
  rough (1 mm steps), Ø6 ball finish.
  - No gouge and no collision in the simulation.
  - The rest pass removes less than a quarter of the roughing volume.
  - The final stock is close to the part (stock remains only on the base's vertical sides, which
    the parallel pass does not reach).
- **CLI `cli.project_simulate_3d`:** the dome sample project simulates with no gouge.
- **CLI `cli.project_simulate_steep_shallow`:** the dome roughed and finished steep and shallow
  simulates at 0.25 mm rays with no gouge and no collision (at 0.5 mm the rays missed the
  rough's rapid grazing a fine level's stair).
- **Clearing test `Z-level roughing takes the stock over a face at one fine level, from the
  air`:** a low round boss in a wide stock, one step-down level at the bottom, fine levels
  1 mm apart: only the fine level just above the face (and its allowance) cuts over the face,
  with no helix or ramp, from the stock and in rest roughing alike.
- **Strategy test `Z-level roughing takes the stairs off a rounded edge with fine levels`:** a
  round boss with a radius of 6 on its top edge and a step down taller than the boss: without
  fine levels nothing is cut between the top and the base; with a fine step of 1 every fine
  level down the round brings the tool as close to the round as the allowance lets it (no
  stair taller than the fine step), and no closer.

## Known limitations

- **Fine levels take time:** each band is traced as an open area (the outside of the flange
  is a long thin ring): on the flange the 1/2" rough took 30 s instead of 7.5 s to generate.

- **Roughing:** inside corners keep a sliver for the finishing pass, and each corner takes
  several short passes with their own entries. Feeds are not adjusted. IP: IP-0004, not cleared
  by counsel.
- **Plunge entry under the limit:** after a plunge every first move engages the tool's whole
  front, so with the engagement limit a plunged area is not cleared (the helix is the entry
  that works; the plunge is for areas with no room, at the user's choice).
- **A slot's two ends alternate:** the fronts advance in lockstep, pass by pass, so each slice
  at one end is followed by one at the other, with a link across the slot's cleared middle
  between them. Working one piece of material at a time finished an end before the other but
  split an open area's late pieces into many passes with their own entries (117 s against
  110 s on the flange), and was backed out; the trochoidal picture of one end at a time wants
  the fronts advanced one at a time only where the material is a closed slot.
- **Rest roughing between levels:** each level's material is the slab above it, so thin steps
  between levels are found only at the next level.
- **Rest finishing:** limited to the parallel pattern.
