# DL-0080: Holding tabs on 2D contours

- **Status:** implemented
- **Module:** libs/strategies (`contour`, `tabCentres`)
- **Patent gate:** low (citation only); leaving bridges uncut when profiling through a sheet or
  plate is long-standing shop practice (router and plasma work, printed-circuit routing)
- **Author(s):** OxCAM contributors

## Problem

A contour that cuts through the stock frees the part, which then moves under the tool or is
thrown by it. Holding tabs (bridges) are short stretches of the cut left standing at the bottom,
so the part stays attached to the stock until it is broken or sawn out. Contour2d (DL-0008) had
none.

Inputs: the contour's loops (the tool path, each closed), its depth levels, the tool radius r and
the operation's tab parameters. Outputs: the same toolpath, lifted over the tabs on the levels
that go below their top. Invariants:

- no cut move enters a tab's volume;
- every radial pass that cuts the slot leaves the same tabs;
- a tab never changes a feed (ADR-0004): every move over a tab is at the cut feed.

## Approach

### Parameters

All are contour2d `ParamDef`s, shown only when **Holding tabs** is on:

- **Tab placement:** by count (the same number per outline) or by spacing (the outline's length
  shared out evenly, round(length / spacing) tabs, at least one).
- **Tab width:** along the outline, at the tab's narrowest, which is where the tool's edge
  passes over the middle of the slot. The tool centre stays lifted over the width plus the tool
  diameter (half span w/2 + r), so the radius is accounted for.
- **Tab height:** above the contour's lowest level (its floor, including any extra depth below
  the selected bottom). The tabs' top is floor + height.
- **Tab shape:** square (straight up, across at the top, straight down) or triangular (ramps up
  and down).

### Placement

On each loop of the final radial pass (`tabCentres`):

1. n tabs at the even positions (k + 1/2)·L/n along the loop from its start, so the start, where
   the leads are, falls midway between two tabs.
2. Each tab may move by up to 40% of the spacing (41 candidates each way). A candidate's cost:
   1e6 if its lifted span comes within the lead radius of the loop's start (where the lead arcs
   run), 1e3 per corner under the span (a vertex where the path turns by more than 10°), 10 per
   segment under it that is shorter than the span (a corner arc, short wiggles), plus its
   distance from the even position as a fraction of the window (at most 1). The cheapest wins,
   so a tab moves only as far as it must.

The passes farther out (radial passes) cut the same slot, so each tab centre is projected onto
the nearest loop of every other pass, when that loop is within the pass's extra offset (plus
0.1·r + 10 chord tolerances). The spring pass repeats the final loops, tabs included.

On each loop the tab spans [s − w/2 − r, s + w/2 + r] (arc lengths) are sorted. If one still
crosses the loop's start (a short loop, or a farther pass whose start differs), the loop restarts
in the middle of the widest gap between tabs. Overlapping spans merge into one longer tab. If
the tabs cover a whole loop, that loop is not cut below their top, with a warning.

### The lift

On each level z below the tabs' top T (and only those: the levels above go round unchanged), the
loop is followed piece by piece between arc lengths, keeping its arcs (helical while z changes):

- **Square:** at the span's start a straight feed up to T, along the span at T, straight down to
  z at its end.
- **Triangular:** ramps over w/2 at each end of the span, flat at T over the middle 2r. For a
  flat-bottomed tool this leaves a triangle of material of base w and height h on the floor: at
  u along the tab, the lowest the tool went within r of u is T − h·|u|/(w/2). A level z above
  the floor meets the ramps part-way: the lift starts at w/2 · (z − floor)/h into the span.

Every one of these moves is FeedClass::Cut at the cut feed. A separate feed for them would be a
new labelled parameter; none is added.

### Simulation

The tool moves are ordinary moves, so the stock simulation leaves the tabs standing with no change
there.

## Sources

- P. Smid, *CNC Programming Handbook*, Industrial Press. Profiling practice and cutter radius
  compensation (DL-0008).
- S. Kalpakjian and S. R. Schmid, *Manufacturing Engineering and Technology*, Pearson. Sheet and
  plate cutting practice, where parts are commonly held to the skeleton by uncut bridges.

## Deviations from sources

None of substance: the sources describe the practice, not an algorithm. The placement cost and
the projection between radial passes are this project's own.

## Correctness and tests

`tests/unit/strategies/StrategiesTest.cpp`:

- "contour tabs: evenly spaced, off corners and the start": on a circle the tabs sit exactly at
  (k + 1/2)·L/n; on a rectangle starting at a corner they stay within 40% of the even positions,
  every span clear of the corners, the one already clear unmoved and the one over a corner moved
  only as far as it must; by spacing, round(L / spacing) tabs and at least one.
- "contour tabs: the tool goes over them on the lowest levels": the lift runs at floor + height,
  only on the levels below it; a square tab lifts over w + 2r with straight steps; a triangular
  one ramps at h/(w/2) and is flat over 2r; every move over a tab is at the cut feed; no sampled
  point of any feed move below the tabs' top lies where the tool's flat bottom would cut into a
  tab's volume (square or triangle, across the slot); two radial passes and the spring pass leave
  the same tabs, straight across the slot from each other; inside contours with compensation in
  control lift over w + 2r along the programmed path.
- "contour tabs: parameters": the tab settings show only with tabs on, the count or the spacing
  as placed; the reader takes the defaults from the `ParamDef`s.

`tests/unit/document/SimulateTest.cpp` "contour tabs stand in the simulated stock": around the
block fixture, four 5 × 3 mm tabs leave the expected volume standing in the 1 mm stock band, as
excess and not gouges.

## Known limitations

- The width is measured along the tool path; on a curved outline it differs from the width at the
  wall by the curvature.
- The triangle assumes a flat-bottomed tool; with a ball or bull nose the tab is larger, never
  smaller.
- Square tabs step straight down into the material at the cut feed. Triangular tabs avoid that.
- With a loop so short that no gap is wider than the lead radius, the lead arcs may touch a tab.
- No option to put tabs only on some levels: every level below the tabs' top must go over them,
  or it cuts them away.
