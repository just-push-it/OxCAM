# DL-0034: Default operation heights

- **Status:** implemented
- **Module:** libs/document (heights), apps/opencam-gui (heights rows)
- **Patent gate:** low (no algorithm; conventions of CAM practice)
- **Author(s):** OxCAM contributors

## Problem

An operation has five heights: clearance, retract, feed, top and bottom. Each one is a
reference plus an offset. Until now, a "Default" reference meant something different for each
height and each strategy. Clearance and retract came from the setup, feed was 1 mm above the
material, and top and bottom were whatever the strategy chose. The interface showed only
"Default", so nobody could see what an operation would do without generating it.

## Approach

Every height is explicit. `defaultHeights(op)` gives a new operation its heights by type:

| Height | Default |
|---|---|
| Clearance | retract height + 0.2" |
| Retract | stock top + 0.2" |
| Feed | retract height; hole operations: each hole's top + 2 mm (the R plane) |
| Top | stock top for roughing (facing, 2D pocket, plunge roughing, 3D roughing); part top for finishing (2D contour, 3D finishing) |
| Bottom | part bottom; the selected floor or outline (2D pocket, plunge roughing, 2D contour); the part top (facing) |

A `Default` setting can still come from files of earlier versions. `effectiveHeights(op)`
replaces it with the type's default, both for generation and for display. The interface never
shows "Default". Editing any height stores all five explicitly.

The setup's own clearance and retract heights are no longer used. The project file keeps them
so that older files still load.

**Clearance from retract.** The clearance height may now be measured from the retract height.
The two are resolved in the order they refer to each other. Measuring each from the other is
an error.

**Feed height on entry.** The strategies enter every level a small clearance (1 mm) above the
material, as before. After a strategy runs, the document layer looks for rapids that come
straight down from above the feed height into the operation's top, to enter its first level.
Each such rapid stops at the feed height, and the rest of the way is fed. Entries to lower
levels keep the strategy's clearance. Before this change, the feed height set that clearance
for every level. With the feed at the retract height, each step down would then have fed about
5 mm of air.

**Holes.** The strategies' "R plane above hole" parameter is now the feed height, measured
from each hole's top (`HeightRef::HoleTop`). A feed height measured from anything else puts one
R plane over all the holes, as before. An earlier version's `retract_clearance` parameter
becomes the default offset.

## Correctness and tests

`GenerateTest` "operation heights and per-operation cutting values" checks:

- the default values and the per-type references;
- that a Default setting equals the type's default;
- the earlier versions' R plane parameter;
- that a pocket's feed-height plunges go into the top level only;
- that clearance and retract measured from each other fail.

Drilling checks that the default R plane is 2 mm above the hole's top. `kToolpathRevision` is
2, so cached toolpaths are generated again.

## Known limitations

The 0.2" allowance is the same in metric projects (5.08 mm). Edit a height to change it. For
3D finishing contained to selected faces, top and bottom are now the part's top and bottom,
not the selected faces' range. The XY containment still applies, but levels above or below
the faces inside it are cut too.
