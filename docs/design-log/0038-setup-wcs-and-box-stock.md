# DL-0038: The setup's WCS from its references, and stock by shape and size

- **Status:** implemented
- **Module:** libs/document (`AxisPick`, `WcsOrientation`, `WcsOrigin`, `Setup::orientation`,
  `Setup::originFrom`, `Stock` (`StockKind`, `StockSize`, sizes, `shift`) in `Project.hpp`; `Wcs.hpp`:
  `orientedAxes`, `setOrientation`, `axisOfShape`, `pointOfShape`, `originBox`, `originReference`,
  `resolveWcsOrigin`; `workholdingPoint` in `Workholding.hpp`; `Document::modify` keeping the
  origin; `resolveStock`, `Document::modelRadius`; the tube in `stockMeshInWcs` and
  `latheGeometry`), GUI (the setup page's Part orientation, WCS origin and Stock settings
  sections; picking on the part, fixture models and the workholding in the view)
- **Patent gate:** low (citation only). Defining a work coordinate system from picked faces and
  edges and a box or point of the stock, part or fixture, and stock as a block, bar or tube
  sized from the part or typed, is how every CAM program sets up a job; nothing here plans
  toolpaths or touches speeds and feeds.
- **Author(s):** OxCAM contributors

## Problem

A setup's WCS was a frame stored as numbers: preset tool axes, an X direction, origin
coordinates in the part's frame and buttons that wrote new numbers once. It forgot how it was
made, so when the stock grew, or a vise (DL-0035) moved around a larger stock, the origin stayed
where it was: a vise on a zero-point plate, whose WCS never changes, could not keep the origin
at its bottom centre. Box stock was two corners in the project frame, awkward to size around the
part.

## Approach

### Part orientation

`Setup::orientation` says how the WCS axes were chosen: the part's own axes (the project
frame's), X and Z from picks, or Y and Z from picks. A pick is a face or edge of the part, a
fixture model or the workholding: a flat face gives its outward normal, a round face or circular
edge its axis, a straight edge its direction (`axisOfShape`); each has a Flip box.
`orientedAxes` takes Z from its pick and makes X (or Y) square to it; a pick not made yet keeps
the current axis; parallel picks are refused with a message. The picks are worked out when made
and stored with their direction and label. The workholding is placed along the WCS axes
(DL-0035: its rotation is measured from WCS X), so every edge of it already runs along a WCS
axis and turning the WCS onto one would turn the workholding with it: X never seemed to move.
A pick on the workholding therefore turns the WCS about Z by the quarter turns it asks and the
workholding back by as much (`setOrientation`), so it stays where it is around the part; a Z pick
on it is refused (it stands on WCS Z), as is an edge neither along nor across its jaws.

### Origin from

`Setup::originFrom` says where the origin is: a point of the stock's, the part's or the
fixtures' box (the fixture models and the placed workholding together), chosen by its level
(top, middle, bottom) and its position seen from above (3 x 3), or a picked point (a circle's
centre, a face's centre, a round face's axis, a vertex: `pointOfShape`; on the workholding, a
picked datum moving with its part's member: `workholdingPoint`), moved by an offset along the
WCS axes. Boxes are measured along the WCS axes (`originBox`: the stock's corners, a cylinder's
exact extent, the models' bounds, the workholding parts' B-rep bounds placed). `Document::modify`
moves `wcs.origin` there after every change (`resolveWcsOrigin`), so the origin follows the
stock, the part and the workholding. The reference does not depend on where the origin was (the
stock and the workholding are placed in the WCS and move with it), so one pass settles it; a
reference that cannot be found leaves the origin where it was and the setup page says why. Code
that sets `wcs.origin` itself clears `originFrom`. Everything that reads the WCS (generation,
the post, the simulation, the views) is unchanged.

New setups start with the part's own axes and the origin at the stock box's top front left
corner, where they started before. Setups of earlier files have neither: their axes and origin
stay as stored ("As set" on the page) until a choice is made.

### Stock by shape and size

The maintainer asked for the setup's stock in two choices, under a Stock settings heading:

- **Shape**: a box along the WCS axes, a cylinder or a tube along a WCS axis (X, Y or Z; Z by
  default), or the stock an earlier setup left (which the simulation carries as that setup cut
  it, `carryEarlierSetups`).
- **Size** (box, cylinder and tube): **Fixed** (a box's width, length and height; a cylinder's
  or tube's diameter and length), **Offset from part** (the part's extent plus a side offset
  across the stock and top and bottom offsets, or end offsets along X or Y, rounded up to a
  multiple of "Round up to") or **No additional stock** (the part's extent, nothing added or
  rounded).

A tube's inner diameter is typed for every size: OxCAM cannot know the bore of the bar bought.
Stock is centred on the part's box along the WCS axes (an offset stock where the offsets put it;
fixed stock without a model on the project origin), and **Base X, Y and Z** move it from there
along the WCS axes: 0 is centred. A cylinder's or tube's axis goes through the centre of the
part's box, and it is centred on the part along its axis too. Sized from the part, its diameter
is twice the part's largest distance from that axis, measured on the part's mesh (its vertices
lie on the surfaces, so a round part's radius is exact, `Document::modelRadius`), not the box's
diagonal, plus twice the side offset. Rounding up sizes the diameter and the length; across the
stock the extra goes equally to both sides, along WCS Z to the top, along X or Y equally to both
ends (as a box's). The page shows only the fields that the shape and size use; the shift resets
to 0 when the size becomes one from the part, and fixed stock chosen from another shape or size
starts as the stock was, where it was (a cylinder around a box: the box's corners on its circle).

Milling sees a tube by its outer outline (the strategies cut air over the bore); the stock mesh
of the simulation has the bore. On a lathe a tube is the profile from the inner to the outer
radius, so turning sees the hole; a cylinder or tube from the part on a lathe is the bar around
the part's profile, as before, with its rounding.

Stock of an earlier file placed in the project frame (a box's two corners, a cylinder's base and
any axis) stays as it is until the page edits it: the first edit turns it into the same box by
size and shift, or the same bar on the nearest WCS axis.

### Files

The project's setup gains `orientation` (`mode`, `first`, `z`: label, direction, flip, the
model face or edge) and `origin_from` (`from`, `box`, `point` or `workholding_point`, `label`,
`offset`); files without them read and write as before. The stock is `shape` (`box`, `cylinder`,
`tube`, `from_setup`) and `size` (`fixed`, `offset_from_part`, `part`) with `axis`, `box_size`,
`diameter`, `length`, `inner_diameter`, the offsets, `round_to` and `shift`; `box_min`/`box_max`
and `cylinder_base`/`cylinder_axis` are written only for stock of earlier files kept where it was.
Earlier files have one `kind` instead, read as the same stock: `offset_from_model` as a box
offset from the part, `box` as a fixed box (`box_size` and `box_shift`, or its corners),
`cylinder` as a fixed cylinder placed as it was (`cylinder_diameter`, `cylinder_length`),
`from_setup` as it was. Toolpaths of earlier projects do not change (no toolpath revision).
The setup workholding's `origin` (written briefly before `origin_from`) is read as a point on the
workholding.

## Sources

- The persistent references of DL-0005 and the workholding datums of DL-0035.

## Deviations from sources

None.

## Correctness and tests

- `tests/unit/document/WcsTest.cpp`: the part's own axes; X and Z, Y and Z, flips, a pick not
  made yet, parallel picks refused; X along a picked straight edge with Z from a face; X from an
  edge of the vise across X (the WCS turns, the vise stays where it is), Z on the vise refused; a new setup at the stock box's top front left, following a
  wider stock; the part box's top centre with an offset, then turned over (the box point and the
  offset along the new axes); a picked face's centre, a vertex, a circle's centre; saved and
  read back; a point origin with no point reported; no fixture to measure from leaves the origin
  in place; the bottom centre of the vise's box followed when the stock grows (its mount at WCS
  zero); a point picked on the vise's jaw; box stock by size centred on the part, nudged (the
  origin following it), read back, corners of an earlier file, a zero size reported; a
  workholding origin of the earlier format read as a point.
- `tests/unit/document/ProjectTest.cpp`: every shape and size resolved (a box from the part and
  moved by Base X/Y/Z, a fixed box centred, on the origin without a model; a fixed cylinder along
  X centred on the part and moved; a cylinder offset from the part about the line through the
  box's centre, rounded up with the extra on top along Z and at both ends along X; a tube with no
  additional stock along Y, a bore outside the stock refused); validation of a tube's bore; every
  shape and size written and read back, the stock of earlier files too; every earlier `kind` read
  as the same stock and written in the new keys.
- `tests/unit/document/SimulateTest.cpp`: a fixed tube around the block is a closed mesh of the
  tube's volume and simulates hollow (no material on its axis, the wall its length); the shaft in
  a tube with no additional stock is sized on its mesh (its largest diameter and length), turns
  from the bore out and simulates to the tube's volume. `gui.panel_widths` sweeps the setup page
  with its Stock settings.
- GUI: `gui.smoke_test_workholding_origin` shows the setup page of `vise_origin_project.ocam`
  (the fixture box's bottom centre); `gui.wcs_pick` picks the origin on a face of the part in the
  view; `gui.wcs_axis_pick` picks X on an edge of the part in the view (X along it).

## Known limitations

- **Milling cuts air over a tube's bore.** The stock outline the strategies get is the tube's
  outer outline; the simulation and rest machining know the bore is empty.

- **Picks are not followed.** An axis pick keeps the direction it gave when made; re-importing
  the part does not turn the WCS with it (pick again).
- **One box.** The fixture box is the box around all the fixtures and the workholding; a point of
  one fixture is a picked point.
