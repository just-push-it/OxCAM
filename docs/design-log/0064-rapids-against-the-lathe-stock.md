# DL-0064: Rapids against a lathe's stock

- **Status:** implemented
- **Module:** libs/sim (`simulateTurning`, `latheStockDexels`), libs/document (`simulateLatheSetup`, `lastStock`)
- **Patent gate:** low (citation only). Checking a rapid against the material is what DL-0035's milling check already does.
- **Author(s):** OxCAM contributors

## Problem

The maintainer asked why rapids across the part were not checked. Looking at it, on a lathe they were not, and it was
worse than DL-0063 said:

- A **turning insert's** rapid through the bar swept material away like any other move, silently. Only the tool's body
  (DL-0044's holder check) was looked at.
- A **live tool** was checked against the fixtures only: its rapids, shank and holder never met the stock.

## Approach

**A turning insert.** `simulateTurning` sweeps every move, so at a rapid it looks first: the insert's hull along the
rapid against the profile at that moment. More than 0.001 mm² of overlap reports a `Rapid` collision, and the sweep goes on
as before, so the stock shown still has the damage. The threshold is for an insert resting on a wall it has just cut, whose
overlap is a rounding error. It is reported with the operation's other collisions, in the same list and panel.

The check is against the profile. A thread or a hole off the axis is not in it, so a rapid through a thread's groove reads
as through material. Threading's rapids stand outside the thread, so this has not shown in practice.

**A live tool.** It does not sweep the lathe plane, so a copy of the stock goes to dexels (`latheStockDexels`) and the
milling simulation runs its toolpath on that: rapids, shank and holder, as on a mill, and only the collisions are kept.
The exact stock (DL-0061) still takes the holes. A round stock converts ray by ray from the profile, and one with cuts by
its mesh, so an earlier hole counts as empty space.

**Going on to a milling setup.** `lastStock` uses the same conversion, so a later milling setup now starts with the
lathe's threads and holes instead of a warning that it leaves them out. A thread finer than the dexel spacing does not
resolve there.

## Measurements

The axisymmetric shaft of DL-0061, RelWithDebInfo: 12.9 to 14.0 ms over six runs against 12.7 to 13.5 ms before, within
what runs vary by. The shaft with M20 × 1.5: 22 ms, against 21 ms. A setup with live tools adds one conversion and one
simulation per live operation, at the stock's resolution.

## Correctness and tests

`LatheTest` "a turning insert's rapids against the stock": clear, through the bar at the right move, and resting on a wall
just cut. `LatheHoleTest` "a live tool's rapids against the stock": the fixture goes clear and a rapid across the part is
reported; the dexel copy's volume against the exact stock's within 1%.

## Known limitations

- A rapid's check is the straight line in the WCS. A C axis turning during a rapid moves the part along an arc; DL-0066
  says when that happens.
- Rapids through a thread's groove or a hole off the axis on a turning insert read as through material.
