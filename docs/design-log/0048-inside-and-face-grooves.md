# DL-0048: Grooves in a bore and in a face

- **Status:** implemented
- **Module:** libs/strategies (`turnGroove`), libs/tooling (`turningShape`, `turningSideOf`)
- **Patent gate:** low (citation only). Plain geometry in the lathe plane.
- **Author(s):** OxCAM contributors

## Problem

`turn_groove` found recesses on the outside diameter only.

## Approach

A **Side** parameter: outside, inside or face.

**Inside** is the outside's algorithm with radii signed so that out of the cut is up: a recess is
where the bore is wider than it is seen from the front. The tool comes in clear of the narrowest of
the stock and the part in front of the groove, and leaves along Z before it goes anywhere else. On
stock with no bore it is refused, like inside roughing and threading.

**Face.** A groove in a front face is where the part's front, seen along Z, dips below higher faces
on both sides of the radius. A step down at the outside or at a bore is not one: nothing stands on
its other side. The insert stands along Z, with its width across the radius: `turningShape(…, Face)`,
with the programmed point on its radially inner corner, tip quadrant 8 (`turningSideOf`). It plunges
along -Z from clear of the stock's front, with the same stepping and pecking.

**Refusals.** Only a grooving insert has a face form.

## Correctness and tests

`LatheStockTest` "grooves outside, in a bore and in a face", on `grooved_project.ocam`: each groove's
plunges reach its floor between its walls; the part comes out with no gouge; the program posts and
verifies. The toolpaths were plotted per operation.

## Known limitations

- **Tool reach:** a grooving tool's body has no shape yet, in a bore or a face. A face groove's curve
  (real face tools are curved to the groove's radius) is not modelled, so nothing warns of a tool for
  the wrong diameter.
