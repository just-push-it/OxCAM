# DL-0004: Canonical revolved tool profiles

- **Status:** implemented (phase 2)
- **Module:** libs/tooling (`toolProfile`)
- **Patent gate:** low (citation only); elementary tool geometry
- **Author(s):** OxCAM contributors

## Problem

Simulation, gouge checks and collision checks need one geometric description of every rotating
tool, independent of its catalogue type.

## Approach

Each tool type maps its parameters to an open polyline in the (radius, height) half-plane. The
polyline starts at the tip, runs outward and up, and ends on the axis at the top of the shank.
Arcs are exact (bulge segments).

**Per-type rules:**

| Type | Profile rule |
|---|---|
| Ball | Quarter arc centred at height R. |
| Bull nose | Flat bottom of radius R − r, then a quarter arc of radius r. |
| Drill, spot drill | Cone whose height is R / tan(θ/2), for included point angle θ. |
| Chamfer, engraver | Flat tip, then a cone. |
| Centre drill | Pilot cone, pilot, then a 60° countersink. |
| Lollipop | Spherical arc of radius R from the tip up to the neck radius. The sweep is π/2 + atan2(h − R, r_neck), where h is the arc's top height. |
| Form | User-supplied points. |

**Common steps for all types:**

1. Add an optional neck (radius and length).
2. Step to the shank radius.
3. Run up to the overall length and close on the axis.

Two lengths accompany the outline:

- `cuttingLength`: material can only be removed below this height;
- `maxRadius`: the largest cutting radius.

## Sources

- Elementary plane geometry.
- Tool terminology follows the general vocabulary of ISO 13399 (cutting tool data
  representation). No catalogue data was used.

## Correctness and tests

`tests/unit/tooling/ToolTest.cpp`:

- every default tool of every type produces a valid profile that starts and ends on the axis;
- the radius at sample heights matches the analytic values (ball, bull nose, drill point, stepped
  shank, lollipop).

## Known limitations

- There are no turning insert profiles yet (phase 8).
- Taper angle and back taper are not modelled.
- Thread forms are modelled as cylinders.
