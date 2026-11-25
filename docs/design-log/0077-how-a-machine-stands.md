# DL-0077: How a machine stands, and the views that follow it

- **Status:** implemented
- **Module:** libs/machine (`MachineOrientation`, `defaultOrientation`, `Machine::orientation` and `standing()`, machine file
  version 12, the enclosure's door and roof), apps/opencam-gui (`OcctView::setReferenceFrame`'s standing, the view cube's
  names, the standard views, the simulation's first look, the machine library's Stands row)
- **Patent gate:** low (citation only): a fixed rotation of the camera's frame, chosen by the user. Nothing here makes a
  toolpath or changes the output.
- **Author(s):** OxCAM contributors

## Problem

The maintainer asked for the machine to say which way Z points, so the views turn the way the machine stands: Z up and down on a
mill, across on a lathe, up again on a vertical lathe. Until now the standard views and the view cube took the setup's WCS with Z
up. A lathe's work stood on end in the main view, while the simulation turned a lathe to its door on its own (DL-0067).

## Approach

### What the file says

`orientation` in the machine file, two signed machine axes:
- `up`, the axis that points up in the world;
- `front`, the one that points out of the machine's front, towards where the operator stands.

Each is `+X`, `-X`, `+Y`, `-Y`, `+Z` or `-Z`, on two different axes, which `Machine::validate` checks. The two pin the view down:
up gives Top, front gives Front, and their cross product gives Right. One axis alone (where Z points) does not: a lathe's X can go
either way, with the turret behind the spindle or in front of it.

Without the field, a machine stands as its kinematics says (`defaultOrientation`, the one place the default lives):
- **Mills:** `+Z` up, front at `-Y`. The world's axes are the machine's, so mills look as they always have.
- **Lathes:** `+Y` up, front at `-X`. The spindle's Z runs left to right and the turret is behind it at +X, as the lathe layout's
  model frame has it (DL-0067: the door at -X, the roof at +Y). A vertical lathe sets `+Z` up.

The generic lathes say it in their files (version 12). The mills leave it out.

### What follows it

`MachineOrientation::worldAxes` gives the world's right, away-from-the-operator and up directions as columns in machine axes. The
view takes them with the setup's WCS axes (`setReferenceFrame(axes, standing)`):
- **Standard views:** Top looks down, Front looks from the operator, Right from the right, and so on, with up the view. Looking
  down, the side away from the operator is up the view: a horizontal lathe's top view has the turret at the top.
- **The view cube** still turns with the WCS axes, so its triad names the machine's axes. Its faces are named as the machine
  stands: on a horizontal lathe its +Y face says Top and its -X face Front. A click on a face, an edge or a corner turns the view to
  that face, edge or corner of the world.
- **A change of machine** (another setup's, or the setup's machine changed) turns the view to the isometric view of the new
  standing. Changing only the WCS (another setup on the same kind of machine) turns the cube and leaves the view where it is, as
  before.
- **The simulation's first look at a lathe** is from its door with up up, in the machine's frame, whether the machine or the work
  is held still.
- **The machine library's preview** is in the machine's axes, turned as it stands. It looks from the door (lathes) or isometric
  (mills) when it first shows a machine and when the standing changes.
- **The lathe enclosure:** the walls on the machine's front and top start hidden (the door and the roof), so a vertical lathe's
  hidden roof is its +Z wall.

Setups without a machine take the generic 3-axis mill, which stands as before.

### The editor

The machine library's Machine group has a Stands row: Up, whose first choice is "As its kinematics (+Z)" (or +Y for a lathe), and
Front. While Up is as the kinematics, Front shows the kinematics' front and cannot be changed. Choosing Up writes the field. The
preview turns at once.

## Sources

No sources: the camera's frame turned by a signed permutation of the axes.

## Correctness and tests

- `TurretTest`, "how a machine stands":
  - the defaults and their world axes (right-handed);
  - the built-in lathes say theirs, the mills do not;
  - a vertical lathe read and written, and its enclosure's hidden walls;
  - a version 11 file read as before;
  - up and front on one axis refused, as is a name that is not an axis.
- `gui.machine_orientation_view` (VM), on the turret project:
  - the cube's names;
  - the Top command and the Front view, as a horizontal lathe;
  - a click on the cube's edge above its Front face;
  - the machine as a vertical lathe in the project, which turns the view and renames the cube;
  - no machine (the generic mill).
- `gui.machine_library_orientation` (VM): the demo trunnion stood with Y up and its front at +Z in its editor (not saved); the
  preview's up and the cube's names before and after.

## Known limitations

- **Upright only:** up and front are machine axes. A slant bed's X, tilted between up and back, is shown upright: X across, Y up.
- **The simulation with the machine held still** takes its standard views in the setup's WCS axes, as before. The views turn the
  setup's axes with the machine's standing, not the machine's turned frame.
- **The cube's face names** are OpenCASCADE's own English names, as before, moved to the faces they stand for.
