# DL-0052: Tailstock and steady rest

- **Status:** implemented
- **Module:** libs/cad (`revolvedAboutZ`), libs/document (`Setup::tailstock`, `Setup::steadyRest`,
  `latheSupports`, `fixtureBodiesInWcs`), apps/opencam-gui (`PropertyPanel`, `MainWindow`)
- **Patent gate:** low (citation only). Plain geometry.
- **Author(s):** OxCAM contributors

## Problem

A long shaft on a real lathe stands in a tailstock centre or a steady rest. OxCAM could only have them
as an imported STEP fixture, placed by hand, and nothing said they were there otherwise.

## Approach

**Parametric bodies on the setup.** A lathe setup can have a tailstock (a centre's point at a Z, its
diameter and included angle, a quill's diameter and length) and a steady rest (at a Z, width, what it
grips, outer diameter). Both are in the setup's WCS, and both revolve about the spindle axis. A steady
rest gripping 0 closes on the stock there, as the carried stock has it (DL-0044).

**They are fixtures from then on.** `latheSupports` makes solids of them (`revolvedAboutZ`), and
`fixtureBodiesInWcs` tessellates them with the chuck, so what already held for a chuck holds:

- The insert and holder checks refuse a move that reaches them, naming the body.
- A body behind the part's front is a stop for outside passes: rough and finish stop in front of a
  steady rest, with the warning that the part behind it is not turned. A Z range or another setup does
  the rest (DL-0049, DL-0044).
- The 3D view draws them with the workholding, and the simulation reports collisions with them.

**The setup is the unit.** A tailstock stays in for the setup. Facing and centre drilling come before
it goes in, so they belong to an earlier setup, and the tailstock's setup continues that stock: the
same WCS, stock from the earlier setup. A facing operation under the tailstock is refused.

## Correctness and tests

`LatheStockTest` "a tailstock and a steady rest are kept clear of", on `centres_project.ocam`: turning
on the tailstock runs with no collision from the first setup's stock; facing and an oversized quill are
refused; passes stop in front of a steady rest, with the warning; the project keeps them; on a mill
they are nothing. `gui.lathe_simulation_tailstock` draws them. The program posts and verifies.

## Known limitations

- **The approach:** a tool's start and rapids take no account of the tailstock, so a wide quill close
  to the front is refused rather than driven around. A narrower quill, or the centre further out,
  settles it.
- **M-codes:** the programs do not advance the quill, or open and close the jaws: they are a post's to
  write for the machine.
- **Gouge against the part:** a centre hole shows as a gouge, since the part model does not have one.
- **Live centres:** one shape stands for both, a dead centre's.
- **Not done:** sub-spindles and bar feeders. A sub-spindle takes machine kinematics for a second
  chuck and a part transfer (planned with mill-turn), and a bar feeder is M-codes and a stock length
  rule: both change what a program is, not a toolpath.
