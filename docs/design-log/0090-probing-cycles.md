# DL-0090: Probing cycles with a spindle probe

- **Status:** implemented (first pass)
- **Module:** libs/tooling (`ToolType::Probe`), libs/toolpath (`MoveKind::Probe`, `ProbeCycle`),
  libs/strategies (`Probing.cpp`), libs/document (`Probing.cpp`), libs/sim (`Simulate.cpp`),
  libs/post (`probe` instruction, `oxcam:fanuc` hooks), apps/opencam-gui
- **Patent gate:** low (citation only). The cycles are the textbook touch patterns of a
  touch-trigger probe (one face, two opposite faces, four points on a circle, two faces of a
  corner); nothing here models a vendor's cycle or macro program.
- **Author(s):** OxCAM contributors

## Problem

A spindle probe (a touch-trigger probe held in the spindle like a tool) measures the part on the
machine: to set a work offset from where the part really is, and to check a feature in process.
OxCAM needs:

- a probe in the tool library, with what the motion needs: the stylus ball's diameter, the
  stylus length and the effective trigger offset found by calibration;
- operations for the usual cycles: one surface (X, Y or Z), a web or pocket width, a bore or boss
  centre, an outside or inside corner, and a feature check against the model within a tolerance;
- each cycle's own record of what it measures, its nominal values (filled in by picking the faces
  on the model) and the work offset it sets;
- probing moves of their own kind in the toolpath, so the simulation shows them, never shows them
  cutting, and checks the stylus, body and holder like a tool;
- probing in the post API (ADR-0008), with a generic output and hooks for the probe's macros.

## Approach

### The probe

`ToolType::Probe` (family `Probing`) reuses the rotating tool geometry: `diameter` is the stylus
ball, `fluteLength` the stylus length (ball tip to probe body), `neckDiameter` the stylus stem,
`shankDiameter` the body, and the new `MillGeometry::triggerOffset` the effective trigger offset:
how far the ball travels past first contact before the probe triggers, as calibration finds it.
The effective radius is the ball's radius less it (`effectiveStylusRadius`). The revolved profile
is a ball on its stem up to the stylus length, then the body; the stylus counts as the "cutting"
part for the simulation's checks and the body as the shank. A probe has no cutting preset.

### Cycles

Five operation types (`probe_surface`, `probe_width`, `probe_circle`, `probe_corner`,
`probe_check`) in a **Probing** group. Each generates one `toolpath::ProbeCycle`:

| Feature | Nominal | Direction | Touches |
|---|---|---|---|
| Surface | the touch point on the face | the way the stylus moves (+-X, +-Y, -Z) | one |
| Web / pocket | the centre (Z: the ball centre's height) | the axis measured across | two, opposite |
| Bore / boss | the centre | — | four, at 0, 90, 180 and 270 degrees |
| Outside / inside corner | the corner | the stylus's move on each face (+-1, +-1) | two, `inset` along each face |

A touch of surface point S moving along unit d starts with the ball's centre at
S - (re + approach) d and stops, nominally, at S - re d (re: the effective radius); the toolpath
runs the tool tip, a ball radius below the centre. Every move below the retract height is a probe
move: a protected positioning move at the positioning feed, or the measuring move at the
measuring feed. Inside features (pocket, bore, inside corner) stay down and go through a hub point
between touches; outside features go up to the retract height and over by protected moves, so a
cycle is one unbroken run of probe moves. Only the move into the first retract position and the
final retract to clearance are rapids.

The parameters every cycle has: positioning and measuring feeds (`ParamKind::Feed`, mm/min, their
defaults in the ParamDef only, ADR-0004), approach distance, overtravel (how far past the nominal
surface a measuring move looks), and either the work offset to set (0: the setup's own) or, for a
check, the tolerance. A feed of 0 refuses the toolpath by name; nothing stands in for it.

### Nominal values from the model

`document::probeNominals` reads the picked faces in the setup's WCS: one planar face is a surface
(the stylus moves against its outward normal; a face looking down is refused), two parallel
faces a web (normals away from each other) or a pocket (towards each other), two faces square to
X and Y a corner (outside when the X face runs from the corner away from where the Y face looks),
one cylindrical face along Z a bore or, when the part is on its axis, a boss. The touch point or
measuring height is the face centroid (a cylinder: the middle of its axis). Faces must be square to
the WCS within 1e-3.

Picking faces in the GUI writes these values into the operation's parameters, where the user sees
and can change them; the toolpath uses the parameters. When the picked faces later give other
values (the model changed, or a value was typed over), generation says so in a warning.

### Simulation and collisions

Probe moves never remove material. Each is checked with the stylus shrunk by `probeAllowance`
(0.05 mm plus the probe's trigger offset, sideways and along Z) against the stock: material there
is a `CollisionKind::Probe` (the stylus meets stock where no touch is expected, for example stock
not yet machined to the face it measures), and the body and holder are checked as a shank and
holder. Machine and fixture checks see the probe as any tool.

### Post (ADR-0008)

Lowering turns each cycle's run of probe moves into one `probe` instruction: the cycle and its
moves, each measuring move with its `target` (the nominal stop plus the overtravel along the move)
and `expected` (the nominal stop). The generic Fanuc-style base writes each move as a skip move
(G31, a setting) between named hooks. Plain G-code cannot apply what the probe measured, so (the
maintainer's decision) a cycle that sets a work offset is refused unless the post overrides
`probeEnd`, naming the operation; a feature check posts its moves, with a comment and one warning
that the check is not made. A machine's post overrides the hooks to call its probe's macro
programs.

## Sources

- ISO 230-10, *Test code for machine tools — Part 10: Determination of the measuring
  performance of probing systems of numerically controlled machine tools*: the terms of
  touch-trigger probing on a machine tool (no text taken).
- Fanuc-style controller programming manuals: the skip function G31 and the macro call G65
  (syntax facts only). No probe maker's macro program, cycle or argument list is used.

## Deviations from sources

None: the touch patterns are the geometric minimum for each feature. Cycles in probe makers'
software do more (several measuring passes, slower second touch, angle measurement); those belong
to the macros a post calls.

## Correctness and tests

- `test_tooling` `[probe]`: the default probe is valid, has no preset, its profile is ball, stem,
  body; bad trigger offsets and stems are refused; the trigger offset round-trips in libraries.
- `test_toolpath` `[probing]`: names, JSON round-trip, comparison, durations, statistics, sampled
  runs and frame changes of probe moves and cycles.
- `test_strategies` `[probing]`: every feature's touch points (stop positions to 1e-9), rapids
  only above the retract height, hub points inside, refusals (not a probe, zero feeds, a retract
  height below the start, features too small for the stylus).
- `test_document` `[probing]`: nominals from the block, hole and boss fixtures; the setup's work
  offset; the warning when values differ from the faces; the simulation cuts nothing and reports
  the stylus meeting uncut stock.
- `test_sim` `[probing]`: touches cut nothing and report nothing; a protected move into stock is a
  `Probe` collision.
- `test_post` `[probing]`: one instruction per cycle with its overtravel targets; the JSON form
  round-trips; the generic post refuses a cycle that sets a work offset, and posts it once
  `probeEnd` is overridden; feature checks give the golden program
  `tests/golden/posts/mill-probing.nc` and one warning; a post's hooks replace the skip moves.

## Known limitations

- Milling machines only, along the setup's Z: no probing on a lathe, with a tool orientation of
  its own, or of faces not square to the WCS.
- Bores and bosses are touched at four fixed angles; no start angle or point count yet.
- No angle (rotation) measurement, no tool setting, no second slow touch.
- Probe moves are not rerouted around the workholding (they are checked in the simulation).
- The generic posts do not use the measurement: they refuse cycles that set offsets and write
  checks as moves only. A post with the probe's macros is needed.
