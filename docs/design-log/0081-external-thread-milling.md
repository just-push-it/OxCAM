# DL-0081: External thread milling, and the thread mill's direction, passes and form

- **Status:** implemented
- **Module:** libs/strategies (`threadMill`, `threadMillGeometry`, `Common`), libs/cad (`findBosses`),
  libs/document (`selectedHoles`), apps/opencam-gui (`PropertyPanel`, the selection's help)
- **Patent gate:** low (citation only). Milling a thread by helical interpolation, inside or outside, with
  tangent arc leads, climb or conventional, in radial passes, with single point or multi-form tools, is
  long-established machine shop practice described in the handbooks below.
- **Author(s):** OxCAM contributors

## Problem

`threadmill` (DL-0014) cut internal threads only, from the hole's bottom up, with the cut direction fixed by
the hand and a single or multi-form tool guessed from its flute length. The maintainer asked for external
threads on bosses and studs, on mills and on lathes with a live tool (DL-0066), with the choices a machinist
makes for a thread mill on the operation's page.

## Approach

**What is threaded.** `cad::findBosses` finds convex cylinders parallel to the tool axis, grouped by axis
line and radius as `findHoles` groups concave ones (the two share `cylinderGroups`). A boss's free end is
the one towards the tool, unless a planar face there faces away from the tool and reaches past the
cylinder's bounding square: then the boss hangs from what is above it and cannot be reached, and generation
skips it like a hole drilled from the other side. A turned part's diameters are bosses too, which is what
a lathe needs. Picking works as for holes: any face can be clicked, and generation keeps the bosses that own
a picked face; with nothing picked, the bosses within 0.2 mm of the major diameter.

**Parameters** (each with its default in its `ParamDef` only, ADR-0004):

| Key | Default | What |
|---|---|---|
| `side` | internal | Internal (holes) or external (bosses, studs, turned diameters) |
| `major_diameter` | 0 | 0 = internal: hole + 1.0825 P (ISO 68-1's minor); external: the boss's diameter |
| `radial_depth` | 0 | Thread form depth h; 0 = 0.6134 P external, 0.5413 P internal (the DL-0047 values) |
| `thread_depth` | 0 | The thread's length (the key keeps its name for old projects; the label is now *Thread length*) |
| `hand` | right | Right or left hand |
| `direction` | climb | Climb or conventional |
| `form` | single | Single point: one turn per pitch. Multi-form: one turn |
| `passes` | 1 | Radial passes in equal steps of h / n |
| `spring_pass` | off | The last pass again |
| `lead_radius` | ½ D (from the tool) | External only: the lead arc's radius, more than h |

**Helix radius** (`threadMillGeometry`, r the tool radius, pass k of n):

- Internal: major / 2 − h + k·h / n − r. The last is major / 2 − r, as before: one pass is unchanged.
- External: major / 2 − k·h / n + r. The last is major / 2 − h + r: the tool's teeth reach the minor diameter.

**Direction.** With the spindle forward (M03), climb milling goes counter-clockwise inside a hole and clockwise
around a boss, seen from the tool. A right-hand helix rises counter-clockwise. So:

| | Climb | Conventional |
|---|---|---|
| Internal, right hand | CCW, up | CW, down |
| Internal, left hand | CCW, down | CW, up |
| External, right hand | CW, down | CCW, up |
| External, left hand | CW, up | CCW, down |

Up and down cover the same span, from the thread's bottom to a whole number of pitches above it; going down
starts at that top, so the lead-in is at or above the thread's top.

**Leads.** Internal, unchanged: a semicircle from the hole's axis, tangent to the helix at angle 0. External: a
quarter circle of the lead radius ρ, centred outside the helix on its radius through angle 0, turning the other
way. It starts at (R + ρ, ∓ρ) from the axis, outside the major diameter by the tool when ρ > h, which is
refused otherwise, naming the lead radius. It leaves on the mirror quarter. The tool comes down to the
helix's start height beside the boss and goes back up there after each pass.

**Feeds (ADR-0004).** The helix at the cut feed, the leads at the preset's lead feed (they used the cut feed
until now), the way down at the plunge feed: each the user's value, unscaled. There is still no arc feed
compensation: the feed is the tool centre's, as everywhere else.

**Passes.** Each radial pass is a whole pass: down, lead on, helix, lead off, up to the R plane. The spring pass
repeats the last.

**Single point or multi-form** is now chosen, not guessed from the flute length. A multi-form tool whose flutes
are shorter than the thread is refused, naming the choice.

**On a lathe.** A thread mill is a live tool going around (DL-0066): the bosses come from the part's model along
the operation's tool axis, which on the face is the spindle's, and the toolpath goes through DL-0066's machine
unchanged: polar interpolation on any C axis, or C held still with the Y axis (`live_axes` = y), refused without
a Y axis when it leaves Y 0. The outside of a bar's front is a boss like any other. A stud standing out of the
part's side is threaded by a tool oriented across the part, C held still: the operation's frame makes its axis
the tool's, and polar interpolation is refused there as for any milling across the part.

## Sources

- P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press, 2008, ISBN 978-0-8311-3347-4. Thread milling by
  helical interpolation, internal and external, the climb direction with M03 and its relation to the hand,
  tangent approach arcs.
- E. Oberg et al., *Machinery's Handbook*, 31st ed., Industrial Press, 2020, ISBN 978-0-8311-3631-4. Thread
  milling with single and multiple form cutters; thread forms and depths.
- ISO 68-1:1998, *ISO general purpose screw threads — Basic profile — Metric screw threads*: the basic profile
  (paraphrased; no text copied). The depths are DL-0047's.
- DL-0014 (thread milling's first form), DL-0047 (thread depths), DL-0066 (live tools on a lathe).

## Deviations from sources

- Radial passes step in equal radial depths. A degressive infeed (as in turning, DL-0047) would load the tool
  more evenly on coarse pitches; it can come as a choice.
- The external lead is a quarter circle with no straight before it, and the tool comes down beside the boss
  at the plunge feed rather than with a rapid, as the space beside a stud is not known to be clear above it.

## Correctness and tests

- `HoleOpsTest`: the external helix radius (major − h + r, with h set, from ISO, and from a typed major); the
  sense and rise for all eight internal/external × hand × direction cases, and the span covered; the external
  leads' radius, tangency, sense, start and end outside the major diameter, lead and cut feeds; three passes
  and a spring pass; the lead radius not above the depth refused; multi-form one turn and its flute check;
  internal radial passes ending at major / 2 − r; the parameters read from JSON.
- `SectionFeatureTest`: bosses on the boss plate and the stepped shaft, a boss's free end seen from either side,
  and holes unchanged.
- `GenerateTest` "thread milling around a boss": found by its diameter and by a picked face, its length, a
  boss's face refused for an internal thread; the simulation removes the groove between the major and minor
  diameters down the thread and nothing inside the minor.
- `LiveMillTest` "an external thread milled on the turned diameter with a live tool": polar and C held still,
  refused without a Y axis; the whole setup simulated, no collision, the volume of the root's ring, no gouge;
  the Haas program both ways agreeing with its back-plot. "An external thread on a stud across the part, C
  held still" (`stud_bar.step`): the stud found along the oriented tool axis, its helix around the stud's axis,
  the program and back-plot, polar interpolation refused.
- `boss_thread_project.ocam` (a pocket around the boss plate's boss, then an M20 × 2.5 thread on it): its program
  against the back-plot (`FixturePostTest`), and `gui.threadmill_external_part_way`, `_all` and `_panel`, which
  show the root cut down the boss and the operation's page.

## Known limitations

- **The simulation's thread mill is a cylinder** of the tool's diameter, so it takes out the whole ring down to
  the minor diameter, not the thread's form. What it takes is treated as meant (DL-0066), so it reads as no gouge.
- **Changed behaviour of old projects.** A left-hand internal thread was milled conventionally from the bottom up;
  with *Climb*, the default, it is now climb milled from the top down. A tool long enough to be taken for
  multi-form made one turn; it now goes a turn per pitch unless *Multi-form* is chosen. The leads now run at the
  lead feed, which a preset may have left at 0: generation then refuses, naming it.
- **Tapered and multi-start threads** are not supported.
- **Not run on a machine.** Run the first external thread in the air: above all the direction around the stud.
