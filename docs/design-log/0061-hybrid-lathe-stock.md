# DL-0061: A turning stock with threads and holes in it

- **Status:** implemented
- **Module:** libs/sim (`LatheStock`, `LatheThreadCut`, `LatheHoleCut`, `latheStockMesh`), libs/document
  (`simulateSetup`, `latheDisplayMesh`, `SimulationCache`, `carriedLatheStock`), apps/opencam-gui
  (`SimulationPanel`)
- **Patent gate:** low (citation only). A helix and a cylinder as stated geometry, not a published method.
- **Author(s):** OxCAM contributors

## Problem

The turning stock is a half profile (DL-0021): exact and fast, and it cannot hold anything that is not the same all
the way round. So threads removed nothing (DL-0047), and holes off the axis had nowhere to go. The maintainer asked
for threads to show, and expected a hybrid would be needed for off-centre holes anyway. A job that stays axisymmetric
must keep today's profile and speed: dexels were what made turning simulation "incredibly slow" before.

## Options

| Approach | Thread at fine pitch | Cross hole | Axisymmetric job | Verdict |
|---|---|---|---|---|
| Convert to tri-dexels when something is not axisymmetric | Limited by rays: a 0.5 mm pitch wants 0.05 mm rays over the part | Good | Unchanged until then, then back to the slow case | No |
| Profile sections at many angles | Exact per section, stitching is hard | Topology changes between sections | Unchanged | No |
| **Profile plus exact cuts beside it** | **Exact at any pitch: it is the helix** | **Exact, mesh clipped to it** | **Literally the same code** | **Chosen** |

## Approach

**The stock (`LatheStock`).** `profile()` is what turning leaves, exactly as before. Beside it are two lists:

- **Thread passes (`LatheThreadCut`).** Where the insert went, the lead, and how much of the pass is done.
- **Holes (`LatheHoleCut`).** Tip, axis, length, and the tool's silhouette as (distance from the tip, radius).

A point is material when it is in the profile and in no cut. Removal commutes, so turning after a thread simply
subtracts from the profile and the thread's surface follows.

**The helix.** With the spindle forward the part turns the same way whichever way the tool goes, so feeding towards the
chuck cuts a right-hand helix and away from it a left-hand one. At part angle φ the tool passes z = z₀ + cφ + n·lead,
with c = ±lead/2π. The surface radius at (z, φ) is the lowest of the insert's lower outline over the passes that got
there. Flipped end for end with a setup, a helix keeps its hand.

**Taps are left as they were.** A tap's silhouette takes the tap drill's size, and its moves are synchronised, but it
adds nothing to the stock. Only a turning toolpath's passes become thread cuts.

**The surface (`latheStockMesh`).**

- *No cuts:* `revolvedMesh`, the same call as ever.
- *Threads:* the profile's loops are split at the thread's zone. Edges outside it revolve as ever. Edges inside it are
  replaced by a grid whose rows run along the helix, with a row on every corner of the insert's form and where its flanks
  leave the uncut surface. So a corner is on a row at any pitch, and facets across rows stay flat. Faces take the groove
  at their rim. Each edge is drawn once, with every thread that crosses it.
- *Holes:* the rest of the mesh is clipped by the hole's solid, refining triangles near its surface and placing the
  crossing by bisection. The wall is then added, clipped to the stock and to other holes.

Grooves are in the colour of the operation that cut them. Crests keep the base surface's colour (at size, or the cut
that made them).

**The comparison.** The profile holds no cut, so `compareProfile` reads exactly as before: **a thread cannot read as a
gouge.** Part models usually show threads at their nominal diameter (DL-0047), and even a modelled thread would not line
up in phase. The other side of that: **a thread's depth is not compared with the part.** Holes get their own check when
they are used (DL-0063).

**Playback.** A frame cuts on the profile as before. A thread pass cut part way is the same pass with a smaller `done`,
extended as playback goes, so the groove grows along the helix. Only the groove's surface is rebuilt in a frame that has one.

**Resuming from a changed operation.** Lathe setups do not resume, and they did not before: they are simulated whole
from cached toolpaths, as it takes milliseconds. Recording a cut is a list entry. The saved simulation keeps the cuts
(`kSimulationCacheVersion` 9).

**The next setup.** A stock carried to another lathe setup carries its cuts, along or flipped. A milling setup going on
from a lathe setup gets it as dexels with the cuts (DL-0064).

## Measurements

`test_document "[turnbench]"`, this machine, RelWithDebInfo (msvc-release agrees within a millisecond), other work idle:

| Case | Simulate | Display mesh | Triangles |
|---|---|---|---|
| Shaft, no thread (face, rough, finish, part off) | 13 ms | 2.5 ms | 122 k |
| Shaft with M20 × 1.5 × 20 mm, 6 passes | 21 ms | 6.9 ms | 155 k |
| Threads fixture: 30 passes, 4 threads | 53 ms | 7 to 19 ms | 143 to 216 k |

**A part with no thread did not get slower.** The same benchmark on the commit before this work (790a3b6), built the same
way, alternating eight runs each: 12.5 to 13.3 ms before, 12.7 to 13.5 ms after, mean 13.0 and 13.1. DL-0021's 5.6 ms is an
older build: the rise to about 13 ms came before this work, and this entry does not explain it.

How far a thread goes (Ø20, 6 passes), root radius found in the mesh against the depth asked for:

| Pitch × length | Mesh | Triangles | Root found | Expected |
|---|---|---|---|---|
| 0.25 × 10 mm | 15 ms | 214 k | 9.8466 | 9.8467 |
| 0.50 × 20 mm | 13 ms | 185 k | 9.6933 | 9.6933 |
| 1.50 × 20 mm | 5 ms | 65 k | 9.0799 | 9.0799 |
| 1.50 × 100 mm | 19 ms | 270 k | 9.0799 | 9.0799 |
| 0.50 × 200 mm | 43 ms | 588 k | 9.6933 | 9.6933 |

**How well a fine pitch resolves.** The form is exact across the rows at any pitch: it is not sampled by rays. What is
limited is the circle: at most 96 segments a turn, so 3.75° on the helix, and fewer on a very long thread, down to 36,
to hold about 400 k vertices per edge. On Ø20 at 96 segments the chord leaves 5 µm, under the form's size even at 0.25 mm
pitch. The volume is sampled, 24 steps a lead and 48 around: within the 0.3 % the mesh test checks.

## Correctness and tests

`LatheTest` "a thread cut beside the profile": the profile stays; removal against Pappus; the mesh encloses the volume
within 0.3 % and reaches the pass's depth; the groove follows the helix, right and left hand; the deepest pass decides; a
pass cut in parts is the pass; turning afterwards; a fine pitch; inside a bore; crossing threads (3 %: the second thread's
corners are not on rows); flipped. "A hole beside the profile": blind in the face, across the diameter, deeper is one
hole, no cuts is exactly `revolvedMesh`. `LatheHoleTest`: the shaft's thread reads no gouge, is coloured, nothing red,
survives the cache; part way is part of it; it goes on to the next setup. `gui.lathe_thread_stock`,
`gui.lathe_thread_part_way`, `gui.lathe_threads_all_kinds`.

## Known limitations

- **A tap's thread** does not show: it removes the silhouette it always did.
- **Thread depth** is not compared with a part: see above.
- **A groove against a shoulder inside the thread's zone:** the shoulder's face takes the groove at its rim, but the
  flank does not run into the shoulder's material as a true sweep would.
- **Grooves and holes:** a hole's wall does not show a groove it crosses.
