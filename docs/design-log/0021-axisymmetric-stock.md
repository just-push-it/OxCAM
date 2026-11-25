# DL-0021: Axisymmetric stock simulation for turning

- **Status:** implemented (phase 8)
- **Module:** libs/sim (`LatheStock`, `simulateTurning`, `revolvedVolume`,
  `Stock::fromRevolvedProfile`), libs/document (lathe branch of `simulateSetup`,
  `simulatePartial`)
- **Patent gate:** low (citation only)
- **Author(s):** OxCAM contributors

## Approach

- **Stock.** The turning stock is its half profile in the lathe plane (u = Z, v = radius ≥ 0),
  as polygon regions.
- **Removal.** A turning insert is convex, so the area it sweeps along a straight move is exactly
  the convex hull of its outline at the start and the end.
  - Hulls are collected (rapids included, arcs sampled) and subtracted from the profile with
    Clipper in batches of 256.
  - Threading toolpaths are skipped. The helical form cannot be represented axisymmetrically, and
    part models show threads at their nominal diameter.
- **Volume:** 2π ∫∫ v dA. Per loop, (1/6) Σ (u_i v_{i+1} − u_{i+1} v_i)(v_i + v_{i+1}).
- **One-way conversion to the tri-dexel stock** (`Stock::fromRevolvedProfile`). Every ray is
  filled exactly from the profile:
  - Rays along the spindle axis take the Z intervals of the profile at radius √(x² + y²).
  - Rays along X (or Y) at offset c take, for each annulus [r₁, r₂] of the profile at their Z,
    the parts where c² + t² lies in [r₁², r₂²].
  - Display, part comparison and later milling (mill-turn, phase 10) then use the dexel stock.
- **Simulating a setup.** `simulateSetup` simulates lathe setups on the axisymmetric stock and
  converts after every operation. Checkpoints keep both forms. Playback inside a turning
  operation replays it from the operation's axisymmetric checkpoint and converts the result.

  *Amended 2026-11-01.* The profile is the exact stock and the smooth display draws it directly
  (DL-0040), so with that display nothing needs the dexels, and they were nearly all of the cost.
  `SimulationOptions::turningDexels` (on by default) leaves them out:

  - **Simulating.** `initialStock` and `stockAfter` stay empty; the operations' volumes come from
    the profile as before. A setup whose stock comes from this one still gets its dexels
    (`sourceOptions`), and so does the cell display, which simulates again when it is chosen.
  - **Comparing.** `sim::compareProfile` works in the lathe plane: the excess is the stock outside
    the part grown by the tolerance, the gouge is the part shrunk by the tolerance that the stock
    no longer holds. Volumes come from the revolved volume of those areas and the deepest values
    from their vertices' distances to the part's outline. It is exact, and costs well under a
    millisecond a frame on the shaft.
  - **Playback.** A frame cuts on the profile from the previous frame, compares and revolves the
    profile for display; the panel no longer touches rays or the dexel display.

  Measured with `test_document "[turnbench]"` on the shaft fixture (face, rough at 0.5 mm, finish,
  part off, 0.05 mm rays, RelWithDebInfo): simulating 836 ms with dexels and 5.6 ms without; with
  the bar 300 mm longer, 1364 ms and 5.6 ms. In the GUI (5 s of playback from roughing) a frame
  went from about 60 ms to about 17 ms, of which about 12 ms is drawing: 82 frames became 297. The
  cell display costs what it did (about 165 ms a frame).

## Sources

- Pappus's centroid theorem (volume of revolution).
- A. M. Andrew, *Another efficient algorithm for convex hulls in two dimensions*, Information
  Processing Letters 9(5), 1979. The monotone chain hull.
- DL-0012 (dexel stock).

## Correctness and tests

- **Volumes:** bars, tubes and the stepped shaft have exact revolved volumes. The dexel
  conversion is within 0.5 % on all three grids at 0.1 mm.
- **Strategy simulation tests:** see DL-0020.
- **CLI `cli.project_simulate_lathe`:** the shaft sample project (face, rough, finish, thread,
  part off) reports no gouge.

## Known limitations

- **Display:** the Z rays run along the spindle axis, so the viewer's height field shows the part
  from its front, and the comparison's excess is measured axially. Turned diameters left
  oversize show as thick (blue) columns. A radial colouring belongs with the planned wall
  colouring from the X/Y grids. *Since DL-0040 this applies only to the cell display. The GUI
  draws lathe stock as its revolved profile, compared radially in the lathe plane.*
- **No collisions:** there is no holder or chuck collision check.
