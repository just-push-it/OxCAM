# DL-0020: Turning strategies, insert outlines and nose-radius compensation

- **Status:** implemented (phase 8)
- **Module:** libs/tooling (`turningShape`), libs/strategies (`Turning.hpp`), libs/cad
  (`axialProfile`), libs/document (`latheGeometry`, `turn_*` operations)
- **Patent gate:** low (citation only). These are the classic lathe cycles: facing, axial
  roughing with a profile pass, contour finishing, plunge grooving, multi-pass threading and
  parting off.
- **Author(s):** OxCAM contributors

## Coordinates

A lathe setup's WCS Z axis is the spindle axis, with the part's front at its largest Z.
Strategies work in the lathe plane (u = Z, v = radius). Toolpaths use the common IR with
x = radius, y = 0 and z = axial position, so statistics, JSON, solve and post apply unchanged.

The part's half profile is `cad::axialProfile`: the model is sectioned by the WCS XZ plane and
the half with x ≥ 0 is kept. It is done by turning the shape so that plane becomes Z = 0 and
reusing the planar section.

The stock is a bar:

- **Cylinder stock:** on the WCS Z axis, used as it is.
- **Other stock kinds:** the part's envelope grown by the side, top (front) and bottom (back)
  offsets.

## Insert outlines

`tooling::turningShape` gives a closed convex outline relative to the programmed point:

- **Turning inserts (ISO shapes C, D, K, S, T, V, W):** a rhombus with the insert's included
  angle α, as on a 93–95° holder.
  - The trailing (end) edge leaves the nose at max(5°, 85° − α) above +Z, and the leading edge
    follows at α.
  - The nose is rounded to r_n. The programmed point is the imaginary tip, so the nose centre is
    at (r_n, r_n).
  - Round inserts (R) are circles.
  - Inside (ID) tools are mirrored in the radius.
  - *Amended 2026-11-01.* The outline is the shape's own, not always a rhombus. It walks from the
    nose along the shape's sides around the inscribed circle: triangles (T) and the regular
    pentagon, hexagon and octagon (P, H, O) with side ic · tan(π/n), trigons (W) with six equal
    sides of (ic/2)(tan 50° + tan 10°), rhombi with side ic / sin α. Rectangles and parallelograms
    (L, A, B, K) stay rhombi on the circle, as their length is not in the tool. The nose and both
    cutting edges, all the toolpaths use, are as before; the simulation sweeps the true shape.
- **Grooving and parting inserts:** the insert width with rounded bottom corners. The programmed
  point is the left (−Z) bottom corner.
- **Threading inserts:** a 60° form with the programmed point at the tip.

## Nose-radius compensation (in the computer)

A round nose must stay outside the part grown by the allowance a, so its centre stays outside
C = part ⊕ (a + r_n), computed with a polygon offset.

Programming the imaginary tip (centre − (r_n, r_n) for OD tools) makes straight axial and radial
cuts land exactly on their targets. Tapers and radii are compensated by the offset. G41/G42
compensation in the control is not used for turning.

**Contour path.** The finish and profile passes follow the outer envelope g(z) of C from the
front to the back.

- g is piecewise linear between C's vertex Z values, with jumps at walls. It is evaluated just
  in front of and behind each breakpoint.
- The path is limited so it never descends towards −Z faster than the insert's trailing edge
  allows: path(z) = max(g(z), previous − tan(trailing angle) · Δz), with exact crossing points.
- Rising faces (shoulders) are climbed by the leading edge (95° lead).
- Inside paths use the bore's inner envelope, mirrored.

## Strategies

| Operation | What it does |
|---|---|
| **Face** | Evenly spaced axial levels from the stock front to the part front (+ allowance). Each feeds radially past the axis by the overshoot **plus r_n**: the nose centre must cross the axis, or its arc leaves a nub (0.06 mm with r_n 0.8 and 0.5 overshoot). Tubes stop short of the bore. |
| **Rough (OD/ID)** | Evenly spaced radial levels down to the allowance (see below). Ends with a **profile pass** along the allowance, which removes the steps the level passes leave (at most one depth of cut), so finishing meets a uniform allowance. Inside roughing needs a bore in the stock. |
| **Finish (OD/ID)** | The contour path with zero (or the given) allowance, with a lead-in from the front and a radial lead-out. |
| **Groove** | Recesses are where the part lies more than 0.01 mm below its profile seen from the front (the running maximum, i.e. the contour path with zero descent). Each is plunged from its back wall forward, at most the plunge step apart (80% of the insert width unless set); each plunge goes down to the highest floor under the insert. Pecking is optional. Grooves narrower than the insert are reported. |
| **Thread** | ISO depth 0.6134 P by default, degressive radial infeed (depth · √(k/n)), spring passes, a run-up of 3 P in air, and constant rpm. Each pass is a spindle-synchronized `Move::pitch` move (posted as G32). |
| **Part off** | The insert's right face cuts the part's back face. It plunges past the axis, with the preset's part-off end feed over the last `slow_distance` (DL-0054) and optional pecks. |

**Roughing levels in detail.** Each level's pass runs from the front towards −Z until the nose
reaches C. A level exactly on the allowance is kept by testing just off the boundary. Material in
front of the part's front face is left to facing. Levels with no material between them and the
previous level are skipped.

**Feeds.** Turning feeds come only from the insert's cutting preset: its surface speed, and its
feed per tooth read as feed per revolution. There are no built-in defaults. An operation without a
preset, or whose preset lacks the speed or (except for threading, which feeds by the pitch) the
feed, fails to generate with a message naming the tool. New tools get a starting preset,
"Default (check before use)", that the user checks against the insert vendor's data.

- spindle = min(vc / (π · d_stock), limit);
- cut feed = f · spindle.

Constant surface speed (G96 with a G50 limit) is on by default. Threads always use constant rpm.

## Sources

- P. Smid, *CNC Programming Handbook*, 3rd ed., Industrial Press, 2008. Covers lathe cycles,
  imaginary tool tip programming, nose-radius compensation, threading infeed and G32.
- ISO 1832:2017, *Indexable inserts for cutting tools — Designation*. Shape letters and included
  angles (paraphrased).
- ISO 68-1:1998 (basic profile of metric threads) and ISO 965-1. External thread depth.

## Correctness and tests

- **Insert outlines (`tests/unit/tooling`):** the nose touches the tip box, the outline is
  counter-clockwise, ID tools are mirrored, and the groove width is right.
- **Axial profile (`tests/unit/cad`):** the shaft's half profile has the right area and bounds.
- **Simulation (`tests/unit/sim/LatheTest.cpp`):**
  - Face, rough and finish of a stepped shaft: no gouge. Only the shoulder fillet r_n²(1 − π/4)
    and the bar behind the part remain.
  - A 20° taper finished with a D insert: no gouge, nothing left on the taper.
  - Grooving clears a recess to within the insert's corner radii.
  - Threading: degressive depths and spring passes.
  - Parting off separates the part.
- **Document (`tests/unit/document`):** the shaft fixture on the generic lathe (face, rough,
  finish, part off):
  - no gouge;
  - the remnant behind the part-off cut is the only excess;
  - playback works;
  - the lathe post back-plot verifies.

## Known limitations

- **Arcs:** turning toolpaths contain straight moves only (compensated arcs are sampled at the
  chordal tolerance). G2/G3 in the XZ plane is not used yet.
- **Undercuts and back faces:** these are not roughed (Type I roughing). Grooving handles
  rectangular and sloped recesses.
- **Raw stock:** roughing assumes the raw bar, not the in-process stock. Passes over a faced front
  cut air. In-process turning stock would fall under IP-0002.
- **Threads:** no flank or alternating infeed, multi-start threads, internal threads or tapered
  threads yet.
- **Holder collisions:** the insert holder is not checked against the part.
