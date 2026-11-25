# DL-0026: Material removal by tilted tools on the dexel stock

- **Status:** implemented (phase 9)
- **Module:** libs/sim (`ToolShape::cutterPieces` / `shankPieces` / `holderPieces`,
  `src/Tilted.cpp`, tilted moves in `Simulate.cpp`)
- **Patent gate:** low (citation only). Swept volumes of convex solids along straight motions on
  dexel models are textbook simulation.
- **Author(s):** OxCAM contributors

## Problem

Multi-axis toolpaths move the tool with an axis that is not +Z and that turns along a move. The
3-axis sweep (DL-0012) relies on the vertical axis: Z rays see height spans of the revolved
profile, and X/Y rays see disks. A tilted tool needs the interval each ray of each of the three
grids loses.

## Approach

- **Convex slices.** The tool solid is split along its axis into convex slices
  (`SolidPiece`):
  - straight profile segments give frustums (radius linear in height);
  - arcs centred on the axis (ball ends) give spherical zones;
  - other arcs (bull-nose corners) are split into frustums by chords of 2 µm.
  - The cutting part, the rest of the body up to the holder face, and the holder bands are kept
    apart.
- **Pieces of a move.** A move is split so its axis turns little within each piece: the far end
  of the cutting part (length plus radius) moves at most half a cell by the turning. Each piece
  translates with a fixed axis (the slerp at its middle). When the axis turns, the tool is also
  placed at its exact final pose, so the next move starts from material that pose removed.
- **Exact span of a translating slice.** For the ray o + s e (e a grid axis) and the slice moving
  from p0 by d (t in [0, 1]), with w = o + s e − p0 − t d and h = w·u:
  - a frustum r(h) = a + b h contains w when |w|² − h² − (a + b h)² ≤ 0 with h in [h0, h1];
  - a sphere zone when |w − c u|² − R² ≤ 0 with h in [h0, h1].
  - Both are one quadric Q(s, t) ≤ 0 and four lines (the height limits, t ≥ 0, t ≤ 1). Within the
    slab the region is convex, as the set of (s, t) where a convex solid swept along a segment
    meets a line. Its extent in s is attained at one of:
    - the points of Q = 0 where ∂Q/∂t = 0;
    - the intersections of Q = 0 with each line;
    - the corners of the lines.
  - All candidates are computed in closed form (quadratics), the feasible ones kept, and their
    smallest and largest s are the removed interval.
- **Batches.** Tilted pieces join the parallel batches of the remover. Rays are binned by tile
  row as in DL-0012, and rays whose material lies outside the piece's bounding box along the ray
  are skipped.
- **Collision checks** use the same spans. A rapid sweeping material uses the cutting slices; a
  shank or holder collision uses those slices made 0.1 cell thinner (to ignore grazing contact,
  like the 3-axis checks). Checks run on all grids of a tri-dexel stock and only where the
  slices dip below the stock top.

## Sources

- H. Müller, T. Surmann, M. Stautner, F. Albersmann, K. Weinert, "Online sculpting and visualization
  of multi-dexel volumes", ACM Symposium on Solid Modeling and Applications, 2003. Dexel
  simulation of moving tools.
- S. Abrams, P. K. Allen, "Computing swept volumes", *Journal of Visualization and Computer
  Animation* 11, 2000. Swept volumes of convex solids along motions.
- DL-0012 (dexel stock and 3-axis sweeps).

## Correctness and tests (`tests/unit/sim/TiltedTest.cpp`)

- **Spans.** 400 random frustums and sphere zones, random axes, motions and rays: the computed
  interval contains the brute-force one (convex minimization over t for each s, sampled every
  0.01 along s) and exceeds it by no more than the sampling.
- **Equivalence.** A tool tilted by 10⁻⁶ removes the same volume as the 3-axis sweep along
  straight and sloped moves, to 0.5 %.
- **Analytic slab.** A horizontal end mill along +Y sweeping along X removes 40 × 10 × 25 mm³
  within 1 %. The X grid is emptied too, and the shank running through the rest of the box is
  reported.
- **Collisions.** A tool tilted 60° and plunged deep reports a shank collision; a rapid through
  the stock reports a rapid collision.
- **CLI.** The 4- and 5-axis sample projects simulate without gouges or collisions (DL-0024).

## Known limitations

- **Speed.** Tilted moves are slower than vertical ones: every ray under the slanted flutes is
  computed. The 5-axis sample takes about 11 s at 0.5 mm.
- **Turning within a piece** is approximated by the middle axis (at most half a cell at the far
  end of the cutting part).
- **Arcs** with a tilted tool are sampled at an eighth of a cell.
