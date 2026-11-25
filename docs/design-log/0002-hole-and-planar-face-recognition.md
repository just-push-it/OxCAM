# DL-0002: Hole and planar-face recognition

- **Status:** implemented (phase 1)
- **Module:** libs/cad (`findHoles`, `findPlanarFaces`)
- **Patent gate:** low (citation only); rule-based recognition of simple analytic features
- **Author(s):** OxCAM contributors

## Problem

Seed drilling and facing operations from the B-rep:

- cylindrical holes parallel to the tool axis, with diameter, depth, entry point and bottom
  type;
- planar faces perpendicular to the tool axis.

## Approach

**Holes.**

1. **Candidate faces.** Take cylindrical faces whose axis is parallel to the tool axis (within
   the angular tolerance).
2. **Concavity.** At the face's UV midpoint, take the outward face normal (the surface normal,
   flipped for reversed faces). The face is a hole wall if the normal points *towards* the
   cylinder axis. Bosses point away and are skipped.
3. **Grouping.** Group walls by axis line (projected onto the plane perpendicular to the tool
   axis) and radius. Seam-split half cylinders merge. Coaxial cylinders of different radius,
   such as a counterbore and its through hole, stay separate.
4. **Extent.** The axial extent of each group is the min/max over a UV-grid sample of its faces.
5. **Bottom closure at each end.** Check for either of these:
   - a coaxial cone whose wide end meets the wall at that end (radius equals the wall radius)
     and whose other end is a point. That is a drill point, with included angle twice the
     cone's semi-angle.
   - a planar face perpendicular to the axis at that end, with its outward normal pointing back
     into the hole, and lying within the hole circle. A disc's UV bounding square reaches at
     most r√2 from the axis. This rejects faces that extend outward, such as the top face
     around a hole, and accepts the annular step at the bottom of a counterbore.
6. **Classification.**
   - Closed at the low end: entry at the high end, direction −axis.
   - Closed at the high end: entry at the low end, direction +axis.
   - Open at both ends: a through hole, entry at the high end.
   - Closed at both ends: an internal void, skipped.

**Planar faces.** Plane-type faces whose oriented normal is parallel to the axis. Height is the
plane location projected on the axis; area comes from surface properties.

## Sources

- J. Han, M. Pratt, W. C. Regli, "Manufacturing feature recognition from solid models: a status
  report", *IEEE Transactions on Robotics and Automation* 16(6), 2000, pp. 782–796. This survey
  covers rule-based and graph-based recognition; we implement the simplest rule-based subset.

## Deviations from sources

The rules are deliberately minimal and work on analytic surfaces only.

## Correctness and tests

`tests/unit/cad/SectionFeatureTest.cpp` uses the generated `hole_plate.step`, which has a
through hole, a flat-bottom hole, a drill-point hole and a counterbore, and `boss_plate.step`,
which has a boss that must be ignored. Diameter, depth, entry, bottom type and tip angle match
the generator parameters.

## Known limitations

- Spline-approximated cylinders, tapered holes, threads, intersecting holes and holes split by
  cross holes are not handled.
- Holes interrupted by a cross hole may merge into one depth range.
