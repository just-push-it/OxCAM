# DL-0005: Persistent face and edge references

- **Status:** implemented (phase 2)
- **Module:** libs/cad (`SubShape.hpp`), libs/document (`ShapeRef`)
- **Patent gate:** low (citation only)
- **Author(s):** OxCAM contributors

## Problem

Operations refer to faces and edges of imported models. Indices into a topology map change when a
model is re-imported or revised (the "persistent naming" problem). References must survive these
changes where the geometry is the same.

## Approach

**What a reference stores:**

- the 1-based index hint (TopExp map order);
- a signature: centroid, characteristic direction (plane normal, cylinder or cone axis, line
  direction or circle axis), and measure (area or length).

**Matching:**

1. If the face or edge at the hint index matches the signature within tolerance, use it.
2. Otherwise, scan all faces or edges of that type and pick the best match. The score is the
   centroid distance plus the relative difference in measure, and directions must agree up to
   sign.
3. If nothing is within tolerance, the reference is broken. The UI must ask the user to reselect.

## Sources

- J. Kripac, "A mechanism for persistently naming topological entities in history-based
  parametric solid models", *Computer-Aided Design* 29(2), 1997.
- D. Marcheix, G. Pierra, "A survey of the persistent naming problem", *Proceedings of the ACM
  Symposium on Solid Modeling and Applications*, 2002.

## Deviations from sources

Imported STEP models have no feature history, so this uses purely geometric signature matching,
not history-based naming.

## Correctness and tests

`tests/unit/document/DocumentTest.cpp` ("face references survive re-import"):

- every face of `hole_plate.step` is found again after an export and re-import round-trip,
  starting from a deliberately wrong index hint;
- a bogus signature finds nothing.

## Known limitations

- Symmetric duplicates (identical faces at the same centroid) cannot be told apart.
- Edited faces (resized, moved) intentionally fail to match.
