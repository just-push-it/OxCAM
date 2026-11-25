# IP-0005: Swarf (flank) milling of ruled walls, and Z-level roughing in a tilted orientation

- **Status:** low risk by the policy's categories (proposed by the implementer, to be confirmed by
  the review group). No prior-art search is required at low risk; the references below are the
  design log's citations.
- **Risk category:** low, with conditions (below). The rest option of tilted roughing stays
  under [IP-0002](0002-rest-machining-from-in-process-stock.md) (medium, pending), unchanged.
- **Reviewers:** (to be assigned)
- **Date:** 2026-11-09
- **Related design log:** docs/design-log/0085-swarf-milling.md, docs/design-log/0036-operation-tool-orientation.md
- **Related PR:** branch `claude/fiveaxis-swarf-d28ad1`

## Why this record

The IP policy lists **automatic 5-axis tilt or collision avoidance** as medium risk. Both
features tilt the tool, so they were checked against that category before any code was written.

## Method summary

**Swarf milling (`swarf5x`).** The user selects ruled wall faces (and optionally one flat floor).
For each wall:

1. The wall's rulings are taken from its exact surface: for a plane, lines in the plane's steepest
   direction along the tool axis; otherwise the parameter lines of the direction that is straight
   (within the user's tolerance). A wall straight in neither direction is refused, naming its
   deviation. Rulings are spaced so that straight moves between them stay within the chord
   tolerance.
2. Each tool position puts the cylindrical flank of a flat, bull-nose or ball end mill along one
   ruling: the two end points of the ruling are offset by the tool radius (plus stock to leave)
   along their own surface normals, and the tool axis is the line through the two offset points
   (the "double point offset" of the flank milling literature, Liu 1995). On a developable wall
   (plane, cone, cylinder) this is exact along the whole ruling; on a twisted wall the flank leaves
   or cuts into the wall between the ends, by an amount computed at points along the ruling,
   reported, and refused beyond the user's tolerance.
3. The tool tip is the bottom offset point moved along the axis by the user's axis offset, or,
   with a floor selected, the position along the axis where the tool's lowest point stays the
   axis offset above the floor's plane.
4. Depth passes along the ruling: the same positions shifted up the axis by fractions of the
   ruling's length, the deepest last.
5. Convex corners between walls are turned about the shared ruling; sharp inside corners are
   refused.
6. A ruling steeper than the user's largest tilt is refused (nothing is re-tilted). The tilted
   flutes and shank are checked against the part and reported, as for 5-axis ball finishing
   (DL-0024); nothing is moved to avoid them.

**Z-level roughing in a tilted orientation.** The existing Z-level roughing (DL-0018) is
generated in the operation's own tool orientation (DL-0036), a frame the user picks. The rest
option slices the simulated in-process stock in that frame, as DL-0036 already does for every
operation.

## Assessment against the risk categories

- **Swarf:** the tool axis is fully determined by the wall's geometry (its rulings) and the user's
  parameters. OxCAM never chooses or changes a tilt to avoid a collision or a gouge: steep rulings,
  inside corners and twisted walls beyond the tolerance are refused, and clearance problems are
  reported. This is not "automatic tilt or collision avoidance". Flank milling of ruled surfaces
  along their rulings, with the tool offset along the surface normals at the ruling's end points,
  is decades-old published practice (references below). **Low.**
- **Tilted roughing:** the orientation is chosen by the user per operation (DL-0036, already
  classed low); offset pocketing per level is low (DL-0007, DL-0018). Nothing in the roughing picks
  the orientation. **Low.** Its rest option is the method under IP-0002, with no change; it stays
  off by default and must be cleared with IP-0002 before release.

## Prior art (design-log citations)

| Reference | Year | What it discloses |
|---|---|---|
| X.-W. Liu, "Five-axis NC cylindrical milling of sculptured surfaces", *Computer-Aided Design* 27(12) | 1995 | Flank milling of ruled surfaces with a cylindrical cutter; tool positions from offsets of a ruling's points along the surface normals (single and double point offset) |
| S. Bedi, S. Mann, C. Menzel, "Flank milling with flat end milling cutters", *Computer-Aided Design* 35(3) | 2003 | Tool positioning along the rulings of a ruled surface and the error of the flank on non-developable surfaces |
| H. Pottmann, J. Wallner, *Computational Line Geometry*, Springer | 2001 | Ruled and developable surfaces, rulings, the normal along a ruling |
| B. K. Choi, R. B. Jerard, *Sculptured Surface Machining*, Kluwer | 1998 | Multi-axis tool orientation and positioning |

## Search summary

No patent search was performed (the policy reserves searching for the review group, and low-risk
methods need the design-log citations only).

## Outcome and conditions

Low risk, under these conditions:

- The swarf tool axis follows the wall's rulings with fixed, user-given parameters. Any automatic
  change of tilt (to avoid collisions, to reduce the twist error, to stay within the machine's
  limits) needs a review first.
- No optimization of the tool position beyond the double point offset (fitting the flank to a
  twisted surface by least squares or similar) without a review.
- Tilted roughing uses only the orientation the user sets; choosing an orientation automatically
  needs a review.
- The rest option stays under IP-0002.

## Counsel sign-off (high-risk only)

Not applicable.
