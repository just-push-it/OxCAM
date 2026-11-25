# IP-0002: Rest machining driven by the simulated in-process stock

- **Status:** pending
- **Risk category:** medium. The IP policy lists "automatic rest machining driven by in-process
  stock" as needing a prior-art search.
- **Reviewers:** (to be assigned)
- **Date:** 2026-10-19 (opened)
- **Related design log:** docs/design-log/0013-rest-machining.md
- **Related PR:** phase 6

## Method summary

For a 2D pocket operation marked "rest machining":

1. Simulate the setup's earlier operations on a dexel stock (DL-0012).
2. At each depth level z of the pocket, take the rays of the vertical dexel grid that hold
   material between z and the level above. Trace the boundary of those cells into polygons.
3. Keep the parts inside the area a round tool can reach in the pocket. Remove thin slivers with
   a morphological opening of about one ray spacing.
4. The tool-centre area is the pocket's allowed centre area intersected with the remaining
   material grown by the tool radius. Clear it with the existing contour-parallel offset loops and
   residual clean-up (DL-0007).

The method uses only the operation's own boundary and the simulated stock. It does not change
feeds, engagement or the order of operations.

**Phase 7 adds two 3D variants (DL-0018) under the same review:**

- **3D rest roughing:** Z-level roughing whose material per level comes from the simulated
  in-process stock (the same per-slab regions as above).
- **3D rest finishing:** parallel finishing limited to where the simulated stock stands more
  than a threshold above the part (a per-ray comparison of the dexel stock with the part's
  mesh), grown by the tool radius.

The same containment, unchanged, limits contour finishing (DL-0033) and, since DL-0082, pencil
and steep and shallow finishing when their "Rest machining" option is on. Off (the default),
neither uses the in-process stock: pencil finishing finds what a larger tool left from the
reference tool's size alone (DL-0082, low risk).

## Prior art

These references are candidates for the search. Each must be checked to confirm what it
discloses.

| Reference | Year | What it is expected to disclose |
|---|---|---|
| B. K. Choi, R. B. Jerard, *Sculptured Surface Machining: Theory and Applications*, Kluwer | 1998 | Z-map workpiece models and the detection of uncut regions for re-machining with smaller tools |
| T. Van Hook, *Real-time shaded NC milling display*, SIGGRAPH '86 | 1986 | Dexel models of the workpiece after machining |
| Commercial CAM documentation describing "rest" or "remachining" pocket options | 1990s onwards | That re-machining the material a larger tool left behind with a smaller tool is long-established practice |

## Search summary

Not yet performed. To do:

- Search for patents claiming the detection of remaining material from a simulated
  workpiece model, combined with generating 2.5D toolpaths restricted to it.
- Record the databases searched, the query terms and the date range here. Do not paste claim
  text.

## Outcome and conditions

Pending. The feature may stay in development builds, but it must be cleared before any public
release. Until then:

- It is off by default (`rest_machining: false`).
- It stays limited to the method summarized above.

## Counsel sign-off (high-risk only)

Not applicable (medium risk).
