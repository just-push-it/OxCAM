# IP-0001: Residual clean-up inside 2D pocketing

- **Status:** pending
- **Risk category:** medium. The IP policy lists "automatic rest machining driven by in-process
  stock". This method is narrower: it uses only the operation's own swept area and no stock
  model. Even so, it should be reviewed before public release.
- **Reviewers:** (to be assigned)
- **Date:** 2026-10-18 (opened)
- **Related design log:** docs/design-log/0007-pocketing.md
- **Related PR:** phase 3

## Method summary

After cutting the contour-parallel loops of one depth level:

1. Compute the material a round tool could reach but this level's moves did not sweep (polygon
   booleans of the reachable area minus the swept disks).
2. Intersect the allowed tool-centre area with that residual grown by the tool radius.
3. Pocket the result with offset loops at a stepover of at most the tool radius.

## Prior art

| Reference | Year | What it discloses |
|---|---|---|
| M. Held, *On the Computational Geometry of Pocket Machining*, LNCS 500 | 1991 | Contour-parallel pocketing and the problem of uncut regions between offsets |

## Search summary

Not yet performed. To do: search for patents claiming uncut-region detection plus re-machining
within a single pocketing operation.

## Outcome and conditions

Pending. Until it is cleared, the feature stays limited to geometry computed from the current
operation. It must not consume simulated in-process stock; that is phase 6 rest machining, which
has its own review.

## Counsel sign-off (high-risk only)

Not applicable (medium risk).
