# IP-0003: Roughing passes offset from the cleared side

- **Status:** pending
- **Risk category:** low (offset pocketing), recorded for review because it is close to the
  high-risk "constant-engagement" category. The method does not control engagement: it offsets
  the cleared area by a fixed stepover and makes no adjustment from measured or simulated
  engagement, and it does not use trochoidal or other engagement-limiting motion.
- **Reviewers:** (to be assigned)
- **Date:** 2026-10-20 (opened)
- **Related design log:** docs/design-log/0018-zlevel-roughing-and-3d-rest.md
- **Related PR:** (Z-level roughing, open areas)

## Method summary

At each Z level of 3D roughing, the allowed tool-centre area is split into connected pieces. A
piece that contains tool positions clear of all material (open to the side) is cleared by
passes that are the boundaries of successive offsets, by one stepover, of the area already
cleared (starting from the positions in air), clipped to the allowed area. A last pass follows
the part's walls. Each pass is entered from air or from a path already cut, by a straight lead
at a shallow angle where there is room. Closed pieces (pockets) keep the inside-out
contour-parallel loops of DL-0007.

## Prior art

| Reference | Year | What it discloses |
|---|---|---|
| M. Held, *On the Computational Geometry of Pocket Machining*, LNCS 500 | 1991 | Contour-parallel offset pocketing, including offsets from the stock boundary of an open area |

## Search summary

Not yet performed. To do: search for patents claiming roughing passes generated as offsets of
the machined region in open areas.

## Outcome and conditions

Pending review. Engagement measurement and trochoidal loops on top of these passes are a
separate, high-risk method: see IP-0004 (proceeding on the maintainer's decision, without
counsel sign-off).

## Counsel sign-off (high-risk only)

Not applicable at this risk level.
