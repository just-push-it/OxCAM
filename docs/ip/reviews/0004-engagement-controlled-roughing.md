# IP-0004: Engagement-limited roughing

- **Status:** proceeding on the maintainer's decision; counsel sign-off **not** obtained
- **Risk category:** high ("constant-engagement, adaptive, dynamic or trochoidal clearing with
  engagement control", IP-POLICY.md)
- **Reviewers:** project maintainer (Alan Litz)
- **Date:** 2026-10-20 (decision recorded)
- **Related design log:** docs/design-log/0018-zlevel-roughing-and-3d-rest.md
- **Related records:** IP-0003 (the offset passes this method builds on)

## Decision

IP-POLICY.md requires counsel sign-off before work starts in this category. The maintainer
decided to proceed without it, in these words:

> I'll take responsibility for the IP-policy decision on engagement-controlled roughing. Record
> that in docs/ip and go ahead with a trochoidal / engagement-controlled roughing strategy.

The maintainer also stated that they found no active patent that would prevent it. No search
record was provided, and none was made for this record.

**Before any public release**, this record needs either counsel sign-off or a documented
prior-art and claim review that closes it. Until then the feature stays as described below.

## Method summary

Z-level roughing clears each level pass by pass. Each pass is the boundary of the remaining
material grown by (tool radius − stepover), so the tool would cut one stepover deep, plus the
area's edge where the material is thinner than that. The pass is traced step by step against a
grid of the remaining material: a step is taken only when the cutter's engagement angle (the
arc of its circumference ahead of the motion that lies in material) is within the angle of a
straight cut one stepover wide at both ends of the step; otherwise the heading turns toward the
cleared side until it is. The same rule produces the lead-in from a clear point behind the pass
and the pulling away from inside corners, which are then taken in thinner layers over later
passes. Nothing adjusts feeds; there are no trochoidal loops. The maintainer's decision quoted
above covered "a trochoidal / engagement-controlled roughing strategy"; what was built is the
engagement-controlled part, which is the high-risk category of IP-POLICY.md.

The method uses published general techniques only: the engagement angle as the contact arc of
the cutter with the remaining material, and constant-engagement offsetting (passes kept at the
engagement of the stepover, pulled away at corners). It does not reproduce any vendor's
algorithm, and it uses neutral naming only ("engagement-limited roughing").

## Prior art

| Reference | Year | What it discloses |
|---|---|---|
| J.A. Stori, P.K. Wright, "Constant engagement tool path generation for convex geometries", *Journal of Manufacturing Systems* 19(3) | 2000 | Offsetting passes so the cutter's engagement stays constant, and the corner problem |
| G. Elber, E. Cohen, S. Drake, "MATHSM: medial axis transform toward high speed machining of pockets", *Computer-Aided Design* 37 | 2005 | Pocket tool paths that avoid full-width engagement |
| M. Held, *On the Computational Geometry of Pocket Machining*, LNCS 500 | 1991 | Contour-parallel offset pocketing |

## Search summary

Not performed for this record (see Decision).

## Outcome and conditions

Proceeding on the maintainer's decision, recorded above. Not cleared by counsel.

## Counsel sign-off (high-risk only)

Not obtained.
