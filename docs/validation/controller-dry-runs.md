# Controller dry-run log

Every post configuration needs at least one documented dry run on a real control (or the
vendor's control simulator) before it is called supported. Run with no tool or stock: single
block, reduced rapids, feed override low, Z offset raised.

| Date | Post config | Control / model | Program | Result | Notes | By |
|---|---|---|---|---|---|---|
| | `fanuc-generic-mill` | | `sample_project.nc` | pending | | |

## Checklist

- [ ] The program loads without alarms (header, comments, line numbers).
- [ ] Tool change, G43 tool length and work offset select correctly.
- [ ] Rapids and feeds move in the expected directions and at the expected rates.
- [ ] Arcs (G2/G3, IJK) and helical arcs run without radius alarms.
- [ ] Canned cycles (G81/G83/G73) run, and G80 cancels them.
- [ ] G41/G42 start and cancel without alarms (contour with a lead-in line).
- [ ] The program ends: G28 home, M30.
