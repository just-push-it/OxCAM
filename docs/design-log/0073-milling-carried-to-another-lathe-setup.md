# DL-0073: Milling carried to another lathe setup

- **Status:** implemented
- **Module:** libs/document (`simulateLatheSetup`, `stockIsMilled`, `latheStockBefore`)
- **Patent gate:** none. The stock carried as DL-0044 and DL-0066 already hold it.
- **Author(s):** OxCAM contributors

## Problem

A lathe setup going on from one a live tool milled (second side, a re-grip) carried the profile and its cuts, and warned that it left out the
milling (DL-0066). The maintainer asked whether it could carry it. The dexels were there all along, so it can.

## Approach

**In the simulation.** If a lathe setup on the way milled (`stockIsMilled`), the setup's simulation gets that setup's as a mill's does: its final
stock moves into this setup's WCS, flipped end for end when it is, sampled again on this setup's grid, and clipped to the profile, so a
piece that was parted off stays behind. The stock is dexels from the first frame (`milledFrom` 0), with all DL-0066 gives: rapid, shank and holder
checks, and the comparison with the part. What a thread mill took stays exempt (`initialChamfered`).

A setup whose chain has no milling is untouched: same profile, same speed. Its source is not even simulated.

**A bug this found.** The stock carried from a lathe setup (`latheStockBefore`) took a live tap as a hole of the tap's full diameter, which the
simulation of its own setup does not: a tap adds nothing to the stock (DL-0065). A second setup then showed a tapped hole as a Ø6 one, a false gouge.

**Generating stays in the lathe plane.** Turning plans on the profile, the envelope of what is left, so it treats a flat as still full. That
means air cuts where the flat is, never less clearance. The warning is gone, as the simulation now shows the truth, and a program does what the
toolpath says. A tap that follows relies on the simulation for its hole (DL-0072).

## Correctness and tests

`LiveMillTest` "another lathe setup goes on from the milling": the second setup starts at the volume the first left, under the profile's; turning
keeps the milling; no warning of leaving it out; no gouge; the cache keeps it.

## Known limitations

- **A mill's setup, then a lathe's** is still refused (DL-0044): there is no profile to start from without a turned setup.
- **Air cuts** over a flat, as above: a turning pass does not skip what milling took.
