# DL-0029: Step down and stepover from the cutting preset

- **Status:** implemented
- **Module:** libs/strategies (`presetStepDown`, `presetStepover`, the `*Params` readers),
  libs/document (`PresetOverrides`, loading of older files), apps/opencam-gui (operation page)
- **Patent gate:** low (a data-model decision, no algorithm)
- **Author(s):** OxCAM contributors

## Problem

Each strategy had its own step down and stepover parameters (`step_down`, `stepover_ratio`,
`stepover`, turning's `depth_of_cut`), while a tool's cutting preset already carries an axial
depth ap and a radial depth ae, and the operation page lets an operation change the preset's
values for itself. Two places held the same quantity: some strategies took the preset's value
when their own was unset, some ignored the preset, and a user setting ae on the operation page
could see no effect.

## Approach

The preset is the one source. For every strategy the step down (the depth of each level or
pass; the depth of cut in turning) is the preset's ap and the stepover (the width of each pass)
its ae, as a fraction of the tool diameter where the strategy measures stepovers that way (at
most 1), or of the flat bottom's diameter for flat area finishing. An operation that needs other
values changes ap or ae on its page (`document::PresetOverrides`), for that operation only. The
strategies' own parameters are gone; a preset without the value stops the operation with a
message naming the preset and where to set it, as the speeds and feeds already do.

Finishing is the one place the stepover has another source: parallel and five-axis finishing
take it from a target scallop height when one is set (the default, 0, uses ae). Plunge roughing
spaces its rows by ae and keeps its own plunge spacing along a row (a fraction of the diameter).
Default presets for turning inserts now carry a depth of cut (2 mm) so turning works out of the
box as milling does.

Older project files set these as strategy parameters. On loading, an operation's `step_down`,
`depth_of_cut`, `stepover` or `stepover_ratio` (times its tool's diameter) move onto its preset
overrides, unless it has an override for that value already, and the parameters are dropped;
`step_down: 0` ("one level") is dropped without an override, so the preset decides. No format
version bump: the keys live in the free-form parameter object the schema already allows.

## Sources

None: a data-model decision. The maintainer's call ("keep the values in the tool's preset and
have that generally true for all toolpaths").

## Deviations from sources

—

## Correctness and tests

- `StrategiesTest` "parameter reader": the pocket parameters take ap and ae from the preset, a
  `step_down` parameter changes nothing, a wider ae than the tool gives one diameter, a preset
  without ap is an error.
- `ProjectTest` "older files' step parameters become preset overrides": the four keys move, a
  ratio becomes a width with the tool's diameter, an existing override wins, a zero is dropped.
- The document and simulation tests and the fixture projects set their depths through the
  overrides; their results are unchanged.

## Known limitations

- A strategy that wants a stepover in another unit than ae (facing at 0.7 D, say) needs ae set
  on the operation; there is no per-strategy default any more, by design.
