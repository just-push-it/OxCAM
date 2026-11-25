# DL-0054: A part-off's end feed is its own value

- **Status:** implemented
- **Module:** libs/tooling (`CuttingPreset::partOffEndFeed`, `ToolLibrary`), libs/document
  (`PresetOverrides`, `adoptExplicitFeeds`), libs/strategies (`Feeds::partOffEnd`, `turnPartOff`),
  apps/opencam-gui (`CuttingPresetDialog`, `PropertyPanel`, `ToolLibraryDialog`)
- **Patent gate:** low (citation only). A feed value.
- **Author(s):** OxCAM contributors

## Problem

A part-off ran the last of its cut at `0.5 * ctx.feeds.cut`. The label "Half feed over the last" said
so, but the feed itself was a factor in the code: nothing showed it and nothing could change it, which
ADR-0004 does not allow for a feed. The maintainer's decision, 2026-11-01: make it a value.

## Approach

**A value in the preset.** `CuttingPreset::partOffEndFeed`, a feed per revolution like the feed beside
it, is shown as "Part-off end feed" with its share of the feed. The row is there for parting and
grooving inserts, the tools a part-off takes. A part-off operation shows it in its speeds and feeds and
can change it for itself (`PresetOverrides::partOffEndFeed`).

**The output.** `Feeds::partOffEnd` is that value times the spindle speed, as the cutting feed is, so a
lathe post writes it back as the F per revolution the user set. Nothing relates it to the cutting feed:
it may be above it, and it does not follow when the feed per revolution changes.

**Zero.** With a distance set, a part-off whose end feed is 0 is refused, naming the value and where to
set it, or to set the distance to 0. With the distance at 0 the value is not used and no move runs at
it. (The strategy refuses, since the general check in `generateToolpath` would name the feed rate.)

**The distance.** `slow_distance` keeps its key and meaning. Its label is now "End feed over the last".

**New presets.** A new parting or grooving preset starts at 0.025 mm per revolution, half its starting
0.05, visible before anything uses it (ADR-0004, 6).

**Older files keep what they cut (ADR-0004, 7).** A parting or grooving preset with no
`part_off_end_feed` in its file gets half its feed per revolution. A part-off operation with its own
feed per revolution, on such a preset, gets half of that as its own end feed. A value that is in the
file stays, 0 included. The older programs come out the same.

## Correctness and tests

`LatheTest` "threading and parting off": the end moves run at the end feed and the others at the feed;
0 is refused; with no distance every move runs at the feed. `LatheStockTest` "a part-off's end feed":
from the preset and from the operation, through `generateToolpath`, and refused at 0. `ProjectTest` and
`LibraryIoTest`: older files as above, a saved 0 stays 0, the value survives a save.

## Known limitations

- **Pecks** run at the feed per revolution until the end distance, as before.
- **Other turning moves:** turning's leads got their own value next, in
  [DL-0055](0055-turning-lead-feed.md).
