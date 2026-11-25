# DL-0055: Turning's lead moves have their own feed

- **Status:** implemented
- **Module:** libs/tooling (`CuttingPreset::leadFeedPerRev`, `ToolLibrary`), libs/document
  (`PresetOverrides`, `adoptExplicitFeeds`), libs/strategies (`turningFeeds`, `turnFace`, `turnRough`,
  `turnFinish`), apps/opencam-gui (`CuttingPresetDialog`, `PropertyPanel`, `ToolLibraryDialog`)
- **Patent gate:** low (citation only). A feed value.
- **Author(s):** OxCAM contributors

## Problem

`turningFeeds` set the lead feed to the cutting feed, and the preset editor hides the milling lead feed
for turning tools. So turning's leads ran at a feed with no value of its own, which ADR-0004 does not
allow: plunge, ramp and lead feeds are values the user sets. Three lead-outs (facing, and both kinds of
roughing pass) did not even go through the lead feed: they used the cutting feed directly.

## Approach

**A value in the preset.** `CuttingPreset::leadFeedPerRev`, a feed per revolution like the turning
feed beside it, is shown as "Lead feed per revolution" with its share of the feed. The row is there for
turning inserts, the tools that face, rough and finish. An operation shows it in its speeds and feeds
and can change it for itself (`PresetOverrides::leadFeedPerRev`).

**Why not `leadFeed`.** That is a feed rate in mm/s, and every file already holds a 0 there for turning
tools. On a lathe the speed follows the diameter, so a rate would not be the feed the post writes, and
the key could not tell an older file from a value of 0.

**The output.** `Feeds::lead` is that value times the spindle speed, as the cutting feed is, so a lathe
post writes it back as the F per revolution the user set. All lead moves now go through it. It does not
follow when the feed per revolution changes.

**Zero.** A lead feed of 0 is refused by the general check in `generateToolpath`, which names the lead
feed and where to set it.

**New presets.** A new turning insert's preset starts with its lead feed at its starting feed, visible
before anything uses it (ADR-0004, 6).

**Older files keep what they cut (ADR-0004, 7).** A turning insert's preset with no `lead_feed_per_rev`
in its file gets its feed per revolution. An operation with its own feed per revolution, on such a
preset, gets that as its own lead feed. A value that is in the file stays, 0 included.

## Correctness and tests

`LatheStockTest` "turning's lead feed": on the shaft fixture's facing, roughing and finishing, every
lead move runs at the value and every cutting move at the feed, from the preset and from the operation;
the lead does not follow the feed; 0 is refused. `ProjectTest` and `LibraryIoTest`: older files as
above, a saved 0 stays 0. The shaft fixture's posted program is unchanged, its lead feed being its feed.

## Known limitations

- **Grooving, threading and parting** have no lead moves: they rapid on and off, so they have no such
  value.
