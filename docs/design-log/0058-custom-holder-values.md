# DL-0058: Custom values in an ISO holder

- **Status:** implemented
- **Module:** libs/tooling (`TurningHolderCode`, `holderStyleAngle`, `holderClearanceAngle`, `holderLength`),
  apps/opencam-gui (`ToolLibraryDialog`, `HolderEditorCheck`)
- **Patent gate:** low (citation only). ISO 5608 letters.
- **Author(s):** OxCAM contributors

## Problem

The style, clearance and length of an ISO holder came only from the standard's lists. A holder with a 70°
cutting edge or a 175 mm length had no way in except a plain holder, which loses the insert's seat and the
head.

## Approach

**A "custom" item in each list.** The maintainer asked for a custom option in the dropdown that exposes an input
field, and that is what it does:

| List | Item | Field that shows | Stored |
|---|---|---|---|
| Style | X: custom | Cutting edge angle | `customStyle` [deg] |
| Insert clearance | O: custom | Clearance angle | `customClearance` [deg] |
| Length | X: custom | Overall length | `customLength` [mm] |

Height, width, f and the insert size are typed already. Shape, clamping and bar material have no custom item:
shape would need a corner outline of its own, and clamping and material change no number.

**The letters.** X and O are what catalogues commonly print for a special style, length or clearance, so a custom
holder still reads as a designation: `PCXOR 2525X 12`. They were chosen from memory of catalogues and have not been
checked against the standards' text.

**A custom style's angle** is always to the feed towards the chuck, which is what side cutting styles state and
what the toolpaths use. So it has no end cutting form: an end cutting holder gives the equivalent angle.

**Starting values.** A field starts from what the letter stood for, so choosing "custom" changes nothing until a
value is typed. Choosing a letter again drops the value.

**Problems are said, not fixed.** An angle out of range, or one that leaves the corner no clearance, goes through
the holder's `validate` and shows in the editor. The value stays as typed.

**Designations.** A designation cannot state these values. Typing one with X or O in the editor keeps the holder's
values (`parseDesignation`'s `custom`); without values to keep, X and O are unknown letters as before.

**The filter (DL-0057)** compares the custom clearance, and tools in the holder take its custom angle and clearance
through `fitToHolder`.

**Files.** Only a holder with a custom value writes one, so other files do not change.

## Correctness and tests

`ToolTest` "an ISO holder with custom values": the designation, the length, the angle, the filter, `fitToHolder`,
the problems, and designations. `LibraryIoTest`: a file keeps the values, and has none without them.
`gui.holder_editor` drives the widgets: no custom fields for standard letters; each list's custom item shows its
field with the letter's value; typed values reach the holder and the tool in it; typing a designation keeps them;
a bad angle stays and is a problem; a letter drops the value.

## Known limitations

- **Insert size** stays a symbol: its inscribed circle is the nearest of the ISO 1832 series.
