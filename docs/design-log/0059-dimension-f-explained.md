# DL-0059: Dimension f says what it is

- **Status:** implemented
- **Module:** apps/opencam-gui (`ToolLibraryDialog`, `ToolProfileView`, `HolderEditorCheck`)
- **Patent gate:** none. Labels and a drawing.
- **Author(s):** OxCAM contributors

## Problem

The holder editor had a field called "Dimension f", which means something only to someone who already knows
ISO 5610. The preview drew a small "f" under a dimension line, with nothing to show where it ended.

## Approach

**What f is.** On a shank, how far the tool's tip stands from the back of the shank, measured across the shank. On
a boring bar, how far the tip stands from the bar's axis. It is what puts the holder where it is relative to the
programmed point, and the designation does not give it.

**The field** is labelled for what it is, and changes with the holder: "Tip to back of shank (f)" or "Tip to bar's
axis (f)". This goes for ISO and plain holders alike. The tooltip says the same at more length.

**The preview.** The dimension line carries its value ("f = 32.000 mm"). On a shank the back of the shank is drawn
in the same colour, thick, so the far end of the line has something to point at; on a bar the axis is already drawn.
The caption says "f: tip to back of shank" or "f: tip to bar's axis". A plain holder's preview in the Holders tab
now draws it with an insert too, as an ISO holder's does: it used to draw a bare outline, with no f to mark.

**Found on the way.** A left-hand bar's axis was drawn above the tip, and is now below it, where its outline has it.

Nothing about the value or its use changes.

## Correctness and tests

`gui.holder_editor` selects an ISO shank and a plain one, changes each to a bar through its list, checks the label each
time, and saves a screenshot of each.

## Known limitations

- **A left-hand holder's** back is drawn on its own side, as its dimension is. The test only looks at right-hand ones.
