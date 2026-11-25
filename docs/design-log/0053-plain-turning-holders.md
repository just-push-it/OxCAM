# DL-0053: Plain turning holders

- **Status:** implemented
- **Module:** libs/tooling (`TurningStick`, `makeTurningStick`, `turningBody`), libs/document
  (`generateToolpath`), apps/opencam-gui (`ToolLibraryDialog`, `ToolProfileView`)
- **Patent gate:** low (citation only). Plain geometry.
- **Author(s):** OxCAM contributors

## Problem

ISO 5608 and ISO 5609 name holders for indexable turning inserts. Grooving, threading and parting
holders are the makers' own, so such a tool could only sit in an imported outline, which gives no body
for the checks of DL-0045, in a bore or at a chuck.

## Approach

A plain holder (`Holder::stick`): a rectangular shank along +X, or a round bar along +Z, by what any
catalogue gives for one: its width or diameter, f, and its length. It holds any turning tool.

**The body (`turningBody`).** It starts past what the tool's own outline takes up. Outside a shank
stands above it, with its back f behind the tip. Inside a bar lies below it, its axis f from the tip.
In a face the shank runs along Z. With that the checks of DL-0045 hold for these tools: the chuck and
the part refuse, the simulation shows the stock, across the axis for a bar.

**Which side.** A bar on an outside cut, or a shank on an inside one, is refused like an ISO holder.

**The tool's reach.** The outline of a grooving insert (15 mm, DL-0020) stands in for the blade, so a
narrow groove in a small bore shows where a real blade must be chosen with care.

**In the same commit.** Thread rapids kept off the crest only: they now keep off the stock too.

## Correctness and tests

`LatheStockTest` "a plain holder": a bar too thick for a bore is refused across the axis, a thin one
runs; a bar is refused outside, a shank inside; the library file keeps it. `gui.turning_holder_stick`
shows it in the editor.

## Known limitations

- **Hand:** a plain holder has none of its own: the tool's Hand says which way it cuts.
- **Blades and blocks:** a parting blade in a block is a shank here.
