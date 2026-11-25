# DL-0091: Bar pull

- **Status:** implemented
- **Module:** libs/tooling (`ToolType::BarPuller`, `CuttingPreset::pullFeed`), libs/toolpath
  (`MachineFunction`, `Move::function`), libs/strategies (`barPull`, `bar_pull`), libs/sim
  (`LatheStock::heldBar`, `LatheStock::pulled`, `barPulledBy`), libs/document (`trailingBarPull`,
  `latheStockBefore`, `simulateLatheSetup`, the machine check), libs/post (`NcOp::Chuck`,
  `NcOp::BarFeeder`, NC program version 7, `oxcam:fanuc`), apps/opencam-gui
- **Patent gate:** low (citation only). A sequence of straight moves and machine functions that any
  bar-fed lathe's manual describes; no algorithm.
- **Author(s):** OxCAM contributors

## Problem

On a bar-fed lathe the program ends by bringing out the next part's length of bar: a puller in a
turret station grips the bar, the chuck opens, the turret pulls the bar out to a set length, the chuck
closes, and the program runs again. OxCAM had no puller, no operation for it, no way to say "the chuck
opens" in a toolpath or a program, and a setup's stock that never moved.

## Approach

**The tool.** `ToolType::BarPuller` ("bar_puller"), a lathe tool listed with the turning tools, held
on a turret block like a drill: along the spindle, on its axis. It has no insert and cuts nothing. Its
`MillGeometry` says what it is: `diameter` its body, `tipDiameter` the largest bar it grips,
`fluteLength` how deep the bar goes into its jaws. Its outline is a body with the bar's bore in it, as
deep as its jaws (`toolProfile`), so it is drawn on the turret and checked with the machine like any
tool.

**Its feeds are the user's** (ADR-0004). Its cutting preset holds two: the plunge feed, onto the bar and
off it again, and a new `pullFeed`, the pull with the chuck open. Both are per minute: the spindle stands
still. Neither has a fallback: a 0 refuses the toolpath, naming the value and where to set it. The
preset editor shows only those two for a puller (*Feed onto the bar*, *Pull feed*), and so does the
operation's page, where an operation can change them for itself (`PresetOverrides::pullFeed`). A new
puller's preset starts at 500 and 1000 mm/min, a start to check against the machine like every
default preset.

**The operation.** `bar_pull`, *Bar pull*, in the Turning menu:

| Parameter | |
|---|---|
| Pull length | How far the bar comes out. No default: 0 refuses. |
| Grip | How far the jaws go over the bar from its front; while not set, the puller's jaws' depth (the `ParamDef`'s `toolDefault`). Deeper than the jaws refuses. |
| Clearance | In front of the bar, where the rapid stops and the puller lets go. |
| Dwell at the chuck | After it opens and after it closes; 0 = none. |
| Bar feeder pushes | The bar feeder's codes go before the chuck opens and after it closes. |

**The moves** (`strategies::barPull`), on the spindle's axis (x = 0), the spindle stopped: the
approach to the clearance in front of the bar's front; onto the bar to its front less the grip at the
plunge feed; the chuck opens; the pull, by the pull length, at the pull feed; the chuck closes; off the
bar to the clearance in front of where the bar now ends, at the plunge feed. **Where the bar is** is
what the earlier operations left: the pieces of the stock that reach its back end, the lowest Z
(`LatheStock::heldBar`). A piece in front of them was parted off. A bar wider than the puller takes, where
it grips, is refused.

**What the machine does is a move.** `MachineFunction` (chuck open and close, bar feeder on and off)
rides on a Dwell move (`Move::function`), whose seconds are the dwell after it. A dwell already means
"standing still" to every consumer (stats, playback, the checks, smoothing, transforms), so nothing
else had to learn a new kind of move. The toolpath JSON writes it as `"fn"`.

**The NC program** (version 7) has two instructions for it: `chuck` (`spindle: "main"`, `open`) and
`bar_feeder` (`on`). Lowering refuses to open the chuck while the spindle or a live tool turns. The
`oxcam:fanuc` lathe writes them from the post's options, empty on `fanuc-generic-lathe` (Fanuc leaves
these M-codes to the machine builder) and refused while empty, as the live tool's codes are (DL-0063).
`haas-lathe` starts from Haas's M11 (unclamp) and M10 (clamp). With the spindle stopped, feed per
revolution means nothing, so the lathe writes G98 before such a feed and G99 when the spindle turns
again.

**The part loop is the post's**, an option, not logic hidden in the program: *At the program's end*
is `end` (the footer, M30), `repeat` (the footer's return home, then M99: the program runs again
until stopped, as by the bar feeder at the end of a bar), or `count` (Fanuc custom macro: the part
counter variable the user names, one more per part, M99 until *Parts to run*, then the counter back
to 0 at N9999 and M30).

**The stock moves.** In the simulation the held bar moves out by the pull (`LatheStock::pulled`): its
pieces and the threads and holes in them move; behind them the bar goes on as it was at its back end,
so it stays in the chuck; what was parted off is left out, having come off. Playback moves it as the
pull goes (`barPulledBy`: the tool's travel along Z while the chuck is open). The operations after a
pull plan on the bar where it now stands (`latheStockBefore`), so a pull at the start of the cycle works
too, and their toolpath keys depend on it. The machine check sees the bar everywhere it will be during
the pull, and where it is at each move.

**The next cycle, and the part.** The bar pulls that end a setup (`trailingBarPull`) make the next
cycle's bar. So the part is compared as it was before them, and a setup carried on from this one
(DL-0044, DL-0061) takes the part as it came off, not the bar. After them, the bar's front is compared
with where this setup's stock starts: when they differ, the simulation says by how much and which way,
since the next part would then start from another bar than this one.

## Sources

- ISO 6983-1:2009, *Automation systems and integration: Numerical control of machines: Program format
  and definitions of address words: Part 1*. Miscellaneous functions, M codes, as words (paraphrased).
- Fanuc-style lathe programming manuals: G98/G99 feed per minute and per revolution, M99 at the end of
  a main program, custom macro B variables, IF ... GOTO (syntax facts only).
- Haas lathe M-code reference: M10 chuck clamp, M11 chuck unclamp (syntax facts only).

## Deviations from sources

None.

## Correctness and tests

- `ToolTest`, `LibraryIoTest`: the puller is valid, its outline has the bar's bore, and its type and
  pull feed survive library files; older files read 0, never a guess.
- `ToolpathTest`: the function round-trips through JSON and comparisons.
- `sim` `LatheTest`: the held bar; pulled, it goes on behind, keeps a tube a tube, and leaves the
  parted part; how far it has come move by move.
- `document` `BarPullTest`, the shaft turned and parted off on `generic-lathe-turret`, then pulled by
  the part, the part-off's width and the facing allowance: the moves, the feeds and the functions; the
  bar feeder and dwell; each refusal; a pull at the cycle's start moving what follows; the simulation
  moves the stock to where it started, compares the part as it came off, plays the pull back, and says
  a pull that leaves the bar elsewhere; the generic post refuses without chuck codes, writes
  `tests/golden/posts/lathe-bar-pull.nc` with them (the first lathe golden program), and reads back as
  the toolpaths; the part loop's `repeat` and `count`; the bar feeder's codes; Haas's codes; the NC
  program's instructions, version 7, and the refusal to open a turning chuck.

## Known limitations

- **A setup that mills with a live tool** does not simulate a bar pull: its stock is dexels, which do
  not move. It is said on the operation.
- **A bar stop** (the turret as a stop and the bar feeder pushing the bar against it) is not an
  operation: the bar feeder option only adds its codes around a pull.
- **The part loop's label** N9999 is fixed; a post writing its own line numbers there would clash.
- **End of bar:** the bar's length is not modelled; the bar feeder's end-of-bar signal is the machine's.
