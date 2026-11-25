# DL-0011: Machine solve, NC IR, data-driven post processing and back-plotting

- **Status:** implemented (phase 4); the formatter and its JSON post configurations are
  superseded by DL-0039 (JavaScript posts). Lowering and the back-plotter remain.
- **Module:** libs/machine, libs/post, libs/document (`postSetup`)
- **Patent gate:** low (citation only). G-code syntax follows public standards and controller
  manuals (syntax facts only).
- **Author(s):** OxCAM contributors

User documentation of post configurations: [docs/posts](../posts/README.md).

## Problem

Turn machine-independent toolpaths into G-code for a specific control, and prove the output
reproduces the toolpaths.

## Approach

### Pipeline

**CL toolpath → `machine::solve` → NC IR (`post::lower`) → text (`post::format`, dialect from
a PostConfig).**

### `solve` (3-axis)

- The tool axis must be +Z throughout.
- The motion span must fit each axis's travel.
- Spindle speed and feeds must be within the machine limits (spindle min and max, max feed). They
  are never changed (ADR-0004): a value beyond a limit is an error naming the value, the limit and
  where to change them (the cutting preset or the operation's speeds and feeds, or the machine).
  Rapids carry no feed and are not checked.

Axis positions equal WCS tool-tip coordinates, because the control applies the work offset.
Later phases add inverse kinematics here.

### NC IR

A program is a sequence of typed instructions:

| Group | Instructions |
|---|---|
| Tooling | tool change, work offset, spindle, coolant, tool-length positioning |
| Motion | rapid, linear, arc (absolute centre), dwell |
| Cycles | canned cycle, cycle cancel |
| Compensation | compensation on/off |

`lower()` sequences tool changes and the safe approach:

1. position X/Y in the work offset;
2. start the spindle;
3. G43 to the start height;
4. coolant on.

It also cancels cycles before non-cycle motion. Compensation (G41/G42) starts on the first
straight XY move after a descent and is cancelled before the next rapid. Starting compensation on
an arc is reported as a warning.

### Formatter

The formatter is driven by a JSON dialect: templates, decimals, arc mode (IJK or R),
maximum arc sweep, canned-cycle use, comment style and coolant codes. It:

- keeps modal state (motion group, feed, last output axis values) and omits unchanged words;
- merges work-offset and compensation words into the next motion block;
- splits arcs longer than the maximum sweep;
- writes canned cycles as G99 G73/G81-G85 with Z, R, Q, P, F, and later holes as X/Y only;
- strips comments to plain ASCII.

Canned-cycle expansion in the CL IR follows the control: pecks are measured from the R plane
(DL-0009).

### Back-plotter

The back-plotter parses the Fanuc-style subset that the post emits, plus common forms:

- motion: G0-G4, R-format arcs;
- units and coordinates: G20/G21, G90/G91;
- cycles: G73, G80-G85, G98/G99;
- compensation and work offsets: G40-G43, G54-G59;
- comments and N numbers.

It returns a Toolpath. `compareBackplot` walks the plotted motion against the CL toolpaths in
order:

- it skips header and tool-change moves until each toolpath's start;
- it ignores moves shorter than the tolerance;
- it accepts split arcs.

It reports the first deviation in end point, arc centre, move type, feed or dwell.

## Sources

- ISO 6983-1, *Numerical control of machines — Program format and definitions of address words*
  (paraphrased).
- P. Smid, *CNC Programming Handbook*, Industrial Press: word order, modal groups, fixed cycles,
  cutter compensation practice.

## Correctness and tests

`tests/unit/post/PostTest.cpp`:

- exact text of a small program;
- golden programs for the four golden toolpaths (line-by-line, tolerant of last-digit
  rounding);
- back-plot round trips within 1e-4 mm at 4 decimals, the phase exit criterion;
- R-format arcs, inch output, expanded cycles, multi-tool programs, G41/G42 placement, ASCII
  comments, parser edge cases.

`oxcam-cli project post --verify` runs the same check on real projects. CTest runs it on the
generated sample project.

## Known limitations

- 3-axis mills only. Turning, multi-axis and multi-channel output come in later phases.
- There is no scripting (Python posts come in phase 6) and no subprograms.
- G28 is treated as non-motion by the back-plotter.
- Incremental canned-cycle depths are not supported.
- **Not yet validated on a physical controller.** Record dry runs in
  [docs/validation/controller-dry-runs.md](../validation/controller-dry-runs.md).
