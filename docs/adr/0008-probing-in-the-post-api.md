# ADR-0008: Probing cycles in the post API

- **Status:** accepted (points 1, 2, 3 and 5 as proposed; point 4 as the maintainer decided)
- **Date:** 2026-11-09

## Context

OxCAM gains probing operations with a spindle probe (DL-0090). What a probe measures is used on
the machine, by the control: a measuring move stops where the stylus triggers, and a program on
the control computes the feature from the trigger positions and sets a work offset or checks a
tolerance. Probe makers ship those programs as macros with their own numbers and argument
letters; they differ by maker and control, and copying them is forbidden (clean-room rule,
CONTRIBUTING.md). Posts are JavaScript only (ADR-0006), and a post may change anything in the
output (ADR-0004 amendment).

The post API has to carry a probing cycle so that:

- a post for a machine with a probe can call that probe's macros, with everything they need;
- a post without them still writes something safe and says what it does not do;
- the moves the simulation checked are the ones the program makes.

## Decision

1. **One `probe` instruction per cycle** (NC program version 6). It carries its `operation`
   (index into the program's operations), what the cycle
   measures (`feature`, `nominal`, `direction`, `size`), what it does with it (`work_offset` and
   `axes`, or `check` with `tolerance`), its distances and feeds (`approach`, `overtravel`,
   `fast_feed`, `measure_feed`), the probe (`stylus_diameter`, `trigger_offset`) and its `moves`
   in order. Each move is `position` (a protected move) or `measure`; a measuring move gives its
   `target` (past the surface by the overtravel) and `expected` (where it stops on the model's
   surface). The instructions around it are the usual ones: tool change, work offset, tool length,
   rapids to and from the cycle. The spindle does not start for a probe.
2. **The toolpath IR has the same moves** (`MoveKind::Probe`), so the simulation checks what the
   post receives.
3. **`oxcam:fanuc` handles `probe` with named hooks** a post overrides: `probeOn(cycle)` before
   the first cycle after a tool change, `probeBegin(cycle)`, `probePosition(move, cycle)`,
   `probeMeasure(move, cycle)`, `probeEnd(cycle)`, and `probeOff()` before the next tool change
   and at the program's end. After a cycle every axis, the motion code and the feed are written
   again.
4. **A cycle that sets a work offset needs a post that applies it.** Plain G-code cannot apply a
   measurement, and a program that probes and then cuts on the old offset is worse than none.
   So the base refuses to post a cycle that sets a work offset unless the post overrides
   `probeEnd`; the refusal names the operation, the offset it sets, and that the post needs a
   `probeEnd` that applies the result. A feature check (`check`, work offset 0) changes nothing
   the program goes on to use: the generic output writes its moves as plain G-code (every move a
   skip move, `probeSkip`, G31 by default; protected moves at the positioning feed, measuring
   moves at the measuring feed), between a comment naming the cycle and one saying the check is
   not made, and warns once per program.

   **How a post declares that it applies the result:** by overriding `probeEnd`. `probe()`
   compares `this.probeEnd` with `FanucMill.prototype.probeEnd`, the base's own; any other
   function (the post's, or a class between them) counts as applying it. No manifest flag is
   added: a flag could say one thing and the code another, and the override is where the work
   is done. A post that replaces `probe()` itself takes on the whole cycle, this check
   included.
5. **No vendor macros ship.** Examples use made-up program numbers; a post takes its probe's from
   the probe maker's manual.

A post that does not extend `oxcam:fanuc` and has no `probe` handler is refused by the engine
("the post has no handler 'probe'") when a program has probing; no manifest declaration is added.

## Consequences

- `post::kNcProgramJsonVersion` becomes 5; version 1-4 files still read. The engine's field sink
  gains arrays (`moves`).
- A post written for an earlier version that does not extend the base fails on programs with
  probing, by name, and on nothing else.
- The built-in posts refuse probing cycles that set work offsets and write feature checks as
  moves without a result. Setting work offsets from the probe needs a machine post with the
  probe's macros; docs/posts/fanuc-base.md shows how.
- Backplot (`--verify`) does not read G31: programs with probing are not verified that way.
