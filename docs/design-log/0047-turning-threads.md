# DL-0047: Inside, left-hand, multi-start and tapered threads

- **Status:** implemented
- **Module:** libs/strategies (`turnThread`, `Common`), libs/tooling (`turningShape`)
- **Patent gate:** low (citation only). Thread geometry from ISO 68-1 and the lathe's own moves.
- **Author(s):** OxCAM contributors

## Problem

Threading cut one kind of thread: external, right hand, a single start, parallel.

## Approach

New parameters on `turn_thread`, all visible, with their defaults in the `ParamDef`:

- **Side.** Inside starts at the crest, which inside is the minor: the part's bore where the model
  shows a tap drill, or a typed major diameter less the depth. The depth default is 0.5413 × pitch
  (ISO 68-1 internal; 0.6134 stays the external one), and the tool comes off towards the axis. On
  solid stock it is refused like inside roughing: drill first.
- **Hand.** Left hand cuts away from the chuck with the spindle forward, so its run-up lies behind
  the thread. That space needs a relief groove, and the run-up check warns if it crosses material.
- **Starts.** The lead is pitch × starts, which is the feed per revolution the program states (ADR-0004:
  it is the user's values multiplied, and the tooltip says so). Each start begins one pitch further
  along the run-up, so the helices lie a pitch apart, and the default run-up scales with the lead.
- **Taper.** The radius at the thread's end minus at its start. The whole pass keeps that slope, the
  run-up included: on G32 the taper starts where the tool does.

The spindle stays at a constant speed (`G97`) in all of them.

**In the simulation.** Threads still remove nothing (DL-0021). A helix is not axisymmetric, and part
models usually show threads at their nominal diameter, so taking material off would show as a gouge.
The toolpath and the insert on its side are drawn. A through-thread's position can be seen; its form
cannot.

## Correctness and tests

`LatheHoleTest` "threads inside, left hand, multi-start and tapered": the depth and side of an inside
thread and its refusal on solid; the run-up and warning of a left-hand one; lead, feed and the start's
shift; the taper's slope over the whole cut; the programs post and verify against their back-plot.

## Known limitations

- **G76:** threads post as G32 passes. A control's canned threading cycle is a post's to write.
- **Depth per pass:** the degressive infeed is straight in, with no flank infeed.
