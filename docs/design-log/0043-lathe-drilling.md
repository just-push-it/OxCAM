# DL-0043: Drilling, tapping and reaming on the lathe's spindle axis

- **Status:** implemented
- **Module:** libs/tooling (`holeToolOutline`), libs/strategies (`turnRough`, `hole_depth`),
  libs/document (`generateToolpath`, `latheStockBefore`, `simulateSetup`), libs/machine (`solve`),
  libs/post (`Lower`, `FanucLathe`, back-plot), apps/opencam-gui
- **Patent gate:** low (citation only). A tool fed along the spindle axis and plain geometry in the
  lathe plane; G-code syntax facts come from controller documentation.
- **Author(s):** OxCAM contributors

## Problem

A lathe setup refused every rotating tool, so a bar could not be centre drilled, drilled or tapped.
Inside roughing needs a bore to start in, which left tube stock as the only way to bore a part.

Inside roughing also read the setup's first stock, so it could not have seen a drilled bore. And
roughing removed everything between the stock and the part, whichever side it lay on: outside
passes ran down a bored part's hole, and inside passes ran out past the part's outside.

## Approach

**The operations.** Lathe setups use the existing Drill, Tap and Bore operations, with a drill,
spot drill, centre drill, tap, reamer or boring bar that stands still in the turret. Thread milling
and helical boring turn the tool around the hole, which needs live tooling, and are refused. So
are selected faces: there is one place to drill.

**Where the hole goes.** Everything happens on X0. The tool's silhouette in the lathe plane, swept
along the axis, is tested against the part's profile:

- Without a depth, the tool goes as deep as the part lets it: through a through bore (plus the
  breakthrough), or until its tip meets the part. The silhouette is 0.025 mm under the tool's
  radius, the ±0.05 mm on the diameter that hole matching already allows.
- A tap's silhouette is the tap drill (major diameter − 1.5 × pitch), since a tap cuts into the
  wall by design.
- A part with nothing on the axis and no depth set is refused, naming **Hole depth**.
- **Hole depth** (Drill and Bore) and **Thread depth** (Tap) set it by hand, from the part's front.
  A depth that cuts into the part, as a centre hole does, is a warning: it may well be what the
  user wants, and the simulation shows it.

**Where it starts.** The hole's top is the front of the stock the operation finds, within the
tool's radius, so a drill after facing starts at the faced surface. The feed height still sets the
R plane above it.

**The stock an operation finds (`latheStockBefore`).** The setup's stock less what the earlier
enabled operations cut, simulated from their cached toolpaths (milliseconds, DL-0021). An operation
that failed is left out; it shows its own error.

- Hole operations see all earlier operations.
- Turning operations see only the earlier holes. Their stock bounds set their start and approach,
  and those stay where they are for parts without holes.

The toolpath cache key takes in the same earlier operations.

**Refusals.** A tap, reamer or boring bar needs a hole to run in. If the core of its path (a tap's
tap drill, half the radius of the others) meets stock, generating refuses and says to drill first.
A hole operation reaching the chuck is refused like a turning one, by its silhouette.

**Speeds and feeds (ADR-0004).** A hole runs at the constant spindle speed its preset derives for
the tool's diameter (`G97`): a surface speed would run away towards the axis. The feed is the
preset's, posted per revolution like all lathe feeds. Nothing is limited: a speed above the
machine's maximum is refused as in milling.

**Roughing (`turnRough`).** Material is now limited to what the side can reach. Outside passes take
what lies above the part's outer envelope (part offset by the allowance), inside passes what lies
below its inner envelope. The envelope has a vertex at each vertex of the offset part's outline.
Solid parts are unaffected: everything under their envelope is part already.

**Simulation.** The silhouette is the cutter, and the sweep is the one turning uses, on the cycles
expanded into moves. It is convex: a centre drill's shoulder between pilot and countersink is
bridged, which removes slightly more there than the real tool. A tap removes nothing, like every
thread (DL-0021).

**Posting.** Drilling and boring cycles stay expanded into moves on the generic lathe. A tap cannot:
the spindle has to reverse at the bottom, which plain moves do not do. So:

- The generic lathe takes the tap cycle, posted as `G84 X0. Z<bottom> F<lead>`, in feed per
  revolution (`G99`), with the tool already at the start height. No R word: Fanuc reads R from the
  initial level on lathes, Haas as a position, and leaving it out starts the cycle where the tool
  is.
- A post that takes no tap cycle is now refused for tapping, in milling as well. Expanded, the tap
  would run back out with the spindle still going forward and strip the thread.

## Correctness and tests

- **Toolpaths:** a bored part (through and blind) drills to the expected depths from the faced
  front; hole depth, thread depth, off-axis and refusal paths.
- **End to end:** face, drill, rough and finish outside and inside on solid bar leaves no gouge and
  no excess on the part. Outside passes stay out of the bore, inside passes out of the wall.
- **Post:** the lathe program holds the expected blocks and verifies against its back-plot; a tap
  without a tap cycle in the post is refused.

## Known limitations

- **Tool length:** only the cutting length is simulated and checked. A shank or holder running into
  the stock or the chuck is not seen (the same gap as turning holders).
- **Rigid tapping:** the generic post's `G84` is the plain cycle. A control that needs `M29` needs a
  post of its own.
- **Left-hand taps:** like milling, the cycle assumes a right-hand tap.
- **Simulation display:** the tool shows as its flat silhouette, like an insert, not revolved.
- **Off-axis holes:** cross holes and bolt circles need live tooling (mill-turn), not done here.
- **Inside roughing after a wide centre hole:** passes start from the radius at the stock's front,
  so a centre hole wider than the drill that follows makes the first pass heavier.
