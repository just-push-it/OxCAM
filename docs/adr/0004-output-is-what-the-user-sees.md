# ADR-0004: The output is exactly what the user sees

- **Status:** accepted, amended 2026-10-24 (post processors are exempt; see the end)
- **Date:** 2026-10-22

## Context

A CAM program's output moves a machine. A value the program fills in on its own, such as a
feed rate taken from a hidden default, a plunge feed derived from the cutting feed, or a feed
scaled down on small arcs, produces G-code the machinist never chose and cannot see. That is how
tools, parts and machines get broken. The maintainer's rule, from 2026-10-22:

> A plunge feed at 0 must truly be 0. This must be true for any feed or speed: we can not
> assume anything or have any backend magic logic. What the user sees must be what is output
> 100% of the time.

Earlier versions broke it in several places. They were removed when this ADR was written:

- `strategies::feedsFor` started from built-in feeds (1000 mm/min cut, 300 mm/min plunge,
  10000 rpm) and kept them wherever a preset value was zero.
- A plunge feed of 0 meant "half the cutting feed".
- Turning plunged at half its cutting feed, and assumed a 50 mm diameter for stock without one.
- Thread milling and helical boring lowered the programmed feed on small helices (edge-feed
  compensation).
- Tool library files without a plunge feed got half the cutting feed.

## Decision

**Every speed and feed in the output is a value the user can see where they set it, exactly as
shown.** Concretely:

1. **No fallbacks.** Speeds and feeds come only from the operation's cutting preset and the
   operation's own changes to it (`PresetOverrides`). A value of 0 is 0. `strategies::Feeds` is
   all zeros until set. Code never substitutes a default, a minimum, or a value derived from
   another field for a value the user left at 0.
2. **Zero is refused, not replaced.** When a zero would reach the output where it cannot work
   (a feed move with no feed rate, no spindle speed), generation stops with an error that names
   the value to set and where (`generateToolpath`). It never invents a number.
3. **No hidden adjustments.** The output does not scale, clamp or otherwise change a feed or
   speed behind the user's back (no arc feed compensation, no automatic slow-downs). A
   behaviour that changes a feed is a visible parameter with its own label (for example a
   part-off's "End feed over the last" distance and its preset's end feed), so the user sees and
   controls it.
4. **Derived values are shown.** Where one value follows from others (spindle speed from
   cutting speed and diameter, feed rate from fz × flutes × rpm, tapping feed from the pitch),
   the editors display the result next to its inputs. Plunges, ramps and lead moves have their
   own feeds (presets and operations), never derived; older presets without a lead feed led at
   their cutting feed, which becomes their lead feed.
5. **Parameter defaults have one source.** A strategy parameter's default lives in its
   `ParamDef`, which the interface shows. Code reads parameters through `ParamReader`, never
   with a second literal default (`json.value("key", 3000)`).
6. **Starting values are visible values.** A new tool's "Default" preset may hold typical
   numbers, because the user sees them in the editor before anything uses them. That is a
   value the user can read and change, not a fallback.
7. **Older files keep their meaning.** Converting an older file format is allowed only when
   it reproduces what that file already said (for example an older plunge feed share times its
   cutting feed). A missing value stays missing (0); it is not filled in.

## Consequences

- Operations with incomplete presets fail to generate, with a message, instead of producing a
  program the user did not choose. This is intended.
- Reviews treat any `value_or(<number>)`, `std::max(<minimum>, feed)`, literal default speed or
  feed, or arithmetic on a feed between the preset and the output as a defect unless the user
  sees and controls it.
- Tests that need feeds set them explicitly (`strategies::testing::Harness` does).

## Amendment (2026-10-24): post processors are exempt

The rule covers everything OxCAM does up to the NC program, the hand-over to the post
processor (DL-0011, DL-0015): strategies, toolpaths, `machine::solve` and `post::lower`. The NC
program carries the user's speeds and feeds exactly as set.

A post processor is the user's own customization of the output for one machine. It may do
anything with the program it receives, speeds and feeds included: add codes, clamp a feed to a
control's limit, compensate feeds on arcs, convert units or modes, or anything else its author
wants. The maintainer's decision, from 2026-10-24:

> Posts can do what they want, they are what's meant to customize output for a specific
> machine.

This covers every post processor, whatever it is written in ([docs/posts](../posts/README.md),
ADR-0006). It does not loosen the rule anywhere before the post: code
in `libs/strategies`, `libs/toolpath`, `libs/machine`, `libs/document` and `post::lower` still
follows the Decision above. The built-in generic posts that ship with OxCAM write the program's
values as they are, so the output of a project that uses them is still what the user sees.
