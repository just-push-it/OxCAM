# DL-0022: Lathe machine definition, lathe post dialect and back-plotting

- **Status:** implemented (phase 8)
- **Module:** libs/machine (`Kinematics::Lathe2`, `generic-2axis-lathe`), libs/post (lathe
  dialect, `generic Fanuc-style lathe` post, lathe back-plot), libs/document (`defaultPostFor`),
  apps, Python post base class
- **Patent gate:** not applicable. G-code syntax facts come from controller manuals.
- **Author(s):** OxCAM contributors

## Machine

`lathe2` kinematics has X (radius travel) and Z axes. Machine files may now list two axes.

Solve checks two things:

- Every move lies in the XZ plane.
- The toolpath is a turning toolpath. Milling operations need live tooling, which is phase 10.

The built-in `generic-2axis-lathe` has a 4000 rpm spindle and a 12-station turret.

## Post dialect

A post configuration with `"lathe": true`:

| Area | Behaviour |
|---|---|
| Axis words | X words are **diameters** and Y words are never output. |
| Spindle | Toolpaths with a surface speed post `G50 S<limit>` and `G96 S<m/min> M03` (ft/min for inch posts). Others post `G97 S<rpm> M03`. |
| Feeds | With `"feed_per_rev": true`, F is the feed per revolution: the IR feed (mm/s) divided by the nominal spindle speed it was computed with, to `decimals.feed_per_rev`. |
| Threads | `Move::pitch` moves post as `G32 Z.. F<lead>`. |
| Tool changes | Use the `{tool2}` placeholder (two digits), for example `T{tool2}{tool2}` → `T0101`. The empty tool length offset template emits nothing (the T word applies the offsets). |

The built-in `fanuc-generic-lathe.json`:

- **Program start:** `G18 G21 G40 G80 G99`.
- **Program end:** `G28 U0.` / `G28 W0.` / `M30`.
- **Canned cycles:** off, except tapping since [DL-0043](0043-lathe-drilling.md).

`document::defaultPostFor(machine)` picks it for lathe setups. The CLI, GUI and Python use that
when no post is given.

**The NC program JSON** (DL-0015) carries the turning data:

- `pitch` on linear moves;
- `surface_speed` and `max_rpm` on spindle instructions.

The Python `Post` base class has a `lathe = True` mode with the same conventions.

## Back-plotting

`backplot(gcode, {lathe = true})`:

- halves X;
- treats a T word without M06 as a tool change (T0101 → tool 1);
- reads G98/G99 as feed per minute/revolution (instead of the canned-cycle return modes);
- accepts G18, G50, G96 and G97;
- reads G32 as a threading move.

Verification of lathe programs compares positions only (`compareBackplot(..., compareFeeds =
false)`). Under constant surface speed the actual feed in mm/min depends on an rpm the program
does not state.

## Correctness and tests

- **Post (`tests/unit/post`):** a turning rough plus a thread posts with all of `G18`, `T0101`,
  `T0303`, `G50 S3000`, `G96 S200 M03`, `G97 S1500 M03` (threads), `G32 … F1.5`, `F0.2` per
  revolution, diameters and no Y words. The lathe back-plot matches, and the NC JSON
  round-trips.
- **Machine:** the lathe validates and is found by id.
- **CLI `cli.project_post_lathe`:** the shaft project posts with `--verify`.
- **Python:** the shaft project posts with the built-in and the Python lathe post.

## Known limitations

- **Canned turning cycles:** G70–G76 are not emitted; programs are explicit moves.
- **Radius programming:** a radius-mode lathe dialect is not offered.
- **Turret indexing and sub-spindles:** these are phases 10 and 11.
