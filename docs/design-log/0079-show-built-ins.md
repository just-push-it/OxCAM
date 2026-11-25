# DL-0079: Built-in examples hidden until asked for

- **Status:** implemented
- **Module:** apps/opencam-gui (`ShowBuiltins.hpp`, `MachineLibraryDialog`, `LatheFittingsDialog`,
  `PostProcessDialog`)
- **Patent gate:** low (citation only): which rows a list shows. Nothing here changes a toolpath or the output.
- **Author(s):** OxCAM contributors

## Problem

The maintainer asked on 2026-11-07: "In general, when there is a dialog with build in presets, we should have an option to show
built-ins, hide them by default. Will help declutter." The machine library listed every built-in machine above the user's own,
the turret blocks and centres library its built-in examples, and a setup's post choice every built-in post for the machine.

## Approach

One check box, **Show built-ins** (`newShowBuiltinsBox`, object name `showBuiltins`), under the list of each dialog that lists
built-in entries next to the user's own. It is off when the dialog opens, and is not remembered.

- **Machine library** and **Turret Blocks and Tailstock Centres:** the built-in rows are hidden, not removed, so the rows keep
  their places. A built-in that was current gives way to the first entry shown. With nothing of the user's and the box off, the
  list is empty and the editor is disabled. A dialog opened on a built-in (the setup's machine is one, or a test selects one)
  ticks the box, so what it was opened on is shown.
- **Post settings of a setup** (and Post process): the machine's default post is always listed, whatever it is, and so is the
  post the setup has chosen. The other built-in posts are listed with the box ticked.

Not changed:
- **New machine** offers the built-in machines as templates: there they are the choice, not clutter.
- **The machine library's default post** keeps "Built-in for its kinematics" and the built-in posts. A machine's post is usually a
  built-in one.
- **The setup page's lists** (machine, turret blocks, centre) are not dialogs. They offer only what fits.
- **Workholding, tool libraries and holders** have no built-in entries.

## Sources

None.

## Correctness and tests

`gui.show_builtins` (VM), with throwaway libraries and settings of its own:
- **Machine library:** one machine of the user's is the only row and is current, with the box off. Ticked, every built-in shows
  as well. Unticked, the user's machine is current again. Selecting a built-in ticks the box.
- **Blocks and centres library:** the same with one block of the user's.
- **Post settings:** the turret project's setup lists only the machine's default and "Add a post file..." with the box off.
  Ticked, the two built-in lathe posts are listed too. A built-in chosen stays listed after unticking.

## Known limitations

- The box is not remembered between openings.
