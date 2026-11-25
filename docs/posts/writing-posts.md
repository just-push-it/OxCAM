# Writing a post

A post is one JavaScript file: an ES module with three exports. This page describes them, the
rules posts run under, and the usual way to write one: start from the generic Fanuc-style base
and change what your control does differently.

```js
// SPDX-License-Identifier: GPL-3.0-or-later
// Sources: <controller programming manual, edition, the sections used>
import { out } from "oxcam:post";
import { FanucMill } from "oxcam:fanuc";

export const manifest = {
  id: "shop-vmc",
  name: "Shop VMC (Fanuc-style)",
  version: "1.0.0",
  engine: 1,
  machines: ["mill3"],
  extension: "nc",
  arcs: { maxSweep: 180, planes: ["xy", "zx", "yz"], helical: true },
  cycles: ["drill", "dwell", "peck", "chip_break", "tap", "tap_left", "bore"],
  verify: "fanuc-mill",
};

export const options = [
  { id: "optionalStop", label: "Optional stop before each tool", type: "bool", default: true },
];

export default class extends FanucMill {
  constructor(context) {
    super(context);
    this.safeStart = "G17 G21 G40 G49 G80 G90 G94";
  }
  toolChange(i) {
    if (this.options.optionalStop) out.block("M01");
    super.toolChange(i);
  }
}
```

## The manifest

What the post is and what its control can take. Unknown keys are errors, so a misspelled
declaration cannot silently do nothing.

| Key | Default | Meaning |
|---|---|---|
| `id` | required | Lower-case letters, digits, `.`, `_`, `-`. How setups and machines refer to the post. |
| `name` | required | Shown in the GUI and messages. |
| `version` | required | The post's version, e.g. `"1.2.0"`. A post can write it into the program. |
| `engine` | required | The engine API version it needs (1). Newer than OxCAM's: the post is refused. |
| `machines` | required | The machine kinematics it writes for: `mill3`, `mill4_a`, `mill5_table_ac`, `mill5_table_bc`, `mill5_head_table_bc`, `mill5_head_bc`, `lathe2`. Posting for another machine is refused. |
| `extension` | `"nc"` | File extension of the program (1-8 letters or digits). |
| `arcs.maxSweep` | `180` | [deg] Longer arcs are split into equal parts. |
| `arcs.planes` | all | `"xy"`, `"zx"`, `"yz"`: arcs in other planes become lines. |
| `arcs.helical` | `true` | `false`: helical arcs become lines. |
| `tolerance` | `0.002` | [mm] How far lines made from arcs may stray from them. |
| `cycles` | all seven | Canned cycle types the control takes: `drill`, `dwell`, `peck`, `chip_break`, `tap`, `tap_left`, `bore`. Others become moves (`[]`: none). |
| `multiAxis` | `"tcp"` | `"tcp"`: tool tip positions with tool centre point control; `"machine"`: machine positions with inverse-time feeds on rotary moves. |
| `verify` | `null` | Back-plot dialect that reads the output: `"fanuc-mill"`, `"fanuc-lathe"`, or `null`. |
| `subSpindle` | `false` | `true`: it writes a sub-spindle's work and the transfer to it (`sub_axis`, `spindle_sync`, and `chuck` and operations on the sub-spindle; ADR-0007). Without it such a program is refused. |
| `encoding` | `"ascii"` | The output may hold only printable ASCII and tabs. |
| `workOffsets` | G54-G59, G54.1 P1-P48 | The work offsets the control has and how they are written: a list of `{ from, to, code, start, name }`. Setup work offsets `from` to `to` are written as `code` with `{}` replaced by `start + (n - from)`: `[{ from: 1, to: 6, code: "G{}", start: 54 }, { from: 7, to: 105, code: "G154 P{}", start: 1 }]` is G54-G59, then G154 P1-P99. `name` is how the user picks the range (unnamed: "Standard" for the first, "Extended" for the next). The setup page offers the ranges by name and the number in the code ("G154 P" 2), as far as the post declares and the machine has (machine library: *Work offsets*); posting refuses others. The Fanuc base writes them (`workOffsetCode(n)`). |

## Options

Values the user sets per setup (the setup page) or per operation (the operation page).

| Key | Meaning |
|---|---|
| `id` | Identifier (`[A-Za-z_][A-Za-z0-9_]*`); the key in `this.options` or `op.options`. |
| `label` | Shown in the editor. |
| `type` | `"bool"`, `"number"`, `"choice"` or `"text"`. |
| `default` | Required, and must fit the type. |
| `scope` | `"program"` (default): `this.options`. `"operation"`: `op.options` in operation handlers. |
| `choices` | Choice options: the values offered. |
| `min`, `max` | Number options: the range. |

Values are checked before the post runs: a wrong type, a value outside the range or an unknown
option is an error naming it.

## The default export

A class. OxCAM constructs it once per run with a context:

| Context | What it is |
|---|---|
| `context.program` | The program: name, number, project, setup, time, post, tools, operations, machine, options, `peek()` ([nc-program.md](nc-program.md)). |
| `context.options` | The program option values (also `context.program.options`). |
| `context.out` | The output object (also exported by `oxcam:post`). |

The base classes keep them as `this.program`, `this.options`, `this.out`.

Then OxCAM calls one method per NC program instruction, with the instruction as a plain object:

| Method | Called |
|---|---|
| `programBegin(program)` | once, first |
| `operationBegin(op)`, `operationEnd(op)` | around each operation; `op` is `program.operations[i]` |
| `comment`, `toolChange`, `workOffset`, `spindle`, `coolant`, `toolLength`, `rapid`, `linear`, `arc`, `dwell`, `cycle`, `cycleCancel`, `compensationOn`, `compensationOff`, `index`, `homeZ`, `tcpOff` | per instruction ([nc-program.md](nc-program.md#instructions)) |
| `programEnd(program)` | once, last |

A missing method is an error naming it, so a post written for an older OxCAM fails loudly on a
new instruction rather than dropping it. Methods must finish before they return (no `async`).

## Rules posts run under

Posts run sandboxed:

- they can import only OxCAM's modules: `oxcam:post` and `oxcam:fanuc`;
- no files, network, processes or environment; no `Date` or `performance` (the clock):
  `program.time` is when the program was made; `Math.random` throws, so the same program
  always posts to the same text;
- a memory limit (1 GiB) and a stack limit; **Cancel** stops a post, even an endless loop;
- everything written goes through `out` and is checked to be printable ASCII.

Each run evaluates the module afresh: nothing carries over between runs.

## Errors

- A JavaScript error stops posting with the post's file, line, column and message:
  `my-vmc.js:42:7: TypeError: cannot read property 'x' of undefined`.
- `error("message")` from `oxcam:post` stops posting with `my-vmc.js: message`: use it to
  refuse what the machine cannot do ("tool 31: the magazine has 24 pockets").
- `warning("message")` lets posting go on and shows the message with the program.

## A walk-through

1. **Copy** [examples/posts/example-vmc.js](../../examples/posts/example-vmc.js) or a built-in
   post from [data/posts](../../data/posts), and give it its own `id` and `name`.
2. **Set the manifest** from your control's manual: the machines, the canned cycles it has,
   the arc planes, whether it has tool centre point control.
3. **Change settings** of the base in the constructor ([fanuc-base.md](fanuc-base.md)): the
   header, safe start, tool change, end, coolant codes, decimals.
4. **Override handlers** where the control differs, and call `super` for the rest.
5. **Post a project** with it and read the program:
   `oxcam-cli project post part.ocam --post my-vmc.js --verify -o part.nc`.
6. **Put it in your post library** folder to choose it in the GUI.
7. **Dry-run it** on the machine before cutting.

Debugging: add `warning(...)` calls to see values; they appear with the program. `--ir
program.json` shows the NC program the post receives.
