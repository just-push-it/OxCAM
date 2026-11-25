# DL-0039: JavaScript post engine and NC program version 2

- **Status:** implemented (all eight steps: the engine, the output library, NC program version
  2, the JavaScript built-in posts, the switch-over, declarations in lowering, post choice and
  options, the documentation in docs/posts)
- **Module:** libs/post (engine, output library, lowering by declarations), libs/document
  (`lowerSetup`, post choice and options), data/posts, apps (GUI, CLI), bindings/python
- **Patent gate:** not applicable (no machining algorithm). Arc linearization uses the sagitta
  bound of DL-0001.
- **Author(s):** OxCAM contributors

Implements [ADR-0006](../adr/0006-javascript-post-processors.md). Replaces the JSON post
configurations and the C++ formatter of DL-0011 and the Python post base class of DL-0015.

## Problem

Run post processors written in JavaScript inside OxCAM:

- **Input:** a setup's NC program (DL-0015, extended here), the machine, the post, and the
  user's option values.
- **Output:** the program text, warnings, or errors that stop posting.
- **Invariants:**
  - A post can only produce text and messages. It cannot reach files, the network, other
    processes or the environment.
  - The same inputs give the same text, byte for byte (tests and golden files rely on it).
  - Posting can be cancelled, and a failing post reports its file, line and message.
  - The engine ships inside OxCAM. Nothing is installed or looked up at run time.

## Approach

### Pipeline

```
lowerSetup ─▶ post::lower(jobs, declarations) ─▶ NcProgram v2 ─▶ PostScript::run ─▶ text
                   ▲
                   └────────── manifest declarations ◀── PostScript::load
```

1. `PostScript::load(source, name)` compiles the post and reads its `manifest` and `options`.
2. `lowerSetup` lowers with the manifest's declarations (section "Declarations").
3. `PostScript::run` gives the program to the post's handlers and collects the output.

`load` and every `run` each create a QuickJS runtime of their own and destroy it when done; `run`
evaluates the module again, so nothing a load or an earlier run left behind (module-level
variables) reaches it. A `PostScript` holds only the source, manifest and options: it can be
loaded on one thread and run on another, and there is no global state, so posts run on worker
threads (the GUI posts off the UI thread).

### The engine: QuickJS-ng

The engine is QuickJS-ng (vcpkg `quickjs-ng`, MIT), linked privately into `libs/post`. The
public header `ocam/post/Engine.hpp` exposes no QuickJS types.

**Context.** The context is created bare (`JS_NewContextRaw`), with only the language
intrinsics added: base objects (BigInt included), eval (needed to compile modules), RegExp, JSON,
Map/Set, typed arrays, Proxy, Promise. What `JS_NewContext` would add beyond that is left out:

- **QuickJS's `std` and `os` modules (quickjs-libc).** The vcpkg port builds without them (its
  `libc` feature is off), and they are never registered. They are the only file, process and
  network access QuickJS has.
- **`Date` and `performance`.** They read the clock, which would make output depend on when it
  ran. The host passes the run's time as `program.time` instead, and tests fix it.
- **`WeakRef` and `FinalizationRegistry`.** Their behaviour depends on garbage collection.
- **`atob` / `btoa`.** Not needed.
- **`Math.random`.** It is replaced by a function that throws ("Math.random is not available in
  posts"), so output cannot change from run to run.

**Modules.** A post is an ES module. The module loader resolves only `oxcam:` names:

| Module | What it is |
|---|---|
| `oxcam:post` | Output library, native C++ (section "Output library") |
| `oxcam:fanuc` | Generic Fanuc-style base posts, JavaScript, embedded in the binary |

Any other import, including relative paths, fails with "posts can import only oxcam: modules".
The base modules are part of the engine and versioned with it.

**Limits.**

- Memory limit: 1 GiB, enforced by the engine's own allocator (`JS_NewRuntime2`), which keeps
  each block's size in a 16-byte header. QuickJS's built-in limit fails without anything to
  report (it cannot allocate the error object either); counting in the allocator lets the
  engine say "the post ran out of memory (limit 1024 MiB)".
- Stack limit: `JS_SetMaxStackSize`, 512 KiB: below the 1 MiB default thread stack of Windows,
  so endless recursion ends in a RangeError before the native stack overflows.
- Cancellation: an interrupt handler (`JS_SetInterruptHandler`) polls the `CancelToken`. A
  cancelled run throws `OperationCancelled` like other kernels.
- An endless loop in a post is stopped the same way, by cancelling.

**Errors.** A JavaScript exception becomes an `ocam::Error` reading
"`post.js:12:5: RangeError: message`": the post's file name, the line and column of its first
frame in QuickJS's stack trace, the error type and message. `error(message)` from `oxcam:post`
stops posting with "`post.js: message`", without a location. Failures while loading are
`ErrorCode::Parse` (`Unsupported` for a post that needs a newer engine); failures while running
are `ErrorCode::InvalidArgument`.

### A post module

```js
// SPDX-License-Identifier: GPL-3.0-or-later
// Sources: <controller manual, edition, sections used>
import { FanucMill } from "oxcam:fanuc";

export const manifest = {
  id: "example-vmc",
  name: "Example VMC (Fanuc-style)",
  version: "1.0.0",
  engine: 1,                       // engine API version it needs
  machines: ["mill3", "mill5_table_ac"], // machine kinematics it can be chosen for
  extension: "nc",
  arcs: { maxSweep: 180, planes: ["xy", "zx", "yz"], helical: true },
  tolerance: 0.002,                // [mm] arcs the post cannot take become lines within this
  cycles: ["drill", "dwell", "peck", "chip_break", "tap", "bore"],
  multiAxis: "tcp",                // or "machine": machine positions, inverse-time feeds
  verify: "fanuc-mill",            // back-plot dialect, or null
};

export const options = [
  { id: "optionalStop", label: "Optional stop before each tool", type: "bool",
    default: false, scope: "program" },
];

export default class extends FanucMill {
  toolChange(i) {
    if (this.options.optionalStop) this.out.block("M01");
    super.toolChange(i);
  }
}
```

- `manifest`, `options` and the default export are required. The engine validates the
  manifest and options when the post loads. Unknown keys are errors, so a misspelled
  declaration cannot silently do nothing.
- `engine` is the API version. A post that needs a newer engine than OxCAM's is refused, with
  both versions in the message.
- The default export is a class. The engine constructs it once per run with the run's context
  (`program`, `out`, `options`), which the base class stores as `this.program`, `this.out` and
  `this.options`.

### Handlers

The engine walks the program and calls one method per instruction, with the instruction as a
plain object. The method names follow the NC program ops (DL-0015), in camelCase:

| Method | Called |
|---|---|
| `programBegin(program)` | once, first |
| `operationBegin(op)` / `operationEnd(op)` | around each operation (`op` from `program.operations`) |
| `comment`, `toolChange`, `workOffset`, `spindle`, `coolant`, `toolLength`, `rapid`, `linear`, `arc`, `dwell`, `cycle`, `cycleCancel`, `compensationOn`, `compensationOff`, `index`, `homeZ`, `tcpOff` | per instruction |
| `programEnd(program)` | once, last |

A missing method is an error naming the op. A post written for an older engine therefore fails
loudly on a new op rather than dropping it. The base classes define every method.

### What the post receives: NC program version 2

Data fields keep the NC program JSON names (snake_case). The objects a post receives are
exactly what `oxcam-cli project post --ir` writes, so that file shows what a post sees
(`post::toJson`, `libs/post/src/NcJson.cpp`). Units are unchanged: mm, mm/min, rev/min, s,
degrees for rotary positions (`units.angle` is now stated), m/min for surface speed. `null`
coordinates mean "unchanged". Version 1 documents are still read: no operations, tools or
machine, every move a cutting move.

Changes from version 1:

| Change | Why |
|---|---|
| `operation_begin` (`operation`: index into `operations`) and `operation_end` replace the operation-name `comment` | Posts decide what each operation starts and ends with |
| `feed_class` on `linear` and `arc`: `cut`, `plunge`, `ramp`, `lead_in`, `lead_out`, `link` | The toolpath records it (`Move::feedClass`); lowering dropped it |
| Top-level `project`, `setup`, `time`, `tools`, `operations`, `machine` | Look-ahead and context, below. `post` and `options` are added by the engine for the run; they are not in the `--ir` file. |

**Program-level fields** (also on `this.program`):

| Field | Content |
|---|---|
| `name`, `number`, `project`, `setup` | Program name and number as today; project and setup names |
| `time` | When `lowerSetup` made the program, ISO 8601 in UTC to the second ("2026-10-25T14:03:07Z"); tests set it |
| `post` | The manifest's `id`, `name` and `version`, for traceability in the output |
| `tools` | Every tool the program uses, in order of first use: `number`, `name`, `comment`, `type` (`flat_end_mill`, ...), `diameter`, `corner_radius`, `flutes`, `thread_pitch`, `gauge_length` (tool and holder), and for turning tools `insert` (shape, nose radius, orientation) |
| `operations` | Per operation: `name`, `comment`, `strategy` (operation type id), `tool` (number), `coolant`, `work_offset`, `orientation` (`"3axis"`, `"indexed"` with `index` {a, b, c}, `"simultaneous"`), `bounds` {min, max} of its motion in program coordinates (the start, every end point, the quarter points of arcs, hole bottoms and R planes), and from step 7 `options` (per-operation option values) |
| `machine` | `id`, `name`, `kinematics`, `axes` [{name, min, max}], `rotary_axes`, `spindle` {min, max} [rev/min], `max_feed` and `rapid` [mm/min], `tool_capacity`, `work_offsets` |
| `options` | Program option values (also `this.options`) |

**Look-ahead.** `program.operations` and `program.tools` are complete up front.
`program.peek(k)` returns the k-th instruction after the one being handled (`peek()` =
`peek(1)` = the next; in `programBegin` the k-th of the program), or `null` past the end and in
`programEnd`; `k` must be a whole number of 1 or more. Instructions become objects only as they
are dispatched or peeked, so long programs do not hold millions of objects. They are built
straight from the `NcInstr`, with property names made once per run, from the same field list
`post::toJson` writes (`libs/post/src/NcFields.hpp`), so an `--ir` file still shows exactly what a
post receives. `operationBegin` and `operationEnd` receive the
operation object itself: `op === this.program.operations[i]`.

### Declarations: how a post reshapes the program

`post::LowerOptions` is built from the manifest. It replaces `cannedCycles` and `tcp` and the
formatter's `maxArcSweep`:

| Declaration | Lowering does |
|---|---|
| `arcs.planes` | Arcs in a plane not listed become lines |
| `arcs.helical` (false) | Helical arcs (start and end at different heights along the normal) become lines |
| `arcs.maxSweep` [deg] | Longer arcs are split into equal parts (moved here from the formatter, same split) |
| `tolerance` [mm] | Arcs become lines whose sagitta r(1 − cos φ/2) stays within it (DL-0001) |
| `cycles` | Cycles of types not listed are expanded into moves (`toolpath::expandCycles`, per type) |
| `multiAxis` | `"tcp"`: solve for tool centre point control; `"machine"`: machine positions with inverse-time feeds |
| `machines` | Posting with it for a machine of other kinematics is refused (`document::postSetup`), naming the machines it writes for |

Split arcs use the C++ formatter's split (the generic posts' output did not change when it
moved from the base post into lowering). Lines made from an arc keep its feed and feed class;
cutter compensation can start on them. Kept cycles are renumbered; the tool is taken to be at
the R plane after each (G99). These are geometry changes the post asked
for. Feeds and speeds are not touched before the post (ADR-0004).

### Output library (`oxcam:post`)

Native C++ functions and classes (`libs/post/src/OutputLibrary.cpp`). The names come from
OxCAM's own vocabulary (ISO 6983 terms *block* and *word*, the NC program ops). They are not
modelled on any commercial CAM post API, and reviews reject names or structures copied from one.
Settings objects are checked like the manifest: an unknown setting is an error.

| Export | Does |
|---|---|
| `number({decimals, point, zeros, scale})` | A number format; `f.format(value)` returns its text. `decimals` (0-9) is required. `point: "always"` (the default) writes `10.`, `"needed"` writes `10`. Rounds half away from zero, removes trailing zeros (unless `zeros: "keep"`), never writes `-0`, multiplies by `scale` (default 1) first: 1/25.4 for inches, 2 for lathe diameters. Non-finite values are errors. These are the C++ formatter's rules (DL-0011). |
| `word(letter, format, {modal})` | An address word. `w.of(value)` returns `"X10."`, or `""` when `modal` (the default) and the text equals the last one written. `w.of(null)` and `w.of(undefined)` return `""` and change nothing: the NC program's "unchanged". `w.reset()` makes the next call write; `w.last` is the last text (`null` after a reset), `w.letter` the address. |
| `code()` | A modal code group (motion, plane, feed mode, ...): `g.of("G1")` returns `"G1"`, or `""` when it is the last code written; `g.of("")` returns `""`. `reset()` and `last` as for words. |
| `out.block(...words)` | Writes one block: skips empty, `null` and `undefined` words, joins the rest with `out.separator` (default `" "`), and prepends `N<number>` and the separator if numbering is on. A block without words is not written and uses no number. Words must be text: a number is an error ("format numbers with number() or word()"), so nothing reaches the output unformatted. |
| `out.skippable(...words)` | The same block prefixed with `/` (block delete): `/N10 M01`. |
| `out.line(text)` | Writes a line as is, without a number (`%`, headers). |
| `out.comment(text)` | Writes a comment line, never numbered, in `out.commentStyle`: `"parentheses"` (the default) `(TEXT)`, `"semicolon"` `; TEXT`, or `"none"` (not written). The text is cleaned as the C++ formatter does: `Ø`/`ø` become `D`, `°` becomes `DEG`, then `(`, `)`, `;`, line breaks and other non-ASCII characters are removed; it is upper-cased when `out.upperComments` is true (default false). |
| `out.commentText(text)` | The cleaned text alone, for comments inside templates (`O1000 (NAME)`). |
| `out.numbering({start, step, max})` | Line numbers on, from `start` (default 10) in steps of `step` (default 10); after `max` (default 99999) they start again at `start`. `out.numbering(null)` or `false` turns them off. Off until a post turns them on. |
| `out.buffer()` | A buffer; `b.text` is what it holds. |
| `out.into(buffer, fn)` | Runs `fn` with everything written going into `buffer` (line numbers continue), then back where it was, also when `fn` throws. Buffers nest. |
| `out.append(buffer)` | Writes a buffer's text where output goes now (not into itself). |
| `out.separator`, `out.commentStyle`, `out.upperComments` | Settings, read and written as properties. |
| `warning(message)`, `error(message)` | Report a warning, or stop posting with a message |

The engine writes the text with `\n` line ends. With `manifest.encoding: "ascii"` (the default),
the output may hold only printable ASCII and tabs; anything else is an error naming the output
line and the byte ("line 2 of the output has a character that is not printable ASCII (byte
0xC3)").

### `oxcam:fanuc`: the base posts

`libs/post/js/fanuc.js`, embedded in the engine. `FanucMill` and `FanucLathe` re-implement the
C++ formatter (DL-0011, DL-0022, DL-0023) in JavaScript, step for step and with the same
arithmetic, so their output is the formatter's byte for byte. A post changes their settings in
its constructor, after `super(context)`:

| Setting | `FanucMill` | `FanucLathe` |
|---|---|---|
| `units` | `"mm"` (`"in"`: lengths and feeds divided by 25.4) | |
| `lineNumbers` | `{start: 10, step: 10, max: 99999}` (`null`: none) | |
| `decimals` | `{linear: 3, feed: 0, dwell: 3, rotary: 3, feedPerRev: 3}` | |
| `trailingPoint` | `true` (`10.`) | |
| `arcFormat`, `maxArcSweep` | `"ijk"` (or `"r"`), 180 [deg] | |
| `dwellInMilliseconds` | `true` (`G4 P500`) | |
| `commentStyle`, `upperComments` | `"parentheses"`, `true` | |
| `header` | `["%", "O{program_number} ({program_name})"]` | |
| `safeStart` | `"G17 G21 G40 G49 G80 G90 G94"` | `"G18 G21 G40 G80 G99"` |
| `toolChangeBlocks` | `["T{tool} M06 ({tool_name})"]` | `["T{tool2}{tool2} ({tool_name})"]` |
| `toolLengthOffset`, `cutterCompOffset` | `"G43 H{tool}"`, `"D{tool}"` | `""`, `""` |
| `homeZBlocks` | `["G91 G28 Z0.", "G90"]` | |
| `footer` | `["G91 G28 Z0.", "G90", "M30", "%"]` | `["G28 U0.", "G28 W0.", "M30", "%"]` |
| `coolantCodes` | flood M08, mist M07, air M07, through_tool M88, off M09 | |
| `spindleCw`, `spindleStop` | `"M03"`, `"M05"` | |
| `tcpOnOffset`, `tcpOffBlock` | `"G43.4 H{tool}"`, `"G49"` | |
| `lathe`, `feedPerRev` | `false`, `false` | `true`, `true` |

Templates take `{program_number}`, `{program_name}`, `{tool}`, `{tool2}` (two digits) and
`{tool_name}`, as the JSON configurations did. Every handler is a method a post can override and
call with `super`. Until step 6 moves arc splitting into lowering, the base splits arcs at
`maxArcSweep` itself, as the formatter does.

The three built-in posts are subclasses with nothing to change but their manifests, in
`data/posts/` and embedded in the binary (`post::builtinPost(id)`, `post::builtinPostIds()`), so
headless runs and tests need no data folder:

| Post | Machines | Declarations |
|---|---|---|
| `fanuc-generic-mill.js` | 3-axis and 5-axis mills | all cycles, tool centre point control |
| `fanuc-generic-mill-4axis.js` | 3-axis and 4-axis (A) mills | all cycles, machine positions with inverse-time feeds |
| `fanuc-generic-lathe.js` | 2-axis lathes | no cycles (expanded) |
| `haas-classic-mill.js` (step 9) | 3-, 4- (A) and 5-axis trunnion mills | all cycles, machine positions with inverse-time feeds |

`post::lowerOptionsFor(manifest)` turns the declarations into lowering options: canned cycles
when the post takes any (per type from step 6), and the multi-axis mode.

### Choosing a post, options and storage

- **Machine default.** A machine definition gains `post`, its default post id (a built-in post
  or one the project carries). This is machine format version 6; files without it fall back to
  the built-in post for their kinematics (`defaultPostFor`).
- **Setup choice.** A setup gains `post` (`PostChoice`): `{id, source}`, where `source` is
  `"builtin"` or `"project"`; unset for the machine's default. It also gains `post_options`:
  `{"program": {id: value}, "operations": {operation id: {id: value}}}`. Values missing take the
  manifest default, which the editors show (ADR-0004 rule 6). Both are optional fields of the
  project format (version 1, as other optional fields were added).
- **Resolution** (`document::postFor`): the setup's choice, else the machine's `post` (the
  project's posts first, then the built-in ones), else `defaultPostFor`. A chosen post that is
  gone is an error naming it. `postSetup(doc, setup, machine, cancel, progress)` posts with the
  setup's post and option values; the per-operation values reach the post as
  `program.operations[i].options` (the NC program's operations carry the operation `id`).
- **User posts travel with the project.** `Project::posts` holds copies (`{file, source}`),
  as `Project::machines` holds custom machines, so the project opens and posts the same
  anywhere. Choosing a library post or a post file copies it in (a copy with the same id is
  replaced); *Update from library* replaces a copy when the library's file differs.
- **Library folder.** `posts` in the application data folder (`QStandardPaths::AppDataLocation`:
  `%APPDATA%\OxCAM\OxCAM\posts` on Windows), or the `postLibraryFolder` setting. Every `.js`
  file there that loads is listed; a library post whose id a project copy has is listed once,
  as the project's.
- **GUI.** The setup page: *Post* (the machine's, the built-in posts, the project's, the
  library's, *Add a post file...*), *Update from library*, and the post's program options as
  editors (bool: check box, number: spin box within its range, choice: combo box, text: line).
  The operation page: the post's per-operation options under "*post name* options".
- **Post Process dialog** (added after step 9). *Setup > Post Process* first shows the post
  (the same choices, only posts for the machine's kinematics) and its program options, then
  posts. Each option starts from the setup's value, else the user's default for that post
  (*Save as my defaults*: settings key `postDefaults/<post id>`), else the manifest's, and says
  which. Posting stores the choice and every program option value in the setup (one undo
  step), so the CLI and Python post the same without the user's settings. Choosing another post
  drops the old post's values (it would refuse them as unknown options).
- **Machine library.** *Default post* sets `Machine::post`: the built-in post for the
  kinematics, a built-in post, or a library post. A library post is copied into the project
  when a setup there is posted with it. *Configure...* beside it sets `Machine::postOptions`
  (machine file `post_options`): the machine's values of that post's program options, between
  the setup's and the user's in the dialog, and merged under the setup's when posting
  (`document::postOptionsFor`, so the CLI and Python take them too).
- **Setup page.** *Post* and *Configure...* (the dialog without posting); the program options
  are in the dialog only.
- **Work offsets.** A manifest declares `workOffsets`: ranges `{from, to, code, start, name}`, the
  code with `{}` replaced by `start + (n - from)`; without it G54-G59 and G54.1 P1-P48. The
  post gets them as `program.post.work_offsets` (the Fanuc base writes them), the setup and
  operation pages choose the range by name (*Standard*, *Extended*; unnamed ranges are named so)
  and then the number in its code ("G154 P" 2), as far as the post declares and the machine has (`Machine::workOffsets`), and `postSetup`
  refuses others.
- **Mouse wheel.** A spin box, combo box or slider changes with the wheel only while it has
  the keyboard focus (`WheelGuard`, on the application); otherwise the page scrolls.

### Integration

| Where | Change |
|---|---|
| `libs/post` | `Engine.hpp` (`PostScript`, `PostManifest`, `PostOption`, `PostResult`), lowering by declarations, NC program v2 JSON. Removes `PostConfig`, `postConfigFromJson`, `loadPostConfig`, the `Formatter` and the embedded JSON posts. |
| `libs/document` | `lowerSetup` fills the program's tools and operations (`NcTool`, `NcOperation`: plain structs in `libs/post`, so `post` gains no dependency on `tooling` or `document`), project, setup, machine and time. `postSetup` takes a loaded `PostScript` and option values, or the setup's own (`postFor`); `defaultPostFor` returns the built-in post for the machine's kinematics. |
| GUI | Post choice and options on the setup page, per-operation options on the operation page; posting uses the setup's post and option values, shows the post's warnings and errors, and saves with the post's extension. |
| CLI | `--post <id or file.js>` (default: the setup's post, `document::postFor`; its option values, which `--option` overrides), `--option name=value` (repeatable; `true`, `false`, numbers, else text), `--ir` writes NC program v2; `--verify` back-plots with the dialect the post declares (`fanuc-mill`, `fanuc-lathe`; a post declaring none cannot be verified) within `--tolerance` mm (default 0.0015, three decimals). |
| Python | `Document.post(setup, post=None, machine=None, options=None)` runs the setup's post with its option values (`options` overrides program values), a built-in post by id or a post file (`*.js`); `oxcam.builtin_posts()` lists the ids; `Document.nc_program()` returns v2. Removes `oxcam.post.Post`, `format_program` and `examples/python/custom_post.py`. |
| Packaging | `quickjs-ng` in `vcpkg.json`, and in `THIRD_PARTY_NOTICES.md` with both of its licences (MIT, Unicode-3.0); `data/posts/*.js` install with `data/`. |
| Docs | `docs/posts/` rewritten for JavaScript posts, with an example post in `examples/posts/`. |

### Order of work

1. QuickJS-ng dependency, `PostScript` with the sandbox, limits, cancellation and
   errors; engine tests.
2. `oxcam:post` output library; tests per primitive.
3. NC program v2 (`operation_begin`/`end`, `feed_class`, tools, operations, machine,
   `peek`).
4. `oxcam:fanuc` and the three built-in posts; byte-for-byte golden tests against the C++
   formatter.
5. Switch the GUI, CLI and Python to the engine. Remove the JSON posts, `PostConfig`, the
   `Formatter` and the Python post class.
6. Declarations in lowering (arc planes, helical, tolerance, per-type cycles).
7. Post choice, options and project copies; machine format version 6.
8. Rewrite `docs/posts/`, add the example post.
9. (Added after step 8.) A fourth built-in post, `haas-classic-mill.js`: Haas-style controls
   without tool centre point control, 3-, 4- and 5-axis (trunnion) machine positions, on
   `FanucMill` ([docs/posts/haas-classic-mill.md](../posts/haas-classic-mill.md)). The
   back-plot learns what its output uses: a dwell `P` with a decimal point is in seconds
   (without one, milliseconds as before), `G53` blocks are machine positions like `G28` (not
   modelled), and `G154` and `G187` move nothing. `compareBackplot` accepts a rapid written
   as a feed move (Haas-style 5-axis motion has no rapids), not the reverse.

## Sources

- ECMA-262, *ECMAScript Language Specification* (modules, classes).
- QuickJS-ng documentation and `quickjs.h` (runtime, context, intrinsics, module loader,
  interrupt handler, memory limit).
- ISO 6983-1, *Numerical control of machines — Program format and definitions of address words*
  (block, word and address terms; paraphrased).
- P. Smid, *CNC Programming Handbook*, Industrial Press (modal groups, block structure).
- DL-0001 (sagitta bound for arc linearization), DL-0011, DL-0015, DL-0022, DL-0023.
- Step 9 (syntax facts only, paraphrased): Haas *Mill Operator's Manual* 96-8000 Rev Y,
  December 2009 (classic control); Productivity Inc / Haas Factory Outlet, *Haas Mill
  Programming*, revised 05/25 (Next Generation Control).

## Deviations from sources

None. The engine embeds QuickJS-ng as its documentation describes, leaving out its optional
system library.

## Correctness and tests

- **Sandbox:**
  - importing anything but an `oxcam:` module fails;
  - `std`, `os`, `Date` and `Math.random` are unavailable;
  - a post that allocates without bound hits the memory limit with an error;
  - an endless loop stops when the cancel token is set.
- **Errors:**
  - a thrown exception reports the post's file, line and message;
  - `error()` reports the post's message;
  - an unknown op reports the op;
  - a newer `engine` version, unknown manifest keys and malformed options are refused at load.
- **Output library:**
  - number formats (rounding, points, `-0`, scale);
  - modal words and code groups, with reset;
  - numbering with wrap;
  - comment filtering;
  - buffers;
  - the ASCII check.
- **Equivalence (the gate for removing the C++ formatter):**
  - every setup of every fixture project (12 projects: chuck, dome, dome5, dome5 machine,
    fixture, plate32, plate32 machine, rotary, sample, shaft, vise, vise origin) is posted with
    the C++ formatter and with the JavaScript post for its machine; the texts are identical, byte
    for byte. All three built-in posts are exercised: mill (3-axis, 3+2, 5-axis), 4-axis and
    lathe (`tests/unit/document/FixturePostTest.cpp`, one test case per project);
  - the golden toolpaths (facing, pocket, contour with and without compensation, drilling), alone
    and together, with canned cycles and expanded, through the mill post;
  - a post that changes every `FanucMill` setting (inch, no line numbers, no trailing points, R
    arcs split at 90 degrees, dwell in seconds, semicolon comments in mixed case, other
    templates and codes) against the C++ formatter with the same configuration
    (`tests/unit/post/BuiltinPostTest.cpp`).
- **Back-plot:** `oxcam-cli project post --verify` passes on the fixtures with the JavaScript
  posts, as it does today.
- **Declarations:**
  - arcs in excluded planes and excluded helical arcs become lines within the tolerance;
  - excluded cycle types expand;
  - a post for another machine kind is refused.
- **Determinism:** posting twice gives identical text.
- **Python:** `Document.post` with a JavaScript post matches the CLI; the example post runs.
- **Performance:** `test_post "[benchmark]"` posts a program of 400 000 instructions (the golden
  pocket repeated: half arcs, half lines and rapids) with both. The goal was under 2 s per
  million instructions. **Measured (2026-10-25, ci-msvc-headless, RelWithDebInfo): 5.1 s per
  million with the JavaScript mill post, against 0.9 s for the C++ formatter.** The engine
  alone (handlers that write nothing) takes 0.8 s per million; the rest is the base post's
  JavaScript in the QuickJS interpreter. Getting there took building instructions straight from
  `NcInstr` (12.9 s before), cached property names, formatting numbers without allocating, and
  plain loops in the base post. A typical 3-axis program of 100 000 moves posts in about half a
  second; a 5-axis finishing program of a few million moves takes 10 to 20 seconds.

## Known limitations

- **Back-plotting** covers Fanuc-style output only. Posts in other dialects declare
  `verify: null` and are not checked against the toolpaths.
- **No debugger.** A post can only debug through `warning()` output and the error positions.
- **Single-file posts.** A post can import only OxCAM's modules, so shared code between user
  posts means copying it or subclassing a built-in base.
- **Subprograms** are possible with buffers, but OxCAM does not yet mark repeated operations
  (patterns), so a post cannot call one subprogram for several of them.
- **Existing `.json` post files stop working.** No converter is provided; OxCAM has no release.
- **Posting speed** is about five times the C++ formatter's (see Performance): QuickJS-ng is an
  interpreter. Long multi-axis programs take seconds to post.
- **Feed per revolution** divides the program's feed per minute back by 60, so it can differ from
  the C++ formatter's in the last bit, and inch surface speed is computed from m/min; the
  fixtures show no difference in the output.
