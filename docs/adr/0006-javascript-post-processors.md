# ADR-0006: Post processors are JavaScript programs run by an embedded engine

- **Status:** accepted
- **Date:** 2026-10-24

## Context

OxCAM has two kinds of post processor today (DL-0011, DL-0015, [docs/posts](../posts/README.md)):

- **JSON post configurations** (`oxcam.post`): settings for a built-in C++ formatter
  (`post::format`). The GUI and CLI can only use these, and the GUI only the built-in ones.
- **Python posts**: subclasses of `oxcam.post.Post`, a second, simpler formatter written in
  Python. They run only from user scripts, through the optional Python bindings.

The two formatters duplicate each other and disagree (arc planes, inch output, arc splitting,
coolant codes). A configuration cannot express what real machines need: subprograms, macros,
tool preload, retract and park strategies, probing, controller-specific motion. Python cannot
ship inside an installer without bundling CPython, and a Python post shared by another user runs
with full access to the computer.

The maintainer's decisions, from 2026-10-24:

- There is **one** kind of post. The JSON configuration type is removed entirely.
- Posts are written in **JavaScript**.
- Whatever runs posts must ship with OxCAM's installer.
- Posts may change anything in the output, feeds and speeds included (ADR-0004 amendment).

## Decision

### 1. A post is one JavaScript module

A post is a single `.js` file (an ES module). It contains:

- a **manifest** object: identity, version, the machine kinds it supports, and the
  declarations that change what OxCAM sends it (section 3);
- the **option** definitions the user sets when posting (section 6);
- **handler** functions, one per NC program instruction, plus program and operation begin/end
  handlers.

Simple machine customization stays short: OxCAM ships a generic Fanuc-style base module that a
post imports and overrides in a few lines (a different header, tool change, coolant code). There
is no separate configuration format.

### 2. The engine: QuickJS-ng, built in

`libs/post` embeds [QuickJS-ng](https://github.com/quickjs-ng/quickjs) (MIT, about 1 MB, vcpkg
port `quickjs-ng`), linked into OxCAM. No interpreter is installed or found at run time.

Posts run sandboxed:

- no file system, network, process or environment access; the only modules a post can import
  are the ones OxCAM provides (the output library, the generic base posts);
- a memory limit, and an interrupt handler tied to `CancelToken` so a runaway post can be
  stopped and posting can be cancelled;
- output goes only through the engine's output functions; the engine writes the file.

The engine provides the output primitives in C++, called from JavaScript: number formats,
modal words (written only when their value changes, with a way to force them), the block writer
(line numbers with wrap, optional block delete), the comment writer (character filtering,
case), output buffers for subprograms, and `error` / `warning`. Their names and shapes are
designed from OxCAM's own NC program; they are not modelled on any vendor's post API.

### 3. The post declares what it can take; OxCAM reshapes the program before the post runs

Declarations in the manifest change lowering (`post::lower`), not just formatting. Version 1:

| Declaration | Effect |
|---|---|
| Machine kinds (mill 3-axis, 3+2, 4-axis, 5-axis, lathe) | Which machines the post can be chosen for |
| Arcs: maximum sweep, allowed planes, helical allowed | Longer arcs are split; arcs in other planes, or helical ones, become lines within the linearization tolerance |
| Linearization tolerance | How finely those arcs become lines |
| Canned cycles supported, per cycle type | Other cycles are expanded into moves |
| Multi-axis output: tool centre point control, or machine positions with inverse-time feeds | How multi-axis toolpaths are solved (replaces `PostConfig::tcp`) |
| Minimum engine version | Older OxCAM versions refuse the post |

A post can also refuse a program (`error`) for anything it cannot output safely.

### 4. What a post receives (NC program, in memory)

The NC program (DL-0015) stays the hand-over, extended:

- operation begin and end, with the operation's name, strategy, tool, coolant, work offset,
  orientation (3-axis, 3+2, simultaneous) and bounds;
- the setup's tool table up front, and the whole program available to every handler, so a post
  can look ahead (next tool for preloading, next operation's work offset);
- each move's feed class (cut, plunge, ramp, lead-in, lead-out, link, rapid), which the toolpath
  already records and lowering currently drops;
- the machine definition, read-only: axes and travel, rotary axes, spindle and feed limits, tool
  count;
- the option values for this run.

Values are the user's, unrounded, in OxCAM's units (mm, mm/min, rev/min, s, degrees for rotary
positions). The post converts and rounds.

### 5. The machine definition wins

Kinematics, travel, limits and rotary axes belong to the machine definition (DL-0023, DL-0037).
A post reads them and may refuse a job, but cannot redefine them. A machine definition names its
default post; a setup may choose another post that supports the machine's kind.

### 6. Options

The manifest declares options: name, label, type (bool, number, choice, text), default, and
whether they apply to the program or per operation. The GUI builds the post dialog from them and
saves the values in the project with the setup. The default is the manifest's, shown in the
dialog before anything uses it (ADR-0004, rule 6).

### 7. Where posts live and how they ship

- Built-in posts and the base modules live in `data/posts/` as `.js` files and install with
  `data/` (`cmake/Packaging.cmake`). They are written from controller programming manuals only,
  and each lists its sources in a header comment.
- User posts live in a per-user folder (for example `%APPDATA%\OxCAM\posts` on Windows) and are
  listed with the built-in ones. A user post with the same id as a built-in one is shown as
  separate, never silently replacing it.
- The post id and version are available to the post, so it can write them into the program for
  traceability.

### 8. Checking the output

`post::backplot` keeps checking Fanuc-style output against the toolpaths. A post declares which
back-plot dialect, if any, its output can be read with; the CLI `--verify` and the GUI say when
a program cannot be verified this way.

### 9. Removed

When the built-in posts run on the engine:

- JSON post configurations: `PostConfig`, `postConfigFromJson`, `loadPostConfig`,
  `schemas/post.v1.schema.json`, `data/posts/*.json`, and the C++ `Formatter`;
- the Python post base class `oxcam.post.Post`, `oxcam.format_program`, and
  `examples/python/custom_post.py` (replaced by a JavaScript example);
- `--post file.json` on the CLI becomes `--post <id or path to .js>`.

The Python bindings stay for scripting OxCAM (toolpaths, simulation, posting with a JavaScript
post). `Document.nc_program()` and `oxcam-cli project post --ir` stay as a way to inspect what a
post receives; they are not a second post route.

## Consequences

- One formatter: the generic posts become JavaScript, and the golden tests require them to
  reproduce the current C++ formatter's output byte for byte on every fixture project before
  the C++ formatter is deleted.
- A new third-party dependency (QuickJS-ng), listed in `THIRD_PARTY_NOTICES.md`.
- Posting speed depends on the interpreter. Number formatting and block writing in C++ keep the
  per-move cost low; long multi-axis programs are measured before the switch.
- JavaScript is the language of commercial CAM post processors. Porting or adapting a vendor's
  post is forbidden (IP policy) however the language matches; reviews of new posts check their
  sources.
- Existing `.json` post files stop working. OxCAM has no released version, so no conversion is
  provided.
- The user documentation in `docs/posts/` is rewritten for JavaScript posts.

## Implementation order

The detailed design is
[DL-0039](../design-log/0039-javascript-post-engine.md).

1. Design-log entry for the engine and the extended NC program (DL-0039).
2. QuickJS-ng in `libs/post`: module loading, sandbox, limits and cancellation, output
   primitives.
3. The generic base modules and the three built-in posts in JavaScript; golden tests against
   the current output.
4. GUI, CLI and Python post through the engine; remove what section 9 lists.
5. Post choice: machine default, setup override, user post folder, options dialog.
6. Lowering honours the declarations of section 3.
7. Packaging and the rewritten `docs/posts/`.
