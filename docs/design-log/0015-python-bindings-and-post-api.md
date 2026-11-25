# DL-0015: Python bindings and the post API

- **Status:** implemented (phase 6); the Python post base class, `format_program` and the
  custom post example are superseded by DL-0039 (JavaScript posts, NC program version 2).
- **Module:** bindings/python (`oxcam` package, nanobind), libs/post (NC program JSON),
  libs/document (`lowerSetup`), apps/opencam-cli (`project post --ir`)
- **Patent gate:** not applicable (no machining algorithm).
- **Author(s):** OxCAM contributors

User documentation of the NC program, post configurations and Python posts:
[docs/posts](../posts/README.md).

## Problem

Users need two things:

- To script OxCAM: batch toolpaths, simulation reports and posting.
- To write post processors for their own controls without changing C++. Post processors are the
  most control-specific part of CAM. A small, documented exchange format lets anyone write one
  in any language.

## Approach

### The post API: NC program JSON

The NC intermediate representation (DL-0011) is the hand-over point. Everything up to it is
done by OxCAM:

- toolpaths;
- solving against the machine;
- tool changes, spindle and coolant sequencing;
- canned cycles or their expansion;
- cutter-compensation placement.

Only the text formatting is left to the post.

`post::toJson(NcProgram)` writes a versioned document:

```json
{"format": "oxcam-ncprogram", "version": 1,
 "units": {"length": "mm", "feed": "mm/min", "spindle": "rev/min", "time": "s"},
 "name": "...", "number": 1000,
 "instructions": [{"op": "tool_change", "tool": 1, "tool_name": "..."},
                  {"op": "linear", "x": 10.0, "y": null, "z": -2.0, "feed": 600.0}, ...],
 "warnings": []}
```

- **Instructions:** `comment`, `tool_change`, `work_offset`, `spindle`, `coolant`,
  `tool_length`, `rapid`, `linear`, `arc` (absolute centre `cx`/`cy`/`cz`, `clockwise`), `dwell`,
  `cycle` (type, top, bottom, retract, peck, dwell, feed, pitch), `cycle_cancel`,
  `compensation_on` (side, tool) and `compensation_off`.
- **Units and nulls:** coordinates a block leaves unchanged are null. Units follow the file
  conventions (ADR 0002).
- **Reading:** `ncProgramFromJson` reads the document back and rejects unknown formats,
  versions and instructions.

The document is produced by:

- the CLI: `oxcam-cli project post <file.ocam> --ir program.json`;
- Python: `Document.nc_program()`.

### Python package `oxcam`

The binding surface is small and data-oriented, so it stays stable as the C++ evolves:

| Call | What it does |
|---|---|
| `Document.open(path)` | Opens a project. `name`, `setups` and `tools` come back as dicts. |
| `toolpaths(setup)` | Generates the toolpaths. Each comes back in the toolpath JSON format with its statistics. |
| `simulate(setup, resolution, tolerance)` | Simulates a setup. The report has the same shape as `oxcam-cli project simulate --json`. |
| `post(setup, post, machine)` | Posts a setup to G-code. |
| `nc_program(setup, canned_cycles, machine)` | Returns the NC program dict. |
| `format_program(program, post)` | Formats a program with the built-in data-driven formatter. |
| `load_tool_library(path)` | Reads a tool library. |
| `backplot(gcode)` | Back-plots G-code. |

C++ `ocam::Error` becomes `oxcam.Error`. Long calls release the GIL.

`oxcam.post.Post` is a pure-Python base class for post processors:

- It dispatches each instruction to a method named after its `op`.
- It keeps modal state (position, feed, motion code) and suppresses repeated words.
- It merges cutter-compensation starts into the next linear move.
- It provides number formatting, line numbering and comments.

The defaults produce generic Fanuc-like code. A dialect overrides only what differs:
`examples/python/custom_post.py` uses semicolon comments, a different header and expanded
cycles.

### Build

- Building the bindings is optional: `-DOCAM_BUILD_PYTHON=ON`.
- nanobind comes from a CMake package if one is found, otherwise from the Python environment
  (`pip install nanobind`).
- On Windows the package adds its own folder, `../../bin` and `OXCAM_DLL_DIR` to the DLL
  search path, so the extension finds the OCCT and oneTBB DLLs.

## Correctness and tests

- **`post.api` (C++):** five programs (facing, pocket, contour, drilling, and a contour with
  compensation in the control) round-trip through JSON. Formatting the program read back gives
  the same text, with two dialects. Feeds come out in mm/min. Foreign or unknown documents are
  rejected.
- **`python.bindings`:** opens the sample project and checks:
  - toolpaths generate;
  - the simulation finds no gouges and matches the part volume;
  - `format_program(nc_program())` reproduces `post()` byte for byte;
  - the generic Python post back-plots to the same feed length as the built-in post;
  - tool libraries load;
  - missing files raise `oxcam.Error`.
- **`python.example_custom_post`:** the example dialect back-plots to the same motion as the
  built-in post.

## Known limitations

- **Read-only:** documents are opened read-only from Python. Editing projects (adding operations
  and so on) is not exposed yet.
- **Python version:** the extension is built for one Python version. A stable-ABI (abi3) wheel
  needs Python 3.12+ and is not set up yet.
- **External posts:** the CLI cannot run Python posts directly. Use `--ir` with a script, or the
  Python API.
