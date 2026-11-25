# OxCAM

OxCAM is a clean-room, open-source computer-aided manufacturing (CAM) application.

Goals:

- Import STEP models (via OpenCASCADE).
- User-defined, versioned tool libraries.
- 2.5D and 3D milling, turning, mill-turn and Swiss-type machines.
- Stock and tool simulation with in-process stock, remaining-stock visualization and
  rest machining driven by the simulated stock.
- Machine-independent toolpaths lowered to G-code through post processors written in
  JavaScript and run by an engine built into OxCAM, with generic Fanuc-style posts and a
  classic Haas-style mill post built in. Writing a post for your own machine:
  [docs/posts](docs/posts/README.md).

![Simulation playback partway through the pocket: faced top at size (green), side stock still to be removed (blue)](docs/images/phase5-simulation.png)

## Status

Version 0.1.0, in active development. 2.5D and 3D milling, turning and much of multi-axis
milling work end-to-end: STEP import, setups, toolpaths, stock simulation and posting.

> **The G-code has not been validated on a physical controller yet.** Dry-run every program,
> and help by recording results in
> [docs/validation/controller-dry-runs.md](docs/validation/controller-dry-runs.md).
>
> Some features are waiting on patent reviews that must be cleared before a public release
> (rest machining, [IP-0002](docs/ip/reviews/0002-rest-machining-from-in-process-stock.md),
> and others in [docs/ip/reviews](docs/ip/reviews)).

Each feature has a design log entry with the algorithm, its references and its limits; the
links below point to them.

### Implemented

**Foundation**

- STEP import (AP203/AP214/AP242), tessellation, planar sections, and recognition of holes and
  flat faces ([DL-0001](docs/design-log/0001-arc-polylines-and-sections.md)–[DL-0003](docs/design-log/0003-point-ordering.md)).
- Projects (`*.ocam`) with embedded models, setups, stock, WCS, persistent face references, undo
  and redo ([ADR-0002](docs/adr/0002-file-formats-and-undo.md)); saved toolpaths
  ([ADR-0003](docs/adr/0003-toolpath-cache.md)).
- Tool libraries (`*.ocamtools`): milling, drilling, turning and probe tools, holders (from
  dimensions or a vendor STEP) and cutting-data presets. Speeds, feeds, step down and stepover
  come only from the presets; nothing is filled in behind your back
  ([ADR-0004](docs/adr/0004-output-is-what-the-user-sees.md), [DL-0029](docs/design-log/0029-cutting-data-from-presets.md)).
- Fixtures and workholding templates (vises, dovetails, plates, chucks, collets, soft jaws)
  placed on the stock and avoided by the toolpaths
  ([DL-0034](docs/design-log/0034-fixtures-and-holders.md), [DL-0035](docs/design-log/0035-workholding-templates.md)).
- Machine definitions and a machine library, with machine models imported from STEP or STL and
  work placed on the machine ([DL-0037](docs/design-log/0037-machine-models.md)).

**Milling operations**

- 2.5D: facing, pocket (with optional rest machining), contour with leads and holding tabs,
  chamfer, 2D trace, plunge roughing.
- Holes: drill, tap, thread milling (internal and external), bore, back bore.
- 3D: Z-level roughing and rest roughing (medial-axis clearing), parallel, waterline, contour,
  flat area, pencil and steep/shallow finishing, 3D edge break.
- Arc smoothing (G2/G3) on operations that don't arc by themselves
  ([DL-0032](docs/design-log/0032-arc-smoothing.md)).
- Probing cycles with a spindle probe: surface, width, circle and corner
  ([DL-0090](docs/design-log/0090-probing-cycles.md), first pass).

**Multi-axis milling**

- 3+2 indexed setups and a tool orientation per operation
  ([DL-0019](docs/design-log/0019-indexed-3plus2-setups.md), [DL-0036](docs/design-log/0036-operation-tool-orientation.md)).
- 4th-axis wrapping: wrapped pockets and contours ([DL-0024](docs/design-log/0024-wrapping-and-5axis-finishing.md),
  [DL-0084](docs/design-log/0084-wrapped-contour.md)).
- Simultaneous 5-axis ball finishing and swarf milling ([DL-0085](docs/design-log/0085-swarf-milling.md)).
- Kinematics for 4-axis and table-table, head-table and head-head 5-axis machines, posted with
  tool centre point control (G43.4) or inverse-time feeds (G93)
  ([DL-0023](docs/design-log/0023-multi-axis-kinematics-and-posting.md)).

**Turning**

- Lathe setups on bar stock; facing, OD/ID roughing and finishing with nose-radius compensation,
  grooving (OD, ID, face), threading (G32, multi-start, tapered), parting off
  ([DL-0020](docs/design-log/0020-turning-strategies.md), [DL-0047](docs/design-log/0047-turning-threads.md)).
- Holes on the spindle axis, second-side setups, tailstock and steady rest.
- Live tools: holes, milling and wrapped pockets by C, Y and polar or cylindrical interpolation
  ([DL-0063](docs/design-log/0063-live-tools-on-a-lathe.md), [DL-0066](docs/design-log/0066-milling-with-a-live-tool.md)).
- Bar pull and a transfer to the sub-spindle, posted as one program
  ([DL-0091](docs/design-log/0091-bar-pull.md), [ADR-0007](docs/adr/0007-sub-spindle-transfer.md)).

**Simulation and verification**

- Tri-dexel stock simulation for every rotating tool, tilted and turning tools included, with
  in-process stock per operation ([DL-0012](docs/design-log/0012-dexel-stock-simulation.md),
  [DL-0026](docs/design-log/0026-tilted-tool-sweeps.md)); axisymmetric stock for turning
  ([DL-0021](docs/design-log/0021-axisymmetric-stock.md)).
- Part comparison (gouge, at size, excess) and collision checks for holders, shanks, rapids,
  fixtures and the machine itself ([DL-0025](docs/design-log/0025-machine-collision-checks.md),
  [DL-0067](docs/design-log/0067-lathe-machine-simulation.md)).
- Back-plotter: posted programs are verified against their toolpaths.

**Post processing**

- JavaScript post engine ([ADR-0006](docs/adr/0006-javascript-post-processors.md),
  [DL-0039](docs/design-log/0039-javascript-post-engine.md)). Built-in posts in
  [data/posts](data/posts): generic Fanuc-style mill, 4-axis mill and lathe, and Haas-style mill
  and lathe. See [docs/posts](docs/posts/README.md) to write your own.

**Applications**

- `oxcam-gui`: Qt 6 desktop application with a 3D view, project tree, tool, workholding and
  machine libraries, simulation playback, preferences (units, shortcuts, toolbar, colours) and
  translations ([docs/translations.md](docs/translations.md)).
- `oxcam-cli`: headless `toollib check`, `project info | toolpaths | post | simulate` and
  `backplot`.
- `oxcam` Python package (nanobind): open projects, generate, simulate and post
  ([DL-0015](docs/design-log/0015-python-bindings-and-post-api.md)).

### Planned

- Mill-turn with multiple channels (phase 10): synchronised spindles and several turrets.
  Sub-spindle transfer and live tooling exist; multi-channel output does not yet.
- Swiss-type machines.
- Controller validation of the built-in posts on real machines.
- Clearing the pending patent reviews ([docs/ip](docs/ip/IP-POLICY.md)) before a public release.

## Building

Requirements: CMake 3.25+, Ninja, a C++20 compiler (MSVC 19.3x+, GCC 12+, Clang 16+), and
[vcpkg](https://github.com/microsoft/vcpkg) with `VCPKG_ROOT` set. vcpkg installs the
dependencies listed in [vcpkg.json](vcpkg.json) (OpenCASCADE, Clipper2, Eigen, oneTBB,
QuickJS-ng, Qt 6 for the GUI, and others).

```bash
cmake --preset linux-gcc-headless
cmake --build --preset linux-gcc-headless
ctest --preset linux-gcc-headless
```

On Windows, run from an "x64 Native Tools" developer prompt (or after `vcvars64.bat`) and use
`msvc-headless`. `cmake --list-presets` shows every preset.

| Presets | What they build |
|---|---|
| `*-headless` | Libraries, CLI and tests (Debug) |
| `*-gui` | Also the Qt 6 desktop application (Debug) |
| `*-release` | GUI build, optimized (RelWithDebInfo); use these for real work, about 100× faster simulation |
| `ci-*` | Optimized, assertions kept, warnings as errors |
| `linux-clang-asan`, `linux-clang-fuzz` | Sanitizers and libFuzzer targets |

The first GUI build is long: vcpkg compiles OpenCASCADE and Qt from source, and later builds
reuse its binary cache. Tests run in parallel with `ctest --preset <preset> -j N`.

**Package:** `cd build/msvc-release && cpack -C RelWithDebInfo` writes a relocatable ZIP
(Windows) or TGZ.

**Python bindings** (optional, needs nanobind):

```bash
python -m pip install nanobind
cmake --preset msvc-release -DOCAM_BUILD_PYTHON=ON -DPython_EXECUTABLE=/path/to/python
```

The package is written to `build/<preset>/python/oxcam`. Add `build/<preset>/python` to
`PYTHONPATH`; on Windows also set `OXCAM_DLL_DIR=build/<preset>/bin`.

## Running

```bash
build/msvc-gui/bin/oxcam-gui path/to/part.step     # GUI; without a file it opens the start hub
oxcam-cli project post build/<preset>/fixtures/sample_project.ocam -o part.nc --verify
oxcam-cli project simulate build/<preset>/fixtures/sample_project.ocam --resolution 0.5
```

Test fixtures (STEP files and sample projects such as `plate32_project`, `shaft_project` and
`rotary_project`) are generated at build time into `build/<preset>/fixtures/`. A sample tool
library is in [data/tool-libraries](data/tool-libraries/sample-metric.ocamtools); its cutting
values are generic starting points, so always check your tool vendor's data.

## Repository layout

| Path | Contents |
|---|---|
| `libs/` | Core libraries (foundation, geom2d, geom3d, cad, tooling, machine, toolpath, strategies, sim, post, document). Each has a public `include/ocam/<lib>/` and a private `src/`. |
| `apps/` | `oxcam-cli` (headless) and `oxcam-gui` (Qt 6). |
| `bindings/python/` | The `oxcam` Python package (nanobind). |
| `data/` | Bundled machines, post processors (JavaScript), tooling blocks and a sample tool library. |
| `examples/` | An example post processor to start your own from (`examples/posts`). |
| `schemas/` | JSON Schemas for tool library, project, workholding and machine files. |
| `tests/` | Unit, golden-file, fixture and fuzz tests. |
| `translations/` | Qt Linguist translation files. |
| `docs/adr/` | Architecture decision records. |
| `docs/design-log/` | One entry per algorithm, with citations. |
| `docs/ip/` | Clean-room IP policy and patent-review records. |
| `docs/posts/` | Writing and configuring post processors. |

Library layering is enforced at configure time by [cmake/ModuleGraph.cmake](cmake/ModuleGraph.cmake).

## Contributing

Read [CONTRIBUTING.md](CONTRIBUTING.md) first. Every commit needs a DCO `Signed-off-by` line,
and the clean-room rules are mandatory.

## License

GPL-3.0-or-later. See [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
