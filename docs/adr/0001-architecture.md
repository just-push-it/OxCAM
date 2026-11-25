# ADR-0001: Core stack, layering and toolpath pipeline

- **Status:** accepted
- **Date:** 2026-10-18

## Context

OxCAM must:

- import STEP;
- manage tool libraries;
- generate toolpaths for milling, turning, mill-turn and Swiss machines;
- simulate stock;
- run rest machining from the simulated stock.

It is clean-slate and GPL-3.0-or-later. Kernels must be testable without a GUI and reusable from
a CLI and, later, from Python.

## Decision

**Language and frameworks.** C++20, using:

- OpenCASCADE for STEP import and B-rep;
- Qt 6 Widgets for the desktop UI;
- CMake presets with a vcpkg manifest;
- CI on Windows, Linux and macOS.

**Layering.** Libraries under `libs/` form an acyclic graph, enforced at configure time by
`cmake/ModuleGraph.cmake`:

```
foundation -> geom2d, geom3d -> tooling -> toolpath -> strategies, machine -> sim, post
           -> document -> apps (cli, gui), bindings
```

- OpenCASCADE is linked only by `cad`. Toolpath strategies take neutral inputs: 2D regions,
  depth ranges, hole lists and meshes.
- Qt is linked only by `apps/opencam-gui`.

**Pipeline.** Each operation runs through these stages:

1. A strategy produces a neutral cutter-location toolpath (the CL intermediate representation,
   in the setup's WCS, with a tool-axis vector on each move).
2. `machine::solve()` applies inverse kinematics and limits, producing the NC intermediate
   representation (per-channel blocks with sync points).
3. A data-driven dialect formatter turns the NC IR into G-code.

Turning uses the same IR, with X as the radial coordinate.

**Simulation.**

- Milling uses tri-dexel stock.
- Turning uses an axisymmetric R–Z profile, which converts one-way to tri-dexel at the first
  non-axisymmetric operation.
- Tools are reduced to a canonical revolved profile.
- Each operation checkpoints its in-process stock.

**Units.** Millimetres, radians and seconds everywhere inside the libraries.

## Consequences

- Kernels are unit-tested without OpenCASCADE or Qt, which keeps tests fast.
- New tool types need only a profile generator; the simulator is unaffected.
- Mill-turn and Swiss support comes from the channel and sync model in `machine`, rather than
  from separate code paths.
- vcpkg builds OpenCASCADE and Qt from source, so CI depends on a binary cache.
