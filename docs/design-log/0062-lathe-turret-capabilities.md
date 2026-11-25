# DL-0062: What a lathe's turret can do

- **Status:** implemented
- **Module:** libs/machine (`Machine::liveTooling`, `liveMaxSpindle`, `hasYAxis`, machine file version 7),
  data/machines, apps/opencam-gui (`MachineLibraryDialog`, `PropertyPanel`)
- **Patent gate:** none. Facts about a machine.
- **Author(s):** OxCAM contributors

## Problem

A lathe machine said nothing about driven tools or a Y axis, so a setup could not know whether either was there.

## Approach

**The machine says.**

- **Live tooling:** `live_tool` in the machine file, with the top speed. Having it means the spindle is also a C axis,
  so it is one fact, not two.
- **Y axis:** a row in the axes, as any other axis, with its travel. The editor's checkbox adds or removes that row.

**The editor.** A "Lathe turret" group, shown for lathes only: **Live tooling**, **Live tool max**, **Y axis**.

**Built-in machines.** The plain `generic-2axis-lathe` stays as it was, with neither. `generic-lathe-live` has live
tooling, and `generic-lathe-live-y` has both, with ±50 mm of Y. These numbers are placeholders like the other generic
machines': a machine of your own has its own.

**Setups follow the machine.** The setup page shows a read-only **Turret** line from the machine chosen. Nothing is
stored on the setup, so changing the machine changes what its operations may do. What is refused because of it is in
DL-0063, at the operation.

**Older files.** A machine file from before version 7 has no `live_tool`, and has none. Nothing is assumed.

## Correctness and tests

`MachineTest` "what a lathe's turret can do": the built-ins, a file round trip, an older file, and the problems reported.
`gui.machine_library_live_lathe` opens the live, Y-axis lathe in the editor. Ticking the boxes is exercised in DL-0063's
check.

## Known limitations

- **C axis limits** are not modelled: a turning spindle has none in practice, and it is not a row in the axes.
- **Y axis and turret shape:** the machine model (DL-0037) still has no lathe parts, so the machine view does not show them.
