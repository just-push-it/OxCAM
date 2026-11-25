# DL-0041: Holder kinds, tool list filters, stickout and gauge length

- **Status:** implemented
- **Module:** libs/tooling (`HolderKind`, `holds`, `ToolFamily`, `ToolFilter`, `visibleLength`,
  `gaugeLength`, `stickoutForGauge`; the `kind` of a holder in tool library files),
  libs/document (`PostSetup`), libs/sim (`ToolShape::fromTool`), libs/strategies (`Avoid`,
  `MultiAxis`), apps/opencam-gui (`ToolLibraryDialog`, `ToolProfileView`)
- **Patent gate:** low (no algorithm; data model and interface)
- **Author(s):** OxCAM contributors

## Problem

Three requests from the maintainer about the tool library:

1. The holder chooser in the tool editor listed every holder of the library: an end mill
   offered turning tool holders, and a turning insert offered collet chucks. A holder did not
   record what it holds.
2. A library with many tools had no way to narrow its list.
3. Milling tools had a stickout (`Tool::stickout`, from the holder's face to the tool tip), but
   the editor did not show the assembly's gauge length (spindle gauge line to tip), which is the
   number a tool presetter measures and the machine's tool length. Each consumer computed the
   tool's length out of the holder on its own (the stickout, or the overall length when it is
   0; the post setup added the holder's height itself).

## Approach

### Holder kinds

`Holder::kind` is `HolderKind::Milling` (collet, shrink-fit, end mill and drill chucks, tapping
holders in the milling spindle) or `HolderKind::Turning` (holders on the lathe).
`holderKindFor(type)` gives a tool type's kind (turning inserts: turning; everything else:
milling) and `holds(holder, type)` compares them. Two kinds are enough to keep the lists apart;
every rotating tool type fits every milling holder, as a collet chuck takes end mills, drills
and taps alike.

The tool editor's holder list shows the holders of the tool's kind, plus the holder the tool
already has when that one is of another kind (marked "(turning holder)" or "(milling holder)",
with a tooltip), so an older library's choice stays visible and is not changed behind the
user's back. Changing a tool's type keeps its holder only when that holder holds the new type.
The holders tab has a "Holds" column and field. Holders made from dimensions or imported from
STEP start as milling holders.

The outline of a turning holder is kept and edited like a milling holder's; the lathe
simulation does not draw holders yet.

### File format

The tool library format stays at version 1 (the field is additive and optional, like the ramp
and lead feeds before it). A holder writes `"kind": "turning"` only for turning holders, so
milling holders, and the files and toolpath keys that contain them, are byte for byte as before.
A holder without `kind` is read as a milling holder, except in one case: a holder that only
turning tools use (at least one, and no rotating tool) is read as a turning holder, since that
is what the user meant by it. Projects embed a tool library document, so they read and write
holders the same way.

### Filters

`ToolFilter` (libs/tooling, so the rule is tested without the GUI) holds a text, a family
(`ToolFamily`: milling, hole making, turning) or one tool type, and a diameter range in mm.
The text matches part of the name, vendor or part number in any case, or the tool number
("12" or "T12"). With a diameter limit, tools without a diameter (turning inserts) drop out.
Empty fields match every tool.

The tool library dialog shows a search field, a type combo (all types, the three families, then
each type) and a diameter range ("Any" at 0) above the tool list, and "n of m tools shown"
below it. The type and diameter filters are kept per user (QSettings `toolLibrary/*`); the
search text is not, so a reopened library never hides tools behind a forgotten search. Rows the
filters leave out are hidden, not removed, so a row is still the tool's index in the library; a
tool just added or duplicated stays shown even when the filters would leave it out. Delete and
"Copy selected to project" act only on the selected tools that are shown.

### Stickout and gauge length

`visibleLength(tool)` is the stickout, or the overall length when the stickout is 0;
`gaugeLength(tool, holder)` adds the height of the holder's gauge line above its tool face (the
highest point of its outline). Every consumer now uses them: the post setup (the tool table's
`gauge_length` and `machine::solve`'s tool length), `sim::ToolShape::fromTool` (the simulated
tool, shank and holder, and through it the holder, fixture and machine collision checks and the
machine simulation's tool tip, `sim::gaugeLength`), fixture avoidance and the multi-axis shank
check. The formulas are the ones each consumer used before, so toolpaths and programs do not
change and `kToolpathRevision` stays.

The tool editor shows "Stickout" and "Gauge length" together for rotating tools (turning tools
have neither). The gauge length is derived, not stored: setting it sets the stickout to the
gauge length less the holder's height (`stickoutForGauge`), and its minimum is a step above the
holder's height. Storing the stickout keeps the physical meaning when the holder changes: the
tool still comes out of the new holder as far, and the gauge length follows. The profile preview
draws the gauge line dashed.

## Sources

None: no algorithm. The terms (stickout, gauge line, gauge length) are the usual machine shop
ones.

## Deviations from sources

None.

## Correctness and tests

- **Families and kinds** (`tests/unit/tooling/ToolTest.cpp`): every tool type is in exactly one
  family, turning family = turning inserts; holder kinds round-trip by name; collet chucks hold
  end mills and taps, not inserts.
- **Gauge length**: 0 stickout = the overall length; without a holder the gauge length is the
  stickout; with a stepped holder it adds the holder's height; setting a gauge length gives back
  that gauge length; a gauge length shorter than the holder gives no positive stickout.
- **Files** (`tests/unit/tooling/LibraryIoTest.cpp`): milling holders write no `kind`, turning
  holders do and round-trip; an older file's holder used only by turning inserts reads as a
  turning holder, one shared with a drill or unused as a milling holder, and the tools keep
  their holders; an unknown kind is rejected by the schema.
- **Filters**: family, type, inclusive diameter range (turning inserts drop out), text over
  name, vendor and part number in any case, and the tool number with or without "T".
- **Existing tests** of the post's tool table, the machine collision checks, fixture avoidance
  and the simulation pass unchanged (same lengths as before).

## Known limitations

- The lathe simulation does not draw turning holders; their outline is a revolved one like a
  milling holder's.
- The gauge length is per tool (the tool, its holder and its stickout): a library cannot hold
  one tool in two assemblies; duplicate the tool for that.
