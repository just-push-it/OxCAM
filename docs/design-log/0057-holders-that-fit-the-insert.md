# DL-0057: A tool is offered the holders that fit its insert

- **Status:** implemented
- **Module:** libs/tooling (`InsertLocking`, `holderMismatches`, `clampingTakes`), apps/opencam-gui
  (`ToolLibraryDialog`, `HolderEditorCheck`)
- **Patent gate:** low (citation only). ISO 1832 and ISO 5608 letters.
- **Author(s):** OxCAM contributors

## Problem

A tool's holder list offered every turning holder, and picking one rewrote the insert to suit it. So a CN
insert could be put in a DJ holder, and the insert quietly became a D.

## Approach

**What has to match (`holderMismatches`).**

| What | Insert | ISO holder |
|---|---|---|
| Shape | Its shape letter | The designation's second letter |
| Size | Its inscribed circle, within 0.01 mm | From the shape and the size symbol |
| Clearance | Its relief angle, within 0.01° | The designation's clearance letter |
| Locking | Its locking style, when set | The designation's clamping letter |

**The hand is left out on purpose.** A tool takes its hand from its holder (`fitToHolder`), because that is
what the toolpaths use, so a tool in an R holder reads as R. Filtering on that would hide the L holder from
a neutral insert just because it sat in an R holder first.

**Holders with no designation** (plain ones, imported outlines) say nothing about their seat: only their kind
counts, as before. Grooving, threading and parting tools still see the plain holders only.

**The locking style (`InsertLocking`).** New on the insert, shown as "Locking style" for turning inserts. An
insert is held by the hole it has, so that is what is stored:

| Style | Takes clamping |
|---|---|
| Cylindrical hole | P (lever), M (top and hole) |
| No hole | C (top) |
| Countersunk hole | S (screw) |
| Not set | Any |

**Older libraries** have no locking style, so they read as Not set and it hides nothing. They are still
filtered on shape, size and clearance. Nothing is written for Not set, so their files do not change.

**What the filter hides is stated.** A note under the list says how many holders are hidden, and its tooltip
names each with its reason. To use a hidden holder, set the holder to (none) and change the insert.

**A tool whose holder no longer fits keeps it.** That can happen by a locking style, or by a file with an
insert its holder does not take. Nothing is rewritten, and generation is not refused, since the geometry
comes from the holder either way. The list marks it "(does not fit)" and a red note says why. Before this,
any edit to the tool re-fitted its insert to the holder: now only choosing a holder does, or changing that
holder's own designation, which goes on updating the tools in it.

**A new insert's relief** is now 0° for turning inserts, not 7°. A new holder is `PCLNR` (N, 0°), so a new tool
would not have been offered a new holder. The relief is only ever read here: it changes no geometry.

## Correctness and tests

`ToolTest` "which holders fit an insert": each row of both tables. `LibraryIoTest`: the style survives a save,
an older file has none, nonsense is refused. `gui.holder_editor` (`HolderEditorCheck`) runs the real dialog and
sets its widgets through their own signals: the lists for a new insert, a D insert and a countersunk one; the
hidden note; picking a holder; a tool that stops fitting, and editing something else after that. It saves a
screenshot of each step. It does not move a mouse or open a list by hand.

## Known limitations

- **The locking style is not read from a designation:** the tool editor has no insert designation (CNMG...).
- **Thickness and corner radius** are not matched: an ISO holder's designation does not give them.
