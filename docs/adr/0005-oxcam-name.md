# ADR-0005: OxCAM, formerly OpenCAM: names, file formats and settings

- **Status:** accepted
- **Date:** 2026-10-22

## Context

The program was called OpenCAM, a working name. Other projects use that name or one close to it
(OpenCAMLib, and older projects called OpenCAM; see [IP-POLICY](../ip/IP-POLICY.md#trademarks)).
The maintainer chose OxCAM, short for OxygenCAM.

The old name was in four kinds of places:

1. text the user sees: window titles, the About box, the start hub, dialogs, tooltips, the
   translations, the package and installer names;
2. the names of the executables, the Python package and the shared libraries;
3. the `"format"` names inside the persisted JSON formats (`opencam.project`, `opencam.toollib`,
   `opencam.machine`, `opencam.post`, `opencam.workholding`, `opencam.toolpath-cache`,
   `opencam-ncprogram`);
4. where the GUI keeps the user's settings and data: the `QSettings` organization and application
   name `OpenCAM` (the registry key `HKCU\Software\OpenCAM\OpenCAM` on Windows) and the data folder
   derived from it (`%APPDATA%\OpenCAM\OpenCAM`: tool libraries, machines, workholding).

Projects, tool libraries and settings that OpenCAM wrote must keep working.

## Decision

### Names

Everything the user sees says OxCAM. The executables are `oxcam-gui` and `oxcam-cli`, the Python
package is `oxcam`, the shared libraries are `oxcam_<module>`, the CMake project is `OxCAM` (it
sets `ocam::kProductName`), the package is `OxCAM-<version>-<system>`. SPDX headers say "OxCAM
contributors".

Unchanged, because the user never sees them and they fit OxCAM: the C++ namespace `ocam`, the
`ocam_`/`OCAM_` prefixes, and the file extensions `.ocam` and `.ocamtools`. The source folders
(`apps/opencam-gui`, `apps/opencam-cli`, `cmake/OpenCAMHelpers.cmake`, `translations/opencam_*.ts`)
keep their names for now, so that work in progress on other branches merges without moving files;
renaming them is a separate, purely mechanical change.

### File formats: read both names, write the new one

Every format gets an `oxcam` name (`oxcam.project`, `oxcam.toollib`, `oxcam.machine`,
`oxcam.post`, `oxcam.workholding`, `oxcam.toolpath-cache`, `oxcam-ncprogram`). Writers write
it; the schemas name it.
`FormatSpec` has a list of `formerNames`: a document carrying one of them reads exactly like one
carrying the current name, and `upgrade()` renames it before the schema check and the version
migrations. The format *version* does not change: the content is the same.

The toolpath cache and the NC program JSON, which do not go through `FormatSpec`, accept both
names the same way. The toolpath cache key keeps its old seed text, so the toolpaths cached in
projects saved by OpenCAM still match and are not generated again.

Projects saved by OxCAM do not open in OpenCAM builds (which only know the old names). Nothing
was released under the old name, so no one depends on that direction.

`tests/fixtures/opencam-0.1/` holds files written by OpenCAM 0.1.0 (a project, a tool library, a
machine, a post configuration, an NC program, a vise template and a project holding it);
`tests/unit/document/LegacyFormatTest.cpp` opens them. They are frozen: never regenerate them.
`tests/unit/machine/data/machine-v1-trunnion-ac.json` (version 1, old name) is another.

### Settings and data carried over once

On start, before anything reads a setting, `importOpenCamSettings()` (apps/opencam-gui) looks at
OxCAM's settings. When there are none yet and OpenCAM's has some, it copies every OpenCAM setting
(preferences, key bindings, recent projects, operation defaults, the tool library list). It copies
the files of OpenCAM's data folder that OxCAM's does not have yet, and settings that name files
in OpenCAM's data folder (the tool library list, the default library, the machine and
workholding library folders)
name the copies instead. OpenCAM's own settings and files are left where they are: copying is
safe to repeat and keeps an older build working. Once OxCAM has settings of its own, OpenCAM's are
not read again.

With `--settings-dir` (tests) only the settings are copied; the data folders of the user running
the tests are left alone. The test `gui.settings_from_opencam` covers that path.

## Consequences

- Searching the sources for "OpenCAM" finds this ADR, the "formerly OpenCAM" notes, the
  compatibility code (marked LEGACY), the frozen fixtures and the folder names above.
- A new format must still be written under its `oxcam` name only; `formerNames` is only for
  documents an earlier build wrote.
- Changes on branches made before the rename that add "OpenCAM" text, SPDX headers or
  `opencam-gui`/`opencam-cli` targets need the same mechanical rename when they merge.
