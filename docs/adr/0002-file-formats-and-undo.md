# ADR-0002: Persistent file formats and the undo model

- **Status:** accepted
- **Date:** 2026-10-18

## Context

Tool libraries and projects must stay readable for years across versions, diff cleanly in
version control, and resist damaged or hostile input. Edits made in the GUI must be undoable.

## Decision

### Versioned JSON documents

Every format is a JSON object carrying `"format"` and `"version"`:

| Format | File |
|---|---|
| `oxcam.toollib` | `*.ocamtools` |
| `oxcam.project` | `project.json` inside `*.ocam` |
| `oxcam.workholding` | `workholding.json` inside `*.ocamhold` (DL-0035) |
| `oxcam.machine` | `*.json`, with its model's part files in `<name>.meshes/` next to it: `<sha256>.step` (exact geometry), `<sha256>.stl` (parts with only a mesh) (DL-0037) |

**Loading** runs three steps in order:

1. `FormatSpec::upgrade` checks the format and applies the chain of migrations up to the current
   version. A newer file is rejected with `Unsupported`.
2. The document is validated against a JSON Schema (`schemas/*.schema.json`), which is embedded
   in the binary at build time.
3. It is converted to C++ value types.

Errors are reported as `ocam::Error` with the JSON pointer of each violation.

**Output** is deterministic: keys are sorted, indentation is two spaces, and there is a trailing
newline.

**Units:**

- Tool libraries declare `units` (`mm` or `in`). Angles are stored in degrees. Surface speed is
  m/min in mm files and ft/min in inch files.
- Projects are always stored in mm.

### `.ocam` project archive

The archive is a zip file containing:

- `project.json`;
- `models/<sha256>.step`, the embedded STEP files named by content hash (parts, fixtures and
  the bodies of the workholding templates its setups use);
- `machines/<sha256>.step` and `machines/<sha256>.stl`, the project machines' model parts: STEP
  files, and meshes of parts that have no STEP file (DL-0037);
- optionally `cache/toolpaths.json`, the generated toolpaths (ADR-0003).

The reader:

- verifies each model's and machine mesh's hash;
- caps the size of each entry;
- ignores unknown entries;
- never fails because of the cache: a cache that does not read is generated again.

Writes are atomic: the archive is written to a temporary file, then renamed over the target.

Each project embeds a **snapshot** of the tools it uses. Tools keep their library UUIDs, so a
project still opens if the library is lost or edited.

### Undo

`Project` is a plain value type. `Document::modify(label, fn)` applies `fn` to a copy of the
project and records the before and after snapshots as one step. No-op edits are not recorded,
and an edit that throws leaves the project unchanged.

Runtime caches (STEP bytes, imported B-rep models) live outside the snapshots, keyed by content
hash, so undo and redo never reload geometry.

### Persistent sub-shape references

Operation geometry is stored as a `ShapeRef`:

- the model id;
- the face or edge type;
- an index hint;
- a geometric signature (centroid, axis or normal, area or length).

On use, the reference is re-matched by signature (see DL-0005).

## Consequences

- Snapshot undo is simple and robust, but costs memory proportional to project size times
  history length. History is capped at 500 steps. Projects are small, since meshes and B-rep
  models stay outside the snapshot.
- Every format change needs a schema update, a migration from the previous version, and a frozen
  sample file from that version in the tests.
- The loaders are exercised by mutation tests on every CI run and by libFuzzer every night
  (`tests/fuzz`).
