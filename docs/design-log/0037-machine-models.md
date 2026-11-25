# DL-0037: Machine models (mesh parts on the kinematic tree)

- **Status:** implemented
- **Module:** libs/cad (`Assembly.hpp`: assembly STEP files split into parts; `Model.hpp`:
  STEP data in memory, compounds, moved copies; `SubShape.hpp`: the exact geometry of a face,
  edge or vertex), libs/document (`MachineImport.hpp`: parts from STEP and STL files, grouped,
  merged, meshed on load, snapped on exactly; `StepBodies.hpp`: STEP bodies by content hash and
  the meshes derived from them), libs/machine
  (`MachineModel` in `Machine.hpp`; `Model.hpp`: poses, the linear
  axis solve, default trees, validation, mesh storage, the table chain and centre, work mounts
  and snapping; `rotaryTrack` and `mountedWcs` in `Kinematics.hpp`), libs/geom3d (`Stl.hpp`,
  `TriangleBvh::queryPairs`), libs/sim (`checkMachine`), libs/document (archive entries,
  `SetupPlacement` in `Project.hpp`; `defaultPlacement`, `workMount`, `wcsInTable` in
  `PostSetup.hpp`; `OperationSimulation::rotary` and `mount`), GUI (machine library editor and
  preview, the setup placement dialog, the simulation panel and view)
- **Patent gate:** low (citation only). Forward kinematics of serial axis chains with
  homogeneous transforms, solving the linear axes for a tool tip, and checking meshes on sampled
  poses with bounding volume hierarchies are textbook material. Collisions are reported, never
  avoided; no toolpath, speed or feed changes.
- **Author(s):** OxCAM contributors

## Problem

DL-0025 checks a few convex components (spindle head, table, cradle) that move with the rotary
axes only: the linear axes, the machine's base and column and its real shapes are missing, and
the machine is not shown. A shop wants its own machine, from the maker's CAD, in the simulation:
the parts imported as meshes, each carried by the axis that moves it, saved in the machine
library, and the machine moving with every move of the toolpath, with its collisions reported.

Inputs: the machine's kinematics type (which fixes how `machine::solve` turns the rotary axes,
DL-0023), an axis tree, the parts. Outputs: where every part is for any axis position, the axis
positions for each pose of a toolpath, and the contacts between parts, tool, part, stock and
fixtures.

## Approach

### Frames

The **model frame** is the machine's own frame with every axis at 0 (machine zero), +Z along the
spindle from the tool tip towards the spindle. Parts are placed in it as they stand at machine
zero, each by a translation and a rotation (about X, then Y, then Z) of its mesh.

The **table frame** is solve's (DL-0019, DL-0023): the model frame's axes at machine zero, its
origin at the **table centre**, where the table's rotary axes meet. The machine gives no point
for the work: each setup's **placement** puts its WCS somewhere on the machine (below), and
solve gets that WCS relative to the table centre. The **spindle frame** has its origin at the
**spindle gauge point** (given at machine zero) and +Z up the tool axis.

### The axis tree

Each axis of the model is one of the machine's axes (X Y Z A B C), carried by the base or by
another axis, with a unit direction and, for rotary axes, a pivot (a point of its centre line at
machine zero). An axis moves what it carries: a linear axis by q·d, a rotary axis by the
rotation of q about the line through its pivot along d (right-handed). With M_i the motion of
axis i alone,

    T_i(q) = T_parent(i)(q) · M_i(q_i),    T_base = I,

so an axis's motion applies inside the frame of everything that carries it. Parts, the spindle
and a setup's work are carried by an axis (or the base): a part at placement P is at
T_carrier · P.

The **table chain** is the chain of axes that do not carry the spindle; its end (`tableEnd`) is
the last of them (C of a trunnion, A of a 4th axis, the base of a head-head machine). The
**table rotaries** are the rotary axes on it. The table centre is where they meet: on the inner
one, the point nearest the outer one's line (the two lines need not cross exactly); with one
rotary axis, the point of its line nearest the WCS origin, so that along a 4th axis the program
is measured from the WCS as usual; without any, the model origin.

The rotary axes must turn the table and the head exactly as solve assumes (DL-0023:
T = Rx(A) Rz(C) for an A/C trunnion, H = Ry(B) for a B head, ...). The linear axes must move the
tool tip relative to the table along the machine axes: an axis that moves the table points
against its machine axis (X moving the table: direction -X).

### Default trees

`defaultModel` gives each kinematics type a tree, used for machines without a model of their own
and as the editor's starting point:

| Kinematics | Table side | Spindle side |
|---|---|---|
| `mill3` | Y (base, -Y), X (on Y, -X); table on X | Z (base, +Z); spindle on Z |
| `mill4_a` | ... A (on X, +X); table on A | Z |
| `mill5_table_ac` | ... A (on X, +X), C (on A, +Z); table on C | Z |
| `mill5_table_bc` | ... B (on X, +Y), C (on B, +Z); table on C | Z |
| `mill5_head_table_bc` | ... C (on X, +Z); table on C | Z, B (on Z, +Y); spindle on B |
| `mill5_head_bc` | table on the base | X, Y (on X), Z (on Y), C (on Z, +Z), B (on C, +Y); spindle on B |

Table rotaries pivot about the model origin, head rotaries `pivotLength` above the gauge point.
The gauge point stands 500 mm above the model origin at machine zero. The machine's collision
components (DL-0025) become parts of this tree: head components on the spindle's carrier, table
components on the table chain's end at the table centre, cradle components on the outer of two
table rotaries (else on the first linear axis under the table, so they move with the table
without turning).

### Validation

`validateModel` checks names, parents (no loops), directions, carriers and meshes, then the
motion numerically:

- the axes that do not carry the spindle must carry each other (one table chain);
- each linear axis at value 1 (rotaries at 0) must move the gauge point relative to the table
  chain's end by the unit machine axis; otherwise it names the axis, which side it moves and the
  direction it needs;
- each rotary axis at 0.7 rad alone must turn the table frame by solve's table rotation and the
  spindle frame by solve's head rotation; otherwise it names the axis and what it must turn about
  which machine axis;
- all rotaries together must agree too (the order of the axes in the tree).

### Axis positions for a pose

The rotary positions come from the kinematics as solve chooses them (`rotaryTrack`):
`planRotary` for multi-axis toolpaths, the 3+2 index (`indexAngles`) on machines that index,
else 0. A toolpath along WCS +Z on a machine that cannot index is a separate clamping: the part
sits with its WCS square to the machine (`mountedWcs`), as solve treats it.

With the rotaries fixed, the gap between the tool tip (gauge length below the gauge point along
the spindle's +Z) and the programmed point on the work (carried by the placement's axis), g(x,
y, z), is affine in the linear axis
values: translations along fixed directions, also below rotaries. `positionFor` evaluates g at 0
and at the three unit values, and solves the 3×3 system J·l = -g(0) (full-pivot LU; no solution
when the linear axes cannot reach every direction). This is what a control with tool centre
point control does with the machine's real pivots, so the positions shown are machine positions,
including the work offset, and are compared with the axis travel in the panel.

### Collisions

`checkMachine` samples poses as DL-0025 (every 2 mm of tip travel or 2° of rotary motion, rapid
ends, the last pose). Everything is a collider on a **body**: the base, each axis, the tool (on
the spindle's carrier) and the workpiece (part, stock box and fixtures, on the axis the setup's
placement names, where it puts them).
Colliders are stored in the model frame at machine zero and moved by their body's motion.

Pairs are skipped when they cannot meaningfully collide:

- same body, or a body and the one carrying it (a slide on its ways, the table and the part, the
  spindle and the tool);
- the tool against the part and stock (the stock simulation checks those) and the fixtures
  (`checkFixtures`, DL-0035);
- parts the user marked as not colliding.

Pair tests: convex against convex (GJK, as DL-0025), convex against mesh (a box query of the
convex's bounds in the mesh's frame, GJK per triangle), mesh against mesh (`queryPairs`: both
hierarchies traversed together, the other's boxes moved into this frame as the bounding box of
the moved box, grown by the clearance, descending into the larger box; GJK on triangle pairs). A
broad phase compares the colliders' moved bounding boxes first. Contacts merge over consecutive
moves as before; positions are reported in the WCS.

### Importing the machine's CAD

Machine makers usually ship the whole machine as one STEP file: an assembly, or one part made
of many bodies (base, enclosure, column, saddle, tables, trunnion, spindle head), or both. Every
body must become a part the user can carry on its own axis. `importStepAssembly` reads the file
into an XCAF document (product structure, product and body names, colours; lengths converted to
millimetres from the file's unit) and walks its free shapes: through assemblies and component
references, composing the references' locations, down to the leaf products, and splits every
leaf into its bodies (solids). Each body of each instance becomes a part, its shape placed in
the file's frame, named after the body where the file names it (STEP names on the solid's
representation item, read as sub-shape names), else after its product (numbered when the
product has several bodies; such a product joins the body's path), else after the file. Instance
names are mostly generated ("1", "NAUO12") and OCCT's own placeholders ("Product 3.1") count as
none. The colour is the body's, else the product's, else the nearest enclosing one. Shapes
without solids (surface models) stay whole.

`readMachinePartFile` drops the assembly levels all parts of a file share (the machine itself)
and also reads STL (one part, scaled by a chosen unit). `groupParts` offers merges as a
convenience: at level 0 (the default) every body on its own (repeated names numbered), at level
n the bodies under the same assembly n levels down share a group named after it.
`machinePartsFrom` makes one machine part per group, with the colour of its first coloured
member: the group's bodies as one STEP file (a compound of their shapes, `cad::compound`) when
they all come from STEP, else a mesh part (STL, its meshes appended). The parts stay in the
file's frame, so a machine modelled at machine zero needs no placement.

In the editor the import runs in a worker thread behind a progress dialog whose Cancel reaches
the STEP transfer (OCCT's progress indicator polls the cancel token) and the tessellation. The
list shows each body's assembly, file and solid count, every body a part of its own; the user
may pick a merge level, edit part names, merge selected rows or leave bodies out. Parts can
also be merged later in the parts table (`mergeParts`: each part moved by its placement into
one; exact when they all are, `cad::transformed` and `cad::compound`, else a mesh), and
several parts given one carrier at once.

### Exact geometry: STEP parts, derived meshes

A machine part keeps its exact geometry: the STEP file of its body or bodies, named by the
SHA-256 of its bytes like the meshes were (`ModelPart::step`). That file is the part's only data.
Its mesh, for the views and the collision checks, is derived: tessellated from the B-rep when
the machine is loaded (`meshMachineParts`, at the machine's `meshTolerance`, 0.2 mm by default,
0.5 rad angular; set in the editor next to the parts table), never edited and never saved. Parts
imported from STL have nothing better than a mesh and stay mesh parts (`ModelPart::mesh`).

`StepBodies.hpp` holds the shared pieces, for machine parts and (next) workholding bodies:
`stepFileOf` writes a B-rep as STEP bytes (and remembers the B-rep for them), `stepModel` reads
a file's B-rep once per session, `stepMesh` tessellates it once per file and tolerance. The
meshes go through the STL's 32-bit floats, so a mesh is the same made now or read back from the
disk cache (`setStepMeshCacheFolder`: `<sha256>-<tolerance>.stl`, the application's cache folder
in the GUI, always safe to delete; damaged files are made again). The cache counts what it did
(`stepCacheCounts`) for tests. Load times (release build): the 19 bodies of the demo trunnion in
0.06 s from STEP; a real machine, the maker's STEP of a 3-axis VMC (5.1 MB, 36 bodies, 72 000
triangles at 0.2 mm), in 2.05 s from STEP, 0.08 s again in the same session, 0.11 s with the
meshes from the disk cache (importing it: 1.1 s to read and split, 1.1 s to write and mesh the
parts).

**Picking on the B-rep.** The editor's preview and the setup placement view draw STEP parts as
their exact shapes (`OcctView::MachineScene::solids`: an AIS_Shape per part, faces, edges and
vertices selectable together, like the main view's models); the simulation keeps drawing the
meshes. A click reports the part and its sub-shape (`machineShapeClicked`: its type and index as
`cad::SubShape` numbers them, found by `cad::subShapeIndex`), and `cad::subShapeGeometry` gives
its exact geometry: a plane's area centroid and outward normal, a cylinder's or cone's axis (the
point of it nearest the face's centroid) and radius, a circle's or arc's centre, normal and
radius, a line's middle and direction, a vertex. The pickers use it:

- *Pick axis*: a round face or a circular edge gives the axis exactly (`axisOfShape`); *Pick 3
  rim points* takes a rim's circular edge at once, else three points (vertices exactly).
- *Pick gauge point*: a round face for the centre line, then the seat (its exact plane); or the
  circular edge where the holder seats, both at once.
- The setup placement (`snapOnShape`): a face's centre is a plane's centroid (a round face: its
  axis point; a circle: its centre; a vertex; a line's middle); the rotary axis meets a plane or
  a circle's plane; a point clicked on a vertex is the vertex. The work's part, fixtures and
  workholding bodies are B-rep too (in the WCS), the stock its mesh.

The mesh fits below (`geom3d::revolutionAxis`, `flatFace`, the circle through three points) stay
for mesh parts and the stock.

### Rotary axes from the geometry

A rotary axis must turn about the centre line of the part it carries (a 4th-axis faceplate on
the end of the X table, not the model origin that Default axes assumes). Imported parts are
meshes, so the line is found on the mesh (`geom3d::revolutionAxis`). From the triangle under a
picked point, the smooth region around it is grown across edges that bend less than 0.6 rad,
stopping at the sharp edges that bound a face. On a surface of revolution every normal line
meets the axis, so (for mesh parts; STEP parts give the axis exactly, above):

- the direction is the one the area-weighted normals vary least along: the eigenvector of the
  smallest eigenvalue of their covariance (for a narrow arc, whose normals vary in one direction
  only, the cross product of the mean normal and the direction they vary along);
- the point is where the normal lines through the facet centres meet in the least-squares
  sense: the solution of sum w (I - n nT) p = sum w (I - n nT) c. It lies at the face's mean
  height along the axis;
- a region whose normals barely vary is flat and gives no axis.

For tessellated cylinders and cones both are exact (each facet's normal through its centre
crosses the axis). The fallback for any rim is the circle through three picked points.

In the editor, *Pick axis on a round face* and *Pick 3 rim points* act on the selected rotary
axis. The picked point (from OCCT's picking, in the preview at the jogged position) is taken
back to machine zero through the part's carrier, the line fitted there, and the axis's pivot and
direction set to it, the direction in the sense the axis had (asked when the line is nearly
perpendicular to it). Every rotary axis is drawn as a dashed line through its pivot, moved by
what carries it, the selected one highlighted; jogging the axis then shows whether the part
turns in place.


The spindle gauge point is picked the same way, in two prompted clicks (*Pick gauge point*):
a round face of the spindle (the nose's bore, taper or outside) gives its centre line; then the
flat face the holder seats against, which must be square to that line (within 5 degrees, by the
normal of the triangle under the click), gives the height: the gauge point is where the line
meets that face (`gaugePointOn`). The model keeps the tool axis along Z at machine zero, so a
centre line that leans more than half a degree from Z is reported (the parts' placement is
probably off) and only its position is used. The preview draws the spindle's axis through the
gauge point and a stand-in tool (16 mm diameter, 100 mm long) hanging from it, which moves with
the spindle as the axes are jogged.

### Where the work sits: setup placements

The machine has no work origin: one setup puts its work on the top of a zero-point plate on the
table, another on the faceplate of the A axis. A setup's `SetupPlacement` holds:

- the **carrier**, the axis that carries the work (empty: the base);
- the **target**, a machine point at machine zero (model frame), and what it was snapped to;
- the **work point**, the point of the work placed on the target (project frame; unset: the WCS
  origin), and what it was picked on;
- an **offset** (model frame) and a **rotation** about machine X, then Y, then Z about the
  target, typed on top of the snap. Rotation 0 keeps the project axes along the machine's (the
  part upright as modelled), so a tilted WCS still indexes the table as before.

The project frame then sits at T(target + offset) · R · T(-work point), and the WCS at that times
the setup's WCS frame (`workMount`: the carrier and the WCS at machine zero). Solve, the post
and the checks get the WCS relative to the table centre for its origin (`wcsInTable`): the 3+2
index from its orientation, rotary positions without tool centre point control measured from
the table centre, positional (3+2) operations posted from the setup's work offset through the
table centre (DL-0036), and the simulation's and the collision checks' work carried by the
placement's axis (`OperationSimulation::mount`, the WCS squared as `mountedWcs` has it).

**Snapping** (`snapOnPart`): a click on a machine part, at machine zero, snaps to the centre of
the flat face there (`geom3d::flatFace`: the area-weighted centre of the coplanar region around
the triangle clicked), to where the centre line of the innermost rotary axis carrying the part
meets that face (`rotaryLineOf`, `lineMeetsPlane`: a rotary table's or a faceplate's centre) or
to the point clicked; the part's carrier becomes the work's. A click on the work (the part,
stock, fixtures and workholding, all drawn in the WCS) picks the work point the same way, so a
zero-point plate or vise imported as workholding is stood on the machine by its bottom face.
The work's own bodies cannot be targets: they move with the work.

**Without a placement** a setup keeps the default (`defaultPlacement`), shown as such in its
properties and the placement view: the work on the end of the table chain, the project origin
at the table centre, the project axes along the machine's. It is how setups sat before
placements existed (the project frame was the table frame), so older projects post as before
on machines without a mesh model. Machine files of version 3 kept a table origin (the work
origin, a single point for every setup); it is dropped when they load.

**Refusals:** a carrier on the spindle's side, or one that a linear axis moving the table does
not move (the base of a machine whose table travels), cannot carry the work: posting is refused
naming it and the machine is not simulated (a warning says so). A carrier that a table rotary
does not carry (a vise on the X table beside a 4th axis) is allowed, but that rotary must stay
at 0: operations that turn it are refused when posting and not simulated on the machine.

### Storage

Machine files are version 5 of `oxcam.machine` (`schemas/machine.v5.schema.json`; versions 1
and 2 migrate unchanged: version 2 added the optional `model`, version 3 the parts' optional
`shown`; version 4 dropped the model's `table`, whose carrier and origin are ignored; version 5
added STEP parts and `mesh_tolerance`, older mesh parts load as they are). The optional `model`
holds the axes, spindle, mesh tolerance and parts; each part names its file by its SHA-256:
`step` (its STEP file) or, for parts with only a mesh, `mesh` (its binary STL). The files are
stored as `<sha256>.step` or `<sha256>.stl` in `<machine file name>.meshes/` next to the machine
file (the folder keeps the name it had; the library keeps `<id>.json` and `<id>.meshes/`; export
writes both), and as `machines/<sha256>.step` or `.stl` in project archives. Loading checks each
hash. A setup's placement is the setup's optional `placement` in the project file
(`schemas/project.v1.schema.json`); files without it load with the default placement. The
toolpath key leaves it out (toolpaths are generated in the WCS) unless an operation has a tool
orientation of its own, whose reach depends on it; the simulation key includes it. Imported STL
meshes are rounded to the STL's 32-bit floats once, so writing a loaded mesh again gives the
same bytes and the same name; STEP files are written once and copied as they are (their header
holds the time written).

### Display

The simulation panel computes the pose for the playback position (rotaries interpolated along
the move, the tip from the toolpath) and draws each part at its place. Two views: **machine
fixed** (the machine stands still; the part, stock, toolpaths and tool move with the table) and
part fixed (the machine moves around the part). The axis positions are shown, red beyond their
travel. The library editor shows the same model with sliders for each axis. There the parts are
pickable: the rows selected in the parts table (or among the boxes and cylinders) are drawn in a
highlight colour, a click on a part selects its row, and Hide selected, Isolate selected (which
also frames them) and Show all override what the preview shows without touching the machine.

Each part can be shown or hidden in the views (an enclosure hides the work). Visibility never
changes the checks: hidden parts are checked and reported like any other. Two levels:

- **The machine's default**, saved with it (`shown`, the editor's Shown column): a machine can
  ship with its enclosure hidden, and projects carry it with their copy of the machine.
- **The session**, in the simulation panel: a list of the machine's parts with a check box each
  starts from the defaults; changes last until the application closes (kept per machine id and
  part name), so trying views does not edit the machine.

## Sources

- ISO 10303-214:2010, *Application protocol: Core data for automotive mechanical design
  processes* (and AP242). Product structure: products, their definitions and assembly usage
  occurrences with placements, read here through OpenCASCADE's XCAF.
- J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 3rd ed., Pearson, 2005.
  Homogeneous transforms and forward kinematics of serial chains (ch. 2-3).
- ISO 841:2001, *Industrial automation systems and integration — Numerical control of
  machines — Coordinate system and motion nomenclature*. Axis names and positive directions
  (paraphrased: positive motion is the tool moving relative to the workpiece).
- C. Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2005. Bounding volume
  hierarchies and tree-against-tree traversal (6.3), GJK (9.5).
- 3D Systems, *StereoLithography Interface Specification*, 1989. The STL file layout.

## Deviations from sources

- Tree-against-tree traversal uses axis-aligned boxes in each tree's own frame, re-boxed after the
  relative motion, instead of oriented boxes: looser boxes, but no rebuild per pose.
- Adjacent bodies are not checked (as robot collision checkers commonly skip adjacent links):
  parts that slide on each other would otherwise always touch.

## Correctness and tests

- **Placement (`tests/unit/machine/ModelTest.cpp`):** for every mill kinematics type, 50 random
  rotary positions and tips: the tool tip lands on the programmed point of the table, the table
  and spindle turn as `tableRotation` and `headRotation`, the tool axis is `toolAxis`, table parts
  stay put on the table and cradle parts turn as `cradleRotation`; with the rotaries at 0 the
  linear axes are the tip relative to the table.
- **Validation:** a table axis pointing the machine axis's way, a rotary on the wrong side, the
  rotaries in the wrong order, unknown carriers, missing axes and loops are named.
- **Files:** a machine with three mesh parts written and read back equal (meshes byte for byte),
  unused meshes removed, a missing mesh an error; a frozen version 1 file loads; project archives
  keep machine meshes once each and refuse a missing one (`DocumentTest.cpp`).
- **STL and BVH (`StlTest.cpp`):** binary round trip, ASCII, malformed files; `queryPairs` finds
  every triangle pair closer than the margin among 300 × 300 random triangles.
- **Checks (`MachineCheckTest.cpp`):** the DL-0025 cases unchanged; a mesh trunnion machine whose
  table travels into a fixed column, the head on the part, parts not colliding left out, the
  spindle nose coming down on a clamp, the tool in a clamp left to `checkFixtures`.
- **One file, many bodies (`tests/unit/cad/AssemblyTest.cpp`,
  `tests/unit/document/MachineImportTest.cpp`):** `trunnion_machine.step`, a whole 5-axis
  trunnion machine as one STEP part of 19 bodies written in inches (base, enclosure, column,
  saddle, x table, two supports, cradle, rotary table, spindle nose and head, named; eight bolts
  unnamed): 19 parts, each by its own name, colour and place in millimetres; each assigned on its
  own (saddle on Y, table and supports on X, cradle on A, round table on C, spindle on Z, the rest
  fixed) gives a valid machine whose bodies move with their own axes.
- **Axis picking (`tests/unit/geom3d/AxisFitTest.cpp`, `MachineImportTest.cpp`):** the axis of
  a tilted tessellated cylinder (exact direction, point on the axis at the face's middle,
  radius), a cone, a 50 degree arc; flat caps give none; the circle through three points. On the
  demo machine, a point on the round table's rim gives Z through the table centre, halfway up the
  rim; the spindle nose gives Z through the spindle; a flat face and a point off the part give
  none. A GUI test picks the C axis on the isolated round table through the preview's own
  picking and reads the pivot (0, 0, -25) and direction (0, 0, 1) back.
- **Table chain (`ModelTest.cpp`):** the trunnion's chain ends at C, its centre where A and C
  meet (moved pivots too); a 4th axis's centre is the point of A nearest the WCS origin; a
  head-head machine's work sits on the base; the spindle's side is refused. A version 3 file
  with a table carrier and origin loads without them; a version 4 file with one is refused.
- **Placements (`tests/unit/document/PlacementTest.cpp`, `ProjectTest.cpp`):** block_pocket with
  its top faced and its front faced at an orientation of its own (A -90):
  - on a 50 mm zero-point plate on the trunnion's round table, snapped to the plate's top centre
    with the stock's bottom centre on it: carried by C, the WCS origin at (-51, -31, 81) from the
    table centre; the front posted 51 mm below the setup's work offset (31 unplaced); the
    simulation carries the work on C there, and the machine puts the tool tip on the work; turned
    90 degrees on the plate, the WCS turns with it;
  - on a 4th axis's faceplate, snapped where A meets its face with the stock's end against it:
    carried by A, the WCS 31 mm in front of and 15.5 mm above the axis; the front posted 14.5 mm
    above the work offset (-31 unplaced), simulated on A;
  - refused on Z (the spindle's side; the post refuses, the machine is not simulated), on the
    base of a machine whose table travels, and on the X table beside the 4th axis for an
    operation that turns A (allowed without it);
  - the vise of `vise_project.ocam` (workholding) stood on the plate by its base's bottom centre,
    picked on the workholding's mesh; simulated there;
  - the project round trip of placements (snapped, picked and typed), files without them, and a
    carrier that is no axis refused by the schema. A GUI test places the 3+2 plate project on the
    demo trunnion through the placement view's own picking: the plate's side face centre onto the
    round table where C meets its top, carried by C.
- **STEP parts (`tests/unit/document/MachineStepTest.cpp`, `tests/unit/cad/SubShapeGeometryTest.cpp`,
  `ModelTest.cpp`):** the demo machine's 19 parts are STEP files next to it, their meshes closed
  and within 2% of the B-rep volumes; meshes are made once per file and tolerance (a second load
  makes none, another tolerance makes them again, coarser), the disk cache gives the same meshes
  without reading STEP, damaged cache files are made again; projects keep the parts as
  `machines/<sha256>.step` and open them in a new session; merged STEP parts stay exact (volume,
  the moved one where it stood), with a mesh part among them the result is a mesh. Picking on
  the demo: the round table's rim gives the C axis exactly (pivot (0, 0, -25), radius 210), its
  top edge the axis at the top, its top face the centre (0, 0, 0) for both snaps, a vertex
  exactly, the spindle nose's bottom rim the gauge point (0, 0, 450). The exact geometry of the
  hole plate's faces, edges and vertices (planes with outward normals, the through hole's
  cylinder and rims); STEP data in memory, compounds and moved copies keep the volume and the
  face numbering. Machine files: a STEP part and a mesh part saved and loaded (the STEP bytes as
  they were), a part with neither or both refused, an unused STEP file removed, a corrupt one an
  error, a version 4 file's mesh parts as they were. GUI tests pick the C axis on the rim's
  exact cylinder and on its top circular edge. A benchmark (`test_document "[.bench]"` with
  `OXCAM_BENCH_MACHINE_STEP`) times importing and loading a maker's STEP file.
- **Gauge point:** on the demo machine, the spindle nose's outside and its bottom face give the
  gauge point (0, 0, 450), the machine's own; a GUI test picks both faces through the preview
  (the seat seen from below) and reads it back.
- **Visibility:** a part's `shown` flag survives the machine file round trip; a frozen version 2
  file loads; hidden parts are still reported in the collision checks.
- **Assemblies:** `trunnion_assembly.step`, the machine as an assembly with nested
  subassemblies, a product used twice and the bolts, every product modelled at its own origin
  and placed by its component: 18 parts with their names, paths and colours where the assembly
  puts them; merged one level down into 6 parts with the expected bounds; files without
  structure split into solids; cancelling throws; STL in inches scaled.
- **Fixtures:** `machines/trunnion-demo.json`, built from `trunnion_machine.step` as the editor
  does (every body its own part, carried by its axis), and `dome5_machine_project.ocam`, the
  5-axis dome on it: simulated without machine contacts. GUI tests open the import dialog on
  both files.

## Known limitations

- **Sampled poses** every 2 mm / 2° (as DL-0025); rapids follow the straight tool-tip path.
- **Cost:** mesh checks run on every pose. The 5-axis dome on the demo machine simulates
  in 80 s against 60 s on the built-in trunnion's components.
- **Travel** beyond the axis limits is shown in the panel, not reported as an error.
- **Placement comes from the file:** an assembly is expected to be modelled at machine zero; a
  part from elsewhere is placed by hand (offset, rotation). Parts are not matched to axes by
  their names: the user assigns each one.
- **Large files:** every body is a part until merged; very detailed CAD (threads, fasteners)
  makes large meshes and slower checks: a coarser chord tolerance, merging and leaving bodies out
  keep them manageable.
- **Visibility by name:** the session's choices are kept by part name; parts with the same name
  show or hide together.
- **Parts on the same chain** that are not adjacent (a table on a saddle on the base) are checked
  against each other; parts that touch by design can be marked as not colliding.
- **Lathes** have no machine model yet.
- **Two triangulations** in the picking views: the AIS shapes of STEP parts are triangulated by
  the viewer for display, apart from the meshes the checks use.
- **Older machines stay meshes:** parts of files before version 5 have no STEP file; import the
  maker's STEP again to pick on exact geometry.
- **Snaps are points:** a placement keeps the snapped point, not the face; after the machine's
  parts move or change, snap again.
- **The work and the parts on its carrier** are not checked against each other (they sit on
  each other, as the table and the part did): a placement that sinks the stock into a zero-point
  plate on the same axis is not reported.
- **One rotary table chain:** the table centre is where the table rotaries meet; machines whose
  table rotaries are offset from each other are posted as if they met there (solve's kinematics,
  DL-0023).
