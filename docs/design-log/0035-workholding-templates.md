# DL-0035: Workholding templates (fixtures with moving members and stock holdings)

- **Status:** implemented (version 3: chucks and collets; version 2: parts, members, datums and
  stock holdings; version 1 had vises with jaw roles and one datum frame)
- **Module:** libs/document (`WorkholdingTemplate`, `WorkholdingPart`, `WorkholdingMember`,
  `WorkholdingDatum`, `StockHolding`, `SetupWorkholding` in `Project.hpp`; `Workholding.hpp`:
  file format, `datumAt`, `memberTravels`, `poseWorkholding`, `setupWorkholdingPose`,
  `pickedDatum`, `viseTemplate`; `withPlacedWorkholding` in `PostSetup.hpp`;
  `fixtureBodiesInWcs`, `latheBodies` in `Generate.hpp`; `solidPartsFrom` in
  `MachineImport.hpp`), libs/strategies (`TurnGeometry::backStop`), libs/cad
  (`datumGeometry`, `pointOnFace` in `Datum.hpp`, on `cad::subShapeGeometry`;
  `Model::exportStepData`, `cad::compound`), libs/sim (`checkFixtures` in `FixtureCheck.hpp`;
  `revolvedEnvelope`, `checkTurningFixtures` in `Lathe.hpp`), GUI (the workholding editor, the
  setup wizard, the parts table
  shared with the machine editor, the setup page's Workholding rows, the placement dialog, the
  view, the simulation panel)
- **Patent gate:** low (citation only). Placing a fixture model by datums picked on it, moving
  clamping members along lines to an opening, seating stock on a datum frame, and checking a
  tool against fixture solids at sampled poses are long-established practice; nothing here plans
  toolpaths or controls engagement.
- **Author(s):** OxCAM contributors

## Problem

DL-0034 made fixtures static models placed like the part. Shops use the same fixtures job after
job: standard vises, self-centring vises, dovetail fixtures, zero-point plates with a vise on
them, custom fixtures. A fixture should be defined once from its maker's STEP file (its bodies,
which of them move and how far, where the stock sits), saved, and placed in any setup with its
members closed on that setup's stock. The setup's work then sits on the machine by the fixture:
the fixture snaps onto the machine like a setup placement (DL-0037), and the stock, part and WCS
follow. The placed fixture is kept clear of like any fixture, checked in the simulation and shown
in the view.

Version 1 had one kind of fixture, a vise: bodies with jaw roles, one datum frame on the fixed
jaw and a range of openings. That cannot describe a dovetail, a plate with a seat, a second vise
station, or datums picked on the geometry.

## Approach

### Parts, members, datums, holdings

A template is:

- **Parts**: STEP bodies, each an embedded file in the template's coordinates (one file's bodies,
  as its assembly places them), each carried by the fixed **base** or by a **member**, with a
  colour, whether it collides (toolpaths keep clear of it; it is checked in the simulation) and
  whether it is shown. Parts stay B-rep: datums are picked on exact faces, edges and vertices.
- **Members** (a jaw, a clamp): a direction, the travel range from where the parts are modelled,
  and optionally a link: a linked member's travel is `ratio` times its leader's, along its own
  direction. A self-centring vise is two jaws in opposite directions, the second linked to the
  first with ratio 1: they move apart symmetrically and their midpoint stays put.
- **Datums**: named points with a direction (unit or none), a small relationship graph:
  - *picked* on a part: a flat face gives its centre of mass and outward normal, a cylindrical or
    conical face its axis (the face's centre projected onto it), a circular edge its centre and
    axis, a straight edge its middle and direction, a vertex its point, a click on a face the
    nearest point of the face and the normal there (`cad::datumGeometry`, `cad::pointOnFace`,
    exact on the B-rep). The geometry is stored with a persistent sub-shape reference
    (DL-0005); the datum moves with its part's member;
  - *typed*: a point and direction in template coordinates, moving with a chosen member;
  - *offset*: another datum moved a distance along template X, Y, Z or that datum's direction;
  - *midpoint*: halfway between two datums, with the first one's direction.
  `datumAt(template, name, travel)` evaluates a datum with the members at given travels
  (recursively; cycles are refused, and validation reports them).
- **Stock holdings**, one or more per template (two vise stations, step jaws):
  - **jaws**: faces A and B clamp the stock's width; its side rests on face A (and is centred
    when both faces move), its bottom the grip depth below the jaw tops (the *seat* datum);
  - **dovetail**: flanks A and B hold a tenon of a given width at its shoulder (0 = the flanks
    as modelled) and depth below the shoulder, which sits on the seat;
  - **seat**: a frame (origin and Z from the seat datum, X from another datum's direction or
    template X) that a chosen point of the stock's box (min, middle or max per axis) goes to.
- **Mount**: the datum the template sits on the machine by (its base's bottom, a zero-point
  pin); empty = the template origin. **Up**: the template's up direction.

### Closing the members on the stock

`poseWorkholding(template, placement, stock bounds in the WCS)`:

1. The stock's axes are the WCS's turned by the setup's rotation (0, 90, 180 or 270 degrees
   about WCS Z): `aW` (the holding's X), `bW = Z × aW`. The stock spans `[a0, a1]`, `[b0, b1]`,
   `[z0, z1]`.
2. **Jaws.** `aT` is the direction from face A to face B in template coordinates, square to up.
   The opening is `(B − A)·aT`, a linear function of the members' travels. The target is the
   stock's width `a1 − a0` (or the typed opening). The driving member is the one face B moves
   with (or the first member that changes the opening); members that follow it are set from its
   travel. One probe gives the slope `k`, and the driver's travel is `(target − now) / k`. Every
   member's travel must be within its range, else placing fails naming the member, its travel and
   its range; generation refuses (no silent substitute, in ADR-0004's spirit). A typed opening
   narrower than the stock is placed with a warning. The stock's anchor in template coordinates
   is face A's point along `aT`, face A's point plus the shift across (`bT = up × aT`), and the
   seat's height minus the grip depth along up; in the WCS it is `a0` along `aW`, the middle of
   `[b0, b1]` and `z0`.
3. **Dovetail.** As for jaws, with the opening measured between the flank planes where they
   cross the seat plane: for a flank through `p` with normal `n`,
   `s = (p·n − (p·bT)(bT·n) − zs (up·n)) / (aT·n)`. The target is the tenon width. The anchor is
   midway between the flank lines at the seat, the seat's height minus the tenon depth; in the
   WCS the stock's middle across and along and `z0`.
4. **Seat.** Members stay at the setup's typed travels. The template frame is the seat's; the
   anchor in the WCS is the stock's chosen box point.
5. The template maps to the WCS by `Frame{anchorW, aW, Z} · Frame{anchorT, aT, up}⁻¹`, each part
   moved first by its member's travel along the member's direction. Members a holding does not
   close stay where the setup types them (`SetupWorkholding::travel`).

Version 1 vises migrate to the same behaviour exactly: the moving jaw becomes a member along the
old X axis with the travel range `opening − modelled`, typed datums on the jaw faces (the old
origin, and the origin plus the modelled opening) and tops, and one jaw holding; a self-centring
vise gets two linked jaws travelling half the change each.

### The workholding on the machine

A setup placement (DL-0037) puts a point of the work at a machine point. With
`SetupPlacement::onWorkholding` that point is the workholding's mount datum, placed around the
stock (`withPlacedWorkholding` turns it into the work point before workMount, the post, solve
and the simulation use it). So the fixture stays on the machine where it was snapped, and a
larger stock moves the WCS, not the fixture. Everything else stays in the project frame: the
WCS is defined on the part, and the workholding's pose in the WCS follows from the holding.

A vise on a zero-point plate keeps one WCS for every job: the setup's WCS origin can be at the
bottom centre of the fixture box (the fixtures and the placed workholding) or at a point picked
on the workholding (`workholdingPoint`: a picked datum moving with its member), and it follows
the workholding when the stock changes (DL-0038).

### Using the placed parts

The placed parts that collide are fixtures like those of DL-0034: `fixtureBodiesInWcs`
tessellates them, the avoidance pass keeps the tool, shank and holder the fixture clearance away,
and the simulation checks the tool assembly against them (`checkFixtures`: convex tool slices
against the fixtures' triangles with GJK at poses sampled along the moves, tilted along the
toolpath's axis; DL-0035 version 1). The toolpath key holds the template (parts by file hash,
members, datums, holdings) and the setup's placement in it. The view draws the shown parts
(not pickable: they are no project models).

### Files

A template file (`*.ocamhold`) is a zip archive of `workholding.json` (format
`oxcam.workholding`, version 3, `schemas/workholding.v3.schema.json`; version 1 documents are
migrated when read, version 2 documents are version 3 documents without chucks) and `bodies/<sha256>.step`, checked against their hashes. The user's library
is a folder of such files, one per template, named after its id. A project keeps copies of the
templates its setups use in `workholdings` and their part files among its embedded models; the
setup's `workholding` gains `holding` and `travel` and an optional `grip_depth`, the placement an
`on_workholding` flag; files without workholding are written exactly as before.

### The editor and the setup wizard

Members, links, ratios and datums are what a template is made of, but asking for them directly
proved hard to follow (a self-centring vise needs a second jaw linked to the first with the
opposite direction and ratio 1). Common fixtures are set up by a wizard instead, which asks what
the fixture is and then for clicks in a 3D view, one question at a time:

- **Standard vise**: the moving jaw's parts, the fixed jaw's clamping face, the moving jaw's
  clamping face, the jaw tops, what it stands on (optional).
- **Self-centring vise**: the first jaw's parts, the second jaw's, their clamping faces, the jaw
  tops, what it stands on.
- **Dovetail fixture**: the moving flank's parts (optional: both fixed), the two flanks, the
  seat, what it stands on.
- **Plate or custom seat**: the face the stock sits on, an edge along the stock's X (optional),
  what it stands on.
- **Chuck**: every jaw's parts, a round face on the spindle axis (the body's outside or bore), a
  jaw's gripping face, the front of the jaws (or the step the bar sits against), the back of the
  chuck (optional).
- **Collet**: its bore, its front face, the back (optional).

Makers' files often carry more than the fixture in hand (other jaw sets, optional risers) or
split one part into several bodies. The wizard's parts list removes parts
(`removeWorkholdingPart`: the part, its file when no other part uses it, datums picked on it) and
merges them (`mergeWorkholdingParts`: one STEP file with all their solids, in the first one's
place, on its member); answers that named them are cleared (a merge keeps the parts answer, on
the merged part). Selected parts show in the view. The last page builds the template from the
clicked answers once and applies its typed values (name, range, depths, the stock point) to a
copy on each edit, and `cad::Model::info` is worked out once per model: rebuilding from the
clicks on every keystroke re-measured every part's exact bounds and volume, which froze the page
for large maker's models.

`buildWorkholding(answers, parts, models)` turns the answers into a template: the clicked parts
on the type's members (the second jaw of a self-centring vise linked to the first, moving the
other way), picked datums on the clicked faces (exact B-rep), the type's holding, up from the jaw
tops' (or seat's) normal, the mount on the clicked face or at the bottom centre of all the parts,
and the opening range typed on the last page set on the members (`setOpeningRange`: the driving
member's travel range and its followers'). A moving jaw's face on a part that does not move is
refused with a message. The last page has the jaws moved by one Opening slider
(`travelsForOpening`: the driving member and those linked to it, the same function the pose
uses) and closed on a trial stock of a given size, drawn in the view.

The editor shows the same simple controls first: what the template is, its opening range, the
grip depth, an Opening slider and a trial stock in the preview, and "Set up with the wizard"
(again on the template's parts). Parts, members, datums and holdings are under **Advanced**, for
fixtures the wizard does not know (clamps, several stations, jaws that move on their own): the
multi-body STEP import
with merging by subassembly or by hand (kept as solids: `solidPartsFrom`), the multi-edit parts
table (`PartTable`, lifted from the machine editor), member jogging, and datums picked on faces,
edges and vertices in the preview, typed, offset or midway. New from STEP imports the file (every
body a part) and goes straight into the wizard.

### Chucks and collets on the lathe

A chuck holding (`HoldingKind::Chuck`, format version 3) grips a bar on its diameter, centred on
an axis: `axis` is a datum on the spindle axis, `faceA` a jaw's gripping face (its distance from
the axis is the radius gripped, so the opening is a diameter), `seat` the front of the jaws (or a
jaw step) facing out of the chuck, and the grip length (`gripDepth`) how far the bar reaches
behind the seat's plane. The pose puts the axis on WCS Z through the stock's middle, out of the
chuck along +Z (the lathe convention: the part's front at the largest Z), the jaw front the grip
length above the stock's bottom end, and the face A jaw towards the setup's rotation direction.
The members close through the same opening functions as vises (`travelsForOpening` with the
gripping face's member as the driver).

The wizard groups the clicked jaw parts by their angle about the axis (within 10 degrees: a
master jaw and its top jaw are one jaw), makes each group a member moving radially outward
through its centre, and links every jaw to the one with the gripping face at ratio 1, so 3-, 4-
and 6-jaw chucks (any number from two) close together. A round gripping face (soft jaws bored to
size, a collet's bore) is picked at a point on it at its radius. A collet has no members: its
bore is the opening, and `gripRange` says how much smaller a bar it still grips; a bar outside
that range is refused with the range.

Turning keeps clear of what turns with the spindle. `latheBodies` turns each colliding body of
the setup's fixtures and workholding into the area it sweeps about the spindle axis in the lathe
plane (`revolvedEnvelope`: per 0.5 mm slice along Z, from the body's nearest radius to its
farthest, the last slice ending at its front), grown by the fixture clearance. Outside roughing
and finishing stop in front of the bodies behind the part's front (`TurnGeometry::backStop`: their
front plus the insert outline's reach behind its programmed point), with a warning when the part
reaches behind it. Every turning toolpath is then checked: the insert outline swept along each
move, as the lathe simulation sweeps it, must not overlap any area
(`checkTurningFixtures`); a toolpath that does is refused, naming the body and where (Z and
diameter). The lathe simulation runs the same check without the clearance and lists the
contacts with the operation. Lathes have no machine model yet (DL-0037), so the chuck is not
snapped onto a spindle; its axis is the WCS Z the lathe already turns about.

## Sources

- C. Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2005 (bounding volume
  hierarchies, ch. 6; GJK, 9.5).
- E. G. Gilbert, D. W. Johnson, S. S. Keerthi, "A fast procedure for computing the distance
  between complex objects in three-dimensional space", *IEEE Journal on Robotics and
  Automation* 4(2), 1988.
- The fixture avoidance of DL-0034, the persistent references of DL-0005, the convex tool slices
  of DL-0026 and the machine models and placements of DL-0037.

## Deviations from sources

None.

## Correctness and tests

- **Format and migration** (`tests/unit/document/WorkholdingTest.cpp`): version 1 vises
  (standard and self-centring) migrate and hold the stock exactly as before; JSON round-trip and
  schema rejections; template problems (unknown members, travel ranges, missing and cyclic
  datums, missing holdings, links to unknown members); archive round-trip and integrity; project
  round-trip with a holding, member travel and a placement on the workholding; older files
  unchanged. The frozen OpenCAM 0.1 vise and project (`LegacyFormatTest.cpp`) open, place the
  stock as before and generate.
- **Datums**: a plate's top face (exact centre and normal), a straight edge, a pin's round face
  (its axis), a point clicked on a face, a vertex; offsets and midpoints.
- **Holdings**: a standard vise imported from one multi-body STEP file, its fixed jaw on the
  stock's side, a typed opening, one narrower than the stock, out of range; a self-centring vise
  centring stock of three widths; a dovetail seating the tenon as modelled and a wider tenon
  (the moving flank opens, the stock moves half way); a plate seating the stock's bottom centre
  and its min corner.
- **End to end**: the vise template placed on block_pocket's stock keeps the profile clear of the
  jaws, a part that does not collide is left out, the simulation finds no collision for the
  trimmed path and finds the untrimmed one, the project saves and opens with the parts, undo
  removes the vise.
- **Placement** (`PlacementTest.cpp`): the vise's mount datum snapped onto a zero-point plate
  stays there when the stock grows (the WCS moves), in the mount and the simulation.
- **The WCS origin on the workholding**: see DL-0038 (`WcsTest.cpp`).
- **Fixture check** (`tests/unit/sim/FixtureCheckTest.cpp`).
- **Wizard** (`[wizard]`): its answers make a standard vise (the opening range on the member,
  the stock held as by the bounding-box vise, a moving face on a fixed part refused), a
  self-centring vise whose jaws move together from one opening, a dovetail (and one with both
  flanks fixed: no range) and a seat.
- **Chucks** (`[chuck]`): the wizard's 3-jaw chuck (three linked radial members, its range of
  diameters, every jaw closing by the same amount, the bar centred with the jaw front the grip
  length up, a typed grip length and opening, too large a bar, the gripping face on the body and
  one jaw alone refused); the collet gripping its bore and a bar 0.4 mm smaller, refusing bars
  outside its range; the generated templates' files and JSON. The shaft held in the chuck
  (`chuck_project.ocam`) generates every turning operation clear of the jaws and simulates with no
  contact; gripped deeper, roughing and finishing stop in front of the jaws and say so, and
  parting off into them is refused, naming the jaw. The revolved area of a jaw block and the
  insert check against it (`tests/unit/sim/LatheTest.cpp`). The frozen version 2 templates in
  `tests/fixtures/workholding-v2` open (`LegacyFormatTest.cpp`).
- **GUI**: the library lists the test fixtures; the editor makes a template of a multi-body STEP
  file and picks a datum; the wizard sets up a self-centring vise from clicks, closes it on a
  trial stock and links its jaws, a 3-jaw chuck whose jaws grip a trial bar together, and a
  maker's file with extras (a riser removed, a jaw's two bodies merged) set up as the
  self-centring vise; the vise project shows the vise. Removing and merging parts
  (`[wizard]`): files and picked datums go with the part, the merged part has both solids in the
  first one's place and sets up like a one-body jaw.

## Known limitations

- **Linear members only.** Clamps that swing and cams are not modelled; a rotary clamp would be
  a member turning about a datum's axis. Chuck jaws move radially along lines, which they do.
- **Jaws together.** The wizard links every chuck jaw to the first; an independent 4-jaw chuck
  (each jaw set on its own to hold work off centre) needs its members set by hand, and the bar is
  still centred on the axis.
- **No tool holder on the lathe.** Turning clearance checks the insert outline only; the
  holder's shank behind it is not modelled yet.
- **Upright in the WCS.** A holding turns the stock about WCS Z only (0, 90, 180, 270 degrees);
  stock tilted in a fixture is a setup of its own.
- **Box stock.** Holdings use the stock's bounding box in the WCS (a cylinder's bounds for round
  stock; a chuck grips its width along the setup's direction as the diameter); V-blocks are not
  modelled.
- **The tenon is not checked.** A dovetail holding takes the tenon width and depth as given; the
  stock's tenon is not compared with the flanks' angle.
