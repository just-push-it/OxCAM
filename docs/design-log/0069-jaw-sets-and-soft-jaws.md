# DL-0069: Jaw sets on a chuck, and soft jaws bored to fit

- **Status:** implemented
- **Module:** libs/document (`JawMount`, `JawSet`, `JawOption`, `SetupWorkholding` jaw choices, `poseWorkholding`, `posedPartModel`,
  `softJawBoring`, workholding file version 4), libs/cad (`subtracted`), libs/strategies (`turn_soft_jaws`), apps/opencam-gui (the
  workholding library's Jaws section, the setup page's jaw choices)
- **Patent gate:** low (citation only) by the risk table: rigid placement of jaws from datums, and DL-0020's turning roughing in a
  region worked out from the setup. Generating an operation from the setup is new automation, so the change carries
  `needs-patent-review` for the review group. Not reviewed, and no search done (the policy leaves searching to the group).
- **Author(s):** OxCAM contributors

## Problem

A lathe user changes the top jaws on one chuck: hard step jaws for first operations, soft jaws bored on the machine for most
second operations. The work can be held on an outside diameter or in a bore, and step jaws go on either way round. Until now
a chuck template held one fixed jaw set: another set meant another template, and soft jaws could not be modelled at all.

What is needed:
- the chuck apart from its jaws, with the jaw sets it takes;
- the setup saying which set, which way round, which step and which diameter;
- soft jaws cut to the bore the setup asks for, in the checks and the views;
- the operation that bores them, from the same numbers.

## Approach

### What belongs where

- **Chuck template:** the body and the master jaws (members `Jaw 1` to `Jaw N`, as the chuck wizard makes them), its stroke
  (their travel range), and its **jaw mount**: the interface by name (a label the user chooses) and a seat datum on
  master jaw 1, where a jaw's seat goes, its direction out of the chuck. Its own top jaws are parts marked `top_jaw`: the chuck
  holds with them when no set is chosen.
- **Jaw set:** a template of its own (a library item like any other), one jaw's parts, copied onto each master jaw. It names its
  interface and kind (hard or soft), its front, and its **options**, the ways it goes on: each a name, the datum that seats on
  the master jaw, which way is in towards the axis, and its steps (a gripping face and the face the work stops against).
- **Setup:** the jaw set and its option, outside or inside grip, the step, and for soft jaws the bore.

Interfaces are compared as mount names are (DL-0068): case and spacing do not matter. A set for another interface is not offered,
and one already chosen is said, not refused.

### Options, as a vise's holdings

A step jaw that goes on either way round is two options, such as "OD" and "Reversed (ID)": the same parts, each option with its
own seat datum, way in and steps. That is how a vise template carries its ways of holding (DL-0035: several stock holdings, each
with its own faces, chosen by the setup by name), so a chuck's jaws follow the pattern instead of a reversing flag and the math
that turned a jaw about its seat. The setup names the option; none named is the set's first.

### Placing a set

Master jaw 1's seat datum at travel `t` and its radial direction `r` (from the axis through the seat) give the seat point `S(t)`.
The option's seat point goes there, as modelled: its seat direction out of the chuck, its inward direction to `-r`. The other master
jaws get the same jaw turned about the chuck's axis by their angle.

A step's gripping face then stands at radius `ρ0 + k·t`, where `k` is how far the seat moves out per mm of travel. It grips an
outside diameter when its normal, once mounted, faces the axis, else an inside one: OD or ID is the step's own, from its face.

The step is the setup's, or (Auto) the first that grips the way asked (OD or ID) and reaches the held diameter within master jaw
1's travel, else the first that grips that way. An entry with no step that grips the way asked is refused, and shown where its
jaws stand, in the error colour, so it can be seen which way they face.

With a jaw set on, the work goes into the jaws as far as the step's stop face: the chuck's grip length does not apply, and the
setup page does not show it. The setup's `stop_offset` (Jaw face offset, 0 by default) stands the work's held end that far out from
the stop face (a soft set: from its bore's floor), for work that is not pushed home against it; the stop moves down the work, not
the work along the jaws. It cannot be negative: the work does not go in past the stop. (The maintainer asked for it on
2026-11-07: "stock won't always be hard against a back stop, but it's a perfectly fine default.")

Gripping diameter `D` needs `t = (D/2 − ρ0)/k`. The range of stock a set holds is the master jaws' travel range, as for any
workholding: a diameter that needs master jaw 1 outside it is beyond its reach (below).

### Stock beyond reach

Any workholding (a vise, a dovetail, a chuck, a collet, a jaw set) whose members cannot reach the stock is posed as far as it
goes: the members stop at the end of their travel nearest the stock, and the parts they carry are marked (`PosedPart::outOfReach`).
`WorkholdingPose::outOfReach` says what the stock asks and what the holding reaches, in one measure: a diameter for jaws, a
step, a clamp ring or a collet, a width for a vise or a tenon. `reachMessage` says it in the user's unit, naming no members,
travel or millimetres when the user works in inches: "the jaws can't reach Ø3.163 in: on step 'Step 1' they grip
Ø0.996–1.343 in". For a jaw set the range is what the step grips over master jaw 1's travel.

- **Shown** (`shownWorkholdingPose`): the 3D view and the placement dialog draw it so, the parts that do not reach in red, and
  the setup page says it once, in its workholding section.
- **Used** (`setupWorkholdingPose`): generation, posting and the simulation refuse it with that message, in the project's
  display unit, as before (the message was longer, in mm, naming the member and its travel).

(The first version also placed jaws at numbered positions out along the master jaws, worked out to keep them in the middle of
their stroke, and turned a reversed jaw about its seat. The maintainer asked on 2026-11-07 for neither: the programmer
models the jaws where they go, and a jaw that goes on either way is two options.)

The work's held end sits against the step's stop face (or the Jaw face offset beyond it), on the axis. The held diameter is the stock's: its width for an outside
grip. For an inside grip it is the bore of the stock within the grip length at its held end (from the earlier setup's stock, or a
tube's bore), or the setup's typed opening. Stock with no bore there is refused, saying to set the opening.

### Soft jaws

A soft set is a blank. Its step is what it clamps a ring or plug by while it is bored. The bore diameter is the setup's, or the held
diameter plus the clearance (minus it for an inside grip, where the jaws are turned to a step on their outside). Its depth runs back
from the jaws' front.

- **While bored:** the jaws clamp the ring, `t_b` from the clamp diameter on the step.
- **Holding the work:** the bore's surface moves with the jaws, so they hold the work at `t_g = t_b + (D_held/2 − bore/2)/k`. With a
  clearance of 0 the jaws stand where they were bored. Either travel outside the master jaws' range is refused, as above.
- **The cut:** every jaw carries its cut in its own coordinates, the chuck's axis as it stood while bored. That is everything nearer
  the axis than the bore's radius (or farther, for a step), from the bore's floor out. `posedPartModel` takes it off the jaw's solid
  once (a boolean cut, `cad::subtracted`), for the toolpath clearance, the simulation's checks, the views and the WCS box. The work
  stops on the bore's floor.

### The boring operation

`turn_soft_jaws` reads the setup's jaws, so the jaws and the operation cannot disagree. `softJawBoring` gives the jaws' swept area
in the lathe plane, each jaw moved back to where it stood while bored, before and after the bore. Turning roughing (DL-0020) then
runs in it: inside for a bore, outside for a step, down to the bore's floor (the Z range), with its profile pass. Feeds and speeds
come from the operation's tool and preset like any turning operation's; the strategy adds none (ADR-0004). The generated toolpath
says what it bores and what to load first.

The simulation leaves it out: it cuts nothing of the work, and its contact with the jaws is the cut, so it is said, not checked.
Generation does not hold it clear of the jaws either.

### Older files

Workholding version 4 adds the jaw mount, the jaw set and `top_jaw`. A version 3 chuck reads unchanged. It has no mount, so its own
jaws are its only set and its setups hold the stock exactly as before. Projects gain the setup's jaw choices, written only when a
set is chosen. Files from the first draft of version 4 read: a mount's `pitch` and `positions` and a setup's `serration` and
`jaw_reversed` are ignored, and a set's single `seat`, `inward` and `steps` read as one option named "Normal".

### Each copy its own id

Every copy of a jaw set's part has an id of its own (the part's, its last byte changed by the master jaw's number). Views keep
their models by id, and drew one jaw for all three while the copies shared the part's id.

### The setup page

The setup page folds into sections, the usual ones first and open: the setup itself (name, model, machine), WCS, Stock and
Workholding. Jaws (titled with the set), Tailstock and supports, the Turret's tools and blocks, Fixtures and Post start folded. A
section stays as it was left while the program runs.

- **Plain words:** Jaw position is Auto (closed on the stock) or Manual (an opening typed). Part gripping is OD or ID. A step
  left to choose itself is Auto.
- **One list for the jaws:** each way a set goes on is its own entry ("HB10A1 (OD)", "HB10A1 (Reversed, ID)"); a set with one
  way shows its name alone.
- **One message:** why the workholding cannot hold the stock is said once, in the workholding section; the jaws' section says
  only what they hold when they can.
- **Set in a dialog:** the chuck, its jaw set and how they hold the stock (Jaw position, First jaw along, Part gripping, Step,
  Jaw face offset, the soft jaws' bore) are set in the workholding dialog (DL-0078), which also updates the project's copies
  from the library. The page names them and has no Jaws section.
- **The library's differs:** when the library's copy of the setup's chuck or jaw set differs from the project's (it was changed
  after the project took it), the page says so. (A user's project kept a chuck copied in before the library's master jaw travel
  was changed: its jaws could not reach a bar the library's could.)

A turret tool no operation uses is put in with "Put another tool in the turret" and shown as "not used here": it is drawn and checked for collisions as
the turret moves.

## Sources

No sources beyond DL-0020 (turning roughing), DL-0035 (workholding templates) and DL-0068 (mount names): the jaw placement is rigid
transforms from datums, and the bore is a region the roughing works in. Interfaces and jaw shapes are the user's: nothing here
carries a maker's catalogue.

## Correctness and tests

- `WorkholdingTest`, "jaw mounts and jaw sets: JSON and validation": round trips (a set with two options), what is wrong said
  (options with one name), a version 3 chuck without them, the first draft's mount, one-way set and setup read, the
  setup's choices in a project and an option the set lacks, a used jaw set kept.
- "Jaw sets on a chuck's master jaws" (on the test chuck, a jaw set made of one of its jaws):
  - a hard set grips the stock seated as modelled, by its first option: the grip face's radius from the travel, the three copies
    (each its own id) on the stock's surface, the held end on the stop face; stock the master jaws cannot reach is refused
    briefly, and shown with master jaw 1 at the end of its travel and the three jaws marked, said in inches too;
  - the reversed option's step grips an inside diameter (and refuses an outside one), a bore out of reach is refused, and an
    option the set lacks is said;
  - an inside grip on solid stock is refused;
  - soft jaws: the bore from the held diameter and clearance, the clamp travel, the closing travel, each jaw's cut volume (the
    segment of the bore beyond the jaw's flat face, 12 deep), the cut made once;
  - the boring operation down to the floor where there is jaw (the nose centre's), and the simulation leaving it out;
  - no set: the chuck's own jaws (an older setup's; the workholding dialog no longer offers it, DL-0078).
- "Step jaws turned either way on the master jaws" (a placeholder jaw laid out as a user's was: steps rising outwards, seated at
  its middle, its back flat): as modelled, step 1 grips the bar with its end on the stop, and step 2 cannot (said, shown); Auto
  takes step 2 for a bar step 1 cannot reach; turned round, its back grips the bar and its steps a bore; an entry with no step
  for the way asked is refused, and shown with its jaws marked; the stock stood 12 out from the stop still gripped, the stop 12
  down it, and a negative distance refused (by validation and by the pose). The setup's choices round trip with Out from stop.
- The vise, self-centring vise, dovetail, chuck and collet tests: stock beyond their reach refused with the short message; the
  vise posed as far as it goes, its moving jaw at the end of its travel and marked.
- `SubShapeGeometryTest` and the strategies' count follow the rest.
- `gui.jaws_setup` (VM): see DL-0078; it chooses the hard set's entries, takes a Jaw face offset of 5 mm and an opening of
  200 mm ("The jaws can't reach Ø200.000 mm: on step 'Inner' they grip Ø9.000–135.000 mm.", said once, the jaws drawn in red
  as far as they go), bores the soft set and adds the boring operation.
- `gui.workholding_jaws` (VM): the library's Jaws section reads a jaw set with two options, a soft one and a chuck's mount and top
  jaws and gives them back; it shows the second option, adds a third as its copy, and unticks a top jaw.

## Known limitations

- **Every master jaw is the same:** the other master jaws' seats are master jaw 1's turned about the axis. A chuck whose master jaws
  differ is not modelled.
- **The stroke is master jaw 1's.** Linked master jaws are assumed to follow at a ratio of 1.
- **Soft jaws are bored to one diameter and depth:** one bore, square at its floor. No stepped bores or chamfers. The jaws are cut
  where they stood while bored and moved as a whole to the work, so a clearance shows as the jaws standing nearer or farther, not as
  a gap at the bore.
- **The boring operation is not simulated:** neither against the jaws it cuts nor against the ring it clamps. It runs first in the
  setup's program, so the ring must be loaded before it.
- **Hard jaws' steps are datums:** what a step grips and where the work stops come from them, not from the jaw's shape.
- **The jaw mount takes one interface.** A chuck that takes two kinds of jaw is not modelled.
- **Library sets are copied in:** a set from the library is copied into the project when chosen, as workholding is. A set no
  setup uses leaves the project.
- **The master jaws' stroke and the top jaws' place on the serration are one travel.** A library chuck whose master jaw travel
  spans its whole gripping range moves the master jaws' own parts with it unless they are modelled on the body.
