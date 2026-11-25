# DL-0068: Tooling blocks and tailstock centres

- **Status:** implemented
- **Module:** libs/document (`LatheFitting`, `Setup::turretBlocks`, `LatheTailstock::centre`, `turretLoad`, `latheSupports`, `oxcam.fitting`
  files, `data/fittings`), libs/machine (`LatheTurret::blockRadius`, `blockAlong`, machine file version 9), libs/sim (`TurretBlock` in the
  machine check), apps/opencam-gui (`LatheFittingsDialog`, the setup page's stations and centre, the machine library's Stations and block seat,
  `SimulationPanel`)
- **Patent gate:** low (citation only). Rigid transforms from picked points, on top of DL-0067's check. Collisions are reported, never avoided. Not reviewed.
- **Author(s):** OxCAM contributors

## Problem

The maintainer, looking at DL-0067 in the app: the turret needs setting up. How many stations it has, so the simulation knows where tools go; what
kind of tooling block is on each position, with models, a turret mount point and a tool mount point; and the model of the tailstock's support, such as a live centre.

**What was wrong in DL-0067.** The stations were already evenly spaced, but their number was the tool changer's *capacity*, in another box of the machine's editor, and nothing
said it was the turret's. More to the point, **nothing held a tool.** Its tip stood at one number from the index line (`radialReach`, `axialPitch`), whatever the tool. The
shank ran into a plain disc, solid through solid, and a tip could only be moved by typing a tool setter's numbers. So the check knew where tips were on a machine with no blocks, not where a real
tool stands, and a block, which is often what actually hits things, was not there to hit anything.

## Approach

### Where each thing belongs

| Thing | Where | Why |
|---|---|---|
| How many stations | The machine (its tool capacity, now also **Stations** in the turret's box) | One fact, one place: the T word's range and the turret's |
| Where a block seats on a station | The machine: `block_radius`, `block_along` (a file before version 9 reads half the turret's diameter and 0) | Every station the same, turned about the index line |
| A block or centre's model and points | A library of the user's (`<app data>/fittings`), copied into a project that uses it | As workholding (DL-0035), so a project opens anywhere |
| Which block is on which station, which centre is fitted | The setup | They change from job to job, as the workholding does |
| How far a tool sticks out | The block's default; a setup can type its own for a tool | The tool setter's number for this job |

### A fitting

One type for both, `LatheFitting`: a STEP file, as machine parts and workholding keep (their meshes derived on load), and two points. Each is a point with a direction, **picked on the
exact B-rep or typed, or both**: a pick fills the numbers, and the numbers can be edited after.

| | Block | Centre |
|---|---|---|
| `seat` | Turret mount: on its seat, the direction out of the turret | Where it sits in the quill, the direction towards the tip |
| `mount` | Tool mount: on the tool's centre line at the block's mouth, **at the height of the tool's tip**, the direction towards the tip | Its tip |
| What else | `overhang`, what it takes (`takes`, `square_shank`, `round_bar`), `seatTurn`, `mount_type` | `mount_type`: what it fits in the quill |

A pick reads what was clicked (`cad::subShapeGeometry`). A flat face gives its centre and normal: a seat's direction goes into the fitting, a mouth's out of it. A round face or a circular edge
gives its axis, the sense away from the fitting's middle, and **Reverse** turns it. A vertex gives only the point.

**The block's orientation comes from its points.** The tool's direction along the seat's means a tool **across** the spindle (radial); square to it means a tool **along** the spindle (axial). Anything else, or one pointing
into the turret, is refused by name. Which way round the block goes on the seat is what `towardsWork` works out: the tool's direction for a tool along; else from the seat towards the tool
mount (the side a shank lies on); else the block's X. `seatTurn` turns it when the points do not say. So nothing has to be modelled in a particular orientation.

**What it takes** is said, not read from a size: square shanks (a turning holder's), round ones (a boring bar's, a drill's, a mill's) or both. Each size it takes is either stated or not; **a 0 never means "any"** (fitting file version 2; version 1's `shank` and `bore` of 0 become sizes not stated, and a block with neither becomes *both*). A tool with the other kind of shank, or a stated size that differs, is **reported, not refused**: sleeves exist. A plain stick says neither and is not compared.

**How it goes on** is a name, `mount_type`: a block's kind of mount ("VDI 40", "BMT 55", a maker's own), a centre's interface with the quill ("MT 4"). The machine says which kinds its turret takes (`block_mounts`) and what its quill takes (`quill_interface`), machine file version 10. Names compare without regard to case or spacing. The setup offers only what fits; a project whose block or centre does not fit opens, uses it as it is, and says so. A machine that names nothing takes anything, and a fitting that names nothing fits only such a machine. Names, not a list we keep: the standards and makers' systems are many, and a name is all a check needs.

### Station to block to tool

`blockOnStation` puts a block's seat on the station's seat point in the working position, its seat direction towards the spindle, `towardsWork` towards the work. The tip is then:

    tip = block(mount point) + block(mount direction) * overhang - the tool's centre line

The centre line is what lies between a tool's tip and its axis (the shank's middle in Z for a tool across, the bar's axis in X for one along), so a tool's own geometry decides where its tip lands, not just the block's. The tool's own orientation stays what the cut needs: its frame is the WCS's, and a block never rolls it. A
block only says across or along, and a tool that lies the other way is reported.

**A tip off the centre height** (a tool mount not at the tip's height) is reported on a machine with no Y to take it up.

**Stations with no block behave as before**, by the same numbers. A setup's typed tip still wins over a block, since it is what the tool setter measured. Existing projects have no `turret_blocks`, and nothing changes for them.

### Blocks on the turret

A block is a mesh on T, turned back by its station's angle as the tools are. So it rides along, is drawn, and is checked: against the chuck by triangles against the swept area, against the enclosure by its box (as tools are, DL-0067, so one that has gone through a wall still counts), against the rest as a mesh. Blocks are checked at
every pose, the working station's too: unlike its tool, a block is not cutting. Not checked: tool against its own block, since they are on one body, which is what the sizes are for.

### The tailstock's centre

With a centre fitted, `latheSupports` gives its model, moved so its tip is at the setup's point on the axis pointing at the work, and the quill from where it seats. They go on as DL-0052's bodies did: generating keeps clear of them, the view draws them, the machine check has them on W. A plain cone stays the default.
Only the model's swept outline counts in the checks, so a live centre's nose turning with the work is checked as what it sweeps.

### Found on a real machine (2026-11-06)

Setting up a Wasino LJ-63M with these (its V12 drum has six turning and six milling stations whose faces are shifted sideways, and a chuck the user has as a STEP model) found what the first version could not say. Machine file version 11.

- **Stations that differ.** `station_mounts` lists stations with the kinds of block they take; a station in none takes the turret's `block_mounts`. `mountMismatch(fitting, machine, station)` checks one station; with no station, a block fits when some station takes it. The setup page's combo for each station offers what that station takes.
- **A seat sideways.** On the LJ-63M each station face is shifted 75 sideways and the turret's centre stands 75 above the spindle's plane, so a holder's centre is on the spindle's plane, not on the line through the turret's centre. `block_across` is the seat's offset along the working station's Y, turned with the station like the rest: the seat is at the centre + (-`block_radius`, `block_across`, `block_along`), and a tool without a block stands as far sideways (`tipPoint`).
- **Where workholding mounts.** A chuck is workholding, as it already is for setups that bring one: the machine only says where it seats, `workholding_mount` (Z from the nose, on the spindle's axis, turning with C: a back plate's front). A setup's chuck seats its back there, so another chuck is another workholding choice; the machine's own chuck, for setups without one, stands there too. A chuck in the machine's model as parts is not supported: it would be in every setup, whatever holds the work.

**Picking a bore** gave the middle of its round face, the bore's middle; a tool mount wants its mouth. A round face now gives the centre of one of its ends (`SubShapeGeometry::axisStart` and `axisEnd`), the one nearest where it was clicked, and says which (*the outer end of a round face* or *the inner end*). Picked by index (the tests), it takes the end the direction points to, away from the fitting's middle.

### Built-in examples

Two blocks and a live centre (`data/fittings`, written by `oxcam-fixture-gen --fitting-models` and kept in the repository so their hashes hold). Plain shapes of this project's own, **all sizes placeholders**, marked so in their names and notes. No maker's model or catalogue value is in them.

The built-in generic lathes state a seat of 150 and -30, and their placeholder enclosure's far X went from 640 to 700: a block on the opposite station stood through the back wall at home, and the check was right to say so. That is the one change an existing project can see, and only on those two generic machines.

### Editors

- **Machine library:** *Stations* (the same number as Tool capacity), *Block seat from the index line*, *across* and *along Z*, marked per station in the preview; *Blocks it takes* (names, with commas), *Stations that differ* (a table: stations, the kinds they take), the tailstock's *Quill takes*, and the spindle's *Workholding mount*.
- **Tools > Turret Blocks and Tailstock Centres:** new from a STEP file, duplicate, delete; the two points picked or typed; *Mounts by* (or *Fits a quill taking*), with suggested names to edit; *Takes* square shanks, round ones or both, each size with *States its size*.
- **The preview says what it shows**, in the view, not only in a legend (the maintainer found the bare lines confusing): *Turret mount* and *out of the turret* on the seat and its line, the *Turret face* it sits on, *Tool mount* and *towards the tip*, and *towards the work*. **An example tool** sits in a block that works: a square shank with an insert across, a bar with one along, of the size the block states or a placeholder said to be one. It is a picture, made from the block's points (`exampleTool`), not a tool of the project. *Show it on a lathe* puts the block on station 1 of the generic lathe's turret in the working position, moved so the example tool is at the work's end, or a centre in the work's centre hole, so which way round the points put it can be seen. Picking turns that off: points are picked on the fitting alone, and the example tool is never picked.
- **Setup page:** a row per station, choosing its block (from the project or the library, as workholding does), with the tool on it, its overhang and where its tip comes out. A *Centre fitted* under the tailstock. Both offer what the machine takes; a block or centre already chosen that does not fit stays, marked *does not fit this machine*, and a line says how many more were left out and what the machine takes. Each station offers what it takes.

  **The station's tool is chosen on its row** (2026-11-07). The first version listed each tool under its station with a box holding its number, so a block was chosen on one line and its tool on another, and a station with a block and no tool had nothing to choose a tool with. The maintainer, setting up the Wasino LJ-63M: "For a tool that is already assigned a tool number, I can't seem to select a block for it to use. When selecting a block on a different station, I then can't seem to assign a tool to it." Each station's row now has *Block on it* and *Tool in it*. Choosing a tool gives it the station's number, everywhere in the project, as the number box did; a tool that had that number takes the chosen tool's old one. A tool no operation of the setup uses rides along, and *(no tool)* takes it out; a tool an operation uses can go to another station but not out. *Reset* shows under a tool whose tip or lie the setup has typed. On that machine the odd stations took milling units only at first (`station_mounts`); the maintainer: "odd stations are the only ones that take milling tools, but can take any bolt on tool holder as well", so the machine file's odd stations take both. A station's block list says what it takes.

  **Whole names in the lists** (2026-11-08). The maintainer: the tool and block names "are too long and get truncated, making it near impossible to figure out what's what until you click it or make the panel wider. Especially true if they say "Library" first." The open list now takes the width of its widest entry (`showEntriesInFull`), however narrow the box, and a box too narrow for its choice shows its start, cut at the end (the application's style), never in the middle. Where an entry comes from is a heading over its group (*In your library*, *In this project*, *Built in*) instead of words before each name; each entry's tooltip has its whole name and where it comes from, and the note on blocks a station does not take is short, with what the station takes in its tooltip. The same goes for the setup's machine, centre and post lists, and the post dialog's.


  **A machine changed in the library** (2026-11-08). The odd stations were given the bolt-on mount in the machine library, and the maintainer still could not put a turning holder on them: "looks like I still can't put turning holders on odd numbered stations". The project keeps its own copy of the machine (so it opens anywhere), taken before the change, and the setup page left the library's machine of the same id out of its list, so nothing could bring the change in. The maintainer: "can we version the machine definitions internally so if a project is using an old one, we can present a button next to the machine selection dropdown in the setup props to update it?" Machine files (version 13) have a `revision`: the machine library counts it, one more each time a machine is saved changed (saved unchanged, it stays; a file without one is revision 0). A project's copy keeps the revision it was taken at. When the library's machine of the same id is of a higher revision than the copy the setup uses, or of the same one with other contents (its file changed outside the library, as the odd stations were), *Update* stands beside the machine list with a line saying the two revisions; it replaces the project's copy, for every setup of the project using it, in one undo step. A copy of a higher revision than the library's (a project from another library) is left, with nothing said.
- **Turret preview** (2026-11-08). The maintainer: "would be nice to be able to see a preview of the turret and machine while setting up tools and blocks. Easier to visualize the layout to prevent collisions that may occur due to tool placement". *Turret preview* in the setup page's turret section opens a view docked beside the page. It shows the setup's machine, the blocks and tools on every station and the setup's stock and chuck, placed by the same code as the simulation: `latheMachineParts` (the machine's parts, blocks and tools at T 0, carried by the turret), `turretTools` and `latheWorkBodies`, taken out of `simulateLatheMachine` and the simulation panel so both use them. The station whose row is chosen in the table (its number is a button, and choosing a block or tool on a row chooses it too) is highlighted and turned to the working position, and the preview's own *Station* box chooses the row back. *At the work* puts that station's tool tip on the front of the stock at its largest diameter (a tool along the spindle, at its centre), by `latheAxes` as the simulation moves the axes; *At home* leaves the axes at machine zero. An axis beyond its travel is said. *Whole machine* off leaves the turret, the spindle and the work. It follows the project as it changes, and draws nothing while hidden.

  **The turret in a window of its own** (2026-11-08). The maintainer: "it may be best to move the turret setup into it's own window so the preview isn't jammed along side the main window and can work a little more smoothly." The setup page's turret section now sums up what is on the stations (a line a station: its block, its tool), says the turret's problems, and has *Turret setup...*, which opens `TurretWindow`: the stations' table (a `PropertyPanel` that shows only the turret, `setTurretOnly`) beside the preview, in a window that follows the project as it changes and closes when its setup goes. The dock beside the page is gone.

  **The tool in the preview, and which side of its block it is on** (2026-11-08). The maintainer, looking at the Wasino's OD holder: "On the OD turning holders, it's putting the tool on the wrong side of the holder. That bottom area isn't flat, it's where the wedge goes that gets tightened which holds the tool against the top side of the holder that is actually flat." The block's points were right: its tool mount is at the rake face's height, and its shank lies on the side of the mount that `blockUp` (the seat's direction across towards the work) points away from, as the block editor's example tool always drew it. On a turret that is the station's Y, which looks down the work's Y: a rear turret holds its tools face down. The simulation drew every tool face up in the work's frame, so in a block its shank stood on the wedge's side. A tool in a block now follows it (`TurretStation::faceDown`): its solids turned over across the tip's height, its outline in the lathe plane unchanged, in the preview, the simulation and the machine check alike. Also: "the tool would be a lot easier to see if it was colored differently. Maybe black for the tool, and gold for the insert. Leave the holder blue." A tool's body is black and what cuts gold, in the simulation too, and the preview highlights the chosen station's block only, so its tool is seen in it.

  **Stickout** (2026-11-08). The box on a tool's row in a block was labelled with how the tool lies (*Across*, *Along*), which read as the box's meaning. The maintainer: "I assume that's supposed to be tool stickout? Is that value from the tip of the tool to the face of the holder or what?" It is: how far the tip stands beyond the block's tool mount, along the tool; on a turning holder whose mount is at its mouth, from the face the shank comes out of to the tip. It is labelled *Stickout*, and its tooltip says so and how the block holds the tool.

  **Reopened, flickering, and a bore that is a largest size** (2026-11-08). Three more from the maintainer.
  - "Reopening the turret setup window seems to have broken the turret preview": shown again, the 3D view was a white box and the notes under it were painted over each other. Closing a window destroys its native windows; Qt gives the view a new one when it is shown again, and the view keeps drawing into the one it was made for (each reopening logged another handle). The window now hides when closed (`TurretWindow::closeEvent`), so its view keeps its window, and `OcctView::drawsIntoItsWindow` lets a test say so. The `wglMakeCurrent() has failed. The handle is invalid.` printed at exit after a second 3D view is likely the same thing in a dialog closed before its view goes.
  - "Z fighting on stick tools and inserts": the insert's top lay on the holder's and its sides on the pocket's. A turning tool's body now stands 0.05 mm in from its outline all round (`kBodyInset`), so the insert's faces are in front where they meet; the insert, what cuts, is as it was.
  - "Getting an error/warning about a drill being used on an axial milling unit saying it takes a 20.0mm. These milling units are basically just collet holders. Same with the boring tool holders. So those are more like maximum tool sizes, not required." A block's round size is its bore, the largest round shank it takes: a smaller one goes in through a collet or sleeve, only a larger one is a problem. A note saying so came first, and the maintainer had it removed: "Remove this message." A square slot still takes its own size. The block editor calls it *Round bore*.

## Measurements

`test_document "[lathemachinebench]"`, ci build, one run of a setup with its machine loaded, best of five:

| Setup | Simulate |
|---|---|
| No layout | 4.9 ms |
| Turret, 0 tools | 12.4 ms |
| 1 tool | 12.5 ms |
| 4 tools | 13.8 ms |
| 11 tools | 16.4 ms |
| A block on all 12 stations | 12.5 ms (3 reported) |

A block costs about what a tool does or less. Playback of `turret_blocks_project` for 3 s from 0:20 gave 151 frames in the off-screen build.

## Sources

No sources beyond DL-0037 and DL-0067: rigid transforms from two points, and the same checks.

## Correctness and tests

- `LatheMachineTest`, "tools mount through the block on their station": no blocks as before; a block across (seat on the seat point, the tip by the block's 60 and the overhang, its X towards the work, a shank's top at the tip height); a setup's own overhang and typed tip; a block along (pitch and reach; a size it does not take); a tool lying the other way; a block's collisions with the chuck, reported by station; a station beyond the turret.
- "The built-in project": clean, the blocks and centre back from the file, tips where the blocks put them, the centre's tip at the setup's point within a swept step, the tailstock's body behind the quill behind the centre, the cache.
- "A fitting's file, library and validation", "a fitting in the project and removal": round trips, a changed model is said, the sentences for what does not work, the built-ins valid and marked as placeholders, keys.
- `TurretTest`: the block seat in the machine file, and a version 8 file reading without it; the blocks a turret takes and what its quill takes, a version 9 file without them, names compared without case or spacing.
- "A fitting's file": version 1's sizes become *takes* and stated sizes (0 is not stated); a size stated must be more than 0; the built-ins name a placeholder mount. "Tools mount through the block": a round shank in a block for square ones is said, a size not stated is not compared; a block or centre the machine does not take is said and still used, a machine that names none takes anything, a block that names none on a machine that does is said. "The example tool in a block": out of the mouth by the overhang, at the tip's height, its shank short of the turret face; the bar along the tool mount.
- `FixturePostTest` posts and verifies `turret_blocks_project`: a block moves the machine, not the program.
- `gui.fittings_edit`: in a throwaway library, a new fitting from `axial-block.step`; picks on a flat face (seat into the fitting, mount out of it), a circular edge (the axis, away from the middle) and a vertex (the direction left alone); **a click on the preview**, through the Pick button and mouse events on the view; save, and the same numbers back from the folder.
- `gui.fittings_edit` also: picking turns the lathe view off; *Mounts by* typed with odd spacing, *Takes* both and a square size stated, saved and read back. `gui.fittings_block_on_lathe`, `gui.fittings_bar_block`, `gui.fittings_centre_on_lathe` save the views. **Screenshots from the VM's test run can show another test's window:** its runner puts six tests on one Xvfb display, and a capture reads what is on the screen there. Look at them taken one at a time.
- "The kinds of block a station takes", "a turret whose stations stand sideways" and, in "a setup's own chuck", the workholding mount (`LatheMachineTest`): per station and on no station; a seat and tips on the spindle's plane from a turret centre off it; a setup's chuck with its back on the mount, the machine's chuck and the work in it moved by it. `TurretTest`: version 11's fields round trip, what is wrong with them said, a version 10 file without them. `SubShapeGeometryTest`: a hole's two ends.
- `gui.fittings_edit` also picks the bore of `axial-block.step`: by index, its outer end; with a point near the other end, that one.
- `gui.lathe_machine_turret_edit` chooses a block and a centre from the library in the setup page's widgets and reads them back from the project. Then, on a copy of the machine whose station 2 takes only "Unit": station 2 offers the block that mounts by "Unit" and no library block, says what it takes, and takes the block; station 1 does not offer it. `gui.lathe_machine_station_tools`, on a copy of the machine whose odd stations take only "Unit", clicks in the lists' pop-ups as a user does: station 1 shows its turning tool and no OD holder, saying what it takes; the turning tool is chosen on station 2 and the OD holder under it, and it sits in it; a milling unit on station 3 is given the drill; the turning tool chosen on station 3 swaps numbers with the drill; *(no tool)* does not take out a tool an operation uses; each choice is one undo step. `gui.fittings_library` shows the dialog, `gui.lathe_machine_blocks` plays the fixture.
- `gui.lathe_machine_station_tools` also checks a long library name in a narrow station list: the library's blocks under a disabled *In your library* heading with no prefix, each tooltip naming the block and where it is from, the left-out note's detail in its tooltip, and the open list wider than the name while the box is narrower. It saves the open list (`_list`).
- `LatheMachineTest` (*tools mount through the block*): a tool in a block on the generic turret is face down, its shank above the tip's height. A 12 mm drill in a 32 mm bore is not a problem; in a 10 mm one it is.
- `gui.setup_machine_list` also follows the revisions of the machine the setup uses: the first save is revision 1 and saving it unchanged keeps it; changed in the library, *Update* shows beside the machine list with both revisions said, and makes the project's copy the library's (revision 2) in its own undo step; a file changed outside the library at the same revision offers it too, and a library copy of a lower revision does not. `ModelTest` checks the revision in the file.
- `gui.turret_preview` checks the setup page's summary, opens the turret window from it and chooses a row in its table: that station's block highlighted, its tool not (black, what cuts gold) and nothing of the other station's, the turret at the station's angle, its tool's tip on the work's front edge within 0.001 mm. Choosing another row moves the preview; choosing a station in the preview marks its row; *At home* puts the axes at zero with the turret still on the station; *Whole machine* off keeps the block and the stock and fewer parts; a block put on a station in the project shows on it. It saves the preview's view (`_preview`). Then it closes the window and opens it again three times, on other stations: the view keeps its native window and draws each time, with one note under it.

## Known limitations

- **One seat per machine.** Every station has the same seat turned about the index line, so a turret with staggered stations, or a second row, is not modelled.
- **Blocks are chosen per setup.** There is no machine-level set of blocks, so a turret that stays set up needs choosing again in each new setup.
- **Tool against its own block** is not checked. Only sizes are compared, and only where the tool states one (ISO holders, rotating tools; not plain sticks or solid bars).
- **A block rolled about the tool's axis** has no effect on the tool: it faces as the cut needs.
- **Sub-spindle:** a block is turned to face the work on the sub-spindle in that setup, as if fitted for it. Two blocks on one station are not modelled: a block that carries another is one block.
- **The generic lathes' chuck** is still the layout's cylinder for setups without workholding of their own. Moving it into workholding needs a built-in workholding library, which there is not yet.
- **Live centre's bearing:** one solid. Nothing separates the turning nose from the fixed body.
- **Numbers shown for a typed tip with a block:** the tip wins silently; Reset goes back to the block.
- **Mount names are the user's.** Nothing knows that "VDI 40" and "VDI40" are a standard, or that a sleeve makes a block take a smaller bar: a name either matches or it does not, and sizes are reported, not refused.
- **The example tool** is drawn from the block's points alone: a square shank below the tip's height towards the work's side, a bar on the tool mount's axis. A block whose points are wrong shows a wrong tool, which is what it is for, but it is not checked against anything.
- **The turret preview does not check collisions.** It shows where things are; the simulation still checks them. It shows one position, not the moves between tools, and the work as the stock, not as the part.
- **The preview's station follows the table, not the program.** Choosing an operation does not choose its tool's station.
- **A click is tested at the preview's middle, by Qt mouse events on the Debian VM** (`gui.fittings_edit`). No person has used it with a real mouse, and Windows has only seen off-screen screenshots.
