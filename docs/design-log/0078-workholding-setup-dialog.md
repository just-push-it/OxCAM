# DL-0078: Setting up a setup's workholding in its own dialog

- **Status:** implemented
- **Module:** apps/opencam-gui (`WorkholdingSetupDialog`, `MainWindow::setUpWorkholding`, the setup page's workholding and jaw
  rows), libs/document (`Document::takeWorkholding`, `Document::updateWorkholding`)
- **Patent gate:** low (citation only): lists of the project's and the library's templates, and a view. Nothing here makes a
  toolpath or changes the output.
- **Author(s):** OxCAM contributors

## Problem

The maintainer asked on 2026-11-07: "Would be great if when selecting jaws or workholding, it was it's own popup, maybe just a
general workholding setup workflow so it's easy to see and we can manage what is copied into the project". The setup page chose
the workholding and the jaw set from two combo boxes mixing the project's copies and the library's ("Library (changed): ..."),
with an Update from library and a Save Workholding to Library button below. What was copied into the project, which setups used
it and whether the library's copy differed could not be seen in one place, and the stock could not be seen held while choosing.

## Approach

### The dialog

Set up... beside the workholding's name on the setup page opens it, modal, titled with the setup. It has three lists, the form
for how the stock is held, and a view:

- **Workholding:** (none), the project's templates, then the library's the project has not. Each row says its kind (Vise,
  Dovetail, Fixture, Chuck with the interface its jaw mount takes) and where it is from: "This project", "This project (the
  library's differs)", "This project only" or "Library". Choosing a library one copies it in (`Document::useWorkholding`), as
  the page's list did. Jaws that do not go on the chuck chosen come off it.
- **Jaw Selection:** shown when the chosen chuck has a jaw mount. Each way every jaw set for its interface goes on (DL-0069),
  the project's then the library's, with the same columns; how many more there are for other interfaces is said under it, and
  that none fits when none does. A chuck with a jaw mount holds the stock by a jaw set: choosing one puts the first set that
  fits on it (the project's, as it had them, then the library's), on its first way. A setup whose chuck has none on (an older
  project) is told to choose them, in the dialog and on the page.

  (The first version also offered the chuck's own top jaws, the parts its file marks as top jaws, held at its grip length. The
  maintainer asked on 2026-11-07 for it to go: a chuck's jaws are a jaw set. Those parts are still drawn, and still hold the
  stock, for a setup that has no set chosen, until one is.)
- **Holding the stock:** every field of how the workholding holds the stock: the holding, Jaw position and the opening, the grip
  length or depth, First jaw along (or Jaws close along), Shift along jaws, members' travel and, with a jaw set, Part gripping,
  Step, Jaw face offset (DL-0069) and the soft jaws' bore, with what the jaws come to under them.
- **Copied into this project:** every template the project holds, the setups that use it, and whether the library has it the
  same, differently, or not at all. Update from library replaces the project's copy with the library's, keeping what every setup
  chose (`Document::updateWorkholding`); Save to library puts the project's copy in the library (asking first when that replaces
  the library's). Manage library... opens the workholding library.
- **The view:** the setup's part, stock and workholding as the main view draws them (jaws out of reach in red), turned as the
  machine stands (DL-0077), with what holds it named and why it cannot hold the stock said under it.

The setup page's Workholding section names the workholding (with Set up...) and the jaws, says what is wrong or that the
library's copy differs, and offers the soft jaws' boring operation, which is an operation of the setup, not of its workholding.
It has no Jaws section and no workholding fields. (The first version kept the fields on the page; the maintainer asked on
2026-11-07 for all of them in the dialog, with its lists titled Workholding and Jaw Selection and their first column Name.)

### Apart until OK

The dialog edits a snapshot of the document (`Document::snapshot`), so choosing and updating are seen in its view without
touching the project. OK hands it back: `Document::takeWorkholding` takes the setup's workholding settings and every template the
snapshot has that the project has not (and uses) or has otherwise, with their files and models, as one undo step ("Set up
workholding"); templates no setup uses any more are dropped. Cancel leaves the project as it was. A save to the library is the
one thing done at once: the library is not the project.

## Sources

None: the dialog shows what DL-0035 and DL-0069 already define.

## Correctness and tests

- `WorkholdingTest`, "a setup's workholding set up apart and taken back": a jaw set chosen and the chuck's copy updated in a
  snapshot leave the document as it was; taken back, the setup's settings and the updated copy come with one undo step, an unused
  set is dropped, and undo restores both; updating a template the project lacks is refused.
- `gui.jaws_setup` (VM): on the chuck project, with no Jaws section on the page, the dialog chooses the jaw sets, finds Jaw
  position and First jaw along in it, says once why the reversed entry cannot grip outside, takes a Jaw face offset (no helper
  text) and an opening out of reach (said once); the project is unchanged until OK, and the page then names the set and has
  none of the fields. Soft jaws are bored in the dialog and their operation added from the page.
- `gui.workholding_setup` (VM): the dialog on the chuck project, a throwaway library holding the test chuck changed: the lists'
  titles, the lists' rows (no entry for the chuck's own top jaws), "choose the chuck's jaws" said before a set is chosen, the copy
  said to differ and updated, a jaw set chosen; the chuck chosen again from the library gets the library's set that fits; Cancel
  leaves the project unchanged, OK takes it in one undo step.

## Known limitations

- **A template only in the project and the library's are not compared part by part:** the copy differs or it does not.
- **The dialog is for one setup.** Copies other setups use can be updated from it, but their choices are made in their own.
- **A chuck file's own top jaws** are no choice of their own. Made into a jaw set in the library, they are one.
