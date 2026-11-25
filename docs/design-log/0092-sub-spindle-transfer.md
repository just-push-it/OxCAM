# DL-0092: The transfer to the sub-spindle

- **Status:** implemented
- **Module:** libs/machine (`LatheSubSpindle::minSpindle`, `maxSpindle`, the `Z2` axis, machine file
  version 16), libs/toolpath (`SpindleTransfer`, `Toolpath::transfer`), libs/strategies
  (`spindle_transfer`, `ParamKind::Feed`), libs/document (`transferOperation`, `transferTarget`,
  `transferInto`, `transferGrip`, `pickOffZ2`, `latheWorkMount`, `lowerSetup`, `postSetup`), libs/post
  (`NcOp::SubAxis`, `NcOp::SpindleSync`, `NcInstr::sub`, `NcOperation::subSpindle`,
  `PostManifest::subSpindle`, `oxcam:fanuc`), apps/opencam-gui
- **Patent gate:** low (citation only). Where two frames line up, and a fixed order of machine
  functions; no algorithm.
- **Author(s):** OxCAM contributors

## Problem

ADR-0007, accepted by the maintainer on 2026-11-09 ("Approve as written"): the part handed from the
main spindle to the sub-spindle as an operation, the second setup's stock from it, and both setups
posted as one program. This entry is how it was built; the ADR says what was decided.

## Approach

**The machine.** The sub-spindle gains its speed range (`min_rpm`, `max_rpm`), and Z2 its travel as an
axis `Z2` in the machine's `axes`, both optional (machine file version 16). The machine library
edits the speed range beside the sub-spindle's sizes; Z2 is a row of the axes table. The built-in
`generic-lathe-sub` has both, as placeholders like its other numbers.

**The operation.** `spindle_transfer`, *Sub-spindle transfer*, in the main-spindle setup, with
no tool. Its parameters are the ADR's; the Z2 feed is a new parameter kind, a feed (`ParamKind::Feed`,
mm/min in the operation, mm/s in code), since no preset holds it. Generating it (`transferToolpath`)
finds the setup it hands the part to (`transferTarget`: the first setup on the sub-spindle of the same
machine whose stock comes from this one) and refuses, naming what to set:

- no sub-spindle on the machine, no setup to take the part, or a setup on the sub-spindle handing over;
- a Z2 feed of 0, a speed of 0 while synchronised, or a speed outside either spindle's range;
- *Part off* with no part-off after it in the setup;
- a grip of 0 with no grip from the machine either;
- two setups that do not line up (one axis, Z turned round);
- Z2 outside its travel where it takes the part, comes up to it, or cuts.

**Where Z2 takes the part** (`pickOffZ2`): the sub-spindle setup's work mount, with the transfer's
grip in place of the machine's (`latheWorkMount`, `transferGrip`), and the main setup's mount, both
taken back to the project; Z2 is what puts the part at the same place in both. It is the computation
DL-0067's simulation already made for the sub-spindle setup's first step, now one function both use.
The operation page shows it: *Takes the part at* "Z2 -869.998 (grip 20.000 mm, the machine's)". Z2 grows
away from the main spindle, so the jaws come up from `pick + grip + clearance` at the feed.

**The toolpath** has no moves: `Toolpath::transfer` holds Z2's three positions, the feed, how the
spindles go, what follows and the dwell. Its JSON is `"transfer"`. Its toolpath key takes in both
setups and the machine.

**The stock.** The transfer cuts nothing. The part-off after it cuts the part free, as any part-off
does; the sub-spindle setup takes the piece with the part in it (DL-0044) and holds it by the
transfer's grip. The stub stays as the main setup's final stock, the next cycle's bar after a bar pull
(DL-0091).

**The simulation.** The sub-spindle setup's first steps are DL-0067's (in to take the part, back to
cut), now only when a transfer hands the part over; without one the part is loaded by hand and Z2 starts
where it cuts. Z2's moves are checked there, against the turret, the tools and the main chuck. The
main setup shows the transfer as an operation with no motion of its own.

**Posting.** `lowerSetup` lowers a setup with a transfer together with the setup it hands the part to,
as one program; lowering that setup alone is refused, naming the setup to post. A sub-spindle setup
with no transfer posts on its own. `post::lower` writes the transfer as:

1. the sub-spindle's chuck opens; Z2 rapids to where it comes up; Z2 feeds onto the part; its chuck
   closes; the dwell;
2. synchronisation on (at an angle when asked) and the speed, or the spindle stopped;
3. with *Open the main chuck*, the main chuck opens.

Before the first operation on the sub-spindle: the spindle stops, synchronisation off, and Z2 to where it
cuts. The operations there say `spindle: "sub"`, and so do their spindle instructions. A post must say
`subSpindle: true` in its manifest to be given such a program.

**The Fanuc base** writes it from options, each empty on `fanuc-generic-lathe` and refused while empty
(the builder's codes, as for live tools): selecting the sub-spindle and the main spindle again, the
sub-spindle's chuck, synchronisation on, at an angle (`{angle}`) and off, and the letter Z2 is written
with. The work on the sub-spindle has its Z running towards the main spindle; with `subZ` `mirror`
(the start) the post writes Z and arc centres negated and arcs turned the other way, and with `as_is`
leaves that to the control. Canned cycles on the sub-spindle are refused while the post mirrors.

## Measurements

The fixture (`turret_sub_project`): the spool's front turned, the transfer, a part-off 1 mm behind its
back, the back faced and turned on the sub-spindle. Z2 takes it at -869.998 with the machine's 20 mm
grip, 10 mm nearer the main spindle with a 30 mm grip.

## Sources

- Fanuc-style lathe programming manuals: G98 feed per minute, M-codes as the builder's (syntax facts
  only).
- ADR-0007, the maintainer's decision.

## Deviations from sources

None.

## Correctness and tests

- `document` `TransferTest` on `turret_sub_project`: the transfer's Z2s, feed and speed; the pick-off is
  the sub-spindle setup's first step; a grip of its own moves it; each refusal; both stopped; a
  part-off must follow unless the main chuck opens; no setup to take it; the main setup simulates and
  parts off into two pieces, the sub-spindle setup starts from the part's; the generic post asks for
  the codes, writes `tests/golden/posts/lathe-sub-transfer.nc` with them in the order above; the
  sub-spindle setup alone is refused, and posts alone when loaded by hand; a post without `subSpindle`
  is refused; the NC program's instructions and their JSON.
- `LatheMachineTest` "taken over by the sub-spindle": the steps, the line-up to 1e-9, and posting the
  sub-spindle setup alone refused.

## Known limitations

- **One part at a time:** the program works the front, hands over and works the back in turn (ADR-0007).
- **The main setup's simulation** does not move Z2: the sub-spindle holding the part during the
  part-off is not drawn there, and its moves are checked in the sub-spindle setup's simulation.
- **A tool change under synchronisation** stops the spindles, as every tool change does.
- **Canned cycles and live tools on the sub-spindle** are not written by the Fanuc base while it
  mirrors Z.
- **Ejecting the finished part** is the post's (ADR-0007).
