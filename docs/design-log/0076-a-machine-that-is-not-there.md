# DL-0076: A machine that is not there, and a part across the rotary axis

- **Status:** implemented
- **Module:** libs/document (`findMachineFor`, `machineProblem`, `machineFor`, `lowerSetup`, `cachedToolpathInFrame`), libs/strategies (`rotaryPocket`), apps/opencam-gui (setup properties, post, placement)
- **Patent gate:** low (citation only). Refusing a setup that cannot run; nothing here chooses a toolpath. The check on the part's coverage is a plain count over the radial map of DL-0024.
- **Author(s):** OxCAM contributors

## Problem

A test named a machine that does not exist, with a wrapped pocket on a shaft along Z. It took about two hours, and then it passed for the wrong reason. Two things were at fault.

**An unknown machine became a 3-axis mill.** `machineFor` ended in `genericMill3()`. A setup naming a machine neither in the project nor built in generated, simulated and posted as a 3-axis mill, with a 3-axis post, and the setup's properties showed the 3-axis mill as chosen. Only `oxcam-cli project info` (`Project::validate`) said anything. In the spirit of ADR-0004, what reaches the output is what the user chose, or nothing.

**A wrapped pocket has no reason to run where it cannot be cut, or across the axis.** A mill with no rotary axis generated it anyway. And on a machine that has one, a part lying across the axis took as long.

## Approach

**The machine.** `machineFor` throws when the setup has none, naming the machine and where to choose one. Generating, simulating and posting all come through it, so they refuse. `lowerSetup` asks first and reports it among the program's errors, and that holds for a machine passed in (`--machine`), since the setup's toolpaths are generated for the setup's own. `cachedToolpathInFrame` asks before its cache: its key has the machine's id and not whether it is there.

`findMachineFor` is what can be asked for showing a setup, which has to go on working: it gives nullptr. The toolpath and simulation keys use it, as do `latheSupports` and the GUI's panels. The keys are computed for what is on screen, so they cannot be what refuses.

No machine named at all stays the generic 3-axis VMC, and `generic-3axis` another name for it, as both are written down (`Setup::machine`) and the setup's properties show it. Only a name that finds nothing is refused.

**A project that names one.** It opens as before, as `Document::open` never validated: nothing is lost, the id stays in the file, and it saves as it was. The setup's Machine box shows `Not found: <id>` in place of a machine, with the reason beneath it. Post Process and Place on Machine say why and stop, and the toolpaths and simulation show the refusal. Choosing a machine puts it right, including one from the user's library of that id.

This can only come from a hand-edited file, a project from a newer OxCAM, or the one day of October 2026 between setups having a machine (phase 2) and projects carrying their machines (phase 9). No built-in machine's id has been renamed or removed.

**No rotary axis.** `generateIn` refuses a wrapped pocket on a mill without one, naming the machine and where to choose another. Every other rotary machine goes on to `solve`, which decides whether the tool axis can be reached there.

**A part across the axis.** Measured on `wrapped.step`, diameter 40 and length 60, a 6 mm end mill, in a `ci-msvc-gui` build on 24 threads:

| The shaft on `generic-4axis-vmc` | Levels | A level's contours | A level's pocketing | Generate | Post |
|---|---|---|---|---|---|
| Along X, on the A axis | 4 | 0.3 s | 0.01 s | 1.7 s | 2.0 s |
| Along Z, standing on the axis | 59 | 4 s | 110 s | about 1.8 h, projected from its first levels; stopped at 12 of 59 | not reached |

So a wrapped pocket on a 4th axis is not slow, and the two hours were the second row: a part that was never around the axis. Its stock radius is the far corner of the part (64 mm for a 20 mm shaft) and everything below that is a recess. Each level is hundreds of slivers along the part's silhouette: 221 regions, giving about 900 moves. Nearly all of the time is in the pocket kernel (the timers went around the whole call, not inside it), and making it quicker would only get to a useless toolpath sooner. Levels at about 110 s each, with the machine otherwise quiet, put the whole on course for 1.8 hours, close to the two hours reported. It was not run to the end, as nothing in it needed the rest.

`rotaryPocket` refuses that, from the radial map, before any level: where the part's surface comes within the tool's radius of the axis (no wrapped floor can be cut there) and the rays hit the part in under 90% of the map over its length. It says how close, and to set the WCS with X along the part. With the file opened, that takes 0.4 s.

Both are needed, because each alone has legitimate parts:

- A shaft with a round or nearly pointed end comes to the axis but goes around it: every ray hits.
- A part held off the axis, such as a saddle on a riser, is missed by most rays but stays away from the axis.
- Holes are misses too, but well under 10% of a part, and the surface does not come to the axis at them.

A part on a lathe is not checked: the strategy works from what the turning left there, and is told so (`stockRadiusAt`).

**A surface at the axis.** A shaft coming to a true point on the axis passes the check above, rightly, and then took 191 GB of memory before it was stopped by hand, once more at 79 GB, starving every other process on the machine. Under the test programs' 8 GB cap it reached that in about 12 s. Its deepest level was the point itself, at a radius of 3.55e-15 mm. There `wrapToolpath` divides a move's length by the radius to split it: one 2 µm move became 791 million moves, one by one. The seam's extension overflowed `int` at the same radius.

A level with the tool and its way out (its radius, the wall allowance and two map cells) wider than a turn has no plane to unroll, whatever it is split into. So `rotaryPocket` stops at the first such level, and says from which radius in nothing is cut, as a warning on the toolpath: the simulation shows what is left there. Levels above it are as they were. This is 0.64 mm for a 6 mm tool on the default map, and it scales with the tool, so a groove with its floor inside the tool's radius but outside this still cuts. `wrapToolpath` itself now refuses a radius at or below its tolerance, so nothing else can reach that.

| A point on the axis, 6 mm tool | Peak commit | Generate |
|---|---|---|
| Before | 191 GB, stopped by hand | never finished |
| After, a 1 mm step down, the default map | 0.03 GB | about 27 s (108 s over four tips, 0.5 mm down to a point) |

## Correctness and tests

`GenerateTest` "a machine that does not exist is refused, not taken for another": generating, a toolpath kept from before the machine went, simulating, posting with another machine passed in, the keys and supports still answering, the project saved, reopened and put right. `ProjectTest`: `machineFor` and `machineProblem`, the name kept through JSON, no name still the 3-axis VMC. `LiveMillTest` "wrapping a pocket": 3-axis refused; on a 4th axis refused across, and on the A axis generated, radial about X, and posted with A. `MultiAxisTest` "refuses a part across the axis": across refused; a shaft with a 1 mm tip and a part held off the axis not; a true point and a 0.01 mm tip finish with a bounded toolpath and the warning, which the test programs' memory cap would end otherwise. "wrapping refuses a radius": `wrapToolpath` at 1e-9 mm.

## Known limitations

- **A shaft across the axis with the axis through it**, the WCS at the middle of its length: every ray hits, so it is not refused, and it is slow as above. It still shows progress and can be cancelled between levels.
- **What is left at a point:** the material inside the last level cut stays, and is only warned about. A smaller tool goes further in.
- **`multiaxis_finish` on a 3-axis mill** is not refused at generation. Its tool axis can be set to stay upright, so it is left to `solve`, which refuses at posting.
- **The GUI's refusals are not covered by a test here**, as the GUI label opens windows: the box, the dialogs and the panels were built, not clicked.
