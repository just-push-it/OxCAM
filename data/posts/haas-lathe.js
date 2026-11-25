// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Haas-style lathe with live tooling: the generic Fanuc-style lathe, with Haas's codes for the
// driven tool and the C axis as its starting options: M133 P<rpm> forward, M135 stop, M154 C axis
// engage, M155 disengage, and M11 and M10 to open and close the chuck for a bar pull (DL-0091).
// They are Haas's own: other builders use other codes. Check them against your machine's manual,
// and set your own in the options if they differ. Holes are plain moves, not Haas's G81/G241 cycles
// (DL-0063); tapping is G95 and G195 (DL-0065); milling on the face is G112 (DL-0066).
// Sources: Haas Factory Outlet, "Live Tool for Lathe" training manual, September 2014, and the Haas
// lathe M-code reference (syntax facts only).

import { FanucLathe } from "oxcam:fanuc";

export const manifest = {
  id: "haas-lathe",
  name: "Haas-style lathe (diameter programming, live tooling M133/M154)",
  version: "1.0.0",
  engine: 1,
  machines: ["lathe2"],
  extension: "nc",
  arcs: { maxSweep: 180, planes: ["xy", "zx", "yz"], helical: true },
  cycles: ["tap", "tap_left"],
  multiAxis: "tcp",
  verify: "fanuc-lathe",
  workOffsets: [{ from: 1, to: 6, code: "G{}", start: 54, name: "Standard" }],
};

export const options = [
  { id: "liveToolOn", label: "Live tool on ({rpm} is the speed)", type: "text",
    default: "M133 P{rpm}" },
  { id: "liveToolOff", label: "Live tool off", type: "text", default: "M135" },
  { id: "cAxisOn", label: "C axis on", type: "text", default: "M154" },
  { id: "cAxisOff", label: "C axis off", type: "text", default: "M155" },
  // G95 and G195 start the tool from S themselves, at feed per revolution. Some radial live tools
  // turn the other way: Haas says G196 for a right-hand thread with those.
  { id: "liveTapAxial", label: "Live tap along Z (blocks, | between them)", type: "text",
    default: "S{rpm}|G18 G95 C{c} Z{z} R{r} F{lead}" },
  { id: "liveTapRadial", label: "Live tap across the part (blocks, | between them)", type: "text",
    default: "S{rpm}|G19 G195 X{x} F{lead}" },
  { id: "liveTapAxialLeft", label: "Live tap along Z, left hand (blocks, | between them)",
    type: "text", default: "S{rpm}|G18 G186 C{c} Z{z} R{r} F{lead}" },
  { id: "liveTapRadialLeft", label: "Live tap across the part, left hand (blocks, | between them)",
    type: "text", default: "S{rpm}|G19 G196 X{x} F{lead}" },
  { id: "tapLeft", label: "Left-hand tap cycle, on the spindle's axis", type: "text",
    default: "G184" },
  // G107 maps Y onto C: Y is a length along the cylinder, in G19, with Y0 at C0.
  { id: "wrapOn", label: "Cylindrical interpolation on (blocks, | between them; {r} radius, {d} diameter)",
    type: "text", default: "G19|G107 Y0 C0 R{r}" },
  { id: "wrapOff", label: "Cylindrical interpolation off (blocks, | between them)", type: "text",
    default: "G107|G18" },
  { id: "wrapLetter", label: "Letter for the way around, in cylindrical interpolation", type: "text",
    default: "Y" },
  { id: "wrapDegrees", label: "That letter is in degrees (otherwise a length on the cylinder)",
    type: "bool", default: false },
  // In Haas's G112, X and Y are radial values, in G17. A Haas in Fanuc mode takes G12.1 instead.
  { id: "polarOn", label: "Polar interpolation on (blocks, | between them)", type: "text",
    default: "G17|G112" },
  { id: "polarOff", label: "Polar interpolation off (blocks, | between them)", type: "text",
    default: "G113|G18" },
  { id: "polarY", label: "Letter for Y in polar interpolation", type: "text", default: "Y" },
  { id: "polarXDiameter", label: "X is a diameter in polar interpolation", type: "bool",
    default: false },
  // Bar work (DL-0091).
  { id: "chuckOpen", label: "Chuck open, for a bar pull (blocks, | between them)", type: "text",
    default: "M11" },
  { id: "chuckClose", label: "Chuck close, for a bar pull (blocks, | between them)", type: "text",
    default: "M10" },
  { id: "barFeederOn", type: "text", default: "",
    label: "Bar feeder on, while the chuck is open (your machine's blocks, | between them)" },
  { id: "barFeederOff", type: "text", default: "",
    label: "Bar feeder off (your machine's blocks, | between them)" },
  { id: "partLoop", type: "choice", default: "end", choices: ["end", "repeat", "count"],
    label: "At the program's end: end (M30), repeat (M99, until stopped) or count parts" },
  { id: "partCount", label: "Parts to run, when counting", type: "number", default: 0, min: 0,
    max: 999999 },
  { id: "partCounter", type: "text", default: "",
    label: "Part counter, when counting: a macro variable free on your machine (as #500)" },
];

export default class extends FanucLathe {}
