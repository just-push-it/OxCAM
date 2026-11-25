// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Generic Fanuc-style 2-axis lathe: diameter programming, G96/G97 spindle with a G50 limit, feed
// per revolution, G32 threads, G84 tapping. Drilling and boring cycles are expanded into moves.
// Live tooling and bar work (the chuck, the bar feeder): Fanuc leaves their M-codes to each
// machine builder, so they are options here, and are empty until you set them for your machine. A post for your control can give them.
// Sources: ISO 6983-1; Fanuc-style lathe programming manuals (syntax facts only); DL-0022.

import { FanucLathe } from "oxcam:fanuc";

export const manifest = {
  id: "fanuc-generic-lathe",
  name: "Generic Fanuc-style lathe (2-axis, diameter programming)",
  version: "1.0.0",
  engine: 1,
  machines: ["lathe2"],
  extension: "nc",
  arcs: { maxSweep: 180, planes: ["xy", "zx", "yz"], helical: true },
  cycles: ["tap", "tap_left"],
  multiAxis: "tcp",
  verify: "fanuc-lathe",
  workOffsets: [{ from: 1, to: 6, code: "G{}", start: 54, name: "Standard" }],
  subSpindle: true,
};

export const options = [
  { id: "liveToolOn", label: "Live tool on ({rpm} is the speed; your machine's code)", type: "text",
    default: "" },
  { id: "liveToolOff", label: "Live tool off (your machine's code)", type: "text", default: "" },
  { id: "cAxisOn", label: "C axis on (your machine's code)", type: "text", default: "" },
  { id: "cAxisOff", label: "C axis off (your machine's code)", type: "text", default: "" },
  { id: "liveTapAxial", type: "text", default: "",
    label: "Live tap along Z (your machine's blocks, | between them; {rpm} {lead} {x} {y} {z} {c} {r})" },
  { id: "liveTapRadial", type: "text", default: "",
    label: "Live tap across the part (your machine's blocks, | between them; the same values)" },
  { id: "liveTapAxialLeft", type: "text", default: "",
    label: "Live tap along Z, left hand (your machine's blocks, | between them; the same values)" },
  { id: "liveTapRadialLeft", type: "text", default: "",
    label: "Live tap across the part, left hand (your machine's blocks, | between them; the same values)" },
  { id: "tapLeft", type: "text", default: "",
    label: "Left-hand tap cycle, on the spindle's axis (your control's code)" },
  // Fanuc's own: G7.1 C with the radius, in degrees, and G7.1 C0 off, each in its own block.
  { id: "wrapOn", label: "Cylindrical interpolation on (blocks, | between them; {r} radius, {d} diameter)",
    type: "text", default: "G7.1 C{r}" },
  { id: "wrapOff", label: "Cylindrical interpolation off (blocks, | between them)", type: "text",
    default: "G7.1 C0" },
  { id: "wrapLetter", label: "Letter for the way around, in cylindrical interpolation", type: "text",
    default: "C" },
  { id: "wrapDegrees", label: "That letter is in degrees (otherwise a length on the cylinder)",
    type: "bool", default: true },
  // Fanuc's own: G1 has to be on before G12.1, no G0 inside it, X a diameter, C the Y. Some
  // controls have G112 and G113 for them.
  { id: "polarOn", label: "Polar interpolation on (blocks, | between them)", type: "text",
    default: "G1 G12.1" },
  { id: "polarOff", label: "Polar interpolation off (blocks, | between them)", type: "text",
    default: "G13.1" },
  { id: "polarY", label: "Letter for Y in polar interpolation", type: "text", default: "C" },
  { id: "polarXDiameter", label: "X is a diameter in polar interpolation", type: "bool",
    default: true },
  // Bar work (DL-0091).
  { id: "chuckOpen", label: "Chuck open, for a bar pull (your machine's blocks, | between them)", type: "text",
    default: "" },
  { id: "chuckClose", label: "Chuck close, for a bar pull (your machine's blocks, | between them)", type: "text",
    default: "" },
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
  // The sub-spindle (ADR-0007): each builder's codes, empty until you set them.
  { id: "subSpindleSelect", type: "text", default: "",
    label: "Select the sub-spindle: S and M03 apply to it from here (your machine's blocks, | between them)" },
  { id: "mainSpindleSelect", type: "text", default: "",
    label: "Select the main spindle again (your machine's blocks, | between them)" },
  { id: "subChuckOpen", type: "text", default: "",
    label: "Sub-spindle chuck open (your machine's blocks, | between them)" },
  { id: "subChuckClose", type: "text", default: "",
    label: "Sub-spindle chuck close (your machine's blocks, | between them)" },
  { id: "syncOn", type: "text", default: "",
    label: "Spindles synchronised on (your machine's blocks, | between them)" },
  { id: "syncAngleOn", type: "text", default: "",
    label: "Spindles synchronised at an angle ({angle} in degrees; your machine's blocks)" },
  { id: "syncOff", type: "text", default: "",
    label: "Spindles synchronised off (your machine's blocks, | between them)" },
  { id: "z2Letter", type: "text", default: "", label: "Letter for Z2, the sub-spindle's axis" },
  { id: "subZ", type: "choice", default: "mirror", choices: ["mirror", "as_is"],
    label: "The sub-spindle's work: Z written mirrored, or as it is (the control mirrors it)" },
];

export default class extends FanucLathe {}
