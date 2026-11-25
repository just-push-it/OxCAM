// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Generic Fanuc-style mill with a 4th axis: machine positions with a rotary A axis and
// inverse-time feeds (G93) on moves that turn it.
// Sources: ISO 6983-1; Fanuc-style controller programming manuals (syntax facts only); DL-0023.

import { FanucMill } from "oxcam:fanuc";

export const manifest = {
  id: "fanuc-generic-mill-4axis",
  name: "Generic Fanuc-style mill with a 4th axis (machine positions, inverse-time feeds)",
  version: "1.0.0",
  engine: 1,
  machines: ["mill3", "mill4_a"],
  extension: "nc",
  arcs: { maxSweep: 180, planes: ["xy", "zx", "yz"], helical: true },
  cycles: ["drill", "dwell", "peck", "chip_break", "tap", "tap_left", "bore"],
  multiAxis: "machine",
  verify: "fanuc-mill",
};

export const options = [];

export default class extends FanucMill {}
