// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Generic Fanuc-style mill: 3-axis, 3+2 indexing, multi-axis with tool centre point control.
// Sources: ISO 6983-1; Fanuc-style controller programming manuals (syntax facts only); the
// output of the generic posts is described in DL-0011, DL-0019 and DL-0023.

import { FanucMill } from "oxcam:fanuc";

export const manifest = {
  id: "fanuc-generic-mill",
  name: "Generic Fanuc-style mill (3-axis, 3+2 indexing, multi-axis with tool centre point control)",
  version: "1.0.0",
  engine: 1,
  machines: ["mill3", "mill5_table_ac", "mill5_table_bc", "mill5_head_table_bc", "mill5_head_bc"],
  extension: "nc",
  arcs: { maxSweep: 180, planes: ["xy", "zx", "yz"], helical: true },
  cycles: ["drill", "dwell", "peck", "chip_break", "tap", "tap_left", "bore"],
  multiAxis: "tcp",
  verify: "fanuc-mill",
};

export const options = [];

export default class extends FanucMill {}
