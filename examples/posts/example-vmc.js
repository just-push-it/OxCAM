// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// An example post processor for a Fanuc-style vertical machining centre (docs/posts). It starts
// from the generic base and changes what this imaginary control does differently:
//
//   - a program header with the program number in five digits and a tool list;
//   - an optional stop (M01) before each tool change, and a retract before it;
//   - the next tool preloaded after each tool change;
//   - a per-operation "high-speed mode" option, noted before the operation;
//   - feeds clamped to the machine's maximum feed, with a warning (posts may change feeds:
//     ADR-0004 amendment).
//
// Sources: ISO 6983-1 and Fanuc-style controller programming manuals (syntax facts only).
// Copy it into your post library folder, give it your own id and name, and change it for your
// machine. Dry-run every new post before cutting.

import { out, warning } from "oxcam:post";
import { FanucMill } from "oxcam:fanuc";

export const manifest = {
  id: "example-vmc",
  name: "Example VMC (Fanuc-style)",
  version: "1.0.0",
  engine: 1,
  machines: ["mill3"],
  extension: "nc",
  arcs: { maxSweep: 180, planes: ["xy", "zx", "yz"], helical: true },
  tolerance: 0.002,
  cycles: ["drill", "dwell", "peck", "chip_break", "tap", "bore"],
  verify: "fanuc-mill",
};

export const options = [
  { id: "optionalStop", label: "Optional stop before each tool", type: "bool", default: true },
  { id: "preload", label: "Preload the next tool", type: "bool", default: true },
  { id: "highSpeed", label: "High-speed mode", type: "choice", default: "off",
    choices: ["off", "roughing", "finishing"], scope: "operation" },
];

export default class extends FanucMill {
  constructor(context) {
    super(context);
    this.header = ["%", "O{program_number5}"];
    this.safeStart = "G17 G21 G40 G49 G80 G90 G94";
  }

  expand(template, tool, toolName) {
    // One more template field: the program number in five digits (O01000).
    return super.expand(template.replaceAll("{program_number5}",
                                            String(this.program.number).padStart(5, "0")),
                        tool, toolName);
  }

  programBegin(program) {
    super.programBegin(program);
    out.comment(`${program.project} - ${program.setup}`);
    out.comment(`POST ${program.post.id} ${program.post.version}`);
    for (const tool of program.tools) {
      out.comment(`T${tool.number} ${tool.name} D${tool.diameter.toFixed(3)}`);
    }
  }

  operationBegin(op) {
    super.operationBegin(op);
    // Write your control's high-speed (look-ahead) code for each level here, from its manual.
    // This example only notes the level in a comment.
    if (op.options.highSpeed !== "off") {
      out.comment(`HIGH SPEED ${op.options.highSpeed}`);
    }
  }

  toolChange(i) {
    out.block("G91 G28 Z0.");
    out.block("G90");
    if (this.options.optionalStop) {
      out.block("M01");
    }
    super.toolChange(i);
    if (this.options.preload) {
      // Tools are listed in order of first use.
      const k = this.program.tools.findIndex((t) => t.number === i.tool);
      const next = this.program.tools[k + 1];
      if (next) {
        out.block(`T${next.number}`);
      }
    }
  }

  feedWord(feed) {
    const max = this.program.machine?.max_feed ?? 0;
    if (max > 0 && feed > max) {
      warning(`a feed of ${feed.toFixed(0)} mm/min was clamped to the machine's ${max} mm/min`);
      feed = max;
    }
    return super.feedWord(feed);
  }
}
