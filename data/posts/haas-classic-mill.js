// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Mill post for Haas-style controls, written for the classic control and suited to the newer
// one: 3-axis, a 4th axis (A) and 5-axis trunnion tables, WITHOUT tool centre point control.
// Rotary moves are machine positions with inverse-time feeds (G93): a program with simultaneous
// rotary motion is valid only for the tool lengths and machine it was posted for, and it says so.
//
// Sources (syntax facts only, paraphrased; docs/ip/IP-POLICY.md):
//   [OM] Haas Mill Operator's Manual 96-8000 Rev Y, December 2009 (classic control): program
//        format and comments, tooling (Tnn M06), the G-code reference (G02/G03, G04, G28, G53,
//        G54-G59, G73, G81-G85, G93/G94, G98/G99, G110-G129, G154, G187), the M-code reference
//        (M00-M09, M10-M13, M30, M83/M84, M88/M89) and the 4th/5th axis notes (pp. 123-125).
//   [PM] Haas Mill Programming, Productivity Inc / Haas Factory Outlet, revised 05/25 (Next
//        Generation Control): work offsets, program structure (a safe-start block after each
//        tool change, G53 G49 Z0 retract), G93.
// No other post processor was used as a source. "Haas" names the control the output is for
// (nominative use); OxCAM is not affiliated with Haas Automation. See docs/posts/haas-classic-mill.md.
//
// What the control requires, and how this post meets it:
//   - Programs begin and end with "%" and are numbered Onnnnn (O09xxx is left to macros): the
//     header is "%" and "O01000 (NAME)".
//   - Comments are in parentheses, at most 80 characters: comments are cut to fit.
//   - One M code per block: every M code has its own block.
//   - G28 cancels tool length compensation: after a Z return before the table turns, the next
//     approach applies G43 again.
//   - A G04 / cycle P with a decimal point is in seconds; G73 and G83 keep P for later holes:
//     dwells are written in seconds, and G73/G83 always state theirs (P0. when none).
//   - Simultaneous 4/5-axis motion needs G93, with F (1/min) in every block; rapid moves are not
//     allowed in it, and a feed move makes the axes arrive together [OM p. 124]: every move of a
//     simultaneous toolpath is a G93 G1 move, its positioning moves taking the time the user set.
//   - Brakes: M10/M11 (4th axis) and M12/M13 (5th axis) engage/release. The brakes should be on
//     for other cutting; M11/M13 just before G93 avoid a pause each block: released before a
//     simultaneous toolpath and engaged after it, when the option is on.
//   - Extended work offsets: G154 P1-P99 (P1-P20 are also G110-G129): offsets 7-105, declared in
//     the manifest (the Fanuc base writes them).
//   - Through-spindle coolant: M88 is commanded before the spindle speed.

import { out, word, number, error, warning } from "oxcam:post";
import { FanucMill } from "oxcam:fanuc";

export const manifest = {
  id: "haas-classic-mill",
  name: "Haas-style mill, classic control (3-, 4- and 5-axis, no tool centre point control)",
  version: "1.0.0",
  engine: 1,
  machines: ["mill3", "mill4_a", "mill5_table_ac", "mill5_table_bc"],
  extension: "nc",
  arcs: { maxSweep: 180, planes: ["xy", "zx", "yz"], helical: true },
  cycles: ["drill", "dwell", "peck", "chip_break", "tap", "tap_left", "bore"],
  multiAxis: "machine",
  verify: "fanuc-mill",
  // G54-G59, then G154 P1-P99 (P1-P20 are also G110-G129); the machine's work offset count
  // limits what a setup can choose.
  workOffsets: [
    { from: 1, to: 6, code: "G{}", start: 54, name: "Standard" },
    { from: 7, to: 105, code: "G154 P{}", start: 1, name: "Extended" },
  ],
};

export const options = [
  { id: "units", label: "Program units", type: "choice", default: "mm", choices: ["mm", "inch"] },
  { id: "lineNumbers", label: "Line numbers", type: "bool", default: false },
  { id: "optionalStop", label: "Optional stop (M01) before each tool change after the first",
    type: "bool", default: true },
  { id: "safeStartAfterToolChange", label: "Safe start (G00 G90 G40 G49) after each tool change",
    type: "bool", default: true },
  { id: "homeWith", label: "Return Z (and Y at the end) with", type: "choice", default: "G28",
    choices: ["G28", "G53"] },
  { id: "homeYAtEnd", label: "Return Y at the end of the program", type: "bool", default: true },
  { id: "mistCoolant", label: "Mist coolant (the control has no mist code)", type: "choice",
    default: "refuse", choices: ["refuse", "shower M07", "flood M08"] },
  { id: "fifthAxisLetter", label: "Letter of an A/C trunnion's rotary (C) axis", type: "choice",
    default: "C", choices: ["C", "B"] },
  { id: "rotaryBrakes", label: "Release the rotary brakes (M11/M13) for simultaneous toolpaths",
    type: "bool", default: true },
  { id: "positioningTime", label: "Positioning moves of simultaneous toolpaths take (s)",
    type: "number", default: 1, min: 0.05, max: 600 },
  { id: "smoothness", label: "Smoothness (G187)", type: "choice", default: "control default",
    choices: ["control default", "rough", "medium", "finish"], scope: "operation" },
  { id: "cornerRounding", label: "Max corner rounding (G187 E, program units; 0: the control's)",
    type: "number", default: 0, min: 0, max: 100, scope: "operation" },
];

// An option's value, or its default above when the caller left it out.
const DEFAULTS = Object.fromEntries(options.map((o) => [o.id, o.default]));
const optionOf = (values, id) => values?.[id] ?? DEFAULTS[id];

const CYCLE_CODES = {
  drill: "G81", dwell: "G82", peck: "G83", chip_break: "G73", tap: "G84", tap_left: "G74",
  bore: "G85",
};
const SMOOTHNESS = { rough: "P1", medium: "P2", finish: "P3" };
const COMMENT_LENGTH = 78; // inside the parentheses: 80 characters in all
const COOLANT_ON = { flood: "M08", air: "M83", through_tool: "M88" };
const COOLANT_OFF = { flood: "M09", mist: "M09", air: "M84", through_tool: "M89" };

export default class HaasClassicMill extends FanucMill {
  constructor(context) {
    super(context);
    const opt = (id) => optionOf(this.options, id);
    const inch = opt("units") === "inch";
    this.units = inch ? "in" : "mm";
    this.decimals = inch ? { linear: 4, feed: 2, dwell: 3, rotary: 3, feedPerRev: 4 }
                         : { linear: 3, feed: 0, dwell: 3, rotary: 3, feedPerRev: 3 };
    this.lineNumbers = opt("lineNumbers") ? { start: 10, step: 10, max: 99999 } : null;
    this.header = ["%", "O{program_number5} ({program_name})"];
    this.safeStart = `G00 G17 ${inch ? "G20" : "G21"} G40 G49 G80 G90 G94`;
    this.toolChangeBlocks = ["T{tool} M06 ({tool_name})"];
    this.toolLengthOffset = "G43 H{tool}";
    this.cutterCompOffset = "D{tool}";
    const g53 = opt("homeWith") === "G53";
    const homeY = opt("homeYAtEnd");
    this.homeZBlocks = g53 ? ["G00 G53 G49 Z0."] : ["G91 G28 Z0.", "G90"];
    const end = g53 ? ["G00 G53 G49 Z0.", ...(homeY ? ["G53 Y0."] : [])]
                    : ["G91 G28 Z0.", ...(homeY ? ["G28 Y0."] : []), "G90"];
    this.footer = [...end, "M30", "%"];
    this.spindleCw = "M03";
    this.spindleCcw = "M04";
    this.spindleStop = "M05";
  }

  opt(id) {
    return optionOf(this.options, id);
  }

  // -- Program ------------------------------------------------------------------------------

  programBegin(program) {
    const n = program.number;
    if (!(Number.isInteger(n) && n >= 1 && n <= 99999)) {
      error(`the program number ${n} does not fit Onnnnn (1 to 99999)`);
    }
    if (n >= 9000 && n <= 9999) {
      warning(`program O0${n}: O09xxx numbers are best left to macro programs`);
    }
    const capacity = program.machine?.tool_capacity ?? 0;
    for (const tool of program.tools ?? []) {
      if (capacity > 0 && tool.number > capacity) {
        error(`tool ${tool.number} (${tool.name}): the machine holds tools 1 to ${capacity}`);
      }
    }
    super.programBegin(program);
    this.seconds = number({ decimals: 3, point: "always" });
    // A Haas trunnion may name its platter axis B: the machine's C words become B words.
    if (this.opt("fifthAxisLetter") === "B" && program.machine?.kinematics === "mill5_table_ac") {
      this.rotaryWords.C = word("B", this.formats.rotary);
    }
    this.toolsChanged = 0;
    this.tool = 0;
    this.coolantState = "off";
    this.lengthCancelled = false; // G28 or G53 G49 cancelled G43
    this.simultaneous = false;
    this.brakesReleased = []; // the engage codes of the brakes released
    this.smoothing = this.smoothingOf(null);
    if (program.project || program.setup) {
      this.note([program.project, program.setup].filter((s) => s).join(" - "));
    }
    if (program.post) {
      this.note(`POSTED WITH ${program.post.id} ${program.post.version}` +
                (program.time ? ` ${program.time}` : ""));
    }
  }

  programEnd(program) {
    this.engageBrakes();
    this.coolant({ coolant: "off" });
    super.programEnd(program);
  }

  /// The program number in five digits; names cut to fit a comment.
  expand(template, tool, toolName) {
    return super.expand(
      template.replaceAll("{program_number5}", String(this.program.number).padStart(5, "0"))
        .replaceAll("{program_name}", out.commentText(this.program.name).slice(0, 60)),
      tool, (toolName ?? "").slice(0, 60));
  }

  /// A comment that fits the control's 80 characters.
  note(text) {
    const clean = out.commentText(text ?? "").slice(0, COMMENT_LENGTH);
    if (clean !== "") {
      out.comment(clean);
    }
  }

  comment(i) {
    this.note(i.text);
  }

  // -- Operations ---------------------------------------------------------------------------

  /// "level/corner" of an operation's G187 options (null: the control's defaults).
  smoothingOf(op) {
    return `${optionOf(op?.options, "smoothness")}/${optionOf(op?.options, "cornerRounding")}`;
  }

  operationBegin(op) {
    this.note(op.name);
    this.note(op.comment);
    // Smoothness and corner rounding as the user set them for the operation (G187).
    const smoothing = this.smoothingOf(op);
    if (smoothing !== this.smoothing) {
      const level = optionOf(op.options, "smoothness");
      const corner = optionOf(op.options, "cornerRounding");
      out.block("G187", SMOOTHNESS[level] ?? "",
                corner > 0 ? "E" + this.formats.linear.format(corner) : "");
      this.smoothing = smoothing;
    }
    this.simultaneous = op.orientation === "simultaneous";
    if (this.simultaneous) {
      const tool = (this.program.tools ?? []).find((t) => t.number === op.tool);
      const length = tool ? `${this.length(tool.gauge_length)} ${this.inch ? "IN" : "MM"}` : "?";
      this.note(`NO TCP: VALID ONLY FOR T${op.tool} GAUGE LENGTH ${length}`);
      this.note("AND THE PIVOT OF THE MACHINE IT WAS POSTED FOR");
    }
  }

  operationEnd(op) {
    this.engageBrakes();
    this.simultaneous = false;
  }

  /// Releases the brakes of the rotary axes a simultaneous move turns (M11 4th, M13 5th).
  releaseBrakes(i) {
    if (!this.opt("rotaryBrakes") || this.brakesReleased.length > 0 || !i.axes) {
      return;
    }
    this.flushPending();
    const codes = [["M11", "M10"], ["M13", "M12"]];
    for (let k = 0; k < Math.min(i.axes.length, 2); ++k) {
      out.block(codes[k][0]);
      this.brakesReleased.push(codes[k][1]);
    }
  }

  engageBrakes() {
    for (const code of this.brakesReleased) {
      out.block(code);
    }
    this.brakesReleased = [];
  }

  // -- Tools, offsets, spindle, coolant -----------------------------------------------------

  toolChange(i) {
    if (this.toolsChanged > 0 && this.opt("optionalStop")) {
      this.flushPending();
      out.block("M01");
    }
    super.toolChange(i);
    this.toolsChanged += 1;
    this.tool = i.tool;
    this.lengthCancelled = false;
    if (this.opt("safeStartAfterToolChange")) {
      out.block("G00 G90 G40 G49");
      this.motion = "G0";
    }
  }

  spindle(i) {
    // Through-spindle coolant goes on before the spindle speed: look ahead to the coolant of
    // this approach.
    if (i.rpm > 0) {
      for (let k = 1; k <= 4; ++k) {
        const next = this.program.peek(k);
        if (!next || next.op === "linear" || next.op === "arc" || next.op === "cycle") {
          break;
        }
        if (next.op === "coolant") {
          if (next.coolant === "through_tool") {
            this.coolant(next);
          }
          break;
        }
      }
    }
    super.spindle(i);
  }

  coolant(i) {
    const want = i.coolant;
    if (want === this.coolantState) {
      return;
    }
    let code = COOLANT_ON[want];
    if (want === "mist") {
      const mist = this.opt("mistCoolant");
      if (mist === "refuse") {
        error("an operation uses mist coolant, which the control has no code for: set the " +
              "post's \"Mist coolant\" option, or choose another coolant for the operation");
      }
      code = mist === "shower M07" ? "M07" : "M08";
    }
    this.flushPending();
    if (this.coolantState !== "off") {
      out.block(COOLANT_OFF[this.coolantState]);
    }
    this.coolantState = "off";
    if (code) {
      out.block(code);
      this.coolantState = want;
    }
  }

  // -- Motion ---------------------------------------------------------------------------------

  /// Leaves inverse time before a move whose F is per minute (arcs, cycles).
  feedPerMinute() {
    if (this.inverseTimeMode) {
      this.pending.push("G94");
      this.inverseTimeMode = false;
      this.lastFeed = null;
    }
  }

  rapid(i) {
    if (this.simultaneous && i.axes) {
      // A move of a simultaneous toolpath: a G93 feed move of the positioning time.
      this.releaseBrakes(i);
      if (this.plane !== "XY") this.selectPlane();
      const moves = [];
      this.axes(i, false, moves);
      this.rotary(i, false, moves);
      const words = this.takePending();
      if (moves.length === 0 && words.length === 0) {
        return;
      }
      words.push(this.feedMode({ inverse_time: 1 }), this.motionWord("G1"), ...moves);
      this.lastFeed = null;
      words.push("F" + this.formats.inverseTime.format(60 / this.opt("positioningTime")));
      out.block(...words);
      return;
    }
    if (this.lengthCancelled && i.z != null) {
      // After a Z return (G28 / G53 G49) the approach applies tool length compensation again.
      if (this.plane !== "XY") this.selectPlane();
      const words = this.takePending();
      words.unshift(this.motionWord("G0"));
      words.push(this.expand(this.toolLengthOffset, this.tool, ""));
      this.axes(i, false, words);
      this.rotary(i, false, words);
      out.block(...words);
      this.lengthCancelled = false;
      return;
    }
    super.rapid(i);
  }

  linear(i) {
    if (this.simultaneous && i.axes) {
      this.releaseBrakes(i);
    }
    super.linear(i);
  }

  arc(i) {
    this.feedPerMinute();
    super.arc(i);
  }

  dwell(i) {
    this.flushPending();
    out.block("G4", "P" + this.seconds.format(i.seconds));
  }

  toolLength(i) {
    super.toolLength(i);
    this.lengthCancelled = false;
  }

  cycle(i) {
    this.feedPerMinute();
    this.selectPlane();
    const c = i.cycle;
    const code = CYCLE_CODES[c.type] ?? "G81";
    // Dwell in seconds; G73 and G83 always state theirs, since P carries over to later cycles.
    const dwell = c.type === "dwell" || c.type === "peck" || c.type === "chip_break"
      ? "P" + this.seconds.format(c.dwell > 0 ? c.dwell : 0) : "";
    const signature = [code, this.length(c.bottom), this.length(c.retract), this.length(c.peck),
                       dwell, this.formats.feed.format(this.toUnit(c.feed)), c.pitch].join(" ");
    const hole = { x: i.x, y: i.y };
    const words = this.takePending();
    if (signature !== this.activeCycle) {
      this.activeCycle = signature;
      this.motion = code;
      words.push("G99", code);
      this.axes(hole, true, words);
      words.push("Z" + this.length(c.bottom), "R" + this.length(c.retract));
      if (c.type === "peck" || c.type === "chip_break") {
        words.push("Q" + this.length(c.peck));
      }
      words.push(dwell);
      this.lastFeed = null;
      words.push(this.feedWord(c.feed));
    } else {
      this.axes(hole, false, words);
    }
    out.block(...words);
    // After the hole the tool sits at the R plane (G99).
    this.position.z = c.retract;
    this.axisWords[2].reset();
    this.axisWords[2].of(this.toUnit(c.retract));
  }

  homeZ(i) {
    super.homeZ(i);
    this.lengthCancelled = true;
  }
}
