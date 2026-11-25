// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// oxcam:fanuc: generic Fanuc-style base posts (DL-0039). A machine's post imports one, changes
// its settings in its constructor, and overrides the handlers that differ:
//
//   import { FanucMill } from "oxcam:fanuc";
//   export default class extends FanucMill {
//     constructor(context) { super(context); this.safeStart = "G17 G21 G40 G49 G80 G90"; }
//   }
//
// The output is the C++ formatter's of DL-0011, DL-0022 and DL-0023, byte for byte (the built-in
// posts are tested against it): G-code syntax from ISO 6983-1 and Fanuc-style controller manuals
// (syntax facts only).

import { number, word, out, warning, error } from "oxcam:post";

const CYCLE_CODES = {
  drill: "G81", dwell: "G82", peck: "G83", chip_break: "G73", tap: "G84", tap_left: "G74",
  bore: "G85", back_bore: "G87",
};

// The code that selects each arc plane.
const PLANES = { XY: { code: "G17" }, ZX: { code: "G18" }, YZ: { code: "G19" } };

/// Generic Fanuc-style mill: 3-axis, 3+2 indexing, multi-axis with tool centre point control.
export class FanucMill {
  constructor(context) {
    this.program = context.program;
    this.options = context.options;
    this.out = context.out;

    // -- Settings: change them in a subclass's constructor ----------------------------------
    this.units = "mm"; // "mm" or "in": lengths, feeds and thread leads are converted
    this.lineNumbers = { start: 10, step: 10, max: 99999 }; // null: none
    this.decimals = { linear: 3, feed: 0, dwell: 3, rotary: 3, feedPerRev: 3 };
    this.trailingPoint = true; // "10." rather than "10"
    this.arcFormat = "ijk"; // "ijk" or "r"
    this.maxArcSweep = 180; // [deg] longer arcs are split
    this.dwellInMilliseconds = true; // G4 P500 rather than G4 X0.5
    this.commentStyle = "parentheses"; // "parentheses", "semicolon" or "none"
    this.upperComments = true;
    // Templates: {program_number}, {program_name}, {tool}, {tool2} (two digits), {tool_name}.
    this.header = ["%", "O{program_number} ({program_name})"]; // lines without numbers
    this.safeStart = "G17 G21 G40 G49 G80 G90 G94"; // one block
    this.toolChangeBlocks = ["T{tool} M06 ({tool_name})"];
    this.toolLengthOffset = "G43 H{tool}";
    this.cutterCompOffset = "D{tool}";
    this.homeZBlocks = ["G91 G28 Z0.", "G90"]; // before the table turns without a tool change
    this.footer = ["G91 G28 Z0.", "G90", "M30", "%"]; // "%" and "" are lines without numbers
    this.coolantCodes = { flood: "M08", mist: "M07", air: "M07", through_tool: "M88",
                          off: "M09" };
    this.spindleCw = "M03";
    this.spindleCcw = "M04";
    this.spindleStop = "M05";
    // Spindle orientation at an angle (a back-boring bar passing its hole off centre). Controls
    // differ in the word that takes the angle: change it in a subclass. {angle}: degrees.
    this.spindleOrientBlock = "M19 R{angle}";
    this.tcpOnOffset = "G43.4 H{tool}"; // tool length offset that starts tool centre point control
    this.tcpOffBlock = "G49"; // ends tool centre point control
    this.lathe = false; // X as diameter, no Y, G96/G97 spindle, G32 threads
    this.feedPerRev = false; // F per revolution of the spindle
    // Probing (DL-0090): the skip function, a move the control ends when the probe triggers.
    this.probeSkip = "G31";
  }

  // -- Program ------------------------------------------------------------------------------

  programBegin(program) {
    const point = this.trailingPoint ? "always" : "needed";
    this.formats = {
      linear: number({ decimals: this.decimals.linear, point }),
      feed: number({ decimals: this.decimals.feed, point }),
      feedPerRev: number({ decimals: this.decimals.feedPerRev, point }),
      dwell: number({ decimals: this.decimals.dwell, point }),
      rotary: number({ decimals: this.decimals.rotary, point }),
      inverseTime: number({ decimals: 3, point }),
      lead: number({ decimals: 4, point }),
      whole: number({ decimals: 0, point: "needed" }),
    };
    out.numbering(this.lineNumbers);
    out.commentStyle = this.commentStyle;
    out.upperComments = this.upperComments;

    // Modal state, as the control has it.
    this.axisWords = ["X", "Y", "Z"].map((a) => word(a, this.formats.linear));
    this.rotaryWords = { A: word("A", this.formats.rotary), B: word("B", this.formats.rotary),
                         C: word("C", this.formats.rotary) };
    this.motion = null; // the motion code in effect (G0, G1, G2, G3, G32, a cycle)
    this.inch = this.units === "in";
    this.position = { x: 0, y: 0, z: 0 }; // [mm] where the tool is (program coordinates)
    this.lastFeed = null;
    this.inverseTimeMode = false;
    this.rpm = 0;
    this.pending = []; // words merged into the next block that moves the machine
    this.plane = "XY";
    this.activeCycle = null;
    this.probeIsOn = false; // probeOn() was called for the tool in the spindle
    this.probeWarned = false;

    for (const line of this.header) {
      out.line(this.expand(line, 0, ""));
    }
    out.block(this.safeStart);
  }

  programEnd(program) {
    this.flushPending();
    this.endProbing();
    for (const line of this.footer) {
      const text = this.expand(line, 0, "");
      if (text === "%" || text === "") {
        out.line(text);
      } else {
        out.block(text);
      }
    }
  }

  operationBegin(op) {
    out.comment(op.name);
  }

  operationEnd(op) {}

  // -- Helpers ------------------------------------------------------------------------------

  /// A length in the output unit.
  toUnit(mm) {
    return this.inch ? mm / 25.4 : mm;
  }

  length(mm) {
    return this.formats.linear.format(this.toUnit(mm));
  }

  expand(template, tool, toolName) {
    return template
      .replaceAll("{program_number}", String(this.program.number))
      .replaceAll("{program_name}", out.commentText(this.program.name))
      .replaceAll("{tool2}", String(tool).padStart(2, "0"))
      .replaceAll("{tool}", String(tool))
      .replaceAll("{tool_name}", out.commentText(toolName));
  }

  takePending() {
    const words = this.pending;
    this.pending = [];
    return words;
  }

  flushPending() {
    if (this.pending.length > 0) {
      out.block(...this.takePending());
    }
  }

  /// The arc plane is modal: an arc out of the current plane selects its own (G18, G19), and
  /// the first motion, cycle or compensation after such arcs selects XY again (G17). Called by
  /// the handlers of those instructions.
  selectPlane(plane = "XY") {
    if (plane !== this.plane) {
      this.pending.push(PLANES[plane].code);
      this.plane = plane;
    }
  }

  /// The motion code when it changes, else "".
  motionWord(g) {
    if (g === this.motion) return "";
    this.motion = g;
    return g;
  }

  /// Pushes the axis words of a target onto `words`; null coordinates are not written.
  /// `forceXY` writes X and Y even when unchanged (arcs).
  axes(target, forceXY, words) {
    const w = this.axisWords;
    const x = target.x;
    if (x != null) {
      if (forceXY) w[0].reset();
      // Lathes program diameters in X.
      const t = w[0].of(this.toUnit(this.lathe ? 2 * x : x));
      if (t !== "") words.push(t);
      this.position.x = x;
    }
    const y = target.y;
    // A lathe has a Y word only once something has left the spindle's centre: most have no Y axis.
    if (this.lathe && y) {
      this.latheY = true;
    }
    if (y != null && (!this.lathe || this.latheY)) {
      if (forceXY) w[1].reset();
      const t = w[1].of(this.toUnit(y));
      if (t !== "") words.push(t);
      this.position.y = y;
    }
    const z = target.z;
    if (z != null) {
      const t = w[2].of(this.toUnit(z));
      if (t !== "") words.push(t);
      this.position.z = z;
    }
  }

  /// Pushes the rotary axis words of a multi-axis move or an index onto `words` (modal, like the
  /// linear axes; `force` writes them all).
  rotary(i, force, words) {
    const axes = i.axes;
    if (!axes) return;
    for (let k = 0; k < axes.length; ++k) {
      const letter = axes[k];
      const w = letter === "A" ? this.rotaryWords.A
              : letter === "B" ? this.rotaryWords.B : this.rotaryWords.C;
      if (force) w.reset();
      const t = w.of(letter === "A" ? i.a : letter === "B" ? i.b : i.c);
      if (t !== "") words.push(t);
    }
  }

  /// Feed mode word of a linear move when it changes: inverse time (G93) or per minute (G94).
  feedMode(i) {
    const inverse = i.inverse_time > 0;
    if (inverse === this.inverseTimeMode) {
      return "";
    }
    this.inverseTimeMode = inverse;
    this.lastFeed = null;
    return inverse ? "G93" : "G94";
  }

  /// F word of a feed [mm/min]: per revolution when set and the spindle turns.
  feedWord(feed) {
    const f = this.feedPerRev && this.rpm > 0
                ? this.formats.feedPerRev.format(this.toUnit((feed / 60) / (this.rpm / 60)))
                : this.formats.feed.format(this.toUnit(feed));
    if (f === this.lastFeed) {
      return "";
    }
    this.lastFeed = f;
    return "F" + f;
  }

  // -- Instructions -------------------------------------------------------------------------

  comment(i) {
    out.comment(i.text);
  }

  toolChange(i) {
    this.flushPending();
    this.endProbing();
    for (const line of this.toolChangeBlocks) {
      out.block(this.expand(line, i.tool, i.tool_name));
    }
    // The tool change moves the machine: state every axis again afterwards.
    this.axisWords.forEach((w) => w.reset());
    this.motion = null;
  }

  workOffset(i) {
    this.pending.push(this.workOffsetCode(i.work_offset));
  }

  /// The code of work offset n as the manifest's `workOffsets` declares it (program.post).
  workOffsetCode(n) {
    const ranges = this.program.post.work_offsets;
    for (const r of ranges) {
      if (n >= r.from && n <= r.to) {
        return r.code.replace("{}", String(r.start + n - r.from));
      }
    }
    const has = ranges.map((r) => `${r.from}-${r.to}`).join(", ");
    error(`work offset ${n}: ${this.program.post.name} has work offsets ${has}`);
  }

  spindle(i) {
    this.rpm = i.rpm;
    this.lastFeed = null; // feed per revolution depends on the spindle speed
    const whole = this.formats.whole;
    const direction = i.reverse ? this.spindleCcw : this.spindleCw;
    if (i.rpm <= 0) {
      out.block(this.spindleStop);
    } else if (this.lathe && (i.surface_speed ?? 0) > 0) {
      if ((i.max_rpm ?? 0) > 0) {
        out.block(`G50 S${whole.format(i.max_rpm)}`);
      }
      // m/min, or ft/min for inch programs.
      const vc = this.units === "in" ? i.surface_speed / 0.3048 : i.surface_speed;
      out.block("G96", `S${whole.format(vc)}`, direction);
    } else if (this.lathe) {
      out.block("G97", `S${whole.format(i.rpm)}`, direction);
    } else {
      out.block(`S${whole.format(i.rpm)}`, direction);
    }
  }

  spindleOrient(i) {
    this.flushPending();
    this.rpm = 0;
    out.block(this.spindleOrientBlock.replaceAll("{angle}", this.formats.rotary.format(i.angle)));
  }

  coolant(i) {
    out.block(this.coolantCodes[i.coolant] ?? this.coolantCodes.off);
  }

  toolLength(i) {
    const words = this.takePending();
    words.unshift(this.motionWord("G0"));
    words.push(this.expand(i.tcp ? this.tcpOnOffset : this.toolLengthOffset, i.tool, ""));
    this.axes({ z: i.z }, false, words);
    out.block(...words);
  }

  rapid(i) {
    if (this.plane !== "XY") this.selectPlane();
    const words = this.takePending();
    words.unshift(this.motionWord("G0"));
    this.axes(i, false, words);
    this.rotary(i, false, words);
    if (words.length === 1 && words[0] === "") {
      return; // nothing changes
    }
    out.block(...words);
  }

  linear(i) {
    if (this.plane !== "XY") this.selectPlane();
    if (i.pitch > 0) {
      // Thread cut: spindle-synchronized, F is the lead.
      const words = this.takePending();
      words.push(this.motionWord("G32"));
      this.axes(i, false, words);
      words.push("F" + this.formats.lead.format(this.toUnit(i.pitch)));
      this.lastFeed = null;
      out.block(...words);
      return;
    }
    const words = this.takePending();
    const moves = [];
    this.axes(i, false, moves);
    this.rotary(i, false, moves);
    if (moves.length === 0 && words.length === 0 && this.motion === "G1") {
      return;
    }
    words.push(this.feedMode(i), this.motionWord("G1"));
    for (let k = 0; k < moves.length; ++k) words.push(moves[k]);
    if (i.inverse_time > 0) {
      // Inverse time: F is due in every block.
      this.lastFeed = null;
      words.push("F" + this.formats.inverseTime.format(i.inverse_time));
    } else {
      words.push(this.feedWord(i.feed));
    }
    out.block(...words);
  }

  arc(i) {
    const plane = i.plane ?? "XY";
    this.selectPlane(plane);
    // The plane's two coordinates (u, v) and its normal (n), as toolpath::planeCoords: XY is
    // (x, y | z), ZX is (z, x | y), YZ is (y, z | x).
    const pos = this.position;
    let cu, cv, su, sv, eu, ev, n0, n1;
    if (plane === "XY") {
      cu = i.cx; cv = i.cy; su = pos.x; sv = pos.y; eu = i.x; ev = i.y; n0 = pos.z; n1 = i.z;
    } else if (plane === "ZX") {
      cu = i.cz; cv = i.cx; su = pos.z; sv = pos.x; eu = i.z; ev = i.x; n0 = pos.y; n1 = i.y;
    } else {
      cu = i.cy; cv = i.cz; su = pos.y; sv = pos.z; eu = i.y; ev = i.z; n0 = pos.x; n1 = i.x;
    }
    const r = Math.sqrt((su - cu) * (su - cu) + (sv - cv) * (sv - cv));
    const a0 = Math.atan2(sv - cv, su - cu);
    const a1 = Math.atan2(ev - cv, eu - cu);
    let sweep = (a1 - a0) % (2 * Math.PI);
    if (!i.clockwise && sweep <= 1e-12) {
      sweep += 2 * Math.PI;
    } else if (i.clockwise && sweep >= -1e-12) {
      sweep -= 2 * Math.PI;
    }
    const maxSweep = this.maxArcSweep * Math.PI / 180;
    const parts = Math.max(1, Math.ceil(Math.abs(sweep) / maxSweep - 1e-6));
    const motion = i.clockwise ? "G2" : "G3";
    for (let k = 1; k <= parts; ++k) {
      // The last part ends at the arc's end; the others on the arc, the normal coordinate
      // moving evenly (helices).
      let end = i;
      if (k < parts) {
        const t = k / parts;
        const a = a0 + sweep * t;
        const u = cu + r * Math.cos(a);
        const v = cv + r * Math.sin(a);
        const n = n0 + (n1 - n0) * t;
        end = plane === "XY" ? { x: u, y: v, z: n }
            : plane === "ZX" ? { x: v, y: n, z: u } : { x: n, y: u, z: v };
      }
      const fx = pos.x;
      const fy = pos.y;
      const fz = pos.z;
      const words = this.takePending();
      words.push(this.motionWord(motion));
      this.axes(end, true, words);
      if (this.arcFormat === "ijk") {
        // The centre's offsets along the plane's two axes: I/J (G17), I/K (G18), J/K (G19).
        if (plane !== "YZ") words.push("I" + this.length(i.cx - fx));
        if (plane !== "ZX") words.push("J" + this.length(i.cy - fy));
        if (plane !== "XY") words.push("K" + this.length(i.cz - fz));
      } else {
        words.push("R" + this.length(Math.abs(sweep / parts) > Math.PI + 1e-9 ? -r : r));
      }
      words.push(this.feedWord(i.feed));
      out.block(...words);
    }
  }

  dwell(i) {
    out.block("G4", this.dwellInMilliseconds
                      ? `P${this.formats.whole.format(i.seconds * 1000)}`
                      : "X" + this.formats.dwell.format(i.seconds));
  }

  cycle(i) {
    this.selectPlane();
    const c = i.cycle;
    const cycleCode = CYCLE_CODES[c.type] ?? "G81";
    const signature = [cycleCode, this.length(c.bottom), this.length(c.retract),
                       this.length(c.peck), c.dwell, this.formats.feed.format(this.toUnit(c.feed)),
                       c.pitch].join(" ");
    const hole = { x: i.x, y: i.y };
    const words = this.takePending();
    if (c.type === "back_bore") {
      this.backBore(i, words);
      return;
    }
    if (signature !== this.activeCycle) {
      this.activeCycle = signature;
      this.motion = cycleCode;
      words.push("G99", cycleCode);
      this.axes(hole, true, words);
      words.push("Z" + this.length(c.bottom), "R" + this.length(c.retract));
      if (c.type === "peck" || c.type === "chip_break") {
        words.push("Q" + this.length(c.peck));
      }
      if (c.type === "dwell" && c.dwell > 0) {
        words.push(`P${this.formats.whole.format(c.dwell * 1000)}`);
      }
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

  /// Back boring (G87) from the initial plane the tool stands at, returning there (G98): Z ends
  /// the cut, R is where it starts below the hole, Q the shift. On these controls the shift's
  /// direction and the spindle's angle are the control's own settings, so a post declares
  /// "back_bore" in its cycles only for a machine whose settings match the operation's (DL-0083).
  backBore(i, words) {
    const c = i.cycle;
    this.activeCycle = null; // every back bore is written in full
    this.motion = "G87";
    words.push("G98", "G87");
    this.axisWords[1].reset();
    this.axes({ x: i.x, y: i.y }, true, words);
    words.push("Z" + this.length(c.top), "R" + this.length(c.bottom),
               "Q" + this.length(Math.hypot(c.shift_x, c.shift_y)));
    if (c.dwell > 0) {
      words.push(`P${this.formats.whole.format(c.dwell * 1000)}`);
    }
    this.lastFeed = null;
    words.push(this.feedWord(c.feed));
    out.block(...words);
    this.position.z = c.retract;
    this.axisWords[2].reset();
    this.axisWords[2].of(this.toUnit(c.retract));
  }

  cycleCancel(i) {
    out.block("G80");
    this.activeCycle = null;
    this.motion = null;
  }

  compensationOn(i) {
    this.selectPlane();
    this.pending.push(i.side === "left" ? "G41" : "G42", this.expand(this.cutterCompOffset,
                                                                     i.tool, ""));
  }

  compensationOff(i) {
    this.selectPlane();
    this.pending.push("G40");
  }

  index(i) {
    const words = this.takePending();
    words.unshift(this.motionWord("G0"));
    this.rotary(i, true, words);
    out.block(...words);
  }

  tcpOff(i) {
    this.flushPending();
    out.block(this.tcpOffBlock);
  }

  homeZ(i) {
    this.flushPending();
    for (const line of this.homeZBlocks) {
      out.block(line);
    }
    this.axisWords[2].reset(); // Z is at the reference now
    this.motion = null;
  }

  // -- Probing (DL-0090) ----------------------------------------------------------------------
  //
  // A probing cycle arrives as one `probe` instruction with its moves. This base writes them as
  // plain G-code: each move a skip move (probeSkip), the protected positioning moves at the
  // cycle's positioning feed and the measuring moves at its measuring feed. Plain G-code cannot
  // use what the probe measured, so the work offset is not set and the check not made: a post
  // for a machine with a probe overrides the hooks below to call its probe's macro programs.
  // Their numbers and arguments are in the probe maker's manual; OxCAM ships none.
  //
  //   probeOn(cycle)                before the first cycle after a tool change
  //   probeBegin(cycle)             each cycle: a comment naming it
  //   probePosition(move, cycle)    each protected positioning move
  //   probeMeasure(move, cycle)     each measuring move
  //   probeEnd(cycle)               each cycle: where the work offset is set or the check made
  //   probeOff()                    before the next tool change, and at the program's end

  probe(i) {
    // A cycle that sets a work offset needs a probeEnd that applies the measurement: without
    // one the program would probe, then cut on the old offset. The base's own probeEnd says the
    // post has none.
    if (!i.check && this.probeEnd === FanucMill.prototype.probeEnd) {
      const op = this.program.operations[i.operation];
      error(`${op ? op.name : "a probing operation"}: it sets ` +
            `${this.workOffsetCode(i.work_offset)} ${i.axes} from what the probe measures, ` +
            `which ${this.program.post.name} cannot do: the post needs a probeEnd that applies ` +
            "the result (docs/posts/fanuc-base.md, Probing)");
    }
    this.selectPlane();
    this.flushPending();
    if (!this.probeIsOn) {
      this.probeIsOn = true;
      this.probeOn(i);
    }
    this.probeBegin(i);
    for (const move of i.moves) {
      if (move.kind === "measure") {
        this.probeMeasure(move, i);
      } else {
        this.probePosition(move, i);
      }
    }
    this.probeEnd(i);
    // Skip moves and macro programs may stop anywhere and leave their own modal state: state the
    // motion, every axis and the feed again in the next block.
    this.motion = null;
    this.axisWords.forEach((w) => w.reset());
    this.lastFeed = null;
  }

  /// Calls probeOff() once the probe's cycles are done (a tool change, the program's end).
  endProbing() {
    if (this.probeIsOn) {
      this.probeIsOn = false;
      this.probeOff();
    }
  }

  /// Switches the probe on (for example a macro call or an M code). Nothing here.
  probeOn(cycle) {}

  /// Switches the probe off. Nothing here.
  probeOff() {}

  /// A comment naming the cycle: what it measures, and what it sets or checks.
  probeBegin(cycle) {
    out.comment(`PROBE ${this.probeFeatureName(cycle)}: ${this.probeResultText(cycle)}`);
  }

  probePosition(move, cycle) {
    this.skipMove(move);
  }

  probeMeasure(move, cycle) {
    this.skipMove(move);
  }

  /// Here a post sets the work offset from the measurement, or checks it. Plain G-code cannot.
  /// This base's own probeEnd is reached for checks only (probe() refuses a cycle that sets a
  /// work offset while it is not overridden): it says the check is not made, in the program and
  /// once as a warning.
  probeEnd(cycle) {
    out.comment(`PROBE RESULT NOT USED: ${this.probeResultText(cycle)}`);
    if (!this.probeWarned) {
      this.probeWarned = true;
      warning(`${this.program.post.name} writes the moves of feature checks only: it does not ` +
              "compare what the probe measures with the tolerance. Use a post that calls your " +
              "probe's macro programs (probeEnd)");
    }
  }

  /// "BORE", "OUTSIDE CORNER", ...
  probeFeatureName(cycle) {
    return cycle.feature.replaceAll("_", " ").toUpperCase();
  }

  /// "SET G54 XY" or "CHECK WITHIN 0.05".
  probeResultText(cycle) {
    if (cycle.check) {
      return `CHECK WITHIN ${this.length(cycle.tolerance)}`;
    }
    return `SET ${this.workOffsetCode(cycle.work_offset)} ${cycle.axes}`;
  }

  /// A skip move to `move` (x, y, z, feed). It may end early, so where the tool is afterwards
  /// is not known: every axis is written again in the next move.
  skipMove(move) {
    const words = this.takePending();
    words.push(this.probeSkip);
    this.axisWords.forEach((w) => w.reset());
    this.axes(move, false, words);
    this.lastFeed = null;
    words.push(this.feedWord(move.feed));
    out.block(...words);
    this.axisWords.forEach((w) => w.reset());
    this.motion = null; // the skip function is one block only
  }
}

/// Generic Fanuc-style 2-axis lathe: X as diameter, G96/G97 spindle, feed per revolution.
export class FanucLathe extends FanucMill {
  constructor(context) {
    super(context);
    this.lathe = true;
    this.feedPerRev = true;
    this.safeStart = "G18 G21 G40 G80 G99";
    this.toolChangeBlocks = ["T{tool2}{tool2} ({tool_name})"];
    this.toolLengthOffset = ""; // the T word applies the offsets
    this.cutterCompOffset = "";
    this.homeZBlocks = ["G28 U0.", "G28 W0."]; // a turret set to index at home (DL-0067)
    this.footer = ["G28 U0.", "G28 W0.", "M30", "%"];
    // Live tooling: these codes are each machine builder's, so they come from the post's options
    // (`liveToolOn` holds {rpm} for the speed). Empty means not set, and live tooling is refused.
    const live = context.options ?? {};
    this.liveToolOn = live.liveToolOn ?? "";
    this.liveToolOff = live.liveToolOff ?? "";
    this.cAxisOn = live.cAxisOn ?? "";
    this.cAxisOff = live.cAxisOff ?? "";
    // A rigid tap's blocks, separated by "|": the codes are each builder's too.
    this.liveTapAxial = live.liveTapAxial ?? "";
    this.liveTapRadial = live.liveTapRadial ?? "";
    this.liveTapAxialLeft = live.liveTapAxialLeft ?? "";
    this.liveTapRadialLeft = live.liveTapRadialLeft ?? "";
    // The cycle for a left-hand thread on the spindle's axis, which controls differ on.
    this.tapLeft = live.tapLeft ?? "";
    // Polar interpolation: its blocks, the letter for the Y it programs, and whether X is a
    // diameter there as it is elsewhere on a lathe. Controls differ on all of them.
    this.polarOn = live.polarOn ?? "";
    this.polarOff = live.polarOff ?? "";
    this.polarY = live.polarY ?? "Y";
    this.polarXDiameter = live.polarXDiameter === true || live.polarXDiameter === "true";
    this.polarMode = false;
    // Cylindrical interpolation: its blocks, with {r} the cylinder's radius and {d} its diameter,
    // and how a position around it is written: the letter, as a length on the cylinder or in degrees.
    this.wrapOn = live.wrapOn ?? "";
    this.wrapOff = live.wrapOff ?? "";
    this.wrapLetter = live.wrapLetter ?? "C";
    this.wrapDegrees = live.wrapDegrees === undefined || live.wrapDegrees === true ||
                       live.wrapDegrees === "true";
    this.wrapRadius = 0;
    // Bar work (DL-0091): the chuck's and the bar feeder's codes are each builder's too, and how
    // the program ends: "end" (M30), "repeat" (M99: again until stopped, as by the bar feeder at the
    // end of a bar) or "count" (M99 until `partCount` parts, counted in the macro variable
    // `partCounter`, then M30).
    this.chuckOpen = live.chuckOpen ?? "";
    this.chuckClose = live.chuckClose ?? "";
    this.barFeederOn = live.barFeederOn ?? "";
    this.barFeederOff = live.barFeederOff ?? "";
    this.partLoop = live.partLoop ?? "end";
    this.partCount = Number(live.partCount ?? 0);
    this.partCounter = live.partCounter ?? "";
    this.stoppedFeeds = false; // feeds per minute while the spindle stands still
    // The sub-spindle (ADR-0007): each builder's codes, from the options. `subZ` says how its
    // work's Z is written: "mirror" (negated, arcs turned the other way: the work's Z runs towards
    // the main spindle, the machine's away from it) or "as_is" (the control mirrors it, by the
    // codes that select the sub-spindle).
    this.subSpindleSelect = live.subSpindleSelect ?? "";
    this.mainSpindleSelect = live.mainSpindleSelect ?? "";
    this.subChuckOpen = live.subChuckOpen ?? "";
    this.subChuckClose = live.subChuckClose ?? "";
    this.syncOn = live.syncOn ?? "";
    this.syncAngleOn = live.syncAngleOn ?? "";
    this.syncOff = live.syncOff ?? "";
    this.z2Letter = live.z2Letter ?? "";
    this.subZ = live.subZ ?? "mirror";
    this.onSub = false;
    this.plane = "ZX"; // the program's start says G18
    this.feedPerMinute = "G98";
    this.feedPerRevolution = "G99";
    this.latheY = false;
  }

  /// The chuck opens or closes (DL-0091), the main spindle's or the sub-spindle's (ADR-0007), by
  /// the post's options.
  chuck(i) {
    this.flushPending();
    const sub = i.spindle === "sub";
    const code = sub ? (i.open ? this.subChuckOpen : this.subChuckClose)
                     : (i.open ? this.chuckOpen : this.chuckClose);
    if (!code) {
      const name = sub ? `"Sub-spindle chuck open" and "Sub-spindle chuck close"`
                       : `"Chuck open" and "Chuck close"`;
      error(`${this.program.post.name}: the chuck's codes differ between machine builders: set ` +
            `${name} in the post's options, or use a post for your control`);
    }
    for (const block of code.split("|")) out.block(block.trim());
  }

  /// The work on the main spindle or on the sub-spindle (ADR-0007): the codes that select it when
  /// it changes.
  operationBegin(op) {
    const sub = op.spindle === "sub";
    if (sub !== this.onSub) {
      this.flushPending();
      const code = sub ? this.subSpindleSelect : this.mainSpindleSelect;
      if (!code) {
        error(`${this.program.post.name}: selecting the sub-spindle has its own codes on each ` +
              `machine: set "Select the sub-spindle" and "Select the main spindle" in the post's ` +
              `options, or use a post for your control`);
      }
      for (const block of code.split("|")) out.block(block.trim());
      this.onSub = sub;
      this.motion = null;
      this.axisWords.forEach((w) => w.reset());
    }
    super.operationBegin(op);
  }

  /// Z2 moves, by the letter the control moves it with: at rapid, or at a feed per minute.
  subAxis(i) {
    this.flushPending();
    if (!this.z2Letter) {
      error(`${this.program.post.name}: the sub-spindle's axis has its own letter on each machine: ` +
            `set "Letter for Z2" in the post's options, or use a post for your control`);
    }
    const z2 = `${this.z2Letter}${this.formats.linear.format(this.toUnit(i.z2))}`;
    if (i.feed == null) {
      out.block("G0", z2);
    } else {
      if (this.feedPerRev) {
        out.block(this.feedPerMinute);
        this.feedPerRev = false;
        this.stoppedFeeds = true;
      }
      out.block("G1", z2, "F" + this.formats.feed.format(this.toUnit(i.feed)));
    }
    this.motion = null;
    this.lastFeed = null;
  }

  /// The spindles synchronised, at an angle or not, or no longer: by the post's options.
  spindleSync(i) {
    this.flushPending();
    const code = !i.on ? this.syncOff : i.angle == null ? this.syncOn : this.syncAngleOn;
    if (!code) {
      const name = !i.on ? "Spindles synchronised off"
                 : i.angle == null ? "Spindles synchronised on" : "Spindles synchronised at an angle";
      error(`${this.program.post.name}: synchronising the spindles has its own codes on each ` +
            `machine: set "${name}" in the post's options, or use a post for your control`);
    }
    const text = code.replaceAll("{angle}", this.formats.rotary.format(i.angle ?? 0));
    for (const block of text.split("|")) out.block(block.trim());
  }

  /// A position of the sub-spindle's work as the program writes it: Z negated, and arcs turned,
  /// unless the control mirrors it.
  subWork(i) {
    if (!this.onSub || this.subZ === "as_is") return i;
    if (this.subZ !== "mirror") {
      error(`${this.program.post.name}: the sub-spindle's Z is "mirror" or "as_is", not ` +
            `"${this.subZ}"`);
    }
    const m = { ...i };
    if (m.z != null) m.z = -m.z;
    if (m.cz != null) m.cz = -m.cz;
    if (m.clockwise !== undefined) m.clockwise = !m.clockwise;
    return m;
  }

  rapid(i) {
    super.rapid(this.subWork(i));
  }

  arc(i) {
    super.arc(this.subWork(i));
  }

  toolLength(i) {
    super.toolLength(this.subWork(i));
  }

  /// The bar feeder pushes or stops (DL-0091), by the post's options.
  barFeeder(i) {
    this.flushPending();
    const code = i.on ? this.barFeederOn : this.barFeederOff;
    if (!code) {
      error(`${this.program.post.name}: the bar feeder's codes differ between machines: set ` +
            `"Bar feeder on" and "Bar feeder off" in the post's options, or use a post for your ` +
            `control`);
    }
    for (const block of code.split("|")) out.block(block.trim());
  }

  /// With the spindle standing still (a bar pull), feed per revolution means nothing: feeds go per
  /// minute until it turns again.
  linear(input) {
    const i = this.subWork(input);
    if (this.feedPerRev && this.rpm <= 0 && !(i.pitch > 0)) {
      out.block(this.feedPerMinute);
      this.feedPerRev = false;
      this.stoppedFeeds = true;
      this.lastFeed = null;
    }
    super.linear(i);
  }

  spindle(i) {
    if (this.stoppedFeeds && i.rpm > 0) {
      out.block(this.feedPerRevolution);
      this.feedPerRev = true;
      this.stoppedFeeds = false;
      this.lastFeed = null;
    }
    super.spindle(i);
  }

  /// The program's end, as the part loop says: the footer, or M99 to run again for the next part.
  programEnd(program) {
    this.flushPending();
    if (this.partLoop === "end") {
      super.programEnd(program);
      return;
    }
    if (this.partLoop !== "repeat" && this.partLoop !== "count") {
      error(`${this.program.post.name}: the part loop is "end", "repeat" or "count", not ` +
            `"${this.partLoop}"`);
    }
    // The footer up to its M30 returns the machine home; then M99 runs the program again.
    const footer = this.footer.map((line) => this.expand(line, 0, ""));
    const end = footer.indexOf("M30");
    const home = footer.slice(0, end < 0 ? footer.length : end)
                       .filter((line) => line !== "%" && line !== "");
    for (const line of home) out.block(line);
    if (this.partLoop === "count") {
      if (!(this.partCount >= 1) || !/^#[0-9]+$/.test(this.partCounter)) {
        error(`${this.program.post.name}: counting parts needs "Parts to run" (1 or more) and ` +
              `"Part counter", a macro variable free on your machine (as #500)`);
      }
      // Fanuc custom macro: count this part; at the count, reset and end at N9999.
      const v = this.partCounter;
      out.block(`${v}=${v}+1`);
      out.block(`IF[${v}GE${this.formats.whole.format(this.partCount)}]GOTO9999`);
      out.block("M99");
      out.line(`N9999 ${v}=0`);
      out.block("M30");
    } else {
      out.block("M99");
    }
    out.line("%");
  }

  /// A move on a lathe stays in the program's plane; only arcs and a live tool choose another.
  selectPlane(plane = this.plane) {
    super.selectPlane(plane);
  }

  /// Polar interpolation on or off, from the post's options.
  polar(i) {
    this.flushPending();
    if (i.on) {
      if (!this.polarOn || !this.polarOff) {
        error(`${this.program.post.name}: polar interpolation has its own codes on each control: ` +
              `set "Polar interpolation on" and "off" in the post's options, or use a post for ` +
              `your control`);
      }
      for (const block of this.polarOn.split("|")) out.block(block.trim());
      this.polarMode = true;
      this.polarYWord = word(this.polarY, this.formats.linear);
      this.position = { x: i.x, y: i.y, z: i.z };
      this.plane = "XY";
    } else {
      for (const block of this.polarOff.split("|")) out.block(block.trim());
      this.polarMode = false;
      this.plane = "ZX";
    }
    this.motion = null;
    this.axisWords.forEach((w) => w.reset());
    this.rotaryWords.C.reset();
  }

  axes(target, forceXY, words) {
    if (!this.polarMode) {
      super.axes(target, forceXY, words);
      return;
    }
    const w = this.axisWords;
    if (target.x != null) {
      if (forceXY) w[0].reset();
      const t = w[0].of(this.toUnit(this.polarXDiameter ? 2 * target.x : target.x));
      if (t !== "") words.push(t);
      this.position.x = target.x;
    }
    if (target.y != null) {
      if (forceXY) this.polarYWord.reset();
      const t = this.polarYWord.of(this.toUnit(target.y));
      if (t !== "") words.push(t);
      this.position.y = target.y;
    }
    if (target.z != null) {
      const t = w[2].of(this.toUnit(target.z));
      if (t !== "") words.push(t);
      this.position.z = target.z;
    }
  }

  rotary(i, force, words) {
    if (this.polarMode) return;
    if (this.wrapRadius > 0 && i.axes) {
      const value = this.wrapDegrees ? i.c : this.toUnit((i.c * Math.PI / 180) * this.wrapRadius);
      if (force) this.wrapWord.reset();
      const t = this.wrapWord.of(value);
      if (t !== "") words.push(t);
      return;
    }
    super.rotary(i, force, words);
  }

  /// Cylindrical interpolation on or off, from the post's options.
  wrap(i) {
    this.flushPending();
    if (i.on) {
      if (!this.wrapOn || !this.wrapOff) {
        error(`${this.program.post.name}: cylindrical interpolation has its own codes on each ` +
              `control: set "Cylindrical interpolation on" and "off" in the post's options, or ` +
              `use a post for your control`);
      }
      const text = this.wrapOn
        .replaceAll("{r}", this.formats.linear.format(this.toUnit(i.radius)))
        .replaceAll("{d}", this.formats.linear.format(this.toUnit(2 * i.radius)));
      for (const block of text.split("|")) out.block(block.trim());
      this.wrapRadius = i.radius;
      this.wrapWord = word(this.wrapLetter, this.wrapDegrees ? this.formats.rotary : this.formats.linear);
    } else {
      for (const block of this.wrapOff.split("|")) out.block(block.trim());
      this.wrapRadius = 0;
    }
    this.motion = null;
    this.axisWords.forEach((w) => w.reset());
    this.rotaryWords.C.reset();
  }

  /// A driven tool on: feed per minute (the part stands still, so per revolution means nothing),
  /// the spindle a C axis, the tool turning. Off: all that back.
  liveTool(i) {
    this.flushPending();
    this.stoppedFeeds = false; // the live tool sets the feed mode itself
    if (i.rpm > 0 && i.tapping) {
      // The tap's own block starts the tool, from its speed, and feeds per revolution.
      if (!this.cAxisOn) {
        error(`${this.program.post.name}: a live tool needs your machine's codes, which differ ` +
              `between machine builders: set "C axis on" in the post's options, or use a post ` +
              `for your control`);
      }
      out.block(this.feedPerRevolution);
      out.block(this.cAxisOn);
    } else if (i.rpm > 0) {
      if (!this.liveToolOn || !this.cAxisOn) {
        error(`${this.program.post.name}: a live tool needs your machine's codes, which differ ` +
              `between machine builders: set "Live tool on" and "C axis on" in the post's options, ` +
              `or use a post for your control`);
      }
      this.feedPerRev = false;
      this.lastFeed = null;
      out.block(this.feedPerMinute);
      out.block(this.cAxisOn);
      out.block(this.liveToolOn.replaceAll("{rpm}", this.formats.whole.format(i.rpm)));
      this.arcFormat = "r"; // both of the manuals' examples, and no diameter to mistake in I
      if (i.plane) {
        out.block(PLANES[i.plane].code);
        this.plane = i.plane;
      }
    } else {
      out.block(this.liveToolOff);
      out.block(this.cAxisOff);
      out.block(this.feedPerRevolution);
      this.arcFormat = "ijk";
      if (this.plane !== "ZX") {
        out.block(PLANES.ZX.code);
        this.plane = "ZX";
      }
      this.feedPerRev = true;
      this.lastFeed = null;
      this.rotaryWords.C.reset();
    }
  }

  /// A rigid tap with a driven tool. The blocks are the post's options, with what they need in
  /// {rpm}, {lead}, {x} (diameter), {y}, {z}, {c} and {r} (the R plane, along Z or in X's diameter).
  liveTap(i) {
    this.flushPending();
    const template = i.left_hand ? (i.radial ? this.liveTapRadialLeft : this.liveTapAxialLeft)
                                 : (i.radial ? this.liveTapRadial : this.liveTapAxial);
    if (!template) {
      const name = (i.radial ? "Live tap across the part" : "Live tap along Z") +
                   (i.left_hand ? ", left hand" : "");
      error(`${this.program.post.name}: a rigid tap with a live tool has its own codes on each ` +
            `control: set "${name}" in the post's options, or use a post for your control`);
    }
    const f = this.formats;
    const len = (v) => f.linear.format(this.toUnit(v));
    const text = template
      .replaceAll("{rpm}", f.whole.format(i.rpm))
      .replaceAll("{lead}", f.lead.format(this.toUnit(i.pitch)))
      .replaceAll("{x}", len(2 * i.x))
      .replaceAll("{y}", len(i.y ?? 0))
      .replaceAll("{z}", len(i.z))
      .replaceAll("{c}", f.rotary.format(i.c ?? 0))
      .replaceAll("{r}", len(i.radial ? 2 * i.r_plane : i.r_plane));
    for (const block of text.split("|")) {
      out.block(block.trim());
    }
    this.motion = null;
    this.lastFeed = null;
    this.axisWords.forEach((w) => w.reset());
    this.rotaryWords.C.reset();
  }

  /// The control offsets the nose by its offset page: the program says what that has to hold.
  compensationOn(i) {
    const tool = (this.program.tools ?? []).find((t) => t.number === i.tool);
    if (tool?.insert) {
      out.comment(`T${i.tool} nose radius ${this.length(tool.insert.nose_radius)} tip ` +
                  `${tool.insert.orientation} on the offset page`);
    }
    super.compensationOn(i);
  }

  /// Tapping on the spindle axis, from the R plane the tool is brought to: no R word, since
  /// controls read it differently on lathes. The cycle reverses the spindle, and in feed per
  /// revolution F is the lead. A post taking other cycles writes them itself.
  cycle(i) {
    if (this.onSub && this.subZ === "mirror") {
      error(`${this.program.post.name}: a canned cycle on the sub-spindle with its Z mirrored by ` +
            `the post: set the sub-spindle's Z to "as_is" for a control that mirrors it`);
    }
    if (i.cycle.type !== "tap" && i.cycle.type !== "tap_left") {
      super.cycle(i);
      return;
    }
    const left = i.cycle.type === "tap_left";
    if (left && !this.tapLeft) {
      error(`${this.program.post.name}: a left-hand tap on the spindle's axis has its own cycle on ` +
            `each control: set "Left-hand tap cycle" in the post's options, or use a post for ` +
            `your control`);
    }
    const code = left ? this.tapLeft : "G84";
    this.selectPlane();
    const approach = this.takePending();
    approach.unshift(this.motionWord("G0"));
    this.axes({ z: i.cycle.retract }, false, approach);
    if (approach.length > 1) {
      out.block(...approach);
    }
    const words = [];
    this.motion = code;
    words.push(code);
    this.axes({ x: i.x, y: i.y }, true, words);
    words.push("Z" + this.length(i.cycle.bottom));
    this.lastFeed = null;
    words.push(this.feedWord(i.cycle.feed));
    out.block(...words);
  }
}
