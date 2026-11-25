# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""Smoke tests of the Python bindings: python test_oxcam.py <fixture dir> <data dir>."""

import os
import sys

import oxcam


def check(condition, message):
    if not condition:
        raise AssertionError(message)
    print("ok:", message)


def main(fixtures, data):
    sample = os.path.join(fixtures, "sample_project.ocam")
    check(oxcam.__version__.count(".") == 2, f"version {oxcam.__version__}")

    doc = oxcam.Document.open(sample)
    setups = doc.setups
    check(len(setups) == 1 and len(setups[0]["operations"]) == 3, "sample project has 3 operations")
    check(len(doc.tools) == 2, "sample project has 2 tools")

    toolpaths = doc.toolpaths()
    check(all("toolpath" in t and t["stats"]["moves"] > 0 for t in toolpaths), "every operation generates")

    report = doc.simulate(resolution=0.25)
    check(report["comparison"]["gouge_volume"] == 0.0, "simulation finds no gouges")
    check(abs(report["final_volume"] - report["part_volume"]) < 0.005 * report["part_volume"],
          "simulated stock matches the part")

    gcode = doc.post()
    check(gcode.startswith("%") and "M30" in gcode, "built-in post produces a program")
    check(doc.post(post="fanuc-generic-mill") == gcode, "the mill post by id gives the same")
    check(set(oxcam.builtin_posts()) == {"fanuc-generic-mill", "fanuc-generic-mill-4axis",
                                         "fanuc-generic-lathe", "haas-classic-mill"},
          "built-in posts are listed")
    haas = doc.post(post="haas-classic-mill")
    check(haas.startswith("%\nO0") and "M30" in haas, "the Haas-style post by id")

    # What a post receives.
    program = doc.nc_program()
    check(program["format"] == "oxcam-ncprogram" and program["version"] == 2
          and program["instructions"] and program["operations"] and program["tools"],
          "NC program dict, version 2")

    # A JavaScript post file with an option.
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "stop-post.js")
        with open(path, "w", encoding="utf-8") as f:
            f.write("""
import { out } from "oxcam:post";
import { FanucMill } from "oxcam:fanuc";
export const manifest = { id: "stop-post", name: "Stop post", version: "1", engine: 1,
                          machines: ["mill3"] };
export const options = [{ id: "optionalStop", label: "Optional stop", type: "bool",
                          default: false }];
export default class extends FanucMill {
  toolChange(i) {
    if (this.options.optionalStop) out.block("M01");
    super.toolChange(i);
  }
}
""")
        check(doc.post(post=path) == gcode, "a JavaScript post file runs")
        stopped = doc.post(post=path, options={"optionalStop": True})
        check(" M01" in stopped and stopped != gcode, "post options reach the post")
        try:
            doc.post(post=path, options={"nope": 1})
        except oxcam.Error as e:
            check("nope" in str(e), "unknown post options raise oxcam.Error")
        else:
            raise AssertionError("an unknown option did not raise")

    # Turning: the shaft project posts with the lathe post.
    shaft = oxcam.Document.open(os.path.join(fixtures, "shaft_project.ocam"))
    lathe_gcode = shaft.post()
    check("G96" in lathe_gcode and "G32" in lathe_gcode, "lathe post: constant surface speed and threads")

    # 4th axis: machine positions with inverse-time feeds.
    rotary = oxcam.Document.open(os.path.join(fixtures, "rotary_project.ocam"))
    rotary_gcode = rotary.post()
    check("G93" in rotary_gcode and " A" in rotary_gcode, "4-axis post: A words, inverse time")
    check(oxcam.backplot(rotary_gcode)["stats"]["cut_length"] > 0, "4-axis program back-plots")

    # 5 axes with tool centre point control.
    five = oxcam.Document.open(os.path.join(fixtures, "dome5_project.ocam"))
    five_text = five.post()
    check("G43.4" in five_text and "G49" in five_text and " C" in five_text,
          "tool centre point control and A/C motion")

    lib = oxcam.load_tool_library(os.path.join(data, "tool-libraries", "sample-metric.ocamtools"))
    check(len(lib["tools"]) == 8 and not lib["problems"], "sample tool library loads")

    try:
        oxcam.Document.open(os.path.join(fixtures, "missing.ocam"))
    except oxcam.Error as e:
        check("missing" in str(e) or e, "missing files raise oxcam.Error")
    else:
        raise AssertionError("opening a missing file did not raise")
    print("all Python binding checks passed")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
