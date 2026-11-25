# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""OxCAM: open-source CAM, from Python.

    import oxcam
    doc = oxcam.Document.open("part.ocam")
    report = doc.simulate()
    gcode = doc.post()                           # the machine's built-in post
    gcode = doc.post(post="my-post.js",          # or a JavaScript post file (ADR-0006)
                     options={"optionalStop": True})
    program = doc.nc_program()                   # what a post receives, as a dict

Results are plain dicts and lists in the shapes of OxCAM's JSON formats.
"""

import os as _os

if _os.name == "nt":
    # The extension links OpenCASCADE, oneTBB and friends dynamically: look for their DLLs next
    # to the package, in an installed ../../bin, or in OXCAM_DLL_DIR.
    _here = _os.path.dirname(_os.path.abspath(__file__))
    for _d in (_os.environ.get("OXCAM_DLL_DIR"), _here, _os.path.join(_here, "..", "..", "bin")):
        if _d and _os.path.isdir(_d):
            _os.add_dll_directory(_os.path.abspath(_d))

from ._oxcam import (  # noqa: E402
    Document,
    Error,
    __version__,
    backplot,
    builtin_posts,
    load_tool_library,
)

__all__ = [
    "Document",
    "Error",
    "__version__",
    "backplot",
    "builtin_posts",
    "load_tool_library",
]
