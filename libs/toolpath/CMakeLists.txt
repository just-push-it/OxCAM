# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

ocam_add_library(toolpath
    SOURCES
        src/PathBuilder.cpp
        src/Smoothing.cpp
        src/Toolpath.cpp
    PUBLIC_DEPS
        ocam::foundation
        ocam::geom2d
        ocam::geom3d
        ocam::tooling)
