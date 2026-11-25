# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

ocam_add_library(geom3d
    SOURCES
        src/Bvh.cpp
        src/Convex.cpp
        src/AxisFit.cpp
        src/Mesh.cpp
        src/Stl.cpp
    PUBLIC_DEPS
        ocam::foundation)
