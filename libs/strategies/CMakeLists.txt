# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

find_package(TBB CONFIG REQUIRED)

ocam_add_library(strategies
    SOURCES
        src/Avoid.cpp
        src/BackBore.cpp
        src/Chamfer.cpp
        src/Common.cpp
        src/DropCutter.cpp
        src/EdgeBreak.cpp
        src/Clearing.cpp
        src/Medial.cpp
        src/MedialAxis.cpp
        src/Contour.cpp
        src/Drill.cpp
        src/Facing.cpp
        src/Holes.cpp
        src/Internal.cpp
        src/MultiAxis.cpp
        src/RotaryContour.cpp
        src/Plunge.cpp
        src/Pencil.cpp
        src/Probing.cpp
        src/Pocket.cpp
        src/SteepShallow.cpp
        src/Surface.cpp
        src/Trace.cpp
        src/Swarf.cpp
        src/Turning.cpp
    PUBLIC_DEPS
        ocam::foundation
        ocam::geom2d
        ocam::geom3d
        ocam::tooling
        ocam::toolpath
    PRIVATE_DEPS
        TBB::tbb)
