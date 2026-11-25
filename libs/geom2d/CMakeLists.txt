# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

find_package(Clipper2 CONFIG REQUIRED)

ocam_add_library(geom2d
    SOURCES
        src/Clip.cpp
        src/Contour.cpp
        src/Ordering.cpp
        src/PathOps.cpp
        src/Polyline.cpp
        src/Region.cpp
    PUBLIC_DEPS
        ocam::foundation
    PRIVATE_DEPS
        Clipper2::Clipper2)
