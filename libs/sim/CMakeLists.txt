# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

find_package(TBB CONFIG REQUIRED)

ocam_add_library(sim
    SOURCES
        src/Compare.cpp
        src/Dexel.cpp
        src/Lathe.cpp
        src/LatheCuts.cpp
        src/LatheMachine.cpp
        src/FixtureCheck.cpp
        src/MachineCheck.cpp
        src/Rest.cpp
        src/Simulate.cpp
        src/StockCodec.cpp
        src/Surface.cpp
        src/Tilted.cpp
        src/ToolShape.cpp
    PUBLIC_DEPS
        ocam::foundation
        ocam::geom2d
        ocam::geom3d
        ocam::machine
        ocam::tooling
        ocam::toolpath
    PRIVATE_DEPS
        TBB::tbb)
