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

# The dexel grid stores heights as float. Widening them to the double math types
# is this library's boundary, and -Wdouble-promotion flags every one of those reads.
# The rest of the project keeps the warning.
if(NOT MSVC)
    target_compile_options(ocam_sim PRIVATE -Wno-double-promotion)
endif()
