# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

ocam_add_test(strategies
    SOURCES
        AvoidanceTest.cpp
        BackEdgeTest.cpp
        ClearingTest.cpp
        HoleOpsTest.cpp
        MultiAxisTest.cpp
        PocketPropertyTest.cpp
        RotaryContourTest.cpp
        ProbingTest.cpp
        StrategiesTest.cpp
        SurfaceTest.cpp
        TraceTest.cpp
        SwarfTest.cpp
        StrategyTestUtil.cpp)

# Engagement control is internal to the strategies library: its test reads the private header.
target_include_directories(test_strategies PRIVATE ${PROJECT_SOURCE_DIR}/libs/strategies/src)
target_compile_definitions(test_strategies PRIVATE OCAM_GOLDEN_DIR="${PROJECT_SOURCE_DIR}/tests/golden")
