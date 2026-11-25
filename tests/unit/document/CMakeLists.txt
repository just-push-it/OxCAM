# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

ocam_add_test(document
    SOURCES
        BackEdgeDocTest.cpp
        BarPullTest.cpp
        DocumentTest.cpp
        FixturePostTest.cpp
        GenerateTest.cpp
        LegacyFormatTest.cpp
        MachineImportTest.cpp
        MachineStepTest.cpp
        PlacementTest.cpp
        ProbingTest.cpp
        ProjectTest.cpp
        RobustnessTest.cpp
        LatheHoleTest.cpp
        LatheMachineTest.cpp
        LiveMillTest.cpp
        LatheStockTest.cpp
        SimulateTest.cpp
        SwarfTest.cpp
        SmoothStockBench.cpp
        TransferTest.cpp
        TurningBench.cpp
        WcsTest.cpp
        WorkholdingTest.cpp
        WrappedContourTest.cpp)

target_include_directories(test_document PRIVATE ${PROJECT_SOURCE_DIR}/tests/fixtures/gen)
target_compile_definitions(test_document PRIVATE OCAM_FIXTURE_DIR="${OCAM_FIXTURE_DIR}"
    OCAM_SOURCE_DIR="${PROJECT_SOURCE_DIR}" OCAM_GOLDEN_DIR="${PROJECT_SOURCE_DIR}/tests/golden")
add_dependencies(test_document fixtures)
