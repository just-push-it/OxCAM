# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

ocam_add_test(cad
    SOURCES
        AssemblyTest.cpp
        DatumTest.cpp
        ImportTest.cpp
        RevolveTest.cpp
        SectionFeatureTest.cpp
        SubShapeGeometryTest.cpp)

target_include_directories(test_cad PRIVATE ${PROJECT_SOURCE_DIR}/tests/fixtures/gen)
target_compile_definitions(test_cad PRIVATE OCAM_FIXTURE_DIR="${OCAM_FIXTURE_DIR}")
add_dependencies(test_cad fixtures)
