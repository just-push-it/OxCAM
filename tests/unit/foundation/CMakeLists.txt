# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

ocam_add_test(foundation
    SOURCES
        CancelProgressTest.cpp
        JsonShaTest.cpp
        LogTest.cpp
        MemoryCapTest.cpp
        ToleranceTest.cpp
        UnitsTest.cpp
        UuidTest.cpp)

if(NOT OCAM_SANITIZERS AND (WIN32 OR CMAKE_SYSTEM_NAME STREQUAL "Linux"))
    add_test(NAME foundation.memory_cap COMMAND test_foundation "[memorycap]")
    set_tests_properties(foundation.memory_cap PROPERTIES
        LABELS foundation
        ENVIRONMENT "OCAM_MEMORY_CAP_GB=1"
        PASS_REGULAR_EXPRESSION "memory cap of 1 GB")
endif()
