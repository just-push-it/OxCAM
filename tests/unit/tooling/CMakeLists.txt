# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

ocam_add_test(tooling
    SOURCES
        LibraryIoTest.cpp
        ToolTest.cpp)

target_compile_definitions(test_tooling PRIVATE
    OCAM_SAMPLE_TOOL_LIBRARY="${PROJECT_SOURCE_DIR}/data/tool-libraries/sample-metric.ocamtools")
