# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

ocam_add_test(post
    SOURCES BackBorePostTest.cpp BuiltinPostTest.cpp EngineTest.cpp MultiAxisPostTest.cpp
            NcProgramTest.cpp
            OutputLibraryTest.cpp PostTest.cpp ProbingPostTest.cpp
    DEPS ocam::strategies)
target_compile_definitions(test_post PRIVATE
    OCAM_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
    OCAM_GOLDEN_DIR="${PROJECT_SOURCE_DIR}/tests/golden")
