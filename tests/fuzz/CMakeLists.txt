# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later
#
# libFuzzer targets (Clang only). Configure with the linux-clang-fuzz preset, which also builds
# the libraries with -fsanitize=fuzzer-no-link for coverage. Run, for example:
#   build/linux-clang-fuzz/bin/fuzz_toollib -max_total_time=300 data/tool-libraries

if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR "OCAM_BUILD_FUZZERS requires Clang (libFuzzer)")
endif()

foreach(name ToolLibrary Project Archive)
    string(TOLOWER ${name} lower)
    if(lower STREQUAL "toollibrary")
        set(lower toollib)
    endif()
    add_executable(fuzz_${lower} Fuzz${name}.cpp)
    target_link_libraries(fuzz_${lower} PRIVATE ocam::document)
    target_compile_options(fuzz_${lower} PRIVATE -fsanitize=fuzzer,address,undefined)
    target_link_options(fuzz_${lower} PRIVATE -fsanitize=fuzzer,address,undefined)
endforeach()
