# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.25)

project(OxCAM
    VERSION 0.1.0
    DESCRIPTION "Clean-room open-source CAM"
    HOMEPAGE_URL "https://github.com/opencam/opencam"
    LANGUAGES CXX)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not supported. Use a preset: cmake --preset <name>")
endif()

option(OCAM_BUILD_TESTS "Build unit tests" ON)
option(OCAM_BUILD_GUI "Build the Qt 6 desktop application" OFF)
option(OCAM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(OCAM_BUILD_FUZZERS "Build libFuzzer targets (Clang only)" OFF)
option(OCAM_BUILD_PYTHON "Build the Python bindings (needs nanobind)" OFF)
set(OCAM_SANITIZERS "" CACHE STRING "Semicolon list of sanitizers (e.g. address;undefined)")
option(OCAM_KEEP_ASSERTS
       "Keep assertions (no NDEBUG; libstdc++ assertions) in optimized builds, as CI does" OFF)
if(OCAM_KEEP_ASSERTS)
    # The optimized configurations define NDEBUG by default; without it Eigen's and the other
    # libraries' asserts stay live at optimized speed. (MSVC's checked iterators cannot be turned
    # on here: the vcpkg libraries are built without them and would not link.)
    foreach(config RELEASE RELWITHDEBINFO MINSIZEREL)
        foreach(lang C CXX)
            string(REGEX REPLACE "[/-]DNDEBUG" "" CMAKE_${lang}_FLAGS_${config}
                                 "${CMAKE_${lang}_FLAGS_${config}}")
        endforeach()
    endforeach()
    if(NOT MSVC)
        add_compile_definitions(_GLIBCXX_ASSERTIONS)
    endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(CompilerWarnings)
include(Sanitizers)
include(ModuleGraph)
include(OpenCAMHelpers)

# Imported targets are directory-scoped, so OpenCASCADE is found here for libs/cad, the GUI and
# the fixture generator alike.
find_package(OpenCASCADE CONFIG REQUIRED)

# Generated test fixtures (STEP files and a sample project), used by tests in several directories.
set(OCAM_FIXTURE_DIR ${PROJECT_BINARY_DIR}/fixtures)

if(OCAM_BUILD_TESTS)
    enable_testing()
    find_package(Catch2 3 CONFIG REQUIRED)
    include(Catch)
endif()

add_subdirectory(libs)
add_subdirectory(apps)

if(OCAM_BUILD_PYTHON)
    add_subdirectory(bindings/python)
endif()

if(OCAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(OCAM_BUILD_FUZZERS)
    add_subdirectory(tests/fuzz)
endif()

include(Packaging)
