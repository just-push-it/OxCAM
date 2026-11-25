# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Python bindings (nanobind, BSD-3-Clause). nanobind comes from a CMake package (for example
# vcpkg's "nanobind" port) or from the Python environment: pip install nanobind.

find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(nanobind CONFIG QUIET)
if(NOT nanobind_FOUND)
    execute_process(
        COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
        OUTPUT_VARIABLE nanobind_ROOT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE nanobind_query)
    if(NOT nanobind_query EQUAL 0)
        message(FATAL_ERROR "OCAM_BUILD_PYTHON needs nanobind: pip install nanobind "
                            "(for ${Python_EXECUTABLE}) or provide a nanobind CMake package")
    endif()
    find_package(nanobind CONFIG REQUIRED)
endif()

set(OCAM_PYTHON_DIR ${PROJECT_BINARY_DIR}/python/oxcam)
nanobind_add_module(_oxcam NB_STATIC src/module.cpp)
target_link_libraries(_oxcam PRIVATE ocam::document)
set_target_properties(_oxcam PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${OCAM_PYTHON_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${OCAM_PYTHON_DIR})
configure_file(oxcam/__init__.py ${OCAM_PYTHON_DIR}/__init__.py COPYONLY)

include(GNUInstallDirs)
install(TARGETS _oxcam LIBRARY DESTINATION python/oxcam RUNTIME DESTINATION python/oxcam)
install(FILES oxcam/__init__.py DESTINATION python/oxcam)

if(OCAM_BUILD_TESTS)
    add_test(NAME python.bindings
        COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_oxcam.py
                ${OCAM_FIXTURE_DIR} ${PROJECT_SOURCE_DIR}/data)
    set_tests_properties(python.bindings PROPERTIES
        ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/python;OXCAM_DLL_DIR=${PROJECT_BINARY_DIR}/bin"
        LABELS python
        TIMEOUT 600)
endif()
