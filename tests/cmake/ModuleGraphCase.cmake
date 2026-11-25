# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Script-mode test driver for cmake/ModuleGraph.cmake.
# Usage: cmake -DSOURCE_DIR=<repo> -DCASE=<name> -P ModuleGraphCase.cmake
# "allowed*" cases must succeed; every other case must fail with FATAL_ERROR.

cmake_minimum_required(VERSION 3.25) # script mode starts with no policies set

include(${SOURCE_DIR}/cmake/ModuleGraph.cmake)

if(CASE STREQUAL "allowed_layering")
    ocam_check_module_deps(strategies ocam::foundation ocam::geom2d ocam::toolpath Eigen3::Eigen)
elseif(CASE STREQUAL "allowed_occt_in_cad")
    ocam_check_module_deps(cad ocam::geom2d TKernel TKSTEP)
elseif(CASE STREQUAL "upward_edge")
    ocam_check_module_deps(foundation ocam::geom2d)
elseif(CASE STREQUAL "skip_layer")
    ocam_check_module_deps(strategies ocam::sim)
elseif(CASE STREQUAL "occt_outside_cad")
    ocam_check_module_deps(strategies TKBRep)
elseif(CASE STREQUAL "qt_in_library")
    ocam_check_module_deps(document Qt6::Core)
elseif(CASE STREQUAL "unknown_module")
    ocam_check_module_deps(notamodule ocam::foundation)
else()
    message(FATAL_ERROR "unknown CASE '${CASE}'")
endif()
