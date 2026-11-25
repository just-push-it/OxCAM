# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

# Allowed direct dependencies between OxCAM libraries (see docs/adr/0001-architecture.md).
# ocam_add_library() rejects any internal dependency that is not listed here, so the layering
# is checked on every configure, locally and in CI.
#
# To add an edge, update this file in the same PR and explain why in the PR description.

set(OCAM_MODULES
    foundation geom2d geom3d cad tooling toolpath strategies machine sim post document)

set(OCAM_ALLOWED_DEPS_foundation "")
set(OCAM_ALLOWED_DEPS_geom2d     foundation)
set(OCAM_ALLOWED_DEPS_geom3d     foundation)
set(OCAM_ALLOWED_DEPS_cad        foundation geom2d geom3d)
set(OCAM_ALLOWED_DEPS_tooling    foundation geom2d)
set(OCAM_ALLOWED_DEPS_toolpath   foundation geom2d geom3d tooling)
set(OCAM_ALLOWED_DEPS_strategies foundation geom2d geom3d tooling toolpath)
set(OCAM_ALLOWED_DEPS_machine    foundation geom3d toolpath)
set(OCAM_ALLOWED_DEPS_sim        foundation geom2d geom3d tooling toolpath machine)
set(OCAM_ALLOWED_DEPS_post       foundation toolpath machine)
set(OCAM_ALLOWED_DEPS_document
    foundation geom2d geom3d cad tooling toolpath strategies machine sim post)

# External libraries that only specific modules may link.
# OpenCASCADE targets are named TK* (TKernel, TKSTEP, ...).
set(OCAM_OCCT_PATTERN "^(TK[A-Za-z0-9]+|OpenCASCADE::.*)$")
set(OCAM_OCCT_ALLOWED cad)
set(OCAM_QT_PATTERN "^Qt[56]?::")

# ocam_check_module_deps(<module> <dep>...)
function(ocam_check_module_deps module)
    if(NOT module IN_LIST OCAM_MODULES)
        message(FATAL_ERROR "ModuleGraph: '${module}' is not a registered module. "
                            "Add it to OCAM_MODULES in cmake/ModuleGraph.cmake.")
    endif()

    foreach(dep IN LISTS ARGN)
        if(dep MATCHES "^ocam::(.+)$")
            set(name ${CMAKE_MATCH_1})
            if(NOT name IN_LIST OCAM_ALLOWED_DEPS_${module})
                message(FATAL_ERROR "ModuleGraph: '${module}' may not depend on '${name}'. "
                                    "Allowed: [${OCAM_ALLOWED_DEPS_${module}}]")
            endif()
        elseif(dep MATCHES "${OCAM_OCCT_PATTERN}")
            if(NOT module IN_LIST OCAM_OCCT_ALLOWED)
                message(FATAL_ERROR "ModuleGraph: OpenCASCADE (${dep}) may only be linked by "
                                    "[${OCAM_OCCT_ALLOWED}], not '${module}'.")
            endif()
        elseif(dep MATCHES "${OCAM_QT_PATTERN}")
            message(FATAL_ERROR "ModuleGraph: Qt (${dep}) may only be linked by apps/opencam-gui, "
                                "not library '${module}'.")
        endif()
    endforeach()
endfunction()
