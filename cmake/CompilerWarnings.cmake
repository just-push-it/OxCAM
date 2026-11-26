# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

# ocam_set_warnings(<target>)
# Applies the project warning set to a target we own (never to third-party code).
function(ocam_set_warnings target)
    if(MSVC)
        set(warnings /W4 /permissive- /w14242 /w14254 /w14263 /w14265 /w14287 /w14296
                     /w14311 /w14826 /w14905 /w14906 /w14928
                     # "Unreachable code" comes from the optimizer after inlining, also from
                     # third-party headers (/external:W0 does not reach it; Clipper2 trips it).
                     /wd4702)
        set(werror /WX)
    else()
        set(warnings -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wold-style-cast
                     -Wcast-align -Woverloaded-virtual -Wnull-dereference -Wdouble-promotion
                     -Wimplicit-fallthrough
                     # Aggregates rely on default member initializers for trailing fields.
                     -Wno-missing-field-initializers)
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            list(APPEND warnings -Wduplicated-cond -Wlogical-op
                 # Middle-end warning: it fires inside Eigen (a system header) once the
                 # outer product is inlined, and -Werror would fail the build on a false
                 # positive. AxisFit's covariance update is the case CI hit.
                 -Wno-error=null-dereference)
        endif()
        set(werror -Werror)
    endif()

    target_compile_options(${target} PRIVATE ${warnings})
    if(OCAM_WARNINGS_AS_ERRORS)
        target_compile_options(${target} PRIVATE ${werror})
    endif()

    if(MSVC)
        # /Zi with a parallel build (Ninja, sccache) has several cl.exe processes write one PDB.
        target_compile_options(${target} PRIVATE /utf-8 /Zc:__cplusplus /Zc:preprocessor /FS)
        # _CRT_SECURE_NO_WARNINGS: std::getenv (the diagnostics switches) is standard C++; MSVC's
        # suggestion of _dupenv_s is not portable.
        target_compile_definitions(${target} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN _USE_MATH_DEFINES
                                                     _CRT_SECURE_NO_WARNINGS)
    endif()
endfunction()
