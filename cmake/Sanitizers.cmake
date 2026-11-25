# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

# ocam_enable_sanitizers(<target>)
# Applies OCAM_SANITIZERS (e.g. "address;undefined") to a target. GCC/Clang only;
# MSVC supports "address" alone.
function(ocam_enable_sanitizers target)
    if(NOT OCAM_SANITIZERS)
        return()
    endif()

    if(MSVC)
        if("address" IN_LIST OCAM_SANITIZERS)
            target_compile_options(${target} PRIVATE /fsanitize=address)
        endif()
        return()
    endif()

    list(JOIN OCAM_SANITIZERS "," sanitizers)
    target_compile_options(${target} PRIVATE -fsanitize=${sanitizers} -fno-omit-frame-pointer
                                             -fno-sanitize-recover=all)
    target_link_options(${target} PRIVATE -fsanitize=${sanitizers})
endfunction()
