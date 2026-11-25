# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

# ocam_add_library(<module>
#     SOURCES <src>...
#     [PUBLIC_DEPS <target>...]
#     [PRIVATE_DEPS <target>...])
#
# Creates static library target ocam_<module> with alias ocam::<module>. Public headers live in
# libs/<module>/include/ocam/<module>/ and sources in libs/<module>/src/. Dependencies are checked
# against cmake/ModuleGraph.cmake.
function(ocam_add_library module)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "" "SOURCES;PUBLIC_DEPS;PRIVATE_DEPS")
    if(arg_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "ocam_add_library(${module}): unknown arguments ${arg_UNPARSED_ARGUMENTS}")
    endif()

    ocam_check_module_deps(${module} ${arg_PUBLIC_DEPS} ${arg_PRIVATE_DEPS})

    set(target ocam_${module})
    add_library(${target} STATIC ${arg_SOURCES})
    add_library(ocam::${module} ALIAS ${target})

    target_include_directories(${target}
        PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(${target} PUBLIC ${arg_PUBLIC_DEPS} PRIVATE ${arg_PRIVATE_DEPS})
    target_compile_features(${target} PUBLIC cxx_std_20)
    set_target_properties(${target} PROPERTIES OUTPUT_NAME oxcam_${module})

    ocam_set_warnings(${target})
    ocam_enable_sanitizers(${target})
endfunction()

# ocam_embed_text(<target> <symbol> <file> [NAMESPACE <ns>])
# Embeds <file> into <target> as `extern const std::string_view <ns>::<symbol>` (default
# namespace ocam::embedded). Declare the symbol yourself where it is used.
function(ocam_embed_text target symbol file)
    cmake_parse_arguments(PARSE_ARGV 3 arg "" "NAMESPACE" "")
    if(NOT arg_NAMESPACE)
        set(arg_NAMESPACE ocam::embedded)
    endif()
    get_filename_component(input ${file} ABSOLUTE)
    set(output ${CMAKE_CURRENT_BINARY_DIR}/embedded/${symbol}.cpp)
    add_custom_command(
        OUTPUT ${output}
        COMMAND ${CMAKE_COMMAND} -DINPUT=${input} -DOUTPUT=${output} -DSYMBOL=${symbol}
                -DNAMESPACE=${arg_NAMESPACE} -P ${PROJECT_SOURCE_DIR}/cmake/EmbedText.cmake
        DEPENDS ${input} ${PROJECT_SOURCE_DIR}/cmake/EmbedText.cmake
        COMMENT "Embedding ${file}"
        VERBATIM)
    target_sources(${target} PRIVATE ${output})
endfunction()

# ocam_add_executable(<target> SOURCES <src>... [DEPS <target>...])
function(ocam_add_executable target)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "" "SOURCES;DEPS")
    add_executable(${target} ${arg_SOURCES})
    target_link_libraries(${target} PRIVATE ${arg_DEPS})
    ocam_set_warnings(${target})
    ocam_enable_sanitizers(${target})
endfunction()

# The memory cap of a process running tests [GB], so a runaway test fails on its own.
set(OCAM_TEST_MEMORY_CAP_GB 8)

# ocam_add_memory_cap(<target> <default GB>)
# Compiles the cap into a program (tests/support/MemoryCap.cpp). OCAM_MEMORY_CAP_GB in the
# environment overrides the default; 0 means no cap.
function(ocam_add_memory_cap target default_gb)
    target_sources(${target} PRIVATE ${PROJECT_SOURCE_DIR}/tests/support/MemoryCap.cpp)
    target_compile_definitions(${target} PRIVATE OCAM_MEMORY_CAP_DEFAULT_GB=${default_gb})
endfunction()

# ocam_add_test(<module> SOURCES <src>... [DEPS <target>...])
# Builds test_<module> against Catch2 and registers each test case with CTest.
function(ocam_add_test module)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "" "SOURCES;DEPS")
    set(target test_${module})
    add_executable(${target} ${arg_SOURCES})
    ocam_add_memory_cap(${target} ${OCAM_TEST_MEMORY_CAP_GB})
    target_link_libraries(${target} PRIVATE ocam::${module} ${arg_DEPS} Catch2::Catch2WithMain)
    ocam_set_warnings(${target})
    ocam_enable_sanitizers(${target})
    catch_discover_tests(${target}
        TEST_PREFIX "${module}."
        DISCOVERY_MODE PRE_TEST
        PROPERTIES LABELS ${module})
endfunction()
