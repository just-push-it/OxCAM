# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

add_subdirectory(fixtures/gen)
add_subdirectory(unit/foundation)
add_subdirectory(unit/geom2d)
add_subdirectory(unit/geom3d)
add_subdirectory(unit/cad)
add_subdirectory(unit/tooling)
add_subdirectory(unit/toolpath)
add_subdirectory(unit/strategies)
add_subdirectory(unit/machine)
add_subdirectory(unit/sim)
add_subdirectory(unit/post)
add_subdirectory(unit/document)

# The library layering in cmake/ModuleGraph.cmake must accept legal edges and reject illegal ones.
foreach(case allowed_layering allowed_occt_in_cad)
    add_test(NAME modulegraph.${case}
        COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${PROJECT_SOURCE_DIR} -DCASE=${case}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ModuleGraphCase.cmake)
    set_tests_properties(modulegraph.${case} PROPERTIES LABELS modulegraph)
endforeach()

foreach(case upward_edge skip_layer occt_outside_cad qt_in_library unknown_module)
    add_test(NAME modulegraph.rejects_${case}
        COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${PROJECT_SOURCE_DIR} -DCASE=${case}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ModuleGraphCase.cmake)
    set_tests_properties(modulegraph.rejects_${case} PROPERTIES
        PASS_REGULAR_EXPRESSION "ModuleGraph:"
        LABELS modulegraph)
endforeach()
