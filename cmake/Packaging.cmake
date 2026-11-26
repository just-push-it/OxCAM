# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Install rules and CPack archives. Build an optimized preset, then:
#   cmake --build --preset msvc-release --target package
# produces OxCAM-<version>-<system>.zip with the applications, their shared-library
# dependencies (Windows: the vcpkg DLLs and the Qt platform plugin), the bundled machine, post
# and tool-library data, and the licence files.

include(GNUInstallDirs)

set(OCAM_INSTALL_TARGETS oxcam-cli)
if(TARGET oxcam-gui)
    list(APPEND OCAM_INSTALL_TARGETS oxcam-gui)
endif()

if(WIN32)
    # Ship the DLLs the executables load. vcpkg keeps release DLLs in <triplet>/bin and debug
    # ones in <triplet>/debug/bin.
    install(TARGETS ${OCAM_INSTALL_TARGETS}
        RUNTIME_DEPENDENCY_SET ocam_runtime
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    set(_vcpkg_root "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}")
    install(RUNTIME_DEPENDENCY_SET ocam_runtime
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        PRE_EXCLUDE_REGEXES "^api-ms-" "^ext-ms-"
        POST_EXCLUDE_REGEXES "[Ss]ystem32" "[Ss]ys[Ww][Oo][Ww]64"
        DIRECTORIES "$<IF:$<CONFIG:Debug>,${_vcpkg_root}/debug/bin,${_vcpkg_root}/bin>")
    # The Qt platform plugin is installed by apps/opencam-gui (where Qt is found).
else()
    # vcpkg's default Linux and macOS triplets link dependencies statically. A shared Qt
    # platform plugin, when the port builds one, is installed by apps/opencam-gui.
    install(TARGETS ${OCAM_INSTALL_TARGETS} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

install(DIRECTORY ${PROJECT_SOURCE_DIR}/data/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/oxcam
    PATTERN "*.license" EXCLUDE)
install(FILES
        ${PROJECT_SOURCE_DIR}/LICENSE
        ${PROJECT_SOURCE_DIR}/README.md
        ${PROJECT_SOURCE_DIR}/THIRD_PARTY_NOTICES.md
    DESTINATION ${CMAKE_INSTALL_DOCDIR})

set(CPACK_PACKAGE_NAME "OxCAM")
set(CPACK_PACKAGE_VENDOR "OxCAM contributors")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "OxCAM")
set(CPACK_PACKAGE_FILE_NAME "OxCAM-${PROJECT_VERSION}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
if(WIN32)
    set(CPACK_GENERATOR "ZIP")
else()
    set(CPACK_GENERATOR "TGZ")
endif()
set(CPACK_SOURCE_GENERATOR "")
include(CPack)
