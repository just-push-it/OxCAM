# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

find_package(Eigen3 CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(nlohmann_json_schema_validator CONFIG REQUIRED)

set(OCAM_GENERATED_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
configure_file(src/Version.hpp.in ${OCAM_GENERATED_INCLUDE_DIR}/ocam/foundation/Version.hpp @ONLY)

ocam_add_library(foundation
    SOURCES
        src/Cancel.cpp
        src/Error.cpp
        src/Json.cpp
        src/Log.cpp
        src/Progress.cpp
        src/Sha256.cpp
        src/Units.cpp
        src/Uuid.cpp
    PUBLIC_DEPS
        Eigen3::Eigen
        fmt::fmt
        nlohmann_json::nlohmann_json
    PRIVATE_DEPS
        nlohmann_json_schema_validator::validator)

target_include_directories(ocam_foundation PUBLIC $<BUILD_INTERFACE:${OCAM_GENERATED_INCLUDE_DIR}>)
