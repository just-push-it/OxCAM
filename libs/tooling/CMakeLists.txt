# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

ocam_add_library(tooling
    SOURCES
        src/Profile.cpp
        src/Turning.cpp
        src/TurningHolder.cpp
        src/Tool.cpp
        src/ToolLibrary.cpp
    PUBLIC_DEPS
        ocam::foundation
        ocam::geom2d)

ocam_embed_text(ocam_tooling toolLibrarySchemaV1 ${PROJECT_SOURCE_DIR}/schemas/toollib.v1.schema.json)
