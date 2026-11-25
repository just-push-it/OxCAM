# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

# QuickJS-ng runs the JavaScript posts (ADR-0006, DL-0039). Linked privately: no QuickJS type
# appears in the public headers.
find_package(qjs CONFIG REQUIRED)

ocam_add_library(post
    SOURCES
        src/Backplot.cpp
        src/Engine.cpp
        src/Lower.cpp
        src/NcJson.cpp
        src/OutputLibrary.cpp
    PUBLIC_DEPS
        ocam::foundation
        ocam::toolpath
        ocam::machine
    PRIVATE_DEPS
        qjs)

ocam_embed_text(ocam_post postManifestSchemaV1 ${PROJECT_SOURCE_DIR}/schemas/post-manifest.v1.schema.json)
# oxcam:fanuc, part of the engine, and the built-in posts.
ocam_embed_text(ocam_post fanucModule ${CMAKE_CURRENT_SOURCE_DIR}/js/fanuc.js)
ocam_embed_text(ocam_post fanucGenericMillJs ${PROJECT_SOURCE_DIR}/data/posts/fanuc-generic-mill.js)
ocam_embed_text(ocam_post fanucGenericMill4AxisJs ${PROJECT_SOURCE_DIR}/data/posts/fanuc-generic-mill-4axis.js)
ocam_embed_text(ocam_post fanucGenericLatheJs ${PROJECT_SOURCE_DIR}/data/posts/fanuc-generic-lathe.js)
ocam_embed_text(ocam_post haasLatheJs ${PROJECT_SOURCE_DIR}/data/posts/haas-lathe.js)
ocam_embed_text(ocam_post haasClassicMillJs ${PROJECT_SOURCE_DIR}/data/posts/haas-classic-mill.js)
