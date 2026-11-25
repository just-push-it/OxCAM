# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

find_package(miniz CONFIG REQUIRED)

ocam_add_library(document
    SOURCES
        src/BreakEdges.cpp
        src/Document.cpp
        src/Generate.cpp
        src/LatheFitting.cpp
        src/LatheMachine.cpp
        src/MachineImport.cpp
        src/PostSetup.cpp
        src/Probing.cpp
        src/Project.cpp
        src/ProjectIo.cpp
        src/Simulate.cpp
        src/SimulationCache.cpp
        src/StepBodies.cpp
        src/Stock.cpp
        src/ToolpathCache.cpp
        src/Wcs.cpp
        src/Workholding.cpp
    PUBLIC_DEPS
        ocam::foundation
        ocam::geom3d
        ocam::cad
        ocam::tooling
        ocam::toolpath
        ocam::machine
        ocam::post
        ocam::sim
    PRIVATE_DEPS
        ocam::geom2d
        ocam::strategies
        miniz::miniz)

ocam_embed_text(ocam_document projectSchemaV1 ${PROJECT_SOURCE_DIR}/schemas/project.v1.schema.json)
ocam_embed_text(ocam_document fittingSchemaV2 ${PROJECT_SOURCE_DIR}/schemas/fitting.v2.schema.json)
ocam_embed_text(ocam_document fittingRadialBlock ${PROJECT_SOURCE_DIR}/data/fittings/radial-block.step)
ocam_embed_text(ocam_document fittingAxialBlock ${PROJECT_SOURCE_DIR}/data/fittings/axial-block.step)
ocam_embed_text(ocam_document fittingLiveCentre ${PROJECT_SOURCE_DIR}/data/fittings/live-centre.step)
ocam_embed_text(ocam_document workholdingSchemaV4
    ${PROJECT_SOURCE_DIR}/schemas/workholding.v4.schema.json)
