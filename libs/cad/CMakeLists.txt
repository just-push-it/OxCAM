# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

# OpenCASCADE toolkits used by libs/cad. Consumers that include ocam/cad/occt/ShapeAccess.hpp
# must link OCAM_OCCT_MODELING_LIBS themselves.
set(OCAM_OCCT_MODELING_LIBS
    TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo TKBRep TKTopAlgo TKPrim TKBO TKShHealing
    TKMesh TKXSBase TKDESTEP TKCDF TKLCAF TKCAF TKXCAF
    CACHE INTERNAL "OpenCASCADE modeling toolkits")

ocam_add_library(cad
    SOURCES
        src/Assembly.cpp
        src/Chain.cpp
        src/Datum.cpp
        src/EdgeFaces.cpp
        src/Features.cpp
        src/Model.cpp
        src/Revolve.cpp
        src/Ruled.cpp
        src/Section.cpp
        src/SubShape.cpp
        src/Tessellation.cpp
    PUBLIC_DEPS
        ocam::foundation
        ocam::geom2d
        ocam::geom3d
    PRIVATE_DEPS
        ${OCAM_OCCT_MODELING_LIBS})
