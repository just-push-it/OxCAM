# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

ocam_add_library(machine
    SOURCES
        src/Kinematics.cpp
        src/Machine.cpp
        src/Model.cpp
        src/Turret.cpp
    PUBLIC_DEPS
        ocam::foundation
        ocam::geom3d
        ocam::toolpath)

ocam_embed_text(ocam_machine machineSchemaV16 ${PROJECT_SOURCE_DIR}/schemas/machine.v16.schema.json)
ocam_embed_text(ocam_machine genericMill3Machine ${PROJECT_SOURCE_DIR}/data/machines/generic-3axis-vmc.json)
ocam_embed_text(ocam_machine genericMill5TableACMachine ${PROJECT_SOURCE_DIR}/data/machines/generic-5axis-trunnion-ac.json)
ocam_embed_text(ocam_machine genericLathe2Machine ${PROJECT_SOURCE_DIR}/data/machines/generic-2axis-lathe.json)
ocam_embed_text(ocam_machine genericLatheLiveMachine ${PROJECT_SOURCE_DIR}/data/machines/generic-lathe-live.json)
ocam_embed_text(ocam_machine genericLatheLiveYMachine ${PROJECT_SOURCE_DIR}/data/machines/generic-lathe-live-y.json)
ocam_embed_text(ocam_machine genericLatheTurretMachine ${PROJECT_SOURCE_DIR}/data/machines/generic-lathe-turret.json)
ocam_embed_text(ocam_machine genericLatheSubMachine ${PROJECT_SOURCE_DIR}/data/machines/generic-lathe-sub.json)
ocam_embed_text(ocam_machine genericMill4AMachine ${PROJECT_SOURCE_DIR}/data/machines/generic-4axis-vmc.json)
ocam_embed_text(ocam_machine genericMill5HeadTableBCMachine ${PROJECT_SOURCE_DIR}/data/machines/generic-5axis-headtable-bc.json)
