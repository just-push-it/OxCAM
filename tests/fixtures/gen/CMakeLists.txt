# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

ocam_add_executable(oxcam-fixture-gen
    SOURCES main.cpp LatheMachineFixtures.cpp
    DEPS ocam::document ${OCAM_OCCT_MODELING_LIBS} TKFillet TKOffset TKBool)

set(fixtures block_pocket block_pocket_inch fillet_pocket hole_plate boss_plate shaft bored live millturn stud_bar wrapped bushing spool grooved tapered dome swarf vise_jaws vise_base
    vise_fixed_jaw vise_moving_jaw vise_assembly selfcentring_assembly selfcentring_extras
    dovetail_assembly
    plate_assembly chuck_assembly collet_assembly wedge_hole)
list(TRANSFORM fixtures PREPEND ${OCAM_FIXTURE_DIR}/)
list(TRANSFORM fixtures APPEND .step)
list(APPEND fixtures ${OCAM_FIXTURE_DIR}/sample_project.ocam ${OCAM_FIXTURE_DIR}/trace_project.ocam ${OCAM_FIXTURE_DIR}/holders_project.ocam ${OCAM_FIXTURE_DIR}/live_project.ocam ${OCAM_FIXTURE_DIR}/millturn_project.ocam ${OCAM_FIXTURE_DIR}/wrapped_project.ocam ${OCAM_FIXTURE_DIR}/dome_project.ocam ${OCAM_FIXTURE_DIR}/shaft_project.ocam ${OCAM_FIXTURE_DIR}/bored_project.ocam ${OCAM_FIXTURE_DIR}/flip_project.ocam ${OCAM_FIXTURE_DIR}/threads_project.ocam ${OCAM_FIXTURE_DIR}/grooved_project.ocam ${OCAM_FIXTURE_DIR}/ranges_project.ocam ${OCAM_FIXTURE_DIR}/backturn_project.ocam ${OCAM_FIXTURE_DIR}/nose_project.ocam ${OCAM_FIXTURE_DIR}/centres_project.ocam ${OCAM_FIXTURE_DIR}/fixture_project.ocam
    ${OCAM_FIXTURE_DIR}/vise.ocamhold ${OCAM_FIXTURE_DIR}/vise_project.ocam
    ${OCAM_FIXTURE_DIR}/vise_origin_project.ocam
    ${OCAM_FIXTURE_DIR}/selfcentring.ocamhold ${OCAM_FIXTURE_DIR}/dovetail.ocamhold
    ${OCAM_FIXTURE_DIR}/plate.ocamhold ${OCAM_FIXTURE_DIR}/chuck.ocamhold
    ${OCAM_FIXTURE_DIR}/collet.ocamhold ${OCAM_FIXTURE_DIR}/chuck_project.ocam
    ${OCAM_FIXTURE_DIR}/dome5_machine_project.ocam ${OCAM_FIXTURE_DIR}/machines/trunnion-demo.json
    ${OCAM_FIXTURE_DIR}/trunnion_machine.step ${OCAM_FIXTURE_DIR}/trunnion_assembly.step
    ${OCAM_FIXTURE_DIR}/plate32_project.ocam ${OCAM_FIXTURE_DIR}/plate32_machine_project.ocam
    ${OCAM_FIXTURE_DIR}/turret_project.ocam ${OCAM_FIXTURE_DIR}/turret_collisions_project.ocam
    ${OCAM_FIXTURE_DIR}/turret_tailstock_project.ocam ${OCAM_FIXTURE_DIR}/turret_sub_project.ocam
    ${OCAM_FIXTURE_DIR}/turret_full_project.ocam ${OCAM_FIXTURE_DIR}/turret_live_project.ocam
    ${OCAM_FIXTURE_DIR}/turret_blocks_project.ocam
    ${OCAM_FIXTURE_DIR}/dome_steep_shallow_project.ocam ${OCAM_FIXTURE_DIR}/fillet_pocket_project.ocam
    ${OCAM_FIXTURE_DIR}/boss_thread_project.ocam
    ${OCAM_FIXTURE_DIR}/swarf_project.ocam)

add_custom_command(
    OUTPUT ${fixtures}
    COMMAND oxcam-fixture-gen ${OCAM_FIXTURE_DIR}
    DEPENDS oxcam-fixture-gen
    COMMENT "Generating STEP fixtures"
    VERBATIM)
add_custom_target(fixtures ALL DEPENDS ${fixtures})
