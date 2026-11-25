# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

find_package(CLI11 CONFIG REQUIRED)

ocam_add_executable(oxcam-cli
    SOURCES main.cpp
    DEPS ocam::document CLI11::CLI11)
ocam_add_memory_cap(oxcam-cli 0)

if(OCAM_BUILD_TESTS)
    add_test(NAME cli.version COMMAND oxcam-cli --version)
    set_tests_properties(cli.version PROPERTIES
        PASS_REGULAR_EXPRESSION "^oxcam-cli ${PROJECT_VERSION}"
        LABELS cli)
    add_test(NAME cli.info COMMAND oxcam-cli info)
    set_tests_properties(cli.info PROPERTIES
        PASS_REGULAR_EXPRESSION "internal units: mm, rad, s"
        LABELS cli)
    add_test(NAME cli.toollib_check_sample
        COMMAND oxcam-cli toollib check ${PROJECT_SOURCE_DIR}/data/tool-libraries/sample-metric.ocamtools)
    set_tests_properties(cli.toollib_check_sample PROPERTIES
        PASS_REGULAR_EXPRESSION "8 tools, 1 holders, no problems"
        LABELS cli)
    add_test(NAME cli.project_info
        COMMAND oxcam-cli project info ${OCAM_FIXTURE_DIR}/sample_project.ocam)
    set_tests_properties(cli.project_info PROPERTIES
        PASS_REGULAR_EXPRESSION "Setup 1 .G54., stock 102.000 x 62.000 x 31.000 mm, 3 operation"
        LABELS cli)
    add_test(NAME cli.project_toolpaths
        COMMAND oxcam-cli project toolpaths ${OCAM_FIXTURE_DIR}/sample_project.ocam)
    set_tests_properties(cli.project_toolpaths PROPERTIES
        PASS_REGULAR_EXPRESSION "3 of 3 operations generated"
        LABELS cli)
    add_test(NAME cli.project_toolpaths_fixtures
        COMMAND oxcam-cli project toolpaths ${OCAM_FIXTURE_DIR}/fixture_project.ocam)
    set_tests_properties(cli.project_toolpaths_fixtures PROPERTIES
        PASS_REGULAR_EXPRESSION "kept 1 mm clear of the fixtures.*3 of 3 operations generated"
        LABELS cli)
    add_test(NAME cli.project_info_workholding
        COMMAND oxcam-cli project info ${OCAM_FIXTURE_DIR}/vise_project.ocam)
    set_tests_properties(cli.project_info_workholding PROPERTIES
        PASS_REGULAR_EXPRESSION "workholding: Test vise, jaws 62.000 mm apart"
        LABELS cli)
    add_test(NAME cli.project_toolpaths_workholding
        COMMAND oxcam-cli project toolpaths ${OCAM_FIXTURE_DIR}/vise_project.ocam)
    set_tests_properties(cli.project_toolpaths_workholding PROPERTIES
        PASS_REGULAR_EXPRESSION "kept 1 mm clear of the workholding.*2 of 2 operations generated"
        LABELS cli)
    add_test(NAME cli.project_simulate_workholding
        COMMAND oxcam-cli project simulate ${OCAM_FIXTURE_DIR}/vise_project.ocam
                --resolution 1.0)
    set_tests_properties(cli.project_simulate_workholding PROPERTIES
        PASS_REGULAR_EXPRESSION "result: ok"
        FAIL_REGULAR_EXPRESSION "workholding collision"
        LABELS cli)
    add_test(NAME cli.project_post_verify
        COMMAND oxcam-cli project post ${OCAM_FIXTURE_DIR}/sample_project.ocam
                -o ${PROJECT_BINARY_DIR}/sample_project.nc --verify)
    set_tests_properties(cli.project_post_verify PROPERTIES
        PASS_REGULAR_EXPRESSION "verified: back-plotted G-code matches"
        FIXTURES_SETUP posted_program
        LABELS cli)
    add_test(NAME cli.backplot COMMAND oxcam-cli backplot ${PROJECT_BINARY_DIR}/sample_project.nc)
    set_tests_properties(cli.backplot PROPERTIES
        PASS_REGULAR_EXPRESSION "tools: 1, 2, 1"
        FIXTURES_REQUIRED posted_program
        LABELS cli)
    add_test(NAME cli.project_simulate
        COMMAND oxcam-cli project simulate ${OCAM_FIXTURE_DIR}/sample_project.ocam
                --resolution 0.25 --json ${PROJECT_BINARY_DIR}/sample_project.sim.json)
    set_tests_properties(cli.project_simulate PROPERTIES
        PASS_REGULAR_EXPRESSION "gouge 0.0 mm3.*result: ok"
        LABELS cli)
    add_test(NAME cli.project_simulate_3d
        COMMAND oxcam-cli project simulate ${OCAM_FIXTURE_DIR}/dome_project.ocam --resolution 0.5)
    set_tests_properties(cli.project_simulate_3d PROPERTIES
        PASS_REGULAR_EXPRESSION "gouge 0.0 mm3"
        LABELS cli
        TIMEOUT 900)
    add_test(NAME cli.project_simulate_pencil
        COMMAND oxcam-cli project simulate ${OCAM_FIXTURE_DIR}/fillet_pocket_project.ocam
                --resolution 0.25)
    set_tests_properties(cli.project_simulate_pencil PROPERTIES
        PASS_REGULAR_EXPRESSION "gouge 0.0 mm3.*result: ok"
        LABELS cli
        TIMEOUT 900)
    add_test(NAME cli.project_simulate_steep_shallow
        COMMAND oxcam-cli project simulate ${OCAM_FIXTURE_DIR}/dome_steep_shallow_project.ocam
                --resolution 0.25)
    set_tests_properties(cli.project_simulate_steep_shallow PROPERTIES
        PASS_REGULAR_EXPRESSION "gouge 0.0 mm3.*result: ok"
        LABELS cli
        TIMEOUT 900)
    add_test(NAME cli.project_post_lathe
        COMMAND oxcam-cli project post ${OCAM_FIXTURE_DIR}/shaft_project.ocam
                -o ${PROJECT_BINARY_DIR}/shaft_project.nc --verify)
    set_tests_properties(cli.project_post_lathe PROPERTIES
        PASS_REGULAR_EXPRESSION "verified: back-plotted G-code matches"
        LABELS cli)
    add_test(NAME cli.project_post_lathe_drilled
        COMMAND oxcam-cli project post ${OCAM_FIXTURE_DIR}/bored_project.ocam
                -o ${PROJECT_BINARY_DIR}/bored_project.nc --verify)
    set_tests_properties(cli.project_post_lathe_drilled PROPERTIES
        PASS_REGULAR_EXPRESSION "verified: back-plotted G-code matches"
        LABELS cli)
    add_test(NAME cli.project_post_lathe_wrapped
        COMMAND oxcam-cli project post ${OCAM_FIXTURE_DIR}/wrapped_project.ocam
                -o ${PROJECT_BINARY_DIR}/wrapped_project.nc --verify)
    set_tests_properties(cli.project_post_lathe_wrapped PROPERTIES
        PASS_REGULAR_EXPRESSION "verified: back-plotted G-code matches"
        LABELS cli)
    add_test(NAME cli.project_simulate_lathe
        COMMAND oxcam-cli project simulate ${OCAM_FIXTURE_DIR}/shaft_project.ocam --resolution 0.25)
    set_tests_properties(cli.project_simulate_lathe PROPERTIES
        PASS_REGULAR_EXPRESSION "gouge 0.0 mm3"
        LABELS cli
        TIMEOUT 900)
    add_test(NAME cli.project_post_4axis
        COMMAND oxcam-cli project post ${OCAM_FIXTURE_DIR}/rotary_project.ocam
                -o ${PROJECT_BINARY_DIR}/rotary_project.nc --verify)
    set_tests_properties(cli.project_post_4axis PROPERTIES
        PASS_REGULAR_EXPRESSION "verified: back-plotted G-code matches"
        LABELS cli)
    add_test(NAME cli.project_simulate_4axis
        COMMAND oxcam-cli project simulate ${OCAM_FIXTURE_DIR}/rotary_project.ocam --resolution 0.25)
    set_tests_properties(cli.project_simulate_4axis PROPERTIES
        PASS_REGULAR_EXPRESSION "gouge 0.0 mm3.*result: ok"
        LABELS cli
        TIMEOUT 900)
    add_test(NAME cli.project_post_5axis
        COMMAND oxcam-cli project post ${OCAM_FIXTURE_DIR}/dome5_project.ocam
                -o ${PROJECT_BINARY_DIR}/dome5_project.nc --verify)
    set_tests_properties(cli.project_post_5axis PROPERTIES
        PASS_REGULAR_EXPRESSION "verified: back-plotted G-code matches"
        LABELS cli)
    add_test(NAME cli.project_simulate_5axis
        COMMAND oxcam-cli project simulate ${OCAM_FIXTURE_DIR}/dome5_project.ocam --resolution 0.5)
    set_tests_properties(cli.project_simulate_5axis PROPERTIES
        PASS_REGULAR_EXPRESSION "gouge 0.0 mm3.*result: ok"
        LABELS cli
        TIMEOUT 900)
    # Swarf milling and tilted rest roughing (DL-0085) on the trunnion.
    add_test(NAME cli.project_post_swarf
        COMMAND oxcam-cli project post ${OCAM_FIXTURE_DIR}/swarf_project.ocam
                -o ${PROJECT_BINARY_DIR}/swarf_project.nc --verify)
    set_tests_properties(cli.project_post_swarf PROPERTIES
        PASS_REGULAR_EXPRESSION "verified: back-plotted G-code matches"
        LABELS cli
        TIMEOUT 900)
    add_test(NAME cli.project_simulate_swarf
        COMMAND oxcam-cli project simulate ${OCAM_FIXTURE_DIR}/swarf_project.ocam --resolution 0.5)
    set_tests_properties(cli.project_simulate_swarf PROPERTIES
        PASS_REGULAR_EXPRESSION "gouge 0.0 mm3.*result: ok"
        LABELS cli
        TIMEOUT 900)
    add_test(NAME cli.project_post_3plus2
        COMMAND oxcam-cli project post ${OCAM_FIXTURE_DIR}/plate32_project.ocam
                -o ${PROJECT_BINARY_DIR}/plate32_project.nc --verify)
    set_tests_properties(cli.project_post_3plus2 PROPERTIES
        PASS_REGULAR_EXPRESSION "verified: back-plotted G-code matches"
        LABELS cli)
    add_test(NAME cli.project_simulate_3plus2
        COMMAND oxcam-cli project simulate ${OCAM_FIXTURE_DIR}/plate32_project.ocam --resolution 0.5)
    set_tests_properties(cli.project_simulate_3plus2 PROPERTIES
        PASS_REGULAR_EXPRESSION "gouge 0.0 mm3.*result: ok"
        LABELS cli
        TIMEOUT 900)
    add_test(NAME cli.project_simulate_3plus2_machine
        COMMAND oxcam-cli project simulate ${OCAM_FIXTURE_DIR}/plate32_machine_project.ocam
                --resolution 0.5)
    set_tests_properties(cli.project_simulate_3plus2_machine PROPERTIES
        PASS_REGULAR_EXPRESSION "gouge 0.0 mm3.*result: ok"
        FAIL_REGULAR_EXPRESSION "machine collision"
        LABELS cli
        TIMEOUT 900)
    add_test(NAME cli.project_info_missing COMMAND oxcam-cli project info does-not-exist.ocam)
    set_tests_properties(cli.project_info_missing PROPERTIES WILL_FAIL TRUE LABELS cli)
    get_property(cliTests DIRECTORY PROPERTY TESTS)
    set_property(TEST ${cliTests} APPEND PROPERTY ENVIRONMENT
        "OCAM_MEMORY_CAP_GB=${OCAM_TEST_MEMORY_CAP_GB}")
endif()
