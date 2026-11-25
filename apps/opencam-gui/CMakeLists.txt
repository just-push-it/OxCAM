# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

find_package(Qt6 REQUIRED COMPONENTS Widgets)

ocam_add_executable(oxcam-gui
    SOURCES
        main.cpp
        Commands.cpp
        Commands.hpp
        ComboLists.cpp
        ComboLists.hpp
        CuttingPresetDialog.cpp
        CuttingPresetDialog.hpp
        MachineLibrary.cpp
        MachineLibrary.hpp
        MachineLibraryDialog.cpp
        MachineLibraryDialog.hpp
        MachinePartsImportDialog.cpp
        MachinePartsImportDialog.hpp
        HolderImportDialog.cpp
        HolderImportDialog.hpp
        MainWindow.cpp
        MainWindow.hpp
        OcctView.cpp
        OcctView.hpp
        OpenCamSettings.cpp
        OpenCamSettings.hpp
        OperationDefaults.cpp
        OperationDefaults.hpp
        ParamEditors.cpp
        ParamEditors.hpp
        PartTable.cpp
        PartTable.hpp
        PostChoices.cpp
        PostChoices.hpp
        PostLibrary.cpp
        PostLibrary.hpp
        PostDialog.cpp
        PostDialog.hpp
        PostOptionsForm.cpp
        PostOptionsForm.hpp
        PostProcessDialog.cpp
        PostProcessDialog.hpp
        PreferencesDialog.cpp
        PreferencesDialog.hpp
        ProjectTree.cpp
        ProjectTree.hpp
        PropertyPanel.cpp
        PropertyPanel.hpp
        LatheFittingsDialog.cpp
        LatheFittingsDialog.hpp
        SetupPlacementDialog.cpp
        SetupPlacementDialog.hpp
        ShowBuiltins.hpp
        SimPrs.cpp
        SimPrs.hpp
        SimulationPanel.cpp
        SimulationPanel.hpp
        StartHub.cpp
        StartHub.hpp
        ToolLibrariesDialog.cpp
        ToolLibrariesDialog.hpp
        ToolLibraries.cpp
        ToolLibraries.hpp
        ToolLibraryDialog.cpp
        HolderEditorCheck.cpp
        ToolLibraryDialog.hpp
        ToolProfileView.cpp
        ToolProfileView.hpp
        ToolpathDataDialog.cpp
        ToolpathDataDialog.hpp
        ToolpathPrs.cpp
        ToolpathPrs.hpp
        TurretPreview.cpp
        TurretPreview.hpp
        TurretWindow.cpp
        TurretWindow.hpp
        ToolTips.cpp
        ToolTips.hpp
        Translations.cpp
        Translations.hpp
        Units.cpp
        Units.hpp
        ViewColors.cpp
        ViewColors.hpp
        WheelGuard.cpp
        WheelGuard.hpp
        WorkholdingLibrary.cpp
        WorkholdingLibrary.hpp
        WorkholdingLibraryDialog.cpp
        WorkholdingSetupDialog.cpp
        WorkholdingSetupDialog.hpp
        WorkholdingLibraryDialog.hpp
        WorkholdingWizard.cpp
        WorkholdingWizard.hpp
    DEPS
        ocam::document
        ocam::strategies
        Qt6::Widgets
        ${OCAM_OCCT_MODELING_LIBS}
        TKService TKV3d TKOpenGl)

set_target_properties(oxcam-gui PROPERTIES AUTOMOC ON WIN32_EXECUTABLE ON)
ocam_add_memory_cap(oxcam-gui 0)

# Translations (docs/translations.md): translations/opencam_<language>.ts, compiled by Qt's
# lrelease and embedded under :/i18n. Qt's Linguist tools are optional: a Qt installation, a
# distribution's Qt 6 tools package or `pip install PySide6-Essentials` provide them; point
# OCAM_QT_TOOLS_DIR at their folder when they are not on the PATH. Without lrelease the
# interface is in English only.
set(OCAM_QT_TOOLS_DIR "" CACHE PATH "Folder with Qt's lupdate and lrelease (optional)")
find_program(OCAM_LUPDATE NAMES lupdate lupdate-qt6
    HINTS ${OCAM_QT_TOOLS_DIR} ${QT6_INSTALL_PREFIX}/${QT6_INSTALL_BINS})
find_program(OCAM_LRELEASE NAMES lrelease lrelease-qt6
    HINTS ${OCAM_QT_TOOLS_DIR} ${QT6_INSTALL_PREFIX}/${QT6_INSTALL_BINS})
file(GLOB OCAM_TS_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/translations/opencam_*.ts)
if(OCAM_LRELEASE AND OCAM_TS_FILES)
    set(qm_files)
    foreach(ts IN LISTS OCAM_TS_FILES)
        get_filename_component(name ${ts} NAME_WE)
        set(qm ${CMAKE_CURRENT_BINARY_DIR}/i18n/${name}.qm)
        add_custom_command(OUTPUT ${qm}
            COMMAND ${OCAM_LRELEASE} -silent ${ts} -qm ${qm}
            DEPENDS ${ts}
            COMMENT "Compiling translation ${name}"
            VERBATIM)
        list(APPEND qm_files ${qm})
    endforeach()
    qt_add_resources(oxcam-gui translations
        PREFIX /i18n
        BASE ${CMAKE_CURRENT_BINARY_DIR}/i18n
        FILES ${qm_files})
else()
    message(STATUS "Qt's lrelease not found (OCAM_QT_TOOLS_DIR): the interface is in English only")
endif()
if(OCAM_LUPDATE)
    # Brings the .ts files up to date with the texts in the sources; run it after changing
    # interface texts or the libraries' (regenerate i18n/LibraryStrings.cpp first).
    add_custom_target(update_translations
        COMMAND ${OCAM_LUPDATE} -silent -locations none -no-ui-lines
                ${CMAKE_CURRENT_SOURCE_DIR} -ts ${OCAM_TS_FILES}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMENT "Updating translations/*.ts"
        VERBATIM)
endif()

# Qt loads its platform plugin from <exe dir>/platforms. Copy it there so the build tree runs
# without a deployment step (installers will use the Qt deploy tooling instead).
foreach(plugin QWindowsIntegrationPlugin QCocoaIntegrationPlugin QXcbIntegrationPlugin)
    if(TARGET Qt6::${plugin})
        add_custom_command(TARGET oxcam-gui POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:oxcam-gui>/platforms
            COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:Qt6::${plugin}>
                    $<TARGET_FILE_DIR:oxcam-gui>/platforms/
            VERBATIM)
    endif()
endforeach()

if(WIN32 AND TARGET Qt6::QWindowsIntegrationPlugin)
    include(GNUInstallDirs)
    install(FILES $<TARGET_FILE:Qt6::QWindowsIntegrationPlugin>
        DESTINATION ${CMAKE_INSTALL_BINDIR}/platforms)
endif()

if(OCAM_BUILD_TESTS)
    # Needs a display (use xvfb-run on headless Linux). Label "gui" so CI can select it.
    add_test(NAME gui.smoke_test
        COMMAND oxcam-gui --smoke-test ${OCAM_FIXTURE_DIR}/hole_plate.step)
    set_tests_properties(gui.smoke_test PROPERTIES LABELS gui TIMEOUT 120)
    add_test(NAME gui.smoke_test_project
        COMMAND oxcam-gui --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam)
    set_tests_properties(gui.smoke_test_project PROPERTIES LABELS gui TIMEOUT 120)
    add_test(NAME gui.smoke_test_simulation
        COMMAND oxcam-gui --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam
                --screenshot-sim ${PROJECT_BINARY_DIR}/sample_project_sim.png)
    set_tests_properties(gui.smoke_test_simulation PROPERTIES LABELS gui TIMEOUT 300)
    # 3D finishing (DL-0080): the pocket's fillets cleaned up by pencil passes after a larger
    # ball, and the dome finished steep and shallow.
    add_test(NAME gui.simulation_pencil
        COMMAND oxcam-gui --smoke-test ${OCAM_FIXTURE_DIR}/fillet_pocket_project.ocam
                --screenshot-sim ${PROJECT_BINARY_DIR}/fillet_pocket_sim.png)
    add_test(NAME gui.simulation_steep_shallow
        COMMAND oxcam-gui --smoke-test ${OCAM_FIXTURE_DIR}/dome_steep_shallow_project.ocam
                --screenshot-sim ${PROJECT_BINARY_DIR}/dome_steep_shallow_sim.png)
    set_tests_properties(gui.simulation_pencil gui.simulation_steep_shallow
        PROPERTIES LABELS gui TIMEOUT 300)
    # The stock display (DL-0040) during a milling operation and during a turning one.
    add_test(NAME gui.smoke_test_simulation_smooth_milled
        COMMAND oxcam-gui --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam
                --screenshot-sim ${PROJECT_BINARY_DIR}/sample_project_smooth_mid_pocket.png
                --sim-position 1:90)
    add_test(NAME gui.smoke_test_simulation_smooth_turned
        COMMAND oxcam-gui --smoke-test ${OCAM_FIXTURE_DIR}/shaft_project.ocam
                --screenshot-sim ${PROJECT_BINARY_DIR}/shaft_project_smooth_mid_rough.png
                --sim-position 1:20)
    # A preference for the cell display, which has gone, loads without complaint.
    file(WRITE ${PROJECT_BINARY_DIR}/test-settings-cells/OxCAM/OxCAM.ini
         "[simulation]\nstockDisplay=cells\n")
    add_test(NAME gui.simulation_old_cells_preference
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings-cells
                --smoke-test ${OCAM_FIXTURE_DIR}/shaft_project.ocam
                --screenshot-sim ${PROJECT_BINARY_DIR}/old_cells_preference.png)
    set_tests_properties(gui.smoke_test_simulation_smooth_milled
        gui.smoke_test_simulation_smooth_turned gui.simulation_old_cells_preference
        PROPERTIES LABELS gui TIMEOUT 300)
    # Turning: the insert shows mid-operation (the shaft's rough turning), and playback moves
    # on through the operations with the stock.
    add_test(NAME gui.lathe_simulation_insert
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/shaft_project.ocam --sim-position 1:40
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_simulation_insert.png)
    set_tests_properties(gui.lathe_simulation_insert PROPERTIES LABELS gui TIMEOUT 300)
    # Drilling on the spindle axis: the drill mid-hole, then the part bored from solid.
    add_test(NAME gui.lathe_simulation_drill
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/bored_project.ocam --sim-position 1:6
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_simulation_drill.png)
    add_test(NAME gui.lathe_simulation_bored
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/bored_project.ocam --sim-position 5:0
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_simulation_bored.png)
    # A live tool's holes in the stock: the bolt circle part way, the cross holes with the tool in
    # one, and all of them at the end.
    add_test(NAME gui.lathe_live_bolt_circle
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/live_project.ocam --sim-position 2:16
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_live_bolt_circle.png)
    add_test(NAME gui.lathe_live_cross_hole
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/live_project.ocam --sim-position 3:5
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_live_cross_hole.png)
    add_test(NAME gui.lathe_live_all
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/live_project.ocam
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_live_all.png)
    # A bar turned and then milled: the pocket part way, the flat with its tool, everything at the
    # end, and the page's Live tool axes.
    add_test(NAME gui.lathe_mill_pocket
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/millturn_project.ocam --sim-position 2:10
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_mill_pocket.png)
    add_test(NAME gui.lathe_mill_flat
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/millturn_project.ocam --sim-position 8:12
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_mill_flat.png)
    add_test(NAME gui.lathe_mill_all
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/millturn_project.ocam
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_mill_all.png)
    add_test(NAME gui.lathe_mill_turned_after
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/millturn_project.ocam --sim-position 9:8
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_mill_turned_after.png)
    add_test(NAME gui.lathe_mill_panel
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/millturn_project.ocam --select-operation 8
                --screenshot-window ${PROJECT_BINARY_DIR}/lathe_mill_panel.png)
    # Live tool axes is on the page: set to polar across the part it is refused, naming it.
    add_test(NAME gui.lathe_mill_panel_polar
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/millturn_project.ocam --select-operation 8
                --set-choice live_axes=polar
                --screenshot-window ${PROJECT_BINARY_DIR}/lathe_mill_panel_polar.png)
    # A slot wrapped around a turned diameter: part way, finished, and its page.
    add_test(NAME gui.lathe_wrap_part_way
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/wrapped_project.ocam --sim-position 3:320
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_wrap_part_way.png)
    add_test(NAME gui.lathe_wrap_all
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/wrapped_project.ocam
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_wrap_all.png)
    add_test(NAME gui.lathe_wrap_panel
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/wrapped_project.ocam --select-operation 3
                --screenshot-window ${PROJECT_BINARY_DIR}/lathe_wrap_panel.png)
    set_tests_properties(gui.lathe_mill_pocket gui.lathe_mill_flat gui.lathe_mill_all
        gui.lathe_mill_turned_after gui.lathe_mill_panel gui.lathe_mill_panel_polar
        gui.lathe_wrap_part_way gui.lathe_wrap_all gui.lathe_wrap_panel
        PROPERTIES LABELS gui TIMEOUT 300)
    # An external thread milled around a boss (DL-0081): part way, finished, and its page.
    add_test(NAME gui.threadmill_external_part_way
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/boss_thread_project.ocam --sim-position 1:9
                --screenshot-sim ${PROJECT_BINARY_DIR}/threadmill_external_part_way.png)
    add_test(NAME gui.threadmill_external_all
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/boss_thread_project.ocam
                --screenshot-sim ${PROJECT_BINARY_DIR}/threadmill_external_all.png)
    add_test(NAME gui.threadmill_external_panel
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/boss_thread_project.ocam --select-operation 1
                --screenshot-window ${PROJECT_BINARY_DIR}/threadmill_external_panel.png)
    set_tests_properties(gui.threadmill_external_part_way gui.threadmill_external_all
        gui.threadmill_external_panel PROPERTIES LABELS gui TIMEOUT 300)
    # The operation's page: Tool drive, the live tool's speed; set to the spindle it is refused.
    add_test(NAME gui.lathe_live_panel
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/live_project.ocam --select-operation 2
                --screenshot-window ${PROJECT_BINARY_DIR}/lathe_live_panel.png)
    add_test(NAME gui.lathe_live_panel_spindle
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/live_project.ocam --select-operation 3
                --set-choice drive=spindle
                --screenshot-window ${PROJECT_BINARY_DIR}/lathe_live_panel_spindle.png)
    add_test(NAME gui.lathe_setup_turret
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/live_project.ocam --select-setup
                --screenshot-window ${PROJECT_BINARY_DIR}/lathe_setup_turret.png)
    set_tests_properties(gui.lathe_live_bolt_circle gui.lathe_live_cross_hole gui.lathe_live_all
        gui.lathe_live_panel gui.lathe_live_panel_spindle gui.lathe_setup_turret
        PROPERTIES LABELS gui TIMEOUT 300)
    # A contour's holding tabs: their settings are on its page only once Holding tabs is ticked,
    # the count or the spacing as placed.
    add_test(NAME gui.contour_tabs_off
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam --select-operation 2
                --expect-params tabs
                --expect-no-params tab_placement,tab_count,tab_spacing,tab_width,tab_height,tab_shape)
    add_test(NAME gui.contour_tabs_on
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam --select-operation 2
                --set-flag tabs=on
                --expect-params tabs,tab_placement,tab_count,tab_width,tab_height,tab_shape
                --expect-no-params tab_spacing)
    add_test(NAME gui.contour_tabs_by_spacing
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam --select-operation 2
                --set-flag tabs=on --set-choice tab_placement=spacing
                --expect-params tab_spacing --expect-no-params tab_count)
    set_tests_properties(gui.contour_tabs_off gui.contour_tabs_on gui.contour_tabs_by_spacing
        PROPERTIES LABELS gui TIMEOUT 300)
    # An operation's right-click menu > Show Toolpath Data: its moves, one row each, as the
    # toolpath holds them, each with a kind of move it must list: milling arcs, a turning finish,
    # a canned cycle (one row), a live tool's axis, then one that is refused, and a long toolpath.
    add_test(NAME gui.toolpath_data_milling
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam --select-operation 1
                --toolpath-data-test ${PROJECT_BINARY_DIR}/toolpath_data_milling.png
                --toolpath-data-expect "Arc")
    add_test(NAME gui.toolpath_data_turning
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/shaft_project.ocam --select-operation 2
                --toolpath-data-test ${PROJECT_BINARY_DIR}/toolpath_data_turning.png
                --toolpath-data-expect "Linear")
    add_test(NAME gui.toolpath_data_cycle
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/bored_project.ocam --select-operation 1
                --toolpath-data-test ${PROJECT_BINARY_DIR}/toolpath_data_cycle.png
                --toolpath-data-expect "cycle")
    add_test(NAME gui.toolpath_data_live
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/live_project.ocam --select-operation 3
                --toolpath-data-test ${PROJECT_BINARY_DIR}/toolpath_data_live.png
                --toolpath-data-expect "Linear")
    add_test(NAME gui.toolpath_data_refused
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/millturn_project.ocam --select-operation 8
                --set-choice live_axes=polar
                --toolpath-data-test ${PROJECT_BINARY_DIR}/toolpath_data_refused.png
                --toolpath-data-expect refused)
    add_test(NAME gui.toolpath_data_long
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam
                --toolpath-data-long ${PROJECT_BINARY_DIR}/toolpath_data_long.png)
    set_tests_properties(gui.toolpath_data_milling gui.toolpath_data_turning gui.toolpath_data_cycle
        gui.toolpath_data_live gui.toolpath_data_refused gui.toolpath_data_long
        PROPERTIES LABELS gui TIMEOUT 300)
    # Threads in the stock: the shaft's at the end of its pass, and every kind of them.
    add_test(NAME gui.lathe_thread_stock
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/shaft_project.ocam --sim-position 3:100000
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_thread_stock.png)
    add_test(NAME gui.lathe_thread_part_way
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/shaft_project.ocam --sim-position 3:8
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_thread_part_way.png)
    add_test(NAME gui.lathe_threads_all_kinds
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/threads_project.ocam
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_threads_all_kinds.png)
    set_tests_properties(gui.lathe_thread_stock gui.lathe_thread_part_way gui.lathe_threads_all_kinds
        PROPERTIES LABELS gui TIMEOUT 300)
    # A turning tool's holder in playback: a shank outside, a bar in the bore.
    add_test(NAME gui.lathe_holder_outside
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/holders_project.ocam --sim-position 2:20
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_holder_outside.png)
    add_test(NAME gui.lathe_holder_inside
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/holders_project.ocam --sim-position 3:10
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_holder_inside.png)
    set_tests_properties(gui.lathe_holder_outside gui.lathe_holder_inside
        PROPERTIES LABELS gui TIMEOUT 300)
    # The second side of a part: the stock as the first side left it, flipped.
    add_test(NAME gui.lathe_simulation_flip
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/flip_project.ocam --sim-last-setup
                --sim-position 0:0
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_simulation_flip.png)
    add_test(NAME gui.lathe_simulation_tailstock
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/centres_project.ocam --sim-last-setup
                --sim-position 0:20
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_simulation_tailstock.png)
    set_tests_properties(gui.lathe_simulation_drill gui.lathe_simulation_bored
        gui.lathe_simulation_flip gui.lathe_simulation_tailstock
        PROPERTIES LABELS gui TIMEOUT 300)
    # A lathe's machine (DL-0067): the turret, chuck, tailstock and enclosure in playback.
    add_test(NAME gui.lathe_machine_rough
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_project.ocam --sim-position 1:30
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_machine_rough.png)
    add_test(NAME gui.lathe_machine_collisions
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_collisions_project.ocam --sim-position 1:53
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_machine_collisions.png)
    add_test(NAME gui.lathe_machine_tailstock
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_tailstock_project.ocam --sim-position 0:20
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_machine_tailstock.png)
    add_test(NAME gui.lathe_machine_sub_spindle
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_sub_project.ocam --sim-last-setup
                --sim-position 1:10
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_machine_sub_spindle.png)
    add_test(NAME gui.lathe_machine_setup_turret
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_project.ocam --select-setup
                --screenshot-window ${PROJECT_BINARY_DIR}/lathe_machine_setup_turret.png)
    add_test(NAME gui.lathe_machine_live
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_live_project.ocam --sim-position 0:5
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_machine_live.png)
    add_test(NAME gui.lathe_machine_turret_edit
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_tailstock_project.ocam --select-setup
                --turret-edit-test
                --screenshot-window ${PROJECT_BINARY_DIR}/lathe_machine_turret_edit.png)
    # Tools and blocks put on the turret's stations through the lists' pop-ups, on a turret whose
    # odd stations take milling units only (DL-0068).
    add_test(NAME gui.lathe_machine_station_tools
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --fittings-folder ${PROJECT_BINARY_DIR}/station-tools-library
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_project.ocam --select-setup
                --station-tools-test
                --screenshot-window ${PROJECT_BINARY_DIR}/lathe_machine_station_tools.png)
    set_tests_properties(gui.lathe_machine_station_tools PROPERTIES LABELS gui TIMEOUT 300
        PASS_REGULAR_EXPRESSION "station tools test: all ok")
    # The turret preview beside the setup page: opened from the turret section, following the
    # row chosen in the table and the project, the station's tool brought to the work.
    add_test(NAME gui.turret_preview
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/turret-preview-settings
                --fittings-folder ${PROJECT_BINARY_DIR}/turret-preview-library
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_project.ocam --select-setup
                --turret-preview-test
                --screenshot-window ${PROJECT_BINARY_DIR}/turret_preview.png)
    set_tests_properties(gui.turret_preview PROPERTIES LABELS gui TIMEOUT 300
        PASS_REGULAR_EXPRESSION "turret preview test: all ok")
    # The libraries opened from the setup page's lists (Manage machine library..., Manage
    # blocks..., Manage centres...), the page rebuilt after them: a machine used from there; then,
    # in a run of its own (the machine library opens once a run), each library closed.
    add_test(NAME gui.setup_machine_list
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/setup-machine-list-settings
                --fittings-folder ${PROJECT_BINARY_DIR}/setup-machine-list-library
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_project.ocam --select-setup
                --setup-machine-list-test
                --screenshot-window ${PROJECT_BINARY_DIR}/setup_machine_list.png)
    set_tests_properties(gui.setup_machine_list PROPERTIES LABELS gui TIMEOUT 300
        PASS_REGULAR_EXPRESSION "setup machine list test: all ok")
    add_test(NAME gui.setup_machine_list_close
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/setup-machine-list-close-settings
                --fittings-folder ${PROJECT_BINARY_DIR}/setup-machine-list-close-library
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_project.ocam --select-setup
                --setup-machine-list-test --setup-machine-list-close
                --screenshot-window ${PROJECT_BINARY_DIR}/setup_machine_list_close.png)
    set_tests_properties(gui.setup_machine_list_close PROPERTIES LABELS gui TIMEOUT 300
        PASS_REGULAR_EXPRESSION "setup machine list test: all ok")
    # The libraries' Show built-ins box (machines, turret blocks and centres, posts), in throwaway
    # libraries with settings of its own.
    add_test(NAME gui.show_builtins
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/show-builtins-settings
                --show-builtins-test ${OCAM_FIXTURE_DIR}/turret_project.ocam
                --screenshot-window ${PROJECT_BINARY_DIR}/show_builtins.png)
    set_tests_properties(gui.show_builtins PROPERTIES LABELS gui TIMEOUT 300
        PASS_REGULAR_EXPRESSION "show built-ins test: all ok")
    # The view as the setup's machine stands (DL-0077): a horizontal lathe, the same as a vertical
    # lathe, then no machine.
    add_test(NAME gui.machine_orientation_view
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_project.ocam --select-setup
                --orientation-test
                --screenshot-window ${PROJECT_BINARY_DIR}/machine_orientation_view.png)
    set_tests_properties(gui.machine_orientation_view PROPERTIES LABELS gui TIMEOUT 300)
    # Jaw sets on the chuck's master jaws and soft jaws bored to fit (DL-0069).
    add_test(NAME gui.jaws_setup
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/chuck_project.ocam --select-setup --jaws-test
                --screenshot-window ${PROJECT_BINARY_DIR}/jaws_setup.png)
    set_tests_properties(gui.jaws_setup PROPERTIES LABELS gui TIMEOUT 300
        PASS_REGULAR_EXPRESSION "jaws test: all ok")
    # The workholding dialog (DL-0078), its library a folder of the build's own (the test writes
    # the same file into it each run).
    add_test(NAME gui.workholding_setup
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --workholding-folder ${PROJECT_BINARY_DIR}/workholding-setup-library
                --smoke-test ${OCAM_FIXTURE_DIR}/chuck_project.ocam --select-setup
                --workholding-setup-test
                --screenshot-window ${PROJECT_BINARY_DIR}/workholding_setup.png)
    set_tests_properties(gui.workholding_setup PROPERTIES LABELS gui TIMEOUT 300
        PASS_REGULAR_EXPRESSION "workholding setup test: all ok")
    add_test(NAME gui.workholding_jaws
        COMMAND ${CMAKE_COMMAND} -DGUI=$<TARGET_FILE:oxcam-gui>
                -DFIXTURES=${OCAM_FIXTURE_DIR} -DOUT=${PROJECT_BINARY_DIR}/workholding-jaws
                -DSETTINGS=${PROJECT_BINARY_DIR}/test-settings
                -P ${CMAKE_CURRENT_SOURCE_DIR}/WorkholdingJawsTest.cmake)
    set_tests_properties(gui.workholding_jaws PROPERTIES LABELS gui TIMEOUT 300)
    set_tests_properties(gui.lathe_machine_turret_edit PROPERTIES LABELS gui TIMEOUT 300
        PASS_REGULAR_EXPRESSION "station test: centre ok, station 2 ok, station 1 ok")
    set_tests_properties(gui.lathe_machine_rough gui.lathe_machine_collisions
        gui.lathe_machine_tailstock gui.lathe_machine_sub_spindle gui.lathe_machine_setup_turret
        gui.lathe_machine_live
        PROPERTIES LABELS gui TIMEOUT 300)
    # Playback from the start of the part-off: the index, the rapid and the cut, and frames coming.
    add_test(NAME gui.lathe_machine_play
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_project.ocam --sim-position 3:0
                --sim-play 2 --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_machine_play.png)
    set_tests_properties(gui.lathe_machine_play PROPERTIES LABELS gui TIMEOUT 300
        PASS_REGULAR_EXPRESSION "playback: from 3:0 to [0-9]+:[0-9]+, [0-9]+ frames")
    add_test(NAME gui.lathe_simulation_play
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/shaft_project.ocam --sim-position 1:0
                --sim-play 3 --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_simulation_play.png)
    set_tests_properties(gui.lathe_simulation_play PROPERTIES LABELS gui TIMEOUT 300
        PASS_REGULAR_EXPRESSION "playback: from 1:0 to [1-4]:[0-9]+, [0-9]+ frames")
    # A 2D trace picks single edges: two of the block's top edges meeting at a corner make one
    # open chain (DL-0088).
    add_test(NAME gui.trace_pick_open_chain
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam
                --trace-pick-test 50,0,30/100,30,30
                --screenshot-window ${PROJECT_BINARY_DIR}/trace_pick_open_chain.png)
    set_tests_properties(gui.trace_pick_open_chain PROPERTIES LABELS gui TIMEOUT 120
        PASS_REGULAR_EXPRESSION "trace edges: 2, chains: 1, open: 1")
    # The traces in the simulation (the corner's groove and the ring beside the pocket), and a
    # trace's page.
    add_test(NAME gui.trace_simulation
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/trace_project.ocam
                --screenshot-sim ${PROJECT_BINARY_DIR}/trace_simulation.png)
    add_test(NAME gui.trace_panel
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/trace_project.ocam --select-operation 2
                --screenshot-window ${PROJECT_BINARY_DIR}/trace_panel.png)
    set_tests_properties(gui.trace_simulation gui.trace_panel PROPERTIES LABELS gui TIMEOUT 300)
    # Clicking a face of the part selects it (regression: OCCT 8 selection activation).
    add_test(NAME gui.pick_face
        COMMAND oxcam-gui --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam --pick-test)
    set_tests_properties(gui.pick_face PROPERTIES LABELS gui TIMEOUT 120)
    # The mouse wheel changes a field only when it has the focus.
    add_test(NAME gui.wheel_guard
        COMMAND oxcam-gui --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam --select-setup
                --wheel-test)
    set_tests_properties(gui.wheel_guard PROPERTIES LABELS gui TIMEOUT 120)
    # Posting asks for the post and its options, then shows the program; its progress dialog is
    # gone by then.
    add_test(NAME gui.post
        COMMAND oxcam-gui --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam
                --screenshot-post ${PROJECT_BINARY_DIR}/sample_project_post.png
                --screenshot-post-settings ${PROJECT_BINARY_DIR}/sample_project_post_settings.png)
    set_tests_properties(gui.post PROPERTIES LABELS gui TIMEOUT 120)
    # The Properties panel at every dock width: fields never squeezed by a long label, a page
    # wider than the dock scrolls instead of being cut off.
    add_test(NAME gui.panel_widths
        COMMAND oxcam-gui --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam
                --select-operation 1 --panel-sweep ${PROJECT_BINARY_DIR}/panel_sweep)
    set_tests_properties(gui.panel_widths PROPERTIES LABELS gui TIMEOUT 180)
    # The setup page with its Stock settings, and a cylinder's (the 4th-axis bar).
    add_test(NAME gui.panel_widths_setup
        COMMAND oxcam-gui --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam
                --select-setup --panel-sweep ${PROJECT_BINARY_DIR}/panel_sweep_setup)
    set_tests_properties(gui.panel_widths_setup PROPERTIES LABELS gui TIMEOUT 180)
    add_test(NAME gui.panel_widths_cylinder
        COMMAND oxcam-gui --smoke-test ${OCAM_FIXTURE_DIR}/rotary_project.ocam
                --select-setup --panel-sweep ${PROJECT_BINARY_DIR}/panel_sweep_cylinder)
    set_tests_properties(gui.panel_widths_cylinder PROPERTIES LABELS gui TIMEOUT 180)
    add_test(NAME gui.start_hub
        COMMAND oxcam-gui --screenshot-hub ${PROJECT_BINARY_DIR}/start_hub.png)
    set_tests_properties(gui.start_hub PROPERTIES LABELS gui TIMEOUT 60)
    # A lathe's turret in the machine editor; a plain lathe shows the same group, unticked.
    add_test(NAME gui.machine_library_live_lathe
        COMMAND oxcam-gui --screenshot-machines ${PROJECT_BINARY_DIR}/machine_library_live_lathe.png
                --machine generic-lathe-live-y)
    set_tests_properties(gui.machine_library_live_lathe PROPERTIES LABELS gui TIMEOUT 60)
    # Tooling blocks and centres (DL-0068).
    add_test(NAME gui.fittings_library
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --screenshot-fittings ${PROJECT_BINARY_DIR}/fittings_library.png)
    add_test(NAME gui.lathe_machine_blocks
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/turret_blocks_project.ocam --sim-position 0:20
                --screenshot-sim ${PROJECT_BINARY_DIR}/lathe_machine_blocks.png)
    add_test(NAME gui.fittings_edit
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --fittings-edit-test ${PROJECT_SOURCE_DIR}/data/fittings/axial-block.step)
    add_test(NAME gui.fittings_block_on_lathe
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --screenshot-fittings ${PROJECT_BINARY_DIR}/fittings_block_on_lathe.png
                --fitting "Block for a 25 mm shank across (generic)" --fitting-on-lathe)
    add_test(NAME gui.fittings_bar_block
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --screenshot-fittings ${PROJECT_BINARY_DIR}/fittings_bar_block.png
                --fitting "Block for a 32 mm bar along (generic)")
    add_test(NAME gui.fittings_centre_on_lathe
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --screenshot-fittings ${PROJECT_BINARY_DIR}/fittings_centre_on_lathe.png
                --fitting "Live centre (generic)" --fitting-on-lathe)
    set_tests_properties(gui.fittings_library gui.fittings_edit gui.lathe_machine_blocks
                         gui.fittings_block_on_lathe gui.fittings_bar_block
                         gui.fittings_centre_on_lathe
        PROPERTIES LABELS gui TIMEOUT 300)
    add_test(NAME gui.machine_library_lathe_layout
        COMMAND oxcam-gui --screenshot-machines ${PROJECT_BINARY_DIR}/machine_library_lathe_layout.png
                --machine generic-lathe-sub)
    set_tests_properties(gui.machine_library_lathe_layout PROPERTIES LABELS gui TIMEOUT 60)
    add_test(NAME gui.machine_library
        COMMAND oxcam-gui --screenshot-machines ${PROJECT_BINARY_DIR}/machine_library.png)
    set_tests_properties(gui.machine_library PROPERTIES LABELS gui TIMEOUT 60)
    # A machine's STEP file split into its bodies, and an assembly merged by subassembly
    # (DL-0037).
    add_test(NAME gui.machine_parts_import
        COMMAND oxcam-gui --screenshot-parts-import ${PROJECT_BINARY_DIR}/machine_parts_import.png
                ${OCAM_FIXTURE_DIR}/trunnion_machine.step)
    set_tests_properties(gui.machine_parts_import PROPERTIES LABELS gui TIMEOUT 60)
    add_test(NAME gui.machine_assembly_import
        COMMAND oxcam-gui --screenshot-parts-import ${PROJECT_BINARY_DIR}/machine_assembly_import.png
                --merge-level 1 ${OCAM_FIXTURE_DIR}/trunnion_assembly.step)
    set_tests_properties(gui.machine_assembly_import PROPERTIES LABELS gui TIMEOUT 60)
    # In the machine library (the fixtures' demo machine), a selected part is highlighted and
    # isolated in the preview, and clicking it there selects its row again.
    file(WRITE ${PROJECT_BINARY_DIR}/test-settings-machines/OxCAM/OxCAM.ini
         "[General]\nmachineLibraryFolder=${OCAM_FIXTURE_DIR}/machines\n")
    add_test(NAME gui.machine_part_pick
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings-machines
                --screenshot-machines ${PROJECT_BINARY_DIR}/machine_part_pick.png
                --machine trunnion-demo --select-part cradle --isolate --pick-preview)
    set_tests_properties(gui.machine_part_pick PROPERTIES LABELS gui TIMEOUT 60
        PASS_REGULAR_EXPRESSION "picked: cradle")
    # The C axis picked on the round table's rim (shown alone): its centre line.
    add_test(NAME gui.machine_axis_pick
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings-machines
                --screenshot-machines ${PROJECT_BINARY_DIR}/machine_axis_pick.png
                --machine trunnion-demo --select-part "rotary table" --isolate
                --pick-axis C@0,-210,-25)
    # The spindle gauge point picked on the spindle nose: its outside, then its seat face.
    add_test(NAME gui.machine_gauge_pick
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings-machines
                --screenshot-machines ${PROJECT_BINARY_DIR}/machine_gauge_pick.png
                --machine trunnion-demo --select-part "spindle nose" --isolate
                "--pick-gauge=0,-55,505;20,10,450")
    # How a machine stands, chosen in its editor (DL-0077): the demo trunnion stood on its side,
    # Y up and its front at +Z (not saved).
    add_test(NAME gui.machine_library_orientation
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings-machines
                --screenshot-machines ${PROJECT_BINARY_DIR}/machine_library_orientation.png
                --machine trunnion-demo "--machine-stands=+Y,+Z")
    set_tests_properties(gui.machine_library_orientation PROPERTIES LABELS gui TIMEOUT 60
        PASS_REGULAR_EXPRESSION "after: stands: \\+Y up, front \\+Z[;] preview up \\(-0\\.41, 0\\.82, -0\\.41\\)[;] cube axes right, back, up: X -Z Y\n")
    set_tests_properties(gui.machine_gauge_pick PROPERTIES LABELS gui TIMEOUT 60
        PASS_REGULAR_EXPRESSION "gauge point \\(0\\.0, 0\\.0, 450\\.0\\)")
    # The plate of the 3+2 project placed by picks: its right face's centre (a point of the
    # work) onto the round table where the C axis meets its top (carried by C).
    add_test(NAME gui.setup_placement_pick
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings-machines
                --smoke-test ${OCAM_FIXTURE_DIR}/plate32_machine_project.ocam
                --screenshot-placement ${PROJECT_BINARY_DIR}/setup_placement_pick.png
                --place-snap axis --place-work 40,0,-40 --place-on 0,-190,0)
    set_tests_properties(gui.setup_placement_pick PROPERTIES LABELS gui TIMEOUT 120
        PASS_REGULAR_EXPRESSION "WCS origin is at \\(-40[.0]*, 0[.0]*, 40[.0]* mm\\), carried by the C axis")
    # On the STEP part's exact geometry: the rim's cylinder, radius exactly the table's.
    set_tests_properties(gui.machine_axis_pick PROPERTIES LABELS gui TIMEOUT 60
        PASS_REGULAR_EXPRESSION "round face of 'rotary table' \\(radius 210\\.00 mm\\): pivot \\(0\\.0, 0\\.0, -25\\.0\\), direction \\(0\\.0, 0\\.0, 1\\.0\\)")
    # The C axis from the round table's top edge, a circle of the B-rep: at the table's top.
    add_test(NAME gui.machine_axis_pick_edge
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings-machines
                --screenshot-machines ${PROJECT_BINARY_DIR}/machine_axis_pick_edge.png
                --machine trunnion-demo --select-part "rotary table" --isolate
                --pick-axis C@0,-210,0)
    set_tests_properties(gui.machine_axis_pick_edge PROPERTIES LABELS gui TIMEOUT 60
        PASS_REGULAR_EXPRESSION "circular edge of 'rotary table' \\(radius 210\\.00 mm\\): pivot \\(0\\.0, 0\\.0, 0\\.0\\), direction \\(0\\.0, 0\\.0, 1\\.0\\)")
    # The library lists the test vise (vise.ocamhold among the fixtures) and edits it.
    add_test(NAME gui.workholding_library
        COMMAND oxcam-gui --workholding-folder ${OCAM_FIXTURE_DIR}
                --screenshot-workholding ${PROJECT_BINARY_DIR}/workholding_library.png)
    set_tests_properties(gui.workholding_library PROPERTIES LABELS gui TIMEOUT 60)
    # The editor makes a template of a multi-body STEP file (every body a part) and picks a datum.
    add_test(NAME gui.workholding_editor
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings-workholding
                --workholding-folder ${PROJECT_BINARY_DIR}/test-workholding-library
                --screenshot-workholding ${PROJECT_BINARY_DIR}/workholding_editor.png
                --workholding-step ${OCAM_FIXTURE_DIR}/dovetail_assembly.step)
    set_tests_properties(gui.workholding_editor PROPERTIES LABELS gui TIMEOUT 60)
    # The setup wizard makes a self-centring vise of clicks; its jaws are linked.
    add_test(NAME gui.workholding_wizard
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings-workholding
                --screenshot-wizard ${PROJECT_BINARY_DIR}/workholding_wizard.png
                --workholding-step ${OCAM_FIXTURE_DIR}/selfcentring_assembly.step)
    set_tests_properties(gui.workholding_wizard PROPERTIES LABELS gui TIMEOUT 60)
    # A 3-jaw chuck from clicks: its jaws move together and grip a trial bar.
    add_test(NAME gui.workholding_wizard_chuck
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings-workholding
                --screenshot-wizard ${PROJECT_BINARY_DIR}/workholding_wizard_chuck.png
                --workholding-step ${OCAM_FIXTURE_DIR}/chuck_assembly.step)
    set_tests_properties(gui.workholding_wizard_chuck PROPERTIES LABELS gui TIMEOUT 60)
    # A maker's file with extras: a riser removed and a jaw's two bodies merged in the wizard.
    add_test(NAME gui.workholding_wizard_parts
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings-workholding
                --screenshot-wizard ${PROJECT_BINARY_DIR}/workholding_wizard_parts.png
                --workholding-step ${OCAM_FIXTURE_DIR}/selfcentring_extras.step)
    set_tests_properties(gui.workholding_wizard_parts PROPERTIES LABELS gui TIMEOUT 60)
    # A setup with a vise template: the vise shows, the setup page shows its jaws.
    add_test(NAME gui.smoke_test_workholding
        COMMAND oxcam-gui --workholding-folder ${OCAM_FIXTURE_DIR}
                --smoke-test ${OCAM_FIXTURE_DIR}/vise_project.ocam --select-setup
                --screenshot-window ${PROJECT_BINARY_DIR}/vise_project.png)
    set_tests_properties(gui.smoke_test_workholding PROPERTIES LABELS gui TIMEOUT 120)
    # The WCS origin kept at the vise's mount datum: the setup page shows it, its fields locked.
    add_test(NAME gui.smoke_test_workholding_origin
        COMMAND oxcam-gui --workholding-folder ${OCAM_FIXTURE_DIR}
                --smoke-test ${OCAM_FIXTURE_DIR}/vise_origin_project.ocam --select-setup
                --screenshot-window ${PROJECT_BINARY_DIR}/vise_origin_project.png)
    set_tests_properties(gui.smoke_test_workholding_origin PROPERTIES LABELS gui TIMEOUT 120)
    # The WCS origin picked in the view: a face of the part, shown on the setup page.
    add_test(NAME gui.wcs_pick
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam --select-setup
                --wcs-pick-test --screenshot-window ${PROJECT_BINARY_DIR}/wcs_pick.png)
    set_tests_properties(gui.wcs_pick PROPERTIES LABELS gui TIMEOUT 120)
    # X from a picked edge of the part (its top edge along Y at X 0): X along the edge.
    add_test(NAME gui.wcs_axis_pick
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam --select-setup
                --wcs-axis-test 0,30,30)
    set_tests_properties(gui.wcs_axis_pick PROPERTIES LABELS gui TIMEOUT 120
        PASS_REGULAR_EXPRESSION "WCS X: [-]?0\\.000000 [-]?1\\.000000 [-]?0\\.000000")
    # Settings of their own: the preferences of whoever runs the tests stay untouched.
    add_test(NAME gui.preferences
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --screenshot-preferences ${PROJECT_BINARY_DIR}/preferences.png
                --preferences-page 3 --preferences-type zlevel_rough)
    set_tests_properties(gui.preferences PROPERTIES LABELS gui TIMEOUT 60)
    # A key binding in the settings replaces the command's built-in shortcut.
    file(WRITE ${PROJECT_BINARY_DIR}/test-settings-bindings/OxCAM/OxCAM.ini
         "[shortcuts]\nsetup.simulate=Ctrl+Shift+F5\n")
    add_test(NAME gui.command_bindings
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings-bindings
                --list-commands)
    # Settings of OxCAM, formerly OpenCAM, carry over on the first start, once (ADR-0005).
    add_test(NAME gui.settings_from_opencam
        COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:oxcam-gui>
                -DDIR=${PROJECT_BINARY_DIR}/test-settings-opencam
                -P ${CMAKE_CURRENT_SOURCE_DIR}/CheckOpenCamSettings.cmake)
    set_tests_properties(gui.settings_from_opencam PROPERTIES LABELS gui TIMEOUT 60)
    # i18n/LibraryStrings.cpp (the libraries' texts, for lupdate) is up to date.
    add_test(NAME gui.library_strings
        COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:oxcam-gui>
                -DOUT=${PROJECT_BINARY_DIR}/LibraryStrings.cpp
                -DREF=${CMAKE_CURRENT_SOURCE_DIR}/i18n/LibraryStrings.cpp
                -P ${CMAKE_CURRENT_SOURCE_DIR}/i18n/CheckLibraryStrings.cmake)
    set_tests_properties(gui.library_strings PROPERTIES LABELS gui TIMEOUT 60)
    set_tests_properties(gui.command_bindings PROPERTIES LABELS gui TIMEOUT 60
        PASS_REGULAR_EXPRESSION "setup\\.simulate\tCtrl\\+Shift\\+F5\n")
    # A key two toolpath modes share toggles between them (from whichever mode the last run
    # left saved).
    file(WRITE ${PROJECT_BINARY_DIR}/test-settings-cycle/OxCAM/OxCAM.ini
         "[General]\ntoolpathDisplay=0\n[shortcuts]\nview.toolpaths.all=T\nview.toolpaths.hidden=T\n")
    add_test(NAME gui.shared_shortcut
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings-cycle
                --press-shortcut T)
    set_tests_properties(gui.shared_shortcut PROPERTIES LABELS gui TIMEOUT 60
        PASS_REGULAR_EXPRESSION
        "view\\.toolpaths\\.hidden\r?\nview\\.toolpaths\\.all\r?\nview\\.toolpaths\\.hidden;view\\.toolpaths\\.all\r?\nview\\.toolpaths\\.hidden\r?\nview\\.toolpaths\\.all")
    # An operation's tooltip shows, titled and wrapped.
    add_test(NAME gui.tooltip
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --screenshot-tooltip ${PROJECT_BINARY_DIR}/tooltip.png
                --tooltip-command operation.zlevel_rough)
    set_tests_properties(gui.tooltip PROPERTIES LABELS gui TIMEOUT 60)
    # Enter in a tool editor field commits it and presses no button (it duplicated the tool).
    # The turning preview draws the insert; ISO holders have their own editor and draw with it.
    add_test(NAME gui.tool_preview_turning
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --screenshot-tools ${PROJECT_BINARY_DIR}/tool_preview_turning.png
                ${PROJECT_SOURCE_DIR}/data/tool-libraries/sample-metric.ocamtools
                --new-tool turning_insert)
    add_test(NAME gui.turning_holder_shank
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --screenshot-tools ${PROJECT_BINARY_DIR}/turning_holder_shank.png
                ${PROJECT_SOURCE_DIR}/data/tool-libraries/sample-metric.ocamtools
                --new-turning-holder shank)
    add_test(NAME gui.turning_holder_boring
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --screenshot-tools ${PROJECT_BINARY_DIR}/turning_holder_boring.png
                ${PROJECT_SOURCE_DIR}/data/tool-libraries/sample-metric.ocamtools
                --new-turning-holder boring)
    add_test(NAME gui.solid_boring_bar
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --screenshot-tools ${PROJECT_BINARY_DIR}/solid_boring_bar.png
                ${PROJECT_SOURCE_DIR}/data/tool-libraries/sample-metric.ocamtools
                --new-tool solid_boring_bar)
    set_tests_properties(gui.solid_boring_bar PROPERTIES LABELS gui TIMEOUT 60)
    # The holder editor driven through its widgets: what the checks do is in HolderEditorCheck.cpp.
    add_test(NAME gui.holder_editor
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --check-holder-editor ${PROJECT_BINARY_DIR}/holder_editor)
    set_tests_properties(gui.holder_editor PROPERTIES LABELS gui TIMEOUT 60)
    add_test(NAME gui.turning_holder_plain
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --screenshot-tools ${PROJECT_BINARY_DIR}/turning_holder_plain.png
                ${PROJECT_SOURCE_DIR}/data/tool-libraries/sample-metric.ocamtools
                --new-turning-holder plain)
    set_tests_properties(gui.turning_holder_plain PROPERTIES LABELS gui TIMEOUT 60)
    set_tests_properties(gui.tool_preview_turning gui.turning_holder_shank
        gui.turning_holder_boring PROPERTIES LABELS gui TIMEOUT 60)
    add_test(NAME gui.tool_enter
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/test-settings
                --screenshot-tools ${PROJECT_BINARY_DIR}/tool_enter.png
                ${PROJECT_SOURCE_DIR}/data/tool-libraries/sample-metric.ocamtools
                --select-tool 1 --press-enter)
    set_tests_properties(gui.tool_enter PROPERTIES LABELS gui TIMEOUT 60)
    # Picking another operation of the setup while its in-process stock is simulated, and the
    # first again, lets the simulation go on (it started over).
    add_test(NAME gui.stock_reselect
        COMMAND oxcam-gui --settings-dir ${PROJECT_BINARY_DIR}/reselect-settings
                --smoke-test ${OCAM_FIXTURE_DIR}/sample_project.ocam --reselect-test)
    set_tests_properties(gui.stock_reselect PROPERTIES LABELS gui TIMEOUT 180)
    # GUI tests never touch the user's simulation cache.
    get_property(guiTests DIRECTORY PROPERTY TESTS)
    set_property(TEST ${guiTests} APPEND PROPERTY ENVIRONMENT "OXCAM_SIMULATION_CACHE=none"
        "OCAM_MEMORY_CAP_GB=${OCAM_TEST_MEMORY_CAP_GB}")
endif()
