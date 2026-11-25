# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

ocam_add_test(machine SOURCES KinematicsTest.cpp MachineTest.cpp ModelTest.cpp TurretTest.cpp)
target_compile_definitions(test_machine PRIVATE OCAM_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
