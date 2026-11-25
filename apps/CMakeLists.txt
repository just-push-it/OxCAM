# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later

add_subdirectory(opencam-cli)

if(OCAM_BUILD_GUI)
    add_subdirectory(opencam-gui)
endif()
