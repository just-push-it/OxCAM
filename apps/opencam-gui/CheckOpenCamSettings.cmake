# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Test gui.settings_from_opencam: settings OpenCAM left in DIR (as --settings-dir keeps them)
# carry over to OxCAM on its first start, and only then. EXE is oxcam-gui.

file(REMOVE_RECURSE ${DIR})
file(WRITE ${DIR}/OpenCAM/OpenCAM.ini
     "[General]\nrecentProjects=C:/parts/bracket.ocam\nshowStartHub=false\n"
     "[shortcuts]\nsetup.simulate=Ctrl+Shift+F6\n")

function(list_commands expected)
    execute_process(COMMAND ${EXE} --settings-dir ${DIR} --list-commands
                    OUTPUT_VARIABLE out RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "oxcam-gui --list-commands failed (${result})")
    endif()
    if(NOT out MATCHES "setup\\.simulate\t${expected}\r?\n")
        message(FATAL_ERROR "setup.simulate is not bound to ${expected}:\n${out}")
    endif()
endfunction()

list_commands("Ctrl\\+Shift\\+F6")
if(NOT EXISTS ${DIR}/OxCAM/OxCAM.ini)
    message(FATAL_ERROR "no OxCAM settings were written")
endif()
file(READ ${DIR}/OxCAM/OxCAM.ini copied)
if(NOT copied MATCHES "recentProjects=C:/parts/bracket.ocam" OR NOT copied MATCHES "showStartHub=false")
    message(FATAL_ERROR "OpenCAM's settings did not carry over:\n${copied}")
endif()
if(NOT EXISTS ${DIR}/OpenCAM/OpenCAM.ini)
    message(FATAL_ERROR "OpenCAM's settings were removed")
endif()

# Once OxCAM has settings of its own, OpenCAM's are not read again.
file(WRITE ${DIR}/OpenCAM/OpenCAM.ini "[shortcuts]\nsetup.simulate=Ctrl+Shift+F7\n")
list_commands("Ctrl\\+Shift\\+F6")
