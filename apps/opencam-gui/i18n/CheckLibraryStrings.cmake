# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Test gui.library_strings: writes the libraries' texts with the GUI (EXE) to OUT and fails
# when they differ from the committed REF. To update REF:
#   oxcam-gui --write-library-strings apps/opencam-gui/i18n/LibraryStrings.cpp

execute_process(COMMAND ${EXE} --write-library-strings ${OUT} RESULT_VARIABLE result)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "oxcam-gui --write-library-strings failed (${result})")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files --ignore-eol ${OUT} ${REF}
                RESULT_VARIABLE differ)
if(NOT differ EQUAL 0)
    message(FATAL_ERROR "${REF} is out of date: the libraries' texts changed. Regenerate it with "
                        "`oxcam-gui --write-library-strings ${REF}`, then run the "
                        "update_translations target.")
endif()
