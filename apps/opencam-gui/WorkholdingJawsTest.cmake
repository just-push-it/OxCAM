# SPDX-FileCopyrightText: OxCAM contributors
# SPDX-License-Identifier: GPL-3.0-or-later
#
# gui.workholding_jaws: a throwaway workholding library with a copy of the test chuck, which the
# GUI gives a jaw mount and a jaw set and reads in its Jaws section (DL-0069).
file(REMOVE_RECURSE "${OUT}")
file(MAKE_DIRECTORY "${OUT}/library")
file(COPY "${FIXTURES}/chuck.ocamhold" DESTINATION "${OUT}/library")
execute_process(
    COMMAND "${GUI}" --settings-dir "${SETTINGS}" --workholding-folder "${OUT}/library"
            --workholding-jaws-test "${OUT}/workholding_jaws.png"
    RESULT_VARIABLE result OUTPUT_VARIABLE out ERROR_VARIABLE err)
message("${out}${err}")
if(NOT result EQUAL 0)
    message(FATAL_ERROR "the jaws section did not read or give back the jaws (${result})")
endif()
