# T-0122 AC1: runs carvetto-cli on the fixture (DXF -> pocket -> profile with tabs -> Grbl) and
# compares the SHA-256 of the G-code with the golden. Regenerate the golden with
# tools/regen_golden.py, never by copying this script's output by hand.
# Usage: cmake -DCLI=<carvetto-cli> -DDATA_DIR=<dir> -DPOST=<grbl_mm.pp> -DGOLDEN=<nc> -DWORK_DIR=<dir> -P <this>
file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}")

function(cli)
  execute_process(COMMAND "${CLI}" ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
  message(STATUS "carvetto-cli ${ARGN}\n${out}${err}")
  if(NOT rc EQUAL 0)
    message(FATAL_ERROR "carvetto-cli ${ARGN} failed with exit code ${rc}:\n${err}")
  endif()
endfunction()

cli(import "${DATA_DIR}/part.dxf" "${WORK_DIR}/part.carv")
cli(toolpath pocket "${DATA_DIR}/pocket.json" "${WORK_DIR}/part.carv" "${WORK_DIR}/part.carv")
cli(toolpath profile "${DATA_DIR}/profile.json" "${WORK_DIR}/part.carv" "${WORK_DIR}/part.carv")
cli(info "${WORK_DIR}/part.carv")
cli(post "${POST}" "${WORK_DIR}/part.carv" "${WORK_DIR}/part.nc")

file(SHA256 "${WORK_DIR}/part.nc" got)
file(SHA256 "${GOLDEN}" want)
if(NOT got STREQUAL want)
  message(FATAL_ERROR "G-code checksum ${got} differs from the golden ${want} (${GOLDEN}); "
                      "compare ${WORK_DIR}/part.nc with it")
endif()
message(STATUS "G-code matches the golden (sha256 ${got})")
