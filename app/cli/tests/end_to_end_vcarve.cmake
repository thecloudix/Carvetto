# T-0153: runs carvetto-cli on the V-carve fixture (sign.dxf -> V-bit letters -> flat-bottomed panel with an
# end mill clearing the flat area -> Grbl) and compares the SHA-256 of the G-code with the V-carve golden.
# Regenerate the golden with tools/regen_golden.py, never by copying this script's output by hand.
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

cli(import "${DATA_DIR}/sign.dxf" "${WORK_DIR}/sign.carv")
cli(toolpath vcarve "${DATA_DIR}/vcarve_letters.json" "${WORK_DIR}/sign.carv" "${WORK_DIR}/sign.carv")
cli(toolpath vcarve "${DATA_DIR}/vcarve_flat.json" "${WORK_DIR}/sign.carv" "${WORK_DIR}/sign.carv")
cli(info "${WORK_DIR}/sign.carv")
cli(post "${POST}" "${WORK_DIR}/sign.carv" "${WORK_DIR}/sign.nc")

file(SHA256 "${WORK_DIR}/sign.nc" got)
file(SHA256 "${GOLDEN}" want)
if(NOT got STREQUAL want)
  message(FATAL_ERROR "G-code checksum ${got} differs from the golden ${want} (${GOLDEN}); "
                      "compare ${WORK_DIR}/sign.nc with it")
endif()
message(STATUS "G-code matches the golden (sha256 ${got})")
