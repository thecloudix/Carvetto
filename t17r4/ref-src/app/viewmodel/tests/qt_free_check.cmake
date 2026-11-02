# Fails when a Qt header is included under core/ or app/viewmodel/, or when carvetto_core /
# carvetto_viewmodel link a Qt target. Run as: cmake -DSOURCE_ROOT=... -DLINK_FILE=... -P <this>
set(offenders "")
foreach(dir core app/viewmodel)
  file(GLOB_RECURSE sources
    "${SOURCE_ROOT}/${dir}/*.hpp" "${SOURCE_ROOT}/${dir}/*.h" "${SOURCE_ROOT}/${dir}/*.cpp")
  foreach(src IN LISTS sources)
    file(STRINGS "${src}" hits REGEX "^[ \t]*#[ \t]*include[ \t]*[<\"]Q")
    if(hits)
      list(APPEND offenders "${src}")
    endif()
  endforeach()
endforeach()
if(offenders)
  message(FATAL_ERROR "Qt include found in Qt-free code: ${offenders}")
endif()

file(READ "${LINK_FILE}" libs)
if(libs MATCHES "Qt[0-9]*::" OR libs MATCHES "Qt6")
  message(FATAL_ERROR "carvetto_core/carvetto_viewmodel link Qt: ${libs}")
endif()
message(STATUS "Qt-free check passed (link libraries: ${libs})")
