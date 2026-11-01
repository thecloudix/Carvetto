# carvetto_viewmodel: Qt-free view logic for the desktop app (ADR-0014). It must never link Qt;
# tests/qt_free_check.cmake enforces that (and that core/ includes no Qt header).
add_library(carvetto_viewmodel STATIC
  src/view_transform.cpp
  src/ruler.cpp
  src/units_format.cpp
  src/canvas_scene.cpp
  src/pixel_probe.cpp
  src/new_job_form.cpp
)
target_include_directories(carvetto_viewmodel PUBLIC include)
target_link_libraries(carvetto_viewmodel
  PUBLIC carvetto::core
  PRIVATE carvetto_warnings
)
add_library(carvetto::viewmodel ALIAS carvetto_viewmodel)

if(CARVETTO_BUILD_TESTS)
  add_subdirectory(tests)
endif()
