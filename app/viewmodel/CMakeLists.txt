# carvetto_viewmodel: Qt-free view logic for the desktop app (ADR-0014). It must never link Qt;
# tests/qt_free_check.cmake enforces that (and that core/ includes no Qt header).
add_library(carvetto_viewmodel STATIC
  src/vector_ops.cpp
  src/vector_selector.cpp
  src/view_transform.cpp
  src/ruler.cpp
  src/toolpath_calc.cpp
  src/toolpath_form.cpp
  src/toolpath_runner.cpp
  src/toolpath_session.cpp
  src/tools.cpp
  src/transform_form.cpp
  src/transform_gesture.cpp
  src/transform_ops.cpp
  src/units_format.cpp
  src/canvas_scene.cpp
  src/array_copies.cpp
  src/pixel_probe.cpp
  src/new_job_form.cpp
  src/dxf_exchange.cpp
  src/file_session.cpp
  src/node_edit.cpp
  src/node_ops.cpp
  src/snapping.cpp
  src/selection_tools.cpp
)
find_package(Threads REQUIRED)
target_include_directories(carvetto_viewmodel PUBLIC include)
target_link_libraries(carvetto_viewmodel
  PUBLIC carvetto::core Threads::Threads
  PRIVATE carvetto_warnings
)
add_library(carvetto::viewmodel ALIAS carvetto_viewmodel)

if(CARVETTO_BUILD_TESTS)
  add_subdirectory(tests)
endif()
