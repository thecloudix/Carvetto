# carvetto_viewmodel: Qt-free view logic for the desktop app (ADR-0014). It must never link Qt;
# tests/qt_free_check.cmake enforces that (and that core/ includes no Qt header).
# The Text tool's starter font is compiled in (T-0211): a generated source holds the bytes of
# data/fonts/CarvettoStarter-Regular.ttf, regenerated when the font file changes.
set(STARTER_FONT_FILE ${PROJECT_SOURCE_DIR}/data/fonts/CarvettoStarter-Regular.ttf)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${STARTER_FONT_FILE})
file(READ ${STARTER_FONT_FILE} STARTER_FONT_HEX HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," STARTER_FONT_ARRAY "${STARTER_FONT_HEX}")
configure_file(starter_font.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/starter_font.cpp @ONLY)

add_library(carvetto_viewmodel STATIC
  ${CMAKE_CURRENT_BINARY_DIR}/starter_font.cpp
  src/vector_ops.cpp
  src/vector_selector.cpp
  src/view_transform.cpp
  src/ruler.cpp
  src/save_toolpaths.cpp
  src/toolpath_calc.cpp
  src/toolpath_form.cpp
  src/toolpath_runner.cpp
  src/toolpath_session.cpp
  src/tools.cpp
  src/text_tool.cpp
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
