# Qt desktop shell. Thin glue over carvetto_viewmodel (ADR-0014); no class uses Q_OBJECT, so
# AUTOMOC (enabled by qt_standard_project_setup) has nothing to generate.
qt_standard_project_setup()

# The starter tool database is compiled in (same mechanism as carvetto-cli, ADR-0019): the Toolpaths
# dock offers its tools until the tool-database UI exists.
set(CARVETTO_STARTER_TOOLS_JSON ${PROJECT_SOURCE_DIR}/data/tools/starter_tools.json)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CARVETTO_STARTER_TOOLS_JSON})
file(READ ${CARVETTO_STARTER_TOOLS_JSON} _starter_hex HEX)
string(LENGTH "${_starter_hex}" _starter_hex_len)
math(EXPR _starter_size "${_starter_hex_len} / 2")
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," _starter_bytes "${_starter_hex}")
string(REGEX REPLACE "(0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,)" "\\1\n" _starter_bytes "${_starter_bytes}")
configure_file(src/starter_tools_embedded.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/app_starter_tools_embedded.cpp @ONLY)

qt_add_executable(carvetto
  src/main.cpp
  src/bitmap_decoder.cpp
  src/bitmap_decoder.hpp
  src/array_copy_menu.cpp
  src/array_copy_menu.hpp
  src/main_window.cpp
  src/main_window.hpp
  src/canvas_widget.cpp
  src/canvas_widget.hpp
  src/ruler_widget.cpp
  src/ruler_widget.hpp
  src/dimension_menu.cpp
  src/dimension_menu.hpp
  src/document_panels.cpp
  src/document_panels.hpp
  src/new_job_dialog.cpp
  src/new_job_dialog.hpp
  src/node_edit_menu.cpp
  src/node_edit_menu.hpp
  src/preview_panel.cpp
  src/preview_panel.hpp
  src/preview_self_test.cpp
  src/panels_self_test.cpp
  src/properties_panel.cpp
  src/properties_panel.hpp
  src/relief_view.cpp
  src/relief_view.hpp
  src/tool_panel.cpp
  src/tool_panel.hpp
  src/save_toolpaths_dialog.cpp
  src/save_toolpaths_dialog.hpp
  src/tool_setup_dialog.cpp
  src/tool_setup_dialog.hpp
  src/tool_setup_self_test.cpp
  src/main_window_setup.cpp
  src/toolpath_form_dialog.cpp
  src/toolpath_form_dialog.hpp
  src/toolpaths_panel.cpp
  src/toolpaths_panel.hpp
  src/toolpaths_self_test.cpp
  src/photo_self_test.cpp
  src/self_test.cpp
  src/self_test.hpp
  src/starter_tools.hpp
  ${CMAKE_CURRENT_BINARY_DIR}/app_starter_tools_embedded.cpp
  src/texture_menu.cpp
  src/texture_menu.hpp
  src/transform_menu.cpp
  src/transform_menu.hpp
  src/keepout_panel.cpp
  src/keepout_self_test.cpp
  src/keepout_panel.hpp
  src/vector_ops_menu.cpp
  src/vector_ops_menu.hpp
)
# Development builds find the shipped posts in the source tree (a packaged app has posts/ beside it).
target_compile_definitions(carvetto PRIVATE CARVETTO_SHIPPED_POSTS_DIR="${PROJECT_SOURCE_DIR}/posts")
target_link_libraries(carvetto PRIVATE carvetto::viewmodel carvetto::core Qt6::Widgets carvetto_warnings)
# GUI subsystem on Windows (no console window). CI waits for `--self-test` with
# Start-Process -Wait and redirected output. See ADR-0014.
set_target_properties(carvetto PROPERTIES WIN32_EXECUTABLE ON)
target_include_directories(carvetto PRIVATE src)
