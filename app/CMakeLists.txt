# Qt desktop shell. Thin glue over carvetto_viewmodel (ADR-0014); no class uses Q_OBJECT, so
# AUTOMOC (enabled by qt_standard_project_setup) has nothing to generate.
qt_standard_project_setup()

qt_add_executable(carvetto
  src/main.cpp
  src/array_copy_menu.cpp
  src/array_copy_menu.hpp
  src/main_window.cpp
  src/main_window.hpp
  src/canvas_widget.cpp
  src/canvas_widget.hpp
  src/ruler_widget.cpp
  src/ruler_widget.hpp
  src/new_job_dialog.cpp
  src/new_job_dialog.hpp
  src/node_edit_menu.cpp
  src/node_edit_menu.hpp
  src/tool_panel.cpp
  src/tool_panel.hpp
  src/save_toolpaths_dialog.cpp
  src/save_toolpaths_dialog.hpp
  src/self_test.cpp
  src/self_test.hpp
  src/transform_menu.cpp
  src/transform_menu.hpp
  src/vector_ops_menu.cpp
  src/vector_ops_menu.hpp
)
# Development builds find the shipped posts in the source tree (a packaged app has posts/ beside it).
target_compile_definitions(carvetto PRIVATE CARVETTO_SHIPPED_POSTS_DIR="${PROJECT_SOURCE_DIR}/posts")
target_link_libraries(carvetto PRIVATE carvetto::viewmodel carvetto::core Qt6::Widgets carvetto_warnings)
# GUI subsystem on Windows (no console window). CI waits for `--self-test` with
# Start-Process -Wait and redirected output. See ADR-0014.
set_target_properties(carvetto PROPERTIES WIN32_EXECUTABLE ON)
