# Qt desktop shell. Thin glue over carvetto_viewmodel (ADR-0014); no class uses Q_OBJECT, so
# AUTOMOC (enabled by qt_standard_project_setup) has nothing to generate.
qt_standard_project_setup()

qt_add_executable(carvetto
  src/main.cpp
  src/main_window.cpp
  src/main_window.hpp
  src/canvas_widget.cpp
  src/canvas_widget.hpp
  src/ruler_widget.cpp
  src/ruler_widget.hpp
  src/new_job_dialog.cpp
  src/new_job_dialog.hpp
  src/self_test.cpp
  src/self_test.hpp
)
target_link_libraries(carvetto PRIVATE carvetto::viewmodel carvetto::core Qt6::Widgets carvetto_warnings)
# Console subsystem for now so `carvetto --self-test` can be waited on and print its result in
# CI (a WIN32 GUI executable returns immediately from PowerShell). See ADR-0014.
set_target_properties(carvetto PROPERTIES WIN32_EXECUTABLE ON)
