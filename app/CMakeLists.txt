qt_standard_project_setup()

qt_add_executable(carvetto
  src/main.cpp
)
target_link_libraries(carvetto PRIVATE carvetto::core Qt6::Widgets Qt6::OpenGLWidgets carvetto_warnings)
set_target_properties(carvetto PROPERTIES WIN32_EXECUTABLE ON)
