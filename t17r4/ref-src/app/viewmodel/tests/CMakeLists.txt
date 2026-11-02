# Catch2 is fetched by core/tests; reuse it here.
include(FetchContent)
FetchContent_GetProperties(Catch2)
if(NOT catch2_POPULATED)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.7.1
    GIT_SHALLOW    TRUE
  )
  FetchContent_MakeAvailable(Catch2)
endif()

add_executable(carvetto_viewmodel_tests
  view_transform_test.cpp
  ruler_test.cpp
  units_format_test.cpp
  canvas_scene_test.cpp
  pixel_probe_test.cpp
  new_job_form_test.cpp
)
target_link_libraries(carvetto_viewmodel_tests PRIVATE carvetto::viewmodel Catch2::Catch2WithMain carvetto_warnings)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(carvetto_viewmodel_tests)

# Acceptance criterion 5: core and the viewmodel stay Qt-free.
file(GENERATE
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/viewmodel_link_libraries.txt
  CONTENT "$<TARGET_PROPERTY:carvetto_viewmodel,LINK_LIBRARIES>;$<TARGET_PROPERTY:carvetto_viewmodel,INTERFACE_LINK_LIBRARIES>;$<TARGET_PROPERTY:carvetto_core,LINK_LIBRARIES>;$<TARGET_PROPERTY:carvetto_core,INTERFACE_LINK_LIBRARIES>\n"
)
add_test(NAME viewmodel.qt_free
  COMMAND ${CMAKE_COMMAND}
    -DSOURCE_ROOT=${PROJECT_SOURCE_DIR}
    -DLINK_FILE=${CMAKE_CURRENT_BINARY_DIR}/viewmodel_link_libraries.txt
    -P ${CMAKE_CURRENT_SOURCE_DIR}/qt_free_check.cmake
)
