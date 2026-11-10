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
  vector_ops_test.cpp
  keepout_test.cpp
  vector_validator_test.cpp
  weld_subtract_test.cpp
  view_transform_test.cpp
  ruler_test.cpp
  save_toolpaths_test.cpp
  tools_test.cpp
  text_tool_test.cpp
  dimension_tool_test.cpp
  text_dxf_test.cpp
  starter_font_test.cpp
  transform_form_test.cpp
  transform_gesture_test.cpp
  transform_ops_test.cpp
  units_format_test.cpp
  canvas_scene_test.cpp
  array_copies_test.cpp
  pixel_probe_test.cpp
  new_job_form_test.cpp
  file_session_test.cpp
  node_edit_fuzz_test.cpp
  node_edit_test.cpp
  node_ops_test.cpp
  snapping_test.cpp
  selection_tools_test.cpp
  toolpath_form_test.cpp
  toolpath_chamfer_test.cpp
  toolpath_thread_mill_test.cpp
  toolpath_prism_test.cpp
  toolpath_preview_test.cpp
  simulation_test.cpp
  toolpath_robustness_test.cpp
  toolpath_runner_test.cpp
  toolpath_session_test.cpp
  job_sheet_test.cpp
  tool_setup_test.cpp
  vector_selector_test.cpp
  properties_model_test.cpp
  layers_sheets_test.cpp
  calculation_test.cpp
  variables_model_test.cpp
  texture_form_test.cpp
)
# The toolpath tests drive carvetto-cli in process (cli::run) to compare its records with the session's.
target_link_libraries(carvetto_viewmodel_tests PRIVATE carvetto::viewmodel Catch2::Catch2WithMain carvetto_warnings)
if(CARVETTO_BUILD_CLI)
  target_sources(carvetto_viewmodel_tests PRIVATE save_toolpaths_cli_test.cpp toolpath_cli_identity_test.cpp)
  target_link_libraries(carvetto_viewmodel_tests PRIVATE carvetto_cli_lib)
endif()
target_compile_definitions(carvetto_viewmodel_tests PRIVATE
  CARVETTO_CLI_DATA_DIR="${PROJECT_SOURCE_DIR}/app/cli/tests/data"
  CARVETTO_VM_WORK_DIR="${CMAKE_CURRENT_BINARY_DIR}/work"
  CARVETTO_TEXT_FIXTURE_DIR="${PROJECT_SOURCE_DIR}/core/tests/fixtures/text"
  CARVETTO_DXF_FIXTURE_DIR="${PROJECT_SOURCE_DIR}/core/tests/fixtures/dxf"
  CARVETTO_POSTS_DIR="${PROJECT_SOURCE_DIR}/posts"
  CARVETTO_STARTER_TOOLS="${PROJECT_SOURCE_DIR}/data/tools/starter_tools.json"
)

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
