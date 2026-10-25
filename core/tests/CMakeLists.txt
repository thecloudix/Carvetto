include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.7.1
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Catch2)

add_executable(carvetto_tests
  geometry/vec2_test.cpp
  geometry/box2_test.cpp
  geometry/transform2_test.cpp
  geometry/span_test.cpp
  geometry/polyline_test.cpp
  geometry/contour_test.cpp
  geometry/contour_ops_test.cpp
  geometry/intersect_test.cpp
  geometry/intersect_residuals_test.cpp
  geometry/fill_rule_test.cpp
  toolpath/move_test.cpp
  toolpath/toolpath_test.cpp
  toolpath/pass_plan_test.cpp
  toolpath/linking_test.cpp
  toolpath/time_test.cpp
  tooling/tool_test.cpp
  tooling/tool_database_test.cpp
  tooling/json_io_test.cpp
  post/variables_test.cpp
  post/parser_test.cpp
  post/interpreter_test.cpp
  post/grbl_golden_test.cpp
  document/job_test.cpp
  document/variables_test.cpp
  document/selection_test.cpp
  document/undo_stack_test.cpp
  document/document_test.cpp
  document/commands_test.cpp
  document/integrity_test.cpp
  document/id_overflow_test.cpp
  document/preset_id_test.cpp
  document/integrity_scale_test.cpp
  geometry/offset_test.cpp
  geometry/open_offset_chains_test.cpp
  geometry/boolean_test.cpp
  document/carv_io_test.cpp
)
# carv_io_test uses the internal zip helper (core/src/document/carv_zip.hpp) to craft archives.
target_include_directories(carvetto_tests PRIVATE ${PROJECT_SOURCE_DIR}/core/src/document)
# Golden files and shipped posts are read from the source tree via absolute paths.
target_compile_definitions(carvetto_tests PRIVATE
  CARVETTO_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden"
  CARVETTO_POSTS_DIR="${PROJECT_SOURCE_DIR}/posts"
)
target_link_libraries(carvetto_tests PRIVATE carvetto::core Catch2::Catch2WithMain carvetto_warnings)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(carvetto_tests)
