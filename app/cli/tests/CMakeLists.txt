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

add_executable(carvetto_cli_tests
  cli_test.cpp
  params_test.cpp
)
target_link_libraries(carvetto_cli_tests PRIVATE carvetto_cli_lib nlohmann_json::nlohmann_json
                      Catch2::Catch2WithMain carvetto_warnings)
target_compile_definitions(carvetto_cli_tests PRIVATE
  CARVETTO_CLI_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
  CARVETTO_CLI_WORK_DIR="${CMAKE_CURRENT_BINARY_DIR}/work"
  CARVETTO_GOLDEN_DIR="${PROJECT_SOURCE_DIR}/core/tests/golden"
  CARVETTO_POSTS_DIR="${PROJECT_SOURCE_DIR}/posts"
  CARVETTO_STARTER_TOOLS="${PROJECT_SOURCE_DIR}/data/tools/starter_tools.json"
  CARVETTO_DXF_FIXTURE_DIR="${PROJECT_SOURCE_DIR}/core/tests/fixtures/dxf"
)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(carvetto_cli_tests)

# AC1: the real executable, DXF -> pocket + profile with tabs -> Grbl G-code, checksum equal to
# the golden. CI also runs this pipeline as an explicit step on Linux and Windows.
add_test(NAME cli.end_to_end_grbl
  COMMAND ${CMAKE_COMMAND}
    -DCLI=$<TARGET_FILE:carvetto-cli>
    -DDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data
    -DPOST=${PROJECT_SOURCE_DIR}/posts/grbl_mm.pp
    -DGOLDEN=${PROJECT_SOURCE_DIR}/core/tests/golden/cli/part_grbl.nc
    -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/e2e
    -P ${CMAKE_CURRENT_SOURCE_DIR}/end_to_end.cmake
)
