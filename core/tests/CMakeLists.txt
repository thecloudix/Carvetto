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
  toolpath/move_test.cpp
  toolpath/toolpath_test.cpp
  toolpath/pass_plan_test.cpp
  toolpath/linking_test.cpp
  toolpath/time_test.cpp
)
target_link_libraries(carvetto_tests PRIVATE carvetto::core Catch2::Catch2WithMain carvetto_warnings)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(carvetto_tests)
