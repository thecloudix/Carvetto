cmake_minimum_required(VERSION 3.24)
project(Carvetto VERSION 0.0.1 LANGUAGES C CXX)  # C: bundled miniz (core/CMakeLists.txt)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(CARVETTO_BUILD_APP   "Build the Qt desktop application" ON)
option(CARVETTO_BUILD_TESTS "Build unit tests" ON)
option(CARVETTO_BUILD_CLI   "Build the Qt-free command-line tool carvetto-cli" ON)
option(CARVETTO_BUILD_VIEWMODEL "Build the Qt-free app view-model library (always on when the app is built)" ON)

# Warnings as errors everywhere.
add_library(carvetto_warnings INTERFACE)
if(MSVC)
  target_compile_options(carvetto_warnings INTERFACE /W4 /WX /permissive- /utf-8)
else()
  target_compile_options(carvetto_warnings INTERFACE -Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wsign-conversion)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # GCC 12-14 emit false-positive -Wstringop-overflow inside std::variant at -O2/-O3
    # (GCC bugs 105746/106199). Disable only this diagnostic, only for GCC.
    target_compile_options(carvetto_warnings INTERFACE -Wno-stringop-overflow)
  endif()
endif()

add_subdirectory(core)

if(CARVETTO_BUILD_TESTS)
  enable_testing()
  add_subdirectory(core/tests)

  # T-0146: changelog fragments. `check` validates changelog.d/ (CI also runs it on every PR, with
  # --base for hand edits to CHANGELOG.md); the unit tests cover tools/changelog.py itself.
  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    add_test(NAME tools.changelog_check
      COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/changelog.py --root ${PROJECT_SOURCE_DIR} check)
    add_test(NAME tools.changelog_unit
      COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/tests/test_changelog.py)
  else()
    message(STATUS "Python 3 not found - skipping the changelog tool tests (CI still runs the check)")
  endif()
endif()

# Qt-free view logic for the app (ADR-0014): built and tested even where Qt is unavailable.
if(CARVETTO_BUILD_VIEWMODEL OR CARVETTO_BUILD_APP)
  add_subdirectory(app/viewmodel)
endif()

# Headless console front end (T-0122): Qt-free, built in the headless configuration too.
if(CARVETTO_BUILD_CLI)
  add_subdirectory(app/cli)
endif()

if(CARVETTO_BUILD_APP)
  find_package(Qt6 6.5 COMPONENTS Widgets QUIET)
  if(Qt6_FOUND)
    add_subdirectory(app)
  else()
    message(STATUS "Qt6 not found - skipping the desktop app (core and tests still build)")
  endif()
endif()
