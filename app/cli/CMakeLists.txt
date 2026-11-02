# carvetto-cli: headless console front end (T-0122). Qt-free, so it builds in the headless
# configuration (CARVETTO_BUILD_APP=OFF) and in CI on Linux and Windows.

# The starter tool database is compiled in as the default tool database of params.json files
# (ADR-0019), so the CLI works from any directory. Regenerated whenever the JSON file changes.
set(CARVETTO_STARTER_TOOLS_JSON ${PROJECT_SOURCE_DIR}/data/tools/starter_tools.json)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CARVETTO_STARTER_TOOLS_JSON})
file(READ ${CARVETTO_STARTER_TOOLS_JSON} _starter_hex HEX)
string(LENGTH "${_starter_hex}" _starter_hex_len)
math(EXPR _starter_size "${_starter_hex_len} / 2")
# A byte array, not a string literal: MSVC limits string literals to 64 KiB.
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," _starter_bytes "${_starter_hex}")
string(REGEX REPLACE "(0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,)" "\\1\n" _starter_bytes "${_starter_bytes}")
configure_file(src/starter_tools_embedded.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/starter_tools_embedded.cpp @ONLY)

add_library(carvetto_cli_lib STATIC
  src/cli.cpp
  src/cli.hpp
  src/params.cpp
  src/params.hpp
  src/starter_tools.hpp
  ${CMAKE_CURRENT_BINARY_DIR}/starter_tools_embedded.cpp
)
target_include_directories(carvetto_cli_lib PUBLIC src)
target_link_libraries(carvetto_cli_lib
  PUBLIC carvetto::core
  PRIVATE nlohmann_json::nlohmann_json
  PRIVATE carvetto_warnings
)
target_compile_definitions(carvetto_cli_lib PRIVATE CARVETTO_VERSION_STRING="${PROJECT_VERSION}")

add_executable(carvetto-cli src/main.cpp)
target_link_libraries(carvetto-cli PRIVATE carvetto_cli_lib carvetto_warnings)

if(CARVETTO_BUILD_TESTS)
  add_subdirectory(tests)
endif()
