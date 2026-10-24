# Third-party: nlohmann/json (MIT), header-only, used for tool-database import/export.
# Pinned to the v3.11.3 release tarball and included as a SYSTEM header so our
# warnings-as-errors flags do not apply to it. It is a PRIVATE dependency: no public
# carvetto header includes it.
include(FetchContent)
set(JSON_BuildTests OFF CACHE INTERNAL "")
set(JSON_Install OFF CACHE INTERNAL "")
set(JSON_SystemInclude ON CACHE INTERNAL "")
FetchContent_Declare(
  nlohmann_json
  URL      https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
)
FetchContent_MakeAvailable(nlohmann_json)

# Clipper2 (Boost Software License 1.0) — polygon offsetting and booleans. Pinned release tag;
# only the C++ library is built (no utils, examples, tests or the Z variant). Its headers are
# consumed as SYSTEM includes so carvetto_warnings do not apply to third-party code.
include(FetchContent)
FetchContent_Declare(
  Clipper2
  GIT_REPOSITORY https://github.com/AngusJohnson/Clipper2.git
  GIT_TAG        Clipper2_1.4.0
  GIT_SHALLOW    TRUE
  SOURCE_SUBDIR  CPP
)
set(CLIPPER2_UTILS    OFF CACHE BOOL   "" FORCE)
set(CLIPPER2_EXAMPLES OFF CACHE BOOL   "" FORCE)
set(CLIPPER2_TESTS    OFF CACHE BOOL   "" FORCE)
set(CLIPPER2_USINGZ   OFF CACHE STRING "" FORCE)
FetchContent_MakeAvailable(Clipper2)
set_target_properties(Clipper2 PROPERTIES
  INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${clipper2_SOURCE_DIR}/CPP/Clipper2Lib/include")
# Clipper2 builds itself with -Werror (/WX on MSVC). Strip that so warnings a future compiler
# finds in third-party code cannot break our build; Carvetto targets keep warnings-as-errors.
get_target_property(_clipper2_opts Clipper2 COMPILE_OPTIONS)
if(_clipper2_opts)
  list(REMOVE_ITEM _clipper2_opts -Werror /WX)
  set_property(TARGET Clipper2 PROPERTY COMPILE_OPTIONS ${_clipper2_opts})
endif()
if(MSVC)
  target_compile_options(Clipper2 PRIVATE /WX-)
else()
  target_compile_options(Clipper2 PRIVATE -Wno-error)
endif()

add_library(carvetto_core STATIC
  src/geometry/vec2.cpp
  src/geometry/box2.cpp
  src/geometry/transform2.cpp
  src/geometry/span.cpp
  src/geometry/polyline.cpp
  src/geometry/contour.cpp
  src/toolpath/move.cpp
  src/toolpath/toolpath.cpp
  src/toolpath/pass_plan.cpp
  src/toolpath/linking.cpp
  src/toolpath/time.cpp
  src/tooling/tool.cpp
  src/tooling/tool_database.cpp
  src/tooling/json_io.cpp
  src/post/variables.cpp
  src/post/parser.cpp
  src/post/interpreter.cpp
  src/document/job.cpp
  src/document/sheet.cpp
  src/document/object.cpp
  src/document/selection.cpp
  src/document/variables.cpp
  src/document/undo_stack.cpp
  src/document/document.cpp
  src/document/commands.cpp
  src/geometry/clipper_interop.cpp
  src/geometry/offset.cpp
  src/geometry/boolean.cpp
)
target_include_directories(carvetto_core PUBLIC include)
target_link_libraries(carvetto_core
  PRIVATE carvetto_warnings
  PRIVATE nlohmann_json::nlohmann_json
  PRIVATE Clipper2
)
add_library(carvetto::core ALIAS carvetto_core)
