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
  INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${clipper2_SOURCE_DIR}/CPP/Clipper2Lib/include>")
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
# Third-party: miniz 3.0.2 (MIT), zip reader/writer for the native .carv container (ADR-0003,
# ADR-0013). Pinned to the release zip by SHA256 and built as a small private C library; only
# core/src/document/carv_zip.cpp includes it. MINIZ_NO_TIME makes every member's DOS time/date 0
# (deterministic archives); the other switches keep its zlib-compatible macros and stdio code out.
FetchContent_Declare(
  miniz
  URL      https://github.com/richgel999/miniz/releases/download/3.0.2/miniz-3.0.2.zip
  URL_HASH SHA256=ada38db0b703a56d3dd6d57bf84a9c5d664921d870d8fea4db153979fb5332c5
)
FetchContent_MakeAvailable(miniz)
add_library(carvetto_miniz STATIC ${miniz_SOURCE_DIR}/miniz.c)
target_include_directories(carvetto_miniz SYSTEM PUBLIC ${miniz_SOURCE_DIR})
target_compile_definitions(carvetto_miniz PUBLIC
  MINIZ_NO_TIME MINIZ_NO_STDIO MINIZ_NO_ZLIB_COMPATIBLE_NAMES)
set_target_properties(carvetto_miniz PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Third-party inflate/CRC code: optimised even in Debug so loading large files stays usable there
# (MSVC Debug keeps its defaults: /O2 is incompatible with /RTC1).
if(NOT MSVC)
  target_compile_options(carvetto_miniz PRIVATE -O2)
endif()

add_library(carvetto_core STATIC
  src/geometry/vec2.cpp
  src/geometry/box2.cpp
  src/geometry/transform2.cpp
  src/geometry/span.cpp
  src/geometry/polyline.cpp
  src/geometry/contour.cpp
  src/geometry/contour_ops.cpp
  src/geometry/intersect.cpp
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
  src/document/carv_zip.cpp
  src/document/carv_json_reader.cpp
  src/document/carv_io.cpp
  src/io/dxf_pairs.cpp
  src/io/dxf_text.cpp
  src/io/dxf_curves.cpp
  src/io/dxf_colour.cpp
  src/io/dxf_import.cpp
  src/io/dxf_export.cpp
  src/toolpath/pocket.cpp
  src/toolpath/pocket_ramp.cpp
)
target_include_directories(carvetto_core PUBLIC include)
target_link_libraries(carvetto_core
  PRIVATE carvetto_warnings
  PRIVATE nlohmann_json::nlohmann_json
  PRIVATE Clipper2
  PRIVATE carvetto_miniz
)
# Written into manifest.json as the app version.
target_compile_definitions(carvetto_core PRIVATE CARVETTO_VERSION_STRING="${PROJECT_VERSION}")
add_library(carvetto::core ALIAS carvetto_core)
