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
)
target_include_directories(carvetto_core PUBLIC include)
target_link_libraries(carvetto_core
  PRIVATE carvetto_warnings
  PRIVATE nlohmann_json::nlohmann_json
)
add_library(carvetto::core ALIAS carvetto_core)
