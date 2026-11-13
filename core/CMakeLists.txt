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
# Backport of the upstream fix for over-shrunk offsets (Clipper2 issues #873/#916, fixed after
# 1.4.0): ClipperOffset inserts the original vertex at every concave join, not only at joins
# sharper than ~8 degrees. Without it, a polygonised curve inset past its inradius comes back
# point-reflected with its own orientation (a 17 mm circle inset by 19.2 mm as a 2.2 mm circle)
# and survives the final union (T-0018, ADR-0006 §6). Applied at configure time, idempotently.
set(_clipper2_offset_cpp "${clipper2_SOURCE_DIR}/CPP/Clipper2Lib/src/clipper.offset.cpp")
file(READ "${_clipper2_offset_cpp}" _clipper2_offset_src)
set(_clipper2_offset_old "if (cos_a < 0.99) path_out.push_back(path[j]); // (#405)")
set(_clipper2_offset_new "path_out.push_back(path[j]); // (#405, #873, #916; Carvetto T-0018 backport)")
string(FIND "${_clipper2_offset_src}" "${_clipper2_offset_new}" _clipper2_offset_patched)
if(_clipper2_offset_patched EQUAL -1)
  string(FIND "${_clipper2_offset_src}" "${_clipper2_offset_old}" _clipper2_offset_at)
  if(_clipper2_offset_at EQUAL -1)
    message(FATAL_ERROR "Clipper2: cannot apply the T-0018 over-shrink fix to ${_clipper2_offset_cpp} "
                        "(source differs from 1.4.0); see ADR-0006 §6")
  endif()
  string(REPLACE "${_clipper2_offset_old}" "${_clipper2_offset_new}" _clipper2_offset_src "${_clipper2_offset_src}")
  file(WRITE "${_clipper2_offset_cpp}" "${_clipper2_offset_src}")
endif()
# Cooperative cancel hook (T-0228): a long Clipper2 sweep (one scanbeam can hold millions of edge
# intersections) must be interruptible. The engine gets a thread-local function pointer
# `Clipper2Lib::carvettoPollHook`, null unless Carvetto installs it (core/src/geometry/cancel.cpp),
# polled every 1024th intersection and scanbeam; the installed function throws to unwind the call.
# Nothing else changes: with a null hook the results are those of 1.4.0 (plus the T-0018 fix).
# Applied at configure time, idempotently, like the patch above.
set(_clipper2_engine_cpp "${clipper2_SOURCE_DIR}/CPP/Clipper2Lib/src/clipper.engine.cpp")
file(READ "${_clipper2_engine_cpp}" _clipper2_engine_src)
# A Windows checkout may have CRLF line endings (autocrlf): the edits below match LF text.
string(REPLACE "\r\n" "\n" _clipper2_engine_src "${_clipper2_engine_src}")
string(FIND "${_clipper2_engine_src}" "carvettoPollHook" _clipper2_engine_patched)
if(_clipper2_engine_patched EQUAL -1)
  macro(_carvetto_clipper_edit old new)
    string(FIND "${_clipper2_engine_src}" "${old}" _at)
    if(_at EQUAL -1)
      message(FATAL_ERROR "Clipper2: cannot apply the T-0228 cancel hook to ${_clipper2_engine_cpp} "
                          "(source differs from 1.4.0)")
    endif()
    string(REPLACE "${old}" "${new}" _clipper2_engine_src "${_clipper2_engine_src}")
  endmacro()
  _carvetto_clipper_edit("namespace Clipper2Lib {\n\n  static const Rect64 invalid_rect = Rect64(false);"
    "namespace Clipper2Lib {\n\n  thread_local void (*carvettoPollHook)() = nullptr; // Carvetto T-0228\n  static inline void CarvettoPoll()\n  {\n    static thread_local unsigned counter = 0;\n    if (carvettoPollHook && (++counter & 1023u) == 0) carvettoPollHook();\n  }\n\n  static const Rect64 invalid_rect = Rect64(false);")
  _carvetto_clipper_edit("  void ClipperBase::DoIntersections(const int64_t top_y)\n  {\n"
    "  void ClipperBase::DoIntersections(const int64_t top_y)\n  {\n    CarvettoPoll();\n")
  _carvetto_clipper_edit("  void ClipperBase::AddNewIntersectNode(Active& e1, Active& e2, int64_t top_y)\n  {\n"
    "  void ClipperBase::AddNewIntersectNode(Active& e1, Active& e2, int64_t top_y)\n  {\n    CarvettoPoll();\n")
  _carvetto_clipper_edit("      node_iter != intersect_nodes_.end();  ++node_iter)\n    {\n"
    "      node_iter != intersect_nodes_.end();  ++node_iter)\n    {\n      CarvettoPoll();\n")
  file(WRITE "${_clipper2_engine_cpp}" "${_clipper2_engine_src}")
endif()
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

# FreeType 2.14.3 (FreeType License, used under that option: it is GPL-3.0 compatible; ADR-0033) -
# parses TrueType/OpenType fonts and decomposes glyph outlines. Pinned release tag (commit
# 0a0221a1347e2f1e07c395263540026e9a0aa7c7). Optional compression/PNG/HarfBuzz back ends are
# disabled and FT_CONFIG_OPTION_USE_ZLIB / ENVIRONMENT_PROPERTIES are undefined (freetype_options.h), so
# gzip/WOFF/WOFF2 fonts are unsupported (and refused by core/text before FreeType sees them).
include(FetchContent)
FetchContent_Declare(
  freetype
  GIT_REPOSITORY https://github.com/freetype/freetype.git
  GIT_TAG        VER-2-14-3
  GIT_SHALLOW    TRUE
)
set(FT_DISABLE_ZLIB     ON CACHE BOOL "" FORCE)
set(FT_DISABLE_BZIP2    ON CACHE BOOL "" FORCE)
set(FT_DISABLE_PNG      ON CACHE BOOL "" FORCE)
set(FT_DISABLE_HARFBUZZ ON CACHE BOOL "" FORCE)
set(FT_DISABLE_BROTLI   ON CACHE BOOL "" FORCE)
set(SKIP_INSTALL_ALL    ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(freetype)
set_target_properties(freetype PROPERTIES
  INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${freetype_SOURCE_DIR}/include>")
set_target_properties(freetype PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Our own option set (src/text/freetype_options.h): no FREETYPE_PROPERTIES environment variable and
# no zlib/gzip/WOFF code, for the library and for every translation unit that includes its headers.
target_compile_definitions(freetype PUBLIC
  "FT_CONFIG_OPTIONS_H=\"${CMAKE_CURRENT_SOURCE_DIR}/src/text/freetype_options.h\"")

# HarfBuzz 15.1.0 (Old MIT licence; ADR-0033) - text shaping. Pinned release tarball by SHA256 and
# built as a small private static library from its single-file amalgamation (src/harfbuzz.cc); no
# FreeType/GLib/ICU/CoreText/DirectWrite back ends and no subsetter. Glyph outlines come from FreeType;
# HarfBuzz only shapes (built-in Unicode data, OpenType layout), so output does not depend on the OS.
FetchContent_Declare(
  harfbuzz
  URL      https://github.com/harfbuzz/harfbuzz/releases/download/15.1.0/harfbuzz-15.1.0.tar.xz
  URL_HASH SHA256=0fc7c565443a5f4be7da65ac5ee5bd737d9b158dc908708b7689a0a5796b0c8d
  SOURCE_SUBDIR carvetto-no-cmake  # sources only: upstream's CMakeLists.txt is not used
)
FetchContent_MakeAvailable(harfbuzz)
add_library(carvetto_harfbuzz STATIC ${harfbuzz_SOURCE_DIR}/src/harfbuzz.cc)
target_include_directories(carvetto_harfbuzz SYSTEM PUBLIC ${harfbuzz_SOURCE_DIR}/src)
set_target_properties(carvetto_harfbuzz PROPERTIES POSITION_INDEPENDENT_CODE ON)
# HarfBuzz must not read HB_SHAPER_LIST / HB_FACE_LOADER / HB_FONT_FUNCS from the environment.
target_compile_definitions(carvetto_harfbuzz PRIVATE HB_NO_GETENV)
if(NOT MSVC)
  # Optimised even in Debug (shaping a long string is otherwise slow in unoptimised template code).
  target_compile_options(carvetto_harfbuzz PRIVATE -O2)
else()
  target_compile_options(carvetto_harfbuzz PRIVATE /bigobj)
endif()

add_library(carvetto_core STATIC
  src/geometry/vec2.cpp
  src/geometry/box2.cpp
  src/geometry/transform2.cpp
  src/geometry/span.cpp
  src/geometry/polyline.cpp
  src/geometry/contour.cpp
  src/geometry/contour_ops.cpp
  src/geometry/shapes.cpp
  src/geometry/vector_texture.cpp
  src/geometry/vector_texture_region.cpp
  src/document/shape_params.cpp
  src/geometry/intersect.cpp
  src/geometry/robust_predicates.cpp
  src/geometry/delaunay.cpp
  src/geometry/edit_ops.cpp
  src/geometry/validator.cpp
  src/geometry/medial_axis.cpp
  src/toolpath/move.cpp
  src/toolpath/toolpath.cpp
  src/toolpath/pass_plan.cpp
  src/toolpath/linking.cpp
  src/toolpath/keepout.cpp
  src/toolpath/tiling.cpp
  src/toolpath/time.cpp
  src/toolpath/path_walk.cpp src/toolpath/clearance.cpp src/toolpath/box_index.cpp src/toolpath/operation.cpp src/toolpath/tab_placement.cpp src/toolpath/lead_moves.cpp src/toolpath/cut_order.cpp src/toolpath/ramp_entry.cpp src/toolpath/profile.cpp src/toolpath/enclosed_pieces.cpp
  src/tooling/tool.cpp
  src/tooling/tool_database.cpp
  src/tooling/json_io.cpp
  src/post/variables.cpp
  src/post/parser.cpp
  src/post/interpreter.cpp
  src/simulate/stock.cpp
  src/simulate/sweep.cpp
  src/simulate/simulator.cpp
  src/simulate/tool_shape.cpp
  src/document/job.cpp
  src/document/double_sided.cpp
  src/document/node_edit_command.cpp
  src/document/object_property_commands.cpp
  src/document/text_object.cpp
  src/document/text_commands.cpp
  src/document/bitmap_store.cpp
  src/document/sheet.cpp
  src/document/object.cpp
  src/document/dimension.cpp
  src/document/selection.cpp
  src/document/variables.cpp
  src/document/undo_stack.cpp
  src/document/document.cpp
  src/document/commands.cpp
  src/document/keepout_commands.cpp
  src/document/toolpath_record.cpp
  src/geometry/cancel.cpp
  src/geometry/clipper_interop.cpp
  src/geometry/offset.cpp
  src/geometry/hole_nesting.cpp
  src/geometry/boolean.cpp
  src/document/carv_zip.cpp
  src/document/carv_json_reader.cpp
  src/document/carv_moves.cpp
  src/document/carv_io.cpp
  src/io/dxf_pairs.cpp
  src/io/dxf_text.cpp
  src/io/dxf_curves.cpp
  src/io/dxf_colour.cpp
  src/io/dxf_import.cpp
  src/io/dxf_export.cpp
  src/io/raster.cpp
  src/io/bitmap_trace.cpp
  src/io/svg_export.cpp
  src/io/svg_import.cpp
  src/io/svg_path.cpp
  src/io/svg_xml.cpp
  src/toolpath/pocket.cpp
  src/toolpath/drill.cpp
  src/toolpath/engrave.cpp
  src/toolpath/chamfer.cpp
  src/toolpath/prism.cpp
  src/toolpath/thread_mill.cpp
  src/toolpath/inlay.cpp
  src/toolpath/vcarve_inlay.cpp
  src/toolpath/fluting.cpp
  src/toolpath/photo_vcarve.cpp
  src/toolpath/v_carve.cpp
  src/toolpath/v_carve_corners.cpp
  src/text/font.cpp
  src/text/layout_builder.cpp
  src/text/layout.cpp
  src/text/path_layout.cpp
)
target_include_directories(carvetto_core PUBLIC include)
target_link_libraries(carvetto_core
  PRIVATE carvetto_warnings
  PRIVATE nlohmann_json::nlohmann_json
  PRIVATE Clipper2
  PRIVATE carvetto_miniz
  PRIVATE freetype
  PRIVATE carvetto_harfbuzz
)
# Identical floating-point arithmetic on every compiler (T-0153, ADR-0025 §11): GCC and Clang may
# fuse a * b + c into one fma (GCC 15 at -march=x86-64-v3 does, Clang does by default), which rounds
# differently from MSVC (/fp:precise never contracts) and from builds without FMA. Outputs that
# depend on the last bit (the point-sample medial axis behind V-Carve: sample positions, Delaunay
# predicates and refinement thresholds) then differ byte for byte. Contraction is off for the core
# and Clipper2 (its offsets use doubles); explicit std::fma (exact predicates) is unaffected.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(carvetto_core PRIVATE -ffp-contract=off)
  target_compile_options(Clipper2 PRIVATE -ffp-contract=off)
endif()
# MSVC: /fp:precise (its default, stated explicitly) never contracts a * b + c into an fma on the
# SSE2 baseline and rounds every double operation as written; ADR-0032 section 7 (byte-identical
# simulation heightfields) and ADR-0025 section 11 rely on it. /fp:fast would break both.
if(MSVC)
  target_compile_options(carvetto_core PRIVATE /fp:precise)
endif()
# Written into manifest.json as the app version.
target_compile_definitions(carvetto_core PRIVATE CARVETTO_VERSION_STRING="${PROJECT_VERSION}")
# Platform-stable vector texture (T-0251): no fused multiply-add contraction in its arithmetic.
if(NOT MSVC)
  set_source_files_properties(src/geometry/vector_texture.cpp PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")
endif()
add_library(carvetto::core ALIAS carvetto_core)
