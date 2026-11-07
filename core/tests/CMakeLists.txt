include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.7.1
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Catch2)

add_executable(carvetto_tests
  geometry/bezier_work_bounds_test.cpp
  geometry/gap_trail_test.cpp
  geometry/hit_index_test.cpp
  geometry/vec2_test.cpp
  geometry/box2_test.cpp
  geometry/transform2_test.cpp
  geometry/span_test.cpp
  geometry/polyline_test.cpp
  geometry/contour_test.cpp
  geometry/shapes_test.cpp
  geometry/region_polygon_test.cpp
  geometry/contour_ops_test.cpp
  geometry/signed_area_far_test.cpp
  geometry/winding_degeneracy_test.cpp
  geometry/intersect_test.cpp
  geometry/intersect_residuals_test.cpp
  geometry/intersect_residuals_ii_test.cpp
  geometry/intersect_ends_test.cpp
  geometry/intersect_cancellation_test.cpp
  geometry/intersect_slack_test.cpp
  geometry/fill_rule_test.cpp
  geometry/medial_axis_test.cpp
  geometry/medial_axis_robustness_test.cpp
  geometry/delaunay_test.cpp
  geometry/edit_ops_test.cpp
  geometry/edit_ops_fuzz_test.cpp
  geometry/edit_ops_hardening_test.cpp
  toolpath/move_test.cpp
  toolpath/toolpath_test.cpp
  toolpath/pass_plan_test.cpp
  toolpath/linking_test.cpp
  toolpath/time_test.cpp
  toolpath/block_settle_test.cpp
  toolpath/profile_tab_layer_test.cpp
  toolpath/path_walk_test.cpp toolpath/profile_test.cpp toolpath/profile_allowance_limit_test.cpp toolpath/profile_on_allowance_limit_test.cpp toolpath/profile_tolerance_test.cpp toolpath/profile_golden_test.cpp toolpath/profile_property_test.cpp toolpath/profile_empty_walk_test.cpp toolpath/profile_tip_start_test.cpp
  tooling/tool_test.cpp
  tooling/tool_database_test.cpp
  tooling/json_io_test.cpp
  post/variables_test.cpp
  post/parser_test.cpp
  post/interpreter_test.cpp
  post/grbl_golden_test.cpp
  post/shipped_posts_test.cpp
  post/post_safety_test.cpp
  post/spindle_off_test.cpp
  post/line_length_test.cpp
  tooling/starter_tools_test.cpp
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
  geometry/offset_quality_test.cpp
  geometry/offset_narrow_loops_test.cpp
  geometry/box_tree_test.cpp
  geometry/offset_hatch_cost_test.cpp
  geometry/offset_shortcuts_test.cpp
  geometry/offset_near_tol_test.cpp
  geometry/offset_inradius_test.cpp
  geometry/offset_dust_test.cpp
  geometry/offset_batch_test.cpp
  geometry/boolean_test.cpp
  geometry/boolean_batch_test.cpp
  geometry/offset_unnested_hole_test.cpp
  geometry/hole_nesting_test.cpp
  document/carv_io_test.cpp
  document/carv_extras_member_test.cpp
  document/carv_toolpath_test.cpp
  document/carv_storage_residuals_test.cpp
  document/carv_drill_record_test.cpp
  document/carv_engrave_record_test.cpp
  document/carv_v_carve_record_test.cpp
  io/dxf_import_test.cpp
  io/dxf_export_test.cpp
  io/dxf_curves_test.cpp
  io/dxf_fuzz_test.cpp
  io/svg_export_test.cpp
  io/svg_fuzz_test.cpp
  io/svg_import_test.cpp
  io/svg_xml_test.cpp
  toolpath/pocket_coverage_test.cpp
  toolpath/pocket_test.cpp
  toolpath/pocket_gouge_test.cpp
  toolpath/pocket_golden_test.cpp
  toolpath/pocket_centre_test.cpp
  toolpath/pocket_guard_test.cpp
  toolpath/pocket_junction_test.cpp
  toolpath/pocket_thin_island_test.cpp
  toolpath/pocket_move_check_test.cpp
  toolpath/toolpath_api_test.cpp
  toolpath/toolpath_scale_test.cpp
  toolpath/enclosed_pieces_test.cpp
  toolpath/toolpath_residual_scale_test.cpp
  toolpath/drill_test.cpp
  toolpath/drill_post_test.cpp
  toolpath/engrave_test.cpp
  toolpath/engrave_post_test.cpp
  toolpath/v_carve_test.cpp
  toolpath/v_carve_post_test.cpp
  toolpath/v_carve_corner_test.cpp
)
# carv_io_test uses the internal zip helper (core/src/document/carv_zip.hpp) to craft archives.
target_include_directories(carvetto_tests PRIVATE ${PROJECT_SOURCE_DIR}/core/src/document)
# DXF tests use internal io helpers (pair reader value types, curve conversions).
target_include_directories(carvetto_tests PRIVATE ${PROJECT_SOURCE_DIR}/core/src/io)
# Offset tests check the internal start-angle junction turn (core/src/geometry/offset_junction.hpp).
target_include_directories(carvetto_tests PRIVATE ${PROJECT_SOURCE_DIR}/core/src/geometry)
# Pocket guard tests feed hand-made insets to the internal strict guard (core/src/toolpath/pocket_guard.hpp).
target_include_directories(carvetto_tests PRIVATE ${PROJECT_SOURCE_DIR}/core/src/toolpath)
# Golden files and shipped posts are read from the source tree via absolute paths.
target_compile_definitions(carvetto_tests PRIVATE
  CARVETTO_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden"
  CARVETTO_POSTS_DIR="${PROJECT_SOURCE_DIR}/posts"
  CARVETTO_STARTER_TOOLS="${PROJECT_SOURCE_DIR}/data/tools/starter_tools.json"
  CARVETTO_DXF_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/dxf"
  CARVETTO_SVG_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/svg"
  CARVETTO_POCKET_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/pocket"
)
target_link_libraries(carvetto_tests PRIVATE carvetto::core Catch2::Catch2WithMain carvetto_warnings)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(carvetto_tests)
