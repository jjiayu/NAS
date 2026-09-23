# Fragment included by the root CMakeLists.txt (NAS_BUILD_TESTS). One executable (nas_tests, see
# main.cpp) for every test this project has — replaces the ~20 standalone test binaries the
# pre-cleanup per-module CMakeLists.txt files each built. Registered with ctest as two entries
# (fast unit suites / slower golden regression suites) so a quick check-in doesn't have to pay for
# the slow ones (cycle_detection ~30s, merge_consistency ~45s, edge_costs ~10s).

add_executable(nas_tests
    main.cpp
    unit/geometry_test.cpp
    unit/node_test.cpp
    unit/surface_test.cpp
    unit/reachability_test.cpp
    unit/expansion_test.cpp
    unit/footstep_qp_backend_test.cpp
    unit/config_planner_test.cpp
    unit/config_scenario_test.cpp
    unit/config_stl_test.cpp
    golden/astar_search_golden_test.cpp
    golden/astar_search_golden_threepaths_test.cpp
    golden/grid_astar_search_test.cpp
    golden/footstep_qp_golden_test.cpp
    golden/golden_all_scenes_test.cpp
    golden/expansion_oracle_test.cpp
    golden/euclidean_metric_test.cpp
    golden/inclined_scenes_test.cpp
    golden/goal_surface_test.cpp
    golden/cycle_detection_test.cpp
    golden/merge_consistency_test.cpp
    golden/edge_costs_test.cpp
)
target_link_libraries(nas_tests PRIVATE nas nas_fixtures nlohmann_json::nlohmann_json)
target_compile_definitions(nas_tests PRIVATE
    TALOS_REACHABILITY_DATA_DIR="${CMAKE_SOURCE_DIR}/talosReachability/data/reachability_constraints"
    GOLDEN_DATA_DIR="${CMAKE_SOURCE_DIR}/tests/golden_data"
    TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/tests/unit/data"
)

add_test(NAME nas_tests_fast COMMAND nas_tests --suite=unit)
add_test(NAME nas_tests_golden COMMAND nas_tests --suite=golden)
