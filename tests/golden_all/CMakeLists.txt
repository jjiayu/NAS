cmake_minimum_required(VERSION 3.15)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(nas_golden_all VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  include_directories("$ENV{CONDA_PREFIX}/include")
endif()

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)

# nas_config pulls in astar_search (which adds tests/fixtures as a side
# effect), footstep_qp and core/* — everything this test needs.
if(NOT TARGET nas_config)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../config ${CMAKE_CURRENT_BINARY_DIR}/_config)
endif()

enable_testing()
add_executable(nas_golden_all_scenes test_golden_all_scenes.cpp)
target_link_libraries(nas_golden_all_scenes PRIVATE nas_config nas_test_fixtures CGAL::CGAL)
target_compile_definitions(nas_golden_all_scenes PRIVATE
    TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../talosReachability/data/reachability_constraints"
    GOLDEN_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../golden"
)
add_test(NAME nas_golden_all_scenes COMMAND nas_golden_all_scenes)

# Differential test of core/expansion against the OLD get_children on real
# states: needs the JSON dumps from the old repo's old_expansion_dump tool,
# so it takes the dump directory as an argument and isn't registered with
# ctest (see tests/golden_all/README.md).
add_executable(nas_expansion_differential test_expansion_differential.cpp)
target_link_libraries(nas_expansion_differential PRIVATE nas_config CGAL::CGAL)
target_compile_definitions(nas_expansion_differential PRIVATE
    TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../talosReachability/data/reachability_constraints"
)

# Step-1 comparison (edge/plane vs CGAL slicer vs exact) on the dumped real
# parent states; takes the dump dir, not registered with ctest.
add_executable(nas_bench_plane_cut bench_plane_cut.cpp)
target_link_libraries(nas_bench_plane_cut PRIVATE nas_config CGAL::CGAL)
target_compile_definitions(nas_bench_plane_cut PRIVATE
    TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../talosReachability/data/reachability_constraints"
)

# Diagnostic: where do two heap states first make the search diverge?
add_executable(nas_trace_divergence trace_divergence.cpp)
target_link_libraries(nas_trace_divergence PRIVATE nas_config CGAL::CGAL)
target_compile_definitions(nas_trace_divergence PRIVATE
    TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../talosReachability/data/reachability_constraints"
)

# Search + QP timing on every scenario (see perf_all_scenes.cpp).
add_executable(nas_perf_all_scenes perf_all_scenes.cpp)
target_link_libraries(nas_perf_all_scenes PRIVATE nas_config nas_footstep_qp nas_footstep_qp_backend nas_planners_astar_search CGAL::CGAL)
target_compile_definitions(nas_perf_all_scenes PRIVATE
    TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../talosReachability/data/reachability_constraints"
)
