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

# core/expansion against an exact-arithmetic oracle, on the states a real search expands.
add_executable(nas_expansion_oracle test_expansion_oracle.cpp)
target_link_libraries(nas_expansion_oracle PRIVATE nas_config CGAL::CGAL)
target_compile_definitions(nas_expansion_oracle PRIVATE
    TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../talosReachability/data/reachability_constraints"
)
add_test(NAME nas_expansion_oracle COMMAND nas_expansion_oracle)

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

# The alternative Euclidean heuristic (see test_euclidean_metric.cpp).
add_executable(nas_euclidean_metric test_euclidean_metric.cpp)
target_link_libraries(nas_euclidean_metric PRIVATE nas_config CGAL::CGAL)
target_compile_definitions(nas_euclidean_metric PRIVATE
    TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../talosReachability/data/reachability_constraints"
)
add_test(NAME nas_euclidean_metric COMMAND nas_euclidean_metric)

# Inclined surfaces: the polytope's frame follows the contact surface (test_inclined_scenes.cpp).
add_executable(nas_inclined_scenes test_inclined_scenes.cpp)
target_link_libraries(nas_inclined_scenes PRIVATE nas_config nas_footstep_qp nas_footstep_qp_backend nas_planners_astar_search CGAL::CGAL)
target_compile_definitions(nas_inclined_scenes PRIVATE TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../talosReachability/data/reachability_constraints")
add_test(NAME nas_inclined_scenes COMMAND nas_inclined_scenes)

# The goal as a surface (test_goal_surface.cpp).
add_executable(nas_goal_surface test_goal_surface.cpp)
target_link_libraries(nas_goal_surface PRIVATE nas_config nas_footstep_qp nas_footstep_qp_backend nas_planners_astar_search CGAL::CGAL)
target_compile_definitions(nas_goal_surface PRIVATE TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../talosReachability/data/reachability_constraints")
add_test(NAME nas_goal_surface COMMAND nas_goal_surface)

# Cycle exclusion checked from the outside, on the parent chain of every expanded node (test_cycle_detection.cpp).
add_executable(nas_cycle_detection test_cycle_detection.cpp)
target_link_libraries(nas_cycle_detection PRIVATE nas_config nas_planners_astar_search CGAL::CGAL)
target_compile_definitions(nas_cycle_detection PRIVATE TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../talosReachability/data/reachability_constraints")
add_test(NAME nas_cycle_detection COMMAND nas_cycle_detection)
