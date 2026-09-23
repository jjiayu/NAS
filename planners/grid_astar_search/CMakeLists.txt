cmake_minimum_required(VERSION 3.15)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(nas_planners_grid_astar_search VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  include_directories("$ENV{CONDA_PREFIX}/include")
endif()

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)

if(NOT TARGET nas_core_reachability)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core/reachability ${CMAKE_CURRENT_BINARY_DIR}/_reachability)
endif()
if(NOT TARGET nas_core_node)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core/node ${CMAKE_CURRENT_BINARY_DIR}/_node)
endif()
if(NOT TARGET nas_core_surface)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core/surface ${CMAKE_CURRENT_BINARY_DIR}/_surface)
endif()
if(NOT TARGET nas_core_geometry)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core/geometry ${CMAKE_CURRENT_BINARY_DIR}/_geometry)
endif()
if(NOT TARGET nas_config)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../config ${CMAKE_CURRENT_BINARY_DIR}/_config)
endif()

add_library(nas_planners_grid_astar_search src/grid_environment.cpp src/grid_astar_search.cpp)
target_include_directories(nas_planners_grid_astar_search PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(nas_planners_grid_astar_search PUBLIC
    nas_core_node
    nas_core_reachability
    nas_core_surface
    nas_core_geometry
    CGAL::CGAL
    Boost::boost
)

enable_testing()
# nas_config is a test-only convenience (config::load_scenario() to get
# NarrowPassage/ThreePathsNAS's Surfaces easily) — the library itself has
# no dependency on config/.
add_executable(nas_planners_grid_astar_search_tests tests/test_grid_astar_search.cpp)
target_link_libraries(nas_planners_grid_astar_search_tests PRIVATE nas_planners_grid_astar_search nas_config)
target_compile_definitions(nas_planners_grid_astar_search_tests PRIVATE
    TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../talosReachability/data/reachability_constraints"
)
add_test(NAME nas_planners_grid_astar_search_tests COMMAND nas_planners_grid_astar_search_tests)
