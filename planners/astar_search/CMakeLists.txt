cmake_minimum_required(VERSION 3.15)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(nas_planners_astar_search VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  include_directories("$ENV{CONDA_PREFIX}/include")
endif()

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)

if(NOT TARGET nas_core_expansion)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core/expansion ${CMAKE_CURRENT_BINARY_DIR}/_expansion)
endif()

add_library(nas_planners_astar_search src/astar_search.cpp)
target_include_directories(nas_planners_astar_search PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(nas_planners_astar_search PUBLIC
    nas_core_expansion
    CGAL::CGAL
    Boost::boost
)

# fixtures added after this target exists (see fixtures/CMakeLists.txt —
# it needs nas_planners_astar_search already defined, to avoid a circular
# add_subdirectory graph).
if(NOT TARGET nas_test_fixtures)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../tests/fixtures ${CMAKE_CURRENT_BINARY_DIR}/_fixtures)
endif()

enable_testing()
add_executable(nas_planners_astar_search_golden_test tests/test_astar_search_golden.cpp)
target_link_libraries(nas_planners_astar_search_golden_test PRIVATE nas_planners_astar_search nas_test_fixtures)
target_compile_definitions(nas_planners_astar_search_golden_test PRIVATE
    TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../talosReachability/data/reachability_constraints"
    GOLDEN_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../tests/golden"
)
add_test(NAME nas_planners_astar_search_golden_test COMMAND nas_planners_astar_search_golden_test)

add_executable(nas_planners_astar_search_golden_test_threepaths tests/test_astar_search_golden_threepaths.cpp)
target_link_libraries(nas_planners_astar_search_golden_test_threepaths PRIVATE nas_planners_astar_search nas_test_fixtures)
target_compile_definitions(nas_planners_astar_search_golden_test_threepaths PRIVATE
    TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../talosReachability/data/reachability_constraints"
    GOLDEN_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../tests/golden"
)
add_test(NAME nas_planners_astar_search_golden_test_threepaths COMMAND nas_planners_astar_search_golden_test_threepaths)
