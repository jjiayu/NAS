cmake_minimum_required(VERSION 3.15)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(nas_config VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  include_directories("$ENV{CONDA_PREFIX}/include")
endif()

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(Boost REQUIRED)

if(NOT TARGET nas_core_surface)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core/surface ${CMAKE_CURRENT_BINARY_DIR}/_surface)
endif()
if(NOT TARGET nas_planners_astar_search)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../planners/astar_search ${CMAKE_CURRENT_BINARY_DIR}/_astar_search)
endif()
if(NOT TARGET nas_footstep_qp)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../footstep_qp ${CMAKE_CURRENT_BINARY_DIR}/_footstep_qp)
endif()

add_library(nas_config src/scenario_library.cpp src/planner_config.cpp)
target_include_directories(nas_config PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(nas_config PUBLIC
    nas_core_surface
    nas_planners_astar_search
    nas_footstep_qp
    CGAL::CGAL
)

enable_testing()
add_executable(nas_config_tests tests/test_scenario_library.cpp)
target_link_libraries(nas_config_tests PRIVATE nas_config)
add_test(NAME nas_config_tests COMMAND nas_config_tests)

add_executable(nas_config_planner_tests tests/test_planner_config.cpp)
target_link_libraries(nas_config_planner_tests PRIVATE nas_config)
target_compile_definitions(nas_config_planner_tests PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
)
add_test(NAME nas_config_planner_tests COMMAND nas_config_planner_tests)
