cmake_minimum_required(VERSION 3.15)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(nas_footstep_qp VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  include_directories("$ENV{CONDA_PREFIX}/include")
endif()

find_package(Eigen3 REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)

if(NOT TARGET nas_core_expansion)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core/expansion ${CMAKE_CURRENT_BINARY_DIR}/_expansion)
endif()
if(NOT TARGET nas_planners_astar_search)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../planners/astar_search ${CMAKE_CURRENT_BINARY_DIR}/_astar_search)
endif()
# Must come after astar_search (see tests/fixtures/CMakeLists.txt).
if(NOT TARGET nas_test_fixtures)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../tests/fixtures ${CMAKE_CURRENT_BINARY_DIR}/_fixtures)
endif()

add_library(nas_footstep_qp_backend src/quadprog_backend.cpp)
target_include_directories(nas_footstep_qp_backend PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(nas_footstep_qp_backend PUBLIC
    Eigen3::Eigen
    eiquadprog::eiquadprog
)

add_library(nas_footstep_qp src/footstep_qp.cpp)
target_include_directories(nas_footstep_qp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(nas_footstep_qp PUBLIC
    nas_footstep_qp_backend
    nas_core_expansion
    CGAL::CGAL
)

enable_testing()
add_executable(nas_footstep_qp_backend_tests tests/test_quadprog_backend.cpp)
target_link_libraries(nas_footstep_qp_backend_tests PRIVATE nas_footstep_qp_backend)
add_test(NAME nas_footstep_qp_backend_tests COMMAND nas_footstep_qp_backend_tests)

add_executable(nas_footstep_qp_golden_test tests/test_footstep_qp_golden.cpp)
target_link_libraries(nas_footstep_qp_golden_test PRIVATE nas_footstep_qp nas_planners_astar_search nas_test_fixtures)
target_compile_definitions(nas_footstep_qp_golden_test PRIVATE
    TALOS_REACHABILITY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../talosReachability/data/reachability_constraints"
    GOLDEN_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../tests/golden"
)
add_test(NAME nas_footstep_qp_golden_test COMMAND nas_footstep_qp_golden_test)
