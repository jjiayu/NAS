cmake_minimum_required(VERSION 3.15)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(nas_core_node VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  include_directories("$ENV{CONDA_PREFIX}/include")
endif()

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)

if(NOT TARGET nas_core_geometry)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../geometry ${CMAKE_CURRENT_BINARY_DIR}/_geometry)
endif()

add_library(nas_core_node src/node.cpp)
target_include_directories(nas_core_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(nas_core_node PUBLIC
    nas_core_geometry
    CGAL::CGAL
)

enable_testing()
add_executable(nas_core_node_tests tests/test_node.cpp)
target_link_libraries(nas_core_node_tests PRIVATE nas_core_node)
add_test(NAME nas_core_node_tests COMMAND nas_core_node_tests)
