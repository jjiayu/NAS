cmake_minimum_required(VERSION 3.15)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(nas_test_fixtures VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  include_directories("$ENV{CONDA_PREFIX}/include")
endif()

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)

if(NOT TARGET nas_planners_astar_search)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../planners/astar_search ${CMAKE_CURRENT_BINARY_DIR}/_astar_search)
endif()

add_library(nas_test_fixtures src/scenarios.cpp)
target_include_directories(nas_test_fixtures PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(nas_test_fixtures PUBLIC
    nas_planners_astar_search
    CGAL::CGAL
)
