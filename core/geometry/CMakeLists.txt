cmake_minimum_required(VERSION 3.15)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(nas_core_geometry VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  include_directories("$ENV{CONDA_PREFIX}/include")
endif()

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(coal REQUIRED)

add_library(nas_core_geometry src/geometry.cpp)
target_include_directories(nas_core_geometry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(nas_core_geometry PUBLIC
    CGAL::CGAL
    coal::coal
    Eigen3::Eigen
)

# Standalone smoke/unit tests for this module (not linked into any
# executable outside core/geometry). See PLAN.md phase 2.
enable_testing()
add_executable(nas_core_geometry_tests tests/test_geometry.cpp)
target_link_libraries(nas_core_geometry_tests PRIVATE nas_core_geometry)
add_test(NAME nas_core_geometry_tests COMMAND nas_core_geometry_tests)
