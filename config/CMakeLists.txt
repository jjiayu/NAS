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

if(NOT TARGET nas_core_surface)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core/surface ${CMAKE_CURRENT_BINARY_DIR}/_surface)
endif()

add_library(nas_config src/scenario_library.cpp)
target_include_directories(nas_config PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(nas_config PUBLIC
    nas_core_surface
    CGAL::CGAL
)

enable_testing()
add_executable(nas_config_tests tests/test_scenario_library.cpp)
target_link_libraries(nas_config_tests PRIVATE nas_config)
add_test(NAME nas_config_tests COMMAND nas_config_tests)
