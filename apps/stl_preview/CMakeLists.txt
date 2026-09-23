cmake_minimum_required(VERSION 3.15)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(nas_apps_stl_preview VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  include_directories("$ENV{CONDA_PREFIX}/include")
endif()

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)

if(NOT TARGET nas_config)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../config ${CMAKE_CURRENT_BINARY_DIR}/_config)
endif()

add_executable(stl_preview src/main.cpp)
target_link_libraries(stl_preview PRIVATE nas_config CGAL::CGAL)
