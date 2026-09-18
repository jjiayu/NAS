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

add_library(nas_footstep_qp_backend src/quadprog_backend.cpp)
target_include_directories(nas_footstep_qp_backend PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(nas_footstep_qp_backend PUBLIC
    Eigen3::Eigen
    eiquadprog::eiquadprog
)

enable_testing()
add_executable(nas_footstep_qp_backend_tests tests/test_quadprog_backend.cpp)
target_link_libraries(nas_footstep_qp_backend_tests PRIVATE nas_footstep_qp_backend)
add_test(NAME nas_footstep_qp_backend_tests COMMAND nas_footstep_qp_backend_tests)
