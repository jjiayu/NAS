cmake_minimum_required(VERSION 3.15)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(nas_bindings VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  include_directories("$ENV{CONDA_PREFIX}/include")
endif()

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)

# nas_config and everything it pulls in (core/*, planners/astar_search,
# footstep_qp) are static libraries built for linking into executables
# elsewhere in the repo — none of them opt into position-independent code.
# A Python extension module is a shared object, which requires every object
# file it contains to be PIC; set this before adding those subdirectories
# so it propagates to all of them.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT TARGET nas_config)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../config ${CMAKE_CURRENT_BINARY_DIR}/_config)
endif()

find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

# nanobind ships as a pip package (its C++ headers + a small CMake package,
# not a Python module to import at runtime) — find it via the interpreter
# that's active when CMake runs, same pattern nanobind's own docs use.
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(nas_bindings src/module.cpp)
target_link_libraries(nas_bindings PRIVATE nas_config CGAL::CGAL)

enable_testing()
add_test(NAME nas_bindings_tests
    COMMAND "${Python_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_bindings.py" "$<TARGET_FILE_DIR:nas_bindings>")
add_test(NAME nas_bindings_memory
    COMMAND "${Python_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_memory.py" "$<TARGET_FILE_DIR:nas_bindings>")
