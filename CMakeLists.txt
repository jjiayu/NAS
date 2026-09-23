cmake_minimum_required(VERSION 3.15)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(nas VERSION 1.0 LANGUAGES CXX
    DESCRIPTION "NAS/CASSR footstep planning library")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Position-independent code globally, not just before bindings/ as the pre-cleanup per-module
# CMakeLists.txt files did — there is now a single library target, so "set it before the target
# that needs it" and "set it globally" are the same thing; global is simpler.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  include_directories("$ENV{CONDA_PREFIX}/include")
endif()

option(NAS_BUILD_APPS "Build apps/ (the astar_plan CLI)" ON)
option(NAS_BUILD_TOOLS "Build tools/ (benchmark/diagnostic executables, not pass/fail tests)" ON)
option(NAS_BUILD_TESTS "Build tests/ (the nas_tests executable, registered with ctest)" ON)
option(NAS_BUILD_BINDINGS "Build bindings/ (the nas_bindings Python extension, needs Python+nanobind)" OFF)

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)
find_package(coal REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(nas
    src/core/geometry.cpp
    src/core/node.cpp
    src/core/surface.cpp
    src/core/reachability.cpp
    src/core/expansion.cpp
    src/planners/astar_search.cpp
    src/planners/grid_astar_search.cpp
    src/planners/grid_environment.cpp
    src/footstep_qp/footstep_qp.cpp
    src/footstep_qp/quadprog_backend.cpp
    src/config/planner_config.cpp
    src/config/scenario_library.cpp
    src/config/stl_import.cpp
)
add_library(nas::nas ALIAS nas)

target_include_directories(nas PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(nas PUBLIC
    Eigen3::Eigen
    CGAL::CGAL
    Boost::boost
    coal::coal
    eiquadprog::eiquadprog
    nlohmann_json::nlohmann_json
)

# --- Install (find_package(nas) support for external consumers) ---
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS nas EXPORT nasTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/nas DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT nasTargets FILE nasTargets.cmake NAMESPACE nas::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nas)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/nasConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/nasConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nas
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/nasConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/nasConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/nasConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nas
)

# Shared test fixtures (nas/fixtures/...) — not gated behind NAS_BUILD_TESTS: a couple of tools/
# (nas_bench_compare, nas_dump_plan) also depend on it.
add_subdirectory(tests/common)

if(NAS_BUILD_APPS)
  add_subdirectory(apps/astar_plan)
endif()

if(NAS_BUILD_TOOLS)
  add_subdirectory(tools)
endif()

if(NAS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(NAS_BUILD_BINDINGS)
  add_subdirectory(bindings)
endif()
