cmake_minimum_required(VERSION 3.15)

# Set build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(
  nas
  VERSION 1.0
  LANGUAGES C CXX
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add conda prefix to search paths if available
if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()

# Use PkgConfig to find FreeType (Homebrew doesn't provide CMake config)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FREETYPE REQUIRED freetype2)

# Set custom path for CasADi


# Find required packages
find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)
find_package(VTK 9.1 REQUIRED COMPONENTS 
  CommonCore
  CommonDataModel
  FiltersSources
  InteractionStyle
  RenderingCore
  RenderingOpenGL2
  RenderingFreeType
)
find_package(yaml-cpp REQUIRED)
find_package(CasADi REQUIRED)
find_package(coal REQUIRED)

# Add the include directory
include_directories(include)
include_directories(${VTK_INCLUDE_DIRS})
if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  include_directories("$ENV{CONDA_PREFIX}/include")
endif()
include_directories(${EIGEN3_INCLUDE_DIR})  # For Eigen3 headers

# Add library with source files
add_library(utils_lib src/utils.cpp)
target_include_directories(utils_lib PRIVATE
    include
    ${VTK_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
add_library(node_lib src/node.cpp)
add_library(tree_lib src/tree.cpp)
add_library(geometry_lib src/geometry.cpp)
add_library(surface_lib src/surface.cpp)
add_library(grid_environment_lib src/grid_environment.cpp)
add_library(visualizer_lib src/visualizer.cpp)
add_library(astar_search_lib src/astar_search.cpp)
add_library(astar_grid_search_lib src/astar_grid_search.cpp)
add_library(footstep_planner_lib src/footstep_planner.cpp)

# Set library dependencies
target_link_libraries(node_lib PRIVATE geometry_lib CGAL::CGAL)
target_link_libraries(tree_lib PRIVATE utils_lib node_lib surface_lib geometry_lib visualizer_lib ${VTK_LIBRARIES})
target_link_libraries(utils_lib PRIVATE 
    ${VTK_LIBRARIES}
    CGAL::CGAL
)
target_link_libraries(grid_environment_lib PRIVATE
    surface_lib
    geometry_lib
    CGAL::CGAL
)
target_link_libraries(visualizer_lib PRIVATE
    ${VTK_LIBRARIES}
    CGAL::CGAL
    grid_environment_lib
)
target_link_libraries(geometry_lib PRIVATE
    CGAL::CGAL
    coal::coal
)

target_include_directories(geometry_lib PRIVATE include)
target_link_libraries(surface_lib PRIVATE 
    utils_lib
    geometry_lib
    ${VTK_LIBRARIES}
    CGAL::CGAL
)
target_link_libraries(astar_search_lib PRIVATE
    tree_lib
    utils_lib
    node_lib
    surface_lib
    geometry_lib
    visualizer_lib
    ${VTK_LIBRARIES}
    CGAL::CGAL
    Boost::boost
)
target_link_libraries(astar_grid_search_lib PRIVATE
    tree_lib
    utils_lib
    node_lib
    surface_lib
    geometry_lib
    grid_environment_lib
    visualizer_lib
    ${VTK_LIBRARIES}
    CGAL::CGAL
    Boost::boost
)
target_link_libraries(footstep_planner_lib PRIVATE
    astar_search_lib
    tree_lib
    utils_lib
    geometry_lib
    visualizer_lib
    ${VTK_LIBRARIES}
    CGAL::CGAL
    casadi::casadi
)

# Add executables
add_executable(nas_plan src/nas_plan.cpp)
add_executable(astar_plan src/astar_plan.cpp)
add_executable(astar_grid_plan src/astar_grid_plan.cpp)
add_executable(test_grid_visualization test_grid_visualization.cpp)
add_executable(test_auto_plot test_auto_plot.cpp)
add_executable(test_bench_operations test_bench_operations.cpp)
add_executable(test_print_paths test_print_paths.cpp)
add_executable(golden_capture tests/golden_capture.cpp)

# Link libraries for first executable
target_link_libraries(nas_plan PRIVATE
    footstep_planner_lib
    astar_search_lib
    tree_lib
    utils_lib
    node_lib
    surface_lib
    geometry_lib
    visualizer_lib
    ${VTK_LIBRARIES}
    CGAL::CGAL
    yaml-cpp::yaml-cpp
    casadi::casadi
)

# Link libraries for A* planning executable
target_link_libraries(astar_plan PRIVATE
    tree_lib
    utils_lib
    node_lib
    surface_lib
    geometry_lib
    visualizer_lib
    ${VTK_LIBRARIES}
    CGAL::CGAL
    yaml-cpp::yaml-cpp
    astar_search_lib
    footstep_planner_lib
    casadi::casadi
)

# Link libraries for grid A* planning executable
target_link_libraries(astar_grid_plan PRIVATE
    tree_lib
    utils_lib
    node_lib
    surface_lib
    geometry_lib
    grid_environment_lib
    visualizer_lib
    ${VTK_LIBRARIES}
    CGAL::CGAL
    yaml-cpp::yaml-cpp
    astar_search_lib
    astar_grid_search_lib
    footstep_planner_lib
    casadi::casadi
)

# Link libraries for test grid visualization executable
target_link_libraries(test_grid_visualization PRIVATE
    astar_grid_search_lib
    grid_environment_lib
    visualizer_lib
    surface_lib
    geometry_lib
    utils_lib
    node_lib
    ${VTK_LIBRARIES}
    CGAL::CGAL
    yaml-cpp::yaml-cpp
)

# Link libraries for test auto plot executable
target_link_libraries(test_auto_plot PRIVATE
    astar_grid_search_lib
    grid_environment_lib
    visualizer_lib
    surface_lib
    geometry_lib
    utils_lib
    node_lib
    ${VTK_LIBRARIES}
    CGAL::CGAL
    yaml-cpp::yaml-cpp
)

# Enable VTK module autoinit for all executables
vtk_module_autoinit(
  TARGETS nas_plan astar_plan astar_grid_plan test_grid_visualization test_auto_plot test_bench_operations golden_capture
  MODULES ${VTK_LIBRARIES}
)

# Link libraries for bench operations
target_link_libraries(test_bench_operations PRIVATE
    utils_lib
    node_lib
    surface_lib
    geometry_lib
    CGAL::CGAL
    Boost::boost
)

target_link_libraries(test_print_paths PRIVATE
    utils_lib
    node_lib
    surface_lib
    geometry_lib
    CGAL::CGAL
    Boost::boost
)

# Link libraries for golden_capture (Stage A phase 0, see PLAN.md) — mirrors
# astar_plan's dependency set since it drives the same AstarSearch/Tree/
# FootstepPlanner classes, minus any actual VTK usage (headless capture).
target_link_libraries(golden_capture PRIVATE
    tree_lib
    utils_lib
    node_lib
    surface_lib
    geometry_lib
    visualizer_lib
    ${VTK_LIBRARIES}
    CGAL::CGAL
    astar_search_lib
    footstep_planner_lib
    casadi::casadi
)
