cmake_minimum_required(VERSION 3.15)

# Set build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_BUILD_TYPE Release)

project(
  nas
  VERSION 1.0
  LANGUAGES CXX
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set Homebrew paths for Apple Silicon
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/homebrew/opt/cgal")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/homebrew/opt/vtk@9.2")

# Use PkgConfig to find FreeType (Homebrew doesn't provide CMake config)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FREETYPE REQUIRED freetype2)

# Set custom path for CasADi
set(CMAKE_PREFIX_PATH "/Users/jiayu/local" ${CMAKE_PREFIX_PATH})

# Find required packages
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(VTK 9.2 REQUIRED COMPONENTS 
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

# Add the include directory
include_directories(include)
include_directories(${VTK_INCLUDE_DIRS})
include_directories("/opt/homebrew/include")  # For CGAL headers
include_directories(${EIGEN3_INCLUDE_DIR})  # For Eigen3 headers

# Add library with source files
add_library(utils_lib src/utils.cpp)
add_library(node_lib src/node.cpp)
add_library(tree_lib src/tree.cpp)
add_library(geometry_lib src/geometry.cpp)
add_library(surface_lib src/surface.cpp)
add_library(visualizer_lib src/visualizer.cpp)
add_library(astar_search_lib src/astar_search.cpp)
add_library(footstep_planner_lib src/footstep_planner.cpp)

# Set library dependencies
target_link_libraries(node_lib PRIVATE geometry_lib CGAL::CGAL)
target_link_libraries(tree_lib PRIVATE utils_lib node_lib surface_lib geometry_lib visualizer_lib ${VTK_LIBRARIES})
target_link_libraries(utils_lib PRIVATE 
    ${VTK_LIBRARIES}
    visualizer_lib
    CGAL::CGAL
)
target_link_libraries(visualizer_lib PRIVATE
    ${VTK_LIBRARIES}
    CGAL::CGAL
)
target_link_libraries(geometry_lib PRIVATE
    CGAL::CGAL
    /opt/homebrew/lib/libcoal.dylib
)
target_include_directories(geometry_lib PRIVATE /opt/homebrew/include)
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


# Enable VTK module autoinit for both executables
vtk_module_autoinit(
  TARGETS nas_plan astar_plan
  MODULES ${VTK_LIBRARIES}
)