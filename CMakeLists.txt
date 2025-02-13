cmake_minimum_required(VERSION 3.15)

project(
  nas
  VERSION 1.0
  LANGUAGES CXX
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set Homebrew paths for Apple Silicon
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/homebrew/opt/cgal")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/homebrew/opt/eigen")

# Find required packages
find_package(CGAL REQUIRED)
find_package(VTK REQUIRED COMPONENTS 
  CommonCore
  CommonDataModel
  FiltersSources
  InteractionStyle
  RenderingCore
  RenderingOpenGL2
)

# Find Eigen3 (Mac Silicon specific)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add the include directory
include_directories(include)
include_directories(${VTK_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories("/opt/homebrew/include")  # For CGAL headers

# Add library with source files
add_library(utils_lib src/utils.cpp)
add_library(node_lib src/node.cpp)
add_library(tree_lib src/tree.cpp)

# Set library dependencies
target_link_libraries(tree_lib PRIVATE utils_lib node_lib)

# Add executable
add_executable(nas_plan src/nas_plan.cpp)

# Link libraries
target_link_libraries(nas_plan PRIVATE 
    node_lib
    tree_lib
    utils_lib
    CGAL::CGAL
    ${VTK_LIBRARIES}
    Eigen3::Eigen
)

# Enable VTK modules
vtk_module_autoinit(
  TARGETS nas_plan
  MODULES ${VTK_LIBRARIES}
)