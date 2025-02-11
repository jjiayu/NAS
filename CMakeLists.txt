cmake_minimum_required(VERSION 3.15)

project(
  nas
  VERSION 1.0
  LANGUAGES CXX
)

# Find required packages
find_package(CGAL REQUIRED)
find_package(Matplot++ REQUIRED)

# Add the include directory
include_directories(include)

add_library(node_lib src/node.cpp)

add_executable(nas_plan src/nas_plan.cpp)
target_link_libraries(nas_plan PRIVATE 
    node_lib
    CGAL::CGAL
    Matplot++::matplot
)

# Set C++ standard (Matplot++ requires C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)