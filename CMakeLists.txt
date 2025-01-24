cmake_minimum_required(VERSION 3.15)

project(
  nas
  VERSION 1.0
  LANGUAGES CXX
)

# add_library(MyLibExample simple_lib.cpp simple_lib.hpp)

# Add the include directory
include_directories(include)

add_library(node_lib src/node.cpp)

add_executable(nas_plan src/nas_plan.cpp)
target_link_libraries(nas_plan PRIVATE node_lib)

# target_link_libraries(nas_plan PRIVATE MyLibExample)