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
add_library(visualizer_lib src/visualizer.cpp)
add_library(footstep_planner_lib src/footstep_planner.cpp)

# Set library dependencies
target_link_libraries(node_lib PRIVATE geometry_lib CGAL::CGAL)
target_link_libraries(tree_lib PRIVATE utils_lib node_lib surface_lib geometry_lib visualizer_lib ${VTK_LIBRARIES})
target_link_libraries(utils_lib PRIVATE 
    ${VTK_LIBRARIES}
    CGAL::CGAL
)
target_link_libraries(visualizer_lib PRIVATE
    ${VTK_LIBRARIES}
    CGAL::CGAL
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
target_link_libraries(footstep_planner_lib PRIVATE
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
add_executable(test_bench_operations test_bench_operations.cpp)
add_executable(test_print_paths test_print_paths.cpp)
add_executable(old_qp_check tests/old_qp_check.cpp)

# Old QP started cold vs warm (tests/old_qp_warmstart.cpp): a patched COPY of src/footstep_planner.cpp is
# generated at configure time (a global warm-start vector and one line setting arg["x0"]), the original is untouched.
file(READ ${CMAKE_SOURCE_DIR}/src/footstep_planner.cpp OLD_FP_SRC)
string(REPLACE "namespace nas {" "std::vector<double> g_qp_warm_start;\nnamespace nas {" OLD_FP_SRC "${OLD_FP_SRC}")
string(REPLACE "arg[\"lbx\"] = lbx;" "arg[\"lbx\"] = lbx; if (!g_qp_warm_start.empty()) { arg[\"x0\"] = casadi::DM(g_qp_warm_start); }" OLD_FP_SRC "${OLD_FP_SRC}")
# OLD_QP_REG=<value> in the environment adds <value> * |x|^2 to the objective (a Hessian regularization); unset = the old QP as is.
string(REPLACE "qp[\"f\"] = objective;" "{ const char* reg_env = std::getenv(\"OLD_QP_REG\"); if (reg_env) objective += std::atof(reg_env) * casadi::SX::dot(all_vars, all_vars); } qp[\"f\"] = objective;" OLD_FP_SRC "${OLD_FP_SRC}")
file(WRITE ${CMAKE_BINARY_DIR}/old_qp_warmstart_planner.cpp "${OLD_FP_SRC}")
add_executable(old_qp_warmstart tests/old_qp_warmstart.cpp ${CMAKE_BINARY_DIR}/old_qp_warmstart_planner.cpp)
target_include_directories(old_qp_warmstart PRIVATE include)

# Link libraries for first executable
target_link_libraries(nas_plan PRIVATE
    footstep_planner_lib
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

# Enable VTK module autoinit for all executables
vtk_module_autoinit(
  TARGETS nas_plan test_bench_operations old_qp_check old_qp_warmstart
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

# Feasibility check of the rewritten CASSR's plans with the old footstep QP (tests/old_qp_check.cpp).
target_link_libraries(old_qp_check PRIVATE
    footstep_planner_lib
    tree_lib
    utils_lib
    node_lib
    surface_lib
    geometry_lib
    visualizer_lib
    ${VTK_LIBRARIES}
    CGAL::CGAL
    casadi::casadi
)

target_link_libraries(old_qp_warmstart PRIVATE
    tree_lib
    utils_lib
    node_lib
    surface_lib
    geometry_lib
    visualizer_lib
    ${VTK_LIBRARIES}
    CGAL::CGAL
    casadi::casadi
)
