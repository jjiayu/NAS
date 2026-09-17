cmake_minimum_required(VERSION 3.15)

# -----------------------------
# Project definition
# -----------------------------
set(PROJECT_NAME talosReachability)
set(PROJECT_DESCRIPTION ".obj files describing the kinematic reachability constraints of the Talos robot")

project(${PROJECT_NAME} LANGUAGES NONE DESCRIPTION "${PROJECT_DESCRIPTION}")

# -----------------------------
# Source / data files
# -----------------------------
set(REACHABILITY_CONSTRAINTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data/reachability_constraints")
file(GLOB OBJ_FILES "${REACHABILITY_CONSTRAINTS_DIR}/*.obj")

# -----------------------------
# Installation of OBJ files
# -----------------------------
# Install into share/<PROJECT_NAME>/reachability_constraints
install(FILES ${OBJ_FILES}
        DESTINATION "share/${PROJECT_NAME}/reachability_constraints")

# -----------------------------
# Generate portable CMake Config
# -----------------------------
set(CONFIG_INSTALL_DIR "share/${PROJECT_NAME}/cmake")

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/talosReachabilityConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/talosReachabilityConfig.cmake
    @ONLY
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/talosReachabilityConfig.cmake
        DESTINATION ${CONFIG_INSTALL_DIR})

# -----------------------------
# Generate portable Python Config
# -----------------------------
set(PY_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/talosReachabilityConfig.py")

file(WRITE "${PY_CONFIG_FILE}"
"import os\n"
"TALOSREACHABILITY_CONSTRAINTS_DIR = os.path.join(os.path.dirname(__file__), 'reachability_constraints/')\n"
)

install(FILES "${PY_CONFIG_FILE}"
        DESTINATION "share/${PROJECT_NAME}/")

# -----------------------------
# Info messages
# -----------------------------
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Description: ${PROJECT_DESCRIPTION}")
message(STATUS "Found OBJ files: ${OBJ_FILES}")
message(STATUS "OBJ files will be installed to: share/${PROJECT_NAME}/reachability_constraints")
message(STATUS "CMake config will be installed to: ${CONFIG_INSTALL_DIR}")
message(STATUS "Python config will be installed to: share/${PROJECT_NAME}/")
