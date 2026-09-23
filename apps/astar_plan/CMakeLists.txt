# Fragment included by the root CMakeLists.txt (NAS_BUILD_APPS) — no project()/find_package() of
# its own, the `nas` target already carries everything it needs.
add_executable(astar_plan src/main.cpp)
target_link_libraries(astar_plan PRIVATE nas)
