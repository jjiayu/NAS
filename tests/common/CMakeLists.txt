# Fragment included unconditionally by the root CMakeLists.txt (not gated behind
# NAS_BUILD_TESTS/NAS_BUILD_TOOLS: both tests/ and some tools/ executables need it).
# Shared test scenarios (nas/fixtures/scenarios.hpp) and golden-comparison helpers
# (nas/fixtures/golden_compare.hpp).
add_library(nas_fixtures
    src/scenarios.cpp
    src/golden_compare.cpp
)
target_include_directories(nas_fixtures PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(nas_fixtures PUBLIC nas nlohmann_json::nlohmann_json)
