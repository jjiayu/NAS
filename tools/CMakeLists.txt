# Fragment included by the root CMakeLists.txt (NAS_BUILD_TOOLS) — diagnostics/benchmarks, not
# pass/fail tests (see tests/ for those). No project()/find_package() of its own.

set(TALOS_DATA_DIR "${CMAKE_SOURCE_DIR}/talosReachability/data/reachability_constraints")

# Search + QP timing on every scenario. Kept deliberately dependent only on the public nas/ API
# (nothing tests/common-only) — written so the same source also builds in a checkout of an older
# commit for a same-machine baseline (see the file's own header comment).
add_executable(nas_bench_perf nas_bench_perf.cpp)
target_link_libraries(nas_bench_perf PRIVATE nas)
target_compile_definitions(nas_bench_perf PRIVATE TALOS_REACHABILITY_DATA_DIR="${TALOS_DATA_DIR}")

# Old-vs-new timing comparison (phase 8b of the rewrite).
add_executable(nas_bench_compare nas_bench_compare.cpp)
target_link_libraries(nas_bench_compare PRIVATE nas nas_fixtures)

# Diagnostic: where do two heap states first make the search diverge?
add_executable(nas_trace_divergence nas_trace_divergence.cpp)
target_link_libraries(nas_trace_divergence PRIVATE nas)
target_compile_definitions(nas_trace_divergence PRIVATE TALOS_REACHABILITY_DATA_DIR="${TALOS_DATA_DIR}")

# Visual-sanity export: dumps a scenario's search+QP result as JSON for viz/plans_report.py.
add_executable(nas_dump_plan nas_dump_plan.cpp)
target_link_libraries(nas_dump_plan PRIVATE nas nas_fixtures)

# Dumps an STL-imported scenario's surfaces to JSON — no search/QP involved.
add_executable(stl_preview stl_preview.cpp)
target_link_libraries(stl_preview PRIVATE nas)
