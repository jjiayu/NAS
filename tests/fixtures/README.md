# tests/fixtures

Shared test scenarios and golden-comparison helpers, extracted (8d-2) after the `NarrowPassage`/`ThreePathsNAS` setup and `ReachabilityModel` construction had been duplicated across 5 test files.

## API

- [`include/nas/fixtures/scenarios.hpp`](include/nas/fixtures/scenarios.hpp): `Scenario{name, surfaces, astar_config}`, `make_narrow_passage()`, `make_three_paths_nas()`, `make_forward_reachability_model(talos_data_dir)`.
- [`include/nas/fixtures/golden_compare.hpp`](include/nas/fixtures/golden_compare.hpp): `check_path_matches_golden(path, golden_json_path)`.

Consumers: `planners/astar_search`'s golden tests, `footstep_qp`'s golden test, `tests/perf/compare_perf.cpp`, `tests/viz_dump/dump_plan.cpp`.

## Dependencies

`planners/astar_search`, CGAL. **Important**: this module needs `planners/astar_search`'s types, but `astar_search`'s own tests need this module — a naive `add_subdirectory` on both sides would be circular. This `CMakeLists.txt` deliberately does *not* `add_subdirectory(astar_search)` itself (it `FATAL_ERROR`s if the target is missing instead); every consumer must `add_subdirectory(astar_search)` **then** `add_subdirectory(tests/fixtures)`, in that order.

## Testing standalone

Not independently testable (no tests of its own) — built as a dependency of the consumers listed above.
