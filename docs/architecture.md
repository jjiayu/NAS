# Architecture

One CMake project (`nas`), consolidating what used to be ~17 independent CMake sub-projects (each
its own `project()`/`find_package()`/`build/`) from the staged CASSR rewrite — that fragmentation
was useful while the rewrite was in progress (isolate each step, compare against the old code
piece by piece) but had no reason to persist once it was validated. See
`docs/history/PLAN.md`/`docs/history/PROGRESS.md` for that history, and `docs/paper-deltas.md` for
where the implementation departs from the CASSR/NAS papers.

```
NAS/
├── CMakeLists.txt          # the one CMake project: library + apps + tools + tests + bindings
├── pyproject.toml          # Python packaging (scikit-build-core) for bindings/
├── cmake/nasConfig.cmake.in
├── include/nas/            # public headers, namespace nas::
│   ├── core/                geometry, node, surface, reachability, expansion
│   ├── planners/             astar_search (CASSR), grid_astar_search (grid baseline)
│   ├── footstep_qp/          the QP formulation + backend(s)
│   └── config/                RobotModel, Scenario, PlannerConfig, STL import
├── src/                    # mirrors include/nas/ — all compiled into the single `nas` library
├── apps/astar_plan/        # CLI: one scenario + one config in, one JSON result out
├── bindings/                Python extension (nanobind), `import nas_bindings`
├── tools/                   diagnostics/benchmarks, not pass/fail tests (see below)
├── tests/
│   ├── common/               nas_fixtures: shared scenarios + golden-comparison helpers
│   ├── unit/                 fast (<1s) directed tests
│   ├── golden/                slower regression suite against recorded golden data
│   ├── golden_data/           the golden JSON itself (captured once from the old code)
│   └── main.cpp               the one nas_tests executable, see below
├── talosReachability/       sibling package: Talos's reachability .obj data, own install/find_package
├── viz/                      Python plotting scripts (plans_report.py, meshcat_view.py, ...)
├── docs/                      this file, paper-deltas.md, paper-corrections-draft.md, history/
└── legacy/                   the original NAS implementation, kept for reference — own build, see legacy/README.md
```

## The library

One target, `nas` (alias `nas::nas`), covering:

- **`core/geometry`** — pure geometric operations shared by every search strategy and the footstep
  QP: 2D/3D transforms, polygon clipping, EPA distance, polytope↔half-space conversion. No
  dependency on `Node`, a search strategy, or global config.
- **`core/node`** — `Node` (biped-only, de-globalized) and `NodePool`, shared by CASSR and the grid
  baseline.
- **`core/surface`** — one walkable surface: 3D/2D vertex loops, plane, centroid, world↔surface
  transforms, shrunk by half the foot dimensions.
- **`core/reachability`** — `ReachabilityModel`, a query keyed by `(moving_effector,
  support_effector, direction)` over loaded `.obj` polytopes; caches each polytope's H-rep the
  first time it's asked for (see `ReachabilityModel::half_space_constraint`).
- **`core/expansion`** — `expand_node()`, the single expansion step (Minkowski sum of the parent
  patch with the reachability polytope, clipped to every surface) shared by both planners.
- **`planners/astar_search`** — `AstarSearch`, CASSR's planner: weighted A\* over continuous
  surface patches, `AstarSearchConfig` for every tunable (distance metric, optional edge costs on
  rotation/heading, goal as a point or a surface, cycle detection, `max_expansions`).
- **`planners/grid_astar_search`** — the discretized grid baseline used for comparison in the
  paper: same overall search shape, but a uniform 2D grid instead of continuous patches.
- **`footstep_qp`** — builds a backend-agnostic `QPProblem` from a CASSR result path, solved with
  `QuadprogBackend` (eiquadprog; the only backend kept — see `docs/paper-deltas.md` for why ProxQP
  was tried and dropped).
- **`config`** — `RobotModel`, the 15-scenario `Scenario` registry, `PlannerConfig` (JSON-loaded
  planner configuration), STL scene import.

## `apps/`, `bindings/`, `tools/`

- **`apps/astar_plan`** — thin CLI: `<scenario_name> <planner_config.json>
  <talos_reachability_data_dir> <output.json>`. No logic of its own — correctness comes from the
  library's own tests.
- **`bindings/`** — `nas_bindings`, the Python entry point (nanobind). `NAS_BUILD_BINDINGS=OFF` by
  default in a plain C++ dev build; built by `pip install .` via `pyproject.toml`.
- **`tools/`** — diagnostics and benchmarks, not pass/fail tests: `nas_bench_perf` (search+QP timing
  per scenario, kept buildable against an older git tag for a same-machine baseline),
  `nas_bench_compare` (old-vs-new timing), `nas_trace_divergence` (where two heap states first
  diverge), `nas_dump_plan` (JSON export consumed by `viz/plans_report.py`), `stl_preview` (dumps an
  STL-imported scenario's surfaces to JSON, no search/QP involved).

## Tests

`nas_tests`, one executable for everything (`tests/main.cpp`) — each test file keeps its original
body (including its own anonymous-namespace helpers, e.g. `check()`/`g_failures`; those have
internal linkage per translation unit, so they don't collide when linked together), only its `int
main()` was renamed to a unique `int run_<name>()`, declared and dispatched from `main.cpp`.

```bash
ctest --test-dir build -R nas_tests_fast     # tests/unit/*, <1s
ctest --test-dir build -R nas_tests_golden   # tests/golden/*, ~2-3 min (cycle_detection ~30s, merge_consistency ~45s)
```

Known trade-off: `tests/unit/config_*_test.cpp` still use `assert()` internally (ported as-is) —
an assertion failure there aborts the whole `nas_tests` process rather than just failing its own
suite. Not a live problem (every suite passes as of this writing); worth converting to the
`check()`/`g_failures` idiom the other files use if it ever bites.

## Data flow (a typical run)

`config::load_scenario(name)` (or `load_scenario_from_stl`) → surfaces → `AstarSearch::search()`
(needs a `ReachabilityModel`, loaded once from `talosReachability`'s `.obj` files) → a path of
`Node*` → `solve_footstep_qp(path, ...)` → a `FootstepPlan` (footstep positions + feasibility). This
is exactly what `apps/astar_plan`, `bindings/src/module.cpp`, and every `tests/golden/*` file do.
