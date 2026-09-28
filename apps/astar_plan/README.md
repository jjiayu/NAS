# apps/astar_plan

Thin CLI driver for CASSR/AstarSearch (see PLAN.md phase 10). Not itself unit-tested — it's a thin wrapper over already-tested library code (`config/`, `planners/astar_search`, `footstep_qp`); correctness comes from those modules' own tests. Smoke-tested manually against `NarrowPassage`/`ThreePathsNAS` (see PROGRESS.md).

Where `tools/nas_dump_plan.cpp` is a test tool restricted to `tests/fixtures`'s 2 hardcoded scenarios/configs, this is the production entrypoint: any scenario from `config::available_scenarios()` and any `AstarSearchConfig`/`FootstepQPConfig` loaded from a JSON file. Both tools dump the same JSON shape on purpose, so either can feed the same downstream SVG/plot render.

## Usage

```sh
./astar_plan <scenario_name> <planner_config.json> <talos_reachability_data_dir> <output.json>
```

- `scenario_name`: one of `config::available_scenarios()`.
- `planner_config.json`: see [`docs/tutorial-running-a-scenario.md`](../../docs/tutorial-running-a-scenario.md) for the full JSON schema (the goal is always `astar.foot_goals`, a point/surface/polytope/2D-polygon per foot).
- `talos_reachability_data_dir`: e.g. `talosReachability/data/reachability_constraints`.
- `output.json`: surfaces/path/footsteps dump, same shape as `nas_dump_plan`'s output, plus `path_found`, `start`, `foot_goals`, `expansions`, `search_ms`, `qp_ms`. It is written even when no path is found (`path_found: false`, exit code 1).
- `examples/`: one config per scenario (`<Scenario>.json`, goal given by `foot_goals.left.offset`), two with an absolute point (`foot_goals.left.point`), one with a whole-surface goal (`Stairs_goal_surface.json`), and one demonstrating a 2D-polygon-on-a-surface goal (`Flat_polygon_2d.json`). `viz/` renders any of these dumps (see `viz/README.md`).

## Dependencies

`config/` (transitively: `core/*`, `planners/astar_search`, `footstep_qp`), CGAL, nlohmann::json.

## Building standalone

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2
```
