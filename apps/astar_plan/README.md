# apps/astar_plan

Thin CLI driver for CASSR/AstarSearch (see PLAN.md phase 10). Not itself unit-tested — it's a thin wrapper over already-tested library code (`config/`, `planners/astar_search`, `footstep_qp`); correctness comes from those modules' own tests. Smoke-tested manually against `NarrowPassage`/`ThreePathsNAS` (see PROGRESS.md).

Where [`tests/viz_dump/dump_plan.cpp`](../../tests/viz_dump/dump_plan.cpp) is a test tool restricted to `tests/fixtures`'s 2 hardcoded scenarios/configs, this is the production entrypoint: any scenario from `config::available_scenarios()` (11 today, plus whatever an STL import adds later, see PLAN.md phase 9d) and any `AstarSearchConfig`/`FootstepQPConfig` loaded from a JSON file. Both tools dump the same JSON shape on purpose, so either can feed the same downstream SVG/plot render.

## Usage

```sh
./astar_plan <scenario_name> <planner_config.json> <talos_reachability_data_dir> <output.json>
```

- `scenario_name`: one of `config::available_scenarios()` — see [`config/README.md`](../../config/README.md) for the list.
- `planner_config.json`: see [`config/README.md`](../../config/README.md) for the schema.
- `talos_reachability_data_dir`: e.g. `talosReachability/data/reachability_constraints`.
- `output.json`: surfaces/path/footsteps dump, same shape as `dump_plan.cpp`'s output.

## Dependencies

`config/` (transitively: `core/*`, `planners/astar_search`, `footstep_qp`), CGAL, nlohmann::json.

## Building standalone

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2
```
