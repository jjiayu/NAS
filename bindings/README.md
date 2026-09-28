# bindings

Python entrypoint for CASSR (see PLAN.md phase 12), built with [nanobind](https://github.com/wjakob/nanobind).

## API

```python
import nas_bindings

nas_bindings.available_scenarios()  # -> list[str], same names as config::available_scenarios()

result = nas_bindings.plan(scenario_name, planner_config_path, talos_reachability_data_dir)
result.success          # bool
result.positions        # list[list[float]], one [x, y, z] per path node
result.stance_feet      # list[int], 0=Left, 1=Right (matches StanceFoot's own values)
result.foot_yaws        # list[float], radians
result.expansion_count  # int, AstarSearch::expansion_count()
result.search_ms        # float, wall time of the A* search
result.qp_ms            # float, wall time of the footstep QP (0 if no path was found)
```

`planner_config_path` is the same JSON schema the `astar_plan` CLI reads — see
[`docs/tutorial-running-a-scenario.md`](../docs/tutorial-running-a-scenario.md) for the full schema.

### Building the config directly in Python (no JSON file)

```python
config = nas_bindings.PlannerConfig(start_position=(0.0, 0.0, 0.0))
config.foot_goals.left = nas_bindings.FootGoal.point((1.0, 0.0, 0.0))
# also: FootGoal.surface(surface_id), .polytope([(x,y,z), ...]), .offset((dx,dy,dz)),
# .polygon_2d(surface_id, [(u,v), ...]) -- same 5 region shapes as the JSON schema.
# goal.yaw_range_deg = (min, max) is settable on any of them.

result = nas_bindings.plan_with_config(scenario_name, config, talos_reachability_data_dir)
```

`PlannerConfig` mirrors the same fields as the JSON `"astar"`/`"qp"` sections (same defaults) —
`start_stance_foot`, `heuristic_weight`, `rotation_enabled`, `qp_rotation_enabled`, etc. — see
`docs/tutorial-running-a-scenario.md` for the full field list. `foot_goals.left`/`.right`: at least
one must be set (`plan_with_config` raises `ValueError` otherwise, same message as the JSON path).
`FootGoal`'s region is fixed at construction — one of the 5 static factories above, never a raw
union — so there's no "exactly one region" validation to get wrong, unlike JSON. `plan()` (the
file-based entrypoint above) is unchanged; this is an additional, equivalent way to reach the same
`AstarSearch`/`solve_footstep_qp` pipeline (both converge on the same internal
`nas::config::PlannerConfig`), useful for iterating on a goal from a REPL/notebook without writing a
file each time.

**Couche 0 only** — every argument is an explicit path/name, nothing is discovered or guessed (same contract as `core/reachability`, `config/`, `talosReachability`). A convenience layer (building a planner from a package that extracts its own files) was explicitly descoped for now, see PLAN.md's "Différé".

**Flat DTO, not the C++ types** — `FootstepResult` is plain data; `Node`/`Surface`/CGAL types are never exposed to Python (CGAL kernel types in particular have no sane nanobind binding, and they're implementation detail, not a stable public surface).

**GIL released during the solve** — the search + QP call (the only part that can take tens of milliseconds) runs with `nb::gil_scoped_release`; no Python object is touched until it returns.

Errors: bad scenario name / malformed config / missing `.obj` file raise Python exceptions via nanobind's built-in translation (`std::out_of_range` → `IndexError`, `std::invalid_argument` → `ValueError`, everything else → `RuntimeError`).

## Dependencies

`config/` (transitively: `core/*`, `planners/astar_search`, `footstep_qp`), CGAL, [nanobind](https://github.com/wjakob/nanobind) (`pip install nanobind` — light, no transitive C++ dependency chain, unlike e.g. meshcat-cpp).

Building a Python extension module requires every static library it links to be position-independent — `CMAKE_POSITION_INDEPENDENT_CODE ON` is set here (not in the libraries themselves, which are also linked into plain executables elsewhere in the repo).

## Testing standalone

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2 && ctest
```
