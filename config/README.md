# config

Runtime configuration types (see PLAN.md phase 9): `RobotModel` (foot/CoM dimensions), the `Scenario` registry, and `PlannerConfig` — replacing the old code's `environments.hpp`/`constants.hpp` globals.

## API

- [`include/nas/config/robot_model.hpp`](include/nas/config/robot_model.hpp): `RobotModel{foot_length=0.22, foot_width=0.22, com_z_height=0.75}` — couche 0 only (no file loader), same pattern as `core/reachability`.
- [`include/nas/config/scenario.hpp`](include/nas/config/scenario.hpp): `Scenario{name, std::vector<Surface> surfaces}`, `available_scenarios()`, `load_scenario(name, robot_model = RobotModel{})`.
- [`include/nas/config/planner_config.hpp`](include/nas/config/planner_config.hpp): `PlannerConfig{AstarSearchConfig astar, FootstepQPConfig qp}`, `load_planner_config(json_path)`.

`load_scenario` builds `Surface` objects from one of the 11 raw vertex lists ported verbatim from the old code's `include/environments.hpp` (`Stairs`, `TwoFlatSurfaces`, `Flat`, `LongStairs`, `LongLongStairs`, `LongStairsComplete`, `LongStairsExp`, `ThreePathsScene`, `Stairs_Up_Down`, `ThreePathsNAS`, `NarrowPassage`), shrunk by the given `RobotModel`'s foot dimensions.

This is distinct from [`tests/fixtures`](../tests/fixtures), which stays the test harness (2 scenarios, bundled with an `AstarSearchConfig`) — `config/` is the production-facing registry covering every scene the old code knew, with no search config attached. The two modules don't share their `NarrowPassage`/`ThreePathsNAS` vertex data (deliberately not deduplicated yet, see PLAN.md).

`load_planner_config` parses a JSON file into `AstarSearchConfig`/`ExpansionParams`/`FootstepQPConfig` — it does not duplicate those structs' fields, it only overrides what's present in the JSON, everything else keeps that struct's own default. `astar.start_position`/`astar.goal_location` are the only required fields (no sensible scenario-independent default). Schema:

```jsonc
{
  "astar": {
    "start_position": [0.0, 0.0, 0.0],       // required, [x, y, z]
    "goal_location": [8.0, 0.0, 0.0],        // required, [x, y, z]
    "start_stance_foot": "Right",            // "Left" | "Right"
    "start_foot_yaw": 0.0,
    "goal_stance_foot": "Left",
    "distance_metric": "Epa",                // "Euclidean" | "Gjk" | "Epa"
    "heuristic_weight": 10.0,
    "node_similarity_threshold": 0.02,
    "expansion": {
      "rotation_enabled": true,
      "yaw_discretization_num": 3,
      "yaw_angle_increment_deg": 10.0,       // degrees in JSON, radians internally
      "cycle_detection_enabled": true
    }
  },
  "qp": {                                    // whole section optional
    "alpha_weight": 10.0,
    "rotation_enabled": true,
    "hessian_regularization": 1e-8
  }
}
```

Not yet implemented (see PLAN.md phase 9d): importing a scenario from an STL file instead of a hardcoded vertex list.

## Dependencies

`core/surface`, `planners/astar_search`, `footstep_qp`, CGAL, nlohmann::json (header-only, picked up via the conda prefix include path like elsewhere in the repo).

## Testing standalone

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2 && ctest
```
