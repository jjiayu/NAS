# config

Runtime configuration types (see PLAN.md phase 9): `RobotModel` (foot/CoM dimensions) and the `Scenario` registry that replaces the old code's `environments.hpp`/`constants.hpp` globals.

## API

- [`include/nas/config/robot_model.hpp`](include/nas/config/robot_model.hpp): `RobotModel{foot_length=0.22, foot_width=0.22, com_z_height=0.75}` — couche 0 only (no file loader), same pattern as `core/reachability`.
- [`include/nas/config/scenario.hpp`](include/nas/config/scenario.hpp): `Scenario{name, std::vector<Surface> surfaces}`, `available_scenarios()`, `load_scenario(name, robot_model = RobotModel{})`.

`load_scenario` builds `Surface` objects from one of the 11 raw vertex lists ported verbatim from the old code's `include/environments.hpp` (`Stairs`, `TwoFlatSurfaces`, `Flat`, `LongStairs`, `LongLongStairs`, `LongStairsComplete`, `LongStairsExp`, `ThreePathsScene`, `Stairs_Up_Down`, `ThreePathsNAS`, `NarrowPassage`), shrunk by the given `RobotModel`'s foot dimensions.

This is distinct from [`tests/fixtures`](../tests/fixtures), which stays the test harness (2 scenarios, bundled with an `AstarSearchConfig`) — `config/` is the production-facing registry covering every scene the old code knew, with no search config attached. The two modules don't share their `NarrowPassage`/`ThreePathsNAS` vertex data (deliberately not deduplicated yet, see PLAN.md).

Not yet implemented (see PLAN.md phase 9d): importing a scenario from an STL file instead of a hardcoded vertex list.

## Dependencies

`core/surface`, CGAL.

## Testing standalone

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2 && ctest
```
