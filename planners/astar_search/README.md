# planners/astar_search

`AstarSearch` — CASSR's search planner, de-globalized (every old global read from `constants.hpp` is now a constructor parameter via `AstarSearchConfig`), with the actual expansion delegated entirely to `core/expansion::expand_node()`.

## API

See [`include/nas/planners/astar_search.hpp`](include/nas/planners/astar_search.hpp).
- `AstarSearch(surfaces, reachability, config)`, `search()`, `result_path()`, `expansion_count()`.
- `AstarSearchConfig`: start/goal position+stance foot, `distance_metric` (Euclidean/Gjk/Epa), `heuristic_weight` (×10 for gjk/epa only, matching the old code — see `docs/paper-deltas.md`), `node_similarity_threshold` (paper: "set empirically to 2cm"), `expansion_params`.

Behavior is a faithful port of the old `AstarSearch` — same priority queue (`boost::heap::fibonacci_heap`), same node-similarity dedup by quantized `(surface_id, stance_foot, yaw, centroid, perimeter)`. Validated against golden references from the old implementation: exact node-for-node match on `NarrowPassage` and `ThreePathsNAS`.

## Dependencies

`core/expansion`, `core/node`, `core/reachability`, `core/surface`, Boost (heap, functional/hash).

## Testing standalone

Golden-comparison tests use fixtures from [`tests/fixtures`](../../tests/fixtures) — that module must be added *after* this one in any consuming `CMakeLists.txt` (circular dependency otherwise, see `tests/fixtures/CMakeLists.txt`).

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2 && ctest
```
