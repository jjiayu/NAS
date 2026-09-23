# planners/astar_search

`AstarSearch` — CASSR's search planner, de-globalized (every old global read from `constants.hpp` is now a constructor parameter via `AstarSearchConfig`), with the actual expansion delegated entirely to `core/expansion::expand_node()`.

## API

See [`include/nas/planners/astar_search.hpp`](include/nas/planners/astar_search.hpp).
- `AstarSearch(surfaces, reachability, config)`, `search()`, `result_path()`, `expansion_count()`.
- `AstarSearchConfig`: start/goal position+stance foot, `distance_metric` (Euclidean/Epa, default Epa), `heuristic_weight` (×10 for gjk/epa only, matching the old code — see `docs/paper-deltas.md`), `node_similarity_threshold` (paper: "set empirically to 2cm"), `expansion_params`.

Behavior: same weighted A* as the old `AstarSearch` (priority queue `boost::heap::fibonacci_heap`, EPA/GJK heuristic ×10), with three deliberate, measured differences ("Profil retenu" in `docs/paper-deltas.md`), which make the search deterministic (identical across heap states):
- the open set orders by f rounded to 1 nm, then creation order (the old code compared raw f, so last-bit noise decided ties);
- two nodes are "the same" when surface, stance foot, yaw bin match and their patches are within `node_similarity_threshold` (2 cm) of each other (largest vertex-to-boundary distance, both ways), found through a spatial index; the old code compared `int(centroid / 2cm)` and `int(perimeter / 2cm)`, which splits patches a few mm apart across a cell boundary;
- the expansion it calls uses the corrected 2D clip and convex, cleaned patches (see `core/expansion`).

Validated against the old plans on all 11 scenes: same path length, depths, stance feet and surfaces (yaw ties are free), QP succeeding wherever the old QP did; `tests/golden_all`.

## Dependencies

`core/expansion`, `core/node`, `core/reachability`, `core/surface`, Boost (heap, functional/hash).

## Testing standalone

Golden-comparison tests use fixtures from [`tests/fixtures`](../../tests/fixtures) — that module must be added *after* this one in any consuming `CMakeLists.txt` (circular dependency otherwise, see `tests/fixtures/CMakeLists.txt`).

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2 && ctest
```
