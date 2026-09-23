# planners/grid_astar_search

Discretized grid baseline (see PLAN.md phase 13: low priority, direct port of the old code's algorithm). Same overall shape as [`planners/astar_search`](../astar_search) (fibonacci-heap open set, hash-deduplicated closed/open sets, one child per candidate yaw), but the search space is a uniform 2D grid instead of continuous patches: no Minkowski sum, no polygon clipping, just "is this grid cell's center inside the Forward reachability polytope's H-representation?".

## API

- [`include/nas/planners/grid_environment.hpp`](include/nas/planners/grid_environment.hpp): `GridEnvironment` — rasterizes `Surface`s into a grid; each traversable cell caches its surface id and exact height (from the surface's own plane equation, not just its centroid Z).
- [`include/nas/planners/grid_astar_search.hpp`](include/nas/planners/grid_astar_search.hpp): `GridAstarSearch(surfaces, reachability, GridAstarSearchConfig)`, `search()`, `result_path()`, `expansion_count()`. Config: start/goal, `cell_size` (0.05m default), `search_radius_m` (1.5m half-width), `heuristic_weight` (10.0), `GridExpansionParams` (rotation on/off, yaw discretization).

Reuses the rewrite's shared pieces instead of the old code's hand-rolled equivalents: `ReachabilityModel` (Forward direction) instead of loading two polytope files in the constructor, `NodePool` instead of manual `new`/`delete`.

## Known limit (by design, not a bug)

A grid can't represent anything narrower than a cell. `NarrowPassage`'s middle surface is 0.24m wide, shrunk by the foot width to ~2cm — no 5cm cell center lands inside it, so the grid search has no way across and exhausts its reachable set with no path (see `test_narrow_passage_is_not_crossable_at_five_cm_cells`). The continuous `AstarSearch` crosses it in 29 steps. That contrast is what this baseline is for.

## Memory note

`NodePool` never frees individual nodes, so `get_grid_children` asks a `should_skip` predicate about a stack-allocated probe *before* allocating anything in the pool (already closed, or already open with an equal/better g-score). The naive port — allocate every candidate, discard duplicates afterwards, as the old code did with `new`/`delete` — exhausted RAM on the `NarrowPassage` case above (the whole reachable set gets explored, ~3700 candidate cells per expansion). With the predicate the same case peaks at ~25 MB.

## Dependencies

`core/node`, `core/reachability`, `core/surface`, `core/geometry`, CGAL, Boost (heap, functional/hash). `config/` is a test-only dependency (`config::load_scenario`).

## Testing standalone

```sh
mkdir build && cd build && cmake .. && cmake --build . -j1 && ctest
```
