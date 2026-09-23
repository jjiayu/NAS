# planners/grid_astar_search

Discretized grid baseline (see PLAN.md phase 13: low priority, direct port of the old code's algorithm). Same overall shape as [`planners/astar_search`](../astar_search) (fibonacci-heap open set, hash-deduplicated closed/open sets, one child per candidate yaw), but the search space is a uniform 2D grid instead of continuous patches: no Minkowski sum, no polygon clipping, just "is this grid cell's center inside the Forward reachability polytope's H-representation?".

## API

- [`include/nas/planners/grid_environment.hpp`](include/nas/planners/grid_environment.hpp): `GridEnvironment` — rasterizes `Surface`s into a grid; each traversable cell caches its surface id and exact height (from the surface's own plane equation, not just its centroid Z).
- [`include/nas/planners/grid_astar_search.hpp`](include/nas/planners/grid_astar_search.hpp): `GridAstarSearch(surfaces, reachability, GridAstarSearchConfig)`, `search()`, `result_path()`, `expansion_count()`. Config: start/goal, `cell_size` (0.05m default), `search_radius_m` (1.5m half-width), `heuristic_weight` (10.0), `GridExpansionParams` (rotation on/off, yaw discretization).

Reuses the rewrite's shared pieces instead of the old code's hand-rolled equivalents: `ReachabilityModel` (Forward direction) instead of loading two polytope files in the constructor, `NodePool` instead of manual `new`/`delete`.

## Open question vs. the paper (2026-09-19)

On `NarrowPassage` at 5cm cells this port finds no path, and neither does the old `build/astar_grid_plan` binary (checked: no path within 2 minutes, same configuration). **The CASSR paper's discretised A\* does cross that scenario** (Table I, narrow passage, with rotation: 1456 ms, 1851 nodes, 41 steps, at "a granularity of 0.05m as in [7]"; without rotation it fails, like CASSR). So this baseline, as ported from the old repo, does **not** reproduce the paper's baseline on that scene.

Working hypothesis (not verified): the repo builds one *global* grid anchored on the scene's bounding box, so whether a cell center lands inside the ~2cm foot-shrunk passage depends on arbitrary grid phase (here the centers fall at y = -0.015 and +0.035, missing the strip [-0.01, 0.01]). The paper says it discretises the reachable set K_e, which suggests candidate footsteps on a lattice anchored at the *current* foothold (start at y = 0 puts lattice points inside the strip). If so, the paper's variant is not sensitive to this and the repo's is.

The port stays faithful to the old code (both agree with each other), so the tests pin *current behaviour*, not the paper's. Resolving this means anchoring the lattice at the start foothold and re-running `NarrowPassage` with rotation on (heavier: the old code needed >2 min just to fail).

## Memory note

`NodePool` never frees individual nodes, so `get_grid_children` asks a `should_skip` predicate about a stack-allocated probe *before* allocating anything in the pool (already closed, or already open with an equal/better g-score). The naive port — allocate every candidate, discard duplicates afterwards, as the old code did with `new`/`delete` — exhausted RAM on the `NarrowPassage` case above (the whole reachable set gets explored, ~3700 candidate cells per expansion). With the predicate the same case peaks at ~25 MB.

## Dependencies

`core/node`, `core/reachability`, `core/surface`, `core/geometry`, CGAL, Boost (heap, functional/hash). `config/` is a test-only dependency (`config::load_scenario`).

## Testing standalone

```sh
mkdir build && cd build && cmake .. && cmake --build . -j1 && ctest
```
