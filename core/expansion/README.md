# core/expansion

`expand_node()` — the single expansion step shared by NAS/Tree and CASSR/AstarSearch, extracted from the ~90%-duplicated `Tree::get_children`/`AstarSearch::get_children` in the old code. Computes the Minkowski sum of the parent patch with the appropriate reachability polytope, intersects it with every surface, and creates one child `Node` per non-empty intersection (more if rotation is enabled).

## API

See [`include/nas/core/expansion.hpp`](include/nas/core/expansion.hpp).
- `expand_node(parent, surfaces, reachability, direction, params, pool)`: `direction` selects Forward (CASSR, start->goal) or Antecedent (NAS, goal->start) reachability polytopes.
- `ExpansionParams`: `rotation_enabled` (real parameter since NAS eventually wants it too, even though only CASSR enables it today), `yaw_discretization_num`/`yaw_angle_increment` (fan-out is `2n+1` candidate yaws when enabled), `cycle_detection_enabled`.

Scope decided 2026-09-18: 2 effectors only, no `GaitSequencer` (alternation is just `other_foot(parent->stance_foot)`).

Perf note (8d-4): the reachability polytope is queried by reference and only copied when `rotation_enabled` actually needs to rotate it — a `Polyhedron` copy rebuilds a CGAL halfedge structure, not free.

## Dependencies

`core/node`, `core/reachability`, `core/surface`, `core/geometry`, CGAL.

## Testing standalone

Tests run against the real forward-direction [`talosReachability`](../../talosReachability) assets (rotation fan-out, foot alternation, cycle detection).

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2 && ctest
```
