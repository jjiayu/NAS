# core/node

`Node` — de-globalized, biped-only port shared by both search strategies (NAS/Tree and CASSR/AstarSearch), plus `NodePool`, its owner.

## API

See [`include/nas/core/node.hpp`](include/nas/core/node.hpp).
- `Node`: one concrete type shared by both planners — fields only meaningful to one strategy are commented as such rather than split into a type hierarchy (a flat struct was deliberately kept over inheritance, see `PLAN.md`'s "Node ownership" decision: inheritance would break `NodePool`'s value semantics via slicing and force `expand_node` to become type-templated).
- `NodePool`: owns every `Node` created for one search run via a `std::deque` (pointer-stable across insertions, unlike `std::vector`), hands out non-owning `Node*` valid for the pool's lifetime, assigns `node_id` sequentially. Nodes are never freed individually — only when the pool itself is destroyed.
- `cycle_path_detection()`: prevents re-visiting the same surface with the same foot after having left it once (CASSR paper Sec. IV).

## Dependencies

`core/geometry` (for `Point_3`/`Polygon_2`/etc. types), CGAL.

## Testing standalone

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2 && ctest
```
