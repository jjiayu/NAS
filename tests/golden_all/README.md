# tests/golden_all

Two checks against the OLD code, covering every scene of `environments.hpp` (the dedicated golden tests only cover `NarrowPassage` and `ThreePathsNAS`).

## `nas_golden_all_scenes` (registered with ctest)
Replays each `tests/golden/*_astar.json` scene with the old `constants.hpp` configuration and compares the final path (depth, stance foot, yaw, surface id per node) and, where the old QP succeeded, the footsteps.

## `nas_expansion_differential <dump_dir>` (needs old-code dumps)
The strong check. `tests/old_expansion_dump.cpp` calls the OLD `AstarSearch::get_children` on real parent states (150 breadth-first expansions per scene, ~18 000 children) and dumps everything, including intermediate stages (rotated polytope, Minkowski sum, plane/polytope intersections). The test rebuilds each parent and pushes it through `expand_node`.

```sh
cmake --build build --target old_expansion_dump -j1     # old tree, conda env rwa
tests/golden_all/generate_old_dumps.sh /tmp/old_dumps
cd tests/golden_all && mkdir -p build && cd build && cmake .. && cmake --build . -j1 --target nas_expansion_differential
./nas_expansion_differential /tmp/old_dumps
```

Asserted: same number/order of children, surface, stance foot, depth, yaw, cycle history (exact), and the patch **polygon** (two-way Hausdorff of the hull boundaries + area, 1e-7). Reported only: centroid, perimeter, raw vertex list.

## Why centroid/perimeter/vertex lists are not asserted
They are not reproducible even by the old code: `compute_polygon_perimeter` sums every edge of a thin prism polyhedron (triangulation diagonals included) and the centroid averages raw clip vertices, and both depend on the order/triangulation `CGAL::convex_hull_3` returns for P_union. Running the old expansion twice with only its heap allocation order changed (`NAS_SCRAMBLE=<seed>` in `old_expansion_dump`) changes P_union's vertex order in 15/15 expansions and ~270 of ~300 children centroids (up to 12.6 cm). On the stair scenes it even changes patch polygons (0.3-1.8% of children, up to 0.9 m) — in those scenes the surface plane can coincide with a facet of P_union, and the edge-based plane/polytope intersection then depends on the triangulation. The differential test therefore asserts exact polygons on the scenes where the old code is stable and a bounded rate (5%) where it is not. See `docs/paper-deltas.md`.
