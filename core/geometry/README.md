# core/geometry

Pure geometric operations shared by every search strategy (NAS/Tree, CASSR/AstarSearch, the grid baseline) and by the footstep QP stage. No dependency on `Node`, on a search strategy, or on global configuration — every function takes exactly the data it needs as parameters.

## API

See [`include/nas/core/geometry.hpp`](include/nas/core/geometry.hpp). Main groups:
- 2D/3D transforms between world and surface-plane coordinates.
- Polytope/polygon geometry: Minkowski sum, plane-polytope intersection, 2D polygon clip (Sutherland-Hodgman — a CGAL-native replacement was tried and reverted, see `docs/paper-deltas.md`).
- Distance-to-patch: EPA (via [coal](https://github.com/coal-library/coal)) - the CASSR heuristic; the GJK/Euclidean variants of the old code were dropped.
- H-representation builders: `convert_polytope_to_half_space_constraint`, `generate_surface_constraint` (used by `footstep_qp`).
- `rotate_polyhedron_z`.

Error handling (see `docs/paper-deltas.md`, 8d-1): invalid caller input (empty polygon, <3-point patch) throws `std::invalid_argument`; a whole-surface degenerate plane fit throws `std::runtime_error`; a single degenerate facet on an otherwise-valid polytope is tolerated (skipped); coal itself failing on a numerically thin patch falls back to centroid distance.

## Dependencies

CGAL, Eigen, coal.

## Testing standalone

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2 && ctest
```
