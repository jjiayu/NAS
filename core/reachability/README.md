# core/reachability

`ReachabilityModel` — replaces the hardcoded pairs of global polytope paths duplicated across the old `Tree`/`AstarSearch`/`FootstepPlanner`. Generalizes the old binary "stance_foot==0 ? A : B" lookup into a query keyed by `(moving_effector, support_effector, direction)`.

## API

See [`include/nas/core/reachability.hpp`](include/nas/core/reachability.hpp).
- `ReachabilityModel::load(entries)`: loads every `ReachabilityEntry{path, moving_effector, support_effector, direction}`'s `.obj` file. Throws on any read failure — fails the whole load rather than silently skipping an entry.
- `has()` / `query()`: lookup by key; `query()` throws `std::out_of_range` if the key was never loaded.

**Loading contract — "couche 0" only**: the caller supplies an explicit manifest, no filename-parsing or package-discovery convenience layer. Two reasons (see the header's own comment and `docs/paper-deltas.md`): it was explicitly descoped for now, and Talos's antecedent file naming turned out to be genuinely ambiguous (auto-inferring the key from a filename would have silently guessed wrong).

## Dependencies

CGAL (`polygon_mesh_io.h`).

## Testing standalone

Tests run against the real [`talosReachability`](../../talosReachability) assets, not just synthetic data.

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2 && ctest
```
