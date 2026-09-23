# apps/stl_preview

Dumps an STL-imported scenario's surfaces to JSON, no search/QP involved (see PLAN.md phase 9d). Exists to let a human eyeball whether `config::load_scenario_from_stl()` grouped triangles into the right surfaces before trusting it in a real planning run — `astar_plan` isn't a substitute for this: a grouping bug (a face split into two surfaces, or two faces merged into one) wouldn't necessarily make a search fail loudly.

## Usage

```sh
./stl_preview <stl_path> <output.json>
```

Output: `{"name": ..., "surfaces": [{"surface_id", "centroid", "vertices_3d"}, ...]}`.

## Dependencies

`config/`, CGAL, nlohmann::json.

## Building standalone

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2
```
