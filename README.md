# NAS / CASSR

Footstep planning library implementing:
- **CASSR**: [Continuous A-Star Search through Reachability for real-time footstep planning](https://arxiv.org/abs/2603.02989) — the maintained planner, `nas::AstarSearch` (continuous patches) and `nas::GridAstarSearch` (discretized grid baseline used for comparison in the paper).
- **NAS**: [Navigating with A-Star through reachability](https://arxiv.org/abs/2407.12962) — the original implementation this was rewritten from; kept as reference/baseline in [`legacy/`](legacy/) (own build, own instructions there), not part of the maintained package.

See [`docs/architecture.md`](docs/architecture.md) for the module layout, and [`docs/paper-deltas.md`](docs/paper-deltas.md)/[`docs/paper-corrections-draft.md`](docs/paper-corrections-draft.md) for where the implementation departs from the papers.

## Prerequisites

- C++20 compiler (GCC 10+ or Clang 13+), CMake 3.15+
- Eigen3, CGAL, Boost, nlohmann-json — via system packages or conda
- coal, eiquadprog — via conda-forge only (not packaged for apt/Homebrew)

```bash
# Ubuntu
sudo apt install -y build-essential cmake libeigen3-dev libcgal-dev libboost-all-dev nlohmann-json3-dev
# macOS (Homebrew)
brew install cgal boost eigen nlohmann-json
# both: coal and eiquadprog are conda-forge only
conda install -c conda-forge coal eiquadprog
```

If using conda, activate the environment before building — the build picks up `$CONDA_PREFIX` automatically to find `coal`/`eiquadprog`.

## Build (C++)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4   # cap parallelism on a RAM-constrained machine, e.g. -j1
ctest --test-dir build    # nas_tests_fast (<1s) + nas_tests_golden (~2-3 min)
```

Produces the `nas` library (`libnas.a`), the `astar_plan` CLI, and the diagnostic/benchmark tools
under `tools/`. Build options (all CMake `-D...=ON/OFF`):

| Option | Default | Builds |
|---|---|---|
| `NAS_BUILD_APPS` | `ON` | `apps/astar_plan` (the CLI) |
| `NAS_BUILD_TOOLS` | `ON` | `tools/` (benchmarks, JSON dump/preview) |
| `NAS_BUILD_TESTS` | `ON` | `tests/` (`nas_tests`, registered with ctest) |
| `NAS_BUILD_BINDINGS` | `OFF` | `bindings/` (`nas_bindings`, needs Python + nanobind) |

### Using `nas` from another CMake project

```bash
cmake --install build --prefix /desired/install/path
```

then, in the consumer's `CMakeLists.txt`:

```cmake
find_package(nas REQUIRED)
target_link_libraries(your_target PRIVATE nas::nas)
```

## Build (Python bindings)

```bash
pip install --no-build-isolation -e .
python3 -c "import nas_bindings; print(nas_bindings.available_scenarios())"
```

`--no-build-isolation` is the tested path: it lets CMake see the active conda environment's
`coal`/`eiquadprog` directly, which a pip-managed isolated build environment would not have (those
two are conda-forge only, not on PyPI). `scikit-build-core` and `nanobind` (the Python build-time
requirements, from `pyproject.toml`) must already be installed in that environment:
`pip install scikit-build-core nanobind`.

```python
import nas_bindings

nas_bindings.available_scenarios()  # -> list[str], 15 built-in scenes
result = nas_bindings.plan(scenario_name, planner_config_path, talos_reachability_data_dir)
result.success        # bool
result.positions       # one (x, y, z) per footstep
```

See [`bindings/README.md`](bindings/README.md) for the full API.

## The `astar_plan` CLI

```bash
./build/apps/astar_plan/astar_plan <scenario_name> <planner_config.json> <talos_reachability_data_dir> <output.json>
```

`<scenario_name>` is one of the 15 built-in scenes (`Flat`, `NarrowPassage`, `Stairs`, ... — see
`docs/architecture.md` or `nas_bindings.available_scenarios()`); example configs are under
`apps/astar_plan/examples/`. `<talos_reachability_data_dir>` is normally
`talosReachability/data/reachability_constraints`. Writes a JSON result (path, footsteps, search/QP
timing) to `<output.json>` — see [`apps/astar_plan/README.md`](apps/astar_plan/README.md).

## Data

Talos reachability polytopes (`.obj` files) are a separate sibling package, [`talosReachability/`](talosReachability/) — its own `CMakeLists.txt`, its own `find_package`/Python import, see its README.
