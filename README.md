# NAS / CASSR

Implementation of:
- **NAS**: [Navigating with A-Star through reachability](https://arxiv.org/abs/2407.12962)
- **CASSR**: [Continuous A-Star Search through Reachability for real-time footstep planning](https://arxiv.org/abs/2603.02989)

## Prerequisites

- C++20 compiler (GCC 10+ or Clang 13+)
- CMake 3.15+

### System packages (Ubuntu 24.04)

```bash
sudo apt install -y \
  build-essential cmake pkg-config \
  libeigen3-dev libcgal-dev libboost-all-dev \
  libvtk9-dev libyaml-cpp-dev libfreetype6-dev \
  nlohmann-json3-dev
```

### Conda packages

CasADi and coal are easiest to install via conda:

```bash
conda install -c conda-forge casadi coal
```

### macOS (Homebrew)

```bash
brew install cgal boost eigen vtk yaml-cpp nlohmann-json freetype
# CasADi and coal via conda as above
```

## Build

```bash
cd NAS
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

If using conda, make sure `CONDA_PREFIX` is set (it is automatically when the environment is activated). The build system picks it up to find CasADi and coal.

## Executables

| Binary | Description |
|---|---|
| `astar_plan` | A\* search + QP footstep optimization (main planner) |
| `astar_grid_plan` | Grid-discretized A\* search |
| `nas_plan` | Tree-based exhaustive search + footstep planning |
| `test_grid_visualization` | Grid environment visualization |
| `test_auto_plot` | Auto-plot test |

Run from the build directory:

```bash
cd build
./astar_plan
./astar_grid_plan
./nas_plan
```

None of the executables take command-line arguments. All configuration is compile-time.

## Selecting a scenario

Edit `include/constants.hpp` and change the `surf_list` assignment to one of the predefined environments:

```cpp
// Available environments (defined in include/environments.hpp):
const std::vector<std::vector<Point_3>> surf_list = NarrowPassage;
// const std::vector<std::vector<Point_3>> surf_list = Flat;
// const std::vector<std::vector<Point_3>> surf_list = Stairs;
// const std::vector<std::vector<Point_3>> surf_list = LongStairs;
// const std::vector<std::vector<Point_3>> surf_list = LongLongStairs;
// const std::vector<std::vector<Point_3>> surf_list = LongStairsComplete;
// const std::vector<std::vector<Point_3>> surf_list = ThreePathsScene;
// const std::vector<std::vector<Point_3>> surf_list = ThreePathsNAS;
// const std::vector<std::vector<Point_3>> surf_list = Stairs_Up_Down;
// const std::vector<std::vector<Point_3>> surf_list = TwoFlatSurfaces;
```

You must also set the initial foot position to match the scenario:

```cpp
const Point_3 current_foot_pos(0.0, 0.0, 0.0); // for NarrowPassage, LongStairs, etc.
// const Point_3 current_foot_pos(0.1, 0.0, 0.0); // for Stairs
// const Point_3 current_foot_pos(2.2, 0.7, 0.0); // for TwoFlatSurfaces
```

Then rebuild and run:

```bash
cmake --build build -j$(nproc) && ./build/astar_plan
```

## Key parameters (constants.hpp)

| Parameter | Default | Description |
|---|---|---|
| `a_star_distance_metric` | `"epa"` | Distance metric: `"euclidean"`, `"gjk"`, `"epa"` |
| `a_star_grid_resolution` | `0.05` | Grid cell size in meters (for `astar_grid_plan`) |
| `total_num_steps` | `40` | Maximum number of planning steps |
| `foot_yaw_rotation_flag` | `true` | Enable foot yaw discretization |
| `foot_yaw_angle_increment` | `10°` | Yaw angle step |
| `current_stance_foot_flag` | `RIGHT_FOOT` | Initial stance foot |
| `com_z_height` | `0.75` | CoM height for footstep planning |
| `merge_node_flag` | `true` | Merge similar nodes in the search |
| `node_search_method` | `"bruteforce"` | Node search: `"bruteforce"`, `"kdtree"`, `"knn"` |

## Data files

Reachability constraint polytopes (`.obj` meshes) are in `data/constraints_files/`. The paths are set as absolute paths in `constants.hpp` — update them if your checkout directory differs.
