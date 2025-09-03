#include "tree.hpp"
#include "types.hpp"
#include "visualizer.hpp"
#include "utils.hpp"
#include "geometry.hpp"
#include "constants.hpp"
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>
#include "astar_grid_search.hpp"
#include <vtkRendererCollection.h>

using namespace nas;

int main() {

    std::cout << "=== A* Path Planning with Footstep Optimization (using |||Discretization|||) ===" << std::endl;
    // Create FootstepPlanner instance

    AstarGridSearch grid_search;
    // grid_search.search();
}