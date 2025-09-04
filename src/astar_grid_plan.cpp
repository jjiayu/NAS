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
    
    AstarGridSearch grid_search;
    grid_search.search();
    
    // Plot the grid environment with A* footstep results
    grid_search.plot_grid_environment();
    
    return 0;
}