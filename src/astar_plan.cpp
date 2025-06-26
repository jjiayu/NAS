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
#include "astar_search.hpp"
#include "footstep_planner.hpp"

int main() {
    using namespace nas;

    // Create A* search instance
    AstarSearch astar_search;

    // Create FootstepPlanner instance
    FootstepPlanner footstep_planner;

    // Search for the path
    astar_search.search();

    // Plot the path
    astar_search.plot_path();

    // Plan the footstep
    footstep_planner.plan();
    
    return 0;
}