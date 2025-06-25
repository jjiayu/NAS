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

using namespace nas;

int main() {

    // Create A* search instance
    AstarSearch astar_search;

    HalfSpacePolytopeConstraint constraints = convert_polytope_to_half_space_constraint(astar_search.rf_in_lf_polytope);
    std::cout << "Constraints: " << constraints.A << std::endl;
    std::cout << "Constraints: " << constraints.b << std::endl;

    // Search for the path
    astar_search.search();

    // Plot the path
    astar_search.plot_path();
    
    return 0;
}