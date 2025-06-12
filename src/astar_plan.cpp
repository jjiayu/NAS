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

    astar_search.search();
    
    return 0;
}