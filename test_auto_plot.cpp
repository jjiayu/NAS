#include "astar_grid_search.hpp"
#include <iostream>

int main() {
    std::cout << "Testing Automatic Grid Visualization" << std::endl;
    
    // Creating AstarGridSearch object will automatically plot the grid environment
    nas::AstarGridSearch grid_search;
    
    std::cout << "AstarGridSearch object created - grid visualization should be displayed!" << std::endl;
    
    return 0;
}
