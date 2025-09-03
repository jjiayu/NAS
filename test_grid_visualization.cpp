#include "astar_grid_search.hpp"
#include "visualizer.hpp"
#include "constants.hpp"
#include <iostream>

int main() {
    std::cout << "Testing Grid Visualization" << std::endl;
    
    // Create AstarGridSearch instance to initialize grid environment
    nas::AstarGridSearch grid_search;
    
    // Access the grid environment and surfaces for visualization
    const nas::GridEnvironment* grid_env = grid_search.get_grid_environment();
    const std::vector<nas::Surface>& surfaces = grid_search.surfaces;
    
    std::cout << "Grid dimensions: " << grid_env->get_width() 
              << "x" << grid_env->get_height() << std::endl;
    std::cout << "Number of surfaces: " << surfaces.size() << std::endl;
    
    // Show grid with surfaces
    nas::Visualizer::show_grid_with_surfaces(*grid_env, surfaces);
    
    return 0;
}
