#include <iostream>
#include "nas_plan.hpp"
#include "node.hpp"
#include "visualizer.hpp"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include "constants.hpp"
#include "tree.hpp"
#include "utils.hpp"

using namespace nas;

int main() {



    //Create the tree
    Tree tree;

    // Expand tree to depth 3
    std::cout << "Expanding tree to depth: " << tree.num_steps << std::endl;
    tree.expand_to_depth(tree.num_steps);
    std::cout << std::endl;
    
    // Print information about each layer
    for (size_t i = 0; i < tree.layers.size(); ++i) {
        std::cout << "Layer " << i << " has " << tree.layers[i].size() 
                  << " nodes" << std::endl;
        
        // Print node IDs in this layer
        std::cout << "Node IDs: ";
        for (const auto& node : tree.layers[i]) {
            std::cout << node->node_id << " ";
        }
        std::cout << std::endl;
    }

    for (const auto& plane : tree.env_model) {
        Visualizer::show_polyhedron(plane);
    }

    // // Visualize the polytopes
    // Visualizer::show_polyhedron(tree.rf_in_lf_polytope);
    // Visualizer::show_polyhedron(tree.lf_in_rf_polytope);

    std::cout << surf_pts_list.size() << std::endl;

    return 0;
}
