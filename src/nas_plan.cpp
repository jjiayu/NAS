#include <iostream>
#include "nas_plan.hpp"
#include "node.hpp"
#include "visualizer.hpp"
#include "constants.hpp"
#include "tree.hpp"
#include "utils.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>

using namespace nas;

int main() {
    //Create the tree
    Tree tree;

    // Expand the tree (Depth = num_steps)
    tree.expand(tree.num_steps);
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

    // Find nodes containing initial stance foot
    std::vector<Node*> nodes = tree.find_nodes_containing_initial_stance_foot(LEFT_FOOT, Point_3(6.0, 0.5, 0.0));
    std::cout << "Nodes containing initial stance foot: ";
    for (const auto& node : nodes) {
        std::cout << node->node_id << " ";
    }
    std::cout << std::endl;

    return 0;
}