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
    std::cout << "=== Tree Layers ===" << std::endl;
    for (size_t i = 0; i < tree.layers.size(); ++i) {
        std::cout << "Layer " << i << " has " << tree.layers[i].size() 
                  << " nodes" << std::endl;
        
        // Print node IDs in this layer
        std::cout << "Node IDs: "<< std::endl;
        for (const auto& node : tree.layers[i]) {
            std::cout << node->node_id << " ";
            std::cout << "Stance Foot: " << node->stance_foot << std::endl;
        }
        std::cout << std::endl;
    }

    // Find nodes containing initial stance foot
    std::cout << "\n=== Finding nodes containing initial stance foot (Brute Force Method) ===" << std::endl;
    std::vector<Node*> nodes_brute_force_search = tree.find_nodes_containing_current_stance_foot_brute_force(current_stance_foot_flag, current_foot_pos);
    std::cout << "Nodes containing current stance foot: "<< std::endl;
    for (const auto& node : nodes_brute_force_search) {
        std::cout << "\nNode ID: " << node->node_id << std::endl;
        std::cout << "Node depth: " << node->depth << std::endl;
        std::cout << "Stance Foot: " << node->stance_foot << std::endl;
        std::cout << "Surface ID: " << node->surface_id << std::endl;
    }
    std::cout << std::endl;

    // Construct KD-trees for left and right foot
    std::cout << "\n=== Constructing KD-trees for left and right foot ===" << std::endl;
    tree.construct_kd_trees_for_left_and_right_foot();

    // Find nodes containing initial stance foot using KD-tree method and back track the paths
    std::cout << "\n=== Finding nodes containing initial stance foot (KD-tree Method) ===" << std::endl;
    std::vector<Node*> nodes_kd_tree_search = tree.find_nodes_containing_contact_location_kd_tree(current_stance_foot_flag, current_foot_pos);
    std::cout << "Nodes containing current stance foot: "<< std::endl;
    for (const auto& node : nodes_kd_tree_search) {
        std::cout << "\nNode ID: " << node->node_id << std::endl;
        std::cout << "Node depth: " << node->depth << std::endl;
        std::cout << "Stance Foot: " << node->stance_foot << std::endl;
        std::cout << "Surface ID: " << node->surface_id << std::endl;

        // Find all paths from this node to root
        std::vector<std::vector<Node*>> all_paths = tree.find_paths_to_root(node);
        std::cout << "Number of paths to root: " << all_paths.size() << std::endl;
        
        // Print each path
        for (size_t i = 0; i < all_paths.size(); ++i) {
            std::cout << "Path " << i + 1 << ": ";
            for (const auto& path_node : all_paths[i]) {
                std::cout << path_node->node_id << " -> ";
            }
            std::cout << "Root" << std::endl;
        }
    }

    return 0;
}