#include <iostream>
#include "nas_plan.hpp"
#include "node.hpp"
#include "visualizer.hpp"
#include "constants.hpp"
#include "tree.hpp"
#include "utils.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <chrono>
#include <thread>
#include <vtkRendererCollection.h>

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
    
    // // Visualize all nodes returned by KD-tree search, each with a different color
    // auto renderWindow = nas::Visualizer::create_figure("KD-tree Search Result Nodes");
    // auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();
    // int color_idx = 0;
    // for (const auto& node : nodes_kd_tree_search) {
    //     // Generate a unique color for each patch (cycling through hues)
    //     double r = 0.5 + 0.5 * std::sin(color_idx * 2.0 * M_PI / nodes_kd_tree_search.size());
    //     double g = 0.5 + 0.5 * std::sin(color_idx * 2.0 * M_PI / nodes_kd_tree_search.size() + 2.0);
    //     double b = 0.5 + 0.5 * std::sin(color_idx * 2.0 * M_PI / nodes_kd_tree_search.size() + 4.0);
    //     double color[3] = {r, g, b};
    //     // Visualize the patch polyhedron (3D)
    //     nas::Visualizer::add_polyhedron(renderer, node->patch_polyhedron_3d, color, 0.5);
    //     // Visualize the patch vertices (as points, same color but more opaque)
    //     nas::Visualizer::add_points(renderer, node->patch_vertices, color, 0.10);
    //     color_idx++;
    // }
    // nas::Visualizer::show(renderWindow);

    // Back track the paths
    if (nodes_kd_tree_search.empty()) {
        std::cout << "No nodes found containing the current stance foot position." << std::endl;
    } else {
        // For each node containing current stance foot
        for (const auto& node : nodes_kd_tree_search) {
            std::cout << "\nNode ID: " << node->node_id << std::endl;
            std::cout << "Node depth: " << node->depth << std::endl;
            std::cout << "Stance Foot: " << node->stance_foot << std::endl;
            std::cout << "Surface ID: " << node->surface_id << std::endl;

            // Find all paths from this node to root
            std::vector<std::vector<Node*>> all_paths = tree.find_paths_to_root(node);
            std::cout << "Number of paths to root: " << all_paths.size() << std::endl;
            
            // Visualize each path in a separate window
            for (size_t i = 0; i < all_paths.size(); ++i) {
                std::cout << "Path " << i + 1 << ": ";
                
                // Create a new window for this path
                auto renderWindow = Visualizer::create_figure("Path " + std::to_string(i + 1));
                auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();

                // Add coordinate axes
                Visualizer::add_coordinate_axes(renderer);

                // Add surfaces
                for (const auto& surface : tree.surfaces) {
                    double surface_color[3] = {0.7, 0.9, 1.0};  // Light blue
                    Visualizer::add_polyhedron(renderer, surface.polyhedron_3d, surface_color, 0.3);
                }

                // Add current foot position
                double current_foot_color[3] = {1.0, 0.0, 0.0};  // Red
                Visualizer::add_points(renderer, {current_foot_pos}, current_foot_color, 0.1);

                // Add goal position
                double goal_color[3] = {0.0, 1.0, 0.0};  // Green
                Visualizer::add_points(renderer, {tree.goal_location}, goal_color, 0.1);
                
                // Add patches along the current path
                for (const auto& path_node : all_paths[i]) {
                    std::cout << path_node->node_id << " foot ID: " << path_node->stance_foot << " -> ";
                    
                    // Color patches based on stance foot
                    double patch_color[3];
                    if (path_node->stance_foot == LEFT_FOOT) {
                        patch_color[0] = 1.0;  // Red for left foot
                        patch_color[1] = 0.0;
                        patch_color[2] = 0.0;
                    } else {
                        patch_color[0] = 0.0;  // Blue for right foot
                        patch_color[1] = 0.0;
                        patch_color[2] = 1.0;
                    }
                    
                    Visualizer::add_polyhedron(renderer, path_node->patch_polyhedron_3d, patch_color, 0.5);
                }
                std::cout << std::endl;
                
                // Show this path's window and wait for it to be closed
                Visualizer::show(renderWindow);                
            }
        }
    }

    return 0;
}