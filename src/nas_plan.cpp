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
#include "footstep_planner.hpp"


int main() {

    using namespace nas;

    //Create the tree
    Tree tree;

    // Create FootstepPlanner instance
    FootstepPlanner footstep_planner;

    // Expand the tree (Depth = num_steps)
    tree.expand(tree.num_steps);

    // Find nodes containing initial stance foot
    std::vector<Node*> current_stance_foot_nodes;
    if (node_search_method == "bruteforce") {
        std::cout << "\n=== Finding nodes containing initial stance foot (Brute Force Method) ===" << std::endl;
        current_stance_foot_nodes = tree.find_nodes_containing_current_stance_foot_brute_force(current_stance_foot_flag, current_foot_pos);
    } else if (node_search_method == "kdtree") {
        std::cout << "\n=== Finding nodes containing initial stance foot (KD-tree Method) ===" << std::endl;
        std::cout << "\n=== Constructing KD-trees for left and right foot ===" << std::endl;
        tree.construct_kd_trees_for_left_and_right_foot();
        current_stance_foot_nodes = tree.find_nodes_containing_contact_location_kd_tree(current_stance_foot_flag, current_foot_pos);
    } else if (node_search_method == "knn") {
        std::cout << "\n=== Finding nodes containing initial stance foot (KNN Method) ===" << std::endl;
    }

    // Back track the paths (find the shortest paths)
    // Plot the shortest path nodes
    std::cout << "\n=== Printing Node search result (already filtered with the shortest path) ===" << std::endl;
    if (current_stance_foot_nodes.empty()){
        std::cout << "No shortest path nodes found." << std::endl;
    } else {
        for (const auto& node : current_stance_foot_nodes){
            std::cout << "\nNode ID: " << node->node_id << std::endl;
            std::cout << "Node depth (excluding the root): " << node->depth << std::endl;
            // Find all paths from this node to root
            std::vector<std::vector<Node*>> all_paths = tree.find_paths_to_root(node);
            std::cout << "Number of paths to root: " << all_paths.size() << std::endl;
            // Visualize each path in a separate window
            for (size_t i = 0; i < all_paths.size(); ++i) {
                std::cout << "Path " << i + 1 << ": ";

                // Plan the footstep positions
                std::cout << "\n=== Planning the footstep positions ===" << std::endl;
                footstep_planner.plan(current_stance_foot_flag, current_foot_pos, 
                          tree.goal_stance_foot, 
                          tree.goal_location,
                          all_paths[i]);
                
                // Create a new window for this path
                auto renderWindow = Visualizer::create_figure("Path " + std::to_string(i + 1) + " with Footsteps");
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
                
                // Add computed footsteps to the visualization
                std::cout << "\n=== Adding footsteps to visualization ===" << std::endl;
                const auto& computed_footsteps = footstep_planner.get_computed_footsteps();
                if (!computed_footsteps.empty()) {
                    // Add optimized footsteps (blue with transparency)
                    double footstep_color[3] = {0.0, 0.5, 1.0};  // Blue
                    Visualizer::add_footsteps(renderer, computed_footsteps, footstep_color);
                    
                    // Also add desired footstep positions for comparison (green with transparency)
                    std::vector<Point_3> desired_footsteps;
                    for (size_t j = 0; j < all_paths[i].size(); j++) {
                        if (j == 0) {
                            desired_footsteps.push_back(current_foot_pos);
                        } else if (j == all_paths[i].size() - 1) {
                            desired_footsteps.push_back(tree.goal_location);
                        } else {
                            desired_footsteps.push_back(all_paths[i][j]->centroid);
                        }
                    }
                    double desired_color[3] = {0.0, 1.0, 0.0};  // Green
                    Visualizer::add_footsteps(renderer, desired_footsteps, desired_color);
                    
                    std::cout << "Added " << computed_footsteps.size() << " optimized footsteps (blue)" << std::endl;
                    std::cout << "Added " << desired_footsteps.size() << " desired footsteps (green)" << std::endl;
                } else {
                    std::cout << "No computed footsteps available for visualization." << std::endl;
                }
                
                // Show this path's window and wait for it to be closed
                Visualizer::show(renderWindow);
            }
        }
    }

    return 0;
}