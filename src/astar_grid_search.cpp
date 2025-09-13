#include "astar_grid_search.hpp"
#include "constants.hpp"
#include "geometry.hpp"
#include <chrono>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <cmath>
#include "tree.hpp"
#include "types.hpp"
#include "visualizer.hpp"
#include "utils.hpp"
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/polygon_mesh_io.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <vtkRendererCollection.h>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <iomanip>

namespace nas {

    AstarGridSearch::AstarGridSearch() {

    std::cout << "Initializing A* search algorithm" << std::endl;

    // Initialize polytopes
    load_obj(rf_in_lf_path_forward, this->rf_in_lf_polytope);
    load_obj(lf_in_rf_path_forward, this->lf_in_rf_polytope);

    // Creat the Surfaces List
    std::cout << "\n[ Generate Environment Inforamtion (Get Surface Parameters) ]" << std::endl;
    for (int i = 0; i < surf_list.size(); ++i) {
        this->surfaces.push_back(Surface(surf_list[i], i));
    }

    // Initialize Grid Environment
    std::cout << "\n[ Initialize Grid Environment ]" << std::endl;
    this->grid_env.initialize_from_surfaces(this->surfaces);

    // Initialize parameters
    std::cout << "\n[ Initializing Parameters ]" << std::endl;
    this->node_counter = 0;
    this->goal_stance_foot = stance_foot_at_goal;
    this->goal_location = surfaces.back().centroid + goal_offset;  // Add offset vector to centroid point
    std::cout << "  - Node counter: " << this->node_counter << std::endl;
    std::cout << "  - Goal Stance Foot: " << 
        (goal_stance_foot == 0 ? "LEFT FOOT (0)" : 
         goal_stance_foot == 1 ? "RIGHT FOOT (1)" : "INVALID") << std::endl;
    std::cout << "  - Goal Location (World Frame): " << this->goal_location << std::endl;

    this->distance_metric = a_star_distance_metric;
    std::cout << "  - Distance Metric: " << this->distance_metric << std::endl;

    this->cycle_detection_flag = cycle_detection;
    std::cout << "  - Cycle Detection: " << (this->cycle_detection_flag ? "ON" : "OFF") << std::endl;

    // Initialize the start node
    Node* start_node = new Node();
    start_node->parent_ptrs = std::vector<Node*>();  // Empty vector for root node
    start_node->node_id = this->node_counter++;
    start_node->patch_vertices = std::vector<Point_3>({current_foot_pos});  // Already Point_3, no conversion needed
    start_node->stance_foot = current_stance_foot_flag;
    start_node->centroid = current_foot_pos;
    start_node->depth = 0;
    if (foot_yaw_rotation_flag) {
        start_node->foot_yaw = current_foot_yaw;
    }
    start_node->perimeter = 0.0;
    start_node->g_score = 0;
    start_node->h_score = compute_euclidean_distance(current_foot_pos, this->goal_location);
    start_node->f_score = start_node->g_score + start_node->h_score;
    start_node->parent = nullptr;

    // TODO: Do we need surface id for the start node?

    // Initialize the open set
    this->node_handles[start_node] = this->open_set.push(start_node);

    // Cache half-space constraints for reachability polytopes
    this->rf_in_lf_constraint = convert_polytope_to_half_space_constraint(this->rf_in_lf_polytope);
    this->lf_in_rf_constraint = convert_polytope_to_half_space_constraint(this->lf_in_rf_polytope);
    
    // Plot grid environment after initialization
    std::cout << "\n[ Plotting Grid Environment ]" << std::endl;
    // this->plot_grid_environment();

}

void AstarGridSearch::search() {
    std::cout << "\n[ Start Grid-based A* search ]" << std::endl;

    // Timer start
    auto start_time = std::chrono::high_resolution_clock::now();

    // Main loop
    while (!open_set.empty()) {
        // Get the node with the lowest f_score
        Node* current_node = open_set.top();
        open_set.pop();
        this->expansion_count++;
        
        // Debug prints and visualization disabled
        
        // Remove from handles map since we're processing it
        node_handles.erase(current_node);

        // Check if we reached the goal (grid-based goal checking)
        if (current_node->stance_foot == goal_stance_foot) {
            // Convert goal location to grid coordinates
            auto goal_grid_coords = grid_env.world_to_grid(this->goal_location);
            auto current_grid_coords = grid_env.world_to_grid(current_node->centroid);
            
            if (goal_grid_coords.first == current_grid_coords.first && 
                goal_grid_coords.second == current_grid_coords.second) {
                
                // Reached goal - timer end
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
                std::cout << "Grid A-star Time taken: " << std::fixed << std::setprecision(3) << duration.count()/1000.0 << " milliseconds (ms)" << std::endl;
                std::cout << "Total node expansion count: " << this->expansion_count << std::endl;
                
                // Reconstruct the path
                std::cout << "Goal reached!" << std::endl;
                Node* current = current_node;
                while (current != nullptr) {
                    this->result_path.push_back(current);
                    current = current->parent;
                }
                std::cout << "Path Found with " << this->result_path.size() << " steps" << std::endl;
                std::reverse(this->result_path.begin(), this->result_path.end());
                for (Node* node : this->result_path) {
                    std::cout << "Node ID: " << node->node_id << ", Grid: (" 
                              << grid_env.world_to_grid(node->centroid).first << "," 
                              << grid_env.world_to_grid(node->centroid).second << "), Stance Foot: " 
                              << node->stance_foot;
                    if (foot_yaw_rotation_flag) {
                        std::cout << ", Foot Yaw: " << std::fixed << std::setprecision(3) 
                                  << node->foot_yaw << " rad (" << (node->foot_yaw * 180.0 / M_PI) << "°)";
                    }
                    std::cout << std::endl;
                }
                return;
            }
        }

        // Add the current node to the closed set
        this->closed_set.insert(current_node);

        // Expand the current node (grid-based)
        std::vector<Node*> children = get_grid_children(current_node);
        
        // Loop over all the children
        for (Node* child : children) {
            // Check if the child is already in the closed set
            if (closed_set.find(child) != closed_set.end()) {
                delete child;
                continue;
            }
            
            // Calculate the tentative g_score for this child using mid-pose interpolation
            
            // Calculate mid-pose between current and child positions
            Point_3 current_pos = current_node->centroid;
            Point_3 child_pos = child->centroid;
            Point_3 mid_pos((CGAL::to_double(current_pos.x()) + CGAL::to_double(child_pos.x())) / 2.0,
                           (CGAL::to_double(current_pos.y()) + CGAL::to_double(child_pos.y())) / 2.0,
                           (CGAL::to_double(current_pos.z()) + CGAL::to_double(child_pos.z())) / 2.0);
            
            // Calculate mid-orientation interpolation
            double current_yaw = current_node->foot_yaw;
            double child_yaw = child->foot_yaw;
            double mid_yaw = current_yaw;
            
            if (foot_yaw_rotation_flag) {
                // Handle angle wrapping for interpolation
                double yaw_diff = child_yaw - current_yaw;
                if (yaw_diff > M_PI) {
                    yaw_diff -= 2 * M_PI;
                } else if (yaw_diff < -M_PI) {
                    yaw_diff += 2 * M_PI;
                }
                mid_yaw = current_yaw + yaw_diff / 2.0;
            }
            
            // Calculate distance cost using mid-pose to mid-pose
            // We need to calculate the previous mid-pose from parent to current
            Point_3 prev_mid_pos = current_pos; // Default to current position if no parent
            if (current_node->parent != nullptr) {
                Point_3 parent_pos = current_node->parent->centroid;
                prev_mid_pos = Point_3((CGAL::to_double(parent_pos.x()) + CGAL::to_double(current_pos.x())) / 2.0,
                                      (CGAL::to_double(parent_pos.y()) + CGAL::to_double(current_pos.y())) / 2.0,
                                      (CGAL::to_double(parent_pos.z()) + CGAL::to_double(current_pos.z())) / 2.0);
            }
            
            double total_distance_cost = compute_euclidean_distance(prev_mid_pos, mid_pos);
            
            // Add yaw angle penalty based on mid-pose yaw change
            double yaw_penalty = 0.0;
            if (foot_yaw_rotation_flag) {
                // Calculate previous mid-yaw
                double prev_mid_yaw = current_yaw; // Default to current yaw if no parent
                if (current_node->parent != nullptr) {
                    double parent_yaw = current_node->parent->foot_yaw;
                    double yaw_diff_parent = current_yaw - parent_yaw;
                    if (yaw_diff_parent > M_PI) {
                        yaw_diff_parent -= 2 * M_PI;
                    } else if (yaw_diff_parent < -M_PI) {
                        yaw_diff_parent += 2 * M_PI;
                    }
                    prev_mid_yaw = parent_yaw + yaw_diff_parent / 2.0;
                }
                
                // Calculate yaw difference between previous mid-yaw and current mid-yaw
                double yaw_diff = std::abs(mid_yaw - prev_mid_yaw);
                // Normalize yaw difference to [0, π]
                if (yaw_diff > M_PI) {
                    yaw_diff = 2 * M_PI - yaw_diff;
                }
                // Apply yaw penalty (weight can be tuned)
                double yaw_weight = 0.1;  // Adjust this weight as needed
                yaw_penalty = yaw_weight * yaw_diff;
            }
            
            // double tentative_g_score = current_node->g_score + total_distance_cost + yaw_penalty;
            double tentative_g_score = current_node->g_score + 1.0;
            double tentative_h_score = compute_euclidean_distance(mid_pos, this->goal_location);
            double tentative_f_score = tentative_g_score + 10.0*tentative_h_score;
            
            // Check if this node is already in the open set
            auto handle_it = node_handles.find(child);
            if (handle_it == node_handles.end()) {
                // New node - add to open set
                child->g_score = tentative_g_score;
                child->h_score = tentative_h_score;
                child->f_score = tentative_f_score;
                child->parent = current_node;
                
                // Add to open set and store handle
                node_handles[child] = open_set.push(child);
            } else {
                // Node exists in open set - check if this path is better
                Node* existing_node = handle_it->first;
                OpenSet::handle_type existing_handle = handle_it->second;
                
                if (tentative_g_score < existing_node->g_score) {
                    // Better path found - update the existing node
                    existing_node->g_score = tentative_g_score;
                    existing_node->h_score = tentative_h_score;
                    existing_node->f_score = tentative_f_score;
                    existing_node->parent = current_node;
                    
                    // Update the heap (decrease-key operation)
                    open_set.increase(existing_handle);
                }
                
                // Delete child node as it is not needed anymore
                delete child;
            }
        }
    }

    auto failure_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time);
    std::cout << "If fail, then the computation time is: " << failure_time.count() / 1000.0 << " ms" << std::endl;
    std::cout << "\n[ Grid A* search completed - No path found ]" << std::endl;
    std::cout << "Total nodes expanded: " << this->expansion_count << std::endl;
    std::cout << "Total nodes in closed set: " << this->closed_set.size() << std::endl;
    std::cout << "Total nodes in open set: " << this->open_set.size() << std::endl;
}

std::vector<Node*> AstarGridSearch::get_grid_children(Node* parent) {
    std::vector<Node*> children;
    
    // Get the appropriate cached reachability constraint based on current stance foot
    const HalfSpacePolytopeConstraint& reachability_constraint = parent->stance_foot == LEFT_FOOT ? this->rf_in_lf_constraint : this->lf_in_rf_constraint;
    
    // Get current grid position
    auto parent_grid_coords = grid_env.world_to_grid(parent->centroid);
    int parent_grid_x = parent_grid_coords.first;
    int parent_grid_y = parent_grid_coords.second;
    
    // Define search radius in grid cells (reasonable footstep reach)
    int search_radius = static_cast<int>(std::ceil(1.5 / grid_env.get_cell_size())); // 3 meter radius
    
    // Iterate through potential grid cells within reachability
    for (int dy = -search_radius; dy <= search_radius; ++dy) {
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
            int target_grid_x = parent_grid_x + dx;
            int target_grid_y = parent_grid_y + dy;
            
            // 1. Skip current position (cheapest check)
            if (dx == 0 && dy == 0) continue;
            
            // 2. Check grid validity and traversability (cheap grid lookup)
            if (!grid_env.is_valid_cell(target_grid_x, target_grid_y) || 
                !grid_env.is_traversable(target_grid_x, target_grid_y)) {
                continue;
            }
            
            // Get target grid cell center in world coordinates
            Point_3 target_world_pos = grid_env.grid_to_world(target_grid_x, target_grid_y);
            const auto& target_cell = grid_env.get_cell(target_grid_x, target_grid_y);
            target_world_pos = Point_3(target_world_pos.x(), target_world_pos.y(), target_cell.height);
            
            // Check if target position is reachable using polytope
            // Transform target position relative to parent position
            Point_3 relative_pos = Point_3(
                target_world_pos.x() - parent->centroid.x(),
                target_world_pos.y() - parent->centroid.y(),
                target_world_pos.z() - parent->centroid.z()
            );
            
            // If foot yaw rotation is enabled, rotate the relative position by the inverse of parent's yaw
            // This accounts for the fact that the reachability polytope should be oriented according to the parent's foot yaw
            if (foot_yaw_rotation_flag && parent->foot_yaw != 0.0) {
                // Rotate relative position by negative parent yaw (inverse rotation)
                double cos_yaw = std::cos(-parent->foot_yaw);
                double sin_yaw = std::sin(-parent->foot_yaw);
                
                double rotated_x = cos_yaw * relative_pos.x() - sin_yaw * relative_pos.y();
                double rotated_y = sin_yaw * relative_pos.x() + cos_yaw * relative_pos.y();
                
                relative_pos = Point_3(rotated_x, rotated_y, relative_pos.z());
            }
            
            // Check if relative position is inside reachability polytope
            if (is_point_in_reachability_polytope(relative_pos, reachability_constraint)) {
                // Create child nodes with different foot yaw angles if rotation is enabled
                std::vector<double> yaw_angles;
                if (foot_yaw_rotation_flag) {
                    // Generate discretized yaw angles relative to parent's foot yaw
                    for (int i = -foot_yaw_angle_discretization_num; i <= foot_yaw_angle_discretization_num; ++i) {
                        yaw_angles.push_back(parent->foot_yaw + i * foot_yaw_angle_increment);
                    }
                } else {
                    // No rotation, use zero yaw angle
                    yaw_angles.push_back(0.0);
                }
                
                // Create a child node for each yaw angle
                for (double yaw_angle : yaw_angles) {
                    Node* child = new Node();
                    child->parent_ptrs.push_back(parent);
                    child->node_id = node_counter++;
                    child->patch_vertices = std::vector<Point_3>({target_world_pos});
                    child->stance_foot = parent->stance_foot == LEFT_FOOT ? RIGHT_FOOT : LEFT_FOOT; // Alternate stance foot
                    child->surface_id = target_cell.surface_id;
                    child->depth = parent->depth + 1;
                    child->centroid = target_world_pos;
                    // Normalize yaw angle to [-π, π] range
                    double normalized_yaw = yaw_angle;
                    while (normalized_yaw > M_PI) normalized_yaw -= 2.0 * M_PI;
                    while (normalized_yaw < -M_PI) normalized_yaw += 2.0 * M_PI;
                    child->foot_yaw = normalized_yaw; // Set the normalized foot yaw angle for this child
                    child->perimeter = 0.0; // Not used in grid-based search
                    
                    // Initialize scores for A* search
                    child->g_score = std::numeric_limits<double>::infinity();
                    child->h_score = 0.0;
                    child->f_score = std::numeric_limits<double>::infinity();
                    child->parent = nullptr; // Will be set during search
                    
                    children.push_back(child);
                }
            }
        }
    }
    
    return children;
}

bool AstarGridSearch::is_point_in_reachability_polytope(const Point_3& relative_pos, const HalfSpacePolytopeConstraint& constraint) {
    // Convert Point_3 to Eigen vector
    Eigen::Vector3d point;
    point << CGAL::to_double(relative_pos.x()), 
             CGAL::to_double(relative_pos.y()), 
             CGAL::to_double(relative_pos.z());
    
    // Check if point satisfies all half-space constraints: A*x <= b
    Eigen::VectorXd result = constraint.A * point;
    
    // Point is inside if all constraints are satisfied (with small tolerance)
    const double tolerance = 1e-6;
    for (int i = 0; i < result.size(); ++i) {
        if (result(i) > constraint.b(i) + tolerance) {
            return false;  // Point violates constraint i
        }
    }
    return true;  // Point satisfies all constraints
}

void AstarGridSearch::plot_grid_environment(){
    std::cout << "Creating grid environment visualization..." << std::endl;
    
    // Create a new window for grid visualization
    auto renderWindow = Visualizer::create_figure("Grid Environment Visualization");
    auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();

    // Add coordinate axes
    Visualizer::add_coordinate_axes(renderer);

    // Add surfaces with transparency
    for (const auto& surface : this->surfaces) {
        double surface_color[3] = {0.7, 0.9, 1.0};  // Light blue
        Visualizer::add_polyhedron(renderer, surface.polyhedron_3d, surface_color, 0.4);
    }

    // Collect all grid points (both traversable and non-traversable)
    std::vector<Point_3> traversable_points;
    std::vector<Point_3> non_traversable_points;
    
    for (int y = 0; y < grid_env.get_height(); ++y) {
        for (int x = 0; x < grid_env.get_width(); ++x) {
            if (grid_env.is_traversable(x, y)) {
                const auto& cell = grid_env.get_cell(x, y);
                traversable_points.push_back(cell.world_position);
            } else {
                // Add non-traversable points at z=0 plane
                Point_3 world_pos = grid_env.grid_to_world(x, y);
                Point_3 non_traversable_pos(world_pos.x(), world_pos.y(), 0.0);  // Set z=0
                non_traversable_points.push_back(non_traversable_pos);
            }
        }
    }

    // Add traversable grid points (green)
    if (!traversable_points.empty()) {
        double traversable_color[3] = {0.0, 1.0, 0.0};  // Green
        Visualizer::add_points(renderer, traversable_points, traversable_color, 0.02);
        std::cout << "Added " << traversable_points.size() << " traversable grid points" << std::endl;
    }

    // Add ALL non-traversable grid points (red, at z=0 plane)
    if (!non_traversable_points.empty()) {
        double non_traversable_color[3] = {1.0, 0.0, 0.0};  // Red
        Visualizer::add_points(renderer, non_traversable_points, non_traversable_color, 0.015);
        std::cout << "Added " << non_traversable_points.size() << " non-traversable grid points at z=0" << std::endl;
    }

    // Add A* path footstep positions if available
    if (!this->result_path.empty()) {
        std::vector<Point_3> left_foot_positions;
        std::vector<Point_3> right_foot_positions;
        
        for (const auto& node : this->result_path) {
            if (node->stance_foot == LEFT_FOOT) {
                left_foot_positions.push_back(node->centroid);
            } else {
                right_foot_positions.push_back(node->centroid);
            }
        }
        
        // Add left foot positions (blue)
        if (!left_foot_positions.empty()) {
            double left_foot_color[3] = {0.0, 0.0, 1.0};  // Blue
            Visualizer::add_points(renderer, left_foot_positions, left_foot_color, 0.06);
            std::cout << "Added " << left_foot_positions.size() << " left foot positions (blue)" << std::endl;
        }
        
        // Add right foot positions (red)
        if (!right_foot_positions.empty()) {
            double right_foot_color[3] = {1.0, 0.0, 0.0};  // Red
            Visualizer::add_points(renderer, right_foot_positions, right_foot_color, 0.06);
            std::cout << "Added " << right_foot_positions.size() << " right foot positions (red)" << std::endl;
        }
        
        std::cout << "A* Path: " << this->result_path.size() << " footsteps visualized" << std::endl;
    }

    // Add start and goal positions
    double start_color[3] = {1.0, 1.0, 0.0};  // Yellow
    double goal_color[3] = {1.0, 0.0, 1.0};   // Magenta
    Visualizer::add_points(renderer, {current_foot_pos}, start_color, 0.08);
    Visualizer::add_points(renderer, {this->goal_location}, goal_color, 0.08);

    std::cout << "Grid Environment: " << grid_env.get_width() << "x" << grid_env.get_height() 
              << " cells, cell_size=" << grid_env.get_cell_size() << std::endl;

    // Show the window
    Visualizer::show(renderWindow);
}

void AstarGridSearch::plot_path(){
    // Plotting path
    if (!this->result_path.empty()) {
        std::cout << "A* Path found with " << this->result_path.size() << " nodes (including the root)." << std::endl;

        // Create a new window for the path
        auto renderWindow = Visualizer::create_figure("A* Path");
        auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();

        // Add coordinate axes
        Visualizer::add_coordinate_axes(renderer);

        // Add surfaces (if you want to show the environment)
        for (const auto& surface : this->surfaces) {
            double surface_color[3] = {0.7, 0.9, 1.0};  // Light blue
            Visualizer::add_polyhedron(renderer, surface.polyhedron_3d, surface_color, 0.3);
        }

        // Optionally, add start and goal positions
        double start_color[3] = {1.0, 0.0, 0.0};  // Red
        double goal_color[3] = {0.0, 1.0, 0.0};   // Green
        Visualizer::add_points(renderer, {current_foot_pos}, start_color, 0.1);
        Visualizer::add_points(renderer, {this->goal_location}, goal_color, 0.1);

        // Add patches along the path
        for (const auto& node : this->result_path) {
            double patch_color[3];
            if (node->stance_foot == LEFT_FOOT) {
                patch_color[0] = 1.0; patch_color[1] = 0.0; patch_color[2] = 0.0; // Red
            } else {
                patch_color[0] = 0.0; patch_color[1] = 0.0; patch_color[2] = 1.0; // Blue
            }
            Visualizer::add_polyhedron(renderer, node->patch_polyhedron_3d, patch_color, 0.5);
        }

        // Show the window and wait for it to be closed
        Visualizer::show(renderWindow);
    } else {
        std::cout << "No path found by A* search." << std::endl;
    }
}

} // namespace nas