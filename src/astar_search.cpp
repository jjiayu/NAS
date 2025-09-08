#include "astar_search.hpp"
#include "tree.hpp"
#include "types.hpp"
#include "visualizer.hpp"
#include "utils.hpp"
#include "geometry.hpp"
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

AstarSearch::AstarSearch() {

    std::cout << "Initializing A* search algorithm" << std::endl;

    // Initialize polytopes
    load_obj(rf_in_lf_path_forward, this->rf_in_lf_polytope);
    load_obj(lf_in_rf_path_forward, this->lf_in_rf_polytope);

    // Creat the Surfaces List
    std::cout << "\n[ Generate Environment Inforamtion (Get Surface Parameters) ]" << std::endl;
    for (int i = 0; i < surf_list.size(); ++i) {
        this->surfaces.push_back(Surface(surf_list[i], i));
    }

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
    if (foot_yaw_rotation_flag) {
        start_node->foot_yaw = current_foot_yaw;
    }
    start_node->perimeter = 0.0;
    start_node->g_score = 0;
    start_node->h_score = compute_euclidean_distance(current_foot_pos, this->goal_location);
    start_node->f_score = start_node->g_score + start_node->h_score;
    start_node->parent = nullptr;

    // Initialize the open set
    this->node_handles[start_node] = this->open_set.push(start_node);
}

void AstarSearch::search() {
    std::cout << "\n[ Start A* search ]" << std::endl;

    //timer start
    auto start_time = std::chrono::high_resolution_clock::now();

    // Main loop
    while (!open_set.empty()) {
        // Get the node with the lowest f_score
        Node* current_node = open_set.top();
        open_set.pop();
        this->expansion_coount++;
        
        // Remove from handles map since we're processing it
        node_handles.erase(current_node);

        // Check if we reached the goal
        if (current_node->stance_foot == goal_stance_foot && 
            current_node->check_if_node_contains_point(this->goal_location)) {
            
            //reached goaltimer end
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            std::cout << "A-star Time taken: " << std::fixed << std::setprecision(3) << duration.count()/1000.0 << " milliseconds (ms)" << std::endl;
            std::cout << "Total node expansion count: " << this->expansion_coount << std::endl;
            std::cout << "Total minkowski time: " << this->total_minkowski_time << " ms" << std::endl;
            std::cout << "Total clipping time: " << this->total_clipping_time << " ms" << std::endl;
            std::cout << "Total plane-polytope intersection time: " << this->total_plane_polytope_intersect_time << " ms" << std::endl;
            std::cout << "Total polygon intersection time: " << this->total_polygon_2d_intersect_time << " ms" << std::endl;
            
            //reconstruct the path
            std::cout << "Goal reached!" << std::endl;
            Node* current = current_node;
            while (current != nullptr) {
                this->result_path.push_back(current);
                current = current->parent;
            }
            std::cout << "Path Found:  "<< std::endl;
            std::reverse(this->result_path.begin(), this->result_path.end());
            for (Node* node : this->result_path) {
                std::cout << "Node ID: " << node->node_id << ", Surface ID: " << node->surface_id << ", Stance Foot: " << node->stance_foot;
                if (foot_yaw_rotation_flag) {
                    std::cout << ", Foot Yaw: " << std::fixed << std::setprecision(3) 
                              << node->foot_yaw << " rad (" << (node->foot_yaw * 180.0 / M_PI) << "°)";
                }
                std::cout << std::endl;
            }
            break;
        }

        // Add the current node to the closed set
        this->closed_set.insert(current_node);

        // Expand the current node
        std::vector<Node*> children = get_children(current_node);
        
        // Loop over all the children
        for (Node* child : children) {
            // Check if the child is already in the closed set
            if (closed_set.find(child) != closed_set.end()) {
                continue;
            }
            
            // Calculate the tentative g_score for this child
            double tentative_g_score = current_node->g_score + compute_euclidean_distance(current_node->centroid, child->centroid);
            double tentative_h_score = 0.0;
            if (this->distance_metric == "gjk") {
                tentative_h_score = calculate_gjk_distance_point_to_patch(child->patch_vertices, this->goal_location);
            }
            else if (this->distance_metric == "euclidean") {
                tentative_h_score = compute_euclidean_distance(child->centroid, this->goal_location);
            }
            double tentative_f_score = tentative_g_score + tentative_h_score;
            
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

                    // delete child node as it is not needed anymore
                    delete child;
                }
            }
        }
    }

    std::cout << "\n[ A* search completed ]" << std::endl;
    std::cout << "Total nodes expanded: " << this->node_counter << std::endl;
    std::cout << "Total nodes in closed set: " << this->closed_set.size() << std::endl;
    std::cout << "Total nodes in open set: " << this->open_set.size() << std::endl;
}

std::vector<Node*> AstarSearch::get_children(Node* parent){

    std::vector<Node*> children;

    // Step 1: Compute minkowski sum based on the patch vertices and the base polytope
    // TODO: make sure we get the correct base polytop
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    Polyhedron base_polytope = parent->stance_foot == 0 ? this->rf_in_lf_polytope : this->lf_in_rf_polytope;
    
    // Rotate the polytope based on the parent's foot yaw angle if foot yaw rotation is enabled
    if (foot_yaw_rotation_flag == true) {
        base_polytope = rotate_polyhedron_z(base_polytope, parent->foot_yaw);
    }
    
    Polyhedron P_union = minkowski_sum(parent->patch_vertices, base_polytope);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    this->total_minkowski_time += duration.count() / 1000.0;

    // Step 2: Loop over all surfaces
    auto start_time_clipping = std::chrono::high_resolution_clock::now();
    for (const auto& surface : surfaces) {

        // Sub-Step 1: Compute intersection between P_union and current surface
        start_time = std::chrono::high_resolution_clock::now();
        std::vector<Point_3> polytope_plane_intersect_pts_3d = compute_polytope_plane_intersection(surface.plane, P_union);
        end_time = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        this->total_plane_polytope_intersect_time += duration.count() / 1000.0;
        
        // Sub-Step 2: Compute intersection between polygons (if we have the polytope and the plane has intersection)

        if (polytope_plane_intersect_pts_3d.size() > 2) {
            // Convert 3d intersection points to 2d surface plane
            std::vector<Point_2> polytope_plane_intersect_pts_2d = transform_3d_points_to_surface_plane(polytope_plane_intersect_pts_3d, surface.transform_to_surface);

            // Convert polytope plane intersection points into convex hull
            Polygon_2 polytope_plane_intersect_convex_hull;
            CGAL::convex_hull_2(polytope_plane_intersect_pts_2d.begin(), polytope_plane_intersect_pts_2d.end(), std::back_inserter(polytope_plane_intersect_convex_hull));
            std::vector<Point_2> polytope_plane_intersect_convex_hull_pts;
            for (auto it = polytope_plane_intersect_convex_hull.vertices_begin(); it != polytope_plane_intersect_convex_hull.vertices_end(); ++it) {
                polytope_plane_intersect_convex_hull_pts.push_back(*it);
            }

            //compute intersection between 2d intersection polygon (subject polygon) and the surface polygon (clipping polygon)
            start_time = std::chrono::high_resolution_clock::now();

            std::vector<Point_2> polygon_2d_intersect_pts = compute_2d_polygon_intersection(polytope_plane_intersect_convex_hull_pts, surface.vertices_2d);
            
            end_time = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            this->total_polygon_2d_intersect_time += duration.count() / 1000.0;

            // Sub-Step 3: Convert the 2D intersection polygon to 3D (using the inverse transformation), only if we have polygon intersection result
            //             Also create the child node
            if (polygon_2d_intersect_pts.size() > 2) {
            
                Polygon_2 polygon_2d_intersect_result;
                CGAL::convex_hull_2(polygon_2d_intersect_pts.begin(), polygon_2d_intersect_pts.end(), std::back_inserter(polygon_2d_intersect_result));
                std::vector<Point_3> polytope_surf_3d_intersect_pts = transform_2d_points_to_world(polygon_2d_intersect_pts, surface.transform_to_3d);
                Polyhedron polytope_surf_3d_intersect_polygon;        // Create intersection polygon (just for visualization)
                CGAL::convex_hull_3(polytope_surf_3d_intersect_pts.begin(), polytope_surf_3d_intersect_pts.end(), polytope_surf_3d_intersect_polygon);
                
                // // Visualization
                // auto renderWindow = Visualizer::create_figure("3D Polytope-Surface Intersection Visualization"); 
                // auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();
                // Visualizer::add_polyhedron(renderer, surface.polyhedron_3d, (double[]){0.7, 0.9, 1.0}, 0.3);  // Add Surface (light blue)
                // Visualizer::add_polyhedron(renderer, P_union, (double[]){1.0, 0.7, 0.8}, 0.5);  // Add P_union (pink)
                // Visualizer::add_polyhedron(renderer, polytope_surf_3d_intersect_polygon, (double[]){0.0, 1.0, 0.0}, 0.7);  // Add intersection polygon (green)
                // Visualizer::add_points(renderer, polytope_surf_3d_intersect_pts, (double[]){1.0, 0.0, 0.0}, 0.05);  // Add intersection points (red)
                // Visualizer::show(renderWindow);        // Show the 3D visualization
                
                // Found intersection, create child node
                // Filter children if it has been detected as a cycle
                int stance_foot = parent->stance_foot == 0 ? 1 : 0; // Alternate stance foot
                if ((this->cycle_detection_flag == true) && (cycle_path_detection(parent, stance_foot, surface.surface_id) == true)) {
                    continue; //the same surface visited 2 steps before
                }

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
                    child->patch_vertices = polytope_surf_3d_intersect_pts;
                    child->stance_foot = parent->stance_foot == 0 ?  1 : 0; //Alternate stance foot
                    child->surface_id = surface.surface_id;
                    child->depth = parent->depth + 1;
                    child->patch_polygon_2d = polygon_2d_intersect_result;
                    child->patch_polyhedron_3d = polytope_surf_3d_intersect_polygon;
                    child->transformation_to_2d = surface.transform_to_surface;
                    child->transformation_to_3d = surface.transform_to_3d;
                    child->perimeter = compute_polygon_perimeter(polytope_surf_3d_intersect_polygon);
                    child->centroid = get_centroid(polytope_surf_3d_intersect_pts);
                    child->foot_yaw = yaw_angle; // Set the foot yaw angle for this child
                    // Copy parent's pred_surface_ids and add parent's surface as new layer
                    child->pred_surface_ids = parent->pred_surface_ids;
                    child->pred_surface_ids[parent->stance_foot].push_back({parent->surface_id});
                    
                    // Initialize scores for A* search
                    child->g_score = std::numeric_limits<double>::infinity();
                    child->h_score = 0.0; // Will be computed when needed
                    child->f_score = std::numeric_limits<double>::infinity();
                    child->parent = nullptr; // Will be set during search
                    
                    children.push_back(child);
                }
            }
        }
    }
    auto end_time_clipping = std::chrono::high_resolution_clock::now();
    auto duration_clipping = std::chrono::duration_cast<std::chrono::microseconds>(end_time_clipping - start_time_clipping);
    this->total_clipping_time += duration_clipping.count() / 1000.0;

    return children;
}

void AstarSearch::plot_path(){
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