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
    load_obj(rf_in_lf_path_astar, this->rf_in_lf_polytope);
    load_obj(lf_in_rf_path_astar, this->lf_in_rf_polytope);

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


    // Initialize the start node
    Node* start_node = new Node();
    start_node->parent_ptrs = std::vector<Node*>();  // Empty vector for root node
    start_node->node_id = this->node_counter++;
    start_node->patch_vertices = std::vector<Point_3>({current_foot_pos});  // Already Point_3, no conversion needed
    start_node->stance_foot = current_stance_foot_flag;
    start_node->g_score = 0;
    start_node->h_score = compute_euclidean_distance(current_foot_pos, this->goal_location);
    start_node->f_score = 0;
    start_node->parent = nullptr;

    // Initialize the open set
    this->open_set.push(start_node);

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

        // Check if we reached the goal
        if (current_node->stance_foot == goal_stance_foot && 
            current_node->check_if_node_contains_point(this->goal_location)) {
            
            //reached goaltimer end
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            std::cout << "A-star Time taken: " << std::fixed << std::setprecision(3) << duration.count()/1000.0 << " milliseconds (ms)" << std::endl;

            //reconstruct the path
            std::cout << "Goal reached!" << std::endl;
            std::vector<Node*> path;
            Node* current = current_node;
            while (current != nullptr) {
                path.push_back(current);
                current = current->parent;
            }
            std::cout << "Path Found:  "<< std::endl;
            std::reverse(path.begin(), path.end());
            for (Node* node : path) {
                std::cout << "Node ID: " << node->node_id << ", Surface ID: " << node->surface_id << std::endl;
            }
            break;
        }

        // Add the current node to the closed set
        // TODO: this part need more care to compare if two patches are the same
        this->closed_set.insert(current_node);

        // Expand the current node
        std::vector<Node*> children = get_children(current_node);
        
        // Loop over all the children
        for (Node* child : children) {
            // Check if the child is already in the closed set
            if (closed_set.find(child) != closed_set.end()) {   
                continue;
            }
            else{
                // Add the child to the open set and update the g_score, h_score, and f_score
                child->g_score = current_node->g_score + compute_euclidean_distance(get_centroid(current_node->patch_vertices), get_centroid(child->patch_vertices));
                child->h_score = compute_euclidean_distance(get_centroid(child->patch_vertices), this->goal_location);
                child->f_score = child->g_score + child->h_score;
                child->parent = current_node;
                open_set.push(child);
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
    Polyhedron base_polytope = parent->stance_foot == 0 ? this->rf_in_lf_polytope : this->lf_in_rf_polytope;
    Polyhedron P_union = minkowski_sum(parent->patch_vertices, base_polytope);

    // Step 2: Loop over all surfaces
    for (const auto& surface : surfaces) {

        // Sub-Step 1: Compute intersection between P_union and current surface
        std::vector<Point_3> polytope_plane_intersect_pts_3d = compute_polytope_plane_intersection(surface.plane, P_union);
        
        // Sub-Step 2: Compute intersection between polygons (if we have the polytope and the plane has intersection)

        if (polytope_plane_intersect_pts_3d.size() > 2) {

            //convert 3d intersection points to 2d surface plane
            std::vector<Point_2> polytope_plane_intersect_pts_2d = transform_3d_points_to_surface_plane(polytope_plane_intersect_pts_3d, surface.transform_to_surface);

            // Convert polytope plane intersection points into convex hull
            Polygon_2 polytope_plane_intersect_convex_hull;
            CGAL::convex_hull_2(polytope_plane_intersect_pts_2d.begin(), polytope_plane_intersect_pts_2d.end(), std::back_inserter(polytope_plane_intersect_convex_hull));
            std::vector<Point_2> polytope_plane_intersect_convex_hull_pts;
            for (auto it = polytope_plane_intersect_convex_hull.vertices_begin(); it != polytope_plane_intersect_convex_hull.vertices_end(); ++it) {
                polytope_plane_intersect_convex_hull_pts.push_back(*it);
            }

            //compute intersection between 2d intersection polygon (subject polygon) and the surface polygon (clipping polygon)
            std::vector<Point_2> polygon_2d_intersect_pts = compute_2d_polygon_intersection(polytope_plane_intersect_convex_hull_pts, surface.vertices_2d);
            
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
                children.push_back(child);
            }
        }
    }
    return children;
}


} // namespace nas