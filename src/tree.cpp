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

namespace nas {

// Tree Constructor
Tree::Tree() {
    std::cout << "\n=== Initialization code ===\n" << std::endl;

    // TODO(jiayu): Change to antecedant polytopes
    std::cout << "[ Loading Polytopes ]" << std::endl;
    load_obj(rf_in_lf_path, this->rf_in_lf_polytope);
    load_obj(lf_in_rf_path, this->lf_in_rf_polytope);

    // Creat the Surfaces List
    std::cout << "\n[ Generate Environment Inforamtion (Get Surface Parameters) ]" << std::endl;
    for (int i = 0; i < surf_list.size(); ++i) {
        this->surfaces.push_back(Surface(surf_list[i], i));
    }

    // Initialize parameters
    std::cout << "\n[ Initializing Parameters ]" << std::endl;
    this->node_counter = 0;
    this->num_steps = total_num_steps;
    this->goal_stance_foot = stance_foot_at_goal;
    this->goal_location = surfaces.back().centroid + goal_offset;  // Add offset vector to centroid point
    std::cout << "  - Node counter: " << this->node_counter << std::endl;
    std::cout << "  - Total number of steps: " << this->num_steps << std::endl;
    std::cout << "  - Goal Stance Foot: " << 
        (goal_stance_foot == 0 ? "LEFT FOOT (0)" : 
         goal_stance_foot == 1 ? "RIGHT FOOT (1)" : "INVALID") << std::endl;
    std::cout << "  - Goal Location (World Frame): " << this->goal_location << std::endl;

    // Initialize the root node
    Node*  root_ptr = new Node();
    root_ptr->parent_ptr = nullptr;
    root_ptr->node_id = this->node_counter++;
    root_ptr->patch_vertices = std::vector<Point_3>({this->goal_location});  // Already Point_3, no conversion needed
    root_ptr->stance_foot = this->goal_stance_foot;
    root_ptr->surface_id = this->surfaces.back().surface_id;
    root_ptr->depth = 0;
    std::cout << "\n[ Root Node (Goal) Information ]" << std::endl;
    std::cout << "  - Node ID: " << root_ptr->node_id << std::endl;
    std::cout << "  - Stance Foot: " << 
        (root_ptr->stance_foot == 0 ? "LEFT FOOT (0)" : 
         root_ptr->stance_foot == 1 ? "RIGHT FOOT (1)" : "INVALID") << std::endl;
    std::cout << "  - Goal Location (World Frame): " << root_ptr->patch_vertices[0] << std::endl;

    // Initialize first layer with root
    this->layers.push_back({root_ptr});
    this->expansion_queue.push(root_ptr);
    
    std::cout << "\n=== Tree Generation Module Initialized ===\n" << std::endl;
}

// Expand the tree to target depth
void Tree::expand(int target_depth) {
    std::cout << "=== Expand the Tree (Depth = " << target_depth << ") ==="  << std::endl;
    
    while (!this->expansion_queue.empty() && this->layers.size() - 1 < static_cast<size_t>(target_depth)) { // -1 because we start from layer 0
        std::cout << "[ Expanding Layer " << this->layers.size() - 1 << " ]" << std::endl;
        std::cout << "  - Nodes to expand: " << this->expansion_queue.size() << std::endl;
        
        // Process all nodes at the current depth
        std::vector<Node*> new_layer;
        size_t nodes_at_current_depth = this->expansion_queue.size();

        for (size_t i = 0; i < nodes_at_current_depth; ++i) {
            Node* current_node = this->expansion_queue.front();
            this->expansion_queue.pop();

            // Get children for current node
            auto children = get_children(current_node);
            
            // Add children to the queue and layers
            if (!children.empty()) {
                this->layers.push_back(children);
                for (auto child : children) {
                    this->expansion_queue.push(child);
                }
                std::cout << "  - New nodes created: " << children.size() << std::endl;
            }
        }
    }

    std::cout << "\n=== Tree Generation Finished ===\n" << std::endl;
}

std::vector<Node*> Tree::get_children(Node* parent) {

    std::vector<Node*> children;

    // Step 1: Compute minkowski sum based on the patch vertices and the base polytope
    // TODO: make sure we get the correct base polytop
    Polyhedron base_polytope = parent->stance_foot == 0 ? rf_in_lf_polytope : lf_in_rf_polytope;
    Polyhedron P_union = minkowski_sum(parent->patch_vertices, base_polytope);

    // Step 2: Loop over all surfaces
    for (const auto& surface : surfaces) {

        // Sub-Step 1: Compute intersection between P_union and current surface
        std::vector<Point_3> polytope_plane_intersect_pts_3d = compute_polytope_plane_intersection(surface.plane, P_union);
        
        // Sub-Step 2: Compute intersection between P_union and current surface (if we have the polytope and the plane has intersection)

        if (polytope_plane_intersect_pts_3d.size() > 2) {

            //convert 3d intersection points to 2d surface plane
            std::vector<Point_2> polytope_plane_intersect_pts_2d = transform_3d_points_to_surface_plane(polytope_plane_intersect_pts_3d, surface.transform);

            //compute intersection between 2d intersection polygon (subject polygon) and the surface polygon (clipping polygon)
            std::vector<Point_2> polygon_2d_intersect_pts = compute_2d_polygon_intersection(polytope_plane_intersect_pts_2d, surface.vertices_2d);
            
            // Sub-Step 3: Convert the 2D intersection polygon to 3D (using the inverse transformation), only if we have polygon intersection result
            //             Also create the child node
            if (polygon_2d_intersect_pts.size() > 2) {
            
                Polygon_2 polygon_2d_intersect_result;
                CGAL::convex_hull_2(polygon_2d_intersect_pts.begin(), polygon_2d_intersect_pts.end(), std::back_inserter(polygon_2d_intersect_result));
                std::vector<Point_3> polytope_surf_3d_intersect_pts = transform_2d_points_to_world(polygon_2d_intersect_pts, surface.transform_inverse);
                Polyhedron polytope_surf_3d_intersect_polygon;        // Create intersection polygon (just for visualization)
                CGAL::convex_hull_3(polytope_surf_3d_intersect_pts.begin(), polytope_surf_3d_intersect_pts.end(), polytope_surf_3d_intersect_polygon);
                
                // Visualization
                auto renderWindow = Visualizer::create_figure("3D Polytope-Surface Intersection Visualization"); 
                auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();
                Visualizer::add_polyhedron(renderer, surface.polyhedron_3d, (double[]){0.7, 0.9, 1.0}, 0.3);  // Add Surface (light blue)
                Visualizer::add_polyhedron(renderer, P_union, (double[]){1.0, 0.7, 0.8}, 0.5);  // Add P_union (pink)
                Visualizer::add_polyhedron(renderer, polytope_surf_3d_intersect_polygon, (double[]){0.0, 1.0, 0.0}, 0.7);  // Add intersection polygon (green)
                Visualizer::add_points(renderer, polytope_surf_3d_intersect_pts, (double[]){1.0, 0.0, 0.0}, 0.05);  // Add intersection points (red)
                Visualizer::show(renderWindow);        // Show the 3D visualization

                // Found intersection, create child node
                Node* child = new Node();
                child->parent_ptr = parent;
                child->node_id = node_counter++;
                child->patch_vertices = polytope_surf_3d_intersect_pts;
                child->stance_foot = parent->stance_foot == 0 ?  1 : 0; //Alternate stance foot
                child->surface_id = surface.surface_id;
                child->depth = parent->depth + 1;
                children.push_back(child);
            }
        }
    }

    return children;
}

} // namespace nas  