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

    // KD-trees are automatically initialized to nullptr by std::unique_ptr

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
    Node* root_ptr = new Node();
    root_ptr->parent_ptrs = std::vector<Node*>();  // Empty vector for root node
    root_ptr->node_id = this->node_counter++;
    root_ptr->patch_vertices = std::vector<Point_3>({this->goal_location});  // Already Point_3, no conversion needed
    root_ptr->stance_foot = this->goal_stance_foot;
    root_ptr->surface_id = this->surfaces.back().surface_id;
    root_ptr->depth = 0;
    root_ptr->patch_polygon_2d = Polygon_2(); //NOTE: for now root node is just a point, so we initialize with an empty polygon
    root_ptr->patch_polyhedron_3d = Polyhedron(); //NOTE: for now root node is just a point, so we initialize with an empty polyhedron
    root_ptr->transformation_to_2d = this->surfaces.back().transform_to_surface; // Transformation to surface coordinate system
    root_ptr->transformation_to_3d = this->surfaces.back().transform_to_3d; // Transformation from surface coordinate system to world coordinate system
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
        int total_children = 0;

        for (size_t i = 0; i < nodes_at_current_depth; ++i) {
            Node* current_node = this->expansion_queue.front();
            this->expansion_queue.pop();

            // Get children for current node
            auto children = get_children(current_node);

            // Merge node if they are similar
            auto merged_children = merge_nodes(new_layer, children);
            
            // Add children to the new layer and queue
            if (!merged_children.empty()) {
                new_layer.insert(new_layer.end(), merged_children.begin(), merged_children.end());
                for (auto child : merged_children) {
                    this->expansion_queue.push(child);
                }
                total_children += merged_children.size();
            }
        }

        // Add all children from this depth level as a new layer
        if (!new_layer.empty()) {
            this->layers.push_back(new_layer);
            std::cout << "  - New nodes created: " << total_children << std::endl;
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
            std::vector<Point_2> polytope_plane_intersect_pts_2d = transform_3d_points_to_surface_plane(polytope_plane_intersect_pts_3d, surface.transform_to_surface);

            //compute intersection between 2d intersection polygon (subject polygon) and the surface polygon (clipping polygon)
            std::vector<Point_2> polygon_2d_intersect_pts = compute_2d_polygon_intersection(polytope_plane_intersect_pts_2d, surface.vertices_2d);
            
            // Sub-Step 3: Convert the 2D intersection polygon to 3D (using the inverse transformation), only if we have polygon intersection result
            //             Also create the child node
            if (polygon_2d_intersect_pts.size() > 2) {
            
                Polygon_2 polygon_2d_intersect_result;
                CGAL::convex_hull_2(polygon_2d_intersect_pts.begin(), polygon_2d_intersect_pts.end(), std::back_inserter(polygon_2d_intersect_result));
                std::vector<Point_3> polytope_surf_3d_intersect_pts = transform_2d_points_to_world(polygon_2d_intersect_pts, surface.transform_to_3d);
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

std::vector<Node*> Tree::find_nodes_containing_current_stance_foot_brute_force(const bool foot_flag, const Point_3& foot_pos) {
    std::vector<Node*> nodes;
    // Start from index 1 to skip the root layer (goal)
    for (size_t layer_idx = 1; layer_idx < this->layers.size(); ++layer_idx) {
        for (const auto& node : this->layers[layer_idx]) {
            if (node->stance_foot == foot_flag && node->check_if_node_contains_point(foot_pos)) {
                nodes.push_back(node);
            }
        }
    }
    return nodes;
}

// Build KD-trees for both left and right foot
void Tree::construct_kd_trees_for_left_and_right_foot() {

    // Collect all nodes for both left and right foot
    std::vector<Node*> all_left_foot_nodes;
    std::vector<Node*> all_right_foot_nodes;
    
    // Collect nodes for each foot, skipping layer 0 (goal)
    for (size_t layer_idx = 1; layer_idx < layers.size(); ++layer_idx) {
        for (Node* node : layers[layer_idx]) {
            if (node->stance_foot == LEFT_FOOT) {
                all_left_foot_nodes.push_back(node);
            } else {
                all_right_foot_nodes.push_back(node);
            }
        }
    }

    // Build left foot KD-tree
    if (!all_left_foot_nodes.empty()) {
        kd_tree_left_foot_root = build_kd_tree_recursive(all_left_foot_nodes, 0);
    } else {
        kd_tree_left_foot_root = nullptr;
    }

    // Build right foot KD-tree
    if (!all_right_foot_nodes.empty()) {
        kd_tree_right_foot_root = build_kd_tree_recursive(all_right_foot_nodes, 0);
    } else {
        kd_tree_right_foot_root = nullptr;
    }

    std::cout << "KD-trees built successfully." << std::endl;
    std::cout << "Left foot nodes: " << all_left_foot_nodes.size() << std::endl;
    std::cout << "Right foot nodes: " << all_right_foot_nodes.size() << std::endl;
}

// Helper function to build KD-tree recursively
Node* Tree::build_kd_tree_recursive(std::vector<Node*>& nodes, int depth) {
    if (nodes.empty()) return nullptr;

    // Determine splitting axis (x=0, y=1, z=2)
    int axis = depth % 3;

    // Sort nodes along current splitting axis
    std::sort(nodes.begin(), nodes.end(), 
        [axis](Node* a, Node* b) {
            Point_3 centroid_a = get_centroid(a->patch_vertices);
            Point_3 centroid_b = get_centroid(b->patch_vertices);
            double a_coord = axis == 0 ? centroid_a.x() : 
                           axis == 1 ? centroid_a.y() : 
                                      centroid_a.z();
            double b_coord = axis == 0 ? centroid_b.x() : 
                           axis == 1 ? centroid_b.y() : 
                                      centroid_b.z();
            return a_coord < b_coord;
        });

    // Find median node
    size_t median_idx = nodes.size() / 2;
    Node* median_node = nodes[median_idx];

    // Split nodes into left and right subtrees
    std::vector<Node*> left_nodes(nodes.begin(), nodes.begin() + median_idx);
    std::vector<Node*> right_nodes(nodes.begin() + median_idx + 1, nodes.end());

    // Set left and right pointers
    if (!left_nodes.empty()) {
        median_node->kd_left_ptr = build_kd_tree_recursive(left_nodes, depth + 1);
    }
    if (!right_nodes.empty()) {
        median_node->kd_right_ptr = build_kd_tree_recursive(right_nodes, depth + 1);
    }

    return median_node;
}

void Tree::traverse_kd_tree(Node* node, int depth, const Point_3& contact_location, 
                           std::vector<Node*>& result_nodes) {
    if (!node) return;

    // Get the node's centroid
    Point_3 centroid = get_centroid(node->patch_vertices);

    // Check if the contact location is within the node's patch
    if (node->check_if_node_contains_point(contact_location)) {
        result_nodes.push_back(node);
    }

    // Determine which axis to split on (x=0, y=1, z=2)
    int axis = depth % 3;

    // Compare query point with centroid along the current splitting axis
    double query_coord = axis == 0 ? contact_location.x() : 
                       axis == 1 ? contact_location.y() : 
                                  contact_location.z();
    double centroid_coord = axis == 0 ? centroid.x() : 
                          axis == 1 ? centroid.y() : 
                                     centroid.z();

    // Traverse left subtree if query point is less than or equal to centroid
    if (query_coord <= centroid_coord) {
        traverse_kd_tree(node->kd_left_ptr, depth + 1, contact_location, result_nodes);
    }
    
    // Traverse right subtree if query point is greater than or equal to centroid
    if (query_coord >= centroid_coord) {
        traverse_kd_tree(node->kd_right_ptr, depth + 1, contact_location, result_nodes);
    }
}

std::vector<Node*> Tree::find_nodes_containing_contact_location_kd_tree(const bool foot_flag, 
                                                              const Point_3& contact_location) {
    std::vector<Node*> result_nodes;
    
    // Get the root node for the appropriate foot
    Node* root = foot_flag == LEFT_FOOT ? kd_tree_left_foot_root : kd_tree_right_foot_root;
    
    if (!root) {
        return result_nodes;
    }

    traverse_kd_tree(root, 0, contact_location, result_nodes); //always start search from the root node
    return result_nodes;
}

bool Tree::check_node_similarity(Node* node1, Node* node2){

    // similarity flag
    bool same_node_flag = false;

    // Check the distance between the two centroids
    double centroid_distance = CGAL::squared_distance(get_centroid(node1->patch_vertices), get_centroid(node2->patch_vertices));

    // Compute Perimeter of the two nodes
    double perimeter1 = compute_polygon_perimeter(node1->patch_polyhedron_3d);
    double perimeter2 = compute_polygon_perimeter(node2->patch_polyhedron_3d);

    double perimeter_distance = fabs(perimeter1 - perimeter2);

    if ((centroid_distance < node_similarity_threshold) && (perimeter_distance < node_similarity_threshold)){
        same_node_flag = true;
    }

    return same_node_flag;
}

// Node Merging
std::vector<Node*> Tree::merge_nodes(std::vector<Node*> existing_nodes, std::vector<Node*> new_child_nodes){
    std::vector<Node*> filtered_child_nodes;

    // Check each new child node against existing nodes
    for (Node* new_child_node : new_child_nodes) {
        bool is_similar = false;
        
        for (Node* existing_node : existing_nodes) {
            if (check_node_similarity(existing_node, new_child_node)) {
                // Transfer parent pointers to the existing node
                existing_node->parent_ptrs.insert(existing_node->parent_ptrs.end(),
                                                  new_child_node->parent_ptrs.begin(),
                                                  new_child_node->parent_ptrs.end());
                is_similar = true;
                break;
            }
        }
        
        // Only add to filtered list if not similar to any existing node
        if (!is_similar) {
            filtered_child_nodes.push_back(new_child_node);
        }

    }

    return filtered_child_nodes;
}

std::vector<std::vector<Node*>> Tree::find_paths_to_root(Node* start_node) {
    std::vector<std::vector<Node*>> all_paths;
    std::vector<Node*> current_path;
    
    // Start recursive path finding
    find_paths_recursive(start_node, current_path, all_paths);
    
    return all_paths;
}

void Tree::find_paths_recursive(Node* current_node, 
                                std::vector<Node*>& current_path,
                                std::vector<std::vector<Node*>>& all_paths) {
    // Add current node to path
    current_path.push_back(current_node);
    
    // If we've reached the root (node with no parents)
    if (current_node->parent_ptrs.empty()) {
        // Add a copy of the current path to all_paths
        all_paths.push_back(current_path);
    } else {
        // For each parent, recursively find paths
        for (Node* parent : current_node->parent_ptrs) {
            find_paths_recursive(parent, current_path, all_paths);
        }
    }
    
    // Backtrack: remove current node from path
    current_path.pop_back();
}

} // namespace nas  