#include "tree.hpp"
#include "types.hpp"
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/polygon_mesh_io.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <fstream>
#include "utils.hpp"
#include <algorithm>
#include "visualizer.hpp"

namespace nas {

// Tree Constructor
Tree::Tree() {

    // Initialize the node counter
    node_counter = 0;
    num_steps = total_num_steps;

    // Load the RF in LF polytope
    if (load_obj(rf_in_lf_path, rf_in_lf_polytope)) {
        std::cout << "Successfully loaded the rf in lf OBJ file." << std::endl;
        // std::cout << "Number of vertices: " << rf_in_lf_polytope.size_of_vertices() << std::endl;
        // std::cout << "Number of faces: " << rf_in_lf_polytope.size_of_facets() << std::endl;
    } else {
        std::cerr << "Failed to load the OBJ file." << std::endl;
    }

    // Load the LF in RF polytope
    if (load_obj(lf_in_rf_path, lf_in_rf_polytope)) {
        std::cout << "Successfully loaded the lf in rf OBJ file." << std::endl;
        // std::cout << "Number of vertices: " << lf_in_rf_polytope.size_of_vertices() << std::endl;
        // std::cout << "Number of faces: " << lf_in_rf_polytope.size_of_facets() << std::endl;
    } else {
        std::cerr << "Failed to load the OBJ file." << std::endl;
    }

    polytope_surf_intersection(surf_pts_list[0], lf_in_rf_polytope);
    
    // Create the Environment
    env_model = convert_surf_pts_to_polyhedron(surf_pts_list);

    // Set up Goal
    std::cout << std::endl;
    std::cout << "Setting up the Goal" << std::endl;
    // Set up stance foot at goal
    int goal_stance_foot = goal_stance_foot;
    // Set the Goal location of the goal stance foot
    goal_location = get_centroid(env_model.back()) + goal_offset;

    // Initialize the root node
    Node* root_ptr = new Node();
    root_ptr->parent_ptr = nullptr;    
    root_ptr->node_id = node_counter++;
    root_ptr->patch_vertices = std::vector<Vector_3>({goal_location});
    root_ptr->stance_foot = goal_stance_foot;

    // Print goal state (root node) information
    std::cout << "Goal State (Root node information):" << std::endl;
    if (root_ptr->stance_foot == 0) {
        std::cout << "Goal stance foot: LEFT FOOT" << std::endl;
    } else if (root_ptr->stance_foot == 1) {
        std::cout << "Goal stance foot: RIGHT FOOT" << std::endl;
    } else {
        std::cout << "Invalid stance foot value" << std::endl; // Optional: handle unexpected values
    }
    std::cout << "Goal Location: " << root_ptr->patch_vertices[0] << std::endl;

    // Initialize first layer with root
    layers.push_back({root_ptr});
    expansion_queue.push(root_ptr);
}

// Expand the tree from current depth
void Tree::expand_layer() {
    if (expansion_queue.empty()) {
        return;
    }

    std::vector<Node*> new_layer;
    size_t nodes_at_current_depth = expansion_queue.size();

    // Process all nodes at the current depth
    for (size_t i = 0; i < nodes_at_current_depth; ++i) {
        Node* current_node = expansion_queue.front();
        expansion_queue.pop();

        // Get children for current node
        auto children = get_children(current_node);
        
        // Add children to the new layer and queue
        for (auto child : children) {
            new_layer.push_back(child);
            expansion_queue.push(child);
        }
    }

    // Add new layer to layers if not empty
    if (!new_layer.empty()) {
        layers.push_back(new_layer);
    }
}

// Expand the tree to a target depth
void Tree::expand_to_depth(int target_depth) {
    while (!expansion_queue.empty() && layers.size() < static_cast<size_t>(target_depth)) {
        std::cout << "Expanding layer " << layers.size()-1 << std::endl;
        expand_layer();
    }
}

std::vector<Node*> Tree::get_children(Node* parent) {
    std::vector<Node*> children;

    // Step 1: Compute minkowski sum based on the patch vertices and the base polytope
    Polyhedron base_polytope = parent->stance_foot == 0 ? rf_in_lf_polytope : lf_in_rf_polytope;
    Polyhedron P_union = minkowski_sum(parent->patch_vertices, base_polytope);

    // Example: create two children for each node
    for (int i = 0; i < 2; ++i) {
        Node* child = new Node();
        child->parent_ptr = parent;
        child->node_id = node_counter++;
        child->patch_vertices = parent->patch_vertices; //ToDo: need to change
        child->stance_foot = (parent->stance_foot == 0) ? 1 : 0;  // Alternate feet
        
        children.push_back(child);
    }
    return children;
}

} // namespace nas  

