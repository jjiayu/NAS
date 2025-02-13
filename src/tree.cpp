#include "tree.hpp"
#include "constants.hpp"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/polygon_mesh_io.h>
#include <fstream>
#include "utils.hpp"

typedef CGAL::Simple_cartesian<double> Kernel; // Using Simple Geometry Kernel
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
namespace nas {

Tree::Tree() {

    // Initialize the node counter
    node_counter = 0;

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

    // Create the Environment
    env_model = convert_surfaces_to_planes(surf_list);

    // Set up Goal
    std::cout << "\nSetting up the Goal" << std::endl;
    // Set up stance foot at goal
    int goal_stance_foot = goal_stance_foot;
    if (goal_stance_foot == 0) {
        std::cout << "Goal stance foot: LEFT FOOT" << std::endl;
    } else if (goal_stance_foot == 1) {
        std::cout << "Goal stance foot: RIGHT FOOT" << std::endl;
    } else {
        std::cout << "Invalid stance foot value" << std::endl; // Optional: handle unexpected values
    }
    // Set the Goal location of the goal stance foot
    goal_location = get_centroid(env_model.back()) + goal_offset;
    std::cout << "Goal location: " << goal_location << std::endl;

    // Initialize the root node
    Node* root_ptr = new Node();
    root_ptr->parent_ptr = nullptr;    
    root_ptr->node_id = node_counter++;

    // Initialize first layer with root
    layers.push_back({root_ptr});
    expansion_queue.push(root_ptr);
}

void Tree::expand_layer() {
    if (expansion_queue.empty()) {
        return;
    }

    std::vector<Node*> new_layer;
    size_t nodes_at_current_depth = expansion_queue.size();

    // Process all nodes at the current depth
    for (size_t i = 0; i < nodes_at_current_depth; ++i) {
        Node* current = expansion_queue.front();
        expansion_queue.pop();

        // Get children for current node
        auto children = get_children(current);
        
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

void Tree::expand_to_depth(int target_depth) {
    while (!expansion_queue.empty() && layers.size() < static_cast<size_t>(target_depth)) {
        expand_layer();
    }
}

std::vector<Node*> Tree::get_children(Node* parent) {
    std::vector<Node*> children;
    
    // Example: create two children for each node
    for (int i = 0; i < 2; ++i) {
        Node* child = new Node();
        child->parent_ptr = parent;
        child->node_id = node_counter++;
        
        // Set other node properties based on parent
        // For example:
        // child->stance_foot = (parent->stance_foot == 0) ? 1 : 0;  // Alternate feet
        // child->depth = parent->depth + 1;
        
        children.push_back(child);
    }
    
    return children;
}



} // namespace nas  

