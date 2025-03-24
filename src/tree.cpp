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
#include <fstream>
#include <algorithm>

namespace nas {

// Tree Constructor
Tree::Tree() {
    std::cout << "\n=== Initialization code ===\n" << std::endl;

    // TODO(jiayu): Change to antecedant polytopes
    std::cout << "[ Loading Polytopes ]" << std::endl;
    load_obj(rf_in_lf_path, rf_in_lf_polytope);
    load_obj(lf_in_rf_path, lf_in_rf_polytope);

    // Creat the Surfaces List
    std::cout << "\n[ Generate Environment Inforamtion (Get Surface Parameters) ]" << std::endl;
    for (int i = 0; i < surf_list.size(); ++i) {
        surfaces.push_back(Surface(surf_list[i], i));
    }

    // Initialize parameters
    std::cout << "\n[ Initializing Parameters ]" << std::endl;
    node_counter = 0;
    num_steps = total_num_steps;
    goal_stance_foot = stance_foot_at_goal;
    goal_location = get_centroid(surf_list.back()) + goal_offset;
    std::cout << "  - Node counter: " << node_counter << std::endl;
    std::cout << "  - Total number of steps: " << num_steps << std::endl;
    std::cout << "  - Goal Stance Foot: " << 
        (goal_stance_foot == 0 ? "LEFT FOOT (0)" : 
         goal_stance_foot == 1 ? "RIGHT FOOT (1)" : "INVALID") << std::endl;
    std::cout << "  - Goal Location (World Frame): " << goal_location << std::endl;

    // Initialize the root node
    Node* root_ptr = new Node();
    root_ptr->parent_ptr = nullptr;
    root_ptr->node_id = node_counter++;
    root_ptr->patch_vertices = std::vector<Vector_3>({goal_location});
    root_ptr->stance_foot = goal_stance_foot;

    std::cout << "\n[ Root Node (Goal) Information ]" << std::endl;
    std::cout << "  - Node ID: " << root_ptr->node_id << std::endl;
    std::cout << "  - Stance Foot: " << 
        (root_ptr->stance_foot == 0 ? "LEFT FOOT (0)" : 
         root_ptr->stance_foot == 1 ? "RIGHT FOOT (1)" : "INVALID") << std::endl;
    std::cout << "  - Goal Location (World Frame): " << root_ptr->patch_vertices[0] << std::endl;

    // Initialize first layer with root
    layers.push_back({root_ptr});
    expansion_queue.push(root_ptr);
    
    std::cout << "\n=== Tree Generation Module Initialized ===\n" << std::endl;
}

// Expand the tree to target depth
void Tree::expand(int target_depth) {
    std::cout << "=== Expand the Tree (Depth = " << target_depth << ") ==="  << std::endl;
    
    while (!expansion_queue.empty() && layers.size() < static_cast<size_t>(target_depth)) {
        std::cout << "[ Expanding Layer " << layers.size()-1 << " ]" << std::endl;
        std::cout << "  - Nodes to process: " << expansion_queue.size() << std::endl;
        
        // Process all nodes at the current depth
        std::vector<Node*> new_layer;
        size_t nodes_at_current_depth = expansion_queue.size();

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
            std::cout << "  - New nodes created: " << new_layer.size() << std::endl;
        }
    }

    std::cout << "\n=== Initialization code finished ===\n" << std::endl;
}

std::vector<Node*> Tree::get_children(Node* parent) {
    std::vector<Node*> children;

    // Step 1: Compute minkowski sum based on the patch vertices and the base polytope
    // ToDO: make sure we get the correct base polytop
    Polyhedron base_polytope = parent->stance_foot == 0 ? rf_in_lf_polytope : lf_in_rf_polytope;
    Polyhedron P_union = minkowski_sum(parent->patch_vertices, base_polytope);

    // Example: create two children for each node
    for (int i = 0; i < 2; ++i) {
        Node* child = new Node();
        child->parent_ptr = parent;
        child->node_id = node_counter++;
        // TODO(jiayu): Update patch vertices computation
        child->patch_vertices = parent->patch_vertices;
        child->stance_foot = (parent->stance_foot == 0) ? 1 : 0;  // Alternate feet
        
        children.push_back(child);
    }
    return children;
}

} // namespace nas  