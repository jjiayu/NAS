#pragma once

#include "types.hpp"
#include "surface.hpp"
#include "geometry.hpp"
#include <vector>
#include <string>
#include "constants.hpp"
#include <iostream>
#include <queue>
#include "node.hpp"
#include <memory>

namespace nas {

class Tree {
public:

    // Reachability Polytope
    Polyhedron rf_in_lf_polytope;
    Polyhedron lf_in_rf_polytope;

    // Environment
    std::vector<Polyhedron> env_model;

    // Goal
    int goal_stance_foot;
    Point_3 goal_location;

    // Number of steps
    int num_steps;

    // Surfaces
    std::vector<Surface> surfaces;

    // Variables representing the Tree (Breadth First Search Expansion)
    std::vector<std::vector<Node*>> layers;  // Store nodes by layer
    std::queue<Node*> expansion_queue;       // Queue for BFS expansion

    // Number of node counter
    int node_counter;

    // KD-trees for spatial queries (one for each foot)
    std::unique_ptr<KD_Tree> left_foot_kd_tree; //kd tree for left foot nodes
    std::unique_ptr<KD_Tree> right_foot_kd_tree; //kd tree for right foot nodes
    Node* kd_tree_left_foot_root; //root of kd tree for left foot nodes
    Node* kd_tree_right_foot_root; //root of kd tree for right foot nodes

    //Constructor
    Tree();

    // Methods
    void expand(int depth);
    std::vector<Node*> get_children(Node* parent);

    //Brute force methods for searching the nodes
    std::vector<Node*> find_nodes_containing_current_stance_foot_brute_force(const bool foot_flag, const Point_3& foot_pos);
    
    // KD-tree methods
    void construct_kd_trees_for_left_and_right_foot(); //function to construct the KD-trees for the left and right foot
    Node* build_kd_tree_recursive(std::vector<Node*>& nodes, int depth); //recursive function to build the KD-tree recursively
    void traverse_kd_tree(Node* node, int depth, const Point_3& contact_location, std::vector<Node*>& result_nodes, size_t& min_depth); //function to traverse the KD-tree
    std::vector<Node*> find_nodes_containing_contact_location_kd_tree(const bool foot_flag, const Point_3& contact_location); //function to find the nodes containing the contact location using the KD-tree    

    // Check if two nodes are similar
    bool check_node_similarity(Node* node1, Node* node2);

    // Merge nodes
    std::vector<Node*> merge_nodes(std::vector<Node*> existing_nodes, std::vector<Node*> new_nodes);
    
    // Find all paths from a given node to the root
    std::vector<std::vector<Node*>> find_paths_to_root(Node* start_node);

    // Helper function for path finding
    void find_paths_recursive(Node* current_node, 
                            std::vector<Node*>& current_path,
                            std::vector<std::vector<Node*>>& all_paths);
};

} // namespace nas