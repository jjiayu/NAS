// include/prob_data.hpp

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
    Vector_3 goal_location;

    // Number of steps
    int num_steps;

    // Surfaces
    std::vector<Surface> surfaces;

    // Variables representing the Tree (Breadth First Search Expansion)
    std::vector<std::vector<Node*>> layers;  // Store nodes by layer
    std::queue<Node*> expansion_queue;       // Queue for BFS expansion

    // Number of node counter
    int node_counter;

    //Constructor
    Tree();
    // Destructor
    // ~ProblemData();

    // Methods
    void expand(int depth);
    std::vector<Node*> get_children(Node* parent);
};

} // namespace nas