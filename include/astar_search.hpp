#pragma once

#include "types.hpp"
#include "surface.hpp"
#include "geometry.hpp"
#include "constants.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <queue>
#include "node.hpp"
#include <memory>

namespace nas {

// Custom comparator for priority queue
struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const {
        // We want the node with lower f_score to have higher priority
        return a->f_score > b->f_score;
    }
};

class AstarSearch {
public:
    // Reachability Polytope
    Polyhedron rf_in_lf_polytope;
    Polyhedron lf_in_rf_polytope;

    // Environment
    std::vector<Polyhedron> env_model;

    // Number of steps
    int total_num_steps;

    // Goal
    int goal_stance_foot;
    Point_3 goal_location;

    // Surfaces
    std::vector<Surface> surfaces;

    // Number of node counter
    int node_counter;

    // Open Set
    std::priority_queue<Node*, std::vector<Node*>, CompareNodes> open_set;

    // Close set
    std::unordered_set<Node*> closed_set;

    // Constructor
    AstarSearch();

    // Main search method
    void search();

    // Get children method
    std::vector<Node*> get_children(Node* current_node);
};

} // namespace nas