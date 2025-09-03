#pragma once

#include "types.hpp"
#include "surface.hpp"
#include "geometry.hpp"
#include "constants.hpp"
#include "grid_environment.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <queue>
#include "node.hpp"
#include <memory>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/functional/hash.hpp>

namespace nas {

class AstarGridSearch {
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

    // Grid Environment
    GridEnvironment grid_env;

    // Number of node counter
    int node_counter;

    // Path found
    std::vector<Node*> result_path = {};

    // Open Set - using Boost.Heap for efficient decrease-key operations
    // typedef boost::heap::fibonacci_heap<Node*, boost::heap::compare<CompareNodes>> OpenSet;
    // OpenSet open_set;

    // Track handles for each node in open set (needed for decrease-key)
    // std::unordered_map<Node*, OpenSet::handle_type, NodeHash, NodeEqual> node_handles;

    // Close set
    // std::unordered_set<Node*, NodeHash, NodeEqual> closed_set;

    // Computation time statistics
    double total_minkowski_time = 0.0;
    double total_clipping_time = 0.0;
    double total_plane_polytope_intersect_time = 0.0;
    double total_polygon_2d_intersect_time = 0.0;

    // A star realted
    std::string distance_metric;
    bool cycle_detection_flag;

    // Counters
    int expansion_coount = 0;

    // Constructor
    AstarGridSearch();

    // Main search method
    void search();

    // Get children method
    std::vector<Node*> get_children(Node* current_node);

    // Plot path method
    void plot_path();

    // Plot grid environment method
    void plot_grid_environment();

    // Getter for grid environment
    const GridEnvironment& get_grid_environment() const { return grid_env; }

};

} // namespace nas