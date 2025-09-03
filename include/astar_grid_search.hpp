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
#include <Eigen/Dense>

namespace nas {

// Grid-specific hash function for nodes
struct GridNodeHash {
    size_t operator()(const Node* node) const {
        // For grid-based search, hash based on grid position and stance foot
        // Convert world position to grid coordinates
        // Assuming we have access to grid environment through a static reference or similar
        // For now, use quantized world coordinates as grid approximation
        
        // Quantize to grid cell resolution (0.05m default)
        const double grid_resolution = a_star_grid_resolution;
        int grid_x = static_cast<int>(std::round(node->centroid.x() / grid_resolution));
        int grid_y = static_cast<int>(std::round(node->centroid.y() / grid_resolution));
        int grid_z = static_cast<int>(std::round(node->centroid.z() / grid_resolution));
        
        // Use Boost's hash_combine for better distribution
        size_t seed = 0;
        boost::hash_combine(seed, grid_x);
        boost::hash_combine(seed, grid_y);
        boost::hash_combine(seed, grid_z);
        boost::hash_combine(seed, node->stance_foot);  // Critical for grid-based search
        boost::hash_combine(seed, node->surface_id);   // Include surface for completeness
        
        return seed;
    }
};

// Grid-specific equality comparison for nodes
struct GridNodeEqual {
    bool operator()(const Node* a, const Node* b) const {
        // For grid-based search, compare grid position and stance foot
        const double grid_resolution = a_star_grid_resolution;
        
        int grid_x_a = static_cast<int>(std::round(a->centroid.x() / grid_resolution));
        int grid_y_a = static_cast<int>(std::round(a->centroid.y() / grid_resolution));
        int grid_z_a = static_cast<int>(std::round(a->centroid.z() / grid_resolution));
        
        int grid_x_b = static_cast<int>(std::round(b->centroid.x() / grid_resolution));
        int grid_y_b = static_cast<int>(std::round(b->centroid.y() / grid_resolution));
        int grid_z_b = static_cast<int>(std::round(b->centroid.z() / grid_resolution));
        
        // Compare grid coordinates, stance foot, and surface
        return (grid_x_a == grid_x_b && 
                grid_y_a == grid_y_b && 
                grid_z_a == grid_z_b && 
                a->stance_foot == b->stance_foot &&
                a->surface_id == b->surface_id);
    }
};

// Custom comparator for priority queue (same as in astar_search.hpp)
struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const {
        // We want the node with lower f_score to have higher priority
        return a->f_score > b->f_score;
    }
};

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
    typedef boost::heap::fibonacci_heap<Node*, boost::heap::compare<CompareNodes>> OpenSet;
    OpenSet open_set;

    // Track handles for each node in open set (needed for decrease-key)
    std::unordered_map<Node*, OpenSet::handle_type, GridNodeHash, GridNodeEqual> node_handles;

    // Close set
    std::unordered_set<Node*, GridNodeHash, GridNodeEqual> closed_set;

    // Computation time statistics
    double total_minkowski_time = 0.0;
    double total_clipping_time = 0.0;
    double total_plane_polytope_intersect_time = 0.0;
    double total_polygon_2d_intersect_time = 0.0;

    // A star realted
    std::string distance_metric;
    bool cycle_detection_flag;
    
    // Cached half-space constraints for reachability polytopes
    HalfSpacePolytopeConstraint rf_in_lf_constraint;
    HalfSpacePolytopeConstraint lf_in_rf_constraint;

    // Counters
    int expansion_count = 0;

    // Constructor
    AstarGridSearch();

    // Main search method
    void search();

    // Get children method
    std::vector<Node*> get_children(Node* current_node);

    bool is_point_in_reachability_polytope(const Point_3& relative_pos, const HalfSpacePolytopeConstraint& constraint);

    std::vector<Node*> get_grid_children(Node* parent);

    // Plot path method
    void plot_path();

    // Plot grid environment method
    void plot_grid_environment();

    // Getter for grid environment
    const GridEnvironment& get_grid_environment() const { return grid_env; }

};

} // namespace nas