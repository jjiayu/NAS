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
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/functional/hash.hpp>

namespace nas {

// Custom comparator for priority queue
struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const {
        // We want the node with lower f_score to have higher priority
        return a->f_score > b->f_score;
    }
};

struct NodeHash {
    size_t operator()(const Node* node) const {
        
        // Quantize coordinates to reduce floating point precision issues
        int x = static_cast<int>(node->centroid.x() / node_similarity_threshold);
        int y = static_cast<int>(node->centroid.y() / node_similarity_threshold);
        int z = static_cast<int>(node->centroid.z() / node_similarity_threshold);
        
        // Compute and quantize perimeter
        int quantized_perimeter = static_cast<int>(node->perimeter / node_similarity_threshold);
        
        // Use Boost's hash_combine for better distribution
        size_t seed = 0;
        boost::hash_combine(seed, x);
        boost::hash_combine(seed, y);
        boost::hash_combine(seed, z);
        boost::hash_combine(seed, quantized_perimeter);
        boost::hash_combine(seed, node->surface_id);
        boost::hash_combine(seed, node->stance_foot);
        
        return seed;
    }
};

struct NodeEqual {
    bool operator()(const Node* a, const Node* b) const {
        
        // Quantize coordinates using same method as hash
        int x_a = static_cast<int>(a->centroid.x() / node_similarity_threshold);
        int y_a = static_cast<int>(a->centroid.y() / node_similarity_threshold);
        int z_a = static_cast<int>(a->centroid.z() / node_similarity_threshold);
        
        int x_b = static_cast<int>(b->centroid.x() / node_similarity_threshold);
        int y_b = static_cast<int>(b->centroid.y() / node_similarity_threshold);
        int z_b = static_cast<int>(b->centroid.z() / node_similarity_threshold);
        
        int quantized_perimeter_a = static_cast<int>(a->perimeter / node_similarity_threshold);
        int quantized_perimeter_b = static_cast<int>(b->perimeter / node_similarity_threshold);
        
        // Compare quantized coordinates, perimeter, and other properties
        return (x_a == x_b && 
                y_a == y_b && 
                z_a == z_b && 
                quantized_perimeter_a == quantized_perimeter_b &&
                a->surface_id == b->surface_id && 
                a->stance_foot == b->stance_foot);
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

    // Path found
    std::vector<Node*> result_path = {};

    // Open Set - using Boost.Heap for efficient decrease-key operations
    typedef boost::heap::fibonacci_heap<Node*, boost::heap::compare<CompareNodes>> OpenSet;
    OpenSet open_set;

    // Track handles for each node in open set (needed for decrease-key)
    std::unordered_map<Node*, OpenSet::handle_type, NodeHash, NodeEqual> node_handles;

    // Close set
    std::unordered_set<Node*, NodeHash, NodeEqual> closed_set;

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
    AstarSearch();

    // Main search method
    void search();

    // Get children method
    std::vector<Node*> get_children(Node* current_node);

    // Plot path method
    void plot_path();

};

} // namespace nas