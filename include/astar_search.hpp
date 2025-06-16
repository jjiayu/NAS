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

struct NodeHash {
    size_t operator()(const Node* node) const {
        // Compute centroid from the patch
        Point_3 centroid = get_centroid(node->patch_vertices);
        
        // Hash the centroid coordinates and perimeter
        size_t h1 = std::hash<double>()(centroid.x());
        size_t h2 = std::hash<double>()(centroid.y());
        size_t h3 = std::hash<double>()(centroid.z());
        size_t h4 = std::hash<double>()(compute_polygon_perimeter(node->patch_polyhedron_3d));
        return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3);
    }
};

struct NodeEqual {
    bool operator()(const Node* a, const Node* b) const {
        const double TOLERANCE = 0.02;
        
        // Compute centroids
        Point_3 centroid_a = get_centroid(a->patch_vertices);
        Point_3 centroid_b = get_centroid(b->patch_vertices);
        
        return (std::abs(CGAL::to_double(centroid_a.x()) - CGAL::to_double(centroid_b.x())) < TOLERANCE &&
                std::abs(CGAL::to_double(centroid_a.y()) - CGAL::to_double(centroid_b.y())) < TOLERANCE &&
                std::abs(CGAL::to_double(centroid_a.z()) - CGAL::to_double(centroid_b.z())) < TOLERANCE &&
                std::abs(compute_polygon_perimeter(a->patch_polyhedron_3d) - compute_polygon_perimeter(b->patch_polyhedron_3d)) < TOLERANCE);
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

    // Open Set
    std::priority_queue<Node*, std::vector<Node*>, CompareNodes> open_set;

    // Close set
    std::unordered_set<Node*, NodeHash, NodeEqual> closed_set;

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