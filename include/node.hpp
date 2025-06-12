#pragma once

#include "types.hpp"
#include <vector>
#include <memory>
#include "surface.hpp"
namespace nas {

class Node {
public:
    Node();
    ~Node();

    public:
        // For NAS Graph Generation
        int node_id;    //The node ID
        std::vector<Node*> parent_ptrs = std::vector<Node*>();   //Pointer to the parent node
        Node* kd_left_ptr;     //Pointer to the left child in KD-tree
        Node* kd_right_ptr;    //Pointer to the right child in KD-tree
        std::vector<Point_3> patch_vertices; // The vertices of the patch
        Polygon_2 patch_polygon_2d; // The polygon of the patch (NOTE: defined in the surface coordinate system)
        Polyhedron patch_polyhedron_3d; // The polyhedron of the patch in 3d world frame
        Transformation transformation_to_2d; // Transformation the surface coordinate system
        Transformation transformation_to_3d; // Transformation from surface coordinate system to world coordinate system        
        int stance_foot; // The foot that is on the ground
        int surface_id; // The surface ID that the node belongs to
        int depth; // The depth of the node in the tree

        // For A* search
        Node* parent; // Pointer to the parent node
        double g_score;      // Cost from start to current
        double h_score;      // Heuristic cost to goal
        double f_score;      // Total cost (g + h)

        bool check_if_node_contains_point(const Point_3& point);
    };

}