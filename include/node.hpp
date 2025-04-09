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
        int node_id;    //The node ID
        Node* parent_ptr;   //Pointer to the parent node
        std::vector<Point_3> patch_vertices; // The vertices of the patch
        Polygon_2 patch_polygon; // The polygon of the patch (NOTE: defined in the surface coordinate system)
        Transformation transform_to_2d; // Transformation the surface coordinate system
        Transformation transform_to_3d; // Transformation from surface coordinate system to world coordinate system        
        int stance_foot; // The foot that is on the ground
        int surface_id; // The surface ID that the node belongs to
        int depth; // The depth of the node in the tree
        Node* kd_left_ptr; // Pointer to the left child node in the KD tree
        Node* kd_right_ptr; // Pointer to the right child node in the KD tree

        bool check_if_node_contains_point(const Point_3& point);
    };

}