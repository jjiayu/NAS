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
        int stance_foot; // The foot that is on the ground
        int surface_id; // The surface ID that the node belongs to
        int depth; // The depth of the node in the tree
        Node* kd_left_ptr; // Pointer to the left child node in the KD tree
        Node* kd_right_ptr; // Pointer to the right child node in the KD tree
    };

}