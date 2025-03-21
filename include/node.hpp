#pragma once

#include "types.hpp"
#include <vector>
#include <memory>

namespace nas {

    class Node {
    public:
        Node();
        ~Node();

    public:
        // int surface_id; // The surface ID that the node belongs to
        int node_id;    //The node ID
        Node* parent_ptr;   //Pointer to the parent node
        std::vector<Vector_3> patch_vertices; // The vertices of the patch
        int stance_foot; // The foot that is on the ground
    };

}