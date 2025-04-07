#pragma once

#include "types.hpp"
#include <vector>
#include <memory>
#include "surface.hpp"
namespace nas {

// Forward declaration
class Surface;

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
        const Surface* surface_ptr; // Pointer to the surface
        int depth; // The depth of the node in the tree
    };

}