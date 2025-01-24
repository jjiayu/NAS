#include <vector>
#ifndef NODE_HPP
#define NODE_HPP

using namespace std;

namespace nas {

    class Node {
    public:
        Node();
        ~Node();

    public:
        int surface_id; // The surface ID that the node belongs to
        int node_id;    //The node ID
        Node* parent_node;   //Pointer to the parent node
        vector<Node*> child_nodes;  //Vector of pointers to the child nodes
        
    };

}

#endif // NODE_HPP