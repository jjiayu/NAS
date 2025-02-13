#include <vector>
#ifndef NODE_HPP
#define NODE_HPP

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>

using namespace std;

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;

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

#endif // NODE_HPP