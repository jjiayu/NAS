// include/prob_data.hpp

#ifndef TREE_HPP
#define TREE_HPP

#include <vector>
#include <string>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <Eigen/Dense>
#include "constants.hpp" // Include the constants header
#include <iostream>
#include <queue>
#include "node.hpp"

// Define types
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Vector_3<Kernel> Vector_3;

namespace nas {

class Tree {
public:

    // Reachability Polytope
    Polyhedron rf_in_lf_polytope;
    Polyhedron lf_in_rf_polytope;

    // Environment
    std::vector<Polyhedron> env_model;

    // Goal
    int goal_stance_foot;
    Vector_3 goal_location;

    // Number of steps
    int num_steps;

    // Variables representing the Tree (Breadth First Search Expansion)
    std::vector<std::vector<Node*>> layers;  // Store nodes by layer
    std::queue<Node*> expansion_queue;       // Queue for BFS expansion

    // Number of node counter
    int node_counter;

    //Constructor
    Tree();
    // Destructor
    // ~ProblemData();

    // Methods
    void expand_layer();
    void expand_to_depth(int depth);
    std::vector<Node*> get_children(Node* parent);

};

} // namespace nas

#endif // TREE_HPP