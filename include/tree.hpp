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

// Define types
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

namespace nas {

class Tree {
public:

    // Reachability Polytope
    Polyhedron rf_in_lf_polytope;
    Polyhedron lf_in_rf_polytope;

    //Constructor
    Tree();
    // Destructor
    // ~ProblemData();

};

} // namespace nas

#endif // TREE_HPP