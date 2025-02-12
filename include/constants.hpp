#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

namespace nas {

// Define Foot stance status
const int LEFT_FOOT = 0;   // Represents the left foot in stance
const int RIGHT_FOOT = 1;  // Represents the right foot in stance

// Define Kinematics Reachability Path
const std::string rf_in_lf_path = "data/constants_files/RF_constraints_in_LF.obj";
const std::string lf_in_rf_path = "data/constants_files/LF_constraints_in_RF.obj";

// Define Goal Specificaitons
const int goal_stance_foot = LEFT_FOOT;
const Vector_3 goal_offset(0.0, 0.0, 0.0); // goal offset applied to move in the local frame of the goal surface (last surface)

// Define Surfaces lists
// Define the list of surfaces as a vector of vectors of Point_3
// Each surface is a vector of 4 points, which are the vertices of the surface
// The surfaces are defined in the following order:
// 1) bottom left, 2) bottom right, 3) top right, 4) top left

// Flat Terrain for testing
const std::vector<std::vector<Vector_3>> surf_list = {
    // Surface 1
    {
        Vector_3(0.0, 0.0, 0.0),  // bottom left
        Vector_3(5.0, 0.0, 0.0),  // bottom right
        Vector_3(5.0, 1.0, 0.0),  // top right
        Vector_3(0.0, 1.0, 0.0)   // top left
    },
    // // Surface 2: Another rectangle or surface
    // {
    //     Vector_3(5.0,  0.0, 0.0),  // bottom left
    //     Vector_3(10.0, 0.0, 0.0),  // bottom right
    //     Vector_3(10.0, 1.0, 0.0),  // top right
    //     Vector_3(5.0,  1.0, 0.0)   // top left
    // },
    // // Surface 3: Another rectangle or surface
    // {
    //     Vector_3(5.0,  0.0, 0.0),  // bottom left
    //     Vector_3(10.0, 0.0, 0.0),  // bottom right
    //     Vector_3(10.0, 1.0, 0.0),  // top right
    //     Vector_3(5.0,  1.0, 0.0)   // top left
    // }
    // Add more surfaces as needed
};

}

#endif // CONSTANTS_HPP