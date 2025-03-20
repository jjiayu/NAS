#ifndef NAS_CONSTANTS_HPP
#define NAS_CONSTANTS_HPP

#include "types.hpp"

namespace nas {

// Constants for the NAS algorithm
const double EPSILON = 1e-10;
const double SURFACE_EPSILON = 1e-3;
const double COLLISION_EPSILON = 1e-3;

// Define Foot stance status
const int LEFT_FOOT = 0;   // Represents the left foot in stance
const int RIGHT_FOOT = 1;  // Represents the right foot in stance

// Define Kinematics Reachability Path
const std::string rf_in_lf_path = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/RF_constraints_in_LF_quasi_flat_REDUCED.obj";
const std::string lf_in_rf_path = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/LF_constraints_in_RF_quasi_flat_REDUCED.obj";

// Define Goal Specificaitons
const int goal_stance_foot = LEFT_FOOT;
const Vector_3 goal_offset(0.0, 0.0, 0.0); // goal offset applied to move in the local frame of the goal surface (last surface)

// Define Number of Steps
const int total_num_steps = 3;

// Define Surfaces lists
// Define the list of surfaces as a vector of vectors of Point_3
// Each surface is a vector of 4 points, which are the vertices of the surface
// The surfaces are defined in the following order:
// 1) bottom left, 2) bottom right, 3) top right, 4) top left

// Flat Terrain for testing
const std::vector<std::vector<Point_3>> surf_pts_list = {
    // Surface 1
    {
        Point_3(0.0, 0.0, 0.0),  // bottom left
        Point_3(5.0, 0.0, 0.0),  // bottom right
        Point_3(5.0, 1.0, 0.0),  // top right
        Point_3(0.0, 1.0, 0.0)   // top left
    },
    // Surface 2: Another rectangle or surface
    {
        Point_3(5.5,  0.0, 0.0),  // bottom left
        Point_3(10.0, 0.0, 0.0),  // bottom right
        Point_3(10.0, 1.0, 0.0),  // top right
        Point_3(5.5,  1.0, 0.0)   // top left
    },
    // // Surface 3: Another rectangle or surface
    // {
    //     Point_3(5.0,  0.0, 0.0),  // bottom left
    //     Point_3(10.0, 0.0, 0.0),  // bottom right
    //     Point_3(10.0, 1.0, 0.0),  // top right
    //     Point_3(5.0,  1.0, 0.0)   // top left
    // }
    // Add more surfaces as needed
};

}

#endif // NAS_CONSTANTS_HPP