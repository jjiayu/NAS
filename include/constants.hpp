#pragma once

#include "types.hpp"
#include "environments.hpp"

namespace nas {

// CoM z Height (for footstep planning))
const double com_z_height = 0.75;

// A star related
const std::string a_star_distance_metric = "gjk"; // "euclidean", "gjk"

// Discrete a star related
const double a_star_grid_resolution = 0.04; // meters

const bool cycle_detection = true;

// Define Foot stance status
const int LEFT_FOOT = 0;   // Represents the left foot in stance
const int RIGHT_FOOT = 1;  // Represents the right foot in stance

// Foot size information
const double foot_length = 0.22;//0.22;
const double foot_width  = 0.12;//0.12;

// Foot yaw angle range
const bool foot_yaw_rotation_flag = true;
const double current_foot_yaw = 0.0;
const double foot_yaw_angle_increment = 10.0/180.0*M_PI; // radians (15 degrees)
const int foot_yaw_angle_discretization_num = 3; // one side discretize times 3*foot_yaw_angle_discretization

// Merge Node Flag
const bool merge_node_flag = true;

// Define Antecedent Kinematics Reachability Path
// const std::string rf_in_lf_path = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/RF_constraints_in_LF.obj";
// const std::string lf_in_rf_path = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/LF_constraints_in_RF.obj";
const std::string rf_in_lf_path_antecedent = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/LF_antecedent_CUTZ.obj";
const std::string lf_in_rf_path_antecedent = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/RF_antecedent_CUTZ.obj";

// for A* and footstep planning qp, we use forward polytopes
const std::string rf_in_lf_path_forward = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/RF_constraints_in_LF_quasi_flat_REDUCED.obj";
const std::string lf_in_rf_path_forward = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/LF_constraints_in_RF_quasi_flat_REDUCED.obj";
const std::string com_in_lf_path_forward = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/COM_constraints_in_LF_effector_frame_REDUCED.obj";
const std::string com_in_rf_path_forward = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/COM_constraints_in_RF_effector_frame_REDUCED.obj";

// Define Goal Specificaitons
const int stance_foot_at_goal = LEFT_FOOT;
const Vector_3 goal_offset(0.0, 0.0, 0.0); // goal offset applied to move in the local frame of the goal surface (last surface)

// Define Number of Steps (max number of steps)
const int total_num_steps = 40;

// Define Current (Initial) Foot Position
const int current_stance_foot_flag = RIGHT_FOOT;
// const Point_3 current_foot_pos(2.2, 0.7, 0.0); // starting point for two flat
// const Point_3 current_foot_pos(0.1, 0.0, 0.0); // starting point for stairs
const Point_3 current_foot_pos(0.0, 0.0, 0.0); // starting point for long stairs and longlong stairs
// const Point_3 current_foot_pos(6.25, 0.5, 0.0);

// Node similarity threshold (compare for centroid distance and/or perimeter distance)
const double node_similarity_threshold = 0.02;

// Node Search method
const std::string node_search_method = "bruteforce"; // "bruteforce", "kdtree", "knn"

// Define Surfaces lists
// Define the list of surfaces as a vector of vectors of Point_3
// Each surface is a vector of 4 points, which are the vertices of the surface
// The surfaces are defined in the following order:
// 1) bottom left, 2) bottom right, 3) top right, 4) top left

// const std::vector<std::vector<Point_3>> surf_list = Stairs;
// const std::vector<std::vector<Point_3>> surf_list = TwoFlatSurfaces;
// const std::vector<std::vector<Point_3>> surf_list = LongStairsComplete;
// const std::vector<std::vector<Point_3>> surf_list = LongStairs;
const std::vector<std::vector<Point_3>> surf_list = LongLongStairs;
// const std::vector<std::vector<Point_3>> surf_list = ThreePathsScene;
// const std::vector<std::vector<Point_3>> surf_list = Stairs_Up_Down;

}