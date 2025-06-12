#pragma once

#include "types.hpp"
#include "environments.hpp"

namespace nas {

// Define Foot stance status
const int LEFT_FOOT = 0;   // Represents the left foot in stance
const int RIGHT_FOOT = 1;  // Represents the right foot in stance

// Foot size information
const double foot_length = 0.22;
const double foot_width  = 0.12;

// Merge Node Flag
const bool merge_node_flag = true;

// Define Kinematics Reachability Path
// const std::string rf_in_lf_path = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/RF_constraints_in_LF.obj";
// const std::string lf_in_rf_path = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/LF_constraints_in_RF.obj";
const std::string rf_in_lf_path = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/LF_antecedent_CUTZ.obj";
const std::string lf_in_rf_path = "/Users/jiayu/Desktop/nas_ws/NAS/data/constraints_files/RF_antecedent_CUTZ.obj";

// Define Goal Specificaitons
const int stance_foot_at_goal = LEFT_FOOT;
const Vector_3 goal_offset(0.0, 0.0, 0.0); // goal offset applied to move in the local frame of the goal surface (last surface)

// Define Number of Steps (max number of steps)
const int total_num_steps = 30;

// Define Current (Initial) Foot Position
const int current_stance_foot_flag = RIGHT_FOOT;
const Point_3 current_foot_pos(2.2, 0.7, 0.0); // starting point for two flat
// const Point_3 current_foot_pos(0.1, 0.0, 0.0); // starting point for stairs
// const Point_3 current_foot_pos(0.0, 0.0, 0.0); // starting point for long stairs and longlong stairs

// Node similarity threshold (compare for centroid distance and/or perimeter distance)
const double node_similarity_threshold = 0.02;

// Node Search method
const std::string node_search_method = "bruteforce"; // "bruteforce", "kdtree", "knn"

// Define Surfaces lists
// Define the list of surfaces as a vector of vectors of Point_3
// Each surface is a vector of 4 points, which are the vertices of the surface
// The surfaces are defined in the following order:
// 1) bottom left, 2) bottom right, 3) top right, 4) top left

// Flat Terrain for testing
// const std::vector<std::vector<Point_3>> surf_list = Stairs;
const std::vector<std::vector<Point_3>> surf_list = TwoFlatSurfaces;
// const std::vector<std::vector<Point_3>> surf_list = LongStairs;
// const std::vector<std::vector<Point_3>> surf_list = LongLongStairs;

}
