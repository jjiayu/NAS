#pragma once

#include "types.hpp"
#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Plane_3.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Aff_transformation_3.h>
#include <coal/collision_object.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/shape/convex.h>
#include <coal/distance.h>
#include <coal/collision.h>
#include <Eigen/Dense>

namespace nas {

// Half-space polytope constraint representation
struct HalfSpacePolytopeConstraint{
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
};

// Surface constraint representation (the last row is on surface constraint)
struct SurfaceConstraint{
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
};

class Node;  // Forward declaration

std::vector<Point_2> transform_3d_points_to_surface_plane(const std::vector<Point_3>& points, const Transformation& transformation);

std::vector<Point_3> transform_2d_points_to_world(const std::vector<Point_2>& points, const Transformation& inverse_transformation);

Point_3 get_centroid(const std::vector<Point_3>& points);

Polyhedron minkowski_sum(const std::vector<Point_3>& patch_vertices, const Polyhedron& polytope);

std::vector<Point_3> compute_polytope_plane_intersection(const Plane_3& plane, const Polyhedron& polytope);

std::vector<Point_2> compute_2d_polygon_intersection(const std::vector<Point_2>& obj_polygon_vertices, const std::vector<Point_2>& clip_polygon_vertices);

std::vector<Point_2> compute_2d_polygon_intersection(const std::vector<Point_2>& subject_polygon, const std::vector<Point_2>& clip_polygon);

double is_leftside_of_edge(const Point_2& point, const Point_2& edge_start, const Point_2& edge_end);

// Compute the perimeter of a polygon
double compute_polygon_perimeter(const Polyhedron& polyhedron);

//Compare polygons defined by vertices in 3D space
double compare_polygon_similarity_3d(const std::vector<Point_3>& polygon1, const std::vector<Point_3>& polygon2);

double compute_euclidean_distance(const Point_3& start_location, const Point_3& end_location);

// COAL GJK distance computation function
double calculate_gjk_distance_point_to_patch(const std::vector<Point_3>& patch_points, const Point_3& goal);

// COAL EPA distance computation function
double calculate_epa_distance_point_to_patch(const std::vector<Point_3>& patch_points, const Point_3& goal);

// Convert half-space polytope constraint to H-representation
HalfSpacePolytopeConstraint convert_polytope_to_half_space_constraint(const Polyhedron& polytope);

// Convert surface constraint to H-representation
SurfaceConstraint generate_surface_constraint(const Polyhedron& surface_3d);

// Rotate polyhedron around Z-axis by given angle in radians
Polyhedron rotate_polyhedron_z(const Polyhedron& polytope, double yaw_angle);

} // namespace nas