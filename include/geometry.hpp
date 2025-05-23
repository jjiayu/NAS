#pragma once

#include "types.hpp"
#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Plane_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/Vector_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Aff_transformation_3.h>

namespace nas {

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

} // namespace nas