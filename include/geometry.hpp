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

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef Kernel::Plane_3 Plane_3;
typedef CGAL::Aff_transformation_3<Kernel> Transformation;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

// Function declarations
Vector_3 get_centroid(const std::vector<Point_3>& points);
Polyhedron minkowski_sum(const std::vector<Vector_3>& patch_vertices, const Polyhedron& polytope);
Polygon_2 project_to_surface_plane(const std::vector<Point_3>& points, Transformation& transform);
bool compute_2d_edge_intersection(const Point_2& p1, const Point_2& p2,
                                const Point_2& p3, const Point_2& p4,
                                Point_2& intersection);
bool is_point_inside_edge(const Point_2& point, const Point_2& edge_start, const Point_2& edge_end);
Polygon_2 compute_polygon_intersection(const Polygon_2& intersection_polygon, const Polygon_2& surf_polygon);
void polytope_surf_intersection(const std::vector<Point_3>& surf_pts, const Polyhedron& polytope);
void compute_surface_param_and_frame(const std::vector<Point_3>& surf_pts, Plane_3& out_plane, Transformation& out_transformation);
std::vector<Point_3> polytope_plane_intersection(const Polyhedron& polytope, const Plane_3& plane);

} // namespace nas