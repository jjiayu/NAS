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

std::vector<Point_2> transform_3d_points_to_surface_plane(const std::vector<Point_3>& points, const Transformation& transformation);
std::vector<Point_3> transform_2d_points_to_world(const std::vector<Point_2>& points, const Transformation& inverse_transformation);

} // namespace nas