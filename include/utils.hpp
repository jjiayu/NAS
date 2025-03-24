#pragma once

#include "types.hpp"
#include <vector>
#include <string>
#include <stdexcept>

namespace nas {

void load_obj(const std::string& filename, Polyhedron& polyhedron);

Vector_3 compute_centroid(const std::vector<Point_3>& points);

void polytope_surf_intersection(const std::vector<Point_3>& surf_pts, const Polyhedron& polytope);

Polygon_2 compute_polygon_intersection(const Polygon_2& intersection_polygon, const Polygon_2& surf_polygon);

// Transformation function
Transformation create_transfomation_to_surface_center(const std::vector<Point_3>& surf_pts, const Plane_3& plane);

} // namespace nas