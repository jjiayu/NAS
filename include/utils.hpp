#pragma once

#include "types.hpp"
#include <vector>
#include <string>
#include <stdexcept>

namespace nas {

void load_obj(const std::string& filename, Polyhedron& polyhedron);

Vector_3 compute_centroid(const std::vector<Point_3>& points);

Polyhedron minkowski_sum(const std::vector<Vector_3>& patch_vertices, 
                         const Polyhedron& polytope);

void polytope_surf_intersection(const std::vector<Point_3>& surf_pts, const Polyhedron& polytope);

// Transformation function
Transformation create_transfomation_to_surface_center(const std::vector<Point_3>& surf_pts, const Plane_3& plane);

} // namespace nas