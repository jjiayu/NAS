#ifndef UTILS_HPP
#define UTILS_HPP

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <vector>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;

namespace nas {

bool load_obj(const std::string& filename, Polyhedron& polyhedron);

std::vector<Polyhedron> convert_surf_pts_to_polyhedron(std::vector<std::vector<Point_3>> surface_list);

Vector_3 get_centroid(const Polyhedron& polyhedron);

Polyhedron minkowski_sum(const std::vector<Vector_3>& patch_vertices, 
                         const Polyhedron& polytope);

void polytope_surf_intersection(const std::vector<Point_3>& surf_pts, const Polyhedron& polytope);

} // namespace nas

#endif