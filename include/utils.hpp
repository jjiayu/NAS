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

std::vector<Polyhedron> convert_surfaces_to_planes(std::vector<std::vector<Point_3>> surface_list);

Vector_3 get_centroid(const Polyhedron& polyhedron);

} // namespace nas

#endif