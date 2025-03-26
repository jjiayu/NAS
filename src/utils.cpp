#include "utils.hpp"
#include "tree.hpp"
#include "constants.hpp"
#include "visualizer.hpp"
#include "geometry.hpp"
#include "types.hpp"
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/polygon_mesh_io.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <fstream>
#include <iostream>

namespace nas {

void load_obj(const std::string& filename, Polyhedron& polyhedron) {
    if (!CGAL::IO::read_polygon_mesh(filename, polyhedron)) {
        throw std::runtime_error("Failed to load polyhedron from: " + filename);
    }
    std::cout << "Successfully loaded polytope from: " << filename << std::endl;
}

} // namespace nas